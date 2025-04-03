# This is the main script for the positioning algorithm. This script includes CPAP communication, dynamic sound velocity calculation, sensor-data aquisition through pymavlink, position estimation with multilateration, and publishing of the point estimation to the GPS_INPUT message, allowing for the position to be displayed on the QGroundControl map
import asyncio
import serial_asyncio
import random
import numpy as np
from numpy.linalg import norm
from scipy.optimize import least_squares
import threading
from pymavlink import mavutil
import time

mav = None # Global MAVLink connection used for both sending and receiving
# Global variables for dynamic sound velocity (from SCALED_PRESSURE2)
latest_pressure = None  # in hPa
latest_temp = None      # in Celsius

# Helper function to calculate the NMEA checksums
def nmea_checksum(nmea_message):
    nmea_message = nmea_message.strip()
    assert nmea_message.startswith('$'), "[x] NMEA message must start with '$'"
    nmea_message = nmea_message[1:]
    data_list = nmea_message.split('*')
    assert len(data_list) == 2, "[x] FormatError, NMEA message must contain exactly one '*'"
    data = data_list[0]
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    return f"{checksum:02X}"

# MAVLink listener thread: listen for VFR_HUD and SCALED_PRESSURE2 messages.
def mavlink_listener():
    global latest_pressure, latest_temp, mav
    while True:
        msg = mav.recv_match(blocking=True, timeout=1)
        if msg:
            msg_type = msg.get_type()
            if msg_type == "SCALED_PRESSURE2":
                # SCALED_PRESSURE2: press_abs in hPa, temperature in 0.01 °C.
                latest_pressure = msg.press_abs  
                latest_temp = msg.temperature / 100.0  
                #print(f"Received SCALED_PRESSURE2: Pressure = {latest_pressure:.1f} hPa, Temp = {latest_temp:.2f} °C")

pending_responses = {}  # Global dictionary for pending responses

# Background reader task: continuously dispatch incoming replies from serial port
async def background_reader(reader):
    while True:
        try:
            line = await reader.readline()
            if not line:
                continue
            response = line.decode('utf-8').strip()
            key = None
            if response.startswith("$PSIMCUP,TPR,"):
                parts = response.split(',')
                if len(parts) >= 3:
                    key = parts[2]
            elif response.startswith("$PSIMCUP,BAT,") or response.startswith("$PSIMCUP,BAA,"):
                key = "BAT"
            elif response.startswith("$PSIMCUP,VER,") or response.startswith("$PSIMCUP,VEA,"):
                key = "VER"
            elif response.startswith("$PSIMCUP,PWR,") or response.startswith("$PSIMCUP,PWA,"):
                key = "PWR"
            else:
                print("[Background Reader] Unrecognized response format")

            if key and key in pending_responses:
                future = pending_responses.pop(key)
                if not future.done():
                    future.set_result(response)
        except Exception as e:
            print("[Background Reader] Error:", e)

# Helper function to construct NMEA message to interrogate cNODEs (for distance)
async def query_transponder(reader, writer, channel, timeout=5):
    message = f"$PSIMCUP,TPI,{channel},{float(timeout)}*"
    message += nmea_checksum(f"$PSIMCUP,TPI,{channel},{float(timeout)}*")
    message += "\r\n"
    writer.write(message.encode('utf-8'))
    await writer.drain()
    loop = asyncio.get_running_loop()
    future = loop.create_future()
    pending_responses[channel] = future
    try:
        response = await asyncio.wait_for(future, timeout=timeout)
        return response
    except asyncio.TimeoutError:
        pending_responses.pop(channel, None)
        return None

# Helper function to construct NMEA message to read battery level for cPAP
async def read_battery_level(reader, writer, timeout=5):
    print(f"{'-'*70}\n[+] Read Battery Level")
    message = "$PSIMCUP,BAT*" + nmea_checksum("$PSIMCUP,BAT*") + "\r\n"
    print(f"User -> cPAP command: {message.strip()}")
    writer.write(message.encode('utf-8'))
    await writer.drain()
    loop = asyncio.get_running_loop()
    future = loop.create_future()
    pending_responses["BAT"] = future
    try:
        response = await asyncio.wait_for(future, timeout=timeout)
        return response
    except asyncio.TimeoutError:
        print("Timeout waiting for battery response")
        pending_responses.pop("BAT", None)
        return None

# Helper function to construct NMEA message to read version info for cPAP
async def read_version_info(reader, writer, timeout=5):
    print(f"{'-'*70}\n[+] Read Version Info")
    message = "$PSIMCUP,VER*" + nmea_checksum("$PSIMCUP,VER*") + "\r\n"
    print(f"User -> cPAP command: {message.strip()}")
    writer.write(message.encode('utf-8'))
    await writer.drain()
    loop = asyncio.get_running_loop()
    future = loop.create_future()
    pending_responses["VER"] = future
    try:
        response = await asyncio.wait_for(future, timeout=timeout)
        return response
    except asyncio.TimeoutError:
        print("Timeout waiting for version info")
        pending_responses.pop("VER", None)
        return None

# Helper function to construct NMEA message to set the transmit power level
async def set_transmit_power(reader, writer, level, timeout=5):
    print(f"{'-'*70}\n[+] Set Transmit Power to level {level}")
    assert level in [1, 2, 3, 4], "[x] FormatError, Transmit Power Level must be in [1,2,3,4]"
    message = f"$PSIMCUP,PWR,{level}*" + nmea_checksum(f"$PSIMCUP,PWR,{level}*") + "\r\n"
    print(f"User -> cPAP command: {message.strip()}")
    writer.write(message.encode('utf-8'))
    await writer.drain()
    loop = asyncio.get_running_loop()
    future = loop.create_future()
    pending_responses["PWR"] = future
    try:
        response = await asyncio.wait_for(future, timeout=timeout)
        return response
    except asyncio.TimeoutError:
        print("Timeout waiting for power command response")
        pending_responses.pop("PWR", None)
        return None

# Function to get distances from a list of channels
async def get_multiple_distances(reader, writer, channels, timeout):
    tasks = []
    for channel in channels:
        tasks.append(asyncio.create_task(query_transponder(reader, writer, channel, timeout)))
        await asyncio.sleep(0.50)  # slight delay between queries
    responses = await asyncio.gather(*tasks)
    return responses

# Helper function to calculate the distance from t_time from the received message
def parse_distance(response, speed=1500):
    try:
        parts = response.split(',')
        if len(parts) < 5:
            return None
        t_time = float(parts[4])
        distance = round(t_time * speed, 3)
        return distance
    except Exception as e:
        print("Error parsing distance:", e)
        return None

# Helper function to convert from latlon coordinates to meters (relative to origin)
def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    R = 6371000  # Earth's radius in meters
    dlat = np.deg2rad(lat - origin_lat)
    dlon = np.deg2rad(lon - origin_lon)
    x = R * dlon * np.cos(np.deg2rad(origin_lat))
    y = R * dlat
    return x, y

# Helper function to convert from meters back to latlon coordinates (relative to origin)
def meters_to_latlon(x, y, origin_lat, origin_lon):
    R = 6371000
    dlat = y / R
    dlon = x / (R * np.cos(np.deg2rad(origin_lat)))
    lat = origin_lat + np.rad2deg(dlat)
    lon = origin_lon + np.rad2deg(dlon)
    return lat, lon

# Get an initial estimate for position using linear least squares.
def initial_guess_for_xy(positions, distances, fixed_z):
    N = len(positions)
    r = []
    for pos, d in zip(positions, distances):
        vertical_offset = fixed_z - pos[2]
        horizontal = np.sqrt(np.abs(d**2 - vertical_offset**2))
        r.append(horizontal)
    r = np.array(r)
    x1, y1 = positions[0, 0], positions[0, 1]
    r1 = r[0]
    A = []
    b = []
    for i in range(1, N):
        xi, yi = positions[i, 0], positions[i, 1]
        ri = r[i]
        A.append([2 * (xi - x1), 2 * (yi - y1)])
        b.append(xi**2 - x1**2 + yi**2 - y1**2 + r1**2 - ri**2)
    A = np.array(A)
    b = np.array(b)
    guess_xy, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    return guess_xy

# Estimate the ROV's position using multilateration.
def multilaterate(positions, distances, fixed_z):
    def residual_func(point, beacon_positions, measured_distances, fixed_z):
        return [
            np.sqrt((point[0] - pos[0])**2 + (point[1] - pos[1])**2 + (fixed_z - pos[2])**2) - d
            for pos, d in zip(beacon_positions, measured_distances)
        ]
    guess_xy = initial_guess_for_xy(positions, distances, fixed_z)
    initial_guess = np.array([guess_xy[0], guess_xy[1]])
    result = least_squares(residual_func, x0=initial_guess, args=(positions, distances, fixed_z), diff_step=1e-3)
    return np.array([result.x[0], result.x[1], fixed_z])

# Calculate depth (in meters) from pressure in hPa.
def calculate_depth(pressure_hPa, surface_pressure_hPa=1013.25):
    # Convert hPa to dbar: 1 dbar = 100 hPa.
    pressure_dbar = pressure_hPa / 100.0
    surface_pressure_dbar = surface_pressure_hPa / 100.0
    depth = pressure_dbar - surface_pressure_dbar
    depth -= 0.25
    return -depth  # negative z-coordinate because we are below water

# Sound velocity calculation (using UNESCO 1983 algorithm)
# Python implementation from https://github.com/bjornaa/seawater/blob/master/seawater/misc.py
def soundvel(S, T, P=0):
    # S: Salinity [PSS-78], T: Temperature [°C], P: Pressure [dbar] (default=0)
    P = 0.1 * P  # Convert dbar to bar
    c00 = 1402.388; c01 =  5.03711; c02 = -5.80852e-2; c03 =  3.3420e-4; c04 = -1.47800e-6; c05 =  3.1464e-9
    c10 =  0.153563; c11 =  6.8982e-4; c12 = -8.1788e-6; c13 =  1.3621e-7; c14 = -6.1185e-10
    c20 =  3.1260e-5; c21 = -1.7107e-6; c22 =  2.5974e-8; c23 = -2.5335e-10; c24 =  1.0405e-12
    c30 = -9.7729e-9; c31 =  3.8504e-10; c32 = -2.3643e-12
    P2 = P * P; P3 = P2 * P
    Cw = (c00 + (c01 + (c02 + (c03 + (c04 + c05 * T) * T) * T) * T) * T +
          (c10 + (c11 + (c12 + (c13 + c14 * T) * T) * T) * T) * P +
          (c20 + (c21 + (c22 + (c23 + c24 * T) * T) * T) * T) * P2 +
          (c30 + (c31 + c32 * T) * T) * P3)
    a00 =  1.389; a01 = -1.262e-2; a02 =  7.164e-5; a03 =  2.006e-6; a04 = -3.21e-8
    a10 =  9.4742e-5; a11 = -1.2580e-5; a12 = -6.4885e-8; a13 =  1.0507e-8; a14 = -2.0122e-10
    a20 = -3.9064e-7; a21 =  9.1041e-9; a22 = -1.6002e-10; a23 =  7.988e-12
    a30 =  1.100e-10; a31 =  6.649e-12; a32 = -3.389e-13
    A = (a00 + (a01 + (a02 + (a03 + a04 * T) * T) * T) * T +
         (a10 + (a11 + (a12 + (a13 + a14 * T) * T) * T) * P +
         (a20 + (a21 + (a22 + a23 * T) * T) * T) * P2 +
         (a30 + (a31 + a32 * T) * T) * P3))
    b00 = -1.922e-2; b01 = -4.42e-5; b10 =  7.3637e-5; b11 =  1.7945e-7
    B = b00 + b01 * T + (b10 + b11 * T) * P
    d00 =  1.727e-3; d10 = -7.9836e-6
    D = d00 + d10 * P
    return Cw + A * S + B * S**1.5 + D * S**2

# Main receiver loop: Compute sound velocity, query distances, perform multilateration, and publish GPS_INPUT.
async def main():
    global mav
    global_timeout = 2.0   # timeout for queries
    polling_delay = 0.50   # delay between query cycles

    # Open serial port
    try:
        reader, writer = await serial_asyncio.open_serial_connection(url='/dev/ttyUSB0', baudrate=9600)
    except Exception as e:
        print("Failed to open serial connection:", e)
        return

    # Start the background reader for the serial port.
    asyncio.create_task(background_reader(reader))

    # Define transponder channels and their lat/lon positions (in decimal degrees).
    channels = ["M22", "M16", "M29", "M20"]
    transponder_latlon = [
        (59.428465173, 10.465137681),  # t1 (origin)
        (59.429155409, 10.464601094),  # far
        (59.428427280, 10.465334121),  # t3
        #(59.428463528, 10.464801958)    # hjørnet av målestasjonen
        (59.428464167, 10.464801112)    #nytt hjørnet av målestasjon
        #(59.428445150, 10.465241581)   # old t2, new t4
        #(59.428407132, 10.465439580),  # t4
    ]
    origin_lat, origin_lon = transponder_latlon[1]  # Use first transponder as the origin

    # Convert transponder positions from lat/lon to meters.
    transponder_depth_const = -1.5  # Static depth of placed transponders
    transponder_positions = []
    for i, (lat_val, lon_val) in enumerate(transponder_latlon):
        x, y = latlon_to_meters(lat_val, lon_val, origin_lat, origin_lon)
        # To avoid degenerate geometry, add a small offset for one transponder if needed.
        depth_to_use = transponder_depth_const
        if i == 0:  # example: change depth for third transponder
            depth_to_use = -5.0
        transponder_positions.append([x, y, depth_to_use])
    transponder_positions = np.array(transponder_positions)

    # Example of adjusting a transponder's lat/lon if needed.
    offset = 1.0  # offset in meters
    theta = np.deg2rad(18)
    dx = offset * np.sin(theta)
    dy = offset * np.cos(theta)
    R = 6371000
    dlat = (dy / R) * (180 / np.pi)
    dlon = (dx / (R * np.cos(np.deg2rad(origin_lat)))) * (180 / np.pi)
    lat_t2, lon_t2 = transponder_latlon[1]
    transponder_latlon[1] = (lat_t2 + dlat, lon_t2 + dlon)
    print("Adjusted transponder positions if needed.")

    # Define GPS_INPUT ignore flags. We want to publish lat/lon, and avoid overriding the altitude
    GPS_INPUT_IGNORE_FLAG_ALT = 7  # Altitude is the 8th argument (index 7)
    ignore_flags = (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                    mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                    mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY)

    print("Receiver started. Polling transponders and performing multilateration...")

    battery = await read_battery_level(reader, writer)
    print("Battery reply:", battery)

    version = await read_version_info(reader, writer)
    print("Version reply:", version)

    power = await set_transmit_power(reader, writer, level=4)
    print("Power reply:", power)
    print("-" * 70)

    # Create a list to store the previous valid reading for each channel.
    prev_distances = [None] * len(channels)
    required_total = len(channels)   # e.g., 4 channels
    min_new_required = 3  # require at least 3 new valid readings
    prev_x = 0
    prev_y = 0
    # Main loop
    while True:
        responses = await get_multiple_distances(reader, writer, channels, timeout=global_timeout)

        # Compute dynamic sound velocity and depth from pressure.
        if latest_pressure is not None and latest_temp is not None:
            P_dbar = latest_pressure / 100.0  # Convert hPa to dbar
            dynamic_speed = soundvel(32, latest_temp, P_dbar)
            fixed_z = calculate_depth(latest_pressure)
        else:
            dynamic_speed = 1500  # Fallback value
            fixed_z = 0

        distances = []
        new_valid_flags = []  # True if this channel provided a new valid reading

        for i, (ch, resp) in enumerate(zip(channels, responses)):
            new_reading = None
            if resp:
                new_reading = parse_distance(resp, dynamic_speed)
            # Check if the new reading is valid (i.e. not None and not 0.0)
            if new_reading is not None and new_reading != 0.0:
                d = new_reading
                is_new = True
                print(f"Channel {ch} new reading: {d} m")
            else:
                # Fallback: use the previous valid reading if available.
                if prev_distances[i] is not None:
                    d = prev_distances[i]
                    is_new = False
                    print(f"Channel {ch} invalid new reading. Using previous valid reading: {d} m")
                else:
                    d = None
                    is_new = False
                    print(f"Channel {ch} invalid new reading and no previous reading available.")
            distances.append(d)
            new_valid_flags.append(is_new if d is not None else False)
            # Update stored reading only if we got a fresh valid measurement.
            if is_new and d is not None:
                prev_distances[i] = d

        new_valid_count = sum(1 for flag in new_valid_flags if flag)
        print(f"New valid readings: {new_valid_count} / {required_total} (required: {min_new_required})")

        # Only proceed if all channels have some reading (even fallback) and at least the minimum number are new.
        if all(d is not None and d != 0.0 for d in distances) and new_valid_count >= min_new_required:
            # Perform multilateration using all channels (some may be fallback values)
            est = multilaterate(transponder_positions, np.array(distances), fixed_z)
            est_lat, est_lon = meters_to_latlon(est[0], est[1], origin_lat, origin_lon)
            offset = np.sqrt((prev_x-est[0])**2 + (prev_y-est[1])**2)
            print("\n[Multilateration Result]")
            print("Estimated position (meters):", est)
            print("Estimated position (lat, lon, z):", est_lat, est_lon, est[2])
            print(f"Calculated offset: {offset}")
            print("-" * 95)

            prev_x = est[0]
            prev_y = est[1]

            # Publish the calculated GPS coordinates using GPS_INPUT.
            mav.mav.gps_input_send(
                int(time.time() * 1e6),  # Timestamp (microseconds)
                0,                       # GPS ID (0 for primary)
                ignore_flags,            # Flags (including altitude ignore)
                0,                       # GPS time (ms from start of week; 0 if unknown)
                0,                       # GPS week number (0 if unknown)
                3,                       # Fix type: 3 = 3D fix
                int(est_lat * 1e7),      # Latitude (WGS84 * 1E7)
                int(est_lon * 1e7),      # Longitude (WGS84 * 1E7)
                0.0,                     # Altitude in meters (dummy, ignored)
                1.0,                     # HDOP
                1.0,                     # VDOP
                0.0,                     # Velocity North
                0.0,                     # Velocity East
                0.0,                     # Velocity Down
                100.0,                     # Speed accuracy (1.0 works ok, 5 is better)
                0.6,                     # Horizontal accuracy
                0.1,                     # Vertical accuracy
                17                       # Number of satellites
            )
            print("Published GPS_INPUT message (lat/lon only).")
        else:
            print("Not enough new valid readings. Skipping multilateration cycle.\n")

        await asyncio.sleep(polling_delay)


if __name__ == "__main__":
    # Initialize global MAVLink connection.
    mav = mavutil.mavlink_connection('udpin:0.0.0.0:7777')
    print("Waiting for heartbeat from system...")
    mav.wait_heartbeat()
    print("Heartbeat received from system (system %u component %u)" %
          (mav.target_system, mav.target_component))

    # Start the MAVLink listener thread.
    listener_thread = threading.Thread(target=mavlink_listener, daemon=True)
    listener_thread.start()

    # Start the main async loop.
    asyncio.run(main())
