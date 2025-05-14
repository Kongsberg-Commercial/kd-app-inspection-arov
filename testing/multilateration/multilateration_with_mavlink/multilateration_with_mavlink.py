import asyncio
import serial_asyncio
import random
import numpy as np
from numpy.linalg import norm
from scipy.optimize import least_squares
import threading
from pymavlink import mavutil
import time

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


latest_alt = None

# Mavlink listener thread: Listen for VFR_HUD messages and update latest_alt.
def mavlink_listener():
    global latest_alt
    connection = mavutil.mavlink_connection('udpin:0.0.0.0:7777')
    print("Waiting for heartbeat from system...")
    connection.wait_heartbeat()
    print("Heartbeat received from system (system %u component %u)" %
          (connection.target_system, connection.target_component))
    while True:
        msg = connection.recv_match(type='VFR_HUD', blocking=True)
        if msg:
            latest_alt = msg.alt
            # print("VFR_HUD alt:", latest_alt)

pending_responses = {} # Global dictionary for pending responses
# Background reader task: continuously dispatch incoming replies
async def background_reader(reader):
    while True:
        try:
            line = await reader.readline()
            if not line:
                continue
            response = line.decode('utf-8').strip()
            # Determine key based on reply type
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


# Helper function to construct NMEA message to interrogate cNODEs. Receive distance
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
        await asyncio.sleep(0.250) # slight delay between individual queries. This may have to be increased to 0.5s because of cNODE overhead
    responses = await asyncio.gather(*tasks)
    return responses


# Helper function to calculate the distacne from t_time from the received message
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


# Helper function to convert from latlon coordinates to meters from a reference point (origo)
def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    R = 6371000  # Earth's radius in meters
    dlat = np.deg2rad(lat - origin_lat)
    dlon = np.deg2rad(lon - origin_lon)
    x = R * dlon * np.cos(np.deg2rad(origin_lat))
    y = R * dlat
    return x, y

# Helper function to convert from meters to latlon coordinates from a reference point (origo)
def meters_to_latlon(x, y, origin_lat, origin_lon):
    R = 6371000
    dlat = y / R
    dlon = x / (R * np.cos(np.deg2rad(origin_lat)))
    lat = origin_lat + np.rad2deg(dlat)
    lon = origin_lon + np.rad2deg(dlon)
    return lat, lon

# Helper function to get an initial estimate for the position using linear least squares
# This function closely follows the mathematics described in the bachelor thesis.
def initial_guess_for_xy(positions, distances, fixed_z):
    N = len(positions)
    r = []
    for pos, d in zip(positions, distances):
        vertical_offset = fixed_z - pos[2]
        horizontal = np.sqrt(np.abs(d**2 - vertical_offset**2))
        r.append(horizontal)
    r = np.array(r)
    
    # Use the first transponder as reference
    x1, y1 = positions[0, 0], positions[0, 1]
    r1 = r[0]
    
    A = []
    b = []
    for i in range(1, N):
        xi, yi = positions[i, 0], positions[i, 1]
        ri = r[i]
        A.append([2*(xi - x1), 2*(yi - y1)])
        b.append(xi**2 - x1**2 + yi**2 - y1**2 + r1**2 - ri**2)
    
    A = np.array(A)
    b = np.array(b)
    guess_xy, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    return guess_xy

# Estimate the ROV's position using multilateration. 
# For details about the mathematics, refer to the bachelor thesis.
def multilaterate(positions, distances, fixed_z):
    def residual_func(point, beacon_positions, measured_distances, fixed_z):
        return [
            np.sqrt((point[0] - pos[0])**2 + (point[1] - pos[1])**2 + (fixed_z - pos[2])**2) - d
            for pos, d in zip(beacon_positions, measured_distances)
        ]
    guess_xy = initial_guess_for_xy(positions, distances, fixed_z)
    initial_guess = np.array([guess_xy[0], guess_xy[1]])
    result = least_squares(residual_func, x0=initial_guess, args=(positions, distances, fixed_z))
    return np.array([result.x[0], result.x[1], fixed_z])


# Helper function to get the latest altitude (depth) reading. If not available, default to a guess of -6 meters.
def read_barometer():
    global latest_alt
    if latest_alt is not None:
        return latest_alt
    else:
        return -6

# Main Receiver Loop: Query distances and perform multilateration
async def main():
    global_timeout = 2.0   # timeout for queries
    polling_delay = 0.250  # delay between query cycles
    
    # Open port
    try:
        reader, writer = await serial_asyncio.open_serial_connection(url='/dev/pts/0', baudrate=9600)
    except Exception as e:
        print("Failed to open serial connection:", e)
        return

    # Start background reader task.
    asyncio.create_task(background_reader(reader))

    # Define transponder channels and lat/lon positions.
    channels = ["M20", "M22", "M25", "M30"]
    transponder_latlon = [
        (59.428465173, 10.465137681),  # t1 (origin)
        (59.428445150, 10.465241581),  # t2
        (59.428427280, 10.465334121),  # t3
        (59.428407132, 10.465439580),  # t4
    ]
    origin_lat, origin_lon = transponder_latlon[0] # get origo
    
    # Convert transponder positions from latlon to meters.
    transponder_positions = []
    for i, (lat, lon) in enumerate(transponder_latlon):
        x, y = latlon_to_meters(lat, lon, origin_lat, origin_lon)
        # Cheesy hack to solve geometry issue when all transponders are on the exact same line 
        # This is quite important. The 10cm discrepency has a minimal effect on the inaccuracy
        if i == 1:
            x += 0.1
            y += 0.1
        transponder_positions.append([x, y, -1])
    transponder_positions = np.array(transponder_positions)
    
    # Main loop
    try:
        print("Receiver started. Polling transponders and performing multilateration...")
        battery = await read_battery_level(reader, writer)
        print("Battery reply:", battery)

        version = await read_version_info(reader, writer)
        print("Version reply:", version)

        power = await set_transmit_power(reader, writer, level=1)
        print("Power reply:", power)
        print("-"*70)
        while True:
            # Query each transponder channel.
            responses = await get_multiple_distances(reader, writer, channels, timeout=global_timeout)

            # Parse distances for each channel.
            distances = []
            for ch, resp in zip(channels, responses):
                if resp:
                    d = parse_distance(resp)
                    distances.append(d)
                    print(f"Channel {ch} distance: {d} m")
                else:
                    distances.append(None)
                    print(f"Channel {ch} no response.")
            
            # Only proceed if all distances were received.
            # This can get changed later, so we use previous readings just to get any position at all
            if None not in distances:
                distances = np.array(distances)
                fixed_z = read_barometer()
                est = multilaterate(transponder_positions, distances, fixed_z)
                est_lat, est_lon = meters_to_latlon(est[0], est[1], origin_lat, origin_lon)
                print("\n[Multilateration Result]")
                print("Estimated position (meters):", est)
                print("Estimated position (lat, lon, z):", est_lat, est_lon, est[2])
                print("-" * 95)
            else:
                print("Incomplete distance measurements. Skipping multilateration cycle.\n")
            
            await asyncio.sleep(polling_delay)
    except KeyboardInterrupt:
        print("Exiting receiver...")
    finally:
        writer.close()
        await writer.wait_closed()

if __name__ == "__main__":
    # Start pymavlink listener to get depth data
    thread = threading.Thread(target=mavlink_listener, daemon=True)
    thread.start()
    # Start main loop and background reader
    asyncio.run(main())
