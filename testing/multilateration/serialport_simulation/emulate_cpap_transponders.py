import asyncio
import serial_asyncio
import random
import numpy as np
from numpy.linalg import norm

# -----------------------------------------------------------------
# Conversion Functions: LatLon <-> Meters (using a reference/origo)
# -----------------------------------------------------------------
def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    R = 6371000  # Earth's radius in meters
    dlat = np.deg2rad(lat - origin_lat)
    dlon = np.deg2rad(lon - origin_lon)
    x = R * dlon * np.cos(np.deg2rad(origin_lat))
    y = R * dlat
    return x, y

def meters_to_latlon(x, y, origin_lat, origin_lon):
    R = 6371000
    dlat = y / R
    dlon = x / (R * np.cos(np.deg2rad(origin_lat)))
    lat = origin_lat + np.rad2deg(dlat)
    lon = origin_lon + np.rad2deg(dlon)
    return lat, lon

# -----------------------------------------------------------------
# NMEA Checksum Calculation
# -----------------------------------------------------------------
def nmea_checksum(nmea_message):
    nmea_message = nmea_message.strip()
    if not nmea_message.startswith('$'):
        raise ValueError("NMEA message must start with '$'")
    nmea_message = nmea_message[1:]
    data_list = nmea_message.split('*')
    if len(data_list) != 2:
        raise ValueError("NMEA message must contain exactly one '*'")
    data = data_list[0]
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    return f"{checksum:02X}"

# -----------------------------------------------------------------
# Simulated Reply Functions
# -----------------------------------------------------------------
def simulate_transponder_reply(command, rov, transponders):
    """
    Simulate a transponder interrogation reply (TPR) for a TPI command.
    Instead of using a static time, we update the transponder's distance
    and compute t_time = distance / 1500.
    """
    parts = command.split(',')
    if len(parts) < 3:
        return None
    channel = parts[2]
    status = "1"  # OK
    if channel in transponders:
        # Update distance on demand
        transponder = transponders[channel]
        transponder.update_distance(rov)
        distance = transponder.distance
        sound_speed = 1500.0
        t_time_val = distance / sound_speed
    else:
        # Fallback in case channel is unknown (should not occur)
        t_time_val = random.uniform(0, 0.03)
    t_time = f"{t_time_val:.5f}"
    age = f"{random.uniform(0.001, 0.100):.3f}"
    snr = f"{random.randint(0, 20)}"
    reply_without_checksum = f"$PSIMCUP,TPR,{channel},{status},{t_time},{age},{snr}*"
    checksum = nmea_checksum(reply_without_checksum)
    reply = reply_without_checksum + checksum + "\r\n"
    return reply

def simulate_battery_reply(command):
    chg_level = random.randint(0, 100)
    reply_without_checksum = f"$PSIMCUP,BAA,{chg_level}*"
    checksum = nmea_checksum(reply_without_checksum)
    reply = reply_without_checksum + checksum + "\r\n"
    return reply

def simulate_version_reply(command):
    dsp_ver = "1.0"
    fpga_ver = "1.0"
    reply_without_checksum = f"$PSIMCUP,VEA,DSP,{dsp_ver},FPGA,{fpga_ver}*"
    checksum = nmea_checksum(reply_without_checksum)
    reply = reply_without_checksum + checksum + "\r\n"
    return reply

def simulate_power_reply(command):
    parts = command.split(',')
    if len(parts) < 2:
        return None
    level = parts[2].split('*')[0]
    reply_without_checksum = f"$PSIMCUP,PWA,{level}*"
    checksum = nmea_checksum(reply_without_checksum)
    reply = reply_without_checksum + checksum + "\r\n"
    return reply

# -----------------------------------------------------------------
# Asynchronous Command Handler (now accepts rov and transponders)
# -----------------------------------------------------------------
async def handle_command(command, writer, rov, transponders):
    print("Handling command:", command)
    reply = None
    if command.startswith("$PSIMCUP,TPI,"):
        reply = simulate_transponder_reply(command, rov, transponders)
        await asyncio.sleep(random.uniform(0, 0.125))
    elif command.startswith("$PSIMCUP,BAT"):
        reply = simulate_battery_reply(command)
        await asyncio.sleep(random.uniform(0, 0.5))
    elif command.startswith("$PSIMCUP,VER"):
        reply = simulate_version_reply(command)
        await asyncio.sleep(random.uniform(0, 0.5))
    elif command.startswith("$PSIMCUP,PWR"):
        reply = simulate_power_reply(command)
        await asyncio.sleep(random.uniform(0, 0.5))
    else:
        print("Unknown command type")
    
    if reply:
        print("Outgoing data:", reply.strip())
        writer.write(reply.encode('utf-8'))
        await writer.drain()

# -----------------------------------------------------------------
# ROV and Transponder Classes (unchanged)
# -----------------------------------------------------------------
class ROV:
    def __init__(self, center_x, center_y, center_z, radius=10.0, speed=0.025):
        self.cx = center_x
        self.cy = center_y
        self.cz = center_z
        self.radius = radius
        self.angle = 0.0
        self.speed = speed
        self.x = self.cx + self.radius * np.cos(self.angle)
        self.y = self.cy + self.radius * np.sin(self.angle)
        self.z = self.cz

    def move(self):
        self.angle += self.speed
        self.x = self.cx + self.radius * np.cos(self.angle)
        self.y = self.cy + self.radius * np.sin(self.angle)
        self.z = self.cz + 1.0 * np.sin(self.angle * 0.5)

    def get_pos(self):
        return np.array([self.x, self.y, self.z])

class Transponder:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.distance = 0.0

    def update_distance(self, rov):
        true_dist = norm(rov.get_pos() - self.get_pos())
        noise = np.random.rand() * 0.15
        self.distance = true_dist + noise

    def get_pos(self):
        return np.array([self.x, self.y, self.z])

# -----------------------------------------------------------------
# Main Asynchronous Emulator Loop
# -----------------------------------------------------------------
async def main():
    port = "/dev/pts/2"
    try:
        reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=9600)
    except Exception as e:
        print("Error opening port:", e)
        return

    # Create an ROV moving in a circular path.
    rov = ROV(center_x=15, center_y=30, center_z=-8, radius=10.0, speed=0.025)

    # Define transponder positions in lat–lon.
    transponder_latlon = [
        (59.428465173, 10.465137681),   # t1 (origin)  -> channel "M20"
        (59.428445150, 10.465241581),   # t2          -> channel "M22"
        (59.428427280, 10.465334121),   # t3          -> channel "M25"
        (59.428407132, 10.465439580),   # t4          -> channel "M30"
    ]
    channels = ["M20", "M22", "M25", "M30"]
    origin_lat, origin_lon = transponder_latlon[0]

    # Create transponder objects (convert latlon -> meters) and assign to channels.
    transponders = {}
    for i, (lat, lon) in enumerate(transponder_latlon):
        x, y = latlon_to_meters(lat, lon, origin_lat, origin_lon)
        t = Transponder(x, y, -1)  # fixed z = -1 m
        if channels[i] == "M22":
            # Slight offset to improve geometry if needed.
            t.x += 0.1
            t.y += 0.1
        transponders[channels[i]] = t

    print(f"Simulated responder running on {port}...")

    while True:
        try:
            # Update the ROV's position on each loop.
            rov.move()
            # Read an incoming command.
            line = await reader.readline()
            if line:
                command = line.decode('utf-8').strip()
                print("Incoming data:", command)
                # Pass the current rov and transponders to the command handler.
                asyncio.create_task(handle_command(command, writer, rov, transponders))
            print("Current ROV position:", rov.get_pos())
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as ex:
            print("Error:", ex)
    writer.close()
    await writer.wait_closed()

if __name__ == '__main__':
    asyncio.run(main())
