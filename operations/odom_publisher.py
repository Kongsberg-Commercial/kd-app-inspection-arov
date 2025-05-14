import threading
import serial
from pymavlink import mavutil
import paho.mqtt.client as mqtt
import time
import json
import math

# Shared state
state = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "lat": 0.0, "lon": 0.0, "alt": 0.0,
    "vx": 0.0, "vy": 0.0, "vz": 0.0,
    "zu": 0.0,  # ultrasonic height
    "speed": 0.0  # NEW: groundspeed
}
lock = threading.Lock()

# Global MAVLink connection
connection = None

def connect_mavlink():
    global connection
    while True:
        try:
            print("🔌 Connecting to MAVLink on udpin:0.0.0.0:7777...")
            connection = mavutil.mavlink_connection('udpin:0.0.0.0:7777')
            connection.wait_heartbeat(timeout=5)
            print(f"✅ Connected to system {connection.target_system}, component {connection.target_component}")
            return
        except Exception as e:
            print(f"⚠ MAVLink connection failed: {e}")
            time.sleep(2)

# Connect to MQTT
mqtt_client = mqtt.Client()
mqtt_client.connect("localhost", 1883)

# MAVLink listener thread
def mavlink_listener():
    global connection
    last_msg_time = time.time()

    while True:
        msg = connection.recv_match(blocking=False)
        if msg:
            last_msg_time = time.time()  # Reset timeout
            msg_type = msg.get_type()

            with lock:
                if msg_type == "ATTITUDE":
                    state["roll"] = math.degrees(msg.roll)
                    state["pitch"] = math.degrees(msg.pitch)
                    state["yaw"] = math.degrees(msg.yaw)

                elif msg_type == "GLOBAL_POSITION_INT":
                    state["lat"] = msg.lat / 1e7
                    state["lon"] = msg.lon / 1e7
                    state["alt"] = msg.relative_alt / 1000.0

                elif msg_type == "VFR_HUD":
                    state["speed"] = msg.groundspeed  # NEW: store groundspeed
                    heading_rad = math.radians(msg.heading)
                    state["vx"] = state["speed"] * math.cos(heading_rad)
                    state["vy"] = state["speed"] * math.sin(heading_rad)
                    state["vz"] = -msg.climb
        else:
            time.sleep(0.1)  # Avoid CPU hogging

        # Reconnect if no message for 5 seconds
        if time.time() - last_msg_time > 5:
            print("⚠ No MAVLink messages received for 5 seconds. Reconnecting...")
            try:
                connection.close()
            except:
                pass
            connect_mavlink()
            last_msg_time = time.time()

# Ultrasonic sensor reader thread
def ultrasonic_reader():
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    ser = None

    while True:
        if ser is None:
            for port in ports:
                try:
                    ser = serial.Serial(port, 115200, timeout=0.1)
                    print(f"📡 Ultrasonic sensor connected to {port}")
                    break
                except Exception:
                    continue

        try:
            ser.write(b'\x55')
            time.sleep(0.1)
            if ser.in_waiting > 0:
                time.sleep(0.004)
                if ser.read(1) == b'\xff':
                    rest = ser.read(3)
                    if len(rest) == 3:
                        chksum = (0xff + rest[0] + rest[1]) & 0xFF
                        if rest[2] == chksum:
                            dist_cm = (rest[0] << 8 | rest[1]) / 10.0
                            with lock:
                                state["zu"] = dist_cm / 100.0
        except Exception as e:
            print(f"🔌 Lost ultrasonic connection: {e}")
            ser = None
        time.sleep(0.05)

# Start threads
connect_mavlink()
threading.Thread(target=mavlink_listener, daemon=True).start()
threading.Thread(target=ultrasonic_reader, daemon=True).start()

# Main loop: publish /odom
try:
    print("📡 Publishing /odom at 10Hz...")
    while True:
        with lock:
            odom_msg = {
                "position": {
                    "x": round(state["lat"], 7),
                    "y": round(state["lon"], 7),
                    "z": round(state["alt"], 2),
                    "zu": round(state["zu"], 2)
                },
                "orientation": {
                    "roll": round(state["roll"], 2),
                    "pitch": round(state["pitch"], 2),
                    "yaw": round(state["yaw"], 2)
                },
                "velocity": {
                    "x": round(state["vx"], 2),
                    "y": round(state["vy"], 2),
                    "z": round(state["vz"], 2),
                    "speed": round(state["speed"], 2)  # NEW: Add speed here
                },
                "velocity_source": "VFR_HUD",
                "source": "IMU+GPS+Ultrasonic",
                "timestamp": time.time()
            }

        mqtt_client.publish("/odom", json.dumps(odom_msg))
        print(f"📤 Published /odom: {odom_msg}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("❌ Stopped by user.")
    mqtt_client.disconnect()
