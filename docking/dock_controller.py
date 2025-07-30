#!/usr/bin/env python3
"""
Dock controller node.
Configurable MQTT broker IP, port, topic, MAVLink connection, and button mask at the top.
"""
import time
import threading
import json
import paho.mqtt.client as mqtt
from pymavlink import mavutil

# === Configuration ===
BROKER_IP       = '100.78.45.94'          # MQTT broker IP address
BROKER_PORT     = 1883                    # MQTT broker port
PLATFORM_TOPIC  = 'bluerov2/platform'     # MQTT topic for platform data
# For MAVLink: to listen on port 7777 use 'udpin:0.0.0.0:7777';
#              to send to remote at 192.168.2.20:7778 use 'udpout:192.168.2.20:7778'
MAVLINK_CONN    = 'udpin:0.0.0.0:7777'     # MAVLink connection string
BUTTONS_MASK    = 1 + (1 << 3) + (1 << 7)  # Buttons bitmask

# Shared state
distance_to_mid = 0.0
angle_from_mid  = 0.0

# Setup MAVLink connection
master = mavutil.mavlink_connection(MAVLINK_CONN)

# Control helper
def control(x=0, y=0, z=500, r=0):
    master.mav.manual_control_send(
        master.target_system,
        x, y, z, r,
        BUTTONS_MASK
    )

# Arm/disarm routines
def arm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Waiting to arm...")
    master.motors_armed_wait()
    print("✅ Armed")


def disarm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Waiting to disarm...")
    master.motors_disarmed_wait()
    print("✅ Disarmed")

# MQTT handling
def on_message(client, userdata, msg):
    global distance_to_mid, angle_from_mid
    try:
        payload = json.loads(msg.payload.decode())
        if msg.topic == PLATFORM_TOPIC:
            distance_to_mid = float(payload["distance"])
            angle_from_mid  = float(payload["angle"])
    except Exception as e:
        print(f"MQTT handling error: {e}")

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect(BROKER_IP, BROKER_PORT)
mqtt_client.subscribe(PLATFORM_TOPIC)
mqtt_thread = threading.Thread(target=mqtt_client.loop_forever, daemon=True)
mqtt_thread.start()

# Wait for MAVLink heartbeat & arm vehicle
master.wait_heartbeat()
arm_vehicle()

# PID constants
Kp_distance, Ki_distance, Kd_distance = 12.0, 0.05, 10.0
Kp_angle,    Kd_angle               = 16.0,  0.5

prev_dist_err  = 0.0
prev_ang_err   = 0.0
integral_error = 0.0
prev_time      = time.time()

print("Control loop starting...")
try:
    while True:
        now = time.time()
        dt  = max(now - prev_time, 1e-3)

        dist_err = distance_to_mid
        ang_err  = angle_from_mid

        d_dist = (dist_err - prev_dist_err) / dt
        d_ang  = (ang_err  - prev_ang_err ) / dt

        integral_error = max(-500, min(500, integral_error + dist_err * dt))

        y_out = int(max(-400, min(400,
                    Kp_distance * dist_err
                  + Ki_distance * integral_error
                  + Kd_distance * d_dist)))
        r_out = int(max(-400, min(400,
                    Kp_angle * ang_err
                  + Kd_angle * d_ang)))

        # small forward thrust if roughly centered
        x_out = 1000 if abs(dist_err) < 100 and abs(ang_err) < 10 else 0

        control(x_out, y_out, 500, r_out)
        print(f"y={y_out} r={r_out} dist={dist_err:.1f} ang={ang_err:.1f}")

        prev_dist_err = dist_err
        prev_ang_err  = ang_err
        prev_time      = now
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping and disarming...")
    control(0, 0, 500, 0)
    disarm_vehicle()