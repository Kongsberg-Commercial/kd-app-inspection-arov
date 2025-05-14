import time
import math
import sys
import threading
import numpy as np
import json
import paho.mqtt.client as mqtt
from pymavlink import mavutil

# Shared data
distance_to_mid = 0
angle_from_mid = 0
distance_height = 50  # from zu
buttons = 1 + 1 << 3 + 1 << 7

def control(x=0, y=0, z=500, r=0):
    master.mav.manual_control_send(
        master.target_system,
        x,
        y,
        z,
        r,
        buttons
    )

def arm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Waiting for the vehicle to arm...")
    master.motors_armed_wait()
    print("✅ Armed!")

def disarm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Waiting for the vehicle to disarm...")
    master.motors_disarmed_wait()
    print("✅ Disarmed!")

# --- MQTT Setup ---
def on_message(client, userdata, msg):
    global distance_to_mid, angle_from_mid, distance_height
    try:
        payload = json.loads(msg.payload.decode())

        if msg.topic == "/pipe":
            distance_to_mid = float(payload["distance"])
            angle_from_mid = float(payload["angle"])
        elif msg.topic == "/odom":
            distance_height = float(payload["position"]["zu"])
    except Exception as e:
        print("Error handling MQTT message:", e)

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect("localhost", 1883)
mqtt_client.subscribe("/pipe")
mqtt_client.subscribe("/odom")
mqtt_thread = threading.Thread(target=mqtt_client.loop_forever, daemon=True)
mqtt_thread.start()

# --- MAVLink Setup ---
master = mavutil.mavlink_connection('udpin:0.0.0.0:7777')
master.wait_heartbeat()
boot_time = time.time()
arm_vehicle()

# PID constants
Kp_distance = 12
Ki_distance = 0.05
Kd_distance = 10
Kp_angle = 16.0
Kd_angle = 0.5
Kp_z = 0 
Ki_z = 0
Kd_z = 0

SP = 50  # cm setpoint
acc_e = e_prev = 0
distance_height_prev = 50

prev_distance_error = 0
prev_angle_error = 0
integral_error = 0
prev_time = time.time()
start_time = time.time()
delta_time = 0

# --- Main Control Loop ---
try:
    while delta_time < 1000.0:
        current_time = time.time()
        dt = current_time - prev_time or 0.1

        distance_error = distance_to_mid
        angle_error = angle_from_mid

        d_distance = (distance_error - prev_distance_error) / dt
        d_angle = (angle_error - prev_angle_error) / dt

        integral_error += distance_error * dt
        integral_error = max(-500, min(500, integral_error))

        y_output = Kp_distance * distance_error + Ki_distance * integral_error + Kd_distance * d_distance
        r_output = Kp_angle * angle_error + Kd_angle * d_angle
        y_output = max(-400, min(400, y_output))
        r_output = max(-400, min(400, r_output))

        if abs(angle_error) < 30 and abs(distance_error) < 150 and distance_error != 0:
            forward_speed = min(2000, 1000)
        else:
            forward_speed = 0

        if abs(distance_error) > 200:
            r_output = 0

        if distance_height_prev == distance_height:
            distance_height = SP
        e = SP - distance_height
        acc_e += e * dt
        acc_e = max(-500, min(500, acc_e))
        u = Kp_z * e + Ki_z * acc_e + Kd_z * (e - e_prev) / dt
        z_output = 500 + u
        e_prev = e
        distance_height_prev = distance_height

        control(x=int(forward_speed), y=int(y_output), z=int(z_output), r=int(r_output))
        print(f"[{round(delta_time, 1)}s] z={round(z_output, 2)} | y={int(y_output)} | r={int(r_output)} | dist={distance_to_mid:.1f} | ang={angle_from_mid:.1f} | zu={distance_height:.2f}")
        
        prev_distance_error = distance_error
        prev_angle_error = angle_error
        prev_time = current_time
        delta_time = current_time - start_time

        time.sleep(0.1)

except KeyboardInterrupt:
    print("❌ Stopping...")

# Stop the motors and disarm
control()
disarm_vehicle()
