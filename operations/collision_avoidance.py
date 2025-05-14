import json
import time
import threading
import paho.mqtt.client as mqtt
from pymavlink import mavutil

from pymavlink.dialects.v20 import common as mavlink2  # Add this at top with other imports

IGNORE = 32767

# Shared variable for zu
current_zu = 100.0  # Default high so it doesn't trigger immediately
buttons = 1 + (1 << 3) + (1 << 7)  # Manual control buttons
reset = False

# --- MQTT message handler ---
def on_message(client, userdata, msg):
    global current_zu
    try:
        if msg.topic == "/odom":
            payload = json.loads(msg.payload.decode())
            current_zu = float(payload["position"]["zu"])
    except Exception as e:
        print("Error parsing /odom message:", e)

# --- MQTT setup ---
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect("localhost", 1883)
mqtt_client.subscribe("/odom")
mqtt_thread = threading.Thread(target=mqtt_client.loop_forever, daemon=True)
mqtt_thread.start()

# --- MAVLink setup ---
print("Connecting to MAVLink...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:7777')
master.wait_heartbeat()
print("? MAVLink connected!")

def arm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Waiting for the vehicle to arm...")
    master.motors_armed_wait()
    print("? Armed!")

def disarm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Waiting for the vehicle to disarm...")
    master.motors_disarmed_wait()
    print("? Disarmed!")

def control(x=IGNORE, y=IGNORE, z=IGNORE, r=IGNORE):
    master.mav.manual_control_send(
        master.target_system,
        x,
        y,
        z,
        r,
        buttons
    )

# --- Main Loop ---
try:
    arm_vehicle()
    print("Running z-controller override (Ctrl+C to stop)...")
    while True:
        if current_zu <= 0.5 and current_zu > 0.01:
            control(z=600)  # Full upward thrust
            print(f"[z-override] zu={current_zu:.2f} m ? z=0 (UP)")
            reset = True
        else:
            if reset == True:
              control(z=IGNORE)  # Neutral thrust
              print(f"[z-hold] zu={current_zu:.2f} m ? z=500 (Neutral)")
              reset = False
            else:
              control(z=IGNORE)  # Neutral thrust
              print(f"[z-hold] zu={current_zu:.2f} m ? z=500 (Neutral)")
        time.sleep(0.03)

except KeyboardInterrupt:
    print("\nCtrl+C detected! Stopping...")

finally:
    # Always stop motors and disarm safely
    control()  # Neutral control
    disarm_vehicle()
