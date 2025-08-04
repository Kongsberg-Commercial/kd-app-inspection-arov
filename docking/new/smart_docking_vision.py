#!/usr/bin/env python3
"""
Docking controller with MQTT-based decision switching.
Starts platform detection, monitors its output, and switches to yellow line tracking.
"""

import os
import subprocess
import time
import json
import threading
import paho.mqtt.client as mqtt

# Configuration
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PLATFORM_SCRIPT = os.path.join(BASE_DIR, "videoprosessor_test.py")
LINE_SCRIPT     = os.path.join(BASE_DIR, "yellow_line_tracking.py")

BROKER_IP = "100.86.23.108"
BROKER_PORT = 1883
MQTT_TOPIC = "bluerov2/platform"

# Switching logic
SWITCH_DISTANCE_THRESHOLD = 30      # pixels
SWITCH_ANGLE_THRESHOLD = 10         # degrees
ANGLE_FLUCTUATION_THRESHOLD = 15    # degrees
WINDOW_SIZE = 10                    # number of messages to track

recent_angles = []
should_switch = False

def run_script(path):
    """Start a Python script as a subprocess."""
    print(f"▶️ Launching: {os.path.basename(path)}")
    return subprocess.Popen(["python3", path])

def on_message(client, userdata, msg):
    global should_switch, recent_angles

    try:
        data = json.loads(msg.payload.decode())
        distance = abs(data.get("distance", 999))
        angle = data.get("angle", 0)

        # Collect recent angles
        recent_angles.append(angle)
        if len(recent_angles) > WINDOW_SIZE:
            recent_angles.pop(0)

        angle_fluctuation = max(recent_angles) - min(recent_angles)

        print(f"[MQTT] distance={distance}, angle={angle:.1f}, fluct={angle_fluctuation:.1f}")

        # Decide to switch
        if distance < SWITCH_DISTANCE_THRESHOLD and angle_fluctuation < ANGLE_FLUCTUATION_THRESHOLD:
            print("📏 Platform close and stable. Ready to switch.")
            should_switch = True

    except Exception as e:
        print(f"⚠️ Error parsing MQTT message: {e}")

def monitor_mqtt():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(BROKER_IP, BROKER_PORT)
    client.subscribe(MQTT_TOPIC)
    client.loop_forever()

def main():
    global should_switch

    print("🚀 Docking controller started.")
    mqtt_thread = threading.Thread(target=monitor_mqtt, daemon=True)
    mqtt_thread.start()

    # Start platform detection
    platform_proc = run_script(PLATFORM_SCRIPT)

    # Monitor for switch condition
    while platform_proc.poll() is None:
        if should_switch:
            print("✅ Switching to yellow line tracking...")
            platform_proc.terminate()
            break
        time.sleep(0.5)

    # Give it time to release resources
    time.sleep(1)

    # Start yellow line tracking
    run_script(LINE_SCRIPT).wait()
    print("🏁 Docking sequence complete.")

if __name__ == "__main__":
    main()
