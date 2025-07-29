#!/usr/bin/env python3
import time, math, json
import paho.mqtt.client as mqtt

# ——— MQTT configuration ———
BROKER_HOST = "localhost"
BROKER_PORT = 1883
TOPIC       = "/hipos_input_gps_and_heading"
# ——— Track configuration ———
CENTER_LAT  = 59.428465173    # circle center
CENTER_LON  = 10.465137681
RADIUS_M    = 10        # radius in meters
PERIOD_S    = 60          # seconds per revolution
# ——————————————

# Precompute degree‐per‐meter factors
DEG_PER_M_LAT = 1 / 111320.0
DEG_PER_M_LON = 1 / (111320.0 * math.cos(math.radians(CENTER_LAT)))

# Setup MQTT
client = mqtt.Client()
client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
client.loop_start()

start = time.time()
try:
    while True:
        # calculate position on circle
        t = (time.time() - start) % PERIOD_S
        theta = 2 * math.pi * (t / PERIOD_S)
        # offsets in meters
        dy =  RADIUS_M * math.sin(theta)
        dx =  RADIUS_M * math.cos(theta)
        # convert to decimal degrees
        lat = CENTER_LAT + dy * DEG_PER_M_LAT
        lon = CENTER_LON + dx * DEG_PER_M_LON
        # heading: tangent to circle (theta + 90°), mod 360
        heading = (math.degrees(theta) + 90) % 360

        payload = json.dumps({
            "lat":     round(CENTER_LAT, 10),
            "lon":     round(CENTER_LON, 10),
            "heading": round(0, 1)
        })
        client.publish(TOPIC, payload)
        print(f"Published → {payload}")

        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    client.loop_stop()
    client.disconnect()
