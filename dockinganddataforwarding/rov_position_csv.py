#!/usr/bin/env python3
"""
rov_logger.py

Listens on MQTT topic "sommer/rovposition" and logs timestamped
latitude, longitude, and down values to a CSV file.
"""

import os
import csv
import json
import logging
from datetime import datetime, timezone
import paho.mqtt.client as mqtt

# ——— Configuration ———
BROKER_HOST = "100.78.45.94"    
BROKER_PORT = 1883
TOPIC       = "sommer/rovposition"
USERNAME    = "formula2boat"
PASSWORD    = "formula2boat"

OUTPUT_FILE = "rov_positions.csv"
# ————————————————

def ensure_header(path):
    """Write CSV header if file doesn't exist or is empty."""
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_utc", "lat", "lng", "down"])

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info(f"Connected to MQTT broker, subscribing to {TOPIC}")
        client.subscribe(TOPIC)
    else:
        logging.error(f"MQTT connect failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8", errors="ignore")
        data = json.loads(payload)
        lat = float(data["lat"])
        lng = float(data["lng"])
        down = float(data.get("down", 0.0))
    except Exception as e:
        logging.warning(f"Failed to parse message: {e}")
        return

    timestamp = datetime.now(timezone.utc).isoformat()
    row = [timestamp, lat, lng, down]

    # Append to CSV
    try:
        with open(OUTPUT_FILE, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)
        logging.info(f"Logged: {row}")
    except Exception as e:
        logging.error(f"Failed to write to {OUTPUT_FILE}: {e}")

def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s: %(message)s")

    ensure_header(OUTPUT_FILE)

    client = mqtt.Client()
    client.username_pw_set(USERNAME, PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message

    while True:
        try:
            client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            client.loop_forever()
        except Exception as e:
            logging.error(f"MQTT error: {e}; retrying in 5 seconds")
            import time; time.sleep(5)

if __name__ == "__main__":
    main()
