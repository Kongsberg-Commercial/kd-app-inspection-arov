#!/usr/bin/env python3
import json
import logging
import time

import paho.mqtt.client as mqtt

# ——— Configuration ———
BROKER_HOST = "localhost"
BROKER_PORT = 1883
TOPIC       = "/hipos_output_cnode_position"
# ————————————————

# MQTT callbacks
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        logging.info(f"Connected to MQTT broker {BROKER_HOST}:{BROKER_PORT}")
        client.subscribe(TOPIC)
    else:
        logging.error(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8", errors="ignore")
        data = json.loads(payload)
    except Exception as e:
        logging.warning(f"Received non-JSON or malformed payload: {e}")
        return

    lat      = data.get("lat")
    lon      = data.get("lon")
    depth    = data.get("depth_m")
    accuracy = data.get("accuracy_m")
    ping     = data.get("ping")
    ts       = data.get("timestamp")

    # Convert timestamp to human‐readable if present
    ts_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts)) if ts else "N/A"

    print(
        f"[{ts_str}] Ping {ping}: "
        f"Lat={lat:.6f}, Lon={lon:.6f}, "
        f"Depth={depth} m, Accuracy=±{accuracy} m"
    )

def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s: %(message)s")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    client.loop_forever()

if __name__ == "__main__":
    main()
