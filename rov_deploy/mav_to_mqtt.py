#!/usr/bin/env python3
import time
import logging
import sys
from pymavlink import mavutil
import paho.mqtt.client as mqtt

# --- MAVLink Connection ---
mav = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# --- MQTT Configuration ---
MQTT_BROKER   = '100.78.45.94'
MQTT_PORT     = 1883
MQTT_USERNAME = 'formula2boat'
MQTT_PASSWORD = 'formula2boat'
MQTT_TOPIC    = 'bluerov2/relay1'

# Logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

client = mqtt.Client()
client.enable_logger(logging.getLogger())
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)


def on_disconnect(client, userdata, rc):
    logging.warning(f"[MQTT] Disconnected (rc={rc}), reconnecting…")
    while True:
        try:
            client.reconnect()
            logging.info("[MQTT] Reconnected")
            break
        except Exception as e:
            logging.error(f"[MQTT] Reconnect failed: {e}, retrying in 5s")
            time.sleep(5)

client.on_disconnect = on_disconnect

client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

logging.info("Listening for relay1 toggles…")
while True:
    try:
        msg = mav.recv_match(type='COMMAND_LONG', blocking=True)
        if not msg:
            continue
        # MAV_CMD_DO_SET_RELAY = 181; relay 1 → param1 == 1
        if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_RELAY and msg.param1 == 1:
            client.publish(MQTT_TOPIC, 'toggled', qos=1)
            logging.info("▶️  Published ‘toggled’")
    except Exception as e:
        logging.error(f"[ERROR] Exception in main loop: {e}")
        time.sleep(1)
