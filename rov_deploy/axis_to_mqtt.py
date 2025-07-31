#!/usr/bin/env python3
import pygame
import time
import logging
import sys
import paho.mqtt.client as mqtt

# --- CONFIGURATION ---
TRIGGER_AXIS      = 4    # replace with the axis index you found
TRIGGER_THRESHOLD = 0.6  # pull beyond this (0…1)

MQTT_BROKER   = '100.78.45.94'
MQTT_PORT     = 1883
MQTT_USERNAME = 'formula2boat'
MQTT_PASSWORD = 'formula2boat'
MQTT_TOPIC    = 'bluerov2/relay1'

# Logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

# INIT PYGAME
pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
joy.init()

# INIT MQTT
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

prev_pressed = False
logging.info("Watching trigger; pull to send ‘toggled’…")

try:
    while True:
        pygame.event.pump()
        raw = joy.get_axis(TRIGGER_AXIS)
        # some controllers give –1→+1; normalize if so
        val = (raw + 1) / 2 if raw < -0.1 else raw
        pressed = val > TRIGGER_THRESHOLD

        if pressed and not prev_pressed:
            client.publish(MQTT_TOPIC, 'toggled', qos=1)
            logging.info("▶️  Trigger → ‘toggled’")

        prev_pressed = pressed
        time.sleep(0.02)
except Exception as e:
    logging.error(f"[ERROR] Exception in joystick loop: {e}")
    time.sleep(1)
    sys.exit(1)