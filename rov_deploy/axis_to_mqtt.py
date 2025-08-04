# joystick_toggle.py
#!/usr/bin/env python3
import pygame
import time
import logging
import sys
import paho.mqtt.client as mqtt

# --- CONFIGURATION ---
# Axis indices for triggers
PLATFORM_AXIS      = 4   # original shoulder trigger for platform
DOCK_AXIS          = 5   # other shoulder trigger for docking
# thresholds
PLATFORM_THRESHOLD = 0.6
DOCK_THRESHOLD     = 0.6

MQTT_BROKER   = '100.86.23.108'
MQTT_PORT     = 1883
MQTT_USERNAME = 'formula2boat'
MQTT_PASSWORD = 'formula2boat'
# Topics match relay_toggle
MQTT_TOPIC_PLAT = 'bluerov2/relay1'
MQTT_TOPIC_DOCK = 'bluerov2/relay2'

# Logging
tlogging = logging.getLogger(); logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

# Initialize joystick
def init_joystick():
    pygame.init(); pygame.joystick.init()
    try:
        j = pygame.joystick.Joystick(0); j.init(); return j
    except Exception as e:
        logging.error(f"Joystick init failed: {e}"); sys.exit(1)

# MQTT setup
client = mqtt.Client()
client.enable_logger()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
def on_disconnect(cl, userdata, rc):
    logging.warning(f"[MQTT] Disconnected (rc={rc}), reconnecting…")
    while True:
        try:
            cl.reconnect(); logging.info("[MQTT] Reconnected"); break
        except Exception as e:
            logging.error(f"Reconnect failed: {e}, retry in 5s"); time.sleep(5)
client.on_disconnect = on_disconnect
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

joy = init_joystick()
prev_plat = prev_dock = False
logging.info("Listening on axes %d (platform) & %d (docking)…", PLATFORM_AXIS, DOCK_AXIS)

try:
    while True:
        pygame.event.pump()
        # Platform trigger
        raw_p = joy.get_axis(PLATFORM_AXIS)
        val_p = (raw_p+1)/2 if raw_p< -0.1 else raw_p
        pressed_p = val_p > PLATFORM_THRESHOLD
        if pressed_p and not prev_plat:
            logging.info("▶️ Platform trigger → 'toggled'")
            client.publish(MQTT_TOPIC_PLAT, 'toggled', qos=1)
        prev_plat = pressed_p
        # Docking trigger
        raw_d = joy.get_axis(DOCK_AXIS)
        val_d = (raw_d+1)/2 if raw_d< -0.1 else raw_d
        pressed_d = val_d > DOCK_THRESHOLD
        if pressed_d and not prev_dock:
            logging.info("▶️ Docking trigger → 'toggled'")
            client.publish(MQTT_TOPIC_DOCK, 'toggled', qos=1)
        prev_dock = pressed_d

        time.sleep(0.02)
except KeyboardInterrupt:
    logging.info("Exiting on interrupt…"); sys.exit(0)
