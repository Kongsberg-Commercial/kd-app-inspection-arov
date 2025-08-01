# relay_toggle.py
#!/usr/bin/env python3
import threading
import time
import logging
import paho.mqtt.client as mqtt

# --- MQTT Configuration ---
MQTT_BROKER   = '100.78.45.94'
MQTT_PORT     = 1883
MQTT_USERNAME = 'formula2boat'
MQTT_PASSWORD = 'formula2boat'

# --- Topics ---
# Platform control
PLATFORM_SUB_TOPIC = 'bluerov2/relay1'
PLATFORM_PUB_TOPIC = '/rov_deployment'
# Docking control
DOCK_SUB_TOPIC     = 'bluerov2/relay2'
DOCK_PUB_TOPIC     = '/docking/control'

# State vars
platform_state = 0  # 0=STOP, 1=DOWN, 2=UP
platform_last  = 2
dock_state     = 0  # 0=stopped, 1=docking
lock = threading.Lock()

# Logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

# Automatic stop for platform (after 10s)
platform_timer = None
def schedule_platform_stop():
    global platform_timer
    if platform_timer:
        platform_timer.cancel()
    platform_timer = threading.Timer(10.0, platform_auto_stop)
    platform_timer.start()

def platform_auto_stop():
    global platform_state, platform_last
    with lock:
        if platform_state in (1,2):
            logging.info("[AUTO] Platform 10s elapsed → STOP")
            client.publish(PLATFORM_PUB_TOPIC, 'stop', qos=1)
            platform_last = platform_state
            platform_state = 0

# Toggle functions

def platform_toggle():
    global platform_state, platform_last
    with lock:
        if platform_state == 0:
            # STOP → alternate direction
            platform_state = 2 if platform_last == 1 else 1
            cmd = 'deploy' if platform_state == 1 else 'retrieve'
            logging.info(f"[PLATFORM] STOP → {cmd.upper()}")
            client.publish(PLATFORM_PUB_TOPIC, cmd, qos=1)
            schedule_platform_stop()
        else:
            # DOWN/UP → STOP
            logging.info("[PLATFORM] sending STOP")
            client.publish(PLATFORM_PUB_TOPIC, 'stop', qos=1)
            platform_last = platform_state
            platform_state = 0
            if platform_timer:
                platform_timer.cancel()


def dock_toggle():
    global dock_state
    with lock:
        if dock_state == 0:
            cmd = 'start'; dock_state = 1
        else:
            cmd = 'stop'; dock_state = 0
        logging.info(f"[DOCKING] toggling → {cmd.upper()}")
        client.publish(DOCK_PUB_TOPIC, cmd, qos=1)

# MQTT callbacks

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info("[MQTT] Connected, subscribing…")
        client.subscribe([(PLATFORM_SUB_TOPIC,1), (DOCK_SUB_TOPIC,1)])
    else:
        logging.error(f"[MQTT] Connection failed: rc={rc}")


def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode('utf-8').strip().lower()
    if payload != 'toggled':
        return
    if topic == PLATFORM_SUB_TOPIC:
        logging.info("[MQTT] Platform toggle received")
        platform_toggle()
    elif topic == DOCK_SUB_TOPIC:
        logging.info("[MQTT] Docking toggle received")
        dock_toggle()


def on_disconnect(client, userdata, rc):
    logging.warning(f"[MQTT] Disconnected (rc={rc}), reconnecting…")
    while True:
        try:
            client.reconnect(); logging.info("[MQTT] Reconnected"); break
        except Exception as e:
            logging.error(f"Reconnect failed: {e}, retry in 5s"); time.sleep(5)

# Entry point
if __name__ == '__main__':
    client = mqtt.Client()
    client.enable_logger()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect    = on_connect
    client.on_message    = on_message
    client.on_disconnect = on_disconnect
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()

