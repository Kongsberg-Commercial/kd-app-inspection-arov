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
SUB_TOPIC = 'bluerov2/relay1'    # input from mav_to_mqtt.py
PUB_TOPIC = '/rov_deployment'    # drives rov_deploy.py

# State: 0=STOP, 1=DOWN, 2=UP
state    = 0
last_dir = 2     # so first press goes DOWN
timer    = None
lock     = threading.Lock()

def schedule_auto_stop():
    global timer
    if timer:
        timer.cancel()
    timer = threading.Timer(10.0, auto_stop)
    timer.start()


def auto_stop():
    global state, last_dir
    with lock:
        if state in (1,2):
            logging.info("[AUTO] 10 s elapsed → STOP")
            client.publish(PUB_TOPIC, 'stop', qos=1)
            last_dir = state
            state = 0


def toggle_cycle():
    global state, last_dir
    try:
        with lock:
            if state == 0:
                # STOP → alternate direction
                state = 2 if last_dir == 1 else 1
                cmd   = 'deploy' if state == 1 else 'retrieve'
                logging.info(f"[TOGGLE] STOP → {cmd.upper()}")
                client.publish(PUB_TOPIC, cmd, qos=1)
                schedule_auto_stop()
            else:
                # DOWN/UP → STOP
                logging.info("[TOGGLE] sending STOP")
                client.publish(PUB_TOPIC, 'stop', qos=1)
                last_dir = state
                state = 0
                if timer:
                    timer.cancel()
    except Exception as e:
        logging.error(f"Error in toggle_cycle: {e}")


def on_connect(cl, userdata, flags, rc):
    try:
        if rc == 0:
            logging.info("[MQTT] Connected, subscribing…")
            cl.subscribe(SUB_TOPIC, qos=1)
        else:
            logging.error(f"[MQTT] Connection failed: {rc}")
    except Exception as e:
        logging.error(f"Error in on_connect: {e}")


def on_message(cl, userdata, msg):
    try:
        if msg.payload.decode().strip().lower() == 'toggled':
            logging.info("[MQTT] Got ‘toggled’ → cycling")
            toggle_cycle()
    except Exception as e:
        logging.error(f"Error in on_message: {e}")


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


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    client = mqtt.Client()
    client.enable_logger(logging.getLogger())
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect    = on_connect
    client.on_message    = on_message
    client.on_disconnect = on_disconnect

    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()