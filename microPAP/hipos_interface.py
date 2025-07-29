#!/usr/bin/env python3
import threading
import socket
import json
import time
import logging
from datetime import datetime, timezone

from pyproj import Transformer
import paho.mqtt.client as mqtt

# ——— Configuration ———
BROKER_HOST   = "100.78.45.94"
BROKER_PORT   = 1883
TOPIC_IN      = "sommer/position"
USERNAME      = "formula2boat"
PASSWORD      = "formula2boat"

INPUT_HOST    = "127.0.0.1"
GPS_PORT      = 6001    # fake_gps.py port
HEADING_PORT  = 6002    # fake_heading.py port

OUTPUT_HOST   = "0.0.0.0"
OUTPUT_PORT   = 5005    # PSIMSSB UDP port

# cNODE‐TCP destination
POSITION_HOST = "192.168.2.2"
POSITION_PORT = 6003

UTM_EPSG      = "32632"
# ————————————————

# Prepare UTM→WGS84 transformer
transformer = Transformer.from_crs(f"epsg:{UTM_EPSG}", "epsg:4326", always_xy=True)

# Setup UDP sockets
gps_sock     = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
heading_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

output_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
output_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
output_sock.bind((OUTPUT_HOST, OUTPUT_PORT))

# MQTT client (for input)
client = mqtt.Client()

# ——— NMEA sentence builders ———
def _format_coord(value, is_lat):
    hemi = 'N' if value >= 0 and is_lat else \
           'S' if is_lat else \
           'E' if value >= 0 else 'W'
    v = abs(value)
    d = int(v)
    m = (v - d) * 60
    fmt = f"{d:02d}{m:07.4f}" if is_lat else f"{d:03d}{m:07.4f}"
    return fmt, hemi

def _nmea_checksum(body: str):
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f"*{cs:02X}"

def make_gpgga(lat, lon):
    now   = datetime.now(timezone.utc)
    stamp = now.strftime("%H%M%S") + ".00"
    lat_s, lat_h = _format_coord(lat, True)
    lon_s, lon_h = _format_coord(lon, False)
    body = f"GPGGA,{stamp},{lat_s},{lat_h},{lon_s},{lon_h},1,08,0.9,5.0,M,0.0,M,,"
    return f"${body}{_nmea_checksum(body)}\r\n".encode("ascii")

def make_gphdt(heading):
    body = f"GPHDT,{heading:06.2f},T"
    return f"${body}{_nmea_checksum(body)}\r\n".encode("ascii")

# ——— MQTT callbacks ———
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info(f"MQTT connected to {BROKER_HOST}:{BROKER_PORT}")
        client.subscribe(TOPIC_IN)
        logging.info(f"Subscribed to MQTT topic: {TOPIC_IN}")
    else:
        logging.error(f"MQTT connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8", errors="ignore")
        data    = json.loads(payload)
        lat     = float(data["lat"])
        lon     = float(data["lng"])      # note: new field is 'lng'
        hdg     = float(data["heading"])
    except Exception as e:
        logging.warning(f"Bad MQTT payload on {msg.topic}: {e}")
        return

    # Build NMEA sentences
    gga = make_gpgga(lat, lon)
    hdt = make_gphdt(hdg)

    # Debug print
    print(f"[bridge → HiPOS] UDP:{GPS_PORT} {gga.decode().strip()}")
    print(f"[bridge → HiPOS] UDP:{HEADING_PORT} {hdt.decode().strip()}")

    # Send to fake GPS/heading inputs
    gps_sock.sendto(gga, (INPUT_HOST, GPS_PORT))
    heading_sock.sendto(hdt, (INPUT_HOST, HEADING_PORT))

client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

# ——— Thread loops ———
def mqtt_loop():
    while True:
        try:
            client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            client.loop_forever()
        except Exception as e:
            logging.error(f"MQTT error: {e}; retrying in 5s")
            time.sleep(5)

def udp_loop():
    while True:
        data, _ = output_sock.recvfrom(4096)
        text     = data.decode("ascii", errors="ignore").strip()
        print(f"[HiPOS → bridge] {text}")
        if not text.startswith('$PSIMSSB'):
            continue

        parts = text.split('*')[0].split(',')
        if len(parts) < 12 or parts[5] != 'U':
            continue

        try:
            northing = float(parts[8])
            easting  = float(parts[9])
        except ValueError:
            continue

        lon, lat = transformer.transform(easting, northing)
        pos_str  = f"{lat};{lon}"
        print(f"[bridge → cNODE TCP] {pos_str}")

        # Send over TCP
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tsock:
                tsock.settimeout(2)
                tsock.connect((POSITION_HOST, POSITION_PORT))
                tsock.sendall(pos_str.encode("ascii"))
        except Exception as e:
            logging.error(f"Failed to send to cNODE {POSITION_HOST}:{POSITION_PORT}: {e}")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s: %(message)s")
    threading.Thread(target=mqtt_loop, daemon=True).start()
    threading.Thread(target=udp_loop,   daemon=True).start()
    logging.info("HiPOS↔cNODE-TCP bridge running")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down…")
        client.disconnect()
