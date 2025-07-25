import threading
import socket
import json
import time
import logging
from datetime import datetime, timezone
from pyproj import Transformer
import paho.mqtt.client as mqtt

# --- Configuration ---
broker_host     = "localhost"
broker_port     = 1883
topic_in        = "/hipos_input_gps_and_heading"

input_host      = "127.0.0.1"
gps_port        = 6001    # fake_gps.py port
heading_port    = 6002    # fake_heading.py port

output_host     = "0.0.0.0"
output_port     = 5005    # your PSIMSSB UDP port

# NEW: where to send the cNODE position over TCP
position_host   = "192.168.2.2"
position_port   = 6003    # adjust as required

utm_epsg        = "32632"
# ----------------------

# Prepare UTM→WGS84 transformer
transformer = Transformer.from_crs(f"epsg:{utm_epsg}", "epsg:4326", always_xy=True)

# Setup UDP sockets
gps_sock     = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
heading_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

output_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
output_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
output_sock.bind((output_host, output_port))

# MQTT client setup (only for input)
client = mqtt.Client()

# --- NMEA helpers matching fake_gps.py / fake_heading.py ---
def _format_coord(value, is_lat):
    hemi = 'N' if value>=0 and is_lat else 'S' if is_lat else 'E' if value>=0 else 'W'
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
    now = datetime.now(timezone.utc)
    stamp = now.strftime("%H%M%S") + ".00"
    lat_s, lat_h = _format_coord(lat, True)
    lon_s, lon_h = _format_coord(lon, False)
    body = f"GPGGA,{stamp},{lat_s},{lat_h},{lon_s},{lon_h},1,08,0.9,5.0,M,0.0,M,,"
    sentence = f"${body}{_nmea_checksum(body)}\r\n"
    return sentence.encode("ascii")

def make_gphdt(heading):
    body = f"GPHDT,{heading:06.2f},T"
    sentence = f"${body}{_nmea_checksum(body)}\r\n"
    return sentence.encode("ascii")

# --- MQTT callbacks ---
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        logging.info(f"Connected to MQTT broker {broker_host}:{broker_port}")
        client.subscribe(topic_in)
    else:
        logging.error(f"MQTT connection failed with code {rc}")

def on_message(client, userdata, msg):
    payload_str = msg.payload.decode('utf-8', errors='ignore')
    print(f"[MQTT → bridge] {msg.topic} {payload_str}")
    print("WE GOT MQTT MESSAGE!!!")

    try:
        data = json.loads(payload_str)
        lat = float(data['lat'])
        lon = float(data['lon'])
        hd  = float(data['heading'])
    except Exception as e:
        logging.warning(f"Invalid payload: {e}")
        return

    gga = make_gpgga(lat, lon)
    hdt = make_gphdt(hd)

    # debug print exactly what we send
    print(f"[bridge → HiPOS] UDP:{gps_port} {gga.decode().strip()}")
    print(f"[bridge → HiPOS] UDP:{heading_port} {hdt.decode().strip()}")

    gps_sock.sendto(gga, (input_host, gps_port))
    heading_sock.sendto(hdt, (input_host, heading_port))

client.on_connect = on_connect
client.on_message = on_message

# --- Thread loops ---
def mqtt_loop():
    while True:
        try:
            client.connect(broker_host, broker_port, keepalive=60)
            client.loop_forever()
        except Exception as e:
            logging.error(f"MQTT error: {e}, retrying in 5s")
            time.sleep(5)

def udp_loop():
    while True:
        data, _ = output_sock.recvfrom(4096)
        text = data.decode('ascii', errors='ignore').strip()
        print(f"[HiPOS → bridge] {text}")
        if not text.startswith('$PSIMSSB'):
            continue

        parts = text.split('*')[0].split(',')
        # Expect UTM output: parts[5] == 'U'
        if len(parts) < 12 or parts[5] != 'U':
            continue

        try:
            # **SWAPPED**: field[8] is NORTHING, field[9] is EASTING
            northing = float(parts[8])
            easting  = float(parts[9])
        except ValueError:
            continue

        # Convert UTM → lon/lat
        lon, lat = transformer.transform(easting, northing)

        # Format as "lat;lon"
        position_str = f"{lat};{lon}"
        print(f"[bridge → cNODE TCP] {position_str}")

        # Send over TCP
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tsock:
                tsock.settimeout(2)
                tsock.connect((position_host, position_port))
                tsock.sendall(position_str.encode('ascii'))
        except Exception as e:
            logging.error(f"Failed to send position to {position_host}:{position_port}: {e}")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s: %(message)s')
    threading.Thread(target=mqtt_loop, daemon=True).start()
    threading.Thread(target=udp_loop,   daemon=True).start()
    logging.info("HiPOS↔cNODE-TCP bridge running")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down…")
        client.disconnect()
