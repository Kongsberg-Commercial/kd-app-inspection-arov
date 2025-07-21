#!/usr/bin/env python3
import socket
import time
from pymavlink import mavutil

# --- Configuration ---------------------------------------------------------------
TCP_HOST        = '0.0.0.0'        # listen on all interfaces
TCP_PORT        = 6003
MAV_CONNECTION  = 'udpin:0.0.0.0:7777'  # adjust if your FC is on a different port
# --------------------------------------------------------------------------------

def main():
    # 1) Set up MAVLink connection and wait for heartbeat

    mav = mavutil.mavlink_connection('udpin:0.0.0.0:7777')
    mav.wait_heartbeat()
    print(f"Heartbeat from system {mav.target_system}, component {mav.target_component}")

    # 2) Build ignore-flags: we only send lat/lon (ignore velocity & speed-accuracy)
    ignore_flags = (
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT  |
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY
    )

    # 3) Listen for TCP connections
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((TCP_HOST, TCP_PORT))
        srv.listen()
        print("Listening for TCP on ",TCP_HOST)

        while True:
            conn, addr = srv.accept()
            with conn:
                raw = conn.recv(1024).decode('ascii', errors='ignore').strip()
                if not raw:
                    continue

                # Expect exactly "lat;lon"
                parts = raw.split(';', 1)
                if len(parts) != 2:
                    print(f"[!] Malformed packet from {addr}: {raw!r}")
                    continue

                try:
                    lat = float(parts[0])
                    lon = float(parts[1])
                except ValueError:
                    print(f"[!] Non-numeric data from {addr}: {raw!r}")
                    continue

                print(f"[+] Received position ? lat: {lat}, lon: {lon}")

                # 4) Send GPS_INPUT (see example in threedistances.py) :contentReference[oaicite:1]{index=1}
                timestamp = int(time.time() * 1e6)
                mav.mav.gps_input_send(
                    timestamp,
                    0,                # GPS ID
                    ignore_flags,     # flags: lat/lon only
                    0,                # time (ms, unknown)
                    0,                # week (unknown)
                    3,                # fix type = 3D
                    int(lat  * 1e7),  # lat [1E7]
                    int(lon  * 1e7),  # lon [1E7]
                    0.0,              # alt (ignored)
                    1.0,              # HDOP
                    1.0,              # VDOP
                    0.0, 0.0, 0.0,    # vn, ve, vd
                    100.0,            # speed accuracy
                    0.4,              # horiz accuracy
                    0.1,              # vert accuracy
                    18                # satellites visible (arbitrary)
                )
                print("[?] GPS_INPUT sent to FC")

if __name__ == '__main__':
    main()
