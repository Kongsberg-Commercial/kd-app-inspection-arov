# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import socket

HOST = '0.0.0.0' 
PORT = 6005  

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"Listening for cNODE position on {HOST}:{PORT}")
        while True:
            conn, addr = s.accept()
            with conn:
                data = conn.recv(1024).decode('ascii', errors='ignore').strip()
                if not data:
                    continue
                
                try:
                    lat_str, lon_str = data.split(';', 1)
                    lat = float(lat_str)
                    lon = float(lon_str)
                    print(f"Received position ? lat: {lat}, lon: {lon}")
                    print(data)
                except ValueError:
                    print(f"Malformed data from {addr}: '{data}'")

if __name__ == '__main__':
    main()
