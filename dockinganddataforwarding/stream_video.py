#!/usr/bin/env python3
import socket
import argparse
import sys

def forwarder(local_port: int, remote_ip: str, remote_port: int):
    """Listen on 0.0.0.0:local_port and forward all UDP packets to remote_ip:remote_port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', local_port))
    print(f"Listening on UDP 0.0.0.0:{local_port} → forwarding to {remote_ip}:{remote_port}")
    try:
        while True:
            data, addr = sock.recvfrom(65536)
            sock.sendto(data, (remote_ip, remote_port))
    except KeyboardInterrupt:
        print("\nStopping.")
        sock.close()
        sys.exit(0)

def main():
    p = argparse.ArgumentParser(
        description="Simple UDP forwarder for ROV video over Tailscale."
    )
    p.add_argument('--local-port', type=int, default=5600,
                   help="Port to listen on locally (default 5600)")
    p.add_argument('--remote-ip', required=True,
                   help="Tailscale IP of the on-shore machine")
    p.add_argument('--remote-port', type=int, default=5600,
                   help="Port to forward to on the remote (default 5600)")
    args = p.parse_args()
    forwarder(args.local_port, args.remote_ip, args.remote_port)

if __name__ == '__main__':
    main()
