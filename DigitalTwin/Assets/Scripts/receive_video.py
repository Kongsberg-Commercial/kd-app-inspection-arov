import socket
import struct
import cv2
import numpy as np

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

def start_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('0.0.0.0', 8889))
    s.listen(1)
    print("Listening on port 8889...")
    return s

def handle_connection(conn, addr):
    print("Connected by", addr)
    try:
        while True:
            header = recvall(conn, 4)
            if not header:
                break
            length = struct.unpack('>I', header)[0]
            data = recvall(conn, length)
            if not data:
                break
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('Video Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Keyboard interrupt received during streaming. Closing connection.")
    finally:
        conn.close()
        cv2.destroyAllWindows()
        print(f"Closed connection to {addr}")

def main():
    server_socket = start_server()
    try:
        while True:
            print("Waiting for a new connection...")
            conn, addr = server_socket.accept()
            handle_connection(conn, addr)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down server.")
    finally:
        server_socket.close()

if __name__ == '__main__':
    main()