# This script will run on the topside computer and receive the video stream from the image processor on the rov.
import socket, struct, cv2, numpy as np

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

# Set up TCP server
print(f"Listening...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('0.0.0.0', 8889))
s.listen(1)
conn, addr = s.accept()
print("Connected by", addr)

while True:
    header = recvall(conn, 4)
    if not header: break
    length = struct.unpack('>I', header)[0]
    data = recvall(conn, length)
    if not data: break
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    cv2.imshow('Video Feed', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

conn.close()
s.close()
cv2.destroyAllWindows()
print(f"Closed connection to {addr}")
