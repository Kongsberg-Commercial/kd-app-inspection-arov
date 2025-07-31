import cv2
import subprocess
import numpy as np

ffmpeg_cmd = [
    'ffmpeg',
    '-i', 'udp://127.0.0.1:5606?fifo_size=5000000&overrun_nonfatal=1',
    '-f', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-'
]

proc = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, bufsize=10**8)

width = 800   # set your stream's width
height = 600   # set your stream's height

while True:
    raw_frame = proc.stdout.read(width * height * 3)
    if not raw_frame:
        break
    frame = np.frombuffer(raw_frame, np.uint8).reshape((height, width, 3))
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

proc.terminate()
cv2.destroyAllWindows()
