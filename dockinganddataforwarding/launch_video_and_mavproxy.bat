@echo off
REM Start MAVProxy in a new window
start cmd /k "mavproxy.exe --master=udp:192.168.2.1:14550 --master=udpin:127.0.0.1:14550 --out=udp:100.127.125.86:14551 --console"

REM Start stream_video.py in a new window
start cmd /k "python .\stream_video.py --remote-ip 100.127.125.86"
