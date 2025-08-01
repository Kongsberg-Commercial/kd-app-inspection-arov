@echo off
REM ─────────────────────────────────────────────────────────────────────────────
REM launch_all.bat — All conda-activated tasks for inspection AROV
REM Now both docking_videprocessor2.py and gst-launch-1.0 run under 'rov2'.
REM ─────────────────────────────────────────────────────────────────────────────

echo.
REM ==== 1) Docking Video Processor (in conda) ====
START "Dock Video Proc" cmd /k "call conda activate rov2 && cd /d %~dp0dockinganddataforwarding\rov_docking && C:\Users\larsmo\AppData\Local\miniconda3\envs\rov2\python.exe docking_videprocessor2.py --stream-port 5606 --height 600 --width 800 --pause 20 --win-size 0.8"

echo.
REM ==== 2) GStreamer UDP→UDP H.264 Tunnel (in conda) ====
START "GStreamer" cmd /k "call conda activate rov2 && gst-launch-1.0 -v udpsrc port=5602 caps=\"application/x-rtp,encoding-name=H264,payload=96\" ! rtph264depay ! h264parse config-interval=1 ! video/x-h264,stream-format=byte-stream,alignment=au ! udpsink host=127.0.0.1 port=5606 sync=false"

TIMEOUT /T 2 >nul

echo.
REM ==== 3) MAVProxy Ground Station ====
START "MAVProxy" cmd /k "cd /d %~dp0dockinganddataforwarding && mavproxy.exe --master=udp:192.168.2.1:14550 --master=udpin:127.0.0.1:14550 --out=udp:100.127.125.86:14551 --out=udp:100.127.125.86:14550 --console"

echo.
REM ==== 4) Python Stream Video Script ====
START "Stream Video" cmd /k "cd /d %~dp0dockinganddataforwarding && python stream_video.py --remote-ip 100.127.125.86"

echo.
REM ==== 5) ROV-side microPAP_positioning over SSH ====
set ROV_USER=pi
set ROV_HOST=192.168.2.2
set ROV_SCRIPT_DIR=python
set SCRIPT2=microPAP_positioning.py
START "ROV: TCP" cmd /k "ssh %ROV_USER%@%ROV_HOST% \"cd %ROV_SCRIPT_DIR% && python %SCRIPT2%\""

echo.
REM ==== 6) On-shore HiPOS Interface ====
set ONSHORE_SCRIPT=hipos_interface.py
START "HiPOS" cmd /k "cd /d %~dp0microPAP && py -3.13 %ONSHORE_SCRIPT%"

echo.
echo All windows launched. Press any key to close this launcher.
pause >nul
