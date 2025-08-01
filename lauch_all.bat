@echo off
REM ─────────────────────────────────────────────────────────────────────────────
REM launch_all.bat — All conda-activated tasks for inspection AROV
REM Includes ROV Logger and GStreamer Capture with startup delays to avoid conda contention
REM ─────────────────────────────────────────────────────────────────────────────

REM ─── CONFIGURE ────────────────────────────────────────────────────────────────
set REMOTE_IP=100.127.125.86
REM Edit the line above to change the target ROV IP address.
REM ─────────────────────────────────────────────────────────────────────────────

echo.
REM ==== 0) ROV Position Logger (system Python) ====
START "ROV Logger" cmd /k "cd /d %~dp0dockinganddataforwarding && python rov_position_csv.py"

TIMEOUT /T 3 >nul

echo.
REM ==== 0b) GStreamer Capture (in rov2 env) ====
START "GStreamer Capture" cmd /k "call conda activate rov2 && gst-launch-1.0 -e -v udpsrc port=5602 caps=\"application/x-rtp,encoding-name=H264,payload=96\" ! rtph264depay ! h264parse config-interval=1 ! mp4mux faststart=true name=mux ! filesink location=\"C:\path\to\output.mp4\" sync=false"

TIMEOUT /T 3 >nul

echo.
REM ==== 1) Docking Video Processor (in conda) ====
START "Dock Video Proc" cmd /k "call conda activate rov2 && cd /d %~dp0dockinganddataforwarding\rov_docking && C:\Users\larsmo\AppData\Local\miniconda3\envs\rov2\python.exe docking_videprocessor2.py --stream-port 5606 --height 600 --width 800 --pause 20 --win-size 0.8"

TIMEOUT /T 2 >nul

echo.
REM ==== 2) GStreamer UDP→UDP H.264 Tunnel (in conda) ====
START "GStreamer" cmd /k "call conda activate rov2 && gst-launch-1.0 -v udpsrc port=5602 caps=\"application/x-rtp,encoding-name=H264,payload=96\" ! rtph264depay ! h264parse config-interval=1 ! video/x-h264,stream-format=byte-stream,alignment=au ! udpsink host=127.0.0.1 port=5606 sync=false"

TIMEOUT /T 2 >nul

echo.
REM ==== 3) MAVProxy Ground Station ====
START "MAVProxy" cmd /k "cd /d %~dp0dockinganddataforwarding && mavproxy.exe --master=udp:192.168.2.1:14550 --master=udpin:127.0.0.1:14550 --out=udp:%REMOTE_IP%:14551 --out=udp:%REMOTE_IP%:14550 --console"

TIMEOUT /T 2 >nul

echo.
REM ==== 4) Python Stream Video Script ====
START "Stream Video" cmd /k "cd /d %~dp0dockinganddataforwarding && python stream_video.py --remote-ip %REMOTE_IP%"

TIMEOUT /T 2 >nul

echo.
REM ==== 5) ROV-side microPAP_positioning over SSH ====
set ROV_USER=pi
set ROV_HOST=192.168.2.2
set ROV_SCRIPT_DIR=python
set SCRIPT2=microPAP_positioning.py
START "ROV: TCP" cmd /k "ssh %ROV_USER%@%ROV_HOST% \"cd %ROV_SCRIPT_DIR% && python %SCRIPT2%\""

TIMEOUT /T 2 >nul

echo.
REM ==== 6) On-shore HiPOS Interface ====
set ONSHORE_SCRIPT=hipos_interface.py
START "HiPOS" cmd /k "cd /d %~dp0microPAP && py -3.13 %ONSHORE_SCRIPT%"

echo.
echo All windows launched. Press any key to close this launcher.
pause >nul
