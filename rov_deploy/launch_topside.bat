REM launch.bat - start all ROV deployment components

REM Change directory to script location
dir /b >nul 2>&1 && cd /d "%~dp0"

REM 1) Gamepad trigger → MQTT
start "Gamepad Trigger" cmd /k python axis_to_mqtt.py

REM 3) Toggle-cycle controller
start "Toggle Cycle" cmd /k python toggle_handler.py

echo.
REM ==== 4b) FFmpeg RTP to Local UDP MPEG-TS (in conda env) ====
START "FFmpeg RTP→UDP" cmd /k "call conda activate rov_opencv && cd /d C:\Users\larsmo\Documents\ROV\kd-app-inspection-arov\rov_deploy && ffmpeg -fflags nobuffer -flags low_delay -protocol_whitelist file,udp,rtp -i rtp_in.sdp -c copy -f mpegts ""udp://127.0.0.1:5600?pkt_size=1316"""
