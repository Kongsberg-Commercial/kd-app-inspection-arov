
@echo off
REM Activate the conda environment
CALL conda activate rov2

REM Start GStreamer pipeline in a new command window
START "" cmd /k gst-launch-1.0 -v udpsrc port=5602 caps="application/x-rtp,encoding-name=H264,payload=96" ^
! rtph264depay ! h264parse config-interval=1 ! video/x-h264,stream-format=byte-stream,alignment=au ^
! udpsink host=127.0.0.1 port=5606 sync=false

REM Optional: Wait a couple seconds for pipeline to start
TIMEOUT /T 2

REM Run your Python script (replace with your actual filename if different)
C:\Users\larsmo\AppData\Local\miniconda3\python.exe rtovideo.py

REM Optional: Pause so the window stays open
pause
