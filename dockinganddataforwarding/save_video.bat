@echo off

REM Start the ROV position logger (system Python)
START "ROV Logger" cmd /k python "%~dp0rov_position_csv.py"

REM Start the GStreamer pipeline inside the conda rov2 env
START "GStreamer Capture" cmd /k ^
    CALL conda activate rov2 ^&^& ^
    gst-launch-1.0 -e -v udpsrc port=5602 caps="application/x-rtp,encoding-name=H264,payload=96" ! ^
      rtph264depay ! ^
      h264parse config-interval=1 ! ^
      mp4mux faststart=true name=mux ! ^
      filesink location="C:\path\to\output.mp4" sync=false
