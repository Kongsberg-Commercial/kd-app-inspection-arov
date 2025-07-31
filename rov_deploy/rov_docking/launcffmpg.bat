@echo off
REM =============================================
REM start_bridge.bat
REM Activates your rov_opencv env and uses your
REM SDP file to bridge RTP→MPEG‑TS UDP for OpenCV
REM =============================================

:: 1) Activate your Conda environment
call conda activate rov_opencv


:: 3) Run FFmpeg with SDP to listen on 5600 and forward to localhost:5601
ffmpeg -protocol_whitelist "file,udp,rtp" -i stream.sdp ^
  -c copy -f mpegts "udp://127.0.0.1:5601"

pause
