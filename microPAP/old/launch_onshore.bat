@echo off
REM launch_onshore.bat — start all on-shore Python scripts

REM If Python is on your PATH, this will spawn each one in its own window
echo Starting on-shore services...

start "Dummy GPS"       python "%~dp0mqtt_dummy_gps.py"
start "HiPOS Interface" py -3.13 "%~dp0hipos_interface.py"
start "Dummy ROV Sub"   python "%~dp0mqtt_dummy_sub_rov.py"

echo All scripts launched.
pause
