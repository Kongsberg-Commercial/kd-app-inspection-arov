@echo off
REM ─────────────────────────────────────────────────────────────────────────────
REM launch_rov.bat — SSH into the ROV, kill old instance & run microPAP_positioning.py
REM ─────────────────────────────────────────────────────────────────────────────

REM ROV login info
set ROV_USER=pi
set ROV_HOST=192.168.2.2
set ROV_SCRIPT_DIR=python
set ROV_SCRIPT=microPAP_positioning.py

echo.
echo Connecting to %ROV_USER%@%ROV_HOST%...
echo.

REM In one SSH session: kill old, cd into folder, then run script interactively
start "ROV-microPAP" cmd /k ssh %ROV_USER%@%ROV_HOST% ^
  "pkill -f %ROV_SCRIPT% 2>/dev/null || true; cd %ROV_SCRIPT_DIR% && python %ROV_SCRIPT%"

REM You’ll be prompted once for your password; after that you’ll see all the script’s output live.
