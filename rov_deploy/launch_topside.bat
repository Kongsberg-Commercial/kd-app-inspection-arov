REM launch.bat - start all ROV deployment components

REM Change directory to script location
dir /b >nul 2>&1 && cd /d "%~dp0"

REM 1) Gamepad trigger → MQTT
start "Gamepad Trigger" cmd /k python axis_to_mqtt.py

REM 3) Toggle-cycle controller
start "Toggle Cycle" cmd /k python toggle_handler.py

