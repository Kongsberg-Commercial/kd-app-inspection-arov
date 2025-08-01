@echo off
REM ─────────────────────────────────────────────────────────────────────────────
REM launch_all.bat — SSH into the ROV and run its two real scripts; then start
REM                    the on‑shore HiPOS interface. Dummy scripts are omitted.
REM ─────────────────────────────────────────────────────────────────────────────

REM ==== Configuration ====
set ROV_USER=pi
set ROV_HOST=192.168.2.2
set ROV_SCRIPT_DIR=python

set SCRIPT2=microPAP_positioning.py
set ONSHORE_SCRIPT=hipos_interface.py

REM ==== Launch ROV scripts (each in its own window) ====
echo.

echo [ROV] Starting %SCRIPT2% on %ROV_HOST%...
start "ROV: TCP" cmd /k ssh %ROV_USER%@%ROV_HOST% "cd %ROV_SCRIPT_DIR% && python %SCRIPT2%"

REM ==== Launch on‑shore HiPOS Interface ====
echo.
echo [On‑shore] Starting %ONSHORE_SCRIPT%...
start "HiPOS Interface" cmd /k py -3.13 "%~dp0%ONSHORE_SCRIPT%"

echo.
echo All real scripts launched.
pause
