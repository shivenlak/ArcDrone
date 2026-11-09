@echo off
setlocal
title ArcDrone - Airborne Drone Node (Sender)
cd /d "%~dp0"

echo ======================================================================
echo    ArcDrone Airborne Drone Node (Sender) - One-Click Launcher
echo ======================================================================
echo.

if not exist ".venv\Scripts\python.exe" (
    echo [.venv not detected] Running automatic environment setup first...
    call setup.bat
    if errorlevel 1 (
        echo [ERROR] Setup failed.
        pause
        exit /b 1
    )
)

echo [*] Starting Airborne Drone Node (Sender)...
.venv\Scripts\python.exe scripts\run_sender.py %*

if errorlevel 1 (
    echo.
    echo [!] Drone Node exited with code %errorlevel%.
    pause
)
