@echo off
setlocal
title ArcDrone - Ground Station (Receiver)
cd /d "%~dp0"

echo ======================================================================
echo    ArcDrone Ground Station Receiver - One-Click Launcher
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

echo [*] Starting Ground Station Receiver...
.venv\Scripts\python.exe scripts\run_receiver.py %*

if errorlevel 1 (
    echo.
    echo [!] Ground Station exited with code %errorlevel%.
    pause
)
