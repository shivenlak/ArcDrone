@echo off
setlocal
title ArcDrone - Complete Live Simulation Showcase
cd /d "%~dp0"

echo ======================================================================
echo    ArcDrone Live Simulation Showcase - One-Click Launcher
echo    (Virtual Radios + Drone Node + Ground Station + Auto Dashboard)
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

echo [*] Starting ArcDrone End-to-End Simulation...
.venv\Scripts\python.exe scripts\run_live_showcase.py %*

if errorlevel 1 (
    echo.
    echo [!] Simulation exited with code %errorlevel%.
    pause
)
