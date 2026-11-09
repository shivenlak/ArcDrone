@echo off
setlocal
cd /d "%~dp0"

echo ======================================================================
echo    ArcDrone Environment Setup - Automated One-Click Installer
echo    natHacks 2026 - Drone Zone
echo ======================================================================
echo.

:: Check for uv package manager
where uv >nul 2>nul
if %errorlevel% equ 0 goto USE_UV

:: Fallback: standard Python
where python >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Python was not found in your PATH!
    echo Please install Python 3.9+ from https://www.python.org/
    pause
    exit /b 1
)

echo [INFO] Using standard Python...
if not exist ".venv\Scripts\python.exe" (
    echo [*] Creating virtual environment in .venv...
    python -m venv .venv
    if errorlevel 1 (
        echo [ERROR] Failed to create virtual environment.
        pause
        exit /b 1
    )
) else (
    echo [OK] Virtual environment .venv already exists.
)

echo [*] Installing requirements via pip...
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install -r requirements.txt
if errorlevel 1 (
    echo [ERROR] pip failed to install requirements.
    pause
    exit /b 1
)
goto DONE

:USE_UV
echo [FOUND] Ultra-fast 'uv' package manager detected.
if not exist ".venv\Scripts\python.exe" (
    echo [*] Creating virtual environment with Python 3.11...
    uv venv --python 3.11 .venv
    if errorlevel 1 (
        echo [*] Python 3.11 not cached; creating with system default...
        uv venv .venv
    )
) else (
    echo [OK] Virtual environment .venv already exists.
)

echo [*] Installing / syncing requirements with uv...
uv pip install -r requirements.txt
if errorlevel 1 (
    echo [ERROR] uv failed to install requirements.
    pause
    exit /b 1
)

:DONE
echo.
echo ======================================================================
echo  [SUCCESS] ArcDrone environment is fully set up and ready!
echo ======================================================================
echo.
