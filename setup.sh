#!/usr/bin/env bash
set -e

# Resolve repository root
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "======================================================================"
echo "   ArcDrone Environment Setup - Automated One-Click Installer (macOS)"
echo "   natHacks 2026 - Drone Zone"
echo "======================================================================"
echo ""

if command -v uv >/dev/null 2>&1; then
    echo "[FOUND] Ultra-fast 'uv' package manager detected."
    if [ ! -f ".venv/bin/python" ]; then
        echo "[*] Creating virtual environment with Python 3.11 via uv..."
        uv venv --python 3.11 .venv 2>/dev/null || {
            echo "[*] Creating with system default Python via uv..."
            uv venv .venv
        }
    else
        echo "[OK] Virtual environment .venv already exists."
    fi

    echo "[*] Installing / syncing requirements with uv..."
    uv pip install -r requirements.txt
else
    echo "[INFO] 'uv' not found; using standard python3..."
    if ! command -v python3 >/dev/null 2>&1; then
        echo "[ERROR] python3 was not found in your PATH!"
        echo "Please install Python 3.9+ (e.g. brew install python@3.11 or from python.org)"
        exit 1
    fi

    if [ ! -f ".venv/bin/python" ]; then
        echo "[*] Creating virtual environment (.venv)..."
        python3 -m venv .venv
    else
        echo "[OK] Virtual environment .venv already exists."
    fi

    echo "[*] Installing requirements via pip..."
    .venv/bin/python -m pip install --upgrade pip
    .venv/bin/python -m pip install -r requirements.txt
fi

echo ""
echo "======================================================================"
echo "  [SUCCESS] ArcDrone environment is fully set up and ready to fly!"
echo "======================================================================"
echo ""
