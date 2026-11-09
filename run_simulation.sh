#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "======================================================================"
echo "   ArcDrone Live Simulation Showcase - One-Click Launcher (macOS)    "
echo "   (Virtual Radios + Drone Node + Ground Station + Auto Dashboard)   "
echo "======================================================================"
echo ""

if [ ! -f ".venv/bin/python" ]; then
    echo "[.venv not detected] Running automatic environment setup first..."
    bash ./setup.sh
fi

echo "[*] Starting ArcDrone End-to-End Simulation..."
exec .venv/bin/python scripts/run_live_showcase.py "$@"
