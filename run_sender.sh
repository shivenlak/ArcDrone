#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "======================================================================"
echo "   ArcDrone Airborne Drone Node (Sender) - One-Click Launcher (macOS) "
echo "======================================================================"
echo ""

if [ ! -f ".venv/bin/python" ]; then
    echo "[.venv not detected] Running automatic environment setup first..."
    bash ./setup.sh
fi

echo "[*] Starting Airborne Drone Node (Sender)..."
exec .venv/bin/python scripts/run_sender.py "$@"
