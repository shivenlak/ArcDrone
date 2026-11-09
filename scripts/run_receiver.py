"""One-click interactive launcher for the ArcDrone Ground Station (Receiver).

Handles:
1. Auto-detection of USB serial / COM ports (Heltec V3, ESP32, CP210x, CH340).
2. Selection menu with intelligent defaults and timeout fallback.
3. Auto-launch of the browser dashboard (http://localhost:8000) when the server starts.
4. Clean launch of the ground station server.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
import urllib.request
import webbrowser
from typing import List, Optional, Tuple

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import serial.tools.list_ports
import uvicorn
from ground_station.server import make_app


def is_likely_radio_device(device: str, desc: str, hwid: str) -> bool:
    """Check if device name, description or HWID looks like an ESP32 / Heltec / USB-UART bridge."""
    keywords = [
        "cp210", "ch34", "ch910", "ftdi", "usb serial", "uart",
        "esp32", "heltec", "jtag", "usbserial", "usbmodem", "wchusb", "slab"
    ]
    combined = f"{device} {desc} {hwid}".lower()
    return any(k in combined for k in keywords)


def detect_ports() -> List[Tuple[str, str, bool]]:
    """Return list of (port_name, description, is_recommended)."""
    ports = serial.tools.list_ports.comports()
    results = []
    for p in ports:
        # On macOS, prefer /dev/cu.* over /dev/tty.* to avoid DCD assertion hang
        if sys.platform == "darwin" and p.device.startswith("/dev/tty."):
            continue
        recommended = is_likely_radio_device(p.device, p.description, p.hwid)
        results.append((p.device, p.description, recommended))
    return sorted(results, key=lambda x: (not x[2], x[0]))


def prompt_for_port(default_port: Optional[str] = None) -> Optional[str]:
    """Interactively select a serial port or simulated mode."""
    if default_port:
        return default_port

    env_port = os.environ.get("ARCDRONE_GROUND_PORT")
    if env_port:
        print(f"[ENV] Using Ground Port from ARCDRONE_GROUND_PORT: {env_port}")
        return env_port

    ports = detect_ports()

    print("\n" + "=" * 65)
    print("   ArcDrone Ground Station (Receiver) - Port Selection")
    print("=" * 65)

    if not ports:
        print("\n[!] No physical COM ports detected.")
        print("    If your Heltec/ESP32 board is plugged in, check USB drivers.")
        print("\nOptions:")
        print("  [S] Run Full Simulation Showcase (no hardware required)")
        print("  [C] Enter custom port manually (e.g. COM3)")
        print("  [Q] Quit")
        choice = input("\nSelect option [default: S]: ").strip().upper()
        if choice in ("", "S"):
            return "SIM"
        elif choice == "Q":
            return None
        elif choice == "C":
            custom = input("Enter port (e.g. COM3): ").strip()
            return custom if custom else None
        else:
            return choice

    print("\nAvailable Serial Ports:")
    default_idx = 1
    for idx, (port_name, desc, recommended) in enumerate(ports, start=1):
        tag = " [RECOMMENDED RADIO]" if recommended else ""
        print(f"  [{idx}] {port_name} - {desc}{tag}")

    print(f"  [S] Run Full Simulation Showcase (in-memory virtual radio)")
    print(f"  [C] Custom port")
    print(f"  [Q] Quit")

    prompt = f"\nSelect port [1-{len(ports)}, default: {default_idx}]: "
    user_input = input(prompt).strip()

    if not user_input:
        return ports[default_idx - 1][0]
    if user_input.upper() == "S":
        return "SIM"
    if user_input.upper() == "Q":
        return None
    if user_input.upper() == "C":
        custom = input("Enter port (e.g. COM3): ").strip()
        return custom if custom else None

    try:
        idx = int(user_input)
        if 1 <= idx <= len(ports):
            return ports[idx - 1][0]
    except ValueError:
        pass

    # If user typed the port name directly (e.g., COM3)
    return user_input


def start_scene_relay():
    """Serve reconstructor.html on :8080 and UDP :9999 -> ws :8765. Skip if already up."""
    try:
        urllib.request.urlopen("http://127.0.0.1:8080/reconstructor.html", timeout=0.4)
        print("[*] Scene reconstructor already running on :8080")
        return None
    except Exception:
        pass
    print("[*] Starting scene reconstructor relay (http://127.0.0.1:8080/reconstructor.html)...")
    return subprocess.Popen(
        [sys.executable, "-u", "-m", "drone_node.scene_ws_relay"],
        cwd=_REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def open_browser_when_ready(url: str, timeout: float = 12.0) -> None:
    """Poll URL until server responds, then open default web browser."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            req = urllib.request.Request(url, method="HEAD")
            with urllib.request.urlopen(req, timeout=0.8):
                pass
            time.sleep(0.3)
            webbrowser.open(url)
            return
        except Exception:
            time.sleep(0.4)
    # Open anyway as fallback
    webbrowser.open(url)


def main() -> None:
    parser = argparse.ArgumentParser(description="ArcDrone Ground Station Receiver Launcher")
    parser.add_argument("--port", default=None, help="Serial port of ground ESP32 (e.g. COM4)")
    parser.add_argument("--http-port", type=int, default=8000, help="Web dashboard port (default: 8000)")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP server host (default: 0.0.0.0)")
    parser.add_argument("--no-browser", action="store_true", help="Do not auto-open browser")
    args = parser.parse_args()

    selected_port = prompt_for_port(args.port)
    if not selected_port:
        print("[*] Exiting.")
        return

    dashboard_url = f"http://localhost:{args.http_port}"
    relay = start_scene_relay()

    if selected_port == "SIM":
        print("\n[*] Starting Live Simulation Showcase with virtual radio bridge...")
        from scripts.run_live_showcase import main as run_showcase
        if not args.no_browser:
            threading.Thread(target=open_browser_when_ready, args=(dashboard_url,), daemon=True).start()
        try:
            run_showcase()
        finally:
            if relay is not None and relay.poll() is None:
                relay.terminate()
        return

    print("\n" + "=" * 65)
    print(f"  ArcDrone Ground Station starting on {selected_port}")
    print(f"  Dashboard URL: {dashboard_url}")
    print("  Reconstructor: http://127.0.0.1:8080/reconstructor.html")
    print("=" * 65 + "\n")

    if not args.no_browser:
        threading.Thread(target=open_browser_when_ready, args=(dashboard_url,), daemon=True).start()

    try:
        app = make_app(selected_port)
        uvicorn.run(app, host=args.host, port=args.http_port, log_level="info")
    finally:
        if relay is not None and relay.poll() is None:
            relay.terminate()


if __name__ == "__main__":
    main()
