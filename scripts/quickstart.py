#!/usr/bin/env python3
"""ArcDrone Unified Quickstart & Environment Validator.

Provides an interactive CLI menu to easily run ArcDrone across:
  Approach 1: Virtual Trials (Zero hardware, 100% simulated)
  Approach 2: Real Hardware (ESP32 ESP-NOW or Heltec LoRa 915 MHz)
  Approach 3: Test Cases, Benchmarks & Analytics
"""

import os
import platform
import subprocess
import sys
import time

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def check_environment():
    print("=" * 70)
    print(" ARCDRONE SYSTEM & ENVIRONMENT DIAGNOSTIC")
    print("=" * 70)
    print(f" OS Platform     : {platform.system()} {platform.release()} ({platform.machine()})")
    print(f" Python Version  : {platform.python_version()} ({sys.executable})")

    # Dependency check
    required_pkgs = ["serial", "cv2", "ultralytics", "fastapi", "uvicorn", "cryptography"]
    missing = []
    for pkg in required_pkgs:
        try:
            __import__(pkg)
            print(f"   [+] Dependency: {pkg:<14s} OK")
        except ImportError:
            missing.append(pkg)
            print(f"   [-] Dependency: {pkg:<14s} MISSING")

    # Asset checks
    weights_path = os.path.join(_REPO_ROOT, "yolov8n-visdrone.pt")
    if os.path.isfile(weights_path):
        sz_mb = os.path.getsize(weights_path) / (1024 * 1024)
        print(f"   [+] Model Weights: yolov8n-visdrone.pt OK ({sz_mb:.1f} MB)")
    else:
        print(f"   [-] Model Weights: yolov8n-visdrone.pt MISSING in repo root")

    data_dir = os.path.join(_REPO_ROOT, "data")
    if os.path.isdir(data_dir):
        print(f"   [+] Data Folder  : data/ OK")
    else:
        print(f"   [-] Data Folder  : data/ MISSING")

    # Serial port discovery
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            print(f"   [+] Detected USB Serial Ports ({len(ports)}):")
            for p in ports:
                print(f"       - {p.device}: {p.description}")
        else:
            print("   [*] Serial Ports : None detected (Virtual trials ready)")
    except Exception:
        pass

    print("=" * 70)
    if missing:
        print(f"\n[!] WARNING: Missing packages: {', '.join(missing)}")
        print("    Install them with: pip install -r requirements.txt\n")


def run_command(cmd_args):
    print(f"\n>>> Running: {' '.join(cmd_args)}\n")
    try:
        subprocess.run(cmd_args, cwd=_REPO_ROOT)
    except KeyboardInterrupt:
        print("\n[Stopped by user]")


def main_menu():
    while True:
        print("\n" + "=" * 70)
        print(" ARCDRONE MASTER LAUNCHER — CHOOSE AN APPROACH")
        print("=" * 70)
        print(" APPROACH 1: VIRTUAL TRIALS (Zero Hardware, 100% Simulated)")
        print("   1. One-Click Interactive Mission Dashboard & Radio Sim (localhost:8000)")
        print("   2. Live VisDrone UDP Radar Reconstructor UI (localhost:8080)")
        print("   3. Render 1:1 Side-by-Side Video & Radar Canvas (MP4/PNG)")
        print("   4. Stream VisDrone Scenes to UDP Relay (:9999)")
        print("   5. Launch One-Command Footage->Radio->Dashboard Demo (drone_node.run_demo)")
        print("\n APPROACH 2: REAL HARDWARE TRIALS (ESP32 or LoRa)")
        print("   6. Launch Drone Node (Edge YOLO Inference at imgsz 1280)")
        print("   7. Launch Ground Station Server (http://localhost:8001)")
        print("   8. Generate & Display AES-128-CCM Key (python -m protocol.crypto)")
        print("   9. Run Range Walk Logger (Carry drone board & log RSSI/SNR/RTT to CSV)")
        print("\n APPROACH 3: TEST CASES, BENCHMARKS & ANALYTICS")
        print("  10. Run Full Pytest Suite (pytest -q)")
        print("  11. Run Automated Headless Verification Suite (9 Integration Tests, 49 Repo Total)")
        print("  12. Run Authenticated AES-128-CCM & Tactical Encryption Suite (9 tests)")
        print("  13. Run Real Edge YOLO Inference Benchmark (scripts/bench_pi.py)")
        print("  14. Run 6 Multi-Scenario Aerial Benchmarks (Batch MP4/PNG/JSON)")
        print("  15. Run Primary Link Bandwidth Compliance Auditor (<= 3 kB/s)")
        print("  16. Run Industry Expert Defense & Architecture Audit")
        print("  17. Run On-Demand LoRa Target Crop Transfer Demo (Scavenger Protocol)")
        print("\n  18. Re-run Environment & Hardware Diagnostics")
        print("   0. Exit")
        print("=" * 70)

        choice = input("Enter choice [0-18]: ").strip()
        if choice == "0":
            print("\nExiting. Good luck with natHacks!")
            sys.exit(0)
        elif choice == "1":
            run_command([sys.executable, "scripts/run_live_showcase.py"])
        elif choice == "2":
            print("\nStarting Relay. In a 2nd terminal run: python scripts/stream_visdrone_to_udp.py")
            print("Then open in browser: http://127.0.0.1:8080/reconstructor.html\n")
            run_command([sys.executable, "-m", "drone_node.scene_ws_relay"])
        elif choice == "3":
            run_command([sys.executable, "scripts/ishan_side_by_side.py"])
        elif choice == "4":
            run_command([sys.executable, "scripts/stream_visdrone_to_udp.py"])
        elif choice == "5":
            run_command([sys.executable, "-m", "drone_node.run_demo"])
        elif choice == "6":
            port = input("Enter drone ESP32 serial port (e.g. COM3 or /dev/cu.usbserial-1): ").strip()
            video = input("Enter video path (default: data/visdrone_aerial_sample.mp4): ").strip()
            if not video:
                video = "data/visdrone_aerial_sample.mp4"
            run_command([sys.executable, "-m", "drone_node.main", "--port", port, "--video", video, "--imgsz", "1280", "--fps", "10"])
        elif choice == "7":
            port = input("Enter ground ESP32 serial port (e.g. COM4 or /dev/cu.usbserial-2): ").strip()
            run_command([sys.executable, "-m", "ground_station.server", "--port", port, "--http-port", "8001"])
        elif choice == "8":
            from protocol import crypto
            key = crypto.generate_key() if hasattr(crypto, "generate_key") else os.urandom(16).hex().upper()
            os.environ["ARCDRONE_KEY"] = key
            print("\n" + "=" * 70)
            print(" GENERATED AES-128-CCM PRE-SHARED KEY:")
            print(f" {key}")
            print("=" * 70)
            print(f"To use on both machines, run in your terminal before launching:")
            print(f"export ARCDRONE_KEY={key}    # On macOS/Linux")
            print(f"$env:ARCDRONE_KEY=\"{key}\"   # On Windows PowerShell\n")
        elif choice == "9":
            port = input("Enter drone ESP32 serial port (e.g. COM3 or /dev/cu.usbserial-1): ").strip()
            run_command([sys.executable, "-m", "protocol.range_walk", "--port", port])
        elif choice == "10":
            run_command([sys.executable, "-m", "pytest", "protocol/test_packets.py", "scripts/test_integration_suite.py", "scripts/test_encryption.py", "scripts/test_enhanced_features.py", "scripts/test_lora_image_transfer.py", "-q"])
        elif choice == "11":
            run_command([sys.executable, "-m", "scripts.test_integration_suite"])
        elif choice == "12":
            run_command([sys.executable, "-m", "unittest", "scripts/test_encryption.py"])
        elif choice == "13":
            run_command([sys.executable, "scripts/bench_pi.py"])
        elif choice == "14":
            run_command([sys.executable, "scripts/run_test_cases.py"])
        elif choice == "15":
            run_command([sys.executable, "-m", "scripts.test_lora_bandwidth"])
        elif choice == "16":
            run_command([sys.executable, "scripts/industry_expert_audit.py"])
        elif choice == "17":
            run_command([sys.executable, "-m", "scripts.lora_image_transfer"])
        elif choice == "18":
            check_environment()
        else:
            print("\nInvalid selection. Please choose 0 to 18.")


if __name__ == "__main__":
    check_environment()
    main_menu()
