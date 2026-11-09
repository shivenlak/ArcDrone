"""One-click interactive launcher for the ArcDrone Airborne Node (Sender).

Handles:
1. Auto-detection of USB serial / COM ports (Heltec V3, ESP32, CP210x, CH340).
2. Auto-discovery of video files (data/*.mp4, test_cases, and live webcam 0).
3. Auto-detection of YOLO weights (yolov8n-visdrone.pt or yolov8n.pt).
4. Launch of drone_node.main with live OpenCV HUD and radio transmission.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import List, Optional, Tuple

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import serial.tools.list_ports


def is_likely_radio_device(desc: str, hwid: str) -> bool:
    keywords = ["cp210", "ch34", "ch910", "ftdi", "usb serial", "uart", "esp32", "heltec", "jtag"]
    combined = f"{desc} {hwid}".lower()
    return any(k in combined for k in keywords)


def detect_ports() -> List[Tuple[str, str, bool]]:
    ports = serial.tools.list_ports.comports()
    results = []
    for p in ports:
        recommended = is_likely_radio_device(p.description, p.hwid)
        results.append((p.device, p.description, recommended))
    return sorted(results, key=lambda x: (not x[2], x[0]))


def prompt_for_port(default_port: Optional[str] = None) -> Optional[str]:
    if default_port:
        return default_port

    env_port = os.environ.get("ARCDRONE_DRONE_PORT")
    if env_port:
        print(f"[ENV] Using Drone Port from ARCDRONE_DRONE_PORT: {env_port}")
        return env_port

    ports = detect_ports()

    print("\n" + "=" * 65)
    print("   ArcDrone Airborne Node (Sender) - Port Selection")
    print("=" * 65)

    if not ports:
        print("\n[!] No physical COM ports detected.")
        print("    Plug in your drone's Heltec/ESP32 radio bridge via USB.")
        print("\nOptions:")
        print("  [C] Enter custom port manually (e.g. COM3)")
        print("  [Q] Quit")
        choice = input("\nSelect option [default: C]: ").strip().upper()
        if choice in ("", "C"):
            custom = input("Enter port (e.g. COM3): ").strip()
            return custom if custom else None
        return None

    print("\nAvailable Serial Ports:")
    default_idx = 1
    for idx, (port_name, desc, recommended) in enumerate(ports, start=1):
        tag = " [RECOMMENDED RADIO]" if recommended else ""
        print(f"  [{idx}] {port_name} - {desc}{tag}")

    print(f"  [C] Custom port")
    print(f"  [Q] Quit")

    prompt = f"\nSelect drone radio port [1-{len(ports)}, default: {default_idx}]: "
    user_input = input(prompt).strip()

    if not user_input:
        return ports[default_idx - 1][0]
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

    return user_input


def find_available_videos() -> List[str]:
    """Find all mp4 videos in data/ and data/test_cases/."""
    candidates = []
    preferred = [
        os.path.join(_REPO_ROOT, "data", "visdrone_aerial_sample.mp4"),
        os.path.join(_REPO_ROOT, "data", "sample_traffic.mp4"),
        os.path.join(_REPO_ROOT, "data", "ishan_visdrone_side_by_side.mp4"),
    ]
    for p in preferred:
        if os.path.isfile(p):
            candidates.append(p)

    all_mp4 = glob.glob(os.path.join(_REPO_ROOT, "data", "**", "*.mp4"), recursive=True)
    for p in all_mp4:
        if p not in candidates:
            candidates.append(p)

    return candidates


def prompt_for_video(default_video: Optional[str] = None) -> Optional[str]:
    if default_video:
        return default_video

    videos = find_available_videos()

    print("\n" + "=" * 65)
    print("   ArcDrone Video Source Selection")
    print("=" * 65)

    options: List[Tuple[str, str]] = []
    for v in videos:
        rel = os.path.relpath(v, _REPO_ROOT)
        options.append((v, rel))

    options.append(("0", "Live Webcam (Camera 0)"))

    print("\nAvailable Video Sources:")
    for idx, (_, label) in enumerate(options, start=1):
        def_tag = " (Recommended Default)" if idx == 1 else ""
        print(f"  [{idx}] {label}{def_tag}")
    print(f"  [C] Custom file path or camera index")
    print(f"  [Q] Quit")

    prompt = f"\nSelect video source [1-{len(options)}, default: 1]: "
    user_input = input(prompt).strip()

    if not user_input:
        return options[0][0]
    if user_input.upper() == "Q":
        return None
    if user_input.upper() == "C":
        custom = input("Enter path to video or camera index: ").strip()
        return custom if custom else None

    try:
        idx = int(user_input)
        if 1 <= idx <= len(options):
            return options[idx - 1][0]
    except ValueError:
        pass

    return user_input


def find_best_model() -> str:
    """Return path to yolov8n-visdrone.pt if present, else yolov8n.pt."""
    visdrone_pt = os.path.join(_REPO_ROOT, "yolov8n-visdrone.pt")
    if os.path.isfile(visdrone_pt):
        return visdrone_pt
    yolo_pt = os.path.join(_REPO_ROOT, "yolov8n.pt")
    if os.path.isfile(yolo_pt):
        return yolo_pt
    return "yolov8n.pt"


def main() -> None:
    parser = argparse.ArgumentParser(description="ArcDrone Drone Node Sender Launcher")
    parser.add_argument("--port", default=None, help="Serial port of drone ESP32 (e.g. COM3)")
    parser.add_argument("--video", default=None, help="Video path or camera index")
    parser.add_argument("--model", default=None, help="YOLO model path")
    parser.add_argument("--imgsz", type=int, default=None, help="Inference resolution")
    parser.add_argument("--fps", type=float, default=2.0, help="Camera / YOLO inference rate")
    parser.add_argument("--preview-port", type=int, default=8081, help="Wi-Fi MJPEG preview port (default: 8081)")
    parser.add_argument("--headless", action="store_true", help="Run without OpenCV GUI window")
    args = parser.parse_args()

    port = prompt_for_port(args.port)
    if not port:
        print("[*] Exiting.")
        return

    video = prompt_for_video(args.video)
    if not video:
        print("[*] Exiting.")
        return

    model = args.model if args.model else find_best_model()
    imgsz = args.imgsz if args.imgsz else (640 if video == "0" else 1280)

    print("\n" + "=" * 65)
    print("  Starting ArcDrone Airborne Edge Node (Sender)")
    print(f"  Port:         {port}")
    print(f"  Video:        {video}")
    print(f"  Model:        {os.path.basename(model)}")
    print(f"  Resolution:   {imgsz}")
    print(f"  Target FPS:   {args.fps}")
    print(f"  Preview Port: {args.preview_port}")
    print("=" * 65 + "\n")

    # Construct sys.argv to invoke drone_node.main
    sys_argv = [
        "drone_node.main",
        "--port", port,
        "--video", video,
        "--model", model,
        "--imgsz", str(imgsz),
        "--fps", str(args.fps),
        "--preview-port", str(args.preview_port),
    ]
    if args.headless:
        sys_argv.append("--headless")

    sys.argv = sys_argv

    from drone_node.main import main as run_drone_main
    run_drone_main()


if __name__ == "__main__":
    main()
