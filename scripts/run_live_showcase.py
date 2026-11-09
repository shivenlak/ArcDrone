"""Live Interactive natHacks Showcase Server.

Spins up:
1. SimLinkBridge connecting virtual drone and ground radios.
2. An airborne drone simulation emitting real-time detections and responding to commands.
3. Ground Station web server + live WebSocket dashboard on http://localhost:8000.
4. UAARG telemetry logging in tmp/log/.
"""

import argparse
import atexit
import collections
import glob
import json
import math
import os
import random
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
import webbrowser
import uvicorn

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from ground_station.server import make_app
from protocol import packets as p
from protocol.link import Link
from scripts.bandwidth_monitor import BandwidthMonitor
from scripts.sim_link_bridge import SimLinkBridge

CLASSES = ["pedestrian", "car", "van", "truck", "bus", "motor"]


def _load_visdrone_frames():
    frames = []
    clips_dir = os.path.join(_REPO_ROOT, "data", "clips")
    for jf in sorted(glob.glob(os.path.join(clips_dir, "uav*.json"))):
        try:
            with open(jf, "r") as f:
                data = json.load(f)
            seq_id = data.get("id", os.path.basename(jf))
            for fr in data.get("frames", [])[::10]:
                if fr:
                    frames.append((seq_id, fr))
        except Exception:
            pass
    return frames


def start_drone_node(drone_port: str, bridge: SimLinkBridge, stop_event: threading.Event):
    visdrone_samples = _load_visdrone_frames()

    drone_state = {"mode": p.MODE_PATROL, "lat": 53.5232, "lon": -113.5263, "heading": 45.0}

    def on_drone_packet(pkt):
        if pkt.get("type") == p.COMMAND:
            cmd = pkt.get("cmd")
            ref_seq = pkt.get("ref_seq")
            cmd_name = p.CMD_NAMES.get(cmd, str(cmd))
            print(f"\n>>> [DRONE RECEIVED COMMAND] {cmd_name} (ref_seq={ref_seq})")

            if cmd == p.CMD_HOLD:
                drone_link.mode = p.MODE_HOLD
                drone_state["mode"] = p.MODE_HOLD
                print(">>> [DRONE FLIGHT MODE] Paused in HOLD - Operator Override")
            elif cmd == p.CMD_RESUME:
                drone_link.mode = p.MODE_PATROL
                drone_state["mode"] = p.MODE_PATROL
                print(">>> [DRONE FLIGHT MODE] Resumed PATROL")
            elif cmd == p.CMD_RTB:
                drone_link.mode = p.MODE_RTB
                drone_state["mode"] = p.MODE_RTB
                print(">>> [DRONE FLIGHT MODE] Executing RETURN TO BASE (RTB)")
            elif cmd == p.CMD_OPERATOR_ACK:
                print(f">>> [OPERATOR ACKNOWLEDGED] Alert #{ref_seq}")

    drone_link = Link(
        drone_port,
        on_packet=on_drone_packet,
        retry_timeout=0.25,
        boot_wait=0.0,
    ).start()
    default_clip = os.path.join(_REPO_ROOT, "data", "clips", "uav0000268_05773_v.mp4")
    if not os.path.isfile(default_clip):
        default_clip = os.path.join(_REPO_ROOT, "ground_station", "static", "clips", "uav0000268_05773_v.mp4")
    drone_link.set_video_source_info(
        is_video=True,
        filename="uav0000268_05773_v.mp4",
        filepath=default_clip,
    )

    frame_idx = 0
    while not stop_event.is_set():
        time.sleep(2.0)
        frame_idx += 1

        # Move drone if in PATROL
        if drone_state["mode"] == p.MODE_PATROL:
            drone_state["lat"] += 0.0001 * math.cos(math.radians(drone_state["heading"]))
            drone_state["lon"] += 0.0001 * math.sin(math.radians(drone_state["heading"]))
        elif drone_state["mode"] == p.MODE_RTB:
            # Fly back towards base
            drone_state["lat"] += (53.5232 - drone_state["lat"]) * 0.2
            drone_state["lon"] += (-113.5263 - drone_state["lon"]) * 0.2

        if visdrone_samples:
            seq_id, objs = visdrone_samples[(frame_idx - 1) % len(visdrone_samples)]
            counts = collections.Counter(max(0, min(9, int(o.get("cls", 4)) - 1)) for o in objs)
            cls_idx, count = counts.most_common(1)[0]
            matching_confs = [float(o.get("conf", 0.88)) for o in objs if max(0, min(9, int(o.get("cls", 4)) - 1)) == cls_idx]
            conf = round(max(matching_confs) if matching_confs else 0.88, 2)
            count = min(255, count)
        else:
            cls_idx = random.choice([0, 3, 3, 4, 5, 9])
            conf = round(random.uniform(0.78, 0.95), 2)
            count = random.randint(1, 5)

        alert_lat = drone_state["lat"] + random.uniform(-0.0002, 0.0002)
        alert_lon = drone_state["lon"] + random.uniform(-0.0002, 0.0002)

        ts = p.now_ms()
        seq = drone_link.send(
            p.DETECTION,
            ts_ms=ts,
            cls=cls_idx,
            conf=conf,
            count=count,
            lat=alert_lat,
            lon=alert_lon,
        )

        cls_name = p.CLASS_NAMES[cls_idx]
        print(f"[DRONE TX] Alert #{seq}: {cls_name} x{count} (conf={conf}) at ({alert_lat:.4f}, {alert_lon:.4f})")

    drone_link.stop()


def _free_port(port: int) -> None:
    """Free any stale process occupying the given TCP port so uvicorn never hits Errno 48."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        if s.connect_ex(("127.0.0.1", port)) != 0:
            return
    my_pid = os.getpid()
    try:
        if sys.platform == "win32":
            out = subprocess.check_output(["netstat", "-ano"], text=True, stderr=subprocess.DEVNULL)
            for line in out.splitlines():
                if f":{port}" in line and "LISTENING" in line:
                    parts = line.split()
                    pid = int(parts[-1])
                    if pid != my_pid and pid > 0:
                        subprocess.run(["taskkill", "/F", "/PID", str(pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            out = subprocess.check_output(["lsof", "-ti", f"tcp:{port}"], text=True, stderr=subprocess.DEVNULL)
            for tok in out.split():
                pid = int(tok)
                if pid != my_pid:
                    os.kill(pid, signal.SIGTERM)
        time.sleep(0.4)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description="ArcDrone End-to-End Live Simulation Showcase")
    parser.add_argument("--http-port", type=int, default=8000, help="Web dashboard port (default: 8000)")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP server host (default: 0.0.0.0)")
    parser.add_argument("--no-browser", action="store_true", help="Do not auto-open browser")
    args = parser.parse_args()

    print("=" * 70)
    print("  ARCDRONE LIVE INTERACTIVE SHOWCASE")
    print("  natHacks 2026 Defense & Security Challenge")
    print("=" * 70)

    _free_port(args.http_port)
    _free_port(8080)
    _free_port(8765)

    # 1. Start Virtual Radio Bridge
    bridge = SimLinkBridge(name_a="sim://drone", name_b="sim://ground")
    bw_monitor = BandwidthMonitor(budget_limit=3000.0)
    bw_monitor.attach_to_bridge(bridge)

    # 2. Start Simulated Drone in background
    stop_event = threading.Event()
    drone_thread = threading.Thread(
        target=start_drone_node,
        args=("sim://drone", bridge, stop_event),
        daemon=True,
    )
    drone_thread.start()

    print("\n[+] Virtual Radio Link Connected: sim://drone <-> sim://ground")
    print("[+] Airborne Drone Patrol Active: Transmitting detections every 2s")
    print("\n" + "*" * 70)
    print(f"  >>> DASHBOARD URL: http://localhost:{args.http_port}")
    print("      (Opening automatically in your default browser...)")
    print("*" * 70 + "\n")

    def _open_browser():
        url = f"http://localhost:{args.http_port}"
        deadline = time.time() + 10.0
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{args.http_port}", timeout=0.8):
                    pass
                time.sleep(0.3)
                webbrowser.open(url)
                return
            except Exception:
                time.sleep(0.4)
        webbrowser.open(url)

    if not args.no_browser:
        threading.Thread(target=_open_browser, daemon=True).start()

    relay = None

    def _cleanup_relay(*_args):
        nonlocal relay
        if relay is not None and relay.poll() is None:
            try:
                relay.terminate()
                relay.wait(timeout=2.0)
            except Exception:
                try:
                    relay.kill()
                except Exception:
                    pass

    atexit.register(_cleanup_relay)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        ws_up = s.connect_ex(("127.0.0.1", 8765)) == 0
    if ws_up:
        print("[+] Scene WebSocket relay already running on :8765")
    else:
        print(f"[+] Starting scene WebSocket relay (:8765, UI served on http://localhost:{args.http_port}/reconstructor.html)")
        relay = subprocess.Popen(
            [sys.executable, "-u", "-m", "drone_node.scene_ws_relay", "--no-http"],
            cwd=_REPO_ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # 3. Start Ground Station Web Server
    app = make_app("sim://ground")
    try:
        uvicorn.run(app, host=args.host, port=args.http_port, log_level="warning")
    finally:
        stop_event.set()
        _cleanup_relay()


if __name__ == "__main__":
    main()
