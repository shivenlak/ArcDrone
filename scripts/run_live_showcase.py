"""Live Interactive natHacks Showcase Server.

Spins up:
1. SimLinkBridge connecting virtual drone and ground radios.
2. An airborne drone simulation emitting real-time detections and responding to commands.
3. Ground Station web server + live WebSocket dashboard on http://localhost:8000.
4. UAARG telemetry logging in tmp/log/.
"""

import argparse
import math
import os
import random
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
from scripts.uaarg_adapter.event_logger import EventLogger
from scripts.uaarg_adapter.mission_monitor import MissionMonitor

CLASSES = ["pedestrian", "car", "van", "truck", "bus", "motor"]


def start_drone_node(drone_port: str, bridge: SimLinkBridge, stop_event: threading.Event):
    event_logger = EventLogger(base_dir="tmp/log")
    mission_monitor = MissionMonitor()

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
                mission_monitor.update_mode("HOLD")
                print(">>> [DRONE FLIGHT MODE] Paused in HOLD - Operator Override")
            elif cmd == p.CMD_RESUME:
                drone_link.mode = p.MODE_PATROL
                drone_state["mode"] = p.MODE_PATROL
                mission_monitor.update_mode("PATROL")
                print(">>> [DRONE FLIGHT MODE] Resumed PATROL")
            elif cmd == p.CMD_RTB:
                drone_link.mode = p.MODE_RTB
                drone_state["mode"] = p.MODE_RTB
                mission_monitor.update_mode("RTB")
                print(">>> [DRONE FLIGHT MODE] Executing RETURN TO BASE (RTB)")
            elif cmd == p.CMD_OPERATOR_ACK:
                print(f">>> [OPERATOR ACKNOWLEDGED] Alert #{ref_seq}")

    drone_link = Link(
        drone_port,
        on_packet=on_drone_packet,
        retry_timeout=0.25,
        boot_wait=0.0,
    ).start()

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

        # Generate a simulated detection
        cls_idx = random.choice([0, 1, 1, 1, 2, 3])  # bias to cars & pedestrians
        conf = round(random.uniform(0.75, 0.96), 2)
        count = random.randint(1, 3)

        alert_lat = drone_state["lat"] + random.uniform(-0.0003, 0.0003)
        alert_lon = drone_state["lon"] + random.uniform(-0.0003, 0.0003)

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
        print(f"[DRONE TX] Alert #{seq}: {cls_name} (conf={conf}) at ({alert_lat:.4f}, {alert_lon:.4f})")

        # Log event with UAARG event logger
        event_logger.log_event(
            seq=seq,
            detection={"cls": cls_name, "conf": conf, "count": count},
            location={"lat": alert_lat, "lon": alert_lon},
            mode=p.MODE_NAMES.get(drone_state["mode"], "UNKNOWN"),
        )

    drone_link.stop()


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
    print("[+] UAARG Event Logger Active: Logging to tmp/log/")
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

    # 3. Start Ground Station Web Server
    app = make_app("sim://ground")
    try:
        uvicorn.run(app, host=args.host, port=args.http_port, log_level="warning")
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
