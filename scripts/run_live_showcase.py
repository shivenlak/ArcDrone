"""Live Interactive natHacks Showcase Server.

Spins up:
1. SimLinkBridge connecting virtual drone and ground radios.
2. An airborne drone simulation emitting real-time detections and responding to commands.
3. Ground Station web server + live WebSocket dashboard on http://localhost:8000.
4. UAARG telemetry logging in tmp/log/.
"""

import math
import os
import random
import sys
import threading
import time
import uvicorn

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from ground_station.server import make_app
from protocol import packets as p
from protocol.link import Link
from drone_node.delivery_verifier import DeliveryVerifier
from drone_node.sim_gps import SimGps
from scripts.bandwidth_monitor import BandwidthMonitor
from scripts.sim_link_bridge import SimLinkBridge
from scripts.uaarg_adapter.event_logger import EventLogger
from scripts.uaarg_adapter.mission_monitor import MissionMonitor

CLASSES = ["pedestrian", "car", "van", "truck", "bus", "motor"]


def start_drone_node(drone_port: str, bridge: SimLinkBridge, stop_event: threading.Event):
    event_logger = EventLogger(base_dir="tmp/log")
    mission_monitor = MissionMonitor()
    verifier = DeliveryVerifier()
    sim_gps = SimGps()

    drone_state = {"mode": p.MODE_PATROL, "lat": 53.5232, "lon": -113.5263, "heading": 45.0}

    def on_drone_event(event_type: str, **kwargs):
        if event_type == "delivered":
            seq = kwargs.get("seq")
            rtt = kwargs.get("rtt_ms")
            if seq is not None:
                verifier.on_delivered(int(seq), rtt)
        elif event_type == "link_down":
            verifier.on_link_down()
        elif event_type == "link_up":
            verifier.on_link_up()

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
        on_event=on_drone_event,
        retry_timeout=0.25,
        boot_wait=0.0,
    ).start()

    frame_idx = 0
    while not stop_event.is_set():
        time.sleep(2.0)
        frame_idx += 1

        # Move drone on figure-eight patrol if in PATROL
        if drone_state["mode"] == p.MODE_PATROL:
            drone_state["lat"], drone_state["lon"] = sim_gps.position_at_frame(frame_idx, fps=0.5)
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
        verifier.on_send(
            seq,
            "DETECTION",
            f"{cls_name} x{count}",
            buffered=(not drone_link.up or drone_link.buffered > 1),
        )
        print(f"[DRONE TX] Alert #{seq}: {cls_name} (conf={conf}) at ({alert_lat:.4f}, {alert_lon:.4f})")

        # Log event with UAARG event logger
        event_logger.log_event(
            seq=seq,
            detection={"cls": cls_name, "conf": conf, "count": count},
            location={"lat": alert_lat, "lon": alert_lon},
            mode=p.MODE_NAMES.get(drone_state["mode"], "UNKNOWN"),
        )

    try:
        print("\n" + verifier.report(drone_link.summary()))
    except Exception:
        pass
    drone_link.stop()


def main():
    print("=" * 70)
    print("  ARCDRONE LIVE INTERACTIVE SHOWCASE")
    print("  natHacks 2026 Defense & Security Challenge")
    print("=" * 70)

    # 1. Start Virtual Radio Bridge
    bridge = SimLinkBridge(name_a="sim://drone", name_b="sim://ground")
    bw_monitor = BandwidthMonitor(budget_limit=3000.0)
    bw_monitor.attach_to_bridge(bridge)

    enc_key = os.environ.get("ARCDRONE_KEY", "").strip()
    if enc_key:
        print(f"[+] AES-128-CCM Authenticated Encryption: ACTIVE (ARCDRONE_KEY set)")
    else:
        print("[*] Link Encryption: Plaintext (set ARCDRONE_KEY to activate AES-128-CCM)")

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
    print("  >>> OPEN THIS URL IN YOUR BROWSER:")
    print("      http://localhost:8000")
    print("*" * 70 + "\n")

    # 3. Start Ground Station Web Server on port 8000
    app = make_app("sim://ground")
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
