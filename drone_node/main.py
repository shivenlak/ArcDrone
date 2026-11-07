"""ArcDrone airborne edge node: CV detection, telemetry, HUD, and command handling."""
import argparse
import queue
import time
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from drone_node.detector import Detector
from drone_node.sim_gps import SimGps
from protocol import packets as p
from protocol.link import Link


def draw_hud(
    frame: np.ndarray,
    mode: int,
    link_up: bool,
    buffered_count: int,
    ack_banner: str,
    last_rtt: Optional[float],
    boxes: List[Dict[str, Any]],
    drone_lat: float,
    drone_lon: float,
    last_tx_str: str,
) -> np.ndarray:
    canvas = frame.copy()
    h, w = canvas.shape[:2]

    # Draw detected bounding boxes
    for b in boxes:
        x1, y1, x2, y2 = b["bbox"]
        label = f"{b['label']} {b['conf']:.2f}"
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(
            canvas,
            label,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )

    # Top header bar
    cv2.rectangle(canvas, (0, 0), (w, 40), (20, 20, 20), -1)

    if mode == p.MODE_HOLD:
        mode_str = "MODE: HOLD - OPERATOR OVERRIDE"
        mode_color = (0, 165, 255)
    elif mode == p.MODE_RTB:
        mode_str = "MODE: RETURNING TO BASE (RTB)"
        mode_color = (0, 0, 255)
    else:
        mode_str = "MODE: PATROL"
        mode_color = (0, 255, 0)

    cv2.putText(
        canvas, mode_str, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, mode_color, 2
    )

    link_str = "LINK: UP" if link_up else "LINK: DOWN"
    link_color = (0, 255, 0) if link_up else (0, 0, 255)
    rtt_str = f" ({int(last_rtt)}ms)" if last_rtt is not None else ""
    right_status = f"{link_str}{rtt_str} | BUF: {buffered_count}"
    cv2.putText(
        canvas,
        right_status,
        (max(10, w - 310), 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        link_color,
        2,
    )

    # Center overrides
    if mode == p.MODE_HOLD:
        cv2.rectangle(canvas, (0, h // 2 - 30), (w, h // 2 + 20), (0, 0, 0), -1)
        cv2.putText(
            canvas,
            "HOLD - OPERATOR OVERRIDE (VIDEO PAUSED)",
            (max(10, w // 2 - 230), h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 165, 255),
            2,
        )
    elif mode == p.MODE_RTB:
        cv2.rectangle(canvas, (0, h // 2 - 30), (w, h // 2 + 20), (0, 0, 0), -1)
        cv2.putText(
            canvas,
            "RETURNING TO BASE",
            (max(10, w // 2 - 120), h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

    # Bottom status bar
    cv2.rectangle(canvas, (0, h - 35), (w, h), (20, 20, 20), -1)
    gps_str = f"GPS: SIMULATED ({drone_lat:.4f}, {drone_lon:.4f})"
    cv2.putText(
        canvas, gps_str, (12, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1
    )

    if last_tx_str:
        cv2.putText(
            canvas,
            last_tx_str,
            (w // 2 - 130, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
        )

    if ack_banner:
        cv2.putText(
            canvas,
            ack_banner,
            (max(10, w - 310), h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 0),
            1,
        )

    return canvas


def parse_args():
    parser = argparse.ArgumentParser(description="ArcDrone Edge CV & Telemetry Node")
    parser.add_argument("--port", required=True, help="Serial port or fake_bridge pty")
    parser.add_argument("--video", required=True, help="Path to video clip or camera index (e.g. 0)")
    parser.add_argument("--conf", type=float, default=0.5, help="Detection confidence gate")
    parser.add_argument("--cooldown", type=float, default=3.0, help="Per-class cooldown (sec)")
    parser.add_argument("--fps", type=float, default=1.0, help="Target inference loop FPS")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Path to YOLO weights (.pt)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    cmd_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
    state: Dict[str, Any] = {
        "link_up": False,
        "last_rtt": None,
        "ack_banner": "",
        "last_tx_str": "",
    }

    def on_event(event_type: str, **kwargs: Any) -> None:
        if event_type == "up":
            state["link_up"] = True
        elif event_type == "down":
            state["link_up"] = False
        elif event_type == "deliver":
            state["link_up"] = True
            state["last_rtt"] = kwargs.get("rtt_ms")
        print(f"[EVENT] {event_type} | {kwargs}")

    def on_packet(pkt: Dict[str, Any]) -> None:
        cmd_queue.put(pkt)

    print(f"[MAIN] Initializing Link on {args.port}...")
    link = Link(args.port, on_packet=on_packet, on_event=on_event)
    link.mode = p.MODE_PATROL
    link.start()

    detector = Detector(
        model_path=args.model,
        conf_thresh=args.conf,
        cooldown_sec=args.cooldown,
    )
    sim_gps = SimGps()

    video_src = int(args.video) if str(args.video).isdigit() else args.video
    cap = cv2.VideoCapture(video_src)
    if not cap.isOpened():
        print(f"[WARN] Could not open video '{args.video}'. Running with fallback test frame.")

    last_frame: Optional[np.ndarray] = None
    frame_idx = 0
    frame_period = 1.0 / max(0.1, args.fps)
    current_boxes: List[Dict[str, Any]] = []

    print("[MAIN] Pipeline active. Press 'q' or ESC in OpenCV window to exit.")
    try:
        while True:
            t0 = time.time()

            # 1. Drain COMMAND queue strictly matching protocol contract
            while not cmd_queue.empty():
                pkt = cmd_queue.get_nowait()
                if pkt.get("type") != p.COMMAND:
                    continue

                cmd = pkt["cmd"]
                ref_seq = pkt.get("ref_seq", 0)

                if cmd == p.CMD_OPERATOR_ACK:
                    state["ack_banner"] = f"Alert #{ref_seq} acknowledged"
                    print(f"[CMD] Operator acknowledged Alert #{ref_seq}")
                elif cmd == p.CMD_HOLD:
                    link.mode = p.MODE_HOLD
                    print("[CMD] Switching to MODE_HOLD (pausing video)")
                elif cmd == p.CMD_RESUME:
                    link.mode = p.MODE_PATROL
                    print("[CMD] Switching to MODE_PATROL (resuming video)")
                elif cmd == p.CMD_RTB:
                    link.mode = p.MODE_RTB
                    print("[CMD] Switching to MODE_RTB")

            # 2. Advance frame and compute GPS
            drone_lat, drone_lon = sim_gps.position_at_frame(frame_idx, fps=args.fps)

            if link.mode != p.MODE_HOLD or last_frame is None:
                if cap.isOpened():
                    ret, frame = cap.read()
                    if not ret:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, frame = cap.read()
                    if ret and frame is not None:
                        last_frame = frame
                        frame_idx += 1

                if last_frame is None:
                    last_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        last_frame,
                        f"Waiting for video: {args.video} (frame {frame_idx})",
                        (40, 240),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (180, 180, 180),
                        2,
                    )
                    frame_idx += 1

                # 3. Run inference when not in HOLD
                current_boxes, alert = detector.process_frame(last_frame)

                # 4. Fire DETECTION packet if an alert qualified
                if alert is not None:
                    t_lat, t_lon = sim_gps.target_location(
                        drone_lat, drone_lon, alert["bbox"], last_frame.shape[:2]
                    )
                    ts = p.now_ms()
                    seq = link.send(
                        p.DETECTION,
                        ts_ms=ts,
                        cls=alert["cls"],
                        conf=alert["conf"],
                        count=alert["count"],
                        lat=t_lat,
                        lon=t_lon,
                    )
                    state["last_tx_str"] = (
                        f"TX #{seq}: {alert['label']} x{alert['count']} ({alert['conf']:.2f})"
                    )
                    print(
                        f"[TX DETECTION] seq={seq} cls={alert['cls']} ({alert['label']}) "
                        f"conf={alert['conf']:.2f} count={alert['count']} "
                        f"lat={t_lat:.5f} lon={t_lon:.5f}"
                    )

            # 5. Render HUD with exact buffered count
            hud_frame = draw_hud(
                last_frame,
                mode=link.mode,
                link_up=state["link_up"],
                buffered_count=link.buffered,
                ack_banner=state["ack_banner"],
                last_rtt=state["last_rtt"],
                boxes=current_boxes,
                drone_lat=drone_lat,
                drone_lon=drone_lon,
                last_tx_str=state["last_tx_str"],
            )
            cv2.imshow("ArcDrone - Airborne Edge Node", hud_frame)

            elapsed = time.time() - t0
            wait_ms = max(1, int((frame_period - elapsed) * 1000))
            key = cv2.waitKey(wait_ms) & 0xFF
            if key in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
    finally:
        if cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()
        link.stop()


if __name__ == "__main__":
    main()