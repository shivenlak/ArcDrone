"""ArcDrone airborne edge node: CV detection, telemetry, HUD, and command handling."""
import argparse
import queue
import socket
import struct
import time
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from drone_node.delivery_verifier import DeliveryVerifier
from drone_node.detector import Detector
from drone_node.frame_source import open_frame_source
from drone_node.preview_stream import PreviewStream
from drone_node.scene_codec import (
    LINK_MAX_KBPS,
    SCENE_HZ_DEFAULT,
    SCENE_MAX_KBPS,
    SceneCodec,
)
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
    scene_kbps: float = 0.0,
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
    right_status = (
        f"{link_str}{rtt_str} | BUF: {buffered_count} | "
        f"scene {scene_kbps:.2f}/{SCENE_MAX_KBPS:.1f} kbps (link {LINK_MAX_KBPS:.1f})"
    )
    cv2.putText(
        canvas,
        right_status,
        (max(10, w - 410), 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
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
    parser.add_argument(
        "--video",
        required=True,
        help="MP4 path, VisDrone-VID image folder (uav.../0000001.jpg), or camera index (0)",
    )
    parser.add_argument("--conf", type=float, default=0.5, help="Detection confidence gate")
    parser.add_argument(
        "--imgsz",
        type=int,
        default=1280,
        help="YOLO inference size; use 1280/1536 for high-res aerial footage",
    )
    parser.add_argument("--cooldown", type=float, default=3.0, help="Per-class cooldown (sec)")
    parser.add_argument(
        "--spatial-hold",
        type=float,
        default=25.0,
        help="Suppress repeat alerts for the same contact while it stays in view (sec)",
    )
    parser.add_argument("--fps", type=float, default=2.0, help="Camera / YOLO inference rate")
    parser.add_argument(
        "--scene-hz",
        type=float,
        default=SCENE_HZ_DEFAULT,
        help="Scene vector UDP rate (default 12/s); shares %.1f kbps link cap"
        % LINK_MAX_KBPS,
    )
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Path to YOLO weights (.pt)")
    parser.add_argument("--scene-host", type=str, default="127.0.0.1", help="Ground reconstructor IP")
    parser.add_argument("--scene-port", type=int, default=9999, help="Ground reconstructor UDP port")
    parser.add_argument(
        "--preview-port",
        type=int,
        default=0,
        help="Serve the HUD video over Wi-Fi (MJPEG) for the dashboard's Wi-Fi camera tab, "
        "e.g. 8081. Off by default; never sent over the radio.",
    )
    parser.add_argument("--headless", action="store_true", help="Run without local video window (Pi/Blind demo)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    cmd_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
    state: Dict[str, Any] = {
        "last_rtt": None,
        "ack_banner": "",
        "last_tx_str": "",
        "scene_kbps": 0.0,
        # Live numbers for the dashboard (over Wi-Fi via --preview-port; not the radio).
        "infer_ms": None,
        "fps": None,
        "alerts_sent": 0,
        "last_alert": "",
        "new_frame": False,
        "preview_at": 0.0,
    }

    verifier = DeliveryVerifier()

    def on_event(event_type: str, **kwargs: Any) -> None:
        if event_type == "delivered":
            seq = kwargs.get("seq")
            rtt = kwargs.get("rtt_ms")
            state["last_rtt"] = rtt
            if seq is not None:
                verifier.on_delivered(int(seq), rtt)
        elif event_type == "link_down":
            verifier.on_link_down()
        elif event_type == "link_up":
            verifier.on_link_up()
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
        spatial_hold_sec=args.spatial_hold,
        imgsz=args.imgsz,
    )
    sim_gps = SimGps()
    scene_codec = SceneCodec(max_kbps=SCENE_MAX_KBPS, dst_size=(720, 540))
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass
    preview = PreviewStream(port=args.preview_port).start() if args.preview_port > 0 else None
    if preview is not None:
        print("[MAIN] Wi-Fi camera preview on http://0.0.0.0:%d/video.mjpg (not over the radio)" % args.preview_port)
    print(
        "[MAIN] Scene %d Hz, budget capped at %.1f kbps"
        % (int(args.scene_hz), SCENE_MAX_KBPS)
    )

    source = open_frame_source(args.video)
    use_camera = source.is_live_camera()
    _clip_order = [
        "uav0000268_05773_v",
        "uav0000086_00000_v",
        "uav0000182_00000_v",
        "uav0000117_02622_v",
        "uav0000339_00001_v",
        "uav0000137_00458_v",
    ]
    clip_idx = 0
    for _i, _cid in enumerate(_clip_order, start=1):
        if _cid in str(args.video):
            clip_idx = _i
            break
    if not source.isOpened():
        if use_camera:
            print(
                f"[WARN] Could not open camera index {args.video}. "
                "On macOS: grant Camera access and try --video 0 or --video 1."
            )
        else:
            print(
                f"[WARN] Could not open video or sequence '{args.video}'. "
                "Using fallback test frame."
            )

    last_frame: Optional[np.ndarray] = None
    frame_idx = 0
    scene_tick = 0
    infer_period = 1.0 / max(0.1, args.fps)
    scene_period = 1.0 / max(1.0, args.scene_hz)
    loop_period = 1.0 / max(args.fps, args.scene_hz, 1.0)
    last_infer = float("-inf")  # read a frame on the first loop (macOS monotonic() starts near 0)
    last_scene_tx = 0.0
    current_boxes: List[Dict[str, Any]] = []
    drone_lat, drone_lon = sim_gps.base_lat, sim_gps.base_lon

    print("[MAIN] Pipeline active. Press 'q' or ESC in OpenCV window (or Ctrl+C) to exit.")
    try:
        while True:
            t0 = time.time()
            now_mono = time.monotonic()

            # 1. Drain COMMAND queue strictly matching protocol contract
            while not cmd_queue.empty():
                pkt = cmd_queue.get_nowait()
                if pkt.get("type") != p.COMMAND:
                    continue

                cmd = pkt["cmd"]
                ref_seq = pkt.get("ref_seq", 0)

                if cmd == p.CMD_OPERATOR_ACK:
                    state["ack_banner"] = f"Alert #{ref_seq} acknowledged by operator"
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

            drone_lat, drone_lon = sim_gps.position_at_frame(frame_idx, fps=args.fps)
            alert = None

            # 2. Camera + YOLO at --fps (skipped in HOLD once a frame exists)
            if (link.mode != p.MODE_HOLD or last_frame is None) and (
                now_mono - last_infer >= infer_period
            ):
                last_infer = now_mono
                if source.isOpened():
                    ret, frame = source.read()
                    if not ret and not use_camera:
                        source.rewind()
                        ret, frame = source.read()
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

                t_infer = time.perf_counter()
                current_boxes, alert = detector.process_frame(last_frame)
                infer_ms = (time.perf_counter() - t_infer) * 1000
                state["infer_ms"] = infer_ms if state["infer_ms"] is None else 0.8 * state["infer_ms"] + 0.2 * infer_ms
                if state.get("last_frame_t"):
                    fps_now = 1.0 / max(1e-3, now_mono - state["last_frame_t"])
                    state["fps"] = fps_now if state["fps"] is None else 0.8 * state["fps"] + 0.2 * fps_now
                state["last_frame_t"] = now_mono
                state["new_frame"] = True

            # 3. Scene vectors at --scene-hz (default 12/s); uses latest boxes, not full link cap
            if (
                last_frame is not None
                and link.mode != p.MODE_HOLD
                and (now_mono - last_scene_tx >= scene_period)
            ):
                last_scene_tx = now_mono
                slat, slon = sim_gps.position_at_frame(scene_tick, fps=args.scene_hz)
                scene_tick += 1
                scene_bytes, kbps = scene_codec.encode_frame(
                    current_boxes,
                    last_frame.shape[:2],
                    slat,
                    slon,
                    scene_hz=args.scene_hz,
                )
                scene_bytes = scene_bytes + struct.pack(">BBH", 0xCF, clip_idx, frame_idx & 0xFFFF)
                state["scene_kbps"] = kbps
                link.send_scene(scene_bytes)
                udp_sock.sendto(scene_bytes, (args.scene_host, args.scene_port))
                if args.scene_host == "127.0.0.1":
                    try:
                        udp_sock.sendto(scene_bytes, ("255.255.255.255", args.scene_port))
                    except Exception:
                        pass

            # 4. Reliable DETECTION on serial/radio (separate from scene UDP budget)
            if alert is not None and last_frame is not None:
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
                verifier.on_send(
                    seq,
                    "DETECTION",
                    f"{alert['label']} x{alert['count']}",
                    buffered=(not link.up or link.buffered > 1),
                )
                state["last_tx_str"] = (
                    f"TX #{seq}: {alert['label']} x{alert['count']} ({alert['conf']:.2f})"
                )
                state["alerts_sent"] += 1
                state["last_alert"] = f"#{seq} {alert['label']} x{alert['count']} ({alert['conf']:.2f})"
                print(
                    f"[TX DETECTION] seq={seq} cls={alert['cls']} ({alert['label']}) "
                    f"conf={alert['conf']:.2f} count={alert['count']} "
                    f"lat={t_lat:.5f} lon={t_lon:.5f}"
                )

            # 5. HUD for the local window, and the optional Wi-Fi preview. Neither goes over the radio.
            hud_frame = None
            if last_frame is not None and (not args.headless or preview is not None):
                hud_frame = draw_hud(
                    last_frame,
                    mode=link.mode,
                    link_up=link.up,
                    buffered_count=link.buffered,
                    ack_banner=state["ack_banner"],
                    last_rtt=state["last_rtt"],
                    boxes=current_boxes,
                    drone_lat=drone_lat,
                    drone_lon=drone_lon,
                    last_tx_str=state["last_tx_str"],
                    scene_kbps=state["scene_kbps"],
                )
                # Send a frame only when YOLO produced a new one, plus once a second so banners
                # (HOLD, link lost) still show while the video is frozen.
                if preview is not None and (state["new_frame"] or now_mono - state["preview_at"] > 1.0):
                    preview.update(hud_frame)
                    state["new_frame"] = False
                    state["preview_at"] = now_mono
                    summary = link.summary()
                    preview.update_stats({
                        "source": str(args.video),
                        "model": args.model,
                        "imgsz": args.imgsz,
                        "conf_gate": args.conf,
                        "infer_ms": round(state["infer_ms"], 1) if state["infer_ms"] else None,
                        "fps": round(state["fps"], 1) if state["fps"] else None,
                        "objects": len(current_boxes),
                        "alerts_sent": state["alerts_sent"],
                        "last_alert": state["last_alert"],
                        "mode": p.MODE_NAMES.get(link.mode, "?"),
                        "link_up": link.up,
                        "buffered": link.buffered,
                        "delivered": summary["delivered"],
                        "rtt_ms_avg": summary["rtt_ms_avg"],
                        "radio_bytes": summary["bytes_tx"] + summary["bytes_rx"],
                    })

            if not args.headless and hud_frame is not None:
                cv2.imshow("ArcDrone - Airborne Edge Node", hud_frame)
                elapsed = time.time() - t0
                wait_ms = max(1, int((loop_period - elapsed) * 1000))
                key = cv2.waitKey(wait_ms) & 0xFF
                if key in (ord("q"), 27):
                    break
            else:
                elapsed = time.time() - t0
                sleep_s = max(0.001, loop_period - elapsed)
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
    finally:
        try:
            print(verifier.report(link.summary()))
        except Exception:
            pass
        if preview is not None:
            preview.stop()
        source.release()
        udp_sock.close()
        cv2.destroyAllWindows()
        link.stop()


if __name__ == "__main__":
    main()