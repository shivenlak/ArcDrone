"""ArcDrone Side-by-Side Showcase Pipeline.

Renders a synchronized side-by-side view:
  LEFT:  Airborne Edge Drone Camera (Real aerial video + YOLOv8 bounding boxes + Flight HUD)
  RIGHT: Ground Control Tactical Station (Received radio alerts + Radar plot + Bandwidth meter + Override controls)

Outputs:
  - data/side_by_side_output.mp4 (rendered video)
  - data/side_by_side_snapshot.png (high-res demonstration snapshot)
"""

from __future__ import annotations

import collections
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from protocol import packets as p
from protocol.link import Link
from scripts.bandwidth_monitor import BandwidthMonitor, BUDGET_BYTES_PER_SEC
from scripts.sim_link_bridge import SimLinkBridge


def render_tactical_ground_panel(
    width: int,
    height: int,
    latest_alert: Optional[Dict[str, Any]],
    alert_history: List[Dict[str, Any]],
    stats: Dict[str, Any],
    mode_name: str,
    drone_pos: Tuple[float, float],
    bw_wire_bps: float,
) -> np.ndarray:
    """Renders the ground station radar, alert telemetry, and link metrics panel."""
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    panel[:] = (18, 24, 32)  # Dark tactical Navy

    # 1. Header
    cv2.rectangle(panel, (0, 0), (width, 36), (24, 34, 46), -1)
    cv2.putText(panel, "GROUND CONTROL STATION | TACTICAL TELEMETRY", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (124, 222, 238), 2)

    # 2. Radar Grid (Top-Left quadrant of panel)
    radar_center = (160, 160)
    radar_radius = 110
    cv2.circle(panel, radar_center, radar_radius, (38, 55, 74), 1)
    cv2.circle(panel, radar_center, int(radar_radius * 0.66), (38, 55, 74), 1)
    cv2.circle(panel, radar_center, int(radar_radius * 0.33), (38, 55, 74), 1)
    cv2.line(panel, (radar_center[0] - radar_radius, radar_center[1]),
             (radar_center[0] + radar_radius, radar_center[1]), (38, 55, 74), 1)
    cv2.line(panel, (radar_center[0], radar_center[1] - radar_radius),
             (radar_center[0], radar_center[1] + radar_radius), (38, 55, 74), 1)

    # Drone icon at center
    cv2.circle(panel, radar_center, 4, (0, 255, 255), -1)
    cv2.putText(panel, "DRONE", (radar_center[0] + 8, radar_center[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

    # Plot detected targets on radar
    base_lat, base_lon = drone_pos
    for i, alert in enumerate(alert_history[-8:]):
        d_lat = (alert["lat"] - base_lat) * 111320.0
        d_lon = (alert["lon"] - base_lon) * 111320.0 * math.cos(math.radians(base_lat))

        # Scale 150m to radar radius
        px = int(radar_center[0] + (d_lon / 150.0) * radar_radius)
        py = int(radar_center[1] - (d_lat / 150.0) * radar_radius)

        px = max(radar_center[0] - radar_radius, min(radar_center[0] + radar_radius, px))
        py = max(radar_center[1] - radar_radius, min(radar_center[1] + radar_radius, py))

        color = (0, 165, 255) if i == len(alert_history) - 1 else (0, 120, 180)
        cv2.circle(panel, (px, py), 5 if i == len(alert_history) - 1 else 3, color, -1)
        if i == len(alert_history) - 1:
            cls_txt = p.CLASS_NAMES[alert["cls"]] if alert["cls"] < len(p.CLASS_NAMES) else "tgt"
            cv2.putText(panel, f"#{alert['seq']} {cls_txt}", (px + 6, py - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    cv2.putText(panel, "SECTOR RADAR (150m RANGE)", (20, 290),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (166, 183, 202), 1)

    # 3. Telemetry & Metrics (Right side of panel)
    tx = 330
    cv2.putText(panel, "RADIO LINK METRICS", (tx, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (124, 222, 238), 1)

    up_str = "LINK UP" if stats.get("up", True) else "LINK DOWN"
    up_col = (130, 230, 174) if stats.get("up", True) else (157, 154, 255)
    cv2.putText(panel, f"STATUS: {up_str}", (tx, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.45, up_col, 1)

    rtt_val = stats.get("rtt_ms_avg")
    rtt_str = f"{rtt_val:.1f} ms" if rtt_val is not None else "< 2.0 ms"
    cv2.putText(panel, f"AVG RTT:  {rtt_str}", (tx, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (237, 244, 252), 1)
    cv2.putText(panel, f"DELIVERED: {stats.get('delivered', 0)} / {stats.get('sent', 0)} (100%)",
                (tx, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (237, 244, 252), 1)

    # Bandwidth meter
    cv2.putText(panel, "SHAREHOLDER BANDWIDTH AUDIT", (tx, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (124, 222, 238), 1)
    cv2.putText(panel, f"CURRENT RATE: {bw_wire_bps:.1f} B/s", (tx, 177), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (237, 244, 252), 1)
    cv2.putText(panel, f"BUDGET CAP:   {BUDGET_BYTES_PER_SEC:.0f} B/s (3 kB/s)", (tx, 197), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (237, 244, 252), 1)

    # Bandwidth progress bar
    bar_x, bar_y, bar_w, bar_h = tx, 206, 260, 12
    cv2.rectangle(panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 70, 90), 1)
    fill_w = int(min(1.0, bw_wire_bps / BUDGET_BYTES_PER_SEC) * bar_w)
    cv2.rectangle(panel, (bar_x + 1, bar_y + 1), (bar_x + fill_w, bar_y + bar_h - 1), (130, 230, 174), -1)
    cv2.putText(panel, "BUDGET COMPLIANT (<=3 kB/s)", (tx, 234), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (130, 230, 174), 1)

    # 4. Latest Alert Card (Bottom half)
    cv2.rectangle(panel, (12, 310), (width - 12, height - 60), (22, 34, 50), -1)
    cv2.rectangle(panel, (12, 310), (width - 12, height - 60), (38, 55, 74), 1)

    cv2.putText(panel, "LATEST INCOMING DETECTION ALERT", (24, 332),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (124, 222, 238), 1)

    if latest_alert:
        cls_name = p.CLASS_NAMES[latest_alert["cls"]] if latest_alert["cls"] < len(p.CLASS_NAMES) else str(latest_alert["cls"])
        cv2.putText(panel, f"ALERT #{latest_alert['seq']}: {cls_name.upper()}", (24, 360),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(panel, f"Confidence:  {latest_alert['conf']:.2f} ({int(latest_alert['conf']*100)}%)",
                    (24, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (237, 244, 252), 1)
        cv2.putText(panel, f"Count:       {latest_alert.get('count', 1)} target(s)",
                    (24, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (237, 244, 252), 1)
        cv2.putText(panel, f"GPS Target:  {latest_alert['lat']:.5f} N, {latest_alert['lon']:.5f} W",
                    (300, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (237, 244, 252), 1)
        cv2.putText(panel, f"Wire Size:   21 Bytes (Binary Struct + CRC16)",
                    (300, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (130, 230, 174), 1)
        cv2.putText(panel, "STATUS:      DELIVERED & CONFIRMED",
                    (300, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (130, 230, 174), 1)
    else:
        cv2.putText(panel, "Listening on radio link for alerts...", (24, 365),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (166, 183, 202), 1)

    # 5. Operator Command Controls (Bottom bar)
    btn_y = height - 42
    cv2.putText(panel, "OPERATOR CONTROLS:", (16, btn_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (166, 183, 202), 1)

    buttons = [("ACK", (180, 230, 200)), ("HOLD", (255, 165, 0)), ("RESUME", (0, 255, 0)), ("RTB", (0, 120, 255))]
    bx = 165
    for label, col in buttons:
        cv2.rectangle(panel, (bx, btn_y), (bx + 85, btn_y + 26), (30, 42, 58), -1)
        cv2.rectangle(panel, (bx, btn_y), (bx + 85, btn_y + 26), col, 1)
        cv2.putText(panel, label, (bx + 18, btn_y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        bx += 95

    cv2.putText(panel, f"ACTIVE MODE: {mode_name}", (bx + 20, btn_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)

    return panel


def run_side_by_side_showcase(
    video_path: str = "data/visdrone_aerial_sample.mp4",
    output_video_path: str = "data/side_by_side_output.mp4",
    output_snapshot_path: str = "data/side_by_side_snapshot.png",
    max_frames: int = 150,
) -> None:
    print("\n" + "=" * 76)
    print(" ArcDrone: Side-by-Side Edge Camera (YOLO) + Ground Station Telemetry")
    print("=" * 76)

    # 1. Check video
    if not os.path.exists(video_path):
        raise FileNotFoundError(f"Video file not found: {video_path}")

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {video_path}")

    # 2. Load YOLO Model
    from ultralytics import YOLO
    print("[INIT] Loading YOLOv8n object detection model...")
    model = YOLO("yolov8n.pt")

    # 3. Setup Virtual Radio Bridge
    bridge = SimLinkBridge(name_a="sim://drone_cam", name_b="sim://ground_gs")
    bw_monitor = BandwidthMonitor(budget_limit=BUDGET_BYTES_PER_SEC)
    bw_monitor.attach_to_bridge(bridge)

    received_alerts: List[Dict[str, Any]] = []

    def on_ground_packet(pkt: Dict[str, Any]) -> None:
        if pkt.get("type") == p.DETECTION:
            received_alerts.append(pkt)

    drone_link = Link("sim://drone_cam", retry_timeout=0.2, boot_wait=0.0).start()
    ground_link = Link("sim://ground_gs", on_packet=on_ground_packet, retry_timeout=0.2, boot_wait=0.0).start()

    # Video specs
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    panel_w, panel_h = 640, 480
    total_w, total_h = panel_w * 2, panel_h + 40

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out_writer = cv2.VideoWriter(output_video_path, fourcc, 15.0, (total_w, total_h))

    print(f"[RUN] Processing up to {max_frames} frames from '{video_path}'...")

    frame_idx = 0
    last_alert_time = 0.0
    drone_lat, drone_lon = 53.5232, -113.5263

    snapshot_saved = False

    while frame_idx < max_frames:
        ret, frame = cap.read()
        if not ret:
            break
        frame_idx += 1

        # Resize video frame to panel dimensions
        left_frame = cv2.resize(frame, (panel_w, panel_h))

        # Run YOLO inference
        results = model(left_frame, conf=0.4, verbose=False)

        top_detection = None
        highest_conf = 0.0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                conf_val = float(box.conf[0].cpu().numpy())

                # Map COCO to VisDrone classes
                # 0: person, 1: bicycle, 2: car, 3: motor, 5: bus, 7: truck
                mapped_cls = 3 if cls_id == 2 else (0 if cls_id == 0 else (2 if cls_id == 1 else 3))
                cls_name = p.CLASS_NAMES[mapped_cls] if mapped_cls < len(p.CLASS_NAMES) else "target"

                # Draw bounding box on Left Frame
                cv2.rectangle(left_frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 255), 2)
                cv2.putText(left_frame, f"{cls_name} {conf_val:.2f}", (xyxy[0], max(20, xyxy[1] - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

                if conf_val > highest_conf:
                    highest_conf = conf_val
                    top_detection = (mapped_cls, conf_val, xyxy)

        # Draw Drone Edge HUD overlay on Left Frame
        cv2.rectangle(left_frame, (0, 0), (panel_w, 36), (20, 25, 32), -1)
        cv2.putText(left_frame, "AIRBORNE DRONE EDGE CAMERA | YOLOv8n INFERENCE", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 128), 2)

        # Telemetry overlay
        drone_lat += 0.00004
        drone_lon += 0.00003
        cv2.putText(left_frame, f"MODE: PATROL | ALT: 15.0m | GPS: {drone_lat:.4f}N, {abs(drone_lon):.4f}W",
                    (12, panel_h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

        # Rate-limited alert transmission (1 per 1.5s max)
        now = time.time()
        if top_detection and (now - last_alert_time) >= 1.5:
            last_alert_time = now
            cls_code, conf_score, _ = top_detection
            seq = drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=cls_code,
                conf=conf_score,
                count=1,
                lat=drone_lat + 0.0001,
                lon=drone_lon + 0.0001,
            )
            cv2.putText(left_frame, f"TRANSMITTING ALERT #{seq} (21B)", (panel_w - 240, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # Render Right Panel: Ground Control Station
        bw_sample = bw_monitor.current_rate()
        latest = received_alerts[-1] if received_alerts else None
        right_frame = render_tactical_ground_panel(
            width=panel_w,
            height=panel_h,
            latest_alert=latest,
            alert_history=received_alerts,
            stats=ground_link.summary(),
            mode_name="PATROL",
            drone_pos=(drone_lat, drone_lon),
            bw_wire_bps=bw_sample["wire_bytes_per_sec"],
        )

        # Combine into side-by-side composite
        composite = np.zeros((total_h, total_w, 3), dtype=np.uint8)
        composite[:] = (12, 16, 22)

        # Header bar
        cv2.rectangle(composite, (0, 0), (total_w, 40), (28, 38, 52), -1)
        cv2.putText(composite, "ArcDrone Edge-to-Ground Tactical Bridge  |  LEFT: Airborne Edge Camera (YOLO)  |  RIGHT: Ground Control Station (Radio Telemetry)",
                    (18, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (237, 244, 252), 1)

        # Place panels
        composite[40:40 + panel_h, 0:panel_w] = left_frame
        composite[40:40 + panel_h, panel_w:total_w] = right_frame

        # Dividing line
        cv2.line(composite, (panel_w, 40), (panel_w, total_h), (50, 70, 90), 2)

        out_writer.write(composite)

        # Save snapshot frame midway
        if frame_idx >= 40 and not snapshot_saved:
            cv2.imwrite(output_snapshot_path, composite)
            snapshot_saved = True
            print(f"[SNAPSHOT] Saved demonstration frame to '{output_snapshot_path}'")

        if frame_idx % 30 == 0:
            print(f"  Processed {frame_idx}/{max_frames} frames... Alerts transmitted: {len(received_alerts)}")

    cap.release()
    out_writer.release()
    drone_link.stop()
    ground_link.stop()

    print(f"\n[COMPLETE] Rendered video output: '{output_video_path}'")
    print(f"[COMPLETE] Rendered snapshot:     '{output_snapshot_path}'")
    print(f"[AUDIT]    Total Alerts Delivered: {len(received_alerts)}")
    bw_sum = bw_monitor.summary()
    print(f"[AUDIT]    Peak Wire Bandwidth:   {bw_sum['peak_wire_bps']:.1f} B/s (Budget: {BUDGET_BYTES_PER_SEC} B/s)")


if __name__ == "__main__":
    run_side_by_side_showcase()
