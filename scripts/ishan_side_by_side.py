"""ArcDrone Pure Ishan Showcase: VisDrone Model & Video Side-by-Side Pipeline.

Demonstrates Ishan's drone_node CV pipeline and Ground Reconstructor:
  LEFT:  Airborne Edge Drone Camera (VisDrone footage + YOLOv8-VisDrone inference + drone_node HUD)
  RIGHT: Ground Tactical Scene Reconstructor (Decoded SceneCodec v2 vectors + Synthetic FOV + Manifest + Sector Matrix)

100% powered by:
  - drone_node.detector.Detector (with yolov8n-visdrone.pt)
  - drone_node.main.draw_hud
  - drone_node.scene_codec.SceneCodec & LoRaLinkBudget
  - drone_node.sim_gps.SimGps
  - drone_node.reconstructor (ObjectTracker, draw_sector_fov, draw_text_bg, tactical manifest)
"""

import os
import sys
import time
import math
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

# Ensure ArcDrone root is in sys.path
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from drone_node.detector import Detector
from drone_node.main import draw_hud
from drone_node.scene_codec import (
    SceneCodec,
    LoRaLinkBudget,
    SCENE_MAX_KBPS,
    LINK_MAX_KBPS,
)
from drone_node.sim_gps import SimGps
import drone_node.reconstructor as recon


def render_ground_reconstructor_panel(
    pane_w: int,
    pane_h: int,
    scene_data: Dict[str, Any],
    rx_budget: LoRaLinkBudget,
    total_packets: int,
    last_rx_time: float,
) -> np.ndarray:
    """Renders Ishan's complete tactical scene reconstructor dashboard."""
    sector_counts = [[0, 0, 0] for _ in range(3)]
    for obj in scene_data["objects"]:
        cx, cy = obj["center"]
        _, s_row, s_col = recon.get_sector_name(cx, cy, pane_w, pane_h)
        sector_counts[s_row][s_col] += 1

    # 1. Left Sub-pane: Synthetic FOV
    left_fov = np.full((pane_h, pane_w, 3), recon.BG_FOV, dtype=np.uint8)
    recon.draw_sector_fov(left_fov, sector_counts)

    for obj in scene_data["objects"]:
        color = recon._class_color(obj["cls"])
        x1, y1, x2, y2 = obj["bbox"]
        cx, cy = obj["center"]
        tid = obj.get("track_id", 0)
        dx, dy = obj.get("vel", (0.0, 0.0))

        sector_name, _, _ = recon.get_sector_name(cx, cy, pane_w, pane_h)
        obj["sector"] = sector_name

        trail = obj.get("trail", [])
        for i in range(1, len(trail)):
            cv2.line(left_fov, trail[i - 1], trail[i], color, 2, cv2.LINE_AA)

        cv2.rectangle(left_fov, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        cv2.circle(left_fov, (cx, cy), 5, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(left_fov, (cx, cy), 5, color, 1, cv2.LINE_AA)

        if math.hypot(dx, dy) >= 3.0:
            end_x = int(min(pane_w - 5, max(5, cx + dx * 2.5)))
            end_y = int(min(pane_h - 5, max(5, cy + dy * 2.5)))
            cv2.arrowedLine(
                left_fov, (cx, cy), (end_x, end_y), (0, 255, 255), 2, tipLength=0.35
            )

        tag = f"ID #{tid} {obj['label'].upper()} {int(obj['conf'] * 100)}%"
        recon.draw_text_bg(left_fov, tag, (x1, max(36, y1 - 6)), 0.45, color)

    cv2.rectangle(left_fov, (0, 0), (pane_w, 32), (22, 28, 22), -1)
    cv2.putText(
        left_fov,
        "SYNTHETIC FOV | 6-byte vectors | 0 B video",
        (10, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (150, 255, 150),
        1,
        cv2.LINE_AA,
    )

    # 2. Right Sub-pane: Tactical Manifest
    right_manifest = np.full((pane_h, pane_w, 3), recon.BG_PANEL, dtype=np.uint8)
    cv2.rectangle(right_manifest, (0, 0), (pane_w, 32), (28, 28, 34), -1)
    cv2.putText(
        right_manifest,
        "TACTICAL MANIFEST | class / sector / motion",
        (10, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        recon.ACCENT,
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        right_manifest,
        "ID   CLASS         SECTOR        POS(%)    STATUS",
        (10, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (170, 170, 170),
        1,
    )
    cv2.line(right_manifest, (8, 60), (pane_w - 8, 60), (60, 60, 70), 1)

    if not scene_data["objects"]:
        cv2.putText(
            right_manifest,
            "NO ACTIVE TARGETS IN FOV",
            (80, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (100, 100, 110),
            1,
        )
    else:
        row_y = 80
        for obj in scene_data["objects"][:8]:
            color = recon._class_color(obj["cls"])
            tid = obj.get("track_id", 0)
            cx, cy = obj["center"]
            nx = int((cx / pane_w) * 100)
            ny = int((cy / pane_h) * 100)
            sector = obj.get("sector", "CENTER")
            motion = obj.get("motion_str", "STATIONARY")

            line_str = (
                f"#{tid:<2} {obj['label'].upper():<11} "
                f"{sector:<12} ({nx:>2}%,{ny:>2}%) {motion}"
            )
            cv2.putText(
                right_manifest,
                line_str,
                (10, row_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
            )
            row_y += 24

    # Sector Occupancy Mini-Grid
    cv2.line(right_manifest, (8, 260), (pane_w - 8, 260), (60, 60, 70), 1)
    cv2.putText(
        right_manifest,
        "SECTOR OCCUPANCY MATRIX:",
        (10, 280),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (200, 200, 200),
        1,
    )

    grid_x0, grid_y0, cell_w, cell_h = 10, 290, 75, 24
    for r in range(3):
        for c in range(3):
            cnt = sector_counts[r][c]
            gx1 = grid_x0 + c * cell_w
            gy1 = grid_y0 + r * cell_h
            fill = (35, 95, 45) if cnt > 0 else (28, 28, 34)
            cv2.rectangle(right_manifest, (gx1, gy1), (gx1 + cell_w - 3, gy1 + cell_h - 3), fill, -1)
            cv2.rectangle(right_manifest, (gx1, gy1), (gx1 + cell_w - 3, gy1 + cell_h - 3), (80, 80, 90), 1)
            cv2.putText(
                right_manifest,
                f"{recon.SECTOR_NAMES[r][c][:6]}:{cnt}",
                (gx1 + 4, gy1 + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (255, 255, 255) if cnt > 0 else (120, 120, 130),
                1,
            )

    # 3. Header & Footer
    body = np.hstack([left_fov, right_manifest])
    full_w = pane_w * 2
    header = np.full((58, full_w, 3), (12, 14, 18), dtype=np.uint8)
    footer = np.full((32, full_w, 3), (12, 14, 18), dtype=np.uint8)

    kbps = rx_budget.current_kbps()
    pct = min(100.0, (kbps / SCENE_MAX_KBPS) * 100.0)
    saved_pct = max(0.0, (1.0 - (kbps / 480.0)) * 100.0)
    link_live = (time.time() - last_rx_time) < 2.0 and total_packets > 0
    live_col = recon.LIVE_OK if link_live else recon.LIVE_STALE
    live_txt = "LIVE" if link_live else "STALE"

    cv2.circle(header, (18, 28), 6, live_col, -1)
    cv2.putText(
        header,
        f"SCENE LINK {live_txt}: {kbps:.2f} / {SCENE_MAX_KBPS:.1f} kbps ({pct:.0f}%) | "
        f"SAVED VS RAW VIDEO: ~{saved_pct:.1f}%",
        (32, 33),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (0, 255, 128) if kbps <= SCENE_MAX_KBPS else (0, 140, 255),
        1,
        cv2.LINE_AA,
    )

    bar_x1, bar_y1, bar_w, bar_h = full_w - 240, 16, 220, 18
    cv2.rectangle(header, (bar_x1, bar_y1), (bar_x1 + bar_w, bar_y1 + bar_h), (40, 40, 40), -1)
    fill_w = int((pct / 100.0) * bar_w)
    cv2.rectangle(header, (bar_x1, bar_y1), (bar_x1 + fill_w, bar_y1 + bar_h), (0, 230, 110), -1)
    cv2.rectangle(header, (bar_x1, bar_y1), (bar_x1 + bar_w, bar_y1 + bar_h), (160, 160, 160), 1)

    cv2.putText(
        footer,
        f"SEQ #{scene_data['seq']} | TARGETS: {scene_data['count']} | "
        f"GPS: ({scene_data['lat']:.4f}, {scene_data['lon']:.4f}) | WIRE FRAMES: {total_packets}",
        (12, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 220, 255),
        1,
    )

    return np.vstack([header, body, footer])


def run_pipeline(
    video_path: str = "data/visdrone_aerial_sample.mp4",
    model_path: str = "yolov8n-visdrone.pt",
    output_mp4: str = "data/ishan_visdrone_side_by_side.mp4",
    output_snapshot: str = "data/ishan_visdrone_side_by_side.png",
    num_frames: int = 150,
) -> None:
    print(f"[ISHAN SHOWCASE] Loading video: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video file {video_path}")

    print(f"[ISHAN SHOWCASE] Loading VisDrone Model: {model_path}")
    detector = Detector(model_path=model_path, conf_thresh=0.25, cooldown_sec=1.0)
    sim_gps = SimGps()
    
    # 420x420 reconstructor panes -> 840x510 total ground panel
    sub_w, sub_h = 420, 420
    scene_codec = SceneCodec(max_kbps=SCENE_MAX_KBPS, dst_size=(sub_w, sub_h))
    rx_budget = LoRaLinkBudget(max_kbps=SCENE_MAX_KBPS)
    tracker = recon.ObjectTracker(max_len=20, max_dist=70.0)

    drone_view_w, drone_view_h = 840, 510
    total_w = 1680
    total_h = 570  # 60px banner + 510px panels

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_mp4, fourcc, 20.0, (total_w, total_h))

    snapshot_saved = False
    total_packets = 0
    last_rx_time = time.time()
    last_scene = {
        "seq": 0,
        "lat": sim_gps.base_lat,
        "lon": sim_gps.base_lon,
        "count": 0,
        "bytes": 0,
        "objects": [],
    }

    print(f"[ISHAN SHOWCASE] Processing {num_frames} frames using Ishan's native pipeline...")
    frame_idx = 0

    while frame_idx < num_frames:
        ret, raw_frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, raw_frame = cap.read()
            if not ret:
                break

        frame_idx += 1
        t_now = time.time()

        # 1. DRONE SIDE: Ishan's VisDrone YOLO inference
        all_boxes, alert = detector.process_frame(raw_frame)
        drone_lat, drone_lon = sim_gps.position_at_frame(frame_idx, fps=20.0)

        # 2. DRONE SIDE: Ishan's SceneCodec v2 compression (< 2 kbps)
        packet, kbps = scene_codec.encode_frame(
            detections=all_boxes,
            frame_shape=raw_frame.shape[:2],
            drone_lat=drone_lat,
            drone_lon=drone_lon,
        )

        # 3. DRONE SIDE: Render Airborne Camera with Ishan's draw_hud()
        alert_str = f"ALERT: {alert['label'].upper()} ({alert['conf']:.2f})" if alert else "SCANNING..."
        drone_hud = draw_hud(
            frame=raw_frame,
            mode=0,  # MODE_PATROL
            link_up=True,
            buffered_count=0,
            ack_banner="RADIO: ESP-NOW / LORA (<=2 kbps)",
            last_rtt=14.5,
            boxes=all_boxes,
            drone_lat=drone_lat,
            drone_lon=drone_lon,
            last_tx_str=alert_str,
            scene_kbps=kbps,
        )
        drone_panel = cv2.resize(drone_hud, (drone_view_w, drone_view_h))

        # 4. GROUND SIDE: Decode compressed scene packet via SceneCodec
        if packet:
            rx_budget.record_tx(len(packet))
            decoded = scene_codec.decode_frame(packet, canvas_w=sub_w, canvas_h=sub_h)
            if decoded:
                decoded["objects"] = tracker.update(decoded["objects"])
                last_scene = decoded
                total_packets += 1
                last_rx_time = t_now

        # 5. GROUND SIDE: Render Ishan's Ground Tactical Scene Reconstructor
        ground_panel = render_ground_reconstructor_panel(
            pane_w=sub_w,
            pane_h=sub_h,
            scene_data=last_scene,
            rx_budget=rx_budget,
            total_packets=total_packets,
            last_rx_time=last_rx_time,
        )
        ground_panel = cv2.resize(ground_panel, (drone_view_w, drone_view_h))

        # 6. Composite Top Title Banner
        composite = np.zeros((total_h, total_w, 3), dtype=np.uint8)
        
        # Banner Header
        cv2.rectangle(composite, (0, 0), (total_w, 60), (14, 18, 24), -1)
        cv2.putText(
            composite,
            "ARCDRONE: VISDRONE YOLO CAMERA vs GROUND TACTICAL RECONSTRUCTOR",
            (24, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            composite,
            "AIRBORNE EDGE (YOLOv8-VisDrone Model + HUD) | GROUND TWIN (<2 kbps SceneCodec v2)",
            (24, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (160, 200, 220),
            1,
            cv2.LINE_AA,
        )

        badge_txt = f"BANDWIDTH: {kbps:.2f} kbps / 2.0 kbps CAP (NO VIDEO DOWNLINK)"
        cv2.putText(
            composite,
            badge_txt,
            (total_w - 560, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 128),
            1,
            cv2.LINE_AA,
        )

        # Place Left & Right Panels
        composite[60:total_h, 0:drone_view_w] = drone_panel
        composite[60:total_h, drone_view_w:total_w] = ground_panel

        # Vertical Divider
        cv2.line(composite, (drone_view_w, 0), (drone_view_w, total_h), (40, 60, 80), 2)

        out.write(composite)

        # Save snapshot around frame 40-50 where multiple detections are tracked
        if frame_idx >= 30 and not snapshot_saved and len(all_boxes) >= 2:
            cv2.imwrite(output_snapshot, composite)
            snapshot_saved = True
            print(f"[ISHAN SHOWCASE] Saved snapshot: {output_snapshot}")

    cap.release()
    out.release()

    if not snapshot_saved:
        cv2.imwrite(output_snapshot, composite)

    print(f"[ISHAN SHOWCASE] Render complete: {output_mp4} ({os.path.getsize(output_mp4)} bytes)")


if __name__ == "__main__":
    run_pipeline()
