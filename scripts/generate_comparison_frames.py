"""Generates side-by-side comparison images:
1. raw_camera_frame.png: Pure unadorned aerial camera video frame.
2. drone_yolo_detection_frame.png: Drone edge node with VisDrone YOLO bounding boxes + flight HUD.
3. ground_tactical_reconstructor_frame.png: Ground tactical reconstructor (Synthetic FOV + Manifest + Mini-Radar).
4. side_by_side_raw_vs_ground.png: Direct side-by-side: Real Raw Video Input vs Ground Tactical Reconstructor.
5. side_by_side_yolo_vs_ground.png: Direct side-by-side: Edge YOLO HUD vs Ground Tactical Reconstructor.
"""

import os
import sys
import cv2
import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from drone_node.detector import Detector
from drone_node.main import draw_hud
from drone_node.scene_codec import SceneCodec, LoRaLinkBudget, SCENE_MAX_KBPS
from drone_node.sim_gps import SimGps
import drone_node.reconstructor as recon
from scripts.ishan_side_by_side import render_ground_reconstructor_panel

DATA_DIR = os.path.join(_REPO_ROOT, "data")
VIDEO_PATH = os.path.join(DATA_DIR, "visdrone_aerial_sample.mp4")
MODEL_PATH = os.path.join(_REPO_ROOT, "yolov8n-visdrone.pt")


def main():
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"Error opening {VIDEO_PATH}")
        sys.exit(1)

    detector = Detector(model_path=MODEL_PATH, conf_thresh=0.25, cooldown_sec=1.0)
    sim_gps = SimGps()
    sub_w, sub_h = 420, 420
    scene_codec = SceneCodec(max_kbps=SCENE_MAX_KBPS, dst_size=(sub_w, sub_h))
    rx_budget = LoRaLinkBudget(max_kbps=SCENE_MAX_KBPS)
    tracker = recon.ObjectTracker(max_len=20, max_dist=70.0)

    # Fast forward to frame ~45 where dense traffic appears
    target_frame = 45
    cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
    ret, raw_frame = cap.read()
    cap.release()

    if not ret:
        print("Error reading frame")
        sys.exit(1)

    print(f"Read target frame {target_frame} ({raw_frame.shape})")

    # 1. Raw camera frame
    raw_clean = raw_frame.copy()

    # 2. Drone YOLO detection
    all_boxes, alert = detector.process_frame(raw_frame)
    drone_lat, drone_lon = sim_gps.position_at_frame(target_frame, fps=20.0)
    packet, kbps = scene_codec.encode_frame(
        detections=all_boxes,
        frame_shape=raw_frame.shape[:2],
        drone_lat=drone_lat,
        drone_lon=drone_lon,
    )

    alert_str = f"ALERT: {alert['label'].upper()} ({alert['conf']:.2f})" if alert else "PATROLLING"
    drone_hud = draw_hud(
        frame=raw_frame,
        mode=0,
        link_up=True,
        buffered_count=0,
        ack_banner="LINK: <= 2.0 kbps (0 B Video Downlink)",
        last_rtt=14.2,
        boxes=all_boxes,
        drone_lat=drone_lat,
        drone_lon=drone_lon,
        last_tx_str=alert_str,
        scene_kbps=kbps,
    )

    # 3. Ground Reconstructor
    decoded = scene_codec.decode_frame(packet, canvas_w=sub_w, canvas_h=sub_h)
    decoded["objects"] = tracker.update(decoded["objects"])
    ground_panel = render_ground_reconstructor_panel(
        pane_w=sub_w,
        pane_h=sub_h,
        scene_data=decoded,
        rx_budget=rx_budget,
        total_packets=target_frame,
        last_rx_time=1.0,
    )

    # Standardize sizes to 800x520
    panel_w, panel_h = 800, 520
    raw_resized = cv2.resize(raw_clean, (panel_w, panel_h))
    drone_resized = cv2.resize(drone_hud, (panel_w, panel_h))
    ground_resized = cv2.resize(ground_panel, (panel_w, panel_h))

    # Add Top Header Banner to Raw View
    cv2.rectangle(raw_resized, (0, 0), (panel_w, 36), (20, 20, 20), -1)
    cv2.putText(
        raw_resized,
        "REAL AERIAL CAMERA INPUT (RAW VISDRONE FOOTAGE)",
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    # 4. Side-by-Side: Real Raw Video Input (Left) vs Ground Reconstructor (Right)
    side_by_side_raw = np.zeros((panel_h + 60, panel_w * 2, 3), dtype=np.uint8)
    cv2.rectangle(side_by_side_raw, (0, 0), (panel_w * 2, 60), (14, 18, 24), -1)
    cv2.putText(
        side_by_side_raw,
        "DIRECT COMPARISON: REAL AERIAL CAMERA INPUT vs GROUND RECONSTRUCTOR TWIN",
        (20, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 220, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        side_by_side_raw,
        "LEFT: Original VisDrone Aerial Camera Stream | RIGHT: Reconstructed from <2 kbps Binary Vectors (Zero Video Transmission)",
        (20, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (160, 200, 220),
        1,
        cv2.LINE_AA,
    )
    side_by_side_raw[60 : panel_h + 60, 0:panel_w] = raw_resized
    side_by_side_raw[60 : panel_h + 60, panel_w : panel_w * 2] = ground_resized
    cv2.line(side_by_side_raw, (panel_w, 0), (panel_w, panel_h + 60), (50, 70, 90), 2)

    # 5. Side-by-Side: Edge YOLO HUD (Left) vs Ground Reconstructor (Right)
    side_by_side_yolo = np.zeros((panel_h + 60, panel_w * 2, 3), dtype=np.uint8)
    cv2.rectangle(side_by_side_yolo, (0, 0), (panel_w * 2, 60), (14, 18, 24), -1)
    cv2.putText(
        side_by_side_yolo,
        "DETECTION COMPARISON: DRONE YOLOv8-VISDRONE vs GROUND RECONSTRUCTOR",
        (20, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 220, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        side_by_side_yolo,
        "LEFT: Airborne Drone Edge YOLO Detections + HUD | RIGHT: Decoded Tactical Twin (<2 kbps Link)",
        (20, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (160, 200, 220),
        1,
        cv2.LINE_AA,
    )
    side_by_side_yolo[60 : panel_h + 60, 0:panel_w] = drone_resized
    side_by_side_yolo[60 : panel_h + 60, panel_w : panel_w * 2] = ground_resized
    cv2.line(side_by_side_yolo, (panel_w, 0), (panel_w, panel_h + 60), (50, 70, 90), 2)

    # Save to data/
    cv2.imwrite(os.path.join(DATA_DIR, "side_by_side_raw_vs_ground.png"), side_by_side_raw)
    cv2.imwrite(os.path.join(DATA_DIR, "side_by_side_yolo_vs_ground.png"), side_by_side_yolo)
    cv2.imwrite(os.path.join(DATA_DIR, "raw_camera_frame.png"), raw_resized)
    cv2.imwrite(os.path.join(DATA_DIR, "drone_yolo_detection_frame.png"), drone_resized)
    cv2.imwrite(os.path.join(DATA_DIR, "ground_tactical_reconstructor_frame.png"), ground_resized)
    print("All comparison frames written to data/ successfully!")


if __name__ == "__main__":
    main()
