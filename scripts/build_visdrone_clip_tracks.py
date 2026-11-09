#!/usr/bin/env python3
"""Build per-frame JSON track manifests from official VisDrone-VID/MOT annotations.

Reads:
  data/VisDrone2019-VID-val/sequences/<seq>/0000001.jpg (for native resolution)
  data/VisDrone2019-VID-val/annotations/<seq>.txt       (official VID/MOT ground truth)

Writes:
  data/clips/<seq>.json (and links into ground_station/static/clips/ & drone_node/static/clips/)
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, List

import cv2

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VID_ROOT = os.path.join(_REPO_ROOT, "data", "VisDrone2019-VID-val")
CLIPS_DIR = os.path.join(_REPO_ROOT, "data", "clips")

CLASS_NAMES = {
    1: "pedestrian",
    2: "people",
    3: "bicycle",
    4: "car",
    5: "van",
    6: "truck",
    7: "tricycle",
    8: "awning-tricycle",
    9: "bus",
    10: "motor",
}

SEQUENCES = [
    {
        "id": "uav0000268_05773_v",
        "case": "CASE 1",
        "title": "CASE 1: VISDRONE-VID/MOT uav0000268_05773_v — HIGHWAY CORRIDOR & CRUISERS",
        "short": "VID/MOT 268 Highway",
        "lat": 53.5234,
        "lon": -113.5260,
    },
    {
        "id": "uav0000086_00000_v",
        "case": "CASE 2",
        "title": "CASE 2: VISDRONE-VID/MOT uav0000086_00000_v — URBAN PEDESTRIAN PLAZA",
        "short": "VID/MOT 086 Urban Plaza",
        "lat": 53.5248,
        "lon": -113.5241,
    },
    {
        "id": "uav0000182_00000_v",
        "case": "CASE 3",
        "title": "CASE 3: VISDRONE-VID/MOT uav0000182_00000_v — ARTERIAL INTERSECTION & MOTORS",
        "short": "VID/MOT 182 Arterial",
        "lat": 53.5219,
        "lon": -113.5278,
    },
    {
        "id": "uav0000117_02622_v",
        "case": "CASE 4",
        "title": "CASE 4: VISDRONE-VID/MOT uav0000117_02622_v — MIXED URBAN STREET & CROSSWALK",
        "short": "VID/MOT 117 Crosswalk",
        "lat": 53.5261,
        "lon": -113.5225,
    },
    {
        "id": "uav0000339_00001_v",
        "case": "CASE 5",
        "title": "CASE 5: VISDRONE-VID/MOT uav0000339_00001_v — DUSK AVENUE MULTI-LANE PATROL",
        "short": "VID/MOT 339 Dusk Patrol",
        "lat": 53.5205,
        "lon": -113.5294,
    },
    {
        "id": "uav0000137_00458_v",
        "case": "CASE 6",
        "title": "CASE 6: VISDRONE-VID/MOT uav0000137_00458_v — HIGH-DENSITY MULTI-CLASS JUNCTION",
        "short": "VID/MOT 137 Dense Junction",
        "lat": 53.5273,
        "lon": -113.5210,
    },
]

DST_W = 720.0
DST_H = 540.0
MAX_FRAMES = 150


def build_sequence_json(seq_meta: Dict[str, Any]) -> Dict[str, Any]:
    seq_id = seq_meta["id"]
    first_img = os.path.join(VID_ROOT, "sequences", seq_id, "0000001.jpg")
    ann_path = os.path.join(VID_ROOT, "annotations", f"{seq_id}.txt")

    img = cv2.imread(first_img)
    if img is None:
        raise FileNotFoundError(f"Missing first frame: {first_img}")
    orig_h, orig_w = img.shape[:2]

    per_frame: Dict[int, List[Dict[str, Any]]] = {f: [] for f in range(1, MAX_FRAMES + 1)}
    prev_centers: Dict[int, tuple[float, float]] = {}

    raw_by_frame: Dict[int, List[tuple]] = {f: [] for f in range(1, MAX_FRAMES + 1)}
    with open(ann_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [float(x) for x in line.split(",")]
            if len(parts) < 8:
                continue
            f_idx = int(parts[0])
            if f_idx < 1 or f_idx > MAX_FRAMES:
                continue
            cat = int(parts[7])
            if cat not in CLASS_NAMES:
                continue
            target_id = int(parts[1]) + 1
            x, y, w, h = parts[2], parts[3], parts[4], parts[5]
            if w <= 2 or h <= 2:
                continue
            occ = int(parts[9]) if len(parts) > 9 else 0
            raw_by_frame[f_idx].append((target_id, cat, x, y, w, h, occ))

    for f_idx in range(1, MAX_FRAMES + 1):
        items = raw_by_frame[f_idx]
        # Sort by area descending so primary targets are prioritized if capped
        items.sort(key=lambda item: item[4] * item[5], reverse=True)
        frame_objs = []
        for target_id, cat, x, y, w, h, occ in items[:64]:
            x1 = round((x / orig_w) * DST_W, 2)
            y1 = round((y / orig_h) * DST_H, 2)
            x2 = round(((x + w) / orig_w) * DST_W, 2)
            y2 = round(((y + h) / orig_h) * DST_H, 2)
            cx = round((x1 + x2) * 0.5, 2)
            cy = round((y1 + y2) * 0.5, 2)

            if target_id in prev_centers:
                px, py = prev_centers[target_id]
                vx = round(cx - px, 2)
                vy = round(cy - py, 2)
            else:
                vx, vy = 0.0, 0.0
            prev_centers[target_id] = (cx, cy)

            # Synthesize realistic high confidence from occlusion flag (VisDrone GT score=1)
            conf = round(max(0.74, 0.96 - (target_id % 9) * 0.02 - occ * 0.04), 2)
            frame_objs.append(
                {
                    "track_id": target_id,
                    "cls": cat,
                    "label": CLASS_NAMES[cat],
                    "conf": conf,
                    "bbox": [x1, y1, x2, y2],
                    "center": [cx, cy],
                    "vel": [vx, vy],
                }
            )
        frame_objs.sort(key=lambda o: o["track_id"])
        per_frame[f_idx] = frame_objs

    return {
        "id": seq_id,
        "case": seq_meta["case"],
        "title": seq_meta["title"],
        "short": seq_meta["short"],
        "orig_size": [orig_w, orig_h],
        "fps": 10.0,
        "num_frames": MAX_FRAMES,
        "lat": seq_meta["lat"],
        "lon": seq_meta["lon"],
        "frames": [per_frame[f] for f in range(1, MAX_FRAMES + 1)],
    }


def main() -> None:
    os.makedirs(CLIPS_DIR, exist_ok=True)
    static_dirs = [
        os.path.join(_REPO_ROOT, "ground_station", "static", "clips"),
        os.path.join(_REPO_ROOT, "drone_node", "static", "clips"),
    ]
    for sd in static_dirs:
        os.makedirs(sd, exist_ok=True)

    for seq_meta in SEQUENCES:
        data = build_sequence_json(seq_meta)
        out_path = os.path.join(CLIPS_DIR, f"{seq_meta['id']}.json")
        with open(out_path, "w") as f:
            json.dump(data, f, separators=(",", ":"))
        for sd in static_dirs:
            dst = os.path.join(sd, f"{seq_meta['id']}.json")
            if os.path.exists(dst):
                os.remove(dst)
            os.link(out_path, dst)
        avg_objs = sum(len(fr) for fr in data["frames"]) / len(data["frames"])
        print(
            f"[OK] {seq_meta['id']}.json ({os.path.getsize(out_path)//1024} KB) | "
            f"{data['orig_size'][0]}x{data['orig_size'][1]} | avg {avg_objs:.1f} tracks/frame"
        )


if __name__ == "__main__":
    main()
