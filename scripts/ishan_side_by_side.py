"""ArcDrone Side-by-Side Showcase: 1:1 Ratio Indexed Boxes & 12 FPS Spread-Out Vector Stream.

Implements the exact shareholder specification:
  1. LEFT PANEL (960x540, 16:9): Real Aerial RGB Video + YOLOv8-VisDrone Boxes + Persistent #Index.
  2. RIGHT PANEL (960x540, 16:9): RGB Removed (Blank Tactical Canvas) with boxes drawn at the
     EXACT same size and position ratio (x_r, y_r, w_r, h_r) as the RGB video, indexed by #ID,
     moving via incremental vector updates (dx, dy, dw, dh) with clean vector arrows & trails.
  3. SPREAD-OUT 12 FPS PACKET STREAM: Updates are paced across individual packets at 12 FPS
     (83.3 ms intervals, ~8-10 compact 6B vector updates per packet, ~0.85 kB/s <= 3.0 kB/s cap)
     rather than dumped in a single bulk packet.
  4. BOTTOM INDEX MANIFEST (1920x364): Full lookup table for every #Index showing Class (fixing
     the car->bicycle off-by-one bug), Confidence, exact Ratio (X%, Y%, W%, H%), and Vector Delta.
"""

from __future__ import annotations

import math
import os
import struct
import sys
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from drone_node.detector import Detector, VISDRONE_CLASSES
from drone_node.sim_gps import SimGps

# True 0-indexed VisDrone class colors (BGR) — fixes the off-by-one bug
CLASS_COLORS_BGR: Dict[int, Tuple[int, int, int]] = {
    0: (255, 220, 0),    # pedestrian (cyan)
    1: (255, 180, 0),    # people (sky blue)
    2: (120, 255, 120),  # bicycle (light green)
    3: (0, 200, 255),    # car (amber/gold)
    4: (255, 120, 220),  # van (magenta)
    5: (60, 80, 255),    # truck (red-orange)
    6: (200, 140, 255),  # tricycle (lavender)
    7: (180, 220, 255),  # awning-tricycle
    8: (0, 140, 255),    # bus (deep orange)
    9: (0, 255, 160),    # motor (neon green)
}

# Packet OpCodes for Spread-Out 12 FPS Vector Protocol
MAGIC_VEC = 0xB4
OP_INIT_BOX = 0x01   # [op:1B, id:1B, cls:1B, conf:1B, cx_u16:2B, cy_u16:2B, w_u16:2B, h_u16:2B] = 12B
OP_VEC_DELTA = 0x02  # [op:1B, id:1B, dcx_i16:2B, dcy_i16:2B, dw_i8:1B, dh_i8:1B] = 8B
OP_DROP_BOX = 0x03   # [op:1B, id:1B] = 2B


def quantize_u16(ratio: float) -> int:
    return max(0, min(65535, int(round(ratio * 65535.0))))


def dequantize_u16(val: int) -> float:
    return max(0.0, min(1.0, val / 65535.0))


def quantize_i16(delta_ratio: float) -> int:
    return max(-32767, min(32767, int(round(delta_ratio * 65535.0))))


def dequantize_i16(val: int) -> float:
    return val / 65535.0


def quantize_i8_size(delta_ratio: float) -> int:
    return max(-127, min(127, int(round(delta_ratio * 4096.0))))


def dequantize_i8_size(val: int) -> float:
    return val / 4096.0


def compass_from_vec(vx: float, vy: float) -> str:
    speed = math.hypot(vx, vy)
    if speed < 0.0020:
        return "STATIONARY"
    angle = (math.degrees(math.atan2(vx, -vy)) + 360.0) % 360.0
    dirs = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return f"MOVING {dirs[int(round(angle / 45.0)) % 8]}"


def _box_iou_ratio(
    cx1: float, cy1: float, w1: float, h1: float,
    cx2: float, cy2: float, w2: float, h2: float,
) -> float:
    ax1, ay1, ax2, ay2 = cx1 - w1 * 0.5, cy1 - h1 * 0.5, cx1 + w1 * 0.5, cy1 + h1 * 0.5
    bx1, by1, bx2, by2 = cx2 - w2 * 0.5, cy2 - h2 * 0.5, cx2 + w2 * 0.5, cy2 + h2 * 0.5
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = (w1 * h1) + (w2 * h2) - inter
    return inter / max(1e-9, union)


class DroneIndexedTracker:
    """Tracks YOLO detections on the drone with tight IoU + centroid matching and stable #Index."""

    def __init__(self, max_unseen_frames: int = 10) -> None:
        self.next_id = 1
        self.tracks: Dict[int, Dict[str, Any]] = {}
        self.max_unseen = max_unseen_frames
        self.dropped_ids: List[int] = []

    def update(
        self, detections: List[Dict[str, Any]], frame_w: int, frame_h: int, sim_t: float
    ) -> List[Dict[str, Any]]:
        for trk in self.tracks.values():
            trk["unseen"] += 1

        # Filter out noisy low-confidence flicker and sort highest confidence first
        dets_sorted = [d for d in detections if d["conf"] >= 0.34]
        dets_sorted.sort(key=lambda d: d["conf"], reverse=True)

        matched_tids = set()
        used_dets = set()

        # Pass 1: Strict IoU + Tight Centroid matching for existing tracks
        det_ratios = []
        for det in dets_sorted:
            x1, y1, x2, y2 = det["bbox"]
            bw = max(4.0, float(x2 - x1))
            bh = max(4.0, float(y2 - y1))
            cx = float(x1) + bw * 0.5
            cy = float(y1) + bh * 0.5
            det_ratios.append(
                (
                    max(0.0, min(1.0, cx / frame_w)),
                    max(0.0, min(1.0, cy / frame_h)),
                    max(0.004, min(1.0, bw / frame_w)),
                    max(0.004, min(1.0, bh / frame_h)),
                    int(det["cls"]) if det["cls"] is not None else 3,
                    float(det["conf"]),
                )
            )

        # Build candidate pairs (score, tid, det_idx)
        candidates: List[Tuple[float, int, int]] = []
        for d_idx, (cx_r, cy_r, w_r, h_r, cls_id, conf) in enumerate(det_ratios):
            for tid, trk in self.tracks.items():
                dist = math.hypot(cx_r - trk["cx_r"], cy_r - trk["cy_r"])
                if dist > 0.024:  # Strict 2.4% max jump (~90px in 4K) so lanes never cross-match
                    continue
                iou = _box_iou_ratio(cx_r, cy_r, w_r, h_r, trk["cx_r"], trk["cy_r"], trk["w_r"], trk["h_r"])
                cls_bonus = 0.25 if trk["cls"] == cls_id else 0.0
                score = iou + (0.024 - dist) * 15.0 + cls_bonus
                candidates.append((score, tid, d_idx))

        candidates.sort(key=lambda x: x[0], reverse=True)
        for _, tid, d_idx in candidates:
            if tid in matched_tids or d_idx in used_dets:
                continue
            cx_r, cy_r, w_r, h_r, cls_id, conf = det_ratios[d_idx]
            trk = self.tracks[tid]
            dt = max(0.02, sim_t - trk["last_seen_t"])
            inst_vx = (cx_r - trk["cx_r"]) / dt
            inst_vy = (cy_r - trk["cy_r"]) / dt
            trk["vx_r"] = 0.55 * inst_vx + 0.45 * trk["vx_r"]
            trk["vy_r"] = 0.55 * inst_vy + 0.45 * trk["vy_r"]
            trk["cx_r"] = cx_r
            trk["cy_r"] = cy_r
            trk["w_r"] = 0.7 * w_r + 0.3 * trk["w_r"]
            trk["h_r"] = 0.7 * h_r + 0.3 * trk["h_r"]
            trk["conf"] = conf
            if conf > 0.45:
                trk["cls"] = cls_id
                trk["label"] = VISDRONE_CLASSES[cls_id]
            trk["unseen"] = 0
            trk["hits"] += 1
            trk["last_seen_t"] = sim_t
            matched_tids.add(tid)
            used_dets.add(d_idx)

        # Pass 2: Register genuinely new detections
        for d_idx, (cx_r, cy_r, w_r, h_r, cls_id, conf) in enumerate(det_ratios):
            if d_idx in used_dets:
                continue
            if conf < 0.38:  # Require solid confidence to spawn a new index
                continue
            is_confirmed = sim_t <= 0.05
            pub_id = self.next_id if is_confirmed else 0
            if is_confirmed:
                self.next_id += 1
            raw_key = pub_id if is_confirmed else -(len(self.tracks) + 1000 + d_idx)
            self.tracks[raw_key] = {
                "id": pub_id,
                "cls": cls_id,
                "label": VISDRONE_CLASSES[cls_id],
                "conf": conf,
                "cx_r": cx_r,
                "cy_r": cy_r,
                "w_r": w_r,
                "h_r": h_r,
                "vx_r": 0.0,
                "vy_r": 0.0,
                "unseen": 0,
                "hits": 1,
                "last_seen_t": sim_t,
                # Radio sync state
                "synced": False,
                "sent_cx_r": cx_r,
                "sent_cy_r": cy_r,
                "sent_w_r": w_r,
                "sent_h_r": h_r,
                "last_tx_t": -10.0,
            }

        # Promote tentative tracks that reached hits >= 2 to sequential public #Index
        for raw_key in list(self.tracks.keys()):
            trk = self.tracks[raw_key]
            if trk["id"] == 0 and trk["hits"] >= 2:
                pub_id = self.next_id
                self.next_id += 1
                trk["id"] = pub_id
                del self.tracks[raw_key]
                self.tracks[pub_id] = trk

        # Prune stale tracks
        for tid in list(self.tracks.keys()):
            if self.tracks[tid]["unseen"] > self.max_unseen or (self.tracks[tid]["id"] == 0 and self.tracks[tid]["unseen"] > 1):
                if self.tracks[tid]["synced"] and self.tracks[tid]["id"] > 0:
                    self.dropped_ids.append(self.tracks[tid]["id"])
                del self.tracks[tid]

        return [t for t in self.tracks.values() if t["id"] > 0 and t["unseen"] <= 2]


class SpreadOutVectorTransmitter:
    """Paces and spreads object updates across 12 FPS packets (max N updates per packet)."""

    def __init__(self, target_fps: float = 12.0, max_updates_per_pkt: int = 11) -> None:
        self.target_fps = target_fps
        self.pkt_interval = 1.0 / target_fps
        self.max_updates_per_pkt = max_updates_per_pkt
        self.seq = 0
        self.last_pkt_sim_t = -1.0
        self.tx_window: Deque[Tuple[float, int]] = deque()

    def should_send_packet(self, sim_t: float) -> bool:
        return (sim_t - self.last_pkt_sim_t) >= (self.pkt_interval - 1e-4)

    def build_spread_packet(
        self,
        tracker: DroneIndexedTracker,
        active_tracks: List[Dict[str, Any]],
        drone_lat: float,
        drone_lon: float,
        sim_t: float,
    ) -> Tuple[bytes, List[Tuple[int, str]]]:
        """Builds a single compact packet containing a spread-out slice of updates."""
        self.seq = (self.seq + 1) & 0xFFFF
        self.last_pkt_sim_t = sim_t

        payload_chunks: List[bytes] = []
        updated_records: List[Tuple[int, str]] = []

        # 1. Include up to 2 dropped IDs
        while tracker.dropped_ids and len(payload_chunks) < 2:
            drop_id = tracker.dropped_ids.pop(0)
            payload_chunks.append(struct.pack(">BB", OP_DROP_BOX, drop_id & 0xFF))
            updated_records.append((drop_id, "DROP"))

        # 2. Unsynced new objects (spread out: up to 8 INIT_BOX per packet during bootstrap, leaving room for VEC_DELTA)
        unsynced = [t for t in active_tracks if not t["synced"]]
        unsynced.sort(key=lambda t: (t["id"], -t["conf"]))
        max_init = 8 if len(unsynced) > 6 else 4
        for trk in unsynced[:max_init]:
            if len(payload_chunks) >= self.max_updates_per_pkt:
                break
            chunk = struct.pack(
                ">BBBBHHHH",
                OP_INIT_BOX,
                trk["id"] & 0xFF,
                trk["cls"] & 0xFF,
                int(round(trk["conf"] * 100.0)) & 0xFF,
                quantize_u16(trk["cx_r"]),
                quantize_u16(trk["cy_r"]),
                quantize_u16(trk["w_r"]),
                quantize_u16(trk["h_r"]),
            )
            payload_chunks.append(chunk)
            trk["synced"] = True
            trk["sent_cx_r"] = trk["cx_r"]
            trk["sent_cy_r"] = trk["cy_r"]
            trk["sent_w_r"] = trk["w_r"]
            trk["sent_h_r"] = trk["h_r"]
            trk["last_tx_t"] = sim_t
            updated_records.append((trk["id"], "INIT"))

        # 3. Synced objects needing vector updates (prioritized by vector displacement + age)
        synced = [t for t in active_tracks if t["synced"] and t["last_tx_t"] < sim_t]

        def update_priority(t: Dict[str, Any]) -> float:
            disp = math.hypot(t["cx_r"] - t["sent_cx_r"], t["cy_r"] - t["sent_cy_r"])
            size_diff = abs(t["w_r"] - t["sent_w_r"]) + abs(t["h_r"] - t["sent_h_r"])
            age = sim_t - t["last_tx_t"]
            return (disp * 30.0) + (size_diff * 10.0) + (age * 2.0)

        synced.sort(key=update_priority, reverse=True)

        for trk in synced:
            if len(payload_chunks) >= self.max_updates_per_pkt:
                break
            dcx_r = trk["cx_r"] - trk["sent_cx_r"]
            dcy_r = trk["cy_r"] - trk["sent_cy_r"]
            dw_r = trk["w_r"] - trk["sent_w_r"]
            dh_r = trk["h_r"] - trk["sent_h_r"]

            dcx_i16 = quantize_i16(dcx_r)
            dcy_i16 = quantize_i16(dcy_r)
            dw_i8 = quantize_i8_size(dw_r)
            dh_i8 = quantize_i8_size(dh_r)

            chunk = struct.pack(
                ">BBhhbb",
                OP_VEC_DELTA,
                trk["id"] & 0xFF,
                dcx_i16,
                dcy_i16,
                dw_i8,
                dh_i8,
            )
            payload_chunks.append(chunk)
            trk["sent_cx_r"] += dequantize_i16(dcx_i16)
            trk["sent_cy_r"] += dequantize_i16(dcy_i16)
            trk["sent_w_r"] = max(0.004, trk["sent_w_r"] + dequantize_i8_size(dw_i8))
            trk["sent_h_r"] = max(0.004, trk["sent_h_r"] + dequantize_i8_size(dh_i8))
            trk["last_tx_t"] = sim_t
            updated_records.append((trk["id"], "VEC"))

        header = struct.pack(
            ">BBHff",
            MAGIC_VEC,
            len(payload_chunks) & 0xFF,
            self.seq,
            float(drone_lat),
            float(drone_lon),
        )
        pkt = header + b"".join(payload_chunks)
        self.tx_window.append((sim_t, len(pkt)))
        while self.tx_window and (sim_t - self.tx_window[0][0]) > 1.0:
            self.tx_window.popleft()

        return pkt, updated_records

    def rolling_bps(self, sim_t: float) -> float:
        while self.tx_window and (sim_t - self.tx_window[0][0]) > 1.0:
            self.tx_window.popleft()
        return float(sum(b for _, b in self.tx_window))


class GroundVectorReceiver:
    """Ground station state reconstructor driven strictly by spread-out ratio & vector packets."""

    def __init__(self) -> None:
        self.boxes: Dict[int, Dict[str, Any]] = {}
        self.last_seq = 0
        self.drone_lat = 53.5232
        self.drone_lon = -113.5263
        self.total_pkts = 0
        self.total_bytes = 0
        self.last_pkt_bytes = 0
        self.last_updated_ids: List[Tuple[int, str]] = []

    def receive_packet(self, pkt: bytes, sim_t: float) -> None:
        if len(pkt) < 12 or pkt[0] != MAGIC_VEC:
            return
        _, count, seq, lat, lon = struct.unpack(">BBHff", pkt[:12])
        self.last_seq = seq
        self.drone_lat = lat
        self.drone_lon = lon
        self.total_pkts += 1
        self.total_bytes += len(pkt)
        self.last_pkt_bytes = len(pkt)
        self.last_updated_ids = []

        offset = 12
        for _ in range(count):
            if offset >= len(pkt):
                break
            op = pkt[offset]
            if op == OP_INIT_BOX and offset + 12 <= len(pkt):
                _, tid, cls_id, conf_u8, cx_u16, cy_u16, w_u16, h_u16 = struct.unpack(
                    ">BBBBHHHH", pkt[offset : offset + 12]
                )
                offset += 12
                cx_r = dequantize_u16(cx_u16)
                cy_r = dequantize_u16(cy_u16)
                w_r = dequantize_u16(w_u16)
                h_r = dequantize_u16(h_u16)
                self.boxes[tid] = {
                    "id": tid,
                    "cls": cls_id,
                    "label": VISDRONE_CLASSES[cls_id] if cls_id < len(VISDRONE_CLASSES) else f"cls_{cls_id}",
                    "conf": conf_u8 / 100.0,
                    "cx_r": cx_r,
                    "cy_r": cy_r,
                    "w_r": w_r,
                    "h_r": h_r,
                    "dx_r": 0.0,
                    "dy_r": 0.0,
                    "vx_r": 0.0,
                    "vy_r": 0.0,
                    "last_rx_t": sim_t,
                    "last_op": "INIT_BOX",
                    "trail": deque([(cx_r, cy_r)], maxlen=18),
                }
                self.last_updated_ids.append((tid, "INIT"))

            elif op == OP_VEC_DELTA and offset + 8 <= len(pkt):
                _, tid, dcx_i16, dcy_i16, dw_i8, dh_i8 = struct.unpack(
                    ">BBhhbb", pkt[offset : offset + 8]
                )
                offset += 8
                dcx_r = dequantize_i16(dcx_i16)
                dcy_r = dequantize_i16(dcy_i16)
                dw_r = dequantize_i8_size(dw_i8)
                dh_r = dequantize_i8_size(dh_i8)

                if tid in self.boxes:
                    b = self.boxes[tid]
                    dt = max(0.04, sim_t - b["last_rx_t"])
                    b["cx_r"] = max(0.0, min(1.0, b["cx_r"] + dcx_r))
                    b["cy_r"] = max(0.0, min(1.0, b["cy_r"] + dcy_r))
                    b["w_r"] = max(0.004, min(1.0, b["w_r"] + dw_r))
                    b["h_r"] = max(0.004, min(1.0, b["h_r"] + dh_r))
                    b["dx_r"] = dcx_r
                    b["dy_r"] = dcy_r
                    b["vx_r"] = 0.6 * (dcx_r / dt) + 0.4 * b["vx_r"]
                    b["vy_r"] = 0.6 * (dcy_r / dt) + 0.4 * b["vy_r"]
                    b["last_rx_t"] = sim_t
                    b["last_op"] = "VEC_DELTA"
                    if not b["trail"] or math.hypot(b["cx_r"] - b["trail"][-1][0], b["cy_r"] - b["trail"][-1][1]) < 0.03:
                        b["trail"].append((b["cx_r"], b["cy_r"]))
                    else:
                        b["trail"].clear()
                        b["trail"].append((b["cx_r"], b["cy_r"]))
                    self.last_updated_ids.append((tid, "VEC"))

            elif op == OP_DROP_BOX and offset + 2 <= len(pkt):
                _, tid = struct.unpack(">BB", pkt[offset : offset + 2])
                offset += 2
                self.boxes.pop(tid, None)
                self.last_updated_ids.append((tid, "DROP"))
            else:
                break

        # Expire any stale ground boxes not refreshed in >2.0s
        for tid in list(self.boxes.keys()):
            if (sim_t - self.boxes[tid]["last_rx_t"]) > 2.0:
                del self.boxes[tid]


def draw_pill_label(
    canvas: np.ndarray,
    text: str,
    x: int,
    y: int,
    color: Tuple[int, int, int],
    scale: float = 0.42,
    highlight: bool = False,
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, scale, 1)
    x1 = max(2, x)
    y1 = max(36, y - th - 6)
    x2 = min(canvas.shape[1] - 2, x1 + tw + 8)
    y2 = min(canvas.shape[0] - 2, y1 + th + 6)
    bg = (35, 55, 35) if highlight else (12, 14, 18)
    cv2.rectangle(canvas, (x1, y1), (x2, y2), bg, -1)
    cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 1)
    cv2.putText(canvas, text, (x1 + 4, y2 - 4), font, scale, (255, 255, 255) if highlight else color, 1, cv2.LINE_AA)


def render_left_rgb_canvas(
    raw_frame: np.ndarray,
    active_tracks: List[Dict[str, Any]],
    canvas_w: int,
    canvas_h: int,
) -> np.ndarray:
    """Renders the Left RGB Camera Panel (960x540) with indexed bounding boxes."""
    panel = cv2.resize(raw_frame, (canvas_w, canvas_h), interpolation=cv2.INTER_AREA)

    for trk in active_tracks:
        tid = trk["id"]
        color = CLASS_COLORS_BGR.get(trk["cls"], (0, 255, 255))
        cx = int(round(trk["cx_r"] * canvas_w))
        cy = int(round(trk["cy_r"] * canvas_h))
        bw = max(8, int(round(trk["w_r"] * canvas_w)))
        bh = max(8, int(round(trk["h_r"] * canvas_h)))
        x1, y1 = cx - bw // 2, cy - bh // 2
        x2, y2 = cx + bw // 2, cy + bh // 2

        cv2.rectangle(panel, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        draw_pill_label(panel, f"#{tid}", x1, y1, color, scale=0.40)

    # Top overlay bar on Left Canvas
    cv2.rectangle(panel, (0, 0), (canvas_w, 34), (16, 20, 26), -1)
    cv2.putText(
        panel,
        f"DRONE SIDE: REAL RGB VIDEO + YOLOv8-VISDRONE ({len(active_tracks)} INDEXED OBJECTS)",
        (12, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 230, 255),
        1,
        cv2.LINE_AA,
    )
    return panel


def render_right_blank_ratio_canvas(
    receiver: GroundVectorReceiver,
    canvas_w: int,
    canvas_h: int,
    sim_t: float,
) -> np.ndarray:
    """Renders the Right Ground Canvas (960x540) with RGB removed, preserving exact 1:1 box ratios."""
    panel = np.full((canvas_h, canvas_w, 3), (12, 16, 20), dtype=np.uint8)

    # 10% proportional coordinate grid (0% to 100% X and Y ratio lines)
    for i in range(1, 10):
        gx = int(round((i / 10.0) * canvas_w))
        gy = int(round((i / 10.0) * canvas_h))
        col = (26, 34, 42) if i != 5 else (40, 54, 66)
        cv2.line(panel, (gx, 34), (gx, canvas_h), col, 1)
        cv2.line(panel, (0, gy), (canvas_w, gy), col, 1)
        cv2.putText(
            panel,
            f"{i*10}%",
            (gx + 3, canvas_h - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.33,
            (70, 90, 105),
            1,
        )
        if gy > 45:
            cv2.putText(
                panel,
                f"{i*10}%",
                (6, gy - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.33,
                (70, 90, 105),
                1,
            )

    updated_set = {tid for tid, _ in receiver.last_updated_ids}

    for tid, b in sorted(receiver.boxes.items()):
        color = CLASS_COLORS_BGR.get(b["cls"], (0, 255, 255))
        cx = int(round(b["cx_r"] * canvas_w))
        cy = int(round(b["cy_r"] * canvas_h))
        bw = max(8, int(round(b["w_r"] * canvas_w)))
        bh = max(8, int(round(b["h_r"] * canvas_h)))
        x1, y1 = cx - bw // 2, cy - bh // 2
        x2, y2 = cx + bw // 2, cy + bh // 2

        # Draw smooth historical movement trail in ratio space
        trail_pts = [
            (int(round(tx * canvas_w)), int(round(ty * canvas_h)))
            for tx, ty in b["trail"]
        ]
        for idx in range(1, len(trail_pts)):
            cv2.line(panel, trail_pts[idx - 1], trail_pts[idx], color, 2, cv2.LINE_AA)

        # Draw exact ratio bounding box
        is_fresh = (tid in updated_set)
        thickness = 2 if is_fresh else 1
        cv2.rectangle(panel, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
        if is_fresh:
            cv2.rectangle(
                panel,
                (x1 - 2, y1 - 2),
                (x2 + 2, y2 + 2),
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        # Center dot
        cv2.circle(panel, (cx, cy), 3, (255, 255, 255), -1, cv2.LINE_AA)

        # Clean bounded velocity vector arrow
        vx_px = max(-36.0, min(36.0, b["vx_r"] * canvas_w * 0.40))
        vy_px = max(-36.0, min(36.0, b["vy_r"] * canvas_h * 0.40))
        if math.hypot(vx_px, vy_px) >= 3.5:
            end_x = int(max(4, min(canvas_w - 4, cx + vx_px)))
            end_y = int(max(38, min(canvas_h - 4, cy + vy_px)))
            cv2.arrowedLine(
                panel,
                (cx, cy),
                (end_x, end_y),
                (0, 255, 255),
                2,
                cv2.LINE_AA,
                tipLength=0.35,
            )

        draw_pill_label(panel, f"#{tid}", x1, y1, color, scale=0.40, highlight=is_fresh)

    # Top overlay bar on Right Canvas
    cv2.rectangle(panel, (0, 0), (canvas_w, 34), (16, 20, 26), -1)
    upd_str = ", ".join(f"#{i}({op})" for i, op in receiver.last_updated_ids[:7]) or "none"
    cv2.putText(
        panel,
        f"GROUND RECEIVER: NO RGB (1:1 RATIO BOXES + VECTORS) | PKT #{receiver.last_seq} UPD: [{upd_str}]",
        (12, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.44,
        (120, 255, 150),
        1,
        cv2.LINE_AA,
    )
    return panel


def render_bottom_index_manifest(
    receiver: GroundVectorReceiver,
    width: int,
    height: int,
    sim_t: float,
) -> np.ndarray:
    """Renders the full-width bottom lookup table for all indexed bounding boxes (2 columns)."""
    panel = np.full((height, width, 3), (15, 18, 24), dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (width, 32), (24, 30, 40), -1)
    cv2.putText(
        panel,
        "INDEXED OBJECT MANIFEST & VECTOR UPDATES (KEYED BY #INDEX — 16-BIT RATIO PRECISION, SPREAD-OUT AT 12 FPS)",
        (16, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (0, 220, 255),
        1,
        cv2.LINE_AA,
    )

    col_w = width // 2
    headers = "INDEX   CLASS           CONF    POS RATIO (X%, Y%)     SIZE RATIO (W%, H%)    VEC DELTA (dX%, dY%)      MOTION       LAST PKT"
    for c in range(2):
        x0 = c * col_w + 14
        cv2.putText(
            panel,
            headers,
            (x0, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.39,
            (170, 185, 200),
            1,
            cv2.LINE_AA,
        )
        cv2.line(panel, (x0, 60), (x0 + col_w - 28, 60), (50, 65, 80), 1)

    cv2.line(panel, (col_w, 32), (col_w, height), (45, 60, 75), 1)

    boxes_sorted = sorted(receiver.boxes.values(), key=lambda b: b["id"])
    max_rows_per_col = (height - 72) // 22
    max_total = max_rows_per_col * 2

    updated_set = {tid for tid, _ in receiver.last_updated_ids}

    for idx, b in enumerate(boxes_sorted[:max_total]):
        col_idx = idx // max_rows_per_col
        row_idx = idx % max_rows_per_col
        x0 = col_idx * col_w + 14
        y0 = 80 + row_idx * 22

        tid = b["id"]
        color = CLASS_COLORS_BGR.get(b["cls"], (0, 255, 255))
        is_fresh = (tid in updated_set)

        if is_fresh:
            cv2.rectangle(
                panel,
                (x0 - 4, y0 - 15),
                (x0 + col_w - 24, y0 + 5),
                (28, 48, 36),
                -1,
            )

        x_pct = b["cx_r"] * 100.0
        y_pct = b["cy_r"] * 100.0
        w_pct = b["w_r"] * 100.0
        h_pct = b["h_r"] * 100.0
        dx_pct = b["dx_r"] * 100.0
        dy_pct = b["dy_r"] * 100.0
        direction = compass_from_vec(b["vx_r"], b["vy_r"])
        age_ms = int(round((sim_t - b["last_rx_t"]) * 1000.0))
        state_tag = f"{b['last_op']} ({age_ms}ms)"

        row_text = (
            f"#{tid:<5d}  {b['label'].upper():<14s}  {int(b['conf']*100):>3d}%    "
            f"({x_pct:>5.1f}%, {y_pct:>5.1f}%)       "
            f"({w_pct:>4.1f}% x {h_pct:>4.1f}%)       "
            f"({dx_pct:>+5.2f}%, {dy_pct:>+5.2f}%)     "
            f"{direction:<11s}  {state_tag}"
        )
        cv2.putText(
            panel,
            row_text,
            (x0, y0),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.39,
            (255, 255, 255) if is_fresh else color,
            1,
            cv2.LINE_AA,
        )

    return panel


def run_pipeline(
    video_path: str = os.path.join(_REPO_ROOT, "data", "visdrone_aerial_sample.mp4"),
    model_path: str = os.path.join(_REPO_ROOT, "yolov8n-visdrone.pt"),
    output_mp4: str = os.path.join(_REPO_ROOT, "data", "ishan_visdrone_side_by_side.mp4"),
    output_snapshot: str = os.path.join(_REPO_ROOT, "data", "ishan_visdrone_side_by_side.png"),
    num_frames: int = 150,
) -> None:
    print(f"[RATIO-VEC SHOWCASE] Opening video: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {video_path}")

    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[RATIO-VEC SHOWCASE] Source: {src_w}x{src_h} @ {video_fps:.2f} FPS")

    print(f"[RATIO-VEC SHOWCASE] Loading VisDrone weights: {model_path}")
    detector = Detector(model_path=model_path, conf_thresh=0.30, cooldown_sec=1.0)
    sim_gps = SimGps()

    drone_tracker = DroneIndexedTracker(max_unseen_frames=10)
    transmitter = SpreadOutVectorTransmitter(target_fps=12.0, max_updates_per_pkt=9)
    receiver = GroundVectorReceiver()

    # Exact 1:1 16:9 Canvas dimensions: 960x540 Left + 960x540 Right = 1920x540
    canvas_w, canvas_h = 960, 540
    header_h = 56
    manifest_h = 364
    total_w = canvas_w * 2  # 1920
    total_h = header_h + canvas_h + manifest_h  # 960

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_mp4, fourcc, 24.0, (total_w, total_h))

    frame_idx = 0
    snapshot_saved = False
    peak_bps = 0.0

    print(f"[RATIO-VEC SHOWCASE] Rendering {num_frames} frames with 12 FPS spread-out vector packets...")
    while frame_idx < num_frames:
        ret, raw_frame = cap.read()
        if not ret:
            break

        frame_idx += 1
        sim_t = frame_idx / 24.0  # Deterministic simulation clock (24 FPS video timeline)

        # 1. Run YOLOv8-VisDrone at imgsz=1280 so small highway cars in 4K are cleanly detected
        results = detector.model.predict(
            source=raw_frame, imgsz=1280, conf=0.30, iou=0.45, verbose=False
        )
        raw_boxes: List[Dict[str, Any]] = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                conf = float(box.conf[0])
                raw_cls = int(box.cls[0])
                if 0 <= raw_cls < len(VISDRONE_CLASSES):
                    xyxy = [int(v) for v in box.xyxy[0].tolist()]
                    raw_boxes.append(
                        {
                            "bbox": (xyxy[0], xyxy[1], xyxy[2], xyxy[3]),
                            "conf": conf,
                            "cls": raw_cls,
                            "label": VISDRONE_CLASSES[raw_cls],
                        }
                    )

        # 2. Update persistent #Index tracker on drone
        active_tracks = drone_tracker.update(raw_boxes, src_w, src_h, sim_t)
        drone_lat, drone_lon = sim_gps.position_at_frame(frame_idx, fps=24.0)

        # 3. Spread-out 12 FPS packet transmission (1 packet every 83.3 ms = every 2 frames at 24 FPS)
        if transmitter.should_send_packet(sim_t):
            pkt, _ = transmitter.build_spread_packet(
                drone_tracker, active_tracks, drone_lat, drone_lon, sim_t
            )
            receiver.receive_packet(pkt, sim_t)

        bps = transmitter.rolling_bps(sim_t)
        peak_bps = max(peak_bps, bps)

        # 4. Render Left (RGB + Indexed Boxes) & Right (Blank Canvas + 1:1 Ratio Boxes & Vectors)
        left_canvas = render_left_rgb_canvas(raw_frame, active_tracks, canvas_w, canvas_h)
        right_canvas = render_right_blank_ratio_canvas(receiver, canvas_w, canvas_h, sim_t)
        bottom_manifest = render_bottom_index_manifest(receiver, total_w, manifest_h, sim_t)

        # 5. Top Telemetry Header Bar
        header = np.full((header_h, total_w, 3), (10, 14, 18), dtype=np.uint8)
        cv2.putText(
            header,
            "ARCDRONE 1:1 RATIO SIDE-BY-SIDE: DRONE RGB CAMERA vs GROUND INDEXED VECTOR RECEIVER (NO RGB)",
            (18, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 225, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            header,
            f"SPREAD-OUT LINK: 12.0 FPS ({receiver.total_pkts} pkts, {receiver.last_pkt_bytes} B/pkt)  |  "
            f"WIRE RATE: {bps/1000.0:.2f} kB/s (CAP <= 3.00 kB/s)  |  "
            f"ACTIVE INDICES: {len(receiver.boxes)}  |  GPS: ({drone_lat:.4f}, {drone_lon:.4f})",
            (18, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (140, 255, 170),
            1,
            cv2.LINE_AA,
        )

        # Combine into 1920x960 frame
        mid_row = np.hstack([left_canvas, right_canvas])
        cv2.line(mid_row, (canvas_w, 0), (canvas_w, canvas_h), (80, 110, 130), 2)
        composite = np.vstack([header, mid_row, bottom_manifest])

        out.write(composite)

        if frame_idx == 60 and not snapshot_saved:
            cv2.imwrite(output_snapshot, composite)
            snapshot_saved = True
            print(f"[RATIO-VEC SHOWCASE] Saved high-res snapshot at frame 60: {output_snapshot}")

    cap.release()
    out.release()

    if not snapshot_saved:
        cv2.imwrite(output_snapshot, composite)

    print(
        f"[RATIO-VEC SHOWCASE] Complete! Video: {output_mp4} ({os.path.getsize(output_mp4)} bytes), "
        f"Total Pkts: {receiver.total_pkts} (12.0 FPS spread-out), Peak Rate: {peak_bps:.0f} B/s (<= 3000 B/s)"
    )


if __name__ == "__main__":
    run_pipeline()
