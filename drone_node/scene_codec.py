"""SceneCodec v2 (0xAE): keyframes + deltas under 2.0 kbps. Legacy v1 (0xAD) decode supported."""
from __future__ import annotations

import math
import struct
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

MAGIC_V1 = 0xAD
MAGIC_V2 = 0xAE
MAGIC_BYTE = MAGIC_V1  # legacy alias

MSG_KEYFRAME = 0x2
MSG_DELTA = 0x3
MSG_COMPACT_KF = 0x4
MSG_COMPACT_DELTA = 0x5
MSG_COMPACT_STATE = 0x6

LINK_MAX_KBPS = 2.0
RESERVE_KBPS = 0.0
SCENE_MAX_KBPS = LINK_MAX_KBPS
SCENE_HZ_DEFAULT = 12.0
MAX_BYTES_PER_SEC = int(SCENE_MAX_KBPS * 1000 / 8)  # 250 B/s
MAX_LORA_OBJECTS = 6
MAX_ACTIVE_TRACKS = 14
MAX_TRACK_ID = 64

# 3-bit width/height quantization lookup tables in 0..63 space
W3_TABLE: Tuple[int, ...] = (4, 6, 9, 12, 16, 21, 28, 38)
H3_TABLE: Tuple[int, ...] = (4, 6, 9, 12, 16, 21, 28, 38)

# Grid-aware coasting parameters optimized for LoRa stability (no pop-in/pop-out)
GRID_CELLS = 16
EDGE_RINGS = 2
EDGE_UNSEEN_DROP = 10      # ~1.0s at outer border before dropping off-screen contact
INTERIOR_UNSEEN_DROP = 40  # ~3.5s in center/interior so objects never pop out on brief YOLO dips

# v1 wire labels (reconstructor palette)
LABEL_TO_CLS = {
    "pedestrian": 1,
    "people": 2,
    "bicycle": 3,
    "car": 4,
    "van": 5,
    "truck": 6,
    "tricycle": 7,
    "awning-tricycle": 8,
    "bus": 9,
    "motor": 10,
}
CLS_TO_LABEL = {v: k for k, v in LABEL_TO_CLS.items()}
PROTOCOL_IDX_TO_WIRE = {i: i + 1 for i in range(10)}

VISDRONE_CLASSES: Dict[int, str] = {
    0: "pedestrian",
    1: "people",
    2: "bicycle",
    3: "car",
    4: "van",
    5: "truck",
    6: "tricycle",
    7: "awning-tricycle",
    8: "bus",
    9: "motor",
}

CLASS_THREAT_WEIGHT: Dict[int, float] = {
    5: 1.5,
    8: 1.5,
    3: 1.3,
    4: 1.3,
    9: 1.2,
    0: 1.1,
    1: 1.1,
    2: 1.0,
    6: 1.0,
    7: 1.0,
}

SECTORS: Tuple[str, ...] = (
    "TOP-LEFT",
    "TOP-CENTER",
    "TOP-RIGHT",
    "MID-LEFT",
    "CENTER",
    "MID-RIGHT",
    "BOTTOM-LEFT",
    "BOTTOM-CENTER",
    "BOTTOM-RIGHT",
)

V1_HEADER_FMT = ">BHffB"
V1_OBJ_FMT = ">BBBBBB"
V1_HEADER_SIZE = struct.calcsize(V1_HEADER_FMT)
V1_OBJ_SIZE = struct.calcsize(V1_OBJ_FMT)


def _clamp(val: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, val))


def _pack_i6(val: int) -> int:
    val = _clamp(val, -32, 31)
    if val < 0:
        val = 64 + val
    return val & 0x3F


def _unpack_i6(raw: int) -> int:
    return raw - 64 if raw >= 32 else raw


def _sector_from_xy(cx: float, cy: float, w: int, h: int) -> str:
    col = _clamp(int((cx / max(1, w)) * 3), 0, 2)
    row = _clamp(int((cy / max(1, h)) * 3), 0, 2)
    return SECTORS[row * 3 + col]


def _motion_str_from_vel(dx: float, dy: float) -> str:
    if math.hypot(dx, dy) < 1.15:
        return "STATIONARY"
    deg = (math.degrees(math.atan2(dx, -dy)) + 360.0) % 360.0
    dirs = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return "MOVING %s" % dirs[int(round(deg / 45.0)) % 8]


def _normalize_cls(raw_cls: int) -> int:
    """protocol.packets index 0-9; accept wire 1-10 from older paths."""
    if raw_cls is None:
        return 0
    raw_cls = int(raw_cls)
    if 1 <= raw_cls <= 10:
        return raw_cls - 1
    return _clamp(raw_cls, 0, 9)


class LoRaLinkBudget:
    """Rolling bitrate meter (reconstructor header / HUD)."""

    def __init__(self, max_kbps: float = SCENE_MAX_KBPS) -> None:
        self.max_bytes_per_sec = max_kbps * 1000.0 / 8.0
        self.history: Deque[Tuple[float, int]] = deque()

    def record_tx(self, num_bytes: int) -> None:
        self.history.append((time.time(), num_bytes))

    def current_kbps(self) -> float:
        now = time.time()
        while self.history and (now - self.history[0][0]) > 1.0:
            self.history.popleft()
        used = sum(b for _, b in self.history)
        return (used * 8.0) / 1000.0


class _EncoderTrack:
    __slots__ = (
        "track_id",
        "cls",
        "conf",
        "cx_q",
        "cy_q",
        "w6",
        "h6",
        "vx_q",
        "vy_q",
        "sent_cx_q",
        "sent_cy_q",
        "synced_on_wire",
        "confirmed",
        "hits",
        "ticks_unseen",
        "ticks_since_sent",
        "speed_q",
    )

    def __init__(
        self,
        track_id: int,
        cls: int,
        conf: float,
        cx_q: int,
        cy_q: int,
        w6: int,
        h6: int,
        confirmed: bool = False,
    ) -> None:
        self.track_id = track_id
        self.cls = cls
        self.conf = conf
        self.cx_q = cx_q
        self.cy_q = cy_q
        self.w6 = w6
        self.h6 = h6
        self.vx_q = 0.0
        self.vy_q = 0.0
        self.sent_cx_q = cx_q
        self.sent_cy_q = cy_q
        self.synced_on_wire = False
        self.confirmed = confirmed
        self.hits = 1
        self.ticks_unseen = 0
        self.ticks_since_sent = 99
        self.speed_q = 0.0


class _DecoderTrack:
    __slots__ = (
        "track_id",
        "cls",
        "conf",
        "cx_q",
        "cy_q",
        "w6",
        "h6",
        "trail",
        "vel",
        "last_keyframe_ts",
        "last_update_ts",
    )

    def __init__(
        self,
        track_id: int,
        cls: int,
        conf: float,
        cx_q: int,
        cy_q: int,
        w6: int,
        h6: int,
        now: float,
    ) -> None:
        self.track_id = track_id
        self.cls = cls
        self.conf = conf
        self.cx_q = cx_q
        self.cy_q = cy_q
        self.w6 = w6
        self.h6 = h6
        self.trail: Deque[List[int]] = deque(maxlen=25)
        self.vel: List[float] = [0.0, 0.0]
        self.last_keyframe_ts = now
        self.last_update_ts = now


def _quantize_size_idx(val6: int, table: Tuple[int, ...]) -> int:
    best_i = 0
    best_d = 999
    for idx, tval in enumerate(table):
        d = abs(val6 - tval)
        if d < best_d:
            best_d = d
            best_i = idx
    return best_i


class SceneCodec:
    """Stateful v2 encoder/decoder optimized for LoRa hardware stability and minimal lag."""

    def __init__(
        self,
        src_size: Tuple[int, int] = (640, 480),
        dst_size: Tuple[int, int] = (720, 540),
        max_bytes_per_sec: int = MAX_BYTES_PER_SEC,
        keyframe_interval_ticks: int = 6,
        top_k_pinned: int = 4,
        max_kbps: Optional[float] = None,
    ) -> None:
        if max_kbps is not None:
            max_bytes_per_sec = int(max_kbps * 1000.0 / 8.0)
        self.src_w, self.src_h = src_size
        self.dst_w, self.dst_h = dst_size
        self.max_bytes_per_sec = max_bytes_per_sec
        self.keyframe_interval = keyframe_interval_ticks
        self.top_k_pinned = top_k_pinned

        self._seq = 0
        self._next_tid = 0
        self._ticks_since_kf = keyframe_interval_ticks
        self._byte_window: Deque[Tuple[float, int]] = deque()
        self._enc_tracks: Dict[int, _EncoderTrack] = {}
        self._drop_queue: Deque[Tuple[int, int]] = deque()  # (tid, remaining_sends)
        self._pending_reset = False

        self._dec_lat = 53.5232
        self._dec_lon = -113.5263
        self._dec_tracks: Dict[int, _DecoderTrack] = {}

    def reset_for_rewind(self) -> None:
        """Called when the video file rewinds to frame 0 so stale end-of-clip tracks clear."""
        self._enc_tracks.clear()
        self._drop_queue.clear()
        self._seq = 0
        self._pending_reset = True

    def current_kbps(self) -> float:
        now = time.monotonic()
        while self._byte_window and (now - self._byte_window[0][0]) > 1.0:
            self._byte_window.popleft()
        used = sum(b for _, b in self._byte_window)
        return (used * 8.0) / 1000.0

    def _available_budget(self, now: float) -> int:
        while self._byte_window and (now - self._byte_window[0][0]) > 1.0:
            self._byte_window.popleft()
        used = sum(b for _, b in self._byte_window)
        return max(4, self.max_bytes_per_sec - used)

    @staticmethod
    def _edge_distance_q(cx_q: int, cy_q: int) -> int:
        cx_g = min(GRID_CELLS - 1, max(0, (cx_q * GRID_CELLS) // 256))
        cy_g = min(GRID_CELLS - 1, max(0, (cy_q * GRID_CELLS) // 256))
        return min(cx_g, GRID_CELLS - 1 - cx_g, cy_g, GRID_CELLS - 1 - cy_g)

    @staticmethod
    def _box_iou_q(
        cx1: int, cy1: int, w1: int, h1: int,
        cx2: int, cy2: int, w2: int, h2: int,
    ) -> float:
        bw1, bh1 = w1 * 4.0, h1 * 4.0
        bw2, bh2 = w2 * 4.0, h2 * 4.0
        ax1, ay1, ax2, ay2 = cx1 - bw1 * 0.5, cy1 - bh1 * 0.5, cx1 + bw1 * 0.5, cy1 + bh1 * 0.5
        bx1, by1, bx2, by2 = cx2 - bw2 * 0.5, cy2 - bh2 * 0.5, cx2 + bw2 * 0.5, cy2 + bh2 * 0.5
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0.0:
            return 0.0
        union = (bw1 * bh1) + (bw2 * bh2) - inter
        return inter / max(1e-6, union)

    def _alloc_tid(self) -> int:
        for offset in range(MAX_TRACK_ID):
            cand = (self._next_tid + offset) % MAX_TRACK_ID
            if cand not in self._enc_tracks:
                self._next_tid = (cand + 1) % MAX_TRACK_ID
                return cand
        evict_tid = max(
            self._enc_tracks.keys(),
            key=lambda k: self._enc_tracks[k].ticks_unseen,
        )
        if self._enc_tracks[evict_tid].synced_on_wire:
            self._queue_drop(evict_tid)
        del self._enc_tracks[evict_tid]
        return evict_tid

    def _queue_drop(self, tid: int) -> None:
        for idx, (existing_tid, _) in enumerate(self._drop_queue):
            if existing_tid == tid:
                self._drop_queue[idx] = (tid, 2)
                return
        self._drop_queue.append((tid, 2))

    def _update_encoder_tracks(self, detections: List[Dict[str, Any]]) -> None:
        for trk in self._enc_tracks.values():
            trk.ticks_unseen += 1
            trk.ticks_since_sent += 1
            # Gentle damped velocity coasting during brief YOLO dropouts (1..4 ticks), then hold steady
            if 1 <= trk.ticks_unseen <= 4:
                damp = 0.55 ** trk.ticks_unseen
                trk.cx_q = _clamp(int(round(trk.cx_q + trk.vx_q * damp)), 0, 255)
                trk.cy_q = _clamp(int(round(trk.cy_q + trk.vy_q * damp)), 0, 255)
            else:
                trk.vx_q *= 0.5
                trk.vy_q *= 0.5

        # Prepare detections: prioritize center of frame + confidence
        parsed_dets = []
        for det in detections:
            conf = float(det.get("conf", 0.8))
            if conf < 0.22:
                continue
            x1, y1, x2, y2 = det["bbox"]
            bw = max(2.0, float(x2 - x1))
            bh = max(2.0, float(y2 - y1))
            cx = float(x1) + bw * 0.5
            cy = float(y1) + bh * 0.5
            cx_r = cx / max(1.0, float(self.src_w))
            cy_r = cy / max(1.0, float(self.src_h))
            cls_id = _normalize_cls(det.get("cls", 0))
            cx_q = _clamp(int(round(cx_r * 255)), 0, 255)
            cy_q = _clamp(int(round(cy_r * 255)), 0, 255)
            w6 = _clamp(int(round((bw / self.src_w) * 63)), 1, 63)
            h6 = _clamp(int(round((bh / self.src_h) * 63)), 1, 63)
            dist_center = math.hypot(cx_r - 0.50, cy_r - 0.54)
            center_score = conf - dist_center * 0.35
            parsed_dets.append((center_score, cx_r, cy_r, cx_q, cy_q, w6, h6, cls_id, conf))

        parsed_dets.sort(key=lambda x: x[0], reverse=True)

        # Pass 1: Match existing tracks (ByteTrack hysteresis: accept conf >= 0.22)
        candidates: List[Tuple[float, int, int]] = []
        for d_idx, (_, _, _, cx_q, cy_q, w6, h6, cls_id, conf) in enumerate(parsed_dets):
            for tid, trk in self._enc_tracks.items():
                dist = math.hypot(cx_q - trk.cx_q, cy_q - trk.cy_q)
                max_gate = 22.0 if trk.ticks_unseen <= 3 else 28.0
                if dist > max_gate:
                    continue
                iou = self._box_iou_q(cx_q, cy_q, w6, h6, trk.cx_q, trk.cy_q, trk.w6, trk.h6)
                cls_bonus = 6.0 if trk.cls == cls_id else -4.0
                score = iou * 20.0 + (max_gate - dist) + cls_bonus + conf * 4.0
                candidates.append((score, tid, d_idx))

        candidates.sort(key=lambda x: x[0], reverse=True)
        matched_tids = set()
        used_dets = set()

        for _, tid, d_idx in candidates:
            if tid in matched_tids or d_idx in used_dets:
                continue
            _, _, _, cx_q, cy_q, w6, h6, cls_id, conf = parsed_dets[d_idx]
            trk = self._enc_tracks[tid]
            raw_dx = float(cx_q - trk.cx_q)
            raw_dy = float(cy_q - trk.cy_q)
            step = math.hypot(raw_dx, raw_dy)
            # Anti-jitter when stationary (step <= 2.2), low-lag fast tracking when moving (step > 2.2)
            if step <= 2.2:
                alpha_pos = 0.25
                alpha_sz = 0.20
                trk.vx_q *= 0.4
                trk.vy_q *= 0.4
            else:
                alpha_pos = 0.80
                alpha_sz = 0.50
                inst_vx = max(-35.0, min(35.0, raw_dx))
                inst_vy = max(-35.0, min(35.0, raw_dy))
                trk.vx_q = 0.50 * inst_vx + 0.50 * trk.vx_q
                trk.vy_q = 0.50 * inst_vy + 0.50 * trk.vy_q

            trk.cx_q = _clamp(int(round(alpha_pos * cx_q + (1.0 - alpha_pos) * trk.cx_q)), 0, 255)
            trk.cy_q = _clamp(int(round(alpha_pos * cy_q + (1.0 - alpha_pos) * trk.cy_q)), 0, 255)
            trk.w6 = _clamp(int(round(alpha_sz * w6 + (1.0 - alpha_sz) * trk.w6)), 1, 63)
            trk.h6 = _clamp(int(round(alpha_sz * h6 + (1.0 - alpha_sz) * trk.h6)), 1, 63)
            trk.speed_q = math.hypot(trk.vx_q, trk.vy_q)
            trk.conf = 0.7 * conf + 0.3 * trk.conf
            if conf >= 0.45:
                trk.cls = cls_id
            trk.ticks_unseen = 0
            trk.hits += 1
            if trk.hits >= 2 or self._seq <= 2:
                trk.confirmed = True
            matched_tids.add(tid)
            used_dets.add(d_idx)

        # Pass 2: Spawn genuinely new tracks in the center/active area (conf >= 0.38)
        confirmed_count = sum(1 for t in self._enc_tracks.values() if t.confirmed)
        for d_idx, (_, cx_r, cy_r, cx_q, cy_q, w6, h6, cls_id, conf) in enumerate(parsed_dets):
            if d_idx in used_dets:
                continue
            if conf < 0.38:
                continue
            # Focus on the center / main field of view (ignore far outer border clutter)
            if not (0.06 <= cx_r <= 0.88 and 0.12 <= cy_r <= 0.94):
                continue
            if confirmed_count >= MAX_ACTIVE_TRACKS:
                break
            # Do not spawn duplicate overlapping tracks
            overlaps = False
            for trk in self._enc_tracks.values():
                if math.hypot(cx_q - trk.cx_q, cy_q - trk.cy_q) < 12.0 or (
                    self._box_iou_q(cx_q, cy_q, w6, h6, trk.cx_q, trk.cy_q, trk.w6, trk.h6) > 0.28
                ):
                    overlaps = True
                    break
            if overlaps:
                continue

            new_tid = self._alloc_tid()
            is_init_frame = self._seq <= 2
            self._enc_tracks[new_tid] = _EncoderTrack(
                new_tid, cls_id, conf, cx_q, cy_q, w6, h6, confirmed=is_init_frame
            )
            if is_init_frame:
                confirmed_count += 1
            used_dets.add(d_idx)

        # Pass 3: Position-aware drop (only drop when gone from screen for long)
        for tid in list(self._enc_tracks.keys()):
            trk = self._enc_tracks[tid]
            if trk.ticks_unseen <= 0:
                continue
            if not trk.confirmed and trk.ticks_unseen >= 2:
                del self._enc_tracks[tid]
                continue
            d = self._edge_distance_q(trk.cx_q, trk.cy_q)
            limit = EDGE_UNSEEN_DROP if d < EDGE_RINGS else INTERIOR_UNSEEN_DROP
            if trk.ticks_unseen >= limit:
                if trk.synced_on_wire:
                    self._queue_drop(tid)
                del self._enc_tracks[tid]

    def get_sender_boxes(self, frame_w: Optional[int] = None, frame_h: Optional[int] = None) -> List[Dict[str, Any]]:
        """Returns the exact stabilized, center-prioritized boxes for the sender HUD so sender == receiver."""
        w = frame_w or self.src_w
        h = frame_h or self.src_h
        out: List[Dict[str, Any]] = []
        for tid in sorted(self._enc_tracks.keys()):
            trk = self._enc_tracks[tid]
            if not trk.confirmed:
                continue
            cx = int(round((trk.cx_q / 255.0) * w))
            cy = int(round((trk.cy_q / 255.0) * h))
            w3_idx = _quantize_size_idx(trk.w6, W3_TABLE)
            h3_idx = _quantize_size_idx(trk.h6, H3_TABLE)
            bw = max(12, int(round((W3_TABLE[w3_idx] / 63.0) * w)))
            bh = max(12, int(round((H3_TABLE[h3_idx] / 63.0) * h)))
            out.append(
                {
                    "bbox": (cx - bw // 2, cy - bh // 2, cx + bw // 2, cy + bh // 2),
                    "conf": trk.conf,
                    "cls": trk.cls,
                    "label": VISDRONE_CLASSES.get(trk.cls, f"cls_{trk.cls}"),
                    "track_id": trk.track_id + 1,
                }
            )
        return out

    def _schedule_state_updates(self) -> List[_EncoderTrack]:
        """Prioritizes unsynced new tracks and tracks that actually moved; stationary tracks stay put on receiver."""
        confirmed = [t for t in self._enc_tracks.values() if t.confirmed]
        scored: List[Tuple[float, _EncoderTrack]] = []
        for trk in confirmed:
            disp = math.hypot(trk.cx_q - trk.sent_cx_q, trk.cy_q - trk.sent_cy_q)
            center_dist = math.hypot((trk.cx_q / 255.0) - 0.50, (trk.cy_q / 255.0) - 0.54)
            center_bonus = max(0.0, 2.0 - center_dist * 3.0)
            if not trk.synced_on_wire:
                urgency = 1000.0 + center_bonus * 10.0 + trk.conf * 5.0
                scored.append((urgency, trk))
            elif disp >= 1.0:
                urgency = 100.0 + disp * 12.0 + trk.ticks_since_sent * 2.0 + center_bonus * 5.0
                scored.append((urgency, trk))
            elif trk.ticks_since_sent >= 8:
                # Gentle periodic keepalive for stationary objects so late joiners stay synced
                urgency = float(trk.ticks_since_sent) + center_bonus
                scored.append((urgency, trk))

        scored.sort(key=lambda x: x[0], reverse=True)
        selected = [t for _, t in scored[:MAX_LORA_OBJECTS]]
        # If nothing moved or needed sync, send 1 keepalive object so packet is tiny (8 bytes)
        if not selected and confirmed:
            oldest = max(confirmed, key=lambda t: t.ticks_since_sent)
            selected = [oldest]
        return selected

    @staticmethod
    def _pack_state_obj(trk: _EncoderTrack) -> bytes:
        """Compact 4-byte LoRa state object: [tid:6|cls:4|w3:3|h3:3 (2B), cx_q:8 (1B), cy_q:8 (1B)]."""
        w3 = _quantize_size_idx(trk.w6, W3_TABLE)
        h3 = _quantize_size_idx(trk.h6, H3_TABLE)
        u16_head = (
            ((trk.track_id & 0x3F) << 10)
            | ((trk.cls & 0x0F) << 6)
            | ((w3 & 0x07) << 3)
            | (h3 & 0x07)
        )
        return struct.pack(">HBB", u16_head, trk.cx_q, trk.cy_q)

    def encode_frame(
        self,
        detections: List[Dict[str, Any]],
        frame_shape: Tuple[int, int],
        drone_lat: float,
        drone_lon: float,
        scene_hz: float = SCENE_HZ_DEFAULT,
        sender_lora_only: bool = False,
    ) -> Tuple[bytes, float]:
        del scene_hz, drone_lat, drone_lon
        h, w = frame_shape[:2]
        self.src_w, self.src_h = max(1, int(w)), max(1, int(h))

        now = time.monotonic()
        self._seq = (self._seq + 1) & 0xFFFF
        self._update_encoder_tracks(detections)
        scheduled = self._schedule_state_updates()

        reset_bit = 0x40 if self._pending_reset else 0x00
        self._pending_reset = False
        flags_seq = (0x80 if sender_lora_only else 0x00) | reset_bit | (self._seq & 0x3F)

        obj_payloads: List[bytes] = []
        for trk in scheduled:
            obj_payloads.append(self._pack_state_obj(trk))
            trk.sent_cx_q = trk.cx_q
            trk.sent_cy_q = trk.cy_q
            trk.synced_on_wire = True
            trk.ticks_since_sent = 0

        drop_bytes: List[int] = []
        next_drop_q: Deque[Tuple[int, int]] = deque()
        while self._drop_queue:
            dtid, rem = self._drop_queue.popleft()
            if dtid in self._enc_tracks:
                continue
            if len(drop_bytes) < 6:
                drop_bytes.append(dtid & 0x3F)
                if rem > 1:
                    next_drop_q.append((dtid, rem - 1))
            else:
                next_drop_q.append((dtid, rem))
        self._drop_queue = next_drop_q

        type_count = (MSG_COMPACT_STATE << 4) | (len(obj_payloads) & 0x0F)
        drop_hdr = len(drop_bytes) & 0x0F
        pkt = (
            struct.pack(">BBBB", MAGIC_V2, type_count, flags_seq, drop_hdr)
            + b"".join(obj_payloads)
            + bytes(drop_bytes)
        )
        self._byte_window.append((now, len(pkt)))
        return pkt, self.current_kbps()

    def decode_frame(
        self, data: bytes, canvas_w: Optional[int] = None, canvas_h: Optional[int] = None
    ) -> Optional[Dict[str, Any]]:
        if canvas_w is not None:
            self.dst_w = canvas_w
        if canvas_h is not None:
            self.dst_h = canvas_h
        if len(data) < 3:
            return None
        if data[0] == MAGIC_V1:
            return self._decode_v1(data)
        if data[0] != MAGIC_V2:
            return None
        return self._decode_v2(data, time.monotonic())

    def _decode_v2(self, data: bytes, now: float) -> Optional[Dict[str, Any]]:
        type_count = data[1]
        msg_type = (type_count >> 4) & 0x0F
        obj_count = type_count & 0x0F
        sender_lora_only: Optional[bool] = None

        if msg_type == MSG_COMPACT_STATE:
            if len(data) < 4:
                return None
            flags_seq = data[2]
            drop_count = data[3] & 0x0F
            sender_lora_only = bool(flags_seq & 0x80)
            scene_reset = bool(flags_seq & 0x40)
            seq = flags_seq & 0x3F
            if scene_reset:
                self._dec_tracks.clear()
            offset = 4
            updated_tids = set()
            for _ in range(obj_count):
                if offset + 4 > len(data):
                    break
                u16_head, cx_q, cy_q = struct.unpack(">HBB", data[offset : offset + 4])
                offset += 4
                tid = (u16_head >> 10) & 0x3F
                cls_id = (u16_head >> 6) & 0x0F
                w3 = (u16_head >> 3) & 0x07
                h3 = u16_head & 0x07
                w6 = W3_TABLE[w3]
                h6 = H3_TABLE[h3]
                self._apply_decoder_state(
                    tid, cls_id, 0.88, cx_q, cy_q, w6, h6, now, is_kf=True
                )
                updated_tids.add(tid)
            for _ in range(drop_count):
                if offset >= len(data):
                    break
                drop_tid = data[offset] & 0x3F
                offset += 1
                self._dec_tracks.pop(drop_tid, None)
            # Persistent restoration screen: non-updated tracks stay in place, only decaying velocity
            for tid in list(self._dec_tracks.keys()):
                trk = self._dec_tracks[tid]
                if tid not in updated_tids:
                    trk.vel = [trk.vel[0] * 0.70, trk.vel[1] * 0.70]
                    # Only prune if completely abandoned for > 18 seconds
                    if (now - trk.last_update_ts) > 18.0:
                        del self._dec_tracks[tid]
                else:
                    trk.last_keyframe_ts = now
        elif msg_type in (MSG_COMPACT_KF, MSG_COMPACT_DELTA):
            flags_seq = data[2]
            sender_lora_only = bool(flags_seq & 0x80)
            seq = flags_seq & 0x7F
            if msg_type == MSG_COMPACT_KF:
                offset = 3
                seen_in_kf = set()
                for _ in range(obj_count):
                    if offset + 4 > len(data):
                        break
                    b0, cx_q, cy_q, wh = struct.unpack(">BBBB", data[offset : offset + 4])
                    offset += 4
                    tid = (b0 >> 4) & 0x0F
                    cls_id = b0 & 0x0F
                    w6 = max(2, ((wh >> 4) & 0x0F) * 4 + 2)
                    h6 = max(2, (wh & 0x0F) * 4 + 2)
                    seen_in_kf.add(tid)
                    self._apply_decoder_state(
                        tid, cls_id, 0.88, cx_q, cy_q, w6, h6, now, is_kf=True
                    )
                for tid in list(self._dec_tracks.keys()):
                    if tid not in seen_in_kf:
                        del self._dec_tracks[tid]
            else:
                offset = 3
                updated_tids = set()
                for _ in range(obj_count):
                    if offset + 2 > len(data):
                        break
                    (word,) = struct.unpack(">H", data[offset : offset + 2])
                    offset += 2
                    tid = (word >> 12) & 0x0F
                    dcx = _unpack_i6((word >> 6) & 0x3F)
                    dcy = _unpack_i6(word & 0x3F)
                    if dcx == -32 and dcy == -32:
                        self._dec_tracks.pop(tid, None)
                        continue
                    trk = self._dec_tracks.get(tid)
                    if trk is not None:
                        new_cx_q = _clamp(trk.cx_q + dcx, 0, 255)
                        new_cy_q = _clamp(trk.cy_q + dcy, 0, 255)
                        self._apply_decoder_state(
                            tid, trk.cls, trk.conf, new_cx_q, new_cy_q, trk.w6, trk.h6, now, is_kf=False
                        )
                        updated_tids.add(tid)
                for tid, trk in self._dec_tracks.items():
                    if tid not in updated_tids:
                        trk.vel = [trk.vel[0] * 0.4, trk.vel[1] * 0.4]
        else:
            if len(data) < 4:
                return None
            _, _, seq = struct.unpack(">BBH", data[:4])
            if msg_type == MSG_KEYFRAME:
                if len(data) < 12:
                    return None
                self._dec_lat, self._dec_lon = struct.unpack(">ff", data[4:12])
                offset = 12
                seen_in_kf = set()
                for _ in range(obj_count):
                    if offset + 5 > len(data):
                        break
                    b0, cx_q, cy_q, u16_tail = struct.unpack(">BBBH", data[offset : offset + 5])
                    offset += 5
                    tid = (b0 >> 4) & 0x0F
                    cls_id = b0 & 0x0F
                    conf = round(((u16_tail >> 12) & 0x0F) / 15.0, 2)
                    w6 = max(1, (u16_tail >> 6) & 0x3F)
                    h6 = max(1, u16_tail & 0x3F)
                    seen_in_kf.add(tid)
                    self._apply_decoder_state(
                        tid, cls_id, conf, cx_q, cy_q, w6, h6, now, is_kf=True
                    )
                for tid in list(self._dec_tracks.keys()):
                    if tid not in seen_in_kf and (now - self._dec_tracks[tid].last_keyframe_ts) > 1.2:
                        del self._dec_tracks[tid]

            elif msg_type == MSG_DELTA:
                offset = 4
                updated_tids = set()
                for _ in range(obj_count):
                    if offset + 2 > len(data):
                        break
                    (word,) = struct.unpack(">H", data[offset : offset + 2])
                    offset += 2
                    tid = (word >> 12) & 0x0F
                    dcx = _unpack_i6((word >> 6) & 0x3F)
                    dcy = _unpack_i6(word & 0x3F)
                    if dcx == -32 and dcy == -32:
                        self._dec_tracks.pop(tid, None)
                        continue
                    trk = self._dec_tracks.get(tid)
                    if trk is not None:
                        new_cx_q = _clamp(trk.cx_q + dcx, 0, 255)
                        new_cy_q = _clamp(trk.cy_q + dcy, 0, 255)
                        self._apply_decoder_state(
                            tid,
                            trk.cls,
                            trk.conf,
                            new_cx_q,
                            new_cy_q,
                            trk.w6,
                            trk.h6,
                            now,
                            is_kf=False,
                        )
                        updated_tids.add(tid)
                for tid, trk in self._dec_tracks.items():
                    if tid not in updated_tids:
                        trk.vel = [trk.vel[0] * 0.4, trk.vel[1] * 0.4]

        out = self._build_output(seq, len(data), now)
        if sender_lora_only is not None:
            out["sender_lora_only"] = sender_lora_only
        elif len(data) >= 8 and data[-4] == 0xCF:
            raw_clip = int(data[-3])
            out["clip_idx"] = raw_clip & 0x7F
            out["sender_lora_only"] = bool(raw_clip & 0x80)
            (out["video_frame"],) = struct.unpack(">H", data[-2:])
        return out

    def _apply_decoder_state(
        self,
        tid: int,
        cls_id: int,
        conf: float,
        cx_q: int,
        cy_q: int,
        w6: int,
        h6: int,
        now: float,
        is_kf: bool,
    ) -> None:
        cx_px = int(round((cx_q / 255.0) * self.dst_w))
        cy_px = int(round((cy_q / 255.0) * self.dst_h))
        trk = self._dec_tracks.get(tid)
        if trk is None:
            trk = _DecoderTrack(tid, cls_id, conf, cx_q, cy_q, w6, h6, now)
            trk.trail.append([cx_px, cy_px])
            self._dec_tracks[tid] = trk
        else:
            prev_cx, prev_cy = trk.trail[-1] if trk.trail else (cx_px, cy_px)
            dx = float(cx_px - prev_cx)
            dy = float(cy_px - prev_cy)
            trk.vel = [dx * 0.65 + trk.vel[0] * 0.35, dy * 0.65 + trk.vel[1] * 0.35]
            trk.cls = cls_id
            trk.conf = conf
            trk.cx_q, trk.cy_q = cx_q, cy_q
            trk.w6, trk.h6 = w6, h6
            trk.trail.append([cx_px, cy_px])
            trk.last_update_ts = now
            if is_kf:
                trk.last_keyframe_ts = now

    def _build_output(self, seq: int, num_bytes: int, now: float) -> Dict[str, Any]:
        objects_out: List[Dict[str, Any]] = []
        for tid in sorted(self._dec_tracks.keys()):
            trk = self._dec_tracks[tid]
            if (now - trk.last_update_ts) > 18.0:
                continue
            cx = int(round((trk.cx_q / 255.0) * self.dst_w))
            cy = int(round((trk.cy_q / 255.0) * self.dst_h))
            bw = max(12, int(round((trk.w6 / 63.0) * self.dst_w)))
            bh = max(12, int(round((trk.h6 / 63.0) * self.dst_h)))
            wire_cls = trk.cls + 1
            objects_out.append(
                {
                    "cls": wire_cls,
                    "label": VISDRONE_CLASSES.get(trk.cls, "class_%d" % trk.cls),
                    "conf": trk.conf,
                    "bbox": [cx - bw // 2, cy - bh // 2, cx + bw // 2, cy + bh // 2],
                    "center": [cx, cy],
                    "track_id": trk.track_id + 1,
                    "sector": _sector_from_xy(cx, cy, self.dst_w, self.dst_h),
                    "motion_str": _motion_str_from_vel(trk.vel[0], trk.vel[1]),
                    "trail": list(trk.trail),
                    "vel": [round(trk.vel[0], 2), round(trk.vel[1], 2)],
                }
            )
        return {
            "seq": seq,
            "lat": round(self._dec_lat, 6),
            "lon": round(self._dec_lon, 6),
            "count": len(objects_out),
            "bytes": num_bytes,
            "objects": objects_out,
        }

    def _decode_v1(self, data: bytes) -> Optional[Dict[str, Any]]:
        if len(data) < V1_HEADER_SIZE:
            return None
        _, seq, lat, lon, count = struct.unpack(V1_HEADER_FMT, data[:V1_HEADER_SIZE])
        self._dec_lat, self._dec_lon = lat, lon
        objects_out: List[Dict[str, Any]] = []
        offset = V1_HEADER_SIZE
        for idx in range(count):
            if offset + V1_OBJ_SIZE > len(data):
                break
            cls_id, conf_u8, cx_q, cy_q, w_q, h_q = struct.unpack(
                V1_OBJ_FMT, data[offset : offset + V1_OBJ_SIZE]
            )
            offset += V1_OBJ_SIZE
            cx = int(round((cx_q / 255.0) * self.dst_w))
            cy = int(round((cy_q / 255.0) * self.dst_h))
            bw = max(12, int(round((w_q / 255.0) * self.dst_w)))
            bh = max(12, int(round((h_q / 255.0) * self.dst_h)))
            wire_cls = cls_id if cls_id >= 1 else 1
            norm = _clamp(cls_id - 1 if cls_id >= 1 else 0, 0, 9)
            label = CLS_TO_LABEL.get(wire_cls) or VISDRONE_CLASSES.get(norm, "class_%d" % cls_id)
            objects_out.append(
                {
                    "cls": wire_cls,
                    "label": label,
                    "conf": round(conf_u8 / 100.0, 2),
                    "bbox": [cx - bw // 2, cy - bh // 2, cx + bw // 2, cy + bh // 2],
                    "center": [cx, cy],
                    "track_id": idx + 1,
                    "sector": _sector_from_xy(cx, cy, self.dst_w, self.dst_h),
                    "motion_str": "STATIONARY",
                    "trail": [[cx, cy]],
                    "vel": [0.0, 0.0],
                }
            )
        return {
            "seq": seq,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "count": len(objects_out),
            "bytes": len(data),
            "objects": objects_out,
        }
