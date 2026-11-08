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

MSG_HEARTBEAT = 0x1
MSG_KEYFRAME = 0x2
MSG_DELTA = 0x3

LINK_MAX_KBPS = 2.0
RESERVE_KBPS = 0.0
SCENE_MAX_KBPS = LINK_MAX_KBPS
SCENE_HZ_DEFAULT = 12.0
MAX_BYTES_PER_SEC = int(SCENE_MAX_KBPS * 1000 / 8)  # 250 B/s

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
        "sent_cx_q",
        "sent_cy_q",
        "synced_on_wire",
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
    ) -> None:
        self.track_id = track_id
        self.cls = cls
        self.conf = conf
        self.cx_q = cx_q
        self.cy_q = cy_q
        self.w6 = w6
        self.h6 = h6
        self.sent_cx_q = cx_q
        self.sent_cy_q = cy_q
        self.synced_on_wire = False
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


class SceneCodec:
    """Stateful v2 encoder/decoder (12 Hz friendly under 250 B/s)."""

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
        self._ticks_since_kf = keyframe_interval_ticks
        self._byte_window: Deque[Tuple[float, int]] = deque()
        self._enc_tracks: Dict[int, _EncoderTrack] = {}

        self._dec_lat = 53.5232
        self._dec_lon = -113.5263
        self._dec_tracks: Dict[int, _DecoderTrack] = {}

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

    def _update_encoder_tracks(self, detections: List[Dict[str, Any]]) -> None:
        for trk in self._enc_tracks.values():
            trk.ticks_unseen += 1
            trk.ticks_since_sent += 1

        matched_tids = set()
        for det in detections[:16]:
            x1, y1, x2, y2 = det["bbox"]
            bw = max(2.0, float(x2 - x1))
            bh = max(2.0, float(y2 - y1))
            cx = float(x1) + bw * 0.5
            cy = float(y1) + bh * 0.5
            cls_id = _normalize_cls(det.get("cls", 0))
            conf = float(det.get("conf", 0.8))

            cx_q = _clamp(int(round((cx / self.src_w) * 255)), 0, 255)
            cy_q = _clamp(int(round((cy / self.src_h) * 255)), 0, 255)
            w6 = _clamp(int(round((bw / self.src_w) * 63)), 1, 63)
            h6 = _clamp(int(round((bh / self.src_h) * 63)), 1, 63)

            best_tid = None
            best_dist = 30.0
            for tid, trk in self._enc_tracks.items():
                if tid in matched_tids or trk.cls != cls_id:
                    continue
                dist = math.hypot(cx_q - trk.cx_q, cy_q - trk.cy_q)
                if dist <= best_dist:
                    best_dist = dist
                    best_tid = tid

            if best_tid is not None:
                trk = self._enc_tracks[best_tid]
                trk.speed_q = math.hypot(cx_q - trk.cx_q, cy_q - trk.cy_q)
                trk.cx_q, trk.cy_q = cx_q, cy_q
                trk.w6, trk.h6 = w6, h6
                trk.conf = conf
                trk.ticks_unseen = 0
                matched_tids.add(best_tid)
            else:
                free_ids = [i for i in range(16) if i not in self._enc_tracks]
                if not free_ids:
                    evict_tid = max(
                        self._enc_tracks.keys(),
                        key=lambda k: self._enc_tracks[k].ticks_unseen,
                    )
                    del self._enc_tracks[evict_tid]
                    new_tid = evict_tid
                else:
                    new_tid = free_ids[0]
                self._enc_tracks[new_tid] = _EncoderTrack(
                    new_tid, cls_id, conf, cx_q, cy_q, w6, h6
                )
                matched_tids.add(new_tid)

        for tid in list(self._enc_tracks.keys()):
            if self._enc_tracks[tid].ticks_unseen > 5:
                del self._enc_tracks[tid]

    def _schedule_tracks(self) -> List[_EncoderTrack]:
        visible = [t for t in self._enc_tracks.values() if t.ticks_unseen <= 1]

        def threat_score(t: _EncoderTrack) -> float:
            w = CLASS_THREAT_WEIGHT.get(t.cls, 1.0)
            motion_bonus = 0.25 if t.speed_q >= 1.5 else 0.0
            return (w * t.conf) + motion_bonus

        visible.sort(key=threat_score, reverse=True)
        tier1 = visible[: self.top_k_pinned]
        tier2 = visible[self.top_k_pinned :]
        tier2.sort(key=lambda t: t.ticks_since_sent, reverse=True)
        return tier1 + tier2

    @staticmethod
    def _pack_keyframe_obj(trk: _EncoderTrack) -> bytes:
        b0 = ((trk.track_id & 0x0F) << 4) | (trk.cls & 0x0F)
        conf_q = _clamp(int(round(trk.conf * 15.0)), 0, 15)
        u16_tail = (conf_q << 12) | ((trk.w6 & 0x3F) << 6) | (trk.h6 & 0x3F)
        return struct.pack(">BBBH", b0, trk.cx_q, trk.cy_q, u16_tail)

    @staticmethod
    def _pack_delta_obj(track_id: int, dcx: int, dcy: int) -> bytes:
        dcx_6 = _pack_i6(dcx)
        dcy_6 = _pack_i6(dcy)
        word = ((track_id & 0x0F) << 12) | (dcx_6 << 6) | dcy_6
        return struct.pack(">H", word)

    def encode_frame(
        self,
        detections: List[Dict[str, Any]],
        frame_shape: Tuple[int, int],
        drone_lat: float,
        drone_lon: float,
        scene_hz: float = SCENE_HZ_DEFAULT,
    ) -> Tuple[bytes, float]:
        del scene_hz  # caller sets wall-clock rate; budget is rolling B/s
        h, w = frame_shape[:2]
        self.src_w, self.src_h = max(1, int(w)), max(1, int(h))

        now = time.monotonic()
        self._seq = (self._seq + 1) & 0xFFFF
        avail = self._available_budget(now)
        self._update_encoder_tracks(detections)
        scheduled = self._schedule_tracks()

        has_unsynced = any(not t.synced_on_wire for t in scheduled)
        want_keyframe = (self._ticks_since_kf >= self.keyframe_interval) or (
            has_unsynced and avail >= 22
        )

        if want_keyframe and avail >= 12:
            obj_payloads: List[bytes] = []
            used_bytes = 12
            for trk in scheduled[:15]:
                if used_bytes + 5 <= avail or (not obj_payloads and avail >= 17):
                    obj_payloads.append(self._pack_keyframe_obj(trk))
                    used_bytes += 5
                    trk.sent_cx_q = trk.cx_q
                    trk.sent_cy_q = trk.cy_q
                    trk.synced_on_wire = True
                    trk.ticks_since_sent = 0

            type_count = (MSG_KEYFRAME << 4) | (len(obj_payloads) & 0x0F)
            pkt = struct.pack(
                ">BBHff",
                MAGIC_V2,
                type_count,
                self._seq,
                float(drone_lat),
                float(drone_lon),
            ) + b"".join(obj_payloads)
            self._ticks_since_kf = 0
            self._byte_window.append((now, len(pkt)))
            return pkt, self.current_kbps()

        delta_payloads: List[bytes] = []
        used_bytes = 4
        for trk in scheduled:
            if not trk.synced_on_wire or len(delta_payloads) >= 15:
                continue
            dcx = _clamp(trk.cx_q - trk.sent_cx_q, -31, 31)
            dcy = _clamp(trk.cy_q - trk.sent_cy_q, -31, 31)
            if abs(dcx) >= 1 or abs(dcy) >= 1:
                if used_bytes + 2 <= avail:
                    delta_payloads.append(self._pack_delta_obj(trk.track_id, dcx, dcy))
                    used_bytes += 2
                    trk.sent_cx_q += dcx
                    trk.sent_cy_q += dcy
                    trk.ticks_since_sent = 0

        self._ticks_since_kf += 1
        if not delta_payloads:
            type_count = (MSG_HEARTBEAT << 4) | 0
            pkt = struct.pack(">BBH", MAGIC_V2, type_count, self._seq)
        else:
            type_count = (MSG_DELTA << 4) | (len(delta_payloads) & 0x0F)
            pkt = struct.pack(">BBH", MAGIC_V2, type_count, self._seq) + b"".join(
                delta_payloads
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
        if len(data) < 4:
            return None
        if data[0] == MAGIC_V1:
            return self._decode_v1(data)
        if data[0] != MAGIC_V2:
            return None
        return self._decode_v2(data, time.monotonic())

    def _decode_v2(self, data: bytes, now: float) -> Optional[Dict[str, Any]]:
        _, type_count, seq = struct.unpack(">BBH", data[:4])
        msg_type = (type_count >> 4) & 0x0F
        obj_count = type_count & 0x0F

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

        elif msg_type == MSG_HEARTBEAT:
            for trk in self._dec_tracks.values():
                trk.vel = [trk.vel[0] * 0.4, trk.vel[1] * 0.4]

        return self._build_output(seq, len(data), now)

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
            if (now - trk.last_keyframe_ts) > 1.6:
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
