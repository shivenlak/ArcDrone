"""Live Manan 0xB4 ground receiver view — same layout as scripts/ishan_side_by_side.run_pipeline.

Dashboard Scene twin expects the 1:1 side-by-side composite (RGB left + blank ratio canvas right),
not a cropped MJPEG. See --manan-panel-port.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from scripts.ishan_side_by_side import (
    VISDRONE_CLASSES,
    DroneIndexedTracker,
    GroundVectorReceiver,
    SpreadOutVectorTransmitter,
    render_bottom_index_manifest,
    render_left_rgb_canvas,
    render_right_blank_ratio_canvas,
)

# Match run_pipeline() in scripts/ishan_side_by_side.py
CANVAS_W, CANVAS_H = 960, 540
HEADER_H = 56
MANIFEST_H = 364

# Grid-aware coasting: frame split into GRID x GRID cells.
# Ring distance from center: 0 = border cell ... (GRID/2 - 1) = dead center.
GRID = 16
EDGE_RING_CELLS = 3          # outer 3 rings = "end" zone (object is leaving frame)
EDGE_UNSEEN_DROP = 2         # edge contacts vanish after 2 missed frames
INTERIOR_UNSEEN_DROP = 3     # central contacts coast 3 frames (likely a YOLO blink)


class GridCoastTracker(DroneIndexedTracker):
    """DroneIndexedTracker with position-aware drop.

    An object that disappears near the frame edge has almost certainly left the
    scene, so drop it fast (EDGE_UNSEEN_DROP). One that blinks out toward the
    center is probably a brief detector miss, so coast it a little longer
    (INTERIOR_UNSEEN_DROP) before removing it. Dropped IDs still flow through the
    normal OP_DROP_BOX path so the ground view clears them.
    """

    @staticmethod
    def _edge_distance(cx_r: float, cy_r: float) -> int:
        cx = min(GRID - 1, max(0, int(cx_r * GRID)))
        cy = min(GRID - 1, max(0, int(cy_r * GRID)))
        return min(cx, GRID - 1 - cx, cy, GRID - 1 - cy)

    def update(
        self,
        detections: List[Dict[str, Any]],
        frame_w: int,
        frame_h: int,
        sim_t: float,
    ) -> List[Dict[str, Any]]:
        super().update(detections, frame_w, frame_h, sim_t)

        for tid in list(self.tracks.keys()):
            trk = self.tracks[tid]
            if trk["unseen"] <= 0:
                continue
            d = self._edge_distance(trk["cx_r"], trk["cy_r"])
            limit = EDGE_UNSEEN_DROP if d < EDGE_RING_CELLS else INTERIOR_UNSEEN_DROP
            if trk["unseen"] >= limit:
                if trk.get("synced") and trk["id"] > 0:
                    self.dropped_ids.append(trk["id"])
                del self.tracks[tid]

        return [
            t
            for t in self.tracks.values()
            if t["id"] > 0 and t["unseen"] <= INTERIOR_UNSEEN_DROP
        ]


def _filter_visdrone_boxes(detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Manan's tracker expects VisDrone cls 0–9; COCO-mapped Nones are dropped."""
    out: List[Dict[str, Any]] = []
    for det in detections:
        cls_id = det.get("cls")
        if cls_id is None:
            continue
        cls_id = int(cls_id)
        if cls_id < 0 or cls_id >= len(VISDRONE_CLASSES):
            continue
        bbox = det.get("bbox")
        if not bbox or len(bbox) != 4:
            continue
        conf = float(det.get("conf", 0.0))
        out.append(
            {
                "bbox": tuple(int(v) for v in bbox),
                "conf": conf,
                "cls": cls_id,
                "label": VISDRONE_CLASSES[cls_id],
            }
        )
    return out


class MananGroundPanel:
    """Same drone tracker + spread TX + ground RX loop as the offline showcase, clocked by frame_idx."""

    def __init__(self, vector_hz: float = 12.0) -> None:
        self.tracker = GridCoastTracker(max_unseen_frames=36)
        self.tx = SpreadOutVectorTransmitter(target_fps=vector_hz, max_updates_per_pkt=12)
        self.rx = GroundVectorReceiver()
        self.baseline_done = False
        self._last_composite: Optional[np.ndarray] = None
        self._src_w = 1920
        self._src_h = 1080

    def on_video_frame(
        self,
        raw_frame: np.ndarray,
        detections: List[Dict[str, Any]],
        frame_idx: int,
        fps: float,
        drone_lat: float,
        drone_lon: float,
        scenario_title: str = "",
    ) -> np.ndarray:
        h, w = raw_frame.shape[:2]
        self._src_w, self._src_h = w, h
        sim_t = frame_idx / max(0.1, fps)
        raw_boxes = _filter_visdrone_boxes(detections)
        active_tracks = self.tracker.update(raw_boxes, w, h, sim_t)

        if frame_idx == 1:
            init_pkts = self.tx.build_initial_baseline_packets(
                active_tracks, drone_lat, drone_lon, sim_t
            )
            for idx_p, ipkt in enumerate(init_pkts):
                self.rx.receive_packet(
                    ipkt, sim_t, count_stats=(idx_p == len(init_pkts) - 1)
                )
            self.baseline_done = True
        elif self.tx.should_send_packet(sim_t):
            pkt, _ = self.tx.build_spread_packet(
                self.tracker, active_tracks, drone_lat, drone_lon, sim_t
            )
            self.rx.receive_packet(pkt, sim_t)

        bps = self.tx.rolling_bps(sim_t)
        left_canvas = render_left_rgb_canvas(
            raw_frame, active_tracks, CANVAS_W, CANVAS_H, scenario_title=scenario_title
        )
        right_canvas = render_right_blank_ratio_canvas(
            self.rx,
            CANVAS_W,
            CANVAS_H,
            sim_t,
            src_w=w,
            src_h=h,
            scenario_title=scenario_title,
        )
        total_w = CANVAS_W * 2
        bottom_manifest = render_bottom_index_manifest(
            self.rx, total_w, MANIFEST_H, sim_t
        )

        header = np.full((HEADER_H, total_w, 3), (10, 14, 18), dtype=np.uint8)
        title_text = (
            "ARCDRONE 1:1 RATIO SIDE-BY-SIDE: DRONE RGB CAMERA vs GROUND INDEXED VECTOR RECEIVER (NO RGB)"
        )
        if scenario_title:
            title_text = f"ARCDRONE TEST BENCH: {scenario_title.upper()} (1:1 RATIO VECTOR LINK)"
        cv2.putText(
            header,
            title_text,
            (18, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (0, 225, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            header,
            f"SPREAD-OUT LINK: 12.0 FPS ({self.rx.total_pkts} pkts, {self.rx.last_pkt_bytes} B/pkt)  |  "
            f"WIRE RATE: {bps/1000.0:.2f} kB/s (CAP <= 3.00 kB/s)  |  "
            f"ACTIVE INDICES: {len(self.rx.boxes)}  |  GPS: ({drone_lat:.4f}, {drone_lon:.4f})",
            (18, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (140, 255, 170),
            1,
            cv2.LINE_AA,
        )

        mid_row = np.hstack([left_canvas, right_canvas])
        cv2.line(mid_row, (CANVAS_W, 0), (CANVAS_W, CANVAS_H), (80, 110, 130), 2)
        composite = np.vstack([header, mid_row, bottom_manifest])
        self._last_composite = composite
        return composite

    def latest_frame(self) -> Optional[np.ndarray]:
        return self._last_composite
