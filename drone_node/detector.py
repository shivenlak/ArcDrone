"""Target detection and alert gating for drone edge node."""
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from ultralytics import YOLO

from protocol import packets as p

VISDRONE_CLASSES = [
    "pedestrian",
    "people",
    "bicycle",
    "car",
    "van",
    "truck",
    "tricycle",
    "awning-tricycle",
    "bus",
    "motor",
]

COCO_TO_VISDRONE = {
    0: 0,  # person -> pedestrian
    1: 2,  # bicycle -> bicycle
    2: 3,  # car -> car
    3: 9,  # motorcycle -> motor
    5: 8,  # bus -> bus
    7: 5,  # truck -> truck
}


class Detector:
    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        conf_thresh: float = 0.5,
        cooldown_sec: float = 3.0,
        spatial_hold_sec: float = 25.0,
        match_gate_frac: float = 0.33,
        imgsz: int = 640,
    ):
        self.conf_thresh = conf_thresh
        self.cooldown_sec = cooldown_sec
        self.imgsz = int(imgsz)  # 1280 finds far more tiny aerial objects than 640
        # A target that leaves view for longer than this is treated as a new contact.
        self.spatial_hold_sec = spatial_hold_sec
        self.match_gate_frac = match_gate_frac
        self.last_alert_time: Dict[int, float] = {}
        # Lightweight per-target tracks so one person = one alert while in view.
        # Each: {"id","cls","cx","cy","last_seen","alerted","conf","bbox","label"}
        self._tracks: List[Dict[str, Any]] = []
        self._next_track_id = 1
        self.repo_classes = getattr(p, "CLASS_NAMES", VISDRONE_CLASSES)

        weights = model_path or "yolov8n.pt"
        print(f"[DETECTOR] Loading YOLO model from '{weights}'...")
        try:
            self.model = YOLO(weights)
        except Exception as e:
            print(f"[DETECTOR] Warning loading '{weights}': {e}. Falling back to 'yolov8n.pt'")
            self.model = YOLO("yolov8n.pt")

        model_names = getattr(self.model, "names", {})
        self.is_coco = "person" in model_names.values()

    def _resolve_class_id(self, raw_cls: int) -> Optional[int]:
        if self.is_coco:
            if raw_cls in COCO_TO_VISDRONE:
                return COCO_TO_VISDRONE[raw_cls]
            return raw_cls % len(self.repo_classes)
        return raw_cls if raw_cls < len(self.repo_classes) else None

    @staticmethod
    def _bbox_center(bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
        x1, y1, x2, y2 = bbox
        return (x1 + x2) * 0.5, (y1 + y2) * 0.5

    def _associate(self, detections: List[Dict[str, Any]], now: float, gate: float) -> None:
        """Match detections to existing tracks (same class, nearest center within gate).

        New, unmatched detections become fresh tracks with alerted=False, so a person
        who stays in view keeps one track (one alert) while they move, and a person who
        left and came back later is a new track (can alert again).
        """
        cutoff = now - self.spatial_hold_sec
        self._tracks = [t for t in self._tracks if t["last_seen"] >= cutoff]
        matched_tracks = set()
        for det in detections:  # highest-confidence first
            cx, cy = self._bbox_center(det["bbox"])
            best = None
            best_dist = gate
            for trk in self._tracks:
                if id(trk) in matched_tracks or trk["cls"] != det["cls"]:
                    continue
                dist = math.hypot(cx - trk["cx"], cy - trk["cy"])
                if dist <= best_dist:
                    best_dist = dist
                    best = trk
            if best is not None:
                best.update(
                    cx=cx, cy=cy, last_seen=now, conf=det["conf"],
                    bbox=det["bbox"], label=det["label"],
                )
                matched_tracks.add(id(best))
            else:
                trk = {
                    "id": self._next_track_id, "cls": det["cls"], "cx": cx, "cy": cy,
                    "last_seen": now, "alerted": False, "conf": det["conf"],
                    "bbox": det["bbox"], "label": det["label"],
                }
                self._next_track_id += 1
                self._tracks.append(trk)
                matched_tracks.add(id(trk))

    def process_frame(
        self, frame_bgr: np.ndarray
    ) -> Tuple[List[Dict[str, Any]], Optional[Dict[str, Any]]]:
        now = time.time()
        results = self.model.predict(
            source=frame_bgr, conf=min(self.conf_thresh, 0.22), imgsz=self.imgsz, verbose=False
        )

        all_boxes: List[Dict[str, Any]] = []
        qualifying: List[Dict[str, Any]] = []

        if not results or len(results) == 0:
            return all_boxes, None

        r = results[0]
        if r.boxes is None:
            return all_boxes, None

        for box in r.boxes:
            conf = float(box.conf[0])
            raw_cls = int(box.cls[0])
            mapped_cls = self._resolve_class_id(raw_cls)

            xyxy = [int(v) for v in box.xyxy[0].tolist()]
            label_str = (
                self.repo_classes[mapped_cls]
                if mapped_cls is not None and mapped_cls < len(self.repo_classes)
                else f"cls_{raw_cls}"
            )

            box_item = {
                "bbox": (xyxy[0], xyxy[1], xyxy[2], xyxy[3]),
                "conf": conf,
                "cls": mapped_cls,
                "label": label_str,
            }
            all_boxes.append(box_item)

            if mapped_cls is not None and conf >= self.conf_thresh:
                qualifying.append(box_item)

        if not qualifying:
            return all_boxes, None

        qualifying.sort(key=lambda item: item["conf"], reverse=True)

        h, w = frame_bgr.shape[:2]
        gate = min(w, h) * self.match_gate_frac
        self._associate(qualifying, now, gate)

        # New contact = a track seen this frame that we have never alerted on.
        fresh = [
            t for t in self._tracks
            if t["last_seen"] == now and not t["alerted"]
        ]
        if not fresh:
            return all_boxes, None

        fresh.sort(key=lambda t: t["conf"], reverse=True)
        top = fresh[0]
        target_cls = top["cls"]

        # Light pacing so a burst of new contacts does not all fire in one tick.
        last_t = self.last_alert_time.get(target_cls, 0.0)
        if (now - last_t) < self.cooldown_sec:
            return all_boxes, None

        top["alerted"] = True
        self.last_alert_time[target_cls] = now
        count = sum(1 for q in qualifying if q["cls"] == target_cls)
        alert = {
            "cls": target_cls,
            "conf": top["conf"],
            "count": count,
            "bbox": top["bbox"],
            "label": top["label"],
        }
        return all_boxes, alert
