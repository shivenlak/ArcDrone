"""Target detection and alert gating for drone edge node."""
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
        imgsz: int = 640,
    ):
        self.conf_thresh = conf_thresh
        self.cooldown_sec = cooldown_sec
        self.imgsz = imgsz  # 1280 finds far more of VisDrone's tiny objects
        self.last_alert_time: Dict[int, float] = {}
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
            return COCO_TO_VISDRONE.get(raw_cls, None)
        return raw_cls if raw_cls < len(self.repo_classes) else None

    def process_frame(
        self, frame_bgr: np.ndarray
    ) -> Tuple[List[Dict[str, Any]], Optional[Dict[str, Any]]]:
        now = time.time()
        results = self.model.predict(
            source=frame_bgr, conf=self.conf_thresh, imgsz=self.imgsz, verbose=False
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

            if mapped_cls is not None:
                qualifying.append(box_item)

        if not qualifying:
            return all_boxes, None

        qualifying.sort(key=lambda item: item["conf"], reverse=True)
        top = qualifying[0]
        target_cls = top["cls"]

        last_t = self.last_alert_time.get(target_cls, 0.0)
        if (now - last_t) < self.cooldown_sec:
            return all_boxes, None

        count = sum(1 for q in qualifying if q["cls"] == target_cls)

        self.last_alert_time[target_cls] = now
        alert = {
            "cls": target_cls,
            "conf": top["conf"],
            "count": count,
            "bbox": top["bbox"],
            "label": top["label"],
        }
        return all_boxes, alert