"""UAARG Hierarchical Event & Image Logger for ArcDrone.

Adapts UAARG shepard session-based logging (tmp/log/<run_id>/) saving:
- Raw frames: images/<seq>.png
- Annotated frames: bb/<seq>.png
- Structured telemetry snapshots: <seq>.json
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple, Union

try:
    from PIL import Image, ImageDraw, ImageFont
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False


class EventLogger:
    """Manages hierarchical session logging adhering to UAARG shepard standards."""

    def __init__(
        self,
        base_dir: str = "tmp/log",
        run_id: Optional[Union[int, str]] = None,
    ) -> None:
        self.base_dir = os.path.abspath(base_dir)
        os.makedirs(self.base_dir, exist_ok=True)

        if run_id is None:
            self.run_id = self._allocate_run_id()
        else:
            self.run_id = str(run_id)

        self.run_dir = os.path.join(self.base_dir, str(self.run_id))
        self.img_dir = os.path.join(self.run_dir, "images")
        self.bb_dir = os.path.join(self.run_dir, "bb")

        os.makedirs(self.run_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.bb_dir, exist_ok=True)

        self._lock = threading.Lock()
        self.logged_seqs: List[int] = []

    def _allocate_run_id(self) -> int:
        """Allocate next sequential numeric run index like UAARG debug_analysis."""
        existing = os.listdir(self.base_dir)
        numeric_dirs = []
        for d in existing:
            full_path = os.path.join(self.base_dir, d)
            if os.path.isdir(full_path) and d.isdigit():
                numeric_dirs.append(int(d))
        return (max(numeric_dirs) + 1) if numeric_dirs else 0

    def log_event(
        self,
        seq: int,
        detection: Optional[Dict[str, Any]] = None,
        location: Optional[Dict[str, float]] = None,
        altitude_m: float = 15.0,
        mode: str = "PATROL",
        link_status: Optional[Dict[str, Any]] = None,
        timestamp_ms: Optional[int] = None,
        image: Optional[Any] = None,
        bbox: Optional[Tuple[int, int, int, int]] = None,
    ) -> Dict[str, Any]:
        """Record raw image, annotated image, and structured telemetry JSON."""
        now_ms = int(time.time() * 1000) if timestamp_ms is None else timestamp_ms

        loc_data = location or {"lat": 53.5232, "lon": -113.5263}
        det_data = detection or {"cls": "car", "conf": 0.90, "count": 1}
        link_data = link_status or {"up": True, "buffered": 0, "rtt_ms": 12.0}

        snapshot: Dict[str, Any] = {
            "seq": int(seq),
            "timestamp_ms": now_ms,
            "location": {
                "lat": round(float(loc_data.get("lat", 0.0)), 6),
                "lon": round(float(loc_data.get("lon", 0.0)), 6),
            },
            "altitude_m": round(float(altitude_m), 1),
            "mode": str(mode).upper(),
            "detection": det_data,
            "link": link_data,
        }

        with self._lock:
            # 1. Save images
            img_path = os.path.join(self.img_dir, f"{seq}.png")
            bb_path = os.path.join(self.bb_dir, f"{seq}.png")
            self._save_images(img_path, bb_path, image, bbox, det_data)

            # 2. Save JSON snapshot
            json_path = os.path.join(self.run_dir, f"{seq}.json")
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(snapshot, f, indent=2)

            self.logged_seqs.append(seq)

        return snapshot

    def _save_images(
        self,
        img_path: str,
        bb_path: str,
        image: Optional[Any],
        bbox: Optional[Tuple[int, int, int, int]],
        detection: Dict[str, Any],
    ) -> None:
        """Save raw and annotated images using PIL."""
        if not _HAS_PIL:
            # Create lightweight dummy binary files if PIL not installed
            with open(img_path, "wb") as f:
                f.write(b"RAW_IMAGE_DATA")
            with open(bb_path, "wb") as f:
                f.write(b"ANNOTATED_IMAGE_DATA")
            return

        pil_img: Image.Image
        if image is None:
            # Create synthetic 320x240 dark tactical frame
            pil_img = Image.new("RGB", (320, 240), color=(25, 30, 36))
            draw = ImageDraw.Draw(pil_img)
            draw.text((10, 10), "ARCDRONE TACTICAL VIEW", fill=(0, 255, 128))
            draw.text((10, 30), f"SEQ: {os.path.basename(img_path)}", fill=(200, 200, 200))
        elif isinstance(image, Image.Image):
            pil_img = image.copy()
        else:
            # Assume numpy array or compatible buffer
            try:
                pil_img = Image.fromarray(image)
            except Exception:
                pil_img = Image.new("RGB", (320, 240), color=(30, 30, 30))

        # Save raw frame
        pil_img.save(img_path, "PNG")

        # Create annotated copy
        annotated = pil_img.copy()
        draw_bb = ImageDraw.Draw(annotated)

        # Determine bounding box coordinates (defaulting to center box if none provided)
        box = bbox if bbox is not None else (70, 60, 250, 180)
        draw_bb.rectangle(box, outline=(255, 60, 60), width=2)

        cls_name = str(detection.get("cls", "target"))
        conf_val = detection.get("conf", 0.0)
        label = f"{cls_name} {conf_val:.2f}" if isinstance(conf_val, float) else cls_name
        draw_bb.text((box[0] + 4, box[1] + 4), label, fill=(255, 255, 60))

        # Save annotated frame
        annotated.save(bb_path, "PNG")

    def read_event(self, seq: int) -> Optional[Dict[str, Any]]:
        """Read back a stored JSON snapshot by sequence number."""
        json_path = os.path.join(self.run_dir, f"{seq}.json")
        if not os.path.exists(json_path):
            return None
        with open(json_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def list_events(self) -> List[Dict[str, Any]]:
        """Read back all event snapshots in this session, sorted by seq."""
        events = []
        with self._lock:
            for s in sorted(self.logged_seqs):
                ev = self.read_event(s)
                if ev:
                    events.append(ev)
        return events
