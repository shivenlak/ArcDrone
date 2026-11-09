"""Resolve default VisDrone / demo clip paths under data/ (not the webcam)."""
from __future__ import annotations

import os
from typing import List, Optional

from drone_node.frame_source import is_image_sequence_path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(_REPO_ROOT, "data")

# VisDrone-VID (Task 2) — extract under data/, e.g. data/VisDrone2019-VID-val/sequences/...
VID_DATASET_DIRS = (
    "VisDrone2019-VID-val",
    "VisDrone2019-VID-test-dev",
    "visdrone_vid",
)


def _first_vid_sequence() -> Optional[str]:
    for root_name in VID_DATASET_DIRS:
        seq_root = os.path.join(DATA_DIR, root_name, "sequences")
        if not os.path.isdir(seq_root):
            continue
        for name in sorted(os.listdir(seq_root)):
            path = os.path.join(seq_root, name)
            if is_image_sequence_path(path):
                return path
    return None


# Same names as Manan's test suite + the main aerial sample used across scripts.
DEMO_CLIP_NAMES: List[str] = [
    "test_cases/uav0000137_00458_v.mp4",
    "clips/uav0000137_00458_v.mp4",
    "clips/uav0000268_05773_v.mp4",
    "clips/uav0000086_00000_v.mp4",
    "clips/uav0000182_00000_v.mp4",
    "clips/uav0000117_02622_v.mp4",
    "clips/uav0000339_00001_v.mp4",
    "clips/uav0000137_00458_v.mp4",
    "demo_aerial.mp4",
    "visdrone_aerial_sample.mp4",
    "case6_aerial_traffic_1080p.mp4",
    "case1_urban_intersection.mp4",
    "case2_dynamic_flight.mp4",
    "case3_perimeter_airport.mp4",
    "case4_high_altitude_sahi.mp4",
    "case5_parking_staging.mp4",
]


def resolve_demo_video(explicit: Optional[str] = None) -> str:
    """Return a path or camera index string to pass to drone_node.main --video."""
    if explicit is not None and str(explicit).strip() != "":
        raw = str(explicit).strip()
        if raw.isdigit():
            return raw
        path = os.path.abspath(os.path.expanduser(raw))
        if os.path.isfile(path):
            return path
        if is_image_sequence_path(path):
            return path
        under_data = os.path.join(DATA_DIR, raw)
        if os.path.isfile(under_data):
            return under_data
        under_seq = os.path.join(DATA_DIR, "VisDrone2019-VID-val", "sequences", raw)
        if is_image_sequence_path(under_seq):
            return under_seq
        raise SystemExit(
            "Video/sequence not found: %s\nPut MP4 or VisDrone-VID folders under %s/"
            % (raw, DATA_DIR)
        )

    env = os.environ.get("ARCDRONE_DEMO_VIDEO", "").strip()
    if env:
        if env.isdigit():
            return env
        if os.path.isfile(env):
            return os.path.abspath(env)
        under_data = os.path.join(DATA_DIR, env)
        if os.path.isfile(under_data):
            return under_data

    for name in DEMO_CLIP_NAMES:
        path = os.path.join(DATA_DIR, name)
        if os.path.isfile(path):
            return path

    vid = _first_vid_sequence()
    if vid is not None:
        return vid

    raise SystemExit(
        "No demo footage in %s/.\n"
        "Add an MP4 (%s) or extract VisDrone-VID val into "
        "data/VisDrone2019-VID-val/sequences/<uav...>/\n"
        "Or run with --video 0 for webcam."
        % (DATA_DIR, ", ".join(DEMO_CLIP_NAMES[:2]))
    )
