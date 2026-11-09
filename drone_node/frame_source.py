"""OpenCV video/camera or VisDrone-VID folders of sequential JPEGs."""
from __future__ import annotations

import glob
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np


def _list_sequence_images(folder: str) -> List[str]:
    paths: List[str] = []
    for pattern in ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG"):
        paths.extend(glob.glob(os.path.join(folder, pattern)))
    return sorted(paths)


def is_image_sequence_path(path: str) -> bool:
    if not os.path.isdir(path):
        return False
    return len(_list_sequence_images(path)) > 0


class ImageSequenceReader:
    """VisDrone-VID style uav.../0000001.jpg folders; loops at end."""

    def __init__(self, folder: str, loop: bool = True) -> None:
        self.folder = os.path.abspath(folder)
        self.loop = loop
        self.paths = _list_sequence_images(self.folder)
        self.index = 0

    def isOpened(self) -> bool:
        return len(self.paths) > 0

    def is_live_camera(self) -> bool:
        return False

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if not self.paths:
            return False, None
        path = self.paths[self.index]
        frame = cv2.imread(path)
        if frame is None:
            return False, None
        self.index += 1
        if self.index >= len(self.paths):
            self.index = 0 if self.loop else len(self.paths) - 1
        return True, frame

    def rewind(self) -> None:
        self.index = 0

    def release(self) -> None:
        pass


class OpenCVVideoReader:
    def __init__(self, source, use_avfoundation: bool = False) -> None:
        self._live = isinstance(source, int)
        if self._live and use_avfoundation and hasattr(cv2, "CAP_AVFOUNDATION"):
            self.cap = cv2.VideoCapture(source, cv2.CAP_AVFOUNDATION)
        else:
            self.cap = cv2.VideoCapture(source)

    def isOpened(self) -> bool:
        return self.cap.isOpened()

    def is_live_camera(self) -> bool:
        return self._live

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        return self.cap.read()

    def rewind(self) -> None:
        if not self._live:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def release(self) -> None:
        if self.cap.isOpened():
            self.cap.release()


def open_frame_source(video_arg: str) -> object:
    """Return a reader with isOpened(), read(), rewind(), release(), is_live_camera()."""
    raw = str(video_arg).strip()
    if raw.isdigit():
        return OpenCVVideoReader(int(raw), use_avfoundation=True)
    path = os.path.abspath(os.path.expanduser(raw))
    if is_image_sequence_path(path):
        count = len(_list_sequence_images(path))
        print("[MAIN] VisDrone-VID sequence: %s (%d frames, loops)" % (path, count))
        return ImageSequenceReader(path, loop=True)
    return OpenCVVideoReader(path, use_avfoundation=False)
