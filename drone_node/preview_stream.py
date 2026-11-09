"""Tiny HTTP preview of the drone's HUD video and live numbers, for the ground dashboard.

Wi-Fi / LAN only. This is NOT the radio path: alerts and commands go over ESP-NOW/LoRa.

    /video.mjpg   the HUD frames (YOLO boxes drawn) as MJPEG, each frame sent once
    /stats.json   drone-side numbers (YOLO ms, fps, objects, alerts, ...) + bytes this server sent
    /             tiny page, also used by the dashboard to check Wi-Fi reachability
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

import cv2
import numpy as np


class PreviewStream:
    """Thread-safe latest-JPEG + stats holder, served over HTTP."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8081, jpeg_quality: int = 60) -> None:
        self.host = host
        self.port = int(port)
        self.jpeg_quality = int(jpeg_quality)
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._frame_id = 0
        self._stats: Dict[str, Any] = {}
        self.bytes_sent = 0  # video bytes written to all viewers: the Wi-Fi cost of the feed
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def update(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr is None or frame_bgr.size == 0:
            return
        ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        with self._lock:
            self._jpeg = buf.tobytes()
            self._frame_id += 1

    def update_stats(self, stats: Dict[str, Any]) -> None:
        with self._lock:
            self._stats = dict(stats)

    def _snapshot(self):
        with self._lock:
            return self._jpeg, self._frame_id

    def _stats_json(self) -> bytes:
        with self._lock:
            body = dict(self._stats, video_bytes_sent=self.bytes_sent, t=time.time())
        return json.dumps(body).encode()

    def start(self) -> "PreviewStream":
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # quiet
                return

            def _small(self, body: bytes, ctype: str) -> None:
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    self._small(b"<html><body style='margin:0;background:#000'><img src='/video.mjpg' "
                                b"style='width:100%;height:100%;object-fit:contain'/></body></html>", "text/html")
                    return
                if path == "/stats.json":
                    self._small(stream._stats_json(), "application/json")
                    return
                if path != "/video.mjpg":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                last_id = -1
                try:
                    while True:
                        jpeg, frame_id = stream._snapshot()
                        if jpeg is None or frame_id == last_id:
                            time.sleep(0.02)  # only send a frame once, when it is new
                            continue
                        last_id = frame_id
                        head = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg)
                        self.wfile.write(head + jpeg + b"\r\n")
                        stream.bytes_sent += len(head) + len(jpeg) + 2
                except (BrokenPipeError, ConnectionResetError):
                    return

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
