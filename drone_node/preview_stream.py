"""Tiny MJPEG HTTP preview of the drone camera / HUD for the ground dashboard.

Dev/demo only (LAN). Not the low-bandwidth radio path — that stays SceneCodec UDP.
"""
from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import numpy as np


class PreviewStream:
    """Thread-safe latest-JPEG holder + HTTP server on /video.mjpg and /."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8081, jpeg_quality: int = 60) -> None:
        self.host = host
        self.port = int(port)
        self.jpeg_quality = int(jpeg_quality)
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def update(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr is None or frame_bgr.size == 0:
            return
        ok, buf = cv2.imencode(
            ".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return
        with self._lock:
            self._jpeg = buf.tobytes()

    def _snapshot(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def start(self) -> "PreviewStream":
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # quiet
                return

            def _send_cors(self):
                self.send_header("Access-Control-Allow-Origin", "*")

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    body = (
                        b"<html><body style='margin:0;background:#000'>"
                        b"<img src='/video.mjpg' style='width:100%;height:100%;object-fit:contain'/>"
                        b"</body></html>"
                    )
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self._send_cors()
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path != "/video.mjpg":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self._send_cors()
                self.end_headers()
                try:
                    while True:
                        jpeg = stream._snapshot()
                        if jpeg is None:
                            import time as _t

                            _t.sleep(0.05)
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            ("Content-Length: %d\r\n\r\n" % len(jpeg)).encode()
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        import time as _t

                        _t.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    return

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        print("[PREVIEW] MJPEG on http://127.0.0.1:%d/video.mjpg" % self.port)
        return self

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
