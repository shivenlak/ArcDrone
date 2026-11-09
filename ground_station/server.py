"""Ground station server: connects the radio link (protocol.link.Link) to the browser.

    python -m ground_station.server --port /dev/ttyUSB0
    python -m ground_station.server --port <fake_bridge port>     # no hardware

Open http://localhost:8000. The WebSocket schema is in CLAUDE.md ("ws://localhost:8000/ws").

radio -> browser: Link callbacks (link thread) -> asyncio queue -> JSON to every client
browser -> radio: {"action": "command", ...} -> link.send(COMMAND), shown PENDING until delivered
"""

import argparse
import asyncio
import json
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from drone_node.scene_codec import SceneCodec
from protocol import packets as p
from protocol.link import Link

HERE = Path(__file__).parent
REPO_ROOT = HERE.parent
STATIC = HERE / "static"
FALLBACK_PAGE = HERE / "playground" / "playground.html"  # used until static/index.html exists
DELAYED_AFTER_MS = 3000  # alert older than this on arrival was buffered during link loss
CMD_CODES = {name: code for code, name in p.CMD_NAMES.items()}


def unwrap_ts(ts_ms):
    """The wire ts_ms is the low 32 bits of epoch ms. Return (full epoch ms, age in ms)."""
    age = (p.now_ms() - ts_ms) & 0xFFFFFFFF
    return int(time.time() * 1000) - age, age


class Station:
    def __init__(self, port):
        self.port = port
        self.clients = set()
        self.pending = {}      # command seq -> its cmd_status fields, until delivered
        self.mode = None       # last drone mode from a heartbeat
        self.drone_buffered = 0
        self.link = None
        self.loop = None
        self.queue = None
        self.scene_decoder = SceneCodec(dst_size=(720, 540))
        self.path_override = "AUTO"
        self.sender_lora_only = False
        self.source_info = {
            "known": False,
            "is_video": False,
            "filename": "",
            "last_espnow_at": 0.0,
        }
        self.active_dl = None
        self.completed_dl = {}

    # --- startup / shutdown ---

    def start(self):
        self.loop = asyncio.get_running_loop()
        self.queue = asyncio.Queue()
        self.link = Link(self.port, on_packet=self._on_packet, on_event=self._on_event).start()
        return [
            asyncio.create_task(self._consume()),
            asyncio.create_task(self._stats_loop()),
        ]

    def stop(self):
        if self.link is not None:
            self.link.stop()

    def is_espnow_available(self) -> bool:
        """True iff ESP-NOW (2.4 GHz) is active on both sides and not overridden to LoRa-only."""
        if self.link is None or not self.link.up or not self.link.is_espnow_live():
            return False
        if self.path_override == "LORA_ONLY" or self.sender_lora_only:
            return False
        last_at = self.source_info.get("last_espnow_at", 0.0)
        if last_at <= 0.0 or (time.monotonic() - last_at) > 4.5:
            return False
        return True

    def source_info_msg(self):
        espnow_ok = self.is_espnow_available()
        is_video = bool(self.source_info.get("is_video"))
        known = bool(self.source_info.get("known")) and (
            (time.monotonic() - self.source_info.get("last_espnow_at", 0.0)) < 4.5
        )
        return {
            "event": "source_info",
            "known": known,
            "is_video": is_video,
            "filename": self.source_info.get("filename", ""),
            "espnow_available": espnow_ok,
            "can_download": bool(espnow_ok and known and is_video),
        }

    def _resolve_video_path(self, filename: str, sender_filepath: str = ""):
        candidates = []
        if sender_filepath:
            candidates.append(Path(sender_filepath))
        if filename:
            bname = os.path.basename(filename)
            candidates.extend([
                STATIC / "clips" / bname,
                REPO_ROOT / "data" / "clips" / bname,
                REPO_ROOT / "data" / bname,
                REPO_ROOT / "drone_node" / "static" / "clips" / bname,
            ])
        for cand in candidates:
            try:
                if cand.is_file():
                    return cand
            except Exception:
                pass
        return None

    # --- radio -> browser ---

    def _on_packet(self, packet):  # link thread: hand over to the event loop, nothing else
        self.loop.call_soon_threadsafe(self.queue.put_nowait, ("packet", packet))

    def _on_event(self, name, **info):  # link thread
        self.loop.call_soon_threadsafe(self.queue.put_nowait, ("event", (name, info)))

    async def _consume(self):
        while True:
            kind, item = await self.queue.get()
            try:
                if kind == "packet":
                    await self._handle_packet(item)
                else:
                    await self._handle_event(*item)
            except Exception as exc:  # one bad message must not stop the bridge
                print("server: error handling", kind, item, exc)

    async def _handle_packet(self, packet):
        if packet["type"] == p.DETECTION:
            ts_ms, age = unwrap_ts(packet["ts_ms"])
            cls = packet["cls"]
            await self.broadcast({
                "event": "alert", "seq": packet["seq"],
                "cls": p.CLASS_NAMES[cls] if cls < len(p.CLASS_NAMES) else str(cls),
                "conf": round(packet["conf"], 2), "count": packet["count"],
                "lat": packet["lat"], "lon": packet["lon"],
                "ts_ms": ts_ms, "delayed": age > DELAYED_AFTER_MS,
                "raw": packet["wire"],
            })
        elif packet["type"] == p.HEARTBEAT:
            self.mode = p.MODE_NAMES.get(packet["mode"], "PATROL")
            self.drone_buffered = packet["buffered"]
            await self.broadcast(self.drone_msg())
        elif packet["type"] in (0xAD, 0xAE) and "scene_bytes" in packet:
            frame = self.scene_decoder.decode_frame(packet["scene_bytes"], canvas_w=720, canvas_h=540)
            if frame is not None:
                if "sender_lora_only" in frame:
                    self.sender_lora_only = bool(frame["sender_lora_only"])
                await self.broadcast({
                    "event": "scene",
                    "frame": frame,
                    "radio": self.link.summary().get("radio"),
                })
        elif packet["type"] == 0xEF:
            sub = packet.get("subtype")
            if sub == "source_info":
                self.source_info = {
                    "known": True,
                    "is_video": bool(packet.get("is_video")),
                    "filename": packet.get("filename", ""),
                    "last_espnow_at": time.monotonic(),
                }
                await self.broadcast(self.source_info_msg())
            elif sub == "dl_chunk":
                self.source_info["last_espnow_at"] = time.monotonic()
                if not self.is_espnow_available():
                    if self.active_dl is not None:
                        self.active_dl = None
                        await self.broadcast({
                            "event": "video_download",
                            "status": "error",
                            "error": "ESP-NOW link offline — download strictly requires ESP-NOW",
                        })
                    return
                if self.active_dl is None or self.active_dl.get("req_id") != packet.get("req_id"):
                    return
                idx = int(packet.get("idx", 0))
                total = max(1, int(packet.get("total", 1)))
                self.active_dl["total"] = total
                self.active_dl["parts"][idx] = packet.get("payload", b"")
                got = len(self.active_dl["parts"])
                if got < total:
                    pct = min(99, int(round(got * 100.0 / total)))
                    await self.broadcast({
                        "event": "video_download",
                        "status": "downloading",
                        "progress": pct,
                        "received_chunks": got,
                        "total_chunks": total,
                        "filename": self.active_dl.get("filename", "source_video.mp4"),
                    })
                else:
                    full_blob = b"".join(self.active_dl["parts"].get(i, b"") for i in range(total))
                    meta = {}
                    raw_sample = full_blob
                    if len(full_blob) >= 2:
                        mlen = int.from_bytes(full_blob[:2], "big")
                        if 2 + mlen <= len(full_blob):
                            try:
                                meta = json.loads(full_blob[2 : 2 + mlen].decode("utf-8"))
                                raw_sample = full_blob[2 + mlen :]
                            except Exception:
                                pass
                    fname = meta.get("filename") or self.active_dl.get("filename") or "source_video.mp4"
                    s_path = meta.get("filepath", "")
                    resolved = self._resolve_video_path(fname, s_path)
                    token = f"espnow_{int(time.time() * 1000)}_{packet.get('req_id', 1)}"
                    self.completed_dl[token] = {
                        "filename": os.path.basename(fname),
                        "filepath": resolved,
                        "raw_bytes": raw_sample,
                        "completed_at": time.monotonic(),
                    }
                    self.active_dl = None
                    await self.broadcast({
                        "event": "video_download",
                        "status": "complete",
                        "progress": 100,
                        "received_chunks": total,
                        "total_chunks": total,
                        "filename": os.path.basename(fname),
                        "url": f"/api/download-source-video?token={token}",
                    })

    async def _handle_event(self, name, info):
        if name in ("link_up", "link_down"):
            await self.broadcast({"event": "link", "up": name == "link_up"})
            await self.broadcast(self.source_info_msg())
        elif name == "delivered":
            cmd = self.pending.pop(info["seq"], None)  # only our COMMANDs are in pending
            if cmd is not None:
                await self.broadcast(dict(cmd, event="cmd_status", status="CONFIRMED"))

    async def _stats_loop(self):
        while True:
            await asyncio.sleep(1)
            await self.broadcast(dict(self.link.summary(), event="stats"))
            await self.broadcast(self.source_info_msg())
            if self.active_dl is not None:
                if not self.is_espnow_available() or (time.monotonic() - self.active_dl["started_at"]) > 8.0:
                    self.active_dl = None
                    await self.broadcast({
                        "event": "video_download",
                        "status": "error",
                        "error": "ESP-NOW link unavailable or transfer timed out",
                    })

    # --- browser -> radio ---

    async def handle_browser(self, msg):
        if msg.get("action") == "radio_path":
            self.path_override = msg.get("path", "AUTO")
            if self.path_override == "LORA_ONLY" and self.active_dl is not None:
                self.active_dl = None
                await self.broadcast({
                    "event": "video_download",
                    "status": "error",
                    "error": "Switched to LoRa-Only failover — download strictly requires ESP-NOW",
                })
            await self.broadcast({"event": "radio_path", "path": self.path_override})
            await self.broadcast(self.source_info_msg())
            return
        if msg.get("action") == "download_video":
            if not self.is_espnow_available():
                await self.broadcast({
                    "event": "video_download",
                    "status": "error",
                    "error": "ESP-NOW (2.4 GHz) is offline — source video download strictly requires ESP-NOW",
                })
                return
            if not self.source_info.get("known") or not self.source_info.get("is_video"):
                await self.broadcast({
                    "event": "video_download",
                    "status": "error",
                    "error": "Sender is using a live webcam — source video download is only available for video files",
                })
                return
            req_id = self.link.request_video_download()
            if req_id is None:
                await self.broadcast({
                    "event": "video_download",
                    "status": "error",
                    "error": "ESP-NOW radio path unavailable",
                })
                return
            fname = self.source_info.get("filename") or "source_video.mp4"
            self.active_dl = {
                "req_id": req_id,
                "total": 1,
                "parts": {},
                "started_at": time.monotonic(),
                "filename": fname,
            }
            await self.broadcast({
                "event": "video_download",
                "status": "downloading",
                "progress": 0,
                "received_chunks": 0,
                "total_chunks": 20,
                "filename": fname,
            })
            return
        if msg.get("action") != "command" or msg.get("cmd") not in CMD_CODES:
            return
        ref_seq = int(msg.get("ref_seq", 0)) & 0xFFFF
        seq = self.link.send(p.COMMAND, cmd=CMD_CODES[msg["cmd"]], ref_seq=ref_seq)
        cmd = {"cmd_id": seq, "cmd": msg["cmd"], "ref_seq": ref_seq}
        self.pending[seq] = cmd  # no await since send(), so "delivered" cannot beat this
        await self.broadcast(dict(cmd, event="cmd_status", status="PENDING"))

    # --- websocket clients ---

    def drone_msg(self):
        return {"event": "drone", "mode": self.mode, "buffered": self.drone_buffered}

    async def broadcast(self, msg):
        text = json.dumps(msg)
        for ws in list(self.clients):
            try:
                await ws.send_text(text)
            except Exception:
                self.clients.discard(ws)

    async def welcome(self, ws):
        """Current state for a client that just connected."""
        await ws.send_text(json.dumps({"event": "link", "up": self.link.up}))
        await ws.send_text(json.dumps(dict(self.link.summary(), event="stats")))
        await ws.send_text(json.dumps(self.source_info_msg()))
        if self.mode is not None:
            await ws.send_text(json.dumps(self.drone_msg()))
        for cmd in self.pending.values():
            await ws.send_text(json.dumps(dict(cmd, event="cmd_status", status="PENDING")))


def make_app(port):
    station = Station(port)

    @asynccontextmanager
    async def lifespan(app):
        tasks = station.start()
        yield
        for task in tasks:
            task.cancel()
        station.stop()

    app = FastAPI(lifespan=lifespan)

    @app.get("/api/download-source-video")
    async def download_source_video(token: str = ""):
        if not station.is_espnow_available():
            raise HTTPException(status_code=403, detail="ESP-NOW is not available")
        rec = station.completed_dl.get(token)
        if not rec:
            raise HTTPException(status_code=404, detail="Invalid or expired ESP-NOW download token")
        fname = rec.get("filename") or "source_video.mp4"
        fpath = rec.get("filepath")
        if fpath and Path(fpath).is_file():
            return FileResponse(
                path=fpath,
                media_type="video/mp4",
                filename=fname,
            )
        raw_bytes = rec.get("raw_bytes") or b""
        return Response(
            content=raw_bytes,
            media_type="video/mp4",
            headers={"Content-Disposition": f'attachment; filename="{fname}"'},
        )

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        station.clients.add(websocket)
        try:
            await station.welcome(websocket)
            while True:
                try:
                    msg = json.loads(await websocket.receive_text())
                except ValueError:
                    continue
                if isinstance(msg, dict):
                    await station.handle_browser(msg)
        except WebSocketDisconnect:
            pass
        finally:
            station.clients.discard(websocket)

    if (STATIC / "index.html").exists():
        app.mount("/", StaticFiles(directory=STATIC, html=True), name="static")
    else:
        @app.get("/")
        async def fallback():
            return FileResponse(FALLBACK_PAGE)

    return app


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", required=True, help="serial port of the ground ESP32 (or a fake_bridge port)")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args()
    uvicorn.run(make_app(args.port), host=args.host, port=args.http_port)
