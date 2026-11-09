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
import time
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from drone_node.scene_codec import SceneCodec
from protocol import packets as p
from protocol.link import Link

HERE = Path(__file__).parent
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

    # --- startup / shutdown ---

    def start(self):
        self.loop = asyncio.get_running_loop()
        self.queue = asyncio.Queue()
        self.link = Link(self.port, on_packet=self._on_packet, on_event=self._on_event).start()
        return [asyncio.create_task(self._consume()), asyncio.create_task(self._stats_loop())]

    def stop(self):
        if self.link is not None:
            self.link.stop()

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
                await self.broadcast({"event": "scene", "frame": frame})

    async def _handle_event(self, name, info):
        if name in ("link_up", "link_down"):
            await self.broadcast({"event": "link", "up": name == "link_up"})
        elif name == "delivered":
            cmd = self.pending.pop(info["seq"], None)  # only our COMMANDs are in pending
            if cmd is not None:
                await self.broadcast(dict(cmd, event="cmd_status", status="CONFIRMED"))

    async def _stats_loop(self):
        while True:
            await asyncio.sleep(1)
            await self.broadcast(dict(self.link.summary(), event="stats"))

    # --- browser -> radio ---

    async def handle_browser(self, msg):
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
