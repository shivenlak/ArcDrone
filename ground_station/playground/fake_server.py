"""Playground: a FAKE ground station you can click around in. No radio needed.

Run from the repo root:
    python -m ground_station.playground.fake_server
Then open http://localhost:8000

It speaks the real WebSocket schema from CLAUDE.md, so the same messages will
come from the real server.py later. A fake drone lives inside this file.
The "Cut link" button is playground-only; it is not part of the contract.
"""
import argparse
import asyncio
import json
import random
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from protocol import packets as p

PAGE = Path(__file__).with_name("playground.html")
MODE_AFTER_CMD = {"HOLD": "HOLD", "RESUME": "PATROL", "RTB": "RTB"}  # OPERATOR_ACK: unchanged
FAKE_CLASSES = ["pedestrian", "car", "van", "truck"]
assert all(c in p.CLASS_NAMES for c in FAKE_CLASSES)


def now_ms():
    return int(time.time() * 1000)


class FakeStation:
    def __init__(self, alert_every):
        self.alert_every = alert_every
        self.clients = set()
        self.link_up = True
        self.mode = "PATROL"
        self.buffer = []          # alerts the fake drone held back during link loss
        self.pending = []         # commands waiting for the link to come back
        self.alert_seq = random.randint(1000, 9000)
        self.cmd_seq = random.randint(100, 900)
        self.sent = self.delivered = self.retries = 0

    async def broadcast(self, msg):
        text = json.dumps(msg)
        for ws in list(self.clients):
            try:
                await ws.send_text(text)
            except Exception:
                self.clients.discard(ws)

    def stats_msg(self):
        return {"event": "stats", "up": self.link_up, "buffered": len(self.buffer),
                "sent": self.sent, "delivered": self.delivered, "retries": self.retries,
                "duplicates": 0, "bad_packets": 0, "dropped": 0,
                "rtt_ms_avg": round(random.uniform(8, 11), 1), "rtt_ms_p95": round(random.uniform(13, 16), 1)}

    def drone_msg(self):
        return {"event": "drone", "mode": self.mode, "buffered": len(self.buffer)}

    async def deliver_alert(self, alert):
        alert = dict(alert, delayed=(now_ms() - alert["ts_ms"]) > 3000)
        self.delivered += 1
        await self.broadcast(alert)

    async def alert_loop(self):
        while True:
            await asyncio.sleep(self.alert_every)
            self.alert_seq += 1
            self.sent += 1
            alert = {"event": "alert", "seq": self.alert_seq, "cls": random.choice(FAKE_CLASSES),
                     "conf": round(random.uniform(0.5, 0.97), 2), "count": random.randint(1, 4),
                     "lat": round(53.52 + random.uniform(-0.002, 0.002), 5),
                     "lon": round(-113.52 + random.uniform(-0.002, 0.002), 5), "ts_ms": now_ms()}
            if self.link_up:
                await self.deliver_alert(alert)
            elif len(self.buffer) < 64:
                self.buffer.append(alert)

    async def heartbeat_loop(self):
        while True:
            await asyncio.sleep(1)
            if self.link_up:
                await self.broadcast(self.drone_msg())
            await self.broadcast(self.stats_msg())

    async def set_link(self, up):
        if up == self.link_up:
            return
        self.link_up = up
        await self.broadcast({"event": "link", "up": up})
        if up:
            self.retries += len(self.buffer) + len(self.pending)
            for alert in self.buffer:          # oldest first, original ts_ms kept
                await self.deliver_alert(alert)
                await asyncio.sleep(0.15)
            self.buffer.clear()
            for cmd in self.pending:
                await self.confirm(cmd)
            self.pending.clear()
            await self.broadcast(self.drone_msg())

    async def confirm(self, cmd):
        self.mode = MODE_AFTER_CMD.get(cmd["cmd"], self.mode)
        self.delivered += 1
        await self.broadcast(dict(cmd, event="cmd_status", status="CONFIRMED"))
        await self.broadcast(self.drone_msg())

    async def command(self, name, ref_seq):
        self.cmd_seq += 1
        self.sent += 1
        cmd = {"cmd_id": self.cmd_seq, "cmd": name, "ref_seq": ref_seq}
        await self.broadcast(dict(cmd, event="cmd_status", status="PENDING"))
        if self.link_up:
            await asyncio.sleep(0.3)           # pretend radio + drone ACK time
            await self.confirm(cmd)
        else:
            self.pending.append(cmd)


def make_app(alert_every):
    app = FastAPI()
    station = FakeStation(alert_every)

    @app.on_event("startup")
    async def start_loops():
        asyncio.create_task(station.alert_loop())
        asyncio.create_task(station.heartbeat_loop())

    @app.get("/")
    async def index():
        return HTMLResponse(PAGE.read_text(encoding="utf-8"))

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        station.clients.add(websocket)
        await websocket.send_text(json.dumps({"event": "link", "up": station.link_up}))
        await websocket.send_text(json.dumps(station.drone_msg()))
        try:
            while True:
                msg = json.loads(await websocket.receive_text())
                if msg.get("action") == "command" and msg.get("cmd") in p.CMD_NAMES.values():
                    await station.command(msg["cmd"], int(msg.get("ref_seq", 0)))
                elif msg.get("action") == "fake_link":      # playground only
                    await station.set_link(bool(msg["up"]))
        except WebSocketDisconnect:
            pass
        finally:
            station.clients.discard(websocket)

    return app


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--alert-every", type=float, default=2.0, help="seconds between fake alerts")
    args = parser.parse_args()
    uvicorn.run(make_app(args.alert_every), host="127.0.0.1", port=args.port)
