"""UDP scene packets (:9999) -> WebSocket JSON (:8765) for the browser reconstructor UI.

Run (do NOT run OpenCV reconstructor on 9999 at the same time):
    python -m drone_node.scene_ws_relay

Then open http://127.0.0.1:8080/reconstructor.html
Drone: python -m drone_node.main ...  (default --scene-port 9999)
"""
from __future__ import annotations

import asyncio
import json
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Set

import websockets
from websockets.server import serve

from drone_node.scene_codec import MAGIC_V1, MAGIC_V2, SCENE_HZ_DEFAULT, SceneCodec

FOV_W, FOV_H = 720, 540
UDP_PORT = 9999
WS_PORT = 8765
HTTP_PORT = 8080

STATIC_DIR = Path(__file__).resolve().parent / "static"
CLIENTS: Set[object] = set()
_DECODER = SceneCodec(dst_size=(FOV_W, FOV_H))


def _decode_udp(data: bytes):
    if len(data) < 4 or data[0] not in (MAGIC_V1, MAGIC_V2):
        return None
    return _DECODER.decode_frame(data, canvas_w=FOV_W, canvas_h=FOV_H)


async def _broadcast(frame: dict) -> None:
    if not CLIENTS:
        return
    msg = json.dumps(frame)
    stale = []
    for ws in list(CLIENTS):
        try:
            await ws.send(msg)
        except Exception:
            stale.append(ws)
    for ws in stale:
        CLIENTS.discard(ws)


class SceneUdpProtocol(asyncio.DatagramProtocol):
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def datagram_received(self, data: bytes, addr) -> None:
        frame = _decode_udp(data)
        if frame is None:
            return
        msg_types = {1: "HEARTBEAT", 2: "KEYFRAME", 3: "DELTA"}
        if len(data) >= 2 and data[0] == 0xAE:
            mt = (data[1] >> 4) & 0x0F
            frame["msg_type"] = mt
            frame["msg_name"] = msg_types.get(mt, "FRAME")
        elif data[0] == 0xAD:
            frame["msg_name"] = "V1"
        frame["wire_hex"] = " ".join(f"{b:02X}" for b in data)
        asyncio.run_coroutine_threadsafe(_broadcast(frame), self._loop)


async def _ws_handler(websocket) -> None:
    CLIENTS.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        CLIENTS.discard(websocket)


def _serve_http() -> None:
    static = str(STATIC_DIR)

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=static, **kwargs)

    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    httpd.serve_forever()


async def _async_main() -> None:
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        lambda: SceneUdpProtocol(loop),
        local_addr=("0.0.0.0", UDP_PORT),
    )
    async with serve(_ws_handler, "127.0.0.1", WS_PORT):
        print(
            "[SCENE RELAY] UDP :%d -> ws://127.0.0.1:%d (~%.0f Hz from drone)"
            % (UDP_PORT, WS_PORT, SCENE_HZ_DEFAULT)
        )
        print("[SCENE RELAY] UI: http://127.0.0.1:%d/reconstructor.html" % HTTP_PORT)
        await asyncio.Future()


def main() -> None:
    if not (STATIC_DIR / "reconstructor.html").is_file():
        print("[WARN] Missing %s/reconstructor.html" % STATIC_DIR)
    t = threading.Thread(target=_serve_http, daemon=True)
    t.start()
    asyncio.run(_async_main())


if __name__ == "__main__":
    main()
