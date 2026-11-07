# Browser scene reconstructor (Gemini UI)

## 1. HTML

`drone_node/static/reconstructor.html` — saved from Gemini, patched for **2.0 kbps** (not 1.6/0.4).

## 2. Run stack

```bash
pip install websockets   # or pip install -r requirements.txt
python -m drone_node.scene_ws_relay
# open http://127.0.0.1:8080/reconstructor.html

python -m protocol.fake_bridge
python -m drone_node.main --port <PORT_A> --video 0 --fps 2 --scene-hz 12
```

Do **not** run `python -m drone_node.reconstructor` on port **9999** at the same time (OpenCV binds the same UDP port).

## 4. Live vs simulator

- **LIVE:** WebSocket `ws://127.0.0.1:8765` when relay is running and `main` is sending UDP.
- **SIMULATOR:** HTML’s built-in 12 Hz fake actors when WS is down (fine for UI demo only).
