# Scene reconstructor — how to run and check it

The drone sends a compact **scene vector stream** (SceneCodec v2, magic byte `0xAE`) over UDP at 12 Hz, capped at **2.0 kbps**. No video goes over the link. The ground side rebuilds the scene in a browser page that shows:

- a synthetic camera view with boxes, trails, and direction arrows
- a **live activity panel** that says what each target is doing ("Moving WEST · FAST · 3.4s", "heading to MID-LEFT")
- an **event history** of behavior changes (started moving, stopped, turned, entered sector, loitering, lost contact)
- a **movement trace grid** (bottom right) for one selected target, with a replay scrubber, and a maximize mode

This is separate from the reliable DETECTION alerts on the serial/radio link (`protocol/`). The scene stream is UDP and best-effort.

---

## Before you start

From the repo root, with your virtual environment active (see Setup in the main README):

```bash
pip install -r requirements.txt
```

Use the **browser** reconstructor described here. `python3 -m drone_node.reconstructor` is an older OpenCV window without the activity panel or trace grid. Don't run it at the same time as the relay; both listen on UDP port **9999**.

---

## Option A: quick UI check (1 terminal, no camera, no radio)

1. Start the relay (serves the page and listens for UDP):
   ```bash
   python3 -m drone_node.scene_ws_relay
   ```
2. Open **http://127.0.0.1:8080/reconstructor.html?demo=1**
3. `?demo=1` runs the page's built-in simulator instead of live data. The top bar shows `SRC: SIMULATOR (12Hz)` and five fake targets move around, so you can try every check in the table below.

Good for checking the UI. Then use Option B for the real pipeline.

---

## Option B: full pipeline (3 terminals)

**Terminal 1: fake serial bridge** (stands in for the two ESP32s)

```bash
python3 -m protocol.fake_bridge
```

Copy **Port A** from the output (for example `/dev/ttys003`). It changes every run.

**Terminal 2: ground reconstructor**

```bash
python3 -m drone_node.scene_ws_relay
```

Open **http://127.0.0.1:8080/reconstructor.html** and hard-refresh once (`Cmd+Shift+R`).

**Terminal 3: drone node**

```bash
python3 -m drone_node.main --port /dev/ttys003 --video 0 --fps 4.0 --headless
```

- Replace `/dev/ttys003` with your Port A.
- Drop `--headless` to also see the drone's camera window with YOLO boxes.
- `--video 0` is the webcam (macOS asks for Camera permission for your terminal). You can pass a video file path instead.

The top bar should switch to `SRC: UDP :9999 RELAY (LIVE)` and the badge to **LIVE 12Hz**.

---

## What to check

| # | Do this | You should see |
|---|---|---|
| 1 | Look at the top bar | **LIVE 12Hz** badge; scene rate below **2.00 kbps** |
| 2 | Put a person or object in front of the camera | A box on the main view and a row in **1. ACTIVE TARGET MANIFEST** |
| 3 | Look at panel **2 → LIVE ACTIVITY** | One card per target: arrow, "Moving WEST · STEADY · 2.1s" or "Stationary for 4.0s", where it's heading, and its last event |
| 4 | Move the object left/right, then hold still for 5+ s | Card changes to the new direction, then STATIONARY, then **LOITERING** |
| 5 | Click a target (main view, table row, or card) | It gets a dashed lock ring; the bottom-right grid draws its past path from **ORG** to **NOW** |
| 6 | Switch panel 2 to **EVENT HISTORY** | Only that target's events, e.g. `[TURNED] Turned NORTH → WEST`, `[ENTERED SECTOR] CENTER → MID-LEFT` |
| 7 | Drag the **REPLAY SCRUB** slider | The trace rewinds; the rest of the path shows as a dashed "ghost" |
| 8 | Press **M** | Trace grid fills the screen with a side panel: behavior, distance travelled, sector chain, target log |
| 9 | Press **ESC** twice | First ESC shrinks the grid; second unlocks the target and all targets' events come back |
| 10 | Click the locked target again | Also unlocks it |
| 11 | Remove the object from view for 2+ s | Event history logs **LOST CONTACT** ("last seen in CENTER, moving WEST") |
| 12 | Stop terminal 3 (`Ctrl+C`) | After 1.5 s the badge turns red **STALE** and the last scene is held on screen |

Keyboard: **SPACE** freeze, **M** maximize trace, **ESC** shrink/unlock, **L** simulate link drop, **0** simulate empty view (the last two only affect the simulator).

---

## Notes

- "North" means the top of the camera image, not real compass north (GPS is simulated).
- Behavior labels use motion over the last ~0.6 s and only change once a new behavior holds for ~0.4 s, so small box jitter doesn't flood the log.
- Track IDs come from the drone (0–15 pool, shown as 1–16). An ID reused for a different class is treated as a new contact.

## Troubleshooting

| Problem | Fix |
|---|---|
| Page says SIMULATOR while the drone runs | Remove `?demo=1` from the URL; make sure the relay is running, then hard-refresh |
| Page says STALE with nothing on screen | Relay is up but no drone is sending: start terminal 3, or use `?demo=1` |
| `Address already in use` on 9999 | The OpenCV reconstructor or another relay is running; stop it |
| Empty view, RX count not rising | `main` not running or wrong `--port`; check terminal 3 for errors |
| `Could not open camera index 0` | Allow Camera access for your terminal app in macOS settings, or use `--video path/to/clip.mp4` |
| Old layout after an update | Hard-refresh (`Cmd+Shift+R`) |
