# Drone Zone — ArcDrone Detection-to-Response Loop

natHacks 2026 · Defense & Security theme · ArcDrone challenge

A drone-side node runs an existing YOLO model on public VisDrone footage. It sends a compact (~21-byte) detection alert over a **real physical radio link**, starting with ESP-NOW and adding LoRa 915 MHz later, to a ground control dashboard. The link handles drops and outages with ACKs, retries, and store-and-forward buffering. The operator can acknowledge alerts or send override commands (HOLD / RESUME / RTB) back to the drone.

## Architecture

```
DRONE SIDE (Pi 4 / MacBook)                     GROUND SIDE (MacBook)
drone_node: video → YOLO → gate                 ground_station: FastAPI + WebSocket
  → protocol.Link (seq, ACK, retry, buffer)       → protocol.Link (same module)
        │ USB serial (hex lines)                  → browser dashboard
   ESP32 bridge  ◄──── ESP-NOW (later +LoRa) ────►  ESP32 bridge
```

- **ESP32 firmware** is a dumb serial↔radio bridge. The same binary runs on both boards.
- **All protocol logic** is in `protocol/` (Python) and shared by both sides.
- See [CLAUDE.md](CLAUDE.md) for the full plan and packet spec.

## Repo layout

| Folder | What | Language |
|---|---|---|
| `protocol/` | Packet encode/decode + CRC, reliable link layer | Python |
| `firmware/bridge/` | ESP32 serial↔radio bridge (PlatformIO) | C++ (Arduino) |
| `drone_node/` | YOLO detection, simulated GPS, command handling | Python |
| `ground_station/` | FastAPI server + dashboard (`static/`) | Python + HTML/JS |
| `scripts/` | Cross-platform sim bridge, bandwidth auditor, UAARG adapter, test harness, live demo | Python |
| `data/` | VisDrone clips (gitignored) | — |

## Hardware

| Item | Qty | Status |
|---|---|---|
| Raspberry Pi 4 | 1 | ✅ have |
| ESP32 dev board | 2 | ✅ have |
| MacBook (M4 Pro) | several | ✅ have |
| Heltec WiFi LoRa 32 V3, **915 MHz** + antennas | 2 | ⏳ to order |

## Setup

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q          # protocol tests, ~15 s
```

**Without hardware:** `python -m protocol.fake_bridge` prints two virtual serial ports. Pass one to each side as `--port`. It's for development only; the demo uses the real radios.

**Team rules** (file ownership, git workflow, interface contracts): see [CLAUDE.md](CLAUDE.md#team-workflow-avoid-blocking-and-merge-conflicts).

## Scene reconstructor (operator view)

Alongside reliable DETECTION alerts, the drone streams a compact scene vector feed (SceneCodec v2, UDP, 12 Hz, ≤ 2.0 kbps). A browser page rebuilds the tactical view, shows what each target is doing, logs behavior changes, and lets you trace and replay one target’s movement.

**Quick UI check (no camera or radio):**

```bash
python3 -m drone_node.scene_ws_relay
# http://127.0.0.1:8080/reconstructor.html?demo=1
```

**Full 3-terminal run + step-by-step checklist for reviewers:** [drone_node/static/RECONSTRUCTOR_SETUP.md](drone_node/static/RECONSTRUCTOR_SETUP.md).

## Progress

- [x] **Phase 0:** Radio smoke test (hex lines echo both ways)
- [x] **Phase 1:** MVP loop (detection → radio → ground print → ACK + RTT)
- [x] **Phase 2:** Real YOLO detections on VisDrone footage (Pi 4 benchmark gate: 95.5 FPS)
- [x] **Phase 3:** Reliability (ACK/retry, dedup, heartbeat, LINK LOST, 64-entry ring buffer)
- [x] **Phase 4:** Ground station UI + operator acknowledge/override (HOLD / RESUME / RTB)
- [x] **Phase 5:** Low-bandwidth compliance (<= 3 kB/s budget monitor, UAARG telemetry/logging adapter)

## Team

| Role | Owner |
|---|---|
| Firmware + link layer | Shiven |
| CV / drone node | Ishan |
| Ground server | Samar |
| Dashboard UI | Marriam |
| Test harness, LoRa, bandwidth compliance, UAARG adapters, demo/pitch | Manan |

## Changelog

See [CHANGELOG.md](CHANGELOG.md).
