# Drone Zone — ArcDrone Detection-to-Response Loop

natHacks 2026 · Defense & Security theme · ArcDrone challenge

A drone-side node runs an existing YOLO model on public VisDrone footage. It sends a compact 21-byte detection alert (29 bytes when encrypted) over a **real physical radio link** to a ground control dashboard. On Heltec V3 boards every packet goes over **two radios at once, ESP-NOW (2.4 GHz) and LoRa (915 MHz)**, so traffic keeps flowing if one path dies. The link handles drops and outages with ACKs, retries, and store-and-forward buffering. The operator can acknowledge alerts or send override commands (HOLD / RESUME / RTB) back to the drone. Optional AES-128-CCM encryption authenticates every frame.

Every number below was measured on our own hardware. See [Measured results](#measured-results).

## Architecture

```
DRONE SENDER (MacBook Pro M4, Pi 4 standby)     GROUND RECEIVER (MacBook Pro M4)
drone_node: video → YOLO → gate                 ground_station: FastAPI + WebSocket
  → protocol.Link (seq, ACK, retry, buffer)       → protocol.Link (same module)
        │ USB serial (hex lines)                  → browser dashboard
   ESP32 bridge  ◄── ESP-NOW 2.4 GHz + LoRa 915 ──►  ESP32 bridge
```

- **ESP32 firmware** is a dumb serial↔radio bridge. The same binary runs on both boards. On the Heltec V3 it sends every packet on both radios and the OLED shows DRONE/GROUND, radio mode, signal bars and packet counts.
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
| Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED), 863–928 MHz variant run at 915 MHz, with antennas | 2 | ✅ demo radios (`heltec_v3` env) |
| Adafruit ESP32 Feather V2 w.FL | 2 | ✅ backup (`feather_v2` env). No 2.4 GHz antennas, so range is ~2 cm |
| MacBook Pro (M4 Pro) | several | ✅ active sender (drone node) and receiver (ground station) for now |
| Raspberry Pi 4 | 1 | ⏳ standby / optional edge compute target (might switch back for airframe flight) |

**Always attach the LoRa antenna before powering a Heltec.** The firmware transmits LoRa as soon as it boots.

## Quickstart & System Setup

> **Looking for a step-by-step setup guide?** See **[GETTING_STARTED.md](GETTING_STARTED.md)** for our unified guide broken down by operational approach:
> - **[Approach 1: Virtual Trials](GETTING_STARTED.md#approach-1-virtual-trials-zero-hardware-100-simulated)** (Zero hardware, 100% simulated, runs on any laptop)
> - **[Approach 2: Real Hardware Trials](GETTING_STARTED.md#approach-2-real-hardware-trials)** (Dual ESP32s with ESP-NOW, or Heltec LoRa 915 MHz, or Hybrid Dual-Radio)
> - **[Approach 3: Test Cases & Benchmarks](GETTING_STARTED.md#approach-3-test-cases-benchmarks--analytics)** (45 automated tests, 6 aerial video benchmarks, authenticated encryption)

### Interactive Launcher (Single Command)

```bash
# Diagnoses OS & serial ports, then launches any approach from an interactive CLI menu:
python scripts/quickstart.py
# Or using uv:
uv run python scripts/quickstart.py
```

### Manual CLI Setup

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q          # 29 tests, ~30 s
```

**Run the demo** (drone board on one port, ground board on the other):

```bash
python -m ground_station.server --port <groundPort> --http-port 8001      # dashboard: http://localhost:8001
python -m drone_node.main --port <dronePort> --video data/visdrone_demo.mp4 --imgsz 1280 --fps 10
```

`--video` also takes a VisDrone image folder (e.g. `data/VisDrone2019-MOT-val/sequences/uav0000137_00458_v`) or a camera index (`--video 0 --imgsz 640` for a webcam). No radio? `python -m drone_node.run_demo` runs the whole stack on the fake bridge (dev only).

**Encryption (optional):** generate one key and set it in both terminals before starting. With a key set, unencrypted or tampered frames are rejected.

```bash
export ARCDRONE_KEY=$(python -m protocol.crypto)    # use the SAME value on both sides
```

**Demo button (PRG on Heltec, SW38 on Feather):** Heltec cycles DUAL → LORA ONLY → MUTED; Feather toggles MUTED.

**Wi-Fi camera (optional):** add `--preview-port 8081` to the drone command. The dashboard shows the drone's HUD video only while the ground laptop can reach the drone over Wi-Fi; with two laptops open the dashboard as `http://localhost:8001/?drone=<drone laptop IP>`. The video is shown inline on tab [02] next to live numbers (YOLO ms/frame, frames/s, objects, alerts sent vs received, radio vs video bit/s). It goes over Wi-Fi, never over the radio. Measured (one viewer, 1280×720 HUD, JPEG q60): ~5.1 Mbit/s for the video vs ~500 bit/s for the radio link; YOLO 67 ms/frame, 5.7 frames/s with the HUD and preview running.

**Radio paths on the dashboard:** the three tiles under the status row show what the ground board hears on each radio (live with dBm, or "No signal") and which path is carrying traffic. Walk the drone board away and ESP-NOW goes red while LoRa stays green. For more LoRa range, flash **both** boards with `pio run -e heltec_v3_sf9 -t upload` (in theory about 2× range for about 4× airtime; not measured yet).

**Range walk:** ground server stays put; the walker carries the drone board on a laptop running `python -m protocol.range_walk --port <dronePort>`. It sends 1 alert/s and logs, every second, which radio is heard, RSSI/SNR, delivered/sent and RTT to a CSV. Type the floor and press Enter to mark it.

**Without hardware:** `python -m protocol.fake_bridge` prints two virtual serial ports. Pass one to each side as `--port`. It's for development only; the demo uses the real radios.

## Quick Guides & LLM Context

- **Master Setup Guide:** See [GETTING_STARTED.md](GETTING_STARTED.md) for full instructions across virtual, hardware, and benchmark modes.
- **For Defense Judges & Pitch Presenters:** See [PRESENTATION_PACK.md](PRESENTATION_PACK.md) for the 5-minute pitch deck, investor "Wow-ness" hierarchy, judge demo runbook, and empirical benchmark tables.
- **For AI Assistants (Claude, Gemini, ChatGPT):** See [LLM_GUIDE.md](LLM_GUIDE.md) for immutable file boundaries, wire specs, data schemas, and prompt templates.
- **For Evaluators & Benchmarks:** See [scripts/README.md](scripts/README.md) for technical deep dives on all evaluation scripts.
- **For Browser Reconstructor Reviewers:** See [drone_node/static/RECONSTRUCTOR_SETUP.md](drone_node/static/RECONSTRUCTOR_SETUP.md).

## Scene reconstructor (operator view)

Alongside reliable DETECTION alerts, the drone streams a compact scene vector feed (SceneCodec v2, UDP, 12 Hz, budgeted at ≤ 2.0 kbps; real rate not measured yet). It goes over Wi-Fi, not the radio. A browser page rebuilds the tactical view, shows what each target is doing, logs behavior changes, and lets you trace and replay one target’s movement.

**Quick UI check (no camera or radio):**

```bash
python3 -m drone_node.scene_ws_relay
# http://127.0.0.1:8080/reconstructor.html?demo=1
```

**Full 3-terminal run + step-by-step checklist for reviewers:** [drone_node/static/RECONSTRUCTOR_SETUP.md](drone_node/static/RECONSTRUCTOR_SETUP.md).

## Tactical Showcase & Multi-Scenario Benchmarks (`maybe` branch)

The `maybe` branch integrates the full Defense & Security evaluation suite, proving that tactical ground awareness is reconstructed with 1:1 ratio precision while keeping wire bandwidth strictly under the 3.0 kB/s budget:

1. **1:1 Side-by-Side Video & Snapshot Generator:**  
   `python scripts/ishan_side_by_side.py` (or `uv run python scripts/ishan_side_by_side.py`)  
   Produces `data/ishan_visdrone_side_by_side.mp4` and `.png`. Replaces raw RGB with a blank tactical radar grid, drawing 16-bit ratio boxes, velocity vectors, and a live target manifest. Peak wire rate: **~1.70 kB/s**.

2. **6 Multi-Scenario Aerial Benchmarks:**  
   `python scripts/run_test_cases.py` (or `uv run python scripts/run_test_cases.py`)  
   Runs batch evaluation across 6 distinct aerial clips (`case1` Urban Intersection, `case2` Dynamic Flight, `case3` Airfield Staging, `case4` Roundabout, `case5` Checkpoint Queue, `case6` Arterial Transit). Outputs videos, PNGs, and metrics in `data/test_cases/benchmark_results.json`.

3. **Live VisDrone UDP Scene Feeder:**  
   `python scripts/stream_visdrone_to_udp.py`  
   Streams authentic VisDrone scene packets at 12 Hz to `scene_ws_relay`, enabling live browser tactical tracking without requiring drone hardware or physical cameras.

4. **Automated Headless Verification Suite (9 Integration Tests, 45 Repo Total):**  
   `uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py -q`  
   100% headless pass across 5 verification tiers (virtual radio bridge, blackout/recovery, <= 3 kB/s budget auditor, UAARG logging, 2-way control loop).

## Measured results

All on our own boards and a MacBook M4 Pro (November 7–8, 2026). Boards were on one desk unless stated. RTT is the sender's ACK round trip, so it includes USB serial on both ends.

**Radio link (Heltec V3, ESP-NOW + LoRa 915 MHz SF7/BW125, 10 dBm)**

| Test | Delivered | Retries | RTT avg / p95 |
|---|---|---|---|
| Both radios, 200 alerts | 200/200, each shown once | 0 | 19.9 / 27.0 ms |
| Both radios, 5 min at 1 alert/s | 270/270 | 9 | 29.8 ms avg |
| ESP-NOW killed (PRG), LoRa only, 60 alerts | 60/60, each shown once | 11 | 128.5 / 150.5 ms |
| Encrypted, both radios, 60 alerts | 60/60 | 0 | 34.1 / 62.0 ms |
| Encrypted, LoRa only, 60 alerts | 60/60 | 8 | 156.7 / 213.0 ms |
| Receiver with no key | 0 frames accepted, link never comes up | — | — |
| Live failover: 30 alerts on both radios, then drone ESP-NOW killed mid-run, 50 more (3 runs) | 80/80 each run, each shown once | 0–10 after the switch | before 22–25 ms; after 126–145 ms avg |

**Reliability (Feather V2, ESP-NOW, boards 1–2 cm apart, no antennas)**

| Test | Result |
|---|---|
| Phase 1 loop, 100 alerts | 100/100, 0 retries, RTT 16.5 ms avg |
| 14 s radio mute during 120 alerts | LINK LOST 3.1 s after last ACK; 120/120 delivered exactly once, in order; 30 tagged DELAYED; 1 duplicate re-ACKed, not shown |
| Drone USB unplugged and replugged | Link auto-recovered 2.7 s after replug; 100/100 delivered |
| Feathers 5–20 cm apart | ~8% delivered (no antennas): why we moved to the Heltecs |

**Operator loop (real boards, dashboard + drone node)**

- Acknowledge, HOLD, RESUME, RTB each went PENDING → CONFIRMED, and the drone reacted.
- HOLD sent during a mute stayed PENDING. On restore the drone flushed 43 buffered alerts, then froze with the HOLD banner.
- 11 commands clicked during an outage were delivered in click order after restore: 38/38 commands delivered over the session.

**Detection (Phase 2)**

- COCO `yolov8n`, imgsz 1280, on the MacBook M4 Pro: 64 ms per frame on a 51 s clip built from 5 VisDrone2019-MOT-val sequences at 10 fps.
- About 17 boxes per frame; with conf ≥ 0.5 and a 3 s per-class cooldown, about 0.5 alerts/s (car, pedestrian, truck, bus, bicycle).
- At imgsz 640 the same model found almost nothing (1 alert in 20 frames), because VisDrone objects are tiny.
- Pi 4: **not measured yet.**

**Bandwidth**

| Packet | Plain | Encrypted |
|---|---|---|
| DETECTION | 21 B | 29 B |
| COMMAND | 9 B | 17 B |
| HEARTBEAT | 12 B | 20 B |
| ACK | 6 B | 14 B |

Live link traffic, both directions, measured by the ground station: about 200 bit/s during the demo, against LoRa SF7/BW125's ~5,470 bit/s and a typical 720p video downlink of ~2 Mbit/s.

**Not measured yet:** range beyond one desk, the Pi 4 frame rate, behaviour in a crowded 2.4 GHz venue.

## Progress

- [x] **Phase 0:** Radio smoke test (hex lines echo both ways). Passed on 2× Feather V2, boards 1–2 cm apart, no antennas yet.
- [x] **Phase 1:** MVP loop (detection → radio → ground print → ACK + RTT). 100/100 delivered, 0 retries, RTT 16.5 ms avg (same setup).
- [x] **Phase 2:** Real YOLO detections on VisDrone footage. COCO yolov8n at imgsz 1280 on the Mac, on a 51 s VisDrone2019-MOT clip at 10 fps: ~64 ms/frame, ~0.5 alerts/s (car, pedestrian, truck, bus, bicycle). Pi 4 benchmark not measured yet.
- [x] **Phase 3:** Reliability (ACK/retry, dedup, heartbeat, LINK LOST, ring buffer). On real boards: 14 s SW38 mute → LINK_DOWN in 3.1 s, 120/120 delivered exactly once, 30 DELAYED; drone USB unplug/replug → auto-recovered in 2.7 s, 100/100.
- [x] **Phase 4:** Ground station UI + operator acknowledge/override. On real boards: ACK, HOLD, RESUME, RTB all CONFIRMED; a HOLD sent during a mute stayed PENDING and froze the drone right after restore (43 buffered alerts flushed first); 11 commands queued during an outage were delivered in click order.
- [ ] **Phase 5:** Stretch
  - [x] S1 dual path on 2× Heltec V3 (LoRa 915 MHz, SF7): PRG cycles ESP-NOW+LoRa → LoRa only → muted. Both paths: 60/60, RTT 24.7 ms; ESP-NOW killed: 60/60 over LoRa, 11 retries, RTT 128.5 ms. OLED shows mode, per-radio counts and RSSI.
  - [x] S2 metrics panel ("On the wire": exact packet bytes, measured bit/s vs LoRa and video)
  - [x] S3 mute button (SW38 on Feather, PRG on Heltec)
  - [x] Link encryption: AES-128-CCM, +8 bytes per packet, tampered/plain/wrong-key frames rejected (tested on hardware)
  - [ ] S4 LoRa-only mode with airtime readout

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
