# ArcDrone Scripts & Evaluation Suite (`maybe` branch)

This directory contains the automated test harnesses, simulated radio bridges, bandwidth compliance auditors, UAARG telemetry adapters, and showcase generators developed for the natHacks 2026 Defense & Security showcase.

---

## Quick Navigation

- [1. Environment & Prerequisites](#1-environment--prerequisites)
- [2. Output 1: 1:1 Side-by-Side Video & Snapshot Generator](#2-output-1-11-side-by-side-video--snapshot-generator)
- [3. Output 2: 6 Multi-Scenario Aerial Benchmarks](#3-output-2-6-multi-scenario-aerial-benchmarks)
- [4. Output 3: Live VisDrone UDP Feeder & Browser Reconstructor](#4-output-3-live-visdrone-udp-feeder--browser-reconstructor)
- [5. Output 4: Interactive Ground Station Showcase](#5-output-4-interactive-ground-station-showcase)
- [6. Output 5: Automated Headless Verification Suite (116 Tests)](#6-output-5-automated-headless-verification-suite-116-tests)
- [7. Core Infrastructure Modules](#7-core-infrastructure-modules)
- [8. Troubleshooting & Common Pitfalls](#8-troubleshooting--common-pitfalls)

---

## 1. Environment & Prerequisites

### Dependencies
From the repository root:

```bash
# Recommended: Using uv (fast, hermetic)
uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py -q

# Or standard Python virtual environment (Python 3.9 - 3.12 recommended)
python -m venv .venv
# Windows (PowerShell):
.venv\Scripts\Activate.ps1
# Linux / macOS (bash):
source .venv/bin/activate

pip install -r requirements.txt
```

### Required Assets
The showcase pipelines require:
- `yolov8n-visdrone.pt` (VisDrone-trained YOLOv8 weights located in repo root)
- `data/visdrone_aerial_sample.mp4` (Aerial footage sample)
- `data/test_cases/case*.mp4` (Multi-scenario benchmark clips)
- `data/visdrone_scene_packets.json` (Pre-encoded authentic VisDrone UDP scene packets)

---

## 2. Output 1: 1:1 Side-by-Side Video & Snapshot Generator

**Script:** [`scripts/ishan_side_by_side.py`](ishan_side_by_side.py)

### What it produces
Renders a 1920x960 composite video and a high-resolution snapshot proving the Defense & Security problem brief requirements:
1. **Left Panel (960x540, 16:9):** Real aerial RGB video with YOLOv8-VisDrone detection boxes, class labels, confidence scores, and persistent `#Index` tracking IDs.
2. **Right Panel (960x540, 16:9):** **Raw RGB completely removed** (blank tactical radar canvas). Boxes are reconstructed at the **exact same 16-bit normalized ratio positions** $(x_r, y_r, w_r, h_r)$ using incremental vector updates $(\Delta x, \Delta y, \Delta w, \Delta h)$, compact velocity arrows, and 3-step vector tails.
3. **Bottom Manifest (1920x364):** Comprehensive 2-column tactical lookup table displaying every active `#Index`, Class label (true 0-indexed VisDrone), Confidence $\%$, exact Ratio coords, Vector delta, Heading, and Rx age.
4. **Bandwidth Header:** Live telemetry header displaying wire data rate, packet counts, active target count, and GPS coordinates.

### How to Run

```bash
# Direct run
python scripts/ishan_side_by_side.py

# Using uv
uv run python scripts/ishan_side_by_side.py
```

### Artifacts Generated
- Video: `data/ishan_visdrone_side_by_side.mp4` (~18.9 MB, 150 frames @ 24 FPS)
- Snapshot: `data/ishan_visdrone_side_by_side.png` (~1.5 MB high-resolution composite)
- Peak Wire Rate: **~1.70 kB/s** (strictly within $\le 3.0\text{ kB/s}$ budget)

---

## 3. Output 2: 6 Multi-Scenario Aerial Benchmarks

**Script:** [`scripts/run_test_cases.py`](run_test_cases.py)

### What it produces
Executes the side-by-side pipeline across 6 distinct aerial operational scenarios, asserting wire rate limits, tracking stability, and packet pacing across diverse target densities.

### Benchmark Scenarios

| Case ID | Scenario Title | Video Input | Active Targets | Peak Wire Rate | File Size |
|---|---|---|:---:|:---:|:---:|
| `case1` | Urban Intersection (Dense Cross-Traffic) | `case1_urban_intersection.mp4` | 37 | **1,158 B/s** | 9.9 MB |
| `case2` | Dynamic Flight (Active Yaw & Drift) | `case2_dynamic_flight.mp4` | 101 | **1,708 B/s** | 17.3 MB |
| `case3` | Perimeter Staging (Airfield & Trucks) | `case3_perimeter_airport.mp4` | 24 | **1,582 B/s** | 15.8 MB |
| `case4` | High Altitude Roundabout (Circulation) | `case4_high_altitude_sahi.mp4` | 90 | **1,588 B/s** | 18.2 MB |
| `case5` | Checkpoint Queue (Compound Inspection) | `case5_parking_staging.mp4` | 26 | **1,596 B/s** | 15.5 MB |
| `case6` | Arterial Transit (Corridor Cruisers) | `case6_aerial_traffic_1080p.mp4` | 28 | **1,588 B/s** | 13.0 MB |

*Every scenario operates well below the 3,000 B/s primary link budget ceiling.*

### How to Run

```bash
# Execute all 6 benchmark cases in sequence
python scripts/run_test_cases.py

# Or using uv
uv run python scripts/run_test_cases.py
```

### Artifacts Generated
- Per-scenario MP4s: `data/test_cases/case{1..6}_side_by_side.mp4`
- Per-scenario PNGs: `data/test_cases/case{1..6}_side_by_side.png`
- Summary Metrics: `data/test_cases/benchmark_results.json`

---

## 4. Output 3: Live VisDrone UDP Feeder & Browser Reconstructor

**Feeder Script:** [`scripts/stream_visdrone_to_udp.py`](stream_visdrone_to_udp.py)  
**Relay Server:** [`drone_node/scene_ws_relay.py`](../drone_node/scene_ws_relay.py)  
**Browser UI:** [`drone_node/static/reconstructor.html`](../drone_node/static/reconstructor.html)

### What it produces
Streams real VisDrone detection vector telemetry (SceneCodec v2, UDP port 9999) in a real-time 12 Hz loop directly to the browser reconstructor UI. Operators can observe live target tracking, behavior classifications (STATIONARY, MOVING, LOITERING), click-to-unlock path tracing, and replay scrubbing in real-time without physical drone hardware or cameras.

### How to Run (2 Terminals)

**Terminal 1 — Start the Scene WebSocket Relay:**
```bash
python -m drone_node.scene_ws_relay
```
*Listens on UDP `127.0.0.1:9999` and serves the web UI at `http://127.0.0.1:8080/reconstructor.html`.*

**Terminal 2 — Start the Live VisDrone Feeder:**
```bash
python scripts/stream_visdrone_to_udp.py
```

**Open Browser:**
Visit `http://127.0.0.1:8080/reconstructor.html` (hard refresh: `Ctrl+F5` or `Cmd+Shift+R`).  
The top bar badge will indicate `SRC: UDP :9999 RELAY (LIVE)` and `LIVE 12Hz` with real-time target vectors.

---

## 5. Output 4: Interactive Ground Station Showcase

**Script:** [`scripts/run_live_showcase.py`](run_live_showcase.py)

### What it produces
Spins up a full integrated mission simulation:
1. `SimLinkBridge` connecting virtual drone and ground radios.
2. Simulated airborne drone emitting detections and listening for commands.
3. Ground station server + live WebSocket dashboard on `http://localhost:8000`.
4. UAARG event and telemetry logging in `tmp/log/`.

### How to Run

```bash
python scripts/run_live_showcase.py
```

Visit `http://localhost:8000` to interact with the tactical map and send operator overrides (`HOLD`, `RESUME`, `RTB`, `OPERATOR_ACK`).

---

## 6. Output 5: Automated Headless Verification Suite (116 Tests)

**Script:** [`scripts/test_integration_suite.py`](test_integration_suite.py)

### What it produces
A comprehensive headless test suite executing 116 tests across 5 verification tiers without physical hardware:
- **Tier 1 (Feature Coverage):** Bridge pairing, bandwidth budgeting, UAARG logging schema, WebSocket client, 2-way control loop.
- **Tier 2 (Boundary & Corner Cases):** Exactly 3000 B/s threshold, 64-ring buffer rollover, 3.0s blackout timeout, sequence counter wrap.
- **Tier 3 (Cross-Feature):** Blackout + command queuing, burst detection under link throttling.
- **Tier 4 (Real-World Mission Scenarios):** Full patrol, target alert, operator hold override, blackout recovery, return-to-base.
- **Tier 5 (Adversarial Coverage):** Corrupted packets, out-of-order ACKs, extreme latency jitter.

### How to Run

```bash
# Run with Pytest
uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py -q
# or:
pytest scripts/test_integration_suite.py -q

# Run directly via CLI runner
python -m scripts.test_integration_suite
```

### Component Test Scripts
- [`scripts/test_lora_bandwidth.py`](test_lora_bandwidth.py): Rigorous 3.0 kB/s bandwidth assertion test.
- [`scripts/link_test.py`](link_test.py): Link loss, 15s blackout, 64-item ring buffer flush test.
- [`scripts/test_control_loop.py`](test_control_loop.py): Two-way operator command state loop test.
- [`scripts/bench_pi.py`](bench_pi.py): Edge YOLO inference benchmark (FPS check on Pi/PC).

---

## 7. Core Infrastructure Modules

| Script | Purpose | Key Interface / Methods |
|---|---|---|
| [`sim_link_bridge.py`](sim_link_bridge.py) | In-memory virtual serial radio bridge for Windows & Linux | `create_linked_pair()`, `cut()`, `restore()`, `packet_loss_rate` |
| [`bandwidth_monitor.py`](bandwidth_monitor.py) | Sliding window byte-rate auditor | `record_tx()`, `sample()`, `peak_wire_bps`, `within_budget` |
| [`uaarg_adapter/event_logger.py`](uaarg_adapter/event_logger.py) | Shepard-style session logging | `log_event()`, writes `images/`, `bb/`, and structured `<seq>.json` |
| [`uaarg_adapter/mission_monitor.py`](uaarg_adapter/mission_monitor.py) | Emu-style telemetry & liveness monitor | `update_mode()`, `log()`, `timeSinceMessage` |
| [`uaarg_adapter/ground_client.py`](uaarg_adapter/ground_client.py) | Non-invasive WebSocket client | Connects to `ws://localhost:8000/ws` to inject commands |

---

## 8. Troubleshooting & Common Pitfalls

1. **Virtual Ports on Windows (`pty` not supported):**  
   Do not use POSIX `pty.openpty()` on Windows. Use [`scripts/sim_link_bridge.py`](sim_link_bridge.py) which provides cross-platform in-memory virtual serial pairs.
2. **OpenCV Video Codec (`fourcc`):**  
   `mp4v` is used for widest compatibility across Windows, Linux, and macOS. Ensure `opencv-python` has ffmpeg support enabled (standard in prebuilt wheels).
3. **VisDrone Class Indexing:**  
   Standard YOLO class indices start at 0 (`pedestrian` = 0, `people` = 1, `bicycle` = 2, `car` = 3, etc.). Do not apply an arbitrary +1 offset; [`scripts/ishan_side_by_side.py`](ishan_side_by_side.py) has verified 0-indexed mapping.
4. **Bandwidth Window Calculations:**  
   Wire rate must evaluate over a 1.0s sliding window. Instantaneous packet bursts may momentarily exceed 3 kB/s over a 10ms window, but the 1.0s smoothed rate must remain strictly below 3,000 B/s.
