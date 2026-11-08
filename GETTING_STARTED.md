# ArcDrone Quickstart & Unified Setup Guide

> **One master guide to run ArcDrone on any machine.** Choose the approach that fits your setup: **Virtual Trials** (no hardware needed), **Real Hardware** (ESP32 ESP-NOW or 915 MHz LoRa), or **Test Cases & Benchmarks**.

---

## ⚡ 10-Second Setup (Prerequisites)

ArcDrone works on **Windows**, **macOS**, and **Linux**.

```bash
# Clone and enter repo
git clone https://github.com/shivenlak/ArcDrone.git
cd ArcDrone

# Recommended: Run with uv (zero configuration needed)
uv run python scripts/quickstart.py

# Or standard Python virtual environment (Python 3.9 - 3.12)
python3 -m venv .venv
# Windows (PowerShell): .venv\Scripts\Activate.ps1 | macOS/Linux: source .venv/bin/activate
pip install -r requirements.txt
python scripts/quickstart.py
```

`scripts/quickstart.py` will automatically diagnose your OS, check Python dependencies, discover any plugged-in serial ports, and present an interactive menu to launch any approach with a single keypress.

---

## 🧭 Choose Your Approach

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           CHOOSE YOUR RUN MODE                              │
├──────────────────────────┬──────────────────────────────────────────────────┤
│ 1. VIRTUAL TRIALS        │ 0 hardware required; runs on any laptop.        │
│ 2. REAL HARDWARE         │ Dual ESP32s (ESP-NOW) or Heltec LoRa 915 MHz.    │
│ 3. BENCHMARKS & TESTS    │ 116 automated tests, 6 aerial video evaluations. │
└──────────────────────────┴──────────────────────────────────────────────────┘
```

---

## 💻 Approach 1: Virtual Trials (Zero Hardware, 100% Simulated)

Ideal for software developers, teammates without microcontrollers, judges, and presentation reviewers.

### Trial 1A: One-Click Interactive Mission Dashboard & Radio Sim
Spins up an in-memory virtual radio bridge (`SimLinkBridge`), simulates airborne drone YOLO detections and GPS movement, starts the ground station web server, and logs structured UAARG telemetry:

```bash
python scripts/run_live_showcase.py
# Or: uv run python scripts/run_live_showcase.py
```
- Open browser: **`http://localhost:8000`**
- **Try this:** Click **HOLD** to pause drone flight; watch status transition from `PENDING` $\to$ `CONFIRMED`. Click **RESUME** or **RTB**.
- Logs are automatically saved to `tmp/log/` (raw frames, annotated boxes, and JSON telemetry snapshots).

---

### Trial 1B: Browser Tactical Radar Reconstructor (Live 12 Hz Stream)
Streams authentic VisDrone aerial detection packets over UDP at 12 Hz to the browser reconstructor canvas without physical cameras:

**Terminal 1 — Start the Scene Relay:**
```bash
python -m drone_node.scene_ws_relay
```
*Listens on UDP `:9999` and serves the web UI at `http://127.0.0.1:8080/reconstructor.html`.*

**Terminal 2 — Start the Live VisDrone Feeder:**
```bash
python scripts/stream_visdrone_to_udp.py
```

**Open Browser:**
Visit **`http://127.0.0.1:8080/reconstructor.html`** (hard refresh: `Ctrl+F5` or `Cmd+Shift+R`).
- The top badge will display `LIVE 12Hz` and wire rate below `2.00 kbps`.
- **Try this:** Click any target on the main canvas to lock its trajectory. Drag the **REPLAY SCRUB** slider to rewind past movement. Press **`M`** to maximize the tactical trace grid.

---

### Trial 1C: 1:1 Side-by-Side Video & Radar Canvas Generator
Renders a 1920x960 composite video and high-resolution snapshot proving that raw RGB can be stripped completely while maintaining 1:1 spatial positioning:

```bash
python scripts/ishan_side_by_side.py
```
- **Output Video:** `data/ishan_visdrone_side_by_side.mp4` (~18.9 MB)
- **Output Snapshot:** `data/ishan_visdrone_side_by_side.png` (~1.5 MB)
- **Left Panel:** Raw RGB video + YOLOv8 detection boxes + persistent tracking IDs.
- **Right Panel:** Raw RGB stripped; boxes drawn at 16-bit ratio precision with velocity arrows and compact tails.
- **Bottom Manifest:** Live 2-column table tracking all active indices, confidence $\%$, and directional heading.
- **Audited Wire Rate:** **~1.70 kB/s** (well within $\le 3.0\text{ kB/s}$ ceiling).

---

### Trial 1D: Zero-Install Static Browser Demos
No Python or servers required—just open in any web browser:
- Tactical Dashboard Demo: `ground_station/static/index.html?demo=1`
- Browser Radar Reconstructor Demo: `drone_node/static/reconstructor.html?demo=1`

---

## 📡 Approach 2: Real Hardware Trials

For developers connecting physical microcontrollers over USB.

```
DRONE SIDE (Pi 4 / Laptop)                        GROUND SIDE (Laptop)
drone_node: video → YOLO → gate                   ground_station: FastAPI + WebSocket
  → protocol.Link (seq, ACK, retry, buffer)         → protocol.Link (same module)
        │ USB serial (hex lines)                    → browser dashboard (localhost:8000)
   ESP32 bridge  ◄──── 2.4 GHz ESP-NOW or 915 MHz LoRa ────►  ESP32 bridge
```

---

### Setup 2A: Dual ESP32s Only (ESP-NOW 2.4 GHz)

*Uses 2x standard ESP32 development boards (Adafruit Feather V2, ESP32 DevKit, etc.). Both boards run the exact same firmware binary.*

1. **Flash Firmware:**
   Connect one ESP32 via USB and flash using PlatformIO:
   ```bash
   cd firmware/bridge
   # For Adafruit Feather V2:
   pio run -t upload -e feather_v2
   # For generic ESP32 DevKit boards:
   pio run -t upload -e esp32dev
   cd ../..
   ```
   *Unplug the first board, plug in the second board, and repeat the command.*

2. **Discover Serial Ports:**
   - **Windows:** `Get-CimInstance Win32_SerialPort | Select DeviceID, Description` (e.g., `COM3`, `COM4`)
   - **macOS:** `ls /dev/cu.usbserial*` or `ls /dev/cu.usbmodem*`
   - **Linux:** `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`

3. **Launch Ground Station:**
   ```bash
   python -m ground_station.server --port COM4
   # Browser: open http://localhost:8000
   ```

4. **Launch Drone Node:**
   ```bash
   python -m drone_node.main --port COM3 --video data/visdrone_aerial_sample.mp4 --fps 10
   ```
   *(Or pass `--video 0` to use your computer's live webcam instead of sample video).*

5. **Hardware Controls:**
   - **Mute Test:** Press the `BOOT` / `GPIO0` button on either board to simulate hardware radio silence/jamming. The link detects disconnection after 3.0s and flushes buffered alerts upon release.
   - **Activity LED:** On-board LED flashes on each valid packet transmission.

---

### Setup 2B: Dual Heltec WiFi LoRa 32 V3/V4 (915 MHz LoRa)

*Uses 2x Heltec WiFi LoRa 32 boards with SX1262 transceivers for long-range point-to-point narrowband radio.*

> [!CAUTION]
> **Always connect 915 MHz antennas before powering on Heltec LoRa boards.** Transmitting without an antenna can permanently damage the radio power amplifier.

1. **Flash Firmware:**
   ```bash
   cd firmware/bridge
   # For Heltec WiFi LoRa 32 V3:
   pio run -t upload -e heltec_v3
   # For Heltec WiFi LoRa 32 V4:
   pio run -t upload -e heltec_v4
   cd ../..
   ```

2. **Launch Stack:**
   - Ground Station: `python -m ground_station.server --port <GROUND_PORT>`
   - Drone Node: `python -m drone_node.main --port <DRONE_PORT> --video data/visdrone_aerial_sample.mp4 --fps 10`
   - *Note:* The link layer automatically de-asserts DTR/RTS lines upon opening the port so the Heltec PRG button (GPIO0) is never held in reset or mute.

---

### Setup 2C: Dual-Radio / Hybrid Mode (LoRa Primary + WiFi Secondary)

- **Primary Tactical Link (LoRa 915 MHz):** Dedicated to $\le 3.0\text{ kB/s}$ low-bandwidth detection alerts, GPS coordinates, and operator control overrides (`HOLD`/`RESUME`/`RTB`).
- **Secondary Link (2.4 GHz):** Activated only when within proximity for high-speed diagnostic telemetry or browser reconstructor streaming (`scene_ws_relay`).

---

## 📊 Approach 3: Test Cases, Benchmarks & Analytics

Run our automated verification suites, cryptographic assertions, and multi-scenario aerial evaluations.

### 1. Automated Headless Verification Suite (116 Tests)
Executes 116 tests across 5 verification tiers (virtual radio bridges, blackout survival, sliding window budget auditor, UAARG logging schemas, and two-way control loops):

```bash
uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py -q
# Or direct CLI runner:
python -m scripts.test_integration_suite
```
*Expected: 9/9 test suites pass, 116 assertions verified in ~6.5 seconds.*

---

### 2. Zero-Overhead AES-128-CTR Encryption Tests (7 Tests)
Verifies military-grade payload encryption with 0% size inflation (+0 bytes):

```bash
uv run python -m unittest scripts/test_encryption.py
```
*Asserts:*
- `DETECTION`: exactly **21 bytes** (plaintext) $\to$ **21 bytes** (encrypted).
- `COMMAND`: exactly **9 bytes** (plaintext) $\to$ **9 bytes** (encrypted).
- `HEARTBEAT`: exactly **12 bytes** (plaintext) $\to$ **12 bytes** (encrypted).
- Deterministic header IV derivation, tamper detection, and direction isolation.

---

### 3. 6 Multi-Scenario Aerial Video Benchmarks
Runs batch evaluation across 6 distinct aerial clips, generating side-by-side composite MP4s, PNG snapshots, and a consolidated JSON metrics report:

```bash
python scripts/run_test_cases.py
```

| Benchmark Case | Flight Scenario | Active Targets | Peak Wire Rate | Headroom Margin |
|---|---|:---:|:---:|:---:|
| `case1` | Urban Intersection (Dense Cross-Traffic) | 37 | **1,158 B/s** | 61.4% under budget |
| `case2` | Dynamic Flight (Active Yaw & Drift) | **101** | **1,708 B/s** | 43.1% under budget |
| `case3` | Perimeter Staging (Airfield & Trucks) | 24 | **1,582 B/s** | 47.3% under budget |
| `case4` | High Altitude Roundabout (Circulation) | 90 | **1,588 B/s** | 47.1% under budget |
| `case5` | Checkpoint Queue (Compound Inspection) | 26 | **1,596 B/s** | 46.8% under budget |
| `case6` | Arterial Transit (Corridor Cruisers) | 28 | **1,588 B/s** | 47.1% under budget |

*Outputs:* `data/test_cases/case{1..6}_side_by_side.mp4`, `.png`, and `data/test_cases/benchmark_results.json`.

---

### 4. Primary Link Bandwidth Compliance Auditor
Asserts sliding-window byte rates across idle heartbeats, active patrol, burst traffic, and two-way tactical exchanges against the 3.0 kB/s budget:

```bash
python -m scripts.test_lora_bandwidth
```
*Outputs audit report to `scripts/bandwidth_report.json`.*

---

## 📁 Repository Reference Map

| File / Folder | What It Is | Primary Owner |
|---|---|---|
| [`GETTING_STARTED.md`](GETTING_STARTED.md) | **This master setup guide (all 3 approaches)** | Shared |
| [`scripts/quickstart.py`](scripts/quickstart.py) | **Interactive CLI launcher & diagnostic tool** | Manan |
| [`PRESENTATION_PACK.md`](PRESENTATION_PACK.md) | Pitch deck, investor "Wow-ness" hierarchy & judge runbook | Manan |
| [`LLM_GUIDE.md`](LLM_GUIDE.md) | AI assistant context for Claude, Gemini, and ChatGPT | Manan |
| [`scripts/README.md`](scripts/README.md) | Technical operations guide for all evaluation scripts | Manan |
| `drone_node/` | Edge YOLO detection & state machine | Ishan |
| `ground_station/` | FastAPI server & tactical web dashboard (`static/`) | Samar & Marriam |
| `protocol/` | Binary packet codec (`packets.py`) & reliable link layer (`link.py`) | Shiven |
| `firmware/` | PlatformIO ESP32 / Heltec LoRa bridge firmware | Shiven |
| `scripts/` | Cross-platform bridge, benchmarks, encryption & test suites | Manan |
