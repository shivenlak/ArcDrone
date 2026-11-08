# LLM Guide — ArcDrone Project Context (`maybe` branch)

> **For AI Assistants:** This document provides comprehensive project context, architectural invariants, code ownership contracts, wire specs, and reproduction commands for **Claude**, **Gemini**, and **ChatGPT**.

---

## 1. Project Summary & Core Mission

ArcDrone is a **natHacks 2026 Defense & Security** challenge project. The system bridges airborne computer vision intelligence to ground station decision-makers over a constrained radio link (**$\le 3.0$ kB/s bandwidth budget**), avoiding raw continuous video streaming.

### The Problem It Solves
- High-resolution aerial video cannot stream continuously through contested, jammed, or low-bandwidth radio channels (ESP-NOW, 915 MHz LoRa).
- Instead, the drone edge node runs YOLO object detection locally and transmits compact structured telemetry (~21B alerts and $\le 2.0$ kbps tactical scene vectors).
- The ground station reconstructs a **1:1 blank-canvas tactical radar view** with persistent tracking IDs, velocity vectors, and full attribute manifests.
- A reliable two-way control loop allows ground operators to confirm alerts or command the drone (`HOLD`, `RESUME`, `RTB`, `OPERATOR_ACK`).

---

## 2. Team Code Ownership Contracts (STRICT IMMUTABILITY)

When assisting team member **Manan** (branch: `manan/test-lora-demo` / `maybe`), adhere to these immutability rules:

| Directory / File | Owner | Branch | LLM Rule |
|---|---|---|---|
| `protocol/` | Shiven | `shiven/firmware-link` | **READ-ONLY — NEVER MODIFY** |
| `firmware/` | Shiven | `shiven/firmware-link` | **READ-ONLY — NEVER MODIFY** |
| `drone_node/` | Ishan | `ishan/cv-drone-node` | **READ-ONLY — NEVER MODIFY** |
| `ground_station/*.py` | Samar | `samar/ground-server` | **READ-ONLY — NEVER MODIFY** |
| `ground_station/static/` | Marriam | `marriam/dashboard-ui` | **READ-ONLY — NEVER MODIFY** |
| `scripts/` | Manan | `manan/test-lora-demo` | **MANAN EXCLUSIVE WRITE DOMAIN** |
| `README.md`, `CLAUDE.md`, `CHANGELOG.md` | Shared | Multi | **Append-only / Small coordination** |

> [!CAUTION]
> Never generate diffs or propose edits to `protocol/`, `firmware/`, `drone_node/`, or `ground_station/` when implementing test, benchmark, or showcase features. All adapters, bridges, test harnesses, and evaluation scripts belong in `scripts/`.

---

## 3. Quick Execution Cheatsheet for LLMs

All commands can be run with **`uv`** (fastest, hermetic) or standard `python` with active virtual environment:

### A. Run Automated Verification Suite (9 Integration Tests, 45 Repo Total)
```bash
# Using uv (recommended on all OS):
uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py -q

# Using active virtual environment:
pytest scripts/test_integration_suite.py -q
# Direct CLI runner:
python -m scripts.test_integration_suite
```
*Expected: 9 integration test suites pass, exit code 0.*

### B. Generate 1:1 Side-by-Side Video & High-Res Snapshot
```bash
uv run python scripts/ishan_side_by_side.py
# or:
python scripts/ishan_side_by_side.py
```
*Outputs: `data/ishan_visdrone_side_by_side.mp4`, `data/ishan_visdrone_side_by_side.png`.*  
*Asserts: Peak wire data rate $\le 3,000$ B/s ($< 1.75$ kB/s achieved).*

### C. Execute 6 Multi-Scenario Aerial Benchmarks
```bash
uv run python scripts/run_test_cases.py
# or:
python scripts/run_test_cases.py
```
*Outputs: 6 MP4s and PNGs in `data/test_cases/case{1..6}_*` and `benchmark_results.json`.*

### D. Run Live VisDrone UDP Stream to Browser Reconstructor (No Hardware Required)
```bash
# Terminal 1: Start WebSocket relay and web server
uv run python -m drone_node.scene_ws_relay

# Terminal 2: Stream authentic VisDrone detections over UDP
uv run python scripts/stream_visdrone_to_udp.py

# Browser: Open http://127.0.0.1:8080/reconstructor.html
```

### E. Run Live Interactive Showcase & Tactical Dashboard
```bash
uv run python scripts/run_live_showcase.py
# Browser: Open http://localhost:8000
```

### F. Run Authenticated AES-128-CCM Encryption Suite (9 Tests)
```bash
uv run python -m unittest scripts/test_encryption.py
```
*Validates: Authenticated AEAD wire sizes (+8B overhead), random nonces, bit-flip tamper rejection, unencrypted frame rejection, and reliable Link communication under packet loss over `SimLinkBridge`.*

### G. Run Enhanced Architectural Features Verification
```bash
uv run python -m unittest scripts/test_enhanced_features.py
```
*Validates: Dynamic role learning (`DRONE` vs `GROUND`), dual-path radio frequency diversity (`BOTH`, `LORA_ONLY`, `MUTED`), grid-aware coasting tracker (2-frame edge drop vs 3-frame interior coasting), and monotonic clock safety.*

---

## 4. Architecture & Wire Protocol Specifications

### 4.1 Primary Radio Link (`protocol.packets`)
All packets carry a 2-byte CRC16-CCITT checksum. Max radio payload: 32 bytes (ESP-NOW / LoRa compatible).

| Packet Type | OpCode | Size | Payload Structure |
|---|:---:|:---:|---|
| `DETECTION` | `0x01` | **21 B** | `[seq:2B, ts_ms:6B, cls:1B, conf:1B, count:1B, lat:4B, lon:4B, crc:2B]` |
| `COMMAND` | `0x02` | **9 B** | `[seq:2B, cmd:1B, ref_seq:2B, pad:2B, crc:2B]` |
| `ACK` | `0x03` | **6 B** | `[seq:2B, ack_seq:2B, crc:2B]` |
| `HEARTBEAT` | `0x04` | **12 B** | `[seq:2B, mode:1B, buffered:1B, lat:4B, lon:4B, crc:2B]` |

**Command Codes (`cmd`):**
- `0x00` = `CMD_OPERATOR_ACK`
- `0x01` = `CMD_HOLD`
- `0x02` = `CMD_RESUME`
- `0x03` = `CMD_RTB`

**Drone Operational Modes (`mode`):**
- `0x00` = `MODE_PATROL`
- `0x01` = `MODE_HOLD`
- `0x02` = `MODE_RTB`

---

### 4.2 Spread-Out 12 FPS Tactical Vector Stream (`scripts/ishan_side_by_side.py`)
- **Magic Byte:** `0xB4`
- **Pacing:** 12 FPS (83.3 ms intervals)
- **Quantization:** Normalized bounding boxes $[0.0, 1.0] \leftrightarrow [0, 65535]$ (unsigned 16-bit integer).
  $$\text{quantize\_u16}(r) = \text{round}(r \times 65535)$$

**Sub-packet OpCodes:**
1. `OP_INIT_BOX` (`0x01`, 12 bytes): New target detection.
   `[op:1B, id:1B, cls:1B, conf:1B, cx_u16:2B, cy_u16:2B, w_u16:2B, h_u16:2B]`
2. `OP_VEC_DELTA` (`0x02`, 8 bytes): Incremental position change.
   `[op:1B, id:1B, dcx_i16:2B, dcy_i16:2B, dw_i8:1B, dh_i8:1B]`
3. `OP_DROP_BOX` (`0x03`, 2 bytes): Target dropped or exited frame.
   `[op:1B, id:1B]`

---

### 4.3 VisDrone True Class Indexing (0-indexed)
The YOLOv8-VisDrone weights use the following 10 classes:
```python
VISDRONE_CLASSES = {
    0: "pedestrian",
    1: "people",
    2: "bicycle",
    3: "car",
    4: "van",
    5: "truck",
    6: "tricycle",
    7: "awning-tricycle",
    8: "bus",
    9: "motor",
}
```
*Note: Ensure NO +1 offset is added to class IDs.*

---

### 4.4 Authenticated Tactical Link Encryption (`scripts/secure_channel.py` & `protocol/crypto.py`)
- **Cipher Standard:** AES-128-CCM (Counter with CBC-MAC) AEAD encryption and authentication.
- **Wire Overhead:** **+8 bytes per packet**. Replaces CRC16 with a 4-byte auth tag:
  * `DETECTION`: 21 B -> 29 bytes
  * `COMMAND`: 9 B -> 17 bytes
  * `HEARTBEAT`: 12 B -> 20 bytes
  * `ACK`: 6 B -> 14 bytes
- **Fresh Nonces:** 6-byte (48-bit) random nonce generated fresh for every transmission (retries included).
- **Authentication & Integrity:** Magic 0xD3 frames; any flipped bit in header, nonce, ciphertext, or auth tag causes tag verification to fail; plain frames (0xD2) and forged packets are rejected.
- **Replay Limitation:** Replay is mitigated across the 64-seq circular window. Old packets outside this window could theoretically be accepted if sequence numbers wrap or across reboots.

---

## 5. Ready-to-Use Prompt Templates for AI Assistants

### For Claude (Coding & Test Extensions)
```markdown
You are assisting on the ArcDrone repo on branch 'maybe'.
Review 'LLM_GUIDE.md' and 'PROJECT.md'.
Strictly maintain file immutability: do not edit files in protocol/, firmware/, drone_node/, or ground_station/.
All changes must be made within scripts/.
Run verification with: uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py -q
```

### For Gemini (Visual & Benchmark Inspection)
```markdown
Inspect the generated side-by-side snapshot at 'data/ishan_visdrone_side_by_side.png' and the benchmark summary at 'data/test_cases/benchmark_results.json'.
Evaluate:
1. Ratio tracking alignment between Left (RGB) and Right (Blank Canvas).
2. Wire bandwidth utilization against the <= 3.0 kB/s budget.
3. Behavior state classification accuracy in the bottom manifest.
```

### For ChatGPT (Pitching, Evaluation & Metrics Analysis)
```markdown
Based on 'data/test_cases/benchmark_results.json' and 'scripts/README.md', generate a defense stakeholder summary of the ArcDrone low-bandwidth detection pipeline. Include:
- Peak wire rates across all 6 scenarios (compare against the 3.0 kB/s budget).
- Target density handling (up to 101 simultaneous targets in Case 2).
- Zero-loss recovery during radio blackouts.
```

### For All Models: Generating Pitch Slides & Defense Briefs
```markdown
Read 'PRESENTATION_PACK.md'.
Using Section 2 (Empirical Benchmark Evidence) and Section 4 (5-Minute Winning Pitch Deck Outline):
1. Format a high-impact 6-slide presentation deck (in Marp markdown or bulleted slide specs).
2. Incorporate the empirical benchmark numbers from all 6 test cases (e.g. Case 2: 101 active targets at only 1,708 B/s wire rate).
3. Draft a 2-minute speaker script for defense/security judges explaining how ArcDrone reduces bandwidth by ~1800x vs raw video while maintaining 1:1 ground tactical awareness.
```

---

## 6. Common Pitfalls & LLM Debugging Traps

1. **Do not use `pty` on Windows:** POSIX pseudo-terminals fail on Windows. Use `scripts.sim_link_bridge.SimLinkBridge` which implements an in-memory virtual serial pair compatible with `pyserial`.
2. **FourCC Video Codecs:** On Windows, OpenCV `cv2.VideoWriter_fourcc(*'mp4v')` produces reliable `.mp4` output without external ffmpeg binaries.
3. **Bandwidth Windowing:** Bandwidth is evaluated via sliding window. Brief sub-second frame bursts are normal; the moving 1-second average must remain below 3,000 bytes/sec.
4. **Git Branching:** Ensure commits for Manan's test and showcase suite remain isolated to `manan/test-lora-demo` and `maybe`.
