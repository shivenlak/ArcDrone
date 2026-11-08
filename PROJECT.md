# Project: ArcDrone natHacks Defense & Security Showcase

## Architecture
ArcDrone connects onboard drone computer vision intelligence to ground station real-time decision-making over a low-bandwidth primary radio link (<= 3 kB/s). The system features:
- **Drone Node (`drone_node/`)**: Edge YOLO detection with confidence gating and per-class cooldown, GPS ground projection, and operational state machine (`MODE_PATROL`, `MODE_HOLD`, `MODE_RTB`).
- **Ground Station (`ground_station/`)**: FastAPI server and tactical WebSocket dashboard (`/ws`), alert feed, coordinate grid, and operator command dispatch (`HOLD`, `RESUME`, `RTB`, `OPERATOR_ACK`).
- **Protocol Layer (`protocol/`)**: Compact binary packet codec (`DETECTION`: 21B, `ACK`: 6B, `HEARTBEAT`: 12B, `COMMAND`: 9B) with CRC16 and stop-and-wait reliable link transport (`Link`).
- **Manan's Test & Adaptation Harness (`scripts/`)**: Cross-platform simulated radio bridge, <= 3 kB/s bandwidth monitor, two-way control loop test harness, UAARG telemetry & logging adaptation package (`uaarg_adapter`), and automated headless verification suite on branch `manan/test-lora-demo`.

### Teammate Ownership Contracts
| Path | Owner | Branch | Immutability Rule |
|---|---|---|---|
| `protocol/`, `firmware/`, `scripts/smoke_test.py` | Shiven | `shiven/firmware-link` | READ-ONLY / NEVER MODIFY |
| `drone_node/` | Ishan | `ishan/cv-drone-node` | READ-ONLY / NEVER MODIFY |
| `ground_station/*.py` | Samar | `samar/ground-server` | READ-ONLY / NEVER MODIFY |
| `ground_station/static/` | Marriam | `marriam/dashboard-ui` | READ-ONLY / NEVER MODIFY |
| `scripts/` (all new files) | Manan | `manan/test-lora-demo` | MANAN EXCLUSIVE WRITE DOMAIN |

## Feature Inventory
| # | Feature | Description | Milestone | Status | Source |
|---|---------|-------------|-----------|--------|--------|
| F1 | Cross-Platform Simulated Link Bridge | Virtual serial / socket bridge connecting `protocol.link.Link` instances in-memory and cross-platform (Windows & Linux) without POSIX PTYs or physical hardware | M1 | DONE | Survey Explorers 1, 2, 3 |
| F2 | Bandwidth Monitor & Assertion Auditor | Real-time sliding window byte rate auditor asserting primary link data rate <= 3000 B/s (3 kB/s) during active transmission | M1 | DONE | ORIGINAL_REQUEST R1, Manan Context |
| F3 | UAARG Telemetry & Mission Monitoring | Adapts UAARG `emu` liveness timer (`timeSinceMessage`), 3-tier severity logs (`normal`, `warning`, `error`), and subsystem tagging | M1 | DONE | ORIGINAL_REQUEST R2, UAARG emu |
| F4 | UAARG Hierarchical Event & Image Logger | Adapts UAARG `shepard` session-based logging (`tmp/log/<run_id>/`) saving raw frames (`images/`), annotated frames (`bb/`), and structured telemetry snapshots (`<seq>.json`) | M1 | DONE | ORIGINAL_REQUEST R2, UAARG shepard |
| F5 | Ground Station WebSocket Observer Client | Non-invasive client connecting to `ws://localhost:8000/ws` to capture alerts and inject operator commands without modifying ground station code | M1 | DONE | Survey Explorer 3 |
| F6 | Reliable Two-Way Control Loop Harness | Verifies operator commands (`HOLD`, `RESUME`, `RTB`, `OPERATOR_ACK`), stop-and-wait retries, ACK confirmations, and drone state transitions | M1 | DONE | ORIGINAL_REQUEST R1, Acceptance Criteria |
| F7 | Link Loss & Outage Recovery Verification | Simulates link loss (15s blackout), asserts 3.0s link-lost timeout, ring buffer accumulation (up to 64), oldest-first flush, and zero duplicates | M1 | DONE | ORIGINAL_REQUEST R1, Acceptance Criteria |
| F8 | Automated Programmatic Test Suite | Pytest-compatible automated suite running all unit & integration tests headlessly on Windows/Linux without hardware | M1 | DONE | Acceptance Criteria Verification Suite |
| F9 | Unified natHacks Showcase Demo Harness | Interactive and scripted showcase orchestrator executing full detection-to-response loop for natHacks presentation | M1 | DONE | Manan Context Problem Brief |
| F10 | Git Branch Packaging & Isolation | Prepares and validates git commits on branch `manan/test-lora-demo`, strictly preserving teammate files untouched | M1 | DONE | High Priority User Constraint |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Manan's natHacks Defense Showcase Suite (`scripts/`) | F1-F10: Simulated bridge, bandwidth auditor, UAARG adapter package, two-way control harness, and automated test suite on `manan/test-lora-demo` | none | **DONE** |
| M2 | Final Acceptance & Adversarial Hardening | Phase 1: 100% E2E test pass (Tiers 1-4). Phase 2: Adversarial coverage hardening (Tier 5). | M1, TEST_READY.md | **DONE** |

## Interface Contracts
### `scripts/sim_link_bridge.py` ↔ `protocol.link.Link`
- Exposes `VirtualSerialPair` or `create_linked_pair()` returning objects conforming to `pyserial` interface (`read()`, `write()`, `in_waiting`, `reset_input_buffer()`, `close()`).
- Provides `cut()` and `restore()` methods for simulated radio blackout.
- Provides configurable `packet_loss_rate` (0.0 - 1.0) and simulated latency.

### `scripts/bandwidth_monitor.py` ↔ Primary Link Transport
- Intercepts or tracks transmitted byte counts (both raw binary wire bytes and serial hex framing).
- Method `record_tx(num_bytes, timestamp)` / `check_budget()` returning boolean `within_budget` and current `bytes_per_sec`.
- Budget threshold: 3000 bytes/sec (<= 3 kB/s).

### `scripts/uaarg_adapter/event_logger.py` ↔ Session Logging
- Directory structure: `tmp/log/<run_id>/`
- Output files: `images/<seq>.png`, `bb/<seq>.png`, `<seq>.json`.
- Schema:
  ```json
  {
    "seq": 1,
    "timestamp_ms": 1731017400000,
    "location": {"lat": 53.5232, "lon": -113.5263},
    "altitude_m": 15.0,
    "mode": "PATROL",
    "detection": {"cls": "car", "conf": 0.88, "count": 1},
    "link": {"up": true, "buffered": 0, "rtt_ms": 12.0}
  }
  ```

### `scripts/uaarg_adapter/mission_monitor.py` ↔ Telemetry & Logging
- Severity levels: `"normal"`, `"warning"`, `"error"`.
- Subsystem prefixes: `ARCDRONE_EDGE`, `ARCDRONE_GROUND`, `ARCDRONE_LINK`.
- Liveness timer: increments every 1.0s, resets to 0 upon valid incoming message.

### `scripts/test_control_loop.py` ↔ Operator Commands
- Commands tested: `CMD_OPERATOR_ACK` (0), `CMD_HOLD` (1), `CMD_RESUME` (2), `CMD_RTB` (3).
- State transitions: `MODE_PATROL` (0), `MODE_HOLD` (1), `MODE_RTB` (2).
- Status lifecycle: `PENDING` -> `CONFIRMED` upon `delivered` ACK event.

## Code Layout
```
C:\Users\aryam\source\repos\ArcDrone\
├── protocol/                      # READ-ONLY (Shiven)
├── firmware/                      # READ-ONLY (Shiven)
├── drone_node/                    # READ-ONLY (Ishan)
├── ground_station/                # READ-ONLY (Samar & Marriam)
└── scripts/                       # MANAN EXCLUSIVE WRITE DOMAIN
    ├── smoke_test.py              # Existing (Shiven - DO NOT TOUCH)
    ├── sim_link_bridge.py         # Cross-platform simulated radio bridge
    ├── bandwidth_monitor.py       # Rolling data rate auditor
    ├── test_lora_bandwidth.py     # Bandwidth verification benchmark
    ├── uaarg_adapter/             # UAARG adaptation package
    │   ├── __init__.py
    │   ├── event_logger.py        # Hierarchical session logger
    │   ├── mission_monitor.py     # UAARG liveness & severity stream
    │   └── ground_client.py       # Non-invasive WebSocket client
    ├── link_test.py               # Reliability, outage & buffer flush test
    ├── test_control_loop.py       # Two-way command state loop harness
    ├── test_integration_suite.py  # Pytest automated test runner
    ├── demo_harness.py            # Unified natHacks showcase script
    ├── run_live_showcase.py       # Real-time presentation runner
    └── bench_pi.py                # Edge Pi inference FPS benchmark
```
