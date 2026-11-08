# E2E Test Infra: ArcDrone natHacks Defense & Security Showcase

## Test Philosophy
- **Requirement-Driven & Opaque-Box**: Tests are derived directly from `ORIGINAL_REQUEST.md` (R1, R2, Acceptance Criteria) and natHacks shareholder priorities (`Manan Context`).
- **Hardware-Free Execution**: All tests run completely headlessly without physical LoRa hardware using cross-platform simulated link bridges (`scripts/sim_link_bridge.py`).
- **Strict Teammate Boundaries**: Verification executes against existing codebase interfaces while only modifying and executing code in `scripts/` on branch `manan/test-lora-demo`.
- **4-Tier Methodology**:
  - Tier 1: Feature Coverage (representative happy-path tests per feature).
  - Tier 2: Boundary & Corner Cases (bandwidth thresholds, sequence overflows, buffer limits, CRC corruption).
  - Tier 3: Cross-Feature Interactions (link outage + operator command, burst detection + bandwidth cap, UAARG logging + disconnects).
  - Tier 4: Real-World Mission Scenarios (full natHacks defense showcase loops).

## Feature Inventory & Test Coverage
| # | Feature | Requirement Source | Tier 1 | Tier 2 | Tier 3 | Tier 4 |
|---|---------|-------------------|:------:|:------:|:------:|:------:|
| F1 | Simulated Link Bridge | ORIGINAL_REQUEST Acceptance Criteria | 5 | 5 | ✓ | ✓ |
| F2 | Bandwidth Budget (<= 3 kB/s) | ORIGINAL_REQUEST R1, Shareholder Brief | 5 | 5 | ✓ | ✓ |
| F3 | UAARG Telemetry & Mission Status | ORIGINAL_REQUEST R2, UAARG emu | 5 | 5 | ✓ | ✓ |
| F4 | UAARG Hierarchical Run Logger | ORIGINAL_REQUEST R2, UAARG shepard | 5 | 5 | ✓ | ✓ |
| F5 | Ground Station WebSocket Client | Survey Explorer 3, ground station API | 5 | 5 | ✓ | ✓ |
| F6 | Reliable Two-Way Control Loop | ORIGINAL_REQUEST R1, Acceptance Criteria | 5 | 5 | ✓ | ✓ |
| F7 | Link Loss & Outage Recovery | ORIGINAL_REQUEST R1, Acceptance Criteria | 5 | 5 | ✓ | ✓ |
| F8 | Automated Test Suite | ORIGINAL_REQUEST Acceptance Criteria | 5 | 5 | ✓ | ✓ |
| F9 | Unified natHacks Demo Harness | Manan Context Problem Brief | 5 | 5 | ✓ | ✓ |
| F10 | Git Branch Packaging & Isolation | High Priority User Constraint | 5 | 5 | ✓ | ✓ |

## Test Architecture
- **Test Runner**:
  - Primary invocation: `uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py`
  - Fallback / Direct CLI invocation: `python -m scripts.test_integration_suite`
  - Bandwidth assertion benchmark: `python -m scripts.test_lora_bandwidth`
  - Control loop verification: `python -m scripts.test_control_loop`
  - Link reliability & outage test: `python -m scripts.link_test`
- **Output Artifacts**:
  - Structured test logs in `scripts/test_results.json`
  - UAARG mission session logs in `tmp/log/<run_id>/`
  - Bandwidth auditor report in `scripts/bandwidth_report.json`

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Expected Outcome |
|---|----------|--------------------|------------------|
| 1 | Standard Patrol & Alert Telemetry | F1, F2, F3, F4 | Detections transmitted within <= 3 kB/s, telemetry logged in UAARG format |
| 2 | Heavy Detection Burst (10 detections/s) | F1, F2, F6 | Rate remains <= 3000 B/s (peak ~600 B/s), zero dropped packets |
| 3 | Operator Tactical Override (`HOLD` & `RESUME`) | F1, F5, F6 | Drone transitions `MODE_PATROL` -> `MODE_HOLD`, pauses CV video, transitions back on `RESUME` |
| 4 | 15-Second Radio Blackout During Active Mission | F1, F6, F7 | 3s timeout triggers link down, up to 64 alerts buffered, reconnect flushes oldest-first with DELAYED badge, 0 duplicates |
| 5 | Command Dispatched During Outage (`RTB`) | F1, F5, F6, F7 | Command remains PENDING during outage, delivers upon link recovery, confirms status, transitions drone to `MODE_RTB` |
| 6 | Full natHacks Showcase Demo Flow | F1, F2, F3, F4, F5, F6, F7, F9 | Complete automated presentation demonstrating detect -> alert -> ground display -> simulated link loss -> operator override |

## Acceptance Pass Criteria
- 100% of automated unit and integration tests pass cleanly with exit code 0.
- Primary communication link data rate measured <= 3000 bytes/sec during active transmission.
- Operator commands receive confirmation acknowledgements across simulated link drops.
- UAARG structured session logs (`tmp/log/<run_id>/`) contain verified timestamps, status indicators, and image/metadata files.
- Zero modifications to teammate files (`protocol/`, `firmware/`, `drone_node/`, `ground_station/`).
- Valid git branch `manan/test-lora-demo` containing only Manan's commits.
