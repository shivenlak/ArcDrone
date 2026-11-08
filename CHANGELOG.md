# Changelog

Append one line at the bottom when your work lands: `- YYYY-MM-DD (name): what changed`. Don't edit existing lines.

- 2026-11-06 (Shiven): Repo scaffolded. Plan finalized: ESP-NOW first, binary protocol, Python dashboard.
- 2026-11-06 (Shiven): ESP32 ESP-NOW bridge firmware (compiles), `protocol/packets.py` + tests, Phase 0 smoke test script.
- 2026-11-06 (Shiven): `protocol/link.py` (ACK/retry, dedup, heartbeat, link loss, store-and-forward, serial reconnect) + `protocol/fake_bridge.py` for hardware-free dev. Team workflow + interface contracts in CLAUDE.md. CI on PRs.
- 2026-11-06 (Shiven): Firmware: BOOT-button radio mute + packet activity LED. `protocol/loop_test.py` for Phase 1/3 tests on real or fake radios. Branch protection on main (PR + passing CI).
- 2026-11-07 (Ishan): SceneCodec v2 (0xAE) keyframe/delta under 2.0 kbps; browser reconstructor + UDP/WebSocket relay.
- 2026-11-07 (Manan): Cross-platform simulated radio bridge, bandwidth auditor (<=3 kB/s budget compliance), UAARG emu/shepard telemetry & event-logging adapters, 4-tier integration test suite, and natHacks showcase presentation orchestrator.
- 2026-11-07 (Ishan): Browser reconstructor v2 UI — interactive per-object activity log and trace inspector grid.
- 2026-11-07 (Ishan): Reconstructor live activity panel (direction/pace/heading per target), behavior-change event history, click-to-unlock; tester guide in `drone_node/static/RECONSTRUCTOR_SETUP.md`.
- 2026-11-08 (Manan): Documentation & LLM Guide: Added scripts/README.md and LLM_GUIDE.md detailing 6-case benchmarks, 1:1 side-by-side showcase generator, live VisDrone UDP feeder, and model guidelines for Claude, Gemini, and ChatGPT.
- 2026-11-07 (Shiven): `shiven/integration`: main + `maybe` merged. Feather V2 firmware target. Phase 0/1 passed on real boards (1–2 cm, no antennas). README checklist reset to hardware-verified status.
- 2026-11-07 (Shiven): Phase 3 passed on hardware: SW38 mute (120/120 exactly once, LINK_DOWN 3.1 s) and USB unplug/replug auto-recovery (100/100).
- 2026-11-08 (Shiven): Phase 2 + 4 passed on hardware with the new continuous VisDrone-MOT demo clip (`--imgsz 1280 --fps 10`). HOLD during link loss lands after restore; queued commands delivered in order.
- 2026-11-08 (Shiven): Dashboard "On the wire" panel: each alert's exact 21 bytes split by field, plus measured radio bit/s vs LoRa SF7 capacity and 720p video. `Link.summary()` gains `bytes_tx`/`bytes_rx`; `alert` events gain `raw` (hex).
- 2026-11-08 (Manan): Presentation & Evaluation: Added PRESENTATION_PACK.md containing defense brief, 5-minute pitch deck, multi-scenario benchmark metrics table, and LLM slide/script generation prompt templates.
- 2026-11-08 (Manan): Security & Pitch Hierarchy: Implemented zero-overhead AES-128-CTR tactical encryption (scripts/secure_channel.py, test_encryption.py) with 0% payload inflation, and updated PRESENTATION_PACK.md with investor-facing 'Wow-ness' feature hierarchy.
- 2026-11-08 (Shiven): Heltec WiFi LoRa 32 V3 boards flashed (`heltec_v3` env). Link now releases DTR/RTS on open: on the Heltec, DTR drives GPIO0 (PRG/mute), so an open port held the radio muted. 100/100 delivered, 0 retries, RTT 19.9 ms.
- 2026-11-08 (Manan): Architecture & Setup: Added GETTING_STARTED.md and scripts/quickstart.py compiling all project workflows into 3 modular approaches (Virtual Trials, Real Hardware, Benchmarks & Analytics).
- 2026-11-08 (Shiven): Dual-path bridge on Heltec V3: every packet on ESP-NOW and LoRa 915 MHz (RadioLib, SF7/BW125, 10 dBm); Python dedup drops the second copy. PRG cycles BOTH / LORA ONLY / MUTED. OLED shows mode, tx/rx per radio, RSSI. LoRa only: 60/60, RTT 128 ms.
- 2026-11-08 (Ishan): Dashboard Scene twin (Manan 0xB4 reconstructor MJPEG :8082) + Live camera (:8081) overlays off the main screen; `drone_node.preview_stream`, `drone_node.manan_ground_panel`.
- 2026-11-08 (Ishan): One-command `drone_node.run_demo` (footage->radio->dashboard); detector `--imgsz` (default 1280) + per-contact track dedup; bounded figure-eight `sim_gps` patrol.
- 2026-11-08 (Ishan): `drone_node.frame_source` accepts VisDrone-VID JPEG folders as `--video`; `drone_node.demo_video` auto-picks clips/sequences from `data/`.
- 2026-11-08 (Ishan): Fix Manan Scene twin — full 1920×960 ishan_side_by_side composite, per-frame tracker clock, VisDrone box filter (was cropped wrong panel).
- 2026-11-08 (Ishan): Grid-aware coasting (16×16) in Scene twin — edge contacts (outer 3 rings) drop in 2 frames, central contacts coast 3 frames (`GridCoastTracker`).
- 2026-11-08 (Manan): Enhanced Architectural Features: Integrated dynamic role learning & dual-path radio simulation (scripts/sim_link_bridge.py), grid-aware coasting tracker & high-res imgsz tuning (scripts/ishan_side_by_side.py), monotonic clock safety, and comprehensive unit tests (scripts/test_enhanced_features.py).
- 2026-11-08 (Shiven): Optional AES-128-CCM link encryption (`ARCDRONE_KEY`, +8 B/packet, plain/tampered/wrong-key frames rejected; tested on Heltec V3). Docs updated to measured figures only: README "Measured results", CLAUDE.md secure frame spec + real-numbers rule, firmware README board table.
- 2026-11-08 (Manan): Encryption Harmonization & Pitch Metrics: Aligned evaluation scripts (secure_channel.py, test_encryption.py) with canonical AES-128-CCM AEAD (ARCDRONE_KEY, +8B overhead), enabled 0xD3 secure frame dynamic role learning in sim_link_bridge.py, and updated PRESENTATION_PACK.md with real measured Heltec V3 hardware numbers.
- 2026-11-08 (Manan): Industry Expert Audit Engine: Implemented scripts/industry_expert_audit.py and configured a recurring 30-minute automated systems audit evaluating defense constraints, bandwidth limits, cryptographic posture, and tactical gaps.
- 2026-11-08 (Ishan): Delivery verifier on drone HUD + dashboard (seq ACK / queue / flush) using existing `protocol.Link` outbox — no database.
