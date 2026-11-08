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
