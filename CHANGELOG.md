# Changelog

Append one line at the bottom when your work lands: `- YYYY-MM-DD (name): what changed`. Don't edit existing lines.

- 2026-11-06 (Shiven): Repo scaffolded. Plan finalized: ESP-NOW first, binary protocol, Python dashboard.
- 2026-11-06 (Shiven): ESP32 ESP-NOW bridge firmware (compiles), `protocol/packets.py` + tests, Phase 0 smoke test script.
- 2026-11-06 (Shiven): `protocol/link.py` (ACK/retry, dedup, heartbeat, link loss, store-and-forward, serial reconnect) + `protocol/fake_bridge.py` for hardware-free dev. Team workflow + interface contracts in CLAUDE.md. CI on PRs.
- 2026-11-07 (Ishan): SceneCodec v2 (0xAE) keyframe/delta under 2.0 kbps; browser reconstructor + UDP/WebSocket relay.
- 2026-11-07 (Ishan): Browser reconstructor v2 UI — interactive per-object activity log and trace inspector grid.
- 2026-11-07 (Ishan): Reconstructor live activity panel (direction/pace/heading per target), behavior-change event history, click-to-unlock; tester guide in `drone_node/static/RECONSTRUCTOR_SETUP.md`.
