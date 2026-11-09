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
- 2026-11-07 (Shiven): `shiven/integration`: main + `maybe` merged. Feather V2 firmware target. Phase 0/1 passed on real boards (1–2 cm, no antennas). README checklist reset to hardware-verified status.
- 2026-11-07 (Shiven): Phase 3 passed on hardware: SW38 mute (120/120 exactly once, LINK_DOWN 3.1 s) and USB unplug/replug auto-recovery (100/100).
- 2026-11-08 (Shiven): Phase 2 + 4 passed on hardware with the new continuous VisDrone-MOT demo clip (`--imgsz 1280 --fps 10`). HOLD during link loss lands after restore; queued commands delivered in order.
- 2026-11-08 (Shiven): Dashboard "On the wire" panel: each alert's exact 21 bytes split by field, plus measured radio bit/s vs LoRa SF7 capacity and 720p video. `Link.summary()` gains `bytes_tx`/`bytes_rx`; `alert` events gain `raw` (hex).
- 2026-11-08 (Shiven): Heltec WiFi LoRa 32 V3 boards flashed (`heltec_v3` env). Link now releases DTR/RTS on open: on the Heltec, DTR drives GPIO0 (PRG/mute), so an open port held the radio muted. 100/100 delivered, 0 retries, RTT 19.9 ms.
- 2026-11-08 (Shiven): Dual-path bridge on Heltec V3: every packet on ESP-NOW and LoRa 915 MHz (RadioLib, SF7/BW125, 10 dBm); Python dedup drops the second copy. PRG cycles BOTH / LORA ONLY / MUTED. OLED shows mode, tx/rx per radio, RSSI. LoRa only: 60/60, RTT 128 ms.
- 2026-11-08 (Shiven): Optional AES-128-CCM link encryption (`ARCDRONE_KEY`, +8 B/packet, plain/tampered/wrong-key frames rejected; tested on Heltec V3). Docs updated to measured figures only: README "Measured results", CLAUDE.md secure frame spec + real-numbers rule, firmware README board table.
- 2026-11-08 (Shiven): From `maybe` (Ishan): per-target tracker (one alert per new contact; same ~0.5 alerts/s on the demo clip, fewer repeats on a webcam), figure-eight sim GPS, VisDrone image-folder input, delivery verifier, `run_demo` dev launcher. Left out: MJPEG "Live camera"/"Scene twin" streams (video over Wi-Fi, against the low-bandwidth story) and the AES-CTR/audit scripts. Hardware run: 16/16 delivered.
- 2026-11-08 (Shiven): Failover you can see: bridge reports per-radio RSSI/counts once a second (`# radio` line → `Link.summary()["radio"]` → dashboard "Radio paths" tiles). LoRa listen-before-talk + heartbeat jitter (before, in DUAL mode the ground heard 0 LoRa packets: both boards transmitted in lock-step). Adaptive retry timeout (2.5× smoothed RTT, doubles per retry, cap 1.5 s). `heltec_v3_sf9` env for range. Failover runs: 80/80 each.
- 2026-11-08 (Shiven): Dashboard = Ishan's flight-deck redesign + our features: radio-path tiles (failover), colour-coded / encrypted packet inspector, Pending→Confirmed in demo mode (real packet bytes). Wi-Fi camera gated by an actual reachability probe of the drone's preview server (`--preview-port 8081`, `?drone=<ip>`), labelled as Wi-Fi video, not radio. Fixed: stats overwriting the drone buffered count; removed invented "ALT 120m". Scene-twin stream dropped.
- 2026-11-08 (Shiven): Drone camera inline on the dashboard (tab [02]) with live numbers from the drone's `/stats.json` over Wi-Fi: YOLO ms, frames/s, objects, alerts sent vs received, radio vs video bit/s. Preview now sends each frame once (it was resending duplicates, which inflated the earlier 15 Mbit/s figure; measured 5.1 Mbit/s).
- Take Manan's real YOLO benchmark (scripts/bench_pi.py); the old one slept instead of running the model.
- README: scene feed is Wi-Fi (not radio) and its 2 kbps is a budget; SF9 range is theory; loop_test names the right mute button.
- protocol/range_walk.py: per-second range logger (radio heard, RSSI/SNR, delivery, RTT) with floor marks, to CSV.
- Range walk measured: ESP-NOW lost 2 floors down, LoRa SF7/10 dBm alone to the 6th floor, link gone by the 5th (README).
- Both Heltecs now on SF9 (heltec_v3_sf9). Desk: 60/60 both radios; LoRa-only failover 80/80 at 1 alert/s, RTT 379 ms; 2/s is too fast for SF9.
- 2026-11-09 (Manan): One-click Windows setup and launchers: setup.bat, run_receiver.bat (ground station + auto-browser), run_sender.bat (airborne node + video/port discovery), and run_simulation.bat.
- 2026-11-09 (Manan): macOS one-click scripts (.sh and double-clickable .command for Finder) with /dev/cu.* serial priority and Darwin auto-detection.
- 2026-11-09 (Ishan): Rebuilt dashboard in a brutalist/monospace engineering theme (hard corners, flat buttons, no animation/blur); 3 stages (init/ops/aux), reconstructor iframe embed, kept the frozen WS schema (event-based, {action:"command",cmd,ref_seq}) and ?demo=1 schema-valid simulator.
- 2026-11-09 (Ishan): Receiver + simulation launchers now start scene_ws_relay (:8080); reconstructor keeps its built-in sim until the first UDP frame so the OPS iframe is not a blank refused connection.
- 2026-11-09 (Ishan): OPS panel now embeds demo's reconstructor.html from the dashboard static dir (`/reconstructor.html` on :8000) instead of depending on :8080 for the page itself.
- 2026-11-09 (Ishan): Reconstructor UI restyled to the maybe-branch 1:1 side-by-side test bench (RGB/YOLO left, no-RGB ratio vectors right, dual index manifest) while keeping live UDP ingest from scene_ws_relay.
- 2026-11-09 (Ishan): Pulled 6 VisDrone-VID val sequences (same UAV clips as VisDrone-MOT) from the official dataset mirror, encoded 150-frame 1280p samples under data/clips/, and added a clip switcher on the reconstructor.
- 2026-11-09 (Ishan): Wired all 6 test cases in reconstructor.html, run_test_cases.py, and run_live_showcase.py to real VisDrone-VID/MOT frame-by-frame multi-object tracking annotations (`build_visdrone_clip_tracks.py` -> `/clips/uav*.json`).
