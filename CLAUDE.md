# CLAUDE.md — Drone Zone (natHacks 2026, ArcDrone challenge)

## Working conventions (read first)
- **MVP-first:** get the simplest end-to-end thing working, then layer improvements. Never skip ahead of the current phase.
- **Keep code simple, clean, minimal.** No over-engineering, no speculative abstractions.
- **ESP32 firmware is a dumb serial<->radio bridge.** All protocol logic (seq, ACK, retry, dedup, heartbeat, buffering) lives in `protocol/` (Python), shared by drone and ground.
- **Packet spec in §4 is frozen.** Change it only deliberately, and update this file + README together.
- **Honest wording:** say "resistant to narrowband interference", "no LoRaWAN network overhead", "no router/AP required". Never "jam-resistant", "bypasses duty cycle", or "encrypted" (unless enabled).
- Shiven is using this hackathon to build embedded/hardware skills — explain firmware/serial/radio decisions rather than hiding them.
- Keep `README.md` progress checklist updated and add a line to `CHANGELOG.md` as work lands.
- Possible future: a real drone. It only changes the frame source (+ optional real GPS); protocol/firmware/ground station stay unchanged. Don't buy until Phase 4 passes.
- Python code must run on **Python 3.9+** (no `match`, no `X | None` hints). CI tests on 3.9.

---

# Team workflow (avoid blocking and merge conflicts)

## File ownership: only the owner edits these files
| Path | Owner | Branch |
|---|---|---|
| `protocol/`, `firmware/`, `scripts/smoke_test.py` | Shiven | `shiven/firmware-link` |
| `drone_node/` | Ishan | `ishan/cv-drone-node` |
| `ground_station/*.py` | Samar | `samar/ground-server` |
| `ground_station/static/` | Marriam | `marriam/dashboard-ui` |
| `scripts/` (everything else) | Manan | `manan/test-lora-demo` |

If you need a change in someone else's file, ask them. Don't edit it on your branch.

Shared files, with rules that keep merges clean:
- `CHANGELOG.md` and `requirements.txt`: **append lines only**, never reorder or edit existing lines. `.gitattributes` merges these by union, so they won't conflict.
- `CLAUDE.md` and `README.md`: Shiven edits them, or other people do so in a small PR of their own.

## Git rules
1. Work on your own branch. Commit small and often.
2. **Pull `main` into your branch at least twice a day:** `git pull origin main`.
3. Merge into `main` via a **small PR as soon as something works** (at least once per phase). CI (`pytest`) must pass.
4. **Contracts** (packet spec, `Link` API, WebSocket schema below) change only via a PR from Shiven, announced to the team. Code against them, not against someone's unfinished work.
5. Never commit `data/`, model weights, `.venv/`, or `.pio/` (already gitignored).

## Building without each other (and without hardware)
- **No radios?** Run `python -m protocol.fake_bridge`. It prints two virtual serial ports. Give one to the drone side and one to the ground side as `--port`. Press Enter to cut or restore the link, and pass `--loss 0.2` to drop 20% of packets. **Dev only. The demo must use the real ESP32s.**
- **No server yet?** The dashboard uses a demo mode (`index.html?demo=1`) that generates schema-valid fake events in JS.
- **No detector yet?** The drone side sends hardcoded DETECTIONs (`drone_node/fake_sender.py`).
- Run everything from the repo root as modules: `python -m drone_node.main --port ...`, `python -m ground_station.server --port ...`.

# Interface contracts

## `protocol.link.Link` (both sides)
```python
from protocol import packets as p
from protocol.link import Link

link = Link(port, on_packet=fn, on_event=fn).start()   # boot_wait=2.0, retry_timeout=0.3
seq = link.send(p.DETECTION, ts_ms=p.now_ms(), cls=3, conf=0.91, count=2, lat=53.52, lon=-113.52)
seq = link.send(p.COMMAND, cmd=p.CMD_HOLD, ref_seq=alert_seq)
link.mode = p.MODE_HOLD      # drone: reported in heartbeats
link.up, link.buffered       # current state
link.summary()               # {"up","buffered","sent","delivered","retries","duplicates","bad_packets","dropped","rtt_ms_avg","rtt_ms_p95"}
```
- `on_packet(packet)` receives each **new** DETECTION/COMMAND (already deduped) plus every HEARTBEAT, as a dict like `{"type": p.DETECTION, "seq": 7, "ts_ms": ..., "cls": 3, "conf": 0.91, "count": 2, "lat": ..., "lon": ...}`.
- `on_event(name, **info)` receives `"link_up"`, `"link_down"`, or `"delivered"` with `seq=` and `rtt_ms=`. A COMMAND is CONFIRMED when its `seq` is delivered.
- Callbacks run on the link thread. Keep them fast (push to a `queue.Queue`).
- Names for the UI: `p.CLASS_NAMES[cls]`, `p.CMD_NAMES[cmd]`, `p.MODE_NAMES[mode]`.

## WebSocket schema: `ground_station/server.py` ↔ `static/index.html`
Endpoint: `ws://localhost:8000/ws`. Messages are JSON objects.

Server → browser:
```json
{"event": "alert", "seq": 4711, "cls": "car", "conf": 0.91, "count": 2, "lat": 53.52, "lon": -113.52, "ts_ms": 1730918400123, "delayed": false}
{"event": "link", "up": true}
{"event": "drone", "mode": "PATROL", "buffered": 0}
{"event": "cmd_status", "cmd_id": 812, "cmd": "HOLD", "ref_seq": 4711, "status": "PENDING"}
{"event": "stats", "up": true, "buffered": 0, "sent": 40, "delivered": 40, "retries": 3, "duplicates": 1, "bad_packets": 0, "dropped": 0, "rtt_ms_avg": 9.2, "rtt_ms_p95": 14.0}
```
- `alert.delayed` is true when it arrives more than 3 s after `ts_ms`, meaning it was buffered during link loss.
- `cmd_status.status` is `"PENDING"` when the command is sent and `"CONFIRMED"` when the drone ACKs it. `cmd_id` is the link seq.
- `drone` comes from heartbeats (1 Hz). `stats` is sent every 1 s.

Browser → server:
```json
{"action": "command", "cmd": "HOLD", "ref_seq": 4711}
```
`cmd` is one of `OPERATOR_ACK`, `HOLD`, `RESUME`, `RTB`. `ref_seq` is the alert being answered, or 0.

## Drone reaction to COMMANDs (`drone_node`)
| cmd | Drone does | `link.mode` |
|---|---|---|
| OPERATOR_ACK | Overlay "Alert #ref_seq acknowledged by operator" | unchanged |
| HOLD | Freeze playback, banner "HOLD — operator override" | `MODE_HOLD` |
| RESUME | Resume playback | `MODE_PATROL` |
| RTB | Banner "RETURNING TO BASE" | `MODE_RTB` |

---

# Drone Zone — natHacks 2026 ArcDrone Plan

## Context
ArcDrone brief: detect person/vehicle in VisDrone footage with an existing model → send a compact alert over a **real physical radio link** → show it in ground control → survive drops/link loss → operator acknowledges or sends an override back to the "drone." Judges score the **detection→operator-action loop and its reliability**, not the CV.

What you have: **1× Raspberry Pi 4, 2× ESP32 (ESP-NOW capable), M4 Pro MacBooks, LoRa radios TBD.** 5 people, 64 hours. Repo is empty.

That hardware reality changes the team's stated plan. Here's the corrected plan.

---

## 1. Corrections to the current plan (direct)

| Current plan | Problem | Fix |
|---|---|---|
| LoRa first, ESP-NOW if time | You don't own LoRa radios. LoRa-first blocks your critical path on shipping and a new radio library. Your own screening table also ranked LoRa alone *lowest* (38 vs 45). | **ESP-NOW first.** LoRa becomes a firmware-only swap later (see §2). |
| Protocol logic on ESP32 | Then ESP-NOW→LoRa means rewriting retries/buffering in C++ twice. | **ESP32 = dumb radio bridge.** All protocol logic lives in one Python module used by both sides. |
| 150-byte JSON | Fits ESP-NOW (250 B max) but costs ~5× airtime on LoRa (≈250 ms at SF7, >1 s at SF10). | **Packed binary struct (~21 B) from day one.** In Python, `struct.pack` is no harder than `json.dumps`. Doing it now means no protocol rewrite before LoRa. |
| Confidence gate >85% | VisDrone objects are tiny. YOLO rarely scores them above 0.85, so you risk **zero alerts on demo day**. At 1 FPS with no rate limit, a lower gate floods the link instead. | Gate at about 0.5 (tune it on the demo clip), send **at most one alert per frame** (top detection plus count), and add a **per-class cooldown** of about 3 s. |
| Next.js dashboard | The serial port and protocol state must sit in one process. Next.js forces a second runtime (Node serialport, or Python plus a bridge into Node). | **Python FastAPI + one static HTML/JS page** (details in §5). |
| Real GPS | VisDrone has no GPS. | Simulated drone track (scripted waypoints) plus an offset for each detection. Label it "simulated GPS" in the UI. |

**LoRa-first vs ESP-NOW-first, honestly:** ESP-NOW first wins on every axis that matters during a hackathon. The hardware is in hand, the API ships with the ESP32 core, latency is a few ms, and both boards run identical firmware. The pitch story still holds: **ESP-NOW needs no router, AP, or network infrastructure.** LoRa adds range and is the redundancy upgrade, not the foundation.

---

## 2. Recommended architecture

```
DRONE SIDE (Pi 4, fallback: MacBook)                GROUND SIDE (MacBook)
┌─────────────────────────────────┐                 ┌──────────────────────────────────┐
│ drone_node (Python)             │                 │ ground_station (Python)          │
│  video → YOLO → gate/cooldown   │                 │  FastAPI + WebSocket             │
│  → protocol.Link (seq, ACK,     │                 │  protocol.Link (same module)     │
│    retry, ring buffer, HB)      │                 │  → browser dashboard (HTML/JS)   │
│  OpenCV window: boxes + MODE    │                 │    alerts, map, link status,     │
└──────────┬──────────────────────┘                 │    ACK/HOLD/RESUME/RTB buttons   │
           │ USB serial 115200, hex lines           └──────────┬───────────────────────┘
┌──────────▼──────────┐   ESP-NOW 2.4 GHz (later +LoRa 915)   ┌▼────────────────────┐
│ ESP32 bridge fw     │ ◄───────────────────────────────────► │ ESP32 bridge fw     │
│ (identical binary)  │                                       │ (identical binary)  │
└─────────────────────┘                                       └─────────────────────┘
```

**Components (what it is, why, and the simplest version):**

- **Radio bridge firmware (ESP32, Arduino framework via PlatformIO).** Reads a hex line from serial, decodes it to bytes, and ESP-NOW-broadcasts it. On receive it hex-encodes the bytes and prints a line. About 60 lines.
  - *Why broadcast rather than unicast:* there are no MAC addresses to hardcode, and the same binary goes on both boards. The magic byte and CRC filter out other teams' traffic.
  - *Why a dumb bridge:* swapping the radio never touches the protocol, and debugging happens in Python, which you already know.
  - Learning goal for Shiven: you own this plus `link.py`. That's the real embedded work: serial framing, radio callbacks, timing.
- **protocol/ (Python, shared).**
  - `packets.py` handles struct encode/decode and the CRC16 (`binascii.crc_hqx`).
  - `link.py` handles serial I/O in a background thread, stop-and-wait reliable send, dedup, heartbeat, link-state detection, ring buffer, and stats.
  - *Why shared:* you write and test the reliability logic once, and both ends behave the same way.
- **drone_node (Python, Ultralytics + OpenCV).**
  - Plays a VisDrone clip at about 1 FPS of inference and draws boxes.
  - Overlays MODE (PATROL / HOLD / RTB), link state, and buffered count.
  - Reacts to commands visibly: HOLD freezes playback with a banner, RTB shows a banner.
  - Model: a community VisDrone YOLOv8n checkpoint from Hugging Face. Fallback is COCO `yolov8n`/`yolo11n` (person/car classes).
  - *Pi 4 vs Mac:* this is decided by a benchmark gate in Phase 2 (§3).
- **ground_station (Python FastAPI + WebSocket + one `index.html`, vanilla JS).**
  - One process owns the serial port, the protocol state, and pushes events to the browser.
  - The map is **Leaflet vendored locally** (truly offline), using `CRS.Simple` plus one image overlay of the operating area. No tile server.
- **LoRa later: buy 2× Heltec WiFi LoRa 32 V3, 915 MHz version, with antennas. Order today.**
  - It's ESP32-S3 + SX1262 on one board, so it does **both ESP-NOW and LoRa**. That enables dual-path redundancy with no extra hardware (Stretch S1).
  - Use RadioLib at SF7 / BW125 / CR4/5 for low airtime.
  - Never transmit without an antenna attached; it can damage the PA.

---

## 3. Phased build plan (64 h, 5 people)

Roles:
- **A = Shiven** (firmware + link layer)
- **B** (CV / drone_node)
- **C** (ground server)
- **D** (dashboard UI)
- **E** (test harness, LoRa procurement and integration, demo/pitch)

B, D, and E start at hour 0 in parallel. None of them need radios at first.

**Hard rule:** the end-to-end loop works by about hour 14. Feature freeze at hour 50. Hours 50–64 are for rehearsal, bug fixes, and sleep.

| Phase | Deliverable | Est. | Owner | Done when |
|---|---|---|---|---|
| **0 Smoke test** | Bridge firmware flashed on both ESP32s; typed hex lines in a serial monitor show up on the other board's monitor | 2–3 h | A | Both directions echo correctly for 60 s, 2 m apart, with no garbled lines |
| **1 MVP loop** | `packets.py` + minimal `link.py`. `drone_node/fake_sender.py` sends a hardcoded DETECTION every 2 s. `ground_station/cli.py` prints it and ACKs. Sender prints the RTT | 4–6 h | A + C | 100 sent → 100 ACKed, RTT printed for each packet, CRC failures counted (expect 0) |
| **2 Real detections** | `detector.py` runs YOLO on the VisDrone clip with gate, cooldown, and one alert per frame, and feeds `link.send()`. **Benchmark gate:** if the Pi 4 can't sustain ≥1 FPS with yolov8n exported to NCNN at imgsz 640 by about hour 20, the MacBook becomes the drone side (same code) | 6–8 h (from h0 on the Mac) | B | On the chosen demo clip, alerts arrive at the ground CLI at a steady 0.2–1/s with sensible classes, with no flood and no long silences |
| **3 Reliability** | seq IDs, stop-and-wait ACK + retry (jittered timeout), dedup with re-ACK, 1 Hz heartbeat both ways, LINK LOST after 3 s of silence, drone ring buffer (64 entries) flushed oldest-first on restore, serial auto-reconnect | 8–10 h | A (E writes the test script) | Scripted test: with the drone radio blocked for 15 s during detections, ground shows LINK LOST within ≤3 s; after restore, **every** buffered alert is delivered **exactly once**. Unplugging and replugging the ground ESP32 USB recovers without a restart |
| **4 UI + override** | FastAPI server + dashboard: alert feed (DELAYED badge for buffered alerts), map pins, link status light, RTT/loss stats. Buttons: **Acknowledge alert**, **HOLD**, **RESUME**, **RTB**. A command shows PENDING until the drone's ACK arrives, then CONFIRMED | 8–12 h (D starts h0 on a fake event feed) | C + D | Clicking HOLD freezes the drone window with a "HOLD — operator override" banner within 1 s. The ground shows CONFIRMED only after the ACK. A HOLD sent during link loss stays PENDING and lands after restore |
| **5 Stretch** (ranked by judge impact per hour) | S1. **Dual-path ESP-NOW + LoRa** on Heltec boards: firmware sends every packet on both radios and the existing dedup drops duplicates (≈6 h, A + E). S2. **On-screen metrics panel**: delivered %, retries, avg/p95 RTT, payload bytes (≈2 h, D). S3. **"Mute radio" button** on the drone ESP32 BOOT pin as a deterministic link-loss demo (≈1 h, A). S4. LoRa-only mode with an airtime readout (≈2 h after S1). S5. Snapshot thumbnail sent over Wi-Fi out-of-band — **skip**, it muddies the low-bandwidth story | — | — | Each stretch item has its own demo moment and doesn't break Phase 3's test |

Unblocking agreements on hour 0:
- Freeze the packet spec (§4).
- Freeze the server→browser WebSocket event JSON (`{"event":"alert"|"link"|"cmd_status"|"stats", ...}`) so D can build against fake data.

---

## 4. Message protocol (little-endian, ≤21 bytes)

Common header (4 B): `magic u8 (0xD2)` · `type u8` · `seq u16`. Trailer: `crc16 u16` over all preceding bytes.

| Type | Code | Body after header | Total |
|---|---|---|---|
| DETECTION | 0x01 | `ts_ms u32` · `cls u8` · `conf u8 (0–255)` · `count u8` · `lat i32 (1e-7°)` · `lon i32 (1e-7°)` | 21 B |
| ACK | 0x02 | (header `seq` = the seq being acknowledged) | 6 B |
| HEARTBEAT | 0x03 | `ts_ms u32` · `mode u8` · `buffered u8` | 12 B |
| COMMAND | 0x04 | `cmd u8` (0=OPERATOR_ACK, 1=HOLD, 2=RESUME, 3=RTB) · `ref_seq u16` (alert being answered, or 0) | 9 B |

COMMAND is your OVERRIDE type, widened so the operator's *acknowledge* and the *override* share one reliable path.

**Rules:**
- **Reliable types:** DETECTION and COMMAND. The sender keeps one message in flight (stop-and-wait). Timeout is 300 ms + 0–100 ms jitter on ESP-NOW (1 s on LoRa), with up to 5 retries.
  - If all retries fail, the drone moves the message to the ring buffer and marks the link degraded.
  - The ground keeps a COMMAND PENDING and keeps retrying while the link is up.
- **Unreliable types:** ACK and HEARTBEAT are never ACKed or retried.
- **Dedup:** the receiver remembers the last 64 seqs per peer. On a duplicate it **re-sends the ACK** (the first ACK may have been lost) but doesn't re-display the message.
- **Seq:** uint16, wraps around, and starts from a **random value at boot**, so a restarted drone isn't treated as sending duplicates.
- **Link state:** each side sends a HEARTBEAT at 1 Hz and declares LINK LOST after 3 s with no valid packet. Any valid packet restores the link and triggers a buffer flush (oldest-first, original `ts_ms` kept, so the UI tags it DELAYED).
- **Latency metric:** use the **ACK round-trip time measured on the sender**. It needs no clock sync between the Pi and the Mac, which avoids measuring bad one-way numbers.
- **Integrity:** link-layer CRCs already exist (ESP-NOW FCS, LoRa payload CRC). The app CRC16 also covers the serial hop and foreign traffic, and costs one line.
- **Serial framing:** one packet per line as uppercase hex + `\n`. Hex is easy to read in a serial monitor and has no escaping problems.

---

## 5. Repo structure and stack

```
ArcDrone/
├── protocol/           # Python: packets.py (struct+CRC), link.py (reliability), test_packets.py
├── firmware/
│   └── bridge/         # PlatformIO project; src/main.cpp; env: esp32dev (+ heltec_v3 later)
├── drone_node/         # Python: main.py, detector.py, sim_gps.py, fake_sender.py
├── ground_station/     # Python: server.py (FastAPI+WS), cli.py (Phase 1), static/index.html, static/leaflet/, static/map.png
├── scripts/            # link_test.py (Phase 3 scripted test), bench_pi.py
├── data/               # VisDrone clip(s), gitignored
└── requirements.txt    # pyserial, ultralytics, opencv-python, fastapi, uvicorn
```

There are two languages total (Python, C++), one Python venv, and one firmware project.

**Next.js vs Python dashboard, the tradeoff:**
- Next.js gives you nicer components and a polished look.
- It costs a second toolchain, a Node↔Python boundary, and a second place for the serial and protocol state to live. That's more integration risk for zero judging points.
- A well-styled single HTML page looks just as "tactical" for this demo.
- **Recommendation: Python.** If the team insists on Next.js, use it **only as a frontend** that consumes the Python WebSocket. Never let Node own the serial port.

---

## 6. Top 5 demo-killing risks

1. **ESP32 serial flakiness.**
   - Failure modes: charge-only USB cables, macOS port naming, and DTR auto-reset when the port opens (the first bytes get lost).
   - Mitigation: use `/dev/cu.*` on Mac, wait 2 s after opening, and auto-reconnect in `link.py`. Keep labeled data cables and pass the port explicitly via a CLI flag.
2. **No alerts or an alert flood from the model.**
   - Tiny VisDrone objects fail a 0.85 gate. 1 FPS with no cooldown floods the link.
   - Mitigation: pick and lock the demo clip by hour 24, tune the gate and cooldown on that clip, and keep a backup clip.
3. **Pi 4 too slow, or the torch install on the Pi turns into a time sink.**
   - Mitigation: benchmark gate by hour 20 (NCNN export). If it fails, the MacBook is the drone, using the same code. Pitch it honestly as "edge-capable; Pi 4 benchmark X FPS."
4. **2.4 GHz congestion at the venue** (hundreds of laptops).
   - Mitigation: retries are built for this (show the retry counter), keep the radios 1–3 m apart, and fix the ESP-NOW channel to whichever of 1, 6, or 11 tests quietest on-site.
   - LoRa at 915 MHz (S1) sidesteps the congestion and doubles as the redundancy story.
5. **LoRa arrives late, is the wrong band (868 MHz), or RadioLib misbehaves.**
   - Mitigation: LoRa is **off the critical path** by design. Order 915 MHz Heltec V3 today. If it isn't working by hour 44, cut it and demo ESP-NOW only.

**Wording to use:**
- Say "resistant to narrowband interference" (LoRa CSS), "no LoRaWAN network overhead," and "no router or access point required" (ESP-NOW).
- Don't say "jam-resistant" or "bypasses duty cycle."
- Don't say "encrypted" unless you enable ESP-NOW encryption.

---

## 7. 3-minute demo script

1. **0:00–0:20 — Problem and architecture.** One diagram: edge detection, a ~21-byte alert over an infrastructure-free radio, and an operator loop.
2. **0:20–0:55 — Detection → alert.**
   - The drone screen plays VisDrone footage with boxes. A person/vehicle triggers an alert.
   - The dashboard shows the alert card and map pin with RTT (for example, "RTT 9 ms, 21 bytes").
   - Point at the physical ESP32s: "This crossed a real radio link."
3. **0:55–1:20 — Operator acknowledge.** Click **Acknowledge**. The drone overlay shows "Alert #… acknowledged by operator," and the ground card turns CONFIRMED.
4. **1:20–2:10 — Link loss and recovery.**
   - Drop the drone ESP32 into a metal cookie tin (backup: the S3 mute button).
   - Within 3 s the dashboard goes **LINK LOST** (red), and the drone overlay shows "LINK DOWN — buffered: 1, 2, 3…".
   - Press **HOLD** during the outage. It shows PENDING.
   - Remove the tin. The link goes green, buffered alerts flush with DELAYED badges, and the stats show retries with **0 duplicates and 100% delivered**.
   - The pending HOLD lands: the drone footage freezes with the "HOLD — operator override" banner.
5. **2:10–2:35 — Resume / RTB.** Press RESUME and playback continues. Press RTB and the banner changes. Each step goes PENDING → CONFIRMED.
6. **2:35–3:00 — Metrics and close.** Show delivered %, avg/p95 RTT, retries, and payload size. If S1 shipped, unplug the LoRa antenna path or block ESP-NOW live and show traffic continuing on the other path. Close with the honest framing lines from §6.

---

## Verification (end-to-end)

- `pytest protocol/` checks encode/decode round-trips, CRC rejection of a flipped byte, and dedup re-ACK behaviour.
- Phase 0/1/3/4 "done when" tests from §3. `scripts/link_test.py` automates the Phase 3 test: it sends N alerts, the operator blocks the radio for 15 s, and it asserts exactly-once delivery and reports loss/RTT.
- Full dress rehearsal of §7 at least 3 times on the real demo machines, including one run with the Mac as the drone-side fallback.

