# ESP32 bridge firmware

A plain serial↔radio relay. The same binary runs on both boards.

- **Serial → radio:** one packet per line as hex (`48454C4C4F\n`). The bytes are broadcast over ESP-NOW on channel 1, and on the Heltec V3 also over LoRa (915 MHz, SF7, BW125, CR 4/5, 10 dBm).
- **Radio → serial:** each received packet is printed as one uppercase hex line. With both radios on, the receiver usually gets two copies; the Python dedup drops the second.
- Lines starting with `#` are diagnostics (`# bridge ready mac=… lora=915MHz/SF7`, `# err bad hex`, `# muted`, `# lora only`).
- **Button** cycles the radio mode. It's the deterministic way to demo link loss and failover. The LED stays solid while muted and flickers on every packet otherwise.
- **OLED (Heltec V3):** DRONE/GROUND (learned from traffic), mode badge, per-radio signal bars, RSSI, tx/rx counts, and the last packet.

| Board | env | Button | LED | Button cycles |
|---|---|---|---|---|
| Heltec WiFi LoRa 32 V3 | `heltec_v3` | PRG (GPIO0) | GPIO35 | DUAL → LORA ONLY → MUTED |
| Adafruit ESP32 Feather V2 | `feather_v2` | SW38 (GPIO38) | GPIO13 | ESP-NOW ↔ MUTED |
| Generic ESP32 DevKit | `esp32dev` | BOOT (GPIO0) | GPIO2 | ESP-NOW ↔ MUTED |
| Heltec V4 | `heltec_v4` | PRG | GPIO35 | ESP-NOW only, untested |

**Heltec: never power it without the LoRa antenna attached.** The firmware transmits LoRa at boot.

On the Heltec, the USB chip's DTR line drives GPIO0 (the PRG button), so an open serial port with DTR asserted holds the button down. `protocol/link.py` releases DTR/RTS after opening the port for this reason.

The bridge knows nothing about the protocol. That lives in `protocol/` (Python). The OLED only peeks at the type and seq bytes to label packets.

## Flash (each board)

```bash
source .venv/bin/activate            # platformio is in requirements.txt
cd firmware/bridge
pio device list                      # find the port, e.g. /dev/cu.usbserial-0001
pio run -e heltec_v3 -t upload --upload-port /dev/cu.usbserial-0001
```

If upload hangs at `Connecting...`, hold the **BOOT/PRG** button, tap **RST**, then release. If no port shows up, the cable is probably charge-only. Also check the CP210x/CH340 driver.

## Phase 0 test

1. Manual: open `pio device monitor -p <port>` on both boards. Type `48454C4C4F` and press Enter on one. The other prints `48454C4C4F`.
2. Measured: run `scripts/smoke_test.py` on both ports at once (see its docstring). **Done when** both sides receive ~all messages for 60 s, 2 m apart, with `garbled=0`.

## Phase 1 test (after Phase 0 passes)

```bash
python -m protocol.loop_test --role ground --port <groundPort>
python -m protocol.loop_test --role drone  --port <dronePort> --count 100
```
**Done when** the drone summary shows `delivered: 100` and the ground shows `received: 100, unique: 100`. Press the button on the drone board mid-run (Heltec: twice, to reach MUTED): both sides should print LINK_DOWN within about 3 s, then LINK_UP after you press it again (Heltec: once more, back to DUAL), still with 100/100. That's an early run of the Phase 3 test.

## Measured on hardware

- Heltec V3, both radios: 200/200, 0 retries, RTT 19.9 ms avg (p95 27.0).
- Heltec V3, LoRa only: 60/60, 11 retries, RTT 128.5 ms avg (p95 150.5).
- Feather V2 (no antennas): 100/100 at 1–2 cm, about 8% at 5–20 cm.
