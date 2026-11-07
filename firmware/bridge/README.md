# ESP32 bridge firmware

A plain serial↔ESP-NOW relay. The same binary runs on both boards.

- **Serial → radio:** one packet per line as hex (`48454C4C4F\n`). The bytes are broadcast over ESP-NOW on channel 1.
- **Radio → serial:** each received packet is printed as one uppercase hex line.
- Lines starting with `#` are diagnostics (`# bridge ready mac=…`, `# err bad hex`).

The bridge knows nothing about the protocol. That lives in `protocol/` (Python).

## Flash (each board)

```bash
source .venv/bin/activate            # platformio is in requirements.txt
cd firmware/bridge
pio device list                      # find the port, e.g. /dev/cu.usbserial-0001
pio run -t upload --upload-port /dev/cu.usbserial-0001
```

If upload hangs at `Connecting...`, hold the **BOOT** button until it starts. If no port shows up, the cable is probably charge-only. Also check the CP210x/CH340 driver.

## Phase 0 test

1. Manual: open `pio device monitor -p <port>` on both boards. Type `48454C4C4F` and press Enter on one. The other prints `48454C4C4F`.
2. Measured: run `scripts/smoke_test.py` on both ports at once (see its docstring). **Done when** both sides receive ~all messages for 60 s, 2 m apart, with `garbled=0`.
