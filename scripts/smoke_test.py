"""Phase 0 smoke test: run on BOTH bridges at once, each with a different --name.

    python scripts/smoke_test.py --port /dev/cu.usbserial-0001 --name A
    python scripts/smoke_test.py --port /dev/cu.usbserial-0002 --name B

Each side sends "HELLO <name> <n>" twice a second and prints what it hears.
Ctrl+C prints a summary. Pass when both sides received ~all of the other's messages.
"""

import argparse
import time

import serial

parser = argparse.ArgumentParser()
parser.add_argument("--port", required=True)
parser.add_argument("--name", required=True)
parser.add_argument("--seconds", type=float, default=60)
args = parser.parse_args()

ser = serial.Serial(args.port, 115200, timeout=0.05)
time.sleep(2)  # opening the port resets the ESP32; wait for it to boot
ser.reset_input_buffer()

sent = received = garbled = 0
next_send = time.time()
end = time.time() + args.seconds
try:
    while time.time() < end:
        if time.time() >= next_send:
            msg = "HELLO {} {}".format(args.name, sent).encode()
            ser.write(msg.hex().upper().encode() + b"\n")
            sent += 1
            next_send += 0.5

        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("#"):
            print(line)
            continue
        try:
            text = bytes.fromhex(line).decode()
            received += 1
            print("rx:", text)
        except ValueError:
            garbled += 1
            print("garbled:", line)
except KeyboardInterrupt:
    pass

print("\nsent={} received={} garbled={}".format(sent, received, garbled))
