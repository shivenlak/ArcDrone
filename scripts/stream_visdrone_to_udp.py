"""Continuous live UDP feeder streaming VisDrone scene packets to scene_ws_relay (:9999).

Reads the authentic VisDrone YOLO-encoded scene packets and streams them in a real-time
12 Hz loop to UDP port 9999 so that the browser reconstructor UI (http://127.0.0.1:8080/reconstructor.html)
receives live tactical radar telemetry.
"""

import json
import os
import socket
import sys
import time

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKETS_FILE = os.path.join(_REPO_ROOT, "data", "visdrone_scene_packets.json")
UDP_HOST = "127.0.0.1"
UDP_PORT = 9999
TARGET_HZ = 12.0
INTERVAL = 1.0 / TARGET_HZ


def main():
    if not os.path.isfile(PACKETS_FILE):
        print(f"[FEEDER ERROR] Missing packets file: {PACKETS_FILE}")
        sys.exit(1)

    with open(PACKETS_FILE, "r") as f:
        hex_packets = json.load(f)

    packets = [bytes.fromhex(h) for h in hex_packets]
    print(f"[FEEDER] Loaded {len(packets)} VisDrone scene packets.")
    print(f"[FEEDER] Streaming to {UDP_HOST}:{UDP_PORT} at {TARGET_HZ:.1f} Hz (looping continuously)...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    idx = 0
    total_sent = 0

    try:
        while True:
            t0 = time.time()
            pkt = packets[idx]
            sock.sendto(pkt, (UDP_HOST, UDP_PORT))
            total_sent += 1
            idx = (idx + 1) % len(packets)

            elapsed = time.time() - t0
            sleep_time = max(0.001, INTERVAL - elapsed)
            time.sleep(sleep_time)

            if total_sent % 60 == 0:
                print(f"[FEEDER] Streamed {total_sent} packets (loop cycle active, {len(pkt)} bytes/pkt)")
    except KeyboardInterrupt:
        print("\n[FEEDER] Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
