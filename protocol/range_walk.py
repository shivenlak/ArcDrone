"""Range walk test: carry the drone board away from the ground board and log the link.

Ground side (stays put): run the normal ground server; it ACKs everything.
    python -m ground_station.server --port <groundPort> --http-port 8001
Walker (drone board + laptop):
    python -m protocol.range_walk --port <dronePort>

Sends one DETECTION per second and prints one line per second: which radio our board
still hears, its RSSI/SNR, delivered/sent, retries and RTT. Type a note (e.g. "7") and
press Enter to mark where you are. Everything also goes to a CSV (range_walk_<time>.csv).
"""
import argparse
import csv
import sys
import threading
import time

from protocol import packets as p
from protocol.link import Link

FRESH_MS = 3000  # a radio counts as "heard" if it received something in the last 3 s


def heard(radio, name):
    age = radio.get(name + "_age")
    return isinstance(age, int) and 0 <= age < FRESH_MS


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True, help="serial port of the drone board")
    ap.add_argument("--interval", type=float, default=1.0, help="seconds between alerts")
    args = ap.parse_args()

    link = Link(args.port).start()
    note = {"text": ""}

    def read_notes():
        for line in sys.stdin:
            note["text"] = line.strip()
            print("   >>> marked:", note["text"] or "(cleared)")

    threading.Thread(target=read_notes, daemon=True).start()

    path = time.strftime("range_walk_%H%M%S.csv")
    cols = ["t_s", "note", "link", "espnow", "espnow_rssi", "lora", "lora_rssi", "lora_snr",
            "sent", "delivered", "buffered", "retries", "rtt_ms_avg"]
    print("logging to", path, "- type a floor/place and press Enter to mark it")
    print("  t   note   link  ESP-NOW        LoRa                 delivered/sent  retries  RTT")
    start = time.time()
    with open(path, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(cols)
        while True:
            try:
                link.send(p.DETECTION, ts_ms=p.now_ms(), cls=3, conf=0.9, count=1, lat=53.52, lon=-113.52)
                time.sleep(args.interval)
            except KeyboardInterrupt:
                break
            s = link.summary()
            r = s["radio"] or {}
            en, lo = heard(r, "espnow"), heard(r, "lora")
            row = [round(time.time() - start), note["text"], "UP" if s["up"] else "DOWN",
                   en, r.get("espnow_rssi") if en else "", lo, r.get("lora_rssi") if lo else "",
                   r.get("lora_snr") if lo else "", s["sent"], s["delivered"], s["buffered"],
                   s["retries"], s["rtt_ms_avg"]]
            out.writerow(row)
            f.flush()
            esp = ("%4s dBm" % r.get("espnow_rssi")) if en else "  --    "
            lor = ("%4s dBm SNR %5s" % (r.get("lora_rssi"), r.get("lora_snr"))) if lo else "  --              "
            print("%4d  %-5s %-4s  %s       %s  %4d/%-4d       %4d    %s ms"
                  % (row[0], note["text"][:5], row[2], esp, lor, s["delivered"], s["sent"],
                     s["retries"], s["rtt_ms_avg"]))
    link.stop()
    print("saved", path)


if __name__ == "__main__":
    main()
