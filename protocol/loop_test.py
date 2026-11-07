"""Phase 1/3 loop test: hardcoded detections drone -> ground, ACKed, with RTT.

Run one side per bridge (real ESP32s, or the two fake_bridge ports):

    python -m protocol.loop_test --role ground --port /dev/cu.usbserial-0002
    python -m protocol.loop_test --role drone  --port /dev/cu.usbserial-0001 --count 100

Phase 1 passes when the drone prints delivered=100 and the ground received 100.
For Phase 3, press the BOOT button (mute) or cut the fake bridge mid-run: the
drone should buffer, and every alert should still arrive exactly once.
"""

import argparse
import time

from protocol import packets as p
from protocol.link import Link


def log(text):
    print(time.strftime("%H:%M:%S"), text, flush=True)


def on_event(name, **info):
    if name == "delivered":
        log("delivered seq={} rtt={:.1f} ms".format(info["seq"], info["rtt_ms"]))
    else:
        log(name.upper())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", choices=["drone", "ground"], required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--count", type=int, default=100, help="drone: detections to send")
    parser.add_argument("--interval", type=float, default=0.5, help="drone: seconds between detections")
    args = parser.parse_args()

    received = []

    def on_packet(packet):
        if packet["type"] == p.DETECTION:
            received.append(packet["seq"])
            log("detection seq={} {} conf={:.2f} (total {})".format(
                packet["seq"], p.CLASS_NAMES[packet["cls"]], packet["conf"], len(received)))

    link = Link(args.port, on_packet=on_packet, on_event=on_event).start()
    log("waiting for link on {} ...".format(args.port))

    try:
        if args.role == "drone":
            for i in range(args.count):
                link.send(p.DETECTION, ts_ms=p.now_ms(), cls=3, conf=0.9, count=1, lat=53.5232, lon=-113.5263)
                time.sleep(args.interval)
            while link.buffered:  # wait for the last ones to be ACKed
                time.sleep(0.2)
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass

    stats = link.summary()
    if args.role == "ground":
        stats["received"] = len(received)
        stats["unique"] = len(set(received))
    log("summary: {}".format(stats))
    link.stop()


if __name__ == "__main__":
    main()
