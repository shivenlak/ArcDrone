import argparse
import time
from typing import Any

from protocol import packets as p
from protocol.link import Link


def handle_event(*args: Any, **kwargs: Any) -> None:
    """Print link status changes (link_up / link_down) and delivery RTT."""
    if len(args) == 1 and isinstance(args[0], dict):
        evt = args[0]
        etype = evt.get("event") or evt.get("type") or evt.get("kind") or "event"
        seq = evt.get("seq")
        rtt = evt.get("rtt_ms")
        print(f"[EVENT] {etype} | seq={seq} | rtt_ms={rtt} | raw={evt}")
    elif len(args) >= 1:
        etype = args[0]
        rest = args[1:]
        info = kwargs if kwargs else (rest[0] if len(rest) == 1 else rest)
        print(f"[EVENT] {etype} | info={info}")
    else:
        print(f"[EVENT] {kwargs}")


def handle_packet(*args: Any, **kwargs: Any) -> None:
    """Print incoming COMMAND packets from the ground station."""
    pkt = args[0] if args else kwargs
    if isinstance(pkt, dict):
        pkt_type = pkt.get("type") or pkt.get("pkt_type")
        if pkt_type == getattr(p, "COMMAND", "COMMAND") or pkt_type == "COMMAND":
            print(f"[COMMAND RECEIVED] {pkt}")
        else:
            print(f"[PACKET RECEIVED] {pkt}")
    else:
        print(f"[PACKET RECEIVED] args={args} kwargs={kwargs}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send hardcoded DETECTION packets to test the telemetry link."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port (e.g., /dev/cu.usbserial-* or fake_bridge pty path)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Seconds between DETECTION packets (default: 2.0)",
    )
    args = parser.parse_args()

    print(f"[FAKE_SENDER] Starting Link on port {args.port} (interval={args.interval}s)...")
    link = Link(args.port, on_packet=handle_packet, on_event=handle_event)
    link.start()

    try:
        while True:
            ts = p.now_ms()
            seq = link.send(
                p.DETECTION,
                ts_ms=ts,
                cls=3,
                conf=0.91,
                count=2,
                lat=53.52,
                lon=-113.52,
            )
            print(
                f"[TX] Sent DETECTION | seq={seq} ts_ms={ts} cls=3 conf=0.91 count=2 "
                f"lat=53.52 lon=-113.52"
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[FAKE_SENDER] Shutting down...")
    finally:
        if hasattr(link, "stop"):
            link.stop()
        elif hasattr(link, "close"):
            link.close()


if __name__ == "__main__":
    main()