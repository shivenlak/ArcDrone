"""Interactive Ground Station command sender to verify Step 3 HUD transitions."""
import argparse
from protocol import packets as p
from protocol.link import Link


def _const(names, fallback):
    for n in names:
        if hasattr(p, n):
            return getattr(p, n)
    return fallback


CMD_ACK = _const(["CMD_OPERATOR_ACK", "OPERATOR_ACK", "CMD_ACK"], 0)
CMD_HOLD = _const(["CMD_HOLD", "HOLD"], 1)
CMD_RESUME = _const(["CMD_RESUME", "RESUME"], 2)
CMD_RTB = _const(["CMD_RTB", "RTB"], 3)


def main():
    parser = argparse.ArgumentParser(description="Step 3 Command Tester")
    parser.add_argument("--port", required=True, help="Port B from fake_bridge")
    args = parser.parse_args()

    def on_rx(*a, **kw):
        print(f"\n[GROUND RX DETECTION] {a} {kw}")

    link = Link(args.port, on_packet=on_rx)
    link.start()
    print(f"[GROUND] Connected on {args.port}. Buffered drone alerts will now flush!")
    print("Commands: [h] HOLD | [r] RESUME | [a] ACK Alert #1 | [b] RTB | [q] Quit")

    try:
        while True:
            choice = input("Send command (h/r/a/b/q): ").strip().lower()
            if choice == "h":
                link.send(p.COMMAND, cmd=CMD_HOLD, ref_seq=0)
                print("-> Sent HOLD")
            elif choice == "r":
                link.send(p.COMMAND, cmd=CMD_RESUME, ref_seq=0)
                print("-> Sent RESUME")
            elif choice == "a":
                link.send(p.COMMAND, cmd=CMD_ACK, ref_seq=1)
                print("-> Sent OPERATOR_ACK for Alert #1")
            elif choice == "b":
                link.send(p.COMMAND, cmd=CMD_RTB, ref_seq=0)
                print("-> Sent RTB")
            elif choice == "q":
                break
    finally:
        if hasattr(link, "stop"):
            link.stop()


if __name__ == "__main__":
    main()