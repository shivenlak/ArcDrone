"""DEV ONLY: stand-in for the two ESP32 bridges, so everything above the radio
can be built and tested without hardware. Never use it in the demo: the brief
requires a real physical radio link.

Creates two virtual serial ports joined by a fake "radio" that forwards hex
lines between them, with optional packet loss and a cut switch.

    python -m protocol.fake_bridge --loss 0.2
    # prints two port paths: give one to the drone side, one to the ground side
    # press Enter to cut / restore the link
"""

import argparse
import os
import random
import select
import sys
import threading
import tty


class FakeBridge:
    def __init__(self, loss=0.0):
        self.loss = loss
        self.up = True
        self.ports = []
        self._masters = []
        self._slaves = []  # kept open so the ports stay valid when no program has them open
        self._buffers = [b"", b""]
        for _ in range(2):
            master, slave = os.openpty()
            tty.setraw(slave)
            os.set_blocking(master, False)
            self._masters.append(master)
            self._slaves.append(slave)
            self.ports.append(os.ttyname(slave))

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def _run(self):
        while True:
            ready, _, _ = select.select(self._masters, [], [])
            for i, fd in enumerate(self._masters):
                if fd in ready:
                    self._forward(i, fd)

    def _forward(self, i, fd):
        try:
            self._buffers[i] += os.read(fd, 4096)
        except BlockingIOError:
            return
        while b"\n" in self._buffers[i]:
            line, self._buffers[i] = self._buffers[i].split(b"\n", 1)
            if not self.up or random.random() < self.loss:
                continue
            try:
                os.write(self._masters[1 - i], line + b"\n")
            except BlockingIOError:
                pass  # other side isn't reading; drop it like a real radio would


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--loss", type=float, default=0.0, help="fraction of packets dropped, 0-1")
    args = parser.parse_args()

    bridge = FakeBridge(args.loss).start()
    print("drone side  --port", bridge.ports[0])
    print("ground side --port", bridge.ports[1])
    print("loss={:.0%}. Press Enter to cut/restore the link, Ctrl+C to quit.".format(args.loss))
    try:
        for _ in sys.stdin:
            bridge.up = not bridge.up
            print("LINK UP" if bridge.up else "LINK CUT")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
