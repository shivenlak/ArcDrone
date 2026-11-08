"""Reliable link over the ESP32 bridge (rules: CLAUDE.md section 4).

One background thread owns the serial port and does everything: reads packets,
sends ACKs and heartbeats, retransmits, detects link loss, reconnects the port.
Other threads only call send() and read attributes.

Usage (same on drone and ground):

    link = Link("/dev/cu.usbserial-0001", on_packet=handle_packet, on_event=handle_event).start()
    seq = link.send(packets.DETECTION, ts_ms=..., cls=3, conf=0.9, count=1, lat=..., lon=...)

Callbacks run on the link thread, so keep them fast (append to a queue, set a flag).
    on_packet(packet)      new DETECTION/COMMAND (deduped), and every HEARTBEAT.
                           packet["wire"] is the frame's exact bytes as hex.
Set ARCDRONE_KEY on both sides to encrypt and authenticate every frame (protocol/crypto.py).
    on_event(name, **info) "link_up", "link_down", "delivered" (seq=, rtt_ms=)
"""

import collections
import random
import threading
import time

import serial

from protocol import crypto
from protocol import packets as p

BAUD = 115200
HEARTBEAT_PERIOD = 1.0  # s
LINK_TIMEOUT = 3.0  # s of silence before the link is declared lost
BUFFER_SIZE = 64  # unacknowledged messages kept; oldest dropped when full
DEDUP_WINDOW = 64  # recent seqs remembered to drop duplicates
RECONNECT_PERIOD = 1.0  # s between attempts to reopen a lost serial port


def _ignore(*args, **kwargs):
    pass


class Link:
    def __init__(self, port, on_packet=_ignore, on_event=_ignore, retry_timeout=0.3, boot_wait=2.0,
                 sealer="env"):
        self.port = port
        # Encryption: on when ARCDRONE_KEY is set (protocol/crypto.py), or pass a crypto.Sealer.
        self.sealer = crypto.from_env() if sealer == "env" else sealer
        self.on_packet = on_packet
        self.on_event = on_event
        self.retry_timeout = retry_timeout  # use ~1.0 on LoRa
        self.boot_wait = boot_wait  # opening the port resets the ESP32
        self.mode = p.MODE_PATROL  # reported in our heartbeats; drone_node sets this

        self.up = False
        self.stats = collections.Counter()
        self.rtts_ms = collections.deque(maxlen=100)

        # Store-and-forward buffer of (seq, bytes). The head is the message in flight;
        # it stays there until ACKed. While the link is down, everything just waits.
        self._outbox = collections.deque(maxlen=BUFFER_SIZE)
        self._lock = threading.Lock()
        self._seq = random.randrange(0x10000)  # random start so a reboot isn't mistaken for duplicates
        self._seen = collections.deque(maxlen=DEDUP_WINDOW)

        self._ser = None
        self._rx = b""
        self._running = False
        self._last_heard = 0.0
        self._last_heartbeat = 0.0
        self._tx_seq = None  # seq of the head message when we last transmitted it
        self._last_tx = 0.0
        self._timeout = retry_timeout

    def start(self):
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def stop(self):
        self._running = False

    def send(self, ptype, **fields):
        """Queue a reliable DETECTION or COMMAND. Returns its seq, which the 'delivered' event names."""
        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFF
            seq = self._seq
            if len(self._outbox) == BUFFER_SIZE:
                self.stats["dropped"] += 1
            self._outbox.append((seq, p.encode(ptype, seq, **fields)))
        return seq

    @property
    def buffered(self):
        return len(self._outbox)

    def summary(self):
        """Stats for the dashboard."""
        rtts = sorted(self.rtts_ms)
        return {
            "up": self.up,
            "buffered": self.buffered,
            "sent": self.stats["sent"],
            "delivered": self.stats["delivered"],
            "retries": self.stats["retries"],
            "duplicates": self.stats["duplicates"],
            "bad_packets": self.stats["bad_packets"],
            "dropped": self.stats["dropped"],
            "rtt_ms_avg": round(sum(rtts) / len(rtts), 1) if rtts else None,
            "rtt_ms_p95": round(rtts[int(0.95 * (len(rtts) - 1))], 1) if rtts else None,
            "bytes_tx": self.stats["bytes_tx"],  # packet bytes sent / valid bytes received over the radio
            "bytes_rx": self.stats["bytes_rx"],  # (binary size; the serial hop is hex, so twice that)
        }

    # --- link thread ---

    def _run(self):
        while self._running:
            if self._ser is None:
                self._open()
            if self._ser is not None:
                try:
                    self._read()
                    self._send_due()
                except (serial.SerialException, OSError):
                    self._close()
            self._check_link()
        self._close()

    def _open(self):
        try:
            self._ser = serial.Serial(self.port, BAUD, timeout=0.05)
            # Release DTR/RTS (the ESP32 "run" state). Opening asserts both, and on boards
            # that wire DTR to GPIO0 (Heltec V3) that holds the PRG/mute button down.
            # Releasing them resets the ESP32 once, which boot_wait covers.
            try:
                self._ser.dtr = False
                self._ser.rts = False
            except (serial.SerialException, OSError):
                pass  # virtual ports (fake_bridge) have no modem lines
            time.sleep(self.boot_wait)
            self._ser.reset_input_buffer()
            self._rx = b""
        except (serial.SerialException, OSError):
            self._ser = None
            time.sleep(RECONNECT_PERIOD)

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except (serial.SerialException, OSError):
                pass
        self._ser = None

    def _write(self, data):
        if self.sealer is not None:
            data = self.sealer.seal(data)  # fresh nonce on every transmission, retries included
        self.stats["bytes_tx"] += len(data)
        self._ser.write(p.to_line(data).encode())

    def _read(self):
        # Accumulate bytes and split on newlines ourselves: a read can stop mid-line.
        self._rx += self._ser.read(self._ser.in_waiting or 1)
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            self._handle_line(line.decode(errors="replace"))

    def _handle_line(self, line):
        data = p.from_line(line)
        if data is None:
            return  # blank line or bridge diagnostic
        wire = data
        if self.sealer is not None:
            data = self.sealer.open(data)  # None for plain, forged, tampered or wrong-key frames
        packet = p.decode(data) if data is not None else None
        if packet is None:
            self.stats["bad_packets"] += 1
            return
        self.stats["bytes_rx"] += len(wire)
        packet["wire"] = wire.hex().upper()  # exact bytes as received, for the dashboard

        now = time.monotonic()
        self._last_heard = now
        if not self.up:
            self.up = True
            self.on_event("link_up")

        ptype, seq = packet["type"], packet["seq"]
        if ptype == p.ACK:
            self._handle_ack(seq, now)
        elif ptype == p.HEARTBEAT:
            self.on_packet(packet)
        else:
            # Always ACK, even duplicates: the first ACK may have been the one that got lost.
            self._write(p.encode(p.ACK, seq))
            if seq in self._seen:
                self.stats["duplicates"] += 1
                return
            self._seen.append(seq)
            self.on_packet(packet)

    def _handle_ack(self, seq, now):
        with self._lock:
            if not self._outbox or self._outbox[0][0] != seq:
                return  # stale or duplicate ACK
            self._outbox.popleft()
        self._tx_seq = None
        rtt_ms = (now - self._last_tx) * 1000
        self.rtts_ms.append(rtt_ms)
        self.stats["delivered"] += 1
        self.on_event("delivered", seq=seq, rtt_ms=rtt_ms)

    def _send_due(self):
        now = time.monotonic()
        if now - self._last_heartbeat >= HEARTBEAT_PERIOD:
            self._last_heartbeat = now
            heartbeat = p.encode(p.HEARTBEAT, 0, ts_ms=p.now_ms(), mode=self.mode, buffered=min(self.buffered, 255))
            self._write(heartbeat)

        if not self.up:
            return  # hold everything in the buffer until we hear the peer again
        with self._lock:
            if not self._outbox:
                return
            seq, data = self._outbox[0]
        if seq == self._tx_seq:
            if now - self._last_tx < self._timeout:
                return
            self.stats["retries"] += 1
        else:
            self.stats["sent"] += 1
        self._write(data)
        self._tx_seq = seq
        self._last_tx = now
        self._timeout = self.retry_timeout + random.uniform(0, 0.1)  # jitter avoids lock-step collisions

    def _check_link(self):
        if self.up and time.monotonic() - self._last_heard > LINK_TIMEOUT:
            self.up = False
            self.stats["link_losses"] += 1
            self.on_event("link_down")
