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
import socket
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
MAX_RETRY_TIMEOUT = 1.5  # s; cap for the adaptive retry timeout and its backoff (SF9 RTT is ~0.4 s)


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
        self.retry_timeout = retry_timeout  # minimum; adapts upward to the measured RTT
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
        # Adaptive retry timeout, like TCP: track a smoothed RTT from first-try ACKs only
        # (a retried packet's ACK is ambiguous) and double the timeout on every retry.
        # So the link adjusts by itself when traffic moves from ESP-NOW (~25 ms) to LoRa (~130 ms+).
        self._srtt = None
        self._retried = False
        self._heartbeat_period = HEARTBEAT_PERIOD
        self.radio = {}  # last "# radio ..." status line from the bridge, parsed
        self._radio_at = 0.0
        self._pending_scene = None
        self._latest_scene = b""
        self._last_scene_tx = 0.0
        self._lora_free_at = 0.0
        self._video_chunks = collections.deque()
        self._last_video_tx = 0.0
        self._video_frame_seq = 0
        self._rx_video_id = None
        self._rx_video_total = 0
        self._rx_video_parts = {}
        self._scene_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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

    def send_scene(self, scene_bytes):
        """Queue the latest SceneCodec (0xAD/0xAE) frame for best-effort transmission over the radio."""
        if scene_bytes:
            with self._lock:
                b = bytes(scene_bytes)
                self._pending_scene = b
                self._latest_scene = b

    def send_video_frame(self, jpeg_bytes):
        """Queue a compact WebP/JPEG frame as 0xEF chunks for ESP-NOW transmission to the receiver."""
        if not jpeg_bytes:
            return
        chunk_size = 96
        total = (len(jpeg_bytes) + chunk_size - 1) // chunk_size
        if total <= 0 or total > 10:
            return
        with self._lock:
            if self._video_chunks:
                return
            self._video_frame_seq = (self._video_frame_seq + 1) & 0xFF
            fid = self._video_frame_seq
            for idx in range(total):
                part = jpeg_bytes[idx * chunk_size : (idx + 1) * chunk_size]
                hdr = bytes([0xEF, fid, idx & 0xFF, total & 0xFF])
                self._video_chunks.append(hdr + part)

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
            "retry_timeout_ms": round(self._timeout * 1000),
            # Our bridge's view of each radio, or None if it hasn't reported in 3 s.
            "radio": dict(self.radio) if self.radio and time.monotonic() - self._radio_at < 3.0 else None,
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
            self._ser = serial.Serial(self.port, BAUD, timeout=0.005)
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

    @staticmethod
    def _lora_airtime(payload_len):
        """SX1262 SF9 / 125 kHz / CR 4/5 Time-on-Air in seconds so loraTxQueue never overflows."""
        import math
        t_sym = (2 ** 9) / 125000.0  # 4.096 ms
        num = max(0, 8 * payload_len - 36 + 28 + 16)
        payload_sym = 8 + max(int(math.ceil(num / 36.0)) * 5, 0)
        return (12.25 + payload_sym) * t_sym + 0.012

    def _write(self, data):
        is_espnow_only = bool(data and data[0] == 0xEF)
        if self.sealer is not None:
            data = self.sealer.seal(data)  # fresh nonce on every transmission, retries included
        if not is_espnow_only:
            now_m = time.monotonic()
            self._lora_free_at = max(now_m, self._lora_free_at) + self._lora_airtime(len(data))
        self.stats["bytes_tx"] += len(data)
        raw_line = p.to_line(data).encode()
        if len(raw_line) <= 240:
            self._ser.write(raw_line)
        else:
            for i in range(0, len(raw_line), 120):
                self._ser.write(raw_line[i : i + 120])
                if i + 120 < len(raw_line):
                    time.sleep(0.004)

    def _read(self):
        # Non-blocking read when bytes are waiting; tiny 1ms sleep when idle so _send_due runs at high rate.
        waiting = self._ser.in_waiting
        if waiting:
            self._rx += self._ser.read(waiting)
        else:
            time.sleep(0.001)
            return
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            self._handle_line(line.decode(errors="replace"))

    def _handle_line(self, line):
        stripped = line.strip()
        if stripped.startswith("# radio "):
            self._handle_radio(stripped)
            return
        if stripped == "# lora only":
            self.radio["mode"] = "LORA_ONLY"
            self._radio_at = time.monotonic()
            return
        if stripped == "# muted":
            self.radio["mode"] = "MUTED"
            self._radio_at = time.monotonic()
            return
        if stripped == "# unmuted":
            self.radio["mode"] = "DUAL"
            self._radio_at = time.monotonic()
            return
        data = p.from_line(line)
        if data is None:
            return  # blank line or bridge diagnostic
        wire = data
        if self.sealer is not None:
            data = self.sealer.open(data)  # None for plain, forged, tampered or wrong-key frames
        if data is not None and len(data) > 4 and data[0] == 0xEF:
            self.stats["bytes_rx"] += len(wire)
            now = time.monotonic()
            self._last_heard = now
            if not self.up:
                self.up = True
                self.on_event("link_up")
            fid = data[1] & 0x7F
            has_piggyback_scene = bool(data[1] & 0x80)
            idx, total = data[2], data[3]
            offset = 4
            if has_piggyback_scene and len(data) > 5:
                slen = data[4]
                offset = 5 + slen
                if slen >= 3 and 5 + slen <= len(data):
                    sbytes = data[5 : 5 + slen]
                    if sbytes[0] in (0xAD, 0xAE):
                        self.on_packet({"type": sbytes[0], "scene_bytes": sbytes, "wire": sbytes.hex().upper()})
            if total == 0:
                return
            payload = data[offset:]
            if fid != self._rx_video_id or total != self._rx_video_total:
                self._rx_video_id = fid
                self._rx_video_total = total
                self._rx_video_parts = {}
            self._rx_video_parts[idx] = payload
            if total > 0 and len(self._rx_video_parts) == total:
                full_jpeg = b"".join(self._rx_video_parts[i] for i in range(total) if i in self._rx_video_parts)
                self._rx_video_parts = {}
                self.on_packet({"type": 0xEF, "frame_id": fid, "jpeg_bytes": full_jpeg})
            return
        if data is not None and len(data) >= 3 and data[0] in (0xAD, 0xAE):
            self.stats["bytes_rx"] += len(wire)
            now = time.monotonic()
            self._last_heard = now
            if not self.up:
                self.up = True
                self.on_event("link_up")
            self.on_packet({"type": data[0], "scene_bytes": data, "wire": wire.hex().upper()})
            return
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

    def _handle_radio(self, line):
        """Bridge status, e.g. '# radio mode=DUAL espnow_rx=12 espnow_rssi=-40 espnow_age=210 ...'."""
        radio = {}
        for item in line[len("# radio "):].split():
            key, _, value = item.partition("=")
            try:
                radio[key] = float(value) if "." in value else int(value)
            except ValueError:
                radio[key] = value
        self.radio = radio
        self._radio_at = time.monotonic()

    def _retry_timeout(self):
        timeout = self.retry_timeout
        if self._srtt is not None:
            timeout = max(timeout, 2.5 * self._srtt)
        return min(timeout, MAX_RETRY_TIMEOUT)

    def _handle_ack(self, seq, now):
        with self._lock:
            if not self._outbox or self._outbox[0][0] != seq:
                return  # stale or duplicate ACK
            self._outbox.popleft()
        self._tx_seq = None
        rtt = now - self._last_tx
        if not self._retried:
            self._srtt = rtt if self._srtt is None else 0.8 * self._srtt + 0.2 * rtt
        rtt_ms = rtt * 1000
        self.rtts_ms.append(rtt_ms)
        self.stats["delivered"] += 1
        self.on_event("delivered", seq=seq, rtt_ms=rtt_ms)

    def _send_due(self):
        now = time.monotonic()
        # If we just transmitted a scene or data packet within the last 0.8s, the peer already
        # heard us (updates _last_heard), so stretch heartbeat interval to 2.2s to save LoRa airtime.
        active_tx = (now - self._last_scene_tx < 0.8) or (now - self._last_tx < 0.8)
        eff_hb_period = self._heartbeat_period * (2.0 if active_tx else 1.0)
        if now - self._last_heartbeat >= eff_hb_period:
            self._last_heartbeat = now
            # Jitter keeps the two sides from locking in step: on LoRa (half duplex) two
            # boards that always transmit at the same instant never hear each other.
            self._heartbeat_period = HEARTBEAT_PERIOD * random.uniform(0.85, 1.15)
            heartbeat = p.encode(p.HEARTBEAT, 0, ts_ms=p.now_ms(), mode=self.mode, buffered=min(self.buffered, 255))
            self._write(heartbeat)

        if not self.up:
            return  # hold everything in the buffer until we hear the peer again
        espnow_live = (
            not self.radio
            or (
                self.radio.get("mode", "DUAL") == "DUAL"
                and (
                    (self.radio.get("espnow_rx", 0) == 0 and self.radio.get("lora_rx", 0) == 0)
                    or 0 <= self.radio.get("espnow_age", -1) < 3500
                )
            )
        )
        lora_ready = now >= self._lora_free_at
        scene_due = (
            (lora_ready and (now - self._last_scene_tx) >= 0.08)
            if not espnow_live
            else ((now - self._last_scene_tx) >= 0.075)
        )
        scene_pkt = None
        video_pkt = None
        with self._lock:
            if not espnow_live:
                self._video_chunks.clear()
            if not self._outbox:
                if self._pending_scene is not None and scene_due:
                    scene_pkt = self._pending_scene
                    self._pending_scene = None
                    self._last_scene_tx = now
                elif espnow_live and self._video_chunks and (now - self._last_video_tx) >= 0.025:
                    raw_v = self._video_chunks.popleft()
                    if self._latest_scene and len(self._latest_scene) <= 32:
                        s_b = self._latest_scene
                        video_pkt = bytes([0xEF, (raw_v[1] & 0x7F) | 0x80, raw_v[2], raw_v[3], len(s_b)]) + s_b + raw_v[4:]
                        self._pending_scene = None
                        self._last_scene_tx = now
                    else:
                        video_pkt = raw_v
                    self._last_video_tx = now
                else:
                    return
            else:
                seq, data = self._outbox[0]
                if (
                    seq == self._tx_seq
                    and (now - self._last_tx) < self._timeout
                    and (now - self._last_tx) >= 0.04
                ):
                    if self._pending_scene is not None and scene_due:
                        scene_pkt = self._pending_scene
                        self._pending_scene = None
                        self._last_scene_tx = now
                    elif espnow_live and self._video_chunks and (now - self._last_video_tx) >= 0.025:
                        raw_v = self._video_chunks.popleft()
                        if self._latest_scene and len(self._latest_scene) <= 32:
                            s_b = self._latest_scene
                            video_pkt = bytes([0xEF, (raw_v[1] & 0x7F) | 0x80, raw_v[2], raw_v[3], len(s_b)]) + s_b + raw_v[4:]
                            self._pending_scene = None
                            self._last_scene_tx = now
                        else:
                            video_pkt = raw_v
                        self._last_video_tx = now
        if scene_pkt is not None:
            if not espnow_live and len(scene_pkt) >= 3 and scene_pkt[0] == 0xAE:
                b = bytearray(scene_pkt)
                b[2] |= 0x80
                scene_pkt = bytes(b)
            elif espnow_live and not lora_ready and self.sealer is None and len(scene_pkt) <= 48:
                # LoRa SX1262 is mid-airtime: send intermediate update on ESP-NOW (0xEF) so loraTxQueue stays at 0 lag
                scene_pkt = bytes([0xEF, 0x80, 0, 0, len(scene_pkt)]) + scene_pkt
            self._write(scene_pkt)
            return
        if video_pkt is not None:
            self._write(video_pkt)
            return
        if seq == self._tx_seq:
            if now - self._last_tx < self._timeout:
                return
            self.stats["retries"] += 1
            self._retried = True
            self._timeout = min(self._timeout * 2, MAX_RETRY_TIMEOUT)  # back off
        else:
            self.stats["sent"] += 1
            self._retried = False
            self._timeout = self._retry_timeout() + random.uniform(0, 0.1)  # jitter avoids lock-step collisions
        self._write(data)
        self._tx_seq = seq
        self._last_tx = now

    def _check_link(self):
        if self.up and time.monotonic() - self._last_heard > LINK_TIMEOUT:
            self.up = False
            self.stats["link_losses"] += 1
            self.on_event("link_down")
