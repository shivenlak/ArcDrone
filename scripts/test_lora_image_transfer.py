"""Automated verification suite for On-Demand LoRa Target Crop Transfer (Scavenger Protocol).

Tests:
1. Packet wire sizes (131 B plain / 139 B AES-128-CCM <= 250 B ESP32/LoRa hardware limit).
2. TargetCropCompressor byte ceiling (<= 960 B, <= 8 chunks) and pure-Python CI fallback.
3. The 4 Anti-Haywire Scavenger Gates (Link Up, Zero-Buffer Priority, SNR Cliff, Pacing).
4. End-to-end encrypted transfer over simulated LoRa with packet loss, mid-transfer alert
   preemption, and <= 3.0 kB/s bandwidth budget verification.
"""

from __future__ import annotations

import os
import sys
import time
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from protocol import crypto
from protocol import packets as p
from protocol.link import Link
from scripts.bandwidth_monitor import BandwidthMonitor, BUDGET_BYTES_PER_SEC
from scripts.lora_image_transfer import (
    CHUNK_PAYLOAD_SIZE,
    CMD_REQUEST_IMAGE,
    DroneImageResponder,
    GroundImageReceiver,
    MAX_THUMBNAIL_BYTES,
    SNAPSHOT_CHUNK,
    TargetCropCompressor,
    register_image_protocol,
    slice_image_into_chunks,
)
from scripts.sim_link_bridge import SimLinkBridge


class DummyLink:
    """Lightweight mock Link for deterministic unit testing of the 4 Scavenger Gates."""

    def __init__(self) -> None:
        self.up = True
        self._buffered = 0
        self.radio = {}
        self.sent_packets = []
        self._seq = 100

    @property
    def buffered(self) -> int:
        return self._buffered

    def send(self, ptype: int, **fields) -> int:
        self._seq += 1
        self.sent_packets.append((self._seq, ptype, fields))
        return self._seq


class TestLoRaImageTransfer(unittest.TestCase):
    def setUp(self) -> None:
        register_image_protocol()

    def test_packet_wire_sizes_and_crypto_roundtrip(self) -> None:
        """SNAPSHOT_CHUNK must be 131 B plain and 139 B encrypted (well below 250 B hardware limit)."""
        req = p.encode(p.COMMAND, 7, cmd=CMD_REQUEST_IMAGE, ref_seq=42)
        self.assertEqual(len(req), 9)
        dec_req = p.decode(req)
        self.assertEqual(dec_req, {"type": p.COMMAND, "seq": 7, "cmd": CMD_REQUEST_IMAGE, "ref_seq": 42})

        payload = b"A" * 55 + b"\x00" * (CHUNK_PAYLOAD_SIZE - 55)
        raw_chunk = p.encode(
            SNAPSHOT_CHUNK,
            19,
            ref_seq=42,
            chunk_idx=2,
            total_chunks=6,
            valid_len=55,
            payload=payload,
        )
        # 4 B header + 125 B body + 2 B CRC16 = 131 bytes
        self.assertEqual(len(raw_chunk), 131)
        self.assertLessEqual(len(raw_chunk), 250, "Must fit ESP-NOW / LoRa 250-byte hardware MAX_LEN")

        sealer = crypto.Sealer(os.urandom(16))
        sealed = sealer.seal(raw_chunk)
        # 131 B + 8 B AES-128-CCM overhead = 139 bytes
        self.assertEqual(len(sealed), 139)
        self.assertEqual(sealed[0], crypto.MAGIC_SECURE)
        self.assertLessEqual(len(sealed), 250)

        opened = sealer.open(sealed)
        self.assertEqual(opened, raw_chunk)
        decoded = p.decode(opened)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded["type"], SNAPSHOT_CHUNK)
        self.assertEqual(decoded["ref_seq"], 42)
        self.assertEqual(decoded["chunk_idx"], 2)
        self.assertEqual(decoded["total_chunks"], 6)
        self.assertEqual(decoded["valid_len"], 55)
        self.assertEqual(decoded["payload"][:55], b"A" * 55)

    def test_compressor_respects_byte_budget(self) -> None:
        """TargetCropCompressor must produce thumbnails strictly <= 960 bytes (<= 8 chunks)."""
        compressor = TargetCropCompressor(thumb_size=(64, 64), max_bytes=MAX_THUMBNAIL_BYTES)
        raw, mime = compressor.compress_roi(cls_name="truck", conf=0.92, seq=14)
        self.assertGreater(len(raw), 50)
        self.assertLessEqual(len(raw), MAX_THUMBNAIL_BYTES)
        self.assertIn(mime, ("image/jpeg", "application/x-pgm-zlib"))

        chunks = slice_image_into_chunks(ref_seq=14, image_bytes=raw)
        self.assertLessEqual(len(chunks), 8)
        reassembled = b"".join(c["payload"][: c["valid_len"]] for c in chunks)
        self.assertEqual(reassembled, raw)

        # Also verify pure-Python fallback explicitly
        fallback_bytes = compressor._compress_fallback_pgm(cls_name="car", conf=0.85, seq=99)
        self.assertLessEqual(len(fallback_bytes), MAX_THUMBNAIL_BYTES)

    def test_scavenger_four_safety_gates(self) -> None:
        """DroneImageResponder must obey Link-Up, Buffered==0, SNR>=-6dB, and Pacing gates."""
        dummy = DummyLink()
        responder = DroneImageResponder(
            dummy,
            min_chunk_interval_s=0.20,
            min_lora_snr_db=-6.0,
        )
        responder.cache_detection_crop(seq=10, precompressed=b"X" * 300)  # 3 chunks
        responder.handle_packet({"type": p.COMMAND, "cmd": CMD_REQUEST_IMAGE, "ref_seq": 10})
        self.assertEqual(responder.pending_chunks, 3)

        # Gate 1: Link DOWN blocks transmission
        dummy.up = False
        self.assertIsNone(responder.step())
        self.assertEqual(responder.stats["paused_link_down"], 1)
        dummy.up = True

        # Gate 2: Non-empty alert buffer (buffered > 0) forces scavenger to yield immediately
        dummy._buffered = 5  # Simulating stairwell recovery flush
        self.assertIsNone(responder.step())
        self.assertEqual(responder.stats["yielded_to_alerts"], 1)
        dummy._buffered = 0

        # Gate 3: Deep negative LoRa SNR (< -6.0 dB) with ESP-NOW dead pauses chunking
        dummy.radio = {"espnow_age": 9999, "lora_age": 100, "lora_rssi": -115, "lora_snr": -8.5}
        self.assertIsNone(responder.step())
        self.assertEqual(responder.stats["paused_low_snr"], 1)

        # Recover SNR to 6th-floor workable level (-2.0 dB): chunk 0 transmits!
        dummy.radio = {"espnow_age": 9999, "lora_age": 100, "lora_rssi": -105, "lora_snr": -2.0}
        seq0 = responder.step()
        self.assertIsNotNone(seq0)
        self.assertEqual(responder.pending_chunks, 2)

        # Gate 4: Immediate second step is blocked by min_chunk_interval_s pacing
        self.assertIsNone(responder.step())

    def test_end_to_end_encrypted_transfer_with_alert_preemption(self) -> None:
        """Full closed-loop encrypted transfer with packet loss and mid-transfer alert priority."""
        sealer = crypto.Sealer(os.urandom(16))
        bridge = SimLinkBridge(
            name_a="sim://drone_img_test",
            name_b="sim://ground_img_test",
            packet_loss_rate=0.10,
            latency=0.005,
        )
        bw_monitor = BandwidthMonitor(budget_limit=BUDGET_BYTES_PER_SEC, window_size_sec=1.0)
        bw_monitor.attach_to_bridge(bridge)

        received_alerts = []
        completed_crops = []

        holder = {}

        def on_drone_pkt(pkt):
            if "resp" in holder:
                holder["resp"].handle_packet(pkt)

        def on_ground_pkt(pkt):
            if pkt.get("type") == p.DETECTION:
                received_alerts.append(pkt)
            if "rec" in holder:
                holder["rec"].handle_packet(pkt)

        drone_link = Link("sim://drone_img_test", on_packet=on_drone_pkt, retry_timeout=0.06, boot_wait=0.01, sealer=sealer)
        ground_link = Link("sim://ground_img_test", on_packet=on_ground_pkt, retry_timeout=0.06, boot_wait=0.01, sealer=sealer)

        responder = DroneImageResponder(drone_link, min_chunk_interval_s=0.12)
        receiver = GroundImageReceiver(ground_link, on_complete=lambda rep: completed_crops.append(rep))
        holder["resp"] = responder
        holder["rec"] = receiver

        drone_link.start()
        ground_link.start()
        responder.start()

        try:
            deadline = time.monotonic() + 2.0
            while not (drone_link.up and ground_link.up) and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue(drone_link.up and ground_link.up)

            # 1. Send initial detection + cache known crop bytes
            expected_crop = b"\xff\xd8\xff\xe0" + os.urandom(520) + b"\xff\xd9"
            seq1 = drone_link.send(p.DETECTION, ts_ms=1000, cls=3, conf=0.95, count=1, lat=53.52, lon=-113.52)
            responder.cache_detection_crop(seq=seq1, precompressed=expected_crop)

            # 2. Request target crop from ground station
            receiver.request_image(seq1)

            # 3. Inject 2 live detections while chunks are trickling
            time.sleep(0.15)
            seq2 = drone_link.send(p.DETECTION, ts_ms=1200, cls=0, conf=0.88, count=2, lat=53.521, lon=-113.521)
            time.sleep(0.15)
            seq3 = drone_link.send(p.DETECTION, ts_ms=1400, cls=5, conf=0.91, count=1, lat=53.522, lon=-113.522)

            deadline = time.monotonic() + 5.0
            while (len(completed_crops) < 1 or len(received_alerts) < 3) and time.monotonic() < deadline:
                time.sleep(0.03)

            self.assertEqual(len(completed_crops), 1, "Target crop must complete reassembly")
            self.assertEqual(completed_crops[0]["image_bytes"], expected_crop)
            self.assertEqual(completed_crops[0]["ref_seq"], seq1)
            self.assertTrue(completed_crops[0]["data_uri"].startswith("data:image/jpeg;base64,"))

            # Verify all 3 live alerts arrived with zero drops
            self.assertEqual(len(received_alerts), 3)
            self.assertEqual({a["seq"] for a in received_alerts}, {seq1, seq2, seq3})
            self.assertEqual(drone_link.stats["dropped"], 0)

            # Verify <= 3.0 kB/s budget compliance
            summary = bw_monitor.summary()
            self.assertTrue(summary["within_budget"], f"Exceeded 3 kB/s budget: {summary}")
        finally:
            responder.stop()
            drone_link.stop()
            ground_link.stop()
            bridge.close()


if __name__ == "__main__":
    unittest.main()
