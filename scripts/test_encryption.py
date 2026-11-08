"""Comprehensive Verification Suite for ArcDrone Encryption.

Tests both:
1. Canonical AES-128-CCM AEAD (protocol/crypto.py, ARCDRONE_KEY, +8 bytes auth tag):
   - Wire format and size verification (DETECTION: 29B, COMMAND: 17B, HEARTBEAT: 20B, ACK: 14B)
   - Random fresh 48-bit nonce validation
   - Bit-flip tamper detection across all frame bytes
   - Rejection of unencrypted (plain 0xD2) injection
   - Rejection of wrong-key frames
   - End-to-end reliable delivery under 30% link loss via SimLinkBridge
   - Mismatched key rejection over link
2. Fallback AES-128-CTR stream cipher (0-byte expansion).
"""

import os
import random
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from protocol import crypto
from protocol import packets as p

try:
    from protocol.link import Link
except ImportError:
    Link = None

from scripts.secure_channel import (
    DIR_DOWNLINK,
    DIR_UPLINK,
    MAGIC_SECURE,
    OVERHEAD,
    SecurePacketCodec,
    generate_key,
)
from scripts.sim_link_bridge import create_linked_pair, patch_serial, unpatch_serial

KEY = bytes(range(16))
FIELDS = {
    "ts_ms": 12345678,
    "cls": 3,  # car
    "conf": 0.92,
    "count": 2,
    "lat": 53.5232,
    "lon": -113.5263,
}


class TestCanonicalCryptoCCM(unittest.TestCase):
    """Tests canonical AES-128-CCM AEAD encryption (protocol/crypto.py)."""

    def setUp(self):
        self.sealer = crypto.Sealer(KEY)
        self.plain_det = p.encode(p.DETECTION, 4711, **FIELDS)
        self.plain_cmd = p.encode(p.COMMAND, 55, cmd=p.CMD_HOLD, ref_seq=4711)
        self.plain_hb = p.encode(p.HEARTBEAT, 1, ts_ms=p.now_ms(), mode=p.MODE_PATROL, buffered=0)
        self.plain_ack = p.encode(p.ACK, 4711)

    def test_authenticated_wire_sizes(self):
        """Secure frames must add exactly +8 bytes (6B nonce + 4B tag - 2B CRC)."""
        frame_det = self.sealer.seal(self.plain_det)
        self.assertEqual(len(frame_det), len(self.plain_det) + OVERHEAD)
        self.assertEqual(len(frame_det), 29, "DETECTION: 21B plain -> 29B encrypted")
        self.assertEqual(frame_det[0], MAGIC_SECURE)
        self.assertEqual(frame_det[1], p.DETECTION)

        frame_cmd = self.sealer.seal(self.plain_cmd)
        self.assertEqual(len(frame_cmd), 17, "COMMAND: 9B plain -> 17B encrypted")

        frame_hb = self.sealer.seal(self.plain_hb)
        self.assertEqual(len(frame_hb), 20, "HEARTBEAT: 12B plain -> 20B encrypted")

        frame_ack = self.sealer.seal(self.plain_ack)
        self.assertEqual(len(frame_ack), 14, "ACK: 6B plain -> 14B encrypted")

    def test_round_trip_clean_decode(self):
        """Sealing then opening must return original plaintext with valid CRC16."""
        frame = self.sealer.seal(self.plain_det)
        opened = self.sealer.open(frame)
        self.assertEqual(opened, self.plain_det)

        pkt = p.decode(opened)
        self.assertIsNotNone(pkt)
        self.assertEqual(pkt["seq"], 4711)
        self.assertEqual(pkt["cls"], 3)
        self.assertAlmostEqual(pkt["conf"], 0.92, places=2)

    def test_body_is_hidden_and_nonce_fresh(self):
        """Identical packets must encrypt to different bytes; body must be hidden."""
        a = self.sealer.seal(self.plain_det)
        b = self.sealer.seal(self.plain_det)
        self.assertNotEqual(a, b, "Fresh random nonces must produce different ciphertexts")
        # Body plaintext must not appear in ciphertext
        self.assertNotIn(self.plain_det[p.HEADER.size:-p.CRC.size], a)

    def test_every_flipped_bit_is_rejected(self):
        """Any bit-flip in header, nonce, ciphertext, or auth tag must be rejected."""
        frame = self.sealer.seal(self.plain_det)
        for byte_idx in range(len(frame)):
            for bit in (0x01, 0x80):
                bad = bytearray(frame)
                bad[byte_idx] ^= bit
                self.assertIsNone(
                    self.sealer.open(bytes(bad)),
                    f"Flipped bit at byte {byte_idx} must fail auth tag check",
                )

    def test_wrong_key_and_plain_injection_rejected(self):
        """Frames sealed with different key or plain 0xD2 frames must be rejected."""
        frame = self.sealer.seal(self.plain_det)
        wrong_sealer = crypto.Sealer(os.urandom(16))
        self.assertIsNone(wrong_sealer.open(frame), "Wrong key must be rejected")
        self.assertIsNone(self.sealer.open(self.plain_det), "Plain frame must not be accepted by sealer")

    def test_key_from_env(self):
        """from_env() must parse ARCDRONE_KEY or return None when unset."""
        orig_key = os.environ.get(crypto.KEY_ENV)
        try:
            os.environ.pop(crypto.KEY_ENV, None)
            self.assertIsNone(crypto.from_env())

            test_hex = KEY.hex()
            os.environ[crypto.KEY_ENV] = test_hex
            sealer = crypto.from_env()
            self.assertIsNotNone(sealer)
            frame = sealer.seal(self.plain_det)
            self.assertEqual(sealer.open(frame), self.plain_det)
        finally:
            if orig_key:
                os.environ[crypto.KEY_ENV] = orig_key
            else:
                os.environ.pop(crypto.KEY_ENV, None)


class TestEncryptedLinkOverSimBridge(unittest.TestCase):
    """End-to-end Link layer tests with AES-128-CCM over SimLinkBridge (Windows & Linux)."""

    def setUp(self):
        if Link is None:
            self.skipTest("pyserial / Link not available in minimal environment")
        patch_serial()

    def tearDown(self):
        unpatch_serial()

    def test_encrypted_link_delivery_under_packet_loss(self):
        """Drone and Ground Link nodes exchange encrypted packets over 30% lossy radio."""
        port_a, port_b = create_linked_pair(
            name_a="sim://drone_sec",
            name_b="sim://ground_sec",
            packet_loss_rate=0.30,
            timeout=0.05,
        )

        got = []
        sealer_a = crypto.Sealer(KEY)
        sealer_b = crypto.Sealer(KEY)

        drone = Link(
            port_a,
            on_packet=lambda k: None,
            boot_wait=0.0,
            sealer=sealer_a,
            retry_timeout=0.2,
        ).start()

        ground = Link(
            port_b,
            on_packet=lambda k: k["type"] == p.DETECTION and got.append(k),
            boot_wait=0.0,
            sealer=sealer_b,
            retry_timeout=0.2,
        ).start()

        try:
            seqs = [drone.send(p.DETECTION, **FIELDS) for _ in range(5)]
            import time
            deadline = time.monotonic() + 10.0
            while len(got) < 5 and time.monotonic() < deadline:
                time.sleep(0.05)

            self.assertEqual(len(got), 5, "All 5 encrypted detections must be delivered")
            self.assertEqual([k["seq"] for k in got], seqs)
            # All received wire frames must have been sealed with D301 (magic 0xD3, type 0x01)
            for k in got:
                self.assertTrue(k["wire"].startswith("D301"))
                self.assertEqual(len(k["wire"]), 58)  # 29 bytes * 2 hex chars
        finally:
            drone.stop()
            ground.stop()

    def test_mismatched_key_link_rejected(self):
        """Endpoints with different keys must never link and must record bad packets."""
        port_a, port_b = create_linked_pair(
            name_a="sim://drone_key1",
            name_b="sim://ground_key2",
            packet_loss_rate=0.0,
            timeout=0.05,
        )

        drone = Link(
            port_a,
            boot_wait=0.0,
            sealer=crypto.Sealer(KEY),
            retry_timeout=0.1,
        ).start()

        ground = Link(
            port_b,
            boot_wait=0.0,
            sealer=crypto.Sealer(os.urandom(16)),
            retry_timeout=0.1,
        ).start()

        try:
            drone.send(p.DETECTION, **FIELDS)
            import time
            time.sleep(0.3)
            self.assertFalse(ground.up)
            self.assertGreater(ground.summary()["bad_packets"], 0)
        finally:
            drone.stop()
            ground.stop()


class TestFallbackCTRMode(unittest.TestCase):
    """Validates the optional zero-expansion (+0 bytes) CTR codec."""

    def test_ctr_zero_byte_expansion(self):
        codec = SecurePacketCodec(key=KEY, mode="CTR")
        plain = p.encode(p.DETECTION, seq=99, **FIELDS)
        self.assertEqual(len(plain), 21)

        enc = codec.encrypt_packet_data(plain, direction=DIR_DOWNLINK)
        self.assertEqual(len(enc), 21, "CTR ciphertext must be exactly 21 bytes")

        dec = codec.decrypt_packet_data(enc, direction=DIR_DOWNLINK)
        self.assertEqual(plain, dec)


if __name__ == "__main__":
    unittest.main()
