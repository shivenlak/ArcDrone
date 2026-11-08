"""Comprehensive Test Suite for Zero-Overhead AES-128-CTR Tactical Encryption."""

import os
import random
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from protocol import packets as p
from scripts.secure_channel import (
    DEFAULT_TACTICAL_PSK,
    DIR_DOWNLINK,
    DIR_UPLINK,
    SecurePacketCodec,
)


class TestSecureChannel(unittest.TestCase):
    def setUp(self):
        self.codec = SecurePacketCodec()

    def test_zero_byte_expansion_detection(self):
        """DETECTION packet must remain exactly 21 bytes with zero size inflation."""
        plain = p.encode(
            p.DETECTION,
            seq=101,
            ts_ms=p.now_ms(),
            cls=3,  # car
            conf=0.92,
            count=2,
            lat=53.5232,
            lon=-113.5263,
        )
        self.assertEqual(len(plain), 21, "Plaintext DETECTION must be 21 bytes")

        encrypted = self.codec.encrypt_packet_data(plain, direction=DIR_DOWNLINK)
        self.assertEqual(len(encrypted), 21, "Ciphertext DETECTION must be exactly 21 bytes (0% expansion)")

        # Verify body bytes are scrambled
        self.assertNotEqual(plain[p.HEADER.size:-p.CRC.size], encrypted[p.HEADER.size:-p.CRC.size])

        # Decrypt and verify round-trip
        decrypted = self.codec.decrypt_packet_data(encrypted, direction=DIR_DOWNLINK)
        self.assertEqual(plain, decrypted)

    def test_zero_byte_expansion_command(self):
        """COMMAND packet must remain exactly 9 bytes with zero size inflation."""
        plain = p.encode(p.COMMAND, seq=55, cmd=p.CMD_HOLD, ref_seq=101)
        self.assertEqual(len(plain), 9, "Plaintext COMMAND must be 9 bytes")

        encrypted = self.codec.encrypt_packet_data(plain, direction=DIR_UPLINK)
        self.assertEqual(len(encrypted), 9, "Ciphertext COMMAND must be exactly 9 bytes (0% expansion)")

        decrypted = self.codec.decrypt_packet_data(encrypted, direction=DIR_UPLINK)
        self.assertEqual(plain, decrypted)

    def test_zero_byte_expansion_heartbeat(self):
        """HEARTBEAT packet must remain exactly 12 bytes with zero size inflation."""
        plain = p.encode(p.HEARTBEAT, seq=1, ts_ms=p.now_ms(), mode=p.MODE_PATROL, buffered=0)
        self.assertEqual(len(plain), 12, "Plaintext HEARTBEAT must be 12 bytes")

        encrypted = self.codec.encrypt_packet_data(plain, direction=DIR_DOWNLINK)
        self.assertEqual(len(encrypted), 12, "Ciphertext HEARTBEAT must be exactly 12 bytes (0% expansion)")

        decrypted = self.codec.decrypt_packet_data(encrypted, direction=DIR_DOWNLINK)
        self.assertEqual(plain, decrypted)

    def test_encode_decode_secure_high_volume(self):
        """Verify 500 random packets round-trip cleanly with zero corruption."""
        rng = random.Random(42)
        for i in range(500):
            seq = rng.randint(0, 65535)
            ts = rng.randint(0, 2**32 - 1)
            cls_idx = rng.randint(0, 9)
            conf = round(rng.uniform(0.1, 1.0), 2)
            count = rng.randint(1, 10)
            lat = round(rng.uniform(-89.0, 89.0), 6)
            lon = round(rng.uniform(-179.0, 179.0), 6)

            enc = self.codec.encode_secure(
                p.DETECTION,
                seq=seq,
                direction=DIR_DOWNLINK,
                ts_ms=ts,
                cls=cls_idx,
                conf=conf,
                count=count,
                lat=lat,
                lon=lon,
            )
            self.assertEqual(len(enc), 21)

            pkt = self.codec.decode_secure(enc, direction=DIR_DOWNLINK)
            self.assertIsNotNone(pkt)
            self.assertEqual(pkt["seq"], seq)
            self.assertEqual(pkt["cls"], cls_idx)
            self.assertAlmostEqual(pkt["conf"], conf, delta=0.01)
            self.assertAlmostEqual(pkt["lat"], lat, places=4)
            self.assertAlmostEqual(pkt["lon"], lon, places=4)

    def test_tamper_detection(self):
        """Bit flips in encrypted payload must trigger CRC rejection."""
        plain = p.encode(p.COMMAND, seq=12, cmd=p.CMD_RTB, ref_seq=99)
        encrypted = self.codec.encrypt_packet_data(plain, direction=DIR_UPLINK)

        # Flip a bit in the encrypted body
        corrupted = bytearray(encrypted)
        corrupted[p.HEADER.size] ^= 0x01
        corrupted_bytes = bytes(corrupted)

        dec = self.codec.decrypt_packet_data(corrupted_bytes, direction=DIR_UPLINK)
        self.assertIsNone(dec, "Tampered packet must be rejected by CRC check")

    def test_wrong_key_rejection(self):
        """Packets decrypted with an incorrect PSK must fail decoding or yield garbage."""
        other_codec = SecurePacketCodec(key_hex="ffffffffffffffffffffffffffffffff")
        enc = self.codec.encode_secure(
            p.DETECTION,
            seq=44,
            direction=DIR_DOWNLINK,
            ts_ms=p.now_ms(),
            cls=1,
            conf=0.85,
            count=1,
            lat=53.52,
            lon=-113.52,
        )

        dec = other_codec.decode_secure(enc, direction=DIR_DOWNLINK)
        # Even if CRC matches header, decrypted body fields will be scrambled
        if dec is not None:
            self.assertNotEqual(dec["cls"], 1, "Wrong key should scramble class ID")

    def test_direction_isolation(self):
        """Downlink encryption must not decode under uplink direction salt."""
        enc = self.codec.encode_secure(
            p.COMMAND, seq=7, direction=DIR_DOWNLINK, cmd=p.CMD_HOLD, ref_seq=10
        )
        dec = self.codec.decode_secure(enc, direction=DIR_UPLINK)
        if dec is not None:
            self.assertNotEqual(dec.get("cmd"), p.CMD_HOLD, "Direction mismatch must scramble body")


if __name__ == "__main__":
    unittest.main()
