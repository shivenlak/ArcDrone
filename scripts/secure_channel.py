"""Zero-Overhead Standard Tactical Encryption for ArcDrone Primary Radio Link.

Implements AES-128-CTR (Counter Mode) encryption for compact tactical telemetry,
matching the defense radio and LoRaWAN (IEEE 802.15.4) low-bandwidth standard:
  1. ZERO PAYLOAD EXPANSION (+0 bytes):
     Because CTR mode transforms a block cipher into a stream cipher, ciphertext
     length is byte-for-byte identical to plaintext:
       - DETECTION: exactly 21 bytes
       - COMMAND:   exactly 9 bytes
       - HEARTBEAT: exactly 12 bytes
  2. DETERMINISTIC NONCE RECONSTRUCTION:
     The 16-byte AES-CTR IV is deterministically constructed from the unencrypted
     header metadata:
       [magic:1B | ptype:1B | seq:2B | dir_salt:1B | zero_padding:11B]
     Both drone and ground station independently reconstruct the exact same IV
     without transmitting a single additional byte over the radio.
  3. COMPATIBLE WITH CRC16 INTEGRITY:
     CRC16 is computed over the encrypted body, ensuring corrupted radio frames
     are discarded before decryption.
"""

from __future__ import annotations

import binascii
import os
import struct
from typing import Any, Dict, Optional, Tuple

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

from protocol import packets as p

# Default 128-bit Pre-Shared Tactical Key (PSK)
# In operational deployments, this is loaded from an environment variable or secure vault
DEFAULT_TACTICAL_PSK = os.getenv(
    "ARCDRONE_TACTICAL_PSK",
    "41726344726f6e655365637572653132",  # hex for "ArcDroneSecure12"
)

# Direction salts to ensure distinct keystreams for uplink vs downlink
DIR_DOWNLINK = 0x01  # Drone -> Ground (Detections, Heartbeats)
DIR_UPLINK = 0x02    # Ground -> Drone (Commands, Overrides)


def derive_ctr_nonce(magic: int, ptype: int, seq: int, direction: int) -> bytes:
    """Build a deterministic 16-byte IV from packet header metadata and direction."""
    # Structure: magic(1B) + ptype(1B) + seq(2B) + direction(1B) + 11 bytes zero-padding
    return struct.pack("<BBHB11x", magic, ptype, seq & 0xFFFF, direction)


class SecurePacketCodec:
    """Zero-overhead AES-128-CTR codec for ArcDrone packets."""

    def __init__(self, key_hex: Optional[str] = None):
        key_str = key_hex or DEFAULT_TACTICAL_PSK
        if len(key_str) == 32:
            self.key = bytes.fromhex(key_str)
        else:
            # Fallback: pad or slice to 16 bytes
            self.key = key_str.encode("utf-8")[:16].ljust(16, b"\x00")

    def _crypt_body(self, ptype: int, seq: int, body_bytes: bytes, direction: int) -> bytes:
        """XOR body bytes with AES-128-CTR keystream (symmetric encrypt/decrypt)."""
        nonce = derive_ctr_nonce(p.MAGIC, ptype, seq, direction)
        cipher = Cipher(algorithms.AES(self.key), modes.CTR(nonce))
        encryptor = cipher.encryptor()
        return encryptor.update(body_bytes) + encryptor.finalize()

    def encrypt_packet_data(self, plaintext_bytes: bytes, direction: int = DIR_DOWNLINK) -> bytes:
        """Encrypt the body of a standard packet without changing its total length.
        
        Header (magic, ptype, seq) remains in plaintext so radios/switches can route.
        Body is AES-128-CTR encrypted.
        CRC16 is recalculated over header + encrypted body.
        """
        if len(plaintext_bytes) < p.HEADER.size + p.CRC.size:
            return plaintext_bytes

        header = plaintext_bytes[:p.HEADER.size]
        body = plaintext_bytes[p.HEADER.size:-p.CRC.size]
        magic, ptype, seq = p.HEADER.unpack(header)

        encrypted_body = self._crypt_body(ptype, seq, body, direction)
        data = header + encrypted_body
        new_crc = p.crc16(data)
        return data + p.CRC.pack(new_crc)

    def decrypt_packet_data(self, encrypted_bytes: bytes, direction: int = DIR_DOWNLINK) -> Optional[bytes]:
        """Decrypt the body of an encrypted packet.
        
        Validates CRC16 over header + encrypted body first.
        Returns the plaintext packet bytes, or None if corrupted/invalid.
        """
        if len(encrypted_bytes) < p.HEADER.size + p.CRC.size:
            return None

        header = encrypted_bytes[:p.HEADER.size]
        body = encrypted_bytes[p.HEADER.size:-p.CRC.size]
        (crc,) = p.CRC.unpack(encrypted_bytes[-p.CRC.size:])

        # Verify radio checksum before decrypting
        if crc != p.crc16(encrypted_bytes[:-p.CRC.size]):
            return None

        magic, ptype, seq = p.HEADER.unpack(header)
        decrypted_body = self._crypt_body(ptype, seq, body, direction)
        data = header + decrypted_body
        original_crc = p.crc16(data)
        return data + p.CRC.pack(original_crc)

    def encode_secure(self, ptype: int, seq: int, direction: int = DIR_DOWNLINK, **fields) -> bytes:
        """Encode packet fields and immediately encrypt body (0-byte expansion)."""
        plaintext = p.encode(ptype, seq, **fields)
        return self.encrypt_packet_data(plaintext, direction=direction)

    def decode_secure(self, data: bytes, direction: int = DIR_DOWNLINK) -> Optional[Dict[str, Any]]:
        """Decrypt body and decode into standard packet dict."""
        decrypted = self.decrypt_packet_data(data, direction=direction)
        if decrypted is None:
            return None
        return p.decode(decrypted)
