"""Tactical Link Encryption Architecture for ArcDrone.

Canonical Implementation:
  AES-128-CCM (Counter with CBC-MAC) AEAD encryption and authentication,
  matching protocol/crypto.py and hardware runs on the Heltec WiFi LoRa 32 V3:
    1. AUTHENTICATED FRAMES:
       Magic 0xD3 replaces CRC16 with an AES-128-CCM 4-byte authentication tag.
       Over-the-wire sizes:
         - DETECTION: 21 B -> 29 B (+8 B)
         - COMMAND:    9 B -> 17 B (+8 B)
         - HEARTBEAT: 12 B -> 20 B (+8 B)
         - ACK:        6 B -> 14 B (+8 B)
    2. FRESH 48-BIT NONCES:
       A 6-byte random nonce is generated fresh for every transmission (including retries).
    3. FORGERY & TAMPER REJECTION:
       Changing any bit in the header, nonce, ciphertext, or auth tag causes tag
       verification to fail; plain frames (0xD2) and wrong-key frames are dropped.
    4. CONFIGURATION:
       Activated automatically on both drone and ground when ARCDRONE_KEY (32 hex chars) is set.
"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional

from protocol import crypto
from protocol import packets as p

# Canonical constants from protocol.crypto
MAGIC_SECURE = crypto.MAGIC_SECURE  # 0xD3
NONCE_LEN = crypto.NONCE_LEN        # 6
TAG_LEN = crypto.TAG_LEN            # 4
OVERHEAD = crypto.OVERHEAD          # 8 bytes (+8B on wire)
KEY_ENV = crypto.KEY_ENV            # "ARCDRONE_KEY"

# Re-export canonical Sealer and from_env
Sealer = crypto.Sealer
from_env = crypto.from_env


def generate_key() -> str:
    """Generate a random 128-bit tactical pre-shared key as 32 hex characters."""
    return os.urandom(16).hex().upper()


class SecurePacketCodec:
    """Tactical encryption codec wrapping canonical AES-128-CCM AEAD (protocol.crypto)."""

    def __init__(self, key: Optional[bytes | str] = None) -> None:
        if key is None:
            env_val = os.getenv(KEY_ENV, "").strip()
            if env_val:
                key = bytes.fromhex(env_val)
            else:
                key = b"\x00" * 16
        elif isinstance(key, str):
            key = bytes.fromhex(key) if len(key) == 32 else key.encode("utf-8")[:16].ljust(16, b"\x00")

        self.key = key
        self.sealer = crypto.Sealer(self.key)

    def encrypt_packet_data(self, plaintext_bytes: bytes) -> bytes:
        """Encrypt and seal standard packet bytes using AES-128-CCM."""
        return self.sealer.seal(plaintext_bytes)

    def decrypt_packet_data(self, encrypted_bytes: bytes) -> Optional[bytes]:
        """Verify auth tag and open secure packet bytes."""
        return self.sealer.open(encrypted_bytes)

    def encode_secure(self, ptype: int, seq: int, **fields) -> bytes:
        """Encode packet fields and seal into an encrypted wire frame."""
        plain = p.encode(ptype, seq, **fields)
        return self.encrypt_packet_data(plain)

    def decode_secure(self, data: bytes) -> Optional[Dict[str, Any]]:
        """Open encrypted frame, verify auth tag, and decode fields."""
        decrypted = self.decrypt_packet_data(data)
        if decrypted is None:
            return None
        return p.decode(decrypted)
