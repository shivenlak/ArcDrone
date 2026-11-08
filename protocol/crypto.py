"""Optional link encryption + authentication (AES-128-CCM). Spec: CLAUDE.md section 4.

Turned on by setting the same 128-bit key on both sides:

    export ARCDRONE_KEY=$(python -m protocol.crypto)     # prints a new random key

A secure frame replaces the CRC16 with an authentication tag:

    plain   D2 | type | seq(2) | body                      | crc16(2)
    secure  D3 | type | seq(2) | nonce(6) | enc(body)     | tag(4)     (+8 bytes)

- The body is encrypted. The header (type, seq) stays readable but is authenticated, so
  changing any byte of the frame makes the tag check fail and the frame is dropped.
- The 6-byte nonce is random for every transmission (retries included), so a nonce is
  never reused in practice (2^48 space).
- With a key set, plain 0xD2 frames are rejected, so nobody can inject unencrypted commands.
- Not covered: replaying a recorded frame. The receiver's dedup drops a replay of any of the
  last 64 seqs, but an older recorded frame would be accepted again.
"""

import os
import sys

from cryptography.exceptions import InvalidTag
from cryptography.hazmat.primitives.ciphers.aead import AESCCM

from protocol import packets as p

MAGIC_SECURE = 0xD3
NONCE_LEN = 6
TAG_LEN = 4
OVERHEAD = NONCE_LEN + TAG_LEN - p.CRC.size  # 8 bytes: DETECTION 21 -> 29 B, ACK 6 -> 14 B

KEY_ENV = "ARCDRONE_KEY"


class Sealer:
    def __init__(self, key):
        if len(key) != 16:
            raise ValueError("ARCDRONE_KEY must be 32 hex characters (128 bits)")
        self._aead = AESCCM(key, tag_length=TAG_LEN)

    def seal(self, data):
        """Plain packet bytes (from packets.encode) -> secure frame."""
        header = bytes([MAGIC_SECURE]) + data[1:p.HEADER.size]
        body = data[p.HEADER.size:-p.CRC.size]
        nonce = os.urandom(NONCE_LEN)
        return header + nonce + self._aead.encrypt(nonce + header, body, header)

    def open(self, frame):
        """Secure frame -> plain packet bytes (with a valid CRC), or None if not authentic."""
        start = p.HEADER.size + NONCE_LEN
        if len(frame) < start + TAG_LEN or frame[0] != MAGIC_SECURE:
            return None
        header, nonce = frame[:p.HEADER.size], frame[p.HEADER.size:start]
        try:
            body = self._aead.decrypt(nonce + header, frame[start:], header)
        except InvalidTag:
            return None
        plain = bytes([p.MAGIC]) + header[1:] + body
        return plain + p.CRC.pack(p.crc16(plain))


def from_env():
    """A Sealer if ARCDRONE_KEY is set, else None (plain packets)."""
    key = os.environ.get(KEY_ENV, "").strip()
    if not key:
        return None
    try:
        return Sealer(bytes.fromhex(key))
    except ValueError:
        raise ValueError("ARCDRONE_KEY must be 32 hex characters (128 bits)")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        sys.exit("usage: python -m protocol.crypto   (prints a new random key)")
    print(os.urandom(16).hex().upper())
