import random

from protocol import packets as p
from protocol.packets import (
    ACK, COMMAND, CMD_HOLD, DETECTION, HEARTBEAT, MODE_PATROL,
    decode, encode, from_line, to_line,
)

DETECTION_FIELDS = dict(ts_ms=123456, cls=3, conf=0.91, count=2, lat=53.5461245, lon=-113.4937266)


def test_sizes_match_spec():
    assert len(encode(DETECTION, 1, **DETECTION_FIELDS)) == 21
    assert len(encode(ACK, 1)) == 6
    assert len(encode(HEARTBEAT, 1, ts_ms=0, mode=MODE_PATROL, buffered=0)) == 12
    assert len(encode(COMMAND, 1, cmd=CMD_HOLD, ref_seq=0)) == 9


def test_detection_round_trip():
    p = decode(encode(DETECTION, 42, **DETECTION_FIELDS))
    assert p["type"] == DETECTION and p["seq"] == 42
    assert p["cls"] == 3 and p["count"] == 2 and p["ts_ms"] == 123456
    assert abs(p["conf"] - 0.91) < 1 / 255
    assert abs(p["lat"] - 53.5461245) < 1e-7
    assert abs(p["lon"] + 113.4937266) < 1e-7


def test_reencode_is_byte_exact():
    # The dashboard shows a received packet's bytes by re-encoding the decoded dict.
    rng = random.Random(1)
    for _ in range(500):
        data = p.encode(p.DETECTION, rng.randrange(0x10000), ts_ms=rng.randrange(2**32),
                        cls=rng.randrange(10), conf=rng.random(), count=rng.randrange(256),
                        lat=rng.uniform(-90, 90), lon=rng.uniform(-180, 180))
        pkt = p.decode(data)
        fields = {k: v for k, v in pkt.items() if k not in ("type", "seq")}
        assert p.encode(pkt["type"], pkt["seq"], **fields) == data


def test_command_round_trip():
    p = decode(encode(COMMAND, 9, cmd=CMD_HOLD, ref_seq=42))
    assert p == {"type": COMMAND, "seq": 9, "cmd": CMD_HOLD, "ref_seq": 42}


def test_seq_wraps():
    assert decode(encode(ACK, 0x10001))["seq"] == 1


def test_rejects_corruption():
    data = bytearray(encode(DETECTION, 1, **DETECTION_FIELDS))
    data[8] ^= 0x01
    assert decode(bytes(data)) is None


def test_rejects_foreign_and_truncated():
    good = encode(ACK, 1)
    assert decode(b"\x00" + good[1:]) is None  # wrong magic
    assert decode(good[:-1]) is None  # truncated
    assert decode(b"") is None


def test_line_round_trip():
    data = encode(DETECTION, 5, **DETECTION_FIELDS)
    line = to_line(data)
    assert line.endswith("\n") and line == line.upper()
    assert from_line(line) == data


def test_line_ignores_diagnostics_and_garbage():
    assert from_line("# bridge ready mac=AA:BB") is None
    assert from_line("ZZ\n") is None
    assert from_line("\n") is None
