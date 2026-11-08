import os

from protocol import crypto
from protocol import packets as p
from protocol.fake_bridge import FakeBridge
from protocol.link import Link
from protocol.test_link import FIELDS, wait_for

KEY = bytes(range(16))
PLAIN = p.encode(p.DETECTION, 4711, **FIELDS)


def test_round_trip_and_size():
    s = crypto.Sealer(KEY)
    frame = s.seal(PLAIN)
    assert len(frame) == len(PLAIN) + crypto.OVERHEAD == 29
    assert frame[0] == crypto.MAGIC_SECURE and frame[1] == p.DETECTION
    assert s.open(frame) == PLAIN
    assert len(s.seal(p.encode(p.ACK, 1))) == 14


def test_body_is_hidden_and_nonce_fresh():
    s = crypto.Sealer(KEY)
    a, b = s.seal(PLAIN), s.seal(PLAIN)
    assert a != b  # new nonce each time, so identical packets look different on air
    assert PLAIN[4:-2] not in a


def test_any_flipped_bit_is_rejected():
    s = crypto.Sealer(KEY)
    frame = s.seal(PLAIN)
    for i in range(len(frame) * 8):
        bad = bytearray(frame)
        bad[i // 8] ^= 1 << (i % 8)
        assert s.open(bytes(bad)) is None


def test_wrong_key_and_plain_frames_are_rejected():
    frame = crypto.Sealer(KEY).seal(PLAIN)
    assert crypto.Sealer(os.urandom(16)).open(frame) is None
    assert crypto.Sealer(KEY).open(PLAIN) is None  # no unencrypted injection


def test_key_from_env(monkeypatch):
    monkeypatch.delenv(crypto.KEY_ENV, raising=False)
    assert crypto.from_env() is None
    monkeypatch.setenv(crypto.KEY_ENV, KEY.hex())
    assert crypto.from_env().open(crypto.Sealer(KEY).seal(PLAIN)) == PLAIN


def _link(port, sealer, got):
    return Link(port, on_packet=lambda k: k["type"] == p.DETECTION and got.append(k),
                boot_wait=0, sealer=sealer).start()


def test_encrypted_link_exactly_once_under_loss():
    bridge = FakeBridge(0.3).start()
    got = []
    drone = _link(bridge.ports[0], crypto.Sealer(KEY), [])
    ground = _link(bridge.ports[1], crypto.Sealer(KEY), got)
    seqs = [drone.send(p.DETECTION, **FIELDS) for _ in range(10)]
    assert wait_for(lambda: drone.summary()["delivered"] == 10, timeout=30)
    assert [k["seq"] for k in got] == seqs
    assert all(k["wire"].startswith("D301") and len(k["wire"]) == 58 for k in got)
    drone.stop(); ground.stop()


def test_mismatched_key_never_links():
    bridge = FakeBridge(0.0).start()
    drone = _link(bridge.ports[0], crypto.Sealer(KEY), [])
    ground = _link(bridge.ports[1], crypto.Sealer(os.urandom(16)), [])
    drone.send(p.DETECTION, **FIELDS)
    assert not wait_for(lambda: drone.up or ground.up, timeout=3)
    assert ground.summary()["bad_packets"] > 0
    drone.stop(); ground.stop()
