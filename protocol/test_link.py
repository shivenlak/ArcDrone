"""Link tests over the fake bridge. These take a few seconds: they use real timeouts."""

import time

from protocol import packets as p
from protocol.fake_bridge import FakeBridge
from protocol.link import Link

FIELDS = dict(ts_ms=0, cls=3, conf=0.9, count=1, lat=53.5, lon=-113.5)


class Side:
    def __init__(self, port):
        self.packets = []
        self.events = []
        self.link = Link(port, on_packet=self.packets.append,
                         on_event=lambda name, **info: self.events.append((name, info)),
                         boot_wait=0).start()

    def detections(self):
        return [pk for pk in self.packets if pk["type"] == p.DETECTION]

    def delivered(self):
        return [info["seq"] for name, info in self.events if name == "delivered"]


def wait_for(condition, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if condition():
            return True
        time.sleep(0.05)
    return False


def setup(loss=0.0):
    bridge = FakeBridge(loss).start()
    drone, ground = Side(bridge.ports[0]), Side(bridge.ports[1])
    assert wait_for(lambda: drone.link.up and ground.link.up)
    return bridge, drone, ground


def teardown(*sides):
    for side in sides:
        side.link.stop()


def test_detection_delivered_and_acked():
    bridge, drone, ground = setup()
    seq = drone.link.send(p.DETECTION, **FIELDS)
    assert wait_for(lambda: drone.delivered() == [seq])
    assert [d["seq"] for d in ground.detections()] == [seq]
    assert drone.link.summary()["rtt_ms_avg"] is not None
    teardown(drone, ground)


def test_exactly_once_under_loss():
    bridge, drone, ground = setup(loss=0.3)
    seqs = [drone.link.send(p.DETECTION, **FIELDS) for _ in range(10)]
    assert wait_for(lambda: drone.delivered() == seqs, timeout=30)
    assert [d["seq"] for d in ground.detections()] == seqs  # in order, no duplicates
    teardown(drone, ground)


def test_buffers_through_link_loss():
    bridge, drone, ground = setup()
    bridge.up = False
    assert wait_for(lambda: not ground.link.up and not drone.link.up, timeout=5)
    seqs = [drone.link.send(p.DETECTION, **FIELDS) for _ in range(3)]
    time.sleep(0.5)
    assert drone.link.buffered == 3 and ground.detections() == []

    bridge.up = True
    assert wait_for(lambda: drone.delivered() == seqs, timeout=10)
    assert [d["seq"] for d in ground.detections()] == seqs
    teardown(drone, ground)


def test_command_goes_ground_to_drone():
    bridge, drone, ground = setup()
    seq = ground.link.send(p.COMMAND, cmd=p.CMD_HOLD, ref_seq=0)
    assert wait_for(lambda: ground.delivered() == [seq])
    commands = [pk for pk in drone.packets if pk["type"] == p.COMMAND]
    assert [c["cmd"] for c in commands] == [p.CMD_HOLD]
    teardown(drone, ground)
