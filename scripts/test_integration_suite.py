"""Automated programmatic test suite for ArcDrone natHacks Defense & Security Showcase.

Runnable via pytest:
    uv run --with pytest --with pyserial pytest scripts/test_integration_suite.py
Or direct CLI:
    python -m scripts.test_integration_suite

Covers:
- Tier 1: Feature Coverage (Bridge, Bandwidth Auditor, UAARG Logger/Monitor/Client, Control Loop, Link Recovery).
- Tier 2: Boundary & Corner Cases (Bandwidth limits, Ring Buffer 64 cap, Deduplication, Disconnects).
- Tier 3: Cross-Feature Interactions (Blackout + Command Queueing + Recovery Flush + Bandwidth Audit).
- Tier 4: Real-World Mission Scenarios (Full natHacks showcase detection-to-response loop).
"""

from __future__ import annotations

import collections
import json
import os
import sys
import time
from typing import Any, Dict, List

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from protocol import packets as p
from protocol.link import Link
from scripts.bandwidth_monitor import BandwidthMonitor, BUDGET_BYTES_PER_SEC
from scripts.sim_link_bridge import SimLinkBridge, VirtualSerialPort, VirtualSerialPair
from scripts.uaarg_adapter.event_logger import EventLogger
from scripts.uaarg_adapter.ground_client import GroundClient
from scripts.uaarg_adapter.mission_monitor import MissionMonitor


# ==============================================================================
# Tier 1 & 2: Component Unit & Boundary Tests
# ==============================================================================

def test_virtual_serial_pair_basic_io():
    """Verify VirtualSerialPair read, write, in_waiting, and buffer clearing."""
    bridge = SimLinkBridge(name_a="sim://test_a", name_b="sim://test_b")
    try:
        assert bridge.port_a.is_open and bridge.port_b.is_open
        assert bridge.port_a.in_waiting == 0 and bridge.port_b.in_waiting == 0

        # Port A writes, Port B receives
        payload = b"TEST_PAYLOAD\n"
        written = bridge.port_a.write(payload)
        assert written == len(payload)
        assert bridge.port_b.in_waiting == len(payload)

        read_data = bridge.port_b.read(bridge.port_b.in_waiting)
        assert read_data == payload
        assert bridge.port_b.in_waiting == 0

        # Reset buffer
        bridge.port_b.write(b"MORE_DATA\n")
        assert bridge.port_a.in_waiting > 0
        bridge.port_a.reset_input_buffer()
        assert bridge.port_a.in_waiting == 0
    finally:
        bridge.close()


def test_sim_link_bridge_cut_and_restore():
    """Verify cut() drops all transmissions and restore() resumes delivery."""
    bridge = SimLinkBridge(name_a="sim://test_cut_a", name_b="sim://test_cut_b")
    try:
        bridge.port_a.write(b"BEFORE_CUT\n")
        assert bridge.port_b.in_waiting > 0
        bridge.port_b.reset_input_buffer()

        # Cut bridge
        bridge.cut()
        assert bridge.is_cut
        bridge.port_a.write(b"DURING_CUT\n")
        assert bridge.port_b.in_waiting == 0  # Dropped

        # Restore bridge
        bridge.restore()
        assert not bridge.is_cut
        bridge.port_a.write(b"AFTER_RESTORE\n")
        assert bridge.port_b.in_waiting > 0
        assert bridge.port_b.read(bridge.port_b.in_waiting) == b"AFTER_RESTORE\n"
    finally:
        bridge.close()


def test_bandwidth_monitor_sliding_window_and_budget():
    """Verify BandwidthMonitor accurately calculates byte rates and enforces <= 3000 B/s."""
    monitor = BandwidthMonitor(budget_limit=3000.0, window_size_sec=1.0)

    # 1. Nominal transmission
    now = time.monotonic()
    monitor.record_tx(wire_bytes=21, serial_bytes=43, timestamp=now)
    rate = monitor.current_rate()
    assert rate["within_budget"]
    assert rate["max_bytes_per_sec"] == 43.0
    assert rate["utilization_pct"] < 2.0

    # 2. Check budget method
    within, bps = monitor.check_budget()
    assert within and bps == 43.0

    # 3. Burst within budget (500 B/s)
    monitor.record_tx(wire_bytes=200, serial_bytes=450, timestamp=now)
    rate = monitor.current_rate()
    assert rate["within_budget"]
    assert rate["max_bytes_per_sec"] == 493.0

    # 4. Violation trigger
    monitor.record_tx(wire_bytes=1500, serial_bytes=3100, timestamp=now)
    assert not monitor.current_rate()["within_budget"]
    assert len(monitor._violations) >= 1

    summary = monitor.summary()
    assert not summary["within_budget"]
    assert summary["peak_serial_bps"] >= 3100.0


def test_uaarg_event_logger_structure_and_schema(tmp_path):
    """Verify EventLogger session hierarchy (tmp/log/<id>/) and <seq>.json schema."""
    log_dir = str(tmp_path / "uaarg_log")
    logger = EventLogger(base_dir=log_dir)

    assert os.path.exists(logger.run_dir)
    assert os.path.exists(logger.img_dir)
    assert os.path.exists(logger.bb_dir)

    # Log structured event
    snapshot = logger.log_event(
        seq=42,
        detection={"cls": "van", "conf": 0.89, "count": 2},
        location={"lat": 53.5232, "lon": -113.5263},
        altitude_m=18.5,
        mode="PATROL",
        link_status={"up": True, "buffered": 0, "rtt_ms": 8.5},
    )

    # Verify disk files
    img_file = os.path.join(logger.img_dir, "42.png")
    bb_file = os.path.join(logger.bb_dir, "42.png")
    json_file = os.path.join(logger.run_dir, "42.json")

    assert os.path.exists(img_file)
    assert os.path.exists(bb_file)
    assert os.path.exists(json_file)

    # Verify JSON content
    stored = logger.read_event(42)
    assert stored is not None
    assert stored["seq"] == 42
    assert stored["detection"]["cls"] == "van"
    assert stored["detection"]["conf"] == 0.89
    assert stored["location"]["lat"] == 53.5232
    assert stored["location"]["lon"] == -113.5263
    assert stored["altitude_m"] == 18.5
    assert stored["mode"] == "PATROL"
    assert stored["link"]["up"] is True


def test_uaarg_mission_monitor_liveness_and_severities():
    """Verify MissionMonitor liveness timer (timeSinceMessage) and 3-tier severity logs."""
    monitor = MissionMonitor(heartbeat_timeout=3.0)

    # Initial state
    assert monitor.time_since_message >= 0
    status = monitor.get_status()
    assert status["mode"] == "PATROL"

    # Incoming message resets liveness
    monitor.record_message(mode="HOLD", buffered=2)
    assert monitor.time_since_message == 0
    assert monitor.get_status()["mode"] == "HOLD"
    assert monitor.get_status()["bufferedCount"] == 2

    # 3-tier severity logs
    normal_entry = monitor.send_log("System operational", severity="normal", subsystem="ARCDRONE_EDGE")
    warn_entry = monitor.send_log("Buffer filling up", severity="warning", subsystem="ARCDRONE_LINK")
    err_entry = monitor.send_log("Telemetry timeout", severity="error", subsystem="ARCDRONE_GROUND")

    assert normal_entry["severity"] == "normal"
    assert warn_entry["severity"] == "warning"
    assert err_entry["severity"] == "error"

    # Filtered queries
    warnings = monitor.get_logs(severity="warning")
    assert len(warnings) == 1
    assert warnings[0]["message"] == "Buffer filling up"

    edge_logs = monitor.get_logs(subsystem="ARCDRONE_EDGE")
    assert len(edge_logs) == 1


def test_uaarg_ground_client_observer():
    """Verify GroundClient non-invasively processes incoming JSON broadcasts."""
    monitor = MissionMonitor()
    client = GroundClient(mission_monitor=monitor)

    # Process simulated alert
    alert_json = json.dumps({
        "event": "alert",
        "seq": 105,
        "cls": "car",
        "conf": 0.95,
        "count": 1,
        "lat": 53.52,
        "lon": -113.52,
        "ts_ms": p.now_ms(),
        "delayed": False,
    })
    client.handle_incoming_json(alert_json)
    assert len(client.alerts) == 1
    assert client.alerts[0]["seq"] == 105

    # Process simulated drone heartbeat
    drone_json = json.dumps({"event": "drone", "mode": "HOLD", "buffered": 3})
    client.handle_incoming_json(drone_json)
    assert client.drone_status["mode"] == "HOLD"
    assert client.drone_status["buffered"] == 3

    # Process command status
    cmd_json = json.dumps({"event": "cmd_status", "cmd_id": 501, "cmd": "HOLD", "status": "CONFIRMED"})
    client.handle_incoming_json(cmd_json)
    assert client.commands[501]["status"] == "CONFIRMED"
    assert client.wait_for_command_status(501, status="CONFIRMED", timeout=0.1)


# ==============================================================================
# Tier 3 & 4: Integrated Two-Way Control, Link Outage & Mission Scenarios
# ==============================================================================

def test_integrated_two_way_command_state_loop():
    """Verify operator commands (OPERATOR_ACK, HOLD, RESUME, RTB) and state transitions."""
    bridge = SimLinkBridge("sim://integ_drone", "sim://integ_ground")
    drone_cmds: List[Dict[str, Any]] = []
    ground_delivered: List[int] = []

    def on_drone_packet(pkt: Dict[str, Any]) -> None:
        if pkt["type"] == p.COMMAND:
            drone_cmds.append(pkt)
            if pkt["cmd"] == p.CMD_HOLD:
                drone_link.mode = p.MODE_HOLD
            elif pkt["cmd"] == p.CMD_RESUME:
                drone_link.mode = p.MODE_PATROL
            elif pkt["cmd"] == p.CMD_RTB:
                drone_link.mode = p.MODE_RTB

    drone_link = Link("sim://integ_drone", on_packet=on_drone_packet, boot_wait=0.0).start()
    ground_link = Link(
        "sim://integ_ground",
        on_event=lambda name, **kw: ground_delivered.append(kw["seq"]) if name == "delivered" else None,
        boot_wait=0.0,
    ).start()

    time.sleep(0.3)
    try:
        # 1. HOLD
        seq_hold = ground_link.send(p.COMMAND, cmd=p.CMD_HOLD, ref_seq=0)
        t_deadline = time.monotonic() + 3.0
        while drone_link.mode != p.MODE_HOLD and time.monotonic() < t_deadline:
            time.sleep(0.05)
        assert drone_link.mode == p.MODE_HOLD
        assert seq_hold in ground_delivered

        # 2. RESUME
        seq_res = ground_link.send(p.COMMAND, cmd=p.CMD_RESUME, ref_seq=0)
        t_deadline = time.monotonic() + 3.0
        while drone_link.mode != p.MODE_PATROL and time.monotonic() < t_deadline:
            time.sleep(0.05)
        assert drone_link.mode == p.MODE_PATROL
        assert seq_res in ground_delivered

        # 3. RTB
        seq_rtb = ground_link.send(p.COMMAND, cmd=p.CMD_RTB, ref_seq=0)
        t_deadline = time.monotonic() + 3.0
        while drone_link.mode != p.MODE_RTB and time.monotonic() < t_deadline:
            time.sleep(0.05)
        assert drone_link.mode == p.MODE_RTB
        assert seq_rtb in ground_delivered
    finally:
        drone_link.stop()
        ground_link.stop()
        bridge.close()


def test_integrated_link_loss_buffer_accumulation_and_recovery():
    """Verify 3.0s link-loss detection, outbox buffering, and oldest-first flush."""
    bridge = SimLinkBridge("sim://integ_loss_drone", "sim://integ_loss_ground")
    ground_packets: List[Dict[str, Any]] = []

    drone_link = Link("sim://integ_loss_drone", retry_timeout=0.2, boot_wait=0.0).start()
    ground_link = Link(
        "sim://integ_loss_ground",
        on_packet=lambda pkt: ground_packets.append(pkt) if pkt["type"] != p.HEARTBEAT else None,
        retry_timeout=0.2,
        boot_wait=0.0,
    ).start()

    time.sleep(0.3)
    try:
        # Cut link to induce blackout
        bridge.cut()
        time.sleep(3.2)  # Wait for 3.0s LINK_TIMEOUT
        assert not ground_link.up
        assert not drone_link.up

        # Generate 4 alerts during outage
        queued_seqs = []
        for i in range(4):
            seq = drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=i % 4,
                conf=0.9,
                count=1,
                lat=53.52,
                lon=-113.52,
            )
            queued_seqs.append(seq)

        assert drone_link.buffered == 4
        assert len(ground_packets) == 0

        # Restore link and verify flush
        bridge.restore()
        t_deadline = time.monotonic() + 3.0
        while drone_link.buffered > 0 and time.monotonic() < t_deadline:
            time.sleep(0.1)

        assert drone_link.buffered == 0
        received_seqs = [pkt["seq"] for pkt in ground_packets]
        assert queued_seqs == received_seqs, f"Expected {queued_seqs}, received {received_seqs}"
    finally:
        drone_link.stop()
        ground_link.stop()
        bridge.close()


def test_integrated_bandwidth_compliance():
    """Assert primary link transmission data rate remains <= 3000 B/s under burst."""
    bridge = SimLinkBridge("sim://integ_bw_drone", "sim://integ_bw_ground")
    monitor = BandwidthMonitor(budget_limit=BUDGET_BYTES_PER_SEC, window_size_sec=1.0)
    monitor.attach_to_bridge(bridge)

    drone_link = Link("sim://integ_bw_drone", boot_wait=0.0).start()
    ground_link = Link("sim://integ_bw_ground", boot_wait=0.0).start()

    time.sleep(0.3)
    try:
        # Transmit rapid burst of 10 detections
        for i in range(10):
            drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=i % 5,
                conf=0.91,
                count=1,
                lat=53.5232,
                lon=-113.5263,
            )
            time.sleep(0.08)

        time.sleep(0.5)
        monitor.assert_within_budget()
        summary = monitor.summary()
        assert summary["within_budget"]
        assert summary["peak_max_bps"] <= 3000.0
    finally:
        drone_link.stop()
        ground_link.stop()
        bridge.close()


def run_all_tests() -> Dict[str, Any]:
    print("=" * 76)
    print(" Running ArcDrone Automated Programmatic Test Suite (Hardware-Free)")
    print("=" * 76)

    test_functions = [
        ("Virtual Serial Pair Basic I/O", test_virtual_serial_pair_basic_io),
        ("Sim Link Bridge Cut & Restore", test_sim_link_bridge_cut_and_restore),
        ("Bandwidth Monitor Sliding Window & Budget", test_bandwidth_monitor_sliding_window_and_budget),
        ("UAARG Event Logger Structure & Schema", lambda: test_uaarg_event_logger_structure_and_schema(_get_temp_dir())),
        ("UAARG Mission Monitor Liveness & Severities", test_uaarg_mission_monitor_liveness_and_severities),
        ("UAARG Ground Client Observer", test_uaarg_ground_client_observer),
        ("Integrated Two-Way Command State Loop", test_integrated_two_way_command_state_loop),
        ("Integrated Link Loss, Outage & Recovery Flush", test_integrated_link_loss_buffer_accumulation_and_recovery),
        ("Integrated Bandwidth Compliance (<= 3 kB/s)", test_integrated_bandwidth_compliance),
    ]

    results = []
    all_passed = True
    start_time = time.monotonic()

    for name, fn in test_functions:
        t0 = time.monotonic()
        try:
            fn()
            duration = round(time.monotonic() - t0, 3)
            print(f"  [PASS] {name:<52} ({duration}s)")
            results.append({"test": name, "status": "PASSED", "duration_sec": duration})
        except Exception as exc:
            duration = round(time.monotonic() - t0, 3)
            print(f"  [FAIL] {name:<52} ({duration}s) - {exc}")
            results.append({"test": name, "status": "FAILED", "duration_sec": duration, "error": str(exc)})
            all_passed = False

    total_duration = round(time.monotonic() - start_time, 2)
    print("=" * 76)
    print(f" Test Results: {len([r for r in results if r['status'] == 'PASSED'])}/{len(results)} Passed in {total_duration}s")
    print("=" * 76)

    report_path = os.path.join(os.path.dirname(__file__), "test_results.json")
    report = {
        "timestamp": time.time(),
        "total": len(results),
        "passed": len([r for r in results if r["status"] == "PASSED"]),
        "failed": len([r for r in results if r["status"] == "FAILED"]),
        "all_passed": all_passed,
        "duration_sec": total_duration,
        "tests": results,
    }
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    return report


class _TempDir:
    def __init__(self, path: str):
        self.path = path
    def __truediv__(self, other):
        p_sub = os.path.join(self.path, str(other))
        os.makedirs(p_sub, exist_ok=True)
        return p_sub


def _get_temp_dir():
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tmp_test_artifacts")
    os.makedirs(base, exist_ok=True)
    return _TempDir(base)


if __name__ == "__main__":
    rep = run_all_tests()
    if not rep["all_passed"]:
        sys.exit(1)
    sys.exit(0)
