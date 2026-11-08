"""Bandwidth Verification Benchmark for ArcDrone LoRa/ESP-NOW primary communication link.

Asserts requirement R1 from ORIGINAL_REQUEST.md:
Primary communication link data rate for detection and bounding box telemetry
remains within the 3 kB/s (3000 B/s) bandwidth limit during active transmission.
"""

from __future__ import annotations

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
from scripts.sim_link_bridge import SimLinkBridge


def run_bandwidth_benchmark(report_path: str = "scripts/bandwidth_report.json") -> Dict[str, Any]:
    print("=" * 70)
    print(" ArcDrone LoRa Primary Link Bandwidth Benchmark (<= 3 kB/s)")
    print("=" * 70)

    bridge = SimLinkBridge(name_a="sim://drone_bw", name_b="sim://ground_bw")
    monitor = BandwidthMonitor(budget_limit=BUDGET_BYTES_PER_SEC, window_size_sec=1.0)
    monitor.attach_to_bridge(bridge)

    ground_received: List[Dict[str, Any]] = []
    drone_received: List[Dict[str, Any]] = []

    drone_link = Link(
        "sim://drone_bw",
        on_packet=lambda pkt: drone_received.append(pkt),
        boot_wait=0.0,
    ).start()
    ground_link = Link(
        "sim://ground_bw",
        on_packet=lambda pkt: ground_received.append(pkt),
        boot_wait=0.0,
    ).start()

    time.sleep(0.3)
    results = []

    try:
        # --- Scenario 1: Baseline Idle Link (1 Hz Heartbeat) ---
        print("\n[Scenario 1] Idle Link Baseline (1 Hz Heartbeats)")
        s1_start = time.monotonic()
        time.sleep(2.1)
        s1_stats = monitor.current_rate()
        print(f"  Peak Rate: {s1_stats['max_bytes_per_sec']} B/s | Budget: 3000 B/s | Utilization: {s1_stats['utilization_pct']}%")
        results.append({
            "scenario": "Idle Baseline (1 Hz Heartbeat)",
            "packets": 2,
            "peak_bps": s1_stats["max_bytes_per_sec"],
            "utilization": s1_stats["utilization_pct"],
            "passed": s1_stats["within_budget"],
        })
        assert s1_stats["within_budget"], "Scenario 1 exceeded bandwidth budget!"

        # --- Scenario 2: Active Patrol Telemetry (1 alert/s + Heartbeats) ---
        print("\n[Scenario 2] Active Patrol Telemetry (1 detection/s with GPS)")
        for i in range(3):
            drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=3,  # car
                conf=0.91,
                count=1,
                lat=53.5232 + (i * 0.0001),
                lon=-113.5263 + (i * 0.0001),
            )
            time.sleep(1.0)

        s2_stats = monitor.current_rate()
        print(f"  Peak Rate: {s2_stats['max_bytes_per_sec']} B/s | Budget: 3000 B/s | Utilization: {s2_stats['utilization_pct']}%")
        results.append({
            "scenario": "Active Patrol (1 detection/s)",
            "packets": 3,
            "peak_bps": s2_stats["max_bytes_per_sec"],
            "utilization": s2_stats["utilization_pct"],
            "passed": s2_stats["within_budget"],
        })
        assert s2_stats["within_budget"], "Scenario 2 exceeded bandwidth budget!"

        # --- Scenario 3: Heavy Detection Burst (10 detections / second) ---
        print("\n[Scenario 3] Heavy Detection Burst (10 detections/s convoy)")
        burst_seqs = []
        for i in range(10):
            seq = drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=i % len(p.CLASS_NAMES),
                conf=0.88,
                count=i + 1,
                lat=53.5232,
                lon=-113.5263,
            )
            burst_seqs.append(seq)
            time.sleep(0.08)  # 10 detections spaced across 800ms

        time.sleep(0.5)
        s3_stats = monitor.current_rate()
        print(f"  Peak Rate: {s3_stats['max_bytes_per_sec']} B/s | Budget: 3000 B/s | Utilization: {s3_stats['utilization_pct']}%")
        results.append({
            "scenario": "Detection Burst (10 detections/s)",
            "packets": 10,
            "peak_bps": s3_stats["max_bytes_per_sec"],
            "utilization": s3_stats["utilization_pct"],
            "passed": s3_stats["within_budget"],
        })
        assert s3_stats["within_budget"], "Scenario 3 exceeded bandwidth budget!"

        # --- Scenario 4: Two-Way Tactical Control Exchange ---
        print("\n[Scenario 4] Two-Way Tactical Control Exchange (Uplink + Downlink Commands)")
        for cmd_type in (p.CMD_OPERATOR_ACK, p.CMD_HOLD, p.CMD_RESUME, p.CMD_RTB):
            # Ground sends command
            ground_link.send(p.COMMAND, cmd=cmd_type, ref_seq=101)
            # Drone sends concurrent alert
            drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=0,  # pedestrian
                conf=0.95,
                count=1,
                lat=53.5200,
                lon=-113.5200,
            )
            time.sleep(0.3)

        time.sleep(0.5)
        s4_stats = monitor.current_rate()
        print(f"  Peak Rate: {s4_stats['max_bytes_per_sec']} B/s | Budget: 3000 B/s | Utilization: {s4_stats['utilization_pct']}%")
        results.append({
            "scenario": "Two-Way Control (Alerts + Commands)",
            "packets": 8,
            "peak_bps": s4_stats["max_bytes_per_sec"],
            "utilization": s4_stats["utilization_pct"],
            "passed": s4_stats["within_budget"],
        })
        assert s4_stats["within_budget"], "Scenario 4 exceeded bandwidth budget!"

        # Overall validation
        monitor.assert_within_budget()
        overall_summary = monitor.summary()
        overall_summary["scenarios"] = results

        print("\n" + "=" * 70)
        print(" BENCHMARK SUMMARY RESULTS")
        print("=" * 70)
        print(f"{'Scenario':<38} | {'Peak Rate':<10} | {'Util %':<8} | {'Status'}")
        print("-" * 70)
        for r in results:
            status_str = "PASS" if r["passed"] else "FAIL"
            print(f"{r['scenario']:<38} | {r['peak_bps']:>7.1f} B/s | {r['utilization']:>6.1f}% | {status_str}")
        print("-" * 70)
        print(f"Peak Overall Wire Rate:   {overall_summary['peak_wire_bps']} B/s")
        print(f"Peak Overall Serial Rate: {overall_summary['peak_serial_bps']} B/s")
        print(f"Budget Limit:             {overall_summary['budget_limit_bps']} B/s (3 kB/s)")
        print(f"All Scenarios Compliant:  {'YES (VERIFIED)' if overall_summary['within_budget'] else 'NO'}")
        print("=" * 70)

        # Export report
        monitor.export_report(report_path)
        print(f"\nAudit report exported to: {report_path}")
        return overall_summary

    finally:
        drone_link.stop()
        ground_link.stop()
        bridge.close()


if __name__ == "__main__":
    report_file = os.path.join(os.path.dirname(__file__), "bandwidth_report.json")
    try:
        res = run_bandwidth_benchmark(report_file)
        if not res["within_budget"]:
            sys.exit(1)
        sys.exit(0)
    except Exception as exc:
        print(f"Benchmark failed with error: {exc}")
        sys.exit(1)
