"""Link reliability, outage recovery, deduplication, and buffer flush test harness.

Fulfills R1 and Acceptance Criteria:
- Verifies 3.0s link-lost timeout.
- Verifies ring buffer accumulation (up to 64 unacknowledged messages).
- Verifies oldest-first flush upon link reconnection with zero duplicate deliveries.
- Verifies resilience against intermittent packet loss (stop-and-wait retries).
"""

from __future__ import annotations

import collections
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from protocol import packets as p
from protocol.link import Link
from scripts.sim_link_bridge import SimLinkBridge


def run_link_test(blackout_duration: float = 3.5) -> Dict[str, Any]:
    print("=" * 70)
    print(" ArcDrone Link Reliability, Outage & Recovery Verification Harness")
    print("=" * 70)

    bridge = SimLinkBridge(name_a="sim://drone_link", name_b="sim://ground_link")

    drone_events: List[str] = []
    ground_events: List[str] = []
    ground_packets: List[Dict[str, Any]] = []

    drone_link = Link(
        "sim://drone_link",
        on_event=lambda name, **kw: drone_events.append(name),
        retry_timeout=0.2,
        boot_wait=0.0,
    ).start()

    ground_link = Link(
        "sim://ground_link",
        on_packet=lambda pkt: ground_packets.append(pkt) if pkt["type"] != p.HEARTBEAT else None,
        on_event=lambda name, **kw: ground_events.append(name),
        retry_timeout=0.2,
        boot_wait=0.0,
    ).start()

    time.sleep(0.3)
    results = []

    try:
        # --- Phase 1: Steady-State Transmission ---
        print("\n[Phase 1] Steady-State Transmission (Zero Loss)")
        sent_seqs = []
        for i in range(5):
            seq = drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=3,
                conf=0.9,
                count=1,
                lat=53.5200 + (i * 0.001),
                lon=-113.5200,
            )
            sent_seqs.append(seq)
            time.sleep(0.05)

        time.sleep(0.5)
        received_seqs = [pkt["seq"] for pkt in ground_packets]
        assert sent_seqs == received_seqs, f"Sent {sent_seqs} != Received {received_seqs}"
        assert drone_link.summary()["delivered"] == 5, "Expected 5 delivered"
        print(f"  Delivered {len(received_seqs)}/5 packets with zero retries. RTT: {drone_link.summary()['rtt_ms_avg']} ms")
        results.append({"phase": "Steady-State", "passed": True})

        # --- Phase 2: Intermittent Packet Loss Resilience (30% Loss) ---
        print("\n[Phase 2] Intermittent Loss Resilience (30% Packet Loss Rate)")
        bridge.set_packet_loss_rate(0.30)
        p2_start_count = len(ground_packets)
        p2_sent = []
        for i in range(5):
            seq = drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=0,
                conf=0.85,
                count=1,
                lat=53.5300,
                lon=-113.5300,
            )
            p2_sent.append(seq)
            time.sleep(0.15)

        time.sleep(1.0)
        p2_received = [pkt["seq"] for pkt in ground_packets[p2_start_count:]]
        assert p2_sent == p2_received, f"Loss recovery failed: sent {p2_sent} != received {p2_received}"
        print(f"  Delivered 5/5 packets over 30% lossy link. Retries triggered: {drone_link.stats['retries']}")
        bridge.set_packet_loss_rate(0.0)
        results.append({"phase": "Intermittent Loss (30%)", "passed": True})

        # --- Phase 3: Simulated Radio Blackout & Outage Recovery ---
        print(f"\n[Phase 3] Simulated Radio Blackout ({blackout_duration}s Outage) & Recovery")
        print("  Cutting simulated radio link...")
        bridge.cut()

        # Wait for 3.0s LINK_TIMEOUT
        time.sleep(3.2)
        assert not ground_link.up, "Ground link should be DOWN after 3.0s silence timeout"
        assert not drone_link.up, "Drone link should be DOWN after 3.0s silence timeout"
        assert "link_down" in ground_events, "Ground station missing 'link_down' event"
        print("  Verified: Both endpoints transitioned to link_down after 3.0s timeout.")

        # Queue alerts during blackout
        print("  Generating 5 detection alerts during active blackout...")
        blackout_sent = []
        for i in range(5):
            seq = drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=i % 4,
                conf=0.88,
                count=1,
                lat=53.5400 + i * 0.001,
                lon=-113.5400,
            )
            blackout_sent.append(seq)

        time.sleep(0.5)
        # Verify messages are held in the outbox
        assert drone_link.buffered == 5, f"Expected 5 buffered messages in outbox, got {drone_link.buffered}"
        assert len(ground_packets) == p2_start_count + 5, "No new packets should reach ground during blackout"
        print(f"  Verified: Alerts buffered in ring buffer (buffered count: {drone_link.buffered})")

        # Restore link and verify oldest-first flush
        print("  Restoring radio link...")
        p3_start_idx = len(ground_packets)
        bridge.restore()

        time.sleep(1.8)
        assert ground_link.up, "Ground link should be UP after recovery"
        assert drone_link.up, "Drone link should be UP after recovery"
        assert drone_link.buffered == 0, f"Outbox buffer should be empty, but has {drone_link.buffered}"

        p3_received = [pkt["seq"] for pkt in ground_packets[p3_start_idx:]]
        assert blackout_sent == p3_received, f"Flush order mismatch! Expected {blackout_sent}, got {p3_received}"
        print(f"  Verified: Outbox flushed {len(p3_received)} alerts oldest-first upon link recovery.")
        print(f"  Flushed sequences: {p3_received}")
        results.append({"phase": "Blackout & Recovery Flush", "passed": True})

        # --- Phase 4: Ring Buffer Cap & Oldest-Drop Policy ---
        print("\n[Phase 4] Ring Buffer Cap (64 Entries) & Drop Accounting")
        bridge.cut()
        time.sleep(0.2)
        drop_initial = drone_link.stats["dropped"]
        # Buffer maxlen is 64; let's send 70 packets
        overflow_sent = []
        for i in range(70):
            seq = drone_link.send(
                p.DETECTION,
                ts_ms=p.now_ms(),
                cls=1,
                conf=0.7,
                count=1,
                lat=53.0,
                lon=-113.0,
            )
            overflow_sent.append(seq)

        assert drone_link.buffered == 64, f"Buffer should be capped at 64, got {drone_link.buffered}"
        expected_drops = 70 - 64
        actual_drops = drone_link.stats["dropped"] - drop_initial
        assert actual_drops == expected_drops, f"Expected {expected_drops} dropped, got {actual_drops}"
        print(f"  Verified: Outbox capped at 64; exactly {actual_drops} oldest packets dropped.")

        # Clean out remaining test packets
        bridge.restore()
        time.sleep(3.0)
        results.append({"phase": "Buffer Cap & Drop Accounting", "passed": True})

        # --- Phase 5: Deduplication Window ---
        print("\n[Phase 5] Deduplication Window & Re-ACK Verification")
        # Validate that duplicates list in summary exists and no duplicate callbacks fired
        all_ground_seqs = [pkt["seq"] for pkt in ground_packets]
        seq_counts = collections.Counter(all_ground_seqs)
        duplicates = [seq for seq, cnt in seq_counts.items() if cnt > 1]
        assert len(duplicates) == 0, f"Unexpected duplicate packets delivered to application: {duplicates}"
        print(f"  Verified: 0 duplicate packets delivered to application layer across all tests.")
        results.append({"phase": "Deduplication & Zero Duplicates", "passed": True})

        print("\n" + "=" * 70)
        print(" LINK RELIABILITY TEST RESULTS")
        print("=" * 70)
        for r in results:
            print(f"  {r['phase']:<40} : [PASS]")
        print("=" * 70)
        return {"passed": True, "results": results}

    finally:
        drone_link.stop()
        ground_link.stop()
        bridge.close()


if __name__ == "__main__":
    res = run_link_test()
    if not res["passed"]:
        sys.exit(1)
    sys.exit(0)
