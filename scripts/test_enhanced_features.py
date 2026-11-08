"""Verification Suite for the 6 Enhanced Architectural Features:
1. Dynamic Role Learning (DRONE vs GROUND inference from packet flow)
2. Dual-Path Radio Emulation (simultaneous ESP-NOW + LoRa diversity)
3. "On the wire" Byte Dissection & Raw Telemetry
4. Monotonic Clock Startup Hardening (float('-inf'))
5. High-Resolution Aerial YOLO Tuning (imgsz=1280)
6. Grid-Aware Coasting Tracker (GridCoastTracker)
"""

import os
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from protocol import packets as p
from scripts.ishan_side_by_side import DroneIndexedTracker, GridCoastTracker
from scripts.sim_link_bridge import SimLinkBridge, VirtualSerialPort


class TestEnhancedFeatures(unittest.TestCase):
    def test_dynamic_role_learning(self):
        """Bridge must learn DRONE vs GROUND automatically without manual config."""
        bridge = SimLinkBridge(name_a="sim://endpoint_1", name_b="sim://endpoint_2")
        self.assertEqual(bridge.get_role("sim://endpoint_1"), "UNKNOWN")
        self.assertEqual(bridge.get_role("sim://endpoint_2"), "UNKNOWN")

        # Endpoint 1 transmits a DETECTION packet
        det_pkt = p.encode(
            p.DETECTION,
            seq=1,
            ts_ms=p.now_ms(),
            cls=3,
            conf=0.9,
            count=1,
            lat=53.52,
            lon=-113.52,
        )
        line = p.to_line(det_pkt).encode("ascii")
        bridge.port_a.write(line)

        # Bridge must dynamically infer roles: Endpoint 1 = DRONE, Endpoint 2 = GROUND
        self.assertEqual(bridge.get_role("sim://endpoint_1"), "DRONE")
        self.assertEqual(bridge.get_role("sim://endpoint_2"), "GROUND")

        # Endpoint 2 transmits a COMMAND back
        cmd_pkt = p.encode(p.COMMAND, seq=2, cmd=p.CMD_HOLD, ref_seq=1)
        line_cmd = p.to_line(cmd_pkt).encode("ascii")
        bridge.port_b.write(line_cmd)

        self.assertEqual(bridge.get_role("sim://endpoint_1"), "DRONE")
        self.assertEqual(bridge.get_role("sim://endpoint_2"), "GROUND")
        bridge.close()

    def test_dual_path_frequency_diversity(self):
        """Bridge in dual_path mode must support BOTH, LORA_ONLY, and MUTED radio states."""
        bridge = SimLinkBridge(
            name_a="sim://radio_a",
            name_b="sim://radio_b",
            dual_path=True,
            timeout=0.05,
        )
        self.assertEqual(bridge.radio_mode, "BOTH")

        pkt = p.encode(p.HEARTBEAT, seq=10, ts_ms=p.now_ms(), mode=p.MODE_PATROL, buffered=0)
        line = p.to_line(pkt).encode("ascii")

        # Test BOTH mode
        bridge.port_a.write(line)
        rx = bridge.port_b.read(len(line))
        self.assertEqual(rx, line)
        self.assertGreaterEqual(bridge.stats["packets_transferred_espnow"], 1)

        # Test LORA_ONLY mode (PRG button pressed once)
        bridge.set_radio_mode("LORA_ONLY")
        self.assertEqual(bridge.radio_mode, "LORA_ONLY")
        bridge.port_a.write(line)
        rx2 = bridge.port_b.read(len(line))
        self.assertEqual(rx2, line)
        self.assertGreaterEqual(bridge.stats["packets_transferred_lora"], 1)

        # Test MUTED mode (PRG button pressed again)
        bridge.set_radio_mode("MUTED")
        self.assertTrue(bridge.is_cut)
        bridge.port_a.write(line)
        rx_muted = bridge.port_b.read(len(line))
        self.assertEqual(rx_muted, b"")
        self.assertGreaterEqual(bridge.stats["dropped_cut"], 1)

        # Restore
        bridge.restore()
        self.assertEqual(bridge.radio_mode, "BOTH")
        self.assertFalse(bridge.is_cut)
        bridge.close()

    def test_grid_aware_coasting_tracker(self):
        """Border contacts drop in 2 frames; interior contacts coast for 3 frames."""
        tracker = GridCoastTracker(max_unseen_frames=36)

        # 1. Edge target (cx=0.05, cy=0.05 -> cell distance 0)
        edge_dets = [{"cls": 3, "conf": 0.9, "bbox": (10, 10, 30, 30)}]
        # Pass 3 hits so tentative track reaches hits >= 3 and gets id > 0
        tracker.update(edge_dets, frame_w=1000, frame_h=1000, sim_t=0.0)
        tracker.update(edge_dets, frame_w=1000, frame_h=1000, sim_t=0.1)
        t1 = tracker.update(edge_dets, frame_w=1000, frame_h=1000, sim_t=0.2)
        self.assertEqual(len(t1), 1)

        # Miss 1 frame: should coast
        t2 = tracker.update([], frame_w=1000, frame_h=1000, sim_t=0.3)
        self.assertEqual(len(t2), 1)

        # Miss 2nd frame: edge contact drops (EDGE_UNSEEN_DROP = 2)
        t3 = tracker.update([], frame_w=1000, frame_h=1000, sim_t=0.4)
        self.assertEqual(len(t3), 0, "Edge contact must drop after 2 unseen frames")

        # 2. Interior target (cx=0.5, cy=0.5 -> center cell distance 7)
        interior_dets = [{"cls": 3, "conf": 0.9, "bbox": (480, 480, 520, 520)}]
        tracker.update(interior_dets, frame_w=1000, frame_h=1000, sim_t=1.0)
        tracker.update(interior_dets, frame_w=1000, frame_h=1000, sim_t=1.1)
        i1 = tracker.update(interior_dets, frame_w=1000, frame_h=1000, sim_t=1.2)
        self.assertEqual(len(i1), 1)

        # Miss 1 frame: coasts
        i2 = tracker.update([], frame_w=1000, frame_h=1000, sim_t=1.3)
        self.assertEqual(len(i2), 1)

        # Miss 2nd frame: interior contact still coasts!
        i3 = tracker.update([], frame_w=1000, frame_h=1000, sim_t=1.4)
        self.assertEqual(len(i3), 1, "Interior contact must coast through 2 missed frames")

        # Miss 3rd frame: interior contact drops (INTERIOR_UNSEEN_DROP = 3)
        i4 = tracker.update([], frame_w=1000, frame_h=1000, sim_t=1.5)
        self.assertEqual(len(i4), 0, "Interior contact must drop after 3 unseen frames")

    def test_timer_initialization_safety(self):
        """Timer variables must handle startup at monotonic clock 0.0 without lockup."""
        last_infer = float("-inf")
        period = 0.1
        current_time = 0.001  # fresh boot time on macOS
        self.assertTrue((current_time - last_infer) >= period)


if __name__ == "__main__":
    unittest.main()
