"""Two-way command state loop and link outage/recovery verification harness.

Verifies:
1. Ground station operator commands (OPERATOR_ACK, HOLD, RESUME, RTB).
2. Stop-and-wait retries and ACK confirmation lifecycle (PENDING -> CONFIRMED).
3. Drone operational state transitions (MODE_PATROL -> MODE_HOLD -> MODE_PATROL -> MODE_RTB).
4. Reliable command execution across simulated radio link loss and recovery.
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
from scripts.uaarg_adapter.mission_monitor import MissionMonitor


class SimulatedDroneAgent:
    """Airborne edge drone agent responding to ground commands and executing state machine."""

    def __init__(self, port: str = "sim://drone_ctrl") -> None:
        self.mode = p.MODE_PATROL
        self.received_commands: List[Dict[str, Any]] = []
        self.events: List[str] = []
        self.acknowledged_alerts: List[int] = []
        self._lock = threading.Lock()

        self.link = Link(
            port=port,
            on_packet=self._on_packet,
            on_event=self._on_event,
            retry_timeout=0.2,
            boot_wait=0.0,
        )

    def start(self) -> SimulatedDroneAgent:
        self.link.mode = self.mode
        self.link.start()
        return self

    def stop(self) -> None:
        self.link.stop()

    def _on_event(self, name: str, **info: Any) -> None:
        with self._lock:
            self.events.append(name)

    def _on_packet(self, packet: Dict[str, Any]) -> None:
        with self._lock:
            ptype = packet.get("type")
            if ptype != p.COMMAND:
                return

            self.received_commands.append(packet)
            cmd = packet.get("cmd")
            ref_seq = packet.get("ref_seq", 0)

            if cmd == p.CMD_OPERATOR_ACK:
                self.acknowledged_alerts.append(ref_seq)
            elif cmd == p.CMD_HOLD:
                self.mode = p.MODE_HOLD
                self.link.mode = p.MODE_HOLD
            elif cmd == p.CMD_RESUME:
                self.mode = p.MODE_PATROL
                self.link.mode = p.MODE_PATROL
            elif cmd == p.CMD_RTB:
                self.mode = p.MODE_RTB
                self.link.mode = p.MODE_RTB

    def send_detection(
        self,
        cls: int = 3,
        conf: float = 0.9,
        lat: float = 53.5232,
        lon: float = -113.5263,
    ) -> int:
        return self.link.send(
            p.DETECTION,
            ts_ms=p.now_ms(),
            cls=cls,
            conf=conf,
            count=1,
            lat=lat,
            lon=lon,
        )


class SimulatedGroundAgent:
    """Ground station agent issuing operator commands and tracking status lifecycle."""

    def __init__(self, port: str = "sim://ground_ctrl") -> None:
        self.received_alerts: List[Dict[str, Any]] = []
        self.events: List[str] = []
        self.pending_commands: Dict[int, Dict[str, Any]] = {}
        self.confirmed_commands: Dict[int, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)

        self.link = Link(
            port=port,
            on_packet=self._on_packet,
            on_event=self._on_event,
            retry_timeout=0.2,
            boot_wait=0.0,
        )

    def start(self) -> SimulatedGroundAgent:
        self.link.start()
        return self

    def stop(self) -> None:
        self.link.stop()

    def _on_event(self, name: str, **info: Any) -> None:
        with self._cond:
            self.events.append(name)
            if name == "delivered":
                seq = info.get("seq")
                if seq in self.pending_commands:
                    cmd_data = self.pending_commands.pop(seq)
                    cmd_data["status"] = "CONFIRMED"
                    cmd_data["rtt_ms"] = info.get("rtt_ms")
                    self.confirmed_commands[seq] = cmd_data
            self._cond.notify_all()

    def _on_packet(self, packet: Dict[str, Any]) -> None:
        with self._cond:
            if packet.get("type") == p.DETECTION:
                self.received_alerts.append(packet)
            self._cond.notify_all()

    def send_command(self, cmd: int, ref_seq: int = 0) -> int:
        """Queue operator command, marking it PENDING."""
        with self._cond:
            seq = self.link.send(p.COMMAND, cmd=cmd, ref_seq=ref_seq)
            self.pending_commands[seq] = {
                "cmd_id": seq,
                "cmd": cmd,
                "cmd_name": p.CMD_NAMES.get(cmd, str(cmd)),
                "ref_seq": ref_seq,
                "status": "PENDING",
            }
            self._cond.notify_all()
        return seq

    def wait_for_confirmed(self, cmd_id: int, timeout: float = 4.0) -> bool:
        deadline = time.monotonic() + timeout
        with self._cond:
            while time.monotonic() < deadline:
                if cmd_id in self.confirmed_commands:
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(timeout=max(remaining, 0.05))
            return cmd_id in self.confirmed_commands


def run_control_loop_tests() -> Dict[str, Any]:
    print("=" * 70)
    print(" ArcDrone Reliable Two-Way Control Loop Verification")
    print("=" * 70)

    bridge = SimLinkBridge("sim://drone_ctrl", "sim://ground_ctrl")
    drone = SimulatedDroneAgent("sim://drone_ctrl").start()
    ground = SimulatedGroundAgent("sim://ground_ctrl").start()
    monitor = MissionMonitor()

    time.sleep(0.3)
    results = []

    try:
        # --- Test 1: OPERATOR_ACK command ---
        print("\n[Test 1] Alert Transmission and Operator Acknowledge (CMD_OPERATOR_ACK)")
        alert_seq = drone.send_detection(cls=3, conf=0.92)
        time.sleep(0.3)
        assert len(ground.received_alerts) >= 1, "Ground failed to receive detection alert"
        print(f"  Drone sent alert #{alert_seq}, ground received #{ground.received_alerts[-1]['seq']}")

        ack_cmd_id = ground.send_command(p.CMD_OPERATOR_ACK, ref_seq=alert_seq)
        confirmed = ground.wait_for_confirmed(ack_cmd_id, timeout=3.0)
        assert confirmed, f"Command #{ack_cmd_id} (OPERATOR_ACK) was not confirmed!"
        assert alert_seq in drone.acknowledged_alerts, "Drone did not record alert acknowledgement"
        print(f"  Command #{ack_cmd_id} (OPERATOR_ACK) -> CONFIRMED (Drone ACK banner active)")
        results.append({"test": "OPERATOR_ACK", "status": "CONFIRMED", "passed": True})

        # --- Test 2: HOLD Command and State Transition ---
        print("\n[Test 2] Tactical Override HOLD (CMD_HOLD)")
        hold_cmd_id = ground.send_command(p.CMD_HOLD, ref_seq=0)
        confirmed = ground.wait_for_confirmed(hold_cmd_id, timeout=3.0)
        assert confirmed, f"Command #{hold_cmd_id} (HOLD) was not confirmed!"
        assert drone.mode == p.MODE_HOLD, f"Drone mode expected {p.MODE_HOLD}, got {drone.mode}"
        assert drone.link.mode == p.MODE_HOLD, "Drone link heartbeat mode did not update"
        print(f"  Command #{hold_cmd_id} (HOLD) -> CONFIRMED | Drone mode: MODE_HOLD")
        results.append({"test": "CMD_HOLD", "status": "CONFIRMED", "passed": True})

        # --- Test 3: RESUME Command and State Transition ---
        print("\n[Test 3] Tactical Override RESUME (CMD_RESUME)")
        resume_cmd_id = ground.send_command(p.CMD_RESUME, ref_seq=0)
        confirmed = ground.wait_for_confirmed(resume_cmd_id, timeout=3.0)
        assert confirmed, f"Command #{resume_cmd_id} (RESUME) was not confirmed!"
        assert drone.mode == p.MODE_PATROL, f"Drone mode expected {p.MODE_PATROL}, got {drone.mode}"
        print(f"  Command #{resume_cmd_id} (RESUME) -> CONFIRMED | Drone mode: MODE_PATROL")
        results.append({"test": "CMD_RESUME", "status": "CONFIRMED", "passed": True})

        # --- Test 4: RTB Command and State Transition ---
        print("\n[Test 4] Tactical Override RTB (CMD_RTB)")
        rtb_cmd_id = ground.send_command(p.CMD_RTB, ref_seq=0)
        confirmed = ground.wait_for_confirmed(rtb_cmd_id, timeout=3.0)
        assert confirmed, f"Command #{rtb_cmd_id} (RTB) was not confirmed!"
        assert drone.mode == p.MODE_RTB, f"Drone mode expected {p.MODE_RTB}, got {drone.mode}"
        print(f"  Command #{rtb_cmd_id} (RTB) -> CONFIRMED | Drone mode: MODE_RTB")
        results.append({"test": "CMD_RTB", "status": "CONFIRMED", "passed": True})

        # --- Test 5: Command Dispatched During Link Blackout & Outage Recovery ---
        print("\n[Test 5] Command Dispatch Across Radio Blackout & Recovery")
        print("  Cutting simulated radio bridge (blackout)...")
        bridge.cut()
        time.sleep(3.2)  # Wait for 3.0s LINK_TIMEOUT
        assert not ground.link.up, "Ground station link should be down after 3.0s"
        print("  Ground link state: DOWN (verified 3.0s silence timeout)")

        # Issue command during blackout
        outage_cmd_id = ground.send_command(p.CMD_HOLD, ref_seq=999)
        time.sleep(0.5)
        # Verify command remains PENDING while link is down
        assert outage_cmd_id in ground.pending_commands, "Command should remain PENDING during blackout"
        assert outage_cmd_id not in ground.confirmed_commands, "Command cannot be confirmed while link is cut"
        print(f"  Command #{outage_cmd_id} remains PENDING during blackout (buffered in outbox)")

        # Restore radio bridge
        print("  Restoring simulated radio bridge...")
        bridge.restore()
        confirmed = ground.wait_for_confirmed(outage_cmd_id, timeout=4.0)
        assert confirmed, f"Command #{outage_cmd_id} was not confirmed upon link recovery!"
        assert drone.mode == p.MODE_HOLD, "Drone failed to transition to MODE_HOLD after recovery"
        print(f"  Command #{outage_cmd_id} successfully delivered upon recovery -> CONFIRMED")
        print(f"  Drone state transitioned to MODE_HOLD upon recovery (verified)")
        results.append({"test": "OUTAGE_COMMAND_RECOVERY", "status": "CONFIRMED", "passed": True})

        print("\n" + "=" * 70)
        print(" CONTROL LOOP VERIFICATION SUMMARY")
        print("=" * 70)
        for r in results:
            print(f"  {r['test']:<30} : {r['status']:<12} [PASS]")
        print("=" * 70)
        return {"passed": True, "tests": results}

    finally:
        drone.stop()
        ground.stop()
        bridge.close()


if __name__ == "__main__":
    res = run_control_loop_tests()
    if not res["passed"]:
        sys.exit(1)
    sys.exit(0)
