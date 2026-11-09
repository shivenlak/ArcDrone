"""Real-time sliding window data rate auditor for ArcDrone primary link.

Enforces natHacks defense showcase constraint:
Primary communication link data rate for detection and bounding box telemetry
remains within the 3 kB/s (3000 B/s) bandwidth budget during active transmission.
"""

from __future__ import annotations

import collections
import json
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from protocol import packets as p

BUDGET_BYTES_PER_SEC = 3000.0  # 3 kB/s primary link ceiling


class BandwidthRecord:
    __slots__ = ("timestamp", "wire_bytes", "serial_bytes", "ptype", "sender")

    def __init__(
        self,
        timestamp: float,
        wire_bytes: int,
        serial_bytes: int,
        ptype: Optional[int] = None,
        sender: str = "",
    ) -> None:
        self.timestamp = timestamp
        self.wire_bytes = wire_bytes
        self.serial_bytes = serial_bytes
        self.ptype = ptype
        self.sender = sender


class BandwidthMonitor:
    """Sliding-window byte rate auditor tracking wire bytes and serial hex framing."""

    def __init__(
        self,
        budget_limit: float = BUDGET_BYTES_PER_SEC,
        window_size_sec: float = 1.0,
    ) -> None:
        self.budget_limit = float(budget_limit)
        self.window_size_sec = float(window_size_sec)
        self._history: collections.deque[BandwidthRecord] = collections.deque()
        self._lock = threading.Lock()

        self._peak_wire_bps = 0.0
        self._peak_serial_bps = 0.0
        self._total_wire_bytes = 0
        self._total_serial_bytes = 0
        self._violations: List[Dict[str, Any]] = []

    def record_tx(
        self,
        wire_bytes: int,
        serial_bytes: Optional[int] = None,
        timestamp: Optional[float] = None,
        ptype: Optional[int] = None,
        sender: str = "",
    ) -> None:
        """Record a transmission event."""
        now = time.monotonic() if timestamp is None else timestamp
        if serial_bytes is None:
            # ArcDrone serial line is hex string + \n: 2 hex chars per wire byte + 1 byte \n
            serial_bytes = (wire_bytes * 2) + 1 if wire_bytes > 0 else 0

        with self._lock:
            rec = BandwidthRecord(
                timestamp=now,
                wire_bytes=wire_bytes,
                serial_bytes=serial_bytes,
                ptype=ptype,
                sender=sender,
            )
            self._history.append(rec)
            self._total_wire_bytes += wire_bytes
            self._total_serial_bytes += serial_bytes

            self._trim(now)
            wire_bps, serial_bps = self._calc_rates(now)

            if wire_bps > self._peak_wire_bps:
                self._peak_wire_bps = wire_bps
            if serial_bps > self._peak_serial_bps:
                self._peak_serial_bps = serial_bps

            # Note: The physical radio budget applies to both wire payload and framed transmission
            max_rate = max(wire_bps, serial_bps)
            if max_rate > self.budget_limit:
                violation = {
                    "timestamp": now,
                    "wire_bps": round(wire_bps, 2),
                    "serial_bps": round(serial_bps, 2),
                    "budget": self.budget_limit,
                    "sender": sender,
                }
                self._violations.append(violation)

    def record_raw_bytes(self, raw_data: bytes, sender: str = "") -> None:
        """Parse transmitted serial data or raw packet and record byte counts."""
        now = time.monotonic()
        if not raw_data:
            return

        # If data is already serial hex lines (ending with \n)
        if b"\n" in raw_data:
            lines = raw_data.split(b"\n")
            for line in lines:
                if not line:
                    continue
                serial_len = len(line) + 1
                wire_pkt = p.from_line(line.decode(errors="replace"))
                wire_len = len(wire_pkt) if wire_pkt is not None else len(line) // 2
                decoded = p.decode(wire_pkt) if wire_pkt else None
                ptype = decoded["type"] if decoded else None
                self.record_tx(
                    wire_bytes=wire_len,
                    serial_bytes=serial_len,
                    timestamp=now,
                    ptype=ptype,
                    sender=sender,
                )
        else:
            # Raw binary wire packet
            decoded = p.decode(raw_data)
            ptype = decoded["type"] if decoded else None
            serial_len = len(p.to_line(raw_data).encode())
            self.record_tx(
                wire_bytes=len(raw_data),
                serial_bytes=serial_len,
                timestamp=now,
                ptype=ptype,
                sender=sender,
            )

    def attach_to_bridge(self, bridge: Any) -> None:
        """Attach listener hook to a SimLinkBridge instance."""
        def bridge_hook(sender_port: Any, data: bytes) -> None:
            sender_name = getattr(sender_port, "name", str(sender_port))
            self.record_raw_bytes(data, sender=sender_name)

        bridge.add_tx_hook(bridge_hook)

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_size_sec
        while self._history and self._history[0].timestamp < cutoff:
            self._history.popleft()

    def _calc_rates(self, now: float) -> Tuple[float, float]:
        cutoff = now - self.window_size_sec
        wire_sum = sum(rec.wire_bytes for rec in self._history if rec.timestamp >= cutoff)
        serial_sum = sum(rec.serial_bytes for rec in self._history if rec.timestamp >= cutoff)
        # Rates over the sliding window
        window = max(self.window_size_sec, 0.001)
        return (wire_sum / window, serial_sum / window)

    def current_rate(self) -> Dict[str, Any]:
        """Compute current sliding window data rates."""
        now = time.monotonic()
        with self._lock:
            self._trim(now)
            wire_bps, serial_bps = self._calc_rates(now)
            max_bps = max(wire_bps, serial_bps)
            return {
                "wire_bytes_per_sec": round(wire_bps, 2),
                "serial_bytes_per_sec": round(serial_bps, 2),
                "max_bytes_per_sec": round(max_bps, 2),
                "budget_limit": self.budget_limit,
                "within_budget": max_bps <= self.budget_limit,
                "utilization_pct": round((max_bps / self.budget_limit) * 100.0, 2),
            }

    def check_budget(self) -> Tuple[bool, float]:
        """Verify if data rate is currently within budget. Returns (within_budget, bytes_per_sec)."""
        rate = self.current_rate()
        return rate["within_budget"], rate["max_bytes_per_sec"]

    def assert_within_budget(self) -> None:
        """Assert that no violations were detected and peak rate is within budget."""
        with self._lock:
            peak = max(self._peak_wire_bps, self._peak_serial_bps)
            if self._violations or peak > self.budget_limit:
                msg = (
                    f"Bandwidth budget exceeded! Limit: {self.budget_limit} B/s, "
                    f"Peak: {peak:.1f} B/s (Wire: {self._peak_wire_bps:.1f} B/s, "
                    f"Serial: {self._peak_serial_bps:.1f} B/s). "
                    f"Violations count: {len(self._violations)}"
                )
                raise AssertionError(msg)

    def summary(self) -> Dict[str, Any]:
        """Full statistics report."""
        now = time.monotonic()
        with self._lock:
            self._trim(now)
            curr_wire, curr_serial = self._calc_rates(now)
            peak_max = max(self._peak_wire_bps, self._peak_serial_bps)
            return {
                "budget_limit_bps": self.budget_limit,
                "within_budget": peak_max <= self.budget_limit and len(self._violations) == 0,
                "peak_wire_bps": round(self._peak_wire_bps, 2),
                "peak_serial_bps": round(self._peak_serial_bps, 2),
                "peak_max_bps": round(peak_max, 2),
                "current_wire_bps": round(curr_wire, 2),
                "current_serial_bps": round(curr_serial, 2),
                "utilization_peak_pct": round((peak_max / self.budget_limit) * 100.0, 2),
                "total_wire_bytes": self._total_wire_bytes,
                "total_serial_bytes": self._total_serial_bytes,
                "violations_count": len(self._violations),
                "violations": list(self._violations),
            }

    def export_report(self, filepath: str) -> None:
        """Write summary JSON report to disk."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(self.summary(), f, indent=2)
