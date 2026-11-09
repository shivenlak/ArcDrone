"""UAARG Telemetry & Mission Status Monitor for ArcDrone.

Adapts UAARG emu liveness timer (timeSinceMessage), 3-tier severity logs
(normal, warning, error), and subsystem tagging (ARCDRONE_EDGE, ARCDRONE_GROUND, ARCDRONE_LINK).
"""

from __future__ import annotations

from datetime import datetime, timezone
import threading
import time
from typing import Any, Callable, Dict, List, Optional

SEVERITY_LEVELS = ("normal", "warning", "error")
SUBSYSTEMS = ("ARCDRONE_EDGE", "ARCDRONE_GROUND", "ARCDRONE_LINK")


class MissionMonitor:
    """Manages telemetry status indicators, liveness timing, and categorized logging."""

    def __init__(self, heartbeat_timeout: float = 3.0) -> None:
        self.heartbeat_timeout = float(heartbeat_timeout)
        self._lock = threading.RLock()

        self._last_message_time = time.monotonic()
        self._mode: str = "PATROL"
        self._link_up: bool = False
        self._buffered_count: int = 0
        self._total_messages: int = 0

        self._logs: List[Dict[str, Any]] = []
        self._subscribers: List[Callable[[Dict[str, Any]], None]] = []

    @property
    def time_since_message(self) -> int:
        """Elapsed seconds since last incoming telemetry message (UAARG emu convention)."""
        with self._lock:
            elapsed = time.monotonic() - self._last_message_time
            return max(0, int(elapsed))

    def record_message(
        self,
        ptype: Optional[int] = None,
        seq: Optional[int] = None,
        mode: Optional[str] = None,
        buffered: Optional[int] = None,
    ) -> None:
        """Reset liveness timer upon valid packet receipt and update status."""
        now = time.monotonic()
        with self._lock:
            self._last_message_time = now
            self._link_up = True
            self._total_messages += 1
            if mode is not None:
                self._mode = str(mode).upper()
            if buffered is not None:
                self._buffered_count = int(buffered)

    def mark_link_down(self) -> None:
        """Mark communication link as down."""
        with self._lock:
            self._link_up = False
        self.send_log(
            "Primary communication link lost (3.0s timeout)",
            severity="warning",
            subsystem="ARCDRONE_LINK",
        )

    def mark_link_up(self) -> None:
        """Mark communication link as active."""
        with self._lock:
            self._link_up = True
            self._last_message_time = time.monotonic()
        self.send_log(
            "Primary communication link established / restored",
            severity="normal",
            subsystem="ARCDRONE_LINK",
        )

    def update_mode(self, mode: str) -> None:
        """Update operational mode."""
        with self._lock:
            old_mode = self._mode
            self._mode = str(mode).upper()
        self.send_log(
            f"Mode transition: {old_mode} -> {self._mode}",
            severity="normal",
            subsystem="ARCDRONE_EDGE",
        )

    def is_alive(self, timeout_sec: Optional[float] = None) -> bool:
        """Check if telemetry stream is active within timeout threshold."""
        limit = self.heartbeat_timeout if timeout_sec is None else timeout_sec
        return self.time_since_message < limit and self._link_up

    def send_log(
        self,
        message: str,
        severity: str = "normal",
        subsystem: str = "ARCDRONE_LINK",
    ) -> Dict[str, Any]:
        """Emit a structured, severity-categorized log entry matching UAARG emu standard."""
        sev = severity.lower()
        if sev not in SEVERITY_LEVELS:
            sev = "normal"

        sub = subsystem.upper()
        iso_ts = datetime.now(timezone.utc).isoformat()
        mono_ts = time.monotonic()

        formatted_msg = f"[{sub}] {message}"
        entry = {
            "timestamp": mono_ts,
            "timestamp_iso": iso_ts,
            "severity": sev,
            "subsystem": sub,
            "message": message,
            "formatted": formatted_msg,
        }

        with self._lock:
            self._logs.append(entry)
            subs = list(self._subscribers)

        for callback in subs:
            try:
                callback(entry)
            except Exception:
                pass

        return entry

    def subscribe_log(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Register a callback for log stream."""
        with self._lock:
            self._subscribers.append(callback)

    def get_status(self) -> Dict[str, Any]:
        """Get status matching UAARG emu UAVStatus card schema."""
        with self._lock:
            alive = (time.monotonic() - self._last_message_time) < self.heartbeat_timeout and self._link_up
            return {
                "connection": "yes" if self._link_up else "no",
                "alive": alive,
                "mode": self._mode,
                "timeSinceMessage": self.time_since_message,
                "bufferedCount": self._buffered_count,
                "totalMessages": self._total_messages,
            }

    def get_logs(
        self,
        severity: Optional[str] = None,
        subsystem: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Filter logged entries by severity or subsystem."""
        with self._lock:
            filtered = list(self._logs)

        if severity:
            sev_target = severity.lower()
            filtered = [e for e in filtered if e["severity"] == sev_target]
        if subsystem:
            sub_target = subsystem.upper()
            filtered = [e for e in filtered if e["subsystem"] == sub_target]

        return filtered
