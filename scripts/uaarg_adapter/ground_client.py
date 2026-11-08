"""Non-invasive Ground Station WebSocket Observer Client.

Connects to Ground Station WebSocket (ws://localhost:8000/ws or in-process Station)
to capture alerts, monitor telemetry, and inject operator commands without modifying
ground station source code.
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from scripts.uaarg_adapter.event_logger import EventLogger
from scripts.uaarg_adapter.mission_monitor import MissionMonitor


class GroundClient:
    """Ground station WebSocket observer client."""

    def __init__(
        self,
        ws_url: str = "ws://localhost:8000/ws",
        event_logger: Optional[EventLogger] = None,
        mission_monitor: Optional[MissionMonitor] = None,
    ) -> None:
        self.ws_url = ws_url
        self.event_logger = event_logger
        self.mission_monitor = mission_monitor

        self.alerts: List[Dict[str, Any]] = []
        self.drone_status: Dict[str, Any] = {"mode": "PATROL", "buffered": 0}
        self.link_up: bool = False
        self.commands: Dict[int, Dict[str, Any]] = {}  # cmd_id -> cmd dict
        self.stats: Dict[str, Any] = {}

        self._ws: Any = None
        self._station: Any = None  # In-process Station instance if used
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def attach_station(self, station: Any) -> None:
        """Attach directly to an in-process ground station instance."""
        self._station = station

    def handle_incoming_json(self, raw_text: str) -> None:
        """Handle raw JSON text received from WebSocket or Station."""
        try:
            msg = json.loads(raw_text)
        except Exception:
            return

        with self._cond:
            ev = msg.get("event")
            if ev == "alert":
                self.alerts.append(msg)
                if self.event_logger:
                    self.event_logger.log_event(
                        seq=msg.get("seq", 0),
                        detection={
                            "cls": msg.get("cls"),
                            "conf": msg.get("conf"),
                            "count": msg.get("count"),
                        },
                        location={
                            "lat": msg.get("lat", 0.0),
                            "lon": msg.get("lon", 0.0),
                        },
                        timestamp_ms=msg.get("ts_ms"),
                        mode=self.drone_status.get("mode", "PATROL"),
                        link_status={"up": self.link_up, "buffered": self.drone_status.get("buffered", 0)},
                    )
                if self.mission_monitor:
                    self.mission_monitor.record_message(seq=msg.get("seq"))
                    self.mission_monitor.send_log(
                        f"Alert received: #{msg.get('seq')} {msg.get('cls')} (conf={msg.get('conf')})",
                        severity="normal",
                        subsystem="ARCDRONE_GROUND",
                    )

            elif ev == "drone":
                self.drone_status["mode"] = msg.get("mode", "PATROL")
                self.drone_status["buffered"] = msg.get("buffered", 0)
                if self.mission_monitor:
                    self.mission_monitor.record_message(
                        mode=msg.get("mode"),
                        buffered=msg.get("buffered"),
                    )

            elif ev == "link":
                self.link_up = bool(msg.get("up"))
                if self.mission_monitor:
                    if self.link_up:
                        self.mission_monitor.mark_link_up()
                    else:
                        self.mission_monitor.mark_link_down()

            elif ev == "cmd_status":
                cmd_id = msg.get("cmd_id")
                if cmd_id is not None:
                    self.commands[cmd_id] = msg
                    if self.mission_monitor:
                        self.mission_monitor.send_log(
                            f"Command #{cmd_id} ({msg.get('cmd')}) status: {msg.get('status')}",
                            severity="normal" if msg.get("status") == "CONFIRMED" else "warning",
                            subsystem="ARCDRONE_GROUND",
                        )

            elif ev == "stats":
                self.stats = msg

            self._cond.notify_all()

    async def send_command_async(self, cmd: str, ref_seq: int = 0) -> Optional[int]:
        """Dispatch operator command asynchronously."""
        payload = {"action": "command", "cmd": cmd, "ref_seq": int(ref_seq)}
        if self._station is not None:
            await self._station.handle_browser(payload)
            # Find the latest pending command id
            with self._cond:
                for cid, cdata in reversed(list(self.commands.items())):
                    if cdata.get("cmd") == cmd and cdata.get("status") == "PENDING":
                        return cid
            return None
        elif self._ws is not None:
            await self._ws.send(json.dumps(payload))
            return None
        return None

    def send_command(self, cmd: str, ref_seq: int = 0) -> Optional[int]:
        """Dispatch operator command synchronously."""
        if self._station is not None:
            loop = getattr(self._station, "loop", None)
            if loop is not None and loop.is_running():
                future = asyncio.run_coroutine_threadsafe(
                    self.send_command_async(cmd, ref_seq), loop
                )
                return future.result(timeout=2.0)
            else:
                return asyncio.run(self.send_command_async(cmd, ref_seq))
        return None

    def wait_for_command_status(
        self,
        cmd_id: int,
        status: str = "CONFIRMED",
        timeout: float = 5.0,
    ) -> bool:
        """Block until specified command reaches target status."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while time.monotonic() < deadline:
                cmd = self.commands.get(cmd_id)
                if cmd and cmd.get("status") == status:
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(timeout=max(remaining, 0.05))
            cmd = self.commands.get(cmd_id)
            return bool(cmd and cmd.get("status") == status)

    def wait_for_alert_count(self, count: int, timeout: float = 5.0) -> bool:
        """Block until at least `count` alerts have been received."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while len(self.alerts) < count and time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(timeout=max(remaining, 0.05))
            return len(self.alerts) >= count
