"""UAARG Architecture Adaptation Package for ArcDrone natHacks Defense Showcase.

Adapts telemetry, mission status monitoring, and image/event logging architectural
patterns from UAARG ('emu' and 'shepard').
"""

from scripts.uaarg_adapter.event_logger import EventLogger
from scripts.uaarg_adapter.mission_monitor import MissionMonitor
from scripts.uaarg_adapter.ground_client import GroundClient

__all__ = ["EventLogger", "MissionMonitor", "GroundClient"]
