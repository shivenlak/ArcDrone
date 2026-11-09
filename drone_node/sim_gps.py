"""Simulated GPS generator for VisDrone aerial footage."""
import math
from typing import Tuple


class SimGps:
    def __init__(
        self,
        base_lat: float = 53.5232,
        base_lon: float = -113.5263,
        speed_mps: float = 8.0,
        heading_deg: float = 45.0,
    ) -> None:
        self.base_lat = base_lat
        self.base_lon = base_lon
        self.speed_mps = speed_mps
        self.heading_rad = math.radians(heading_deg)

        # Approximate conversion near ~53.5° N
        self.meters_per_deg_lat = 111320.0
        self.meters_per_deg_lon = 111320.0 * math.cos(math.radians(base_lat))

    def position_at_frame(self, frame_idx: int, fps: float = 1.0) -> Tuple[float, float]:
        """Deterministic bounded figure-eight patrol so pins stay on the operating grid.

        A straight constant-heading flight marches the drone (and every detection
        offset) off the map in under a minute. A closed Lissajous loop keeps the
        track inside a fixed box while still looking like continuous flight.
        """
        t_sec = frame_idx / max(0.1, fps)
        # ~90 s to traverse the full loop; radius ~120 m east / 80 m north.
        period = 90.0
        theta = (2.0 * math.pi * t_sec) / period
        radius_east_m = 120.0
        radius_north_m = 80.0

        d_east = radius_east_m * math.sin(theta)
        d_north = radius_north_m * math.sin(2.0 * theta)  # figure-eight

        lat = self.base_lat + (d_north / self.meters_per_deg_lat)
        lon = self.base_lon + (d_east / self.meters_per_deg_lon)
        return round(lat, 7), round(lon, 7)

    def target_location(
        self,
        drone_lat: float,
        drone_lon: float,
        bbox: Tuple[int, int, int, int],
        frame_shape: Tuple[int, int],
        ground_fov_m: float = 60.0,
    ) -> Tuple[float, float]:
        """
        Offsets target lat/lon from the drone's position using bounding box center.
        bbox: (x1, y1, x2, y2)
        frame_shape: (height, width)
        """
        h, w = frame_shape[:2]
        if w == 0 or h == 0:
            return drone_lat, drone_lon

        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        # Normalized offset from center: -0.5 to +0.5
        norm_x = (cx - (w / 2.0)) / float(w)
        norm_y = ((h / 2.0) - cy) / float(h)  # Up is positive North

        meters_east = norm_x * ground_fov_m
        meters_north = norm_y * ground_fov_m

        t_lat = drone_lat + (meters_north / self.meters_per_deg_lat)
        t_lon = drone_lon + (meters_east / self.meters_per_deg_lon)

        return round(t_lat, 7), round(t_lon, 7)