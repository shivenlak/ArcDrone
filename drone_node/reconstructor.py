"""Ground Station <3 kbps Tactical Scene Twin & Live Object Manifest."""
import argparse
import math
import socket
import time
from collections import deque
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np

from drone_node.scene_codec import (
    LINK_MAX_KBPS,
    SCENE_MAX_KBPS,
    LoRaLinkBudget,
    SceneCodec,
)

# Wire class id 1-10 (see scene_codec.LABEL_TO_CLS); 0 = legacy packets
CLASS_COLORS: Dict[int, Tuple[int, int, int]] = {
    0: (0, 255, 128),
    1: (0, 255, 128),   # pedestrian
    2: (0, 220, 255),   # people
    3: (255, 200, 0),   # bicycle
    4: (255, 140, 40),  # car
    5: (255, 80, 180),  # van
    6: (40, 100, 255),  # truck
    7: (200, 120, 255), # tricycle
    8: (255, 180, 120), # awning-tricycle
    9: (0, 165, 255),   # bus
    10: (180, 255, 0),  # motor
}

PANE_W, PANE_H = 720, 540
BG_FOV = (14, 18, 14)
BG_PANEL = (18, 18, 22)
ACCENT = (0, 220, 255)
LIVE_OK = (0, 255, 120)
LIVE_STALE = (80, 80, 90)

SECTOR_NAMES = [
    ["TOP-LEFT", "TOP-CENTER", "TOP-RIGHT"],
    ["MID-LEFT", "CENTER", "MID-RIGHT"],
    ["BOTTOM-LEFT", "BOTTOM-CENTER", "BOTTOM-RIGHT"],
]


def get_sector_name(cx: int, cy: int, w: int, h: int) -> Tuple[str, int, int]:
    col = min(2, max(0, int((cx / max(1, w)) * 3)))
    row = min(2, max(0, int((cy / max(1, h)) * 3)))
    return SECTOR_NAMES[row][col], row, col


def get_compass_dir(dx: float, dy: float) -> str:
    if math.hypot(dx, dy) < 4.0:
        return "STATIONARY"
    angle = math.degrees(math.atan2(-dy, dx)) % 360
    dirs = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
    idx = int((angle + 22.5) // 45) % 8
    return f"MOVING {dirs[idx]}"


class ObjectTracker:
    """Tracks object IDs, breadcrumb trails, and velocity vectors across frames."""

    def __init__(self, max_len: int = 25, max_dist: float = 80.0) -> None:
        self.next_id = 1
        self.tracks: Dict[int, Dict[str, Any]] = {}
        self.max_len = max_len
        self.max_dist = max_dist

    def update(self, objects: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        now = time.time()
        used_tracks = set()

        for obj in objects:
            cx, cy = obj["center"]
            cls_id = obj["cls"]
            best_id = None
            best_dist = self.max_dist

            for tid, trk in self.tracks.items():
                if tid in used_tracks or trk["cls"] != cls_id:
                    continue
                lx, ly = trk["pts"][-1]
                dist = math.hypot(cx - lx, cy - ly)
                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is None:
                best_id = self.next_id
                self.next_id += 1
                self.tracks[best_id] = {
                    "cls": cls_id,
                    "pts": deque(maxlen=self.max_len),
                    "last_seen": now,
                }

            used_tracks.add(best_id)
            pts = self.tracks[best_id]["pts"]
            pts.append((cx, cy))
            self.tracks[best_id]["last_seen"] = now

            if len(pts) >= 3:
                dx = pts[-1][0] - pts[-3][0]
                dy = pts[-1][1] - pts[-3][1]
            elif len(pts) >= 2:
                dx = pts[-1][0] - pts[0][0]
                dy = pts[-1][1] - pts[0][1]
            else:
                dx, dy = 0.0, 0.0

            obj["track_id"] = best_id
            obj["trail"] = list(pts)
            obj["vel"] = (dx, dy)
            obj["motion_str"] = get_compass_dir(dx, dy)

        stale = [tid for tid, trk in self.tracks.items() if now - trk["last_seen"] > 3.5]
        for tid in stale:
            del self.tracks[tid]

        return objects


def _class_color(cls_id: int) -> Tuple[int, int, int]:
    return CLASS_COLORS.get(cls_id, (0, 255, 255))


def draw_text_bg(
    canvas: np.ndarray,
    text: str,
    org: Tuple[int, int],
    font_scale: float,
    color: Tuple[int, int, int],
    thickness: int = 1,
    pad: int = 4,
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = org
    x1, y1 = x - pad, y - th - pad
    x2, y2 = x + tw + pad, y + baseline + pad
    cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 1)
    cv2.putText(canvas, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def tint_occupied_sectors(
    canvas: np.ndarray, sector_counts: List[List[int]]
) -> None:
    h, w = canvas.shape[:2]
    overlay = canvas.copy()
    for r in range(3):
        for c in range(3):
            if sector_counts[r][c] <= 0:
                continue
            x1 = (w * c) // 3
            y1 = (h * r) // 3
            x2 = (w * (c + 1)) // 3
            y2 = (h * (r + 1)) // 3
            alpha = min(0.35, 0.12 + 0.06 * sector_counts[r][c])
            cv2.rectangle(overlay, (x1 + 1, y1 + 1), (x2 - 1, y2 - 1), (40, 120, 60), -1)
            cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0, canvas)


def draw_sector_fov(canvas: np.ndarray, sector_counts: List[List[int]]) -> None:
    """3x3 tactical grid, crosshair, and zone labels."""
    h, w = canvas.shape[:2]
    tint_occupied_sectors(canvas, sector_counts)

    for x in range(0, w, 72):
        cv2.line(canvas, (x, 0), (x, h), (28, 36, 28), 1)
    for y in range(0, h, 72):
        cv2.line(canvas, (0, y), (w, y), (28, 36, 28), 1)

    for i in (1, 2):
        sx = (w * i) // 3
        sy = (h * i) // 3
        cv2.line(canvas, (sx, 0), (sx, h), (70, 100, 70), 2)
        cv2.line(canvas, (0, sy), (w, sy), (70, 100, 70), 2)

    cx, cy = w // 2, h // 2
    cv2.drawMarker(canvas, (cx, cy), (90, 110, 90), cv2.MARKER_CROSS, 18, 1)

    for r in range(3):
        for c in range(3):
            sx = (w * c) // 3 + 10
            sy = (h * r) // 3 + 28
            name = SECTOR_NAMES[r][c]
            cnt = sector_counts[r][c]
            label = "%s (%d)" % (name, cnt) if cnt else name
            col = (120, 200, 120) if cnt else (60, 85, 60)
            cv2.putText(canvas, label, (sx, sy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser(description="ArcDrone <3kbps Tactical Scene Reconstructor")
    parser.add_argument("--udp-port", type=int, default=9999, help="UDP port to listen on")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.udp_port))
    sock.setblocking(False)

    pane_w, pane_h = PANE_W, PANE_H
    tracker = ObjectTracker()
    scene_codec = SceneCodec(dst_size=(pane_w, pane_h))
    rx_budget = LoRaLinkBudget(max_kbps=SCENE_MAX_KBPS)
    last_rx_time = 0.0

    last_scene: Dict[str, Any] = {
        "seq": 0,
        "lat": 53.5232,
        "lon": -113.5263,
        "count": 0,
        "bytes": 0,
        "objects": [],
    }
    total_packets = 0

    print(
        "[RECONSTRUCTOR] Scene UDP on port %d (%.1f kbps cap, %d Hz from drone)"
        % (args.udp_port, SCENE_MAX_KBPS, 12)
    )
    print("[RECONSTRUCTOR] Press 'q' or ESC to quit.")

    try:
        while True:
            while True:
                try:
                    data, _ = sock.recvfrom(1024)
                    rx_budget.record_tx(len(data))
                    decoded = scene_codec.decode_frame(data, canvas_w=pane_w, canvas_h=pane_h)
                    if decoded is None:
                        continue
                    decoded["objects"] = tracker.update(decoded["objects"])
                    last_scene = decoded
                    total_packets += 1
                    last_rx_time = time.time()
                except BlockingIOError:
                    break
                except Exception as exc:
                    print(f"[WARN] Bad packet: {exc}")
                    break

            sector_counts = [[0, 0, 0] for _ in range(3)]
            for obj in last_scene["objects"]:
                cx, cy = obj["center"]
                _, s_row, s_col = get_sector_name(cx, cy, pane_w, pane_h)
                sector_counts[s_row][s_col] += 1

            # --- LEFT PANE: Synthetic FOV ---
            left_pane = np.full((pane_h, pane_w, 3), BG_FOV, dtype=np.uint8)
            draw_sector_fov(left_pane, sector_counts)

            for obj in last_scene["objects"]:
                color = _class_color(obj["cls"])
                x1, y1, x2, y2 = obj["bbox"]
                cx, cy = obj["center"]
                tid = obj.get("track_id", 0)
                dx, dy = obj.get("vel", (0.0, 0.0))

                sector_name, s_row, s_col = get_sector_name(cx, cy, pane_w, pane_h)
                obj["sector"] = sector_name

                trail = obj.get("trail", [])
                for i in range(1, len(trail)):
                    cv2.line(left_pane, trail[i - 1], trail[i], color, 2, cv2.LINE_AA)

                cv2.rectangle(left_pane, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
                cv2.circle(left_pane, (cx, cy), 6, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(left_pane, (cx, cy), 6, color, 1, cv2.LINE_AA)

                if math.hypot(dx, dy) >= 4.0:
                    end_x = int(min(pane_w - 5, max(5, cx + dx * 2.5)))
                    end_y = int(min(pane_h - 5, max(5, cy + dy * 2.5)))
                    cv2.arrowedLine(
                        left_pane, (cx, cy), (end_x, end_y), (0, 255, 255), 2, tipLength=0.35
                    )

                tag = "ID #%d %s %d%%" % (tid, obj["label"].upper(), int(obj["conf"] * 100))
                draw_text_bg(left_pane, tag, (x1, max(52, y1 - 6)), 0.5, color)

            cv2.rectangle(left_pane, (0, 0), (pane_w, 40), (22, 28, 22), -1)
            cv2.putText(
                left_pane,
                "SYNTHETIC FOV  |  6-byte vectors  |  no video downlink",
                (14, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (150, 255, 150),
                1,
                cv2.LINE_AA,
            )

            # --- RIGHT PANE: Manifest ---
            right_pane = np.full((pane_h, pane_w, 3), BG_PANEL, dtype=np.uint8)
            cv2.rectangle(right_pane, (0, 0), (pane_w, 40), (28, 28, 34), -1)
            cv2.putText(
                right_pane,
                "TACTICAL MANIFEST  |  class / sector / motion",
                (14, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                ACCENT,
                1,
                cv2.LINE_AA,
            )

            cv2.putText(
                right_pane,
                "ID   CLASS          SECTOR          POS (X,Y)     STATUS",
                (14, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                (170, 170, 170),
                1,
            )
            cv2.line(right_pane, (12, 70), (pane_w - 12, 70), (60, 60, 70), 1)

            if not last_scene["objects"]:
                cv2.putText(
                    right_pane,
                    "NO ACTIVE TARGETS IN CAMERA FOV",
                    (180, 160),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (100, 100, 110),
                    1,
                )
            else:
                row_y = 96
                for obj in last_scene["objects"][:10]:
                    color = _class_color(obj["cls"])
                    tid = obj.get("track_id", 0)
                    cx, cy = obj["center"]
                    nx = int((cx / pane_w) * 100)
                    ny = int((cy / pane_h) * 100)
                    sector = obj.get("sector", "CENTER")
                    motion = obj.get("motion_str", "STATIONARY")

                    line_str = (
                        f"#{tid:<3} {obj['label'].upper():<13} "
                        f"{sector:<15} ({nx:>2}%, {ny:>2}%)    {motion}"
                    )
                    cv2.putText(
                        right_pane,
                        line_str,
                        (14, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        color,
                        1,
                    )
                    row_y += 26

            # 3x3 Sector Occupancy Mini-Radar & Narrative Summary
            cv2.line(right_pane, (12, 340), (pane_w - 12, 340), (60, 60, 70), 1)
            cv2.putText(
                right_pane,
                "SECTOR OCCUPANCY MATRIX:",
                (14, 364),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (200, 200, 200),
                1,
            )

            grid_x0, grid_y0, cell_w, cell_h = 14, 374, 95, 30
            for r in range(3):
                for c in range(3):
                    cnt = sector_counts[r][c]
                    gx1 = grid_x0 + c * cell_w
                    gy1 = grid_y0 + r * cell_h
                    fill = (35, 95, 45) if cnt > 0 else (28, 28, 34)
                    cv2.rectangle(right_pane, (gx1, gy1), (gx1 + cell_w - 4, gy1 + cell_h - 4), fill, -1)
                    cv2.rectangle(
                        right_pane, (gx1, gy1), (gx1 + cell_w - 4, gy1 + cell_h - 4), (80, 80, 90), 1
                    )
                    cv2.putText(
                        right_pane,
                        f"{SECTOR_NAMES[r][c][:7]}: {cnt}",
                        (gx1 + 6, gy1 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.36,
                        (255, 255, 255) if cnt > 0 else (120, 120, 130),
                        1,
                    )

            cv2.putText(
                right_pane,
                "AUTO SCENE BRIEF:",
                (320, 364),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 220, 255),
                1,
            )
            if not last_scene["objects"]:
                brief_lines = ["Scene clear. 0 objects detected."]
            else:
                brief_lines = []
                for obj in last_scene["objects"][:3]:
                    brief_lines.append(
                        f"- {obj['label'].upper()} #{obj.get('track_id',0)} at {obj.get('sector','')}"
                    )
                    brief_lines.append(f"  ({obj.get('motion_str', 'STATIONARY')})")

            by = 386
            for bline in brief_lines[:4]:
                cv2.putText(
                    right_pane,
                    bline,
                    (320, by),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (220, 220, 220),
                    1,
                )
                by += 20

            # --- Combine Left & Right + Link Budget Header ---
            body = np.hstack([left_pane, right_pane])
            full_w = pane_w * 2
            header = np.full((72, full_w, 3), (10, 10, 10), dtype=np.uint8)
            footer = np.full((44, full_w, 3), (10, 10, 10), dtype=np.uint8)

            kbps = rx_budget.current_kbps()
            pct = min(100.0, (kbps / SCENE_MAX_KBPS) * 100.0)
            saved_pct = max(0.0, (1.0 - (kbps / 480.0)) * 100.0)
            link_live = (time.time() - last_rx_time) < 1.5 and total_packets > 0
            live_col = LIVE_OK if link_live else LIVE_STALE
            live_txt = "LIVE" if link_live else "STALE"

            cv2.circle(header, (24, 58), 7, live_col, -1)
            cv2.putText(
                header,
                "UDP SCENE %s" % live_txt,
                (38, 64),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                live_col,
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                header,
                "SCENE LINK: %.2f / %.1f kbps (%.0f%%)"
                % (kbps, SCENE_MAX_KBPS, pct),
                (16, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 128) if kbps <= SCENE_MAX_KBPS * 0.85 else (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                header,
                "LAST FRAME: %d B (%d obj)  |  saved vs raw video ~%.1f%%"
                % (last_scene["bytes"], last_scene["count"], saved_pct),
                (16, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )

            bar_x1, bar_y1, bar_w, bar_h = full_w - 400, 20, 360, 24
            cv2.rectangle(header, (bar_x1, bar_y1), (bar_x1 + bar_w, bar_y1 + bar_h), (50, 50, 50), -1)
            fill_w = int((pct / 100.0) * bar_w)
            bar_color = (0, 230, 110) if pct < 80 else (0, 140, 255)
            cv2.rectangle(header, (bar_x1, bar_y1), (bar_x1 + fill_w, bar_y1 + bar_h), bar_color, -1)
            cv2.rectangle(header, (bar_x1, bar_y1), (bar_x1 + bar_w, bar_y1 + bar_h), (180, 180, 180), 1)

            cv2.putText(
                footer,
                f"SEQ #{last_scene['seq']} | OBJECTS IN SCENE: {last_scene['count']} | "
                f"DRONE GPS: ({last_scene['lat']:.4f}, {last_scene['lon']:.4f}) | RX PACKETS: {total_packets}",
                (16, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

            dashboard = np.vstack([header, body, footer])
            cv2.imshow("ArcDrone Ground Station | %.0f kbps Scene Twin" % LINK_MAX_KBPS, dashboard)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break

    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()