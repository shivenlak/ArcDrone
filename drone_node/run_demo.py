"""One-command demo: VisDrone footage -> drone -> radio -> ground dashboard.

    python -m drone_node.run_demo                    # first clip in data/ (not webcam)
    python -m drone_node.run_demo --video 0          # webcam instead
    python -m drone_node.run_demo --video case6_aerial_traffic_1080p.mp4
    python -m drone_node.run_demo --loss 0.15        # drop 15% of packets

Dev only: uses protocol.fake_bridge (not a real radio). Ctrl+C stops everything.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
import webbrowser
from typing import List

from drone_node.demo_video import resolve_demo_video
from protocol.fake_bridge import FakeBridge

PY = sys.executable
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DASHBOARD_URL = "http://127.0.0.1:8000"

_DEFAULT_MODEL = os.path.join(_REPO_ROOT, "yolov8n-visdrone.pt")
if not os.path.isfile(_DEFAULT_MODEL):
    _DEFAULT_MODEL = os.path.join(_REPO_ROOT, "yolov8n.pt")


def _pump(proc: subprocess.Popen, tag: str) -> None:
    assert proc.stdout is not None
    for raw in proc.stdout:
        sys.stdout.write("[%s] %s" % (tag, raw))
        sys.stdout.flush()


def _spawn(args: List[str], tag: str) -> subprocess.Popen:
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
    )
    threading.Thread(target=_pump, args=(proc, tag), daemon=True).start()
    return proc


def main() -> None:
    parser = argparse.ArgumentParser(description="ArcDrone one-command footage->dashboard demo")
    parser.add_argument(
        "--video",
        default=None,
        help="Clip under data/, file path, VisDrone-VID folder, or camera index (default: first clip in data/)",
    )
    parser.add_argument("--fps", type=float, default=4.0, help="YOLO inference FPS")
    parser.add_argument("--conf", type=float, default=0.25, help="Detection confidence gate")
    parser.add_argument(
        "--imgsz",
        type=int,
        default=1280,
        help="YOLO inference size (1280/1536 for high-res aerial clips)",
    )
    parser.add_argument(
        "--spatial-hold",
        type=float,
        default=30.0,
        help="Do not re-alert the same contact while it stays in view (sec)",
    )
    parser.add_argument(
        "--model",
        default=_DEFAULT_MODEL,
        help="YOLO weights (prefers yolov8n-visdrone.pt when present)",
    )
    parser.add_argument("--loss", type=float, default=0.0, help="Fake-radio packet loss 0-1")
    parser.add_argument("--no-browser", action="store_true", help="Do not open the dashboard")
    args = parser.parse_args()
    video_src = resolve_demo_video(args.video)

    procs: List[subprocess.Popen] = []
    bridge = None  # type: ignore
    try:
        print("[demo] starting fake radio bridge (in-process)...", flush=True)
        bridge = FakeBridge(args.loss).start()
        drone_port, ground_port = bridge.ports[0], bridge.ports[1]
        print("[demo] drone port=%s  ground port=%s" % (drone_port, ground_port), flush=True)

        print("[demo] starting scene relay (UI :8080, ws :8765, udp :9999)...", flush=True)
        procs.append(_spawn([PY, "-u", "-m", "drone_node.scene_ws_relay"], "relay"))

        print("[demo] starting ground dashboard on :8000...", flush=True)
        procs.append(
            _spawn([PY, "-u", "-m", "ground_station.server", "--port", ground_port], "ground")
        )

        time.sleep(1.5)

        print(
            "[demo] starting drone node on footage %s ..."
            " (YOLO load can take 30-90s the first time — wait for [drone] lines)"
            % video_src,
            flush=True,
        )
        procs.append(
            _spawn(
                [
                    PY,
                    "-u",
                    "-m",
                    "drone_node.main",
                    "--port",
                    drone_port,
                    "--video",
                    str(video_src),
                    "--fps",
                    str(args.fps),
                    "--conf",
                    str(args.conf),
                    "--imgsz",
                    str(args.imgsz),
                    "--spatial-hold",
                    str(args.spatial_hold),
                    "--model",
                    args.model,
                    "--headless",
                    "--preview-port",
                    "8081",
                    "--manan-panel-port",
                    "8082",
                ],
                "drone",
            )
        )

        if not args.no_browser:
            time.sleep(2.0)
            print("[demo] opening %s" % DASHBOARD_URL, flush=True)
            webbrowser.open(DASHBOARD_URL)

        print("\n[demo] Running. Dashboard: %s" % DASHBOARD_URL, flush=True)
        print("[demo]   Main screen updates automatically when detections fire over the radio.", flush=True)
        print("[demo]   Buttons: Scene twin (Manan reconstructor :8082) | Live camera (:8081)", flush=True)
        print("[demo] Press Ctrl+C to stop.\n", flush=True)

        while True:
            dead = [p for p in procs if p.poll() is not None]
            if dead:
                print("[demo] a process exited (code %s); shutting down." % dead[0].returncode, flush=True)
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[demo] Ctrl+C — stopping.", flush=True)
    finally:
        for proc in reversed(procs):
            if proc.poll() is None:
                proc.terminate()
        deadline = time.time() + 5
        for proc in reversed(procs):
            remaining = max(0.0, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                proc.kill()
        print("[demo] done.", flush=True)


if __name__ == "__main__":
    main()
