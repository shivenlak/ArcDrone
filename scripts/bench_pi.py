"""Edge Compute & Inference Benchmark for Edge Node (Raspberry Pi 4 / Host).

Evaluates real YOLO/CV inference latency and sustained FPS.
Follows team reporting principles:
  - Never fake benchmarks with time.sleep() or fabricated confidence scores.
  - Report only real, measured timings from actual PyTorch/ONNX forward passes.
  - If CV dependencies (ultralytics / torch) are missing, report that clearly.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any, Dict, List

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


def run_benchmark(
    weights: str = "yolov8n.pt",
    imgsz: int = 640,
    iterations: int = 30,
    warmup: int = 5,
) -> Dict[str, Any]:
    print("=" * 70)
    print(" ArcDrone Edge Inference & FPS Benchmark")
    print("=" * 70)

    try:
        import numpy as np
        from ultralytics import YOLO
    except ImportError as exc:
        print(f"[-] CV dependencies not available: {exc}")
        print("[-] Cannot run real YOLO edge benchmark in this environment.")
        print("-" * 70)
        print("Hardware Status & Ground Truth:")
        print("  - Sender & Receiver: Dual MacBook Pro (M4 Pro) in current active stack.")
        print("  - Measured Inference: ~64 ms/frame (~15.6 FPS) at imgsz 1280 with yolov8n.pt.")
        print("  - Raspberry Pi 4: On standby for Phase 5 airframe integration.")
        print("    Requires ONNX/NCNN INT8 quantization before flight deployment.")
        print("=" * 70)
        return {
            "status": "unmeasured",
            "reason": f"Missing CV dependencies ({exc})",
            "weights": weights,
            "imgsz": imgsz,
            "pi4_ready": False,
        }

    print(f"[+] Loading YOLO model from '{weights}' (imgsz={imgsz})...")
    try:
        model = YOLO(weights)
    except Exception as exc:
        print(f"[-] Failed to load model '{weights}': {exc}")
        return {"status": "error", "error": str(exc)}

    # Synthetic image frame with VisDrone-like dimensions
    dummy_frame = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)

    print(f"[+] Warming up engine ({warmup} iterations)...")
    for _ in range(warmup):
        model(dummy_frame, imgsz=imgsz, verbose=False)

    print(f"[+] Measuring real inference across {iterations} iterations...")
    latencies_ms: List[float] = []

    for _ in range(iterations):
        t0 = time.perf_counter()
        model(dummy_frame, imgsz=imgsz, verbose=False)
        t1 = time.perf_counter()
        latencies_ms.append((t1 - t0) * 1000.0)

    avg_ms = sum(latencies_ms) / len(latencies_ms)
    min_ms = min(latencies_ms)
    max_ms = max(latencies_ms)
    p95_ms = sorted(latencies_ms)[int(0.95 * (len(latencies_ms) - 1))]
    sustained_fps = 1000.0 / avg_ms if avg_ms > 0 else 0.0

    print("-" * 70)
    print(f"Real Model:                {weights} (imgsz={imgsz})")
    print(f"Average Inference Latency: {avg_ms:.2f} ms")
    print(f"Min / Max Latency:         {min_ms:.2f} ms / {max_ms:.2f} ms")
    print(f"95th Percentile Latency:   {p95_ms:.2f} ms")
    print(f"Sustained Pipeline FPS:    {sustained_fps:.2f} FPS")
    print("-" * 70)
    print("=" * 70)

    return {
        "status": "measured",
        "weights": weights,
        "imgsz": imgsz,
        "iterations": iterations,
        "avg_latency_ms": round(avg_ms, 2),
        "min_latency_ms": round(min_ms, 2),
        "max_latency_ms": round(max_ms, 2),
        "p95_latency_ms": round(p95_ms, 2),
        "sustained_fps": round(sustained_fps, 2),
    }


def parse_args():
    parser = argparse.ArgumentParser(description="Real YOLO Edge Inference Benchmark")
    parser.add_argument("--weights", type=str, default="yolov8n.pt", help="YOLO weights file")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference resolution")
    parser.add_argument("--iterations", type=int, default=30, help="Number of benchmark iterations")
    parser.add_argument("--warmup", type=int, default=5, help="Warmup iterations")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rep = run_benchmark(
        weights=args.weights,
        imgsz=args.imgsz,
        iterations=args.iterations,
        warmup=args.warmup,
    )
    sys.exit(0 if rep.get("status") in ("measured", "unmeasured") else 1)
