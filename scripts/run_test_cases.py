#!/usr/bin/env python3
"""Executes the ArcDrone 1:1 side-by-side test suite across all 6 test cases."""

import os
import sys
import time
import json
from typing import List, Dict, Any

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.ishan_side_by_side import run_pipeline

TEST_CASES = [
    {
        "id": "case1",
        "file": "case1_urban_intersection.mp4",
        "title": "Case 1: Urban Intersection (Dense Cross-Traffic)",
        "frames": 150,
    },
    {
        "id": "case2",
        "file": "case2_dynamic_flight.mp4",
        "title": "Case 2: Dynamic Flight (Active Yaw & Camera Drift)",
        "frames": 150,
    },
    {
        "id": "case3",
        "file": "case3_perimeter_airport.mp4",
        "title": "Case 3: Perimeter Staging (Airfield Tarmac & Trucks)",
        "frames": 150,
    },
    {
        "id": "case4",
        "file": "case4_high_altitude_sahi.mp4",
        "title": "Case 4: High Altitude Roundabout (Multi-Class Circulation)",
        "frames": 150,
    },
    {
        "id": "case5",
        "file": "case5_parking_staging.mp4",
        "title": "Case 5: Checkpoint Queue (Compound Perimeter Inspection)",
        "frames": 150,
    },
    {
        "id": "case6",
        "file": "case6_aerial_traffic_1080p.mp4",
        "title": "Case 6: Arterial Transit (Corridor Cruisers & Buses)",
        "frames": 150,
    },
]


def main() -> None:
    data_dir = os.path.join(_REPO_ROOT, "data", "test_cases")
    model_path = os.path.join(_REPO_ROOT, "yolov8n-visdrone.pt")
    results = []

    print("=" * 80)
    print("ARCDRONE TEST SUITE: EXECUTING 6 SIDE-BY-SIDE VIDEO BENCHMARKS")
    print("=" * 80)

    start_total = time.monotonic()
    for idx, tc in enumerate(TEST_CASES, 1):
        in_path = os.path.join(data_dir, tc["file"])
        out_mp4 = os.path.join(data_dir, f"{tc['id']}_side_by_side.mp4")
        out_png = os.path.join(data_dir, f"{tc['id']}_side_by_side.png")

        print(f"\n[{idx}/6] RUNNING BENCHMARK: {tc['title']}")
        print(f"      Input Video: {in_path}")
        print(f"      Output MP4 : {out_mp4}")
        print(f"      Output PNG : {out_png}")

        t0 = time.monotonic()
        metrics = run_pipeline(
            video_path=in_path,
            model_path=model_path,
            output_mp4=out_mp4,
            output_snapshot=out_png,
            num_frames=tc["frames"],
            scenario_title=tc["title"],
        )
        elapsed = time.monotonic() - t0
        metrics["case_id"] = tc["id"]
        metrics["title"] = tc["title"]
        metrics["elapsed_sec"] = elapsed
        results.append(metrics)
        print(f"      DONE in {elapsed:.1f}s | Active Indices: {metrics['active_indices']} | "
              f"Peak Wire Rate: {metrics['peak_bps']:.0f} B/s | Size: {metrics['file_size']/1e6:.1f} MB")

    total_elapsed = time.monotonic() - start_total
    summary_path = os.path.join(data_dir, "benchmark_results.json")
    with open(summary_path, "w") as f:
        json.dump(results, f, indent=2)

    print("\n" + "=" * 80)
    print("BENCHMARK EXECUTION SUMMARY (ALL 6 CASES COMPLETED)")
    print("=" * 80)
    print(f"{'Case':<8} | {'Scenario Title':<40} | {'Indices':<8} | {'Pkts':<6} | {'Peak Rate':<10} | {'Time':<6}")
    print("-" * 80)
    for r in results:
        peak_kbps = r['peak_bps'] / 1000.0
        print(f"{r['case_id']:<8} | {r['title'][:40]:<40} | {r['active_indices']:<8} | {r['total_pkts']:<6} | {peak_kbps:.2f} kB/s  | {r['elapsed_sec']:<5.1f}s")
    print("-" * 80)
    print(f"Total Test Suite Runtime: {total_elapsed:.1f}s (~{total_elapsed/60.0:.1f} min)")
    print(f"Saved JSON Report: {summary_path}")


if __name__ == "__main__":
    main()
