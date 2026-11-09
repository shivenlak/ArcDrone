#!/usr/bin/env python3
"""Executes the ArcDrone 1:1 side-by-side test suite across all 6 real VisDrone-VID/MOT test cases."""

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
        "seq": "uav0000268_05773_v",
        "file": "uav0000268_05773_v.mp4",
        "title": "Case 1: VisDrone-VID/MOT uav0000268_05773_v (Highway Corridor & Cruisers)",
        "frames": 150,
    },
    {
        "id": "case2",
        "seq": "uav0000086_00000_v",
        "file": "uav0000086_00000_v.mp4",
        "title": "Case 2: VisDrone-VID/MOT uav0000086_00000_v (Urban Pedestrian Plaza)",
        "frames": 150,
    },
    {
        "id": "case3",
        "seq": "uav0000182_00000_v",
        "file": "uav0000182_00000_v.mp4",
        "title": "Case 3: VisDrone-VID/MOT uav0000182_00000_v (Arterial Intersection & Motors)",
        "frames": 150,
    },
    {
        "id": "case4",
        "seq": "uav0000117_02622_v",
        "file": "uav0000117_02622_v.mp4",
        "title": "Case 4: VisDrone-VID/MOT uav0000117_02622_v (Mixed Urban Street & Crosswalk)",
        "frames": 150,
    },
    {
        "id": "case5",
        "seq": "uav0000339_00001_v",
        "file": "uav0000339_00001_v.mp4",
        "title": "Case 5: VisDrone-VID/MOT uav0000339_00001_v (Dusk Avenue Multi-Lane Patrol)",
        "frames": 150,
    },
    {
        "id": "case6",
        "seq": "uav0000137_00458_v",
        "file": "uav0000137_00458_v.mp4",
        "title": "Case 6: VisDrone-VID/MOT uav0000137_00458_v (High-Density Multi-Class Junction)",
        "frames": 150,
    },
]


def main() -> None:
    clips_dir = os.path.join(_REPO_ROOT, "data", "clips")
    data_dir = os.path.join(_REPO_ROOT, "data", "test_cases")
    os.makedirs(data_dir, exist_ok=True)
    model_path = os.path.join(_REPO_ROOT, "yolov8n-visdrone.pt")
    if not os.path.isfile(model_path):
        model_path = os.path.join(_REPO_ROOT, "yolov8n.pt")
    results = []

    print("=" * 80)
    print("ARCDRONE TEST SUITE: EXECUTING 6 REAL VISDRONE-VID/MOT SIDE-BY-SIDE BENCHMARKS")
    print("=" * 80)

    start_total = time.monotonic()
    for idx, tc in enumerate(TEST_CASES, 1):
        in_path = os.path.join(clips_dir, tc["file"])
        if not os.path.isfile(in_path):
            in_path = os.path.join(data_dir, tc["file"])
        out_mp4 = os.path.join(data_dir, f"{tc['id']}_{tc['seq']}_side_by_side.mp4")
        out_png = os.path.join(data_dir, f"{tc['id']}_{tc['seq']}_side_by_side.png")

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
        metrics["seq"] = tc["seq"]
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
    print("BENCHMARK EXECUTION SUMMARY (ALL 6 VISDRONE-VID/MOT CASES COMPLETED)")
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
