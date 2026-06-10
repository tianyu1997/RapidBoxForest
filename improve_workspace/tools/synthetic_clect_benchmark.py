#!/usr/bin/env python3
"""Synthetic benchmark for the improve.md C-LECT sidecar implementation."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from improve_workspace.clect_sidecar import AdaptiveLeafSweep, AdaptiveSweepConfig, Interval
from improve_workspace.clect_sidecar.synthetic import Region, SyntheticValidator, fixed_leaf_evaluation_count, unit_root


def run_case(start_depth: int, max_depth: int) -> dict[str, object]:
    root = unit_root(2)
    validator = SyntheticValidator(
        free_regions=[
            Region((Interval(0.0, 0.5), Interval(0.0, 1.0)), "large_free_half"),
            Region((Interval(0.75, 0.8125), Interval(0.25, 0.75)), "narrow_free_band"),
        ],
        occupied_regions=[
            Region((Interval(0.5, 0.75), Interval(0.0, 1.0)), "solid_blocker"),
        ],
    )
    sweep = AdaptiveLeafSweep(
        root,
        validator,
        AdaptiveSweepConfig(start_depth=start_depth, max_depth=max_depth, max_evaluations=100000),
    )
    result = sweep.run()
    fixed_per_start = fixed_leaf_evaluation_count(start_depth, max_depth)
    start_cells = 1 << start_depth
    return {
        "start_depth": start_depth,
        "max_depth": max_depth,
        "fixed_virtual_leaf_evaluations": start_cells * fixed_per_start,
        "adaptive_evaluations": result.evaluated,
        "adaptive_terminal_count": result.terminal_count,
        "free_terminal_count": len(result.free),
        "occupied_terminal_count": len(result.occupied),
        "collision_terminal_count": len(result.collision),
        "deferred_terminal_count": len(result.deferred),
        "split_count": result.split_count,
        "depth_histogram": dict(sorted(result.depth_histogram.items())),
        "validation_counts": dict(result.validation_counts),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-depth", type=int, default=4)
    parser.add_argument("--max-depth", type=int, default=12)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--md-out", default="improve_workspace/synthetic_clect_benchmark.md")
    args = parser.parse_args()

    payload = run_case(args.start_depth, args.max_depth)
    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        lines = [
            "# Synthetic C-LECT Benchmark",
            "",
            f"- start depth: {payload['start_depth']}",
            f"- max depth: {payload['max_depth']}",
            f"- fixed virtual leaf evaluations: {payload['fixed_virtual_leaf_evaluations']}",
            f"- adaptive evaluations: {payload['adaptive_evaluations']}",
            f"- terminal cells: {payload['adaptive_terminal_count']}",
            f"- free terminals: {payload['free_terminal_count']}",
            f"- occupied terminals: {payload['occupied_terminal_count']}",
            f"- collision terminals: {payload['collision_terminal_count']}",
            f"- deferred terminals: {payload['deferred_terminal_count']}",
            "",
        ]
        out = Path(args.md_out)
        out.write_text("\n".join(lines), encoding="utf-8")
        print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
