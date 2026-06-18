#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (REPO_ROOT, REPO_ROOT / "build" / "python", REPO_ROOT / "build-leaf-sweep" / "python"):
    if candidate.exists() and str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from experiments.common.rbf_leaf_rrt import canonical_q
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


def path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return math.nan
    return sum(math.dist(a, b) for a, b in zip(path[:-1], path[1:]))


def collision_sample_count(robot: Any, obstacles: list[Any], a: list[float], b: list[float], step: float) -> int:
    distance = math.dist(a, b)
    steps = max(1, int(math.ceil(distance / max(float(step), 1e-9))))
    count = 0
    for index in range(steps + 1):
        alpha = index / steps
        q = [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]
        if sbf.check_config_collision(robot, obstacles, q):
            count += 1
    return count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check raw/canonical query-space semantics for shelf queries.")
    parser.add_argument("--audit-segment-step", type=float, default=0.01)
    parser.add_argument("--fail-on-difference", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    rows: list[dict[str, Any]] = []
    for query in sbf.make_combined_queries():
        raw_start = [float(value) for value in query.start]
        raw_goal = [float(value) for value in query.goal]
        canonical_start = canonical_q(robot, raw_start)
        canonical_goal = canonical_q(robot, raw_goal)
        start_delta = [a - c for a, c in zip(raw_start, canonical_start)]
        goal_delta = [a - c for a, c in zip(raw_goal, canonical_goal)]
        same_reflection = max((abs(a - b) for a, b in zip(start_delta, goal_delta)), default=0.0) <= 1e-6
        raw_hits = collision_sample_count(robot, obstacles, raw_start, raw_goal, float(args.audit_segment_step))
        canonical_hits = collision_sample_count(robot, obstacles, canonical_start, canonical_goal, float(args.audit_segment_step))
        row = {
            "label": str(query.label),
            "raw_length": path_length([raw_start, raw_goal]),
            "canonical_length": path_length([canonical_start, canonical_goal]),
            "raw_collision_samples": raw_hits,
            "canonical_collision_samples": canonical_hits,
            "same_constant_reflection": same_reflection,
        }
        rows.append(row)
        print(row)
    differs = [
        row for row in rows
        if int(row["raw_collision_samples"]) != int(row["canonical_collision_samples"])
        or not bool(row["same_constant_reflection"])
    ]
    if differs and bool(args.fail_on_difference):
        print(f"query-space differences detected: {len(differs)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
