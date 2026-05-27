#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import euclidean_path_length, path_passes_post_audit  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import sbf  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import make_random_scene, make_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Strictly audit counted random-scene anytime artifact paths.")
    parser.add_argument("artifacts", nargs="+", type=Path)
    parser.add_argument("--audit-segment-step", type=float, default=0.01)
    parser.add_argument("--fail-on-invalid", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def point_distance(lhs: list[float], rhs: list[float]) -> float:
    return math.sqrt(sum((float(left) - float(right)) ** 2 for left, right in zip(lhs, rhs)))


def scene_for(
    cache: dict[tuple[int, str, str, str, int], tuple[Any, Any]],
    seed_base: int,
    scene_profile: str,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
) -> tuple[Any, Any]:
    key = (int(seed_base), str(scene_profile), str(robot_name), str(difficulty), int(scene_seed))
    if key not in cache:
        cache[key] = (
            make_robot(robot_name),
            make_random_scene(robot_name, difficulty, int(seed_base) + 1009 * int(scene_seed), scene_profile=scene_profile),
        )
    return cache[key]


def counted_tasks(payload: dict[str, Any]):
    params = payload.get("params", {}) or {}
    seed_base = int(params.get("seed_base", 20260504))
    scene_profile = str(params.get("scene_profile", "balanced"))
    for record_index, record in enumerate(payload.get("records", []) or []):
        record_params = record.get("params", {}) or {}
        robot_name = record_params.get("robot")
        difficulty = record_params.get("difficulty")
        scene_seed = record_params.get("scene_seed")
        if robot_name is None or difficulty is None or scene_seed is None:
            continue
        tasks = record.get("incumbent_tasks") or []
        if not tasks and str(record.get("method")) == "ompl_rrtconnect":
            tasks = record.get("raw_tasks") or []
        for task_index, task in enumerate(tasks):
            if not bool(task.get("audit_passed", task.get("ok", False))):
                continue
            waypoints = task.get("waypoints") or (task.get("raw") or {}).get("waypoints") or []
            if not waypoints:
                continue
            yield {
                "record_index": int(record_index),
                "task_index": int(task_index),
                "method": str(record.get("method")),
                "stage_id": str(record.get("stage_id")),
                "robot": str(robot_name),
                "difficulty": str(difficulty),
                "scene_seed": int(scene_seed),
                "seed_base": int(seed_base),
                "scene_profile": scene_profile,
                "waypoints": [[float(value) for value in point] for point in waypoints],
            }


def audit_artifact(path: Path, audit_segment_step: float) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    scene_cache: dict[tuple[int, str, str, str, int], tuple[Any, Any]] = {}
    invalid: list[dict[str, Any]] = []
    ratio_by_method: dict[str, list[float]] = {}
    counted = 0
    for task in counted_tasks(payload):
        counted += 1
        robot, scene = scene_for(
            scene_cache,
            int(task["seed_base"]),
            str(task["scene_profile"]),
            str(task["robot"]),
            str(task["difficulty"]),
            int(task["scene_seed"]),
        )
        waypoints = task["waypoints"]
        direct_length = point_distance(scene.start, scene.goal)
        path_length = euclidean_path_length(waypoints)
        if direct_length > 1e-12:
            ratio_by_method.setdefault(str(task["method"]), []).append(path_length / direct_length)
        valid = path_passes_post_audit(
            sbf,
            robot,
            scene.obstacles,
            waypoints,
            segment_step=float(audit_segment_step),
            start=list(scene.start),
            goal=list(scene.goal),
        )
        if not valid:
            signature_payload = json.dumps(waypoints, separators=(",", ":"), sort_keys=True)
            path_signature = hashlib.sha1(signature_payload.encode("utf-8")).hexdigest()[:16]
            invalid.append({
                key: task[key]
                for key in ("record_index", "task_index", "method", "stage_id", "robot", "difficulty", "scene_seed")
            } | {
                "path_signature": path_signature,
                "path_length": path_length,
                "direct_length": direct_length,
                "path_over_direct": path_length / max(direct_length, 1e-12),
            })
    ratios = {
        method: {
            "count": len(values),
            "median": statistics.median(values) if values else None,
            "min": min(values) if values else None,
            "max": max(values) if values else None,
        }
        for method, values in sorted(ratio_by_method.items())
    }
    return {
        "artifact": str(path),
        "audit_segment_step": float(audit_segment_step),
        "counted_success_paths": int(counted),
        "invalid_count": len(invalid),
        "unique_invalid_path_count": len({str(row.get("path_signature")) for row in invalid}),
        "unique_invalid_scene_method_task_count": len({
            (
                str(row.get("method")),
                str(row.get("robot")),
                str(row.get("difficulty")),
                int(row.get("scene_seed", -1)),
                int(row.get("task_index", -1)),
            )
            for row in invalid
        }),
        "invalid_by_method": dict(sorted(Counter(str(row.get("method")) for row in invalid).items())),
        "invalid_by_robot_difficulty": {
            f"{robot}:{difficulty}": count
            for (robot, difficulty), count in sorted(Counter((str(row.get("robot")), str(row.get("difficulty"))) for row in invalid).items())
        },
        "invalid_by_stage": dict(sorted(Counter(str(row.get("stage_id")) for row in invalid).items())),
        "invalid_examples": invalid[:20],
        "path_over_direct_by_method": ratios,
    }


def main() -> int:
    args = parse_args()
    reports = [audit_artifact(path, float(args.audit_segment_step)) for path in args.artifacts]
    print(json.dumps({"reports": reports}, indent=2, sort_keys=True))
    if bool(args.fail_on_invalid) and any(int(report["invalid_count"]) > 0 for report in reports):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())