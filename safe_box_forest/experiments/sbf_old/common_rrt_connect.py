from __future__ import annotations

import math
import time
from typing import Any

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import sbf


def distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) * (x - y) for x, y in zip(a, b)))


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def path_length(path: list[list[float]]) -> float:
    return sum(distance(path[index], path[index + 1]) for index in range(len(path) - 1))


def collision_free(robot: Any, obstacles: list[Any], q: list[float]) -> bool:
    return not bool(sbf.check_config_collision(robot, obstacles, q))


def segment_free(robot: Any, obstacles: list[Any], a: list[float], b: list[float], step: float) -> bool:
    pieces = max(1, int(math.ceil(distance(a, b) / max(float(step), 1e-9))))
    for item in range(pieces + 1):
        if not collision_free(robot, obstacles, interpolate(a, b, item / pieces)):
            return False
    return True


def rrt_connect(
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    bounds: list[tuple[float, float]],
    rng: Any,
    args: Any,
    seed: int | None = None,
) -> dict[str, Any]:
    del bounds
    if seed is None:
        seed = int(rng.randrange(0, 2**31 - 1)) if hasattr(rng, "randrange") else 42
    start_time = time.perf_counter()
    result = dict(sbf.ompl_rrt_connect_path(
        robot,
        obstacles,
        list(start),
        list(goal),
        float(args.timeout_ms),
        float(args.step_size),
        float(args.segment_step),
        float(getattr(args, "simplify_time_s", 0.0)),
        int(seed),
    ))
    path = [list(point) for point in result.get("path", [])]
    ok = bool(result.get("ok")) and len(path) >= 2
    audit_passed = False
    if ok:
        audit_passed = all(segment_free(robot, obstacles, path[index], path[index + 1], float(args.segment_step)) for index in range(len(path) - 1))
    return {
        "ok": ok,
        "reason": result.get("reason", "ompl_no_solution"),
        "t_s": float(result.get("t_s", time.perf_counter() - start_time)),
        "path": path,
        "length": path_length(path) if ok else 0.0,
        "iterations": 0,
        "nodes": int(result.get("nodes", 0) or 0),
        "audit_passed": bool(audit_passed),
        "planner": "OMPL_RRTConnect",
        "ompl_status": result.get("status"),
        "checking_resolution": result.get("checking_resolution"),
    }