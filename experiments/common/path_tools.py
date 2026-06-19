from __future__ import annotations

import math
import time
from collections.abc import Iterable
from typing import Any


def point_distance(a: Iterable[float], b: Iterable[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def path_length(path: Iterable[Iterable[float]]) -> float:
    pts = [[float(value) for value in point] for point in path]
    if len(pts) < 2:
        return math.nan
    return sum(point_distance(a, b) for a, b in zip(pts, pts[1:]))


def interpolate(a: Iterable[float], b: Iterable[float], alpha: float) -> list[float]:
    return [(1.0 - float(alpha)) * float(x) + float(alpha) * float(y) for x, y in zip(a, b)]


def audit_path(
    sbf_module: Any,
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    *,
    start: list[float] | None = None,
    goal: list[float] | None = None,
    endpoint_tol: float = 1e-6,
    collision_tolerance: float = 0.0,
) -> tuple[bool, float, str]:
    t0 = time.perf_counter()
    if len(path) < 2:
        return False, time.perf_counter() - t0, "empty_path"
    if start is not None and point_distance(path[0], list(start)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "start_mismatch"
    if goal is not None and point_distance(path[-1], list(goal)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "goal_mismatch"
    step = max(1e-9, float(segment_step))
    for a, b in zip(path, path[1:]):
        distance = point_distance(a, b)
        steps = max(1, int(math.ceil(distance / step)))
        for index in range(steps + 1):
            if sbf_module.check_config_collision(
                robot,
                obstacles,
                interpolate(a, b, index / steps),
                float(collision_tolerance),
            ):
                return False, time.perf_counter() - t0, "collision"
    return True, time.perf_counter() - t0, "passed"
