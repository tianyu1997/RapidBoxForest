from __future__ import annotations

import concurrent.futures as futures
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from common_sbf_config import sbf
from sbf.marcucci import (
    load_iiwa14_robot,
    make_bins_obstacles,
    make_combined_obstacles,
    make_shelves_obstacles,
    make_table_obstacles,
)


SCENE_BUILDERS = {
    "shelves": make_shelves_obstacles,
    "bins": make_bins_obstacles,
    "table": make_table_obstacles,
    "combined": make_combined_obstacles,
    "marcucci": make_combined_obstacles,
    "marcucci_combined": make_combined_obstacles,
}

_ROBOT: Any | None = None
_OBSTACLES: list[Any] | None = None
_LIMIT_LO: np.ndarray | None = None
_LIMIT_HI: np.ndarray | None = None


@dataclass(frozen=True)
class RefineConfig:
    enabled: bool = True
    candidates: int = 20
    workers: int = 8
    seed_base: int = 1000
    rrt_max_iters: int = 26000
    rrt_timeout_ms: float = 250.0
    rrt_step_size: float = 0.11
    rrt_goal_bias: float = 0.25
    rrt_segment_resolution: int = 16
    smooth_time_ms: float = 45.0
    segment_step: float = 0.04
    random_samples: int = 10
    improve_epsilon: float = 1e-6


def _load_scene(scene: str) -> list[Any]:
    try:
        return SCENE_BUILDERS[scene]()
    except KeyError as exc:
        raise ValueError(f"unknown scene {scene!r}; choices={sorted(SCENE_BUILDERS)}") from exc


def _init_worker(scene: str) -> None:
    global _ROBOT, _OBSTACLES, _LIMIT_LO, _LIMIT_HI
    _ROBOT = load_iiwa14_robot()
    _OBSTACLES = _load_scene(scene)
    limits = _ROBOT.joint_limits().limits
    _LIMIT_LO = np.array([interval.lo for interval in limits], dtype=float)
    _LIMIT_HI = np.array([interval.hi for interval in limits], dtype=float)


def _ensure_worker(scene: str) -> None:
    if _ROBOT is None or _OBSTACLES is None:
        _init_worker(scene)


def _path_length(points: list[np.ndarray]) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(np.linalg.norm(points[i + 1] - points[i]) for i in range(len(points) - 1)))


def _segment_collision_free(a: np.ndarray, b: np.ndarray, step: float) -> bool:
    assert _ROBOT is not None and _OBSTACLES is not None
    distance = float(np.linalg.norm(b - a))
    samples = max(2, int(np.ceil(distance / max(step, 1e-6))))
    for index in range(1, samples):
        alpha = index / samples
        q = (1.0 - alpha) * a + alpha * b
        if sbf.check_config_collision(_ROBOT, _OBSTACLES, [float(value) for value in q]):
            return False
    return True


def _path_collision_free(points: list[np.ndarray], step: float) -> bool:
    return all(_segment_collision_free(points[i], points[i + 1], step) for i in range(len(points) - 1))


def _shortcut(points: list[np.ndarray], step: float, rounds: int = 2) -> list[np.ndarray]:
    pts = [np.asarray(point, dtype=float) for point in points]
    for _ in range(rounds):
        if len(pts) <= 2:
            break
        out = [pts[0]]
        index = 0
        changed = False
        while index < len(pts) - 1:
            best = index + 1
            for candidate in range(len(pts) - 1, index, -1):
                if _segment_collision_free(pts[index], pts[candidate], step):
                    best = candidate
                    break
            if best > index + 1:
                changed = True
            out.append(pts[best])
            index = best
        pts = out
        if not changed:
            break
    return pts


def _smooth(points: list[np.ndarray], config: RefineConfig, seed: int) -> list[np.ndarray]:
    assert _LIMIT_LO is not None and _LIMIT_HI is not None
    rng = np.random.default_rng(seed)
    pts = _shortcut(points, config.segment_step)
    deadline = time.perf_counter() + max(0.0, config.smooth_time_ms) / 1000.0
    temperature = 0.45

    while time.perf_counter() < deadline and len(pts) > 2:
        for index in range(1, len(pts) - 1):
            current = pts[index].copy()
            previous = pts[index - 1]
            following = pts[index + 1]
            before = float(np.linalg.norm(current - previous) + np.linalg.norm(following - current))
            midpoint = 0.5 * (previous + following)
            candidates = [midpoint, 0.5 * (current + midpoint), 0.25 * current + 0.75 * midpoint]
            for _ in range(max(0, config.random_samples)):
                candidates.append(np.clip(midpoint + rng.normal(scale=temperature, size=current.shape), _LIMIT_LO, _LIMIT_HI))
            for candidate in candidates:
                candidate = np.clip(np.asarray(candidate, dtype=float), _LIMIT_LO, _LIMIT_HI)
                after = float(np.linalg.norm(candidate - previous) + np.linalg.norm(following - candidate))
                if after + config.improve_epsilon >= before:
                    continue
                if _segment_collision_free(previous, candidate, config.segment_step) and _segment_collision_free(
                    candidate, following, config.segment_step
                ):
                    pts[index] = candidate
                    before = after
        pts = _shortcut(pts, config.segment_step)
        temperature *= 0.85
    return pts


def _run_candidate(task: tuple[str, int, str, list[float], list[float], int, RefineConfig]) -> dict[str, Any]:
    scene, pair_index, label, start, goal, candidate_index, config = task
    _ensure_worker(scene)
    assert _ROBOT is not None and _OBSTACLES is not None

    rrt_config = sbf.RRTConnectConfig()
    rrt_config.max_iters = int(config.rrt_max_iters)
    rrt_config.timeout_ms = float(config.rrt_timeout_ms)
    rrt_config.step_size = float(config.rrt_step_size)
    rrt_config.goal_bias = float(config.rrt_goal_bias)
    rrt_config.segment_resolution = int(config.rrt_segment_resolution)

    wall_t0 = time.perf_counter()
    seed = int(config.seed_base + candidate_index)
    path = sbf.rrt_connect_path(_ROBOT, _OBSTACLES, start, goal, rrt_config, seed)
    rrt_s = time.perf_counter() - wall_t0
    if len(path) < 2:
        return {
            "pair_index": pair_index,
            "name": label,
            "candidate_index": candidate_index,
            "ok": False,
            "rrt_time_s": float(rrt_s),
            "wall_s": float(time.perf_counter() - wall_t0),
        }

    smooth_t0 = time.perf_counter()
    points = _smooth([np.asarray(point, dtype=float) for point in path], config, seed=int(2 * config.seed_base + 100 * pair_index + candidate_index))
    smooth_s = time.perf_counter() - smooth_t0
    valid = _path_collision_free(points, config.segment_step)
    length = _path_length(points) if valid else 0.0
    return {
        "pair_index": pair_index,
        "name": label,
        "candidate_index": candidate_index,
        "ok": bool(valid),
        "length": float(length),
        "waypoints": [[float(value) for value in point] for point in points] if valid else [],
        "waypoint_count": len(points) if valid else 0,
        "rrt_time_s": float(rrt_s),
        "smooth_time_s": float(smooth_s),
        "wall_s": float(time.perf_counter() - wall_t0),
    }


def refine_path_rows(rows: list[dict[str, Any]], scene: str, config: RefineConfig) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "enabled": bool(config.enabled),
        "method": "parallel_rrt_connect_collision_smoothing",
        "candidate_count_per_query": int(config.candidates),
        "workers": int(config.workers),
        "wall_s": 0.0,
        "improved_count": 0,
        "successful_candidate_count": 0,
    }
    eligible = [row for row in rows if bool(row.get("ok")) and len(row.get("waypoints", [])) >= 2]
    if not config.enabled or config.candidates <= 0 or not eligible:
        return summary

    before_total = sum(float(row.get("length", 0.0)) for row in eligible)
    best_by_pair: dict[int, dict[str, Any]] = {}
    stats_by_pair: dict[int, dict[str, Any]] = {}
    for row in eligible:
        pair_index = int(row["pair_idx"])
        best_by_pair[pair_index] = {
            "source": "sbf_query",
            "candidate_index": -1,
            "length": float(row["length"]),
            "waypoints": row["waypoints"],
            "waypoint_count": len(row["waypoints"]),
        }
        stats_by_pair[pair_index] = {
            "candidate_count": int(config.candidates),
            "candidate_success_count": 0,
            "candidate_wall_s_sum": 0.0,
            "candidate_wall_s_max": 0.0,
        }

    tasks = [
        (scene, int(row["pair_idx"]), str(row["name"]), list(row["start"]), list(row["goal"]), candidate_index, config)
        for row in eligible
        for candidate_index in range(int(config.candidates))
    ]

    wall_t0 = time.perf_counter()
    if config.workers <= 1:
        _ensure_worker(scene)
        results = [_run_candidate(task) for task in tasks]
    else:
        with futures.ProcessPoolExecutor(max_workers=int(config.workers), initializer=_init_worker, initargs=(scene,)) as pool:
            results = list(pool.map(_run_candidate, tasks))
    summary["wall_s"] = float(time.perf_counter() - wall_t0)

    for result in results:
        pair_index = int(result["pair_index"])
        stats = stats_by_pair[pair_index]
        stats["candidate_wall_s_sum"] += float(result.get("wall_s", 0.0))
        stats["candidate_wall_s_max"] = max(stats["candidate_wall_s_max"], float(result.get("wall_s", 0.0)))
        if not result.get("ok"):
            continue
        stats["candidate_success_count"] += 1
        summary["successful_candidate_count"] += 1
        if float(result["length"]) + config.improve_epsilon < float(best_by_pair[pair_index]["length"]):
            best_by_pair[pair_index] = {
                "source": "rrt_connect_refine",
                "candidate_index": int(result["candidate_index"]),
                "length": float(result["length"]),
                "waypoints": result["waypoints"],
                "waypoint_count": int(result["waypoint_count"]),
            }

    after_total = 0.0
    for row in eligible:
        pair_index = int(row["pair_idx"])
        best = best_by_pair[pair_index]
        original_length = float(row["length"])
        row["sbf_waypoints"] = row["waypoints"]
        row["sbf_length"] = original_length
        row["waypoints"] = best["waypoints"]
        row["length"] = float(best["length"])
        row["optimized_length"] = float(best["length"])
        row["optimized_waypoint_count"] = int(best["waypoint_count"])
        row["waypoint_count"] = int(best["waypoint_count"])
        row["postprocess"] = {
            **stats_by_pair[pair_index],
            "enabled": True,
            "best_source": best["source"],
            "best_candidate_index": int(best["candidate_index"]),
            "length_before": original_length,
            "length_after": float(best["length"]),
            "improved": bool(float(best["length"]) + config.improve_epsilon < original_length),
        }
        if row["postprocess"]["improved"]:
            summary["improved_count"] += 1
        after_total += float(best["length"])

    summary["length_before_total"] = float(before_total)
    summary["length_after_total"] = float(after_total)
    summary["length_delta_total"] = float(after_total - before_total)
    return summary