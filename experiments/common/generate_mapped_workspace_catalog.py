#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[2]
BUILD_PYTHON = REPO_ROOT / "build" / "python"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@dataclass(frozen=True)
class CSpaceBox:
    intervals: list[tuple[float, float]]


def robot_json_payload(robot_name: str) -> dict[str, Any]:
    key = str(robot_name).lower()
    if key == "iiwa":
        return json.loads((REPO_ROOT / "link_interval_envelope" / "examples" / "data" / "iiwa14.json").read_text())
    if key == "ur5":
        return {
            "name": "ur5_like_standalone",
            "dh_params": [
                {"alpha": math.pi / 2.0, "a": 0.0, "d": 0.0892, "theta": 0.0, "type": "revolute"},
                {"alpha": 0.0, "a": -0.425, "d": 0.0, "theta": 0.0, "type": "revolute"},
                {"alpha": 0.0, "a": -0.392, "d": 0.0, "theta": 0.0, "type": "revolute"},
                {"alpha": math.pi / 2.0, "a": 0.0, "d": 0.109, "theta": 0.0, "type": "revolute"},
                {"alpha": -math.pi / 2.0, "a": 0.0, "d": 0.095, "theta": 0.0, "type": "revolute"},
                {"alpha": 0.0, "a": 0.0, "d": 0.0, "theta": 0.0, "type": "revolute"},
            ],
            "joint_limits": [[-math.pi, math.pi]] * 6,
            "link_radii": [0.055, 0.055, 0.055, 0.055, 0.055, 0.0],
        }
    if key == "panda":
        return {
            "name": "panda_like_standalone",
            "dh_params": [
                {"alpha": -math.pi / 2.0, "a": 0.0, "d": 0.333, "theta": 0.0, "type": "revolute"},
                {"alpha": math.pi / 2.0, "a": 0.0, "d": 0.0, "theta": 0.0, "type": "revolute"},
                {"alpha": math.pi / 2.0, "a": 0.0, "d": 0.316, "theta": 0.0, "type": "revolute"},
                {"alpha": -math.pi / 2.0, "a": 0.0825, "d": 0.0, "theta": 0.0, "type": "revolute"},
                {"alpha": math.pi / 2.0, "a": -0.0825, "d": 0.384, "theta": 0.0, "type": "revolute"},
                {"alpha": math.pi / 2.0, "a": 0.0, "d": 0.0, "theta": 0.0, "type": "revolute"},
                {"alpha": 0.0, "a": 0.0, "d": 0.0, "theta": 0.0, "type": "revolute"},
            ],
            "joint_limits": [[-2.8, 2.8], [-1.8, 1.8], [-2.8, 2.8], [-3.0, 0.0], [-2.8, 2.8], [-0.1, 3.7], [-2.8, 2.8]],
            "link_radii": [0.055, 0.055, 0.055, 0.055, 0.055, 0.0, 0.0],
        }
    raise ValueError(f"unknown robot {robot_name!r}")


def flatten_cspace_box(box: CSpaceBox) -> list[list[float]]:
    return [[float(lo), float(hi)] for lo, hi in box.intervals]


def clip_interval(lo: float, hi: float, limit: tuple[float, float]) -> tuple[float, float] | None:
    out_lo = max(float(limit[0]), float(lo))
    out_hi = min(float(limit[1]), float(hi))
    if out_hi <= out_lo + 1e-9:
        return None
    return out_lo, out_hi


def interval_mid(limit: tuple[float, float]) -> float:
    return 0.5 * (float(limit[0]) + float(limit[1]))


def csv_ints(raw: str) -> list[int]:
    text = str(raw).strip()
    if not text:
        return []
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def parse_difficulty_windows(raw: str) -> dict[str, tuple[float, float]]:
    windows: dict[str, tuple[float, float]] = {}
    text = str(raw).strip()
    if not text:
        return windows
    for item in text.split(","):
        if not item.strip():
            continue
        name, value = item.split(":", 1)
        lo_text, hi_text = value.split("-", 1)
        lo = float(lo_text)
        hi = math.inf if hi_text.strip().lower() in {"inf", "infty", "infinite"} else float(hi_text)
        if hi <= lo:
            raise ValueError(f"invalid difficulty window {item!r}")
        windows[name.strip().lower()] = (lo, hi)
    return windows


def difficulty_median_window(difficulty: str, raw: str) -> tuple[float, float] | None:
    return parse_difficulty_windows(raw).get(str(difficulty).strip().lower())


def median_in_window(median_s: float, window: tuple[float, float] | None) -> bool:
    if window is None:
        return True
    if not math.isfinite(float(median_s)):
        return False
    lo, hi = window
    return float(median_s) >= float(lo) - 1e-12 and float(median_s) <= float(hi) + 1e-12


def prefilter_min_for_window(window: tuple[float, float] | None, default_min: float) -> float:
    if window is None:
        return float(default_min)
    lo, _hi = window
    return max(float(default_min), 0.70 * float(lo))


def resolved_allowed_link_idxs(raw: str, robot_name: str) -> list[int]:
    key = str(raw).strip().lower()
    if key and key != "auto":
        return csv_ints(raw)
    robot_key = str(robot_name).lower()
    if robot_key == "iiwa":
        return [4, 6, 7]
    if robot_key == "ur5":
        return [2, 3, 4]
    if robot_key == "panda":
        return [4]
    return [4, 6, 7]


def resolved_wall_dim(raw: int, robot_name: str, dim: int) -> int:
    if int(raw) >= 0:
        return min(max(0, int(raw)), max(0, int(dim) - 1))
    robot_key = str(robot_name).lower()
    if robot_key == "ur5":
        return min(1, max(0, int(dim) - 1))
    return min(2, max(0, int(dim) - 1))


def resolved_subbox_cap(raw: int, robot_name: str) -> int:
    if int(raw) > 0:
        return int(raw)
    robot_key = str(robot_name).lower()
    if robot_key == "ur5":
        return 80
    if robot_key == "panda":
        return 120
    return 160


def difficulty_params(robot_name: str, difficulty: str, rng: random.Random, scene_try: int) -> dict[str, float | int]:
    key = str(difficulty).lower()
    robot_key = str(robot_name).lower()
    if robot_key == "ur5":
        dim_boost = {"easy": 3, "medium": 3, "hard": 3}.get(key, 3)
        base_gap = {"easy": 0.40, "medium": 0.40, "hard": 0.38}.get(key, 0.40)
        base_wall = {"easy": 0.40, "medium": 0.40, "hard": 0.40}.get(key, 0.40)
        base_local = {"easy": 1.55, "medium": 1.55, "hard": 1.55}.get(key, 1.55)
        gate_offset = {"easy": 0.34, "medium": 0.34, "hard": 0.34}.get(key, 0.34)
        hardening = min(0.40, 0.035 * int(scene_try))
        return {
            "gate_count": dim_boost,
            "gate_width": max(0.12, base_gap * (1.0 - hardening) * rng.uniform(0.85, 1.10)),
            "wall_half_width": base_wall * (1.0 + 0.50 * hardening) * rng.uniform(0.90, 1.18),
            "local_radius": base_local * (1.0 + 0.40 * hardening) * rng.uniform(0.90, 1.20),
            "gate_offset_fraction": gate_offset,
        }
    if robot_key == "panda" and key == "medium":
        hardening = min(0.40, 0.035 * int(scene_try))
        return {
            "gate_count": 3,
            "gate_width": max(0.12, 0.38 * (1.0 - hardening) * rng.uniform(0.85, 1.10)),
            "wall_half_width": 0.40 * (1.0 + 0.50 * hardening) * rng.uniform(0.90, 1.18),
            "local_radius": 1.55 * (1.0 + 0.40 * hardening) * rng.uniform(0.90, 1.20),
            "gate_offset_fraction": 0.34,
        }
    dim_boost = {"easy": 2, "medium": 2, "hard": 3}.get(key, 2)
    base_gap = {"easy": 0.46, "medium": 0.42, "hard": 0.38}.get(key, 0.42)
    base_wall = {"easy": 0.30, "medium": 0.35, "hard": 0.40}.get(key, 0.35)
    base_local = {"easy": 1.25, "medium": 1.40, "hard": 1.55}.get(key, 1.40)
    # Failed attempts gradually make the gate tighter, but keep it feasible
    # enough for the 0.5-5.0 s BIT* probes used by Exp.6.
    hardening = min(0.40, 0.035 * int(scene_try))
    return {
        "gate_count": dim_boost,
        "gate_width": max(0.12, base_gap * (1.0 - hardening) * rng.uniform(0.85, 1.10)),
        "wall_half_width": base_wall * (1.0 + 0.50 * hardening) * rng.uniform(0.90, 1.18),
        "local_radius": base_local * (1.0 + 0.40 * hardening) * rng.uniform(0.90, 1.20),
        "gate_offset_fraction": {"easy": 0.28, "medium": 0.31, "hard": 0.34}.get(key, 0.31),
    }


def make_local_hyper_gate(
    limits: Sequence[tuple[float, float]],
    *,
    robot_name: str,
    difficulty: str,
    rng: random.Random,
    scene_try: int,
    wall_dim: int,
) -> tuple[list[float], list[float], list[CSpaceBox], dict[str, Any]]:
    dim = len(limits)
    wall_dim = min(max(0, int(wall_dim)), dim - 1)
    params = difficulty_params(robot_name, difficulty, rng, scene_try)
    gate_count = min(int(params["gate_count"]), max(1, dim - 1))
    gate_dims = [index for index in range(dim) if index != wall_dim][:gate_count]
    nominal: list[float] = []
    for lo, hi in limits:
        span = float(hi) - float(lo)
        nominal.append(rng.uniform(float(lo) + 0.25 * span, float(hi) - 0.25 * span))
    start = list(nominal)
    goal = list(nominal)
    wall_span = float(limits[wall_dim][1]) - float(limits[wall_dim][0])
    wall_delta = min(2.25, max(0.75, 0.38 * wall_span))
    start[wall_dim] = max(float(limits[wall_dim][0]) + 0.20, nominal[wall_dim] - wall_delta)
    goal[wall_dim] = min(float(limits[wall_dim][1]) - 0.20, nominal[wall_dim] + wall_delta)

    gate_centers: dict[int, float] = {}
    gate_half = 0.5 * float(params["gate_width"])
    local_radius = float(params["local_radius"])
    obstacles: list[CSpaceBox] = []
    for gate_dim in gate_dims:
        lo, hi = limits[gate_dim]
        span = float(hi) - float(lo)
        direction = 1.0 if rng.random() < 0.5 else -1.0
        gate_center = nominal[gate_dim] + direction * float(params["gate_offset_fraction"]) * span
        gate_center = min(float(hi) - gate_half - 1e-6, max(float(lo) + gate_half + 1e-6, gate_center))
        gate_centers[gate_dim] = gate_center
        start[gate_dim] = nominal[gate_dim]
        goal[gate_dim] = nominal[gate_dim]

    for gate_dim in gate_dims:
        center = gate_centers[gate_dim]
        for side_lo, side_hi in (
            (nominal[gate_dim] - local_radius, center - gate_half),
            (center + gate_half, nominal[gate_dim] + local_radius),
        ):
            clipped_side = clip_interval(side_lo, side_hi, limits[gate_dim])
            if clipped_side is None:
                continue
            intervals: list[tuple[float, float]] = []
            for dim_index, limit in enumerate(limits):
                if dim_index == wall_dim:
                    center = nominal[wall_dim]
                    interval = clip_interval(
                        center - float(params["wall_half_width"]),
                        center + float(params["wall_half_width"]),
                        limit,
                    )
                elif dim_index == gate_dim:
                    interval = clipped_side
                else:
                    radius = local_radius if dim_index in gate_dims else 0.65 * local_radius
                    interval = clip_interval(nominal[dim_index] - radius, nominal[dim_index] + radius, limit)
                if interval is None:
                    break
                intervals.append(interval)
            if len(intervals) == dim:
                obstacles.append(CSpaceBox(intervals))
    metadata = {
        "generator": "local_hyper_gate",
        "wall_dim": wall_dim,
        "gate_dims": gate_dims,
        "gate_centers": {str(k): float(v) for k, v in gate_centers.items()},
        "nominal": [float(value) for value in nominal],
        **{key: float(value) if isinstance(value, float) else int(value) for key, value in params.items()},
    }
    return start, goal, obstacles, metadata


def point_in_cspace_box(q: Sequence[float], box: CSpaceBox) -> bool:
    return all(float(lo) <= float(value) <= float(hi) for value, (lo, hi) in zip(q, box.intervals))


def make_path_collision_cells(
    limits: Sequence[tuple[float, float]],
    obstacles: Sequence[CSpaceBox],
    start: Sequence[float],
    goal: Sequence[float],
    *,
    samples: int,
    half_width: float,
    max_cells: int,
) -> list[CSpaceBox]:
    cells: list[CSpaceBox] = []
    sample_count = max(1, int(samples))
    for index in range(1, sample_count):
        q = interpolate(start, goal, index / sample_count)
        containing = next((box for box in obstacles if point_in_cspace_box(q, box)), None)
        if containing is None:
            continue
        intervals: list[tuple[float, float]] = []
        for dim_index, value in enumerate(q):
            cell_lo = max(float(limits[dim_index][0]), float(containing.intervals[dim_index][0]), float(value) - float(half_width))
            cell_hi = min(float(limits[dim_index][1]), float(containing.intervals[dim_index][1]), float(value) + float(half_width))
            if cell_hi <= cell_lo + 1e-9:
                cell_lo = max(float(limits[dim_index][0]), float(containing.intervals[dim_index][0]), float(value) - 1e-4)
                cell_hi = min(float(limits[dim_index][1]), float(containing.intervals[dim_index][1]), float(value) + 1e-4)
            if cell_hi <= cell_lo + 1e-9:
                break
            intervals.append((cell_lo, cell_hi))
        if len(intervals) == len(limits):
            cells.append(CSpaceBox(intervals))
    if len(cells) <= max(1, int(max_cells)):
        return cells
    stride = max(1, int(math.ceil(len(cells) / max(1, int(max_cells)))))
    return cells[::stride][: max(1, int(max_cells))]


def subdivide_cspace_box(box: CSpaceBox, *, max_width: float, max_boxes: int) -> list[CSpaceBox]:
    boxes = [box]
    while len(boxes) < max(1, int(max_boxes)):
        widest_index = -1
        widest_width = 0.0
        widest_box_index = -1
        for box_index, candidate in enumerate(boxes):
            for dim_index, (lo, hi) in enumerate(candidate.intervals):
                width = float(hi) - float(lo)
                if width > widest_width:
                    widest_width = width
                    widest_index = dim_index
                    widest_box_index = box_index
        if widest_index < 0 or widest_width <= float(max_width):
            break
        current = boxes.pop(widest_box_index)
        lo, hi = current.intervals[widest_index]
        mid = 0.5 * (float(lo) + float(hi))
        left = list(current.intervals)
        right = list(current.intervals)
        left[widest_index] = (float(lo), mid)
        right[widest_index] = (mid, float(hi))
        boxes.extend([CSpaceBox(left), CSpaceBox(right)])
    return boxes


def aabb_from_nested(nested: Sequence[Sequence[float]], margin: float, shrink: float = 1.0) -> list[float] | None:
    if len(nested) != 3:
        return None
    shrink_factor = max(1e-6, min(1.0, float(shrink)))
    center = [0.5 * (float(pair[0]) + float(pair[1])) for pair in nested]
    half = [0.5 * (float(pair[1]) - float(pair[0])) * shrink_factor for pair in nested]
    values = [
        center[0] - half[0] - margin,
        center[1] - half[1] - margin,
        center[2] - half[2] - margin,
        center[0] + half[0] + margin,
        center[1] + half[1] + margin,
        center[2] + half[2] + margin,
    ]
    if values[3] <= values[0] or values[4] <= values[1] or values[5] <= values[2]:
        return None
    return values


def _worker_map() -> int:
    if str(BUILD_PYTHON) not in sys.path:
        sys.path.insert(0, str(BUILD_PYTHON))
    import link_interval_envelope as lie

    payload = json.load(sys.stdin)
    robot_payload = payload["robot_json"]
    cspace_boxes = payload["cspace_boxes"]
    use_inflated = bool(payload.get("use_inflated", False))
    margin = float(payload.get("workspace_margin", 0.0))
    shrink = float(payload.get("workspace_aabb_shrink", 1.0))
    endpoint_source = str(payload.get("endpoint_source", "critsample"))
    n_samples_crit = int(payload.get("n_samples_crit", 1000))
    endpoint_threads = int(payload.get("endpoint_threads", 1))
    n_subdivisions = int(payload.get("n_subdivisions", 1))
    min_active_link_idx = int(payload.get("min_active_link_idx", 0))
    max_active_link_idx = int(payload.get("max_active_link_idx", 1_000_000))
    allowed_link_idxs = {int(value) for value in payload.get("allowed_link_idxs", [])}
    with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8", delete=False) as handle:
        json.dump(robot_payload, handle)
        robot_path = handle.name
    try:
        obstacles: list[list[float]] = []
        timing_us = 0.0
        key = "inflated_aabb" if use_inflated else "raw_aabb"
        for intervals in cspace_boxes:
            result = lie.compute_envelope(
                robot_path,
                intervals,
                endpoint_source=endpoint_source,
                envelope_type="link_iaabb",
                n_subdivisions=n_subdivisions,
                n_samples_crit=n_samples_crit,
                endpoint_threads=endpoint_threads,
                include_endpoint_iaabbs=False,
            )
            timing_us += float(result.get("timing_us", {}).get("total", 0.0))
            for link in result.get("envelope", {}).get("links", []):
                active_link_idx = int(link.get("active_link_idx", 0))
                if active_link_idx < min_active_link_idx or active_link_idx > max_active_link_idx:
                    continue
                physical_link_idx = int(link.get("link_idx", active_link_idx))
                if allowed_link_idxs and physical_link_idx not in allowed_link_idxs:
                    continue
                aabb = aabb_from_nested(link.get(key, []), margin, shrink)
                if aabb is not None:
                    obstacles.append(aabb)
        print(json.dumps({"obstacles": obstacles, "envelope_time_us": timing_us}))
    finally:
        Path(robot_path).unlink(missing_ok=True)
    return 0


def map_cspace_boxes_to_workspace(
    *,
    robot_name: str,
    boxes: Sequence[CSpaceBox],
    use_inflated: bool,
    workspace_margin: float,
    endpoint_source: str,
    n_samples_crit: int,
    endpoint_threads: int,
    n_subdivisions: int,
    workspace_aabb_shrink: float,
    min_active_link_idx: int,
    max_active_link_idx: int,
    allowed_link_idxs: Sequence[int],
) -> dict[str, Any]:
    payload = {
        "robot_json": robot_json_payload(robot_name),
        "cspace_boxes": [flatten_cspace_box(box) for box in boxes],
        "use_inflated": bool(use_inflated),
        "workspace_margin": float(workspace_margin),
        "endpoint_source": str(endpoint_source),
        "n_samples_crit": int(n_samples_crit),
        "endpoint_threads": int(endpoint_threads),
        "n_subdivisions": int(n_subdivisions),
        "workspace_aabb_shrink": float(workspace_aabb_shrink),
        "min_active_link_idx": int(min_active_link_idx),
        "max_active_link_idx": int(max_active_link_idx),
        "allowed_link_idxs": [int(value) for value in allowed_link_idxs],
    }
    env = dict()
    env.update({"PYTHONPATH": str(BUILD_PYTHON)})
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), "--worker-map"],
        input=json.dumps(payload),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=str(REPO_ROOT),
        env={**dict(**__import__("os").environ), **env},
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"workspace envelope mapping failed:\n{result.stderr}")
    return json.loads(result.stdout)


def dedupe_obstacles(obstacles: Sequence[Sequence[float]], precision: int = 5) -> list[list[float]]:
    seen: set[tuple[float, ...]] = set()
    out: list[list[float]] = []
    for obstacle in obstacles:
        values = [float(value) for value in obstacle]
        key = tuple(round(value, precision) for value in values)
        if key in seen:
            continue
        seen.add(key)
        out.append(values)
    return out


def l2(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def interpolate(a: Sequence[float], b: Sequence[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def strict_path_collision_free(sbf: Any, robot: Any, obstacles: Sequence[Any], path: Sequence[Sequence[float]], step: float) -> bool:
    if len(path) < 2:
        return False
    for a, b in zip(path, path[1:]):
        distance = l2(a, b)
        samples = max(1, int(math.ceil(distance / max(1e-9, float(step)))))
        for index in range(samples + 1):
            if sbf.check_config_collision(robot, list(obstacles), interpolate(a, b, index / samples)):
                return False
    return True


def direct_obstruction_fraction(sbf: Any, robot: Any, obstacles: Sequence[Any], start: Sequence[float], goal: Sequence[float], samples: int) -> float:
    hits = 0
    total = max(1, int(samples)) + 1
    for index in range(total):
        if sbf.check_config_collision(robot, list(obstacles), interpolate(start, goal, index / max(1, int(samples)))):
            hits += 1
    return float(hits) / float(total)


def query_pair_distance(
    start_a: Sequence[float],
    goal_a: Sequence[float],
    start_b: Sequence[float],
    goal_b: Sequence[float],
) -> float:
    forward = l2(start_a, start_b) + l2(goal_a, goal_b)
    swapped = l2(start_a, goal_b) + l2(goal_a, start_b)
    return min(float(forward), float(swapped))


def query_difficulty_close(candidate_median_s: float, reference_median_s: float, abs_tol_s: float, rel_tol: float) -> bool:
    if not math.isfinite(float(candidate_median_s)) or not math.isfinite(float(reference_median_s)):
        return False
    tolerance = max(float(abs_tol_s), float(rel_tol) * max(1e-9, abs(float(reference_median_s))))
    return abs(float(candidate_median_s) - float(reference_median_s)) <= tolerance + 1e-12


def single_obstacle_direct_hits(
    sbf: Any,
    robot: Any,
    obstacle: Any,
    start: Sequence[float],
    goal: Sequence[float],
    samples: int,
) -> int:
    hits = 0
    total = max(1, int(samples)) + 1
    for index in range(total):
        if sbf.check_config_collision(robot, [obstacle], interpolate(start, goal, index / max(1, int(samples)))):
            hits += 1
    return hits


def obstacle_volume(bounds: Sequence[float]) -> float:
    b = [float(value) for value in bounds]
    return max(0.0, b[3] - b[0]) * max(0.0, b[4] - b[1]) * max(0.0, b[5] - b[2])


def bitstar_probe_many(
    *,
    sbf: Any,
    robot: Any,
    obstacles: Sequence[Any],
    start: Sequence[float],
    goal: Sequence[float],
    planner_seeds: int,
    seed_offset: int,
    timeout_s: float,
    checkpoint_interval_s: float,
    segment_step: float,
    samples_per_batch: int,
    rewire_factor: float,
) -> dict[str, Any]:
    probes: list[dict[str, Any]] = []
    for seed_index in range(max(1, int(planner_seeds))):
        seed = int(seed_offset) + seed_index
        result = sbf.ompl_bitstar_trace(
            robot,
            list(obstacles),
            [float(value) for value in start],
            [float(value) for value in goal],
            float(timeout_s) * 1000.0,
            float(checkpoint_interval_s) * 1000.0,
            float(segment_step),
            seed,
            int(samples_per_batch),
            float(rewire_factor),
            False,
        )
        checkpoints = [dict(row) for row in result.get("checkpoints", [])]
        first: dict[str, Any] | None = None
        for checkpoint in checkpoints:
            if not bool(checkpoint.get("ok")):
                continue
            path = [[float(value) for value in point] for point in checkpoint.get("path", [])]
            if strict_path_collision_free(sbf, robot, obstacles, path, segment_step):
                first = checkpoint
                break
        first_checkpoint_s = math.nan if first is None else float(first.get("checkpoint_s", math.nan))
        first_elapsed_s = math.nan if first is None else float(first.get("elapsed_s", first_checkpoint_s))
        probes.append(
            {
                "planner": "OMPL_BITstar_trace",
                "seed": seed,
                "ok": first is not None,
                "status": str(result.get("status", "")) if first is not None else "no_strict_solution_before_timeout",
                "first_success_checkpoint_s": first_checkpoint_s,
                "first_success_elapsed_s": first_elapsed_s,
                "final_ok": bool(result.get("ok")),
                "final_solve_s": float(result.get("solve_s", math.nan)),
                "checkpoint_count": len(checkpoints),
                "success_checkpoint_count": sum(1 for row in checkpoints if bool(row.get("ok"))),
            }
        )
    times = [
        float(row["first_success_checkpoint_s"])
        for row in probes
        if bool(row.get("ok")) and math.isfinite(float(row.get("first_success_checkpoint_s", math.nan)))
    ]
    return {
        "planner": "OMPL_BITstar_trace",
        "planner_seed_count": int(planner_seeds),
        "ok_count": len(times),
        "all_success": len(times) == max(1, int(planner_seeds)),
        "median_first_success_checkpoint_s": statistics.median(times) if times else math.nan,
        "mean_first_success_checkpoint_s": sum(times) / len(times) if times else math.nan,
        "min_first_success_checkpoint_s": min(times) if times else math.nan,
        "max_first_success_checkpoint_s": max(times) if times else math.nan,
        "timeout_s": float(timeout_s),
        "checkpoint_interval_s": float(checkpoint_interval_s),
        "segment_step": float(segment_step),
        "samples_per_batch": int(samples_per_batch),
        "rewire_factor": float(rewire_factor),
        "probes": probes,
    }


def build_record(args: argparse.Namespace, robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    from experiments.common.random_scene_catalog import (
        CATALOG_SCHEMA,
        LECT_SAMPLE_DOMAIN,
        canonical_root_intervals,
        canonicalize_q,
        interval_pairs,
        make_robot,
        obstacle_from_bounds,
        q_in_intervals,
        q_in_lect_root,
        query_record,
        robot_joint_limit_intervals,
        sample_free_pair_with_canonical_record,
        sector_expanded_lect_root_intervals,
    )
    from experiments.common.rbf_defaults import robot_joint_limit_tuples
    from experiments.common.sbf_import import import_sbf

    sbf = import_sbf()
    robot = make_robot(robot_name)
    limits = robot_joint_limit_tuples(robot)
    last_reason = "not_attempted"
    robot_offset = {"iiwa": 0, "ur5": 1, "panda": 2}.get(str(robot_name).lower(), 7) * 1_000_003
    difficulty_offset = {"easy": 0, "medium": 1, "hard": 2}.get(str(difficulty).lower(), 5) * 104_729
    allowed_link_idxs = resolved_allowed_link_idxs(str(args.allowed_link_idxs), robot_name)
    wall_dim = resolved_wall_dim(int(args.wall_dim), robot_name, len(limits))
    subbox_cap = resolved_subbox_cap(int(args.max_subboxes_per_cspace_box), robot_name)
    target_window = difficulty_median_window(str(difficulty), str(args.difficulty_median_windows))
    for scene_try in range(max(1, int(args.max_scene_tries))):
        rng = random.Random(int(args.seed_base) + 1009 * int(scene_seed) + robot_offset + difficulty_offset + 104729 * scene_try)
        start, goal, cspace_obstacles, cspace_meta = make_local_hyper_gate(
            limits,
            robot_name=robot_name,
            difficulty=difficulty,
            rng=rng,
            scene_try=scene_try,
            wall_dim=wall_dim,
        )
        subboxes: list[CSpaceBox] = []
        if str(args.map_mode) == "path_cells":
            subboxes = make_path_collision_cells(
                limits,
                cspace_obstacles,
                start,
                goal,
                samples=int(args.direct_obstruction_samples),
                half_width=float(args.path_cell_half_width),
                max_cells=int(args.max_path_cells),
            )
        else:
            per_box_cap = max(1, int(subbox_cap))
            for box in cspace_obstacles:
                subboxes.extend(
                    subdivide_cspace_box(
                        box,
                        max_width=float(args.cspace_subdivide_width),
                        max_boxes=per_box_cap,
                    )
                )
        if not subboxes:
            last_reason = "no_cspace_subboxes_for_mapping"
            continue
        mapped = map_cspace_boxes_to_workspace(
            robot_name=robot_name,
            boxes=subboxes,
            use_inflated=bool(args.use_inflated_envelope),
            workspace_margin=float(args.workspace_margin),
            endpoint_source=str(args.endpoint_source),
            n_samples_crit=int(args.n_samples_crit),
            endpoint_threads=int(args.endpoint_threads),
            n_subdivisions=int(args.envelope_subdivisions),
            workspace_aabb_shrink=float(args.workspace_aabb_shrink),
            min_active_link_idx=int(args.min_active_link_idx),
            max_active_link_idx=int(args.max_active_link_idx),
            allowed_link_idxs=allowed_link_idxs,
        )
        raw_mapped_count = len(mapped.get("obstacles", []))
        obstacle_bounds = dedupe_obstacles(mapped.get("obstacles", []), precision=int(args.dedupe_precision))
        deduped_mapped_count = len(obstacle_bounds)
        endpoint_safe_bounds: list[list[float]] = []
        for bounds in obstacle_bounds:
            obstacle = obstacle_from_bounds(bounds)
            if sbf.check_config_collision(robot, [obstacle], start) or sbf.check_config_collision(robot, [obstacle], goal):
                continue
            endpoint_safe_bounds.append(bounds)
        obstacle_bounds = endpoint_safe_bounds
        if len(obstacle_bounds) > int(args.max_workspace_obstacles) > 0:
            scored_bounds: list[tuple[int, float, list[float]]] = []
            for bounds in obstacle_bounds:
                obstacle = obstacle_from_bounds(bounds)
                scored_bounds.append(
                    (
                        single_obstacle_direct_hits(
                            sbf,
                            robot,
                            obstacle,
                            start,
                            goal,
                            int(args.direct_obstruction_samples),
                        ),
                        obstacle_volume(bounds),
                        bounds,
                    )
                )
            obstacle_bounds = [
                bounds
                for _hits, _volume, bounds in sorted(
                    scored_bounds,
                    key=lambda item: (item[0], item[1]),
                    reverse=True,
                )[: int(args.max_workspace_obstacles)]
            ]
        obstacles = [obstacle_from_bounds(bounds) for bounds in obstacle_bounds]
        if not obstacles:
            last_reason = (
                "empty_mapped_obstacles"
                f"(raw={raw_mapped_count},dedup={deduped_mapped_count},endpoint_safe={len(endpoint_safe_bounds)})"
            )
            continue
        if sbf.check_config_collision(robot, obstacles, start) or sbf.check_config_collision(robot, obstacles, goal):
            last_reason = "endpoint_collision_after_mapping"
            continue
        direct_fraction = direct_obstruction_fraction(sbf, robot, obstacles, start, goal, int(args.direct_obstruction_samples))
        if direct_fraction <= 0.0:
            max_single_hits = 0
            for obstacle in obstacles:
                max_single_hits = max(
                    max_single_hits,
                    single_obstacle_direct_hits(
                        sbf,
                        robot,
                        obstacle,
                        start,
                        goal,
                        int(args.direct_obstruction_samples),
                    ),
                )
            last_reason = (
                f"direct_path_not_blocked({direct_fraction:.3f},"
                f"obs={len(obstacles)},max_single_hits={max_single_hits})"
            )
            continue
        if (
            int(args.prefilter_planner_seeds) > 0
            and int(args.planner_seeds) > int(args.prefilter_planner_seeds)
            and not bool(args.accept_first_candidate)
        ):
            prefilter = bitstar_probe_many(
                sbf=sbf,
                robot=robot,
                obstacles=obstacles,
                start=start,
                goal=goal,
                planner_seeds=int(args.prefilter_planner_seeds),
                seed_offset=int(args.seed_base) + 524287 + 8191 * int(scene_seed),
                timeout_s=float(args.prefilter_timeout_s),
                checkpoint_interval_s=float(args.bitstar_checkpoint_interval_s),
                segment_step=float(args.audit_segment_step),
                samples_per_batch=int(args.bitstar_samples_per_batch),
                rewire_factor=float(args.bitstar_rewire_factor),
            )
            prefilter_median = float(prefilter.get("median_first_success_checkpoint_s", math.nan))
            prefilter_min = prefilter_min_for_window(
                target_window if bool(args.enforce_difficulty_median_window) else None,
                float(args.prefilter_min_median_first_success_s),
            )
            if not bool(prefilter.get("all_success")):
                last_reason = f"prefilter_not_all_success({prefilter.get('ok_count')}/{args.prefilter_planner_seeds})"
                continue
            if (
                not math.isfinite(prefilter_median)
                or prefilter_median < prefilter_min - 1e-12
            ):
                last_reason = f"prefilter_median_too_fast({prefilter_median:.4f}<min{prefilter_min:.4f})"
                continue
        probe = bitstar_probe_many(
            sbf=sbf,
            robot=robot,
            obstacles=obstacles,
            start=start,
            goal=goal,
            planner_seeds=int(args.planner_seeds),
            seed_offset=int(args.seed_base) + 524287 + 8191 * int(scene_seed),
            timeout_s=float(args.bitstar_timeout_s),
            checkpoint_interval_s=float(args.bitstar_checkpoint_interval_s),
            segment_step=float(args.audit_segment_step),
            samples_per_batch=int(args.bitstar_samples_per_batch),
            rewire_factor=float(args.bitstar_rewire_factor),
        )
        median_first = float(probe.get("median_first_success_checkpoint_s", math.nan))
        q0 = query_record(label="q0", robot=robot, start=start, goal=goal, difficulty_probe=probe)
        q0["direct_obstruction_fraction"] = float(direct_fraction)
        lower_bound_passed = math.isfinite(median_first) and median_first >= float(args.min_median_first_success_s) - 1e-12
        window_passed = (
            median_in_window(median_first, target_window)
            if bool(args.enforce_difficulty_median_window)
            else True
        )
        gate_passed = bool(probe.get("all_success")) and lower_bound_passed and window_passed
        if not gate_passed and not bool(args.accept_first_candidate):
            if not bool(probe.get("all_success")):
                last_reason = f"bitstar_not_all_success({probe.get('ok_count')}/{args.planner_seeds})"
            elif not lower_bound_passed:
                last_reason = f"bitstar_median_too_fast({median_first:.4f})"
            elif not window_passed:
                last_reason = f"bitstar_median_out_of_window({median_first:.4f},target={target_window})"
            else:
                last_reason = "bitstar_gate_failed"
            continue
        q0["difficulty_match"] = {
            "reference_label": "q0",
            "reference_median_first_success_s": float(median_first),
            "median_first_success_s": float(median_first),
            "abs_delta_s": 0.0,
            "passed": True,
        }
        queries = [q0]
        query_rejections: dict[str, int] = {}
        additional_attempts = 0
        target_queries = max(1, int(args.queries_per_scene))
        if target_queries > 1:
            query_rng = random.Random(
                int(args.seed_base)
                + 1009 * int(scene_seed)
                + robot_offset
                + difficulty_offset
                + 1_299_709
                + 65_537 * int(scene_try)
            )

            def reject(reason: str) -> None:
                query_rejections[reason] = int(query_rejections.get(reason, 0)) + 1

            max_attempts = max(0, int(args.additional_query_max_tries))
            while len(queries) < target_queries and additional_attempts < max_attempts:
                additional_attempts += 1
                try:
                    cand_start, cand_goal, _cand_cstart, _cand_cgoal, shift, sector = sample_free_pair_with_canonical_record(
                        robot,
                        list(obstacles),
                        query_rng,
                        min_l2=float(args.query_min_l2),
                        max_l2=float(args.query_max_l2),
                        clearance_margin_m=float(args.query_clearance_margin_m),
                        max_tries=int(args.query_endpoint_max_tries),
                    )
                except RuntimeError:
                    reject("sample_failed")
                    continue
                if any(
                    query_pair_distance(
                        cand_start,
                        cand_goal,
                        existing["start"],
                        existing["goal"],
                    )
                    < float(args.query_pair_min_separation)
                    for existing in queries
                ):
                    reject("duplicate_or_too_close")
                    continue
                cand_direct_fraction = direct_obstruction_fraction(
                    sbf,
                    robot,
                    obstacles,
                    cand_start,
                    cand_goal,
                    int(args.direct_obstruction_samples),
                )
                if cand_direct_fraction <= 0.0:
                    reject("direct_path_not_blocked")
                    continue
                cand_probe = bitstar_probe_many(
                    sbf=sbf,
                    robot=robot,
                    obstacles=obstacles,
                    start=cand_start,
                    goal=cand_goal,
                    planner_seeds=int(args.planner_seeds),
                    seed_offset=(
                        int(args.seed_base)
                        + 1_048_573
                        + 8191 * int(scene_seed)
                        + 131_071 * int(additional_attempts)
                        + 1_000_003 * len(queries)
                    ),
                    timeout_s=float(args.bitstar_timeout_s),
                    checkpoint_interval_s=float(args.bitstar_checkpoint_interval_s),
                    segment_step=float(args.audit_segment_step),
                    samples_per_batch=int(args.bitstar_samples_per_batch),
                    rewire_factor=float(args.bitstar_rewire_factor),
                )
                cand_median = float(cand_probe.get("median_first_success_checkpoint_s", math.nan))
                if not bool(cand_probe.get("all_success")):
                    reject("bitstar_not_all_success")
                    continue
                if not math.isfinite(cand_median) or cand_median < float(args.min_median_first_success_s) - 1e-12:
                    reject("bitstar_median_too_fast")
                    continue
                if bool(args.enforce_difficulty_median_window) and not median_in_window(cand_median, target_window):
                    reject("bitstar_median_out_of_window")
                    continue
                if not query_difficulty_close(
                    cand_median,
                    median_first,
                    float(args.query_difficulty_abs_tol_s),
                    float(args.query_difficulty_rel_tol),
                ):
                    reject("difficulty_not_close_to_q0")
                    continue
                cand_record = query_record(
                    label=f"q{len(queries)}",
                    robot=robot,
                    start=cand_start,
                    goal=cand_goal,
                    symmetry_shift=shift,
                    symmetry_sector=sector,
                    difficulty_probe=cand_probe,
                )
                cand_record["direct_obstruction_fraction"] = float(cand_direct_fraction)
                cand_record["difficulty_match"] = {
                    "reference_label": "q0",
                    "reference_median_first_success_s": float(median_first),
                    "median_first_success_s": float(cand_median),
                    "abs_delta_s": abs(float(cand_median) - float(median_first)),
                    "abs_tol_s": float(args.query_difficulty_abs_tol_s),
                    "rel_tol": float(args.query_difficulty_rel_tol),
                    "passed": True,
                }
                queries.append(cand_record)
            if len(queries) < target_queries and not bool(args.accept_first_candidate):
                last_reason = (
                    f"not_enough_matching_queries({len(queries)}/{target_queries},"
                    f"attempts={additional_attempts},rejections={query_rejections})"
                )
                continue
        first_query = queries[0]
        canonical_start = [float(value) for value in first_query["canonical_start"]]
        canonical_goal = [float(value) for value in first_query["canonical_goal"]]
        record = {
            "schema": CATALOG_SCHEMA,
            "robot": str(robot_name),
            "difficulty": str(difficulty),
            "scene_seed": int(scene_seed),
            "generator_seed": int(args.seed_base) + 1009 * int(scene_seed),
            "scene_profile": "cspace_mapped_workspace_bitstar_gated",
            "queries_per_scene": int(len(queries)),
            "queries": queries,
            "start": [float(value) for value in first_query["start"]],
            "goal": [float(value) for value in first_query["goal"]],
            "canonical_start": canonical_start,
            "canonical_goal": canonical_goal,
            "symmetry_shift": int(first_query.get("symmetry_shift", 0)),
            "symmetry_sector": int(first_query.get("symmetry_sector", 0)),
            "obstacles": obstacle_bounds,
            "sample_domain": LECT_SAMPLE_DOMAIN,
            "canonical_cache": True,
            "lect_root_intervals": interval_pairs(canonical_root_intervals(robot)),
            "planning_root_intervals": interval_pairs(robot_joint_limit_intervals(robot)),
            "sector_expanded_root_intervals": interval_pairs(sector_expanded_lect_root_intervals(robot)),
            "canonical_start_in_lect_root": q_in_intervals(canonical_start, canonical_root_intervals(robot)),
            "canonical_goal_in_lect_root": q_in_intervals(canonical_goal, canonical_root_intervals(robot)),
            "actual_start_in_lect_root": q_in_lect_root(robot, start),
            "actual_goal_in_lect_root": q_in_lect_root(robot, goal),
            "endpoint_clearance_margin_m": 0.0,
            "fixed_robot_clearance_margin_m": 0.0,
            "max_query_l2": float("nan"),
            "direct_segment_blocked": True,
            "segment_resolution": int(args.direct_obstruction_samples),
            "direct_obstruction_fraction": direct_fraction,
            "query_sampling": {
                "target_queries_per_scene": int(target_queries),
                "matching_query_attempts": int(additional_attempts),
                "matching_query_rejections": query_rejections,
                "query_pair_min_separation": float(args.query_pair_min_separation),
                "query_min_l2": float(args.query_min_l2),
                "query_max_l2": float(args.query_max_l2),
                "query_clearance_margin_m": float(args.query_clearance_margin_m),
                "difficulty_reference": "q0",
                "difficulty_abs_tol_s": float(args.query_difficulty_abs_tol_s),
                "difficulty_rel_tol": float(args.query_difficulty_rel_tol),
            },
            "difficulty_median_window_s": (
                [float(target_window[0]), float(target_window[1])]
                if target_window is not None and math.isfinite(float(target_window[1]))
                else [float(target_window[0]), "inf"]
                if target_window is not None
                else None
            ),
            "source_cspace": {
                "obstacles": [{"intervals": flatten_cspace_box(box)} for box in cspace_obstacles],
                "subbox_count": len(subboxes),
                "max_subboxes_per_cspace_box": int(subbox_cap),
                "map_mode": str(args.map_mode),
                "metadata": cspace_meta,
            },
            "workspace_mapping": {
                "policy": "cspace_box_to_link_iaabb_workspace_aabb",
                "endpoint_source": str(args.endpoint_source),
                "n_samples_crit": int(args.n_samples_crit),
                "endpoint_threads": int(args.endpoint_threads),
                "use_inflated_envelope": bool(args.use_inflated_envelope),
                "workspace_margin": float(args.workspace_margin),
                "workspace_aabb_shrink": float(args.workspace_aabb_shrink),
                "min_active_link_idx": int(args.min_active_link_idx),
                "max_active_link_idx": int(args.max_active_link_idx),
                "allowed_link_idxs": list(allowed_link_idxs),
                "wall_dim": int(wall_dim),
                "envelope_subdivisions": int(args.envelope_subdivisions),
                "mapped_obstacle_count": len(obstacle_bounds),
                "envelope_time_us": float(mapped.get("envelope_time_us", math.nan)),
            },
        }
        record["gate_passed"] = bool(gate_passed)
        if not gate_passed:
            record["gate_rejection_reason"] = (
                f"bitstar_not_all_success({probe.get('ok_count')}/{args.planner_seeds})"
                if not bool(probe.get("all_success"))
                else f"bitstar_median_out_of_window({median_first:.4f},target={target_window})"
                if not window_passed
                else f"bitstar_median_too_fast({median_first:.4f})"
            )
        return record
    raise RuntimeError(f"could not generate {robot_name}/{difficulty}/{scene_seed}: {last_reason}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate Exp.6 workspace catalogs from mapped C-space gate boxes.")
    parser.add_argument("--worker-map", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "outputs" / "new_experiments" / "tro2026" / "exp06" / "mapped_workspace_catalog.json")
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=1)
    parser.add_argument("--queries-per-scene", type=int, default=10)
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--max-scene-tries", type=int, default=96)
    parser.add_argument("--accept-first-candidate", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--planner-seeds", type=int, default=8)
    parser.add_argument("--prefilter-planner-seeds", type=int, default=2)
    parser.add_argument("--prefilter-timeout-s", type=float, default=1.0)
    parser.add_argument("--prefilter-min-median-first-success-s", type=float, default=0.08)
    parser.add_argument(
        "--difficulty-median-windows",
        default="easy:0.10-0.30,medium:0.30-0.70,hard:0.70-2.00",
        help="Target BIT* first-solution median windows by difficulty, in seconds.",
    )
    parser.add_argument("--enforce-difficulty-median-window", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--min-median-first-success-s", type=float, default=0.10)
    parser.add_argument("--bitstar-timeout-s", type=float, default=5.0)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=0.005)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=5.0)
    parser.add_argument("--audit-segment-step", type=float, default=0.01)
    parser.add_argument("--direct-obstruction-samples", type=int, default=96)
    parser.add_argument("--additional-query-max-tries", type=int, default=2048)
    parser.add_argument("--query-endpoint-max-tries", type=int, default=2000)
    parser.add_argument("--query-min-l2", type=float, default=0.8)
    parser.add_argument("--query-max-l2", type=float, default=4.0)
    parser.add_argument("--query-clearance-margin-m", type=float, default=0.0)
    parser.add_argument("--query-pair-min-separation", type=float, default=0.75)
    parser.add_argument("--query-difficulty-abs-tol-s", type=float, default=0.10)
    parser.add_argument("--query-difficulty-rel-tol", type=float, default=0.50)
    parser.add_argument("--wall-dim", type=int, default=-1)
    parser.add_argument("--map-mode", choices=["path_cells", "full_subboxes"], default="full_subboxes")
    parser.add_argument("--path-cell-half-width", type=float, default=0.08)
    parser.add_argument("--max-path-cells", type=int, default=72)
    parser.add_argument("--cspace-subdivide-width", type=float, default=0.22)
    parser.add_argument("--max-subboxes-per-cspace-box", type=int, default=-1)
    parser.add_argument("--max-workspace-obstacles", type=int, default=600)
    parser.add_argument("--dedupe-precision", type=int, default=5)
    parser.add_argument("--endpoint-source", default="critsample")
    parser.add_argument("--n-samples-crit", type=int, default=1000)
    parser.add_argument("--endpoint-threads", type=int, default=1)
    parser.add_argument("--envelope-subdivisions", type=int, default=1)
    parser.add_argument("--use-inflated-envelope", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--workspace-margin", type=float, default=0.0)
    parser.add_argument("--workspace-aabb-shrink", type=float, default=0.18)
    parser.add_argument("--min-active-link-idx", type=int, default=0)
    parser.add_argument("--max-active-link-idx", type=int, default=1000000)
    parser.add_argument("--allowed-link-idxs", default="auto")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if bool(args.worker_map):
        return _worker_map()

    from experiments.common.experiment_io import environment_metadata
    from experiments.common.progress import progress
    from experiments.common.random_scene_catalog import CATALOG_SCHEMA

    robots = [item.strip() for item in str(args.robots).split(",") if item.strip()]
    difficulties = [item.strip() for item in str(args.difficulties).split(",") if item.strip()]
    keys = [(robot, difficulty, seed) for robot in robots for difficulty in difficulties for seed in range(max(1, int(args.scene_seeds)))]
    records: list[dict[str, Any]] = []
    t0 = time.perf_counter()

    def write_current_payload(partial: bool) -> None:
        payload = {
            "schema": CATALOG_SCHEMA,
            "scene_profile": "cspace_mapped_workspace_bitstar_gated",
            "robots": robots,
            "difficulties": difficulties,
            "scene_seeds": int(args.scene_seeds),
            "queries_per_scene": int(args.queries_per_scene),
            "seed_base": int(args.seed_base),
            "generation_policy": {
                "cspace_proposal": "local_hyper_gate",
                "workspace_mapping": "link_interval_envelope link_iaabb AABB obstacles",
                "gate": "all planner seeds solve; median BIT* first strict solution >= min_median_first_success_s",
                "planner_seed_count": int(args.planner_seeds),
                "min_median_first_success_s": float(args.min_median_first_success_s),
                "difficulty_median_windows": str(args.difficulty_median_windows),
                "enforce_difficulty_median_window": bool(args.enforce_difficulty_median_window),
                "prefilter_planner_seed_count": int(args.prefilter_planner_seeds),
                "prefilter_timeout_s": float(args.prefilter_timeout_s),
                "prefilter_min_median_first_success_s": float(args.prefilter_min_median_first_success_s),
                "same_scene_query_policy": {
                    "target_queries_per_scene": int(args.queries_per_scene),
                    "additional_query_max_tries": int(args.additional_query_max_tries),
                    "query_pair_min_separation": float(args.query_pair_min_separation),
                    "query_min_l2": float(args.query_min_l2),
                    "query_max_l2": float(args.query_max_l2),
                    "query_clearance_margin_m": float(args.query_clearance_margin_m),
                    "difficulty_abs_tol_s": float(args.query_difficulty_abs_tol_s),
                    "difficulty_rel_tol": float(args.query_difficulty_rel_tol),
                    "difficulty_reference": "q0 BIT* median first-success time",
                },
            },
            "bitstar_probe": {
                "timeout_s": float(args.bitstar_timeout_s),
                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                "samples_per_batch": int(args.bitstar_samples_per_batch),
                "rewire_factor": float(args.bitstar_rewire_factor),
                "audit_segment_step": float(args.audit_segment_step),
            },
            "records": records,
            "partial": bool(partial),
            "expected_records": len(keys),
            "generation_s": time.perf_counter() - t0,
            "environment": environment_metadata(),
        }
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    for robot_name, difficulty, scene_seed in progress(keys, total=len(keys), desc="mapped-catalog"):
        record = build_record(args, robot_name, difficulty, int(scene_seed))
        records.append(record)
        probe = record["queries"][0]["difficulty_probe"]
        print(
            "[mapped-catalog]",
            robot_name,
            difficulty,
            scene_seed,
            f"obs={len(record['obstacles'])}",
            f"queries={len(record.get('queries', []))}",
            f"direct={float(record['direct_obstruction_fraction']):.3f}",
            f"bitstar_median={float(probe['median_first_success_checkpoint_s']):.4f}",
            f"bitstar_range=[{float(probe['min_first_success_checkpoint_s']):.4f},{float(probe['max_first_success_checkpoint_s']):.4f}]",
            flush=True,
        )
        write_current_payload(partial=True)

    write_current_payload(partial=False)
    print({"out": str(args.out), "records": len(records), "generation_s": time.perf_counter() - t0})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
