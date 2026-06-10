from __future__ import annotations

import json
import math
import random
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

from experiments.common.experiment_io import environment_metadata
from experiments.common.rbf_defaults import robot_joint_limit_tuples
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()

ENDPOINT_CLEARANCE_MARGIN_M = 0.12
NARROW_ENDPOINT_CLEARANCE_MARGIN_M = 0.07
FIXED_ROBOT_CLEARANCE_MARGIN_M = 0.025
MAX_QUERY_L2 = 4.0
SEGMENT_RESOLUTION = 96
DIRECT_OBSTRUCTION_MIN_OBSTACLES = 2
DIRECT_OBSTRUCTION_MIN_HITS_PER_OBSTACLE = 5
DIRECT_OBSTRUCTION_MIN_TOTAL_HITS = 10
BALANCED_PROBE_TIMEOUT_MS = 250.0
BALANCED_PROBE_RANGE = 0.35
BALANCED_PROBE_SEGMENT_STEP = 0.06
BALANCED_PROBE_SIMPLIFY_TIME_S = 0.0
TIMED_PROBE_RANGE = 0.35
TIMED_PROBE_SEGMENT_STEP = 0.01
TIMED_PROBE_SIMPLIFY_TIME_S = 0.0
BITSTAR_PROBE_TIMEOUT_S = 0.50
BITSTAR_PROBE_CHECKPOINT_INTERVAL_S = 0.005
BITSTAR_PROBE_MIN_FIRST_SUCCESS_S = 0.10
BITSTAR_PROBE_SAMPLES_PER_BATCH = 100
BITSTAR_PROBE_REWIRE_FACTOR = 5.0
TIMED_PROBE_TIMEOUT_WINDOWS_S = {
    "easy": (0.0, 0.003),
    "medium": (0.003, 0.007),
    "hard": (0.007, 0.050),
}
BITSTAR_MEDIAN_FIRST_SOLUTION_WINDOWS_S = {
    "easy": (0.0, 0.05),
    "medium": (0.05, 0.10),
    "hard": (0.10, BITSTAR_PROBE_TIMEOUT_S),
}
INCREMENTAL_DIFFICULTY_MAX_ADDED_OBSTACLES = {
    "easy": 0,
    "medium": 48,
    "hard": 80,
}
INCREMENTAL_DIFFICULTY_PROBE_SEED_COUNT = 1
NARROW_PASSAGE_TIMEOUT_WINDOWS_S = {
    "easy": (0.02, 0.10),
    "medium": (0.10, 0.40),
    "hard": (0.40, 1.50),
}
DIRECT_OBSTRUCTION_FRACTION_WINDOWS = {
    "easy": (0.02, 0.16),
    "medium": (0.10, 0.35),
    "hard": (0.22, 1.01),
}
BITSTAR_GATED_OBSTACLE_COUNTS = {"easy": 8, "medium": 12, "hard": 16}
BITSTAR_GATED_OBSTRUCTION_TARGETS = {"easy": 0.18, "medium": 0.28, "hard": 0.40}
BITSTAR_GATED_ENDPOINT_CLEARANCE_MARGIN_M = 0.0
DEFAULT_RANDOM_ROBOTS = "iiwa,ur5,panda"
DEFAULT_RANDOM_DIFFICULTIES = "easy,medium,hard"
DEFAULT_RANDOM_SCENE_SEEDS = 50
DEFAULT_QUERIES_PER_SCENE = 10
RANDOM_DIFFICULTY_ORDER = ("easy", "medium", "hard")
RANDOM_OBSTACLE_COUNTS = {"easy": 4, "medium": 10, "hard": 16}
RANDOM_OBSTACLE_SCALES = {"easy": 0.12, "medium": 0.20, "hard": 0.26}
NARROW_OBSTACLE_COUNTS = {"easy": 4, "medium": 8, "hard": 12}
RANDOM_WORKSPACE_Z_MIN = 0.05
RANDOM_WORKSPACE_Z_MAX = 0.90
CANONICAL_SYMMETRY_DESCRIPTOR = "joint_symmetry_native_v1"
CATALOG_SCHEMA = "tro2026_random_scene_catalog_v7"
PREFIX_DISTRIBUTION_CATALOG_SCHEMA = "exp06_distribution_prefix_catalog_v1"
READABLE_CATALOG_SCHEMAS = {
    CATALOG_SCHEMA,
    PREFIX_DISTRIBUTION_CATALOG_SCHEMA,
    "tro2026_random_scene_catalog_v6",
    "tro2026_random_scene_catalog_v5",
}
LECT_SAMPLE_DOMAIN = "full_robot_joint_limits"


@dataclass(frozen=True)
class SceneSpec:
    robot_name: str
    difficulty: str
    obstacles: list[Any]
    start: list[float]
    goal: list[float]
    canonical_start: list[float] | None = None
    canonical_goal: list[float] | None = None
    symmetry_shift: int = 0
    symmetry_sector: int = 0
    endpoint_clearance_margin_m: float = ENDPOINT_CLEARANCE_MARGIN_M
    fixed_robot_clearance_margin_m: float = FIXED_ROBOT_CLEARANCE_MARGIN_M
    direct_segment_blocked: bool = True
    segment_resolution: int = SEGMENT_RESOLUTION


def make_dh(alpha: float, a: float, d: float = 0.0, theta: float = 0.0, joint_type: int = 0) -> Any:
    dh = sbf.DHParam()
    dh.alpha = float(alpha)
    dh.a = float(a)
    dh.d = float(d)
    dh.theta = float(theta)
    dh.joint_type = int(joint_type)
    return dh


def make_limits(bounds: Sequence[tuple[float, float]]) -> Any:
    limits = sbf.JointLimits()
    limits.limits = [sbf.Interval(lo, hi) for lo, hi in bounds]
    return limits


def make_iiwa_robot() -> Any:
    from sbf.marcucci import load_iiwa14_robot

    return load_iiwa14_robot()


def make_ur5_like_robot() -> Any:
    dh = [
        make_dh(math.pi / 2.0, 0.0, 0.0892),
        make_dh(0.0, -0.425, 0.0),
        make_dh(0.0, -0.392, 0.0),
        make_dh(math.pi / 2.0, 0.0, 0.109),
        make_dh(-math.pi / 2.0, 0.0, 0.095),
        make_dh(0.0, 0.0, 0.0),
    ]
    limits = make_limits([(-math.pi, math.pi)] * 6)
    return sbf.Robot("ur5_like_standalone", dh, limits, None, [0.055, 0.055, 0.055, 0.055, 0.055, 0.0])


def make_panda_like_robot() -> Any:
    dh = [
        make_dh(-math.pi / 2.0, 0.0, 0.333),
        make_dh(math.pi / 2.0, 0.0, 0.0),
        make_dh(math.pi / 2.0, 0.0, 0.316),
        make_dh(-math.pi / 2.0, 0.0825, 0.0),
        make_dh(math.pi / 2.0, -0.0825, 0.384),
        make_dh(math.pi / 2.0, 0.0, 0.0),
        make_dh(0.0, 0.0, 0.0),
    ]
    limits = make_limits([(-2.8, 2.8), (-1.8, 1.8), (-2.8, 2.8), (-3.0, 0.0), (-2.8, 2.8), (-0.1, 3.7), (-2.8, 2.8)])
    return sbf.Robot("panda_like_standalone", dh, limits, None, [0.055, 0.055, 0.055, 0.055, 0.055, 0.0, 0.0])


def make_robot(name: str) -> Any:
    if name == "iiwa":
        return make_iiwa_robot()
    if name == "ur5":
        return make_ur5_like_robot()
    if name == "panda":
        return make_panda_like_robot()
    raise ValueError(f"unknown robot {name!r}")


def make_aabb(cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> Any:
    return sbf.Obstacle(cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz)


def obstacle_bounds(obstacle: Any) -> list[float]:
    if hasattr(obstacle, "bounds"):
        return [float(value) for value in list(obstacle.bounds)]
    values = list(obstacle)
    if len(values) != 6:
        raise ValueError(f"invalid obstacle bounds: {obstacle!r}")
    return [float(value) for value in values]


def obstacle_from_bounds(bounds: Sequence[float]) -> Any:
    values = [float(value) for value in bounds]
    if len(values) != 6:
        raise ValueError(f"invalid obstacle bounds: {bounds!r}")
    return sbf.Obstacle(*values)


def normalize_obstacles(obstacles: Iterable[Any]) -> list[Any]:
    return [obstacle if hasattr(obstacle, "bounds") else obstacle_from_bounds(obstacle) for obstacle in obstacles]


def inflate_obstacle(obstacle: Any, margin: float) -> Any:
    b = obstacle_bounds(obstacle)
    return sbf.Obstacle(b[0] - margin, b[1] - margin, b[2] - margin, b[3] + margin, b[4] + margin, b[5] + margin)


def aabb_overlaps(lhs: Any, rhs: Any, margin: float = 0.0) -> bool:
    a = obstacle_bounds(lhs)
    b = obstacle_bounds(rhs)
    return not (
        a[3] < b[0] - margin or a[0] > b[3] + margin
        or a[4] < b[1] - margin or a[1] > b[4] + margin
        or a[5] < b[2] - margin or a[2] > b[5] + margin
    )


def fixed_robot_exclusion_zones(robot_name: str) -> list[Any]:
    if robot_name == "iiwa":
        return [make_aabb(0.0, 0.0, 0.30, 0.26, 0.26, 0.40)]
    if robot_name == "ur5":
        return [make_aabb(0.0, 0.0, 0.07, 0.22, 0.22, 0.18)]
    if robot_name == "panda":
        return [make_aabb(0.0, 0.0, 0.18, 0.24, 0.24, 0.30)]
    return []


def obstacle_clears_fixed_robot(robot_name: str, obstacle: Any, margin: float = FIXED_ROBOT_CLEARANCE_MARGIN_M) -> bool:
    return all(not aabb_overlaps(obstacle, zone, margin) for zone in fixed_robot_exclusion_zones(robot_name))


def obstacles_clear_fixed_robot(robot_name: str, obstacles: list[Any], margin: float = FIXED_ROBOT_CLEARANCE_MARGIN_M) -> bool:
    return all(obstacle_clears_fixed_robot(robot_name, obstacle, margin) for obstacle in obstacles)


def random_obstacle_count(difficulty: str) -> int:
    return RANDOM_OBSTACLE_COUNTS.get(difficulty, RANDOM_OBSTACLE_COUNTS["medium"])


def narrow_obstacle_count(difficulty: str) -> int:
    return NARROW_OBSTACLE_COUNTS.get(difficulty, NARROW_OBSTACLE_COUNTS["medium"])


def random_obstacle_scale(difficulty: str) -> float:
    return RANDOM_OBSTACLE_SCALES.get(difficulty, RANDOM_OBSTACLE_SCALES["medium"])


def random_workspace_obstacle(rng: random.Random, base: float) -> Any:
    hx = rng.uniform(base * 0.5, base)
    hy = rng.uniform(base * 0.5, base)
    hz = rng.uniform(base * 0.5, base * 1.3)
    cx = rng.uniform(-0.55, 0.75)
    cy = rng.uniform(-0.65, 0.65)
    cz = rng.uniform(RANDOM_WORKSPACE_Z_MIN + hz, RANDOM_WORKSPACE_Z_MAX - hz)
    return make_aabb(cx, cy, cz, hx, hy, hz)


def random_narrow_workspace_obstacle(rng: random.Random, base: float) -> Any:
    kind = rng.randrange(4)
    if kind == 0:
        hx = rng.uniform(base * 0.25, base * 0.55)
        hy = rng.uniform(base * 1.6, base * 3.0)
        hz = rng.uniform(base * 1.3, base * 2.5)
    elif kind == 1:
        hx = rng.uniform(base * 1.6, base * 3.0)
        hy = rng.uniform(base * 0.25, base * 0.55)
        hz = rng.uniform(base * 1.3, base * 2.5)
    elif kind == 2:
        hx = rng.uniform(base * 1.0, base * 2.0)
        hy = rng.uniform(base * 1.0, base * 2.0)
        hz = rng.uniform(base * 0.35, base * 0.8)
    else:
        hx = rng.uniform(base * 0.8, base * 1.7)
        hy = rng.uniform(base * 0.8, base * 1.7)
        hz = rng.uniform(base * 0.8, base * 1.7)
    hx = min(hx, 0.55)
    hy = min(hy, 0.55)
    hz = min(hz, 0.36)
    cx = rng.uniform(-0.55, 0.75)
    cy = rng.uniform(-0.65, 0.65)
    cz = rng.uniform(RANDOM_WORKSPACE_Z_MIN + hz, RANDOM_WORKSPACE_Z_MAX - hz)
    return make_aabb(cx, cy, cz, hx, hy, hz)


def random_obstacle_near_workspace_path_config(rng: random.Random, base: float) -> Any:
    hx = rng.uniform(max(0.025, base * 0.12), max(0.08, base * 0.45))
    hy = rng.uniform(max(0.025, base * 0.12), max(0.08, base * 0.45))
    hz = rng.uniform(max(0.025, base * 0.12), max(0.08, base * 0.45))
    cx = rng.uniform(-0.85, 0.90)
    cy = rng.uniform(-0.85, 0.85)
    cz = rng.uniform(0.02 + hz, 1.00 - hz)
    return make_aabb(cx, cy, cz, hx, hy, hz)


def random_obstacle_colliding_with_path_config(
    robot_name: str,
    robot: Any,
    start: list[float],
    goal: list[float],
    rng: random.Random,
    base: float,
    endpoint_clearance_margin_m: float,
    attempts: int = 512,
) -> Any | None:
    for _ in range(max(1, int(attempts))):
        alpha = rng.uniform(0.18, 0.82)
        q = interpolate(start, goal, alpha)
        candidate = random_obstacle_near_workspace_path_config(rng, base)
        if not obstacle_clears_fixed_robot(robot_name, candidate):
            continue
        if not sbf.check_config_collision(robot, [candidate], q):
            continue
        if not endpoint_pair_has_clearance(robot, [candidate], start, goal, margin=endpoint_clearance_margin_m):
            continue
        return candidate
    return None


def random_query_wall_obstacle_colliding_with_path_config(
    robot_name: str,
    robot: Any,
    start: list[float],
    goal: list[float],
    rng: random.Random,
    base: float,
    endpoint_clearance_margin_m: float,
    attempts: int = 512,
) -> Any | None:
    for _ in range(max(1, int(attempts))):
        alpha = rng.uniform(0.14, 0.86)
        q = interpolate(start, goal, alpha)
        if rng.random() < 0.75:
            candidate = random_narrow_workspace_obstacle(rng, max(float(base), 0.22))
        else:
            candidate = random_workspace_obstacle(rng, max(float(base), 0.22))
        if not obstacle_clears_fixed_robot(robot_name, candidate):
            continue
        if not sbf.check_config_collision(robot, [candidate], q):
            continue
        if not endpoint_pair_has_clearance(robot, [candidate], start, goal, margin=endpoint_clearance_margin_m):
            continue
        return candidate
    return None


def random_query_wall_obstacle_colliding_with_config(
    robot_name: str,
    robot: Any,
    q: list[float],
    start: list[float],
    goal: list[float],
    rng: random.Random,
    base: float,
    endpoint_clearance_margin_m: float,
    attempts: int = 512,
) -> Any | None:
    for _ in range(max(1, int(attempts))):
        if rng.random() < 0.80:
            candidate = random_narrow_workspace_obstacle(rng, max(float(base), 0.24))
        else:
            candidate = random_workspace_obstacle(rng, max(float(base), 0.24))
        if not obstacle_clears_fixed_robot(robot_name, candidate):
            continue
        if not sbf.check_config_collision(robot, [candidate], q):
            continue
        if not endpoint_pair_has_clearance(robot, [candidate], start, goal, margin=endpoint_clearance_margin_m):
            continue
        return candidate
    return None


def obstacle_prefix(obstacles: list[Any], difficulty: str) -> list[Any]:
    return list(obstacles[:random_obstacle_count(difficulty)])


def canonical_root_intervals(robot: Any) -> list[Any]:
    if hasattr(sbf, "canonical_root_intervals_for_robot"):
        return list(sbf.canonical_root_intervals_for_robot(robot, True, CANONICAL_SYMMETRY_DESCRIPTOR))
    return list(robot.joint_limits().limits)


def robot_joint_limit_intervals(robot: Any) -> list[Any]:
    return [sbf.Interval(lo, hi) for lo, hi in robot_joint_limit_tuples(robot)]


def robot_name_from_robot(robot: Any) -> str:
    raw_name = getattr(robot, "name", "")
    name = str(raw_name() if callable(raw_name) else raw_name)
    lname = name.lower()
    if "iiwa" in lname:
        return "iiwa"
    if "ur5" in lname:
        return "ur5"
    if "panda" in lname:
        return "panda"
    return lname


def base_lect_root_intervals(robot: Any) -> list[Any]:
    return robot_joint_limit_intervals(robot)


def sector_expanded_lect_root_intervals(robot: Any) -> list[Any]:
    return robot_joint_limit_intervals(robot)


def interval_pairs(intervals: Iterable[Any]) -> list[list[float]]:
    return [[float(interval.lo), float(interval.hi)] for interval in intervals]


def q_in_intervals(q: Sequence[float], intervals: Sequence[Any], tol: float = 1e-9) -> bool:
    if len(q) != len(intervals):
        return False
    return all(float(interval.lo) - tol <= float(value) <= float(interval.hi) + tol for value, interval in zip(q, intervals))


def q_close(lhs: Sequence[float], rhs: Sequence[float], tol: float = 1e-8) -> bool:
    return len(lhs) == len(rhs) and all(abs(float(a) - float(b)) <= tol for a, b in zip(lhs, rhs))


def q_in_lect_root(robot: Any, q: Sequence[float]) -> bool:
    return q_in_intervals(q, sector_expanded_lect_root_intervals(robot))


def q_in_base_lect_root(robot: Any, q: Sequence[float]) -> bool:
    return q_in_intervals(q, base_lect_root_intervals(robot))


def primary_symmetry_dim_and_period(robot: Any) -> tuple[int | None, float]:
    roots = canonical_root_intervals(robot)
    limits = list(robot.joint_limits().limits)
    for index, (root, limit) in enumerate(zip(roots, limits)):
        root_width = float(root.hi) - float(root.lo)
        limit_width = float(limit.hi) - float(limit.lo)
        if root_width > 1e-12 and root_width + 1e-9 < limit_width:
            return index, root_width
    return None, 0.0


def valid_symmetry_shifts(robot: Any, canonical_q: Sequence[float] | None = None) -> list[int]:
    dim, period = primary_symmetry_dim_and_period(robot)
    if dim is None:
        return [0]
    limits = list(robot.joint_limits().limits)
    roots = base_lect_root_intervals(robot)
    root_lo = float(roots[dim].lo)
    root_hi = float(roots[dim].hi)
    lo = float(limits[dim].lo)
    hi = float(limits[dim].hi)
    shifts: list[int] = []
    for shift in range(-16, 17):
        sec_lo = root_lo + float(shift) * period
        sec_hi = root_hi + float(shift) * period
        if sec_hi >= lo - 1e-9 and sec_lo <= hi + 1e-9:
            if canonical_q is None or q_in_lect_root(robot, reflect_canonical_q(robot, canonical_q, shift)):
                shifts.append(shift)
    return shifts


def reflect_canonical_q(robot: Any, canonical_q: Sequence[float], shift: int) -> list[float]:
    out = [float(value) for value in canonical_q]
    dim, period = primary_symmetry_dim_and_period(robot)
    if dim is not None:
        out[dim] = out[dim] + int(shift) * float(period)
    return out


def reflect_canonical_pair(
    robot: Any,
    canonical_start: Sequence[float],
    canonical_goal: Sequence[float],
    rng: random.Random,
) -> tuple[list[float], list[float], int, int]:
    common = sorted(set(valid_symmetry_shifts(robot, canonical_start)).intersection(valid_symmetry_shifts(robot, canonical_goal)))
    if not common:
        raise RuntimeError("canonical query pair has no common inverse symmetry section")
    shift = int(common[rng.randrange(len(common))])
    sector = shift % 4
    return (
        reflect_canonical_q(robot, canonical_start, shift),
        reflect_canonical_q(robot, canonical_goal, shift),
        shift,
        sector,
    )


def sample_reflected_lect_q(robot: Any, rng: random.Random) -> tuple[list[float], list[float], int, int]:
    canonical = sample_q(robot, rng, canonical=True)
    shifts = valid_symmetry_shifts(robot, canonical)
    shift = int(shifts[rng.randrange(len(shifts))])
    actual = reflect_canonical_q(robot, canonical, shift)
    return actual, canonical, shift, shift % 4


def sample_reflected_lect_q_in_shift(robot: Any, rng: random.Random, shift: int) -> tuple[list[float], list[float], int, int]:
    for _ in range(128):
        canonical = sample_q(robot, rng, canonical=True)
        if int(shift) in valid_symmetry_shifts(robot, canonical):
            return reflect_canonical_q(robot, canonical, int(shift)), canonical, int(shift), int(shift) % 4
    raise RuntimeError("could not sample a reflected q in requested symmetry section")


def canonicalize_q(robot: Any, q: list[float]) -> list[float]:
    if hasattr(sbf, "canonicalize_configuration_for_robot"):
        return [
            float(value)
            for value in sbf.canonicalize_configuration_for_robot(
                robot,
                list(q),
                True,
                CANONICAL_SYMMETRY_DESCRIPTOR,
            )
        ]
    return [float(value) for value in q]


def sample_q(robot: Any, rng: random.Random, *, canonical: bool = True) -> list[float]:
    intervals = list(robot.joint_limits().limits)
    q = [rng.uniform(interval.lo, interval.hi) for interval in intervals]
    return canonicalize_q(robot, q) if canonical else [float(value) for value in q]


def config_has_clearance(robot: Any, obstacles: list[Any], q: list[float], margin: float) -> bool:
    inflated = [inflate_obstacle(obstacle, margin) for obstacle in obstacles]
    return not sbf.check_config_collision(robot, inflated, q)


def endpoint_pair_has_clearance(
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    margin: float = ENDPOINT_CLEARANCE_MARGIN_M,
) -> bool:
    return config_has_clearance(robot, obstacles, start, margin) and config_has_clearance(robot, obstacles, goal, margin)


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def segment_is_collision_free(robot: Any, obstacles: list[Any], start: list[float], goal: list[float], resolution: int = SEGMENT_RESOLUTION) -> bool:
    obstacles = normalize_obstacles(obstacles)
    steps = max(1, int(resolution))
    for index in range(steps + 1):
        if sbf.check_config_collision(robot, obstacles, interpolate(start, goal, index / steps)):
            return False
    return True


def path_is_collision_free(robot: Any, obstacles: list[Any], path: list[list[float]], resolution: int = SEGMENT_RESOLUTION) -> bool:
    if len(path) < 2:
        return False
    return all(segment_is_collision_free(robot, obstacles, path[index], path[index + 1], resolution) for index in range(len(path) - 1))


def direct_obstruction_probe(robot: Any, obstacles: list[Any], start: list[float], goal: list[float], resolution: int = SEGMENT_RESOLUTION) -> dict[str, Any]:
    obstacles = normalize_obstacles(obstacles)
    steps = max(1, int(resolution))
    hits = 0
    for index in range(steps + 1):
        if sbf.check_config_collision(robot, obstacles, interpolate(start, goal, index / steps)):
            hits += 1
    return {
        "samples": int(steps + 1),
        "collision_samples": int(hits),
        "collision_fraction": float(hits) / float(steps + 1),
    }


def obstruction_window(difficulty: str) -> tuple[float, float]:
    return DIRECT_OBSTRUCTION_FRACTION_WINDOWS.get(str(difficulty).lower(), DIRECT_OBSTRUCTION_FRACTION_WINDOWS["medium"])


def scene_profile_uses_nested_prefixes(scene_profile: str) -> bool:
    return str(scene_profile).lower() in {
        "balanced",
        "balanced_independent",
        "comparable",
        "paper",
        "balanced_probe",
        "comparable_probe",
        "paper_probe",
        "timed_probe",
        "timed",
        "narrow_passage",
        "narrow",
    }


def catalog_payload_uses_nested_prefixes(payload: dict[str, Any], scene_profile: str) -> bool:
    payload_profile = str(payload.get("scene_profile", "")).lower()
    if "prefix" in payload_profile and "independent" not in payload_profile:
        return True
    for record in payload.get("records", []):
        record_profile = str(record.get("scene_profile", "")).lower()
        mapping = record.get("workspace_mapping", {})
        incremental = record.get("incremental_scene", {})
        if (
            ("prefix" in record_profile and "independent" not in record_profile)
            or "obstacle_prefix_count" in mapping
            or "ordered_obstacles" in mapping
            or bool(incremental.get("shared_query_set", False))
        ):
            return True
    profile = str(scene_profile).lower()
    if "independent" in profile:
        return False
    if scene_profile_uses_nested_prefixes(profile) or "prefix" in profile:
        return True
    return False


def scene_profile_requires_balanced_probe(scene_profile: str) -> bool:
    return str(scene_profile).lower() in {
        "balanced",
        "balanced_independent",
        "comparable",
        "paper",
        "balanced_probe",
        "comparable_probe",
        "paper_probe",
    }


def scene_profile_requires_timed_probe(scene_profile: str) -> bool:
    return str(scene_profile).lower() in {
        "bitstar_gated",
        "bitstar_gated_independent",
        "timed_probe",
        "timed_probe_independent",
        "timed",
        "timed_independent",
        "narrow_passage",
        "narrow_passage_independent",
        "narrow",
        "narrow_independent",
        "narrow_passage_strict_time",
        "narrow_passage_independent_strict_time",
        "narrow_strict_time",
        "narrow_independent_strict_time",
    }


def scene_profile_uses_narrow_construction(scene_profile: str) -> bool:
    return str(scene_profile).lower() in {
        "narrow_passage",
        "narrow_passage_independent",
        "narrow",
        "narrow_independent",
        "narrow_passage_strict_time",
        "narrow_passage_independent_strict_time",
        "narrow_strict_time",
        "narrow_independent_strict_time",
    }


def scene_profile_uses_bitstar_gated_construction(scene_profile: str) -> bool:
    return str(scene_profile).lower() in {
        "bitstar_gated",
        "bitstar_gated_independent",
    }


def scene_profile_requires_strict_time_probe(scene_profile: str) -> bool:
    return str(scene_profile).lower() in {
        "narrow_passage_strict_time",
        "narrow_passage_independent_strict_time",
        "narrow_strict_time",
        "narrow_independent_strict_time",
    }


def scene_passes_balanced_probe(robot: Any, obstacles: list[Any], start: list[float], goal: list[float], seed: int) -> bool:
    obstacles = normalize_obstacles(obstacles)
    try:
        result = sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            start,
            goal,
            BALANCED_PROBE_TIMEOUT_MS,
            BALANCED_PROBE_RANGE,
            BALANCED_PROBE_SEGMENT_STEP,
            BALANCED_PROBE_SIMPLIFY_TIME_S,
            seed,
        )
    except AttributeError:
        return True
    if not bool(result.get("ok")) or result.get("status") != "Exact solution":
        return False
    return path_is_collision_free(robot, obstacles, [list(point) for point in result.get("path", [])])


def timed_probe_window_s(difficulty: str, *, strict_time: bool = False) -> tuple[float, float]:
    windows = NARROW_PASSAGE_TIMEOUT_WINDOWS_S if strict_time else TIMED_PROBE_TIMEOUT_WINDOWS_S
    return windows.get(str(difficulty).lower(), windows["medium"])


def rrtconnect_timed_probe(
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    difficulty: str,
    strict_time: bool = False,
) -> dict[str, Any]:
    obstacles = normalize_obstacles(obstacles)
    min_s, max_s = timed_probe_window_s(difficulty, strict_time=bool(strict_time))
    obstruction = direct_obstruction_probe(robot, obstacles, start, goal, SEGMENT_RESOLUTION)
    obs_min, obs_max = obstruction_window(difficulty)
    obstruction_in_window = (
        float(obstruction["collision_fraction"]) >= float(obs_min) - 1e-12
        and float(obstruction["collision_fraction"]) < float(obs_max) + 1e-12
    )
    # For medium/hard timed scenes, the intended difficulty is primarily
    # narrow/obstructed workspace clearance. Avoid spending an OMPL probe on
    # candidates that are visibly outside the requested obstruction band.
    if (not strict_time) and str(difficulty).lower() != "easy" and not obstruction_in_window:
        return {
            "planner": "OMPL_RRTConnect",
            "ok": False,
            "status": "direct_obstruction_out_of_window",
            "solve_s": math.nan,
            "min_s": float(min_s),
            "max_s": float(max_s),
            "in_window": False,
            "accepted_by": "rejected",
            "nodes": 0,
            "range": float(TIMED_PROBE_RANGE),
            "segment_step": float(TIMED_PROBE_SEGMENT_STEP),
            "simplify_time_s": float(TIMED_PROBE_SIMPLIFY_TIME_S),
            "direct_obstruction": obstruction,
            "direct_obstruction_min": float(obs_min),
            "direct_obstruction_max": float(obs_max),
        }
    try:
        result = sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            start,
            goal,
            max_s * 1000.0,
            TIMED_PROBE_RANGE,
            TIMED_PROBE_SEGMENT_STEP,
            TIMED_PROBE_SIMPLIFY_TIME_S,
            seed,
        )
    except AttributeError:
        return {
            "planner": "OMPL_RRTConnect",
            "ok": True,
            "status": "binding_unavailable",
            "solve_s": math.nan,
            "min_s": float(min_s),
            "max_s": float(max_s),
            "in_window": True,
        }
    solve_s = float(result.get("solve_s", result.get("t_s", math.nan)) or math.nan)
    time_in_window = (
        math.isfinite(solve_s)
        and solve_s >= float(min_s) - 1e-9
        and solve_s <= float(max_s) + 1e-9
    )
    exact_ok = (
        bool(result.get("ok"))
        and str(result.get("status", "")) == "Exact solution"
        and math.isfinite(solve_s)
        and path_is_collision_free(robot, obstacles, [list(point) for point in result.get("path", [])])
    )
    accepted_by = "time_window" if exact_ok and time_in_window else ("direct_obstruction" if exact_ok and obstruction_in_window and not strict_time else "rejected")
    return {
        "planner": "OMPL_RRTConnect",
        "ok": bool(exact_ok and (time_in_window or (obstruction_in_window and not strict_time))),
        "status": str(result.get("status", "")),
        "solve_s": solve_s,
        "min_s": float(min_s),
        "max_s": float(max_s),
        "in_window": bool(time_in_window),
        "accepted_by": accepted_by,
        "nodes": int(result.get("nodes", 0) or 0),
        "range": float(TIMED_PROBE_RANGE),
        "segment_step": float(TIMED_PROBE_SEGMENT_STEP),
        "simplify_time_s": float(TIMED_PROBE_SIMPLIFY_TIME_S),
        "strict_time": bool(strict_time),
        "direct_obstruction": obstruction,
        "direct_obstruction_min": float(obs_min),
        "direct_obstruction_max": float(obs_max),
    }


def bitstar_first_solution_probe(
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    min_first_success_s: float = BITSTAR_PROBE_MIN_FIRST_SUCCESS_S,
    timeout_s: float = BITSTAR_PROBE_TIMEOUT_S,
    checkpoint_interval_s: float = BITSTAR_PROBE_CHECKPOINT_INTERVAL_S,
) -> dict[str, Any]:
    obstacles = normalize_obstacles(obstacles)
    try:
        result = sbf.ompl_bitstar_trace(
            robot,
            obstacles,
            start,
            goal,
            float(timeout_s) * 1000.0,
            float(checkpoint_interval_s) * 1000.0,
            float(TIMED_PROBE_SEGMENT_STEP),
            int(seed),
            int(BITSTAR_PROBE_SAMPLES_PER_BATCH),
            float(BITSTAR_PROBE_REWIRE_FACTOR),
            False,
        )
    except AttributeError:
        return {
            "planner": "OMPL_BITstar_trace",
            "ok": False,
            "status": "binding_unavailable",
            "timeout_s": float(timeout_s),
            "checkpoint_interval_s": float(checkpoint_interval_s),
            "min_first_success_s": float(min_first_success_s),
        }

    checkpoints = [dict(item) for item in result.get("checkpoints", [])]
    first: dict[str, Any] | None = None
    for checkpoint in checkpoints:
        if not bool(checkpoint.get("ok")):
            continue
        path = [list(point) for point in checkpoint.get("path", [])]
        if path_is_collision_free(robot, obstacles, path):
            first = checkpoint
            break

    first_checkpoint_s = math.nan
    first_elapsed_s = math.nan
    if first is not None:
        first_checkpoint_s = float(first.get("checkpoint_s", math.nan))
        first_elapsed_s = float(first.get("elapsed_s", first_checkpoint_s))
    time_ok = math.isfinite(first_checkpoint_s) and first_checkpoint_s >= float(min_first_success_s) - 1e-12
    return {
        "planner": "OMPL_BITstar_trace",
        "ok": bool(time_ok),
        "status": str(result.get("status", "")) if first is not None else "no_valid_solution_before_timeout",
        "timeout_s": float(timeout_s),
        "checkpoint_interval_s": float(checkpoint_interval_s),
        "segment_step": float(TIMED_PROBE_SEGMENT_STEP),
        "seed": int(seed),
        "samples_per_batch": int(BITSTAR_PROBE_SAMPLES_PER_BATCH),
        "rewire_factor": float(BITSTAR_PROBE_REWIRE_FACTOR),
        "min_first_success_s": float(min_first_success_s),
        "first_success_checkpoint_s": first_checkpoint_s,
        "first_success_elapsed_s": first_elapsed_s,
        "checkpoint_count": len(checkpoints),
        "success_checkpoint_count": sum(1 for row in checkpoints if bool(row.get("ok"))),
    }


def bitstar_median_window_s(difficulty: str) -> tuple[float, float]:
    return BITSTAR_MEDIAN_FIRST_SOLUTION_WINDOWS_S.get(
        str(difficulty).lower(),
        BITSTAR_MEDIAN_FIRST_SOLUTION_WINDOWS_S["medium"],
    )


def rrtconnect_first_solution_probe(
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    timeout_s: float,
) -> dict[str, Any]:
    obstacles = normalize_obstacles(obstacles)
    try:
        result = sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            start,
            goal,
            float(timeout_s) * 1000.0,
            TIMED_PROBE_RANGE,
            TIMED_PROBE_SEGMENT_STEP,
            TIMED_PROBE_SIMPLIFY_TIME_S,
            int(seed),
        )
    except AttributeError:
        return {
            "planner": "OMPL_RRTConnect",
            "ok": False,
            "status": "binding_unavailable",
            "first_success_s": math.nan,
            "timeout_s": float(timeout_s),
        }
    path = [list(point) for point in result.get("path", [])]
    ok = (
        bool(result.get("ok"))
        and str(result.get("status", "")) == "Exact solution"
        and path_is_collision_free(robot, obstacles, path)
    )
    solve_s = float(result.get("solve_s", result.get("t_s", math.nan)) or math.nan)
    return {
        "planner": "OMPL_RRTConnect",
        "ok": bool(ok and math.isfinite(solve_s)),
        "status": str(result.get("status", "")) if ok else "no_strict_solution_before_timeout",
        "first_success_s": solve_s if ok else math.nan,
        "path": path if ok else [],
        "timeout_s": float(timeout_s),
        "range": float(TIMED_PROBE_RANGE),
        "segment_step": float(TIMED_PROBE_SEGMENT_STEP),
        "simplify_time_s": float(TIMED_PROBE_SIMPLIFY_TIME_S),
        "nodes": int(result.get("nodes", 0) or 0),
    }


def median_probe_summary(
    planner: str,
    probes: list[dict[str, Any]],
    *,
    time_key: str,
    window: tuple[float, float],
) -> dict[str, Any]:
    times = [
        float(row.get(time_key, math.nan))
        for row in probes
        if bool(row.get("ok")) and math.isfinite(float(row.get(time_key, math.nan)))
    ]
    median_s = statistics.median(times) if times else math.nan
    lo, hi = window
    in_window = (
        len(times) == len(probes)
        and math.isfinite(median_s)
        and median_s >= float(lo) - 1e-12
        and median_s <= float(hi) + 1e-12
    )
    return {
        "planner": str(planner),
        "ok": bool(in_window),
        "query_count": int(len(probes)),
        "success_count": int(len(times)),
        "all_success": len(times) == len(probes),
        "median_first_success_s": float(median_s),
        "mean_first_success_s": float(sum(times) / len(times)) if times else math.nan,
        "min_first_success_s": float(min(times)) if times else math.nan,
        "max_first_success_s": float(max(times)) if times else math.nan,
        "window_s": [float(lo), float(hi)],
        "probes": probes,
    }


def scene_median_difficulty_probe(
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    seed: int,
    difficulty: str,
) -> dict[str, Any]:
    rrt = scene_rrtconnect_median_probe(robot, obstacles, queries, seed, difficulty)
    bitstar = scene_bitstar_median_probe(robot, obstacles, queries, seed, difficulty)
    return {
        "planner": "OMPL_RRTConnect+BITstar",
        "policy": "shared_query_median_first_solution_time_windows",
        "ok": bool(rrt.get("ok")) and bool(bitstar.get("ok")),
        "difficulty": str(difficulty),
        "query_count": int(len(queries)),
        "planner_seed_count": int(INCREMENTAL_DIFFICULTY_PROBE_SEED_COUNT),
        "rrtconnect": rrt,
        "bitstar": bitstar,
    }


def scene_rrtconnect_median_probe(
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    seed: int,
    difficulty: str,
) -> dict[str, Any]:
    rrt_lo, rrt_hi = timed_probe_window_s(str(difficulty), strict_time=False)
    rrt_probes: list[dict[str, Any]] = []
    for query_index, query in enumerate(queries):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        for seed_index in range(max(1, int(INCREMENTAL_DIFFICULTY_PROBE_SEED_COUNT))):
            probe_seed = int(seed) + 1009 * int(query_index) + 65537 * int(seed_index)
            rrt_probes.append(
                {
                    **rrtconnect_first_solution_probe(
                        robot,
                        obstacles,
                        start,
                        goal,
                        probe_seed,
                        timeout_s=float(rrt_hi),
                    ),
                    "query_index": int(query_index),
                    "seed_index": int(seed_index),
                }
            )
    return median_probe_summary(
        "OMPL_RRTConnect",
        rrt_probes,
        time_key="first_success_s",
        window=(rrt_lo, rrt_hi),
    )


def scene_bitstar_median_probe(
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    seed: int,
    difficulty: str,
) -> dict[str, Any]:
    bit_lo, bit_hi = bitstar_median_window_s(str(difficulty))
    bitstar_probes: list[dict[str, Any]] = []
    for query_index, query in enumerate(queries):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        for seed_index in range(max(1, int(INCREMENTAL_DIFFICULTY_PROBE_SEED_COUNT))):
            probe_seed = int(seed) + 1009 * int(query_index) + 65537 * int(seed_index)
            bitstar_probes.append(
                bitstar_first_solution_probe(
                    robot,
                    obstacles,
                    start,
                    goal,
                    probe_seed + 524287,
                    min_first_success_s=0.0,
                    timeout_s=BITSTAR_PROBE_TIMEOUT_S,
                    checkpoint_interval_s=BITSTAR_PROBE_CHECKPOINT_INTERVAL_S,
                )
            )
    return median_probe_summary(
        "OMPL_BITstar_trace",
        bitstar_probes,
        time_key="first_success_checkpoint_s",
        window=(bit_lo, bit_hi),
    )


def timed_difficulty_probe(
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    difficulty: str,
    strict_time: bool = False,
) -> dict[str, Any]:
    rrt_probe = rrtconnect_timed_probe(robot, obstacles, start, goal, seed, difficulty, strict_time=bool(strict_time))
    if not bool(rrt_probe.get("ok")):
        return rrt_probe
    bitstar_probe = bitstar_first_solution_probe(
        robot,
        obstacles,
        start,
        goal,
        int(seed) + 524287,
    )
    combined = dict(rrt_probe)
    combined["ok"] = bool(rrt_probe.get("ok")) and bool(bitstar_probe.get("ok"))
    combined["planner"] = "OMPL_RRTConnect+BITstar"
    combined["bitstar_probe"] = bitstar_probe
    combined["bitstar_first_success_checkpoint_s"] = bitstar_probe.get("first_success_checkpoint_s")
    combined["bitstar_min_first_success_s"] = float(BITSTAR_PROBE_MIN_FIRST_SUCCESS_S)
    if not bool(bitstar_probe.get("ok")):
        combined["accepted_by"] = "rejected_bitstar_first_success"
        combined["status"] = "bitstar_first_success_too_fast_or_missing"
    return combined


def scene_passes_timed_probe(
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    difficulty: str,
    strict_time: bool = False,
) -> bool:
    return bool(timed_difficulty_probe(robot, obstacles, start, goal, seed, difficulty, strict_time=bool(strict_time)).get("ok"))


def sample_free_pair(
    robot: Any,
    obstacles: list[Any],
    rng: random.Random,
    min_l2: float = 0.8,
    max_l2: float = MAX_QUERY_L2,
    clearance_margin_m: float = ENDPOINT_CLEARANCE_MARGIN_M,
    max_tries: int = 2000,
    canonical: bool = True,
) -> tuple[list[float], list[float]]:
    start: list[float] | None = None
    for _ in range(max_tries):
        q = sample_q(robot, rng, canonical=canonical)
        if not sbf.check_config_collision(robot, obstacles, q) and config_has_clearance(robot, obstacles, q, clearance_margin_m):
            start = q
            break
    if start is None:
        raise RuntimeError("could not sample a collision-free start")
    for _ in range(max_tries):
        goal = sample_q(robot, rng, canonical=canonical)
        if sbf.check_config_collision(robot, obstacles, goal):
            continue
        if not config_has_clearance(robot, obstacles, goal, clearance_margin_m):
            continue
        dist = math.sqrt(sum((a - b) * (a - b) for a, b in zip(start, goal)))
        if dist >= min_l2 and dist <= max_l2:
            return start, goal
    raise RuntimeError("could not sample a collision-free goal")


def sample_free_pair_with_canonical_record(
    robot: Any,
    obstacles: list[Any],
    rng: random.Random,
    min_l2: float = 0.8,
    max_l2: float = MAX_QUERY_L2,
    clearance_margin_m: float = ENDPOINT_CLEARANCE_MARGIN_M,
    max_tries: int = 2000,
) -> tuple[list[float], list[float], list[float], list[float], int, int]:
    start, goal = sample_free_pair(
        robot,
        obstacles,
        rng,
        min_l2=min_l2,
        max_l2=max_l2,
        clearance_margin_m=clearance_margin_m,
        max_tries=max_tries,
        canonical=False,
    )
    return start, goal, canonicalize_q(robot, start), canonicalize_q(robot, goal), 0, 0


def append_random_scene_obstacle(
    robot_name: str,
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    rng: random.Random,
    base: float,
    require_direct_blocker: bool,
    max_attempts: int = 5000,
    endpoint_clearance_margin_m: float = ENDPOINT_CLEARANCE_MARGIN_M,
) -> bool:
    for _ in range(max_attempts):
        candidate = random_workspace_obstacle(rng, base)
        if not obstacle_clears_fixed_robot(robot_name, candidate):
            continue
        proposed = [*obstacles, candidate]
        if not endpoint_pair_has_clearance(robot, proposed, start, goal, margin=endpoint_clearance_margin_m):
            continue
        if require_direct_blocker and segment_is_collision_free(robot, [candidate], start, goal):
            continue
        obstacles.append(candidate)
        return True
    return False


def append_best_obstructing_obstacle(
    robot_name: str,
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    rng: random.Random,
    base: float,
    candidate_count: int = 256,
    endpoint_clearance_margin_m: float = ENDPOINT_CLEARANCE_MARGIN_M,
) -> bool:
    current_fraction = float(direct_obstruction_probe(robot, obstacles, start, goal, SEGMENT_RESOLUTION)["collision_fraction"])
    best: tuple[float, Any] | None = None
    for _ in range(max(1, int(candidate_count))):
        if rng.random() < 0.70:
            candidate = random_obstacle_colliding_with_path_config(
                robot_name,
                robot,
                start,
                goal,
                rng,
                base,
                endpoint_clearance_margin_m,
                attempts=64,
            )
            if candidate is None:
                continue
        elif rng.random() < 0.85:
            candidate = random_narrow_workspace_obstacle(rng, base)
        else:
            candidate = random_workspace_obstacle(rng, base)
        if not obstacle_clears_fixed_robot(robot_name, candidate):
            continue
        proposed = [*obstacles, candidate]
        if not endpoint_pair_has_clearance(robot, proposed, start, goal, margin=endpoint_clearance_margin_m):
            continue
        probe = direct_obstruction_probe(robot, proposed, start, goal, SEGMENT_RESOLUTION)
        fraction = float(probe["collision_fraction"])
        gain = fraction - current_fraction
        if gain <= 0.0:
            continue
        # Prefer obstacles that enlarge the blocked interval, but keep a small
        # random tiebreaker so repeated seeds do not collapse to identical walls.
        score = gain + 1e-4 * rng.random()
        if best is None or score > best[0]:
            best = (score, candidate)
    if best is None:
        return False
    obstacles.append(best[1])
    return True


def append_path_blocker(
    robot_name: str,
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    rng: random.Random,
    base: float,
    endpoint_clearance_margin_m: float,
    attempts: int = 2048,
) -> bool:
    current_fraction = float(direct_obstruction_probe(robot, obstacles, start, goal, SEGMENT_RESOLUTION)["collision_fraction"])
    for _ in range(max(1, int(attempts))):
        candidate = random_obstacle_colliding_with_path_config(
            robot_name,
            robot,
            start,
            goal,
            rng,
            base,
            endpoint_clearance_margin_m,
            attempts=1,
        )
        if candidate is None:
            continue
        proposed = [*obstacles, candidate]
        fraction = float(direct_obstruction_probe(robot, proposed, start, goal, SEGMENT_RESOLUTION)["collision_fraction"])
        if fraction <= current_fraction:
            continue
        obstacles.append(candidate)
        return True
    return False


def query_direct_obstruction_values(robot: Any, obstacles: list[Any], queries: list[dict[str, Any]]) -> list[float]:
    return [
        float(direct_obstruction_probe(robot, obstacles, list(query["start"]), list(query["goal"]), SEGMENT_RESOLUTION)["collision_fraction"])
        for query in queries
    ]


def all_query_endpoints_have_clearance(robot: Any, obstacles: list[Any], queries: list[dict[str, Any]], margin: float) -> bool:
    for query in queries:
        if not endpoint_pair_has_clearance(robot, obstacles, list(query["start"]), list(query["goal"]), margin=margin):
            return False
    return True


def append_shared_query_blocker(
    robot_name: str,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    rng: random.Random,
    base: float,
    endpoint_clearance_margin_m: float,
    attempts: int = 4096,
) -> bool:
    values = query_direct_obstruction_values(robot, obstacles, queries)
    order = sorted(range(len(queries)), key=lambda index: values[index])
    for query_index in order:
        query = queries[query_index]
        current_fraction = values[query_index]
        for _ in range(max(1, int(attempts))):
            if rng.random() < 0.80:
                candidate = random_query_wall_obstacle_colliding_with_path_config(
                    robot_name,
                    robot,
                    list(query["start"]),
                    list(query["goal"]),
                    rng,
                    base,
                    endpoint_clearance_margin_m,
                    attempts=1,
                )
            else:
                candidate = random_obstacle_colliding_with_path_config(
                    robot_name,
                    robot,
                    list(query["start"]),
                    list(query["goal"]),
                    rng,
                    base,
                    endpoint_clearance_margin_m,
                    attempts=1,
                )
            if candidate is None:
                continue
            proposed = [*obstacles, candidate]
            if not all_query_endpoints_have_clearance(robot, proposed, queries, endpoint_clearance_margin_m):
                continue
            fraction = float(direct_obstruction_probe(robot, proposed, list(query["start"]), list(query["goal"]), SEGMENT_RESOLUTION)["collision_fraction"])
            if fraction <= current_fraction:
                continue
            obstacles.append(candidate)
            return True
    return False


def append_rrt_path_blocker(
    robot_name: str,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    rrt_probe: dict[str, Any],
    rng: random.Random,
    base: float,
    endpoint_clearance_margin_m: float,
    attempts_per_probe: int = 256,
) -> bool:
    raw_probes = rrt_probe.get("probes", [])
    probes = [dict(probe) for probe in raw_probes] if isinstance(raw_probes, list) else []
    probes = [
        probe
        for probe in probes
        if bool(probe.get("ok"))
        and math.isfinite(float(probe.get("first_success_s", math.nan)))
        and len(probe.get("path", [])) >= 2
    ]
    probes.sort(key=lambda probe: float(probe.get("first_success_s", math.inf)))
    for probe in probes:
        query_index = int(probe.get("query_index", 0) or 0)
        if query_index < 0 or query_index >= len(queries):
            continue
        query = queries[query_index]
        path = [[float(value) for value in point] for point in probe.get("path", [])]
        candidate_indices = list(range(1, max(1, len(path) - 1)))
        if not candidate_indices:
            candidate_indices = [0]
        rng.shuffle(candidate_indices)
        for path_index in candidate_indices[: min(len(candidate_indices), 6)]:
            q = path[path_index]
            for _ in range(max(1, int(attempts_per_probe))):
                candidate = random_query_wall_obstacle_colliding_with_config(
                    robot_name,
                    robot,
                    q,
                    list(query["start"]),
                    list(query["goal"]),
                    rng,
                    base,
                    endpoint_clearance_margin_m,
                    attempts=1,
                )
                if candidate is None:
                    continue
                proposed = [*obstacles, candidate]
                if not all_query_endpoints_have_clearance(robot, proposed, queries, endpoint_clearance_margin_m):
                    continue
                obstacles.append(candidate)
                return True
    return False


def grow_shared_query_obstruction(
    robot_name: str,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    rng: random.Random,
    *,
    target_mean: float,
    min_added: int,
    max_added: int,
    base: float,
    endpoint_clearance_margin_m: float,
) -> int:
    added = 0
    while added < max(0, int(max_added)):
        values = query_direct_obstruction_values(robot, obstacles, queries)
        if added >= int(min_added) and sum(values) / max(1, len(values)) >= float(target_mean):
            break
        if not append_shared_query_blocker(
            robot_name,
            robot,
            obstacles,
            queries,
            rng,
            base,
            endpoint_clearance_margin_m,
        ):
            break
        added += 1
    return added


def probe_median_value(probe: dict[str, Any], planner_key: str) -> float:
    try:
        return float(probe.get(planner_key, {}).get("median_first_success_s", math.nan))
    except Exception:
        return math.nan


def probe_too_slow_for_difficulty(probe: dict[str, Any], difficulty: str) -> bool:
    rrt_median = probe_median_value(probe, "rrtconnect")
    bit_median = probe_median_value(probe, "bitstar")
    _rrt_lo, rrt_hi = timed_probe_window_s(difficulty, strict_time=False)
    _bit_lo, bit_hi = bitstar_median_window_s(difficulty)
    return (
        (math.isfinite(rrt_median) and rrt_median > float(rrt_hi) + 1e-12)
        or (math.isfinite(bit_median) and bit_median > float(bit_hi) + 1e-12)
    )


def difficulty_probe_brief(probe: dict[str, Any]) -> str:
    parts: list[str] = []
    for key in ("rrtconnect", "bitstar"):
        item = probe.get(key, {})
        if not isinstance(item, dict):
            continue
        parts.append(
            (
                f"{key}:ok={bool(item.get('ok'))},"
                f"succ={int(item.get('success_count', 0) or 0)}/{int(item.get('query_count', 0) or 0)},"
                f"median={float(item.get('median_first_success_s', math.nan)):.4g},"
                f"window={item.get('window_s')}"
            )
        )
    return "; ".join(parts) if parts else str(probe.get("status", "no_probe"))


def grow_until_shared_query_median_gate(
    robot_name: str,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    rng: random.Random,
    *,
    difficulty: str,
    seed: int,
    base: float,
    endpoint_clearance_margin_m: float,
) -> tuple[bool, dict[str, Any], int]:
    max_added = int(INCREMENTAL_DIFFICULTY_MAX_ADDED_OBSTACLES.get(str(difficulty).lower(), 16))
    added = 0
    last_probe: dict[str, Any] = {
        "planner": "OMPL_RRTConnect+BITstar",
        "policy": "shared_query_median_first_solution_time_windows",
        "ok": False,
        "difficulty": str(difficulty),
    }
    while added <= max_added:
        rrt_probe = scene_rrtconnect_median_probe(robot, obstacles, queries, int(seed) + 104729 * added, str(difficulty))
        last_probe["rrtconnect"] = rrt_probe
        if bool(rrt_probe.get("ok")):
            bitstar_probe = scene_bitstar_median_probe(robot, obstacles, queries, int(seed) + 524287 + 104729 * added, str(difficulty))
            last_probe["bitstar"] = bitstar_probe
            last_probe["ok"] = bool(bitstar_probe.get("ok"))
            last_probe["query_count"] = int(len(queries))
            last_probe["planner_seed_count"] = int(INCREMENTAL_DIFFICULTY_PROBE_SEED_COUNT)
            if bool(last_probe.get("ok")):
                break
        if probe_too_slow_for_difficulty(last_probe, str(difficulty)):
            break
        if added >= max_added:
            break
        appended = append_rrt_path_blocker(
            robot_name,
            robot,
            obstacles,
            queries,
            rrt_probe,
            rng,
            base,
            endpoint_clearance_margin_m,
        )
        if not appended:
            appended = append_shared_query_blocker(
                robot_name,
                robot,
                obstacles,
                queries,
                rng,
                base,
                endpoint_clearance_margin_m,
            )
        if not appended:
            break
        added += 1
    return bool(last_probe.get("ok")), last_probe, added


def make_narrow_passage_scene(robot_name: str, difficulty: str, seed: int, max_scene_tries: int, strict_time: bool = False) -> SceneSpec:
    robot = make_robot(robot_name)
    last_error: Exception | None = None
    target = narrow_obstacle_count(difficulty)
    base = random_obstacle_scale(difficulty)
    for scene_try in range(max(1, int(max_scene_tries))):
        rng = random.Random(seed + 1000003 * scene_try)
        obstacles: list[Any] = []
        try:
            start, goal, canonical_start, canonical_goal, shift, sector = sample_free_pair_with_canonical_record(
                robot,
                obstacles,
                rng,
                min_l2=2.0,
                clearance_margin_m=NARROW_ENDPOINT_CLEARANCE_MARGIN_M,
            )
            obs_min, obs_max = obstruction_window(difficulty)
            min_count = max(2, min(target, target // 2))
            while len(obstacles) < target:
                current_obstruction = direct_obstruction_probe(robot, obstacles, start, goal, SEGMENT_RESOLUTION)
                if not append_path_blocker(
                    robot_name,
                    robot,
                    obstacles,
                    start,
                    goal,
                    rng,
                    base,
                    endpoint_clearance_margin_m=NARROW_ENDPOINT_CLEARANCE_MARGIN_M,
                    attempts=2048,
                ):
                    if len(obstacles) >= min_count and float(current_obstruction["collision_fraction"]) >= float(obs_min):
                        break
                    raise RuntimeError("could not add an obstructing obstacle")
            if not obstacles:
                raise RuntimeError("could not add any obstructing obstacle")
            if len(obstacles) < min_count:
                raise RuntimeError("too few narrow-passage obstacles")
            if not obstacles_clear_fixed_robot(robot_name, obstacles):
                continue
            if not endpoint_pair_has_clearance(robot, obstacles, start, goal, margin=NARROW_ENDPOINT_CLEARANCE_MARGIN_M):
                continue
            if segment_is_collision_free(robot, obstacles, start, goal):
                continue
            final_obstruction = direct_obstruction_probe(robot, obstacles, start, goal, SEGMENT_RESOLUTION)
            final_fraction = float(final_obstruction["collision_fraction"])
            if final_fraction < float(obs_min) - 1e-12 or final_fraction > float(obs_max) + 1e-12:
                continue
            if strict_time and not scene_passes_timed_probe(robot, obstacles, start, goal, seed + 131071, difficulty, strict_time=True):
                continue
            return SceneSpec(
                robot_name=robot_name,
                difficulty=difficulty,
                obstacles=obstacles,
                start=start,
                goal=goal,
                canonical_start=canonical_start,
                canonical_goal=canonical_goal,
                symmetry_shift=shift,
                symmetry_sector=sector,
                endpoint_clearance_margin_m=NARROW_ENDPOINT_CLEARANCE_MARGIN_M,
            )
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"could not sample a valid {robot_name}/{difficulty} narrow-passage scene after {max_scene_tries} attempts: {last_error}")


def make_bitstar_gated_scene(robot_name: str, difficulty: str, seed: int, max_scene_tries: int) -> SceneSpec:
    robot = make_robot(robot_name)
    last_error: Exception | None = None
    difficulty_key = str(difficulty).lower()
    target_count = BITSTAR_GATED_OBSTACLE_COUNTS.get(difficulty_key, BITSTAR_GATED_OBSTACLE_COUNTS["medium"])
    target_obstruction = BITSTAR_GATED_OBSTRUCTION_TARGETS.get(difficulty_key, BITSTAR_GATED_OBSTRUCTION_TARGETS["medium"])
    base = max(random_obstacle_scale(difficulty_key), 0.20)
    for scene_try in range(max(1, int(max_scene_tries))):
        rng = random.Random(seed + 1000003 * scene_try)
        obstacles: list[Any] = []
        try:
            start, goal, canonical_start, canonical_goal, shift, sector = sample_free_pair_with_canonical_record(
                robot,
                obstacles,
                rng,
                min_l2=2.0,
                clearance_margin_m=BITSTAR_GATED_ENDPOINT_CLEARANCE_MARGIN_M,
            )
            for _ in range(target_count):
                obstruction = direct_obstruction_probe(robot, obstacles, start, goal, SEGMENT_RESOLUTION)
                if float(obstruction["collision_fraction"]) >= float(target_obstruction):
                    break
                if not append_best_obstructing_obstacle(
                    robot_name,
                    robot,
                    obstacles,
                    start,
                    goal,
                    rng,
                    base,
                    candidate_count=384,
                    endpoint_clearance_margin_m=BITSTAR_GATED_ENDPOINT_CLEARANCE_MARGIN_M,
                ):
                    if not append_random_scene_obstacle(
                        robot_name,
                        robot,
                        obstacles,
                        start,
                        goal,
                        rng,
                        max(base, 0.30),
                        require_direct_blocker=True,
                        max_attempts=4096,
                        endpoint_clearance_margin_m=BITSTAR_GATED_ENDPOINT_CLEARANCE_MARGIN_M,
                    ):
                        raise RuntimeError("could not add a BIT*-gated obstructing obstacle")
            while len(obstacles) < target_count:
                if not append_best_obstructing_obstacle(
                    robot_name,
                    robot,
                    obstacles,
                    start,
                    goal,
                    rng,
                    base,
                    candidate_count=128,
                    endpoint_clearance_margin_m=BITSTAR_GATED_ENDPOINT_CLEARANCE_MARGIN_M,
                ):
                    break
            if not obstacles_clear_fixed_robot(robot_name, obstacles):
                continue
            if not endpoint_pair_has_clearance(robot, obstacles, start, goal, margin=BITSTAR_GATED_ENDPOINT_CLEARANCE_MARGIN_M):
                continue
            if segment_is_collision_free(robot, obstacles, start, goal):
                continue
            probe = timed_difficulty_probe(robot, obstacles, start, goal, seed + 131071, difficulty_key)
            if not bool(probe.get("ok")):
                continue
            return SceneSpec(
                robot_name=robot_name,
                difficulty=difficulty,
                obstacles=obstacles,
                start=start,
                goal=goal,
                canonical_start=canonical_start,
                canonical_goal=canonical_goal,
                symmetry_shift=shift,
                symmetry_sector=sector,
                endpoint_clearance_margin_m=BITSTAR_GATED_ENDPOINT_CLEARANCE_MARGIN_M,
            )
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"could not sample a valid {robot_name}/{difficulty} BIT*-gated scene after {max_scene_tries} attempts: {last_error}")


def nested_prefixes_are_valid(
    robot: Any,
    robot_name: str,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    scene_try: int,
    balanced: bool,
    timed: bool = False,
    strict_time: bool = False,
) -> bool:
    for difficulty in RANDOM_DIFFICULTY_ORDER:
        prefix = obstacle_prefix(obstacles, difficulty)
        if not obstacles_clear_fixed_robot(robot_name, prefix):
            return False
        if not endpoint_pair_has_clearance(robot, prefix, start, goal):
            return False
        if segment_is_collision_free(robot, prefix, start, goal):
            return False
        if timed:
            difficulty_offset = RANDOM_DIFFICULTY_ORDER.index(difficulty) * 104729
            if not scene_passes_timed_probe(robot, prefix, start, goal, seed + 131071 + difficulty_offset, difficulty, strict_time=strict_time):
                return False
        if balanced:
            difficulty_offset = RANDOM_DIFFICULTY_ORDER.index(difficulty) * 104729
            if not scene_passes_balanced_probe(robot, prefix, start, goal, seed + 8191 * scene_try + difficulty_offset):
                return False
    return True


def make_nested_random_scene(robot_name: str, difficulty: str, seed: int, max_scene_tries: int, balanced: bool, timed: bool = False, strict_time: bool = False) -> SceneSpec:
    robot = make_robot(robot_name)
    last_error: Exception | None = None
    for scene_try in range(max(1, int(max_scene_tries))):
        rng = random.Random(seed + 1000003 * scene_try)
        obstacles: list[Any] = []
        try:
            start, goal, canonical_start, canonical_goal, shift, sector = sample_free_pair_with_canonical_record(robot, obstacles, rng)
            if not append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, random_obstacle_scale("easy"), True):
                raise RuntimeError("could not add an easy direct blocker")
            for level in RANDOM_DIFFICULTY_ORDER:
                while len(obstacles) < random_obstacle_count(level):
                    if not append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, random_obstacle_scale(level), False):
                        raise RuntimeError(f"could not fill {level} obstacle prefix")
            if not nested_prefixes_are_valid(robot, robot_name, obstacles, start, goal, seed, scene_try, balanced, timed, strict_time):
                continue
            return SceneSpec(
                robot_name=robot_name,
                difficulty=difficulty,
                obstacles=obstacle_prefix(obstacles, difficulty),
                start=start,
                goal=goal,
                canonical_start=canonical_start,
                canonical_goal=canonical_goal,
                symmetry_shift=shift,
                symmetry_sector=sector,
            )
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"could not sample a valid {robot_name}/{difficulty} scene after {max_scene_tries} attempts: {last_error}")


def make_legacy_random_scene(robot_name: str, difficulty: str, seed: int, max_scene_tries: int, balanced: bool, timed: bool = False, strict_time: bool = False) -> SceneSpec:
    robot = make_robot(robot_name)
    last_error: Exception | None = None
    for scene_try in range(max(1, int(max_scene_tries))):
        rng = random.Random(seed + 1000003 * scene_try)
        obstacles: list[Any] = []
        try:
            start, goal, canonical_start, canonical_goal, shift, sector = sample_free_pair_with_canonical_record(robot, obstacles, rng)
            target = random_obstacle_count(difficulty)
            base = random_obstacle_scale(difficulty)
            if not append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, base, True):
                continue
            while len(obstacles) < target:
                if not append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, base, False):
                    break
            if len(obstacles) < target:
                continue
            if not obstacles_clear_fixed_robot(robot_name, obstacles):
                continue
            if not endpoint_pair_has_clearance(robot, obstacles, start, goal):
                continue
            if segment_is_collision_free(robot, obstacles, start, goal):
                continue
            if timed and not scene_passes_timed_probe(robot, obstacles, start, goal, seed + 131071, difficulty, strict_time=strict_time):
                continue
            if balanced and not scene_passes_balanced_probe(robot, obstacles, start, goal, seed + 8191 * scene_try):
                continue
            return SceneSpec(
                robot_name=robot_name,
                difficulty=difficulty,
                obstacles=obstacles,
                start=start,
                goal=goal,
                canonical_start=canonical_start,
                canonical_goal=canonical_goal,
                symmetry_shift=shift,
                symmetry_sector=sector,
            )
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"could not sample a valid {robot_name}/{difficulty} legacy scene after {max_scene_tries} attempts: {last_error}")


def make_random_scene(robot_name: str, difficulty: str, seed: int, max_scene_tries: int = 64, scene_profile: str = "balanced") -> SceneSpec:
    timed = scene_profile_requires_timed_probe(scene_profile)
    strict_time = scene_profile_requires_strict_time_probe(scene_profile)
    if scene_profile_uses_bitstar_gated_construction(scene_profile):
        return make_bitstar_gated_scene(robot_name, difficulty, seed, max_scene_tries)
    if scene_profile_uses_narrow_construction(scene_profile) and "independent" in str(scene_profile).lower():
        return make_narrow_passage_scene(robot_name, difficulty, seed, max_scene_tries, strict_time=strict_time)
    if scene_profile_uses_nested_prefixes(scene_profile):
        return make_nested_random_scene(robot_name, difficulty, seed, max_scene_tries, scene_profile_requires_balanced_probe(scene_profile), timed, strict_time)
    return make_legacy_random_scene(robot_name, difficulty, seed, max_scene_tries, balanced=False, timed=timed, strict_time=strict_time)


def scene_cache_key(robot_name: str, difficulty: str, scene_seed: int) -> str:
    return f"{robot_name}:{difficulty}:{int(scene_seed)}"


def query_record(
    *,
    label: str,
    robot: Any,
    start: list[float],
    goal: list[float],
    symmetry_shift: int = 0,
    symmetry_sector: int = 0,
    difficulty_probe: dict[str, Any] | None = None,
) -> dict[str, Any]:
    canonical_start = canonicalize_q(robot, list(start))
    canonical_goal = canonicalize_q(robot, list(goal))
    record = {
        "label": str(label),
        "start": [float(value) for value in start],
        "goal": [float(value) for value in goal],
        "canonical_start": [float(value) for value in canonical_start],
        "canonical_goal": [float(value) for value in canonical_goal],
        "symmetry_shift": int(symmetry_shift),
        "symmetry_sector": int(symmetry_sector),
        "canonical_start_in_lect_root": q_in_intervals(canonical_start, canonical_root_intervals(robot)),
        "canonical_goal_in_lect_root": q_in_intervals(canonical_goal, canonical_root_intervals(robot)),
        "actual_start_in_lect_root": q_in_lect_root(robot, start),
        "actual_goal_in_lect_root": q_in_lect_root(robot, goal),
    }
    if difficulty_probe is not None:
        record["difficulty_probe"] = dict(difficulty_probe)
    return record


def sample_additional_query_records(
    scene: SceneSpec,
    *,
    scene_seed: int,
    generator_seed: int,
    scene_profile: str,
    queries_per_scene: int,
    max_scene_tries: int,
) -> list[dict[str, Any]]:
    robot = make_robot(str(scene.robot_name))
    records = [
        query_record(
            label=f"q0",
            robot=robot,
            start=list(scene.start),
            goal=list(scene.goal),
            symmetry_shift=int(scene.symmetry_shift),
            symmetry_sector=int(scene.symmetry_sector),
        )
    ]
    target = max(1, int(queries_per_scene))
    if target <= 1:
        return records
    rng = random.Random(int(generator_seed) + 7919 * int(scene_seed) + 104729)
    attempts = 0
    max_attempts = max(2000, int(max_scene_tries) * 250)
    balanced = scene_profile_requires_balanced_probe(scene_profile)
    timed = scene_profile_requires_timed_probe(scene_profile)
    strict_time = scene_profile_requires_strict_time_probe(scene_profile)
    while len(records) < target and attempts < max_attempts:
        attempts += 1
        try:
            start, goal, _canonical_start, _canonical_goal, shift, sector = sample_free_pair_with_canonical_record(
                robot,
                list(scene.obstacles),
                rng,
                clearance_margin_m=float(scene.endpoint_clearance_margin_m),
            )
        except RuntimeError:
            continue
        if segment_is_collision_free(robot, list(scene.obstacles), start, goal):
            continue
        difficulty_probe = None
        if timed:
            difficulty_probe = timed_difficulty_probe(
                robot,
                list(scene.obstacles),
                start,
                goal,
                int(generator_seed) + 8191 * attempts,
                str(scene.difficulty),
                strict_time=strict_time,
            )
            if not bool(difficulty_probe.get("ok")):
                continue
        if balanced and not scene_passes_balanced_probe(
            robot,
            list(scene.obstacles),
            start,
            goal,
            int(generator_seed) + 8191 * attempts,
        ):
            continue
        records.append(
            query_record(
                label=f"q{len(records)}",
                robot=robot,
                start=start,
                goal=goal,
                symmetry_shift=shift,
                symmetry_sector=sector,
                difficulty_probe=difficulty_probe,
            )
        )
    if len(records) < target:
        raise RuntimeError(
            f"could only sample {len(records)}/{target} queries for "
            f"{scene.robot_name}/{scene.difficulty}/{scene_seed}"
        )
    return records


def scene_to_record(
    scene: SceneSpec,
    *,
    scene_seed: int,
    generator_seed: int,
    scene_profile: str,
    queries_per_scene: int = DEFAULT_QUERIES_PER_SCENE,
    max_scene_tries: int = 64,
) -> dict[str, Any]:
    robot = make_robot(str(scene.robot_name))
    queries = sample_additional_query_records(
        scene,
        scene_seed=int(scene_seed),
        generator_seed=int(generator_seed),
        scene_profile=str(scene_profile),
        queries_per_scene=int(queries_per_scene),
        max_scene_tries=int(max_scene_tries),
    )
    first = queries[0]
    if scene_profile_requires_timed_probe(scene_profile) and "difficulty_probe" not in first:
        first["difficulty_probe"] = timed_difficulty_probe(
            robot,
            list(scene.obstacles),
            [float(value) for value in first["start"]],
            [float(value) for value in first["goal"]],
            int(generator_seed) + 131071,
            str(scene.difficulty),
            strict_time=scene_profile_requires_strict_time_probe(scene_profile),
        )
    canonical_start = [float(value) for value in first["canonical_start"]]
    canonical_goal = [float(value) for value in first["canonical_goal"]]
    return {
        "schema": CATALOG_SCHEMA,
        "robot": str(scene.robot_name),
        "difficulty": str(scene.difficulty),
        "scene_seed": int(scene_seed),
        "generator_seed": int(generator_seed),
        "scene_profile": str(scene_profile),
        "queries_per_scene": int(len(queries)),
        "queries": queries,
        "start": [float(value) for value in first["start"]],
        "goal": [float(value) for value in first["goal"]],
        "canonical_start": [float(value) for value in canonical_start],
        "canonical_goal": [float(value) for value in canonical_goal],
        "symmetry_shift": int(first.get("symmetry_shift", scene.symmetry_shift)),
        "symmetry_sector": int(first.get("symmetry_sector", scene.symmetry_sector)),
        "obstacles": [obstacle_bounds(obstacle) for obstacle in scene.obstacles],
        "sample_domain": LECT_SAMPLE_DOMAIN,
        "canonical_cache": True,
        "lect_root_intervals": interval_pairs(canonical_root_intervals(robot)),
        "planning_root_intervals": interval_pairs(robot_joint_limit_intervals(robot)),
        "sector_expanded_root_intervals": interval_pairs(sector_expanded_lect_root_intervals(robot)),
        "canonical_start_in_lect_root": q_in_intervals(canonical_start, canonical_root_intervals(robot)),
        "canonical_goal_in_lect_root": q_in_intervals(canonical_goal, canonical_root_intervals(robot)),
        "actual_start_in_lect_root": q_in_lect_root(robot, scene.start),
        "actual_goal_in_lect_root": q_in_lect_root(robot, scene.goal),
        "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
        "fixed_robot_clearance_margin_m": float(scene.fixed_robot_clearance_margin_m),
        "max_query_l2": float(MAX_QUERY_L2),
        "direct_segment_blocked": bool(scene.direct_segment_blocked),
        "segment_resolution": int(scene.segment_resolution),
    }


def scene_to_record_with_queries(
    scene: SceneSpec,
    *,
    scene_seed: int,
    generator_seed: int,
    scene_profile: str,
    queries: list[dict[str, Any]],
    difficulty_probe: dict[str, Any] | None = None,
) -> dict[str, Any]:
    robot = make_robot(str(scene.robot_name))
    copied_queries = [dict(query) for query in queries]
    first = copied_queries[0]
    canonical_start = [float(value) for value in first["canonical_start"]]
    canonical_goal = [float(value) for value in first["canonical_goal"]]
    start = [float(value) for value in first["start"]]
    goal = [float(value) for value in first["goal"]]
    record = {
        "schema": CATALOG_SCHEMA,
        "robot": str(scene.robot_name),
        "difficulty": str(scene.difficulty),
        "scene_seed": int(scene_seed),
        "generator_seed": int(generator_seed),
        "scene_profile": str(scene_profile),
        "queries_per_scene": int(len(copied_queries)),
        "queries": copied_queries,
        "start": start,
        "goal": goal,
        "canonical_start": canonical_start,
        "canonical_goal": canonical_goal,
        "symmetry_shift": int(first.get("symmetry_shift", scene.symmetry_shift)),
        "symmetry_sector": int(first.get("symmetry_sector", scene.symmetry_sector)),
        "obstacles": [obstacle_bounds(obstacle) for obstacle in scene.obstacles],
        "sample_domain": LECT_SAMPLE_DOMAIN,
        "canonical_cache": True,
        "lect_root_intervals": interval_pairs(canonical_root_intervals(robot)),
        "planning_root_intervals": interval_pairs(robot_joint_limit_intervals(robot)),
        "sector_expanded_root_intervals": interval_pairs(sector_expanded_lect_root_intervals(robot)),
        "canonical_start_in_lect_root": q_in_intervals(canonical_start, canonical_root_intervals(robot)),
        "canonical_goal_in_lect_root": q_in_intervals(canonical_goal, canonical_root_intervals(robot)),
        "actual_start_in_lect_root": q_in_lect_root(robot, start),
        "actual_goal_in_lect_root": q_in_lect_root(robot, goal),
        "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
        "fixed_robot_clearance_margin_m": float(scene.fixed_robot_clearance_margin_m),
        "max_query_l2": float(MAX_QUERY_L2),
        "direct_segment_blocked": not segment_is_collision_free(robot, list(scene.obstacles), start, goal),
        "segment_resolution": int(scene.segment_resolution),
        "incremental_scene": {
            "shared_query_set": True,
            "obstacle_prefix_difficulty": str(scene.difficulty),
        },
    }
    if difficulty_probe is not None:
        record["difficulty_probe"] = dict(difficulty_probe)
    return record


def scene_from_record(record: dict[str, Any]) -> SceneSpec:
    return SceneSpec(
        robot_name=str(record["robot"]),
        difficulty=str(record["difficulty"]),
        obstacles=[obstacle_from_bounds(bounds) for bounds in record.get("obstacles", [])],
        start=[float(value) for value in record["start"]],
        goal=[float(value) for value in record["goal"]],
        canonical_start=[float(value) for value in record.get("canonical_start", record["start"])],
        canonical_goal=[float(value) for value in record.get("canonical_goal", record["goal"])],
        symmetry_shift=int(record.get("symmetry_shift", 0)),
        symmetry_sector=int(record.get("symmetry_sector", 0)),
        endpoint_clearance_margin_m=float(record.get("endpoint_clearance_margin_m", ENDPOINT_CLEARANCE_MARGIN_M)),
        fixed_robot_clearance_margin_m=float(record.get("fixed_robot_clearance_margin_m", FIXED_ROBOT_CLEARANCE_MARGIN_M)),
        direct_segment_blocked=bool(record.get("direct_segment_blocked", True)),
        segment_resolution=int(record.get("segment_resolution", SEGMENT_RESOLUTION)),
    )


def query_records_from_record(record: dict[str, Any]) -> list[dict[str, Any]]:
    queries = [dict(item) for item in record.get("queries", [])]
    if queries:
        return queries
    return [
        {
            "label": "q0",
            "start": [float(value) for value in record["start"]],
            "goal": [float(value) for value in record["goal"]],
            "canonical_start": [float(value) for value in record.get("canonical_start", record["start"])],
            "canonical_goal": [float(value) for value in record.get("canonical_goal", record["goal"])],
            "symmetry_shift": int(record.get("symmetry_shift", 0)),
            "symmetry_sector": int(record.get("symmetry_sector", 0)),
        }
    ]


def record_satisfies_lect_root(record: dict[str, Any]) -> bool:
    if str(record.get("schema", "")) == "tro2026_random_scene_catalog_v5":
        return (
            str(record.get("sample_domain", "")) in {LECT_SAMPLE_DOMAIN, "reflected_canonical_lect_root_sections"}
            and bool(record.get("canonical_cache", False))
            and bool(record.get("actual_start_in_lect_root", False))
            and bool(record.get("actual_goal_in_lect_root", False))
            and bool(record.get("canonical_start_in_lect_root", False))
            and bool(record.get("canonical_goal_in_lect_root", False))
        )
    try:
        robot = make_robot(str(record["robot"]))
        base_ok = (
            str(record.get("sample_domain", "")) == LECT_SAMPLE_DOMAIN
            and bool(record.get("canonical_cache", False))
            and q_in_lect_root(robot, [float(value) for value in record["start"]])
            and q_in_lect_root(robot, [float(value) for value in record["goal"]])
            and q_close(canonicalize_q(robot, [float(value) for value in record["start"]]), [float(value) for value in record["canonical_start"]])
            and q_close(canonicalize_q(robot, [float(value) for value in record["goal"]]), [float(value) for value in record["canonical_goal"]])
        )
        if not base_ok:
            return False
        for query in query_records_from_record(record):
            start = [float(value) for value in query["start"]]
            goal = [float(value) for value in query["goal"]]
            canonical_start = [float(value) for value in query.get("canonical_start", canonicalize_q(robot, start))]
            canonical_goal = [float(value) for value in query.get("canonical_goal", canonicalize_q(robot, goal))]
            if not (
                q_in_lect_root(robot, start)
                and q_in_lect_root(robot, goal)
                and q_close(canonicalize_q(robot, start), canonical_start)
                and q_close(canonicalize_q(robot, goal), canonical_goal)
            ):
                return False
        return True
    except Exception:
        return False


def record_has_shared_query_median_gate(record: dict[str, Any]) -> bool:
    probe = record.get("difficulty_probe", {})
    policy = str(probe.get("policy", ""))
    return (
        isinstance(probe, dict)
        and policy in {"shared_query_median_first_solution_time_windows", "distribution_separation_v1"}
        and bool(probe.get("ok"))
    )


def records_have_strict_nested_prefixes(
    records: dict[str, dict[str, Any]],
    robot: str,
    difficulties: Iterable[str],
    scene_seed: int,
) -> tuple[bool, str]:
    previous_obstacles: list[list[float]] | None = None
    previous_count = -1
    for difficulty in difficulties:
        key = scene_cache_key(robot, difficulty, scene_seed)
        record = records.get(key)
        if record is None:
            return False, f"missing {key}"
        obstacles = [[float(value) for value in item] for item in record.get("obstacles", [])]
        count = len(obstacles)
        if count <= previous_count:
            return False, f"{key} prefix count {count} is not greater than previous {previous_count}"
        if previous_obstacles is not None and obstacles[: len(previous_obstacles)] != previous_obstacles:
            return False, f"{key} is not an extension of the previous difficulty prefix"
        previous_obstacles = obstacles
        previous_count = count
    return True, ""


def records_have_shared_queries(
    records: dict[str, dict[str, Any]],
    robot: str,
    difficulties: Iterable[str],
    scene_seed: int,
) -> tuple[bool, str]:
    reference_queries: list[dict[str, Any]] | None = None
    reference_key = ""
    for difficulty in difficulties:
        key = scene_cache_key(robot, difficulty, scene_seed)
        record = records.get(key)
        if record is None:
            return False, f"missing {key}"
        queries = query_records_from_record(record)
        comparable = [
            {
                "start": [float(value) for value in query["start"]],
                "goal": [float(value) for value in query["goal"]],
                "canonical_start": [float(value) for value in query.get("canonical_start", query["start"])],
                "canonical_goal": [float(value) for value in query.get("canonical_goal", query["goal"])],
            }
            for query in queries
        ]
        if reference_queries is None:
            reference_queries = comparable
            reference_key = key
            continue
        if comparable != reference_queries:
            return False, f"{key} queries differ from {reference_key}"
    return True, ""


def expected_keys(robots: Iterable[str], difficulties: Iterable[str], scene_seeds: int) -> list[str]:
    return [
        scene_cache_key(robot, difficulty, seed)
        for robot in robots
        for difficulty in difficulties
        for seed in range(max(1, int(scene_seeds)))
    ]


def catalog_metadata(*, robots: list[str], difficulties: list[str], scene_seeds: int, scene_profile: str, seed_base: int, queries_per_scene: int = DEFAULT_QUERIES_PER_SCENE) -> dict[str, Any]:
    return {
        "schema": CATALOG_SCHEMA,
        "robots": list(robots),
        "difficulties": list(difficulties),
        "scene_seeds": int(scene_seeds),
        "scene_profile": str(scene_profile),
        "seed_base": int(seed_base),
        "queries_per_scene": int(queries_per_scene),
        "obstacle_counts": dict(RANDOM_OBSTACLE_COUNTS),
        "obstacle_scales": dict(RANDOM_OBSTACLE_SCALES),
        "max_query_l2": float(MAX_QUERY_L2),
        "feasibility_probe": {
            "planner": "OMPL_RRTConnect",
            "timeout_ms": float(BALANCED_PROBE_TIMEOUT_MS),
            "range": float(BALANCED_PROBE_RANGE),
            "segment_step": float(BALANCED_PROBE_SEGMENT_STEP),
            "required_for_profiles": ["balanced", "comparable", "paper", "balanced_probe", "comparable_probe", "paper_probe"],
        },
        "difficulty_probe": {
            "planner": "OMPL_RRTConnect+BITstar",
            "policy": "shared_query_median_first_solution_time_windows",
            "windows_s": {key: [float(lo), float(hi)] for key, (lo, hi) in TIMED_PROBE_TIMEOUT_WINDOWS_S.items()},
            "narrow_windows_s": {key: [float(lo), float(hi)] for key, (lo, hi) in NARROW_PASSAGE_TIMEOUT_WINDOWS_S.items()},
            "bitstar_median_windows_s": {
                key: [float(lo), float(hi)]
                for key, (lo, hi) in BITSTAR_MEDIAN_FIRST_SOLUTION_WINDOWS_S.items()
            },
            "planner_seed_count": int(INCREMENTAL_DIFFICULTY_PROBE_SEED_COUNT),
            "direct_obstruction_fraction_windows": {
                key: [float(lo), float(hi)]
                for key, (lo, hi) in DIRECT_OBSTRUCTION_FRACTION_WINDOWS.items()
            },
            "range": float(TIMED_PROBE_RANGE),
            "segment_step": float(TIMED_PROBE_SEGMENT_STEP),
            "simplify_time_s": float(TIMED_PROBE_SIMPLIFY_TIME_S),
            "bitstar": {
                "timeout_s": float(BITSTAR_PROBE_TIMEOUT_S),
                "checkpoint_interval_s": float(BITSTAR_PROBE_CHECKPOINT_INTERVAL_S),
                "min_first_success_s": float(BITSTAR_PROBE_MIN_FIRST_SUCCESS_S),
                "samples_per_batch": int(BITSTAR_PROBE_SAMPLES_PER_BATCH),
                "rewire_factor": float(BITSTAR_PROBE_REWIRE_FACTOR),
            },
            "required_for_profiles": [
                "bitstar_gated", "bitstar_gated_independent",
                "timed_probe", "timed_probe_independent", "timed", "timed_independent",
                "narrow_passage", "narrow_passage_independent", "narrow", "narrow_independent",
            ],
        },
        "difficulty_order": list(RANDOM_DIFFICULTY_ORDER),
        "sample_domain": LECT_SAMPLE_DOMAIN,
        "canonical_cache": True,
        "root_policy": "sample actual start/goal in full native robot joint limits; store canonical representatives only as LECT metadata.",
        "robot_root_intervals": {
            robot_name: interval_pairs(robot_joint_limit_intervals(make_robot(robot_name)))
            for robot_name in robots
        },
        "robot_sector_expanded_root_intervals": {
            robot_name: interval_pairs(sector_expanded_lect_root_intervals(make_robot(robot_name)))
            for robot_name in robots
        },
        "environment": environment_metadata(),
    }


def load_catalog(path: Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if payload.get("schema") not in READABLE_CATALOG_SCHEMAS:
        raise ValueError(f"unsupported scene catalog schema in {path}: {payload.get('schema')!r}")
    return payload


def catalog_record_map(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    records = payload.get("records", [])
    return {scene_cache_key(str(row["robot"]), str(row["difficulty"]), int(row["scene_seed"])): dict(row) for row in records}


def write_catalog(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def generate_catalog(
    *,
    path: Path,
    robots: list[str],
    difficulties: list[str],
    scene_seeds: int,
    scene_profile: str,
    seed_base: int,
    queries_per_scene: int = DEFAULT_QUERIES_PER_SCENE,
    max_scene_tries: int = 64,
    mode: str = "auto",
) -> dict[str, Any]:
    mode = str(mode)
    keys = expected_keys(robots, difficulties, int(scene_seeds))
    existing: dict[str, dict[str, Any]] = {}
    loaded_payload: dict[str, Any] | None = None
    if path.exists() and mode in {"auto", "reuse", "verify"}:
        try:
            loaded_payload = load_catalog(path)
            existing = catalog_record_map(loaded_payload)
        except ValueError:
            if mode in {"reuse", "verify"}:
                raise
            existing = {}
    if mode in {"reuse", "verify"}:
        missing = [key for key in keys if key not in existing]
        if missing:
            raise RuntimeError(f"scene catalog {path} is missing {len(missing)} required scenes; first={missing[0]}")
        invalid = [key for key in keys if not record_satisfies_lect_root(existing[key])]
        if invalid:
            raise RuntimeError(f"scene catalog {path} has {len(invalid)} records outside the canonical LECT root; first={invalid[0]}")
        too_few_queries = [
            key for key in keys
            if len(query_records_from_record(existing[key])) < max(1, int(queries_per_scene))
        ]
        if too_few_queries:
            raise RuntimeError(
                f"scene catalog {path} has {len(too_few_queries)} records with too few queries; "
                f"first={too_few_queries[0]}"
            )
        if catalog_payload_uses_nested_prefixes(loaded_payload or {}, scene_profile):
            missing_probe = [key for key in keys if not record_has_shared_query_median_gate(existing[key])]
            if missing_probe:
                raise RuntimeError(
                    f"scene catalog {path} has {len(missing_probe)} records without the shared-query median difficulty gate; "
                    f"first={missing_probe[0]}"
                )
            nested_errors: list[str] = []
            query_errors: list[str] = []
            for robot_name in robots:
                for scene_seed in range(max(1, int(scene_seeds))):
                    ok, reason = records_have_strict_nested_prefixes(existing, robot_name, difficulties, scene_seed)
                    if not ok:
                        nested_errors.append(reason)
                    ok, reason = records_have_shared_queries(existing, robot_name, difficulties, scene_seed)
                    if not ok:
                        query_errors.append(reason)
            if nested_errors:
                raise RuntimeError(
                    f"scene catalog {path} has {len(nested_errors)} non-prefix-nested scene groups; "
                    f"first={nested_errors[0]}"
                )
            if query_errors:
                raise RuntimeError(
                    f"scene catalog {path} has {len(query_errors)} scene groups without shared difficulty queries; "
                    f"first={query_errors[0]}"
                )
        return loaded_payload if loaded_payload is not None else load_catalog(path)
    records = dict(existing) if mode == "auto" else {}
    total = len(keys)
    if scene_profile_uses_nested_prefixes(scene_profile) and "independent" not in str(scene_profile).lower():
        for robot_name in robots:
            for scene_seed in range(max(1, int(scene_seeds))):
                group_keys = [scene_cache_key(robot_name, difficulty, scene_seed) for difficulty in difficulties]
                if all(
                    key in records
                    and record_satisfies_lect_root(records[key])
                    and len(query_records_from_record(records[key])) >= max(1, int(queries_per_scene))
                    and record_has_shared_query_median_gate(records[key])
                    for key in group_keys
                ):
                    continue
                base_generator_seed = int(seed_base) + 1009 * int(scene_seed)
                print(f"[catalog] generating nested {robot_name}:{scene_seed} ({len(records) + 1}/{total})", flush=True)
                robot = make_robot(robot_name)
                accepted: tuple[
                    Any,
                    list[dict[str, Any]],
                    dict[str, list[Any]],
                    dict[str, dict[str, Any]],
                    int,
                ] | None = None
                last_error: Exception | None = None
                full_difficulty = "hard" if "hard" in RANDOM_DIFFICULTY_ORDER else difficulties[-1]
                for scene_try in range(max(1, int(max_scene_tries))):
                    generator_seed = base_generator_seed + 1_000_003 * int(scene_try)
                    try:
                        full_scene = make_random_scene(
                            robot_name,
                            full_difficulty,
                            generator_seed,
                            max_scene_tries=int(max_scene_tries),
                            scene_profile=scene_profile,
                        )
                        easy_seed_scene = SceneSpec(
                            robot_name=robot_name,
                            difficulty="easy",
                            obstacles=obstacle_prefix(list(full_scene.obstacles), "easy"),
                            start=list(full_scene.start),
                            goal=list(full_scene.goal),
                            canonical_start=list(full_scene.canonical_start or full_scene.start),
                            canonical_goal=list(full_scene.canonical_goal or full_scene.goal),
                            symmetry_shift=int(full_scene.symmetry_shift),
                            symmetry_sector=int(full_scene.symmetry_sector),
                            endpoint_clearance_margin_m=float(full_scene.endpoint_clearance_margin_m),
                            fixed_robot_clearance_margin_m=float(full_scene.fixed_robot_clearance_margin_m),
                            direct_segment_blocked=True,
                            segment_resolution=int(full_scene.segment_resolution),
                        )
                        query_seed_scene = SceneSpec(
                            robot_name=robot_name,
                            difficulty=full_difficulty,
                            obstacles=list(full_scene.obstacles),
                            start=list(full_scene.start),
                            goal=list(full_scene.goal),
                            canonical_start=list(full_scene.canonical_start or full_scene.start),
                            canonical_goal=list(full_scene.canonical_goal or full_scene.goal),
                            symmetry_shift=int(full_scene.symmetry_shift),
                            symmetry_sector=int(full_scene.symmetry_sector),
                            endpoint_clearance_margin_m=float(full_scene.endpoint_clearance_margin_m),
                            fixed_robot_clearance_margin_m=float(full_scene.fixed_robot_clearance_margin_m),
                            direct_segment_blocked=True,
                            segment_resolution=int(full_scene.segment_resolution),
                        )
                        query_seed_record = scene_to_record(
                            query_seed_scene,
                            scene_seed=scene_seed,
                            generator_seed=generator_seed,
                            scene_profile=scene_profile,
                            queries_per_scene=int(queries_per_scene),
                            max_scene_tries=int(max_scene_tries),
                        )
                        shared_queries = query_records_from_record(query_seed_record)[: max(1, int(queries_per_scene))]
                        prefix_obstacles: dict[str, list[Any]] = {"easy": list(easy_seed_scene.obstacles)}
                        difficulty_probes: dict[str, dict[str, Any]] = {}
                        easy_probe = scene_median_difficulty_probe(
                            robot,
                            prefix_obstacles["easy"],
                            shared_queries,
                            generator_seed + 65_537,
                            "easy",
                        )
                        if not bool(easy_probe.get("ok")):
                            last_error = RuntimeError(
                                "easy shared-query median gate failed: "
                                + difficulty_probe_brief(easy_probe)
                            )
                            continue
                        difficulty_probes["easy"] = easy_probe
                        rng = random.Random(generator_seed + 4_194_301)
                        previous_obstacles = list(prefix_obstacles["easy"])
                        failed = False
                        for difficulty in RANDOM_DIFFICULTY_ORDER[1:]:
                            candidate_obstacles = list(previous_obstacles)
                            ok, probe, _added = grow_until_shared_query_median_gate(
                                robot_name,
                                robot,
                                candidate_obstacles,
                                shared_queries,
                                rng,
                                difficulty=difficulty,
                                seed=generator_seed + 65_537 + 8191 * RANDOM_DIFFICULTY_ORDER.index(difficulty),
                                base=random_obstacle_scale(difficulty),
                                endpoint_clearance_margin_m=float(full_scene.endpoint_clearance_margin_m),
                            )
                            if not ok:
                                last_error = RuntimeError(
                                    f"{difficulty} shared-query median gate failed: "
                                    + difficulty_probe_brief(probe)
                                )
                                failed = True
                                break
                            prefix_obstacles[difficulty] = candidate_obstacles
                            difficulty_probes[difficulty] = probe
                            previous_obstacles = candidate_obstacles
                        if failed:
                            continue
                        accepted = (full_scene, shared_queries, prefix_obstacles, difficulty_probes, generator_seed)
                        break
                    except Exception as exc:
                        last_error = exc
                        continue
                if accepted is None:
                    raise RuntimeError(
                        f"could not generate incremental median-gated scene for {robot_name}/{scene_seed} "
                        f"after {max_scene_tries} attempts: {last_error}"
                    )
                full_scene, shared_queries, prefix_obstacles, difficulty_probes, generator_seed = accepted
                for difficulty in difficulties:
                    scene = SceneSpec(
                        robot_name=robot_name,
                        difficulty=difficulty,
                        obstacles=list(prefix_obstacles[difficulty]),
                        start=list(full_scene.start),
                        goal=list(full_scene.goal),
                        canonical_start=list(full_scene.canonical_start or full_scene.start),
                        canonical_goal=list(full_scene.canonical_goal or full_scene.goal),
                        symmetry_shift=int(full_scene.symmetry_shift),
                        symmetry_sector=int(full_scene.symmetry_sector),
                        endpoint_clearance_margin_m=float(full_scene.endpoint_clearance_margin_m),
                        fixed_robot_clearance_margin_m=float(full_scene.fixed_robot_clearance_margin_m),
                        direct_segment_blocked=True,
                        segment_resolution=int(full_scene.segment_resolution),
                    )
                    records[scene_cache_key(robot_name, difficulty, scene_seed)] = scene_to_record_with_queries(
                        scene,
                        scene_seed=scene_seed,
                        generator_seed=generator_seed,
                        scene_profile=scene_profile,
                        queries=shared_queries,
                        difficulty_probe=difficulty_probes.get(difficulty),
                    )
        payload = {
            **catalog_metadata(
                robots=robots,
                difficulties=difficulties,
                scene_seeds=int(scene_seeds),
                scene_profile=scene_profile,
                seed_base=int(seed_base),
                queries_per_scene=int(queries_per_scene),
            ),
            "records": [records[key] for key in keys],
            "incremental_scene_policy": {
                "obstacle_prefixes": True,
                "shared_queries_across_difficulties": True,
                "base_scene_difficulty": "hard",
                "difficulty_acceptance": "RRTConnect and BIT* shared-query median first-solution time windows",
                "obstacle_counts_are_not_fixed": True,
            },
        }
        write_catalog(path, payload)
        return payload

    for key in keys:
        if (
            key in records
            and record_satisfies_lect_root(records[key])
            and len(query_records_from_record(records[key])) >= max(1, int(queries_per_scene))
        ):
            continue
        robot_name, difficulty, scene_seed_text = key.split(":")
        scene_seed = int(scene_seed_text)
        difficulty_offset = 0
        if (not scene_profile_uses_nested_prefixes(scene_profile)) or "independent" in str(scene_profile).lower():
            difficulty_offset = RANDOM_DIFFICULTY_ORDER.index(difficulty) * 104729 if difficulty in RANDOM_DIFFICULTY_ORDER else 524287
        generator_seed = int(seed_base) + 1009 * scene_seed + difficulty_offset
        print(f"[catalog] generating {key} ({len(records) + 1}/{total})", flush=True)
        scene = make_random_scene(
            robot_name,
            difficulty,
            generator_seed,
            max_scene_tries=int(max_scene_tries),
            scene_profile=scene_profile,
        )
        records[key] = scene_to_record(
            scene,
            scene_seed=scene_seed,
            generator_seed=generator_seed,
            scene_profile=scene_profile,
            queries_per_scene=int(queries_per_scene),
            max_scene_tries=int(max_scene_tries),
        )
    payload = {
        **catalog_metadata(
            robots=robots,
            difficulties=difficulties,
            scene_seeds=int(scene_seeds),
            scene_profile=scene_profile,
            seed_base=int(seed_base),
            queries_per_scene=int(queries_per_scene),
        ),
        "records": [records[key] for key in keys],
    }
    write_catalog(path, payload)
    return payload


def scene_for_key(payload: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> SceneSpec:
    records = catalog_record_map(payload)
    key = scene_cache_key(robot_name, difficulty, int(scene_seed))
    if key not in records:
        raise KeyError(f"scene catalog missing {key}")
    return scene_from_record(records[key])


def queries_for_key(payload: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> list[dict[str, Any]]:
    records = catalog_record_map(payload)
    key = scene_cache_key(robot_name, difficulty, int(scene_seed))
    if key not in records:
        raise KeyError(f"scene catalog missing {key}")
    return query_records_from_record(records[key])
