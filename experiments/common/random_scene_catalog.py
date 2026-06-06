from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

from experiments.common.experiment_io import environment_metadata
from experiments.common.rbf_defaults import robot_joint_limit_tuples
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()

ENDPOINT_CLEARANCE_MARGIN_M = 0.24
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
DEFAULT_RANDOM_ROBOTS = "iiwa,ur5,panda"
DEFAULT_RANDOM_DIFFICULTIES = "easy,medium,hard"
DEFAULT_RANDOM_SCENE_SEEDS = 50
DEFAULT_QUERIES_PER_SCENE = 10
RANDOM_DIFFICULTY_ORDER = ("easy", "medium", "hard")
RANDOM_OBSTACLE_COUNTS = {"easy": 4, "medium": 8, "hard": 12}
RANDOM_OBSTACLE_SCALES = {"easy": 0.12, "medium": 0.16, "hard": 0.20}
RANDOM_WORKSPACE_Z_MIN = 0.05
RANDOM_WORKSPACE_Z_MAX = 0.90
CANONICAL_SYMMETRY_DESCRIPTOR = "joint_symmetry_native_v1"
CATALOG_SCHEMA = "tro2026_random_scene_catalog_v6"
READABLE_CATALOG_SCHEMAS = {CATALOG_SCHEMA, "tro2026_random_scene_catalog_v5"}
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
    steps = max(1, int(resolution))
    for index in range(steps + 1):
        if sbf.check_config_collision(robot, obstacles, interpolate(start, goal, index / steps)):
            return False
    return True


def path_is_collision_free(robot: Any, obstacles: list[Any], path: list[list[float]], resolution: int = SEGMENT_RESOLUTION) -> bool:
    if len(path) < 2:
        return False
    return all(segment_is_collision_free(robot, obstacles, path[index], path[index + 1], resolution) for index in range(len(path) - 1))


def scene_profile_uses_nested_prefixes(scene_profile: str) -> bool:
    return str(scene_profile).lower() in {
        "balanced",
        "balanced_independent",
        "comparable",
        "paper",
        "balanced_probe",
        "comparable_probe",
        "paper_probe",
    }


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


def scene_passes_balanced_probe(robot: Any, obstacles: list[Any], start: list[float], goal: list[float], seed: int) -> bool:
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
) -> bool:
    for _ in range(max_attempts):
        candidate = random_workspace_obstacle(rng, base)
        if not obstacle_clears_fixed_robot(robot_name, candidate):
            continue
        proposed = [*obstacles, candidate]
        if not endpoint_pair_has_clearance(robot, proposed, start, goal):
            continue
        if require_direct_blocker and segment_is_collision_free(robot, [candidate], start, goal):
            continue
        obstacles.append(candidate)
        return True
    return False


def nested_prefixes_are_valid(
    robot: Any,
    robot_name: str,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    scene_try: int,
    balanced: bool,
) -> bool:
    for difficulty in RANDOM_DIFFICULTY_ORDER:
        prefix = obstacle_prefix(obstacles, difficulty)
        if not obstacles_clear_fixed_robot(robot_name, prefix):
            return False
        if not endpoint_pair_has_clearance(robot, prefix, start, goal):
            return False
        if segment_is_collision_free(robot, prefix, start, goal):
            return False
        if balanced:
            difficulty_offset = RANDOM_DIFFICULTY_ORDER.index(difficulty) * 104729
            if not scene_passes_balanced_probe(robot, prefix, start, goal, seed + 8191 * scene_try + difficulty_offset):
                return False
    return True


def make_nested_random_scene(robot_name: str, difficulty: str, seed: int, max_scene_tries: int, balanced: bool) -> SceneSpec:
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
            if not nested_prefixes_are_valid(robot, robot_name, obstacles, start, goal, seed, scene_try, balanced):
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


def make_legacy_random_scene(robot_name: str, difficulty: str, seed: int, max_scene_tries: int, balanced: bool) -> SceneSpec:
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
    if scene_profile_uses_nested_prefixes(scene_profile):
        return make_nested_random_scene(robot_name, difficulty, seed, max_scene_tries, scene_profile_requires_balanced_probe(scene_profile))
    return make_legacy_random_scene(robot_name, difficulty, seed, max_scene_tries, balanced=False)


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
) -> dict[str, Any]:
    canonical_start = canonicalize_q(robot, list(start))
    canonical_goal = canonicalize_q(robot, list(goal))
    return {
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
    while len(records) < target and attempts < max_attempts:
        attempts += 1
        try:
            start, goal, _canonical_start, _canonical_goal, shift, sector = sample_free_pair_with_canonical_record(
                robot,
                list(scene.obstacles),
                rng,
            )
        except RuntimeError:
            continue
        if segment_is_collision_free(robot, list(scene.obstacles), start, goal):
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
    if path.exists() and mode in {"auto", "reuse", "verify"}:
        try:
            existing = catalog_record_map(load_catalog(path))
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
        return load_catalog(path)
    records = dict(existing) if mode == "auto" else {}
    total = len(keys)
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
