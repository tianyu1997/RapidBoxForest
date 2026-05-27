from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Sequence

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import sbf
from sbf.marcucci import load_iiwa14_robot

ENDPOINT_CLEARANCE_MARGIN_M = 0.24
FIXED_ROBOT_CLEARANCE_MARGIN_M = 0.025
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
DEFAULT_RANDOM_SCENE_SEEDS = 5
RANDOM_DIFFICULTY_ORDER = ("easy", "medium", "hard")
RANDOM_OBSTACLE_COUNTS = {"easy": 4, "medium": 8, "hard": 12}
RANDOM_OBSTACLE_SCALES = {"easy": 0.12, "medium": 0.16, "hard": 0.20}
RANDOM_WORKSPACE_Z_MIN = 0.05
RANDOM_WORKSPACE_Z_MAX = 0.90


@dataclass(frozen=True)
class SceneSpec:
    robot_name: str
    difficulty: str
    obstacles: list[sbf.Obstacle]
    start: list[float]
    goal: list[float]
    endpoint_clearance_margin_m: float = ENDPOINT_CLEARANCE_MARGIN_M
    fixed_robot_clearance_margin_m: float = FIXED_ROBOT_CLEARANCE_MARGIN_M
    direct_segment_blocked: bool = True
    segment_resolution: int = SEGMENT_RESOLUTION


def make_dh(alpha: float, a: float, d: float = 0.0, theta: float = 0.0, joint_type: int = 0) -> sbf.DHParam:
    dh = sbf.DHParam()
    dh.alpha = float(alpha)
    dh.a = float(a)
    dh.d = float(d)
    dh.theta = float(theta)
    dh.joint_type = int(joint_type)
    return dh


def make_limits(bounds: Sequence[tuple[float, float]]) -> sbf.JointLimits:
    limits = sbf.JointLimits()
    limits.limits = [sbf.Interval(lo, hi) for lo, hi in bounds]
    return limits


def make_ur5_like_robot() -> sbf.Robot:
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


def make_panda_like_robot() -> sbf.Robot:
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


def make_iiwa_robot() -> sbf.Robot:
    return load_iiwa14_robot()


def make_robot(name: str) -> sbf.Robot:
    if name == "iiwa":
        return make_iiwa_robot()
    if name == "ur5":
        return make_ur5_like_robot()
    if name == "panda":
        return make_panda_like_robot()
    raise ValueError(f"unknown standalone robot '{name}'")


def make_aabb(cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> sbf.Obstacle:
    return sbf.Obstacle(cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz)


def inflate_obstacle(obstacle: sbf.Obstacle, margin: float) -> sbf.Obstacle:
    bounds = list(obstacle.bounds)
    return sbf.Obstacle(
        bounds[0] - margin,
        bounds[1] - margin,
        bounds[2] - margin,
        bounds[3] + margin,
        bounds[4] + margin,
        bounds[5] + margin,
    )


def obstacle_bounds(obstacle: sbf.Obstacle) -> list[float]:
    return [float(value) for value in obstacle.bounds]


def aabb_overlaps(lhs: sbf.Obstacle, rhs: sbf.Obstacle, margin: float = 0.0) -> bool:
    a = obstacle_bounds(lhs)
    b = obstacle_bounds(rhs)
    return not (
        a[3] < b[0] - margin or a[0] > b[3] + margin
        or a[4] < b[1] - margin or a[1] > b[4] + margin
        or a[5] < b[2] - margin or a[2] > b[5] + margin
    )


def fixed_robot_exclusion_zones(robot_name: str) -> list[sbf.Obstacle]:
    if robot_name == "iiwa":
        return [make_aabb(0.0, 0.0, 0.30, 0.26, 0.26, 0.40)]
    if robot_name == "ur5":
        return [make_aabb(0.0, 0.0, 0.07, 0.22, 0.22, 0.18)]
    if robot_name == "panda":
        return [make_aabb(0.0, 0.0, 0.18, 0.24, 0.24, 0.30)]
    return []


def obstacle_clears_fixed_robot(robot_name: str,
                                obstacle: sbf.Obstacle,
                                margin: float = FIXED_ROBOT_CLEARANCE_MARGIN_M) -> bool:
    return all(not aabb_overlaps(obstacle, zone, margin) for zone in fixed_robot_exclusion_zones(robot_name))


def obstacles_clear_fixed_robot(robot_name: str,
                                obstacles: list[sbf.Obstacle],
                                margin: float = FIXED_ROBOT_CLEARANCE_MARGIN_M) -> bool:
    return all(obstacle_clears_fixed_robot(robot_name, obstacle, margin) for obstacle in obstacles)


def random_obstacles(rng: random.Random, difficulty: str) -> list[sbf.Obstacle]:
    n_obs = random_obstacle_count(difficulty)
    base = random_obstacle_scale(difficulty)
    obstacles: list[sbf.Obstacle] = []
    for _ in range(n_obs):
        obstacles.append(random_workspace_obstacle(rng, base))
    return obstacles


def random_workspace_obstacle(rng: random.Random, base: float) -> sbf.Obstacle:
    hx = rng.uniform(base * 0.5, base)
    hy = rng.uniform(base * 0.5, base)
    hz = rng.uniform(base * 0.5, base * 1.3)
    cx = rng.uniform(-0.55, 0.75)
    cy = rng.uniform(-0.65, 0.65)
    cz = rng.uniform(RANDOM_WORKSPACE_Z_MIN + hz, RANDOM_WORKSPACE_Z_MAX - hz)
    return make_aabb(cx, cy, cz, hx, hy, hz)


def random_obstacle_count(difficulty: str) -> int:
    return RANDOM_OBSTACLE_COUNTS.get(difficulty, RANDOM_OBSTACLE_COUNTS["medium"])


def random_obstacle_scale(difficulty: str) -> float:
    return RANDOM_OBSTACLE_SCALES.get(difficulty, RANDOM_OBSTACLE_SCALES["medium"])


def random_difficulty_prefix(difficulty: str) -> list[str]:
    if difficulty not in RANDOM_DIFFICULTY_ORDER:
        return ["medium"]
    return list(RANDOM_DIFFICULTY_ORDER[:RANDOM_DIFFICULTY_ORDER.index(difficulty) + 1])


def obstacle_prefix(obstacles: list[sbf.Obstacle], difficulty: str) -> list[sbf.Obstacle]:
    return list(obstacles[:random_obstacle_count(difficulty)])


def sample_q(robot: sbf.Robot, rng: random.Random) -> list[float]:
    return [rng.uniform(interval.lo, interval.hi) for interval in robot.joint_limits().limits]


def config_has_clearance(robot: sbf.Robot, obstacles: list[sbf.Obstacle], q: list[float], margin: float) -> bool:
    inflated = [inflate_obstacle(obstacle, margin) for obstacle in obstacles]
    return not sbf.check_config_collision(robot, inflated, q)


def endpoint_pair_has_clearance(robot: sbf.Robot,
                                obstacles: list[sbf.Obstacle],
                                start: list[float],
                                goal: list[float],
                                margin: float = ENDPOINT_CLEARANCE_MARGIN_M) -> bool:
    return config_has_clearance(robot, obstacles, start, margin) and config_has_clearance(robot, obstacles, goal, margin)


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def segment_is_collision_free(robot: sbf.Robot,
                              obstacles: list[sbf.Obstacle],
                              start: list[float],
                              goal: list[float],
                              resolution: int = SEGMENT_RESOLUTION) -> bool:
    steps = max(1, int(resolution))
    for index in range(steps + 1):
        if sbf.check_config_collision(robot, obstacles, interpolate(start, goal, index / steps)):
            return False
    return True


def direct_obstruction_profile(robot: sbf.Robot,
                               obstacles: list[sbf.Obstacle],
                               start: list[float],
                               goal: list[float],
                               resolution: int = SEGMENT_RESOLUTION) -> tuple[int, list[int]]:
    steps = max(1, int(resolution))
    per_obstacle = [0 for _ in obstacles]
    total_hits = 0
    for index in range(steps + 1):
        q = interpolate(start, goal, index / steps)
        collided = False
        for obstacle_index, obstacle in enumerate(obstacles):
            if sbf.check_config_collision(robot, [obstacle], q):
                per_obstacle[obstacle_index] += 1
                collided = True
        if collided:
            total_hits += 1
    return total_hits, per_obstacle


def scene_has_stable_direct_obstruction(robot: sbf.Robot,
                                        obstacles: list[sbf.Obstacle],
                                        start: list[float],
                                        goal: list[float]) -> bool:
    total_hits, per_obstacle = direct_obstruction_profile(robot, obstacles, start, goal)
    blocker_count = sum(1 for hits in per_obstacle if hits >= DIRECT_OBSTRUCTION_MIN_HITS_PER_OBSTACLE)
    return total_hits >= DIRECT_OBSTRUCTION_MIN_TOTAL_HITS and blocker_count >= DIRECT_OBSTRUCTION_MIN_OBSTACLES


def path_is_collision_free(robot: sbf.Robot,
                           obstacles: list[sbf.Obstacle],
                           path: list[list[float]],
                           resolution: int = SEGMENT_RESOLUTION) -> bool:
    if len(path) < 2:
        return False
    return all(segment_is_collision_free(robot, obstacles, path[index], path[index + 1], resolution) for index in range(len(path) - 1))


def scene_profile_uses_nested_prefixes(scene_profile: str) -> bool:
    profile = str(scene_profile).lower()
    return profile in {"balanced", "comparable", "paper", "balanced_probe", "comparable_probe", "paper_probe"}


def scene_profile_requires_balanced_probe(scene_profile: str) -> bool:
    profile = str(scene_profile).lower()
    return profile in {"balanced_probe", "comparable_probe", "paper_probe"}


def scene_passes_balanced_probe(robot: sbf.Robot,
                                obstacles: list[sbf.Obstacle],
                                start: list[float],
                                goal: list[float],
                                seed: int) -> bool:
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


def sample_free_pair(robot: sbf.Robot,
                     obstacles: list[sbf.Obstacle],
                     rng: random.Random,
                     min_l2: float = 0.8,
                     clearance_margin_m: float = ENDPOINT_CLEARANCE_MARGIN_M,
                     max_tries: int = 2000) -> tuple[list[float], list[float]]:
    start: list[float] | None = None
    for _ in range(max_tries):
        q = sample_q(robot, rng)
        if not sbf.check_config_collision(robot, obstacles, q) and config_has_clearance(robot, obstacles, q, clearance_margin_m):
            start = q
            break
    if start is None:
        raise RuntimeError("could not sample a collision-free start")
    for _ in range(max_tries):
        goal = sample_q(robot, rng)
        if sbf.check_config_collision(robot, obstacles, goal):
            continue
        if not config_has_clearance(robot, obstacles, goal, clearance_margin_m):
            continue
        dist = math.sqrt(sum((a - b) * (a - b) for a, b in zip(start, goal)))
        if dist >= min_l2:
            return start, goal
    raise RuntimeError("could not sample a collision-free goal")


def append_random_scene_obstacle(robot_name: str,
                                 robot: sbf.Robot,
                                 obstacles: list[sbf.Obstacle],
                                 start: list[float],
                                 goal: list[float],
                                 rng: random.Random,
                                 base: float,
                                 require_direct_blocker: bool,
                                 max_attempts: int = 5000) -> bool:
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


def nested_prefixes_are_valid(robot: sbf.Robot,
                              robot_name: str,
                              obstacles: list[sbf.Obstacle],
                              start: list[float],
                              goal: list[float],
                              seed: int,
                              scene_try: int,
                              balanced: bool) -> bool:
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


def make_nested_random_scene(robot_name: str,
                             difficulty: str,
                             seed: int,
                             max_scene_tries: int,
                             balanced: bool) -> SceneSpec:
    robot = make_robot(robot_name)
    last_error: Exception | None = None
    for scene_try in range(max(1, int(max_scene_tries))):
        rng = random.Random(seed + 1000003 * scene_try)
        obstacles: list[sbf.Obstacle] = []
        try:
            start, goal = sample_free_pair(robot, obstacles, rng)
            easy_base = random_obstacle_scale("easy")
            if not append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, easy_base, require_direct_blocker=True):
                raise RuntimeError("could not add an easy direct blocker")
            for level in RANDOM_DIFFICULTY_ORDER:
                base = random_obstacle_scale(level)
                while len(obstacles) < random_obstacle_count(level):
                    if not append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, base, require_direct_blocker=False):
                        raise RuntimeError(f"could not fill {level} obstacle prefix")
            if not nested_prefixes_are_valid(robot, robot_name, obstacles, start, goal, seed, scene_try, balanced):
                continue
            selected_obstacles = obstacle_prefix(obstacles, difficulty)
            return SceneSpec(
                robot_name=robot_name,
                difficulty=difficulty,
                obstacles=selected_obstacles,
                start=start,
                goal=goal,
                endpoint_clearance_margin_m=ENDPOINT_CLEARANCE_MARGIN_M,
                fixed_robot_clearance_margin_m=FIXED_ROBOT_CLEARANCE_MARGIN_M,
                direct_segment_blocked=True,
                segment_resolution=SEGMENT_RESOLUTION,
            )
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"could not sample a valid {robot_name}/{difficulty} scene after {max_scene_tries} attempts: {last_error}")


def make_legacy_random_scene(robot_name: str,
                             difficulty: str,
                             seed: int,
                             max_scene_tries: int,
                             balanced: bool) -> SceneSpec:
    robot = make_robot(robot_name)
    last_error: Exception | None = None
    for scene_try in range(max(1, int(max_scene_tries))):
        rng = random.Random(seed + 1000003 * scene_try)
        obstacles: list[sbf.Obstacle] = []
        try:
            start, goal = sample_free_pair(robot, obstacles, rng)
            target_random_obstacles = random_obstacle_count(difficulty)
            base = random_obstacle_scale(difficulty)
            blocker_added = append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, base, require_direct_blocker=True)
            if not blocker_added:
                continue
            while len(obstacles) < target_random_obstacles:
                if not append_random_scene_obstacle(robot_name, robot, obstacles, start, goal, rng, base, require_direct_blocker=False):
                    break
            if len(obstacles) < target_random_obstacles:
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
                endpoint_clearance_margin_m=ENDPOINT_CLEARANCE_MARGIN_M,
                fixed_robot_clearance_margin_m=FIXED_ROBOT_CLEARANCE_MARGIN_M,
                direct_segment_blocked=True,
                segment_resolution=SEGMENT_RESOLUTION,
            )
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"could not sample a valid {robot_name}/{difficulty} legacy scene after {max_scene_tries} attempts: {last_error}")


def make_random_scene(robot_name: str,
                      difficulty: str,
                      seed: int,
                      max_scene_tries: int = 64,
                      scene_profile: str = "balanced") -> SceneSpec:
    if scene_profile_uses_nested_prefixes(scene_profile):
        return make_nested_random_scene(
            robot_name,
            difficulty,
            seed,
            max_scene_tries,
            balanced=scene_profile_requires_balanced_probe(scene_profile),
        )
    return make_legacy_random_scene(robot_name, difficulty, seed, max_scene_tries, balanced=False)
