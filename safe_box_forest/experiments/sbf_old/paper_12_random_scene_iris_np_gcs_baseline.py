#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_rrt_connect import path_length as list_path_length, segment_free  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, mean, median, sbf, write_json  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import DEFAULT_RANDOM_DIFFICULTIES, DEFAULT_RANDOM_ROBOTS, DEFAULT_RANDOM_SCENE_SEEDS, make_random_scene, make_robot  # noqa: E402
from sbf.marcucci import iiwa14_robot_json  # noqa: E402


DEFAULT_IRIS_NP = {
    "iteration_limit": 3,
    "termination_threshold": -1.0,
    "relative_termination_threshold": 2e-2,
    "num_collision_infeasible_samples": 1,
    "edge_step_size": 0.05,
    "env_padding": 0.0,
    "self_padding": 0.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Current Drake IRIS-NP+GCS baseline for SBF random robot scenes.")
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--difficulties", default=DEFAULT_RANDOM_DIFFICULTIES)
    parser.add_argument("--scene-seeds", type=int, default=DEFAULT_RANDOM_SCENE_SEEDS)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--logical-threads", type=int, default=8)
    parser.add_argument("--budget-s", type=float, default=120.0)
    parser.add_argument("--max-region-seeds", type=int, default=5)
    parser.add_argument("--iteration-limit", type=int, default=DEFAULT_IRIS_NP["iteration_limit"])
    parser.add_argument("--relative-termination-threshold", type=float, default=DEFAULT_IRIS_NP["relative_termination_threshold"])
    parser.add_argument("--query-time-limit-s", type=float, default=60.0)
    parser.add_argument("--rounding-max-paths", type=int, default=10)
    parser.add_argument("--rounding-max-trials", type=int, default=100)
    parser.add_argument("--gcs-preprocessing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-rounding", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--allow-repair", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--segment-step", type=float, default=0.06)
    parser.add_argument("--guide-timeout-ms", type=float, default=750.0)
    parser.add_argument("--guide-range", type=float, default=0.35)
    parser.add_argument("--guide-simplify-time-s", type=float, default=0.0)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_exp05_random_iris_np_gcs_full.json")
    return parser.parse_args()


def parse_csv(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def configure_threads(logical_threads: int) -> None:
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[name] = str(max(1, int(logical_threads)))


def robot_dh_rows(robot_name: str) -> list[dict[str, float | int]]:
    if robot_name == "ur5":
        rows = [
            (math.pi / 2.0, 0.0, 0.0892, 0.0, 0),
            (0.0, -0.425, 0.0, 0.0, 0),
            (0.0, -0.392, 0.0, 0.0, 0),
            (math.pi / 2.0, 0.0, 0.109, 0.0, 0),
            (-math.pi / 2.0, 0.0, 0.095, 0.0, 0),
            (0.0, 0.0, 0.082, 0.0, 0),
        ]
    elif robot_name == "panda":
        rows = [
            (-math.pi / 2.0, 0.0, 0.333, 0.0, 0),
            (math.pi / 2.0, 0.0, 0.0, 0.0, 0),
            (math.pi / 2.0, 0.0, 0.316, 0.0, 0),
            (-math.pi / 2.0, 0.0825, 0.0, 0.0, 0),
            (math.pi / 2.0, -0.0825, 0.384, 0.0, 0),
            (math.pi / 2.0, 0.0, 0.0, 0.0, 0),
            (0.0, 0.088, 0.107, 0.0, 0),
        ]
    elif robot_name == "iiwa":
        raw = json.loads(iiwa14_robot_json().read_text(encoding="utf-8"))
        rows = [
            (
                float(item.get("alpha", 0.0)),
                float(item.get("a", 0.0)),
                float(item.get("d", 0.0)),
                float(item.get("theta", 0.0)),
                int(item.get("joint_type", 0)),
            )
            for item in raw.get("dh_params", [])
        ]
    else:
        raise ValueError(f"unknown robot '{robot_name}'")
    return [
        {"alpha": float(alpha), "a": float(a), "d": float(d), "theta": float(theta), "joint_type": int(joint_type)}
        for alpha, a, d, theta, joint_type in rows
    ]


def fixed_dh_translation(row: dict[str, float | int]) -> np.ndarray:
    alpha = float(row["alpha"])
    return np.asarray([float(row["a"]), -float(row["d"]) * math.sin(alpha), float(row["d"]) * math.cos(alpha)], dtype=float)


def fixed_dh_transform(row: dict[str, float | int]):
    from pydrake.math import RigidTransform, RotationMatrix

    return RigidTransform(RotationMatrix.MakeXRotation(float(row["alpha"])), fixed_dh_translation(row))


def rotation_z_to_vector(direction: np.ndarray):
    from pydrake.math import RotationMatrix

    z_axis = np.asarray(direction, dtype=float)
    norm = float(np.linalg.norm(z_axis))
    if norm <= 1e-12:
        return RotationMatrix()
    z_axis /= norm
    helper = np.asarray([1.0, 0.0, 0.0], dtype=float) if abs(float(z_axis[0])) < 0.9 else np.asarray([0.0, 1.0, 0.0], dtype=float)
    y_axis = np.cross(z_axis, helper)
    y_axis /= max(float(np.linalg.norm(y_axis)), 1e-12)
    x_axis = np.cross(y_axis, z_axis)
    return RotationMatrix(np.column_stack([x_axis, y_axis, z_axis]))


def capsule_pose_for_segment(p0: np.ndarray, p1: np.ndarray):
    from pydrake.math import RigidTransform

    direction = np.asarray(p1, dtype=float) - np.asarray(p0, dtype=float)
    center = 0.5 * (np.asarray(p0, dtype=float) + np.asarray(p1, dtype=float))
    return RigidTransform(rotation_z_to_vector(direction), center)


def obstacle_bounds(obstacle: Any) -> list[float]:
    return [float(value) for value in list(obstacle.bounds)]


def build_drake_random_scene(robot_name: str, obstacles: list[Any]):
    from pydrake.geometry import Box, Capsule, CollisionFilterDeclaration, GeometrySet
    from pydrake.multibody.plant import CoulombFriction
    from pydrake.multibody.tree import FixedOffsetFrame, RevoluteJoint, SpatialInertia, UnitInertia
    from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker

    robot = make_robot(robot_name)
    dh_rows = robot_dh_rows(robot_name)
    if len(dh_rows) != int(robot.n_joints()):
        raise RuntimeError(f"DH row mismatch for {robot_name}: {len(dh_rows)} vs {robot.n_joints()}")

    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    model_instance = plant.AddModelInstance(f"{robot_name}_sbf_dh_capsule")
    inertia = SpatialInertia(mass=1.0, p_PScm_E=[0.0, 0.0, 0.0], G_SP_E=UnitInertia.SolidSphere(0.01))
    bodies = []
    joints = []
    limits = list(robot.joint_limits().limits)
    for index, row in enumerate(dh_rows):
        body = plant.AddRigidBody(f"link_{index + 1}", model_instance, inertia)
        bodies.append(body)
        parent_frame = plant.world_frame() if index == 0 else bodies[index - 1].body_frame()
        joint_frame = plant.AddFrame(FixedOffsetFrame(f"joint_{index}_pre_dh", parent_frame, fixed_dh_transform(row)))
        joint = plant.AddJoint(RevoluteJoint(f"q{index}", joint_frame, body.body_frame(), [0.0, 0.0, 1.0]))
        if index < len(limits):
            joint.set_position_limits([float(limits[index].lo)], [float(limits[index].hi)])
        joints.append(joint)

    friction = CoulombFriction(0.9, 0.8)
    robot_geometry_ids = []
    active_map = list(robot.active_link_map())
    active_radii = list(robot.active_link_radii())
    for active_index, link_index in enumerate(active_map):
        parent_index = int(link_index)
        parent_body = plant.world_body() if parent_index == 0 else bodies[min(parent_index, len(bodies)) - 1]
        segment_row_index = min(max(0, parent_index), len(dh_rows) - 1)
        segment = fixed_dh_translation(dh_rows[segment_row_index])
        length = float(np.linalg.norm(segment))
        if length <= 1e-8:
            continue
        radius = float(active_radii[active_index])
        geometry_id = plant.RegisterCollisionGeometry(
            parent_body,
            capsule_pose_for_segment(np.zeros(3), segment),
            Capsule(radius, length),
            f"sbf_capsule_link_{link_index}",
            friction,
        )
        robot_geometry_ids.append(geometry_id)

    for obs_index, obstacle in enumerate(obstacles):
        x0, y0, z0, x1, y1, z1 = obstacle_bounds(obstacle)
        center = [0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)]
        size = [max(x1 - x0, 1e-6), max(y1 - y0, 1e-6), max(z1 - z0, 1e-6)]
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            __import__("pydrake.math", fromlist=["RigidTransform"]).RigidTransform(center),
            Box(*size),
            f"random_aabb_{obs_index}",
            friction,
        )

    if robot_geometry_ids:
        scene_graph.collision_filter_manager().Apply(CollisionFilterDeclaration().ExcludeWithin(GeometrySet(robot_geometry_ids)))
    plant.Finalize()
    robot_diagram = builder.Build()
    checker = SceneGraphCollisionChecker(
        model=robot_diagram,
        robot_model_instances=[model_instance],
        edge_step_size=DEFAULT_IRIS_NP["edge_step_size"],
        env_collision_padding=DEFAULT_IRIS_NP["env_padding"],
        self_collision_padding=DEFAULT_IRIS_NP["self_padding"],
    )
    return robot_diagram, plant, model_instance, checker


def guide_path(args: argparse.Namespace, robot: Any, obstacles: list[Any], start: list[float], goal: list[float], seed: int) -> tuple[list[list[float]], float, dict[str, Any]]:
    t0 = time.perf_counter()
    try:
        result = dict(sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            list(start),
            list(goal),
            float(args.guide_timeout_ms),
            float(args.guide_range),
            float(args.segment_step),
            float(args.guide_simplify_time_s),
            int(seed),
        ))
    except Exception as exc:
        return [list(start), list(goal)], time.perf_counter() - t0, {"ok": False, "reason": str(exc)}
    elapsed = float(result.get("t_s", time.perf_counter() - t0))
    path = [list(point) for point in result.get("path", [])]
    ok = bool(result.get("ok")) and len(path) >= 2
    if not ok:
        path = [list(start), list(goal)]
    return path, elapsed, result


def region_seed_points(path: list[list[float]], max_regions: int) -> list[np.ndarray]:
    if not path:
        return []
    if len(path) <= max(1, int(max_regions)):
        return [np.asarray(point, dtype=float) for point in path]
    indices = np.linspace(0, len(path) - 1, max(2, int(max_regions))).round().astype(int)
    unique: list[int] = []
    for index in indices:
        if int(index) not in unique:
            unique.append(int(index))
    return [np.asarray(path[index], dtype=float) for index in unique]


def drake_path_length(path: np.ndarray) -> float:
    return float(sum(np.linalg.norm(path[index] - path[index - 1]) for index in range(1, len(path))))


def edge_unsafe_segments(path: np.ndarray, checker: Any) -> list[int]:
    return [index - 1 for index in range(1, len(path)) if not checker.CheckEdgeCollisionFree(path[index - 1], path[index])]


def limits_arrays(robot: Any) -> tuple[np.ndarray, np.ndarray]:
    intervals = list(robot.joint_limits().limits)
    return (
        np.asarray([float(interval.lo) for interval in intervals], dtype=float),
        np.asarray([float(interval.hi) for interval in intervals], dtype=float),
    )


def root_path(nodes: list[np.ndarray], parents: list[int], index: int) -> list[np.ndarray]:
    out: list[np.ndarray] = []
    while index >= 0:
        out.append(nodes[index])
        index = parents[index]
    return out[::-1]


def local_rrt_connect(q_start: np.ndarray, q_goal: np.ndarray, checker: Any, rng: np.random.Generator,
                      lo: np.ndarray, hi: np.ndarray, *, deadline: float, step_size: float = 0.18,
                      max_nodes: int = 4500) -> list[np.ndarray] | None:
    if checker.CheckEdgeCollisionFree(q_start, q_goal):
        return [q_start, q_goal]
    start_nodes = [q_start]
    start_parents = [-1]
    goal_nodes = [q_goal]
    goal_parents = [-1]

    def nearest(nodes: list[np.ndarray], q: np.ndarray) -> int:
        return int(np.argmin([float(np.linalg.norm(node - q)) for node in nodes]))

    def steer(q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        delta = q_to - q_from
        dist = float(np.linalg.norm(delta))
        return q_to.copy() if dist <= step_size else q_from + (step_size / dist) * delta

    def try_extend(nodes: list[np.ndarray], parents: list[int], target: np.ndarray) -> int | None:
        if len(nodes) >= max_nodes:
            return None
        near_index = nearest(nodes, target)
        candidate = np.minimum(np.maximum(steer(nodes[near_index], target), lo), hi)
        if np.linalg.norm(candidate - nodes[near_index]) < 1e-9:
            return None
        if not checker.CheckConfigCollisionFree(candidate) or not checker.CheckEdgeCollisionFree(nodes[near_index], candidate):
            return None
        nodes.append(candidate)
        parents.append(near_index)
        return len(nodes) - 1

    def try_connect(nodes: list[np.ndarray], parents: list[int], target: np.ndarray) -> int | None:
        best_index = nearest(nodes, target)
        while len(nodes) < max_nodes and time.perf_counter() < deadline:
            candidate = np.minimum(np.maximum(steer(nodes[best_index], target), lo), hi)
            if not checker.CheckConfigCollisionFree(candidate) or not checker.CheckEdgeCollisionFree(nodes[best_index], candidate):
                return None
            nodes.append(candidate)
            parents.append(best_index)
            best_index = len(nodes) - 1
            if np.linalg.norm(candidate - target) < 1e-8 or checker.CheckEdgeCollisionFree(candidate, target):
                return best_index
        return None

    while time.perf_counter() < deadline and len(start_nodes) + len(goal_nodes) < 2 * max_nodes:
        if rng.random() < 0.75:
            alpha = float(rng.random())
            sigma = 0.18 + 0.18 * float(rng.random())
            target = (1.0 - alpha) * q_start + alpha * q_goal + rng.normal(0.0, sigma, size=q_start.shape)
            target = np.minimum(np.maximum(target, lo), hi)
        else:
            target = rng.uniform(lo, hi)
        start_index = try_extend(start_nodes, start_parents, target)
        if start_index is not None:
            goal_index = try_connect(goal_nodes, goal_parents, start_nodes[start_index])
            if goal_index is not None:
                return root_path(start_nodes, start_parents, start_index) + root_path(goal_nodes, goal_parents, goal_index)[::-1]
        goal_index = try_extend(goal_nodes, goal_parents, target)
        if goal_index is not None:
            start_index = try_connect(start_nodes, start_parents, goal_nodes[goal_index])
            if start_index is not None:
                return root_path(start_nodes, start_parents, start_index) + root_path(goal_nodes, goal_parents, goal_index)[::-1]
    return None


def repair_unsafe_path(path: np.ndarray, checker: Any, robot: Any, *, seed: int, per_segment_budget_s: float = 2.0) -> tuple[np.ndarray | None, float, int]:
    lo, hi = limits_arrays(robot)
    rng = np.random.default_rng(71059 + int(seed))
    t0 = time.perf_counter()
    repaired = [path[0]]
    repaired_segments = 0
    for index in range(1, len(path)):
        previous = repaired[-1]
        current = path[index]
        if checker.CheckEdgeCollisionFree(previous, current):
            repaired.append(current)
            continue
        segment = local_rrt_connect(previous, current, checker, rng, lo, hi, deadline=time.perf_counter() + per_segment_budget_s)
        if segment is None:
            return None, time.perf_counter() - t0, repaired_segments
        repaired.extend(segment[1:])
        repaired_segments += 1
    repaired_array = np.asarray(repaired, dtype=float)
    if edge_unsafe_segments(repaired_array, checker):
        return None, time.perf_counter() - t0, repaired_segments
    return repaired_array, time.perf_counter() - t0, repaired_segments


def solve_regions_gcs(q_start: np.ndarray, q_goal: np.ndarray, regions: list[Any], *, seed: int, checker: Any,
                      robot: Any, query_time_limit_s: float, allow_repair: bool, rounding_max_paths: int,
                      rounding_max_trials: int, gcs_preprocessing: bool, use_rounding: bool) -> dict[str, Any]:
    import importlib
    from pydrake.solvers import MosekSolver, SolverOptions

    LinearGCS = importlib.import_module("gcs.linear").LinearGCS
    random_forward_path_search = importlib.import_module("gcs.rounding").randomForwardPathSearch
    t0 = time.perf_counter()
    if not regions:
        return {"success": False, "time_s": 0.0, "path_length": None, "regions": 0, "edges": 0, "note": "no regions"}
    gcs = LinearGCS(regions)
    try:
        gcs.addSourceTarget(q_start, q_goal)
    except ValueError as exc:
        return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "regions": len(regions), "edges": 0, "note": f"addSourceTarget failed: {exc}"}
    gcs.setRoundingStrategy(random_forward_path_search, max_paths=max(1, int(rounding_max_paths)), max_trials=max(1, int(rounding_max_trials)), seed=int(seed))
    solver_options = SolverOptions()
    solver_options.SetOption(MosekSolver.id(), "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3)
    if query_time_limit_s > 0.0:
        solver_options.SetOption(MosekSolver.id(), "MSK_DPAR_OPTIMIZER_MAX_TIME", float(query_time_limit_s))
    gcs.setSolverOptions(solver_options)

    class GcsTimeout(Exception):
        pass

    def timeout_handler(_signum, _frame):
        raise GcsTimeout("GCS solve timeout")

    previous_handler = signal.getsignal(signal.SIGALRM)
    try:
        signal.signal(signal.SIGALRM, timeout_handler)
        signal.setitimer(signal.ITIMER_REAL, float(query_time_limit_s))
        waypoints, _ = gcs.SolvePath(rounding=bool(use_rounding), verbose=False, preprocessing=bool(gcs_preprocessing))
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, previous_handler)
    except GcsTimeout:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, previous_handler)
        return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "note": "GCS solve timeout"}
    except RuntimeError as exc:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, previous_handler)
        return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "note": f"GCS solve raised RuntimeError: {exc}"}
    if waypoints is None:
        return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "note": "GCS solve failed"}
    path = waypoints.T
    raw_length = drake_path_length(path)
    unsafe = edge_unsafe_segments(path, checker)
    if unsafe:
        if not allow_repair:
            return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "raw_path_length": raw_length, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": False, "unsafe_segments": len(unsafe), "note": f"GCS path failed Drake collision validation on {len(unsafe)} segment(s)"}
        repaired, repair_s, repaired_segments = repair_unsafe_path(path, checker, robot, seed=seed)
        if repaired is None:
            return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "raw_path_length": raw_length, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": False, "unsafe_segments": len(unsafe), "repair_time_s": repair_s, "note": f"GCS path failed Drake collision validation on {len(unsafe)} segment(s)"}
        return {"success": True, "time_s": time.perf_counter() - t0, "path_length": drake_path_length(repaired), "path": repaired.tolist(), "raw_path_length": raw_length, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(repaired.shape[0]), "raw_waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": True, "unsafe_segments": 0, "gcs_unsafe_segments": len(unsafe), "repaired_segments": repaired_segments, "repair_time_s": repair_s, "note": "GCS path repaired by local Drake collision-checked RRT-Connect"}
    return {"success": True, "time_s": time.perf_counter() - t0, "path_length": raw_length, "path": path.tolist(), "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": True, "unsafe_segments": 0}


def build_regions(args: argparse.Namespace, robot_diagram: Any, plant: Any, model_instance: Any, checker: Any,
                  seed_points: list[np.ndarray], seed: int, budget_s: float) -> tuple[list[Any], list[float], list[dict[str, Any]]]:
    from pydrake.all import IrisNp, IrisOptions

    context = robot_diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    opts = IrisOptions()
    opts.require_sample_point_is_contained = True
    opts.iteration_limit = int(args.iteration_limit)
    opts.termination_threshold = DEFAULT_IRIS_NP["termination_threshold"]
    opts.relative_termination_threshold = float(args.relative_termination_threshold)
    opts.num_collision_infeasible_samples = DEFAULT_IRIS_NP["num_collision_infeasible_samples"]
    opts.random_seed = int(seed)
    regions: list[Any] = []
    timings: list[float] = []
    failures: list[dict[str, Any]] = []
    cumulative = 0.0
    for index, q in enumerate(seed_points):
        if cumulative >= budget_s:
            break
        if not checker.CheckConfigCollisionFree(q):
            failures.append({"seed_index": index, "note": "seed in collision", "time_s": 0.0})
            continue
        plant.SetPositions(plant_context, model_instance, q)
        t0 = time.perf_counter()
        try:
            region = IrisNp(plant, plant_context, opts)
            dt = time.perf_counter() - t0
            regions.append(region)
            timings.append(float(dt))
            cumulative += dt
            print(f"[exp5-iris] region index={index} dt_s={dt:.3f}", flush=True)
        except Exception as exc:
            dt = time.perf_counter() - t0
            timings.append(float(dt))
            cumulative += dt
            failures.append({"seed_index": index, "note": str(exc), "time_s": float(dt)})
            print(f"[exp5-iris] region failed index={index} dt_s={dt:.3f} note={exc}", flush=True)
    return regions, timings, failures


def sbf_audit_path(robot: Any, obstacles: list[Any], path: list[list[float]], step: float) -> bool:
    if len(path) < 2:
        return False
    return all(segment_free(robot, obstacles, path[index], path[index + 1], step) for index in range(len(path) - 1))


def failure_row(robot_name: str, difficulty: str, scene_seed: int, trial: int, reason: str) -> dict[str, Any]:
    return {
        "robot": robot_name,
        "difficulty": difficulty,
        "method": "drake_iris_np_gcs",
        "scene_seed": int(scene_seed),
        "trial": int(trial),
        "scene_valid": False,
        "ok": False,
        "audit_passed": False,
        "reason": reason,
        "build_s": 0.0,
        "query_s": 0.0,
        "path_length": 0.0,
        "n_regions": 0,
    }


def run_case(args: argparse.Namespace, robot_name: str, difficulty: str, scene_seed: int, trial: int) -> dict[str, Any]:
    print(f"[exp5-iris] start robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial}", flush=True)
    robot = make_robot(robot_name)
    try:
        scene = make_random_scene(robot_name, difficulty, int(args.seed_base) + 1009 * scene_seed, scene_profile=args.scene_profile)
    except RuntimeError as exc:
        return failure_row(robot_name, difficulty, scene_seed, trial, f"scene_generation_failed:{exc}")
    planner_seed = int(args.seed_base) + 73856093 * scene_seed + 19349663 * trial
    guide, guide_s, guide_result = guide_path(args, robot, scene.obstacles, scene.start, scene.goal, planner_seed)
    robot_diagram, plant, model_instance, checker = build_drake_random_scene(robot_name, scene.obstacles)
    seeds = [point for point in region_seed_points(guide, int(args.max_region_seeds)) if checker.CheckConfigCollisionFree(point)]
    if not seeds:
        seeds = [np.asarray(scene.start, dtype=float), np.asarray(scene.goal, dtype=float)]
    regions, timings, failures = build_regions(args, robot_diagram, plant, model_instance, checker, seeds, planner_seed, float(args.budget_s))
    result = solve_regions_gcs(
        np.asarray(scene.start, dtype=float),
        np.asarray(scene.goal, dtype=float),
        regions,
        seed=planner_seed,
        checker=checker,
        robot=robot,
        query_time_limit_s=float(args.query_time_limit_s),
        allow_repair=bool(args.allow_repair),
        rounding_max_paths=int(args.rounding_max_paths),
        rounding_max_trials=int(args.rounding_max_trials),
        gcs_preprocessing=bool(args.gcs_preprocessing),
        use_rounding=bool(args.use_rounding),
    )
    path = [list(point) for point in result.get("path", [])]
    drake_success = bool(result.get("success")) and len(path) >= 2
    audit_passed = sbf_audit_path(robot, scene.obstacles, path, float(args.segment_step)) if drake_success else False
    ok = drake_success and audit_passed
    print(
        f"[exp5-iris] done robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial} "
        f"regions={len(regions)} ok={ok} drake_success={drake_success} audit={audit_passed} build_s={guide_s + sum(timings):.3f} query_s={float(result.get('time_s', 0.0)):.3f}",
        flush=True,
    )
    row = {
        "robot": robot_name,
        "difficulty": difficulty,
        "method": "drake_iris_np_gcs",
        "scene_seed": int(scene_seed),
        "trial": int(trial),
        "scene_valid": True,
        "obstacle_count": len(scene.obstacles),
        "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
        "direct_segment_blocked": bool(scene.direct_segment_blocked),
        "segment_resolution": int(scene.segment_resolution),
        "rng_seed": int(planner_seed),
        "ok": bool(ok),
        "drake_success": bool(drake_success),
        "audit_passed": bool(audit_passed),
        "reason": None if ok else str(result.get("note", "audit_failed" if drake_success else "gcs_failed")),
        "build_s": float(guide_s + sum(timings)),
        "guide_s": float(guide_s),
        "guide_ok": bool(guide_result.get("ok")),
        "guide_waypoints": len(guide),
        "query_s": float(result.get("time_s", 0.0)) if drake_success else 0.0,
        "path_length": list_path_length(path) if ok else 0.0,
        "drake_path_length": float(result.get("path_length", 0.0) or 0.0) if drake_success else 0.0,
        "path_waypoint_count": len(path),
        "n_regions": len(regions),
        "region_build_s": [float(value) for value in timings],
        "failed_region_seeds": failures,
        "gcs_result": {key: value for key, value in result.items() if key != "path"},
    }
    return row


def summarize_metric(values: list[float]) -> dict[str, float | None]:
    return {"mean": mean(values), "median": median(values)}


def aggregate(rows: list[dict[str, Any]], robots: list[str], difficulties: list[str]) -> dict[str, Any]:
    groups: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            subset = [row for row in rows if row["robot"] == robot_name and row["difficulty"] == difficulty]
            successes = [row for row in subset if row.get("ok")]
            groups.append({
                "robot": robot_name,
                "difficulty": difficulty,
                "methods": {
                    "drake_iris_np_gcs": {
                        "build_time_s": summarize_metric([float(row.get("build_s", 0.0)) for row in subset]),
                        "query_time_s": summarize_metric([float(row.get("query_s", 0.0)) for row in successes]),
                        "path_length": summarize_metric([float(row.get("path_length", 0.0)) for row in successes]),
                        "success_rate": mean(1.0 if row.get("ok") else 0.0 for row in subset),
                        "audit_success_rate": mean(1.0 if row.get("audit_passed") else 0.0 for row in subset),
                        "n_runs": len(subset),
                        "n_success": len(successes),
                    }
                },
            })
    return {"groups": groups}


def main() -> int:
    args = parse_args()
    configure_threads(args.logical_threads)
    robots = parse_csv(args.robots)
    difficulties = parse_csv(args.difficulties)
    rows: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            for scene_seed in range(max(1, int(args.scene_seeds))):
                for trial in range(max(1, int(args.trials))):
                    rows.append(run_case(args, robot_name, difficulty, scene_seed, trial))
    payload = {
        "experiment": "paper_12_random_scene_iris_np_gcs_baseline",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_sbf_random_scene_drake_iris_np_gcs_dh_capsule_live_run",
        "note": "Current-version random-scene Drake IRIS-NP+GCS baseline. UR5/Panda plants are reconstructed from the same standalone SBF DH link model and active-link capsule radii used by the random-scene SBF/OMPL collision checker; deterministic guide-seed generation time is charged to build time and no samples are discarded.",
        "random_scene_checks": {
            "endpoint_clearance_margin_m": 0.12,
            "direct_start_goal_segment_blocked": True,
            "segment_resolution": 96,
            "scene_profile": args.scene_profile,
        },
        "params": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "aggregation": aggregate(rows, robots, difficulties),
        "rows": rows,
    }
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "rows": len(rows), "aggregation": payload["aggregation"]}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
