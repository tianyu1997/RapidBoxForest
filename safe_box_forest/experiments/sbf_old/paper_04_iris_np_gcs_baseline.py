#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, REPO_ROOT, mean, median, write_json  # noqa: E402
from sbf.marcucci import ANCHORS, iiwa14_robot_json, make_combined_queries  # noqa: E402


WORKSPACE = REPO_ROOT
GCS_REPO = WORKSPACE / "gcs-science-robotics"
DEFAULT_OUT = ROOT / "outputs" / "paper" / "marcucci_iris_np_gcs.json"
DEFAULT_IRIS_NP = {
    "iteration_limit": 10,
    "termination_threshold": -1.0,
    "relative_termination_threshold": 2e-2,
    "num_collision_infeasible_samples": 1,
    "edge_step_size": 0.05,
    "env_padding": 0.0,
    "self_padding": 0.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Current SBF Drake IRIS-NP+GCS baseline for the shelf+IIWA scene.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true")
    mode.add_argument("--full", action="store_true")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--seed-base", type=int, default=20260507)
    parser.add_argument("--logical-threads", type=int, default=8)
    parser.add_argument("--budget-s", type=float, default=800.0)
    parser.add_argument("--iteration-limit", type=int, default=DEFAULT_IRIS_NP["iteration_limit"])
    parser.add_argument("--relative-termination-threshold", type=float, default=DEFAULT_IRIS_NP["relative_termination_threshold"])
    parser.add_argument("--query-time-limit-s", type=float, default=120.0)
    parser.add_argument("--allow-repair", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rounding-max-paths", type=int, default=10)
    parser.add_argument("--rounding-max-trials", type=int, default=100)
    parser.add_argument("--gcs-preprocessing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-rounding", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gcs-repo", type=Path, default=GCS_REPO)
    return parser.parse_args()


def mode_counts(args: argparse.Namespace) -> tuple[bool, int, int]:
    quick = bool(args.quick or not args.full)
    seeds = int(args.seeds if args.seeds is not None else (1 if quick else 5))
    timeout = int(args.timeout if args.timeout is not None else (30 if quick else 120))
    return quick, max(1, seeds), max(1, timeout)


def configure_threads(logical_threads: int) -> None:
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[name] = str(max(1, int(logical_threads)))


def bootstrap_gcs_repo(path: Path) -> Path:
    repo = path.resolve(strict=False)
    if not repo.is_dir():
        raise FileNotFoundError(f"gcs-science-robotics repo not found: {repo}")
    text = str(repo)
    if text not in sys.path:
        sys.path.insert(0, text)
    return repo


def configure_parser_package_map(parser: Any, gcs_repo: Path) -> None:
    package_map = parser.package_map()
    package_map.Add("gcs", str(gcs_repo))
    drake_package_dir = os.environ.get("DRAKE_PACKAGE_DIR")
    if drake_package_dir:
        drake_path = Path(drake_package_dir)
        if drake_path.is_dir() and not (hasattr(package_map, "Contains") and package_map.Contains("drake")):
            package_map.Add("drake", str(drake_path))


def load_robot_joint_limits() -> tuple[np.ndarray, np.ndarray]:
    raw = json.loads(iiwa14_robot_json().read_text(encoding="utf-8"))
    limits = raw["joint_limits"]
    lo = np.asarray([float(item[0]) for item in limits], dtype=float)
    hi = np.asarray([float(item[1]) for item in limits], dtype=float)
    return lo, hi


def workload() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query in make_combined_queries():
        rows.append({
            "label": query.label,
            "file": f"{query.start_name}_{query.goal_name}.json",
            "q_start": list(query.start),
            "q_goal": list(query.goal),
        })
    return rows


def anchor_cycle() -> list[tuple[str, np.ndarray]]:
    names = ["AS", "TS", "CS", "LB", "RB"]
    return [(name, np.asarray(ANCHORS[name], dtype=float)) for name in names]


def region_seed_configs(items: list[dict[str, Any]]) -> list[tuple[str, np.ndarray]]:
    seeds = list(anchor_cycle())
    for item in items:
        midpoint = 0.5 * (np.asarray(item["q_start"], dtype=float) + np.asarray(item["q_goal"], dtype=float))
        seeds.append((f"mid_{item['label']}", midpoint))
    return seeds


def build_robot_diagram_checker(gcs_repo: Path):
    from pydrake.multibody.parsing import LoadModelDirectives, ProcessModelDirectives
    from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker

    builder = RobotDiagramBuilder(time_step=0.0)
    parser = builder.parser()
    configure_parser_package_map(parser, gcs_repo)
    directives_file = gcs_repo / "models" / "iiwa14_spheres_collision_welded_gripper.yaml"
    directives = LoadModelDirectives(str(directives_file))
    ProcessModelDirectives(directives, builder.plant(), parser)
    builder.plant().Finalize()
    plant = builder.plant()
    iiwa_inst = plant.GetModelInstanceByName("iiwa")
    wsg_inst = plant.GetModelInstanceByName("wsg")
    robot_diagram = builder.Build()
    checker = SceneGraphCollisionChecker(
        model=robot_diagram,
        robot_model_instances=[iiwa_inst, wsg_inst],
        edge_step_size=DEFAULT_IRIS_NP["edge_step_size"],
        env_collision_padding=DEFAULT_IRIS_NP["env_padding"],
        self_collision_padding=DEFAULT_IRIS_NP["self_padding"],
    )
    return robot_diagram, plant, checker


def path_length(path: np.ndarray) -> float:
    return float(sum(np.linalg.norm(path[index] - path[index - 1]) for index in range(1, len(path))))


def edge_unsafe_segments(path: np.ndarray, checker: Any) -> list[int]:
    return [index - 1 for index in range(1, len(path)) if not checker.CheckEdgeCollisionFree(path[index - 1], path[index])]


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


def repair_unsafe_path(path: np.ndarray, checker: Any, *, seed: int, per_segment_budget_s: float = 3.0) -> tuple[np.ndarray | None, float, int]:
    lo, hi = load_robot_joint_limits()
    rng = np.random.default_rng(51047 + int(seed))
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
                      query_time_limit_s: float, allow_repair: bool, rounding_max_paths: int,
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
    raw_length = path_length(path)
    unsafe = edge_unsafe_segments(path, checker)
    if unsafe:
        if not allow_repair:
            return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "raw_path_length": raw_length, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": False, "unsafe_segments": len(unsafe), "note": f"GCS path failed collision validation on {len(unsafe)} segment(s)"}
        repaired, repair_s, repaired_segments = repair_unsafe_path(path, checker, seed=seed)
        if repaired is None:
            return {"success": False, "time_s": time.perf_counter() - t0, "path_length": None, "raw_path_length": raw_length, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": False, "unsafe_segments": len(unsafe), "repair_time_s": repair_s, "note": f"GCS path failed collision validation on {len(unsafe)} segment(s)"}
        return {"success": True, "time_s": time.perf_counter() - t0, "path_length": path_length(repaired), "path": repaired.tolist(), "raw_path_length": raw_length, "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(repaired.shape[0]), "raw_waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": True, "unsafe_segments": 0, "gcs_unsafe_segments": len(unsafe), "repaired_segments": repaired_segments, "repair_time_s": repair_s, "note": "GCS path repaired by local collision-checked RRT-Connect"}
    return {"success": True, "time_s": time.perf_counter() - t0, "path_length": raw_length, "path": path.tolist(), "regions": len(regions), "edges": len([edge for edge in gcs.gcs.Edges()]), "waypoints_count": int(path.shape[0]), "collision_checked": True, "collision_free": True, "unsafe_segments": 0}


def build_regions_for_seed(args: argparse.Namespace, seed: int, seed_configs: list[tuple[str, np.ndarray]], budget_s: float):
    from pydrake.all import IrisNp, IrisOptions

    gcs_repo = bootstrap_gcs_repo(args.gcs_repo)
    robot_diagram, plant, checker = build_robot_diagram_checker(gcs_repo)
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
    for name, q in seed_configs:
        if cumulative >= budget_s:
            break
        plant.SetPositions(plant_context, q)
        t0 = time.perf_counter()
        try:
            region = IrisNp(plant, plant_context, opts)
            dt = time.perf_counter() - t0
            regions.append(region)
            timings.append(float(dt))
            cumulative += dt
            print(f"[exp4-iris-np] region seed={seed} name={name} dt_s={dt:.3f}", flush=True)
        except Exception as exc:
            dt = time.perf_counter() - t0
            timings.append(float(dt))
            cumulative += dt
            failures.append({"seed_name": name, "note": str(exc), "time_s": float(dt)})
            print(f"[exp4-iris-np] region failed seed={seed} name={name} dt_s={dt:.3f} note={exc}", flush=True)
    return regions, timings, failures, checker


def empty_query_record(label: str, seed: int, note: str, failure_time_s: float | None = None) -> dict[str, Any]:
    return {"query": label, "name": label, "seed": seed, "success": False, "time_s": None, "path_length": None, "note": note, "failure_time_s": failure_time_s}


def run_seed_trial(args: argparse.Namespace, seed_index: int, items: list[dict[str, Any]]) -> dict[str, Any]:
    seed = int(args.seed_base) + int(seed_index)
    regions, timings, failures, checker = build_regions_for_seed(args, seed, region_seed_configs(items), float(args.budget_s))
    trial = {"seed": int(seed_index), "rng_seed": seed, "build_s": float(sum(timings)), "n_regions": len(regions), "per_region_s": [float(value) for value in timings], "queries": []}
    if failures:
        trial["failed_region_seeds"] = failures
    for item in items:
        print(f"[exp4-iris-np] query seed={seed_index} label={item['label']} regions={len(regions)}", flush=True)
        result = solve_regions_gcs(
            np.asarray(item["q_start"], dtype=float),
            np.asarray(item["q_goal"], dtype=float),
            regions,
            seed=seed,
            checker=checker,
            query_time_limit_s=float(args.query_time_limit_s),
            allow_repair=bool(args.allow_repair),
            rounding_max_paths=int(args.rounding_max_paths),
            rounding_max_trials=int(args.rounding_max_trials),
            gcs_preprocessing=bool(args.gcs_preprocessing),
            use_rounding=bool(args.use_rounding),
        )
        if not result.get("success"):
            record = empty_query_record(item["label"], seed_index, str(result.get("note", "IRIS-NP+GCS query failed")), result.get("time_s"))
            for key in ("regions", "edges", "raw_path_length", "waypoints_count", "collision_checked", "collision_free", "unsafe_segments", "repair_time_s"):
                if key in result:
                    record[key] = result[key]
            trial["queries"].append(record)
            continue
        record = {
            "query": item["label"],
            "name": item["label"],
            "seed": seed_index,
            "success": True,
            "time_s": float(result["time_s"]),
            "path_length": float(result["path_length"]),
            "regions": int(result["regions"]),
            "edges": int(result["edges"]),
            "waypoints_count": int(result["waypoints_count"]),
            "collision_checked": bool(result.get("collision_checked")),
            "collision_free": result.get("collision_free"),
            "unsafe_segments": result.get("unsafe_segments"),
        }
        for key in ("raw_path_length", "raw_waypoints_count", "gcs_unsafe_segments", "repaired_segments", "repair_time_s", "note"):
            if key in result:
                record[key] = result[key]
        trial["queries"].append(record)
    return trial


def summarize(seed_trials: list[dict[str, Any]], labels: list[str]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    query_summaries: list[dict[str, Any]] = []
    all_rows = [row for trial in seed_trials for row in trial.get("queries", [])]
    for label in labels:
        rows = [row for row in all_rows if row.get("query") == label]
        successes = [row for row in rows if row.get("success")]
        query_summaries.append({
            "name": label,
            "sr": mean(1.0 if row.get("success") else 0.0 for row in rows),
            "t_med_s": median(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
            "t_mean_s": mean(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
            "len_med": median(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
            "len_mean": mean(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
            "trial_count": len(rows),
            "success_count": len(successes),
        })
    build_samples = [float(trial["build_s"]) for trial in seed_trials if trial.get("build_s") is not None]
    successes = [row for row in all_rows if row.get("success")]
    summary = {
        "build_s_mean": mean(build_samples),
        "build_s_median": median(build_samples),
        "query_time_s_mean": mean(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
        "query_time_s_median": median(float(row["time_s"]) for row in successes if row.get("time_s") is not None),
        "query_path_rad_mean": mean(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
        "query_path_rad_median": median(float(row["path_length"]) for row in successes if row.get("path_length") is not None),
        "sr": 100.0 * sum(1 for row in all_rows if row.get("success")) / max(1, len(all_rows)),
        "n_queries": len(all_rows),
        "n_success": sum(1 for row in all_rows if row.get("success")),
    }
    return query_summaries, summary


def main() -> int:
    args = parse_args()
    configure_threads(args.logical_threads)
    quick, seeds, timeout = mode_counts(args)
    items = workload()
    seed_trials = [run_seed_trial(args, seed_index, items) for seed_index in range(seeds)]
    query_summaries, summary = summarize(seed_trials, [item["label"] for item in items])
    payload = {
        "method": "iris_np_gcs",
        "scene": "iiwa14_marcucci_combined",
        "quick": quick,
        "seeds": seeds,
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_sbf_migrated_drake_iris_np_gcs_live_run",
        "note": "Current SBF migration of the original Drake IRIS-NP+GCS baseline. This artifact is generated live on the current machine and does not import historical timing/path values.",
        "params": {
            "timeout": timeout,
            "budget_s": float(args.budget_s),
            "query_time_limit_s": float(args.query_time_limit_s),
            "logical_threads": int(args.logical_threads),
            "edge_step_size": DEFAULT_IRIS_NP["edge_step_size"],
            "env_padding": DEFAULT_IRIS_NP["env_padding"],
            "self_padding": DEFAULT_IRIS_NP["self_padding"],
            "iteration_limit": int(args.iteration_limit),
            "termination_threshold": DEFAULT_IRIS_NP["termination_threshold"],
            "relative_termination_threshold": float(args.relative_termination_threshold),
            "num_collision_infeasible_samples": DEFAULT_IRIS_NP["num_collision_infeasible_samples"],
            "require_sample_point_is_contained": True,
            "allow_repair": bool(args.allow_repair),
            "rounding_max_paths": int(args.rounding_max_paths),
            "rounding_max_trials": int(args.rounding_max_trials),
            "gcs_preprocessing": bool(args.gcs_preprocessing),
            "use_rounding": bool(args.use_rounding),
            "gcs_repo": str(args.gcs_repo),
        },
        "queries": query_summaries,
        "seed_trials": seed_trials,
        "summary": summary,
    }
    write_json(args.out, payload)
    print(json.dumps({"out": str(args.out), "summary": summary, "queries": query_summaries}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
