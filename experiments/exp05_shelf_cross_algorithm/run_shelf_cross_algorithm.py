#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing as mp
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (
    DEFAULT_OUTPUT_ROOT,
    configure_thread_environment,
    environment_metadata,
    run_id,
    write_csv as write_csv_rows,
    write_json,
)
from experiments.common.checkpoints import (
    bitstar_checkpoint_grid_from_args,
    bitstar_trace_interval_for_grid,
    checkpoint_at_or_after,
)
from experiments.common.iris_gcs_dispatch import (
    default_gcs_repo,
    default_iris_python,
    run_shelf_iris_anytime,
    shelf_iris_json_to_run_rows,
    shelf_iris_summary_rows,
)
from experiments.common.metrics import mean, median, tex_num
from experiments.common.progress import progress
from experiments.common.rbf_defaults import (
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    DEFAULT_OMPL_SIMPLIFY_TIME_S,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_OFFLINE_ANCHOR_CANDIDATE_COUNT,
    DEFAULT_RBF_OFFLINE_ANCHOR_COUNT,
    DEFAULT_RBF_OFFLINE_ANCHOR_DISTANCE_MU,
    DEFAULT_RBF_OFFLINE_ANCHOR_LCA_LAMBDA,
    DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS,
    DEFAULT_RBF_SHELF_BOX_BUDGET,
    DEFAULT_RBF_THREADS,
    robot_joint_limit_tuples,
    robot_symmetry_aligned_root_tuples,
    shelf_d23_rbf_profile,
)
from experiments.common.rbf_leaf_rrt import RBFLeafRRTOptions, canonical_q, run_leaf_rrt
from experiments.common.sbf_import import import_sbf
from experiments.exp05_shelf_cross_algorithm.import_old_shelf_baselines import import_old_baselines
from experiments.exp05_shelf_cross_algorithm import run_bitstar_per_query as bitstar_per_query


sbf = import_sbf()
METHODS = ["sbf_leaf_rrt", "rrtconnect", "prm", "bitstar", "iris_np_gcs"]
IMPORTED_BASELINE_METHODS: set[str] = set()
METHOD_LABELS = {
    "sbf_leaf_rrt": "RBF",
    "iris_np_gcs": "IRIS-NP+GCS",
    "prm": "PRM",
    "rrtconnect": "RRTConnect",
    "bitstar": "BIT*",
}


def csv_items(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def csv_floats(raw: str) -> list[float]:
    return [float(item) for item in csv_items(raw)]


def csv_ints(raw: str) -> list[int]:
    return [int(float(item)) for item in csv_items(raw)]


def csv_strings(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def csv_bools(raw: str) -> list[bool]:
    values = []
    for item in csv_items(raw):
        lowered = item.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            values.append(True)
        elif lowered in {"0", "false", "no", "off"}:
            values.append(False)
        else:
            raise ValueError(f"invalid boolean grid value: {item!r}")
    return values


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def load_json_file(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def exp04_registered_dir(args: argparse.Namespace) -> Path:
    explicit = getattr(args, "exp04_registered_dir", None)
    if explicit is not None:
        return Path(explicit)
    return Path(args.out_dir).parent / "exp04"


def exp04_registered_rbf_rows(
    args: argparse.Namespace,
    requested_budgets: list[int],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    """Load Exp.5 RBF rows directly from the registered Exp.4 RBF profile.

    Exp.5 compares cross-algorithm online behavior and should not independently
    rerun RBF.  RBF is the reusable planner profile selected by Exp.4, so the
    Exp.5 RBF row is an import of Exp.4's registered baseline case.
    """
    source_dir = exp04_registered_dir(args)
    summary_path = source_dir / "shelf_leaf_rrt_summary.csv"
    manifest_path = source_dir / "shelf_leaf_rrt_manifest.json"
    if not summary_path.exists() or not manifest_path.exists():
        raise RuntimeError(
            "Exp.5 RBF requires the registered Exp.4 outputs. "
            f"Expected {summary_path} and {manifest_path}; run Exp.4 first "
            "or pass --exp04-registered-dir."
        )
    manifest = load_json_file(manifest_path)
    summary_rows = read_csv_rows(summary_path)
    budget_set = {int(budget) for budget in requested_budgets}
    imported_summary: list[dict[str, Any]] = []
    for row in summary_rows:
        if str(row.get("case")) != "baseline_d23_aafk_support_hull_8t":
            continue
        try:
            budget = int(float(row.get("deep_max_boxes", -1)))
        except (TypeError, ValueError):
            continue
        if budget not in budget_set:
            continue
        imported = dict(row)
        imported.update({
            "method": "sbf_leaf_rrt",
            "method_label": "RBF",
            "stage_id": f"b{budget}",
            "deep_max_boxes": str(budget),
            "source": "current_exp04_registered_profile",
            "status": "exp04_registered_profile",
            "build_s": row.get("offline_build_s_median", row.get("build_s_median", row.get("planning_s_median"))),
        })
        imported_summary.append(imported)
    if len(imported_summary) != len(budget_set):
        found = sorted(int(float(row.get("deep_max_boxes", -1))) for row in imported_summary)
        raise RuntimeError(
            "Exp.4 registered summary does not contain all requested RBF budgets: "
            f"requested={sorted(budget_set)} found={found}"
        )
    imported_rows: list[dict[str, Any]] = []
    for row in manifest.get("rows", []):
        if str(row.get("case")) != "baseline_d23_aafk_support_hull_8t":
            continue
        try:
            budget = int(float(row.get("deep_max_boxes", -1)))
        except (TypeError, ValueError):
            continue
        if budget not in budget_set:
            continue
        imported = dict(row)
        imported.update({
            "method": "sbf_leaf_rrt",
            "method_label": "RBF",
            "stage_id": f"b{budget}",
            "source": "current_exp04_registered_profile",
            "status": row.get("status", "ok"),
        })
        imported_rows.append(imported)
    return imported_rows, imported_summary, {
        "source": "current_exp04_registered_profile",
        "summary": str(summary_path),
        "manifest": str(manifest_path),
        "requested_budgets": sorted(budget_set),
        "imported_runs": len(imported_rows),
        "imported_summary_rows": len(imported_summary),
    }


def fmt_float(value: float) -> str:
    return f"{float(value):g}".replace("-", "m").replace(".", "p")


def path_length(path: Iterable[Iterable[float]]) -> float:
    pts = [[float(value) for value in point] for point in path]
    if len(pts) < 2:
        return math.nan
    total = 0.0
    for a, b in zip(pts, pts[1:]):
        total += math.sqrt(sum((x - y) * (x - y) for x, y in zip(a, b)))
    return total


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def simplify_path_if_requested(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    simplify_time_s: float,
) -> tuple[list[list[float]], float, str]:
    if len(path) < 2 or float(simplify_time_s) <= 0.0:
        return path, 0.0, "not_requested" if float(simplify_time_s) <= 0.0 else "path_too_short"
    result = sbf.ompl_simplify_path(
        robot,
        obstacles,
        path,
        float(segment_step),
        float(simplify_time_s),
    )
    simplified = [[float(value) for value in point] for point in result.get("path", [])]
    if bool(result.get("ok")) and len(simplified) >= 2:
        return simplified, float(result.get("t_s", 0.0)), str(result.get("reason", "simplified"))
    return path, float(result.get("t_s", 0.0)), str(result.get("reason", "simplify_failed"))


def audit_path(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    *,
    start: list[float] | None = None,
    goal: list[float] | None = None,
    endpoint_tol: float = 1e-6,
    collision_tolerance: float = DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
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
        distance = math.sqrt(sum((x - y) * (x - y) for x, y in zip(a, b)))
        steps = max(1, int(math.ceil(distance / step)))
        for index in range(steps + 1):
            if sbf.check_config_collision(
                robot,
                obstacles,
                interpolate(a, b, index / steps),
                float(collision_tolerance),
            ):
                return False, time.perf_counter() - t0, "collision"
    return True, time.perf_counter() - t0, "passed"


def shelf_queries(robot: Any) -> list[dict[str, Any]]:
    queries = []
    for query in list(sbf.make_combined_queries()):
        start = [float(value) for value in query.start]
        goal = [float(value) for value in query.goal]
        queries.append({
            "label": str(query.label),
            "start": start,
            "goal": goal,
            "actual_start": start,
            "actual_goal": goal,
            "canonical_start": canonical_q(robot, start),
            "canonical_goal": canonical_q(robot, goal),
        })
    return queries


def run_sbf(seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]], budget: int) -> dict[str, Any]:
    budget = int(budget)
    full_root = robot_joint_limit_tuples(robot)
    options = RBFLeafRRTOptions(
        seed=int(seed),
        deep_max_boxes=budget,
        use_external_evidence=True,
        external_evidence_path=Path(args.rbf_cache_root) / str(args.warm_cache_label),
        external_evidence_verify_identity=False,
        use_shelf_root_override=False,
        root_override_tuples=None,
        coverage_override_tuples=full_root,
        symmetry_aligned_native_root=False,
        symmetry_aligned_cache_schedule=True,
        case_label="sbf_leaf_rrt",
        threads=int(args.threads),
        parallel_virtual_validation=True,
        leaf_threads=int(args.threads),
        audit_collision_tolerance=float(args.audit_collision_tolerance),
        offline_query_agnostic_build=True,
        offline_random_anchors=bool(args.offline_random_anchors),
        offline_anchor_count=int(args.offline_anchor_count),
        offline_anchor_candidate_count=int(args.offline_anchor_candidate_count),
        offline_anchor_lca_lambda=float(args.offline_anchor_lca_lambda),
        offline_anchor_distance_mu=float(args.offline_anchor_distance_mu),
        query_bridge_to_main_island=bool(getattr(args, "query_bridge_to_main_island", False)),
        adaptive_max_free_boxes=budget,
        final_rrt_simplify_timeout_ms=1000.0 * float(args.ompl_simplify_time_s),
        final_rrt_simplify_attempts=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    )
    row = run_leaf_rrt(
        robot=robot,
        obstacles=obstacles,
        queries=queries,
        database_path=args.out_dir / "active_cache" / f"sbf_seed{seed}_box{budget}",
        options=options,
    )
    row["method"] = "sbf_leaf_rrt"
    row["planner"] = "RBF build_leaf_sweep_refined"
    row["deep_max_boxes"] = budget
    row["stage_id"] = f"b{budget}"
    row["budget_s"] = math.nan
    return row


def run_rrtconnect(seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]]) -> dict[str, Any]:
    qrows: list[dict[str, Any]] = []
    planning_s = 0.0
    audit_s = 0.0
    for index, query in enumerate(progress(queries, desc=f"exp05 rrt seed={seed}", total=len(queries))):
        result = sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            list(query["start"]),
            list(query["goal"]),
            float(args.rrt_timeout_s) * 1000.0,
            float(args.rrt_range),
            float(args.audit_segment_step),
            float(args.ompl_simplify_time_s),
            int(seed) * 1009 + index,
        )
        total_s = float(result.get("t_s", 0.0))
        solve_s = float(result.get("solve_s", max(0.0, total_s - float(result.get("simplify_s", 0.0)))))
        simplify_s = float(result.get("simplify_s", max(0.0, total_s - solve_s)))
        planning_s += total_s
        path = [[float(value) for value in point] for point in result.get("path", [])]
        audit_passed, audit_time_s, audit_status = audit_path(
            robot,
            obstacles,
            path,
            float(args.audit_segment_step),
            start=list(query["start"]),
            goal=list(query["goal"]),
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_s += audit_time_s
        ok = bool(result.get("ok")) and audit_passed
        length = path_length(path) if ok else math.nan
        qrows.append({
            "label": query["label"],
            "success": ok,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_time_s * 1000.0,
            "path_length": length,
            "segment_edge_length": 0.0,
            "segment_fraction": 0.0 if ok else math.nan,
            "waypoint_count": len(path),
            "planner_status": str(result.get("status", result.get("reason", ""))),
        })
    return summarize_method_run(
        "rrtconnect",
        seed,
        planning_s,
        audit_s,
        qrows,
        {
            "planner": "OMPL_RRTConnect",
            "timeout_s": float(args.rrt_timeout_s),
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
        stage_id=f"timeout{float(args.rrt_timeout_s):g}s",
        budget_s=float(args.rrt_timeout_s),
    )


def bitstar_worker(
    queue: Any,
    robot: Any,
    obstacles: list[Any],
    query: dict[str, Any],
    timeout_s: float,
    args: argparse.Namespace,
    rng_seed: int,
    samples_per_batch: int,
    rewire_factor: float,
) -> None:
    try:
        raw = sbf.ompl_bitstar_path(
            robot,
            obstacles,
            list(query["start"]),
            list(query["goal"]),
            float(timeout_s) * 1000.0,
            float(args.audit_segment_step),
            0.0,
            int(rng_seed),
            int(samples_per_batch),
            float(rewire_factor),
            bool(args.bitstar_stop_on_solution_improvement),
            *bitstar_extra_args(args),
        )
        queue.put({"ok": True, "result": dict(raw)})
    except BaseException:
        queue.put({"ok": False, "error": traceback.format_exc()})


def run_bitstar_path_watchdog(
    robot: Any,
    obstacles: list[Any],
    query: dict[str, Any],
    timeout_s: float,
    args: argparse.Namespace,
    rng_seed: int,
    samples_per_batch: int,
    rewire_factor: float,
) -> dict[str, Any]:
    wall_timeout = max(5.0, float(timeout_s) * float(args.bitstar_wall_timeout_factor) + 5.0)
    context = mp.get_context("fork")
    queue = context.Queue(maxsize=1)
    process = context.Process(
        target=bitstar_worker,
        args=(queue, robot, obstacles, query, timeout_s, args, rng_seed, samples_per_batch, rewire_factor),
    )
    process.start()
    process.join(wall_timeout)
    if process.is_alive():
        process.terminate()
        process.join(1.0)
        if process.is_alive():
            process.kill()
            process.join(1.0)
        return {
            "ok": False,
            "status": "wall_timeout",
            "reason": "wall_timeout",
            "t_s": wall_timeout,
            "path": [],
            "iterations": 0,
            "batches": 0,
        }
    payload = queue.get() if not queue.empty() else {"ok": False, "error": "worker exited without result"}
    if bool(payload.get("ok")):
        return dict(payload.get("result", {}))
    return {
        "ok": False,
        "status": "worker_failed",
        "reason": str(payload.get("error", ""))[-2000:],
        "t_s": math.nan,
        "path": [],
        "iterations": 0,
        "batches": 0,
    }


def run_bitstar_pre_audited_query(
    seed: int,
    query_index: int,
    args: argparse.Namespace,
    timeout_s: float,
    samples_per_batch: int,
    rewire_factor: float,
) -> dict[str, Any]:
    worker_args = argparse.Namespace(
        out_dir=args.out_dir,
        seeds=str(seed),
        query_indices=str(query_index),
        timeout_s=float(timeout_s),
        samples_per_batch=int(samples_per_batch),
        rewire_factor=float(rewire_factor),
        wall_timeout_factor=float(args.bitstar_wall_timeout_factor),
        audit_segment_step=float(args.audit_segment_step),
        audit_collision_tolerance=float(args.audit_collision_tolerance),
        simplify_time_s=float(args.ompl_simplify_time_s),
        stop_on_solution_improvement=bool(args.bitstar_stop_on_solution_improvement),
        use_k_nearest=int(args.bitstar_use_k_nearest),
        pruning=int(args.bitstar_pruning),
        prune_threshold_fraction=float(args.bitstar_prune_threshold_fraction),
        delay_rewiring_until_initial_solution=int(args.bitstar_delay_rewiring_until_initial_solution),
        just_in_time_sampling=int(args.bitstar_just_in_time_sampling),
        drop_samples_on_prune=int(args.bitstar_drop_samples_on_prune),
        approximate_solutions=int(args.bitstar_approximate_solutions),
        strict_queue_ordering=int(args.bitstar_strict_queue_ordering),
        cascading_rewirings=int(args.bitstar_cascading_rewirings),
        initial_inflation_factor=float(args.bitstar_initial_inflation_factor),
        inflation_scaling_parameter=float(args.bitstar_inflation_scaling_parameter),
        truncation_scaling_parameter=float(args.bitstar_truncation_scaling_parameter),
        allowed_failed_sampling_attempts=int(args.bitstar_allowed_failed_sampling_attempts),
        append=False,
        worker=True,
        seed=int(seed),
        query_index=int(query_index),
    )
    wall_timeout = max(5.0, float(timeout_s) * float(args.bitstar_wall_timeout_factor) + 5.0)
    context = mp.get_context("fork")
    queue = context.Queue(maxsize=1)
    process = context.Process(target=bitstar_per_query.multiprocessing_worker, args=(worker_args, queue))
    process.start()
    process.join(wall_timeout)
    query_label = str(bitstar_per_query.query_specs()[query_index]["label"])
    if process.is_alive():
        process.terminate()
        process.join(1.0)
        if process.is_alive():
            process.kill()
            process.join(1.0)
        return {
            "method": "bitstar",
            "seed": int(seed),
            "query_index": int(query_index),
            "label": query_label,
            "success": False,
            "audit_passed": False,
            "audit_status": "wall_timeout",
            "planner_status": "wall_timeout",
            "planning_s": wall_timeout,
            "audit_s": 0.0,
            "path_length": math.nan,
            "waypoint_count": 0,
        }
    payload = queue.get() if not queue.empty() else {"ok": False, "error": "worker exited without result"}
    if bool(payload.get("ok")):
        return dict(payload.get("row", {}))
    return {
        "method": "bitstar",
        "seed": int(seed),
        "query_index": int(query_index),
        "label": query_label,
        "success": False,
        "audit_passed": False,
        "audit_status": "worker_failed",
        "planner_status": str(payload.get("error", ""))[-2000:],
        "planning_s": math.nan,
        "audit_s": 0.0,
        "path_length": math.nan,
        "waypoint_count": 0,
    }


def run_bitstar_trace(
    seed: int,
    args: argparse.Namespace,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    *,
    timeout_s: float | None = None,
    samples_per_batch: int | None = None,
    rewire_factor: float | None = None,
    stage_prefix: str | None = None,
) -> list[dict[str, Any]]:
    timeout_s = float(args.bitstar_timeout_s if timeout_s is None else timeout_s)
    checkpoint_grid_s = bitstar_checkpoint_grid_from_args(args, timeout_s)
    interval_s = bitstar_trace_interval_for_grid(args, checkpoint_grid_s, timeout_s)
    samples_per_batch = int(args.bitstar_samples_per_batch if samples_per_batch is None else samples_per_batch)
    rewire_factor = float(args.bitstar_rewire_factor if rewire_factor is None else rewire_factor)
    stage_prefix = stage_prefix or f"batch{samples_per_batch}_rw{fmt_float(rewire_factor)}_trace{timeout_s:g}s"
    query_traces: list[tuple[dict[str, Any], list[dict[str, Any]]]] = []
    for index, query in enumerate(progress(queries, desc=f"exp05 bitstar seed={seed}", total=len(queries))):
        rng_seed = int(seed) * 2003 + index
        result = sbf.ompl_bitstar_trace(
            robot,
            obstacles,
            list(query["start"]),
            list(query["goal"]),
            timeout_s * 1000.0,
            interval_s * 1000.0,
            float(args.audit_segment_step),
            rng_seed,
            samples_per_batch,
            rewire_factor,
            bool(args.bitstar_stop_on_solution_improvement),
            *bitstar_extra_args(args),
        )
        query_traces.append((query, [dict(item) for item in result.get("checkpoints", [])]))
    rows: list[dict[str, Any]] = []
    best_by_query: dict[str, dict[str, Any]] = {}
    for target_checkpoint_s in checkpoint_grid_s:
        qrows: list[dict[str, Any]] = []
        audit_s = 0.0
        checkpoint_s = 0.0
        for query, checkpoints in query_traces:
            checkpoint = checkpoint_at_or_after(checkpoints, target_checkpoint_s)
            checkpoint_s = max(checkpoint_s, float(checkpoint.get("checkpoint_s", target_checkpoint_s) or 0.0))
            elapsed_s = float(checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0)) or 0.0)
            solve_s = float(checkpoint.get("solve_s", checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0))) or 0.0)
            path = [[float(value) for value in point] for point in checkpoint.get("path", [])]
            path, simplify_s, simplify_status = simplify_path_if_requested(
                robot,
                obstacles,
                path,
                float(args.audit_segment_step),
                float(args.ompl_simplify_time_s),
            )
            audit_passed, audit_time_s, audit_status = audit_path(
                robot,
                obstacles,
                path,
                float(args.audit_segment_step),
                start=list(query["start"]),
                goal=list(query["goal"]),
                collision_tolerance=float(args.audit_collision_tolerance),
            )
            audit_s += audit_time_s
            ok = bool(checkpoint.get("ok")) and audit_passed
            length = path_length(path) if ok else math.nan
            current_row = {
                "label": query["label"],
                "success": ok,
                "audit_passed": audit_passed,
                "audit_status": audit_status,
                "query_ms": (elapsed_s + simplify_s) * 1000.0,
                "solve_ms": solve_s * 1000.0,
                "simplify_ms": simplify_s * 1000.0,
                "audit_ms": audit_time_s * 1000.0,
                "path_length": length,
                "segment_edge_length": 0.0,
                "segment_fraction": 0.0 if ok else math.nan,
                "waypoint_count": len(path),
                "planner_status": str(checkpoint.get("status", checkpoint.get("reason", ""))),
                "iterations": int(checkpoint.get("iterations", 0) or 0),
                "batches": int(checkpoint.get("batches", 0) or 0),
                "checkpoint_s": checkpoint_s,
                "target_checkpoint_s": float(target_checkpoint_s),
                "simplify_status": simplify_status,
                "incumbent_checkpoint_s": float(target_checkpoint_s) if ok else math.nan,
            }
            label = str(query["label"])
            best = best_by_query.get(label)
            if ok and math.isfinite(length) and (best is None or length < float(best.get("path_length", math.inf))):
                best_by_query[label] = dict(current_row)
                best = best_by_query[label]
            if best is None:
                qrows.append(current_row)
            else:
                row = dict(best)
                row["query_ms"] = (elapsed_s + float(row.get("simplify_ms", 0.0)) / 1000.0) * 1000.0
                row["solve_ms"] = solve_s * 1000.0
                row["checkpoint_s"] = checkpoint_s
                row["target_checkpoint_s"] = float(target_checkpoint_s)
                row["planner_status"] = str(checkpoint.get("status", checkpoint.get("reason", "")))
                row["iterations"] = int(checkpoint.get("iterations", 0) or 0)
                row["batches"] = int(checkpoint.get("batches", 0) or 0)
                qrows.append(row)
        row = summarize_method_run(
            "bitstar",
            seed,
            sum(float(query.get("query_ms", 0.0)) for query in qrows) / 1000.0,
            audit_s,
            qrows,
            {
                "planner": "OMPL_BITstar_trace",
                "cumulative_bitstar": True,
                "timeout_s": timeout_s,
                "checkpoint_interval_s": interval_s,
                "checkpoint_grid_s": checkpoint_grid_s,
                "checkpoint_s": checkpoint_s,
                "target_checkpoint_s": float(target_checkpoint_s),
                "wall_timeout_factor": float(args.bitstar_wall_timeout_factor),
                "samples_per_batch": samples_per_batch,
                "rewire_factor": rewire_factor,
                "simplify_time_s": float(args.ompl_simplify_time_s),
                **bitstar_extra_metadata(args),
            },
            stage_id=f"{stage_prefix}_t{target_checkpoint_s:g}s",
            budget_s=float(target_checkpoint_s),
        )
        rows.append(row)
    return rows


def bitstar_extra_args(args: argparse.Namespace) -> list[Any]:
    return [
        int(args.bitstar_use_k_nearest),
        int(args.bitstar_pruning),
        float(args.bitstar_prune_threshold_fraction),
        int(args.bitstar_delay_rewiring_until_initial_solution),
        int(args.bitstar_just_in_time_sampling),
        int(args.bitstar_drop_samples_on_prune),
        int(args.bitstar_approximate_solutions),
        int(args.bitstar_strict_queue_ordering),
        int(args.bitstar_cascading_rewirings),
        float(args.bitstar_initial_inflation_factor),
        float(args.bitstar_inflation_scaling_parameter),
        float(args.bitstar_truncation_scaling_parameter),
        int(args.bitstar_allowed_failed_sampling_attempts),
    ]


def bitstar_extra_metadata(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
        "use_k_nearest": int(args.bitstar_use_k_nearest),
        "pruning": int(args.bitstar_pruning),
        "prune_threshold_fraction": float(args.bitstar_prune_threshold_fraction),
        "delay_rewiring_until_initial_solution": int(args.bitstar_delay_rewiring_until_initial_solution),
        "just_in_time_sampling": int(args.bitstar_just_in_time_sampling),
        "drop_samples_on_prune": int(args.bitstar_drop_samples_on_prune),
        "approximate_solutions": int(args.bitstar_approximate_solutions),
        "strict_queue_ordering": int(args.bitstar_strict_queue_ordering),
        "cascading_rewirings": int(args.bitstar_cascading_rewirings),
        "initial_inflation_factor": float(args.bitstar_initial_inflation_factor),
        "inflation_scaling_parameter": float(args.bitstar_inflation_scaling_parameter),
        "truncation_scaling_parameter": float(args.bitstar_truncation_scaling_parameter),
        "allowed_failed_sampling_attempts": int(args.bitstar_allowed_failed_sampling_attempts),
    }


def run_prm(
    seed: int,
    args: argparse.Namespace,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    build_s: float | None = None,
    *,
    query_budget_s: float | None = None,
    max_nearest_neighbors: int | None = None,
    planner_kind: str | None = None,
    prm_range: float | None = None,
    preload_query_endpoints: bool | None = None,
    stage_id: str | None = None,
) -> dict[str, Any]:
    build_s = float(args.prm_build_s if build_s is None else build_s)
    query_budget_s = float(args.prm_query_s if query_budget_s is None else query_budget_s)
    max_nearest_neighbors = int(args.prm_max_nearest_neighbors if max_nearest_neighbors is None else max_nearest_neighbors)
    planner_kind = str(args.prm_planner_kind if planner_kind is None else planner_kind)
    prm_range = float(args.prm_range if prm_range is None else prm_range)
    preload_query_endpoints = bool(args.prm_preload_query_endpoints if preload_query_endpoints is None else preload_query_endpoints)
    starts = [list(query["start"]) for query in queries]
    goals = [list(query["goal"]) for query in queries]
    t0 = time.perf_counter()
    if planner_kind.lower() in {"lazyprm", "lazy_prm"}:
        result = sbf.ompl_lazy_prm_multiquery(
            robot,
            obstacles,
            starts,
            goals,
            query_budget_s,
            float(args.audit_segment_step),
            float(args.ompl_simplify_time_s),
            int(seed),
            max_nearest_neighbors,
            prm_range,
        )
    else:
        result = sbf.ompl_prm_multiquery(
            robot,
            obstacles,
            starts,
            goals,
            build_s,
            query_budget_s,
            float(args.audit_segment_step),
            float(args.ompl_simplify_time_s),
            int(seed),
            max_nearest_neighbors,
            planner_kind,
            preload_query_endpoints,
        )
    planning_s = time.perf_counter() - t0
    qrows: list[dict[str, Any]] = []
    audit_s = 0.0
    qresult_items = list(result.get("queries", []))
    for query, qresult in zip(progress(queries, desc=f"exp05 prm audit seed={seed}", total=len(queries)), qresult_items):
        path = [[float(value) for value in point] for point in qresult.get("path", [])]
        audit_passed, audit_time_s, audit_status = audit_path(
            robot,
            obstacles,
            path,
            float(args.audit_segment_step),
            start=list(query["start"]),
            goal=list(query["goal"]),
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_s += audit_time_s
        ok = bool(qresult.get("ok")) and audit_passed
        total_s = float(qresult.get("t_s", 0.0))
        solve_s = float(qresult.get("solve_s", max(0.0, total_s - float(qresult.get("simplify_s", 0.0)))))
        simplify_s = float(qresult.get("simplify_s", max(0.0, total_s - solve_s)))
        qrows.append({
            "label": query["label"],
            "success": ok,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_time_s * 1000.0,
            "path_length": path_length(path) if ok else math.nan,
            "segment_edge_length": 0.0,
            "segment_fraction": 0.0 if ok else math.nan,
            "waypoint_count": len(path),
            "planner_status": str(qresult.get("status", qresult.get("reason", ""))),
        })
    row = summarize_method_run(
        "prm",
        seed,
        planning_s,
        audit_s,
        qrows,
        {
            "planner": "OMPL_PRM",
            "build_s": float(result.get("build_s", 0.0)),
            "nodes": int(result.get("nodes", 0)),
            "query_budget_s": query_budget_s,
            "max_nearest_neighbors": max_nearest_neighbors,
            "planner_kind": planner_kind,
            "range": prm_range,
            "preload_query_endpoints": preload_query_endpoints,
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
        stage_id=stage_id or f"{planner_kind}_build{build_s:g}s_k{max_nearest_neighbors}_q{fmt_float(query_budget_s)}s_preload{int(preload_query_endpoints)}",
        budget_s=build_s,
    )
    row["build_s"] = float(result.get("build_s", build_s))
    return row


def run_prm_cumulative(
    seed: int,
    args: argparse.Namespace,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    build_checkpoints_s: list[float],
    *,
    query_budget_s: float | None = None,
    max_nearest_neighbors: int | None = None,
    planner_kind: str | None = None,
    preload_query_endpoints: bool | None = None,
) -> list[dict[str, Any]]:
    checkpoints = sorted({float(value) for value in build_checkpoints_s if float(value) > 0.0})
    if not checkpoints:
        raise ValueError("PRM cumulative mode requires at least one positive build checkpoint")
    query_budget_s = float(args.prm_query_s if query_budget_s is None else query_budget_s)
    max_nearest_neighbors = int(args.prm_max_nearest_neighbors if max_nearest_neighbors is None else max_nearest_neighbors)
    planner_kind = str(args.prm_planner_kind if planner_kind is None else planner_kind)
    preload_query_endpoints = bool(args.prm_preload_query_endpoints if preload_query_endpoints is None else preload_query_endpoints)
    starts = [list(query["start"]) for query in queries]
    goals = [list(query["goal"]) for query in queries]
    t0 = time.perf_counter()
    result = sbf.ompl_prm_multiquery_cumulative(
        robot,
        obstacles,
        starts,
        goals,
        checkpoints,
        query_budget_s,
        float(args.audit_segment_step),
        float(args.ompl_simplify_time_s),
        int(seed),
        max_nearest_neighbors,
        planner_kind,
        preload_query_endpoints,
    )
    _wall_s = time.perf_counter() - t0
    incumbents: dict[str, dict[str, Any]] = {}
    rows: list[dict[str, Any]] = []
    for stage in result.get("stages", []):
        checkpoint_s = float(stage.get("checkpoint_s", 0.0))
        build_s = float(stage.get("build_s", checkpoint_s))
        qrows: list[dict[str, Any]] = []
        audit_s = 0.0
        qresult_items = list(stage.get("queries", []))
        for query, qresult in zip(
            progress(queries, desc=f"exp05 prm cumulative audit seed={seed} build={checkpoint_s:g}s", total=len(queries)),
            qresult_items,
        ):
            label = str(query["label"])
            path = [[float(value) for value in point] for point in qresult.get("path", [])]
            raw_audit_passed = False
            audit_time_s = 0.0
            audit_status = "not_attempted"
            raw_length = math.nan
            if bool(qresult.get("ok")) and len(path) >= 2:
                raw_audit_passed, audit_time_s, audit_status = audit_path(
                    robot,
                    obstacles,
                    path,
                    float(args.audit_segment_step),
                    start=list(query["start"]),
                    goal=list(query["goal"]),
                    collision_tolerance=float(args.audit_collision_tolerance),
                )
                audit_s += audit_time_s
                if raw_audit_passed:
                    raw_length = path_length(path)
                    current = incumbents.get(label)
                    if current is None or raw_length <= float(current["path_length"]) + 1e-12:
                        incumbents[label] = {
                            "path": path,
                            "path_length": raw_length,
                            "waypoint_count": len(path),
                            "checkpoint_s": checkpoint_s,
                            "planner_status": str(qresult.get("status", qresult.get("reason", ""))),
                        }
            incumbent = incumbents.get(label)
            total_s = float(qresult.get("t_s", 0.0))
            solve_s = float(qresult.get("solve_s", max(0.0, total_s - float(qresult.get("simplify_s", 0.0)))))
            simplify_s = float(qresult.get("simplify_s", max(0.0, total_s - solve_s)))
            if incumbent is not None:
                qrows.append({
                    "label": label,
                    "success": True,
                    "audit_passed": True,
                    "audit_status": (
                        "current_audit_passed"
                        if math.isfinite(raw_length) and abs(raw_length - float(incumbent["path_length"])) <= 1e-12
                        else f"incumbent_from_{float(incumbent['checkpoint_s']):g}s"
                    ),
                    "query_ms": total_s * 1000.0,
                    "solve_ms": solve_s * 1000.0,
                    "simplify_ms": simplify_s * 1000.0,
                    "audit_ms": audit_time_s * 1000.0,
                    "path_length": float(incumbent["path_length"]),
                    "segment_edge_length": 0.0,
                    "segment_fraction": 0.0,
                    "waypoint_count": int(incumbent["waypoint_count"]),
                    "planner_status": (
                        f"{str(qresult.get('status', qresult.get('reason', '')))}; "
                        f"incumbent_checkpoint={float(incumbent['checkpoint_s']):g}s"
                    ),
                })
            else:
                qrows.append({
                    "label": label,
                    "success": False,
                    "audit_passed": False,
                    "audit_status": audit_status,
                    "query_ms": total_s * 1000.0,
                    "solve_ms": solve_s * 1000.0,
                    "simplify_ms": simplify_s * 1000.0,
                    "audit_ms": audit_time_s * 1000.0,
                    "path_length": math.nan,
                    "segment_edge_length": 0.0,
                    "segment_fraction": math.nan,
                    "waypoint_count": len(path),
                    "planner_status": str(qresult.get("status", qresult.get("reason", ""))),
                })
        row = summarize_method_run(
            "prm",
            seed,
            build_s + sum(float(query.get("query_ms", 0.0)) for query in qrows) / 1000.0,
            audit_s,
            qrows,
            {
                "planner": "OMPL_PRM_cumulative",
                "cumulative_prm": True,
                "checkpoint_s": checkpoint_s,
                "build_checkpoints_s": checkpoints,
                "build_s": build_s,
                "nodes": int(stage.get("nodes", result.get("nodes", 0))),
                "query_budget_s": query_budget_s,
                "max_nearest_neighbors": max_nearest_neighbors,
                "planner_kind": planner_kind,
                "preload_query_endpoints": preload_query_endpoints,
                "simplify_time_s": float(args.ompl_simplify_time_s),
            },
            stage_id=(
                f"{planner_kind}_cumulative_build{checkpoint_s:g}s"
                f"_k{max_nearest_neighbors}_q{fmt_float(query_budget_s)}s"
                f"_preload{int(preload_query_endpoints)}"
            ),
            budget_s=checkpoint_s,
        )
        row["build_s"] = build_s
        rows.append(row)
    return rows


def summarize_method_run(
    method: str,
    seed: int,
    planning_s: float,
    audit_s: float,
    qrows: list[dict[str, Any]],
    extra: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
) -> dict[str, Any]:
    successes = [row for row in qrows if bool(row["audit_passed"])]
    query_count = max(1, len(qrows))
    extra_dict = dict(extra or {})
    build_s = float(extra_dict.get("build_s", 0.0) or 0.0)
    online_batch_s = max(0.0, float(planning_s) - build_s)
    if method in {"rrtconnect", "bitstar"}:
        build_s = 0.0
        online_batch_s = float(planning_s)
    online_simplify_s = 0.0
    online_solve_s = 0.0
    split_available = False
    for query in qrows:
        try:
            total_ms = float(query.get("query_ms", math.nan))
            solve_ms = float(query.get("solve_ms", math.nan))
            simplify_ms = float(query.get("simplify_ms", math.nan))
        except (TypeError, ValueError):
            continue
        if math.isfinite(solve_ms) or math.isfinite(simplify_ms):
            split_available = True
            if math.isfinite(solve_ms):
                online_solve_s += solve_ms / 1000.0
            elif math.isfinite(total_ms) and math.isfinite(simplify_ms):
                online_solve_s += max(0.0, total_ms - simplify_ms) / 1000.0
            if math.isfinite(simplify_ms):
                online_simplify_s += simplify_ms / 1000.0
    if not split_available:
        online_solve_s = online_batch_s
        online_simplify_s = 0.0
    else:
        residual_s = online_batch_s - online_solve_s - online_simplify_s
        if residual_s > 1e-9:
            online_solve_s += residual_s
    online_total_s = online_batch_s
    online_batch_s = online_solve_s
    online_per_query_s = online_solve_s / query_count
    online_total_per_query_s = online_total_s / query_count
    online_solve_per_query_s = online_solve_s / query_count
    online_simplify_per_query_s = online_simplify_s / query_count
    amortized = {
        f"amortized_s_k{k}": build_s / float(k) + online_per_query_s
        for k in (1, 5, 10, 20, 50)
    }
    return {
        "method": method,
        "seed": int(seed),
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        "status": "ok" if len(successes) == len(qrows) else "partial",
        "success_count": len(successes),
        "query_count": len(qrows),
        "planning_s": build_s + online_batch_s,
        "planning_total_s": float(planning_s),
        "build_s": build_s,
        "offline_build_s": build_s,
        "online_batch_s": online_batch_s,
        "online_total_s": online_total_s,
        "online_total_batch_s": online_total_s,
        "online_solve_s": online_solve_s,
        "online_simplify_s": online_simplify_s,
        "online_per_query_s": online_per_query_s,
        "online_total_per_query_s": online_total_per_query_s,
        "online_solve_per_query_s": online_solve_per_query_s,
        "online_simplify_per_query_s": online_simplify_per_query_s,
        **amortized,
        "audit_s": float(audit_s),
        "path_length_mean": mean(row["path_length"] for row in successes),
        "raw_segment_fraction": 0.0,
        "queries": qrows,
        "diagnostics": extra_dict,
    }


def external_pending_row(method: str, seed: int, reason: str) -> dict[str, Any]:
    return {
        "method": method,
        "seed": int(seed),
        "stage_id": method,
        "budget_s": math.nan,
        "status": "external_pending",
        "success_count": 0,
        "query_count": 0,
        "planning_s": math.nan,
        "audit_s": math.nan,
        "path_length_mean": math.nan,
        "raw_segment_fraction": math.nan,
        "queries": [],
        "diagnostics": {"reason": reason},
    }


def run_method(method: str, seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]], budget_s: float | None = None) -> dict[str, Any]:
    if method == "sbf_leaf_rrt":
        return run_sbf(seed, args, robot, obstacles, queries, int(args.sbf_box_budget))
    if method == "rrtconnect":
        return run_rrtconnect(seed, args, robot, obstacles, queries)
    if method == "prm":
        return run_prm(seed, args, robot, obstacles, queries, budget_s)
    if method == "bitstar":
        raise ValueError("BIT* uses run_bitstar_trace() so one fixed-timeout run can emit all checkpoints")
    if method == "iris_np_gcs":
        return external_pending_row(method, seed, "IRIS/GCS is executed once through the prefix-anytime dispatcher, not per seed in this loop.")
    raise ValueError(f"unknown method {method!r}")


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    def timeout_cap(row: dict[str, Any]) -> float:
        diagnostics = row.get("diagnostics", {})
        if isinstance(diagnostics, dict):
            try:
                value = float(diagnostics.get("timeout_s", math.nan))
            except (TypeError, ValueError):
                value = math.nan
            if math.isfinite(value):
                return value
        return math.nan

    out = []
    keys = sorted({
        (str(row["method"]), str(row.get("stage_id", row["method"])), str(row.get("deep_max_boxes", "")))
        for row in rows
    })
    for method, stage_id, budget in keys:
        items = [
            row for row in rows
            if str(row["method"]) == method
            and str(row.get("stage_id", row["method"])) == stage_id
            and str(row.get("deep_max_boxes", "")) == budget
            and row.get("status") != "external_pending"
        ]
        pending = [
            row for row in rows
            if str(row["method"]) == method
            and str(row.get("stage_id", row["method"])) == stage_id
            and str(row.get("deep_max_boxes", "")) == budget
            and row.get("status") == "external_pending"
        ]
        if not items:
            out.append({
                "method": method,
                "stage_id": stage_id,
                "budget_s": None,
                "timeout_cap_s": None,
                "deep_max_boxes": budget,
                "runs": len(pending),
                "success_runs": 0,
                "success_queries": 0,
                "total_queries": 0,
                "offline_build_s_median": None,
                "online_batch_s_median": None,
                "online_solve_s_median": None,
                "online_simplify_s_median": None,
                "online_solve_per_query_s_median": None,
                "online_simplify_per_query_s_median": None,
                "online_per_query_s_median": None,
                "amortized_s_k1": None,
                "amortized_s_k5": None,
                "amortized_s_k10": None,
                "amortized_s_k20": None,
                "amortized_s_k50": None,
                "measured_time_s_median": None,
                "planning_s_median": None,
                "audit_s_median": None,
                "path_length_mean": None,
                "raw_segment_fraction_median": None,
                "status": "external_pending" if pending else "missing",
            })
            continue
        planning_s_median = median(row["planning_s"] for row in items)
        success_queries = sum(int(row.get("success_count", 0)) for row in items)
        total_queries = sum(int(row.get("query_count", 0)) for row in items)
        out.append({
            "method": method,
            "method_label": METHOD_LABELS.get(method, method),
            "stage_id": stage_id,
            "budget_s": median(row.get("budget_s", math.nan) for row in items),
            "timeout_cap_s": median(timeout_cap(row) for row in items) if method == "bitstar" else math.nan,
            "deep_max_boxes": budget if method == "sbf_leaf_rrt" else 0,
            "runs": len(items),
            "success_runs": sum(1 for row in items if int(row["success_count"]) == int(row["query_count"])),
            "success_queries": success_queries,
            "total_queries": total_queries,
            "source": "current_execution",
            "build_s": median(row.get("offline_build_s", row.get("build_s", 0.0)) for row in items),
            "offline_build_s_median": median(row.get("offline_build_s", row.get("build_s", 0.0)) for row in items),
            "online_batch_s_median": median(row.get("online_batch_s", max(0.0, row["planning_s"] - row.get("build_s", 0.0))) for row in items),
            "online_total_s_median": median(row.get("online_total_s", row.get("online_batch_s", max(0.0, row["planning_s"] - row.get("build_s", 0.0)))) for row in items),
            "online_solve_s_median": median(row.get("online_solve_s", row.get("online_batch_s", max(0.0, row["planning_s"] - row.get("build_s", 0.0)))) for row in items),
            "online_simplify_s_median": median(row.get("online_simplify_s", 0.0) for row in items),
            "online_solve_per_query_s_median": median(
                row.get(
                    "online_solve_per_query_s",
                    row.get("online_solve_s", row.get("online_batch_s", max(0.0, row["planning_s"] - row.get("build_s", 0.0)))) / max(1, int(row.get("query_count", 1))),
                )
                for row in items
            ),
            "online_simplify_per_query_s_median": median(
                row.get(
                    "online_simplify_per_query_s",
                    row.get("online_simplify_s", 0.0) / max(1, int(row.get("query_count", 1))),
                )
                for row in items
            ),
            "online_per_query_s_median": median(
                row.get(
                    "online_per_query_s",
                    row.get("online_solve_s", max(0.0, row["planning_s"] - row.get("build_s", 0.0))) / max(1, int(row.get("query_count", 1))),
                )
                for row in items
            ),
            "online_total_per_query_s_median": median(
                row.get(
                    "online_total_per_query_s",
                    row.get("online_total_s", row.get("online_batch_s", max(0.0, row["planning_s"] - row.get("build_s", 0.0)))) / max(1, int(row.get("query_count", 1))),
                )
                for row in items
            ),
            "amortized_s_k1": median(row.get("amortized_s_k1", row["planning_s"]) for row in items),
            "amortized_s_k5": median(row.get("amortized_s_k5", row["planning_s"] / 5.0) for row in items),
            "amortized_s_k10": median(row.get("amortized_s_k10", row["planning_s"] / 10.0) for row in items),
            "amortized_s_k20": median(row.get("amortized_s_k20", row["planning_s"] / 20.0) for row in items),
            "amortized_s_k50": median(row.get("amortized_s_k50", row["planning_s"] / 50.0) for row in items),
            "query_s_median": median(median(q.get("query_ms", math.nan) / 1000.0 for q in row.get("queries", [])) for row in items),
            "measured_time_s_median": planning_s_median,
            "planning_s_median": planning_s_median,
            "audit_s_median": median(row["audit_s"] for row in items),
            "path_length_mean": mean(row["path_length_mean"] for row in items if int(row["success_count"]) == int(row["query_count"])),
            "raw_segment_fraction_median": median(row["raw_segment_fraction"] for row in items if int(row["success_count"]) == int(row["query_count"])),
            "status": "executed",
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "method", "method_label", "stage_id", "budget_s", "timeout_cap_s", "deep_max_boxes",
        "runs", "success_runs", "success_queries", "total_queries", "source",
        "build_s", "offline_build_s_median", "online_batch_s_median", "online_total_s_median",
        "online_per_query_s_median", "online_total_per_query_s_median",
        "online_solve_s_median", "online_simplify_s_median",
        "online_solve_per_query_s_median", "online_simplify_per_query_s_median",
        "amortized_s_k1", "amortized_s_k5", "amortized_s_k10", "amortized_s_k20", "amortized_s_k50",
        "query_s_median", "measured_time_s_median", "planning_s_median", "audit_s_median",
        "path_length_mean", "raw_segment_fraction_median", "status",
    ]
    write_csv_rows(path, rows, fields)


def write_per_query_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "method",
        "stage_id",
        "seed",
        "label",
        "success",
        "audit_passed",
        "audit_status",
        "path_length",
        "query_ms",
        "solve_ms",
        "simplify_ms",
        "audit_ms",
        "waypoint_count",
        "planner_status",
    ]
    flat_rows = []
    for row in rows:
        for query in row.get("queries", []):
            flat_rows.append({
                "method": row.get("method"),
                "stage_id": row.get("stage_id"),
                "seed": row.get("seed"),
                "label": query.get("label"),
                "success": query.get("success"),
                "audit_passed": query.get("audit_passed"),
                "audit_status": query.get("audit_status"),
                "path_length": query.get("path_length"),
                "query_ms": query.get("query_ms"),
                "solve_ms": query.get("solve_ms"),
                "simplify_ms": query.get("simplify_ms"),
                "audit_ms": query.get("audit_ms"),
                "waypoint_count": query.get("waypoint_count"),
                "planner_status": query.get("planner_status"),
            })
    write_csv_rows(path, flat_rows, fields)


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    def path_stat(row: dict[str, Any]) -> float:
        for key in ("path_length_mean", "path_length_median"):
            try:
                value = float(row.get(key, math.nan))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                return value
        return math.nan

    def finite_value(value: Any, fallback: float = 1e9) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return fallback
        return out if math.isfinite(out) else fallback

    def selected_rows() -> list[dict[str, Any]]:
        rbf_rows = [
            row for row in rows
            if str(row.get("method")) == "sbf_leaf_rrt"
            and int(row.get("success_runs", 0) or 0) == int(row.get("runs", 0) or 0)
        ]
        selected_rbf = None
        if rbf_rows:
            selected_rbf = sorted(
                rbf_rows,
                key=lambda row: (
                    finite_value(row.get("online_per_query_s_median")),
                    finite_value(row.get("offline_build_s_median", row.get("build_s"))) / 5.0
                    + finite_value(row.get("online_per_query_s_median")),
                    int(float(row.get("deep_max_boxes", 0) or 0)),
                ),
            )[0]
        row_by_method: dict[str, dict[str, Any]] = {}
        for method in ["iris_np_gcs", "prm", "rrtconnect", "bitstar"]:
            items = [row for row in rows if str(row.get("method")) == method]
            full = [
                row for row in items
                if int(row.get("success_runs", 0) or 0) == int(row.get("runs", 0) or 0)
            ]
            candidates = full or items
            finite_path = [
                path_stat(row)
                for row in candidates
                if math.isfinite(path_stat(row))
            ]
            if finite_path:
                best_path = min(finite_path)
                candidates = [
                    row for row in candidates
                    if math.isfinite(path_stat(row))
                    and path_stat(row) <= 1.08 * best_path
                ] or candidates
            if candidates:
                row_by_method[method] = sorted(
                    candidates,
                    key=lambda row: (
                        finite_value(row.get("online_per_query_s_median")),
                        finite_value(row.get("offline_build_s_median", row.get("build_s"))) / 5.0
                        + finite_value(row.get("online_per_query_s_median")),
                        finite_value(row.get("budget_s")),
                    ),
                )[0]
        if selected_rbf is not None:
            row_by_method["sbf_leaf_rrt"] = selected_rbf
        return [
            row_by_method[method]
            for method in ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
            if method in row_by_method
        ]

    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Shelf+IIWA cross-algorithm reusable-planner comparison under a common fixed-step final audit. RBF uses the Exp.4 two-stage profile. PRM and IRIS/GCS report reusable build/query timing; RRTConnect and BIT* are one-shot online baselines with zero build time. Online/q excludes final simplification; Simplify/q reports the measured cost under the globally fixed 0.01~s OMPL post-processing budget. All timing excludes final audit. Amort@5 amortizes reusable build over the five shelf queries plus Online/q.}",
        r"\label{tab:tro-shelf-cross-algorithm}",
        r"\footnotesize",
        r"\setlength{\tabcolsep}{3.5pt}",
        r"\begin{tabular}{lrrrrrr}",
        r"\toprule",
        r"Method & Build & Online/q & Simplify/q & Amort@5 & Path & SR \\",
        r"\midrule",
    ]
    for row in selected_rows():
        sr = f"{int(row.get('success_queries', row.get('success_runs', 0)) or 0)}/{int(row.get('total_queries', row.get('runs', 0)) or 0)}"
        method = str(row.get("method_label", row.get("method", ""))).replace("_", r"\_")
        if str(row.get("method")) == "sbf_leaf_rrt":
            method = rf"{method} (b{int(float(row.get('deep_max_boxes', 0) or 0))})"
        lines.append(
            f"{method} & {tex_num(row.get('offline_build_s_median', row.get('build_s')))} & "
            f"{tex_num(row.get('online_per_query_s_median', row.get('online_solve_per_query_s_median')))} & "
            f"{tex_num(row.get('online_simplify_per_query_s_median'))} & "
            f"{tex_num(finite_value(row.get('offline_build_s_median', row.get('build_s'))) / 5.0 + finite_value(row.get('online_per_query_s_median', row.get('online_solve_per_query_s_median'))))} & "
            f"{tex_num(path_stat(row))} & {sr} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table*}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.5 Shelf+IIWA cross-algorithm study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp05")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--methods", default=",".join(METHODS))
    parser.add_argument("--rerun-baselines", action=argparse.BooleanOptionalAction, default=True, help="Rerun OMPL baselines instead of importing old audited artifacts. IRIS-NP+GCS may still be imported unless explicitly supported.")
    parser.add_argument(
        "--old-paper-root",
        type=Path,
        default=Path(os.environ.get("RBF_OLD_TRO_PAPER_ROOT", str(REPO_ROOT / "external" / "old_tro2026" / "paper"))),
    )
    parser.add_argument(
        "--old-output-root",
        type=Path,
        default=Path(os.environ.get("RBF_OLD_TRO_OUTPUT_ROOT", str(REPO_ROOT / "external" / "old_tro2026" / "outputs"))),
    )
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--ompl-simplify-time-s", type=float, default=DEFAULT_OMPL_SIMPLIFY_TIME_S)
    parser.add_argument("--sbf-box-budget", type=int, default=DEFAULT_RBF_SHELF_BOX_BUDGET)
    parser.add_argument("--sbf-box-budgets", default=str(DEFAULT_RBF_SHELF_BOX_BUDGET))
    parser.add_argument(
        "--exp04-registered-dir",
        type=Path,
        default=None,
        help=(
            "Directory containing Exp.4 registered RBF outputs. Defaults to "
            "the sibling 'exp04' directory next to --out-dir. Exp.5 imports "
            "RBF from this source instead of rerunning it."
        ),
    )
    parser.add_argument("--offline-random-anchors", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS)
    parser.add_argument("--offline-anchor-count", type=int, default=DEFAULT_RBF_OFFLINE_ANCHOR_COUNT)
    parser.add_argument("--offline-anchor-candidate-count", type=int, default=DEFAULT_RBF_OFFLINE_ANCHOR_CANDIDATE_COUNT)
    parser.add_argument("--offline-anchor-lca-lambda", type=float, default=DEFAULT_RBF_OFFLINE_ANCHOR_LCA_LAMBDA)
    parser.add_argument("--offline-anchor-distance-mu", type=float, default=DEFAULT_RBF_OFFLINE_ANCHOR_DISTANCE_MU)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    parser.add_argument("--rrt-timeout-s", type=float, default=10.0)
    parser.add_argument("--rrt-range", type=float, default=0.35)
    parser.add_argument("--bitstar-timeout-s", type=float, default=10.0)
    parser.add_argument("--bitstar-timeout-grid-s", default="")
    parser.add_argument("--bitstar-checkpoint-grid-s", default="0.05,0.1,0.2,0.3,0.5,0.75,1,1.25,1.5,1.55,1.6,1.65,1.7,1.75,1.8,1.85,1.9,1.95,2,2.25,2.5,3,4,5,7.5,10")
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=0.05)
    parser.add_argument("--bitstar-wall-timeout-factor", type=float, default=1.5)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-samples-per-batch-grid", default="")
    parser.add_argument("--bitstar-rewire-factor", type=float, default=5.0)
    parser.add_argument("--bitstar-rewire-factor-grid", default="")
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--bitstar-use-k-nearest", type=int, default=-1)
    parser.add_argument("--bitstar-pruning", type=int, default=-1)
    parser.add_argument("--bitstar-prune-threshold-fraction", type=float, default=-1.0)
    parser.add_argument("--bitstar-delay-rewiring-until-initial-solution", type=int, default=-1)
    parser.add_argument("--bitstar-just-in-time-sampling", type=int, default=-1)
    parser.add_argument("--bitstar-drop-samples-on-prune", type=int, default=-1)
    parser.add_argument("--bitstar-approximate-solutions", type=int, default=-1)
    parser.add_argument("--bitstar-strict-queue-ordering", type=int, default=-1)
    parser.add_argument("--bitstar-cascading-rewirings", type=int, default=-1)
    parser.add_argument("--bitstar-initial-inflation-factor", type=float, default=-1.0)
    parser.add_argument("--bitstar-inflation-scaling-parameter", type=float, default=-1.0)
    parser.add_argument("--bitstar-truncation-scaling-parameter", type=float, default=-1.0)
    parser.add_argument("--bitstar-allowed-failed-sampling-attempts", type=int, default=-1)
    parser.add_argument("--prm-build-s", type=float, default=20.0)
    parser.add_argument(
        "--prm-build-grid-s",
        default="5,10,15,20",
        help=(
            "Reusable shared-roadmap PRM build budgets. The paper profile keeps "
            "intermediate cumulative checkpoints and the 20 s registered point; "
            "each seed still builds one roadmap and answers all five shelf queries."
        ),
    )
    parser.add_argument("--prm-query-s", type=float, default=1.0)
    parser.add_argument("--prm-query-grid-s", default="")
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-k-grid", default="")
    parser.add_argument("--prm-planner-kind", default="prm")
    parser.add_argument("--prm-planner-kind-grid", default="")
    parser.add_argument("--prm-range", type=float, default=0.0)
    parser.add_argument("--prm-range-grid", default="")
    parser.add_argument(
        "--prm-cumulative",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Grow one PRM roadmap cumulatively across build checkpoints and report audited best-so-far incumbents.",
    )
    parser.add_argument(
        "--prm-preload-query-endpoints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Insert the registered shelf query endpoints as roadmap milestones "
            "before construction; the cost is charged to PRM build time."
        ),
    )
    parser.add_argument("--prm-preload-query-endpoints-grid", default="")
    parser.add_argument("--iris-python", type=Path, default=default_iris_python())
    parser.add_argument("--iris-gcs-repo", type=Path, default=default_gcs_repo())
    parser.add_argument("--iris-budget-s", type=float, default=1200.0)
    parser.add_argument("--iris-stage-region-counts", default="8,12,16,20,24")
    parser.add_argument("--iris-iteration-limit", type=int, default=8)
    parser.add_argument("--iris-query-time-limit-s", type=float, default=120.0)
    parser.add_argument("--iris-rounding-max-paths", type=int, default=24)
    parser.add_argument("--iris-rounding-max-trials", type=int, default=240)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    configure_thread_environment(int(args.threads))
    rbf_profile = shelf_d23_rbf_profile()
    rbf_profile["query"]["final_rrt_simplify_timeout_ms"] = 1000.0 * float(args.ompl_simplify_time_s)
    rbf_profile["query"]["final_rrt_simplify_time_s"] = float(args.ompl_simplify_time_s)
    seeds = [int(item) for item in csv_items(args.seeds)]
    methods = [item for item in csv_items(args.methods) if item in METHODS]
    sbf_budgets = [int(item) for item in csv_items(args.sbf_box_budgets)]
    prm_build_grid_s = [float(item) for item in csv_items(args.prm_build_grid_s)] if str(args.prm_build_grid_s).strip() else [float(args.prm_build_s)]
    prm_query_grid_s = csv_floats(args.prm_query_grid_s) if str(args.prm_query_grid_s).strip() else [float(args.prm_query_s)]
    prm_k_grid = csv_ints(args.prm_k_grid) if str(args.prm_k_grid).strip() else [int(args.prm_max_nearest_neighbors)]
    prm_kind_grid = csv_strings(args.prm_planner_kind_grid) if str(args.prm_planner_kind_grid).strip() else [str(args.prm_planner_kind)]
    prm_range_grid = csv_floats(args.prm_range_grid) if str(args.prm_range_grid).strip() else [float(args.prm_range)]
    prm_preload_grid = csv_bools(args.prm_preload_query_endpoints_grid) if str(args.prm_preload_query_endpoints_grid).strip() else [bool(args.prm_preload_query_endpoints)]
    bitstar_explicit_grid_s = csv_floats(args.bitstar_timeout_grid_s) if str(args.bitstar_timeout_grid_s).strip() else []
    bitstar_checkpoint_grid_s = csv_floats(args.bitstar_checkpoint_grid_s) if str(args.bitstar_checkpoint_grid_s).strip() else bitstar_explicit_grid_s
    bitstar_trace_timeout_s = max(bitstar_checkpoint_grid_s or bitstar_explicit_grid_s) if (bitstar_checkpoint_grid_s or bitstar_explicit_grid_s) else float(args.bitstar_timeout_s)
    bitstar_stage_s = bitstar_checkpoint_grid_from_args(args, bitstar_trace_timeout_s)
    args.bitstar_checkpoint_interval_s = bitstar_trace_interval_for_grid(args, bitstar_stage_s, bitstar_trace_timeout_s)
    bitstar_batch_grid = csv_ints(args.bitstar_samples_per_batch_grid) if str(args.bitstar_samples_per_batch_grid).strip() else [int(args.bitstar_samples_per_batch)]
    bitstar_rewire_grid = csv_floats(args.bitstar_rewire_factor_grid) if str(args.bitstar_rewire_factor_grid).strip() else [float(args.bitstar_rewire_factor)]
    if args.phase == "smoke":
        seeds = seeds[:1]
        sbf_budgets = [int(args.sbf_box_budget)]
        prm_build_grid_s = [min(float(args.prm_build_s), 0.25)]
        prm_query_grid_s = [float(args.prm_query_s)]
        prm_k_grid = [int(args.prm_max_nearest_neighbors)]
        prm_kind_grid = [str(args.prm_planner_kind)]
        prm_range_grid = [float(args.prm_range)]
        prm_preload_grid = [bool(args.prm_preload_query_endpoints)]
        args.bitstar_timeout_s = min(float(args.bitstar_timeout_s), 0.25)
        bitstar_trace_timeout_s = float(args.bitstar_timeout_s)
        bitstar_stage_s = [float(args.bitstar_timeout_s)]
        bitstar_batch_grid = [int(args.bitstar_samples_per_batch)]
        bitstar_rewire_grid = [float(args.bitstar_rewire_factor)]
        args.bitstar_checkpoint_interval_s = min(float(args.bitstar_checkpoint_interval_s), float(args.bitstar_timeout_s))
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = shelf_queries(robot)
    planned_rows = []
    for method in methods:
        if method == "sbf_leaf_rrt":
            budgets: list[Any] = sbf_budgets
        elif method == "prm" and args.rerun_baselines:
            if args.prm_cumulative:
                budgets = [
                    {
                        "cumulative": True,
                        "checkpoints": list(prm_build_grid_s),
                        "build_s": max(prm_build_grid_s),
                        "query_s": query_s,
                        "k": k,
                        "kind": kind,
                        "range": prm_range,
                        "preload": preload,
                    }
                    for query_s in prm_query_grid_s
                    for k in prm_k_grid
                    for kind in prm_kind_grid
                    for prm_range in prm_range_grid
                    for preload in prm_preload_grid
                ]
            else:
                budgets = [
                    {"build_s": build_s, "query_s": query_s, "k": k, "kind": kind, "range": prm_range, "preload": preload}
                    for build_s in prm_build_grid_s
                    for query_s in prm_query_grid_s
                    for k in prm_k_grid
                    for kind in prm_kind_grid
                    for prm_range in prm_range_grid
                    for preload in prm_preload_grid
                ]
        elif method == "bitstar":
            budgets = [
                {"timeout_s": bitstar_trace_timeout_s, "batch": batch, "rewire": rewire}
                for batch in bitstar_batch_grid
                for rewire in bitstar_rewire_grid
            ]
        elif method == "rrtconnect" and args.rerun_baselines:
            budgets = [float(args.rrt_timeout_s)]
        else:
            budgets = [None]
        for seed in seeds:
            for budget in budgets:
                stage_id = (
                    f"b{int(budget)}" if method == "sbf_leaf_rrt"
                    else f"{str(budget.get('kind', 'prm'))}_cumulative_trace{float(budget['build_s']):g}s_k{int(budget['k'])}_q{fmt_float(float(budget['query_s']))}s_r{fmt_float(float(budget.get('range', 0.0)))}_preload{int(bool(budget.get('preload', False)))}" if method == "prm" and isinstance(budget, dict) and bool(budget.get("cumulative", False))
                    else f"{str(budget.get('kind', 'prm'))}_build{float(budget['build_s']):g}s_k{int(budget['k'])}_q{fmt_float(float(budget['query_s']))}s_r{fmt_float(float(budget.get('range', 0.0)))}_preload{int(bool(budget.get('preload', False)))}" if method == "prm" and isinstance(budget, dict)
                    else f"batch{int(budget['batch'])}_rw{fmt_float(float(budget['rewire']))}_trace{float(budget['timeout_s']):g}s" if method == "bitstar" and isinstance(budget, dict)
                    else f"timeout{float(args.rrt_timeout_s):g}s" if method == "rrtconnect" and args.rerun_baselines
                    else method
                )
                planned_rows.append(
        {
            "method": method,
            "seed": seed,
            "stage_id": stage_id,
            "budget_s": (
                float(budget["build_s"]) if method == "prm" and isinstance(budget, dict)
                else float(budget["timeout_s"]) if method == "bitstar" and isinstance(budget, dict)
                else float(budget) if method != "sbf_leaf_rrt" and budget is not None
                else None
            ),
            "deep_max_boxes": budget if method == "sbf_leaf_rrt" else None,
            "planner_params": budget if isinstance(budget, dict) else None,
            "scene": "marcucci_shelf_iiwa",
            "query_set": "AS_TS_CS_LB_RB_raw_actual",
            "audit_segment_step": float(args.audit_segment_step),
            "audit_collision_tolerance": float(args.audit_collision_tolerance),
            "active_planning_root": "full_robot_joint_limits",
            "coverage_root": "full_robot_joint_limits",
            "canonical_mapping_scope": "LECT_internal_only",
            "threads": int(args.threads),
            "ompl_simplify_time_s": float(args.ompl_simplify_time_s),
            "bitstar_profile": {
                "timeout_s": float(bitstar_trace_timeout_s),
                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                "checkpoint_grid_s": bitstar_stage_s,
                "checkpoint_stage_s": bitstar_stage_s,
                "samples_per_batch": int(args.bitstar_samples_per_batch),
                "rewire_factor": float(args.bitstar_rewire_factor),
                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
            } if method == "bitstar" else None,
            "execution_policy": (
                "current_exp04_registered_profile"
                if method == "sbf_leaf_rrt"
                else "import_old_audited_artifact"
                if method in IMPORTED_BASELINE_METHODS and not args.rerun_baselines
                else "current_execution"
            ),
            "exp04_registered_dir": str(exp04_registered_dir(args)) if method == "sbf_leaf_rrt" else None,
            "rbf_default_profile": rbf_profile if method == "sbf_leaf_rrt" else None,
            "offline_query_agnostic_build": True if method == "sbf_leaf_rrt" else None,
            "offline_anchor_count": int(args.offline_anchor_count) if method == "sbf_leaf_rrt" else None,
            "offline_anchor_candidate_count": int(args.offline_anchor_candidate_count) if method == "sbf_leaf_rrt" else None,
            "box_budgets": list(sbf_budgets) if method == "sbf_leaf_rrt" else None,
        }
                )
    rows: list[dict[str, Any]] = []
    import_payload: dict[str, Any] | None = None
    imported_rbf_summary: list[dict[str, Any]] = []
    imported_rbf_audit: dict[str, Any] | None = None
    if not args.dry_run:
        if "sbf_leaf_rrt" in methods:
            imported_rbf_rows, imported_rbf_summary, imported_rbf_audit = exp04_registered_rbf_rows(
                args,
                sbf_budgets,
            )
            rows.extend(imported_rbf_rows)
        if any(method in IMPORTED_BASELINE_METHODS for method in methods):
            import_payload = import_old_baselines(args.out_dir, args.old_paper_root, args.old_output_root)
            if import_payload["audit"]["status"] != "reusable":
                raise RuntimeError("old baseline reuse audit failed; rerun with --rerun-baselines or inspect old_shelf_baseline_reuse_audit.json")
        if "iris_np_gcs" in methods:
            iris_json = args.out_dir / "iris_np_gcs_anytime.json"
            print(f"[exp05] method=iris_np_gcs out={iris_json}", flush=True)
            iris_payload = run_shelf_iris_anytime(
                out_json=iris_json,
                python_executable=Path(args.iris_python),
                gcs_repo=Path(args.iris_gcs_repo),
                seeds=len(seeds),
                threads=int(args.threads),
                budget_s=float(args.iris_budget_s),
                stage_region_counts=str(args.iris_stage_region_counts),
                iteration_limit=int(args.iris_iteration_limit),
                query_time_limit_s=float(args.iris_query_time_limit_s),
                rounding_max_paths=int(args.iris_rounding_max_paths),
                rounding_max_trials=int(args.iris_rounding_max_trials),
                segment_step=float(args.audit_segment_step),
                final_ompl_simplify_time_s=float(args.ompl_simplify_time_s),
            )
            rows.extend(shelf_iris_json_to_run_rows(iris_payload))
        executable_rows = [
            planned
            for planned in planned_rows
            if str(planned["method"]) not in IMPORTED_BASELINE_METHODS
            and str(planned["method"]) != "iris_np_gcs"
            and str(planned["method"]) != "sbf_leaf_rrt"
        ]
        for planned in progress(executable_rows, desc="exp05 runs", total=len(executable_rows)):
            if (
                str(planned["method"]) in IMPORTED_BASELINE_METHODS
            ):
                continue
            print(f"[exp05] method={planned['method']} stage={planned.get('stage_id')} seed={planned['seed']} budget={planned.get('deep_max_boxes')}", flush=True)
            if str(planned["method"]) == "sbf_leaf_rrt":
                rows.append(run_sbf(int(planned["seed"]), args, robot, obstacles, queries, int(planned["deep_max_boxes"])))
            elif str(planned["method"]) == "bitstar":
                params = dict(planned.get("planner_params") or {})
                rows.extend(run_bitstar_trace(
                    int(planned["seed"]),
                    args,
                    robot,
                    obstacles,
                    queries,
                    timeout_s=float(params.get("timeout_s", args.bitstar_timeout_s)),
                    samples_per_batch=int(params.get("batch", args.bitstar_samples_per_batch)),
                    rewire_factor=float(params.get("rewire", args.bitstar_rewire_factor)),
                    stage_prefix=(
                        f"batch{int(params.get('batch', args.bitstar_samples_per_batch))}"
                        f"_rw{fmt_float(float(params.get('rewire', args.bitstar_rewire_factor)))}"
                        f"_trace{float(params.get('timeout_s', args.bitstar_timeout_s)):g}s"
                    ),
                ))
            elif str(planned["method"]) == "prm":
                params = dict(planned.get("planner_params") or {})
                if bool(params.get("cumulative", False)):
                    rows.extend(run_prm_cumulative(
                        int(planned["seed"]),
                        args,
                        robot,
                        obstacles,
                        queries,
                        [float(value) for value in params.get("checkpoints", [params.get("build_s", planned["budget_s"])])],
                        query_budget_s=float(params.get("query_s", args.prm_query_s)),
                        max_nearest_neighbors=int(params.get("k", args.prm_max_nearest_neighbors)),
                        planner_kind=str(params.get("kind", args.prm_planner_kind)),
                        preload_query_endpoints=bool(params.get("preload", args.prm_preload_query_endpoints)),
                    ))
                else:
                    rows.append(run_prm(
                        int(planned["seed"]),
                        args,
                        robot,
                        obstacles,
                        queries,
                        float(params.get("build_s", planned["budget_s"])),
                        query_budget_s=float(params.get("query_s", args.prm_query_s)),
                        max_nearest_neighbors=int(params.get("k", args.prm_max_nearest_neighbors)),
                        planner_kind=str(params.get("kind", args.prm_planner_kind)),
                        prm_range=float(params.get("range", args.prm_range)),
                        preload_query_endpoints=bool(params.get("preload", args.prm_preload_query_endpoints)),
                        stage_id=str(planned.get("stage_id")),
                    ))
            else:
                rows.append(run_method(
                    str(planned["method"]),
                    int(planned["seed"]),
                    args,
                    robot,
                    obstacles,
                    queries,
                    float(planned["budget_s"]) if planned.get("budget_s") is not None else None,
                ))
    summary = aggregate(rows) if rows else []
    if imported_rbf_summary:
        summary = [
            row for row in summary
            if str(row.get("method")) != "sbf_leaf_rrt"
        ]
        summary.extend(imported_rbf_summary)
    if rows:
        summary = [row for row in summary if str(row.get("method")) != "iris_np_gcs"]
        summary.extend(shelf_iris_summary_rows([row for row in rows if str(row.get("method")) == "iris_np_gcs"]))
    if import_payload is not None:
        imported_methods = set(methods) & IMPORTED_BASELINE_METHODS
        summary.extend(row for row in import_payload["summary"] if row["method"] in imported_methods)
    payload: dict[str, Any] = {
        "experiment": "exp05_shelf_cross_algorithm",
        "run_id": run_id("exp05"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "rbf_default_profile": rbf_profile,
        "audit": {
            "segment_step": float(args.audit_segment_step),
            "collision_tolerance": float(args.audit_collision_tolerance),
        },
        "baseline_execution": {
            "threads": int(args.threads),
            "thread_policy": "RBF, IRIS/GCS, and process math libraries use --threads=8 by default; OMPL RRTConnect, PRM, and BIT* are algorithmically single-thread in this runner unless OMPL exposes planner-internal parallelism.",
            "ompl_simplify_time_s": float(args.ompl_simplify_time_s),
            "bitstar": {
                "timeout_s": float(bitstar_trace_timeout_s),
                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                "checkpoint_grid_s": bitstar_stage_s,
                "checkpoint_stage_s": bitstar_stage_s,
                "samples_per_batch": int(args.bitstar_samples_per_batch),
                "rewire_factor": float(args.bitstar_rewire_factor),
                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
            },
        },
        "baseline_reuse_audit": import_payload["audit"] if import_payload is not None else None,
        "rbf_registered_profile_import": imported_rbf_audit,
        "planned_rows": planned_rows,
        "rows": rows,
        "summary": summary,
    }
    write_json(args.out_dir / "shelf_cross_algorithm_manifest.json", payload)
    if summary:
        write_csv(args.out_dir / "shelf_cross_algorithm_summary.csv", summary)
        write_per_query_csv(args.out_dir / "shelf_cross_algorithm_per_query.csv", rows)
        if str(args.phase) == "paper" and any(str(row.get("method")) == "sbf_leaf_rrt" for row in summary):
            write_tex(REPO_ROOT / "paper" / "generated" / "tab_tro_shelf_cross_algorithm.tex", summary)
    print(f"wrote {args.out_dir / 'shelf_cross_algorithm_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
