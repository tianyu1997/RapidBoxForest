#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import (  # noqa: E402
    EPS_PATH_DEFAULT,
    aggregate_stage_records,
    assert_promoted_monotone,
    euclidean_path_length,
    final_ompl_simplify_path,
    incumbent_stage_record,
    path_passes_post_audit,
    task_result,
    update_incumbents,
)
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_rrt_connect import segment_free  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, mean, median, sbf, write_json  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_04_marcucci_combined import (  # noqa: E402
    configure,
    parse_args as parse_exp4_args,
    query_payload,
    refine_corridors,
)
from sbf.marcucci import make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot  # noqa: E402


DEFAULT_SBF_STAGES = "seed:2:0:0:2:48,fast:16:0:0:2500:80,balanced:64:256:450:5000:120,quality:128:1024:1500:8000:160,high:512:2000:5000:20000:200"
DEFAULT_PRM_BUILD_GRID = "2,5,10,20,40,80"
DEFAULT_BITSTAR_TIMEOUT_S = 120.0
DEFAULT_BITSTAR_CHECKPOINT_INTERVAL_S = 2.0
DEFAULT_RRT_TIMEOUT_MS = 120000.0


def parse_csv(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def parse_float_grid(raw: str) -> list[float]:
    return [float(item) for item in parse_csv(raw)]


def parse_int_grid(raw: str) -> list[int]:
    return [int(float(item)) for item in parse_csv(raw)]


def parse_sbf_stages(raw: str) -> list[dict[str, Any]]:
    stages: list[dict[str, Any]] = []
    for index, item in enumerate(parse_csv(raw)):
        parts = item.split(":")
        if len(parts) != 6:
            raise ValueError("SBF stage format is label:quality:extra_boxes:post_budget_ms:max_boxes:ffb_depth")
        label, quality, extra_boxes, post_budget_ms, max_boxes, ffb_depth = parts
        stages.append({
            "stage_index": index,
            "stage_id": label,
            "quality_min_connected_boxes": int(quality),
            "post_connect_extra_boxes": int(extra_boxes),
            "post_connect_time_budget_ms": float(post_budget_ms),
            "max_boxes": int(max_boxes),
            "ffb_depth": int(ffb_depth),
        })
    return stages


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Anytime-incumbent trade-off runner for the shelf+IIWA scene.")
    parser.add_argument("--methods", default="sbf,prm,bitstar,rrt")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_shelf_anytime_tradeoff_full.json")
    parser.add_argument("--epsilon-path", type=float, default=EPS_PATH_DEFAULT)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--task-batch-size", type=int, default=8)
    parser.add_argument("--sbf-stages", default=DEFAULT_SBF_STAGES)
    parser.add_argument("--sbf-preset", choices=["support_hull_coverage", "kdop26_coverage", "crit_link_coverage", "ifk_strict"], default="support_hull_coverage")
    parser.add_argument("--sbf-envelope", choices=["support_hull", "kdop26", "link", "preset"], default="support_hull")
    parser.add_argument("--sbf-stage-protocol", choices=["cumulative_cold_attempts"], default="cumulative_cold_attempts")
    parser.add_argument("--sbf-bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sbf-bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--sbf-corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sbf-corridor-refine-budget-ms", type=float, default=300.0)
    parser.add_argument("--sbf-corridor-refine-max-boxes", type=int, default=24)
    parser.add_argument("--sbf-corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--sbf-corridor-refine-passes", type=int, default=1)
    parser.add_argument("--sbf-corridor-refine-start-margin-ms", type=float, default=80.0)
    parser.add_argument("--sbf-corridor-refine-defer-labels", type=str, default="CS->LB")
    parser.add_argument("--prm-build-grid-s", default=DEFAULT_PRM_BUILD_GRID)
    parser.add_argument("--prm-query-budget-s", type=float, default=2.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-simplify-time-s", type=float, default=0.10)
    parser.add_argument("--bitstar-timeout-s", type=float, default=DEFAULT_BITSTAR_TIMEOUT_S)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=DEFAULT_BITSTAR_CHECKPOINT_INTERVAL_S)
    parser.add_argument("--bitstar-restart-grid", default="", help="Deprecated compatibility flag; ignored because BIT* now runs one fixed-timeout trace per seed/query.")
    parser.add_argument("--bitstar-budget-s", type=float, default=None, help="Deprecated alias for --bitstar-timeout-s when provided.")
    parser.add_argument("--bitstar-simplify-time-s", type=float, default=0.2)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=-1)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=-1.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rrt-timeout-ms", type=float, default=DEFAULT_RRT_TIMEOUT_MS)
    parser.add_argument("--rrt-restart-grid", default="", help="Deprecated compatibility flag; ignored because RRTConnect now runs one max-timeout attempt per seed/query.")
    parser.add_argument("--rrt-step-size", type=float, default=0.35)
    parser.add_argument("--rrt-simplify-time-s", type=float, default=0.05)
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.01)
    parser.add_argument("--segment-step", type=float, default=0.01)
    parser.add_argument("--audit-segment-step", type=float, default=0.01, help="Independent dense post-hoc path audit step in joint-space radians. Use <=0 to reuse --segment-step.")
    return parser.parse_args()


def audit_segment_step(args: argparse.Namespace) -> float:
    value = float(getattr(args, "audit_segment_step", 0.0) or 0.0)
    return value if value > 0.0 else float(args.segment_step)


def audit_path(robot: Any, obstacles: list[Any], path: list[list[float]], step: float) -> bool:
    if len(path) < 2:
        return False
    return all(segment_free(robot, obstacles, path[index], path[index + 1], step) for index in range(len(path) - 1))


def post_audit_query_path(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    step: float,
    *,
    start: list[float],
    goal: list[float],
) -> bool:
    return path_passes_post_audit(
        sbf,
        robot,
        obstacles,
        path,
        segment_step=float(step),
        start=list(start),
        goal=list(goal),
    )


def planner_internal_simplify_time(unified_simplify_s: float, planner_simplify_s: float) -> float:
    return 0.0 if float(unified_simplify_s) > 0.0 else float(planner_simplify_s)


def unified_final_simplify_enabled(args: argparse.Namespace) -> bool:
    return float(getattr(args, "final_ompl_simplify_time_s", 0.0)) > 0.0


def make_exp4_stage_args(args: argparse.Namespace, stage: dict[str, Any], seed_index: int) -> argparse.Namespace:
    exp4 = parse_exp4_args([])
    exp4.preset = str(args.sbf_preset)
    exp4.envelope = str(args.sbf_envelope)
    exp4.seed_base = int(args.seed_base) + int(seed_index)
    exp4.seeds = 1
    exp4.threads = int(args.threads)
    exp4.task_batch_size = int(args.task_batch_size)
    exp4.max_boxes = int(stage["max_boxes"])
    exp4.ffb_depth = int(stage["ffb_depth"])
    exp4.quality_min_connected_boxes = int(stage["quality_min_connected_boxes"])
    exp4.post_connect_extra_boxes = int(stage["post_connect_extra_boxes"])
    exp4.post_connect_time_budget_ms = float(stage["post_connect_time_budget_ms"])
    exp4.corridor_refine = bool(args.sbf_corridor_refine)
    exp4.corridor_refine_budget_ms = float(args.sbf_corridor_refine_budget_ms)
    exp4.corridor_refine_max_boxes = int(args.sbf_corridor_refine_max_boxes)
    exp4.corridor_refine_boxes_per_query = int(args.sbf_corridor_refine_boxes_per_query)
    exp4.corridor_refine_passes = int(args.sbf_corridor_refine_passes)
    exp4.corridor_refine_start_margin_ms = float(args.sbf_corridor_refine_start_margin_ms)
    exp4.corridor_refine_defer_labels = str(args.sbf_corridor_refine_defer_labels)
    exp4.bridge_failed_queries = bool(args.sbf_bridge_failed_queries)
    exp4.bridge_repaired_queries = bool(args.sbf_bridge_repaired_queries)
    exp4.collision_shortcut = False
    return exp4


def run_sbf_query(
    forest: Any,
    robot: Any,
    obstacles: list[Any],
    query: Any,
    exp4_args: argparse.Namespace,
    *,
    segment_step: float,
    audit_step: float,
    final_ompl_simplify_time_s: float,
    epsilon_path: float,
) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    query_s = time.perf_counter() - query_t0
    should_bridge = (not result.success and exp4_args.bridge_failed_queries) or (
        bool(exp4_args.bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        if hasattr(forest, "bridge_query_known_needed"):
            bridge_progress = int(forest.bridge_query_known_needed(list(query.start), list(query.goal)))
        else:
            bridge_progress = int(forest.bridge_query(list(query.start), list(query.goal)))
        bridge_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        retry_s = time.perf_counter() - retry_t0
        row = query_payload(query, result, query_s + bridge_s + retry_s)
        row["bridge_progress"] = int(bridge_progress)
        row["bridge_time_s"] = float(bridge_s)
    else:
        row = query_payload(query, result, query_s)
        row["bridge_progress"] = 0
        row["bridge_time_s"] = 0.0
    if bool(row.get("ok")) and bool(row.get("audit_passed")):
        final_simplify = final_ompl_simplify_path(
            sbf,
            robot,
            obstacles,
            [list(point) for point in row.get("waypoints", [])],
            segment_step=float(segment_step),
            audit_segment_step=float(audit_step),
            simplify_time_s=float(final_ompl_simplify_time_s),
            epsilon_path=float(epsilon_path),
        )
        row["t_s"] = float(row.get("t_s", 0.0) or 0.0) + float(final_simplify["query_s"])
        row["waypoints"] = [list(point) for point in final_simplify["path"]]
        row["waypoint_count"] = len(row["waypoints"])
        if final_simplify["path_length"] is not None:
            row["length"] = float(final_simplify["path_length"])
        row["ompl_final_simplify_time_s"] = float(final_simplify["query_s"])
        row["ompl_final_simplify_applied"] = bool(final_simplify["applied"])
        row["ompl_final_simplify_reason"] = str(final_simplify["reason"])
    if bool(row.get("ok")):
        row["audit_passed"] = post_audit_query_path(
            robot,
            obstacles,
            [list(point) for point in row.get("waypoints", [])],
            float(audit_step),
            start=list(query.start),
            goal=list(query.goal),
        )
        if not bool(row.get("audit_passed")):
            row["audit_status"] = "PostAuditFailed"
            row["ok"] = False
    row["name"] = query.label
    return row


def run_sbf_trace(args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[Any], seed_index: int) -> list[dict[str, Any]]:
    coverage_seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]
    task_count = len(queries)
    incumbents: dict[str, dict[str, Any]] = {}
    cumulative_build_s = 0.0
    cumulative_query_s = 0.0
    records: list[dict[str, Any]] = []
    for stage_index, stage in enumerate(parse_sbf_stages(args.sbf_stages)):
        exp4_args = make_exp4_stage_args(args, stage, seed_index)
        cfg = configure(exp4_args, seed_index)
        forest = sbf.SafeBoxForest(robot, cfg)
        build_t0 = time.perf_counter()
        profile = forest.build_coverage(obstacles, coverage_seeds)
        prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, exp4_args)
        stage_build_s = time.perf_counter() - build_t0
        raw_rows = [
            run_sbf_query(
                forest,
                robot,
                obstacles,
                query,
                exp4_args,
                segment_step=audit_segment_step(args),
                audit_step=audit_segment_step(args),
                final_ompl_simplify_time_s=float(args.final_ompl_simplify_time_s),
                epsilon_path=float(args.epsilon_path),
            )
            for query in queries
        ]
        stage_query_s = sum(float(row.get("t_s", 0.0)) for row in raw_rows)
        candidates = [
            task_result(
                name=str(row["name"]),
                ok=bool(row.get("ok")),
                audit_passed=bool(row.get("audit_passed")),
                path_length=float(row.get("length", 0.0)) if row.get("ok") else None,
                query_s=float(row.get("t_s", 0.0)),
                reason=str(row.get("audit_status", "")),
                extra={"raw": row},
            )
            for row in raw_rows
        ]
        incumbents, improved = update_incumbents(incumbents, candidates, epsilon_path=float(args.epsilon_path))
        cumulative_build_s += stage_build_s
        cumulative_query_s += stage_query_s
        params = {
            **stage,
            "preset": args.sbf_preset,
            "envelope": args.sbf_envelope,
            "prebridge_time_s": float(prebridge_time_s),
            "prebridge_added_boxes": int(prebridge_added_boxes),
            "prebridge_attempts": int(prebridge_attempts),
            "box_count": len(forest.boxes()),
            "segment_edge_count": len(forest.segment_edges()),
            "build_profile_total_ms": float(profile.total_ms),
        }
        records.append(incumbent_stage_record(
            method="sbf_cold",
            stage_id=str(stage["stage_id"]),
            stage_index=stage_index,
            seed_index=seed_index,
            task_count=task_count,
            cumulative_build_s=cumulative_build_s,
            cumulative_query_s=cumulative_query_s,
            stage_build_s=stage_build_s,
            stage_query_s=stage_query_s,
            raw_tasks=candidates,
            incumbents=incumbents,
            improved_tasks=improved,
            params=params,
            protocol=str(args.sbf_stage_protocol),
        ))
    return records


def prm_task_rows(args: argparse.Namespace, raw: dict[str, Any], queries: list[Any], robot: Any, obstacles: list[Any], segment_step: float, seed_index: int, stage_index: int) -> list[dict[str, Any]]:
    rows = []
    for query, item in zip(queries, raw.get("queries", [])):
        path = [list(point) for point in item.get("path", [])]
        exact = str(item.get("status")) == "Exact solution"
        ok = bool(item.get("ok")) and exact and len(path) >= 2
        query_s = float(item.get("t_s", 0.0)) if ok else 0.0
        if ok:
            final_simplify = final_ompl_simplify_path(
                sbf,
                robot,
                obstacles,
                path,
                segment_step=segment_step,
                audit_segment_step=audit_segment_step(args),
                simplify_time_s=float(args.final_ompl_simplify_time_s),
                epsilon_path=float(args.epsilon_path),
            )
            path = [list(point) for point in final_simplify["path"]]
            query_s += float(final_simplify["query_s"])
        audit_passed = post_audit_query_path(
            robot,
            obstacles,
            path,
            audit_segment_step(args),
            start=list(query.start),
            goal=list(query.goal),
        ) if ok else False
        rows.append(task_result(
            name=query.label,
            ok=ok,
            audit_passed=audit_passed,
            path_length=euclidean_path_length(path) if ok else None,
            query_s=query_s,
            reason=item.get("reason"),
            extra={
                "seed_index": int(seed_index),
                "stage_index": int(stage_index),
                "status": item.get("status"),
                "waypoint_count": len(path),
                "waypoints": path,
                "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if ok else 0.0,
                "ompl_final_simplify_applied": bool(final_simplify["applied"]) if ok else False,
                "ompl_final_simplify_reason": str(final_simplify["reason"]) if ok else "disabled",
            },
        ))
    return rows


def run_prm_trace(args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[Any], seed_index: int) -> list[dict[str, Any]]:
    starts = [list(query.start) for query in queries]
    goals = [list(query.goal) for query in queries]
    task_count = len(queries)
    incumbents: dict[str, dict[str, Any]] = {}
    cumulative_build_s = 0.0
    cumulative_query_s = 0.0
    records: list[dict[str, Any]] = []
    for stage_index, build_budget_s in enumerate(parse_float_grid(args.prm_build_grid_s)):
        rng_seed = (int(args.seed_base) + 15485863 * seed_index + 104729 * stage_index) % 2147483647
        raw = sbf.ompl_prm_multiquery(
            robot,
            obstacles,
            starts,
            goals,
            float(build_budget_s),
            float(args.prm_query_budget_s),
            audit_segment_step(args),
            planner_internal_simplify_time(float(args.final_ompl_simplify_time_s), float(args.prm_simplify_time_s)),
            int(rng_seed),
            int(args.prm_max_nearest_neighbors),
        )
        stage_build_s = float(raw.get("build_s", 0.0))
        tasks = prm_task_rows(args, raw, queries, robot, obstacles, audit_segment_step(args), seed_index, stage_index)
        stage_query_s = sum(float(row.get("query_s", 0.0)) for row in tasks)
        incumbents, improved = update_incumbents(incumbents, tasks, epsilon_path=float(args.epsilon_path))
        cumulative_build_s += stage_build_s
        cumulative_query_s += stage_query_s
        records.append(incumbent_stage_record(
            method="ompl_prm",
            stage_id=f"build{build_budget_s:g}s",
            stage_index=stage_index,
            seed_index=seed_index,
            task_count=task_count,
            cumulative_build_s=cumulative_build_s,
            cumulative_query_s=cumulative_query_s,
            stage_build_s=stage_build_s,
            stage_query_s=stage_query_s,
            raw_tasks=tasks,
            incumbents=incumbents,
            improved_tasks=improved,
            params={"build_budget_s": float(build_budget_s), "rng_seed": int(rng_seed), "nodes": int(raw.get("nodes", 0) or 0)},
            protocol="cumulative_independent_shared_roadmaps",
        ))
    return records


def run_bitstar_trace(args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[Any], seed_index: int) -> list[dict[str, Any]]:
    task_count = len(queries)
    incumbents: dict[str, dict[str, Any]] = {}
    records: list[dict[str, Any]] = []
    timeout_s = float(args.bitstar_timeout_s if args.bitstar_budget_s is None else args.bitstar_budget_s)
    interval_s = float(args.bitstar_checkpoint_interval_s)
    query_traces: list[tuple[Any, int, list[dict[str, Any]]]] = []
    for query_index, query in enumerate(queries):
        rng_seed = (int(args.seed_base) + 32452843 * seed_index + 49979687 * query_index) % 2147483647
        raw = sbf.ompl_bitstar_trace(
            robot,
            obstacles,
            list(query.start),
            list(query.goal),
            timeout_s * 1000.0,
            interval_s * 1000.0,
            audit_segment_step(args),
            int(rng_seed),
            int(args.bitstar_samples_per_batch),
            float(args.bitstar_rewire_factor),
            bool(args.bitstar_stop_on_solution_improvement),
        )
        query_traces.append((query, int(rng_seed), [dict(item) for item in raw.get("checkpoints", [])]))
    stage_count = max((len(checkpoints) for _, _, checkpoints in query_traces), default=0)
    previous_elapsed_by_query = {str(query.label): 0.0 for query, _, _ in query_traces}
    for stage_index in range(stage_count):
        tasks: list[dict[str, Any]] = []
        stage_query_s = 0.0
        cumulative_query_s = 0.0
        checkpoint_seconds: list[float] = []
        for query, rng_seed, checkpoints in query_traces:
            checkpoint = checkpoints[min(stage_index, len(checkpoints) - 1)] if checkpoints else {}
            checkpoint_s = float(checkpoint.get("checkpoint_s", (stage_index + 1) * interval_s) or 0.0)
            checkpoint_seconds.append(checkpoint_s)
            path = [list(point) for point in checkpoint.get("path", [])]
            ok = bool(checkpoint.get("ok")) and len(path) >= 2
            query_s = float(checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0)) or 0.0)
            if ok:
                final_simplify = final_ompl_simplify_path(
                    sbf,
                    robot,
                    obstacles,
                    path,
                    segment_step=audit_segment_step(args),
                    audit_segment_step=audit_segment_step(args),
                    simplify_time_s=float(args.final_ompl_simplify_time_s),
                    epsilon_path=float(args.epsilon_path),
                )
                path = [list(point) for point in final_simplify["path"]]
                query_s += float(final_simplify["query_s"])
            audit_passed = post_audit_query_path(
                robot,
                obstacles,
                path,
                audit_segment_step(args),
                start=list(query.start),
                goal=list(query.goal),
            ) if ok else False
            stage_query_s += max(0.0, query_s - previous_elapsed_by_query[str(query.label)])
            cumulative_query_s += query_s
            previous_elapsed_by_query[str(query.label)] = query_s
            tasks.append(task_result(
                name=query.label,
                ok=ok,
                audit_passed=audit_passed,
                path_length=euclidean_path_length(path) if ok else None,
                query_s=query_s,
                reason=checkpoint.get("reason"),
                extra={
                    "checkpoint_s": checkpoint_s,
                    "rng_seed": int(rng_seed),
                    "waypoint_count": len(path),
                    "waypoints": path,
                    "iterations": int(checkpoint.get("iterations", 0) or 0),
                    "batches": int(checkpoint.get("batches", 0) or 0),
                    "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if ok else 0.0,
                    "ompl_final_simplify_applied": bool(final_simplify["applied"]) if ok else False,
                    "ompl_final_simplify_reason": str(final_simplify["reason"]) if ok else "disabled",
                },
            ))
        incumbents, improved = update_incumbents(incumbents, tasks, epsilon_path=float(args.epsilon_path))
        stage_checkpoint_s = max(checkpoint_seconds) if checkpoint_seconds else float((stage_index + 1) * interval_s)
        records.append(incumbent_stage_record(
            method="ompl_bitstar",
            stage_id=f"t{stage_checkpoint_s:g}s",
            stage_index=stage_index,
            seed_index=seed_index,
            task_count=task_count,
            cumulative_build_s=0.0,
            cumulative_query_s=cumulative_query_s,
            stage_build_s=0.0,
            stage_query_s=stage_query_s,
            raw_tasks=tasks,
            incumbents=incumbents,
            improved_tasks=improved,
            params={
                "timeout_s": float(timeout_s),
                "checkpoint_interval_s": float(interval_s),
                "checkpoint_s": float(stage_checkpoint_s),
            },
            protocol="fixed_timeout_checkpoint_trace",
        ))
    return records


def run_rrt_trace(args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[Any], seed_index: int) -> list[dict[str, Any]]:
    task_count = len(queries)
    incumbents: dict[str, dict[str, Any]] = {}
    tasks: list[dict[str, Any]] = []
    stage_query_s = 0.0
    for query_index, query in enumerate(queries):
        rng_seed = (int(args.seed_base) + 6700417 * seed_index + 104729 * query_index) % 2147483647
        raw = sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            list(query.start),
            list(query.goal),
            float(args.rrt_timeout_ms),
            float(args.rrt_step_size),
            audit_segment_step(args),
            planner_internal_simplify_time(float(args.final_ompl_simplify_time_s), float(args.rrt_simplify_time_s)),
            int(rng_seed),
        )
        path = [list(point) for point in raw.get("path", [])]
        exact = bool(raw.get("exact_solution")) or str(raw.get("status")) == "Exact solution"
        ok = bool(raw.get("ok")) and exact and len(path) >= 2
        query_s = float(raw.get("t_s", 0.0) or 0.0)
        if ok:
            final_simplify = final_ompl_simplify_path(
                sbf,
                robot,
                obstacles,
                path,
                segment_step=audit_segment_step(args),
                audit_segment_step=audit_segment_step(args),
                simplify_time_s=float(args.final_ompl_simplify_time_s),
                epsilon_path=float(args.epsilon_path),
            )
            path = [list(point) for point in final_simplify["path"]]
            query_s += float(final_simplify["query_s"])
        audit_passed = post_audit_query_path(
            robot,
            obstacles,
            path,
            audit_segment_step(args),
            start=list(query.start),
            goal=list(query.goal),
        ) if ok else False
        stage_query_s += query_s
        tasks.append(task_result(
            name=query.label,
            ok=ok,
            audit_passed=audit_passed,
            path_length=euclidean_path_length(path) if ok else None,
            query_s=query_s,
            reason=raw.get("reason"),
            extra={
                "rng_seed": int(rng_seed),
                "timeout_ms": float(args.rrt_timeout_ms),
                "waypoint_count": len(path),
                "waypoints": path,
                "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if ok else 0.0,
                "ompl_final_simplify_applied": bool(final_simplify["applied"]) if ok else False,
                "ompl_final_simplify_reason": str(final_simplify["reason"]) if ok else "disabled",
            },
        ))
    incumbents, improved = update_incumbents(incumbents, tasks, epsilon_path=float(args.epsilon_path))
    return [incumbent_stage_record(
        method="ompl_rrtconnect",
        stage_id=f"timeout{float(args.rrt_timeout_ms) / 1000.0:g}s",
        stage_index=0,
        seed_index=seed_index,
        task_count=task_count,
        cumulative_build_s=0.0,
        cumulative_query_s=stage_query_s,
        stage_build_s=0.0,
        stage_query_s=stage_query_s,
        raw_tasks=tasks,
        incumbents=incumbents,
        improved_tasks=improved,
        params={"timeout_ms": float(args.rrt_timeout_ms), "step_size": float(args.rrt_step_size)},
        protocol="single_run_max_timeout",
    )]


def stage_debug(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [{
        "method": row["method"],
        "stage": row["stage_id"],
        "seed": row["seed_index"],
        "total_s": row["cumulative_total_s"],
        "build_s": row["cumulative_build_s"],
        "query_s": row["cumulative_query_s"],
        "path": row["incumbent_mean_length"],
        "path_total": row["incumbent_total_length"],
        "sr": row["incumbent_audit_sr"],
        "improved": row["improved"],
    } for row in records]


def main() -> int:
    args = parse_args()
    methods = set(parse_csv(args.methods))
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    records: list[dict[str, Any]] = []
    for seed_index in range(max(1, int(args.seeds))):
        print(f"[shelf-anytime] seed={seed_index}", flush=True)
        if "sbf" in methods:
            records.extend(run_sbf_trace(args, robot, obstacles, queries, seed_index))
        if "prm" in methods:
            records.extend(run_prm_trace(args, robot, obstacles, queries, seed_index))
        if "bitstar" in methods:
            records.extend(run_bitstar_trace(args, robot, obstacles, queries, seed_index))
        if "rrt" in methods:
            records.extend(run_rrt_trace(args, robot, obstacles, queries, seed_index))
    summary = aggregate_stage_records(records, epsilon_path=float(args.epsilon_path))
    assert_promoted_monotone(summary, epsilon_path=float(args.epsilon_path))
    payload = {
        "experiment": "tro2026_shelf_anytime_tradeoff",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "nested_anytime_incumbent_with_cumulative_charged_time",
        "note": "Each method retains the best audited path found so far. More expensive tiers are promoted as main trade-off events only when the incumbent improves; no-improvement tiers remain in raw records and appendix summaries.",
        "scene": "shelf_iiwa_marcucci_combined",
        "task_names": [query.label for query in queries],
        "params": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "audit_protocol": {
            "planner_segment_step": float(audit_segment_step(args)),
            "path_audit_segment_step": float(audit_segment_step(args)),
            "endpoint_aware": True,
        },
        "summary": summary,
        "records": records,
        "debug_rows": stage_debug(records),
        "method_build_query_semantics": {
            "sbf_cold": "cumulative cold SBF attempts with no fingerprint cache; build time is charged cumulatively and incumbents are retained across tiers",
            "ompl_prm": "cumulative independent shared-roadmap attempts; all roadmap build and query time is charged",
            "ompl_bitstar": "one fixed-timeout BIT* invocation per seed/query with incumbent checkpoints at a fixed interval; each checkpoint counts strict-audited paths available by that elapsed time",
            "ompl_rrtconnect": "one query-only RRTConnect/BiRRT invocation per seed/query with a maximum timeout; the planner returns on connection and timeout is failure",
        },
    }
    write_json(args.out_json, payload)
    print(json.dumps({
        "out_json": str(args.out_json),
        "records": len(records),
        "promoted_points": len(summary.get("promoted_points", [])),
        "points": len(summary.get("points", [])),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())