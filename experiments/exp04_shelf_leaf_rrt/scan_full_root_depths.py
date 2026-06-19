#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id
from experiments.common.metrics import mean, median
from experiments.common.progress import progress
from experiments.common.rbf_defaults import (
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_RESOLUTION,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD,
    DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES,
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION,
    DEFAULT_RBF_CONNECTOR_PAVE_DEPTH,
    DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS,
    DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN,
    DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN,
    DEFAULT_RBF_CONNECTOR_PAVE_STEPS,
    DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS,
    DEFAULT_RBF_CONNECTOR_RRT_ITERS,
    DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE,
    DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION,
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_DOMAIN_ATTEMPT_CAP,
    DEFAULT_RBF_DOMAIN_SEED_CAP,
    DEFAULT_RBF_DOMAIN_SUCCESS_CAP,
    DEFAULT_RBF_FINAL_COLLISION_SHORTCUT,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_QUERY_BRIDGE_ALL,
    DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
    DEFAULT_RBF_REFINE_TIMEOUT_MS,
    DEFAULT_RBF_THREADS,
    DEFAULT_RBF_VALIDATION_BATCH_SIZE,
)
from experiments.exp04_shelf_leaf_rrt.run_shelf_leaf_rrt import run_case


DEFAULT_CASE = "baseline_d23_aafk_support_hull_8t"
MAX_SCAN_LEAF_DEPTH = 20


def parse_csv_ints(raw: str) -> list[int]:
    return [int(item.strip()) for item in str(raw).split(",") if item.strip()]


def parse_depth_configs(raw: str) -> list[tuple[int, int]]:
    configs: list[tuple[int, int]] = []
    for item in str(raw).split(","):
        item = item.strip()
        if not item:
            continue
        parts = item.lower().replace("d", "").split("-")
        if len(parts) != 2:
            raise ValueError(f"bad depth config '{item}', expected START-MAX")
        start, max_depth = int(parts[0]), int(parts[1])
        if max_depth > MAX_SCAN_LEAF_DEPTH:
            raise ValueError(f"leaf_max_depth must not exceed {MAX_SCAN_LEAF_DEPTH} for this scan: {item}")
        configs.append((start, max_depth))
    return configs


def finite(value: Any) -> float:
    try:
        result = float(value)
    except Exception:
        return math.nan
    return result if math.isfinite(result) else math.nan


def flatten_run(row: dict[str, Any], config_id: str) -> dict[str, Any]:
    queries = list(row.get("queries", []))
    ts_cs = next((item for item in queries if str(item.get("label")) == "TS->CS"), {})
    return {
        "config_id": config_id,
        "seed": int(row["seed"]),
        "deep_max_boxes": int(row["deep_max_boxes"]),
        "status": row.get("status"),
        "success_count": int(row.get("success_count", 0)),
        "query_count": int(row.get("query_count", 0)),
        "planning_s": finite(row.get("planning_s")),
        "build_s": finite(row.get("build_s")),
        "query_s": finite(row.get("query_s")),
        "leaf_sweep_s": finite(row.get("leaf_sweep_s")),
        "deep_refine_s": finite(row.get("deep_refine_s")),
        "rrt_grower_s": finite(row.get("rrt_grower_s")),
        "connector_s": finite(row.get("connector_s")),
        "query_bridge_s": finite(row.get("query_bridge_s")),
        "audit_s": finite(row.get("audit_s")),
        "path_length_mean": finite(row.get("path_length_mean")),
        "raw_segment_fraction": finite(row.get("raw_segment_fraction")),
        "ts_cs_length": finite(ts_cs.get("path_length")),
        "ts_cs_raw_length": finite(ts_cs.get("raw_path_length")),
        "ts_cs_segment_fraction": finite(ts_cs.get("segment_fraction")),
        "leaf_free_count": int(row.get("leaf_free_count", 0)),
        "leaf_collision_count": int(row.get("leaf_collision_count", 0)),
        "deep_boxes_added": int(row.get("deep_boxes_added", 0)),
        "rrt_grower_boxes_added": int(row.get("rrt_grower_boxes_added", 0)),
        "query_bridge_added": int(row.get("query_bridge_added", 0)),
        "final_boxes": int(row.get("final_boxes", 0)),
        "segment_edges": int(row.get("segment_edges", 0)),
        "adjacency_islands": int(row.get("adjacency_islands", 0)),
        "external_hits": finite(row.get("external_hits")),
        "priority_prune_free_before": finite(row.get("diagnostics", {}).get("leaf_refine.priority_prune_free_before")),
        "priority_prune_free_after": finite(row.get("diagnostics", {}).get("leaf_refine.priority_prune_free_after")),
        "priority_prune_collision_before": finite(row.get("diagnostics", {}).get("leaf_refine.priority_prune_collision_before")),
        "priority_prune_collision_after": finite(row.get("diagnostics", {}).get("leaf_refine.priority_prune_collision_after")),
        "collision_overlap_pruned": finite(row.get("diagnostics", {}).get("leaf_sweep.collision_overlap_pruned")),
    }


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(str(row["config_id"]), []).append(row)
    summary: list[dict[str, Any]] = []
    for config_id, items in sorted(grouped.items()):
        full = [row for row in items if int(row["success_count"]) == int(row["query_count"]) and int(row["query_count"]) > 0]
        summary.append({
            "config_id": config_id,
            "runs": len(items),
            "success_runs": len(full),
            "planning_s_median": median(row["planning_s"] for row in full),
            "planning_s_max": max((finite(row["planning_s"]) for row in full), default=math.nan),
            "build_s_median": median(row["build_s"] for row in full),
            "leaf_sweep_s_median": median(row["leaf_sweep_s"] for row in full),
            "deep_refine_s_median": median(row["deep_refine_s"] for row in full),
            "rrt_grower_s_median": median(row["rrt_grower_s"] for row in full),
            "connector_s_median": median(row["connector_s"] for row in full),
            "query_bridge_s_median": median(row["query_bridge_s"] for row in full),
            "path_length_mean": mean(row["path_length_mean"] for row in full),
            "path_length_median": median(row["path_length_mean"] for row in full),
            "raw_segment_fraction_median": median(row["raw_segment_fraction"] for row in full),
            "raw_segment_fraction_max": max((finite(row["raw_segment_fraction"]) for row in full), default=math.nan),
            "ts_cs_length_mean": mean(row["ts_cs_length"] for row in full),
            "ts_cs_segment_fraction_median": median(row["ts_cs_segment_fraction"] for row in full),
            "final_boxes_median": median(row["final_boxes"] for row in full),
            "leaf_free_count_median": median(row["leaf_free_count"] for row in full),
            "deep_boxes_added_median": median(row["deep_boxes_added"] for row in full),
            "query_bridge_added_median": median(row["query_bridge_added"] for row in full),
            "adjacency_islands_median": median(row["adjacency_islands"] for row in full),
            "external_hits_median": median(row["external_hits"] for row in full),
            "collision_overlap_pruned_median": median(row["collision_overlap_pruned"] for row in full),
        })
    return summary


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def make_args(base: argparse.Namespace, leaf_start: int, leaf_max: int, deep_ffb: int, ffb_start: int) -> SimpleNamespace:
    return SimpleNamespace(
        out_dir=base.out_dir,
        threads=int(base.threads),
        timeout_ms=float(base.timeout_ms),
        rbf_max_depth=int(base.rbf_max_depth),
        leaf_start_depth=int(leaf_start),
        leaf_max_depth=int(leaf_max),
        deep_ffb_depth=int(deep_ffb),
        refine_timeout_ms=float(base.refine_timeout_ms),
        domain_seed_cap=int(base.domain_seed_cap),
        domain_success_cap=int(base.domain_success_cap),
        domain_attempt_cap=int(base.domain_attempt_cap),
        validation_batch_size=int(base.validation_batch_size),
        ffb_start_depth=int(ffb_start),
        ffb_search_mode=str(base.ffb_search_mode),
        audit_resolution=int(base.audit_resolution),
        audit_segment_step=float(base.audit_segment_step),
        audit_collision_tolerance=float(base.audit_collision_tolerance),
        query_shortcut_boxes=False,
        segment_edges_fallback_only=bool(base.segment_edges_fallback_only),
        connector_birrt=True,
        connector_bridge_boxes=int(base.connector_bridge_boxes),
        connector_pair_batch_size=int(base.connector_pair_batch_size),
        connector_pair_timeout_ms=float(base.connector_pair_timeout_ms),
        connector_max_pairs_per_gap=int(base.connector_max_pairs_per_gap),
        connector_rrt_iters=int(base.connector_rrt_iters),
        connector_rrt_timeout_ms=float(base.connector_rrt_timeout_ms),
        connector_rrt_step_size=float(base.connector_rrt_step_size),
        connector_rrt_goal_bias=float(base.connector_rrt_goal_bias),
        connector_segment_resolution=int(base.connector_segment_resolution),
        connector_pave_max_chain=int(base.connector_pave_max_chain),
        connector_pave_steps=int(base.connector_pave_steps),
        connector_pave_depth=int(base.connector_pave_depth),
        connector_adaptive_min_segment_fraction=float(base.connector_adaptive_min_segment_fraction),
        query_bridge_pave_depth=int(base.query_bridge_pave_depth),
        connector_pave_fill_gaps=bool(base.connector_pave_fill_gaps),
        connector_pave_require_connected_chain=bool(base.connector_pave_require_connected_chain),
        final_collision_shortcut=bool(base.final_collision_shortcut),
        final_rrt_simplify=bool(base.final_rrt_simplify),
        final_rrt_simplify_timeout_ms=float(base.final_rrt_simplify_timeout_ms),
        final_rrt_simplify_max_iters=int(base.final_rrt_simplify_max_iters),
        final_rrt_simplify_attempts=int(base.final_rrt_simplify_attempts),
        corridor_refine=False,
        corridor_refine_budget_ms=0.0,
        corridor_refine_max_boxes=0,
        corridor_refine_boxes_per_query=12,
        corridor_refine_passes=1,
        corridor_refine_start_margin_ms=0.0,
        corridor_refine_long_path_ratio=1.25,
        corridor_refine_min_delta=0.25,
        query_bridge_all=bool(base.query_bridge_all),
        query_bridge_labels=str(base.query_bridge_labels),
        run_rrt_grower=bool(base.run_rrt_grower),
        rrt_grower_extra_boxes=int(base.rrt_grower_extra_boxes),
        rrt_grower_timeout_ms=float(base.rrt_grower_timeout_ms),
        priority_prune_radius=float(base.priority_prune_radius),
        collision_overlap_prune_min_depth=int(base.collision_overlap_prune_min_depth),
        collision_overlap_prune_threshold=float(base.collision_overlap_prune_threshold),
        collision_overlap_prune_ratio_threshold=float(base.collision_overlap_prune_ratio_threshold),
        use_virtual_topology=True,
        parallel_virtual_validation=bool(base.parallel_virtual_validation),
        rbf_cache_root=base.rbf_cache_root,
        warm_cache_label=str(base.warm_cache_label),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Coarse depth scan for Exp.4 RBF shelf cases.")
    parser.add_argument("--case", default=DEFAULT_CASE)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp04_full_root_depth_scan")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--box-budgets", default="200")
    parser.add_argument("--depth-configs", default=f"{DEFAULT_RBF_LEAF_START_DEPTH}-{DEFAULT_RBF_LEAF_MAX_DEPTH}")
    parser.add_argument("--deep-ffb-depths", default=str(DEFAULT_RBF_DEEP_FFB_DEPTH))
    parser.add_argument("--ffb-start-depths", default=str(DEFAULT_RBF_FFB_START_DEPTH))
    parser.add_argument("--ffb-search-mode", default=DEFAULT_RBF_FFB_SEARCH_MODE, choices=["linear", "binary", "binary-depth", "BinaryDepth", "Linear"])
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--rbf-max-depth", type=int, default=60)
    parser.add_argument("--refine-timeout-ms", type=float, default=DEFAULT_RBF_REFINE_TIMEOUT_MS)
    parser.add_argument("--domain-seed-cap", type=int, default=DEFAULT_RBF_DOMAIN_SEED_CAP)
    parser.add_argument("--domain-success-cap", type=int, default=DEFAULT_RBF_DOMAIN_SUCCESS_CAP)
    parser.add_argument("--domain-attempt-cap", type=int, default=DEFAULT_RBF_DOMAIN_ATTEMPT_CAP)
    parser.add_argument("--validation-batch-size", type=int, default=DEFAULT_RBF_VALIDATION_BATCH_SIZE)
    parser.add_argument("--audit-resolution", type=int, default=DEFAULT_RBF_AUDIT_RESOLUTION)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--segment-edges-fallback-only", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-bridge-boxes", type=int, default=DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES)
    parser.add_argument("--connector-pair-batch-size", type=int, default=1)
    parser.add_argument("--connector-pair-timeout-ms", type=float, default=DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP)
    parser.add_argument("--connector-rrt-iters", type=int, default=DEFAULT_RBF_CONNECTOR_RRT_ITERS)
    parser.add_argument("--connector-rrt-timeout-ms", type=float, default=DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS)
    parser.add_argument("--connector-rrt-step-size", type=float, default=DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE)
    parser.add_argument("--connector-rrt-goal-bias", type=float, default=DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS)
    parser.add_argument("--connector-segment-resolution", type=int, default=DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION)
    parser.add_argument("--connector-pave-max-chain", type=int, default=DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN)
    parser.add_argument("--connector-pave-steps", type=int, default=DEFAULT_RBF_CONNECTOR_PAVE_STEPS)
    parser.add_argument("--connector-pave-depth", type=int, default=DEFAULT_RBF_CONNECTOR_PAVE_DEPTH)
    parser.add_argument("--connector-adaptive-min-segment-fraction", type=float, default=DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION)
    parser.add_argument("--query-bridge-pave-depth", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH)
    parser.add_argument("--connector-pave-fill-gaps", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS)
    parser.add_argument("--connector-pave-require-connected-chain", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN)
    parser.add_argument("--final-collision-shortcut", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_COLLISION_SHORTCUT)
    parser.add_argument("--final-rrt-simplify", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY)
    parser.add_argument("--final-rrt-simplify-timeout-ms", type=float, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS)
    parser.add_argument("--final-rrt-simplify-max-iters", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS)
    parser.add_argument("--final-rrt-simplify-attempts", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS)
    parser.add_argument("--query-bridge-all", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_QUERY_BRIDGE_ALL)
    parser.add_argument("--query-bridge-labels", default="TS->CS,CS->LB,RB->AS")
    parser.add_argument("--run-rrt-grower", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rrt-grower-extra-boxes", type=int, default=1)
    parser.add_argument("--rrt-grower-timeout-ms", type=float, default=1.0)
    parser.add_argument("--priority-prune-radius", type=float, default=0.0)
    parser.add_argument("--collision-overlap-prune-min-depth", type=int, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH)
    parser.add_argument("--collision-overlap-prune-threshold", type=float, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD)
    parser.add_argument("--collision-overlap-prune-ratio-threshold", type=float, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    seeds = parse_csv_ints(args.seeds)
    budgets = parse_csv_ints(args.box_budgets)
    depth_configs = parse_depth_configs(args.depth_configs)
    deep_ffb_depths = parse_csv_ints(args.deep_ffb_depths)
    ffb_start_depths = parse_csv_ints(args.ffb_start_depths)
    tasks: list[dict[str, int | str]] = []
    for leaf_start, leaf_max in depth_configs:
        for deep_ffb in deep_ffb_depths:
            for ffb_start in ffb_start_depths:
                for budget in budgets:
                    for seed in seeds:
                        config_id = f"ls{leaf_start}_lm{leaf_max}_dffb{deep_ffb}_sffb{ffb_start}_b{budget}"
                        tasks.append({
                            "config_id": config_id,
                            "leaf_start": leaf_start,
                            "leaf_max": leaf_max,
                            "deep_ffb": deep_ffb,
                            "ffb_start": ffb_start,
                            "budget": budget,
                            "seed": seed,
                        })
    run_rows: list[dict[str, Any]] = []
    raw_rows: list[dict[str, Any]] = []
    for task in progress(tasks, desc="exp04 depth scan", total=len(tasks)):
        print(
            "[depth-scan] "
            f"{task['config_id']} seed={task['seed']} boxes={task['budget']}",
            flush=True,
        )
        run_args = make_args(
            args,
            int(task["leaf_start"]),
            int(task["leaf_max"]),
            int(task["deep_ffb"]),
            int(task["ffb_start"]),
        )
        run_args.active_cache_tag = str(task["config_id"])
        row = run_case(str(args.case), int(task["seed"]), int(task["budget"]), run_args)
        flat = flatten_run(row, str(task["config_id"]))
        flat.update({
            "leaf_start_depth": int(task["leaf_start"]),
            "leaf_max_depth": int(task["leaf_max"]),
            "deep_ffb_depth": int(task["deep_ffb"]),
            "ffb_start_depth": int(task["ffb_start"]),
        })
        run_rows.append(flat)
        raw_rows.append(row)
        write_csv(args.out_dir / "depth_scan_runs.csv", run_rows)
        write_json(args.out_dir / "depth_scan_runs_raw.json", raw_rows)
    summary_rows = aggregate(run_rows)
    write_csv(args.out_dir / "depth_scan_summary.csv", summary_rows)
    write_json(args.out_dir / "depth_scan_summary.json", summary_rows)
    write_json(
        args.out_dir / "depth_scan_manifest.json",
        {
            "experiment": "exp04_full_root_depth_scan",
            "run_id": run_id("exp04_depth"),
            "environment": environment_metadata(),
            "case": str(args.case),
            "cache_root": str(args.rbf_cache_root),
            "warm_cache_label": str(args.warm_cache_label),
            "audit_segment_step": float(args.audit_segment_step),
            "audit_collision_tolerance": float(args.audit_collision_tolerance),
            "final_rrt_simplify_timeout_ms": float(args.final_rrt_simplify_timeout_ms),
            "priority_prune_radius": float(args.priority_prune_radius),
            "collision_overlap_prune_min_depth": int(args.collision_overlap_prune_min_depth),
            "collision_overlap_prune_threshold": float(args.collision_overlap_prune_threshold),
            "collision_overlap_prune_ratio_threshold": float(args.collision_overlap_prune_ratio_threshold),
            "tasks": tasks,
            "summary": summary_rows,
        },
    )
    print(f"wrote {args.out_dir / 'depth_scan_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
