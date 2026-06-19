#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.progress import progress
from experiments.common.rbf_defaults import (
    CRITSAMPLE_D23_CACHE_LABEL,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_RESOLUTION,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD,
    DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION,
    DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES,
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
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
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_FINAL_COLLISION_SHORTCUT,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_QUERY_BRIDGE_ALL,
    DEFAULT_RBF_QUERY_BRIDGE_LABELS,
    DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
    DEFAULT_RBF_REFINE_TIMEOUT_MS,
    DEFAULT_RBF_THREADS,
    DEFAULT_RBF_VALIDATION_BATCH_SIZE,
    D23_CACHE_ROOT,
    D23_CACHE_LABEL,
)
from experiments.exp04_shelf_leaf_rrt.run_shelf_leaf_rrt import run_case


AAFK_D23_CACHE_LABEL = D23_CACHE_LABEL


def parse_ints(text: str) -> list[int]:
    return [int(item) for item in str(text).split(",") if item.strip()]


def finite_mean(values: list[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.fmean(clean) if clean else math.nan


def finite_median(values: list[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.median(clean) if clean else math.nan


def make_args(base: argparse.Namespace, out_dir: Path, ffb_depth: int, leaf_max_depth: int) -> SimpleNamespace:
    return SimpleNamespace(
        out_dir=out_dir,
        threads=int(base.threads),
        timeout_ms=float(base.timeout_ms),
        rbf_max_depth=int(ffb_depth),
        leaf_start_depth=int(base.leaf_start_depth),
        leaf_max_depth=int(leaf_max_depth),
        deep_ffb_depth=int(ffb_depth),
        refine_timeout_ms=float(base.refine_timeout_ms),
        domain_seed_cap=int(base.domain_seed_cap),
        domain_success_cap=int(base.domain_success_cap),
        domain_attempt_cap=int(base.domain_attempt_cap),
        validation_batch_size=int(base.validation_batch_size),
        ffb_start_depth=int(base.ffb_start_depth),
        ffb_search_mode=str(base.ffb_search_mode),
        audit_resolution=int(base.audit_resolution),
        audit_segment_step=float(base.audit_segment_step),
        audit_collision_tolerance=float(base.audit_collision_tolerance),
        query_shortcut_boxes=bool(base.query_shortcut_boxes),
        segment_edges_fallback_only=bool(base.segment_edges_fallback_only),
        connector_birrt=bool(base.connector_birrt),
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
        connector_pave_depth=int(ffb_depth),
        connector_adaptive_min_segment_fraction=float(base.connector_adaptive_min_segment_fraction),
        query_bridge_pave_depth=int(ffb_depth),
        connector_pave_fill_gaps=bool(base.connector_pave_fill_gaps),
        connector_pave_require_connected_chain=bool(base.connector_pave_require_connected_chain),
        final_collision_shortcut=bool(base.final_collision_shortcut),
        final_rrt_simplify=bool(base.final_rrt_simplify),
        final_rrt_simplify_timeout_ms=float(base.final_rrt_simplify_timeout_ms),
        final_rrt_simplify_max_iters=int(base.final_rrt_simplify_max_iters),
        final_rrt_simplify_attempts=int(base.final_rrt_simplify_attempts),
        query_bridge_all=bool(base.query_bridge_all),
        query_bridge_labels=str(base.query_bridge_labels),
        query_bridge_force_indices=str(base.query_bridge_force_indices),
        query_bridge_forced_attempts=int(base.query_bridge_forced_attempts),
        query_bridge_direct_sample_step=float(base.query_bridge_direct_sample_step),
        query_bridge_direct_max_length=float(base.query_bridge_direct_max_length),
        priority_prune_radius=0.0,
        collision_overlap_prune_min_depth=int(base.collision_overlap_prune_min_depth),
        collision_overlap_prune_threshold=float(base.collision_overlap_prune_threshold),
        collision_overlap_prune_ratio_threshold=float(base.collision_overlap_prune_ratio_threshold),
        use_virtual_topology=bool(base.use_virtual_topology),
        parallel_virtual_validation=bool(base.parallel_virtual_validation),
        rbf_cache_root=Path(base.rbf_cache_root),
        warm_cache_label=str(base.warm_cache_label),
        active_cache_tag=f"critd23_l{leaf_max_depth}_d{ffb_depth}",
    )


def flatten_row(row: dict[str, Any], *, ffb_depth: int, leaf_max_depth: int) -> dict[str, Any]:
    out = {
        "case": row.get("case"),
        "seed": row.get("seed"),
        "deep_max_boxes": row.get("deep_max_boxes"),
        "leaf_max_depth": leaf_max_depth,
        "ffb_depth": ffb_depth,
        "status": row.get("status"),
        "success_count": row.get("success_count"),
        "query_count": row.get("query_count"),
        "planning_s": row.get("planning_s"),
        "build_s": row.get("build_s"),
        "query_s": row.get("query_s"),
        "audit_s": row.get("audit_s"),
        "leaf_sweep_s": row.get("leaf_sweep_s"),
        "deep_refine_s": row.get("deep_refine_s"),
        "connector_s": row.get("connector_s"),
        "query_bridge_s": row.get("query_bridge_s"),
        "path_length_mean": row.get("path_length_mean"),
        "raw_segment_fraction": row.get("raw_segment_fraction"),
        "final_boxes": row.get("final_boxes"),
        "adjacency_islands": row.get("adjacency_islands"),
        "external_hits": row.get("external_hits"),
        "segment_edges": row.get("segment_edges"),
    }
    for query in row.get("queries", []):
        label = str(query.get("label", "")).replace("->", "_").replace("-", "_")
        out[f"{label}_length"] = query.get("path_length")
        out[f"{label}_raw_length"] = query.get("raw_path_length")
        out[f"{label}_segment_fraction"] = query.get("segment_fraction")
        out[f"{label}_audit_passed"] = query.get("audit_passed")
    return out


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    keys = sorted({
        (int(row["leaf_max_depth"]), int(row["ffb_depth"]), int(row["deep_max_boxes"]))
        for row in rows
    })
    out: list[dict[str, Any]] = []
    for leaf_max_depth, ffb_depth, budget in keys:
        items = [
            row for row in rows
            if int(row["leaf_max_depth"]) == leaf_max_depth
            and int(row["ffb_depth"]) == ffb_depth
            and int(row["deep_max_boxes"]) == budget
        ]
        query_count = int(items[0].get("query_count") or 0) if items else 0
        full_success = [
            row for row in items
            if int(row.get("success_count") or 0) == query_count
        ]
        all_audit = []
        for row in full_success:
            ok = True
            for key, value in row.items():
                if key.endswith("_audit_passed") and not bool(value):
                    ok = False
                    break
            all_audit.append(row if ok else None)
        audit_success_count = sum(1 for item in all_audit if item is not None)
        out.append({
            "leaf_max_depth": leaf_max_depth,
            "ffb_depth": ffb_depth,
            "deep_max_boxes": budget,
            "runs": len(items),
            "success_runs": len(full_success),
            "audit_success_runs": audit_success_count,
            "planning_s_median": finite_median([row["planning_s"] for row in full_success]),
            "build_s_median": finite_median([row["build_s"] for row in full_success]),
            "connector_s_median": finite_median([row["connector_s"] for row in full_success]),
            "query_bridge_s_median": finite_median([row["query_bridge_s"] for row in full_success]),
            "path_length_mean": finite_mean([row["path_length_mean"] for row in full_success]),
            "raw_segment_fraction_median": finite_median([row["raw_segment_fraction"] for row in full_success]),
            "raw_segment_fraction_max": max([float(row["raw_segment_fraction"]) for row in full_success], default=math.nan),
            "final_boxes_median": finite_median([row["final_boxes"] for row in full_success]),
            "adjacency_islands_median": finite_median([row["adjacency_islands"] for row in full_success]),
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row.keys()})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan Exp4 d23 cache depth/budget configurations.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp04_critsample_d23_scan")
    parser.add_argument("--case", choices=["critsample_d23_cache", "baseline_d23_aafk_support_hull_8t"], default="critsample_d23_cache")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--box-budgets", default="100,200,400,800")
    parser.add_argument("--ffb-depths", default="32,36,40,44,48")
    parser.add_argument("--leaf-max-depths", default=str(DEFAULT_RBF_LEAF_MAX_DEPTH))
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--leaf-start-depth", type=int, default=DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--refine-timeout-ms", type=float, default=DEFAULT_RBF_REFINE_TIMEOUT_MS)
    parser.add_argument("--domain-seed-cap", type=int, default=DEFAULT_RBF_DOMAIN_SEED_CAP)
    parser.add_argument("--domain-success-cap", type=int, default=DEFAULT_RBF_DOMAIN_SUCCESS_CAP)
    parser.add_argument("--domain-attempt-cap", type=int, default=DEFAULT_RBF_DOMAIN_ATTEMPT_CAP)
    parser.add_argument("--validation-batch-size", type=int, default=DEFAULT_RBF_VALIDATION_BATCH_SIZE)
    parser.add_argument("--ffb-start-depth", type=int, default=DEFAULT_RBF_FFB_START_DEPTH)
    parser.add_argument("--ffb-search-mode", default=DEFAULT_RBF_FFB_SEARCH_MODE)
    parser.add_argument("--audit-resolution", type=int, default=DEFAULT_RBF_AUDIT_RESOLUTION)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--query-shortcut-boxes", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--segment-edges-fallback-only", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-birrt", action=argparse.BooleanOptionalAction, default=True)
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
    parser.add_argument("--connector-adaptive-min-segment-fraction", type=float, default=DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION)
    parser.add_argument("--connector-pave-fill-gaps", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS)
    parser.add_argument("--connector-pave-require-connected-chain", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN)
    parser.add_argument("--final-collision-shortcut", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_COLLISION_SHORTCUT)
    parser.add_argument("--final-rrt-simplify", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY)
    parser.add_argument("--final-rrt-simplify-timeout-ms", type=float, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS)
    parser.add_argument("--final-rrt-simplify-max-iters", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS)
    parser.add_argument("--final-rrt-simplify-attempts", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS)
    parser.add_argument("--query-bridge-all", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_QUERY_BRIDGE_ALL)
    parser.add_argument("--query-bridge-labels", default=DEFAULT_RBF_QUERY_BRIDGE_LABELS)
    parser.add_argument("--query-bridge-force-indices", default="")
    parser.add_argument("--query-bridge-forced-attempts", type=int, default=1)
    parser.add_argument("--query-bridge-direct-sample-step", type=float, default=0.0)
    parser.add_argument("--query-bridge-direct-max-length", type=float, default=6.5)
    parser.add_argument("--collision-overlap-prune-min-depth", type=int, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH)
    parser.add_argument("--collision-overlap-prune-threshold", type=float, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD)
    parser.add_argument("--collision-overlap-prune-ratio-threshold", type=float, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD)
    parser.add_argument("--use-virtual-topology", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not str(args.warm_cache_label):
        args.warm_cache_label = (
            CRITSAMPLE_D23_CACHE_LABEL
            if str(args.case) == "critsample_d23_cache"
            else AAFK_D23_CACHE_LABEL
        )
    seeds = parse_ints(args.seeds)
    budgets = parse_ints(args.box_budgets)
    ffb_depths = parse_ints(args.ffb_depths)
    leaf_depths = parse_ints(args.leaf_max_depths)
    planned = [
        (leaf_depth, ffb_depth, budget, seed)
        for leaf_depth in leaf_depths
        for ffb_depth in ffb_depths
        for budget in budgets
        for seed in seeds
    ]
    run_rows: list[dict[str, Any]] = []
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for leaf_depth, ffb_depth, budget, seed in progress(planned, desc="crit d23 scan", total=len(planned)):
        print(f"[{args.case}] leaf={leaf_depth} ffb={ffb_depth} boxes={budget} seed={seed}", flush=True)
        run_args = make_args(args, args.out_dir, ffb_depth, leaf_depth)
        row = run_case(str(args.case), seed, budget, run_args)
        run_rows.append(flatten_row(row, ffb_depth=ffb_depth, leaf_max_depth=leaf_depth))
    summary_rows = aggregate(run_rows)
    payload = {
        "experiment": "exp04_d23_config_scan",
        "run_id": run_id("exp04_d23_scan"),
        "environment": environment_metadata(),
        "config": jsonable(vars(args)) | {
            "warm_cache_label": str(args.warm_cache_label),
            "case": str(args.case),
            "endpoint_source": "critsample" if str(args.case) == "critsample_d23_cache" else "ifk_aafk",
            "cache_depth_semantics": "canonical_lect_tree",
            "planner_state_space": "native_joint_space",
            "scan_axis": "ffb_depth maps to rbf_max_depth/deep_ffb_depth/connector_pave_depth/query_bridge_pave_depth",
        },
        "rows": run_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "d23_config_scan_manifest.json", payload)
    write_csv(args.out_dir / "d23_config_scan_rows.csv", run_rows)
    write_csv(args.out_dir / "d23_config_scan_summary.csv", summary_rows)
    # Backward-compatible filenames for existing CritSample analysis snippets.
    write_json(args.out_dir / "critsample_d23_config_scan_manifest.json", payload)
    write_csv(args.out_dir / "critsample_d23_config_scan_rows.csv", run_rows)
    write_csv(args.out_dir / "critsample_d23_config_scan_summary.csv", summary_rows)
    print(f"wrote {args.out_dir / 'd23_config_scan_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
