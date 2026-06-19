#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import write_json
from experiments.common.progress import progress
from experiments.common.rbf_defaults import (
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    robot_joint_limit_tuples,
)
from experiments.common.rbf_leaf_rrt import (
    RBFLeafRRTOptions,
    canonical_priority_points,
    configure_leaf_rrt,
    make_refine_config,
)
from experiments.common.sbf_import import import_sbf

sbf = import_sbf()


def dist(a: Iterable[float], b: Iterable[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def path_length(path: list[list[float]]) -> float:
    return sum(dist(a, b) for a, b in zip(path[:-1], path[1:]))


def densify(path: list[list[float]], step: float) -> list[list[float]]:
    if len(path) < 2:
        return list(path)
    out: list[list[float]] = []
    for a, b in zip(path[:-1], path[1:]):
        length = dist(a, b)
        n = max(1, int(math.ceil(length / max(step, 1e-12))))
        for i in range(n):
            u = i / n
            out.append([float(x) + u * (float(y) - float(x)) for x, y in zip(a, b)])
    out.append([float(x) for x in path[-1]])
    return out


def box_contains(box: list[list[float]], q: list[float], tol: float = 1e-9) -> bool:
    return all(float(lo) - tol <= float(v) <= float(hi) + tol for (lo, hi), v in zip(box, q))


def coverage(path: list[list[float]], boxes: list[list[list[float]]], step: float) -> tuple[float, int, int]:
    samples = densify(path, step)
    if not samples:
        return 1.0, 0, 0
    covered = sum(1 for q in samples if any(box_contains(box, q) for box in boxes))
    return covered / len(samples), len(samples) - covered, len(samples)


def make_options(args: argparse.Namespace, *, connector_bridge_boxes: int, query_bridge_all: bool) -> RBFLeafRRTOptions:
    robot = sbf.load_iiwa14_robot()
    coverage = robot_joint_limit_tuples(robot)
    return RBFLeafRRTOptions(
        seed=int(args.seed),
        deep_max_boxes=int(args.box_budget),
        rbf_max_depth=int(args.rbf_max_depth),
        timeout_ms=float(args.timeout_ms),
        threads=int(args.threads),
        leaf_start_depth=int(args.leaf_start_depth),
        leaf_max_depth=int(args.leaf_max_depth),
        deep_ffb_depth=int(args.deep_ffb_depth),
        ffb_start_depth=int(args.ffb_start_depth),
        validation_batch_size=int(args.validation_batch_size),
        audit_segment_step=float(args.audit_segment_step),
        audit_collision_tolerance=0.0,
        query_shortcut_boxes=False,
        use_virtual_topology=True,
        parallel_virtual_validation=True,
        leaf_threads=int(args.threads),
        envelope="support_hull",
        endpoint_source="ifk",
        use_external_evidence=bool(args.use_external_evidence),
        external_evidence_path=Path(args.rbf_cache_root) / str(args.warm_cache_label),
        external_evidence_verify_identity=False,
        root_override_tuples=None,
        coverage_override_tuples=coverage,
        database_canonical_mode=True,
        symmetry_aligned_cache_schedule=False,
        connector_bridge_boxes=int(connector_bridge_boxes),
        connector_pair_timeout_ms=float(args.connector_pair_timeout_ms),
        connector_max_pairs_per_gap=int(args.connector_max_pairs_per_gap),
        connector_rrt_iters=50000,
        connector_rrt_timeout_ms=2000.0,
        connector_rrt_step_size=0.5,
        connector_rrt_goal_bias=0.2,
        connector_segment_resolution=16,
        connector_pave_max_chain=int(args.connector_pave_max_chain),
        connector_pave_depth=int(args.connector_pave_depth),
        connector_pave_fill_gaps=True,
        connector_pave_require_connected_chain=True,
        final_collision_shortcut=True,
        final_rrt_simplify=bool(args.final_rrt_simplify),
        final_rrt_simplify_timeout_ms=50.0,
        final_rrt_simplify_max_iters=50000,
        final_rrt_simplify_attempts=4,
        query_bridge_all=bool(query_bridge_all),
        collision_overlap_prune_min_depth=int(args.collision_overlap_prune_min_depth),
        collision_overlap_prune_threshold=float(args.collision_overlap_prune_threshold),
        collision_overlap_prune_ratio_threshold=float(args.collision_overlap_prune_ratio_threshold),
    )


def build_forest(
    args: argparse.Namespace,
    database_path: Path,
    *,
    connector_bridge_boxes: int,
    query_bridge_all: bool,
    segment_edges_enabled: bool = True,
) -> tuple[Any, Any, list[Any], list[Any], Any]:
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    options = make_options(args, connector_bridge_boxes=connector_bridge_boxes, query_bridge_all=query_bridge_all)
    cfg = configure_leaf_rrt(robot, database_path, options)
    cfg.connector.segment_edges_enabled = bool(segment_edges_enabled)
    cfg.connector.rrt_segment_edges = bool(segment_edges_enabled)
    cfg.connector.point_gap_segment_edges = bool(segment_edges_enabled)
    forest = sbf.SafeBoxForest(robot, cfg)
    build = forest.build_leaf_sweep_refined(
        obstacles,
        make_refine_config(options),
        canonical_priority_points(robot, queries, canonicalize=False),
    )
    return forest, robot, obstacles, queries, build


def ts_cs_query(queries: list[Any]) -> Any:
    for query in queries:
        if str(query.label) == "TS->CS":
            return query
    raise RuntimeError("TS->CS query not found")


def make_reference_path(args: argparse.Namespace) -> dict[str, Any]:
    forest, _robot, _obstacles, queries, build = build_forest(
        args,
        args.out_dir / "reference_cache",
        connector_bridge_boxes=0,
        query_bridge_all=False,
        segment_edges_enabled=True,
    )
    query = ts_cs_query(queries)
    t0 = time.perf_counter()
    added = 0
    if hasattr(forest, "bridge_queries"):
        starts = [[float(v) for v in item.start] for item in queries]
        goals = [[float(v) for v in item.goal] for item in queries]
        added_values = [int(value) for value in forest.bridge_queries(starts, goals)]
        added = sum(added_values)
    if added <= 0:
        added = int(forest.bridge_query([float(v) for v in query.start], [float(v) for v in query.goal]))
    bridge_s = time.perf_counter() - t0
    result = forest.query([float(v) for v in query.start], [float(v) for v in query.goal])
    waypoints = [[float(v) for v in point] for point in result.path_as_lists()]
    if not bool(result.success) or not waypoints:
        raise RuntimeError(
            f"failed to obtain TS->CS reference path: "
            f"bridge_added={added}, success={bool(result.success)}, "
            f"audit={bool(result.audit_passed)}"
        )
    return {
        "start": [float(v) for v in query.start],
        "goal": [float(v) for v in query.goal],
        "bridge_added": added,
        "bridge_s": bridge_s,
        "build_ms": float(build.total_ms),
        "path": waypoints,
        "path_length": path_length(waypoints),
        "segment_edges_used": int(result.segment_edges_used),
        "segment_edge_length": float(result.segment_edge_length),
        "audit_passed": bool(result.audit_passed),
    }


def run_one(args: argparse.Namespace, ref: dict[str, Any], max_depth: int, max_chain: int, ffb_calls: int, step: float, require_connected: bool) -> dict[str, Any]:
    forest, _robot, _obstacles, _queries, build = build_forest(
        args,
        args.out_dir / "sweep_cache" / f"d{max_depth}_c{max_chain}_f{ffb_calls}_s{str(step).replace('.', 'p')}_r{int(require_connected)}",
        connector_bridge_boxes=0,
        query_bridge_all=False,
        segment_edges_enabled=False,
    )
    t0 = time.perf_counter()
    if str(args.cover_mode) == "direct-ffb":
        ffb_options = sbf.FindFreeBoxOptions()
        ffb_options.max_depth = int(max_depth)
        ffb_options.start_depth = int(args.ffb_start_depth)
        ffb_options.skip_to_depth = int(args.ffb_start_depth)
        if hasattr(sbf, "FindFreeBoxSearchMode"):
            mode = str(args.ffb_search_mode).strip().lower().replace("-", "_")
            if mode in {"linear", "incremental"}:
                ffb_options.search_mode = sbf.FindFreeBoxSearchMode.LinearDescent
            elif mode in {"binary", "binary_depth"}:
                ffb_options.search_mode = sbf.FindFreeBoxSearchMode.BinaryDepth
            else:
                raise ValueError(f"unsupported --ffb-search-mode: {args.ffb_search_mode}")
        ffb_options.split_reserved_leaf = True
        ffb_options.split_unknown_leaf = True
        ffb_options.reject_seed_collision = False
        debug = dict(forest.debug_cover_path_with_ffb(
            ref["path"],
            list(sbf.make_combined_obstacles()),
            ffb_options,
            float(step),
            int(ffb_calls),
            1e-9,
            bool(args.include_existing_boxes),
            bool(args.cover_disable_caches),
            64,
            [float(v) for v in str(args.cover_refine_steps).split(",") if v.strip()],
            int(args.cover_parallel_workers),
            bool(args.repair_corridor_adjacency),
            int(args.repair_rounds),
            int(args.repair_segment_subdivisions),
        ))
    else:
        debug = dict(forest.debug_chain_pave_waypoints(
            ref["path"],
            max_chain=int(max_chain),
            max_depth=int(max_depth),
            max_gap_fill_steps=8,
            fill_segment_gaps=True,
            gap_fill_min_step=1e-4,
            adjacency_tolerance=1e-9,
            gap_fill_sample_step=float(step),
            gap_fill_time_budget_ms=float(args.pave_timeout_ms),
            gap_fill_max_ffb_calls=int(ffb_calls),
            gap_fill_min_arc_gain=0.0,
            require_connected_chain=bool(require_connected),
            commit_certified_only=bool(args.commit_certified_only),
        ))
    pave_s = time.perf_counter() - t0
    all_boxes = list(debug.get("boxes", [])) if str(args.cover_mode) == "direct-ffb" else list(debug.get("all_boxes", []))
    new_boxes = all_boxes[int(debug.get("initial_box_count", 0)):] if str(args.cover_mode) == "direct-ffb" else list(debug.get("committed_boxes", []))
    cov_all, uncovered_all, samples = coverage(ref["path"], all_boxes, float(args.coverage_step))
    cov_new, uncovered_new, _ = coverage(ref["path"], new_boxes, float(args.coverage_step))
    query = forest.query(ref["start"], ref["goal"])
    raw = float(getattr(query, "raw_path_length", query.path_length)) if bool(query.success) else math.nan
    seg_len = float(query.segment_edge_length) if bool(query.success) else 0.0
    counters = dict(debug.get("counters", {})) if str(args.cover_mode) == "direct-ffb" else {}
    return {
        "cover_mode": str(args.cover_mode),
        "max_depth": int(max_depth),
        "max_chain": int(max_chain),
        "ffb_calls": int(ffb_calls),
        "sample_step": float(step),
        "require_connected_chain": bool(require_connected),
        "build_s": float(build.total_ms) / 1000.0,
        "pave_s": pave_s,
        "debug_cover_total_ms": float(debug.get("total_ms", 0.0)),
        "total_s": float(build.total_ms) / 1000.0 + pave_s,
        "use_external_evidence": bool(args.use_external_evidence),
        "cover_disable_caches": bool(args.cover_disable_caches),
        "ffb_search_mode": str(args.ffb_search_mode),
        "added": int(debug.get("added", debug.get("added_box_count", 0))),
        "fast_gap_fill_ffb_calls": int(debug.get("fast_gap_fill_ffb_calls", 0)),
        "fast_gap_fill_ms": float(debug.get("fast_gap_fill_ms", 0.0)),
        "boundary_ffb_calls": int(debug.get("boundary_ffb_calls", debug.get("ffb_calls", 0))),
        "boundary_commits": int(debug.get("boundary_commits", debug.get("ffb_found", 0))),
        "boundary_reject_not_free": int(debug.get("boundary_reject_not_free", 0)),
        "boundary_reject_non_adjacent": int(debug.get("boundary_reject_non_adjacent", 0)),
        "boundary_stall": int(debug.get("boundary_stall", 0)),
        "boundary_target_hits": int(debug.get("boundary_target_hits", 0)),
        "oracle_external_exact_hits": int(counters.get("materialization_external_exact_hits", 0)),
        "oracle_external_exact_misses": int(counters.get("materialization_external_exact_misses", 0)),
        "oracle_canonical_frame_invalid": int(counters.get("canonical_frame_invalid", 0)),
        "oracle_canonical_reflected_seed_misses": int(counters.get("canonical_reflected_seed_misses", 0)),
        "direct_cover_uncovered_samples": int(debug.get("uncovered_samples", -1)),
        "direct_cover_sample_count": int(debug.get("sample_count", -1)),
        "direct_cover_fail_counts": list(debug.get("fail_counts", [])),
        "direct_cover_first_failures": list(debug.get("failures", []))[:8],
        "direct_cover_pass_summaries": list(debug.get("pass_summaries", [])),
        "direct_cover_final_sample_step": float(debug.get("final_sample_step", step)),
        "direct_cover_presplit_ms": float(debug.get("presplit_ms", 0.0)),
        "direct_cover_parallel_workers": int(debug.get("parallel_workers", 1)),
        "repair_corridor_adjacency": bool(debug.get("repair_corridor_adjacency", False)),
        "repair_ms": float(debug.get("repair_ms", 0.0)),
        "repair_calls": int(debug.get("repair_calls", 0)),
        "repair_added": int(debug.get("repair_added", 0)),
        "repair_bad_transitions_initial": int(debug.get("repair_bad_transitions_initial", 0)),
        "repair_bad_transitions_final": int(debug.get("repair_bad_transitions_final", 0)),
        "repair_bad_transition_length_initial": float(debug.get("repair_bad_transition_length_initial", 0.0)),
        "repair_bad_transition_length_final": float(debug.get("repair_bad_transition_length_final", 0.0)),
        "repair_bad_transition_fraction_final": float(debug.get("repair_bad_transition_fraction_final", math.nan)),
        "coverage_all": cov_all,
        "coverage_new": cov_new,
        "uncovered_all": int(uncovered_all),
        "uncovered_new": int(uncovered_new),
        "samples": int(samples),
        "debug_audit_passed": bool(debug.get("audit_passed", False)),
        "query_success": bool(query.success),
        "query_audit_passed": bool(query.audit_passed),
        "query_path_length": float(query.path_length) if bool(query.success) else math.nan,
        "query_raw_path_length": raw,
        "query_segment_edges_used": int(query.segment_edges_used),
        "query_segment_fraction": (seg_len / raw) if raw > 1e-12 else math.nan,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/exp04_ts_cs_box_cover_study"))
    parser.add_argument("--reference-json", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--box-budget", type=int, default=200)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--leaf-start-depth", type=int, default=14)
    parser.add_argument("--leaf-max-depth", type=int, default=18)
    parser.add_argument("--deep-ffb-depth", type=int, default=40)
    parser.add_argument("--ffb-start-depth", type=int, default=23)
    parser.add_argument("--ffb-search-mode", choices=["binary", "linear"], default="binary")
    parser.add_argument("--validation-batch-size", type=int, default=512)
    parser.add_argument("--audit-segment-step", type=float, default=0.01)
    parser.add_argument("--connector-pair-timeout-ms", type=float, default=10.0)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=4)
    parser.add_argument("--connector-pave-max-chain", type=int, default=80)
    parser.add_argument("--connector-pave-depth", type=int, default=80)
    parser.add_argument("--collision-overlap-prune-min-depth", type=int, default=14)
    parser.add_argument("--collision-overlap-prune-threshold", type=float, default=0.05)
    parser.add_argument("--collision-overlap-prune-ratio-threshold", type=float, default=0.0)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    parser.add_argument("--use-external-evidence", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--depths", default="40,56,72,96")
    parser.add_argument("--chains", default="40,80,160")
    parser.add_argument("--ffb-calls", default="64,128,256")
    parser.add_argument("--sample-steps", default="0.04,0.02")
    parser.add_argument("--require-connected", default="0,1")
    parser.add_argument("--cover-mode", choices=["direct-ffb", "chain-pave"], default="direct-ffb")
    parser.add_argument("--include-existing-boxes", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--cover-disable-caches", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--cover-refine-steps",
        default="",
        help="Optional comma-separated coarse-to-fine FFB seed steps; final coverage uses the finest step.",
    )
    parser.add_argument("--cover-parallel-workers", type=int, default=1)
    parser.add_argument("--repair-corridor-adjacency", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--repair-rounds", type=int, default=2)
    parser.add_argument("--repair-segment-subdivisions", type=int, default=8)
    parser.add_argument("--coverage-step", type=float, default=0.01)
    parser.add_argument("--pave-timeout-ms", type=float, default=250.0)
    parser.add_argument("--commit-certified-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--final-rrt-simplify", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    if args.reference_json is not None and args.reference_json.exists():
        ref = json.loads(args.reference_json.read_text(encoding="utf-8"))
    else:
        ref = make_reference_path(args)
    depths = [int(v) for v in str(args.depths).split(",") if v.strip()]
    chains = [int(v) for v in str(args.chains).split(",") if v.strip()]
    ffb_calls = [int(v) for v in str(args.ffb_calls).split(",") if v.strip()]
    steps = [float(v) for v in str(args.sample_steps).split(",") if v.strip()]
    requires = [bool(int(v)) for v in str(args.require_connected).split(",") if v.strip()]
    tasks = [
        (depth, chain, calls, step, req)
        for depth in depths
        for chain in chains
        for calls in ffb_calls
        for step in steps
        for req in requires
    ]
    rows = []
    for depth, chain, calls, step, req in progress(tasks, desc="ts-cs box cover", total=len(tasks)):
        rows.append(run_one(args, ref, depth, chain, calls, step, req))

    write_json(args.out_dir / "ts_cs_reference_path.json", ref)
    write_json(args.out_dir / "ts_cs_box_cover_manifest.json", {
        "state_space": "native_joint_space",
        "cache_scope": "full_robot_joint_limits",
        "canonical_mapping_scope": "LECT_internal_only",
        "external_evidence_path": str(Path(args.rbf_cache_root) / str(args.warm_cache_label)),
        "reference": ref,
        "rows": rows,
    })
    if rows:
        with (args.out_dir / "ts_cs_box_cover_summary.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    best = sorted(
        rows,
        key=lambda r: (
            not (r["query_audit_passed"] and r["query_segment_edges_used"] == 0),
            -float(r["coverage_all"]),
            float(r["total_s"]),
            float(r["query_path_length"]) if math.isfinite(float(r["query_path_length"])) else 1e9,
        ),
    )[:10]
    print(json.dumps({"reference": ref, "best": best}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
