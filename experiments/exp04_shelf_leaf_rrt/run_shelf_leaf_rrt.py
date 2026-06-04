#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median, tex_num
from experiments.common.progress import progress
from experiments.common.rbf_leaf_rrt import RBFLeafRRTOptions, run_leaf_rrt
from experiments.common.rbf_defaults import (
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    DEFAULT_RBF_AUDIT_RESOLUTION,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES,
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_PAVE_DEPTH,
    DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN,
    DEFAULT_RBF_CONNECTOR_PAVE_STEPS,
    DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS,
    DEFAULT_RBF_CONNECTOR_RRT_ITERS,
    DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE,
    DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION,
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_DEEP_MAX_BOXES,
    DEFAULT_RBF_DOMAIN_ATTEMPT_CAP,
    DEFAULT_RBF_DOMAIN_SEED_CAP,
    DEFAULT_RBF_DOMAIN_SUCCESS_CAP,
    DEFAULT_RBF_FINAL_COLLISION_SHORTCUT,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_REFINE_TIMEOUT_MS,
    DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES,
    DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS,
    DEFAULT_RBF_THREADS,
    DEFAULT_RBF_VALIDATION_BATCH_SIZE,
    default_rbf_profile,
    rbf_budget_grid,
    root_override_intervals,
    shelf_d23_rbf_profile,
)
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


ABLATIONS = [
    "baseline_d23_aafk_support_hull_8t",
    "critsample_support_hull_unsafe",
    "no_external_lect",
    "link_aabb",
    "single_thread",
]

CASE_LABELS = {
    "baseline_d23_aafk_support_hull_8t": "RBF-SH d23",
    "critsample_support_hull_unsafe": "CritSample unsafe",
    "no_external_lect": "No LECT replay",
    "link_aabb": "Link AABB",
    "single_thread": "No LECT, 1 thread",
}


def parse_csv_ints(raw: str) -> list[int]:
    return [int(item.strip()) for item in str(raw).split(",") if item.strip()]


def configure_case(case: str, seed: int, deep_max_boxes: int, args: argparse.Namespace) -> tuple[Any, Any]:
    cfg = sbf.SBFConfig()
    cfg.enable_connector = True
    cfg.endpoint_source.source = sbf.EndpointSource.CritSample if case == "critsample_support_hull_unsafe" else sbf.EndpointSource.IFK
    cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB if case == "link_aabb" else sbf.EnvelopeType.SupportHull
    if case == "critsample_support_hull_unsafe":
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
    else:
        cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
        cfg.validation.accept_unsafe_free = False
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    cfg.database.path = str(args.out_dir / "active_cache" / f"{case}_seed{seed}_box{deep_max_boxes}")
    cfg.database.create_if_missing = True
    cfg.database.max_tree_depth = int(args.rbf_max_depth)
    cfg.database.canonical_mode = True
    cfg.database.symmetry_descriptor = "joint_symmetry_native_v1"
    cfg.database.root_intervals_override = root_override_intervals(sbf)
    split_policy = sbf.SplitPolicyDescriptor()
    split_policy.strategy = sbf.SplitStrategy.RoundRobin if case == "round_robin_split" else sbf.SplitStrategy.AAFKVolumeMin
    cfg.database.split_policy = split_policy
    cfg.validation.enable_endpoint_evidence_cache = False
    cfg.validation.store_endpoint_evidence_cache = False
    cfg.validation.enable_worker_shared_endpoint_cache = False
    cfg.validation.external_evidence_backfill_active = False
    cfg.validation.external_evidence_materialization = True
    cfg.validation.external_evidence_scoring = True
    threads = 1 if case == "single_thread" else int(args.threads)
    cfg.runtime.mode = sbf.ExecutionMode.Parallel if threads > 1 else sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = threads
    cfg.runtime.batch_size = threads
    cfg.runtime.parallel_threshold = 1
    cfg.grower.n_threads = threads
    cfg.grower.task_batch_size = threads
    cfg.grower.parallel_threshold = 1
    cfg.grower.rng_seed = int(seed)
    cfg.grower.mode = sbf.GrowerMode.RRT
    cfg.grower.max_boxes = max(1, int(deep_max_boxes))
    cfg.grower.timeout_ms = float(args.timeout_ms)
    cfg.grower.sample_categorical_allocation = True
    cfg.grower.intertree_goal_bias = 0.25
    cfg.grower.unexplored_sample_prob = 0.20
    cfg.grower.rrt_goal_bias = 0.15
    cfg.grower.anchor_target_prob = 0.025
    cfg.grower.sample_uniform_prob = 0.375
    cfg.grower.component_connect_prob = 0.0
    cfg.connector.n_threads = threads
    cfg.connector.parallel_threshold = 1
    cfg.connector.pair_batch_size = max(1, int(args.connector_pair_batch_size))
    cfg.connector.segment_edges_enabled = True
    cfg.connector.rrt_segment_edges = True
    cfg.connector.point_gap_segment_edges = True
    cfg.connector.segment_edges_fallback_only = bool(args.segment_edges_fallback_only)
    cfg.connector.enable_birrt = bool(args.connector_birrt)
    cfg.connector.max_pairs_per_gap = int(args.connector_max_pairs_per_gap)
    cfg.connector.per_pair_timeout_ms = float(args.connector_pair_timeout_ms)
    cfg.connector.max_total_bridge_boxes = int(args.connector_bridge_boxes)
    cfg.connector.rrt.max_iters = int(args.connector_rrt_iters)
    cfg.connector.rrt.timeout_ms = float(args.connector_rrt_timeout_ms)
    cfg.connector.rrt.step_size = float(args.connector_rrt_step_size)
    cfg.connector.rrt.goal_bias = float(args.connector_rrt_goal_bias)
    cfg.connector.rrt.segment_resolution = int(args.connector_segment_resolution)
    if hasattr(cfg.connector.rrt, "segment_step"):
        cfg.connector.rrt.segment_step = float(args.audit_segment_step)
    if hasattr(cfg.connector, "point_validated_gap_step"):
        cfg.connector.point_validated_gap_step = float(args.audit_segment_step)
    cfg.connector.pave.max_chain = int(args.connector_pave_max_chain)
    cfg.connector.pave.max_steps_per_waypoint = int(args.connector_pave_steps)
    cfg.connector.pave.find_free_box.max_depth = int(args.connector_pave_depth)
    cfg.connector.pave.find_free_box.skip_to_depth = int(args.ffb_start_depth)
    cfg.connector.pave.find_free_box.split_reserved_leaf = True
    cfg.connector.pave.find_free_box.split_unknown_leaf = True
    cfg.connector.pave.find_free_box.reject_seed_collision = False
    cfg.connector.pave.fill_gaps = bool(args.connector_pave_fill_gaps)
    cfg.connector.pave.require_connected_chain = bool(args.connector_pave_require_connected_chain)
    cfg.query.strict_path_audit = True
    cfg.query.audit_resolution = max(int(args.audit_resolution), int(args.connector_segment_resolution))
    cfg.query.audit_segment_step = float(args.audit_segment_step)
    cfg.query.shortcut_boxes = False
    cfg.query.collision_shortcut = bool(args.final_collision_shortcut)
    if hasattr(cfg.query, "final_rrt_simplify"):
        cfg.query.final_rrt_simplify = bool(args.final_rrt_simplify)
    if hasattr(cfg.query, "final_rrt_simplify_timeout_ms"):
        cfg.query.final_rrt_simplify_timeout_ms = float(args.final_rrt_simplify_timeout_ms)
    if hasattr(cfg.query, "final_rrt_simplify_max_iters"):
        cfg.query.final_rrt_simplify_max_iters = int(args.final_rrt_simplify_max_iters)
    if hasattr(cfg.query, "final_rrt_simplify_attempts"):
        cfg.query.final_rrt_simplify_attempts = int(args.final_rrt_simplify_attempts)
    if case.startswith("baseline_d23"):
        warm_path = Path(args.rbf_cache_root) / str(args.warm_cache_label)
        cfg.database.external_evidence_path = str(warm_path)
        cfg.database.external_evidence_use_snapshot = True
        cfg.database.external_evidence_auto_build_snapshot = True
    return sbf.load_iiwa14_robot(), cfg


def make_refine_config(case: str, deep_max_boxes: int, args: argparse.Namespace) -> Any:
    cfg = sbf.LeafSweepRefineConfig()
    cfg.leaf_start_depth = int(args.leaf_start_depth)
    cfg.leaf_max_depth = int(args.leaf_max_depth)
    cfg.obstacle_cluster_gap = 1000.0
    cfg.use_virtual_topology = bool(args.use_virtual_topology)
    cfg.parallel_virtual_validation = bool(args.parallel_virtual_validation)
    cfg.store_group_results = False
    cfg.validation_batch_size = int(args.validation_batch_size)
    cfg.leaf_threads = 1 if case == "single_thread" else int(args.threads)
    cfg.deep_max_boxes = int(deep_max_boxes)
    cfg.deep_ffb_depth = int(args.deep_ffb_depth)
    cfg.domain_seed_cap = int(args.domain_seed_cap)
    cfg.domain_success_cap = int(args.domain_success_cap)
    cfg.domain_attempt_cap = int(args.domain_attempt_cap)
    cfg.allow_anchor_roots = True
    cfg.refine_timeout_ms = float(args.refine_timeout_ms)
    return cfg


def canonical_q(robot: Any, q: Any) -> list[float]:
    return [
        float(value)
        for value in sbf.canonicalize_configuration_for_robot(
            robot,
            list(q),
            True,
            "joint_symmetry_native_v1",
        )
    ]


def canonical_priority_points(robot: Any, queries: list[Any]) -> list[list[float]]:
    points: list[list[float]] = []
    for query in queries:
        start = canonical_q(robot, query.start)
        goal = canonical_q(robot, query.goal)
        for alpha in (0.0, 0.25, 0.5, 0.75, 1.0):
            points.append([(1.0 - alpha) * a + alpha * b for a, b in zip(start, goal)])
    return points


def query_rows(forest: Any, robot: Any, queries: list[Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query in queries:
        start = canonical_q(robot, query.start)
        goal = canonical_q(robot, query.goal)
        result = forest.query(start, goal)
        path_length = float(result.path_length) if bool(result.success) else math.nan
        raw_path_length = float(getattr(result, "raw_path_length", result.path_length)) if bool(result.success) else math.nan
        segment_length = float(result.segment_edge_length) if bool(result.success) else 0.0
        rows.append({
            "label": str(query.label),
            "success": bool(result.success),
            "audit_passed": bool(result.audit_passed),
            "query_ms": float(result.query_time_ms),
            "audit_ms": float(result.audit_time_ms),
            "final_simplify_ms": float(getattr(result, "final_simplify_time_ms", 0.0)),
            "path_length": path_length,
            "final_path_length": path_length,
            "raw_path_length": raw_path_length,
            "segment_edge_length": segment_length,
            "segment_fraction": (segment_length / raw_path_length) if bool(result.success) and raw_path_length > 1e-12 else math.nan,
            "box_sequence_len": len(list(result.box_sequence)),
            "segment_edges_used": int(result.segment_edges_used),
            "waypoint_count": len(result.path_as_lists()),
            "audit_status": str(result.audit_status),
            "canonical_start": start,
            "canonical_goal": goal,
        })
    return rows


def run_case(case: str, seed: int, deep_max_boxes: int, args: argparse.Namespace) -> dict[str, Any]:
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    threads = 1 if case == "single_thread" else int(args.threads)
    options = RBFLeafRRTOptions(
        seed=int(seed),
        deep_max_boxes=int(deep_max_boxes),
        rbf_max_depth=int(args.rbf_max_depth),
        timeout_ms=float(args.timeout_ms),
        threads=threads,
        leaf_start_depth=int(args.leaf_start_depth),
        leaf_max_depth=int(args.leaf_max_depth),
        deep_ffb_depth=int(args.deep_ffb_depth),
        refine_timeout_ms=float(args.refine_timeout_ms),
        domain_seed_cap=int(args.domain_seed_cap),
        domain_success_cap=int(args.domain_success_cap),
        domain_attempt_cap=int(args.domain_attempt_cap),
        validation_batch_size=int(args.validation_batch_size),
        ffb_start_depth=int(args.ffb_start_depth),
        audit_resolution=int(args.audit_resolution),
        audit_segment_step=float(args.audit_segment_step),
        use_virtual_topology=bool(args.use_virtual_topology),
        parallel_virtual_validation=bool(args.parallel_virtual_validation),
        leaf_threads=threads,
        envelope="link_aabb" if case == "link_aabb" else "support_hull",
        endpoint_source="critsample" if case == "critsample_support_hull_unsafe" else "ifk",
        unsafe_sampling_validation=case == "critsample_support_hull_unsafe",
        use_external_evidence=case.startswith("baseline_d23"),
        external_evidence_path=Path(args.rbf_cache_root) / str(args.warm_cache_label),
        use_shelf_root_override=True,
        case_label=case,
        segment_edges_fallback_only=bool(args.segment_edges_fallback_only),
        connector_birrt=bool(args.connector_birrt),
        connector_bridge_boxes=int(args.connector_bridge_boxes),
        connector_pair_batch_size=int(args.connector_pair_batch_size),
        connector_pair_timeout_ms=float(args.connector_pair_timeout_ms),
        connector_max_pairs_per_gap=int(args.connector_max_pairs_per_gap),
        connector_rrt_iters=int(args.connector_rrt_iters),
        connector_rrt_timeout_ms=float(args.connector_rrt_timeout_ms),
        connector_rrt_step_size=float(args.connector_rrt_step_size),
        connector_rrt_goal_bias=float(args.connector_rrt_goal_bias),
        connector_segment_resolution=int(args.connector_segment_resolution),
        connector_pave_max_chain=int(args.connector_pave_max_chain),
        connector_pave_steps=int(args.connector_pave_steps),
        connector_pave_depth=int(args.connector_pave_depth),
        connector_pave_fill_gaps=bool(args.connector_pave_fill_gaps),
        connector_pave_require_connected_chain=bool(args.connector_pave_require_connected_chain),
        final_collision_shortcut=bool(args.final_collision_shortcut),
        final_rrt_simplify=bool(args.final_rrt_simplify),
        final_rrt_simplify_timeout_ms=float(args.final_rrt_simplify_timeout_ms),
        final_rrt_simplify_max_iters=int(args.final_rrt_simplify_max_iters),
        final_rrt_simplify_attempts=int(args.final_rrt_simplify_attempts),
        allow_anchor_roots=True,
        use_priority_points=True,
        run_rrt_grower=bool(args.run_rrt_grower),
        rrt_grower_extra_boxes=int(args.rrt_grower_extra_boxes),
        rrt_grower_timeout_ms=float(args.rrt_grower_timeout_ms),
    )
    return run_leaf_rrt(
        robot=robot,
        obstacles=obstacles,
        queries=queries,
        database_path=args.out_dir / "active_cache" / f"{case}_seed{seed}_box{deep_max_boxes}",
        options=options,
    )


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    keys = sorted({(row["case"], row["deep_max_boxes"]) for row in rows})
    for case, budget in keys:
        items = [row for row in rows if row["case"] == case and row["deep_max_boxes"] == budget]
        out.append({
            "case": case,
            "deep_max_boxes": budget,
            "runs": len(items),
            "success_runs": sum(1 for row in items if row["success_count"] == row["query_count"]),
            "planning_s_median": median(row["planning_s"] for row in items),
            "build_s_median": median(row["build_s"] for row in items),
            "query_s_median": median(row["query_s"] for row in items),
            "leaf_sweep_s_median": median(row["leaf_sweep_s"] for row in items),
            "deep_refine_s_median": median(row["deep_refine_s"] for row in items),
            "connector_s_median": median(row["connector_s"] for row in items),
            "audit_s_median": median(row["audit_s"] for row in items),
            "path_length_mean": mean(row["path_length_mean"] for row in items),
            "raw_segment_fraction_median": median(row["raw_segment_fraction"] for row in items),
            "final_boxes_median": median(row["final_boxes"] for row in items),
            "segment_edges_median": median(row["segment_edges"] for row in items),
            "adjacency_islands_median": median(row["adjacency_islands"] for row in items),
            "external_hits_median": median(row["external_hits"] for row in items),
            "unsafe_sampling_validation": any(bool(row.get("unsafe_sampling_validation")) for row in items),
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["case", "deep_max_boxes", "runs", "success_runs", "planning_s_median", "build_s_median", "query_s_median", "leaf_sweep_s_median", "deep_refine_s_median", "connector_s_median", "audit_s_median", "path_length_mean", "raw_segment_fraction_median", "final_boxes_median", "segment_edges_median", "adjacency_islands_median", "external_hits_median", "unsafe_sampling_validation"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Shelf+IIWA leaf-sweep--RRT grower ablation. Planning excludes final audit and equals build plus five-query graph search. Len. is the success-only mean path length over the five shelf queries. Seg. is raw pre-simplification segment-edge length fraction. CritSample is a sampling-based row and does not provide box-level safety certificates.}",
        r"\label{tab:tro-shelf-ablation}",
        r"\begin{tabular}{lrrrrrrrrrr}",
        r"\toprule",
        r"Case & Boxes & SR & Plan & Build & Query & Leaf & Refine & Conn. & Len. & Seg. \\",
        r"\midrule",
    ]
    for row in rows:
        sr = f"{int(row['success_runs'])}/{int(row['runs'])}"
        label = CASE_LABELS.get(str(row["case"]), str(row["case"])).replace("_", r"\_")
        full_success = int(row["success_runs"]) == int(row["runs"])
        path_length = row["path_length_mean"] if full_success else None
        segment_fraction = row["raw_segment_fraction_median"] if full_success else None
        lines.append(
            f"{label} & {int(row['deep_max_boxes'])} & {sr} & "
            f"{tex_num(row['planning_s_median'])} & {tex_num(row['build_s_median'])} & "
            f"{tex_num(row['query_s_median'])} & {tex_num(row['leaf_sweep_s_median'])} & "
            f"{tex_num(row['deep_refine_s_median'])} & {tex_num(row['connector_s_median'])} & "
            f"{tex_num(path_length)} & {tex_num(segment_fraction)} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table*}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.4 Shelf+IIWA leaf-sweep + RRT grower study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp04")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--box-budgets", default=",".join(str(item) for item in rbf_budget_grid("pilot")))
    parser.add_argument("--only", default="baseline_d23_aafk_support_hull_8t,critsample_support_hull_unsafe,no_external_lect,link_aabb,single_thread")
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--leaf-start-depth", type=int, default=DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--leaf-max-depth", type=int, default=DEFAULT_RBF_LEAF_MAX_DEPTH)
    parser.add_argument("--deep-ffb-depth", type=int, default=DEFAULT_RBF_DEEP_FFB_DEPTH)
    parser.add_argument("--refine-timeout-ms", type=float, default=DEFAULT_RBF_REFINE_TIMEOUT_MS)
    parser.add_argument("--domain-seed-cap", type=int, default=DEFAULT_RBF_DOMAIN_SEED_CAP)
    parser.add_argument("--domain-success-cap", type=int, default=DEFAULT_RBF_DOMAIN_SUCCESS_CAP)
    parser.add_argument("--domain-attempt-cap", type=int, default=DEFAULT_RBF_DOMAIN_ATTEMPT_CAP)
    parser.add_argument("--validation-batch-size", type=int, default=DEFAULT_RBF_VALIDATION_BATCH_SIZE)
    parser.add_argument("--ffb-start-depth", type=int, default=DEFAULT_RBF_FFB_START_DEPTH)
    parser.add_argument("--audit-resolution", type=int, default=DEFAULT_RBF_AUDIT_RESOLUTION)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
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
    parser.add_argument("--connector-pave-depth", type=int, default=DEFAULT_RBF_CONNECTOR_PAVE_DEPTH)
    parser.add_argument("--connector-pave-fill-gaps", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-pave-require-connected-chain", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--final-collision-shortcut", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_COLLISION_SHORTCUT)
    parser.add_argument("--final-rrt-simplify", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY)
    parser.add_argument("--final-rrt-simplify-timeout-ms", type=float, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS)
    parser.add_argument("--final-rrt-simplify-max-iters", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS)
    parser.add_argument("--final-rrt-simplify-attempts", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS)
    parser.add_argument("--run-rrt-grower", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rrt-grower-extra-boxes", type=int, default=DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES)
    parser.add_argument("--rrt-grower-timeout-ms", type=float, default=DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS)
    parser.add_argument("--use-virtual-topology", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    seeds = [int(item) for item in str(args.seeds).split(",") if item.strip()]
    budgets = [int(item) for item in str(args.box_budgets).split(",") if item.strip()]
    wanted = {item.strip() for item in str(args.only).split(",") if item.strip()}
    cases = [case for case in ABLATIONS if not wanted or "all" in wanted or case in wanted]
    if args.phase == "smoke":
        seeds = seeds[:1]
        budgets = budgets[:1]
        cases = cases[:1]
    manifest_rows = [
        {"case": case, "seed": seed, "deep_max_boxes": budget, "backend": "build_leaf_sweep_refined", "grower_mode": "rrt"}
        for case in cases for seed in seeds for budget in budgets
    ]
    run_rows: list[dict[str, Any]] = []
    if not args.dry_run:
        for row in progress(manifest_rows, desc="exp04 runs", total=len(manifest_rows)):
            print(f"[exp04] case={row['case']} seed={row['seed']} boxes={row['deep_max_boxes']}", flush=True)
            run_rows.append(run_case(str(row["case"]), int(row["seed"]), int(row["deep_max_boxes"]), args))
    summary_rows = aggregate(run_rows) if run_rows else []
    payload: dict[str, Any] = {
        "experiment": "exp04_shelf_leaf_rrt",
        "run_id": run_id("exp04"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "config": {
            "d23_cache_root": str(args.rbf_cache_root),
            "warm_cache_label": str(args.warm_cache_label),
            "rbf_default_profile": shelf_d23_rbf_profile(),
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "deep_ffb_depth": int(args.deep_ffb_depth),
            "ffb_start_depth": int(args.ffb_start_depth),
            "audit_segment_step": float(args.audit_segment_step),
            "segment_edges_fallback_only": bool(args.segment_edges_fallback_only),
            "connector_max_pairs_per_gap": int(args.connector_max_pairs_per_gap),
            "connector_pair_timeout_ms": float(args.connector_pair_timeout_ms),
            "connector_rrt_iters": int(args.connector_rrt_iters),
            "connector_rrt_timeout_ms": float(args.connector_rrt_timeout_ms),
            "connector_bridge_boxes": int(args.connector_bridge_boxes),
            "final_collision_shortcut": bool(args.final_collision_shortcut),
            "final_rrt_simplify": bool(args.final_rrt_simplify),
            "final_rrt_simplify_timeout_ms": float(args.final_rrt_simplify_timeout_ms),
            "final_rrt_simplify_max_iters": int(args.final_rrt_simplify_max_iters),
            "final_rrt_simplify_attempts": int(args.final_rrt_simplify_attempts),
            "run_rrt_grower": bool(args.run_rrt_grower),
            "rrt_grower_extra_boxes": int(args.rrt_grower_extra_boxes),
            "rrt_grower_timeout_ms": float(args.rrt_grower_timeout_ms),
            "critical_sample_row": "critsample_support_hull_unsafe uses CritSample + CoverageHeuristic + provisional boxes; final strict audit is still reported, but box-level safety certificates are not guaranteed.",
        },
        "planned_rows": manifest_rows,
        "rows": run_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "shelf_leaf_rrt_manifest.json", payload)
    if summary_rows:
        write_csv(args.out_dir / "shelf_leaf_rrt_summary.csv", summary_rows)
        write_tex(REPO_ROOT / "paper" / "generated" / "tab_tro_shelf_ablation.tex", summary_rows)
    print(f"wrote {args.out_dir / 'shelf_leaf_rrt_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
