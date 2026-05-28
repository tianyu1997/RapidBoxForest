#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


def _bootstrap_imports() -> Path:
    root = Path(__file__).resolve().parents[1]
    build_dir = os.environ.get("SBF_BUILD_DIR")
    candidates = []
    if build_dir:
        candidates.append(Path(build_dir) / "python")
    candidates.extend((root / "build_py310" / "python", root / "build" / "python", root / "python"))
    for candidate in reversed(candidates):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root


ROOT = _bootstrap_imports()
REPO_ROOT = ROOT.parents[1]

import sbf
from sbf.marcucci import make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot

from common_sbf_config import configure_external_evidence_reuse


BASELINE_WRAPPER = ROOT / "experiments" / "paper_04_baselines_marcucci.py"


def mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def box_volume_sum(boxes: list[Any]) -> float:
    return sum(float(box.volume) for box in boxes)


def count_status(boxes: list[Any], status: Any) -> int:
    return sum(1 for box in boxes if box.safety_status == status)


def set_if_available(obj: Any, name: str, value: Any) -> bool:
    try:
        setattr(obj, name, value)
        return True
    except AttributeError:
        return False


def set_path_if_available(obj: Any, path: str, value: Any) -> bool:
    current = obj
    parts = path.split(".")
    for part in parts[:-1]:
        try:
            current = getattr(current, part)
        except AttributeError:
            return False
    return set_if_available(current, parts[-1], value)


def query_payload(query: Any, result: sbf.QueryResult, wall_s: float) -> dict[str, Any]:
    waypoints = [[float(value) for value in waypoint] for waypoint in result.path]
    return {
        "from": query.start_name,
        "to": query.goal_name,
        "t_s": float(wall_s),
        "ok": bool(result.success),
        "audit_status": str(result.audit_status).split(".")[-1],
        "audit_passed": bool(result.audit_passed),
        "audit_time_ms": float(result.audit_time_ms),
        "repair_time_ms": float(result.repair_time_ms),
        "repair_count": int(result.repair_count),
        "failed_segment_index": int(result.failed_segment_index),
        "certified_box_length": float(result.certified_box_length),
        "provisional_audited_length": float(result.provisional_audited_length),
        "segment_edge_length": float(result.segment_edge_length),
        "remaining_unsafe_assumptions": int(result.remaining_unsafe_assumptions),
        "start_box_id": int(result.start_box_id),
        "goal_box_id": int(result.goal_box_id),
        "length": float(result.path_length) if result.success else 0.0,
        "planning_time_ms": float(result.query_time_ms),
        "box_sequence_len": len(result.box_sequence),
        "segment_edges_used": int(result.segment_edges_used),
        "segment_edge_sequence": [int(value) for value in result.segment_edge_sequence],
        "waypoints": waypoints,
        "waypoint_count": len(waypoints),
    }


def summarize_queries(trials: list[dict[str, Any]], query_labels: list[str]) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for label in query_labels:
        rows: list[dict[str, Any]] = []
        for trial in trials:
            for row in trial["queries"]:
                if f"{row['from']}->{row['to']}" == label:
                    rows.append(row)
        successes = [row for row in rows if row.get("ok")]
        summaries.append({
            "name": label,
            "sr": mean([1.0 if row.get("ok") else 0.0 for row in rows]),
            "audit_sr": mean([1.0 if row.get("audit_passed") else 0.0 for row in rows]),
            "t_med_s": median([float(row["t_s"]) for row in successes]) if successes else None,
            "len_med": median([float(row["length"]) for row in successes]) if successes else None,
            "segment_edges_used_med": median([float(row.get("segment_edges_used", 0)) for row in successes]) if successes else None,
            "repair_count_med": median([float(row.get("repair_count", 0)) for row in rows]),
        })
    return summaries


def configure(args: argparse.Namespace, seed: int) -> sbf.SBFConfig:
    cfg = sbf.SBFConfig()
    cfg.enable_merger = bool(args.enable_merger)
    cfg.enable_connector = bool(args.enable_connector)
    cfg.runtime.mode = sbf.ExecutionMode.Parallel if args.threads > 1 else sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = max(1, int(args.threads))
    cfg.runtime.batch_size = max(1, int(args.task_batch_size))
    cfg.runtime.parallel_threshold = 1
    database_path = args.database_path or (args.out_json.parent / "cache" / args.out_json.stem)
    cfg.database.path = str(database_path)
    cfg.database.create_if_missing = True

    if args.preset == "ifk_strict":
        cfg.endpoint_source.source = sbf.EndpointSource.IFK
        cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    elif args.preset in {"crit_link_coverage", "kdop26_coverage", "support_hull_coverage"}:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
        if args.preset == "kdop26_coverage":
            cfg.envelope_type.type = sbf.EnvelopeType.KDOP
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        elif args.preset == "support_hull_coverage":
            cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
            cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)
        else:
            cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
    else:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
        cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
        cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
        cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed

    if args.envelope != "preset":
        if args.envelope == "link":
            cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        elif args.envelope == "kdop26":
            cfg.envelope_type.type = sbf.EnvelopeType.KDOP
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        elif args.envelope == "support_hull":
            cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
            cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)

    cfg.envelope_type.n_subdivisions = int(args.envelope_subdivisions)
    use_best_tighten = args.split_policy == "best-tighten"
    set_path_if_available(cfg, "grower.find_free_box.split.use_best_tighten", use_best_tighten)
    set_path_if_available(cfg, "connector.pave.find_free_box.split.use_best_tighten", use_best_tighten)
    for prefix in ("grower.find_free_box", "connector.pave.find_free_box"):
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.depth_synchronous", bool(args.best_tighten_depth_synchronous))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.prefer_sector_boundary", bool(args.best_tighten_prefer_sector_boundary))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.use_minimax", bool(args.best_tighten_use_minimax))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.shape_balancing", bool(args.best_tighten_shape_balancing))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.recent_dim_cooling", bool(args.best_tighten_recent_dim_cooling))
    cfg.grower.mode = sbf.GrowerMode.RRT
    cfg.grower.rng_seed = int(args.seed_base) + int(seed)
    cfg.grower.max_boxes = int(args.max_boxes)
    cfg.grower.timeout_ms = float(args.timeout_ms)
    cfg.grower.max_consecutive_miss = int(args.max_consecutive_miss)
    cfg.grower.n_threads = max(1, int(args.threads))
    cfg.grower.task_batch_size = max(1, int(args.task_batch_size))
    cfg.grower.parallel_threshold = 1
    cfg.grower.worker_local_ffb = args.threads > 1
    cfg.grower.find_free_box.max_depth = int(args.ffb_depth)
    cfg.grower.find_free_box.split_reserved_leaf = True
    cfg.grower.find_free_box.split_unknown_leaf = True
    cfg.grower.find_free_box.reject_seed_collision = False
    set_if_available(cfg.grower, "rrt_goal_bias", float(args.rrt_goal_bias))
    set_if_available(cfg.grower, "intertree_goal_bias", float(args.intertree_goal_bias))
    set_if_available(cfg.grower, "sustained_goal_bias_cap", min(0.25, float(args.intertree_goal_bias)))
    set_if_available(cfg.grower, "rrt_step_ratio", float(args.step_ratio))
    set_if_available(cfg.grower, "unexplored_sample_prob", float(args.unexplored_prob))
    cfg.grower.connect_mode = True
    cfg.grower.expand_all_roots_per_sample = True
    set_if_available(cfg.grower, "component_connect_prob", float(args.component_connect_prob))
    set_if_available(cfg.grower, "component_connect_candidate_limit", int(args.component_connect_candidate_limit))
    cfg.grower.component_connect_island_aware = True
    cfg.grower.component_connect_frontier_cache = True
    cfg.grower.component_connect_staged_growth = True
    set_if_available(cfg.grower, "component_connect_stage_normalized_linf", float(args.component_connect_stage_normalized_linf))
    set_if_available(cfg.grower, "component_connect_adaptive_ffb", True)
    set_if_available(cfg.grower, "component_connect_ffb_depth_increment", int(args.component_connect_ffb_depth_increment))
    set_if_available(cfg.grower, "component_connect_ffb_max_depth", int(args.component_connect_ffb_max_depth))
    set_if_available(cfg.grower, "stop_after_connect", bool(args.stop_after_connect))
    set_if_available(cfg.grower, "post_connect_extra_boxes", int(args.post_connect_extra_boxes))
    set_if_available(cfg.grower, "quality_min_connected_boxes", int(args.quality_min_connected_boxes))
    set_if_available(cfg.grower, "post_connect_time_budget_ms", float(args.post_connect_time_budget_ms))
    set_if_available(cfg.grower, "coverage_first_stop_loss", bool(args.coverage_first_stop_loss))
    set_if_available(cfg.grower, "hard_frontier_failure_threshold", int(args.hard_frontier_failure_threshold))
    set_if_available(cfg.grower, "hard_frontier_box_horizon", int(args.hard_frontier_box_horizon))

    cfg.query.nearest_if_outside = False
    cfg.query.shortcut_boxes = True
    cfg.query.collision_shortcut = bool(args.collision_shortcut)
    cfg.query.collision_shortcut_resolution = int(args.collision_shortcut_resolution)
    cfg.query.strict_path_audit = bool(args.strict_path_audit)
    cfg.query.audit_resolution = int(args.audit_resolution)
    cfg.query.repair_on_audit_failure = bool(args.repair_on_audit_failure)
    cfg.query.repair_max_attempts = int(args.repair_max_attempts)
    cfg.query.repair_rrt_max_iters = int(args.repair_rrt_max_iters)
    cfg.query.repair_timeout_ms = float(args.repair_timeout_ms)
    set_if_available(cfg.query, "repair_local_sampling_radius", float(args.repair_local_sampling_radius))
    set_if_available(cfg.query, "repair_local_sampling_growth", float(args.repair_local_sampling_growth))
    cfg.validation.enable_validation_cache = bool(args.validation_cache)
    cfg.validation.validation_cache_max_entries = int(args.validation_cache_max_entries)
    if bool(args.use_external_evidence):
        warm_source_path = Path(args.rbf_cache_root) / str(args.warm_cache_label)
        if not warm_source_path.exists():
            raise FileNotFoundError(f"warm d18 cache does not exist: {warm_source_path}")
        configure_external_evidence_reuse(
            cfg,
            warm_source_path,
            args,
            materialization=True,
            scoring=True,
            backfill_active=bool(args.external_evidence_backfill_active),
        )
        cfg.database.online_cache.allow_database_backfill = bool(args.online_cache_allow_database_backfill)
    set_if_available(cfg.connector, "frontier_bridge", bool(args.frontier_bridge))
    cfg.connector.max_total_bridge_boxes = int(args.connector_bridge_boxes)
    cfg.connector.segment_edges_enabled = bool(args.segment_edges)
    cfg.connector.rrt_segment_edges = bool(args.segment_edges)
    cfg.connector.point_gap_segment_edges = bool(args.segment_edges)
    cfg.connector.n_threads = max(1, int(args.threads))
    set_if_available(cfg.connector, "pair_batch_size", max(1, int(args.connector_pair_batch_size)))
    cfg.connector.parallel_threshold = 1
    set_if_available(cfg.connector, "per_pair_timeout_ms", float(args.connector_pair_timeout_ms))
    set_if_available(cfg.connector, "max_pairs_per_gap", int(args.connector_max_pairs_per_gap))
    cfg.connector.rrt.max_iters = int(args.connector_rrt_iters)
    cfg.connector.rrt.timeout_ms = float(args.connector_rrt_timeout_ms)
    cfg.connector.rrt.step_size = float(args.connector_rrt_step_size)
    cfg.connector.rrt.goal_bias = float(args.connector_rrt_goal_bias)
    cfg.connector.rrt.segment_resolution = int(args.connector_segment_resolution)
    cfg.connector.pave.max_chain = int(args.connector_pave_max_chain)
    cfg.connector.pave.max_steps_per_waypoint = int(args.connector_pave_steps)
    cfg.connector.pave.find_free_box.max_depth = int(args.connector_pave_depth)
    cfg.connector.pave.find_free_box.split_reserved_leaf = True
    cfg.connector.pave.find_free_box.split_unknown_leaf = True
    cfg.connector.pave.find_free_box.reject_seed_collision = False
    return cfg


def refine_corridors(forest: sbf.SafeBoxForest, queries: list[Any], args: argparse.Namespace) -> tuple[float, int, int]:
    if not args.corridor_refine:
        return 0.0, 0, 0
    budget_s = max(0.0, float(args.corridor_refine_budget_ms)) / 1000.0
    max_total = max(0, int(args.corridor_refine_max_boxes))
    per_query = max(1, int(args.corridor_refine_boxes_per_query))
    if budget_s <= 0.0 or max_total <= 0:
        return 0.0, 0, 0
    t0 = time.perf_counter()
    added_total = 0
    attempted = 0
    start_margin_s = max(0.0, float(args.corridor_refine_start_margin_ms)) / 1000.0
    defer_labels = {item.strip() for item in str(args.corridor_refine_defer_labels).split(",") if item.strip()}
    ordered_queries = sorted(queries, key=lambda query: (query.label in defer_labels, query.label))
    for _ in range(max(1, int(args.corridor_refine_passes))):
        pass_added = 0
        for query in ordered_queries:
            elapsed_s = time.perf_counter() - t0
            if added_total >= max_total or elapsed_s >= budget_s:
                break
            if attempted > 0 and budget_s - elapsed_s < start_margin_s:
                break
            quota = min(per_query, max_total - added_total)
            added = int(forest.refine_query_corridor(list(query.start), list(query.goal), quota))
            attempted += 1
            added_total += added
            pass_added += added
        if pass_added == 0 or added_total >= max_total or time.perf_counter() - t0 >= budget_s:
            break
    return time.perf_counter() - t0, added_total, attempted


def compare_to_v6(payload: dict[str, Any], v6_path: Path | None) -> dict[str, Any] | None:
    if v6_path is None or not v6_path.exists():
        return None
    v6 = json.loads(v6_path.read_text(encoding="utf-8"))
    out: dict[str, Any] = {
        "v6_path": str(v6_path),
        "build_mean_s_v6": v6.get("build", {}).get("mean_s"),
        "build_mean_s_standalone": payload.get("build", {}).get("mean_s"),
    }
    if out["build_mean_s_v6"] and out["build_mean_s_standalone"] is not None:
        out["build_mean_s_ratio_standalone_over_v6"] = out["build_mean_s_standalone"] / out["build_mean_s_v6"]
    v6_queries = {row["name"]: row for row in v6.get("queries", [])}
    query_rows = []
    for row in payload.get("queries", []):
        base = v6_queries.get(row["name"], {})
        item = {
            "name": row["name"],
            "sr_v6": base.get("sr"),
            "sr_standalone": row.get("sr"),
            "t_med_s_v6": base.get("t_med_s"),
            "t_med_s_standalone": row.get("t_med_s"),
            "len_med_v6": base.get("len_med"),
            "len_med_standalone": row.get("len_med"),
        }
        if item["t_med_s_v6"] and item["t_med_s_standalone"] is not None:
            item["t_med_s_ratio_standalone_over_v6"] = item["t_med_s_standalone"] / item["t_med_s_v6"]
        query_rows.append(item)
    out["queries"] = query_rows
    return out


def run_exp4_baselines(args: argparse.Namespace) -> dict[str, Any] | None:
    if not args.run_baselines:
        return None
    cmd = [
        sys.executable,
        str(BASELINE_WRAPPER),
        "--source",
        args.baseline_source,
        "--methods",
        args.baseline_methods,
        "--out-dir",
        str(args.baseline_out_dir),
        "--logical-threads",
        str(args.baseline_logical_threads),
        "--bitstar-budget-s",
        str(args.bitstar_budget_s),
        "--bitstar-restarts",
        str(args.bitstar_restarts),
        "--bitstar-simplify-time-s",
        str(args.bitstar_simplify_time_s),
        "--prm-build-budget-s",
        str(args.prm_build_budget_s),
        "--prm-query-budget-s",
        str(args.prm_query_budget_s),
        "--prm-max-nearest-neighbors",
        str(args.prm_max_nearest_neighbors),
        "--prm-simplify-time-s",
        str(args.prm_simplify_time_s),
        "--iris-budget-s",
        str(args.iris_budget_s),
        "--iris-query-time-limit-s",
        str(args.iris_query_time_limit_s),
        "--iris-iteration-limit",
        str(args.iris_iteration_limit),
        "--bitstar-stop-on-solution-improvement" if args.bitstar_stop_on_solution_improvement else "--no-bitstar-stop-on-solution-improvement",
    ]
    cmd.append("--full" if args.baseline_mode == "full" else "--quick")
    if args.baseline_seeds is not None:
        cmd += ["--seeds", str(args.baseline_seeds)]
    if args.baseline_timeout is not None:
        cmd += ["--timeout", str(args.baseline_timeout)]
    t0 = time.perf_counter()
    subprocess.run([str(part) for part in cmd], check=True, cwd=ROOT)
    return {
        "source": args.baseline_source,
        "methods": args.baseline_methods,
        "out_dir": str(args.baseline_out_dir),
        "wall_s": time.perf_counter() - t0,
        "command": [str(part) for part in cmd],
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone SBF paper Exp.4 Marcucci combined runner.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_combined_standalone.json")
    parser.add_argument("--database-path", type=Path, default=None)
    parser.add_argument("--v6-json", type=Path, default=None, help="Optional legacy diagnostic comparison JSON; omitted for paper-facing current artifacts.")
    parser.add_argument(
        "--preset",
        choices=["coverage_hybrid", "crit_link_coverage", "kdop26_coverage", "support_hull_coverage", "ifk_strict"],
        default="support_hull_coverage",
    )
    parser.add_argument("--envelope", choices=["preset", "link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--grow-only", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--task-batch-size", type=int, default=8)
    parser.add_argument("--max-boxes", type=int, default=5000)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--max-consecutive-miss", type=int, default=2000)
    parser.add_argument("--grid-delta", type=float, default=0.04)
    parser.add_argument("--envelope-subdivisions", type=int, default=4)
    parser.add_argument("--kdop-safety-epsilon", type=float, default=1e-9)
    parser.add_argument("--support-hull-safety-epsilon", type=float, default=1e-9)
    parser.add_argument("--split-policy", choices=["widest-first", "best-tighten"], default="best-tighten")
    parser.add_argument("--best-tighten-depth-synchronous", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-prefer-sector-boundary", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-use-minimax", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-shape-balancing", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--best-tighten-recent-dim-cooling", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--enable-merger", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--enable-connector", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--coverage-first-stop-loss", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hard-frontier-failure-threshold", type=int, default=1)
    parser.add_argument("--hard-frontier-box-horizon", type=int, default=300)
    parser.add_argument("--rrt-goal-bias", type=float, default=0.2)
    parser.add_argument("--intertree-goal-bias", type=float, default=0.25)
    parser.add_argument("--unexplored-prob", type=float, default=0.45)
    parser.add_argument("--step-ratio", type=float, default=0.08)
    parser.add_argument("--component-connect-prob", type=float, default=0.45)
    parser.add_argument("--component-connect-candidate-limit", type=int, default=4)
    parser.add_argument("--component-connect-stage-normalized-linf", type=float, default=0.35)
    parser.add_argument("--component-connect-ffb-depth-increment", type=int, default=40)
    parser.add_argument("--component-connect-ffb-max-depth", type=int, default=160)
    parser.add_argument("--stop-after-connect", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--post-connect-extra-boxes", type=int, default=0)
    parser.add_argument("--quality-min-connected-boxes", type=int, default=64)
    parser.add_argument("--post-connect-time-budget-ms", type=float, default=450.0)
    parser.add_argument("--frontier-bridge", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-bridge-boxes", type=int, default=0)
    parser.add_argument("--segment-edges", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-shortcut", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-shortcut-resolution", type=int, default=24)
    parser.add_argument("--strict-path-audit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--audit-resolution", type=int, default=32)
    parser.add_argument("--repair-on-audit-failure", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--repair-max-attempts", type=int, default=6)
    parser.add_argument("--repair-rrt-max-iters", type=int, default=20000)
    parser.add_argument("--repair-timeout-ms", type=float, default=750.0)
    parser.add_argument("--repair-local-sampling-radius", type=float, default=0.4)
    parser.add_argument("--repair-local-sampling-growth", type=float, default=2.0)
    parser.add_argument("--validation-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--validation-cache-max-entries", type=int, default=200000)
    parser.add_argument("--rbf-cache-root", type=Path, default=ROOT / "outputs" / "paper" / "rbf_only" / "cache")
    parser.add_argument("--warm-cache-label", default="e5_lifelong_cache_link_d18_canonical_dim0q4")
    parser.add_argument("--use-external-evidence", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-mode", choices=["legacy", "snapshot"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-backfill-active", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--online-cache-allow-database-backfill", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-pair-batch-size", type=int, default=1)
    parser.add_argument("--connector-pair-timeout-ms", type=float, default=250.0)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=8)
    parser.add_argument("--connector-rrt-iters", type=int, default=50000)
    parser.add_argument("--connector-rrt-timeout-ms", type=float, default=2000.0)
    parser.add_argument("--connector-rrt-step-size", type=float, default=0.25)
    parser.add_argument("--connector-rrt-goal-bias", type=float, default=0.4)
    parser.add_argument("--connector-segment-resolution", type=int, default=16)
    parser.add_argument("--connector-pave-max-chain", type=int, default=0)
    parser.add_argument("--connector-pave-steps", type=int, default=12)
    parser.add_argument("--connector-pave-depth", type=int, default=120)
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine-budget-ms", type=float, default=250.0)
    parser.add_argument("--corridor-refine-max-boxes", type=int, default=48)
    parser.add_argument("--corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--corridor-refine-passes", type=int, default=2)
    parser.add_argument("--corridor-refine-start-margin-ms", type=float, default=120.0)
    parser.add_argument("--corridor-refine-defer-labels", type=str, default="CS->LB")
    parser.add_argument("--run-baselines", action="store_true", help="Run or import migrated Exp.4 baseline artifacts after the SBF row.")
    parser.add_argument("--baseline-source", choices=["live"], default="live")
    parser.add_argument("--baseline-mode", choices=["quick", "full"], default="quick")
    parser.add_argument("--baseline-methods", default="iris_np,ompl")
    parser.add_argument("--baseline-out-dir", type=Path, default=ROOT / "outputs" / "paper")
    parser.add_argument("--baseline-seeds", type=int, default=None)
    parser.add_argument("--baseline-timeout", type=int, default=None)
    parser.add_argument("--baseline-logical-threads", type=int, default=8)
    parser.add_argument("--baseline-build-dir", type=Path, default=None)
    parser.add_argument("--baseline-allow-debug-build", action="store_true")
    parser.add_argument("--bitstar-budget-s", type=float, default=10.0)
    parser.add_argument("--bitstar-restarts", type=int, default=5)
    parser.add_argument("--bitstar-simplify-time-s", type=float, default=0.2)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prm-build-budget-s", type=float, default=40.0)
    parser.add_argument("--prm-query-budget-s", type=float, default=2.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-simplify-time-s", type=float, default=0.10)
    parser.add_argument("--iris-budget-s", type=float, default=800.0)
    parser.add_argument("--iris-query-time-limit-s", type=float, default=120.0)
    parser.add_argument("--iris-iteration-limit", type=int, default=10)
    parser.add_argument("--iris-zo-query-time-limit-s", type=float, default=120.0)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.grow_only:
        args.enable_connector = False
        args.corridor_refine = False
        args.bridge_failed_queries = False
        args.bridge_repaired_queries = False
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]
    trials: list[dict[str, Any]] = []
    query_labels = [query.label for query in queries]

    for seed in range(max(1, int(args.seeds))):
        cfg = configure(args, seed)
        forest = sbf.SafeBoxForest(robot, cfg)
        build_t0 = time.perf_counter()
        profile = forest.build_coverage(obstacles, seeds)
        if args.grow_only:
            prebridge_time_s, prebridge_added_boxes, prebridge_attempts = 0.0, 0, 0
        else:
            prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, args)
        build_s = time.perf_counter() - build_t0
        boxes = forest.boxes()
        query_rows: list[dict[str, Any]] = []
        for query in ([] if args.grow_only else queries):
            query_t0 = time.perf_counter()
            result = forest.query(list(query.start), list(query.goal))
            query_s = time.perf_counter() - query_t0
            should_bridge = (not result.success and args.bridge_failed_queries) or (
                bool(args.bridge_repaired_queries)
                and result.success
                and int(result.repair_count) > 0
                and int(result.start_box_id) != int(result.goal_box_id)
            )
            if should_bridge:
                bridge_t0 = time.perf_counter()
                if hasattr(forest, "bridge_query_known_needed"):
                    bridge_progress = forest.bridge_query_known_needed(list(query.start), list(query.goal))
                else:
                    bridge_progress = forest.bridge_query(list(query.start), list(query.goal))
                bridge_s = time.perf_counter() - bridge_t0
                retry_t0 = time.perf_counter()
                retry = forest.query(list(query.start), list(query.goal))
                retry_s = time.perf_counter() - retry_t0
                row = query_payload(query, retry, query_s + bridge_s + retry_s)
                row["bridge_progress"] = int(bridge_progress)
                row["bridge_time_s"] = float(bridge_s)
                row["pre_bridge_ok"] = bool(result.success)
            else:
                row = query_payload(query, result, query_s)
                row["bridge_progress"] = 0
                row["bridge_time_s"] = 0.0
                row["pre_bridge_ok"] = bool(result.success)
            query_rows.append(row)
        diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
        trials.append({
            "seed": seed,
            "seed_index": seed,
            "build_s": float(build_s),
            "prebridge_time_s": float(prebridge_time_s),
            "prebridge_added_boxes": int(prebridge_added_boxes),
            "prebridge_attempts": int(prebridge_attempts),
            "n_boxes": int(len(boxes)),
            "unique_box_count": int(len(boxes)),
            "duplicate_box_count": 0,
            "box_volume_sum": box_volume_sum(boxes),
            "dedup_box_volume_sum": box_volume_sum(boxes),
            "duplicate_box_volume_sum": 0.0,
            "certified_box_count": count_status(boxes, sbf.BoxSafetyStatus.CertifiedFree),
            "provisional_box_count": count_status(boxes, sbf.BoxSafetyStatus.ProvisionalFree),
            "strict_audit_required_box_count": sum(1 for box in boxes if bool(box.strict_audit_required)),
            "segment_edge_count": len(forest.segment_edges()),
            "build_profile": {
                "total_ms": float(profile.total_ms),
                "grow_ms": float(profile.grow_ms),
                "merge_ms": float(profile.merge_ms),
                "connector_ms": float(profile.connector_ms),
                "adjacency_ms": float(profile.adjacency_ms),
                "raw_boxes": int(profile.raw_boxes),
                "final_boxes": int(profile.final_boxes),
                "bridge_boxes_added": int(profile.bridge_boxes_added),
                "segment_edges": int(profile.segment_edges),
                "segment_edges_added": int(profile.segment_edges_added),
                "rrt_segment_edges_added": int(profile.rrt_segment_edges_added),
                "point_gap_segment_edges_added": int(profile.point_gap_segment_edges_added),
                "connector_attempted_pairs": int(profile.connector_attempted_pairs),
                "connector_connected": bool(profile.connector_connected),
                "adjacency_islands": int(profile.adjacency_islands),
                "diagnostics": diagnostics,
            },
            "queries": query_rows,
        })

    payload: dict[str, Any] = {
        "experiment": "marcucci",
        "robot": "iiwa14",
        "scene": "marcucci_combined",
        "source_protocol": "standalone_sbf_build_coverage_query",
        "source_script": str(Path(__file__).resolve()),
        "seeds": max(1, int(args.seeds)),
        "params": {
            "preset": args.preset,
            "envelope": args.envelope,
            "database_path": str(args.database_path) if args.database_path is not None else str(args.out_json.parent / "cache" / args.out_json.stem),
            "grow_only": args.grow_only,
            "seed_points": ["AS", "TS", "CS", "LB", "RB"],
            "max_boxes": args.max_boxes,
            "timeout_ms": args.timeout_ms,
            "ffb_depth": args.ffb_depth,
            "envelope_subdivisions": args.envelope_subdivisions,
            "kdop_safety_epsilon": args.kdop_safety_epsilon,
            "support_hull_safety_epsilon": args.support_hull_safety_epsilon,
            "threads": args.threads,
            "split_policy": args.split_policy,
            "best_tighten_depth_synchronous": args.best_tighten_depth_synchronous,
            "best_tighten_shape_balancing": args.best_tighten_shape_balancing,
            "best_tighten_recent_dim_cooling": args.best_tighten_recent_dim_cooling,
            "coverage_first_stop_loss": args.coverage_first_stop_loss,
            "quality_min_connected_boxes": args.quality_min_connected_boxes,
            "post_connect_time_budget_ms": args.post_connect_time_budget_ms,
            "hard_frontier_failure_threshold": args.hard_frontier_failure_threshold,
            "hard_frontier_box_horizon": args.hard_frontier_box_horizon,
            "segment_edges": args.segment_edges,
            "connector_rrt_iters": args.connector_rrt_iters,
            "connector_rrt_timeout_ms": args.connector_rrt_timeout_ms,
            "connector_rrt_step_size": args.connector_rrt_step_size,
            "connector_rrt_goal_bias": args.connector_rrt_goal_bias,
            "bridge_failed_queries": args.bridge_failed_queries,
            "bridge_repaired_queries": args.bridge_repaired_queries,
            "corridor_refine": args.corridor_refine,
            "corridor_refine_budget_ms": args.corridor_refine_budget_ms,
            "corridor_refine_max_boxes": args.corridor_refine_max_boxes,
            "corridor_refine_boxes_per_query": args.corridor_refine_boxes_per_query,
            "corridor_refine_passes": args.corridor_refine_passes,
            "corridor_refine_start_margin_ms": args.corridor_refine_start_margin_ms,
            "corridor_refine_defer_labels": args.corridor_refine_defer_labels,
            "collision_shortcut": args.collision_shortcut,
            "collision_shortcut_resolution": args.collision_shortcut_resolution,
            "strict_path_audit": args.strict_path_audit,
            "audit_resolution": args.audit_resolution,
            "repair_on_audit_failure": args.repair_on_audit_failure,
            "repair_max_attempts": args.repair_max_attempts,
            "repair_rrt_max_iters": args.repair_rrt_max_iters,
            "repair_timeout_ms": args.repair_timeout_ms,
            "repair_local_sampling_radius": args.repair_local_sampling_radius,
            "repair_local_sampling_growth": args.repair_local_sampling_growth,
            "validation_cache": args.validation_cache,
            "validation_cache_max_entries": args.validation_cache_max_entries,
        },
        "build": {
            "mean_s": mean([float(trial["build_s"]) for trial in trials]),
            "median_s": median([float(trial["build_s"]) for trial in trials]),
            "mean_unique_box_count": mean([float(trial["unique_box_count"]) for trial in trials]),
            "median_unique_box_count": median([float(trial["unique_box_count"]) for trial in trials]),
            "mean_prebridge_time_s": mean([float(trial["prebridge_time_s"]) for trial in trials]),
            "median_prebridge_time_s": median([float(trial["prebridge_time_s"]) for trial in trials]),
            "mean_prebridge_added_boxes": mean([float(trial["prebridge_added_boxes"]) for trial in trials]),
            "median_prebridge_added_boxes": median([float(trial["prebridge_added_boxes"]) for trial in trials]),
            "mean_dedup_box_volume_sum": mean([float(trial["dedup_box_volume_sum"]) for trial in trials]),
            "median_dedup_box_volume_sum": median([float(trial["dedup_box_volume_sum"]) for trial in trials]),
            "mean_segment_edge_count": mean([float(trial["segment_edge_count"]) for trial in trials]),
            "median_segment_edge_count": median([float(trial["segment_edge_count"]) for trial in trials]),
            "mean_provisional_box_count": mean([float(trial["provisional_box_count"]) for trial in trials]),
            "mean_certified_box_count": mean([float(trial["certified_box_count"]) for trial in trials]),
        },
        "queries": [] if args.grow_only else summarize_queries(trials, query_labels),
        "trials": trials,
    }
    if args.v6_json is not None:
        payload["comparison_to_v6"] = compare_to_v6(payload, args.v6_json)
    payload["baseline_migration"] = run_exp4_baselines(args)
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "out_json": str(args.out_json),
        "build": payload["build"],
        "queries": payload["queries"],
        "comparison_to_v6": payload.get("comparison_to_v6"),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())