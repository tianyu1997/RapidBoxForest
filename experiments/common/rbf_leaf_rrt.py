from __future__ import annotations

import math
import os
import random
import shutil
import time
import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from experiments.common.metrics import mean, median
from experiments.common.rbf_defaults import (
    CANONICAL_SYMMETRY_DESCRIPTOR,
    DEFAULT_RBF_AUDIT_RESOLUTION,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES,
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_PAVE_DEPTH,
    DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS,
    DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN,
    DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN,
    DEFAULT_RBF_CONNECTOR_PAVE_STEPS,
    DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION,
    DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FFB_DEPTHS,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_TARGET_SEGMENT_FRACTION,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_PARTITION_APPEND_BATCH_SIZE,
    DEFAULT_RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED,
    DEFAULT_RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS,
    DEFAULT_RBF_QUERY_BRIDGE_LOCAL_SAMPLE_ASSIMILATION,
    DEFAULT_RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP,
    DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET,
    DEFAULT_RBF_QUERY_BRIDGE_FORCE_SELECTED,
    DEFAULT_RBF_QUERY_BRIDGE_FORCE_INDICES,
    DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS,
    DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS,
    DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH,
    DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS,
    DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS,
    DEFAULT_RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE,
    DEFAULT_RBF_QUERY_BRIDGE_RRT_OPTIMIZE_AFTER_FIRST_ITERS,
    DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_FALLBACK_PATHS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS,
    DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY,
    DEFAULT_RBF_QUERY_ENDPOINT_ANCHOR_BEFORE_BRIDGE,
    DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS,
    DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY,
    DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
    DEFAULT_RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS,
    DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS,
    DEFAULT_RBF_CONNECTOR_RRT_ITERS,
    DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE,
    DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION,
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_DEEP_MAX_BOXES,
    DEFAULT_RBF_MAX_DEPTH,
    DEFAULT_RBF_DOMAIN_ATTEMPT_CAP,
    DEFAULT_RBF_DOMAIN_SEED_CAP,
    DEFAULT_RBF_DOMAIN_SUCCESS_CAP,
    DEFAULT_RBF_FINAL_COLLISION_SHORTCUT,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
    DEFAULT_RBF_QUERY_BRIDGE_ALL,
    DEFAULT_RBF_QUERY_BRIDGE_LABELS,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES,
    DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_REFINE_TIMEOUT_MS,
    DEFAULT_RBF_THREADS,
    DEFAULT_RBF_VALIDATION_BATCH_SIZE,
    robot_joint_limit_tuples,
    robot_symmetry_aligned_root_tuples,
)
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


@dataclass(frozen=True)
class QuerySpec:
    label: str
    start: list[float]
    goal: list[float]
    actual_start: list[float] | None = None
    actual_goal: list[float] | None = None


SHELF_QUERY_GLOBAL_INDEX = {
    "AS->TS": 0,
    "TS->CS": 1,
    "CS->LB": 2,
    "LB->RB": 3,
    "RB->AS": 4,
}


def stable_query_index(label: str, fallback: int) -> int:
    return int(SHELF_QUERY_GLOBAL_INDEX.get(str(label), int(fallback)))


def _diagnostic_number(diagnostics: dict[str, float], key: str, default: float = 0.0) -> float:
    value = diagnostics.get(key, default)
    if ("_ms" in key or key.endswith(".ms")):
        profile_total_key = f"profile.{key}.total_ms"
        profile_value = diagnostics.get(profile_total_key)
        if profile_value is not None:
            try:
                if key not in diagnostics or abs(float(value)) <= 1e-15:
                    value = profile_value
            except (TypeError, ValueError):
                value = profile_value
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _diagnostic_max(diagnostics: dict[str, float], keys: list[str]) -> float:
    value = 0.0
    for key in keys:
        value = max(value, _diagnostic_number(diagnostics, key, 0.0))
    return value


def _diagnostic_sum_suffix(diagnostics: dict[str, float], suffix: str) -> float:
    return float(sum(
        float(value)
        for key, value in diagnostics.items()
        if str(key).endswith(suffix)
    ))


def _diagnostic_max_suffix(diagnostics: dict[str, float], suffix: str) -> float:
    value = 0.0
    for key, item in diagnostics.items():
        if str(key).endswith(suffix):
            try:
                value = max(value, float(item))
            except (TypeError, ValueError):
                continue
    return value


def _diagnostic_first_suffix(
    diagnostics: dict[str, float],
    suffix: str,
    default: float = math.nan,
) -> float:
    for key in sorted(diagnostics):
        if str(key).endswith(suffix):
            try:
                return float(diagnostics[key])
            except (TypeError, ValueError):
                return float(default)
    return float(default)


def _external_evidence_diagnostic_fields(diagnostics: dict[str, float]) -> dict[str, float]:
    reused_keys = [
        "oracle.materialization_reused_external_evidence",
        "adaptive.oracle.materialization_reused_external_evidence",
        "adaptive.external_reused_hits_normalized",
        "leaf_sweep.worker_oracle.materialization_reused_external_evidence",
        "grower.worker_oracle.materialization_reused_external_evidence",
    ]
    exact_hit_keys = [
        "oracle.materialization_external_exact_hits",
        "adaptive.oracle.materialization_external_exact_hits",
        "adaptive.external_exact_hits_normalized",
        "leaf_sweep.worker_oracle.materialization_external_exact_hits",
        "grower.worker_oracle.materialization_external_exact_hits",
    ]
    exact_miss_keys = [
        "oracle.materialization_external_exact_misses",
        "adaptive.oracle.materialization_external_exact_misses",
        "adaptive.external_exact_misses_normalized",
        "leaf_sweep.worker_oracle.materialization_external_exact_misses",
        "grower.worker_oracle.materialization_external_exact_misses",
    ]
    replay_check_keys = [
        "oracle.interval_replay_compatibility_checks",
        "adaptive.oracle.interval_replay_compatibility_checks",
        "adaptive.interval_replay_compatibility_checks_normalized",
        "leaf_sweep.worker_oracle.interval_replay_compatibility_checks",
        "grower.worker_oracle.interval_replay_compatibility_checks",
    ]
    replay_compatible_keys = [
        "oracle.interval_replay_compatible",
        "adaptive.oracle.interval_replay_compatible",
        "adaptive.interval_replay_compatible_normalized",
        "leaf_sweep.worker_oracle.interval_replay_compatible",
        "grower.worker_oracle.interval_replay_compatible",
    ]
    replay_incompatible_keys = [
        "oracle.interval_replay_incompatible",
        "adaptive.oracle.interval_replay_incompatible",
        "adaptive.interval_replay_incompatible_normalized",
        "leaf_sweep.worker_oracle.interval_replay_incompatible",
        "grower.worker_oracle.interval_replay_incompatible",
    ]
    replay_direct_hit_keys = [
        "oracle.interval_replay_direct_exact_hits",
        "adaptive.oracle.interval_replay_direct_exact_hits",
        "adaptive.interval_replay_direct_exact_hits_normalized",
        "leaf_sweep.worker_oracle.interval_replay_direct_exact_hits",
        "grower.worker_oracle.interval_replay_direct_exact_hits",
    ]
    replay_key_only_blocked_keys = [
        "oracle.interval_replay_key_only_blocked",
        "adaptive.oracle.interval_replay_key_only_blocked",
        "adaptive.interval_replay_key_only_blocked_normalized",
        "leaf_sweep.worker_oracle.interval_replay_key_only_blocked",
        "grower.worker_oracle.interval_replay_key_only_blocked",
    ]
    reused_hits = _diagnostic_max(diagnostics, reused_keys)
    exact_hits = _diagnostic_max(diagnostics, exact_hit_keys)
    exact_misses = _diagnostic_max(diagnostics, exact_miss_keys)
    return {
        "external_hits": reused_hits,
        "external_reused_hits": reused_hits,
        "external_exact_hits": exact_hits,
        "external_exact_misses": exact_misses,
        "interval_replay_compatibility_checks": _diagnostic_max(diagnostics, replay_check_keys),
        "interval_replay_compatible": _diagnostic_max(diagnostics, replay_compatible_keys),
        "interval_replay_incompatible": _diagnostic_max(diagnostics, replay_incompatible_keys),
        "interval_replay_direct_exact_hits": _diagnostic_max(diagnostics, replay_direct_hit_keys),
        "interval_replay_key_only_blocked": _diagnostic_max(diagnostics, replay_key_only_blocked_keys),
    }


def _query_bridge_diagnostic_fields(
    diagnostics: dict[str, float],
    query_count: int,
    max_task_index: int = 16,
) -> dict[str, float | int]:
    fields: dict[str, float | int] = {}
    scalar_keys = [
        "query_bridge.batch_total_ms",
        "query_bridge.batch_rrt_ms_total",
        "query_bridge.batch_probe_ms_total",
        "query_bridge.batch_pave_ms_total",
        "query_bridge.batch_segment_only_retry_ms_total",
        "query_bridge.batch_no_path_retry_ms_total",
        "query_bridge.batch_no_path_retry_adaptive_ms_total",
        "query_bridge.batch_no_path_retry_adaptive_attempts",
        "query_bridge.batch_no_path_retry_adaptive_successes",
        "query_bridge.batch_tasks_initial",
        "query_bridge.batch_tasks_attempted",
        "query_bridge.batch_tasks_skipped",
        "query_bridge.batch_tasks_skipped_after_rrt",
        "query_bridge.batch_tasks_no_path",
        "query_bridge.batch_tasks_segment_only",
        "query_bridge.parallel_task_rrt",
        "query_bridge.parallel_task_rrt_jobs",
        "query_bridge.rrt_fixed_iters",
        "query_bridge.rrt_fixed_timeout_ms",
        "query_bridge.local_radius_schedule_size",
        "query_bridge.rrt_optimize_after_first_iters",
        "query_bridge.attempt_fallback_paths",
        "query_bridge.attempt_fallback_paths_stored",
        "query_bridge.hybridize_attempt_paths_tasks",
        "query_bridge.hybridize_attempt_paths_candidates",
        "query_bridge.hybridize_attempt_paths_accepts",
        "query_bridge.hybridize_attempt_paths_delta",
        "query_bridge.hybridize_attempt_paths_audit_rejects",
        "query_bridge.no_path_retry_budget_stages",
        "query_bridge.oracle_node_validations",
        "query_bridge.oracle_validation_cache_hits",
        "query_bridge.oracle_validation_cache_misses",
        "query_bridge.oracle_materializations",
        "query_bridge.oracle_external_exact_hits",
        "query_bridge.oracle_external_exact_misses",
        "query_bridge.oracle_interval_replay_compatibility_checks",
        "query_bridge.oracle_interval_replay_compatible",
        "query_bridge.oracle_interval_replay_incompatible",
        "query_bridge.oracle_interval_replay_direct_exact_hits",
        "query_bridge.oracle_interval_replay_key_only_blocked",
        "query_bridge.oracle_shared_endpoint_cache_hits",
        "query_bridge.oracle_endpoint_path_ms",
        "query_bridge.oracle_classify_ms",
        "query_bridge.oracle_validate_total_ms",
        "query_bridge.oracle_materialization_endpoint_ms",
        "query_bridge.oracle_materialization_envelope_ms",
        "query_bridge.oracle_envelope_collision_queries",
        "query_bridge.oracle_envelope_gjk_tests",
        "query_bridge.direct_line_on_no_path",
        "query_bridge.direct_line_on_no_path_attempts",
        "query_bridge.direct_line_on_no_path_successes",
        "query_bridge.direct_line_on_no_path_rejects",
        "query_bridge.detour_on_no_path",
        "query_bridge.detour_candidate",
        "query_bridge.detour_on_no_path_attempts",
        "query_bridge.detour_on_no_path_successes",
        "query_bridge.detour_on_no_path_candidates",
        "query_bridge.detour_on_no_path_rejects",
        "query_bridge.detour_candidate_selected",
        "query_bridge.detour_candidate_not_shorter",
        "query_bridge.direct_segment_after_rrt",
        "query_bridge.direct_segment_after_rrt_min_length",
        "query_bridge.direct_segment_after_rrt_edges",
        "query_bridge.direct_segment_after_rrt_audit_rejects",
        "query_bridge.direct_segment_after_rrt_add_fail",
        "query_bridge.waypoint_quality_retry",
        "query_bridge.waypoint_quality_retry_tasks",
        "query_bridge.waypoint_quality_retry_attempts",
        "query_bridge.waypoint_quality_retry_successes",
        "query_bridge.waypoint_quality_retry_fixed",
        "query_bridge.waypoint_quality_retry_ms_total",
        "query_bridge.direct_corridor_ms",
        "query_bridge.direct_corridor_ms_total",
        "query_bridge.direct_corridor_samples",
        "query_bridge.direct_corridor_samples_total",
        "query_bridge.direct_corridor_ffb_calls",
        "query_bridge.direct_corridor_ffb_calls_total",
        "query_bridge.direct_corridor_all_ffb_calls",
        "query_bridge.direct_corridor_all_ffb_calls_total",
        "query_bridge.direct_corridor_ffb_start_depth",
        "query_bridge.direct_corridor_coverage_order_direct_tasks",
        "query_bridge.direct_corridor_center_out_direct_tasks",
        "query_bridge.direct_corridor_direct_ffb_ms",
        "query_bridge.direct_corridor_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_repair_priority",
        "query_bridge.direct_corridor_adaptive_repair_target_segment_fraction",
        "query_bridge.direct_corridor_adaptive_repair_target_stops",
        "query_bridge.direct_corridor_adaptive_initial_bad_fraction",
        "query_bridge.direct_corridor_adaptive_final_bad_fraction",
        "query_bridge.direct_corridor_lateral_repair_ffb_ms",
        "query_bridge.direct_corridor_segment_audit_ms",
        "query_bridge.direct_corridor_mark_initial_ms",
        "query_bridge.direct_corridor_mark_incremental_ms",
        "query_bridge.direct_corridor_initialize_dsu_ms",
        "query_bridge.direct_corridor_assimilate_ms",
        "query_bridge.direct_corridor_detailed_timing_enabled",
        "query_bridge.direct_corridor_transition_connected_ms",
        "query_bridge.direct_corridor_transition_connected_calls",
        "query_bridge.direct_corridor_bad_transitions_ms",
        "query_bridge.direct_corridor_bad_transitions_calls",
        "query_bridge.direct_corridor_current_cover_ms",
        "query_bridge.direct_corridor_current_cover_calls",
        "query_bridge.direct_corridor_current_cover_partition_ms",
        "query_bridge.direct_corridor_current_cover_corridor_scan_ms",
        "query_bridge.direct_corridor_current_cover_direct_index_ms",
        "query_bridge.direct_corridor_duplicate_lookup_ms",
        "query_bridge.direct_corridor_duplicate_lookup_calls",
        "query_bridge.direct_corridor_commit_total_ms",
        "query_bridge.direct_corridor_commit_calls",
        "query_bridge.direct_corridor_commit_dynamic_policy_ms",
        "query_bridge.direct_corridor_commit_partition_append_ms",
        "query_bridge.direct_corridor_partition_append_calls",
        "query_bridge.direct_corridor_partition_append_boxes",
        "query_bridge.direct_corridor_assimilate_calls",
        "query_bridge.direct_corridor_assimilate_sample_scan_ms",
        "query_bridge.direct_corridor_assimilate_local_hits",
        "query_bridge.direct_corridor_assimilate_full_scan_fallbacks",
        "query_bridge.direct_corridor_assimilate_local_sample_tests",
        "query_bridge.direct_corridor_assimilate_candidate_build_ms",
        "query_bridge.direct_corridor_assimilate_adjacency_ms",
        "query_bridge.direct_corridor_segment_insert_ms",
        "query_bridge.direct_corridor_segment_insert_calls",
        "query_bridge.direct_corridor_direct_task_build_ms",
        "query_bridge.direct_corridor_direct_loop_ms",
        "query_bridge.direct_corridor_repair_loop_ms",
        "query_bridge.direct_corridor_adaptive_loop_ms",
        "query_bridge.direct_corridor_lateral_loop_ms",
        "query_bridge.direct_corridor_residual_segment_loop_ms",
        "query_bridge.direct_corridor_partition_neighbor_candidates",
        "query_bridge.direct_corridor_added",
        "query_bridge.direct_corridor_added_total",
        "query_bridge.direct_corridor_repair_calls",
        "query_bridge.direct_corridor_repair_calls_total",
        "query_bridge.direct_corridor_repair_added",
        "query_bridge.direct_corridor_repair_added_total",
        "query_bridge.direct_corridor_adaptive_repair_calls",
        "query_bridge.direct_corridor_adaptive_repair_calls_total",
        "query_bridge.direct_corridor_adaptive_repair_added",
        "query_bridge.direct_corridor_adaptive_repair_added_total",
        "query_bridge.direct_corridor_lateral_repair_enabled",
        "query_bridge.direct_corridor_lateral_repair_calls",
        "query_bridge.direct_corridor_lateral_repair_calls_total",
        "query_bridge.direct_corridor_lateral_repair_added",
        "query_bridge.direct_corridor_lateral_repair_added_total",
        "query_bridge.direct_corridor_bad_initial",
        "query_bridge.direct_corridor_bad_initial_total",
        "query_bridge.direct_corridor_bad_final",
        "query_bridge.direct_corridor_bad_final_total",
        "query_bridge.direct_corridor_assimilate_covered_samples",
        "query_bridge.direct_corridor_assimilate_coverage_span_max",
        "query_bridge.direct_corridor_assimilate_coverage_span_mean",
        "query_bridge.direct_corridor_segment_edges",
        "query_bridge.direct_corridor_segment_edges_total",
        "query_bridge.direct_corridor_local_connected",
        "query_bridge.direct_corridor_local_residual_overlay_connected",
        "query_bridge.direct_corridor_full_residual_without_local_overlay",
        "query_bridge.direct_corridor_full_residual_audit_rejects",
        "query_bridge.direct_corridor_full_residual_edges",
        "query_bridge.direct_corridor_full_residual_edges_without_local_overlay",
        "query_bridge.direct_corridor_full_adjacency_scans_avoided",
        "query_bridge.partition_graph_dense_chain_pave_skipped",
        "query_bridge.partition_graph_forward_chain_pave_skipped",
        "query_bridge.partition_graph_reverse_chain_pave_skipped",
        "query_bridge.partition_graph_gap_connector_skipped",
        "query_bridge.full_adjacency_rebuilds_avoided",
        "query_bridge.segment_edge_audit_rejects",
        "query_bridge.segment_edge_blocked_no_max_depth_ffb_failure",
    ]
    integer_suffixes = (
        "_tasks",
        "_jobs",
        "_calls",
        "_samples",
        "_added",
        "_edges",
        "_skipped",
        "_attempted",
        "_rejects",
        "_avoided",
        "_connected",
        "_initial",
        "_final",
        "_failures",
        "_successes",
    )
    for key in scalar_keys:
        out_key = "diag_" + key.replace(".", "_")
        value = _diagnostic_number(diagnostics, key, 0.0)
        if out_key.endswith(integer_suffixes):
            fields[out_key] = int(value)
        else:
            fields[out_key] = value

    batch_ms = float(fields.get("diag_query_bridge_batch_total_ms", 0.0))
    fields["diag_query_bridge_batch_per_query_ms"] = batch_ms / max(1, int(query_count))
    for key, value in diagnostics.items():
        prefix = "query_bridge.batch_task."
        if not str(key).startswith(prefix):
            continue
        rest = str(key)[len(prefix):]
        parts = rest.split(".", 1)
        if len(parts) != 2:
            continue
        try:
            task_index = int(parts[0])
        except ValueError:
            continue
        if task_index >= max_task_index:
            continue
        metric = parts[1]
        if metric not in {
            "rrt_ms",
            "pave_ms",
            "total_ms",
            "waypoint_length",
            "skip_reason_code",
            "direct_corridor_ms",
            "direct_corridor_ffb_start_depth",
            "direct_corridor_coverage_order_direct_tasks",
            "direct_corridor_center_out_direct_tasks",
            "direct_corridor_added",
            "direct_corridor_repair_added",
            "direct_corridor_adaptive_repair_calls",
            "direct_corridor_adaptive_repair_added",
            "direct_corridor_bad_initial",
            "direct_corridor_bad_final",
            "direct_corridor_partition_append_calls",
            "direct_corridor_partition_append_boxes",
            "direct_corridor_assimilate_local_hits",
            "direct_corridor_assimilate_full_scan_fallbacks",
            "direct_corridor_assimilate_local_sample_tests",
            "direct_corridor_segment_edges",
            "direct_corridor_local_connected",
            "direct_corridor_full_residual_edge",
            "direct_line_on_no_path",
            "detour_on_no_path",
        }:
            continue
        out_key = f"diag_query_bridge_task{task_index}_{metric}"
        numeric = float(value)
        if not metric.endswith("_ms"):
            fields[out_key] = int(numeric)
        else:
            fields[out_key] = numeric
    return fields


@dataclass
class RBFLeafRRTOptions:
    seed: int = 0
    offline_grower: str = "adaptive_deep_leaf"
    deep_max_boxes: int = DEFAULT_RBF_DEEP_MAX_BOXES
    rbf_max_depth: int = DEFAULT_RBF_MAX_DEPTH
    timeout_ms: float = 8000.0
    threads: int = DEFAULT_RBF_THREADS
    leaf_start_depth: int = DEFAULT_RBF_LEAF_START_DEPTH
    leaf_max_depth: int = DEFAULT_RBF_LEAF_MAX_DEPTH
    deep_ffb_depth: int = DEFAULT_RBF_DEEP_FFB_DEPTH
    refine_timeout_ms: float = DEFAULT_RBF_REFINE_TIMEOUT_MS
    domain_seed_cap: int = DEFAULT_RBF_DOMAIN_SEED_CAP
    domain_success_cap: int = DEFAULT_RBF_DOMAIN_SUCCESS_CAP
    domain_attempt_cap: int = DEFAULT_RBF_DOMAIN_ATTEMPT_CAP
    run_rrt_grower: bool = True
    rrt_grower_extra_boxes: int = DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES
    rrt_grower_timeout_ms: float = DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS
    priority_prune_radius: float = 0.0
    collision_overlap_prune_min_depth: int = -1
    collision_overlap_prune_threshold: float = 0.0
    collision_overlap_prune_ratio_threshold: float = 0.0
    adaptive_target_depth: int = DEFAULT_RBF_LEAF_MAX_DEPTH
    adaptive_time_budget_ms: float = 60000.0
    adaptive_node_budget: int = 50000
    adaptive_fast_virtual_checkpoint_mode: bool = False
    adaptive_defer_min_depth: int = 16
    adaptive_overlap_depth_threshold: float = 0.05
    adaptive_overlap_depth_min_threshold: float = 0.01
    adaptive_overlap_depth_decay_per_depth: float = 0.04
    adaptive_overlap_ratio_threshold: float = 0.0
    adaptive_seed_probe_count: int = 4096
    adaptive_seed_probe_rng_seed: int = 20260607
    adaptive_seed_promote_uncovered: bool = True
    adaptive_seed_anchor_probe_cap: int = 256
    adaptive_promotion_interval: int = 1024
    adaptive_depth_enabled: bool = True
    adaptive_depth_min: int = DEFAULT_RBF_LEAF_MAX_DEPTH
    adaptive_depth_max: int = 16
    adaptive_depth_probe_count: int = 512
    adaptive_depth_anchor_probe_cap: int = 32
    adaptive_depth_probe_seed: int = 20260607
    adaptive_depth_min_free_probes: int = 64
    adaptive_depth_min_covered_probes: int = 3
    adaptive_depth_min_main_probes: int = 3
    adaptive_depth_min_main_ratio: float = 0.35
    adaptive_depth_min_cells: int = 0
    adaptive_depth_min_main_cells: int = 0
    adaptive_depth_max_online_cells: int = 180
    adaptive_depth_max_probe_ms: float = 5.0
    adaptive_max_merge_ms: float = 1500.0
    adaptive_max_merge_rounds: int = 2
    adaptive_max_merge_input_boxes: int = 100000
    adaptive_max_free_boxes: int = 50000
    adaptive_max_unresolved_domains: int = 100000
    adaptive_planning_backend: str = "partition_native"
    adaptive_grid_target_depth: int = 0
    adaptive_grid_face_index_enabled: bool = True
    adaptive_grid_planning_max_expansions: int = 0
    hipac_portal_connectivity: bool = False
    hipac_portal_cell_native_validate: bool = True
    hipac_portal_max_internal_boxes: int = 64
    hipac_portal_max_recursion_depth: int = 8
    hipac_portal_ffb_depth: int = 0
    hipac_portal_ffb_deadline_ms: float = 5.0
    hipac_online_connectivity: bool = False
    hipac_online_before_query_bridge: bool = True
    hipac_promote_query_repairs: bool = False
    hipac_online_ffb_portal_fallback: bool = False
    hipac_online_candidate_max_length: float = 3.0
    hipac_online_max_resolves_per_query: int = 1
    hipac_online_max_hidden_boxes_per_portal: int = 32
    hipac_online_max_ffb_calls_per_portal: int = 64
    hipac_online_prebridge_portal: bool = False
    hipac_online_prebridge_candidate_limit: int = 32
    hipac_online_prebridge_max_pair_distance: float = 1.25
    hipac_online_prebridge_route_distance_weight: float = 1.0
    hipac_online_prebridge_pair_distance_weight: float = 0.25
    hipac_online_transition_portal: bool = False
    hipac_transition_target_query_indices: str = "2,3"
    hipac_transition_max_attempts_per_query: int = 1
    hipac_transition_candidate_limit: int = 16
    hipac_transition_window_stride: int = 2
    hipac_transition_min_predicted_bridge_edges: int = 16
    hipac_transition_max_pair_distance: float = 1.50
    hipac_transition_allow_same_component: bool = True
    hipac_transition_obb_portal: bool = False
    hipac_transition_obb_lateral_radius: float = 0.01
    hipac_transition_obb_longitudinal_margin: float = 0.0
    hipac_transition_obb_safety_epsilon: float = 0.0
    segment_edge_obb_cover: bool = False
    rrt_bridge_obb_cover: bool = False
    strict_obb_bridge_cover: bool = False
    segment_edge_obb_metadata_only: bool = False
    segment_edge_obb_metadata_require_cover: bool = False
    segment_edge_obb_lateral_radius: float = 0.01
    segment_edge_obb_longitudinal_margin: float = 0.0
    segment_edge_obb_safety_epsilon: float = 0.0
    segment_edge_obb_grow_iterations: int = 5
    segment_edge_obb_binary_iterations: int = 5
    segment_edge_obb_split_depth: int = 1
    obb_max_window_segments: int = 16
    obb_max_validations_per_window: int = 16
    obb_fast_primary_orientation: bool = True
    obb_fallback_orientations_on_primary_fail: bool = False
    hipac_promote_transition_slices: bool = False
    hipac_promote_transition_target_query_indices: str = "2,3"
    hipac_promote_transition_min_boxes: int = 8
    hipac_promote_transition_max_boxes: int = 64
    hipac_promote_transition_max_attempts_per_query: int = 1
    validation_batch_size: int = DEFAULT_RBF_VALIDATION_BATCH_SIZE
    split_schedule_kind: str = "aafk_volume_min"
    ffb_start_depth: int = DEFAULT_RBF_FFB_START_DEPTH
    query_bridge_ffb_start_depth: int = -1
    ffb_search_mode: str = DEFAULT_RBF_FFB_SEARCH_MODE
    audit_resolution: int = DEFAULT_RBF_AUDIT_RESOLUTION
    audit_segment_step: float = DEFAULT_RBF_AUDIT_SEGMENT_STEP
    audit_collision_tolerance: float = DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE
    query_shortcut_boxes: bool = False
    use_virtual_topology: bool = True
    parallel_virtual_validation: bool = True
    leaf_threads: int = DEFAULT_RBF_THREADS
    envelope: str = "support_hull"
    support_hull_skip_aabb_broadphase: bool = False
    endpoint_source: str = "ifk"
    hifk_max_depth: int = 9
    unsafe_sampling_validation: bool = False
    use_external_evidence: bool = False
    external_evidence_live_retry_on_maybe: bool = False
    active_endpoint_evidence_cache: bool = False
    active_store_endpoint_evidence_cache: bool = False
    worker_shared_endpoint_cache: bool = False
    external_evidence_path: Path | None = None
    external_evidence_verify_identity: bool = True
    use_shelf_root_override: bool = False
    root_override_tuples: list[tuple[float, float]] | None = None
    coverage_override_tuples: list[tuple[float, float]] | None = None
    symmetry_aligned_native_root: bool = False
    symmetry_aligned_cache_schedule: bool = False
    database_canonical_mode: bool = True
    case_label: str = "rbf_leaf_rrt"
    segment_edges_fallback_only: bool = False
    connector_birrt: bool = True
    connector_bridge_boxes: int = DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES
    connector_pair_batch_size: int = 1
    connector_pair_timeout_ms: float = DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS
    connector_max_pairs_per_gap: int = DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP
    connector_rrt_iters: int = DEFAULT_RBF_CONNECTOR_RRT_ITERS
    connector_rrt_timeout_ms: float = DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS
    connector_rrt_step_size: float = DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE
    connector_rrt_goal_bias: float = DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS
    connector_rrt_local_sampling_radius: float = 0.0
    connector_segment_resolution: int = DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION
    connector_pave_max_chain: int = DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN
    connector_pave_steps: int = DEFAULT_RBF_CONNECTOR_PAVE_STEPS
    connector_pave_depth: int = DEFAULT_RBF_CONNECTOR_PAVE_DEPTH
    connector_adaptive_min_segment_fraction: float = DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION
    query_bridge_pave_depth: int = DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH
    query_endpoint_anchor_ffb_depth: int = 0
    query_bridge_adaptive_ffb_depths: str = DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FFB_DEPTHS
    connector_pave_fill_gaps: bool = DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS
    connector_pave_require_connected_chain: bool = DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN
    final_collision_shortcut: bool = DEFAULT_RBF_FINAL_COLLISION_SHORTCUT
    final_rrt_simplify: bool = DEFAULT_RBF_FINAL_RRT_SIMPLIFY
    final_rrt_simplify_timeout_ms: float = DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS
    final_rrt_simplify_max_iters: int = DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS
    final_rrt_simplify_attempts: int = DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS
    corridor_refine: bool = False
    corridor_refine_budget_ms: float = 0.0
    corridor_refine_max_boxes: int = 0
    corridor_refine_boxes_per_query: int = 12
    corridor_refine_passes: int = 1
    corridor_refine_start_margin_ms: float = 0.0
    corridor_refine_mode: str = "box_only_long_path"
    corridor_refine_long_path_ratio: float = 1.25
    corridor_refine_min_delta: float = 0.25
    query_bridge_all: bool = DEFAULT_RBF_QUERY_BRIDGE_ALL
    query_bridge_adaptive_all: bool = True
    query_bridge_adaptive_max_path_length: float = 4.5
    query_bridge_accept_segment_fraction: float = 0.25
    query_bridge_accept_path_ratio: float = 1.50
    query_bridge_accept_path_additive: float = 0.75
    query_bridge_sequential_reuse: bool = False
    query_bridge_scene_reusable_edges: bool = False
    query_endpoint_anchor_before_bridge: bool = DEFAULT_RBF_QUERY_ENDPOINT_ANCHOR_BEFORE_BRIDGE
    query_bridge_labels: str = DEFAULT_RBF_QUERY_BRIDGE_LABELS
    query_bridge_segment_only_indices: str = ""
    query_bridge_force_indices: str = DEFAULT_RBF_QUERY_BRIDGE_FORCE_INDICES
    query_bridge_force_selected: bool = DEFAULT_RBF_QUERY_BRIDGE_FORCE_SELECTED
    query_bridge_forced_attempts: int = DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS
    query_bridge_attempt_offset: int = DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET
    query_bridge_no_path_retry_attempts: int = DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS
    query_bridge_no_path_retry_stop_on_first_success: bool = (
        DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS
    )
    query_bridge_no_path_retry_budget_iters: str = ""
    query_bridge_no_path_retry_budget_attempts: str = ""
    query_bridge_waypoint_quality_retry: bool = False
    query_bridge_waypoint_quality_retry_attempts: int = 4
    query_bridge_waypoint_quality_retry_iters: int = 0
    query_bridge_waypoint_quality_max_ratio: float = 2.0
    query_bridge_waypoint_quality_max_additive: float = 0.75
    query_bridge_rrt_fixed_iters: int = DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS
    query_bridge_rrt_fixed_timeout_ms: float = DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS
    query_bridge_local_radius_schedule: str = DEFAULT_RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE
    query_bridge_rrt_optimize_after_first_iters: int = DEFAULT_RBF_QUERY_BRIDGE_RRT_OPTIMIZE_AFTER_FIRST_ITERS
    query_bridge_attempt_fallback_paths: int = DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_FALLBACK_PATHS
    query_bridge_hybridize_attempt_paths: bool = DEFAULT_RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS
    query_bridge_hybrid_max_paths: int = DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS
    query_bridge_hybrid_max_vertices: int = DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES
    query_bridge_hybrid_max_cross_checks: int = DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS
    query_bridge_parallel_rrt_early_stop: bool = DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP
    query_bridge_parallel_rrt_early_stop_min_successes: int = (
        DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES
    )
    query_bridge_parallel_rrt_early_stop_ratio: float = (
        DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO
    )
    query_bridge_parallel_rrt_early_stop_additive: float = (
        DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE
    )
    query_bridge_direct_segment_after_rrt: bool = DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT
    query_bridge_direct_segment_after_rrt_min_length: float = (
        DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH
    )
    query_bridge_fast_direct_segment_after_rrt: bool = False
    query_bridge_fast_direct_random_shortcut_iters: int = 0
    query_endpoint_point_anchor: bool = False
    query_bridge_direct_sample_step: float = DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP
    query_bridge_repair_subdivisions: int = DEFAULT_RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS
    query_bridge_adaptive_step_repair: bool = DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR
    query_bridge_adaptive_fine_step: float = DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP
    query_bridge_adaptive_max_repair_subdivisions: int = DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS
    query_bridge_adaptive_max_repair_calls: int = DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS
    query_bridge_adaptive_repair_priority: int = DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY
    query_bridge_adaptive_repair_target_segment_fraction: float = (
        DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_TARGET_SEGMENT_FRACTION
    )
    query_bridge_group_residual_gaps: bool = DEFAULT_RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS
    query_bridge_full_residual_overlay_when_connected: bool = (
        DEFAULT_RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED
    )
    query_bridge_partition_neighbor_candidates: bool = DEFAULT_RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES
    query_bridge_direct_append_partition_immediate: bool = DEFAULT_RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE
    query_bridge_local_sample_assimilation: bool = DEFAULT_RBF_QUERY_BRIDGE_LOCAL_SAMPLE_ASSIMILATION
    query_bridge_direct_partition_append_batch_size: int = (
        DEFAULT_RBF_QUERY_BRIDGE_DIRECT_PARTITION_APPEND_BATCH_SIZE
    )
    query_box_transition_line_deviation_penalty: float = DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY
    query_foreign_edge_cost_penalty: float = DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY
    query_bridge_edge_cost_penalty: float = DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY
    query_bridge_direct_max_length: float = 6.5
    query_bridge_to_main_island: bool = False
    query_bridge_failure_fallback_to_main: bool = False
    query_bridge_to_main_direct_segment_max_length: float = 0.0
    query_bridge_to_main_box_corridor: bool = True
    endpoint_main_target_k: int = 8
    endpoint_main_coarse_step: float = 0.08
    endpoint_main_fine_step: float = 0.02
    endpoint_main_max_ffb_calls: int = 48
    endpoint_main_max_boxes: int = 64
    endpoint_main_adaptive_ffb_depths: str = ""
    endpoint_main_residual_segment_max_length: float = 0.25
    endpoint_main_lateral_offset: float = 0.03
    endpoint_main_lateral_rounds: int = 2
    endpoint_main_face_epsilon: float = 1e-6
    allow_anchor_roots: bool = True
    use_priority_points: bool = True
    offline_coverage_profile: str = ""
    offline_query_agnostic_build: bool = True
    offline_random_anchors: bool = DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS
    offline_anchor_count: int = 16
    offline_anchor_candidate_count: int = 512
    offline_anchor_sampling: str = "random"
    offline_anchor_lca_lambda: float = 0.35
    offline_anchor_distance_mu: float = 0.10
    offline_anchor_skip_if_main_accessible: bool = False
    offline_anchor_main_accessible_threshold: float = 0.95
    offline_shortcut_edges: int = 0
    offline_connector_mode: str = "box_only"
    offline_shortcut_candidate_limit: int = 48
    offline_shortcut_min_gain_ratio: float = 1.6
    offline_shortcut_max_segment_length: float = 3.0
    canonicalize_queries: bool = False


def query_spec(query: Any) -> QuerySpec:
    if isinstance(query, QuerySpec):
        return query
    if isinstance(query, dict):
        raw_start = [float(value) for value in query.get("start", query.get("actual_start", query.get("canonical_start", [])))]
        raw_goal = [float(value) for value in query.get("goal", query.get("actual_goal", query.get("canonical_goal", [])))]
        return QuerySpec(
            label=str(query.get("label", query.get("name", "query"))),
            start=raw_start,
            goal=raw_goal,
            actual_start=[float(value) for value in query.get("actual_start", raw_start)] or None,
            actual_goal=[float(value) for value in query.get("actual_goal", raw_goal)] or None,
        )
    raw_start = [float(value) for value in getattr(query, "start")]
    raw_goal = [float(value) for value in getattr(query, "goal")]
    actual_start = getattr(query, "actual_start", raw_start)
    actual_goal = getattr(query, "actual_goal", raw_goal)
    return QuerySpec(
        label=str(getattr(query, "label", getattr(query, "name", "query"))),
        start=raw_start,
        goal=raw_goal,
        actual_start=[float(value) for value in (raw_start if actual_start is None else actual_start)],
        actual_goal=[float(value) for value in (raw_goal if actual_goal is None else actual_goal)],
    )


def canonical_q(robot: Any, q: Iterable[float]) -> list[float]:
    return [
        float(value)
        for value in sbf.canonicalize_configuration_for_robot(
            robot,
            [float(item) for item in q],
            True,
            CANONICAL_SYMMETRY_DESCRIPTOR,
        )
    ]


def query_point(robot: Any, q: Iterable[float], canonicalize: bool) -> list[float]:
    return canonical_q(robot, q) if canonicalize else [float(item) for item in q]


def canonical_priority_points(robot: Any, queries: Iterable[Any], canonicalize: bool = False) -> list[list[float]]:
    points: list[list[float]] = []
    for raw in queries:
        query = query_spec(raw)
        start = query_point(robot, query.start, canonicalize)
        goal = query_point(robot, query.goal, canonicalize)
        for alpha in (0.0, 0.25, 0.5, 0.75, 1.0):
            points.append([(1.0 - alpha) * a + alpha * b for a, b in zip(start, goal)])
    return points


def serialize_depth_dimensions(depth_dimensions: Iterable[int]) -> str:
    return ",".join(str(int(dim)) for dim in depth_dimensions)


def _normalized_split_schedule_kind(kind: str) -> str:
    key = str(kind or "aafk_volume_min").strip().lower().replace("-", "_")
    if key in {"aafk", "aafk_volume", "aafk_volume_min", "endpoint_aafk"}:
        return "aafk_volume_min"
    if key in {"support_hull", "support_hull_volume", "support_hull_volume_min", "sh", "sh_volume_min"}:
        return "support_hull_volume_min"
    raise ValueError(f"unknown LECT split schedule kind: {kind!r}")


def _volume_min_depth_schedule(
    robot: Any,
    root_intervals: Iterable[Any] | None,
    max_depth: int,
    sample_nodes_per_depth: int,
    split_schedule_kind: str,
) -> list[int]:
    kind = _normalized_split_schedule_kind(split_schedule_kind)
    schedule_fn = (
        sbf.support_hull_volume_min_depth_schedule
        if kind == "support_hull_volume_min"
        else sbf.aafk_volume_min_depth_schedule
    )
    if root_intervals is None:
        return list(schedule_fn(robot, int(max_depth), int(sample_nodes_per_depth)))
    return list(schedule_fn(robot, list(root_intervals), int(max_depth), int(sample_nodes_per_depth)))


def interval_pairs(intervals: Iterable[Any]) -> list[list[float]]:
    pairs: list[list[float]] = []
    for interval in intervals:
        if hasattr(interval, "lo") and hasattr(interval, "hi"):
            pairs.append([float(interval.lo), float(interval.hi)])
        else:
            lo, hi = interval
            pairs.append([float(lo), float(hi)])
    return pairs


def read_lect_cache_depth_dimensions(cache_path: Path | None) -> list[int]:
    if cache_path is None:
        return []
    manifest = Path(cache_path) / "manifest.json"
    if not manifest.exists():
        return []
    for line in manifest.read_text().splitlines():
        if not line.startswith("split_depth_dimensions="):
            continue
        raw = line.split("=", 1)[1].strip()
        return [int(item.strip()) for item in raw.split(",") if item.strip()]
    return []


def make_aafk_split_policy(
    robot: Any,
    max_depth: int,
    root_intervals: Iterable[Any] | None = None,
    *,
    force_dim0_first_two: bool = False,
    forced_tail_schedule: Iterable[int] | None = None,
    split_schedule_kind: str = "aafk_volume_min",
) -> Any:
    split_schedule_kind = _normalized_split_schedule_kind(split_schedule_kind)
    if force_dim0_first_two:
        tail = [int(dim) for dim in (forced_tail_schedule or [])]
        schedule_root = list(root_intervals) if root_intervals is not None else list(
            sbf.canonical_root_intervals_for_robot(
                robot,
                True,
                CANONICAL_SYMMETRY_DESCRIPTOR,
            )
        )
        if schedule_root:
            # The first two binary splits cover the four dim0 symmetry sectors.
            # The remaining schedule should match the per-sector shelf/root
            # resolution, not the widened native dim0 hull.
            schedule_root[0] = sbf.Interval(0.0, 0.5 * math.pi)
        if not tail:
            tail_depth = max(0, int(max_depth) - 2)
            tail = _volume_min_depth_schedule(robot, schedule_root, tail_depth, 8, split_schedule_kind)
        elif len(tail) + 2 < int(max_depth):
            extra_depth = int(max_depth) - 2 - len(tail)
            extra = _volume_min_depth_schedule(robot, schedule_root, extra_depth, 8, split_schedule_kind)
            tail.extend(int(dim) for dim in extra)
        schedule = [0, 0] + [int(dim) for dim in tail]
    else:
        schedule = _volume_min_depth_schedule(robot, root_intervals, int(max_depth), 8, split_schedule_kind)
    if len(schedule) < int(max_depth):
        raise RuntimeError(f"AAFKVolumeMin schedule has {len(schedule)} entries, expected {int(max_depth)}")
    split_policy = sbf.SplitPolicyDescriptor()
    split_policy.strategy = sbf.SplitStrategy.AAFKVolumeMin
    split_policy.min_width = 0.0
    split_policy.midpoint = True
    split_policy.deterministic_tie_break = True
    split_policy.depth_dimensions = [int(dim) for dim in schedule]
    split_policy.dimension_schedule_hash = str(sbf.stable_hash(serialize_depth_dimensions(schedule)))
    return split_policy


def make_aafk_split_policy_from_cache_prefix(
    robot: Any,
    max_depth: int,
    cache_depth_dimensions: Iterable[int],
    root_intervals: Iterable[Any] | None = None,
) -> Any:
    """Build an active split policy whose prefix exactly matches an external LECT cache.

    Exact evidence reuse is interval-keyed, so changing the dimension schedule
    by even one level makes every lookup miss.  When a warm cache is provided,
    the active tree must therefore inherit the cache schedule prefix and only
    generate a local tail beyond the cached depth.
    """
    schedule = [int(dim) for dim in cache_depth_dimensions]
    max_depth = int(max_depth)
    if len(schedule) < max_depth:
        tail_depth = max_depth - len(schedule)
        if root_intervals is None:
            tail = list(sbf.aafk_volume_min_depth_schedule(robot, tail_depth, 8))
        else:
            tail = list(sbf.aafk_volume_min_depth_schedule(robot, list(root_intervals), tail_depth, 8))
        schedule.extend(int(dim) for dim in tail)
    schedule = schedule[:max_depth]
    if len(schedule) < max_depth:
        raise RuntimeError(f"cached AAFK schedule has {len(schedule)} entries, expected {max_depth}")
    split_policy = sbf.SplitPolicyDescriptor()
    split_policy.strategy = sbf.SplitStrategy.AAFKVolumeMin
    split_policy.min_width = 0.0
    split_policy.midpoint = True
    split_policy.deterministic_tie_break = True
    split_policy.depth_dimensions = [int(dim) for dim in schedule]
    split_policy.dimension_schedule_hash = str(sbf.stable_hash(serialize_depth_dimensions(schedule)))
    return split_policy


def _root_tuple_list(robot: Any, options: RBFLeafRRTOptions) -> list[tuple[float, float]]:
    if options.coverage_override_tuples is not None:
        return [(float(lo), float(hi)) for lo, hi in options.coverage_override_tuples]
    return robot_joint_limit_tuples(robot)


def _active_tree_root_tuple_list(robot: Any, options: RBFLeafRRTOptions) -> list[tuple[float, float]]:
    if options.root_override_tuples is not None:
        return [(float(lo), float(hi)) for lo, hi in options.root_override_tuples]
    if bool(options.symmetry_aligned_native_root):
        return robot_symmetry_aligned_root_tuples(robot)
    return robot_joint_limit_tuples(robot)


def _normalized_distance(a: list[float], b: list[float], root: list[tuple[float, float]]) -> float:
    total = 0.0
    for index, (x, y) in enumerate(zip(a, b)):
        lo, hi = root[index]
        width = max(float(hi) - float(lo), 1e-12)
        total += ((float(x) - float(y)) / width) ** 2
    return math.sqrt(total)


def _joint_margin_score(q: list[float], root: list[tuple[float, float]]) -> float:
    score = 0.0
    for value, (lo, hi) in zip(q, root):
        width = max(float(hi) - float(lo), 1e-12)
        margin = max(min(float(value) - float(lo), float(hi) - float(value)) / width, 1e-9)
        score += math.log(margin)
    return score / max(1, len(root))


def _lca_depth_for_points(
    a: list[float],
    b: list[float],
    root: list[tuple[float, float]],
    schedule: list[int],
) -> int:
    intervals = [[float(lo), float(hi)] for lo, hi in root]
    for depth, dim in enumerate(schedule):
        if dim < 0 or dim >= len(intervals):
            return depth
        lo, hi = intervals[dim]
        mid = 0.5 * (lo + hi)
        a_hi = float(a[dim]) >= mid
        b_hi = float(b[dim]) >= mid
        if a_hi != b_hi:
            return depth
        if a_hi:
            intervals[dim][0] = mid
        else:
            intervals[dim][1] = mid
    return len(schedule)


def _halton_value(index: int, base: int) -> float:
    value = 0.0
    factor = 1.0 / float(base)
    current = int(index)
    while current > 0:
        value += factor * float(current % base)
        current //= base
        factor /= float(base)
    return value


def _halton_point(index: int, dim: int) -> list[float]:
    primes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47]
    if dim > len(primes):
        raise ValueError(f"Halton sampler supports at most {len(primes)} dimensions, got {dim}")
    return [_halton_value(index, primes[i]) for i in range(dim)]


def empty_offline_anchor_metrics(reason: str) -> dict[str, Any]:
    return {
        "offline_anchor_candidates": 0,
        "offline_anchor_candidates_free": 0,
        "offline_anchor_roots_requested": 0,
        "offline_anchor_lca_depth_mean": math.nan,
        "offline_anchor_lca_depth_max": math.nan,
        "offline_anchor_min_distance_mean": math.nan,
        "offline_anchor_skip_reason": str(reason),
        "offline_anchor_skip_p_main_accessible": math.nan,
    }


def generate_offline_anchor_points(
    robot: Any,
    obstacles: list[Any],
    options: RBFLeafRRTOptions,
) -> tuple[list[list[float]], dict[str, Any]]:
    count = max(0, int(options.offline_anchor_count))
    candidate_count = max(0, int(options.offline_anchor_candidate_count))
    if not bool(options.offline_random_anchors) or count <= 0 or candidate_count <= 0:
        return [], empty_offline_anchor_metrics("disabled")
    coverage_root = _root_tuple_list(robot, options)
    tree_root = _active_tree_root_tuple_list(robot, options)
    cache_schedule = (
        read_lect_cache_depth_dimensions(options.external_evidence_path)
        if (bool(options.use_external_evidence) or bool(options.symmetry_aligned_cache_schedule))
        and options.external_evidence_path is not None
        else []
    )
    split_schedule_kind = _normalized_split_schedule_kind(getattr(options, "split_schedule_kind", "aafk_volume_min"))
    if cache_schedule and split_schedule_kind == "aafk_volume_min":
        split_policy = make_aafk_split_policy_from_cache_prefix(
            robot,
            int(options.rbf_max_depth),
            cache_schedule,
            [sbf.Interval(float(lo), float(hi)) for lo, hi in tree_root],
        )
    else:
        forced_tail_schedule = (
            read_lect_cache_depth_dimensions(options.external_evidence_path)
            if bool(options.symmetry_aligned_cache_schedule)
            else []
        )
        if (
            bool(options.symmetry_aligned_cache_schedule)
            and len(forced_tail_schedule) >= 2
            and int(forced_tail_schedule[0]) == 0
            and int(forced_tail_schedule[1]) == 0
        ):
            forced_tail_schedule = forced_tail_schedule[2:]
        split_policy = make_aafk_split_policy(
            robot,
            int(options.rbf_max_depth),
            [sbf.Interval(float(lo), float(hi)) for lo, hi in tree_root],
            force_dim0_first_two=bool(options.symmetry_aligned_cache_schedule),
            forced_tail_schedule=forced_tail_schedule,
            split_schedule_kind=split_schedule_kind,
        )
    schedule = [int(dim) for dim in list(split_policy.depth_dimensions)]
    rng = random.Random((int(options.seed) + 1) * 1000003)
    sampling = str(getattr(options, "offline_anchor_sampling", "random")).strip().lower()
    candidates: list[list[float]] = []
    halton_index = 1 + int(options.seed) * 1009
    attempts = 0
    while attempts < candidate_count:
        attempts += 1
        if sampling in {"halton", "low_discrepancy", "low-discrepancy"}:
            unit = _halton_point(halton_index, len(coverage_root))
            halton_index += 1
            q = [
                float(lo) + unit[index] * (float(hi) - float(lo))
                for index, (lo, hi) in enumerate(coverage_root)
            ]
        elif sampling == "mixed" and attempts % 2 == 0:
            unit = _halton_point(halton_index, len(coverage_root))
            halton_index += 1
            q = [
                float(lo) + unit[index] * (float(hi) - float(lo))
                for index, (lo, hi) in enumerate(coverage_root)
            ]
        else:
            q = [rng.uniform(float(lo), float(hi)) for lo, hi in coverage_root]
        if any(float(q[index]) < float(tree_root[index][0]) or float(q[index]) > float(tree_root[index][1])
               for index in range(min(len(q), len(tree_root)))):
            continue
        if sbf.check_config_collision(robot, obstacles, q, float(options.audit_collision_tolerance)):
            continue
        candidates.append(q)
    selected: list[list[float]] = []
    lca_depths: list[int] = []
    min_distances: list[float] = []
    remaining = candidates[:]
    while remaining and len(selected) < count:
        best_index = 0
        best_score = -math.inf
        for index, q in enumerate(remaining):
            if not selected:
                lca_separation = float(len(schedule))
                min_distance = math.sqrt(len(q))
            else:
                lcas = [_lca_depth_for_points(q, other, tree_root, schedule) for other in selected]
                max_lca_depth = max(lcas) if lcas else 0
                lca_separation = float(len(schedule) - max_lca_depth)
                min_distance = min(_normalized_distance(q, other, coverage_root) for other in selected)
            score = (
                _joint_margin_score(q, coverage_root)
                + float(options.offline_anchor_lca_lambda) * lca_separation
                + float(options.offline_anchor_distance_mu) * min_distance
            )
            if score > best_score:
                best_score = score
                best_index = index
        chosen = remaining.pop(best_index)
        if selected:
            lcas = [_lca_depth_for_points(chosen, other, tree_root, schedule) for other in selected]
            lca_depths.append(max(lcas) if lcas else 0)
            min_distances.append(min(_normalized_distance(chosen, other, coverage_root) for other in selected))
        selected.append(chosen)
    metrics = {
        "offline_anchor_candidates": int(candidate_count),
        "offline_anchor_candidates_free": int(len(candidates)),
        "offline_anchor_roots_requested": int(len(selected)),
        "offline_anchor_lca_depth_mean": mean(lca_depths) if lca_depths else math.nan,
        "offline_anchor_lca_depth_max": max(lca_depths) if lca_depths else math.nan,
        "offline_anchor_min_distance_mean": mean(min_distances) if min_distances else math.nan,
        "offline_anchor_skip_reason": "",
        "offline_anchor_skip_p_main_accessible": math.nan,
    }
    return selected, metrics


def configure_leaf_rrt(robot: Any, database_path: Path, options: RBFLeafRRTOptions) -> Any:
    if database_path.exists():
        shutil.rmtree(database_path)
    cfg = sbf.SBFConfig()
    cfg.enable_connector = True
    endpoint_key = str(options.endpoint_source).strip().lower().replace("-", "_")
    if endpoint_key in {"crit", "critsample", "crit_sample"}:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
    elif endpoint_key in {"hifk", "hifk_aa"}:
        cfg.endpoint_source.source = sbf.EndpointSource.HIFK
        cfg.endpoint_source.hifk_max_depth = int(options.hifk_max_depth)
    else:
        cfg.endpoint_source.source = sbf.EndpointSource.IFK
    cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB if options.envelope == "link_aabb" else sbf.EnvelopeType.SupportHull
    if options.envelope != "link_aabb" and hasattr(cfg.envelope_type.support_hull_config, "direct_collision"):
        if hasattr(cfg.envelope_type.support_hull_config, "skip_aabb_broadphase"):
            cfg.envelope_type.support_hull_config.skip_aabb_broadphase = bool(options.support_hull_skip_aabb_broadphase)
    if bool(options.unsafe_sampling_validation):
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
    else:
        cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
        cfg.validation.accept_unsafe_free = False
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly

    cfg.database.path = str(database_path)
    cfg.database.create_if_missing = True
    cfg.database.max_tree_depth = int(options.rbf_max_depth)
    cfg.database.canonical_mode = bool(options.database_canonical_mode)
    cfg.database.symmetry_descriptor = CANONICAL_SYMMETRY_DESCRIPTOR
    cfg.database.verify_identity = bool(options.external_evidence_verify_identity)
    cfg.database.external_evidence_auto_build_snapshot = False
    root_intervals = None
    if options.root_override_tuples is not None:
        root_intervals = [sbf.Interval(float(lo), float(hi)) for lo, hi in options.root_override_tuples]
        cfg.database.root_intervals_override = root_intervals
    elif options.symmetry_aligned_native_root:
        root_intervals = [
            sbf.Interval(float(lo), float(hi))
            for lo, hi in robot_symmetry_aligned_root_tuples(robot)
        ]
        cfg.database.root_intervals_override = root_intervals
    elif options.use_shelf_root_override:
        raise RuntimeError(
            "use_shelf_root_override is deprecated: current experiments use native full-joint "
            "space outside LECT and canonical mapping only inside LECT."
        )
    if options.coverage_override_tuples is not None:
        cfg.database.coverage_intervals_override = [
            sbf.Interval(float(lo), float(hi)) for lo, hi in options.coverage_override_tuples
        ]
    cache_schedule = (
        read_lect_cache_depth_dimensions(options.external_evidence_path)
        if (bool(options.use_external_evidence) or bool(options.symmetry_aligned_cache_schedule))
        and options.external_evidence_path is not None
        else []
    )
    split_schedule_kind = _normalized_split_schedule_kind(getattr(options, "split_schedule_kind", "aafk_volume_min"))
    if cache_schedule and split_schedule_kind == "aafk_volume_min":
        cfg.database.split_policy = make_aafk_split_policy_from_cache_prefix(
            robot,
            int(options.rbf_max_depth),
            cache_schedule,
            root_intervals,
        )
    else:
        forced_tail_schedule = (
            read_lect_cache_depth_dimensions(options.external_evidence_path)
            if bool(options.symmetry_aligned_cache_schedule)
            else []
        )
        if (
            bool(options.symmetry_aligned_cache_schedule)
            and len(forced_tail_schedule) >= 2
            and int(forced_tail_schedule[0]) == 0
            and int(forced_tail_schedule[1]) == 0
        ):
            forced_tail_schedule = forced_tail_schedule[2:]
        cfg.database.split_policy = make_aafk_split_policy(
            robot,
            int(options.rbf_max_depth),
            root_intervals,
            force_dim0_first_two=bool(options.symmetry_aligned_cache_schedule),
            forced_tail_schedule=forced_tail_schedule,
            split_schedule_kind=split_schedule_kind,
        )

    endpoint_cache_enabled = bool(
        options.active_endpoint_evidence_cache
        or options.active_store_endpoint_evidence_cache
        or options.worker_shared_endpoint_cache
    )
    cfg.validation.enable_endpoint_evidence_cache = endpoint_cache_enabled
    cfg.validation.store_endpoint_evidence_cache = bool(options.active_store_endpoint_evidence_cache)
    cfg.validation.enable_worker_shared_endpoint_cache = bool(options.worker_shared_endpoint_cache)
    cfg.validation.external_evidence_backfill_active = False
    cfg.validation.external_evidence_materialization = True
    cfg.validation.external_evidence_scoring = True
    if hasattr(cfg.validation, "external_evidence_live_retry_on_maybe"):
        cfg.validation.external_evidence_live_retry_on_maybe = bool(options.external_evidence_live_retry_on_maybe)
    if options.use_external_evidence and options.external_evidence_path is not None:
        cfg.database.external_evidence_path = str(options.external_evidence_path)
        snapshot_path = Path(options.external_evidence_path) / "lect_snapshot"
        cfg.database.external_evidence_use_snapshot = snapshot_path.exists()
        cfg.database.external_evidence_auto_build_snapshot = False

    threads = max(1, int(options.threads))
    cfg.runtime.mode = sbf.ExecutionMode.Parallel if threads > 1 else sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = threads
    cfg.runtime.batch_size = threads
    cfg.runtime.parallel_threshold = 1
    cfg.grower.n_threads = threads
    cfg.grower.task_batch_size = threads
    cfg.grower.parallel_threshold = 1
    cfg.grower.rng_seed = int(options.seed)
    cfg.grower.mode = sbf.GrowerMode.RRT
    cfg.grower.max_boxes = max(1, int(options.deep_max_boxes))
    cfg.grower.timeout_ms = float(options.timeout_ms)
    cfg.grower.sample_categorical_allocation = True
    cfg.grower.intertree_goal_bias = 0.25
    cfg.grower.unexplored_sample_prob = 0.20
    cfg.grower.rrt_goal_bias = 0.15
    cfg.grower.anchor_target_prob = 0.025
    cfg.grower.sample_uniform_prob = 0.375
    cfg.grower.component_connect_prob = 0.0

    cfg.connector.n_threads = threads
    cfg.connector.parallel_threshold = 1
    cfg.connector.pair_batch_size = max(1, int(options.connector_pair_batch_size))
    cfg.connector.segment_edges_enabled = True
    cfg.connector.rrt_segment_edges = True
    cfg.connector.point_gap_segment_edges = True
    cfg.connector.segment_edges_fallback_only = bool(options.segment_edges_fallback_only)
    cfg.connector.enable_birrt = bool(options.connector_birrt)
    cfg.connector.max_pairs_per_gap = int(options.connector_max_pairs_per_gap)
    cfg.connector.per_pair_timeout_ms = float(options.connector_pair_timeout_ms)
    cfg.connector.max_total_bridge_boxes = int(options.connector_bridge_boxes)
    cfg.connector.rrt.max_iters = int(options.connector_rrt_iters)
    cfg.connector.rrt.timeout_ms = float(options.connector_rrt_timeout_ms)
    cfg.connector.rrt.step_size = float(options.connector_rrt_step_size)
    cfg.connector.rrt.goal_bias = float(options.connector_rrt_goal_bias)
    if hasattr(cfg.connector.rrt, "local_sampling_radius"):
        cfg.connector.rrt.local_sampling_radius = float(options.connector_rrt_local_sampling_radius)
    cfg.connector.rrt.segment_resolution = int(options.connector_segment_resolution)
    if hasattr(cfg.connector.rrt, "segment_step"):
        cfg.connector.rrt.segment_step = float(options.audit_segment_step)
    if hasattr(cfg.connector, "point_validated_gap_step"):
        cfg.connector.point_validated_gap_step = float(options.audit_segment_step)
    cfg.connector.pave.max_chain = int(options.connector_pave_max_chain)
    cfg.connector.pave.max_steps_per_waypoint = int(options.connector_pave_steps)
    cfg.connector.pave.find_free_box.max_depth = int(options.connector_pave_depth)
    if hasattr(cfg.connector.pave, "adaptive_min_segment_fraction"):
        cfg.connector.pave.adaptive_min_segment_fraction = float(options.connector_adaptive_min_segment_fraction)
    cfg.query_bridge_pave_depth = int(options.query_bridge_pave_depth)
    if hasattr(cfg, "query_bridge_ffb_start_depth"):
        cfg.query_bridge_ffb_start_depth = int(getattr(options, "query_bridge_ffb_start_depth", -1))
    if hasattr(cfg, "query_endpoint_anchor_ffb_depth"):
        cfg.query_endpoint_anchor_ffb_depth = int(options.query_endpoint_anchor_ffb_depth)
    if hasattr(cfg, "query_bridge_adaptive_ffb_depths"):
        cfg.query_bridge_adaptive_ffb_depths = [
            int(item.strip())
            for item in str(options.query_bridge_adaptive_ffb_depths).split(",")
            if item.strip()
        ]
    mode_name = str(options.ffb_search_mode).strip().lower().replace("_", "-")
    ffb_search_mode = None
    if hasattr(sbf, "FindFreeBoxSearchMode"):
        if mode_name in {"binary", "binary-depth", "binarydepth"}:
            ffb_search_mode = sbf.FindFreeBoxSearchMode.BinaryDepth
        elif mode_name in {"linear", ""}:
            ffb_search_mode = sbf.FindFreeBoxSearchMode.Linear
        else:
            raise ValueError(f"unknown FFB search mode: {options.ffb_search_mode}")
    cfg.connector.pave.find_free_box.skip_to_depth = int(options.ffb_start_depth)
    if hasattr(cfg.connector.pave.find_free_box, "start_depth"):
        cfg.connector.pave.find_free_box.start_depth = int(options.ffb_start_depth)
    if ffb_search_mode is not None and hasattr(cfg.connector.pave.find_free_box, "search_mode"):
        cfg.connector.pave.find_free_box.search_mode = ffb_search_mode
    cfg.connector.pave.find_free_box.split_reserved_leaf = True
    cfg.connector.pave.find_free_box.split_unknown_leaf = True
    cfg.connector.pave.find_free_box.reject_seed_collision = False
    cfg.connector.pave.fill_gaps = bool(options.connector_pave_fill_gaps)
    cfg.connector.pave.require_connected_chain = bool(options.connector_pave_require_connected_chain)

    cfg.query.strict_path_audit = True
    cfg.query.audit_resolution = max(int(options.audit_resolution), int(options.connector_segment_resolution))
    cfg.query.audit_segment_step = float(options.audit_segment_step)
    cfg.query.audit_collision_tolerance = float(options.audit_collision_tolerance)
    cfg.query.shortcut_boxes = bool(options.query_shortcut_boxes)
    cfg.query.collision_shortcut = bool(options.final_collision_shortcut)
    cfg.grower.find_free_box.skip_to_depth = int(options.ffb_start_depth)
    if hasattr(cfg.grower.find_free_box, "start_depth"):
        cfg.grower.find_free_box.start_depth = int(options.ffb_start_depth)
    if ffb_search_mode is not None and hasattr(cfg.grower.find_free_box, "search_mode"):
        cfg.grower.find_free_box.search_mode = ffb_search_mode
    if hasattr(cfg.query, "final_rrt_simplify"):
        cfg.query.final_rrt_simplify = bool(options.final_rrt_simplify)
    if hasattr(cfg.query, "final_rrt_simplify_timeout_ms"):
        cfg.query.final_rrt_simplify_timeout_ms = float(options.final_rrt_simplify_timeout_ms)
    if hasattr(cfg.query, "final_rrt_simplify_max_iters"):
        cfg.query.final_rrt_simplify_max_iters = int(options.final_rrt_simplify_max_iters)
    if hasattr(cfg.query, "final_rrt_simplify_attempts"):
        cfg.query.final_rrt_simplify_attempts = int(options.final_rrt_simplify_attempts)
    return cfg


def make_refine_config(options: RBFLeafRRTOptions) -> Any:
    cfg = sbf.LeafSweepRefineConfig()
    cfg.leaf_start_depth = int(options.leaf_start_depth)
    cfg.leaf_max_depth = int(options.leaf_max_depth)
    cfg.obstacle_cluster_gap = 1000.0
    cfg.use_virtual_topology = bool(options.use_virtual_topology)
    cfg.parallel_virtual_validation = bool(options.parallel_virtual_validation)
    cfg.store_group_results = False
    cfg.validation_batch_size = int(options.validation_batch_size)
    cfg.leaf_threads = max(1, int(options.leaf_threads))
    cfg.deep_max_boxes = int(options.deep_max_boxes)
    cfg.deep_ffb_depth = int(options.deep_ffb_depth)
    cfg.domain_seed_cap = int(options.domain_seed_cap)
    cfg.domain_success_cap = int(options.domain_success_cap)
    cfg.domain_attempt_cap = int(options.domain_attempt_cap)
    cfg.allow_anchor_roots = bool(options.allow_anchor_roots)
    cfg.refine_timeout_ms = float(options.refine_timeout_ms)
    cfg.run_rrt_grower = bool(options.run_rrt_grower)
    cfg.rrt_grower_extra_boxes = int(options.rrt_grower_extra_boxes)
    cfg.rrt_grower_timeout_ms = float(options.rrt_grower_timeout_ms)
    if hasattr(cfg, "priority_prune_radius"):
        cfg.priority_prune_radius = float(options.priority_prune_radius)
    if hasattr(cfg, "collision_overlap_prune_min_depth"):
        cfg.collision_overlap_prune_min_depth = int(options.collision_overlap_prune_min_depth)
    if hasattr(cfg, "collision_overlap_prune_threshold"):
        cfg.collision_overlap_prune_threshold = float(options.collision_overlap_prune_threshold)
    if hasattr(cfg, "collision_overlap_prune_ratio_threshold"):
        cfg.collision_overlap_prune_ratio_threshold = float(options.collision_overlap_prune_ratio_threshold)
    return cfg


def make_adaptive_leaf_sweep_config(options: RBFLeafRRTOptions) -> Any:
    cfg = sbf.AdaptiveLeafSweepConfig()
    cfg.shallow_start_depth = int(options.leaf_start_depth)
    cfg.shallow_max_depth = int(options.leaf_max_depth)
    cfg.target_max_depth = int(options.adaptive_target_depth)
    cfg.time_budget_ms = float(options.adaptive_time_budget_ms)
    cfg.node_budget = int(options.adaptive_node_budget)
    if hasattr(cfg, "fast_virtual_checkpoint_mode"):
        cfg.fast_virtual_checkpoint_mode = bool(options.adaptive_fast_virtual_checkpoint_mode)
    cfg.threads = max(1, int(options.leaf_threads))
    cfg.validation_batch_size = int(options.validation_batch_size)
    cfg.obstacle_cluster_gap = 1000.0
    cfg.use_virtual_topology = bool(options.use_virtual_topology)
    cfg.parallel_virtual_validation = bool(options.parallel_virtual_validation)
    cfg.store_group_results = False
    cfg.defer_min_depth = int(options.adaptive_defer_min_depth)
    cfg.overlap_depth_threshold = float(options.adaptive_overlap_depth_threshold)
    if hasattr(cfg, "overlap_depth_min_threshold"):
        cfg.overlap_depth_min_threshold = float(options.adaptive_overlap_depth_min_threshold)
    if hasattr(cfg, "overlap_depth_decay_per_depth"):
        cfg.overlap_depth_decay_per_depth = float(options.adaptive_overlap_depth_decay_per_depth)
    cfg.overlap_ratio_threshold = float(options.adaptive_overlap_ratio_threshold)
    cfg.seed_probe_count = int(options.adaptive_seed_probe_count)
    cfg.seed_probe_rng_seed = int(options.adaptive_seed_probe_rng_seed)
    cfg.seed_promote_uncovered = bool(options.adaptive_seed_promote_uncovered)
    cfg.seed_anchor_probe_cap = int(options.adaptive_seed_anchor_probe_cap)
    cfg.promotion_interval = int(options.adaptive_promotion_interval)
    if hasattr(cfg, "adaptive_depth_enabled"):
        cfg.adaptive_depth_enabled = bool(options.adaptive_depth_enabled)
    if hasattr(cfg, "adaptive_depth_min"):
        cfg.adaptive_depth_min = int(options.adaptive_depth_min)
    if hasattr(cfg, "adaptive_depth_max"):
        cfg.adaptive_depth_max = int(options.adaptive_depth_max)
    if hasattr(cfg, "adaptive_depth_probe_count"):
        cfg.adaptive_depth_probe_count = int(options.adaptive_depth_probe_count)
    if hasattr(cfg, "adaptive_depth_anchor_probe_cap"):
        cfg.adaptive_depth_anchor_probe_cap = int(options.adaptive_depth_anchor_probe_cap)
    if hasattr(cfg, "adaptive_depth_probe_seed"):
        cfg.adaptive_depth_probe_seed = int(options.adaptive_depth_probe_seed)
    if hasattr(cfg, "adaptive_depth_min_free_probes"):
        cfg.adaptive_depth_min_free_probes = int(options.adaptive_depth_min_free_probes)
    if hasattr(cfg, "adaptive_depth_min_covered_probes"):
        cfg.adaptive_depth_min_covered_probes = int(options.adaptive_depth_min_covered_probes)
    if hasattr(cfg, "adaptive_depth_min_main_probes"):
        cfg.adaptive_depth_min_main_probes = int(options.adaptive_depth_min_main_probes)
    if hasattr(cfg, "adaptive_depth_min_main_ratio"):
        cfg.adaptive_depth_min_main_ratio = float(options.adaptive_depth_min_main_ratio)
    if hasattr(cfg, "adaptive_depth_min_cells"):
        cfg.adaptive_depth_min_cells = int(options.adaptive_depth_min_cells)
    if hasattr(cfg, "adaptive_depth_min_main_cells"):
        cfg.adaptive_depth_min_main_cells = int(options.adaptive_depth_min_main_cells)
    if hasattr(cfg, "adaptive_depth_max_online_cells"):
        cfg.adaptive_depth_max_online_cells = int(options.adaptive_depth_max_online_cells)
    if hasattr(cfg, "adaptive_depth_max_probe_ms"):
        cfg.adaptive_depth_max_probe_ms = float(options.adaptive_depth_max_probe_ms)
    if hasattr(cfg, "max_merge_ms"):
        cfg.max_merge_ms = float(options.adaptive_max_merge_ms)
    if hasattr(cfg, "max_merge_rounds"):
        cfg.max_merge_rounds = int(options.adaptive_max_merge_rounds)
    if hasattr(cfg, "max_merge_input_boxes"):
        cfg.max_merge_input_boxes = int(options.adaptive_max_merge_input_boxes)
    if hasattr(cfg, "max_free_boxes"):
        cfg.max_free_boxes = int(options.adaptive_max_free_boxes)
    if hasattr(cfg, "max_unresolved_domains"):
        cfg.max_unresolved_domains = int(options.adaptive_max_unresolved_domains)
    if hasattr(cfg, "planning_backend"):
        cfg.planning_backend = str(options.adaptive_planning_backend)
    if hasattr(cfg, "grid_target_depth"):
        cfg.grid_target_depth = int(options.adaptive_grid_target_depth)
    if hasattr(cfg, "grid_face_index_enabled"):
        cfg.grid_face_index_enabled = bool(options.adaptive_grid_face_index_enabled)
    if hasattr(cfg, "grid_planning_max_expansions"):
        cfg.grid_planning_max_expansions = int(options.adaptive_grid_planning_max_expansions)
    if hasattr(cfg, "hipac_portal_connectivity"):
        cfg.hipac_portal_connectivity = bool(options.hipac_portal_connectivity)
    if hasattr(cfg, "hipac_portal_cell_native_validate"):
        cfg.hipac_portal_cell_native_validate = bool(options.hipac_portal_cell_native_validate)
    if hasattr(cfg, "hipac_portal_max_internal_boxes"):
        cfg.hipac_portal_max_internal_boxes = int(options.hipac_portal_max_internal_boxes)
    if hasattr(cfg, "hipac_portal_max_recursion_depth"):
        cfg.hipac_portal_max_recursion_depth = int(options.hipac_portal_max_recursion_depth)
    if hasattr(cfg, "hipac_portal_ffb_depth"):
        cfg.hipac_portal_ffb_depth = int(options.hipac_portal_ffb_depth)
    if hasattr(cfg, "hipac_portal_ffb_deadline_ms"):
        cfg.hipac_portal_ffb_deadline_ms = float(options.hipac_portal_ffb_deadline_ms)
    if hasattr(cfg, "hipac_online_connectivity"):
        cfg.hipac_online_connectivity = bool(options.hipac_online_connectivity)
    if hasattr(cfg, "hipac_online_before_query_bridge"):
        cfg.hipac_online_before_query_bridge = bool(options.hipac_online_before_query_bridge)
    if hasattr(cfg, "hipac_promote_query_repairs"):
        cfg.hipac_promote_query_repairs = bool(options.hipac_promote_query_repairs)
    if hasattr(cfg, "hipac_online_ffb_portal_fallback"):
        cfg.hipac_online_ffb_portal_fallback = bool(options.hipac_online_ffb_portal_fallback)
    if hasattr(cfg, "hipac_online_candidate_max_length"):
        cfg.hipac_online_candidate_max_length = float(options.hipac_online_candidate_max_length)
    if hasattr(cfg, "hipac_online_max_resolves_per_query"):
        cfg.hipac_online_max_resolves_per_query = int(options.hipac_online_max_resolves_per_query)
    if hasattr(cfg, "hipac_online_max_hidden_boxes_per_portal"):
        cfg.hipac_online_max_hidden_boxes_per_portal = int(options.hipac_online_max_hidden_boxes_per_portal)
    if hasattr(cfg, "hipac_online_max_ffb_calls_per_portal"):
        cfg.hipac_online_max_ffb_calls_per_portal = int(options.hipac_online_max_ffb_calls_per_portal)
    if hasattr(cfg, "hipac_online_prebridge_portal"):
        cfg.hipac_online_prebridge_portal = bool(options.hipac_online_prebridge_portal)
    if hasattr(cfg, "hipac_online_prebridge_candidate_limit"):
        cfg.hipac_online_prebridge_candidate_limit = int(options.hipac_online_prebridge_candidate_limit)
    if hasattr(cfg, "hipac_online_prebridge_max_pair_distance"):
        cfg.hipac_online_prebridge_max_pair_distance = float(options.hipac_online_prebridge_max_pair_distance)
    if hasattr(cfg, "hipac_online_prebridge_route_distance_weight"):
        cfg.hipac_online_prebridge_route_distance_weight = float(options.hipac_online_prebridge_route_distance_weight)
    if hasattr(cfg, "hipac_online_prebridge_pair_distance_weight"):
        cfg.hipac_online_prebridge_pair_distance_weight = float(options.hipac_online_prebridge_pair_distance_weight)
    if hasattr(cfg, "hipac_online_transition_portal"):
        cfg.hipac_online_transition_portal = bool(options.hipac_online_transition_portal)
    if hasattr(cfg, "hipac_transition_target_query_indices"):
        cfg.hipac_transition_target_query_indices = str(options.hipac_transition_target_query_indices)
    if hasattr(cfg, "hipac_transition_max_attempts_per_query"):
        cfg.hipac_transition_max_attempts_per_query = int(options.hipac_transition_max_attempts_per_query)
    if hasattr(cfg, "hipac_transition_candidate_limit"):
        cfg.hipac_transition_candidate_limit = int(options.hipac_transition_candidate_limit)
    if hasattr(cfg, "hipac_transition_window_stride"):
        cfg.hipac_transition_window_stride = int(options.hipac_transition_window_stride)
    if hasattr(cfg, "hipac_transition_min_predicted_bridge_edges"):
        cfg.hipac_transition_min_predicted_bridge_edges = int(options.hipac_transition_min_predicted_bridge_edges)
    if hasattr(cfg, "hipac_transition_max_pair_distance"):
        cfg.hipac_transition_max_pair_distance = float(options.hipac_transition_max_pair_distance)
    if hasattr(cfg, "hipac_transition_allow_same_component"):
        cfg.hipac_transition_allow_same_component = bool(options.hipac_transition_allow_same_component)
    if hasattr(cfg, "hipac_transition_obb_portal"):
        cfg.hipac_transition_obb_portal = bool(options.hipac_transition_obb_portal)
    if hasattr(cfg, "hipac_transition_obb_lateral_radius"):
        cfg.hipac_transition_obb_lateral_radius = float(options.hipac_transition_obb_lateral_radius)
    if hasattr(cfg, "hipac_transition_obb_longitudinal_margin"):
        cfg.hipac_transition_obb_longitudinal_margin = float(options.hipac_transition_obb_longitudinal_margin)
    if hasattr(cfg, "hipac_transition_obb_safety_epsilon"):
        cfg.hipac_transition_obb_safety_epsilon = float(options.hipac_transition_obb_safety_epsilon)
    if hasattr(cfg, "segment_edge_obb_cover"):
        cfg.segment_edge_obb_cover = bool(options.segment_edge_obb_cover)
    if hasattr(cfg, "rrt_bridge_obb_cover"):
        cfg.rrt_bridge_obb_cover = bool(options.rrt_bridge_obb_cover)
    if hasattr(cfg, "strict_obb_bridge_cover"):
        cfg.strict_obb_bridge_cover = bool(options.strict_obb_bridge_cover)
    if hasattr(cfg, "segment_edge_obb_lateral_radius"):
        cfg.segment_edge_obb_lateral_radius = float(options.segment_edge_obb_lateral_radius)
    if hasattr(cfg, "segment_edge_obb_longitudinal_margin"):
        cfg.segment_edge_obb_longitudinal_margin = float(options.segment_edge_obb_longitudinal_margin)
    if hasattr(cfg, "segment_edge_obb_safety_epsilon"):
        cfg.segment_edge_obb_safety_epsilon = float(options.segment_edge_obb_safety_epsilon)
    if hasattr(cfg, "segment_edge_obb_grow_iterations"):
        cfg.segment_edge_obb_grow_iterations = int(options.segment_edge_obb_grow_iterations)
    if hasattr(cfg, "segment_edge_obb_binary_iterations"):
        cfg.segment_edge_obb_binary_iterations = int(options.segment_edge_obb_binary_iterations)
    if hasattr(cfg, "segment_edge_obb_split_depth"):
        cfg.segment_edge_obb_split_depth = int(options.segment_edge_obb_split_depth)
    if hasattr(cfg, "obb_max_window_segments"):
        cfg.obb_max_window_segments = int(options.obb_max_window_segments)
    if hasattr(cfg, "obb_max_validations_per_window"):
        cfg.obb_max_validations_per_window = int(options.obb_max_validations_per_window)
    if hasattr(cfg, "hipac_promote_transition_slices"):
        cfg.hipac_promote_transition_slices = bool(options.hipac_promote_transition_slices)
    if hasattr(cfg, "hipac_promote_transition_target_query_indices"):
        cfg.hipac_promote_transition_target_query_indices = str(options.hipac_promote_transition_target_query_indices)
    if hasattr(cfg, "hipac_promote_transition_min_boxes"):
        cfg.hipac_promote_transition_min_boxes = int(options.hipac_promote_transition_min_boxes)
    if hasattr(cfg, "hipac_promote_transition_max_boxes"):
        cfg.hipac_promote_transition_max_boxes = int(options.hipac_promote_transition_max_boxes)
    if hasattr(cfg, "hipac_promote_transition_max_attempts_per_query"):
        cfg.hipac_promote_transition_max_attempts_per_query = int(options.hipac_promote_transition_max_attempts_per_query)
    return cfg


def point_distance(a: Iterable[float], b: Iterable[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def reflect_path_to_actual(
    query: QuerySpec,
    path: list[list[float]],
    planning_start: list[float],
    planning_goal: list[float],
    *,
    endpoint_tol: float = 1e-6,
) -> tuple[list[list[float]], bool, str]:
    if not path:
        return path, False, "empty_path"
    if query.actual_start is None or query.actual_goal is None:
        return path, False, "missing_actual_endpoint"
    actual_start = [float(value) for value in query.actual_start]
    actual_goal = [float(value) for value in query.actual_goal]
    if (
        len(actual_start) != len(planning_start)
        or len(actual_goal) != len(planning_goal)
        or any(len(point) != len(planning_start) for point in path)
    ):
        return path, False, "dimension_mismatch"
    start_delta = [a - c for a, c in zip(actual_start, planning_start)]
    goal_delta = [a - c for a, c in zip(actual_goal, planning_goal)]
    if max((abs(a - b) for a, b in zip(start_delta, goal_delta)), default=0.0) > float(endpoint_tol):
        return path, False, "actual_reflection_endpoint_mismatch"
    reflected = [[float(value) + start_delta[index] for index, value in enumerate(point)] for point in path]
    if point_distance(reflected[0], actual_start) > float(endpoint_tol):
        return reflected, False, "actual_start_mismatch"
    if point_distance(reflected[-1], actual_goal) > float(endpoint_tol):
        return reflected, False, "actual_goal_mismatch"
    return reflected, True, "actual_reflection_ok"


def path_collision_free(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    step: float,
    collision_tolerance: float = DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
) -> bool:
    if len(path) < 2:
        return False
    for a, b in zip(path[:-1], path[1:]):
        dist = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
        steps = max(1, int(math.ceil(dist / max(float(step), 1e-9))))
        for index in range(steps + 1):
            alpha = index / steps
            q = [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]
            if sbf.check_config_collision(robot, obstacles, q, float(collision_tolerance)):
                return False
    return True


def path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return math.nan
    return sum(point_distance(a, b) for a, b in zip(path[:-1], path[1:]))


def query_rows(
    forest: Any,
    robot: Any,
    queries: Iterable[Any],
    obstacles: list[Any] | None = None,
    audit_step: float = DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    audit_collision_tolerance: float = DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    canonicalize_queries: bool = False,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query_index, raw in enumerate(queries):
        query = query_spec(raw)
        active_query_index = stable_query_index(query.label, query_index)
        start = query_point(robot, query.start, canonicalize_queries)
        goal = query_point(robot, query.goal, canonicalize_queries)
        previous_active_query = os.environ.get("RBF_ACTIVE_QUERY_INDEX")
        os.environ["RBF_ACTIVE_QUERY_INDEX"] = str(active_query_index)
        try:
            result = forest.query(start, goal)
        finally:
            if previous_active_query is None:
                os.environ.pop("RBF_ACTIVE_QUERY_INDEX", None)
            else:
                os.environ["RBF_ACTIVE_QUERY_INDEX"] = previous_active_query
        canonical_path = result.path_as_lists()
        actual_path, reflected_ok, reflection_status = reflect_path_to_actual(query, canonical_path, start, goal)
        actual_audit_passed = bool(result.audit_passed)
        actual_audit_status = str(result.audit_status)
        if bool(result.success) and obstacles is not None:
            if reflected_ok:
                actual_audit_passed = path_collision_free(
                    robot,
                    obstacles,
                    actual_path,
                    audit_step,
                    audit_collision_tolerance,
                )
                actual_audit_status = "actual_reflected_audit_passed" if actual_audit_passed else "actual_reflected_audit_failed"
            else:
                actual_audit_passed = False
                actual_audit_status = reflection_status
        combined_audit_passed = bool(result.audit_passed) and bool(actual_audit_passed)
        audited_path_length = path_length(actual_path) if bool(result.success) and reflected_ok else math.nan
        raw_path_length = float(getattr(result, "raw_path_length", result.path_length)) if bool(result.success) else math.nan
        segment_length = float(result.segment_edge_length) if bool(result.success) else 0.0
        residual_segment_fraction = (
            float(getattr(result, "residual_segment_fraction", math.nan))
            if bool(result.success)
            else math.nan
        )
        if not math.isfinite(residual_segment_fraction):
            residual_segment_fraction = (
                segment_length / raw_path_length
                if bool(result.success) and raw_path_length > 1e-12
                else math.nan
            )
        residual_segment_length = (
            residual_segment_fraction * raw_path_length
            if bool(result.success)
            and math.isfinite(residual_segment_fraction)
            and math.isfinite(raw_path_length)
            else 0.0
        )
        query_ms = float(result.query_time_ms)
        simplify_ms = float(getattr(result, "final_simplify_time_ms", 0.0))
        solve_ms = query_ms
        rows.append({
            "label": query.label,
            "query_index": int(active_query_index),
            "success": bool(result.success),
            "audit_passed": combined_audit_passed,
            "canonical_audit_passed": bool(result.audit_passed),
            "actual_reflected_audit_passed": bool(actual_audit_passed),
            "actual_reflection_status": reflection_status,
            "query_ms": query_ms,
            "solve_ms": solve_ms,
            "simplify_ms": simplify_ms,
            "audit_ms": float(result.audit_time_ms),
            "final_simplify_ms": simplify_ms,
            "failed_segment_index": int(getattr(result, "failed_segment_index", -1)),
            "failed_debug_path_length": path_length(actual_path) if canonical_path and reflected_ok else math.nan,
            "path_length": audited_path_length,
            "final_path_length": audited_path_length,
            "raw_path_length": raw_path_length,
            "canonical_path_length": float(result.path_length) if bool(result.success) else math.nan,
            "segment_edge_length": segment_length,
            "segment_residual_length": residual_segment_length,
            "segment_fraction": residual_segment_fraction,
            "box_sequence_len": len(list(result.box_sequence)),
            "segment_edges_used": int(result.segment_edges_used),
            "obb_edges_used": int(getattr(result, "obb_edges_used", 0)),
            "obb_regions_used": int(getattr(result, "obb_regions_used", 0)),
            "obb_edge_length": float(getattr(result, "obb_edge_length", 0.0)),
            "partition_cells_used": int(getattr(result, "partition_cells_used", 0)),
            "partition_search_ms": float(getattr(result, "partition_search_ms", 0.0)),
            "partition_repair_ms": float(getattr(result, "partition_repair_ms", 0.0)),
            "non_grid_cells_used": int(getattr(result, "non_grid_cells_used", 0)),
            "residual_segment_fraction": residual_segment_fraction,
            "waypoint_count": len(canonical_path),
            "audit_status": actual_audit_status if not combined_audit_passed else str(result.audit_status),
            "canonical_audit_status": str(result.audit_status),
            "canonical_start": start,
            "canonical_goal": goal,
            "actual_start": list(query.actual_start) if query.actual_start is not None else start,
            "actual_goal": list(query.actual_goal) if query.actual_goal is not None else goal,
        })
    return rows


def refine_corridors(
    forest: Any,
    robot: Any,
    queries: Iterable[Any],
    options: RBFLeafRRTOptions,
) -> tuple[float, int, int]:
    if not bool(options.corridor_refine):
        return 0.0, 0, 0
    budget_s = max(0.0, float(options.corridor_refine_budget_ms)) / 1000.0
    max_total = max(0, int(options.corridor_refine_max_boxes))
    per_query = max(1, int(options.corridor_refine_boxes_per_query))
    if budget_s <= 0.0 or max_total <= 0:
        return 0.0, 0, 0
    mode = str(options.corridor_refine_mode)
    if mode not in {"box_only_long_path", "legacy_bridge"}:
        raise ValueError(f"unsupported corridor_refine_mode: {mode}")
    query_list = [query_spec(query) for query in queries]
    t0 = time.perf_counter()
    added_total = 0
    attempts = 0
    start_margin_s = max(0.0, float(options.corridor_refine_start_margin_ms)) / 1000.0
    for _pass in range(max(1, int(options.corridor_refine_passes))):
        pass_added = 0
        for query in query_list:
            elapsed_s = time.perf_counter() - t0
            if added_total >= max_total or elapsed_s >= budget_s:
                break
            if attempts > 0 and budget_s - elapsed_s < start_margin_s:
                break
            start = query_point(robot, query.start, bool(options.canonicalize_queries))
            goal = query_point(robot, query.goal, bool(options.canonicalize_queries))
            quota = min(per_query, max_total - added_total)
            added = int(forest.refine_query_corridor(
                start,
                goal,
                quota,
                mode,
                float(options.corridor_refine_long_path_ratio),
                float(options.corridor_refine_min_delta),
            ))
            attempts += 1
            added_total += added
            pass_added += added
        if pass_added == 0 or added_total >= max_total or time.perf_counter() - t0 >= budget_s:
            break
    return time.perf_counter() - t0, added_total, attempts


def bridge_all_queries(
    forest: Any,
    robot: Any,
    queries: Iterable[Any],
    options: RBFLeafRRTOptions,
) -> tuple[float, int, int, dict[str, float], dict[str, int]]:
    adaptive_all = bool(getattr(options, "query_bridge_adaptive_all", False))
    to_main_enabled = bool(getattr(options, "query_bridge_to_main_island", False))
    if not bool(options.query_bridge_all):
        labels = {item.strip() for item in str(options.query_bridge_labels).split(",") if item.strip()}
    else:
        labels = set()
    t0 = time.perf_counter()
    added_total = 0
    attempts = 0
    timing_by_label: dict[str, float] = {}
    added_by_label: dict[str, int] = {}
    selected: list[tuple[str, list[float], list[float], int]] = []
    force_selected_indices: set[int] = set()
    query_items: list[tuple[str, list[float], list[float], int]] = []
    force_selected = bool(getattr(options, "query_bridge_force_selected", False))
    for query_index, raw in enumerate(queries):
        query = query_spec(raw)
        start = query_point(robot, query.start, bool(options.canonicalize_queries))
        goal = query_point(robot, query.goal, bool(options.canonicalize_queries))
        query_items.append((str(query.label), start, goal, stable_query_index(query.label, query_index)))

    def query_good_enough(probe: Any, start: list[float], goal: list[float]) -> bool:
        raw_length = float(getattr(probe, "raw_path_length", getattr(probe, "path_length", math.inf)))
        segment_length = float(getattr(probe, "segment_edge_length", 0.0))
        segment_fraction = (
            segment_length / raw_length
            if raw_length > 1e-12 and math.isfinite(raw_length)
            else math.inf
        )
        segment_ok = segment_fraction <= float(getattr(options, "query_bridge_accept_segment_fraction", 0.0))
        path_length_value = float(getattr(probe, "path_length", math.inf))
        direct = point_distance(start, goal)
        absolute_short_enough = path_length_value <= float(
            getattr(options, "query_bridge_adaptive_max_path_length", math.inf)
        )
        ratio = float(getattr(options, "query_bridge_accept_path_ratio", 1.35))
        additive = float(getattr(options, "query_bridge_accept_path_additive", 0.35))
        short_enough = (
            direct <= 1e-9 or
            path_length_value <= max(direct * ratio, direct + additive) or
            absolute_short_enough
        )
        audit_ok = bool(getattr(probe, "audit_passed", True))
        return bool(getattr(probe, "success", False)) and audit_ok and segment_ok and short_enough

    if bool(getattr(options, "query_endpoint_anchor_before_bridge", True)) and hasattr(forest, "anchor_query_endpoint_box"):
        anchor_t0 = time.perf_counter()
        anchor_added = 0
        anchor_attempts = 0
        for _label, start, goal, _query_index in query_items:
            for point in (start, goal):
                anchor_attempts += 1
                box_id = int(forest.anchor_query_endpoint_box(point))
                if box_id >= 0:
                    anchor_added += 1
        timing_by_label["__endpoint_anchor__"] = time.perf_counter() - anchor_t0
        added_by_label["__endpoint_anchor__"] = anchor_added
        added_total += anchor_added
        attempts += anchor_attempts

    if not labels and not adaptive_all and not to_main_enabled:
        return time.perf_counter() - t0, added_total, attempts, timing_by_label, added_by_label

    # In non-forced mode, probe the existing graph after endpoint anchoring.
    # Endpoint-to-main repair is useful only for queries that are still not
    # acceptable; doing it unconditionally is a major online-time tail for
    # already-connected random-scene queries.
    initial_probe_by_label: dict[str, Any] = {}
    needs_repair_by_label: dict[str, bool] = {}
    if to_main_enabled and not force_selected:
        probe_t0 = time.perf_counter()
        probe_skips = 0
        probe_repairs = 0
        for label, start, goal, _query_index in query_items:
            if labels and label not in labels:
                continue
            probe = forest.query(start, goal)
            initial_probe_by_label[label] = probe
            needs_repair = not query_good_enough(probe, start, goal)
            needs_repair_by_label[label] = needs_repair
            if needs_repair:
                probe_repairs += 1
            else:
                probe_skips += 1
        timing_by_label["__prebridge_probe__"] = time.perf_counter() - probe_t0
        added_by_label["__prebridge_good_enough__"] = probe_skips
        added_by_label["__prebridge_needs_repair__"] = probe_repairs

    if bool(getattr(options, "query_bridge_to_main_island", False)):
        main_t0 = time.perf_counter()
        main_added = 0
        main_attempts = 0
        endpoints: list[tuple[str, list[float]]] = []
        for label, start, goal, _query_index in query_items:
            if labels and label not in labels:
                continue
            if needs_repair_by_label and not needs_repair_by_label.get(label, True):
                added_by_label[f"{label}:to_main_skipped"] = (
                    added_by_label.get(f"{label}:to_main_skipped", 0) + 1
                )
                continue
            endpoints.append((f"{label}:start", start))
            endpoints.append((f"{label}:goal", goal))
        for endpoint_label, point in endpoints:
            main_attempts += 1
            added = 0
            if (
                bool(getattr(options, "query_bridge_to_main_box_corridor", True)) and
                hasattr(forest, "connect_query_endpoint_to_main_box_corridor") and
                hasattr(sbf, "EndpointMainBoxCorridorConfig")
            ):
                corridor_cfg = sbf.EndpointMainBoxCorridorConfig()
                corridor_cfg.target_k = int(getattr(options, "endpoint_main_target_k", 8))
                corridor_cfg.coarse_step = float(getattr(options, "endpoint_main_coarse_step", 0.08))
                corridor_cfg.fine_step = float(getattr(options, "endpoint_main_fine_step", 0.02))
                corridor_cfg.max_ffb_calls = int(getattr(options, "endpoint_main_max_ffb_calls", 48))
                corridor_cfg.max_boxes = int(getattr(options, "endpoint_main_max_boxes", 64))
                corridor_cfg.adaptive_ffb_depths = [
                    int(item.strip())
                    for item in str(getattr(options, "endpoint_main_adaptive_ffb_depths", "")).split(",")
                    if item.strip()
                ]
                corridor_cfg.residual_segment_max_length = float(
                    getattr(options, "endpoint_main_residual_segment_max_length", 0.25)
                )
                corridor_cfg.lateral_offset = float(getattr(options, "endpoint_main_lateral_offset", 0.03))
                corridor_cfg.lateral_rounds = int(getattr(options, "endpoint_main_lateral_rounds", 2))
                corridor_cfg.face_epsilon = float(getattr(options, "endpoint_main_face_epsilon", 1e-6))
                added = int(forest.connect_query_endpoint_to_main_box_corridor(point, corridor_cfg))
            direct_max_length = float(getattr(
                options,
                "query_bridge_to_main_direct_segment_max_length",
                0.0,
            ))
            if (
                added <= 0 and
                direct_max_length > 0.0 and
                hasattr(forest, "connect_query_endpoint_to_main_island")
            ):
                added = int(forest.connect_query_endpoint_to_main_island(point, direct_max_length))
            main_added += added
            added_by_label[endpoint_label] = added_by_label.get(endpoint_label, 0) + added
        timing_by_label["__endpoint_to_main_island__"] = time.perf_counter() - main_t0
        added_by_label["__endpoint_to_main_island__"] = main_added
        added_total += main_added
        attempts += main_attempts
    if not labels and not adaptive_all:
        return time.perf_counter() - t0, added_total, attempts, timing_by_label, added_by_label
    # Endpoint-to-main is a cheap first-stage repair. It should not force the
    # heavier endpoint-to-endpoint bridge for every query; after the endpoint
    # stage, re-probe the graph and only bridge queries that still violate the
    # acceptance criteria.
    for label, start, goal, global_query_index in query_items:
        if labels and label not in labels:
            continue
        if force_selected:
            selected_index = len(selected)
            selected.append((label, start, goal, global_query_index))
            force_selected_indices.add(selected_index)
            continue
        if needs_repair_by_label and not needs_repair_by_label.get(label, True):
            continue
        if labels or adaptive_all:
            probe = forest.query(start, goal)
            if query_good_enough(probe, start, goal):
                continue
            if (
                bool(getattr(options, "query_bridge_to_main_island", False)) and
                query_good_enough(probe, start, goal)
            ):
                continue
        selected_index = len(selected)
        selected.append((label, start, goal, global_query_index))
        if force_selected:
            force_selected_indices.add(selected_index)
    if (
        selected and
        hasattr(forest, "bridge_queries") and
        (len(selected) > 1 or force_selected_indices)
    ):
        starts = [item[1] for item in selected]
        goals = [item[2] for item in selected]
        global_indices_text = ",".join(str(int(item[3])) for item in selected)
        force_indices = {
            int(item.strip())
            for item in str(options.query_bridge_force_indices).split(",")
            if item.strip()
        }
        force_indices.update(force_selected_indices)
        force_indices_text = ",".join(str(item) for item in sorted(force_indices))
        env_updates: dict[str, str | None] = {
            "RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES":
                str(options.query_bridge_segment_only_indices).strip() or None,
            "RBF_QUERY_BRIDGE_FORCE_INDICES":
                force_indices_text or None,
            "RBF_QUERY_BRIDGE_ACCEPT_SEGMENT_FRACTION":
                str(float(options.query_bridge_accept_segment_fraction)),
            "RBF_QUERY_BRIDGE_ACCEPT_PATH_RATIO":
                str(float(options.query_bridge_accept_path_ratio)),
            "RBF_QUERY_BRIDGE_ACCEPT_PATH_ADDITIVE":
                str(float(options.query_bridge_accept_path_additive)),
            "RBF_QUERY_BRIDGE_ADAPTIVE_MAX_PATH_LENGTH":
                str(float(options.query_bridge_adaptive_max_path_length)),
            "RBF_QUERY_BRIDGE_GLOBAL_INDICES":
                global_indices_text or None,
        }
        if int(options.query_bridge_forced_attempts) > 1:
            env_updates["RBF_QUERY_BRIDGE_FORCED_ATTEMPTS"] = str(int(options.query_bridge_forced_attempts))
        env_updates["RBF_QUERY_BRIDGE_ATTEMPT_OFFSET"] = str(
            int(getattr(options, "query_bridge_attempt_offset", 0))
        )
        env_updates["RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS"] = str(
            int(getattr(options, "query_bridge_no_path_retry_attempts", 0))
        )
        env_updates["RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS"] = (
            "1"
            if bool(getattr(options, "query_bridge_no_path_retry_stop_on_first_success", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ITERS"] = (
            str(getattr(options, "query_bridge_no_path_retry_budget_iters", "")).strip() or None
        )
        env_updates["RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ATTEMPTS"] = (
            str(getattr(options, "query_bridge_no_path_retry_budget_attempts", "")).strip() or None
        )
        env_updates["RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY"] = (
            "1"
            if bool(getattr(options, "query_bridge_waypoint_quality_retry", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY_ATTEMPTS"] = str(
            int(getattr(options, "query_bridge_waypoint_quality_retry_attempts", 4))
        )
        env_updates["RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY_ITERS"] = str(
            int(getattr(options, "query_bridge_waypoint_quality_retry_iters", 0))
        )
        env_updates["RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_RATIO"] = str(
            float(getattr(options, "query_bridge_waypoint_quality_max_ratio", 2.0))
        )
        env_updates["RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_ADDITIVE"] = str(
            float(getattr(options, "query_bridge_waypoint_quality_max_additive", 0.75))
        )
        env_updates["RBF_QUERY_BRIDGE_RRT_FIXED_ITERS"] = str(
            int(getattr(options, "query_bridge_rrt_fixed_iters", 0))
        )
        env_updates["RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS"] = str(
            float(getattr(options, "query_bridge_rrt_fixed_timeout_ms", 0.0))
        )
        env_updates["RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE"] = (
            str(getattr(options, "query_bridge_local_radius_schedule", "") or "")
        )
        env_updates["RBF_QUERY_BRIDGE_RRT_OPTIMIZE_AFTER_FIRST_ITERS"] = str(
            int(getattr(options, "query_bridge_rrt_optimize_after_first_iters", 0))
        )
        env_updates["RBF_QUERY_BRIDGE_ATTEMPT_FALLBACK_PATHS"] = str(
            int(getattr(options, "query_bridge_attempt_fallback_paths", 0))
        )
        env_updates["RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS"] = (
            "1"
            if bool(getattr(options, "query_bridge_hybridize_attempt_paths", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS"] = str(
            int(getattr(options, "query_bridge_hybrid_max_paths", 8))
        )
        env_updates["RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES"] = str(
            int(getattr(options, "query_bridge_hybrid_max_vertices", 128))
        )
        env_updates["RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS"] = str(
            int(getattr(options, "query_bridge_hybrid_max_cross_checks", 4096))
        )
        env_updates["RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP"] = (
            "1"
            if bool(getattr(options, "query_bridge_parallel_rrt_early_stop", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES"] = str(
            int(getattr(options, "query_bridge_parallel_rrt_early_stop_min_successes", 1))
        )
        env_updates["RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO"] = str(
            float(getattr(options, "query_bridge_parallel_rrt_early_stop_ratio", 1.75))
        )
        env_updates["RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE"] = str(
            float(getattr(options, "query_bridge_parallel_rrt_early_stop_additive", 0.75))
        )
        env_updates["RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT"] = (
            "1"
            if bool(getattr(options, "query_bridge_direct_segment_after_rrt", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH"] = str(
            float(getattr(options, "query_bridge_direct_segment_after_rrt_min_length", 0.0))
        )
        env_updates["RBF_QUERY_BRIDGE_FAST_DIRECT_SEGMENT_AFTER_RRT"] = (
            "1"
            if bool(getattr(options, "query_bridge_fast_direct_segment_after_rrt", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS"] = str(
            int(getattr(options, "query_bridge_fast_direct_random_shortcut_iters", 0))
        )
        env_updates["RBF_QUERY_ENDPOINT_POINT_ANCHOR"] = (
            "1" if bool(getattr(options, "query_endpoint_point_anchor", False)) else "0"
        )
        env_updates["RBF_OBB_FAST_PRIMARY_ORIENTATION"] = (
            "1" if bool(getattr(options, "obb_fast_primary_orientation", True)) else "0"
        )
        env_updates["RBF_OBB_FALLBACK_ORIENTATIONS_ON_PRIMARY_FAIL"] = (
            "1" if bool(getattr(options, "obb_fallback_orientations_on_primary_fail", False)) else "0"
        )
        if float(options.query_bridge_direct_sample_step) > 0.0:
            env_updates["RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP"] = str(float(options.query_bridge_direct_sample_step))
        if int(options.query_bridge_repair_subdivisions) >= 0:
            env_updates["RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS"] = str(int(options.query_bridge_repair_subdivisions))
        env_updates["RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR"] = (
            "1" if bool(getattr(options, "query_bridge_adaptive_step_repair", True)) else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP"] = str(
            float(getattr(options, "query_bridge_adaptive_fine_step", 0.08))
        )
        env_updates["RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS"] = str(
            int(getattr(options, "query_bridge_adaptive_max_repair_subdivisions", 2))
        )
        env_updates["RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS"] = str(
            int(getattr(options, "query_bridge_adaptive_max_repair_calls", 5))
        )
        env_updates["RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY"] = str(
            int(getattr(options, "query_bridge_adaptive_repair_priority", 1))
        )
        env_updates["RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_TARGET_SEGMENT_FRACTION"] = str(
            float(getattr(options, "query_bridge_adaptive_repair_target_segment_fraction", 0.0))
        )
        env_updates["RBF_QUERY_BRIDGE_SCENE_REUSABLE_EDGES"] = (
            "1"
            if bool(getattr(options, "query_bridge_scene_reusable_edges", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS"] = (
            "1" if bool(getattr(options, "query_bridge_group_residual_gaps", False)) else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED"] = (
            "1"
            if bool(getattr(options, "query_bridge_full_residual_overlay_when_connected", False))
            else "0"
        )
        env_updates["RBF_OBB_METADATA_ONLY"] = (
            "1"
            if bool(getattr(options, "segment_edge_obb_metadata_only", False))
            else "0"
        )
        env_updates["RBF_OBB_METADATA_ONLY_REQUIRE_COVER"] = (
            "1"
            if bool(getattr(options, "segment_edge_obb_metadata_require_cover", False))
            else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES"] = (
            "1" if bool(getattr(options, "query_bridge_partition_neighbor_candidates", False)) else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE"] = (
            "1" if bool(getattr(options, "query_bridge_direct_append_partition_immediate", False)) else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_LOCAL_SAMPLE_ASSIMILATION"] = (
            "1" if bool(getattr(options, "query_bridge_local_sample_assimilation", True)) else "0"
        )
        env_updates["RBF_QUERY_BRIDGE_DIRECT_PARTITION_APPEND_BATCH_SIZE"] = str(
            int(getattr(options, "query_bridge_direct_partition_append_batch_size", 32))
        )
        if float(options.query_bridge_direct_max_length) > 0.0:
            env_updates["RBF_QUERY_BRIDGE_DIRECT_MAX_LENGTH"] = str(float(options.query_bridge_direct_max_length))
        previous_env = {name: os.environ.get(name) for name in env_updates}
        for name, value in env_updates.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
        try:
            if bool(getattr(options, "query_bridge_sequential_reuse", False)):
                added_values: list[int] = []
                reuse_skips = 0
                segment_only_indices = {
                    int(item.strip())
                    for item in str(options.query_bridge_segment_only_indices).split(",")
                    if item.strip()
                }
                for selected_index, (label, start, goal, global_query_index) in enumerate(selected):
                    q0 = time.perf_counter()
                    probe = forest.query(start, goal)
                    forced_query = selected_index in force_indices
                    if (not forced_query) and query_good_enough(probe, start, goal):
                        reuse_skips += 1
                        added_values.append(0)
                        timing_by_label[label] = timing_by_label.get(label, 0.0) + (time.perf_counter() - q0)
                        added_by_label[label] = added_by_label.get(label, 0) + 0
                        continue
                    os.environ["RBF_QUERY_BRIDGE_GLOBAL_INDICES"] = str(int(global_query_index))
                    if forced_query:
                        os.environ["RBF_QUERY_BRIDGE_FORCE_INDICES"] = "0"
                    else:
                        os.environ.pop("RBF_QUERY_BRIDGE_FORCE_INDICES", None)
                    if selected_index in segment_only_indices:
                        os.environ["RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES"] = "0"
                    else:
                        os.environ.pop("RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES", None)
                    added = int(forest.bridge_queries([start], [goal])[0])
                    added_values.append(added)
                    timing_by_label[label] = timing_by_label.get(label, 0.0) + (time.perf_counter() - q0)
                    added_by_label[label] = added_by_label.get(label, 0) + added
                added_by_label["__sequential_reuse_skips__"] = (
                    added_by_label.get("__sequential_reuse_skips__", 0) + reuse_skips
                )
                timing_by_label["__sequential_reuse__"] = time.perf_counter() - t0
            else:
                added_values = [int(value) for value in forest.bridge_queries(starts, goals)]
        finally:
            for name, previous_value in previous_env.items():
                if previous_value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = previous_value
        elapsed = time.perf_counter() - t0
        timing_by_label["__batch_total__"] = elapsed
        for (label, _start, _goal, _global_query_index), added in zip(selected, added_values, strict=True):
            if not bool(getattr(options, "query_bridge_sequential_reuse", False)):
                added_by_label[label] = added_by_label.get(label, 0) + added
            added_total += added
        attempts += len(selected)
        return elapsed, added_total, attempts, timing_by_label, added_by_label

    for label, start, goal, _global_query_index in selected:
        q0 = time.perf_counter()
        added = int(forest.bridge_query(start, goal))
        elapsed = time.perf_counter() - q0
        timing_by_label[label] = timing_by_label.get(label, 0.0) + elapsed
        added_by_label[label] = added_by_label.get(label, 0) + added
        added_total += added
        attempts += 1
    return time.perf_counter() - t0, added_total, attempts, timing_by_label, added_by_label


def forest_adjacency_island_count(forest: Any) -> int:
    boxes = list(forest.boxes())
    if not boxes:
        return 0
    adjacency = dict(forest.adjacency())
    box_ids = [int(box.id) for box in boxes]
    box_id_set = set(box_ids)
    seen: set[int] = set()
    count = 0
    for box_id in box_ids:
        if box_id in seen:
            continue
        count += 1
        stack = [box_id]
        seen.add(box_id)
        while stack:
            current = stack.pop()
            for neighbor in adjacency.get(current, []):
                neighbor_id = int(neighbor)
                if neighbor_id not in box_id_set or neighbor_id in seen:
                    continue
                seen.add(neighbor_id)
                stack.append(neighbor_id)
    return count


def run_leaf_rrt(
    *,
    robot: Any,
    obstacles: list[Any],
    queries: Iterable[Any],
    database_path: Path,
    options: RBFLeafRRTOptions,
) -> dict[str, Any]:
    query_list = list(queries)
    cfg = configure_leaf_rrt(robot, database_path, options)
    forest = sbf.SafeBoxForest(robot, cfg)
    refine_cfg = make_refine_config(options)
    adaptive_cfg = make_adaptive_leaf_sweep_config(options)
    offline_t0 = time.perf_counter()
    offline_anchor_points: list[list[float]] = []
    offline_anchor_select_metrics = empty_offline_anchor_metrics("not_selected")
    offline_anchor_select_s = 0.0
    def select_offline_anchor_points() -> None:
        nonlocal offline_anchor_points, offline_anchor_select_metrics, offline_anchor_select_s
        anchor_select_t0 = time.perf_counter()
        offline_anchor_points, offline_anchor_select_metrics = generate_offline_anchor_points(
            robot,
            list(obstacles),
            options,
        )
        offline_anchor_select_s = time.perf_counter() - anchor_select_t0
    offline_anchor_insert_added = 0
    offline_anchor_insert_attempts = 0
    offline_anchor_insert_s = 0.0
    if str(options.offline_grower) == "adaptive_deep_leaf":
        if not bool(options.offline_anchor_skip_if_main_accessible):
            select_offline_anchor_points()
        build = forest.build_adaptive_deep_leaf_sweep_cover(obstacles, adaptive_cfg)
        if bool(options.offline_anchor_skip_if_main_accessible):
            p_main_accessible = float(getattr(build, "p_main_accessible", math.nan))
            if not math.isfinite(p_main_accessible):
                profile = getattr(build, "profile", None)
                diagnostics = getattr(profile, "diagnostics", {}) if profile is not None else {}
                p_main_accessible = float(diagnostics.get("adaptive.p_main_accessible", math.nan))
            anchor_skip = (
                math.isfinite(p_main_accessible) and
                p_main_accessible >= float(options.offline_anchor_main_accessible_threshold)
            )
            if anchor_skip:
                offline_anchor_select_metrics = empty_offline_anchor_metrics("main_accessible")
                offline_anchor_select_metrics["offline_anchor_skip_p_main_accessible"] = p_main_accessible
            else:
                select_offline_anchor_points()
        if offline_anchor_points and hasattr(forest, "anchor_query_endpoint_box"):
            anchor_insert_t0 = time.perf_counter()
            for point in offline_anchor_points:
                offline_anchor_insert_attempts += 1
                before_count = len(list(forest.boxes()))
                box_id = int(forest.anchor_query_endpoint_box([float(value) for value in point]))
                after_count = len(list(forest.boxes()))
                if box_id >= 0 and after_count > before_count:
                    offline_anchor_insert_added += after_count - before_count
            offline_anchor_insert_s = time.perf_counter() - anchor_insert_t0
    else:
        select_offline_anchor_points()
        build = forest.build_leaf_sweep_refined(
            obstacles,
            refine_cfg,
            (
                canonical_priority_points(robot, query_list, canonicalize=bool(options.canonicalize_queries))
                if bool(options.use_priority_points) and not bool(options.offline_query_agnostic_build)
                else []
            ),
            offline_anchor_points,
        )
    offline_shortcut_t0 = time.perf_counter()
    offline_shortcut_edges_added = 0
    offline_connector_mode = str(getattr(options, "offline_connector_mode", "box_only")).strip().lower().replace("-", "_")
    allow_offline_segment_fallback = offline_connector_mode in {"short_segment", "short_segments", "segment", "segments"}
    if (
        int(options.offline_shortcut_edges) > 0
        and offline_connector_mode not in {"off", "none", "disabled", "disable", "0", "false"}
        and hasattr(forest, "add_offline_shortcut_edges")
    ):
        offline_shortcut_edges_added = int(forest.add_offline_shortcut_edges(
            int(options.offline_shortcut_edges),
            int(options.offline_shortcut_candidate_limit),
            float(options.offline_shortcut_min_gain_ratio),
            float(options.offline_shortcut_max_segment_length),
            bool(allow_offline_segment_fallback),
        ))
    offline_shortcut_s = time.perf_counter() - offline_shortcut_t0
    offline_coverage_s = max(0.0, time.perf_counter() - offline_t0 - offline_shortcut_s)
    offline_build_wall_s = time.perf_counter() - offline_t0
    build_final_boxes = len(list(forest.boxes()))
    build_segment_edges = len(list(forest.segment_edges()))
    query_cost_env = {
        "RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY":
            str(float(options.query_box_transition_line_deviation_penalty)),
        "RBF_QUERY_FOREIGN_EDGE_COST_PENALTY":
            str(float(options.query_foreign_edge_cost_penalty)),
        "RBF_QUERY_BRIDGE_EDGE_COST_PENALTY":
            str(float(options.query_bridge_edge_cost_penalty)),
    }
    previous_query_cost_env = {name: os.environ.get(name) for name in query_cost_env}
    for name, value in query_cost_env.items():
        os.environ[name] = value
    partition_native_requested = (
        str(getattr(options, "adaptive_planning_backend", "")).lower() == "partition_native"
    )
    try:
        corridor_refine_s, corridor_refine_added, corridor_refine_attempts = refine_corridors(
            forest,
            robot,
            query_list,
            options,
        )
        after_corridor_boxes = len(list(forest.boxes()))
        after_corridor_segment_edges = len(list(forest.segment_edges()))
        if bool(options.query_bridge_sequential_reuse):
            query_bridge_s = 0.0
            query_bridge_added = 0
            query_bridge_attempts = 0
            query_bridge_by_label_s: dict[str, float] = {}
            query_bridge_added_by_label: dict[str, int] = {}
            qrows = []
            for raw_query in query_list:
                (
                    step_bridge_s,
                    step_bridge_added,
                    step_bridge_attempts,
                    step_bridge_by_label_s,
                    step_bridge_added_by_label,
                ) = bridge_all_queries(
                    forest,
                    robot,
                    [raw_query],
                    options,
                )
                query_bridge_s += float(step_bridge_s)
                query_bridge_added += int(step_bridge_added)
                query_bridge_attempts += int(step_bridge_attempts)
                for key, value in step_bridge_by_label_s.items():
                    query_bridge_by_label_s[key] = query_bridge_by_label_s.get(key, 0.0) + float(value)
                for key, value in step_bridge_added_by_label.items():
                    query_bridge_added_by_label[key] = query_bridge_added_by_label.get(key, 0) + int(value)
                step_qrows = query_rows(
                    forest,
                    robot,
                    [raw_query],
                    obstacles=list(obstacles),
                    audit_step=float(options.audit_segment_step),
                    audit_collision_tolerance=float(options.audit_collision_tolerance),
                    canonicalize_queries=bool(options.canonicalize_queries),
                )
                if (
                    bool(getattr(options, "query_bridge_parallel_rrt_early_stop", False)) and
                    not all(bool(row.get("audit_passed", False)) for row in step_qrows)
                ):
                    retry_options = copy.copy(options)
                    retry_options.query_bridge_parallel_rrt_early_stop = False
                    (
                        retry_bridge_s,
                        retry_added,
                        retry_attempts,
                        retry_by_label_s,
                        retry_added_by_label,
                    ) = bridge_all_queries(
                        forest,
                        robot,
                        [raw_query],
                        retry_options,
                    )
                    query_bridge_s += float(retry_bridge_s)
                    query_bridge_added += int(retry_added)
                    query_bridge_attempts += int(retry_attempts)
                    for key, value in retry_by_label_s.items():
                        query_bridge_by_label_s[f"serial_retry:{key}"] = (
                            query_bridge_by_label_s.get(f"serial_retry:{key}", 0.0) + float(value)
                        )
                    for key, value in retry_added_by_label.items():
                        query_bridge_added_by_label[f"serial_retry:{key}"] = (
                            query_bridge_added_by_label.get(f"serial_retry:{key}", 0) + int(value)
                        )
                    step_qrows = query_rows(
                        forest,
                        robot,
                        [raw_query],
                        obstacles=list(obstacles),
                        audit_step=float(options.audit_segment_step),
                        audit_collision_tolerance=float(options.audit_collision_tolerance),
                        canonicalize_queries=bool(options.canonicalize_queries),
                    )
                if (
                    bool(getattr(options, "query_bridge_failure_fallback_to_main", False)) and
                    not all(bool(row.get("audit_passed", False)) for row in step_qrows)
                ):
                    fallback_options = copy.copy(options)
                    fallback_options.query_bridge_sequential_reuse = True
                    fallback_options.query_bridge_to_main_island = True
                    fallback_options.query_bridge_force_selected = True
                    fallback_options.query_bridge_direct_segment_after_rrt = True
                    fallback_options.query_bridge_fast_direct_segment_after_rrt = True
                    fallback_options.segment_edge_obb_cover = True
                    fallback_options.rrt_bridge_obb_cover = True
                    fallback_options.strict_obb_bridge_cover = True
                    fallback_options.query_bridge_no_path_retry_attempts = max(
                        int(getattr(options, "query_bridge_no_path_retry_attempts", 0)),
                        2,
                    )
                    if not str(getattr(options, "query_bridge_no_path_retry_budget_iters", "")).strip():
                        fallback_options.query_bridge_no_path_retry_budget_iters = "40000"
                    if not str(getattr(options, "query_bridge_no_path_retry_budget_attempts", "")).strip():
                        fallback_options.query_bridge_no_path_retry_budget_attempts = "4"
                    fallback_options.query_bridge_forced_attempts = max(
                        int(getattr(options, "query_bridge_forced_attempts", 1)) + 2,
                        int(getattr(fallback_options, "query_bridge_forced_attempts", 1)),
                    )
                    (
                        fallback_bridge_s,
                        fallback_added,
                        fallback_attempts,
                        fallback_by_label_s,
                        fallback_added_by_label,
                    ) = bridge_all_queries(
                        forest,
                        robot,
                        [raw_query],
                        fallback_options,
                    )
                    query_bridge_s += float(fallback_bridge_s)
                    query_bridge_added += int(fallback_added)
                    query_bridge_attempts += int(fallback_attempts)
                    for key, value in fallback_by_label_s.items():
                        query_bridge_by_label_s[f"fallback:{key}"] = (
                            query_bridge_by_label_s.get(f"fallback:{key}", 0.0) + float(value)
                        )
                    for key, value in fallback_added_by_label.items():
                        query_bridge_added_by_label[f"fallback:{key}"] = (
                            query_bridge_added_by_label.get(f"fallback:{key}", 0) + int(value)
                        )
                    step_qrows = query_rows(
                        forest,
                        robot,
                        [raw_query],
                        obstacles=list(obstacles),
                        audit_step=float(options.audit_segment_step),
                        audit_collision_tolerance=float(options.audit_collision_tolerance),
                        canonicalize_queries=bool(options.canonicalize_queries),
                    )
                qrows.extend(step_qrows)
        else:
            (
                query_bridge_s,
                query_bridge_added,
                query_bridge_attempts,
                query_bridge_by_label_s,
                query_bridge_added_by_label,
            ) = bridge_all_queries(
                forest,
                robot,
                query_list,
                options,
            )
            qrows = query_rows(
                forest,
                robot,
                query_list,
                obstacles=list(obstacles),
                audit_step=float(options.audit_segment_step),
                audit_collision_tolerance=float(options.audit_collision_tolerance),
                canonicalize_queries=bool(options.canonicalize_queries),
            )
            if (
                bool(getattr(options, "query_bridge_failure_fallback_to_main", False)) and
                (
                    not qrows or
                    not all(bool(row.get("audit_passed", False)) for row in qrows)
                )
            ):
                if len(qrows) == len(query_list):
                    failed_queries = [
                        raw_query
                        for raw_query, row in zip(query_list, qrows, strict=True)
                        if not bool(row.get("audit_passed", False))
                    ]
                else:
                    # Some hard failures can produce no per-query result rows.
                    # Fall back conservatively rather than silently accepting
                    # an incomplete batch.
                    failed_queries = list(query_list)
                if failed_queries:
                    fallback_options = copy.copy(options)
                    fallback_options.query_bridge_to_main_island = True
                    fallback_options.query_bridge_force_selected = True
                    fallback_options.query_bridge_forced_attempts = max(
                        int(getattr(options, "query_bridge_forced_attempts", 1)) + 2,
                        int(getattr(fallback_options, "query_bridge_forced_attempts", 1)),
                    )
                    (
                        fallback_bridge_s,
                        fallback_added,
                        fallback_attempts,
                        fallback_by_label_s,
                        fallback_added_by_label,
                    ) = bridge_all_queries(
                        forest,
                        robot,
                        failed_queries,
                        fallback_options,
                    )
                    query_bridge_s += float(fallback_bridge_s)
                    query_bridge_added += int(fallback_added)
                    query_bridge_attempts += int(fallback_attempts)
                    for key, value in fallback_by_label_s.items():
                        query_bridge_by_label_s[f"fallback:{key}"] = (
                            query_bridge_by_label_s.get(f"fallback:{key}", 0.0) + float(value)
                        )
                    for key, value in fallback_added_by_label.items():
                        query_bridge_added_by_label[f"fallback:{key}"] = (
                            query_bridge_added_by_label.get(f"fallback:{key}", 0) + int(value)
                        )
                    qrows = query_rows(
                        forest,
                        robot,
                        query_list,
                        obstacles=list(obstacles),
                        audit_step=float(options.audit_segment_step),
                        audit_collision_tolerance=float(options.audit_collision_tolerance),
                        canonicalize_queries=bool(options.canonicalize_queries),
                    )
        final_boxes = len(list(forest.boxes()))
        final_segment_edges = len(list(forest.segment_edges()))
        final_adjacency_islands = -1 if partition_native_requested else forest_adjacency_island_count(forest)
    finally:
        for name, previous_value in previous_query_cost_env.items():
            if previous_value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = previous_value
    build_for_diagnostics = forest.last_build_profile() if hasattr(forest, "last_build_profile") else build
    successes = [row for row in qrows if bool(row["audit_passed"])]
    total_len = sum(float(row["raw_path_length"]) for row in successes if math.isfinite(float(row["raw_path_length"])))
    total_seg = sum(float(row.get("segment_residual_length", row["segment_edge_length"])) for row in successes)
    diagnostics = {str(k): float(v) for k, v in dict(build_for_diagnostics.diagnostics).items()}
    partition_island_count = int(diagnostics.get(
        "adaptive.partition_islands",
        getattr(build, "partition_islands", 0),
    ))
    reported_final_islands = (
        partition_island_count
        if partition_native_requested and partition_island_count > 0
        else int(max(0, final_adjacency_islands))
    )
    qroot_pairs_total = int(diagnostics.get(
        "leaf_refine.qroot_pairs_total",
        diagnostics.get("adaptive.qroot_pairs_total", 0.0),
    ))
    qroot_uncovered_endpoints = int(diagnostics.get(
        "leaf_refine.qroot_uncovered_endpoints",
        diagnostics.get("adaptive.qroot_uncovered_endpoints", 0.0),
    ))
    if bool(options.offline_query_agnostic_build) and (qroot_pairs_total != 0 or qroot_uncovered_endpoints != 0):
        raise RuntimeError(
            "offline query-agnostic build invariant failed: "
            f"qroot_pairs_total={qroot_pairs_total}, "
            f"qroot_uncovered_endpoints={qroot_uncovered_endpoints}"
        )
    offline_build_profile_s = float(build.total_ms) / 1000.0
    offline_build_s = float(offline_build_wall_s)
    online_adaptation_s = float(corridor_refine_s) + float(query_bridge_s)
    graph_solve_s = sum(float(row.get("solve_ms", row["query_ms"])) for row in qrows) / 1000.0
    graph_simplify_s = sum(float(row.get("simplify_ms", row.get("final_simplify_ms", 0.0))) for row in qrows) / 1000.0
    graph_query_s = graph_solve_s
    graph_query_total_s = graph_solve_s + graph_simplify_s
    partition_query_s = (
        sum(float(row.get("partition_search_ms", 0.0)) for row in qrows) / 1000.0
        if partition_native_requested
        else 0.0
    )
    partition_query_total_s = partition_query_s + (graph_simplify_s if partition_native_requested else 0.0)
    online_solve_s = online_adaptation_s + graph_solve_s
    online_simplify_s = graph_simplify_s
    query_s = online_solve_s
    query_total_s = online_solve_s + online_simplify_s
    query_count = max(1, len(qrows))
    query_bridge_per_query_s = float(query_bridge_s) / query_count
    online_per_query_s = query_s / query_count
    online_total_per_query_s = query_total_s / query_count
    online_solve_per_query_s = online_solve_s / query_count
    online_simplify_per_query_s = online_simplify_s / query_count
    offline_segment_edges_added = int(getattr(build.profile, "segment_edges_added", 0))
    offline_box_edges_added = int(getattr(build.profile, "bridge_boxes_added", 0))
    amortized = {
        f"amortized_s_k{k}": offline_build_s / float(k) + online_per_query_s
        for k in (1, 5, 10, 20, 50)
    }
    external_evidence_fields = _external_evidence_diagnostic_fields(diagnostics)
    query_bridge_diagnostic_fields = _query_bridge_diagnostic_fields(diagnostics, query_count)
    query_success_by_label = {str(row["label"]): bool(row["audit_passed"]) for row in qrows}
    query_status_by_label = {str(row["label"]): str(row["audit_status"]) for row in qrows}
    query_raw_length_by_label = {
        str(row["label"]): float(row["raw_path_length"])
        for row in qrows
        if math.isfinite(float(row["raw_path_length"]))
    }
    query_segment_fraction_by_label = {
        str(row["label"]): float(row["segment_fraction"])
        for row in qrows
        if math.isfinite(float(row["segment_fraction"]))
    }
    query_segment_edges_used_by_label = {
        str(row["label"]): int(row["segment_edges_used"])
        for row in qrows
    }
    query_obb_edges_used_by_label = {
        str(row["label"]): int(row.get("obb_edges_used", 0))
        for row in qrows
    }
    query_obb_regions_used_by_label = {
        str(row["label"]): int(row.get("obb_regions_used", 0))
        for row in qrows
    }
    query_obb_edge_length_by_label = {
        str(row["label"]): float(row.get("obb_edge_length", 0.0))
        for row in qrows
    }
    return {
        "case": options.case_label,
        "seed": int(options.seed),
        "offline_grower": str(options.offline_grower),
        "deep_max_boxes": int(options.deep_max_boxes),
        "endpoint_source": str(options.endpoint_source),
        "unsafe_sampling_validation": bool(options.unsafe_sampling_validation),
        "status": "ok" if len(successes) == len(qrows) else "partial",
        "success_count": len(successes),
        "query_count": len(qrows),
        "query_success_by_label": query_success_by_label,
        "query_audit_status_by_label": query_status_by_label,
        "query_raw_length_by_label": query_raw_length_by_label,
        "query_segment_fraction_by_label": query_segment_fraction_by_label,
        "query_segment_edges_used_by_label": query_segment_edges_used_by_label,
        "query_obb_edges_used_by_label": query_obb_edges_used_by_label,
        "query_obb_regions_used_by_label": query_obb_regions_used_by_label,
        "query_obb_edge_length_by_label": query_obb_edge_length_by_label,
        "query_bridge_sequential_reuse": bool(options.query_bridge_sequential_reuse),
        "query_bridge_scene_reusable_edges": bool(options.query_bridge_scene_reusable_edges),
        "query_bridge_reuse_scope": "scene_seed_local" if bool(options.query_bridge_scene_reusable_edges) else "query_index_local",
        "query_bridge_full_residual_overlay_when_connected": bool(
            options.query_bridge_full_residual_overlay_when_connected
        ),
        "query_bridge_direct_segment_after_rrt": bool(options.query_bridge_direct_segment_after_rrt),
        "query_bridge_direct_segment_after_rrt_min_length": float(
            options.query_bridge_direct_segment_after_rrt_min_length
        ),
        "query_bridge_fast_direct_segment_after_rrt": bool(
            getattr(options, "query_bridge_fast_direct_segment_after_rrt", False)
        ),
        "query_bridge_fast_direct_random_shortcut_iters": int(
            getattr(options, "query_bridge_fast_direct_random_shortcut_iters", 0)
        ),
        "query_bridge_hybridize_attempt_paths": bool(
            getattr(options, "query_bridge_hybridize_attempt_paths", False)
        ),
        "query_bridge_hybrid_max_paths": int(
            getattr(options, "query_bridge_hybrid_max_paths", 8)
        ),
        "query_bridge_hybrid_max_vertices": int(
            getattr(options, "query_bridge_hybrid_max_vertices", 128)
        ),
        "query_bridge_hybrid_max_cross_checks": int(
            getattr(options, "query_bridge_hybrid_max_cross_checks", 4096)
        ),
        "query_endpoint_point_anchor": bool(getattr(options, "query_endpoint_point_anchor", False)),
        "segment_edge_obb_cover": bool(options.segment_edge_obb_cover),
        "rrt_bridge_obb_cover": bool(options.rrt_bridge_obb_cover),
        "strict_obb_bridge_cover": bool(options.strict_obb_bridge_cover),
        "segment_edge_obb_metadata_only": bool(options.segment_edge_obb_metadata_only),
        "segment_edge_obb_metadata_require_cover": bool(
            options.segment_edge_obb_metadata_require_cover
        ),
        "segment_edge_obb_lateral_radius": float(options.segment_edge_obb_lateral_radius),
        "segment_edge_obb_longitudinal_margin": float(options.segment_edge_obb_longitudinal_margin),
        "segment_edge_obb_safety_epsilon": float(options.segment_edge_obb_safety_epsilon),
        "segment_edge_obb_grow_iterations": int(options.segment_edge_obb_grow_iterations),
        "segment_edge_obb_binary_iterations": int(options.segment_edge_obb_binary_iterations),
        "segment_edge_obb_split_depth": int(options.segment_edge_obb_split_depth),
        "obb_max_window_segments": int(options.obb_max_window_segments),
        "obb_max_validations_per_window": int(options.obb_max_validations_per_window),
        "obb_fast_primary_orientation": bool(getattr(options, "obb_fast_primary_orientation", True)),
        "obb_fallback_orientations_on_primary_fail": bool(
            getattr(options, "obb_fallback_orientations_on_primary_fail", False)
        ),
        "query_bridge_local_sample_assimilation": bool(options.query_bridge_local_sample_assimilation),
        "query_bridge_direct_partition_append_batch_size": int(
            options.query_bridge_direct_partition_append_batch_size
        ),
        "planning_s": offline_build_s + query_s,
        "planning_total_s": offline_build_s + query_total_s,
        "build_s": offline_build_s,
        "offline_build_s": offline_build_s,
        "offline_build_profile_s": offline_build_profile_s,
        "offline_coverage_profile": str(getattr(options, "offline_coverage_profile", "")),
        "offline_coverage_s": float(offline_coverage_s),
        "offline_connector_mode": offline_connector_mode,
        "offline_connector_s": float(offline_shortcut_s),
        "query_s": query_s,
        "query_total_s": query_total_s,
        "online_s": query_s,
        "online_batch_s": query_s,
        "online_total_s": query_total_s,
        "online_total_batch_s": query_total_s,
        "online_adaptation_s": online_adaptation_s,
        "online_solve_s": online_solve_s,
        "online_simplify_s": online_simplify_s,
        "online_per_query_s": online_per_query_s,
        "online_total_per_query_s": online_total_per_query_s,
        "online_solve_per_query_s": online_solve_per_query_s,
        "online_simplify_per_query_s": online_simplify_per_query_s,
        "graph_query_s": graph_query_s,
        "graph_query_total_s": graph_query_total_s,
        "graph_solve_s": graph_solve_s,
        "graph_simplify_s": graph_simplify_s,
        "graph_query_per_query_s": graph_query_s / query_count,
        "graph_query_total_per_query_s": graph_query_total_s / query_count,
        "graph_solve_per_query_s": graph_solve_s / query_count,
        "graph_simplify_per_query_s": graph_simplify_s / query_count,
        "partition_query_s": partition_query_s,
        "partition_query_total_s": partition_query_total_s,
        "partition_query_per_query_s": partition_query_s / query_count,
        "partition_query_total_per_query_s": partition_query_total_s / query_count,
        **amortized,
        "build_wall_s": offline_build_wall_s,
        "offline_anchor_select_s": float(offline_anchor_select_s),
        "offline_anchor_insert_s": float(offline_anchor_insert_s),
        "offline_query_agnostic_build": bool(options.offline_query_agnostic_build),
        "offline_backend": "adaptive_grid_partition" if str(options.offline_grower) == "adaptive_deep_leaf" else str(options.offline_grower),
        "online_backend": "partition_native" if partition_native_requested else "box_graph",
        "portal_membership_policy": int(diagnostics.get("portal_membership.policy", 0.0)),
        "portal_membership_global_forest_only": int(diagnostics.get("portal_membership.global_forest_only", 0.0)),
        "portal_membership_portal_interior_index": int(diagnostics.get("portal_membership.portal_interior_index", 0.0)),
        "portal_membership_global_forest_lookup": int(diagnostics.get("portal_membership.global_forest_lookup", 0.0)),
        "portal_membership_global_forest_only_fallback": int(diagnostics.get("portal_membership.global_forest_only_fallback", 0.0)),
        "portal_membership_portal_interior_index_unavailable": int(diagnostics.get("portal_membership.portal_interior_index_unavailable", 0.0)),
        "qroot_pairs_total": qroot_pairs_total,
        "qroot_uncovered_endpoints": qroot_uncovered_endpoints,
        "offline_anchor_candidates": int(offline_anchor_select_metrics.get("offline_anchor_candidates", 0)),
        "offline_anchor_candidates_free": int(offline_anchor_select_metrics.get("offline_anchor_candidates_free", 0)),
        "offline_anchor_roots_requested": int(offline_anchor_select_metrics.get("offline_anchor_roots_requested", 0)),
        "offline_anchor_roots_added": int(max(
            diagnostics.get("leaf_refine.offline_anchor_roots_added", 0.0),
            float(offline_anchor_insert_added),
        )),
        "offline_anchor_insert_attempts": int(offline_anchor_insert_attempts),
        "offline_anchor_lca_depth_mean": float(offline_anchor_select_metrics.get("offline_anchor_lca_depth_mean", math.nan)),
        "offline_anchor_lca_depth_max": float(offline_anchor_select_metrics.get("offline_anchor_lca_depth_max", math.nan)),
        "offline_anchor_min_distance_mean": float(offline_anchor_select_metrics.get("offline_anchor_min_distance_mean", math.nan)),
        "offline_anchor_skip_reason": str(offline_anchor_select_metrics.get("offline_anchor_skip_reason", "")),
        "offline_anchor_skip_p_main_accessible": float(offline_anchor_select_metrics.get("offline_anchor_skip_p_main_accessible", math.nan)),
        "offline_anchor_box_volume_mean": float(diagnostics.get("leaf_refine.offline_anchor_box_volume_mean", 0.0)),
        "offline_anchor_box_volume_max": float(diagnostics.get("leaf_refine.offline_anchor_box_volume_max", 0.0)),
        "offline_shortcut_s": float(offline_shortcut_s),
        "offline_shortcut_allow_segment_fallback": bool(allow_offline_segment_fallback),
        "offline_shortcut_edges_requested": int(options.offline_shortcut_edges),
        "offline_shortcut_edges_added": int(offline_shortcut_edges_added),
        "offline_shortcut_candidates": int(diagnostics.get("offline_shortcut.candidates", 0.0)),
        "offline_shortcut_tested_pairs": int(diagnostics.get("offline_shortcut.tested_pairs", 0.0)),
        "offline_shortcut_portal_corridor_edges_added": int(diagnostics.get("offline_shortcut.portal_corridor_edges_added", 0.0)),
        "offline_shortcut_portal_corridor_fail": int(diagnostics.get("offline_shortcut.portal_corridor_fail", 0.0)),
        "offline_shortcut_portal_corridor_attempts": int(diagnostics.get("offline_shortcut.portal_corridor_attempts", 0.0)),
        "offline_shortcut_portal_corridor_added": int(diagnostics.get("offline_shortcut.portal_corridor_added", 0.0)),
        "offline_shortcut_portal_corridor_internal_boxes": int(diagnostics.get("offline_shortcut.portal_corridor_internal_boxes", 0.0)),
        "offline_shortcut_portal_corridor_ffb_calls": int(diagnostics.get("offline_shortcut.portal_corridor_ffb_calls", 0.0)),
        "offline_shortcut_portal_corridor_cell_native_validations": int(diagnostics.get("offline_shortcut.portal_corridor_cell_native_validations", 0.0)),
        "offline_shortcut_portal_corridor_cell_native_free": int(diagnostics.get("offline_shortcut.portal_corridor_cell_native_free", 0.0)),
        "offline_shortcut_box_corridor_edges_added": int(diagnostics.get("offline_shortcut.box_corridor_edges_added", 0.0)),
        "offline_shortcut_segment_edges_added": int(diagnostics.get("offline_shortcut.segment_edges_added", 0.0)),
        "offline_shortcut_pave_boxes_added": int(diagnostics.get("offline_shortcut.pave_boxes_added", 0.0)),
        "offline_shortcut_pave_fail": int(diagnostics.get("offline_shortcut.pave_fail", 0.0)),
        "offline_box_edges_added": offline_box_edges_added,
        "offline_segment_edges_added": offline_segment_edges_added,
        "offline_islands_before": int(diagnostics.get("leaf_refine.offline_anchor_islands_before", 0.0)),
        "offline_islands_after": int(build.profile.adjacency_islands),
        "leaf_sweep_s": float(getattr(build, "leaf_sweep_ms", 0.0)) / 1000.0,
        "deep_refine_s": float(getattr(build, "deep_refine_ms", getattr(build, "adaptive_ms", 0.0))) / 1000.0,
        "adaptive_deep_leaf_s": float(getattr(build, "adaptive_ms", 0.0)) / 1000.0,
        "adaptive_target_depth": int(options.adaptive_target_depth),
        "adaptive_validated": int(getattr(build, "adaptive_validated", diagnostics.get("adaptive.validated", 0.0))),
        "adaptive_splits": int(getattr(build, "adaptive_splits", diagnostics.get("adaptive.splits", 0.0))),
        "adaptive_deferred": int(getattr(build, "adaptive_deferred", diagnostics.get("adaptive.deferred", 0.0))),
        "adaptive_promoted": int(getattr(build, "adaptive_promoted", diagnostics.get("adaptive.promoted", 0.0))),
        "adaptive_unresolved_domains": int(getattr(build, "unresolved_domains", diagnostics.get("adaptive.unresolved_domains", 0.0))),
        "adaptive_planning_backend": str(options.adaptive_planning_backend),
        "adaptive_grid_target_depth": int(options.adaptive_grid_target_depth),
        "adaptive_grid_face_index_enabled": bool(options.adaptive_grid_face_index_enabled),
        "adaptive_grid_planning_max_expansions": int(options.adaptive_grid_planning_max_expansions),
        "partition_cell_count": int(diagnostics.get("adaptive.partition_cells", getattr(build, "partition_cell_count", 0))),
        "partition_grid_cell_count": int(diagnostics.get("adaptive.partition_grid_cells", getattr(build, "partition_grid_cell_count", 0))),
        "partition_non_grid_cell_count": int(diagnostics.get("adaptive.partition_non_grid_cells", getattr(build, "partition_non_grid_cell_count", 0))),
        "partition_face_index_entries": int(diagnostics.get("adaptive.partition_face_index_entries", getattr(build, "partition_face_index_entries", 0))),
        "partition_point_index_dims": int(diagnostics.get("adaptive.partition_point_index_dims", 0)),
        "partition_point_index_entries": int(diagnostics.get("adaptive.partition_point_index_entries", 0)),
        "partition_point_index_overflow_cells": int(diagnostics.get("adaptive.partition_point_index_overflow_cells", 0)),
        "partition_sparse_virtual_cells": int(diagnostics.get("adaptive.partition_sparse_virtual_cells", 0)),
        "partition_sparse_virtual_grid_cells": int(diagnostics.get("adaptive.partition_sparse_virtual_grid_cells", 0)),
        "partition_sparse_virtual_non_grid_cells": int(diagnostics.get("adaptive.partition_sparse_virtual_non_grid_cells", 0)),
        "partition_sparse_virtual_exact_index_entries": int(diagnostics.get("adaptive.partition_sparse_virtual_exact_index_entries", 0)),
        "partition_sparse_virtual_max_address_depth": int(diagnostics.get("adaptive.partition_sparse_virtual_max_address_depth", 0)),
        "partition_sparse_virtual_ancestor_refs_avoided": int(diagnostics.get("adaptive.partition_sparse_virtual_ancestor_refs_avoided", 0)),
        "partition_sparse_virtual_index_ms": float(diagnostics.get("adaptive.partition_sparse_virtual_index_ms", 0.0)),
        "oracle_certified_free": int(_diagnostic_max(diagnostics, [
            "oracle.certified_free",
            "adaptive.oracle.certified_free",
            "grower.worker_oracle.certified_free",
        ])),
        "oracle_certified_occupied": int(_diagnostic_max(diagnostics, [
            "oracle.certified_occupied",
            "adaptive.oracle.certified_occupied",
            "grower.worker_oracle.certified_occupied",
        ])),
        "oracle_collision_possible": int(_diagnostic_max(diagnostics, [
            "oracle.collision_possible",
            "adaptive.oracle.collision_possible",
        ])),
        "partition_islands": int(diagnostics.get("adaptive.partition_islands", getattr(build, "partition_islands", 0))),
        "partition_largest_island": int(diagnostics.get("adaptive.partition_largest_island", getattr(build, "partition_largest_island", 0))),
        "partition_build_ms": float(diagnostics.get("adaptive.partition_build_ms", 0.0)),
        "partition_index_rebuild_ms": float(diagnostics.get("adaptive.partition_index_rebuild_ms", 0.0)),
        "partition_face_index_ms": float(diagnostics.get("adaptive.partition_face_index_ms", 0.0)),
        "partition_point_index_ms": float(diagnostics.get("adaptive.partition_point_index_ms", 0.0)),
        "partition_neighbor_cache_ms": float(diagnostics.get("adaptive.partition_neighbor_cache_ms", 0.0)),
        "partition_island_rebuild_ms": float(diagnostics.get("adaptive.partition_island_rebuild_ms", 0.0)),
        "partition_adjacency_candidates": int(diagnostics.get("adaptive.partition_adjacency_candidates", 0.0)),
        "partition_adjacency_tests": int(diagnostics.get("adaptive.partition_adjacency_tests", 0.0)),
        "partition_adjacency_edges": int(diagnostics.get("adaptive.partition_adjacency_edges", 0.0)),
        "partition_overlay_edges": int(diagnostics.get("adaptive.partition_overlay_edges", 0.0)),
        "adaptive_merge_input_boxes": int(diagnostics.get("adaptive.merge_input_boxes", 0.0)),
        "adaptive_merge_output_boxes": int(diagnostics.get("adaptive.merge_output_boxes", 0.0)),
        "adaptive_merge_grid_ms": float(diagnostics.get("adaptive.merge_grid_ms", 0.0)),
        "adaptive_merge_grid_merges": int(diagnostics.get("adaptive.merge_grid_merges", 0.0)),
        "adaptive_merge_grid_rounds": int(diagnostics.get("adaptive.merge_grid_rounds", 0.0)),
        "adaptive_merge_tree_ms": float(diagnostics.get("adaptive.merge_tree_ms", 0.0)),
        "adaptive_merge_tree_merges": int(diagnostics.get("adaptive.merge_tree_merges", 0.0)),
        "adaptive_merge_tree_rounds": int(diagnostics.get("adaptive.merge_tree_rounds", 0.0)),
        "adaptive_merge_containment_ms": float(diagnostics.get("adaptive.merge_containment_ms", 0.0)),
        "adaptive_merge_exact_ms": float(diagnostics.get("adaptive.merge_exact_ms", 0.0)),
        "adaptive_merge_containment_pruned": int(diagnostics.get("adaptive.merge_containment_pruned", 0.0)),
        "adaptive_partition_merge_containment_skipped": int(diagnostics.get("adaptive.partition_merge_containment_skipped", 0.0)),
        "adaptive_partition_merge_containment_bucket_entries": int(diagnostics.get("adaptive.partition_merge_containment_bucket_entries", 0.0)),
        "adaptive_partition_merge_containment_candidates": int(diagnostics.get("adaptive.partition_merge_containment_candidates", 0.0)),
        "adaptive_partition_merge_containment_tests": int(diagnostics.get("adaptive.partition_merge_containment_tests", 0.0)),
        "adaptive_partition_merge_containment_overflow": int(diagnostics.get("adaptive.partition_merge_containment_overflow", 0.0)),
        "adaptive_partition_merge_containment_ms": float(diagnostics.get("adaptive.partition_merge_containment_ms", 0.0)),
        "adaptive_partition_merge_line_ms": float(diagnostics.get("adaptive.partition_merge_line_ms", 0.0)),
        "adaptive_merge_exact_merges": int(diagnostics.get("adaptive.merge_exact_merges", 0.0)),
        "adaptive_merge_stop_reason": int(diagnostics.get("adaptive.merge_stop_reason", 0.0)),
        "adaptive_adjacency_ms": float(diagnostics.get("adaptive.adjacency_ms", 0.0)),
        "adaptive_adjacency_candidates": int(diagnostics.get("adaptive.adjacency_candidates", 0.0)),
        "adaptive_adjacency_exact_tests": int(diagnostics.get("adaptive.adjacency_exact_tests", 0.0)),
        "adaptive_adjacency_edges": int(diagnostics.get("adaptive.adjacency_edges", 0.0)),
        "adaptive_free_cap_reached": int(diagnostics.get("leaf_sweep.free_boxes_cap_reached", 0.0)),
        "adaptive_unresolved_dropped_by_cap": int(diagnostics.get("leaf_sweep.collision_boxes_dropped_by_cap", 0.0)),
        "coverage_probe_free_count": int(getattr(build, "seed_probe_free_count", diagnostics.get("adaptive.seed_probe_free_count", 0.0))),
        "coverage_box_covered_probability": float(getattr(build, "p_box_covered", diagnostics.get("adaptive.p_box_covered", math.nan))),
        "coverage_anchor_success_probability": float(getattr(build, "p_anchor_success", diagnostics.get("adaptive.p_anchor_success", math.nan))),
        "coverage_main_accessible_probability": float(getattr(build, "p_main_accessible", diagnostics.get("adaptive.p_main_accessible", math.nan))),
        "coverage_anchor_to_main_uncovered_probability": float(
            getattr(build, "p_anchor_to_main_uncovered", diagnostics.get("adaptive.p_anchor_to_main_uncovered", math.nan))
        ),
        "selected_leaf_depth": int(getattr(build, "selected_leaf_depth", diagnostics.get("adaptive.selected_leaf_depth", options.leaf_max_depth))),
        "adaptive_depth_readiness_met": bool(
            getattr(build, "adaptive_depth_readiness_met", diagnostics.get("adaptive.depth_readiness_met", 0.0) > 0.5)
        ),
        "adaptive_depth_stop_reason": str(getattr(build, "adaptive_depth_stop_reason", "")),
        "adaptive_depth_snapshots_json": str(getattr(build, "adaptive_depth_snapshots_json", "")),
        "rrt_grower_s": float(getattr(build, "rrt_grower_ms", 0.0)) / 1000.0,
        "connector_s": float(getattr(build, "connector_ms", 0.0)) / 1000.0,
        "corridor_refine_s": float(corridor_refine_s),
        "corridor_refine_added": int(corridor_refine_added),
        "corridor_refine_attempts": int(corridor_refine_attempts),
        "endpoint_main_s": float(diagnostics.get("endpoint_main.ms", 0.0)) / 1000.0,
        "endpoint_main_per_query_s": (float(diagnostics.get("endpoint_main.ms", 0.0)) / 1000.0) / query_count,
        "endpoint_main_success_count": int(diagnostics.get("endpoint_main.main_contact_success", 0.0)),
        "endpoint_main_fallback_to_e2e": int(diagnostics.get("endpoint_main.fallback_to_e2e", 0.0)),
        "query_bridge_s": float(query_bridge_s),
        "query_bridge_per_query_s": query_bridge_per_query_s,
        "query_bridge_added": int(query_bridge_added),
        "query_bridge_attempts": int(query_bridge_attempts),
        "query_bridge_by_label_s": {
            str(label): float(value)
            for label, value in query_bridge_by_label_s.items()
        },
        "query_bridge_added_by_label": {
            str(label): int(value)
            for label, value in query_bridge_added_by_label.items()
        },
        "query_bridge_hipac_online_attempts": int(diagnostics.get("query_bridge.hipac_online_attempts", 0.0)),
        "query_bridge_hipac_online_added": int(diagnostics.get("query_bridge.hipac_online_added", 0.0)),
        "query_bridge_hipac_online_satisfied": int(diagnostics.get("query_bridge.hipac_online_satisfied", 0.0)),
        "query_bridge_hipac_online_failures": int(diagnostics.get("query_bridge.hipac_online_failures", 0.0)),
        "query_bridge_hipac_online_not_sufficient": int(diagnostics.get("query_bridge.hipac_online_not_sufficient", 0.0)),
        "query_bridge_hipac_online_ms": float(diagnostics.get("query_bridge.hipac_online_ms_total", 0.0)),
        "query_bridge_hipac_online_box_edges": int(diagnostics.get("query_bridge.hipac_online.partition_box_corridor_overlay_added", 0.0)),
        "query_bridge_hipac_online_portal_edges": int(diagnostics.get("query_bridge.hipac_online.portal_corridor_added", 0.0)),
        "query_bridge_hipac_online_internal_boxes": int(diagnostics.get("query_bridge.hipac_online.portal_corridor_internal_boxes", 0.0)),
        "query_bridge_hipac_online_ffb_calls": int(diagnostics.get("query_bridge.hipac_online.portal_corridor_ffb_calls", 0.0)),
        "query_bridge_hipac_online_cell_native_validations": int(diagnostics.get("query_bridge.hipac_online.portal_corridor_cell_native_validations", 0.0)),
        "query_bridge_hipac_online_cell_native_free": int(diagnostics.get("query_bridge.hipac_online.portal_corridor_cell_native_free", 0.0)),
        "query_bridge_hipac_prebridge_attempts": int(diagnostics.get("query_bridge.hipac_prebridge_attempts", 0.0)),
        "query_bridge_hipac_prebridge_candidates": int(diagnostics.get("query_bridge.hipac_prebridge_candidates", 0.0)),
        "query_bridge_hipac_prebridge_portal_attempts": int(diagnostics.get("query_bridge.hipac_prebridge_portal_attempts", 0.0)),
        "query_bridge_hipac_prebridge_added": int(diagnostics.get("query_bridge.hipac_prebridge_added", 0.0)),
        "query_bridge_hipac_prebridge_satisfied": int(diagnostics.get("query_bridge.hipac_prebridge_satisfied", 0.0)),
        "query_bridge_hipac_prebridge_failures": int(diagnostics.get("query_bridge.hipac_prebridge_failures", 0.0)),
        "query_bridge_hipac_prebridge_not_sufficient": int(diagnostics.get("query_bridge.hipac_prebridge_not_sufficient", 0.0)),
        "query_bridge_hipac_prebridge_ms": float(diagnostics.get("query_bridge.hipac_prebridge_ms_total", 0.0)),
        "query_bridge_hipac_prebridge_portal_edges": int(diagnostics.get("query_bridge.hipac_online_prebridge.portal_corridor_added", 0.0)),
        "query_bridge_hipac_prebridge_internal_boxes": int(diagnostics.get("query_bridge.hipac_online_prebridge.portal_corridor_internal_boxes", 0.0)),
        "query_bridge_hipac_prebridge_ffb_calls": int(diagnostics.get("query_bridge.hipac_online_prebridge.portal_corridor_ffb_calls", 0.0)),
        "query_bridge_hipac_prebridge_cell_native_validations": int(diagnostics.get("query_bridge.hipac_online_prebridge.portal_corridor_cell_native_validations", 0.0)),
        "query_bridge_hipac_prebridge_cell_native_free": int(diagnostics.get("query_bridge.hipac_online_prebridge.portal_corridor_cell_native_free", 0.0)),
        "query_bridge_hipac_transition_attempts": int(diagnostics.get("query_bridge.hipac_transition_attempts", 0.0)),
        "query_bridge_hipac_transition_candidates": int(diagnostics.get("query_bridge.hipac_transition_candidates", 0.0)),
        "query_bridge_hipac_transition_gated": int(diagnostics.get("query_bridge.hipac_transition_gated", 0.0)),
        "query_bridge_hipac_transition_portal_attempts": int(diagnostics.get("query_bridge.hipac_transition_portal_attempts", 0.0)),
        "query_bridge_hipac_transition_added": int(diagnostics.get("query_bridge.hipac_transition_added", 0.0)),
        "query_bridge_hipac_transition_satisfied": int(diagnostics.get("query_bridge.hipac_transition_satisfied", 0.0)),
        "query_bridge_hipac_transition_not_sufficient": int(diagnostics.get("query_bridge.hipac_transition_not_sufficient", 0.0)),
        "query_bridge_hipac_transition_failures": int(diagnostics.get("query_bridge.hipac_transition_failures", 0.0)),
        "query_bridge_hipac_transition_ms": float(diagnostics.get("query_bridge.hipac_transition_ms_total", 0.0)),
        "query_bridge_hipac_transition_portal_edges": int(diagnostics.get("query_bridge.hipac_online_transition.portal_corridor_added", 0.0)),
        "query_bridge_hipac_transition_internal_boxes": int(diagnostics.get("query_bridge.hipac_online_transition.portal_corridor_internal_boxes", 0.0)),
        "query_bridge_hipac_transition_ffb_calls": int(diagnostics.get("query_bridge.hipac_online_transition.portal_corridor_ffb_calls", 0.0)),
        "query_bridge_hipac_transition_cell_native_validations": int(diagnostics.get("query_bridge.hipac_online_transition.portal_corridor_cell_native_validations", 0.0)),
        "query_bridge_hipac_transition_cell_native_free": int(diagnostics.get("query_bridge.hipac_online_transition.portal_corridor_cell_native_free", 0.0)),
        "query_bridge_hipac_transition_obb_attempts": int(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_attempts", 0.0)),
        "query_bridge_hipac_transition_obb_success": int(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_success", 0.0)),
        "query_bridge_hipac_transition_obb_fail": int(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_fail", 0.0)),
        "query_bridge_hipac_transition_obb_edge_fail": int(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_edge_fail", 0.0)),
        "query_bridge_hipac_transition_obb_joint_limit_rejects": int(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_joint_limit_rejects", 0.0)),
        "query_bridge_hipac_transition_obb_gjk_tests": int(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_gjk_tests", 0.0)),
        "query_bridge_hipac_transition_obb_maybe_pairs": int(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_maybe_pairs", 0.0)),
        "query_bridge_hipac_transition_obb_ms": float(diagnostics.get("query_bridge.hipac_online_transition.obb_zonotope_ms", 0.0)),
        "segment_edge_obb_cover_attempts": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_attempts")
        )),
        "segment_edge_obb_cover_success": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_success")
        )),
        "segment_edge_obb_cover_fail": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_fail")
        )),
        "segment_edge_obb_cover_validations": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_validations")
        )),
        "segment_edge_obb_cover_valid_candidates": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_valid_candidates",
        )),
        "segment_edge_obb_cover_grow_attempts": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_grow_attempts",
        )),
        "segment_edge_obb_cover_regions": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_regions")
        )),
        "segment_edge_obb_cover_region_volume_sum": float(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_region_volume_sum",
        )),
        "segment_edge_obb_cover_region_volume_max": float(_diagnostic_max_suffix(
            diagnostics,
            ".segment_obb_cover_region_volume_max",
        )),
        "segment_edge_obb_cover_region_log_volume_sum": float(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_region_log_volume_sum",
        )),
        "segment_edge_obb_cover_region_volume_count": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_region_volume_count",
        )),
        "segment_edge_obb_cover_clearance_support_attempts": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_clearance_support_attempts",
        )),
        "segment_edge_obb_cover_clearance_support_success": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_clearance_support_success",
        )),
        "segment_edge_obb_cover_clearance_support_fail": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_clearance_support_fail",
        )),
        "segment_edge_obb_cover_clearance_support_samples": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_clearance_support_samples",
        )),
        "segment_edge_obb_cover_clearance_support_error_radius": float(_diagnostic_max_suffix(
            diagnostics,
            ".segment_obb_cover_clearance_support_error_radius",
        )),
        "segment_edge_obb_cover_clearance_support_min_margin": float(_diagnostic_first_suffix(
            diagnostics,
            ".segment_obb_cover_clearance_support_min_margin",
        )),
        "segment_edge_obb_cover_windows_attempted": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_windows_attempted")
        )),
        "segment_edge_obb_cover_windows_success": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_windows_success")
        )),
        "segment_edge_obb_cover_replaced_segments": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_replaced_segments")
        )),
        "segment_edge_obb_cover_partial_edges": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_partial_edges")
        )),
        "segment_edge_obb_cover_partial_regions": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_partial_regions")
        )),
        "segment_edge_obb_cover_partial_committed": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_partial_committed")
        )),
        "segment_edge_obb_cover_metadata_only": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_metadata_only",
        )),
        "segment_edge_obb_cover_metadata_only_segments": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_metadata_only_segments",
        )),
        "segment_edge_obb_cover_metadata_require_cover_reject": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_metadata_require_cover_reject",
        )),
        "segment_edge_obb_cover_partial_covered_length": float(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_partial_covered_length")
        )),
        "segment_edge_obb_cover_ms": float(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".segment_obb_cover_ms")
        )),
        "segment_edge_obb_cover_strict_reject": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_strict_reject",
        )),
        "segment_edge_obb_cover_edge_fail": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_edge_fail",
        )),
        "segment_edge_obb_cover_missing_box": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_missing_box",
        )),
        "segment_edge_obb_cover_failed_leaf_windows": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_failed_leaf_windows",
        )),
        "segment_edge_obb_cover_failed_leaf_length_sum": float(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_failed_leaf_length_sum",
        )),
        "segment_edge_obb_cover_failed_leaf_length_max": float(_diagnostic_max_suffix(
            diagnostics,
            ".segment_obb_cover_failed_leaf_length_max",
        )),
        "segment_edge_obb_cover_recursive_splits": int(_diagnostic_sum_suffix(
            diagnostics,
            ".segment_obb_cover_recursive_splits",
        )),
        "segment_edge_obb_cover_first_failed_leaf_recorded": int(_diagnostic_max_suffix(
            diagnostics,
            ".segment_obb_cover_first_failed_leaf_recorded",
        )),
        "segment_edge_obb_cover_first_failed_leaf_exact": int(_diagnostic_max_suffix(
            diagnostics,
            ".segment_obb_cover_first_failed_leaf_exact",
        )),
        "segment_edge_obb_cover_first_failed_leaf_length": float(_diagnostic_first_suffix(
            diagnostics,
            ".segment_obb_cover_first_failed_leaf_length",
        )),
        "segment_edge_obb_cover_first_failed_leaf_dims": int(_diagnostic_first_suffix(
            diagnostics,
            ".segment_obb_cover_first_failed_leaf_dims",
            0.0,
        )),
        "rrt_bridge_obb_cover_attempts": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_attempts")
        )),
        "rrt_bridge_obb_cover_success": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_success")
        )),
        "rrt_bridge_obb_cover_fail": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_fail")
        )),
        "rrt_bridge_obb_cover_regions": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_regions")
        )),
        "rrt_bridge_obb_cover_region_volume_sum": float(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_region_volume_sum",
        )),
        "rrt_bridge_obb_cover_region_volume_max": float(_diagnostic_max_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_region_volume_max",
        )),
        "rrt_bridge_obb_cover_region_log_volume_sum": float(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_region_log_volume_sum",
        )),
        "rrt_bridge_obb_cover_region_volume_count": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_region_volume_count",
        )),
        "rrt_bridge_obb_cover_validations": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_validations")
        )),
        "rrt_bridge_obb_cover_valid_candidates": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_valid_candidates",
        )),
        "rrt_bridge_obb_cover_grow_attempts": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_grow_attempts",
        )),
        "rrt_bridge_obb_cover_clearance_support_attempts": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_clearance_support_attempts",
        )),
        "rrt_bridge_obb_cover_clearance_support_success": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_clearance_support_success",
        )),
        "rrt_bridge_obb_cover_clearance_support_fail": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_clearance_support_fail",
        )),
        "rrt_bridge_obb_cover_clearance_support_samples": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_clearance_support_samples",
        )),
        "rrt_bridge_obb_cover_clearance_support_error_radius": float(_diagnostic_max_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_clearance_support_error_radius",
        )),
        "rrt_bridge_obb_cover_clearance_support_min_margin": float(_diagnostic_first_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_clearance_support_min_margin",
        )),
        "rrt_bridge_obb_cover_replaced_segments": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_replaced_segments")
        )),
        "rrt_bridge_obb_cover_partial_edges": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_partial_edges")
        )),
        "rrt_bridge_obb_cover_partial_regions": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_partial_regions")
        )),
        "rrt_bridge_obb_cover_partial_committed": int(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_partial_committed")
        )),
        "rrt_bridge_obb_cover_metadata_only": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_metadata_only",
        )),
        "rrt_bridge_obb_cover_metadata_only_segments": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_metadata_only_segments",
        )),
        "rrt_bridge_obb_cover_metadata_require_cover_reject": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_metadata_require_cover_reject",
        )),
        "rrt_bridge_obb_cover_partial_covered_length": float(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_partial_covered_length")
        )),
        "rrt_bridge_obb_cover_ms": float(sum(
            value for key, value in diagnostics.items()
            if key.endswith(".rrt_bridge_obb_cover_ms")
        )),
        "rrt_bridge_obb_cover_strict_reject": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_strict_reject",
        )),
        "rrt_bridge_obb_cover_edge_fail": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_edge_fail",
        )),
        "rrt_bridge_obb_cover_missing_box": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_missing_box",
        )),
        "rrt_bridge_obb_cover_failed_leaf_windows": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_failed_leaf_windows",
        )),
        "rrt_bridge_obb_cover_failed_leaf_length_sum": float(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_failed_leaf_length_sum",
        )),
        "rrt_bridge_obb_cover_failed_leaf_length_max": float(_diagnostic_max_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_failed_leaf_length_max",
        )),
        "rrt_bridge_obb_cover_recursive_splits": int(_diagnostic_sum_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_recursive_splits",
        )),
        "rrt_bridge_obb_cover_first_failed_leaf_recorded": int(_diagnostic_max_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_first_failed_leaf_recorded",
        )),
        "rrt_bridge_obb_cover_first_failed_leaf_exact": int(_diagnostic_max_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_first_failed_leaf_exact",
        )),
        "rrt_bridge_obb_cover_first_failed_leaf_length": float(_diagnostic_first_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_first_failed_leaf_length",
        )),
        "rrt_bridge_obb_cover_first_failed_leaf_dims": int(_diagnostic_first_suffix(
            diagnostics,
            ".rrt_bridge_obb_cover_first_failed_leaf_dims",
            0.0,
        )),
        **{
            f"rrt_bridge_obb_cover_first_failed_leaf_a_{dim}": float(_diagnostic_first_suffix(
                diagnostics,
                f".rrt_bridge_obb_cover_first_failed_leaf_a_{dim}",
            ))
            for dim in range(10)
        },
        **{
            f"rrt_bridge_obb_cover_first_failed_leaf_b_{dim}": float(_diagnostic_first_suffix(
                diagnostics,
                f".rrt_bridge_obb_cover_first_failed_leaf_b_{dim}",
            ))
            for dim in range(10)
        },
        **{
            f"segment_edge_obb_cover_first_failed_leaf_a_{dim}": float(_diagnostic_first_suffix(
                diagnostics,
                f".segment_edge_obb_cover_first_failed_leaf_a_{dim}",
            ))
            for dim in range(10)
        },
        **{
            f"segment_edge_obb_cover_first_failed_leaf_b_{dim}": float(_diagnostic_first_suffix(
                diagnostics,
                f".segment_edge_obb_cover_first_failed_leaf_b_{dim}",
            ))
            for dim in range(10)
        },
        "obb_edges_used_total": int(sum(int(row.get("obb_edges_used", 0)) for row in qrows)),
        "obb_regions_used_total": int(sum(int(row.get("obb_regions_used", 0)) for row in qrows)),
        "obb_edge_length_total": float(sum(float(row.get("obb_edge_length", 0.0)) for row in qrows)),
        "query_bridge_hipac_promote_transition_attempts": int(diagnostics.get("query_bridge.hipac_promote_transition.attempts", 0.0)),
        "query_bridge_hipac_promote_transition_target_rejects": int(diagnostics.get("query_bridge.hipac_promote_transition.target_rejects", 0.0)),
        "query_bridge_hipac_promote_transition_candidate_boxes": int(diagnostics.get("query_bridge.hipac_promote_transition.candidate_boxes", 0.0)),
        "query_bridge_hipac_promote_transition_added": int(diagnostics.get("query_bridge.hipac_promote_transition.added", 0.0)),
        "query_bridge_hipac_promote_transition_failures": int(diagnostics.get("query_bridge.hipac_promote_transition.failures", 0.0)),
        "query_bridge_hipac_promote_transition_chain_fail": int(diagnostics.get("query_bridge.hipac_promote_transition.chain_fail", 0.0)),
        "query_bridge_hipac_promote_transition_short_chain": int(diagnostics.get("query_bridge.hipac_promote_transition.short_chain", 0.0)),
        "query_bridge_hipac_promote_transition_edge_fail": int(diagnostics.get("query_bridge.hipac_promote_transition.edge_fail", 0.0)),
        "query_bridge_hipac_promote_transition_slice_components": int(diagnostics.get("query_bridge.hipac_promote_transition.slice_components", 0.0)),
        "query_bridge_hipac_promote_transition_added_full": int(diagnostics.get("query_bridge.hipac_promote_transition.added_full", 0.0)),
        "query_bridge_hipac_promote_transition_added_slice": int(diagnostics.get("query_bridge.hipac_promote_transition.added_slice", 0.0)),
        "query_bridge_hipac_promote_transition_local_adj_edges": int(diagnostics.get("query_bridge.hipac_promote_transition.local_adj_edges", 0.0)),
        "query_bridge_hipac_promote_transition_local_adj_tests": int(diagnostics.get("query_bridge.hipac_promote_transition.local_adj_tests", 0.0)),
        "query_bridge_hipac_promote_transition_internal_boxes": int(diagnostics.get("query_bridge.hipac_promote_transition.internal_boxes", 0.0)),
        "query_bridge_hipac_promote_transition_ms": float(diagnostics.get("query_bridge.hipac_promote_transition.ms_total", 0.0)),
        "query_bridge_hipac_promote_attempts": int(diagnostics.get("query_bridge.hipac_promote_attempts", 0.0)),
        "query_bridge_hipac_promote_added": int(diagnostics.get("query_bridge.hipac_promote_added", 0.0)),
        "query_bridge_hipac_promote_failures": int(diagnostics.get("query_bridge.hipac_promote_failures", 0.0)),
        "query_bridge_hipac_promote_ms": float(diagnostics.get("query_bridge.hipac_promote_ms_total", 0.0)),
        "query_bridge_hipac_promote_portal_edges": int(diagnostics.get("query_bridge.hipac_promote.portal_corridor_added", 0.0)),
        "query_bridge_hipac_promote_internal_boxes": int(diagnostics.get("query_bridge.hipac_promote.portal_corridor_internal_boxes", 0.0)),
        "query_bridge_hipac_promote_ffb_calls": int(diagnostics.get("query_bridge.hipac_promote.portal_corridor_ffb_calls", 0.0)),
        "query_bridge_hipac_promote_cell_native_validations": int(diagnostics.get("query_bridge.hipac_promote.portal_corridor_cell_native_validations", 0.0)),
        "query_bridge_hipac_promote_cell_native_free": int(diagnostics.get("query_bridge.hipac_promote.portal_corridor_cell_native_free", 0.0)),
        **query_bridge_diagnostic_fields,
        "audit_s": sum(float(row["audit_ms"]) for row in qrows) / 1000.0,
        "path_length_mean": mean(row["path_length"] for row in successes),
        "raw_segment_fraction": (total_seg / total_len) if total_len > 1e-12 else math.nan,
        "leaf_free_count": int(getattr(build, "leaf_free_count", getattr(build, "shallow_free_count", 0))),
        "leaf_collision_count": int(getattr(build, "leaf_collision_count", getattr(build, "shallow_collision_count", 0))),
        "deep_boxes_added": int(getattr(build, "deep_boxes_added", getattr(build, "adaptive_free_added", 0))),
        "rrt_grower_boxes_added": int(getattr(build, "rrt_grower_boxes_added", 0)),
        "build_final_boxes": build_final_boxes,
        "build_segment_edges": build_segment_edges,
        "after_corridor_boxes": int(after_corridor_boxes),
        "after_corridor_segment_edges": int(after_corridor_segment_edges),
        "query_bridge_boxes_added_observed": int(final_boxes - after_corridor_boxes),
        "query_bridge_segment_edges_added_observed": int(final_segment_edges - after_corridor_segment_edges),
        "final_boxes": int(final_boxes),
        "final_segment_edges": int(final_segment_edges),
        "final_adjacency_islands": int(reported_final_islands),
        "legacy_graph_final_adjacency_islands": int(final_adjacency_islands),
        "segment_edges": int(final_segment_edges),
        "adjacency_islands": int(build.profile.adjacency_islands),
        **external_evidence_fields,
        "database_root_intervals": interval_pairs(forest.database_root_intervals()),
        "database_coverage_intervals": interval_pairs(forest.database_coverage_intervals()),
        "queries": qrows,
        "diagnostics": diagnostics,
    }
