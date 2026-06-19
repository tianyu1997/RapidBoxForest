#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
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
from experiments.common.rbf_leaf_rrt import (
    RBFLeafRRTOptions,
    configure_leaf_rrt as configure_leaf_rrt_options,
    make_refine_config as make_leaf_refine_config,
    run_leaf_rrt,
)
from experiments.common.rbf_defaults import (
    CRITSAMPLE_D23_CACHE_LABEL,
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    DEFAULT_RBF_AUDIT_RESOLUTION,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD,
    DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD,
    DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES,
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_PAVE_DEPTH,
    DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS,
    DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN,
    DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN,
    DEFAULT_RBF_CONNECTOR_PAVE_STEPS,
    DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FFB_DEPTHS,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_TARGET_SEGMENT_FRACTION,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR,
    DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH,
    DEFAULT_RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP,
    DEFAULT_RBF_QUERY_BRIDGE_FORCE_SELECTED,
    DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS,
    DEFAULT_RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS,
    DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS,
    DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS,
    DEFAULT_RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES,
    DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS,
    DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS,
    DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY,
    DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY,
    DEFAULT_RBF_QUERY_ENDPOINT_ANCHOR_BEFORE_BRIDGE,
    DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS,
    DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
    DEFAULT_RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS,
    DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS,
    DEFAULT_RBF_CONNECTOR_RRT_ITERS,
    DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE,
    DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION,
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_SHELF_BOX_BUDGET,
    DEFAULT_RBF_DOMAIN_ATTEMPT_CAP,
    DEFAULT_RBF_DOMAIN_SEED_CAP,
    DEFAULT_RBF_DOMAIN_SUCCESS_CAP,
    DEFAULT_RBF_FINAL_COLLISION_SHORTCUT,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
    DEFAULT_RBF_QUERY_BRIDGE_ALL,
    DEFAULT_RBF_QUERY_BRIDGE_FORCE_INDICES,
    DEFAULT_RBF_QUERY_BRIDGE_LABELS,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_REFINE_TIMEOUT_MS,
    DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES,
    DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS,
    DEFAULT_RBF_THREADS,
    DEFAULT_RBF_VALIDATION_BATCH_SIZE,
    RBF_OFFLINE_COVERAGE_PROFILE_NAME,
    apply_offline_coverage_profile,
    offline_coverage_v1_profile,
    robot_joint_limit_tuples,
    robot_symmetry_aligned_root_tuples,
    shelf_d23_rbf_profile,
)
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


ABLATIONS = [
    "baseline_d23_aafk_support_hull_8t",
    "critsample_d23_cache",
    "no_cache_full_root_ts",
    "critsample_support_hull",
    "no_external_lect",
    "support_hull_no_aabb",
    "link_aabb",
    "single_thread",
]

CASE_LABELS = {
    "baseline_d23_aafk_support_hull_8t": "RBF-SH d23",
    "critsample_d23_cache": "CritSample d23",
    "no_cache_full_root_ts": "No-cache full-root TS",
    "critsample_support_hull": "CritSample endpoints",
    "no_external_lect": "No external LECT, HIFK-5",
    "support_hull_no_aabb": "SH w/o broadphase",
    "link_aabb": "Link AABB",
    "single_thread": "No LECT, 1 thread",
}

CASE_ALIASES = {
    "critsample_support_hull_unsafe": "critsample_support_hull",
}

CACHE_DIFF_KEYS = {
    "option.use_external_evidence",
    "option.external_evidence_live_retry_on_maybe",
    "cfg.validation.external_evidence_live_retry_on_maybe",
    "cfg.database.external_evidence_path",
    "cfg.database.external_evidence_use_snapshot",
    "cfg.database.external_evidence_auto_build_snapshot",
}

THREAD_DIFF_KEYS = {
    "option.threads",
    "option.leaf_threads",
    "cfg.runtime.mode",
    "cfg.runtime.n_threads",
    "cfg.runtime.batch_size",
    "cfg.grower.n_threads",
    "cfg.grower.task_batch_size",
    "cfg.connector.n_threads",
    "ref.leaf_threads",
}

ALLOWED_CONFIG_DIFFS = {
    "baseline_d23_aafk_support_hull_8t": set(),
    "critsample_d23_cache": {
        "option.endpoint_source",
        "cfg.endpoint_source.source",
    },
    "no_cache_full_root_ts": set(),
    "no_external_lect": {
        "option.endpoint_source",
        "option.hifk_max_depth",
        "cfg.endpoint_source.source",
        "cfg.endpoint_source.hifk_max_depth",
    },
    "support_hull_no_aabb": {
        "option.support_hull_skip_aabb_broadphase",
        "cfg.envelope_type.support_hull_config.skip_aabb_broadphase",
    },
    "link_aabb": {
        "option.envelope",
        "cfg.envelope_type.type",
    },
    "single_thread": set(THREAD_DIFF_KEYS),
    "critsample_support_hull": {
        "option.endpoint_source",
        "cfg.endpoint_source.source",
    },
}


def configure_thread_environment(threads: int) -> None:
    value = str(max(1, int(threads)))
    for key in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
    ):
        os.environ[key] = value


def query_rows(forest: Any, robot: Any, queries: list[Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query_index, query in enumerate(queries):
        start = [float(value) for value in query.start]
        goal = [float(value) for value in query.goal]
        previous_active_query = os.environ.get("RBF_ACTIVE_QUERY_INDEX")
        os.environ["RBF_ACTIVE_QUERY_INDEX"] = str(query_index)
        try:
            result = forest.query(start, goal)
        finally:
            if previous_active_query is None:
                os.environ.pop("RBF_ACTIVE_QUERY_INDEX", None)
            else:
                os.environ["RBF_ACTIVE_QUERY_INDEX"] = previous_active_query
        path_length = float(result.path_length) if bool(result.success) else math.nan
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
        solve_ms = max(0.0, query_ms - simplify_ms)
        rows.append({
            "label": str(query.label),
            "success": bool(result.success),
            "audit_passed": bool(result.audit_passed),
            "query_ms": query_ms,
            "solve_ms": solve_ms,
            "simplify_ms": simplify_ms,
            "audit_ms": float(result.audit_time_ms),
            "final_simplify_ms": simplify_ms,
            "path_length": path_length,
            "final_path_length": path_length,
            "raw_path_length": raw_path_length,
            "segment_edge_length": segment_length,
            "segment_residual_length": residual_segment_length,
            "segment_fraction": residual_segment_fraction,
            "box_sequence_len": len(list(result.box_sequence)),
            "segment_edges_used": int(result.segment_edges_used),
            "obb_edges_used": int(getattr(result, "obb_edges_used", 0)),
            "obb_regions_used": int(getattr(result, "obb_regions_used", 0)),
            "obb_edge_length": float(getattr(result, "obb_edge_length", 0.0)),
            "residual_segment_fraction": residual_segment_fraction,
            "waypoint_count": len(result.path_as_lists()),
            "audit_status": str(result.audit_status),
            "actual_start": start,
            "actual_goal": goal,
        })
    return rows


def make_case_options(case: str, seed: int, deep_max_boxes: int, args: argparse.Namespace) -> tuple[Any, RBFLeafRRTOptions]:
    robot = sbf.load_iiwa14_robot()
    threads = 1 if case == "single_thread" else int(args.threads)
    task_root = robot_joint_limit_tuples(robot)
    # Canonical semantics stay inside LECT: the active database root is the
    # canonical primary-sector root selected by SBF when no override is passed,
    # while coverage/query inputs remain in native joint space.
    active_root = None
    coverage_root = task_root
    warm_cache_label = str(args.warm_cache_label)
    if case == "critsample_d23_cache" and warm_cache_label == D23_CACHE_LABEL:
        warm_cache_label = CRITSAMPLE_D23_CACHE_LABEL
    leaf_max_depth = int(args.leaf_max_depth)
    adaptive_target_depth = int(args.adaptive_target_depth)
    hipac_improved = bool(args.hipac_improved_leaf_sweep)
    hipac_portal_connectivity = bool(args.hipac_portal_connectivity) or hipac_improved
    hipac_online_connectivity = bool(args.hipac_online_connectivity) or hipac_improved
    hipac_online_prebridge_portal = bool(args.hipac_online_prebridge_portal) or hipac_improved
    query_bridge_no_path_retry_stop_on_first_success = (
        bool(args.query_bridge_no_path_retry_stop_on_first_success) or hipac_improved
    )
    # TransitionPortal remains disabled unless the new OBB-zonotope
    # certificate is explicitly requested.  This avoids enabling the older
    # unvalidated transition resolver in paper runs.
    hipac_online_transition_portal = (
        bool(args.hipac_online_transition_portal) and bool(args.hipac_transition_obb_portal)
    )
    hipac_promote_transition_slices = (
        bool(args.hipac_promote_transition_slices) and bool(args.hipac_transition_obb_portal)
    )
    options = RBFLeafRRTOptions(
        seed=int(seed),
        offline_grower=str(args.offline_grower),
        deep_max_boxes=int(deep_max_boxes),
        rbf_max_depth=int(args.rbf_max_depth),
        timeout_ms=float(args.timeout_ms),
        threads=threads,
        leaf_start_depth=int(args.leaf_start_depth),
        leaf_max_depth=leaf_max_depth,
        adaptive_target_depth=adaptive_target_depth,
        adaptive_time_budget_ms=float(args.adaptive_time_budget_ms),
        adaptive_node_budget=int(args.adaptive_node_budget),
        adaptive_fast_virtual_checkpoint_mode=bool(args.adaptive_fast_virtual_checkpoint_mode),
        adaptive_defer_min_depth=int(args.adaptive_defer_min_depth),
        adaptive_overlap_depth_threshold=float(args.adaptive_overlap_depth_threshold),
        adaptive_overlap_depth_min_threshold=float(args.adaptive_overlap_depth_min_threshold),
        adaptive_overlap_depth_decay_per_depth=float(args.adaptive_overlap_depth_decay_per_depth),
        adaptive_overlap_ratio_threshold=float(args.adaptive_overlap_ratio_threshold),
        adaptive_seed_probe_count=int(args.coverage_probe_count),
        adaptive_seed_anchor_probe_cap=int(args.adaptive_seed_anchor_probe_cap),
        adaptive_depth_enabled=bool(args.adaptive_depth_enabled),
        adaptive_depth_min=int(args.adaptive_depth_min),
        adaptive_depth_max=int(args.adaptive_depth_max),
        adaptive_depth_probe_count=int(args.adaptive_depth_probe_count),
        adaptive_depth_anchor_probe_cap=int(args.adaptive_depth_anchor_probe_cap),
        adaptive_depth_probe_seed=int(args.adaptive_depth_probe_seed),
        adaptive_depth_min_free_probes=int(args.adaptive_depth_min_free_probes),
        adaptive_depth_min_covered_probes=int(args.adaptive_depth_min_covered_probes),
        adaptive_depth_min_main_probes=int(args.adaptive_depth_min_main_probes),
        adaptive_depth_min_main_ratio=float(args.adaptive_depth_min_main_ratio),
        adaptive_depth_min_cells=int(args.adaptive_depth_min_cells),
        adaptive_depth_min_main_cells=int(args.adaptive_depth_min_main_cells),
        adaptive_depth_max_online_cells=int(args.adaptive_depth_max_online_cells),
        adaptive_depth_max_probe_ms=float(args.adaptive_depth_max_probe_ms),
        adaptive_max_merge_ms=float(args.adaptive_max_merge_ms),
        adaptive_max_merge_rounds=int(args.adaptive_max_merge_rounds),
        adaptive_max_merge_input_boxes=int(args.adaptive_max_merge_input_boxes),
        adaptive_max_free_boxes=min(int(args.adaptive_max_free_boxes), int(deep_max_boxes)),
        adaptive_max_unresolved_domains=int(args.adaptive_max_unresolved_domains),
        hipac_portal_connectivity=hipac_portal_connectivity,
        hipac_portal_cell_native_validate=bool(args.hipac_portal_cell_native_validate),
        hipac_portal_max_internal_boxes=int(args.hipac_portal_max_internal_boxes),
        hipac_portal_max_recursion_depth=int(args.hipac_portal_max_recursion_depth),
        hipac_portal_ffb_depth=int(args.hipac_portal_ffb_depth),
        hipac_portal_ffb_deadline_ms=float(args.hipac_portal_ffb_deadline_ms),
        hipac_online_connectivity=hipac_online_connectivity,
        hipac_online_before_query_bridge=bool(args.hipac_online_before_query_bridge),
        hipac_promote_query_repairs=bool(args.hipac_promote_query_repairs),
        hipac_online_ffb_portal_fallback=bool(args.hipac_online_ffb_portal_fallback),
        hipac_online_candidate_max_length=float(args.hipac_online_candidate_max_length),
        hipac_online_max_resolves_per_query=int(args.hipac_online_max_resolves_per_query),
        hipac_online_max_hidden_boxes_per_portal=int(args.hipac_online_max_hidden_boxes_per_portal),
        hipac_online_max_ffb_calls_per_portal=int(args.hipac_online_max_ffb_calls_per_portal),
        hipac_online_prebridge_portal=hipac_online_prebridge_portal,
        hipac_online_prebridge_candidate_limit=int(args.hipac_online_prebridge_candidate_limit),
        hipac_online_prebridge_max_pair_distance=float(args.hipac_online_prebridge_max_pair_distance),
        hipac_online_prebridge_route_distance_weight=float(args.hipac_online_prebridge_route_distance_weight),
        hipac_online_prebridge_pair_distance_weight=float(args.hipac_online_prebridge_pair_distance_weight),
        hipac_online_transition_portal=hipac_online_transition_portal,
        hipac_transition_target_query_indices=str(args.hipac_transition_target_query_indices),
        hipac_transition_max_attempts_per_query=int(args.hipac_transition_max_attempts_per_query),
        hipac_transition_candidate_limit=int(args.hipac_transition_candidate_limit),
        hipac_transition_window_stride=int(args.hipac_transition_window_stride),
        hipac_transition_min_predicted_bridge_edges=int(args.hipac_transition_min_predicted_bridge_edges),
        hipac_transition_max_pair_distance=float(args.hipac_transition_max_pair_distance),
        hipac_transition_allow_same_component=bool(args.hipac_transition_allow_same_component),
        hipac_transition_obb_portal=bool(args.hipac_transition_obb_portal),
        hipac_transition_obb_lateral_radius=float(args.hipac_transition_obb_lateral_radius),
        hipac_transition_obb_longitudinal_margin=float(args.hipac_transition_obb_longitudinal_margin),
        hipac_transition_obb_safety_epsilon=float(args.hipac_transition_obb_safety_epsilon),
        segment_edge_obb_cover=bool(args.segment_edge_obb_cover),
        rrt_bridge_obb_cover=bool(args.rrt_bridge_obb_cover),
        strict_obb_bridge_cover=bool(args.strict_obb_bridge_cover),
        segment_edge_obb_lateral_radius=float(args.segment_edge_obb_lateral_radius),
        segment_edge_obb_longitudinal_margin=float(args.segment_edge_obb_longitudinal_margin),
        segment_edge_obb_safety_epsilon=float(args.segment_edge_obb_safety_epsilon),
        segment_edge_obb_grow_iterations=int(args.segment_edge_obb_grow_iterations),
        segment_edge_obb_binary_iterations=int(args.segment_edge_obb_binary_iterations),
        segment_edge_obb_split_depth=int(args.segment_edge_obb_split_depth),
        obb_max_window_segments=int(args.obb_max_window_segments),
        obb_max_validations_per_window=int(args.obb_max_validations_per_window),
        hipac_promote_transition_slices=hipac_promote_transition_slices,
        hipac_promote_transition_target_query_indices=str(args.hipac_promote_transition_target_query_indices),
        hipac_promote_transition_min_boxes=int(args.hipac_promote_transition_min_boxes),
        hipac_promote_transition_max_boxes=int(args.hipac_promote_transition_max_boxes),
        hipac_promote_transition_max_attempts_per_query=int(args.hipac_promote_transition_max_attempts_per_query),
        deep_ffb_depth=int(args.deep_ffb_depth),
        refine_timeout_ms=float(args.refine_timeout_ms),
        domain_seed_cap=int(args.domain_seed_cap),
        domain_success_cap=int(args.domain_success_cap),
        domain_attempt_cap=int(args.domain_attempt_cap),
        validation_batch_size=int(args.validation_batch_size),
        ffb_start_depth=int(args.ffb_start_depth),
        ffb_search_mode=str(args.ffb_search_mode),
        audit_resolution=int(args.audit_resolution),
        audit_segment_step=float(args.audit_segment_step),
        audit_collision_tolerance=float(args.audit_collision_tolerance),
        query_shortcut_boxes=bool(args.query_shortcut_boxes),
        use_virtual_topology=bool(args.use_virtual_topology),
        parallel_virtual_validation=bool(args.parallel_virtual_validation),
        leaf_threads=threads,
        envelope="link_aabb" if case == "link_aabb" else "support_hull",
        support_hull_skip_aabb_broadphase=(case == "support_hull_no_aabb"),
        endpoint_source=(
            "critsample"
            if case in {"critsample_support_hull", "critsample_d23_cache"}
            else ("hifk" if case == "no_external_lect" else "ifk")
        ),
        hifk_max_depth=5 if case == "no_external_lect" else 9,
        unsafe_sampling_validation=False,
        use_external_evidence=(
            case.startswith("baseline_d23")
            or case == "critsample_d23_cache"
            or case in {"support_hull_no_aabb", "link_aabb"}
        ),
        external_evidence_live_retry_on_maybe=False,
        active_endpoint_evidence_cache=bool(args.active_endpoint_evidence_cache),
        active_store_endpoint_evidence_cache=bool(args.active_store_endpoint_evidence_cache),
        worker_shared_endpoint_cache=bool(args.worker_shared_endpoint_cache),
        external_evidence_path=Path(args.rbf_cache_root) / warm_cache_label,
        external_evidence_verify_identity=False,
        use_shelf_root_override=False,
        root_override_tuples=active_root,
        coverage_override_tuples=coverage_root,
        symmetry_aligned_native_root=False,
        symmetry_aligned_cache_schedule=True,
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
        connector_adaptive_min_segment_fraction=float(args.connector_adaptive_min_segment_fraction),
        query_bridge_pave_depth=int(args.query_bridge_pave_depth),
        query_bridge_ffb_start_depth=int(args.query_bridge_ffb_start_depth),
        query_bridge_adaptive_ffb_depths=str(args.query_bridge_adaptive_ffb_depths),
        query_bridge_direct_sample_step=float(args.query_bridge_direct_sample_step),
        query_bridge_repair_subdivisions=int(args.query_bridge_repair_subdivisions),
        query_bridge_group_residual_gaps=bool(args.query_bridge_group_residual_gaps),
        query_bridge_partition_neighbor_candidates=bool(args.query_bridge_partition_neighbor_candidates),
        query_bridge_direct_append_partition_immediate=bool(
            args.query_bridge_direct_append_partition_immediate
        ),
        query_bridge_force_indices=str(args.query_bridge_force_indices),
        query_bridge_forced_attempts=int(args.query_bridge_forced_attempts),
        query_bridge_attempt_offset=int(args.query_bridge_attempt_offset),
        query_bridge_no_path_retry_attempts=int(args.query_bridge_no_path_retry_attempts),
        query_bridge_no_path_retry_stop_on_first_success=query_bridge_no_path_retry_stop_on_first_success,
        query_bridge_rrt_fixed_iters=int(args.query_bridge_rrt_fixed_iters),
        query_bridge_rrt_fixed_timeout_ms=float(args.query_bridge_rrt_fixed_timeout_ms),
        query_bridge_direct_max_length=float(args.query_bridge_direct_max_length),
        query_bridge_direct_segment_after_rrt=bool(args.query_bridge_direct_segment_after_rrt),
        query_bridge_direct_segment_after_rrt_min_length=float(args.query_bridge_direct_segment_after_rrt_min_length),
        query_bridge_fast_direct_segment_after_rrt=bool(args.query_bridge_fast_direct_segment_after_rrt),
        query_bridge_fast_direct_random_shortcut_iters=int(args.query_bridge_fast_direct_random_shortcut_iters),
        query_bridge_hybridize_attempt_paths=bool(args.query_bridge_hybridize_attempt_paths),
        query_bridge_hybrid_max_paths=int(args.query_bridge_hybrid_max_paths),
        query_bridge_hybrid_max_vertices=int(args.query_bridge_hybrid_max_vertices),
        query_bridge_hybrid_max_cross_checks=int(args.query_bridge_hybrid_max_cross_checks),
        query_bridge_to_main_island=bool(args.query_bridge_to_main_island),
        query_bridge_to_main_direct_segment_max_length=float(args.query_bridge_to_main_direct_segment_max_length),
        query_bridge_to_main_box_corridor=bool(args.query_bridge_to_main_box_corridor),
        endpoint_main_target_k=int(args.endpoint_main_target_k),
        endpoint_main_coarse_step=float(args.endpoint_main_coarse_step),
        endpoint_main_fine_step=float(args.endpoint_main_fine_step),
        endpoint_main_max_ffb_calls=int(args.endpoint_main_max_ffb_calls),
        endpoint_main_max_boxes=int(args.endpoint_main_max_boxes),
        endpoint_main_adaptive_ffb_depths=str(args.endpoint_main_adaptive_ffb_depths),
        endpoint_main_residual_segment_max_length=float(args.endpoint_main_residual_segment_max_length),
        endpoint_main_lateral_offset=float(args.endpoint_main_lateral_offset),
        endpoint_main_lateral_rounds=int(args.endpoint_main_lateral_rounds),
        endpoint_main_face_epsilon=float(args.endpoint_main_face_epsilon),
        connector_pave_fill_gaps=bool(args.connector_pave_fill_gaps),
        connector_pave_require_connected_chain=bool(args.connector_pave_require_connected_chain),
        final_collision_shortcut=bool(args.final_collision_shortcut),
        final_rrt_simplify=bool(args.final_rrt_simplify),
        final_rrt_simplify_timeout_ms=float(args.final_rrt_simplify_timeout_ms),
        final_rrt_simplify_max_iters=int(args.final_rrt_simplify_max_iters),
        final_rrt_simplify_attempts=int(args.final_rrt_simplify_attempts),
        corridor_refine=bool(args.corridor_refine),
        corridor_refine_budget_ms=float(args.corridor_refine_budget_ms),
        corridor_refine_max_boxes=int(args.corridor_refine_max_boxes),
        corridor_refine_boxes_per_query=int(args.corridor_refine_boxes_per_query),
        corridor_refine_passes=int(args.corridor_refine_passes),
        corridor_refine_start_margin_ms=float(args.corridor_refine_start_margin_ms),
        corridor_refine_mode=str(args.corridor_refine_mode),
        corridor_refine_long_path_ratio=float(args.corridor_refine_long_path_ratio),
        corridor_refine_min_delta=float(args.corridor_refine_min_delta),
        query_bridge_all=bool(args.query_bridge_all),
        query_bridge_adaptive_all=bool(args.query_bridge_adaptive_all),
        query_bridge_adaptive_max_path_length=float(args.query_bridge_adaptive_max_path_length),
        query_bridge_accept_segment_fraction=float(args.query_bridge_accept_segment_fraction),
        query_bridge_accept_path_ratio=float(args.query_bridge_accept_path_ratio),
        query_bridge_accept_path_additive=float(args.query_bridge_accept_path_additive),
        query_endpoint_anchor_before_bridge=bool(args.query_endpoint_anchor_before_bridge),
        query_bridge_labels=str(args.query_bridge_labels),
        query_bridge_segment_only_indices=str(args.query_bridge_segment_only_indices),
        query_bridge_force_selected=bool(args.query_bridge_force_selected),
        query_bridge_adaptive_step_repair=bool(args.query_bridge_adaptive_step_repair),
        query_bridge_adaptive_fine_step=float(args.query_bridge_adaptive_fine_step),
        query_bridge_adaptive_max_repair_subdivisions=int(args.query_bridge_adaptive_max_repair_subdivisions),
        query_bridge_adaptive_max_repair_calls=int(args.query_bridge_adaptive_max_repair_calls),
        query_bridge_adaptive_repair_priority=int(args.query_bridge_adaptive_repair_priority),
        query_bridge_adaptive_repair_target_segment_fraction=float(
            args.query_bridge_adaptive_repair_target_segment_fraction
        ),
        query_box_transition_line_deviation_penalty=float(args.query_box_transition_line_deviation_penalty),
        query_foreign_edge_cost_penalty=float(args.query_foreign_edge_cost_penalty),
        query_bridge_edge_cost_penalty=float(args.query_bridge_edge_cost_penalty),
        allow_anchor_roots=True,
        use_priority_points=True,
        offline_coverage_profile=str(args.offline_coverage_profile),
        offline_query_agnostic_build=True,
        offline_random_anchors=bool(args.offline_random_anchors),
        offline_anchor_count=int(args.offline_anchor_count),
        offline_anchor_candidate_count=int(args.offline_anchor_candidate_count),
        offline_anchor_sampling=str(args.offline_anchor_sampling),
        offline_anchor_lca_lambda=float(args.offline_anchor_lca_lambda),
        offline_anchor_distance_mu=float(args.offline_anchor_distance_mu),
        offline_connector_mode=str(args.offline_connector_mode),
        offline_shortcut_edges=int(args.offline_shortcut_edges),
        offline_shortcut_candidate_limit=int(args.offline_shortcut_candidate_limit),
        offline_shortcut_min_gain_ratio=float(args.offline_shortcut_min_gain_ratio),
        offline_shortcut_max_segment_length=float(args.offline_shortcut_max_segment_length),
        run_rrt_grower=bool(args.run_rrt_grower),
        rrt_grower_extra_boxes=int(args.rrt_grower_extra_boxes),
        rrt_grower_timeout_ms=float(args.rrt_grower_timeout_ms),
        priority_prune_radius=float(args.priority_prune_radius),
        collision_overlap_prune_min_depth=int(args.collision_overlap_prune_min_depth),
        collision_overlap_prune_threshold=float(args.collision_overlap_prune_threshold),
        collision_overlap_prune_ratio_threshold=float(args.collision_overlap_prune_ratio_threshold),
    )
    return robot, options


def run_case(case: str, seed: int, deep_max_boxes: int, args: argparse.Namespace) -> dict[str, Any]:
    robot, options = make_case_options(case, seed, deep_max_boxes, args)
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    cache_tag = str(getattr(args, "active_cache_tag", ""))
    cache_name = f"{case}_seed{seed}_box{deep_max_boxes}"
    if cache_tag:
        cache_name = f"{cache_name}_{cache_tag}"
    return run_leaf_rrt(
        robot=robot,
        obstacles=obstacles,
        queries=queries,
        database_path=args.out_dir / "active_cache" / cache_name,
        options=options,
    )


def config_scalar_summary(case: str, seed: int, deep_max_boxes: int, args: argparse.Namespace) -> dict[str, Any]:
    robot, options = make_case_options(case, seed, deep_max_boxes, args)
    cfg = configure_leaf_rrt_options(
        robot,
        args.out_dir / "config_audit_cache" / f"{case}_seed{seed}_box{deep_max_boxes}",
        options,
    )
    refine = make_leaf_refine_config(options)
    return {
        "option.endpoint_source": str(options.endpoint_source),
        "option.unsafe_sampling_validation": bool(options.unsafe_sampling_validation),
        "option.use_external_evidence": bool(options.use_external_evidence),
        "option.external_evidence_live_retry_on_maybe": bool(options.external_evidence_live_retry_on_maybe),
        "option.active_endpoint_evidence_cache": bool(options.active_endpoint_evidence_cache),
        "option.active_store_endpoint_evidence_cache": bool(options.active_store_endpoint_evidence_cache),
        "option.worker_shared_endpoint_cache": bool(options.worker_shared_endpoint_cache),
        "option.envelope": str(options.envelope),
        "option.support_hull_skip_aabb_broadphase": bool(options.support_hull_skip_aabb_broadphase),
        "option.hifk_max_depth": int(options.hifk_max_depth),
        "option.threads": int(options.threads),
        "option.leaf_threads": int(options.leaf_threads),
        "option.adaptive_fast_virtual_checkpoint_mode": bool(options.adaptive_fast_virtual_checkpoint_mode),
        "option.adaptive_depth_enabled": bool(options.adaptive_depth_enabled),
        "option.adaptive_depth_min": int(options.adaptive_depth_min),
        "option.adaptive_depth_max": int(options.adaptive_depth_max),
        "option.adaptive_depth_probe_count": int(options.adaptive_depth_probe_count),
        "option.adaptive_depth_anchor_probe_cap": int(options.adaptive_depth_anchor_probe_cap),
        "option.adaptive_depth_min_covered_probes": int(options.adaptive_depth_min_covered_probes),
        "option.adaptive_depth_min_main_probes": int(options.adaptive_depth_min_main_probes),
        "option.adaptive_depth_min_main_ratio": float(options.adaptive_depth_min_main_ratio),
        "option.adaptive_depth_min_cells": int(options.adaptive_depth_min_cells),
        "option.adaptive_depth_min_main_cells": int(options.adaptive_depth_min_main_cells),
        "option.adaptive_depth_max_online_cells": int(options.adaptive_depth_max_online_cells),
        "option.query_bridge_all": bool(options.query_bridge_all),
        "option.query_bridge_adaptive_all": bool(options.query_bridge_adaptive_all),
        "option.query_bridge_adaptive_max_path_length": float(options.query_bridge_adaptive_max_path_length),
        "option.query_bridge_accept_segment_fraction": float(options.query_bridge_accept_segment_fraction),
        "option.query_bridge_accept_path_ratio": float(options.query_bridge_accept_path_ratio),
        "option.query_bridge_accept_path_additive": float(options.query_bridge_accept_path_additive),
        "option.query_endpoint_anchor_before_bridge": bool(options.query_endpoint_anchor_before_bridge),
        "option.offline_coverage_profile": str(options.offline_coverage_profile),
        "option.offline_connector_mode": str(options.offline_connector_mode),
        "option.offline_shortcut_edges": int(options.offline_shortcut_edges),
        "option.offline_shortcut_candidate_limit": int(options.offline_shortcut_candidate_limit),
        "option.offline_shortcut_min_gain_ratio": float(options.offline_shortcut_min_gain_ratio),
        "option.offline_shortcut_max_segment_length": float(options.offline_shortcut_max_segment_length),
        "option.hipac_portal_connectivity": bool(options.hipac_portal_connectivity),
        "option.hipac_portal_cell_native_validate": bool(options.hipac_portal_cell_native_validate),
        "option.hipac_portal_max_internal_boxes": int(options.hipac_portal_max_internal_boxes),
        "option.hipac_portal_max_recursion_depth": int(options.hipac_portal_max_recursion_depth),
        "option.hipac_portal_ffb_depth": int(options.hipac_portal_ffb_depth),
        "option.hipac_portal_ffb_deadline_ms": float(options.hipac_portal_ffb_deadline_ms),
        "option.hipac_online_connectivity": bool(options.hipac_online_connectivity),
        "option.hipac_online_before_query_bridge": bool(options.hipac_online_before_query_bridge),
        "option.hipac_promote_query_repairs": bool(options.hipac_promote_query_repairs),
        "option.hipac_online_ffb_portal_fallback": bool(options.hipac_online_ffb_portal_fallback),
        "option.hipac_online_candidate_max_length": float(options.hipac_online_candidate_max_length),
        "option.hipac_online_max_resolves_per_query": int(options.hipac_online_max_resolves_per_query),
        "option.hipac_online_max_hidden_boxes_per_portal": int(options.hipac_online_max_hidden_boxes_per_portal),
        "option.hipac_online_max_ffb_calls_per_portal": int(options.hipac_online_max_ffb_calls_per_portal),
        "option.hipac_online_prebridge_portal": bool(options.hipac_online_prebridge_portal),
        "option.hipac_online_prebridge_candidate_limit": int(options.hipac_online_prebridge_candidate_limit),
        "option.hipac_online_prebridge_max_pair_distance": float(options.hipac_online_prebridge_max_pair_distance),
        "option.hipac_online_prebridge_route_distance_weight": float(options.hipac_online_prebridge_route_distance_weight),
        "option.hipac_online_prebridge_pair_distance_weight": float(options.hipac_online_prebridge_pair_distance_weight),
        "option.hipac_online_transition_portal": bool(options.hipac_online_transition_portal),
        "option.hipac_transition_target_query_indices": str(options.hipac_transition_target_query_indices),
        "option.hipac_transition_max_attempts_per_query": int(options.hipac_transition_max_attempts_per_query),
        "option.hipac_transition_candidate_limit": int(options.hipac_transition_candidate_limit),
        "option.hipac_transition_window_stride": int(options.hipac_transition_window_stride),
        "option.hipac_transition_min_predicted_bridge_edges": int(options.hipac_transition_min_predicted_bridge_edges),
        "option.hipac_transition_max_pair_distance": float(options.hipac_transition_max_pair_distance),
        "option.hipac_transition_allow_same_component": bool(options.hipac_transition_allow_same_component),
        "option.hipac_transition_obb_portal": bool(options.hipac_transition_obb_portal),
        "option.hipac_transition_obb_lateral_radius": float(options.hipac_transition_obb_lateral_radius),
        "option.hipac_transition_obb_longitudinal_margin": float(options.hipac_transition_obb_longitudinal_margin),
        "option.hipac_transition_obb_safety_epsilon": float(options.hipac_transition_obb_safety_epsilon),
        "option.segment_edge_obb_cover": bool(options.segment_edge_obb_cover),
        "option.rrt_bridge_obb_cover": bool(options.rrt_bridge_obb_cover),
        "option.strict_obb_bridge_cover": bool(options.strict_obb_bridge_cover),
        "option.segment_edge_obb_lateral_radius": float(options.segment_edge_obb_lateral_radius),
        "option.segment_edge_obb_longitudinal_margin": float(options.segment_edge_obb_longitudinal_margin),
        "option.segment_edge_obb_safety_epsilon": float(options.segment_edge_obb_safety_epsilon),
        "option.segment_edge_obb_grow_iterations": int(options.segment_edge_obb_grow_iterations),
        "option.segment_edge_obb_binary_iterations": int(options.segment_edge_obb_binary_iterations),
        "option.segment_edge_obb_split_depth": int(options.segment_edge_obb_split_depth),
        "option.obb_max_window_segments": int(options.obb_max_window_segments),
        "option.obb_max_validations_per_window": int(options.obb_max_validations_per_window),
        "option.hipac_promote_transition_slices": bool(options.hipac_promote_transition_slices),
        "option.hipac_promote_transition_target_query_indices": str(options.hipac_promote_transition_target_query_indices),
        "option.hipac_promote_transition_min_boxes": int(options.hipac_promote_transition_min_boxes),
        "option.hipac_promote_transition_max_boxes": int(options.hipac_promote_transition_max_boxes),
        "option.hipac_promote_transition_max_attempts_per_query": int(options.hipac_promote_transition_max_attempts_per_query),
        "option.offline_anchor_sampling": str(options.offline_anchor_sampling),
        "option.query_bridge_labels": str(options.query_bridge_labels),
        "option.query_bridge_segment_only_indices": str(options.query_bridge_segment_only_indices),
        "option.query_bridge_force_selected": bool(options.query_bridge_force_selected),
        "option.query_bridge_adaptive_step_repair": bool(options.query_bridge_adaptive_step_repair),
        "option.query_bridge_adaptive_fine_step": float(options.query_bridge_adaptive_fine_step),
        "option.query_bridge_adaptive_max_repair_subdivisions": int(options.query_bridge_adaptive_max_repair_subdivisions),
        "option.query_bridge_adaptive_max_repair_calls": int(options.query_bridge_adaptive_max_repair_calls),
        "option.query_bridge_adaptive_repair_priority": int(options.query_bridge_adaptive_repair_priority),
        "option.query_bridge_adaptive_repair_target_segment_fraction": float(
            options.query_bridge_adaptive_repair_target_segment_fraction
        ),
        "option.query_box_transition_line_deviation_penalty": float(options.query_box_transition_line_deviation_penalty),
        "option.query_foreign_edge_cost_penalty": float(options.query_foreign_edge_cost_penalty),
        "option.query_bridge_edge_cost_penalty": float(options.query_bridge_edge_cost_penalty),
        "option.query_bridge_direct_sample_step": float(options.query_bridge_direct_sample_step),
        "option.query_bridge_repair_subdivisions": int(options.query_bridge_repair_subdivisions),
        "option.query_bridge_force_indices": str(options.query_bridge_force_indices),
        "option.query_bridge_forced_attempts": int(options.query_bridge_forced_attempts),
        "option.query_bridge_attempt_offset": int(options.query_bridge_attempt_offset),
        "option.query_bridge_no_path_retry_attempts": int(options.query_bridge_no_path_retry_attempts),
        "option.query_bridge_no_path_retry_stop_on_first_success": bool(
            options.query_bridge_no_path_retry_stop_on_first_success
        ),
        "option.query_bridge_rrt_fixed_iters": int(options.query_bridge_rrt_fixed_iters),
        "option.query_bridge_rrt_fixed_timeout_ms": float(options.query_bridge_rrt_fixed_timeout_ms),
        "option.query_bridge_direct_max_length": float(options.query_bridge_direct_max_length),
        "option.query_bridge_direct_segment_after_rrt": bool(options.query_bridge_direct_segment_after_rrt),
        "option.query_bridge_direct_segment_after_rrt_min_length": float(
            options.query_bridge_direct_segment_after_rrt_min_length
        ),
        "option.query_bridge_fast_direct_segment_after_rrt": bool(
            getattr(options, "query_bridge_fast_direct_segment_after_rrt", False)
        ),
        "option.query_bridge_fast_direct_random_shortcut_iters": int(
            getattr(options, "query_bridge_fast_direct_random_shortcut_iters", 0)
        ),
        "option.query_bridge_hybridize_attempt_paths": bool(
            getattr(options, "query_bridge_hybridize_attempt_paths", False)
        ),
        "option.query_bridge_hybrid_max_paths": int(
            getattr(options, "query_bridge_hybrid_max_paths", 8)
        ),
        "option.query_bridge_hybrid_max_vertices": int(
            getattr(options, "query_bridge_hybrid_max_vertices", 128)
        ),
        "option.query_bridge_hybrid_max_cross_checks": int(
            getattr(options, "query_bridge_hybrid_max_cross_checks", 4096)
        ),
        "option.query_bridge_to_main_island": bool(options.query_bridge_to_main_island),
        "option.query_bridge_to_main_direct_segment_max_length": float(options.query_bridge_to_main_direct_segment_max_length),
        "option.query_bridge_to_main_box_corridor": bool(options.query_bridge_to_main_box_corridor),
        "option.endpoint_main_target_k": int(options.endpoint_main_target_k),
        "option.endpoint_main_coarse_step": float(options.endpoint_main_coarse_step),
        "option.endpoint_main_fine_step": float(options.endpoint_main_fine_step),
        "option.endpoint_main_max_ffb_calls": int(options.endpoint_main_max_ffb_calls),
        "option.endpoint_main_max_boxes": int(options.endpoint_main_max_boxes),
        "option.endpoint_main_adaptive_ffb_depths": str(options.endpoint_main_adaptive_ffb_depths),
        "option.endpoint_main_residual_segment_max_length": float(options.endpoint_main_residual_segment_max_length),
        "option.endpoint_main_lateral_offset": float(options.endpoint_main_lateral_offset),
        "option.endpoint_main_lateral_rounds": int(options.endpoint_main_lateral_rounds),
        "option.endpoint_main_face_epsilon": float(options.endpoint_main_face_epsilon),
        "cfg.endpoint_source.source": str(cfg.endpoint_source.source),
        "cfg.endpoint_source.hifk_max_depth": int(getattr(cfg.endpoint_source, "hifk_max_depth", 0)),
        "cfg.envelope_type.type": str(cfg.envelope_type.type),
        "cfg.envelope_type.support_hull_config.skip_aabb_broadphase": bool(getattr(cfg.envelope_type.support_hull_config, "skip_aabb_broadphase", False)),
        "cfg.envelope_type.support_hull_config.direct_collision": bool(getattr(cfg.envelope_type.support_hull_config, "direct_collision", False)),
        "cfg.validation.mode": str(cfg.validation.mode),
        "cfg.validation.accept_unsafe_free": bool(cfg.validation.accept_unsafe_free),
        "cfg.validation.enable_endpoint_evidence_cache": bool(getattr(cfg.validation, "enable_endpoint_evidence_cache", False)),
        "cfg.validation.store_endpoint_evidence_cache": bool(getattr(cfg.validation, "store_endpoint_evidence_cache", False)),
        "cfg.validation.enable_worker_shared_endpoint_cache": bool(getattr(cfg.validation, "enable_worker_shared_endpoint_cache", False)),
        "cfg.validation.external_evidence_live_retry_on_maybe": bool(getattr(cfg.validation, "external_evidence_live_retry_on_maybe", False)),
        "cfg.grower.commit_policy": str(cfg.grower.commit_policy),
        "cfg.connector.pave.commit_policy": str(cfg.connector.pave.commit_policy),
        "cfg.database.max_tree_depth": int(cfg.database.max_tree_depth),
        "cfg.database.canonical_mode": bool(cfg.database.canonical_mode),
        "cfg.database.symmetry_descriptor": str(cfg.database.symmetry_descriptor),
        "cfg.database.root_intervals_override": [[float(item.lo), float(item.hi)] for item in list(cfg.database.root_intervals_override)],
        "cfg.database.coverage_intervals_override": [[float(item.lo), float(item.hi)] for item in list(cfg.database.coverage_intervals_override)],
        "cfg.database.split_depth_dimensions": list(getattr(cfg.database.split_policy, "depth_dimensions", [])),
        "cfg.database.split_policy_descriptor": str(sbf.split_policy_descriptor(cfg.database.split_policy)),
        "cfg.database.split_policy_hash": int(sbf.split_policy_hash(cfg.database.split_policy)),
        "cfg.database.external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "cfg.database.external_evidence_use_snapshot": bool(getattr(cfg.database, "external_evidence_use_snapshot", False)),
        "cfg.database.external_evidence_auto_build_snapshot": bool(getattr(cfg.database, "external_evidence_auto_build_snapshot", False)),
        "cfg.database.verify_identity": bool(getattr(cfg.database, "verify_identity", False)),
        "cfg.runtime.mode": str(cfg.runtime.mode),
        "cfg.runtime.n_threads": int(cfg.runtime.n_threads),
        "cfg.runtime.batch_size": int(cfg.runtime.batch_size),
        "cfg.grower.n_threads": int(cfg.grower.n_threads),
        "cfg.grower.task_batch_size": int(cfg.grower.task_batch_size),
        "cfg.grower.mode": str(cfg.grower.mode),
        "cfg.grower.max_boxes": int(cfg.grower.max_boxes),
        "cfg.grower.timeout_ms": float(cfg.grower.timeout_ms),
        "cfg.connector.n_threads": int(cfg.connector.n_threads),
        "cfg.connector.max_pairs_per_gap": int(cfg.connector.max_pairs_per_gap),
        "cfg.connector.per_pair_timeout_ms": float(cfg.connector.per_pair_timeout_ms),
        "cfg.connector.max_total_bridge_boxes": int(cfg.connector.max_total_bridge_boxes),
        "cfg.connector.rrt.max_iters": int(cfg.connector.rrt.max_iters),
        "cfg.connector.rrt.timeout_ms": float(cfg.connector.rrt.timeout_ms),
        "cfg.connector.rrt.step_size": float(cfg.connector.rrt.step_size),
        "cfg.connector.rrt.goal_bias": float(cfg.connector.rrt.goal_bias),
        "cfg.connector.rrt.segment_resolution": int(cfg.connector.rrt.segment_resolution),
        "cfg.connector.pave.max_chain": int(cfg.connector.pave.max_chain),
        "cfg.connector.pave.max_steps_per_waypoint": int(cfg.connector.pave.max_steps_per_waypoint),
        "cfg.connector.pave.find_free_box.max_depth": int(cfg.connector.pave.find_free_box.max_depth),
        "cfg.connector.pave.adaptive_min_segment_fraction": float(
            getattr(cfg.connector.pave, "adaptive_min_segment_fraction", 0.0)
        ),
        "cfg.query_bridge_pave_depth": int(cfg.query_bridge_pave_depth),
        "cfg.query_bridge_ffb_start_depth": int(
            getattr(cfg, "query_bridge_ffb_start_depth", -1)
        ),
        "cfg.query_bridge_adaptive_ffb_depths": list(
            getattr(cfg, "query_bridge_adaptive_ffb_depths", [])
        ),
        "cfg.connector.pave.find_free_box.skip_to_depth": int(cfg.connector.pave.find_free_box.skip_to_depth),
        "cfg.connector.pave.find_free_box.search_mode": str(cfg.connector.pave.find_free_box.search_mode),
        "cfg.connector.pave.fill_gaps": bool(cfg.connector.pave.fill_gaps),
        "cfg.connector.pave.require_connected_chain": bool(cfg.connector.pave.require_connected_chain),
        "cfg.query.audit_resolution": int(cfg.query.audit_resolution),
        "cfg.query.audit_segment_step": float(cfg.query.audit_segment_step),
        "cfg.query.audit_collision_tolerance": float(cfg.query.audit_collision_tolerance),
        "cfg.query.shortcut_boxes": bool(cfg.query.shortcut_boxes),
        "cfg.query.collision_shortcut": bool(cfg.query.collision_shortcut),
        "cfg.query.final_rrt_simplify": bool(getattr(cfg.query, "final_rrt_simplify", False)),
        "cfg.query.final_rrt_simplify_timeout_ms": float(getattr(cfg.query, "final_rrt_simplify_timeout_ms", 0.0)),
        "ref.leaf_start_depth": int(refine.leaf_start_depth),
        "ref.leaf_max_depth": int(refine.leaf_max_depth),
        "ref.leaf_threads": int(refine.leaf_threads),
        "ref.validation_batch_size": int(refine.validation_batch_size),
        "ref.deep_max_boxes": int(refine.deep_max_boxes),
        "ref.deep_ffb_depth": int(refine.deep_ffb_depth),
        "ref.domain_seed_cap": int(refine.domain_seed_cap),
        "ref.domain_success_cap": int(refine.domain_success_cap),
        "ref.domain_attempt_cap": int(refine.domain_attempt_cap),
        "ref.refine_timeout_ms": float(refine.refine_timeout_ms),
        "ref.run_rrt_grower": bool(refine.run_rrt_grower),
        "ref.rrt_grower_extra_boxes": int(refine.rrt_grower_extra_boxes),
        "ref.rrt_grower_timeout_ms": float(refine.rrt_grower_timeout_ms),
        "ref.collision_overlap_prune_min_depth": int(getattr(refine, "collision_overlap_prune_min_depth", -1)),
        "ref.collision_overlap_prune_threshold": float(getattr(refine, "collision_overlap_prune_threshold", 0.0)),
    }


def audit_case_configs(cases: list[str], args: argparse.Namespace, seed: int = 0, deep_max_boxes: int = 200) -> dict[str, Any]:
    baseline_case = "baseline_d23_aafk_support_hull_8t"
    baseline = config_scalar_summary(baseline_case, seed, deep_max_boxes, args)
    audit: dict[str, Any] = {
        "baseline_case": baseline_case,
        "seed": int(seed),
        "deep_max_boxes": int(deep_max_boxes),
        "baseline": baseline,
        "cases": {},
        "unexpected_diffs": {},
    }
    for case in cases:
        summary = config_scalar_summary(case, seed, deep_max_boxes, args)
        diffs = {
            key: {"baseline": baseline.get(key), "case": value}
            for key, value in summary.items()
            if baseline.get(key) != value
        }
        allowed = set(CACHE_DIFF_KEYS)
        allowed.update(ALLOWED_CONFIG_DIFFS.get(case, set()))
        unexpected = {
            key: value
            for key, value in diffs.items()
            if key not in allowed
        }
        audit["cases"][case] = {
            "allowed_diff_keys": sorted(allowed),
            "diffs": diffs,
            "unexpected_diffs": unexpected,
        }
        if unexpected:
            audit["unexpected_diffs"][case] = unexpected
    return audit


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    keys = sorted({(row["case"], str(row.get("offline_grower", "leaf_refine")), row["deep_max_boxes"]) for row in rows})
    for case, offline_grower, budget in keys:
        items = [
            row for row in rows
            if row["case"] == case
            and str(row.get("offline_grower", "leaf_refine")) == offline_grower
            and row["deep_max_boxes"] == budget
        ]
        total_queries = sum(int(row.get("query_count", 0)) for row in items)
        success_queries = sum(int(row.get("success_count", 0)) for row in items)
        out.append({
            "case": case,
            "offline_grower": offline_grower,
            "deep_max_boxes": budget,
            "runs": len(items),
            "success_runs": sum(1 for row in items if row["success_count"] == row["query_count"]),
            "queries_per_scene": median(row.get("query_count", 0) for row in items),
            "success_queries": success_queries,
            "total_queries": total_queries,
            "planning_s_median": median(row["planning_s"] for row in items),
            "build_s_median": median(row["build_s"] for row in items),
            "offline_build_s_median": median(row.get("offline_build_s", row["build_s"]) for row in items),
            "query_s_median": median(row["query_s"] for row in items),
            "online_batch_s_median": median(row.get("online_batch_s", row["query_s"]) for row in items),
            "query_bridge_s_median": median(row.get("query_bridge_s", 0.0) for row in items),
            "query_bridge_per_query_s_median": median(row.get("query_bridge_per_query_s", row.get("query_bridge_s", 0.0) / max(1, row.get("query_count", 1))) for row in items),
            "diag_query_bridge_batch_total_ms_median": median(row.get("diag_query_bridge_batch_total_ms", 0.0) for row in items),
            "diag_query_bridge_batch_per_query_ms_median": median(row.get("diag_query_bridge_batch_per_query_ms", 0.0) for row in items),
            "diag_query_bridge_batch_rrt_ms_total_median": median(row.get("diag_query_bridge_batch_rrt_ms_total", 0.0) for row in items),
            "diag_query_bridge_batch_probe_ms_total_median": median(row.get("diag_query_bridge_batch_probe_ms_total", 0.0) for row in items),
            "diag_query_bridge_batch_pave_ms_total_median": median(row.get("diag_query_bridge_batch_pave_ms_total", 0.0) for row in items),
            "diag_query_bridge_parallel_task_rrt_median": median(row.get("diag_query_bridge_parallel_task_rrt", 0.0) for row in items),
            "diag_query_bridge_parallel_task_rrt_jobs_median": median(row.get("diag_query_bridge_parallel_task_rrt_jobs", 0.0) for row in items),
            "diag_query_bridge_rrt_fixed_iters_median": median(row.get("diag_query_bridge_rrt_fixed_iters", 0.0) for row in items),
            "diag_query_bridge_rrt_fixed_timeout_ms_median": median(row.get("diag_query_bridge_rrt_fixed_timeout_ms", 0.0) for row in items),
            "diag_query_bridge_detour_on_no_path_median": median(row.get("diag_query_bridge_detour_on_no_path", 0.0) for row in items),
            "diag_query_bridge_detour_candidate_median": median(row.get("diag_query_bridge_detour_candidate", 0.0) for row in items),
            "diag_query_bridge_detour_on_no_path_attempts_median": median(row.get("diag_query_bridge_detour_on_no_path_attempts", 0.0) for row in items),
            "diag_query_bridge_detour_on_no_path_successes_median": median(row.get("diag_query_bridge_detour_on_no_path_successes", 0.0) for row in items),
            "diag_query_bridge_detour_on_no_path_candidates_median": median(row.get("diag_query_bridge_detour_on_no_path_candidates", 0.0) for row in items),
            "diag_query_bridge_detour_on_no_path_rejects_median": median(row.get("diag_query_bridge_detour_on_no_path_rejects", 0.0) for row in items),
            "diag_query_bridge_detour_candidate_selected_median": median(row.get("diag_query_bridge_detour_candidate_selected", 0.0) for row in items),
            "diag_query_bridge_detour_candidate_not_shorter_median": median(row.get("diag_query_bridge_detour_candidate_not_shorter", 0.0) for row in items),
            "diag_query_bridge_waypoint_quality_retry_median": median(row.get("diag_query_bridge_waypoint_quality_retry", 0.0) for row in items),
            "diag_query_bridge_waypoint_quality_retry_tasks_median": median(row.get("diag_query_bridge_waypoint_quality_retry_tasks", 0.0) for row in items),
            "diag_query_bridge_waypoint_quality_retry_attempts_median": median(row.get("diag_query_bridge_waypoint_quality_retry_attempts", 0.0) for row in items),
            "diag_query_bridge_waypoint_quality_retry_successes_median": median(row.get("diag_query_bridge_waypoint_quality_retry_successes", 0.0) for row in items),
            "diag_query_bridge_waypoint_quality_retry_fixed_median": median(row.get("diag_query_bridge_waypoint_quality_retry_fixed", 0.0) for row in items),
            "diag_query_bridge_waypoint_quality_retry_ms_total_median": median(row.get("diag_query_bridge_waypoint_quality_retry_ms_total", 0.0) for row in items),
            "diag_query_bridge_batch_tasks_initial_median": median(row.get("diag_query_bridge_batch_tasks_initial", 0.0) for row in items),
            "diag_query_bridge_batch_tasks_attempted_median": median(row.get("diag_query_bridge_batch_tasks_attempted", 0.0) for row in items),
            "diag_query_bridge_batch_tasks_no_path_median": median(row.get("diag_query_bridge_batch_tasks_no_path", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_ms_median": median(row.get("diag_query_bridge_direct_corridor_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_ms_total_median": median(row.get("diag_query_bridge_direct_corridor_ms_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_samples_median": median(row.get("diag_query_bridge_direct_corridor_samples", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_samples_total_median": median(row.get("diag_query_bridge_direct_corridor_samples_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_ffb_calls_median": median(row.get("diag_query_bridge_direct_corridor_ffb_calls", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_ffb_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_ffb_calls_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_all_ffb_calls_median": median(row.get("diag_query_bridge_direct_corridor_all_ffb_calls", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_all_ffb_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_all_ffb_calls_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_direct_ffb_ms_median": median(row.get("diag_query_bridge_direct_corridor_direct_ffb_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_repair_ffb_ms_median": median(row.get("diag_query_bridge_direct_corridor_repair_ffb_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_repair_ffb_ms_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_ffb_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_repair_priority_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_priority", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_repair_target_segment_fraction_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_target_segment_fraction", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_repair_target_stops_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_target_stops", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_initial_bad_fraction_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_initial_bad_fraction", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_final_bad_fraction_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_final_bad_fraction", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_lateral_repair_ffb_ms_median": median(row.get("diag_query_bridge_direct_corridor_lateral_repair_ffb_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_mark_initial_ms_median": median(row.get("diag_query_bridge_direct_corridor_mark_initial_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_mark_incremental_ms_median": median(row.get("diag_query_bridge_direct_corridor_mark_incremental_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_initialize_dsu_ms_median": median(row.get("diag_query_bridge_direct_corridor_initialize_dsu_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_assimilate_ms_median": median(row.get("diag_query_bridge_direct_corridor_assimilate_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_partition_neighbor_candidates_median": median(row.get("diag_query_bridge_direct_corridor_partition_neighbor_candidates", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_segment_audit_ms_median": median(row.get("diag_query_bridge_direct_corridor_segment_audit_ms", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_added_median": median(row.get("diag_query_bridge_direct_corridor_added", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_added_total_median": median(row.get("diag_query_bridge_direct_corridor_added_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_repair_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_repair_calls_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_repair_added_total_median": median(row.get("diag_query_bridge_direct_corridor_repair_added_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_repair_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_calls_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_adaptive_repair_added_total_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_added_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_lateral_repair_enabled_median": median(row.get("diag_query_bridge_direct_corridor_lateral_repair_enabled", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_lateral_repair_calls_median": median(row.get("diag_query_bridge_direct_corridor_lateral_repair_calls", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_lateral_repair_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_lateral_repair_calls_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_lateral_repair_added_median": median(row.get("diag_query_bridge_direct_corridor_lateral_repair_added", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_lateral_repair_added_total_median": median(row.get("diag_query_bridge_direct_corridor_lateral_repair_added_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_bad_initial_median": median(row.get("diag_query_bridge_direct_corridor_bad_initial", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_bad_initial_total_median": median(row.get("diag_query_bridge_direct_corridor_bad_initial_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_bad_final_median": median(row.get("diag_query_bridge_direct_corridor_bad_final", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_bad_final_total_median": median(row.get("diag_query_bridge_direct_corridor_bad_final_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_segment_edges_median": median(row.get("diag_query_bridge_direct_corridor_segment_edges", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_segment_edges_total_median": median(row.get("diag_query_bridge_direct_corridor_segment_edges_total", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_local_residual_overlay_connected_median": median(row.get("diag_query_bridge_direct_corridor_local_residual_overlay_connected", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_full_residual_without_local_overlay_median": median(row.get("diag_query_bridge_direct_corridor_full_residual_without_local_overlay", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_full_residual_audit_rejects_median": median(row.get("diag_query_bridge_direct_corridor_full_residual_audit_rejects", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_full_residual_edges_median": median(row.get("diag_query_bridge_direct_corridor_full_residual_edges", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_full_residual_edges_without_local_overlay_median": median(row.get("diag_query_bridge_direct_corridor_full_residual_edges_without_local_overlay", 0.0) for row in items),
            "diag_query_bridge_direct_corridor_full_adjacency_scans_avoided_median": median(row.get("diag_query_bridge_direct_corridor_full_adjacency_scans_avoided", 0.0) for row in items),
            "endpoint_main_s_median": median(row.get("endpoint_main_s", 0.0) for row in items),
            "endpoint_main_per_query_s_median": median(row.get("endpoint_main_per_query_s", row.get("endpoint_main_s", 0.0) / max(1, row.get("query_count", 1))) for row in items),
            "endpoint_main_success_count_median": median(row.get("endpoint_main_success_count", 0) for row in items),
            "endpoint_main_fallback_to_e2e_median": median(row.get("endpoint_main_fallback_to_e2e", 0) for row in items),
            "online_solve_s_median": median(row.get("online_solve_s", row.get("online_batch_s", row["query_s"])) for row in items),
            "online_simplify_s_median": median(row.get("online_simplify_s", 0.0) for row in items),
            "online_solve_per_query_s_median": median(row.get("online_solve_per_query_s", row.get("online_solve_s", row.get("online_batch_s", row["query_s"])) / max(1, row.get("query_count", 1))) for row in items),
            "online_simplify_per_query_s_median": median(row.get("online_simplify_per_query_s", row.get("online_simplify_s", 0.0) / max(1, row.get("query_count", 1))) for row in items),
            "online_per_query_s_median": median(row.get("online_per_query_s", row.get("online_solve_s", row["query_s"]) / max(1, row.get("query_count", 1))) for row in items),
            "online_total_per_query_s_median": median(row.get("online_total_per_query_s", row.get("query_total_s", row["query_s"]) / max(1, row.get("query_count", 1))) for row in items),
            "partition_query_per_query_s_median": median(row.get("partition_query_per_query_s", math.nan) for row in items),
            "partition_query_total_per_query_s_median": median(row.get("partition_query_total_per_query_s", math.nan) for row in items),
            "amortized_s_k1": median(row.get("amortized_s_k1", row["planning_s"]) for row in items),
            "amortized_s_k5": median(row.get("amortized_s_k5", row["planning_s"] / 5.0) for row in items),
            "amortized_s_k10": median(row.get("amortized_s_k10", row["planning_s"] / 10.0) for row in items),
            "amortized_s_k20": median(row.get("amortized_s_k20", row["planning_s"] / 20.0) for row in items),
            "amortized_s_k50": median(row.get("amortized_s_k50", row["planning_s"] / 50.0) for row in items),
            "leaf_sweep_s_median": median(row["leaf_sweep_s"] for row in items),
            "deep_refine_s_median": median(row["deep_refine_s"] for row in items),
            "offline_coverage_profile": str(items[0].get("offline_coverage_profile", "")),
            "offline_coverage_s_median": median(row.get("offline_coverage_s", math.nan) for row in items),
            "offline_connector_mode": str(items[0].get("offline_connector_mode", "")),
            "offline_connector_s_median": median(row.get("offline_connector_s", math.nan) for row in items),
            "adaptive_deep_leaf_s_median": median(row.get("adaptive_deep_leaf_s", 0.0) for row in items),
            "adaptive_target_depth_median": median(row.get("adaptive_target_depth", math.nan) for row in items),
            "selected_leaf_depth_median": median(row.get("selected_leaf_depth", math.nan) for row in items),
            "adaptive_depth_readiness_rate": mean(1.0 if row.get("adaptive_depth_readiness_met", False) else 0.0 for row in items),
            "adaptive_validated_median": median(row.get("adaptive_validated", 0) for row in items),
            "adaptive_splits_median": median(row.get("adaptive_splits", 0) for row in items),
            "adaptive_deferred_median": median(row.get("adaptive_deferred", 0) for row in items),
            "adaptive_promoted_median": median(row.get("adaptive_promoted", 0) for row in items),
            "adaptive_unresolved_domains_median": median(row.get("adaptive_unresolved_domains", 0) for row in items),
            "adaptive_merge_input_boxes_median": median(row.get("adaptive_merge_input_boxes", 0) for row in items),
            "adaptive_merge_output_boxes_median": median(row.get("adaptive_merge_output_boxes", 0) for row in items),
            "adaptive_merge_grid_ms_median": median(row.get("adaptive_merge_grid_ms", 0.0) for row in items),
            "adaptive_merge_grid_merges_median": median(row.get("adaptive_merge_grid_merges", 0) for row in items),
            "adaptive_merge_tree_ms_median": median(row.get("adaptive_merge_tree_ms", 0.0) for row in items),
            "adaptive_merge_tree_merges_median": median(row.get("adaptive_merge_tree_merges", 0) for row in items),
            "adaptive_merge_containment_ms_median": median(row.get("adaptive_merge_containment_ms", 0.0) for row in items),
            "adaptive_merge_exact_ms_median": median(row.get("adaptive_merge_exact_ms", 0.0) for row in items),
            "adaptive_partition_merge_containment_skipped_median": median(row.get("adaptive_partition_merge_containment_skipped", 0) for row in items),
            "adaptive_partition_merge_containment_bucket_entries_median": median(row.get("adaptive_partition_merge_containment_bucket_entries", 0) for row in items),
            "adaptive_partition_merge_containment_candidates_median": median(row.get("adaptive_partition_merge_containment_candidates", 0) for row in items),
            "adaptive_partition_merge_containment_tests_median": median(row.get("adaptive_partition_merge_containment_tests", 0) for row in items),
            "adaptive_partition_merge_containment_overflow_median": median(row.get("adaptive_partition_merge_containment_overflow", 0) for row in items),
            "adaptive_partition_merge_containment_ms_median": median(row.get("adaptive_partition_merge_containment_ms", 0.0) for row in items),
            "adaptive_partition_merge_line_ms_median": median(row.get("adaptive_partition_merge_line_ms", 0.0) for row in items),
            "adaptive_adjacency_ms_median": median(row.get("adaptive_adjacency_ms", 0.0) for row in items),
            "adaptive_adjacency_candidates_median": median(row.get("adaptive_adjacency_candidates", 0) for row in items),
            "adaptive_adjacency_exact_tests_median": median(row.get("adaptive_adjacency_exact_tests", 0) for row in items),
            "partition_cell_count_median": median(row.get("partition_cell_count", 0) for row in items),
            "partition_grid_cell_count_median": median(row.get("partition_grid_cell_count", 0) for row in items),
            "partition_non_grid_cell_count_median": median(row.get("partition_non_grid_cell_count", 0) for row in items),
            "partition_point_index_dims_median": median(row.get("partition_point_index_dims", 0) for row in items),
            "partition_point_index_entries_median": median(row.get("partition_point_index_entries", 0) for row in items),
            "partition_point_index_overflow_cells_median": median(row.get("partition_point_index_overflow_cells", 0) for row in items),
            "partition_sparse_virtual_cells_median": median(row.get("partition_sparse_virtual_cells", 0) for row in items),
            "partition_sparse_virtual_exact_index_entries_median": median(
                row.get("partition_sparse_virtual_exact_index_entries", 0) for row in items
            ),
            "partition_sparse_virtual_max_address_depth_median": median(
                row.get("partition_sparse_virtual_max_address_depth", 0) for row in items
            ),
            "partition_sparse_virtual_ancestor_refs_avoided_median": median(
                row.get("partition_sparse_virtual_ancestor_refs_avoided", 0) for row in items
            ),
            "partition_sparse_virtual_index_ms_median": median(
                row.get("partition_sparse_virtual_index_ms", 0.0) for row in items
            ),
            "oracle_certified_free_median": median(row.get("oracle_certified_free", 0) for row in items),
            "oracle_certified_occupied_median": median(row.get("oracle_certified_occupied", 0) for row in items),
            "oracle_collision_possible_median": median(row.get("oracle_collision_possible", 0) for row in items),
            "partition_islands_median": median(row.get("partition_islands", 0) for row in items),
            "partition_largest_island_median": median(row.get("partition_largest_island", 0) for row in items),
            "partition_overlay_edges_median": median(row.get("partition_overlay_edges", 0) for row in items),
            "partition_build_ms_median": median(row.get("partition_build_ms", 0.0) for row in items),
            "partition_index_rebuild_ms_median": median(row.get("partition_index_rebuild_ms", 0.0) for row in items),
            "partition_adjacency_candidates_median": median(row.get("partition_adjacency_candidates", 0) for row in items),
            "partition_adjacency_edges_median": median(row.get("partition_adjacency_edges", 0) for row in items),
            "coverage_probe_free_count_median": median(row.get("coverage_probe_free_count", 0) for row in items),
            "coverage_box_covered_probability_median": median(row.get("coverage_box_covered_probability", math.nan) for row in items),
            "coverage_anchor_success_probability_median": median(row.get("coverage_anchor_success_probability", math.nan) for row in items),
            "coverage_main_accessible_probability_median": median(row.get("coverage_main_accessible_probability", math.nan) for row in items),
            "connector_s_median": median(row["connector_s"] for row in items),
            "corridor_refine_s_median": median(row.get("corridor_refine_s", 0.0) for row in items),
            "corridor_refine_added_median": median(row.get("corridor_refine_added", 0) for row in items),
            "audit_s_median": median(row["audit_s"] for row in items),
            "path_length_mean": mean(row["path_length_mean"] for row in items),
            "raw_segment_fraction_median": median(row["raw_segment_fraction"] for row in items),
            "offline_query_agnostic_build": all(bool(row.get("offline_query_agnostic_build", False)) for row in items),
            "portal_membership_policy_median": median(row.get("portal_membership_policy", 0) for row in items),
            "portal_membership_global_forest_only_median": median(
                row.get("portal_membership_global_forest_only", 0) for row in items
            ),
            "portal_membership_global_forest_lookup_median": median(
                row.get("portal_membership_global_forest_lookup", 0) for row in items
            ),
            "portal_membership_global_forest_only_fallback_max": max(
                (int(row.get("portal_membership_global_forest_only_fallback", 0)) for row in items),
                default=0,
            ),
            "portal_membership_portal_interior_index_unavailable_max": max(
                (int(row.get("portal_membership_portal_interior_index_unavailable", 0)) for row in items),
                default=0,
            ),
            "qroot_pairs_total_max": max((int(row.get("qroot_pairs_total", -1)) for row in items), default=-1),
            "qroot_uncovered_endpoints_max": max((int(row.get("qroot_uncovered_endpoints", -1)) for row in items), default=-1),
            "offline_anchor_candidates_median": median(row.get("offline_anchor_candidates", 0) for row in items),
            "offline_anchor_roots_requested_median": median(row.get("offline_anchor_roots_requested", 0) for row in items),
            "offline_anchor_roots_added_median": median(row.get("offline_anchor_roots_added", 0) for row in items),
            "offline_anchor_lca_depth_mean": median(row.get("offline_anchor_lca_depth_mean", math.nan) for row in items),
            "offline_anchor_lca_depth_max": median(row.get("offline_anchor_lca_depth_max", math.nan) for row in items),
            "offline_anchor_box_volume_mean": median(row.get("offline_anchor_box_volume_mean", 0.0) for row in items),
            "offline_anchor_box_volume_max": median(row.get("offline_anchor_box_volume_max", 0.0) for row in items),
            "offline_shortcut_s_median": median(row.get("offline_shortcut_s", 0.0) for row in items),
            "offline_shortcut_edges_requested_median": median(row.get("offline_shortcut_edges_requested", 0) for row in items),
            "offline_shortcut_edges_added_median": median(row.get("offline_shortcut_edges_added", 0) for row in items),
            "offline_shortcut_portal_corridor_edges_added_median": median(row.get("offline_shortcut_portal_corridor_edges_added", 0) for row in items),
            "offline_shortcut_portal_corridor_fail_median": median(row.get("offline_shortcut_portal_corridor_fail", 0) for row in items),
            "offline_shortcut_portal_corridor_attempts_median": median(row.get("offline_shortcut_portal_corridor_attempts", 0) for row in items),
            "offline_shortcut_portal_corridor_internal_boxes_median": median(row.get("offline_shortcut_portal_corridor_internal_boxes", 0) for row in items),
            "offline_shortcut_portal_corridor_ffb_calls_median": median(row.get("offline_shortcut_portal_corridor_ffb_calls", 0) for row in items),
            "offline_shortcut_portal_corridor_cell_native_validations_median": median(row.get("offline_shortcut_portal_corridor_cell_native_validations", 0) for row in items),
            "offline_shortcut_portal_corridor_cell_native_free_median": median(row.get("offline_shortcut_portal_corridor_cell_native_free", 0) for row in items),
            "offline_shortcut_box_corridor_edges_added_median": median(row.get("offline_shortcut_box_corridor_edges_added", 0) for row in items),
            "offline_shortcut_segment_edges_added_median": median(row.get("offline_shortcut_segment_edges_added", 0) for row in items),
            "offline_shortcut_pave_boxes_added_median": median(row.get("offline_shortcut_pave_boxes_added", 0) for row in items),
            "offline_shortcut_pave_fail_median": median(row.get("offline_shortcut_pave_fail", 0) for row in items),
            "offline_box_edges_added_median": median(row.get("offline_box_edges_added", 0) for row in items),
            "offline_segment_edges_added_median": median(row.get("offline_segment_edges_added", 0) for row in items),
            "offline_islands_before_median": median(row.get("offline_islands_before", 0) for row in items),
            "offline_islands_after_median": median(row.get("offline_islands_after", 0) for row in items),
            "build_final_boxes_median": median(row.get("build_final_boxes", row["final_boxes"]) for row in items),
            "after_corridor_boxes_median": median(row.get("after_corridor_boxes", row["final_boxes"]) for row in items),
            "query_bridge_boxes_added_observed_median": median(row.get("query_bridge_boxes_added_observed", 0) for row in items),
            "query_bridge_added_reported_median": median(row.get("query_bridge_added", 0) for row in items),
            "query_bridge_hipac_online_attempts_median": median(row.get("query_bridge_hipac_online_attempts", 0) for row in items),
            "query_bridge_hipac_online_added_median": median(row.get("query_bridge_hipac_online_added", 0) for row in items),
            "query_bridge_hipac_online_satisfied_median": median(row.get("query_bridge_hipac_online_satisfied", 0) for row in items),
            "query_bridge_hipac_online_failures_median": median(row.get("query_bridge_hipac_online_failures", 0) for row in items),
            "query_bridge_hipac_online_not_sufficient_median": median(row.get("query_bridge_hipac_online_not_sufficient", 0) for row in items),
            "query_bridge_hipac_online_ms_median": median(row.get("query_bridge_hipac_online_ms", 0.0) for row in items),
            "query_bridge_hipac_online_box_edges_median": median(row.get("query_bridge_hipac_online_box_edges", 0) for row in items),
            "query_bridge_hipac_online_portal_edges_median": median(row.get("query_bridge_hipac_online_portal_edges", 0) for row in items),
            "query_bridge_hipac_online_internal_boxes_median": median(row.get("query_bridge_hipac_online_internal_boxes", 0) for row in items),
            "query_bridge_hipac_online_ffb_calls_median": median(row.get("query_bridge_hipac_online_ffb_calls", 0) for row in items),
            "query_bridge_hipac_online_cell_native_validations_median": median(row.get("query_bridge_hipac_online_cell_native_validations", 0) for row in items),
            "query_bridge_hipac_online_cell_native_free_median": median(row.get("query_bridge_hipac_online_cell_native_free", 0) for row in items),
            "query_bridge_hipac_prebridge_attempts_median": median(row.get("query_bridge_hipac_prebridge_attempts", 0) for row in items),
            "query_bridge_hipac_prebridge_candidates_median": median(row.get("query_bridge_hipac_prebridge_candidates", 0) for row in items),
            "query_bridge_hipac_prebridge_portal_attempts_median": median(row.get("query_bridge_hipac_prebridge_portal_attempts", 0) for row in items),
            "query_bridge_hipac_prebridge_added_median": median(row.get("query_bridge_hipac_prebridge_added", 0) for row in items),
            "query_bridge_hipac_prebridge_satisfied_median": median(row.get("query_bridge_hipac_prebridge_satisfied", 0) for row in items),
            "query_bridge_hipac_prebridge_failures_median": median(row.get("query_bridge_hipac_prebridge_failures", 0) for row in items),
            "query_bridge_hipac_prebridge_not_sufficient_median": median(row.get("query_bridge_hipac_prebridge_not_sufficient", 0) for row in items),
            "query_bridge_hipac_prebridge_ms_median": median(row.get("query_bridge_hipac_prebridge_ms", 0.0) for row in items),
            "query_bridge_hipac_prebridge_portal_edges_median": median(row.get("query_bridge_hipac_prebridge_portal_edges", 0) for row in items),
            "query_bridge_hipac_prebridge_internal_boxes_median": median(row.get("query_bridge_hipac_prebridge_internal_boxes", 0) for row in items),
            "query_bridge_hipac_prebridge_ffb_calls_median": median(row.get("query_bridge_hipac_prebridge_ffb_calls", 0) for row in items),
            "query_bridge_hipac_prebridge_cell_native_validations_median": median(row.get("query_bridge_hipac_prebridge_cell_native_validations", 0) for row in items),
            "query_bridge_hipac_prebridge_cell_native_free_median": median(row.get("query_bridge_hipac_prebridge_cell_native_free", 0) for row in items),
            "query_bridge_hipac_transition_attempts_median": median(row.get("query_bridge_hipac_transition_attempts", 0) for row in items),
            "query_bridge_hipac_transition_candidates_median": median(row.get("query_bridge_hipac_transition_candidates", 0) for row in items),
            "query_bridge_hipac_transition_gated_median": median(row.get("query_bridge_hipac_transition_gated", 0) for row in items),
            "query_bridge_hipac_transition_portal_attempts_median": median(row.get("query_bridge_hipac_transition_portal_attempts", 0) for row in items),
            "query_bridge_hipac_transition_added_median": median(row.get("query_bridge_hipac_transition_added", 0) for row in items),
            "query_bridge_hipac_transition_satisfied_median": median(row.get("query_bridge_hipac_transition_satisfied", 0) for row in items),
            "query_bridge_hipac_transition_not_sufficient_median": median(row.get("query_bridge_hipac_transition_not_sufficient", 0) for row in items),
            "query_bridge_hipac_transition_failures_median": median(row.get("query_bridge_hipac_transition_failures", 0) for row in items),
            "query_bridge_hipac_transition_ms_median": median(row.get("query_bridge_hipac_transition_ms", 0.0) for row in items),
            "query_bridge_hipac_transition_portal_edges_median": median(row.get("query_bridge_hipac_transition_portal_edges", 0) for row in items),
            "query_bridge_hipac_transition_internal_boxes_median": median(row.get("query_bridge_hipac_transition_internal_boxes", 0) for row in items),
            "query_bridge_hipac_transition_ffb_calls_median": median(row.get("query_bridge_hipac_transition_ffb_calls", 0) for row in items),
            "query_bridge_hipac_transition_cell_native_validations_median": median(row.get("query_bridge_hipac_transition_cell_native_validations", 0) for row in items),
            "query_bridge_hipac_transition_cell_native_free_median": median(row.get("query_bridge_hipac_transition_cell_native_free", 0) for row in items),
            "query_bridge_hipac_transition_obb_attempts_median": median(row.get("query_bridge_hipac_transition_obb_attempts", 0) for row in items),
            "query_bridge_hipac_transition_obb_success_median": median(row.get("query_bridge_hipac_transition_obb_success", 0) for row in items),
            "query_bridge_hipac_transition_obb_fail_median": median(row.get("query_bridge_hipac_transition_obb_fail", 0) for row in items),
            "query_bridge_hipac_transition_obb_joint_limit_rejects_median": median(row.get("query_bridge_hipac_transition_obb_joint_limit_rejects", 0) for row in items),
            "query_bridge_hipac_transition_obb_gjk_tests_median": median(row.get("query_bridge_hipac_transition_obb_gjk_tests", 0) for row in items),
            "query_bridge_hipac_transition_obb_maybe_pairs_median": median(row.get("query_bridge_hipac_transition_obb_maybe_pairs", 0) for row in items),
            "query_bridge_hipac_transition_obb_ms_median": median(row.get("query_bridge_hipac_transition_obb_ms", 0.0) for row in items),
            "segment_edge_obb_cover_attempts_median": median(row.get("segment_edge_obb_cover_attempts", 0) for row in items),
            "segment_edge_obb_cover_success_median": median(row.get("segment_edge_obb_cover_success", 0) for row in items),
            "segment_edge_obb_cover_fail_median": median(row.get("segment_edge_obb_cover_fail", 0) for row in items),
            "segment_edge_obb_cover_validations_median": median(row.get("segment_edge_obb_cover_validations", 0) for row in items),
            "segment_edge_obb_cover_regions_median": median(row.get("segment_edge_obb_cover_regions", 0) for row in items),
            "segment_edge_obb_cover_windows_attempted_median": median(row.get("segment_edge_obb_cover_windows_attempted", 0) for row in items),
            "segment_edge_obb_cover_windows_success_median": median(row.get("segment_edge_obb_cover_windows_success", 0) for row in items),
            "segment_edge_obb_cover_replaced_segments_median": median(row.get("segment_edge_obb_cover_replaced_segments", 0) for row in items),
            "segment_edge_obb_cover_partial_edges_median": median(row.get("segment_edge_obb_cover_partial_edges", 0) for row in items),
            "segment_edge_obb_cover_partial_regions_median": median(row.get("segment_edge_obb_cover_partial_regions", 0) for row in items),
            "segment_edge_obb_cover_partial_committed_median": median(row.get("segment_edge_obb_cover_partial_committed", 0) for row in items),
            "segment_edge_obb_cover_partial_covered_length_median": median(row.get("segment_edge_obb_cover_partial_covered_length", 0.0) for row in items),
            "segment_edge_obb_cover_ms_median": median(row.get("segment_edge_obb_cover_ms", 0.0) for row in items),
            "rrt_bridge_obb_cover_attempts_median": median(row.get("rrt_bridge_obb_cover_attempts", 0) for row in items),
            "rrt_bridge_obb_cover_success_median": median(row.get("rrt_bridge_obb_cover_success", 0) for row in items),
            "rrt_bridge_obb_cover_fail_median": median(row.get("rrt_bridge_obb_cover_fail", 0) for row in items),
            "rrt_bridge_obb_cover_regions_median": median(row.get("rrt_bridge_obb_cover_regions", 0) for row in items),
            "rrt_bridge_obb_cover_validations_median": median(row.get("rrt_bridge_obb_cover_validations", 0) for row in items),
            "rrt_bridge_obb_cover_replaced_segments_median": median(row.get("rrt_bridge_obb_cover_replaced_segments", 0) for row in items),
            "rrt_bridge_obb_cover_partial_edges_median": median(row.get("rrt_bridge_obb_cover_partial_edges", 0) for row in items),
            "rrt_bridge_obb_cover_partial_regions_median": median(row.get("rrt_bridge_obb_cover_partial_regions", 0) for row in items),
            "rrt_bridge_obb_cover_partial_committed_median": median(row.get("rrt_bridge_obb_cover_partial_committed", 0) for row in items),
            "rrt_bridge_obb_cover_partial_covered_length_median": median(row.get("rrt_bridge_obb_cover_partial_covered_length", 0.0) for row in items),
            "rrt_bridge_obb_cover_ms_median": median(row.get("rrt_bridge_obb_cover_ms", 0.0) for row in items),
            "obb_edges_used_total_median": median(row.get("obb_edges_used_total", 0) for row in items),
            "obb_regions_used_total_median": median(row.get("obb_regions_used_total", 0) for row in items),
            "obb_edge_length_total_median": median(row.get("obb_edge_length_total", 0.0) for row in items),
            "query_bridge_hipac_promote_attempts_median": median(row.get("query_bridge_hipac_promote_attempts", 0) for row in items),
            "query_bridge_hipac_promote_added_median": median(row.get("query_bridge_hipac_promote_added", 0) for row in items),
            "query_bridge_hipac_promote_failures_median": median(row.get("query_bridge_hipac_promote_failures", 0) for row in items),
            "query_bridge_hipac_promote_ms_median": median(row.get("query_bridge_hipac_promote_ms", 0.0) for row in items),
            "query_bridge_hipac_promote_portal_edges_median": median(row.get("query_bridge_hipac_promote_portal_edges", 0) for row in items),
            "query_bridge_hipac_promote_internal_boxes_median": median(row.get("query_bridge_hipac_promote_internal_boxes", 0) for row in items),
            "query_bridge_hipac_promote_ffb_calls_median": median(row.get("query_bridge_hipac_promote_ffb_calls", 0) for row in items),
            "query_bridge_hipac_promote_cell_native_validations_median": median(row.get("query_bridge_hipac_promote_cell_native_validations", 0) for row in items),
            "query_bridge_hipac_promote_cell_native_free_median": median(row.get("query_bridge_hipac_promote_cell_native_free", 0) for row in items),
            "final_boxes_median": median(row["final_boxes"] for row in items),
            "build_segment_edges_median": median(row.get("build_segment_edges", row["segment_edges"]) for row in items),
            "query_bridge_segment_edges_added_observed_median": median(row.get("query_bridge_segment_edges_added_observed", 0) for row in items),
            "final_segment_edges_median": median(row.get("final_segment_edges", row["segment_edges"]) for row in items),
            "final_adjacency_islands_median": median(row.get("final_adjacency_islands", row.get("adjacency_islands", 0)) for row in items),
            "segment_edges_median": median(row["segment_edges"] for row in items),
            "adjacency_islands_median": median(row["adjacency_islands"] for row in items),
            "external_hits_median": median(row["external_hits"] for row in items),
            "external_reused_hits_median": median(row.get("external_reused_hits", row["external_hits"]) for row in items),
            "external_exact_hits_median": median(row.get("external_exact_hits", 0.0) for row in items),
            "external_exact_misses_median": median(row.get("external_exact_misses", 0.0) for row in items),
            "interval_replay_compatibility_checks_median": median(row.get("interval_replay_compatibility_checks", 0.0) for row in items),
            "interval_replay_compatible_median": median(row.get("interval_replay_compatible", 0.0) for row in items),
            "interval_replay_incompatible_median": median(row.get("interval_replay_incompatible", 0.0) for row in items),
            "interval_replay_direct_exact_hits_median": median(row.get("interval_replay_direct_exact_hits", 0.0) for row in items),
            "interval_replay_key_only_blocked_median": median(row.get("interval_replay_key_only_blocked", 0.0) for row in items),
            "unsafe_sampling_validation": any(bool(row.get("unsafe_sampling_validation")) for row in items),
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "case",
        "offline_grower",
        "deep_max_boxes",
        "runs",
        "success_runs",
        "queries_per_scene",
        "success_queries",
        "total_queries",
        "planning_s_median",
        "build_s_median",
        "offline_build_s_median",
        "query_s_median",
        "online_batch_s_median",
        "query_bridge_s_median",
        "query_bridge_per_query_s_median",
        "diag_query_bridge_batch_total_ms_median",
        "diag_query_bridge_batch_per_query_ms_median",
        "diag_query_bridge_batch_rrt_ms_total_median",
        "diag_query_bridge_batch_probe_ms_total_median",
        "diag_query_bridge_batch_pave_ms_total_median",
        "diag_query_bridge_parallel_task_rrt_median",
        "diag_query_bridge_parallel_task_rrt_jobs_median",
        "diag_query_bridge_rrt_fixed_iters_median",
        "diag_query_bridge_rrt_fixed_timeout_ms_median",
        "diag_query_bridge_detour_on_no_path_median",
        "diag_query_bridge_detour_candidate_median",
        "diag_query_bridge_detour_on_no_path_attempts_median",
        "diag_query_bridge_detour_on_no_path_successes_median",
        "diag_query_bridge_detour_on_no_path_candidates_median",
        "diag_query_bridge_detour_on_no_path_rejects_median",
        "diag_query_bridge_detour_candidate_selected_median",
        "diag_query_bridge_detour_candidate_not_shorter_median",
        "diag_query_bridge_waypoint_quality_retry_median",
        "diag_query_bridge_waypoint_quality_retry_tasks_median",
        "diag_query_bridge_waypoint_quality_retry_attempts_median",
        "diag_query_bridge_waypoint_quality_retry_successes_median",
        "diag_query_bridge_waypoint_quality_retry_fixed_median",
        "diag_query_bridge_waypoint_quality_retry_ms_total_median",
        "diag_query_bridge_batch_tasks_initial_median",
        "diag_query_bridge_batch_tasks_attempted_median",
        "diag_query_bridge_batch_tasks_no_path_median",
        "diag_query_bridge_direct_corridor_ms_median",
        "diag_query_bridge_direct_corridor_ms_total_median",
        "diag_query_bridge_direct_corridor_samples_median",
        "diag_query_bridge_direct_corridor_samples_total_median",
        "diag_query_bridge_direct_corridor_ffb_calls_median",
        "diag_query_bridge_direct_corridor_ffb_calls_total_median",
        "diag_query_bridge_direct_corridor_all_ffb_calls_median",
        "diag_query_bridge_direct_corridor_all_ffb_calls_total_median",
        "diag_query_bridge_direct_corridor_direct_ffb_ms_median",
        "diag_query_bridge_direct_corridor_repair_ffb_ms_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_ffb_ms_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_priority_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_target_segment_fraction_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_target_stops_median",
        "diag_query_bridge_direct_corridor_adaptive_initial_bad_fraction_median",
        "diag_query_bridge_direct_corridor_adaptive_final_bad_fraction_median",
        "diag_query_bridge_direct_corridor_lateral_repair_ffb_ms_median",
        "diag_query_bridge_direct_corridor_mark_initial_ms_median",
        "diag_query_bridge_direct_corridor_mark_incremental_ms_median",
        "diag_query_bridge_direct_corridor_initialize_dsu_ms_median",
        "diag_query_bridge_direct_corridor_assimilate_ms_median",
        "diag_query_bridge_direct_corridor_partition_neighbor_candidates_median",
        "diag_query_bridge_direct_corridor_segment_audit_ms_median",
        "diag_query_bridge_direct_corridor_added_median",
        "diag_query_bridge_direct_corridor_added_total_median",
        "diag_query_bridge_direct_corridor_repair_calls_total_median",
        "diag_query_bridge_direct_corridor_repair_added_total_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_calls_total_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_added_total_median",
        "diag_query_bridge_direct_corridor_lateral_repair_enabled_median",
        "diag_query_bridge_direct_corridor_lateral_repair_calls_median",
        "diag_query_bridge_direct_corridor_lateral_repair_calls_total_median",
        "diag_query_bridge_direct_corridor_lateral_repair_added_median",
        "diag_query_bridge_direct_corridor_lateral_repair_added_total_median",
        "diag_query_bridge_direct_corridor_bad_initial_median",
        "diag_query_bridge_direct_corridor_bad_initial_total_median",
        "diag_query_bridge_direct_corridor_bad_final_median",
        "diag_query_bridge_direct_corridor_bad_final_total_median",
        "diag_query_bridge_direct_corridor_segment_edges_median",
        "diag_query_bridge_direct_corridor_segment_edges_total_median",
        "diag_query_bridge_direct_corridor_local_residual_overlay_connected_median",
        "diag_query_bridge_direct_corridor_full_residual_without_local_overlay_median",
        "diag_query_bridge_direct_corridor_full_residual_audit_rejects_median",
        "diag_query_bridge_direct_corridor_full_residual_edges_median",
        "diag_query_bridge_direct_corridor_full_residual_edges_without_local_overlay_median",
        "diag_query_bridge_direct_corridor_full_adjacency_scans_avoided_median",
        "endpoint_main_s_median",
        "endpoint_main_per_query_s_median",
        "endpoint_main_success_count_median",
        "endpoint_main_fallback_to_e2e_median",
        "online_solve_s_median",
        "online_simplify_s_median",
        "online_solve_per_query_s_median",
        "online_simplify_per_query_s_median",
        "online_per_query_s_median",
        "online_total_per_query_s_median",
        "partition_query_per_query_s_median",
        "partition_query_total_per_query_s_median",
        "amortized_s_k1",
        "amortized_s_k5",
        "amortized_s_k10",
        "amortized_s_k20",
        "amortized_s_k50",
        "leaf_sweep_s_median",
        "deep_refine_s_median",
        "offline_coverage_profile",
        "offline_coverage_s_median",
        "offline_connector_mode",
        "offline_connector_s_median",
        "adaptive_deep_leaf_s_median",
        "adaptive_target_depth_median",
        "adaptive_validated_median",
        "adaptive_splits_median",
        "adaptive_deferred_median",
        "adaptive_promoted_median",
        "adaptive_unresolved_domains_median",
        "adaptive_merge_input_boxes_median",
        "adaptive_merge_output_boxes_median",
        "adaptive_merge_grid_ms_median",
        "adaptive_merge_grid_merges_median",
        "adaptive_merge_tree_ms_median",
        "adaptive_merge_tree_merges_median",
        "adaptive_merge_containment_ms_median",
        "adaptive_merge_exact_ms_median",
        "adaptive_partition_merge_containment_skipped_median",
        "adaptive_partition_merge_containment_bucket_entries_median",
        "adaptive_partition_merge_containment_candidates_median",
        "adaptive_partition_merge_containment_tests_median",
        "adaptive_partition_merge_containment_overflow_median",
        "adaptive_partition_merge_containment_ms_median",
        "adaptive_partition_merge_line_ms_median",
        "adaptive_adjacency_ms_median",
        "adaptive_adjacency_candidates_median",
        "adaptive_adjacency_exact_tests_median",
        "partition_cell_count_median",
        "partition_grid_cell_count_median",
        "partition_non_grid_cell_count_median",
        "partition_point_index_dims_median",
        "partition_point_index_entries_median",
        "partition_point_index_overflow_cells_median",
        "partition_sparse_virtual_cells_median",
        "partition_sparse_virtual_exact_index_entries_median",
        "partition_sparse_virtual_max_address_depth_median",
        "partition_sparse_virtual_ancestor_refs_avoided_median",
        "partition_sparse_virtual_index_ms_median",
        "oracle_certified_free_median",
        "oracle_certified_occupied_median",
        "oracle_collision_possible_median",
        "partition_islands_median",
        "partition_largest_island_median",
        "partition_overlay_edges_median",
        "partition_build_ms_median",
        "partition_index_rebuild_ms_median",
        "partition_adjacency_candidates_median",
        "partition_adjacency_edges_median",
        "coverage_probe_free_count_median",
        "coverage_box_covered_probability_median",
        "coverage_anchor_success_probability_median",
        "coverage_main_accessible_probability_median",
        "connector_s_median",
        "corridor_refine_s_median",
        "corridor_refine_added_median",
        "audit_s_median",
        "path_length_mean",
        "raw_segment_fraction_median",
        "offline_query_agnostic_build",
        "portal_membership_policy_median",
        "portal_membership_global_forest_only_median",
        "portal_membership_global_forest_lookup_median",
        "portal_membership_global_forest_only_fallback_max",
        "portal_membership_portal_interior_index_unavailable_max",
        "qroot_pairs_total_max",
        "qroot_uncovered_endpoints_max",
        "offline_anchor_candidates_median",
        "offline_anchor_roots_requested_median",
        "offline_anchor_roots_added_median",
        "offline_anchor_lca_depth_mean",
        "offline_anchor_lca_depth_max",
        "offline_anchor_box_volume_mean",
        "offline_anchor_box_volume_max",
        "offline_shortcut_s_median",
        "offline_shortcut_edges_requested_median",
        "offline_shortcut_edges_added_median",
        "offline_shortcut_portal_corridor_edges_added_median",
        "offline_shortcut_portal_corridor_fail_median",
        "offline_shortcut_portal_corridor_attempts_median",
        "offline_shortcut_portal_corridor_internal_boxes_median",
        "offline_shortcut_portal_corridor_ffb_calls_median",
        "offline_shortcut_portal_corridor_cell_native_validations_median",
        "offline_shortcut_portal_corridor_cell_native_free_median",
        "offline_shortcut_box_corridor_edges_added_median",
        "offline_shortcut_segment_edges_added_median",
        "offline_shortcut_pave_boxes_added_median",
        "offline_shortcut_pave_fail_median",
        "offline_box_edges_added_median",
        "offline_segment_edges_added_median",
        "offline_islands_before_median",
        "offline_islands_after_median",
        "build_final_boxes_median",
        "after_corridor_boxes_median",
        "query_bridge_boxes_added_observed_median",
        "query_bridge_added_reported_median",
        "query_bridge_hipac_online_attempts_median",
        "query_bridge_hipac_online_added_median",
        "query_bridge_hipac_online_satisfied_median",
        "query_bridge_hipac_online_failures_median",
        "query_bridge_hipac_online_not_sufficient_median",
        "query_bridge_hipac_online_ms_median",
        "query_bridge_hipac_online_box_edges_median",
        "query_bridge_hipac_online_portal_edges_median",
        "query_bridge_hipac_online_internal_boxes_median",
        "query_bridge_hipac_online_ffb_calls_median",
        "query_bridge_hipac_online_cell_native_validations_median",
        "query_bridge_hipac_online_cell_native_free_median",
        "query_bridge_hipac_prebridge_attempts_median",
        "query_bridge_hipac_prebridge_candidates_median",
        "query_bridge_hipac_prebridge_portal_attempts_median",
        "query_bridge_hipac_prebridge_added_median",
        "query_bridge_hipac_prebridge_satisfied_median",
        "query_bridge_hipac_prebridge_failures_median",
        "query_bridge_hipac_prebridge_not_sufficient_median",
        "query_bridge_hipac_prebridge_ms_median",
        "query_bridge_hipac_prebridge_portal_edges_median",
        "query_bridge_hipac_prebridge_internal_boxes_median",
        "query_bridge_hipac_prebridge_ffb_calls_median",
        "query_bridge_hipac_prebridge_cell_native_validations_median",
        "query_bridge_hipac_prebridge_cell_native_free_median",
        "query_bridge_hipac_transition_attempts_median",
        "query_bridge_hipac_transition_candidates_median",
        "query_bridge_hipac_transition_gated_median",
        "query_bridge_hipac_transition_portal_attempts_median",
        "query_bridge_hipac_transition_added_median",
        "query_bridge_hipac_transition_satisfied_median",
        "query_bridge_hipac_transition_not_sufficient_median",
        "query_bridge_hipac_transition_failures_median",
        "query_bridge_hipac_transition_ms_median",
        "query_bridge_hipac_transition_portal_edges_median",
        "query_bridge_hipac_transition_internal_boxes_median",
        "query_bridge_hipac_transition_ffb_calls_median",
        "query_bridge_hipac_transition_cell_native_validations_median",
        "query_bridge_hipac_transition_cell_native_free_median",
        "query_bridge_hipac_transition_obb_attempts_median",
        "query_bridge_hipac_transition_obb_success_median",
        "query_bridge_hipac_transition_obb_fail_median",
        "query_bridge_hipac_transition_obb_joint_limit_rejects_median",
        "query_bridge_hipac_transition_obb_gjk_tests_median",
        "query_bridge_hipac_transition_obb_maybe_pairs_median",
        "query_bridge_hipac_transition_obb_ms_median",
        "segment_edge_obb_cover_attempts_median",
        "segment_edge_obb_cover_success_median",
        "segment_edge_obb_cover_fail_median",
        "segment_edge_obb_cover_validations_median",
        "segment_edge_obb_cover_regions_median",
        "segment_edge_obb_cover_windows_attempted_median",
        "segment_edge_obb_cover_windows_success_median",
        "segment_edge_obb_cover_replaced_segments_median",
        "segment_edge_obb_cover_partial_edges_median",
        "segment_edge_obb_cover_partial_regions_median",
        "segment_edge_obb_cover_partial_committed_median",
        "segment_edge_obb_cover_partial_covered_length_median",
        "segment_edge_obb_cover_ms_median",
        "rrt_bridge_obb_cover_attempts_median",
        "rrt_bridge_obb_cover_success_median",
        "rrt_bridge_obb_cover_fail_median",
        "rrt_bridge_obb_cover_regions_median",
        "rrt_bridge_obb_cover_validations_median",
        "rrt_bridge_obb_cover_replaced_segments_median",
        "rrt_bridge_obb_cover_partial_edges_median",
        "rrt_bridge_obb_cover_partial_regions_median",
        "rrt_bridge_obb_cover_partial_committed_median",
        "rrt_bridge_obb_cover_partial_covered_length_median",
        "rrt_bridge_obb_cover_ms_median",
        "obb_edges_used_total_median",
        "obb_regions_used_total_median",
        "obb_edge_length_total_median",
        "query_bridge_hipac_promote_attempts_median",
        "query_bridge_hipac_promote_added_median",
        "query_bridge_hipac_promote_failures_median",
        "query_bridge_hipac_promote_ms_median",
        "query_bridge_hipac_promote_portal_edges_median",
        "query_bridge_hipac_promote_internal_boxes_median",
        "query_bridge_hipac_promote_ffb_calls_median",
        "query_bridge_hipac_promote_cell_native_validations_median",
        "query_bridge_hipac_promote_cell_native_free_median",
        "final_boxes_median",
        "build_segment_edges_median",
        "query_bridge_segment_edges_added_observed_median",
        "final_segment_edges_median",
        "final_adjacency_islands_median",
        "segment_edges_median",
        "adjacency_islands_median",
        "external_hits_median",
        "external_reused_hits_median",
        "external_exact_hits_median",
        "external_exact_misses_median",
        "interval_replay_compatibility_checks_median",
        "interval_replay_compatible_median",
        "interval_replay_incompatible_median",
        "interval_replay_direct_exact_hits_median",
        "interval_replay_key_only_blocked_median",
        "unsafe_sampling_validation",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    table_rows = [row for row in rows if int(row.get("deep_max_boxes", -1)) == DEFAULT_RBF_SHELF_BOX_BUDGET]
    if not table_rows:
        table_rows = rows
    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Shelf+IIWA RBF ablation at the registered 100-box point. Times are in seconds; Path is the success-only mean and Seg. is the raw segment fraction.}",
        r"\label{tab:tro-shelf-ablation}",
        r"\begin{tabular}{lrrrrrrrr}",
        r"\toprule",
        r"Case & Build & Online/q & Simplify/q & Amort@5 & Path & Seg. & Boxes & SR \\",
        r"\midrule",
    ]
    for row in table_rows:
        sr = f"{int(row.get('success_queries', row['success_runs']))}/{int(row.get('total_queries', row['runs']))}"
        label = CASE_LABELS.get(str(row["case"]), str(row["case"])).replace("_", r"\_")
        full_success = int(row.get("success_queries", row["success_runs"])) == int(row.get("total_queries", row["runs"]))
        path_length = row["path_length_mean"] if full_success else None
        segment_fraction = row["raw_segment_fraction_median"] if full_success else None
        lines.append(
            f"{label} & {tex_num(row['offline_build_s_median'])} & "
            f"{tex_num(row['online_per_query_s_median'])} & "
            f"{tex_num(row.get('online_simplify_per_query_s_median'))} & "
            f"{tex_num(row['amortized_s_k5'])} & "
            f"{tex_num(path_length)} & {tex_num(segment_fraction)} & "
            f"{tex_num(row['final_boxes_median'])} & {sr} \\\\"
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
    parser.add_argument("--box-budgets", default=str(DEFAULT_RBF_SHELF_BOX_BUDGET))
    parser.add_argument("--only", default="baseline_d23_aafk_support_hull_8t,critsample_d23_cache,no_external_lect,support_hull_no_aabb,link_aabb")
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--rbf-max-depth", type=int, default=60)
    parser.add_argument("--offline-grower", choices=["leaf_refine", "adaptive_deep_leaf"], default="adaptive_deep_leaf")
    parser.add_argument(
        "--offline-coverage-profile",
        choices=["", RBF_OFFLINE_COVERAGE_PROFILE_NAME],
        default="",
        help="Apply a named query-agnostic offline coverage profile before running online queries.",
    )
    parser.add_argument("--leaf-start-depth", type=int, default=DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--leaf-max-depth", type=int, default=DEFAULT_RBF_LEAF_MAX_DEPTH)
    parser.add_argument("--adaptive-target-depth", type=int, default=DEFAULT_RBF_LEAF_MAX_DEPTH)
    parser.add_argument("--adaptive-time-budget-ms", type=float, default=60000.0)
    parser.add_argument("--adaptive-node-budget", type=int, default=50000)
    parser.add_argument("--adaptive-fast-virtual-checkpoint-mode",
                        action=argparse.BooleanOptionalAction,
                        default=False)
    parser.add_argument("--adaptive-defer-min-depth", type=int, default=16)
    parser.add_argument("--adaptive-overlap-depth-threshold", type=float, default=0.05)
    parser.add_argument("--adaptive-overlap-depth-min-threshold", type=float, default=0.01)
    parser.add_argument("--adaptive-overlap-depth-decay-per-depth", type=float, default=0.04)
    parser.add_argument("--adaptive-overlap-ratio-threshold", type=float, default=0.0)
    parser.add_argument("--coverage-probe-count", type=int, default=4096)
    parser.add_argument("--adaptive-seed-anchor-probe-cap", type=int, default=256)
    parser.add_argument("--adaptive-depth-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--adaptive-depth-min", type=int, default=DEFAULT_RBF_LEAF_MAX_DEPTH)
    parser.add_argument("--adaptive-depth-max", type=int, default=16)
    parser.add_argument("--adaptive-depth-probe-count", type=int, default=512)
    parser.add_argument("--adaptive-depth-anchor-probe-cap", type=int, default=32)
    parser.add_argument("--adaptive-depth-probe-seed", type=int, default=20260607)
    parser.add_argument("--adaptive-depth-min-free-probes", type=int, default=64)
    parser.add_argument("--adaptive-depth-min-covered-probes", type=int, default=3)
    parser.add_argument("--adaptive-depth-min-main-probes", type=int, default=3)
    parser.add_argument("--adaptive-depth-min-main-ratio", type=float, default=0.35)
    parser.add_argument("--adaptive-depth-min-cells", type=int, default=0)
    parser.add_argument("--adaptive-depth-min-main-cells", type=int, default=0)
    parser.add_argument("--adaptive-depth-max-online-cells", type=int, default=180)
    parser.add_argument("--adaptive-depth-max-probe-ms", type=float, default=5.0)
    parser.add_argument("--adaptive-max-merge-ms", type=float, default=1500.0)
    parser.add_argument("--adaptive-max-merge-rounds", type=int, default=2)
    parser.add_argument("--adaptive-max-merge-input-boxes", type=int, default=100000)
    parser.add_argument("--adaptive-max-free-boxes", type=int, default=50000)
    parser.add_argument("--adaptive-max-unresolved-domains", type=int, default=100000)
    parser.add_argument("--adaptive-planning-backend", default="partition_native")
    parser.add_argument("--adaptive-grid-target-depth", type=int, default=0)
    parser.add_argument("--adaptive-grid-face-index-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--adaptive-grid-planning-max-expansions", type=int, default=0)
    parser.add_argument("--deep-ffb-depth", type=int, default=DEFAULT_RBF_DEEP_FFB_DEPTH)
    parser.add_argument("--refine-timeout-ms", type=float, default=DEFAULT_RBF_REFINE_TIMEOUT_MS)
    parser.add_argument("--domain-seed-cap", type=int, default=DEFAULT_RBF_DOMAIN_SEED_CAP)
    parser.add_argument("--domain-success-cap", type=int, default=DEFAULT_RBF_DOMAIN_SUCCESS_CAP)
    parser.add_argument("--domain-attempt-cap", type=int, default=DEFAULT_RBF_DOMAIN_ATTEMPT_CAP)
    parser.add_argument("--validation-batch-size", type=int, default=DEFAULT_RBF_VALIDATION_BATCH_SIZE)
    parser.add_argument("--ffb-start-depth", type=int, default=DEFAULT_RBF_FFB_START_DEPTH)
    parser.add_argument("--ffb-search-mode", default=DEFAULT_RBF_FFB_SEARCH_MODE, choices=["linear", "binary", "binary-depth", "BinaryDepth", "Linear"])
    parser.add_argument("--audit-resolution", type=int, default=DEFAULT_RBF_AUDIT_RESOLUTION)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--active-endpoint-evidence-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--active-store-endpoint-evidence-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--worker-shared-endpoint-cache", action=argparse.BooleanOptionalAction, default=False)
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
    parser.add_argument("--connector-pave-depth", type=int, default=DEFAULT_RBF_CONNECTOR_PAVE_DEPTH)
    parser.add_argument("--connector-adaptive-min-segment-fraction", type=float, default=DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION)
    parser.add_argument("--query-bridge-pave-depth", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH)
    parser.add_argument("--query-bridge-ffb-start-depth", type=int, default=-1)
    parser.add_argument("--query-bridge-adaptive-ffb-depths", default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FFB_DEPTHS)
    parser.add_argument("--query-bridge-direct-sample-step", type=float, default=DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP)
    parser.add_argument("--query-bridge-repair-subdivisions", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS)
    parser.add_argument(
        "--query-bridge-group-residual-gaps",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_GROUP_RESIDUAL_GAPS,
    )
    parser.add_argument(
        "--query-bridge-partition-neighbor-candidates",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_PARTITION_NEIGHBOR_CANDIDATES,
    )
    parser.add_argument(
        "--query-bridge-direct-append-partition-immediate",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_DIRECT_APPEND_PARTITION_IMMEDIATE,
    )
    parser.add_argument("--query-bridge-direct-max-length", type=float, default=6.5)
    parser.add_argument("--query-bridge-to-main-island", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--query-bridge-to-main-direct-segment-max-length", type=float, default=0.0)
    parser.add_argument("--query-bridge-to-main-box-corridor", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--endpoint-main-target-k", type=int, default=8)
    parser.add_argument("--endpoint-main-coarse-step", type=float, default=0.08)
    parser.add_argument("--endpoint-main-fine-step", type=float, default=0.02)
    parser.add_argument("--endpoint-main-max-ffb-calls", type=int, default=48)
    parser.add_argument("--endpoint-main-max-boxes", type=int, default=64)
    parser.add_argument("--endpoint-main-adaptive-ffb-depths", default="")
    parser.add_argument("--endpoint-main-residual-segment-max-length", type=float, default=0.25)
    parser.add_argument("--endpoint-main-lateral-offset", type=float, default=0.03)
    parser.add_argument("--endpoint-main-lateral-rounds", type=int, default=2)
    parser.add_argument("--endpoint-main-face-epsilon", type=float, default=1e-6)
    parser.add_argument("--connector-pave-fill-gaps", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS)
    parser.add_argument("--connector-pave-require-connected-chain", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN)
    parser.add_argument("--final-collision-shortcut", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_COLLISION_SHORTCUT)
    parser.add_argument("--final-rrt-simplify", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY)
    parser.add_argument("--final-rrt-simplify-timeout-ms", type=float, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS)
    parser.add_argument("--final-rrt-simplify-max-iters", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS)
    parser.add_argument("--final-rrt-simplify-attempts", type=int, default=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--corridor-refine-budget-ms", type=float, default=0.0)
    parser.add_argument("--corridor-refine-max-boxes", type=int, default=0)
    parser.add_argument("--corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--corridor-refine-passes", type=int, default=1)
    parser.add_argument("--corridor-refine-start-margin-ms", type=float, default=0.0)
    parser.add_argument("--corridor-refine-mode", choices=["box_only_long_path", "legacy_bridge"], default="box_only_long_path")
    parser.add_argument("--corridor-refine-long-path-ratio", type=float, default=1.25)
    parser.add_argument("--corridor-refine-min-delta", type=float, default=0.25)
    parser.add_argument("--query-bridge-all", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_QUERY_BRIDGE_ALL)
    parser.add_argument("--query-bridge-adaptive-all", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-adaptive-max-path-length", type=float, default=4.5)
    parser.add_argument("--query-bridge-accept-segment-fraction", type=float, default=0.25)
    parser.add_argument("--query-bridge-accept-path-ratio", type=float, default=1.50)
    parser.add_argument("--query-bridge-accept-path-additive", type=float, default=0.75)
    parser.add_argument("--query-endpoint-anchor-before-bridge", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_QUERY_ENDPOINT_ANCHOR_BEFORE_BRIDGE)
    parser.add_argument("--query-bridge-labels", default=DEFAULT_RBF_QUERY_BRIDGE_LABELS)
    parser.add_argument(
        "--query-bridge-segment-only-indices",
        default="",
        help="Comma-separated zero-based shelf query indices to connect with audited QueryBridge segment edges instead of box paving.",
    )
    parser.add_argument(
        "--query-bridge-force-indices",
        default=DEFAULT_RBF_QUERY_BRIDGE_FORCE_INDICES,
        help="Comma-separated zero-based shelf query indices that must run query bridge even if normally deferred.",
    )
    parser.add_argument("--query-bridge-force-selected", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_QUERY_BRIDGE_FORCE_SELECTED)
    parser.add_argument("--query-bridge-forced-attempts", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS)
    parser.add_argument("--query-bridge-attempt-offset", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET)
    parser.add_argument("--query-bridge-no-path-retry-attempts", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS)
    parser.add_argument(
        "--query-bridge-no-path-retry-stop-on-first-success",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS,
    )
    parser.add_argument("--query-bridge-rrt-fixed-iters", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS)
    parser.add_argument("--query-bridge-rrt-fixed-timeout-ms", type=float, default=DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS)
    parser.add_argument("--query-bridge-hybridize-attempt-paths", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS)
    parser.add_argument("--query-bridge-hybrid-max-paths", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS)
    parser.add_argument("--query-bridge-hybrid-max-vertices", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES)
    parser.add_argument("--query-bridge-hybrid-max-cross-checks", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS)
    parser.add_argument("--query-bridge-adaptive-step-repair", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR)
    parser.add_argument("--query-bridge-adaptive-fine-step", type=float, default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP)
    parser.add_argument("--query-bridge-adaptive-max-repair-subdivisions", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS)
    parser.add_argument("--query-bridge-adaptive-max-repair-calls", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS)
    parser.add_argument("--query-bridge-adaptive-repair-priority", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY)
    parser.add_argument(
        "--query-bridge-adaptive-repair-target-segment-fraction",
        type=float,
        default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_TARGET_SEGMENT_FRACTION,
    )
    parser.add_argument(
        "--query-bridge-direct-segment-after-rrt",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT,
    )
    parser.add_argument(
        "--query-bridge-direct-segment-after-rrt-min-length",
        type=float,
        default=DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT_MIN_LENGTH,
    )
    parser.add_argument(
        "--query-bridge-fast-direct-segment-after-rrt",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--query-bridge-fast-direct-random-shortcut-iters", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS)
    parser.add_argument("--query-box-transition-line-deviation-penalty", type=float, default=DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY)
    parser.add_argument("--query-foreign-edge-cost-penalty", type=float, default=DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY)
    parser.add_argument("--query-bridge-edge-cost-penalty", type=float, default=DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY)
    parser.add_argument("--offline-random-anchors", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS)
    parser.add_argument("--offline-anchor-count", type=int, default=16)
    parser.add_argument("--offline-anchor-candidate-count", type=int, default=512)
    parser.add_argument("--offline-anchor-sampling", choices=["random", "halton", "mixed"], default="random")
    parser.add_argument("--offline-anchor-lca-lambda", type=float, default=0.35)
    parser.add_argument("--offline-anchor-distance-mu", type=float, default=0.10)
    parser.add_argument("--offline-connector-mode", choices=["off", "box_only", "short_segment"], default="box_only")
    parser.add_argument("--offline-shortcut-edges", type=int, default=0)
    parser.add_argument("--offline-shortcut-candidate-limit", type=int, default=48)
    parser.add_argument("--offline-shortcut-min-gain-ratio", type=float, default=1.6)
    parser.add_argument("--offline-shortcut-max-segment-length", type=float, default=3.0)
    parser.add_argument("--hipac-improved-leaf-sweep", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-portal-connectivity", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-portal-cell-native-validate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hipac-portal-max-internal-boxes", type=int, default=64)
    parser.add_argument("--hipac-portal-max-recursion-depth", type=int, default=8)
    parser.add_argument("--hipac-portal-ffb-depth", type=int, default=0)
    parser.add_argument("--hipac-portal-ffb-deadline-ms", type=float, default=5.0)
    parser.add_argument("--hipac-online-connectivity", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-online-before-query-bridge", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hipac-promote-query-repairs", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-online-ffb-portal-fallback", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-online-candidate-max-length", type=float, default=3.0)
    parser.add_argument("--hipac-online-max-resolves-per-query", type=int, default=1)
    parser.add_argument("--hipac-online-max-hidden-boxes-per-portal", type=int, default=32)
    parser.add_argument("--hipac-online-max-ffb-calls-per-portal", type=int, default=64)
    parser.add_argument("--hipac-online-prebridge-portal", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-online-prebridge-candidate-limit", type=int, default=32)
    parser.add_argument("--hipac-online-prebridge-max-pair-distance", type=float, default=1.25)
    parser.add_argument("--hipac-online-prebridge-route-distance-weight", type=float, default=1.0)
    parser.add_argument("--hipac-online-prebridge-pair-distance-weight", type=float, default=0.25)
    parser.add_argument("--hipac-online-transition-portal", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-transition-target-query-indices", default="2,3")
    parser.add_argument("--hipac-transition-max-attempts-per-query", type=int, default=1)
    parser.add_argument("--hipac-transition-candidate-limit", type=int, default=16)
    parser.add_argument("--hipac-transition-window-stride", type=int, default=2)
    parser.add_argument("--hipac-transition-min-predicted-bridge-edges", type=int, default=16)
    parser.add_argument("--hipac-transition-max-pair-distance", type=float, default=1.50)
    parser.add_argument("--hipac-transition-allow-same-component", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hipac-transition-obb-portal", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-transition-obb-lateral-radius", type=float, default=0.01)
    parser.add_argument("--hipac-transition-obb-longitudinal-margin", type=float, default=0.0)
    parser.add_argument("--hipac-transition-obb-safety-epsilon", type=float, default=0.0)
    parser.add_argument("--segment-edge-obb-cover", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rrt-bridge-obb-cover", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--strict-obb-bridge-cover", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--segment-edge-obb-lateral-radius", type=float, default=0.01)
    parser.add_argument("--segment-edge-obb-longitudinal-margin", type=float, default=0.0)
    parser.add_argument("--segment-edge-obb-safety-epsilon", type=float, default=0.0)
    parser.add_argument("--segment-edge-obb-grow-iterations", type=int, default=5)
    parser.add_argument("--segment-edge-obb-binary-iterations", type=int, default=5)
    parser.add_argument("--segment-edge-obb-split-depth", type=int, default=1)
    parser.add_argument("--obb-max-window-segments", type=int, default=16)
    parser.add_argument("--obb-max-validations-per-window", type=int, default=96)
    parser.add_argument("--hipac-promote-transition-slices", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-promote-transition-target-query-indices", default="2,3")
    parser.add_argument("--hipac-promote-transition-min-boxes", type=int, default=8)
    parser.add_argument("--hipac-promote-transition-max-boxes", type=int, default=64)
    parser.add_argument("--hipac-promote-transition-max-attempts-per-query", type=int, default=1)
    parser.add_argument("--run-rrt-grower", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rrt-grower-extra-boxes", type=int, default=DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES)
    parser.add_argument("--rrt-grower-timeout-ms", type=float, default=DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS)
    parser.add_argument("--priority-prune-radius", type=float, default=0.0)
    parser.add_argument("--collision-overlap-prune-min-depth", type=int, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH)
    parser.add_argument("--collision-overlap-prune-threshold", type=float, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD)
    parser.add_argument("--collision-overlap-prune-ratio-threshold", type=float, default=DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD)
    parser.add_argument("--use-virtual-topology", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    apply_offline_coverage_profile(args, sys.argv[1:])
    configure_thread_environment(int(args.threads))
    seeds = [int(item) for item in str(args.seeds).split(",") if item.strip()]
    budgets = [int(item) for item in str(args.box_budgets).split(",") if item.strip()]
    wanted = {
        CASE_ALIASES.get(item.strip(), item.strip())
        for item in str(args.only).split(",")
        if item.strip()
    }
    cases = [case for case in ABLATIONS if not wanted or "all" in wanted or case in wanted]
    if args.phase == "smoke":
        seeds = seeds[:1]
        budgets = budgets[:1]
        cases = cases[:1]
    config_audit = audit_case_configs(cases, args, seed=seeds[0] if seeds else 0, deep_max_boxes=budgets[0] if budgets else 200)
    if config_audit.get("unexpected_diffs"):
        unexpected = json.dumps(config_audit["unexpected_diffs"], indent=2, sort_keys=True)
        raise RuntimeError(f"Exp.4 case configuration audit failed; unexpected diffs:\n{unexpected}")
    manifest_rows = [
        {
            "case": case,
            "seed": seed,
            "deep_max_boxes": budget,
            "backend": str(args.offline_grower),
            "offline_grower": str(args.offline_grower),
            "offline_coverage_profile": str(args.offline_coverage_profile),
            "adaptive_target_depth": int(args.adaptive_target_depth),
            "adaptive_planning_backend": str(args.adaptive_planning_backend),
            "adaptive_grid_target_depth": int(args.adaptive_grid_target_depth),
            "offline_query_agnostic_build": True,
            "offline_anchor_count": int(args.offline_anchor_count),
            "offline_anchor_candidate_count": int(args.offline_anchor_candidate_count),
            "offline_anchor_sampling": str(args.offline_anchor_sampling),
            "offline_connector_mode": str(args.offline_connector_mode),
            "offline_shortcut_edges": int(args.offline_shortcut_edges),
            "offline_shortcut_candidate_limit": int(args.offline_shortcut_candidate_limit),
            "offline_shortcut_min_gain_ratio": float(args.offline_shortcut_min_gain_ratio),
            "offline_shortcut_max_segment_length": float(args.offline_shortcut_max_segment_length),
            "grower_mode": "rrt",
            "cache_depth_semantics": "canonical_lect_tree",
            "planner_depth_semantics": "lect_active_tree",
            "lect_cache_root": "canonical_primary_sector_with_native_coverage",
            "planner_state_space": "native_joint_space",
            "active_planning_root": "full_robot_joint_limits",
            "active_lect_root": "canonical_primary_sector",
            "coverage_root": "full_robot_joint_limits",
            "valid_planning_root": "full_robot_joint_limits",
            "split_schedule": "external_cache_manifest_prefix",
            "canonical_mapping_scope": "LECT_internal_only",
            "audit_collision_tolerance": float(args.audit_collision_tolerance),
            "query_bridge_edge_cost_penalty": float(args.query_bridge_edge_cost_penalty),
        }
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
            "offline_coverage_profile": str(args.offline_coverage_profile),
            "offline_coverage_profile_details": (
                offline_coverage_v1_profile()
                if str(args.offline_coverage_profile) == RBF_OFFLINE_COVERAGE_PROFILE_NAME
                else {}
            ),
            "offline_query_agnostic_build": True,
            "offline_anchor_count": int(args.offline_anchor_count),
            "offline_anchor_candidate_count": int(args.offline_anchor_candidate_count),
            "offline_anchor_sampling": str(args.offline_anchor_sampling),
            "offline_anchor_lca_lambda": float(args.offline_anchor_lca_lambda),
            "offline_anchor_distance_mu": float(args.offline_anchor_distance_mu),
            "offline_connector_mode": str(args.offline_connector_mode),
            "offline_shortcut_edges": int(args.offline_shortcut_edges),
            "offline_shortcut_candidate_limit": int(args.offline_shortcut_candidate_limit),
            "offline_shortcut_min_gain_ratio": float(args.offline_shortcut_min_gain_ratio),
            "offline_shortcut_max_segment_length": float(args.offline_shortcut_max_segment_length),
            "threads": int(args.threads),
            "thread_policy": "RBF, LECT virtual validation, and process math libraries use --threads; OMPL baselines are algorithmically single-thread unless their planner exposes internal parallelism.",
            "cache_depth_semantics": "canonical_lect_tree",
            "planner_depth_semantics": "lect_active_tree",
            "prewarm_depth": 23,
            "lect_cache_root": "canonical_primary_sector_with_native_coverage",
            "planner_state_space": "native_joint_space",
            "active_planning_root": "full_robot_joint_limits",
            "active_lect_root": "canonical_primary_sector",
            "coverage_root": "full_robot_joint_limits",
            "valid_planning_root": "full_robot_joint_limits",
            "split_schedule": "external_cache_manifest_prefix",
            "canonical_mapping_scope": "LECT_internal_only",
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "adaptive_depth": {
                "enabled": bool(args.adaptive_depth_enabled),
                "min": int(args.adaptive_depth_min),
                "max": int(args.adaptive_depth_max),
                "probe_count": int(args.adaptive_depth_probe_count),
                "anchor_probe_cap": int(args.adaptive_depth_anchor_probe_cap),
                "min_free_probes": int(args.adaptive_depth_min_free_probes),
                "min_covered_probes": int(args.adaptive_depth_min_covered_probes),
                "min_main_probes": int(args.adaptive_depth_min_main_probes),
                "min_main_ratio": float(args.adaptive_depth_min_main_ratio),
                "min_cells": int(args.adaptive_depth_min_cells),
                "min_main_cells": int(args.adaptive_depth_min_main_cells),
                "max_online_cells": int(args.adaptive_depth_max_online_cells),
                "max_probe_ms": float(args.adaptive_depth_max_probe_ms),
            },
            "deep_ffb_depth": int(args.deep_ffb_depth),
            "ffb_start_depth": int(args.ffb_start_depth),
            "query_bridge_ffb_start_depth": int(args.query_bridge_ffb_start_depth),
            "ffb_search_mode": str(args.ffb_search_mode),
            "lect_leaf_start_depth": int(args.leaf_start_depth),
            "lect_leaf_max_depth": int(args.leaf_max_depth),
            "lect_deep_ffb_depth": int(args.deep_ffb_depth),
            "lect_ffb_start_depth": int(args.ffb_start_depth),
            "lect_ffb_search_mode": str(args.ffb_search_mode),
            "lect_connector_pave_depth": int(args.connector_pave_depth),
            "lect_connector_adaptive_min_segment_fraction": float(args.connector_adaptive_min_segment_fraction),
            "lect_query_bridge_pave_depth": int(args.query_bridge_pave_depth),
            "lect_query_bridge_ffb_start_depth": int(args.query_bridge_ffb_start_depth),
            "lect_query_bridge_adaptive_ffb_depths": str(args.query_bridge_adaptive_ffb_depths),
            "lect_rbf_max_depth": int(args.rbf_max_depth),
            "audit_segment_step": float(args.audit_segment_step),
            "audit_collision_tolerance": float(args.audit_collision_tolerance),
            "query_shortcut_boxes": bool(args.query_shortcut_boxes),
            "segment_edges_fallback_only": bool(args.segment_edges_fallback_only),
            "connector_max_pairs_per_gap": int(args.connector_max_pairs_per_gap),
            "connector_pair_timeout_ms": float(args.connector_pair_timeout_ms),
            "connector_rrt_iters": int(args.connector_rrt_iters),
            "connector_rrt_timeout_ms": float(args.connector_rrt_timeout_ms),
            "connector_rrt_step_size": float(args.connector_rrt_step_size),
            "connector_rrt_goal_bias": float(args.connector_rrt_goal_bias),
            "connector_bridge_boxes": int(args.connector_bridge_boxes),
            "connector_pave_max_chain": int(args.connector_pave_max_chain),
            "connector_pave_fill_gaps": bool(args.connector_pave_fill_gaps),
            "connector_pave_require_connected_chain": bool(args.connector_pave_require_connected_chain),
            "connector_adaptive_min_segment_fraction": float(args.connector_adaptive_min_segment_fraction),
            "query_bridge_pave_depth": int(args.query_bridge_pave_depth),
            "query_bridge_adaptive_ffb_depths": str(args.query_bridge_adaptive_ffb_depths),
            "query_bridge_accept_segment_fraction": float(args.query_bridge_accept_segment_fraction),
            "query_bridge_accept_path_ratio": float(args.query_bridge_accept_path_ratio),
            "query_bridge_accept_path_additive": float(args.query_bridge_accept_path_additive),
            "query_endpoint_anchor_before_bridge": bool(args.query_endpoint_anchor_before_bridge),
            "query_bridge_direct_sample_step": float(args.query_bridge_direct_sample_step),
            "query_bridge_repair_subdivisions": int(args.query_bridge_repair_subdivisions),
            "query_bridge_force_indices": str(args.query_bridge_force_indices),
            "query_bridge_force_selected": bool(args.query_bridge_force_selected),
            "query_bridge_forced_attempts": int(args.query_bridge_forced_attempts),
            "query_bridge_attempt_offset": int(args.query_bridge_attempt_offset),
            "query_bridge_no_path_retry_attempts": int(args.query_bridge_no_path_retry_attempts),
            "query_bridge_hybridize_attempt_paths": bool(args.query_bridge_hybridize_attempt_paths),
            "query_bridge_hybrid_max_paths": int(args.query_bridge_hybrid_max_paths),
            "query_bridge_hybrid_max_vertices": int(args.query_bridge_hybrid_max_vertices),
            "query_bridge_hybrid_max_cross_checks": int(args.query_bridge_hybrid_max_cross_checks),
            "query_bridge_adaptive_step_repair": bool(args.query_bridge_adaptive_step_repair),
            "query_bridge_adaptive_fine_step": float(args.query_bridge_adaptive_fine_step),
            "query_bridge_adaptive_max_repair_subdivisions": int(args.query_bridge_adaptive_max_repair_subdivisions),
            "query_bridge_adaptive_max_repair_calls": int(args.query_bridge_adaptive_max_repair_calls),
            "query_bridge_adaptive_repair_priority": int(args.query_bridge_adaptive_repair_priority),
            "query_bridge_adaptive_repair_target_segment_fraction": float(
                args.query_bridge_adaptive_repair_target_segment_fraction
            ),
            "query_box_transition_line_deviation_penalty": float(args.query_box_transition_line_deviation_penalty),
            "query_foreign_edge_cost_penalty": float(args.query_foreign_edge_cost_penalty),
            "query_bridge_edge_cost_penalty": float(args.query_bridge_edge_cost_penalty),
            "query_bridge_direct_max_length": float(args.query_bridge_direct_max_length),
            "query_bridge_direct_segment_after_rrt": bool(args.query_bridge_direct_segment_after_rrt),
            "query_bridge_direct_segment_after_rrt_min_length": float(
                args.query_bridge_direct_segment_after_rrt_min_length
            ),
            "query_bridge_fast_direct_segment_after_rrt": bool(args.query_bridge_fast_direct_segment_after_rrt),
            "query_bridge_fast_direct_random_shortcut_iters": int(args.query_bridge_fast_direct_random_shortcut_iters),
            "query_bridge_to_main_island": bool(args.query_bridge_to_main_island),
            "query_bridge_to_main_direct_segment_max_length": float(args.query_bridge_to_main_direct_segment_max_length),
            "query_bridge_to_main_box_corridor": bool(args.query_bridge_to_main_box_corridor),
            "endpoint_main_target_k": int(args.endpoint_main_target_k),
            "endpoint_main_coarse_step": float(args.endpoint_main_coarse_step),
            "endpoint_main_fine_step": float(args.endpoint_main_fine_step),
            "endpoint_main_max_ffb_calls": int(args.endpoint_main_max_ffb_calls),
            "endpoint_main_max_boxes": int(args.endpoint_main_max_boxes),
            "endpoint_main_adaptive_ffb_depths": str(args.endpoint_main_adaptive_ffb_depths),
            "endpoint_main_residual_segment_max_length": float(args.endpoint_main_residual_segment_max_length),
            "endpoint_main_lateral_offset": float(args.endpoint_main_lateral_offset),
            "endpoint_main_lateral_rounds": int(args.endpoint_main_lateral_rounds),
            "endpoint_main_face_epsilon": float(args.endpoint_main_face_epsilon),
            "final_collision_shortcut": bool(args.final_collision_shortcut),
            "final_rrt_simplify": bool(args.final_rrt_simplify),
            "final_rrt_simplify_timeout_ms": float(args.final_rrt_simplify_timeout_ms),
            "final_rrt_simplify_max_iters": int(args.final_rrt_simplify_max_iters),
            "final_rrt_simplify_attempts": int(args.final_rrt_simplify_attempts),
            "corridor_refine": bool(args.corridor_refine),
            "corridor_refine_budget_ms": float(args.corridor_refine_budget_ms),
            "corridor_refine_max_boxes": int(args.corridor_refine_max_boxes),
            "corridor_refine_boxes_per_query": int(args.corridor_refine_boxes_per_query),
            "corridor_refine_passes": int(args.corridor_refine_passes),
            "corridor_refine_mode": str(args.corridor_refine_mode),
            "corridor_refine_long_path_ratio": float(args.corridor_refine_long_path_ratio),
            "corridor_refine_min_delta": float(args.corridor_refine_min_delta),
            "query_bridge_all": bool(args.query_bridge_all),
            "query_bridge_adaptive_all": bool(args.query_bridge_adaptive_all),
            "query_bridge_adaptive_max_path_length": float(args.query_bridge_adaptive_max_path_length),
            "query_bridge_accept_segment_fraction": float(args.query_bridge_accept_segment_fraction),
            "query_bridge_accept_path_ratio": float(args.query_bridge_accept_path_ratio),
            "query_bridge_accept_path_additive": float(args.query_bridge_accept_path_additive),
            "query_endpoint_anchor_before_bridge": bool(args.query_endpoint_anchor_before_bridge),
            "query_bridge_labels": str(args.query_bridge_labels),
            "query_bridge_segment_only_indices": str(args.query_bridge_segment_only_indices),
            "query_bridge_force_indices": str(args.query_bridge_force_indices),
            "query_bridge_forced_attempts": int(args.query_bridge_forced_attempts),
            "query_bridge_no_path_retry_attempts": int(args.query_bridge_no_path_retry_attempts),
            "query_bridge_hybridize_attempt_paths": bool(args.query_bridge_hybridize_attempt_paths),
            "query_bridge_hybrid_max_paths": int(args.query_bridge_hybrid_max_paths),
            "query_bridge_hybrid_max_vertices": int(args.query_bridge_hybrid_max_vertices),
            "query_bridge_hybrid_max_cross_checks": int(args.query_bridge_hybrid_max_cross_checks),
            "query_bridge_direct_max_length": float(args.query_bridge_direct_max_length),
            "query_bridge_direct_segment_after_rrt": bool(args.query_bridge_direct_segment_after_rrt),
            "query_bridge_direct_segment_after_rrt_min_length": float(
                args.query_bridge_direct_segment_after_rrt_min_length
            ),
            "query_bridge_fast_direct_segment_after_rrt": bool(args.query_bridge_fast_direct_segment_after_rrt),
            "query_bridge_fast_direct_random_shortcut_iters": int(args.query_bridge_fast_direct_random_shortcut_iters),
            "query_bridge_to_main_island": bool(args.query_bridge_to_main_island),
            "query_bridge_to_main_direct_segment_max_length": float(args.query_bridge_to_main_direct_segment_max_length),
            "query_bridge_to_main_box_corridor": bool(args.query_bridge_to_main_box_corridor),
            "endpoint_main_target_k": int(args.endpoint_main_target_k),
            "endpoint_main_coarse_step": float(args.endpoint_main_coarse_step),
            "endpoint_main_fine_step": float(args.endpoint_main_fine_step),
            "endpoint_main_max_ffb_calls": int(args.endpoint_main_max_ffb_calls),
            "endpoint_main_max_boxes": int(args.endpoint_main_max_boxes),
            "endpoint_main_adaptive_ffb_depths": str(args.endpoint_main_adaptive_ffb_depths),
            "endpoint_main_residual_segment_max_length": float(args.endpoint_main_residual_segment_max_length),
            "endpoint_main_lateral_offset": float(args.endpoint_main_lateral_offset),
            "endpoint_main_lateral_rounds": int(args.endpoint_main_lateral_rounds),
            "endpoint_main_face_epsilon": float(args.endpoint_main_face_epsilon),
            "run_rrt_grower": bool(args.run_rrt_grower),
            "rrt_grower_extra_boxes": int(args.rrt_grower_extra_boxes),
            "rrt_grower_timeout_ms": float(args.rrt_grower_timeout_ms),
            "priority_prune_radius": float(args.priority_prune_radius),
            "collision_overlap_prune_min_depth": int(args.collision_overlap_prune_min_depth),
            "collision_overlap_prune_threshold": float(args.collision_overlap_prune_threshold),
            "collision_overlap_prune_ratio_threshold": float(args.collision_overlap_prune_ratio_threshold),
            "critical_sample_row": "critsample_d23_cache changes only the endpoint source to CritSample and replays the matching depth-23 CritSample LECT cache. The planner uses the same envelope-collision free rule and final strict audit as baseline; theoretically CritSample envelopes are not conservative-complete.",
        },
        "config_audit": config_audit,
        "planned_rows": manifest_rows,
        "rows": run_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "shelf_leaf_rrt_manifest.json", payload)
    if summary_rows:
        write_csv(args.out_dir / "shelf_leaf_rrt_summary.csv", summary_rows)
        write_tex(args.out_dir / "tab_tro_shelf_ablation.tex", summary_rows)
    print(f"wrote {args.out_dir / 'shelf_leaf_rrt_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
