from __future__ import annotations

import math
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
CANONICAL_SYMMETRY_DESCRIPTOR = "joint_symmetry_native_v1"

RBF_DEFAULT_PROFILE_NAME = "exp04_partition_leaf13_d23_fixed800_online25ms"
RBF_SHELF_PROFILE_NAME = "exp04_partition_leaf13_d23_fixed800_online25ms"
RBF_OFFLINE_COVERAGE_PROFILE_NAME = "offline_coverage_v1"
RBF_DEFAULT_BACKEND = "build_leaf_sweep_refined"
RBF_DEFAULT_GROWER_MODE = "rrt"
EXP06_REGISTERED_RBF_PROFILE_NAME = "exp06_rbf_robot_tuned_profile_v30_obb_hybrid_online_fast_fullsuccess"

D23_CACHE_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_full_root_d23" / "cache"
D23_CACHE_LABEL = "iiwa_endpoint_only_p23_canonical_full_root"
CRITSAMPLE_D23_CACHE_LABEL = "iiwa_critsample_p23_canonical_full_root"
ROBOT_LECTDB_CACHE_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "tro2026" / "lectdb_defaults" / "cache"
ROBOT_LECTDB_MAX_DEPTH = 40
ROBOT_DEFAULT_LECTDB_DEPTHS: dict[str, int] = {
    "iiwa": 23,
    "ur5": 20,
    "panda": 20,
}
DEFAULT_RBF_LEAF_START_DEPTH = 8
DEFAULT_RBF_LEAF_MAX_DEPTH = 13
DEFAULT_RBF_MAX_DEPTH = 64
DEFAULT_RBF_DEEP_MAX_BOXES = 400
DEFAULT_RBF_SHELF_BOX_BUDGET = 100
DEFAULT_RBF_DEEP_FFB_DEPTH = 62
DEFAULT_RBF_REFINE_TIMEOUT_MS = 800.0
DEFAULT_RBF_DOMAIN_SEED_CAP = 3
DEFAULT_RBF_DOMAIN_SUCCESS_CAP = 2
DEFAULT_RBF_DOMAIN_ATTEMPT_CAP = 3
DEFAULT_RBF_VALIDATION_BATCH_SIZE = 512
DEFAULT_RBF_THREADS = 8
DEFAULT_RBF_FFB_START_DEPTH = 32
DEFAULT_RBF_FFB_SEARCH_MODE = "binary"
DEFAULT_RBF_FFB_IMPLEMENTATION = "virtual_sparse_binary"

DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS = 18.0
DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP = 2
DEFAULT_RBF_CONNECTOR_RRT_ITERS = 50000
DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS = 2000.0
DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE = 0.5
DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS = 0.2
DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION = 16
DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES = 200
DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN = 160
DEFAULT_RBF_CONNECTOR_PAVE_STEPS = 12
DEFAULT_RBF_CONNECTOR_PAVE_DEPTH = 62
DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION = 0.75
DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH = 52
DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP = 0.08
DEFAULT_RBF_QUERY_BRIDGE_FORCE_SELECTED = True
DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS = 12
DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET = 3
DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS = 0
DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS = 1600
DEFAULT_RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE = ""
DEFAULT_RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS = False
DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS = 8
DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES = 128
DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS = 4096
DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP = False
DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES = 1
DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO = 1.75
DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE = 0.75
DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT = False
DEFAULT_RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS = 0
DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP = 0.08
DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS = 2
DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS = 24
DEFAULT_RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED = False
DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS = False
DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY = 2.0
DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY = 2.0
DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY = 5.0
DEFAULT_RBF_QUERY_ENDPOINT_ANCHOR_BEFORE_BRIDGE = False
DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS = False
DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS = True
DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN = True

DEFAULT_RBF_AUDIT_RESOLUTION = 16
DEFAULT_RBF_AUDIT_SEGMENT_STEP = 0.01
DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE = 0.0
DEFAULT_RBF_FINAL_COLLISION_SHORTCUT = True
DEFAULT_RBF_FINAL_RRT_SIMPLIFY = True
DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS = 10.0
DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS = 50000
DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS = 8
DEFAULT_RBF_QUERY_BRIDGE_ALL = True
DEFAULT_RBF_QUERY_BRIDGE_LABELS = "AS->TS,TS->CS,CS->LB,LB->RB,RB->AS"
DEFAULT_RBF_QUERY_BRIDGE_FORCE_INDICES = "0,1,2,3,4"
DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH = 14
DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD = 0.05
DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD = 0.0

OFFLINE_COVERAGE_V1_SETTINGS: dict[str, Any] = {
    # Query-agnostic coverage first: spend modest offline time improving the
    # reusable partition, then let online queries attach and repair locally.
    "leaf_max_depth": 13,
    "adaptive_target_depth": 24,
    "adaptive_grid_target_depth": 24,
    "adaptive_time_budget_ms": 2000.0,
    "adaptive_node_budget": 50000,
    "adaptive_depth_enabled": True,
    "adaptive_depth_min": 13,
    "adaptive_depth_max": 24,
    "adaptive_depth_probe_count": 4096,
    "adaptive_depth_anchor_probe_cap": 256,
    "adaptive_depth_min_free_probes": 64,
    "adaptive_depth_min_covered_probes": 32,
    "adaptive_depth_min_main_probes": 24,
    "adaptive_depth_min_main_ratio": 0.40,
    "adaptive_depth_max_online_cells": 1500,
    "adaptive_depth_max_probe_ms": 20.0,
    "coverage_probe_count": 4096,
    "adaptive_seed_anchor_probe_cap": 256,
    "offline_random_anchors": True,
    "offline_anchor_count": 32,
    "offline_anchor_candidate_count": 1024,
    "offline_anchor_skip_if_main_accessible": False,
    "offline_connector_mode": "short_segment",
    "offline_shortcut_edges": 16,
    "offline_shortcut_candidate_limit": 64,
    "offline_shortcut_min_gain_ratio": 1.2,
    "offline_shortcut_max_segment_length": 0.5,
    "query_bridge_force_selected": False,
    "query_bridge_to_main_island": True,
    "query_bridge_failure_fallback_to_main": True,
    "query_bridge_to_main_box_corridor": True,
    "query_bridge_sequential_reuse": True,
    "query_bridge_scene_reusable_edges": True,
    "query_bridge_accept_segment_fraction": 0.40,
    "query_bridge_accept_path_ratio": 1.50,
    "query_bridge_accept_path_additive": 0.75,
    "hipac_online_connectivity": True,
    "hipac_online_prebridge_portal": False,
    "hipac_promote_query_repairs": True,
}


def offline_coverage_v1_profile() -> dict[str, Any]:
    return dict(OFFLINE_COVERAGE_V1_SETTINGS)


def _profile_flag_was_supplied(argv: list[str], flag: str) -> bool:
    negated = f"--no-{flag[2:]}" if flag.startswith("--") else None
    return any(
        item == flag
        or item.startswith(f"{flag}=")
        or (negated is not None and (item == negated or item.startswith(f"{negated}=")))
        for item in argv
    )


def apply_offline_coverage_profile(args: Any, argv: list[str] | None = None) -> None:
    """Apply the reusable offline-coverage profile unless a CLI flag overrides it."""
    if str(getattr(args, "offline_coverage_profile", "")).strip() != RBF_OFFLINE_COVERAGE_PROFILE_NAME:
        return
    supplied = argv or []
    flag_by_attr = {
        "leaf_max_depth": "--leaf-max-depth",
        "adaptive_target_depth": "--adaptive-target-depth",
        "adaptive_grid_target_depth": "--adaptive-grid-target-depth",
        "adaptive_time_budget_ms": "--adaptive-time-budget-ms",
        "adaptive_node_budget": "--adaptive-node-budget",
        "adaptive_depth_enabled": "--adaptive-depth-enabled",
        "adaptive_depth_min": "--adaptive-depth-min",
        "adaptive_depth_max": "--adaptive-depth-max",
        "adaptive_depth_probe_count": "--adaptive-depth-probe-count",
        "adaptive_depth_anchor_probe_cap": "--adaptive-depth-anchor-probe-cap",
        "adaptive_depth_min_free_probes": "--adaptive-depth-min-free-probes",
        "adaptive_depth_min_covered_probes": "--adaptive-depth-min-covered-probes",
        "adaptive_depth_min_main_probes": "--adaptive-depth-min-main-probes",
        "adaptive_depth_min_main_ratio": "--adaptive-depth-min-main-ratio",
        "adaptive_depth_max_online_cells": "--adaptive-depth-max-online-cells",
        "adaptive_depth_max_probe_ms": "--adaptive-depth-max-probe-ms",
        "coverage_probe_count": "--coverage-probe-count",
        "adaptive_seed_anchor_probe_cap": "--adaptive-seed-anchor-probe-cap",
        "offline_random_anchors": "--offline-random-anchors",
        "offline_anchor_count": "--offline-anchor-count",
        "offline_anchor_candidate_count": "--offline-anchor-candidate-count",
        "offline_anchor_skip_if_main_accessible": "--offline-anchor-skip-if-main-accessible",
        "offline_connector_mode": "--offline-connector-mode",
        "offline_shortcut_edges": "--offline-shortcut-edges",
        "offline_shortcut_candidate_limit": "--offline-shortcut-candidate-limit",
        "offline_shortcut_min_gain_ratio": "--offline-shortcut-min-gain-ratio",
        "offline_shortcut_max_segment_length": "--offline-shortcut-max-segment-length",
        "query_bridge_force_selected": "--query-bridge-force-selected",
        "query_bridge_to_main_island": "--query-bridge-to-main-island",
        "query_bridge_failure_fallback_to_main": "--query-bridge-failure-fallback-to-main",
        "query_bridge_to_main_box_corridor": "--query-bridge-to-main-box-corridor",
        "query_bridge_sequential_reuse": "--query-bridge-sequential-reuse",
        "query_bridge_scene_reusable_edges": "--query-bridge-scene-reusable-edges",
        "query_bridge_accept_segment_fraction": "--query-bridge-accept-segment-fraction",
        "query_bridge_accept_path_ratio": "--query-bridge-accept-path-ratio",
        "query_bridge_accept_path_additive": "--query-bridge-accept-path-additive",
        "hipac_online_connectivity": "--hipac-online-connectivity",
        "hipac_online_prebridge_portal": "--hipac-online-prebridge-portal",
        "hipac_promote_query_repairs": "--hipac-promote-query-repairs",
    }
    for attr, value in OFFLINE_COVERAGE_V1_SETTINGS.items():
        flag = flag_by_attr.get(attr)
        if flag is not None and _profile_flag_was_supplied(supplied, flag):
            continue
        if hasattr(args, attr):
            setattr(args, attr, value)

_EXP06_REGISTERED_RBF_BASE_SETTINGS: dict[str, Any] = {
    # Keep the shallow offline cover fast, but allow endpoint/query FFB to
    # certify isolated hard-scene endpoints beyond the historical d64 cap.
    "rbf_max_depth": 80,
    "leaf_max_depth": 10,
    "deep_ffb_depth": 56,
    "connector_pave_depth": 56,
    "query_bridge_pave_depth": 56,
    "query_endpoint_anchor_ffb_depth": 110,
    "ffb_start_depth": 32,
    "query_bridge_ffb_start_depth": -1,
    "query_bridge_edge_cost_penalty": 5.0,
    "query_bridge_force_selected": True,
    "query_bridge_rrt_fixed_iters": 10000,
    "query_bridge_no_path_retry_attempts": 32,
    "query_bridge_no_path_retry_stop_on_first_success": True,
    "query_bridge_direct_segment_after_rrt": True,
    "query_bridge_fast_direct_segment_after_rrt": True,
    "query_bridge_fast_direct_random_shortcut_iters": 0,
    "query_bridge_direct_max_length": 15.0,
    "query_bridge_to_main_island": False,
    "query_bridge_full_residual_overlay_when_connected": False,
    "hipac_improved_leaf_sweep": False,
    "hipac_online_connectivity": False,
    "hipac_online_prebridge_portal": False,
    "segment_edge_obb_cover": True,
    "rrt_bridge_obb_cover": True,
    "strict_obb_bridge_cover": True,
    "segment_edge_obb_lateral_radius": 1.0e-5,
    "segment_edge_obb_grow_iterations": 0,
    "segment_edge_obb_binary_iterations": 0,
    "segment_edge_obb_split_depth": 8,
    "obb_max_window_segments": 8,
    "obb_max_validations_per_window": 16,
    "obb_fast_primary_orientation": True,
    "obb_fallback_orientations_on_primary_fail": False,
}

# Paper-facing Exp.6 RBF profile.  Keep this table as the single source of
# truth for the registered random-scene RBF trade-off point; experiments that
# sweep parameters must opt out or pass explicit CLI overrides.
EXP06_REGISTERED_RBF_SETTINGS: dict[tuple[str, str], dict[str, Any]] = {
    ("iiwa", "medium"): {**_EXP06_REGISTERED_RBF_BASE_SETTINGS, "deep_max_boxes": 200},
    ("iiwa", "hard"): {
        **_EXP06_REGISTERED_RBF_BASE_SETTINGS,
        "rbf_max_depth": 110,
        "deep_max_boxes": 400,
    },
    ("ur5", "medium"): {
        **_EXP06_REGISTERED_RBF_BASE_SETTINGS,
        # Strict OBB bridge cover is the stable UR5 operating point: it keeps
        # all bridge witnesses conservative and avoids post-audit failures from
        # uncertified direct repair edges.
        "leaf_max_depth": 8,
        "deep_max_boxes": 400,
        "query_endpoint_anchor_ffb_depth": 80,
        "ffb_start_depth": 8,
        "query_bridge_ffb_start_depth": 8,
        "connector_rrt_step_size": 0.75,
        "query_bridge_rrt_fixed_iters": 2000,
        "query_bridge_fast_direct_random_shortcut_iters": 128,
        "query_bridge_sequential_reuse": False,
        "query_bridge_scene_reusable_edges": False,
        "query_bridge_forced_attempts": 32,
        "query_bridge_hybridize_attempt_paths": True,
        "query_bridge_hybrid_max_paths": 32,
        "query_bridge_hybrid_max_vertices": 384,
        "query_bridge_hybrid_max_cross_checks": 16384,
        "query_bridge_no_path_retry_attempts": 0,
        "query_bridge_no_path_retry_budget_iters": "10000,20000",
        "query_bridge_no_path_retry_budget_attempts": "1,1",
        # Try local RRT bridge radii first, then fall back to the original
        # unrestricted bridge attempt in C++ so the schedule cannot remove
        # previously valid OBB witnesses.
        "query_bridge_local_radius_schedule": "0.5,1.0,2.0,4.0",
        "query_bridge_full_residual_overlay_when_connected": True,
    },
    ("ur5", "hard"): {
        **_EXP06_REGISTERED_RBF_BASE_SETTINGS,
        "leaf_max_depth": 8,
        "deep_max_boxes": 400,
        "query_endpoint_anchor_ffb_depth": 80,
        "ffb_start_depth": 8,
        "query_bridge_ffb_start_depth": 8,
        "connector_rrt_step_size": 0.75,
        "query_bridge_rrt_fixed_iters": 3480,
        "query_bridge_fast_direct_random_shortcut_iters": 128,
        # v28 online-aggressive point: forced20 trades a slight median path
        # ratio increase for a large online-query time reduction.
        "query_bridge_forced_attempts": 20,
        "query_bridge_hybridize_attempt_paths": True,
        "query_bridge_hybrid_max_paths": 32,
        "query_bridge_hybrid_max_vertices": 384,
        "query_bridge_hybrid_max_cross_checks": 16384,
        "query_bridge_failure_fallback_to_main": True,
        "query_bridge_parallel_rrt_early_stop": True,
        "query_bridge_parallel_rrt_early_stop_min_successes": 1,
        "query_bridge_parallel_rrt_early_stop_ratio": 1.0,
        "query_bridge_parallel_rrt_early_stop_additive": 0.0,
        "query_bridge_no_path_retry_attempts": 0,
        "query_bridge_no_path_retry_budget_iters": "10000,20000",
        "query_bridge_no_path_retry_budget_attempts": "1,1",
        "query_bridge_local_radius_schedule": "0.5,1.0,2.0,4.0",
        "query_bridge_full_residual_overlay_when_connected": True,
    },
    ("panda", "medium"): {
        **_EXP06_REGISTERED_RBF_BASE_SETTINGS,
        "deep_max_boxes": 400,
        "ffb_start_depth": 8,
        "query_bridge_ffb_start_depth": 8,
        "query_bridge_edge_cost_penalty": 5.0,
        "query_bridge_to_main_island": False,
        "query_bridge_force_selected": True,
        "query_bridge_failure_fallback_to_main": False,
        "query_bridge_rrt_fixed_iters": 10000,
        "query_bridge_fast_direct_random_shortcut_iters": 128,
        # v30 full-success point: forced12 was faster but dropped one
        # Panda-Medium saved-catalog query; forced14 restores 100/100 success
        # while preserving most of the online-query speedup.
        "query_bridge_forced_attempts": 14,
        "query_bridge_hybridize_attempt_paths": True,
        "query_bridge_hybrid_max_paths": 14,
        "query_bridge_hybrid_max_vertices": 176,
        "query_bridge_hybrid_max_cross_checks": 7168,
        "query_bridge_no_path_retry_attempts": 32,
        "query_bridge_no_path_retry_budget_iters": "",
        "query_bridge_no_path_retry_budget_attempts": "",
        "query_bridge_full_residual_overlay_when_connected": True,
        "segment_edge_obb_cover": True,
        "rrt_bridge_obb_cover": True,
        "strict_obb_bridge_cover": True,
        "obb_max_window_segments": 8,
        "obb_max_validations_per_window": 16,
    },
    ("panda", "hard"): {
        **_EXP06_REGISTERED_RBF_BASE_SETTINGS,
        # Panda-Hard needs the high-budget direct witness profile for full
        # saved-catalog success.  We keep the original audited bridge geometry
        # but attach OBB metadata when the bridge can be certified by the OBB
        # cover, so this row remains an OBB-enabled profile without switching
        # back to the non-OBB v4 profile.
        "deep_max_boxes": 400,
        "ffb_start_depth": 8,
        "query_bridge_ffb_start_depth": 8,
        "query_bridge_edge_cost_penalty": 5.0,
        "query_bridge_direct_sample_step": 0.08,
        "query_bridge_direct_segment_after_rrt": True,
        "query_bridge_fast_direct_segment_after_rrt": True,
        "query_bridge_fast_direct_random_shortcut_iters": 128,
        "query_bridge_to_main_island": False,
        "query_bridge_force_selected": True,
        "query_bridge_failure_fallback_to_main": False,
        "query_endpoint_point_anchor": True,
        "query_bridge_accept_path_ratio": 1.50,
        "query_bridge_accept_path_additive": 0.75,
        "query_bridge_rrt_fixed_iters": 40000,
        "query_bridge_forced_attempts": 12,
        "query_bridge_attempt_offset": 3,
        "query_bridge_hybridize_attempt_paths": True,
        "query_bridge_hybrid_max_paths": 8,
        "query_bridge_hybrid_max_vertices": 128,
        "query_bridge_hybrid_max_cross_checks": 4096,
        "query_bridge_parallel_rrt_early_stop": True,
        "query_bridge_parallel_rrt_early_stop_min_successes": 1,
        "query_bridge_parallel_rrt_early_stop_ratio": 100.0,
        "query_bridge_parallel_rrt_early_stop_additive": 100.0,
        "query_bridge_no_path_retry_attempts": 0,
        "query_bridge_no_path_retry_budget_iters": "",
        "query_bridge_no_path_retry_budget_attempts": "",
        "query_bridge_local_radius_schedule": "0.5,1.0,2.0,4.0",
        "query_bridge_sequential_reuse": True,
        "query_bridge_scene_reusable_edges": True,
        "query_bridge_full_residual_overlay_when_connected": True,
        "segment_edge_obb_cover": True,
        "rrt_bridge_obb_cover": True,
        "strict_obb_bridge_cover": False,
        "segment_edge_obb_metadata_only": True,
        "segment_edge_obb_lateral_radius": 1.0e-5,
        "segment_edge_obb_grow_iterations": 0,
        "segment_edge_obb_binary_iterations": 0,
        "segment_edge_obb_split_depth": 8,
        "obb_max_window_segments": 8,
        "obb_max_validations_per_window": 16,
    },
}


def robot_joint_limit_tuples(robot: Any) -> list[tuple[float, float]]:
    return [
        (float(interval.lo), float(interval.hi))
        for interval in list(robot.joint_limits().limits)
    ]


def robot_symmetry_aligned_root_tuples(robot: Any) -> list[tuple[float, float]]:
    intervals = robot_joint_limit_tuples(robot)
    if intervals:
        intervals[0] = (-math.pi, math.pi)
    return intervals


def default_rbf_profile() -> dict[str, Any]:
    return {
        "profile": RBF_DEFAULT_PROFILE_NAME,
        "backend": RBF_DEFAULT_BACKEND,
        "offline_grower": "adaptive_deep_leaf",
        "online_backend": "partition_native",
        "grower_mode": RBF_DEFAULT_GROWER_MODE,
        "cache": {
            "mode": "robot_native_stateless_external_evidence",
            "depth_semantics": "lect_canonical_tree",
            "default_depths": dict(ROBOT_DEFAULT_LECTDB_DEPTHS),
            "root": str(ROBOT_LECTDB_CACHE_ROOT),
            "max_depth": ROBOT_LECTDB_MAX_DEPTH,
            "active_planning_root": "full_robot_joint_limits",
            "coverage_root": "full_robot_joint_limits",
            "canonical_mapping_scope": "LECT_internal_only",
        },
        "leaf_sweep": {
            "depth_semantics": "lect_active_tree",
            "leaf_start_depth": DEFAULT_RBF_LEAF_START_DEPTH,
            "leaf_max_depth": DEFAULT_RBF_LEAF_MAX_DEPTH,
            "adaptive_target_depth": DEFAULT_RBF_LEAF_MAX_DEPTH,
            "adaptive_planning_backend": "partition_native",
            "adaptive_grid_target_depth": DEFAULT_RBF_LEAF_MAX_DEPTH,
            "adaptive_node_budget": 50000,
            "fast_virtual_checkpoint_mode": False,
            "terminal_controller": True,
            "adaptive_depth": {
                "enabled": True,
                "min": DEFAULT_RBF_LEAF_MAX_DEPTH,
                "max": 16,
                "probe_count": 512,
                "anchor_probe_cap": 32,
                "probe_seed": 20260607,
                "min_free_probes": 64,
                "min_covered_probes": 3,
                "min_main_probes": 3,
                "min_main_ratio": 0.35,
                "min_cells": 0,
                "min_main_cells": 0,
                "max_online_cells": 180,
                "max_probe_ms": 5.0,
            },
            "use_virtual_topology": True,
            "parallel_virtual_validation": True,
            "store_group_results": False,
            "obstacle_cluster_gap": 1000.0,
            "validation_batch_size": DEFAULT_RBF_VALIDATION_BATCH_SIZE,
            "leaf_threads": DEFAULT_RBF_THREADS,
            "collision_overlap_prune_min_depth": DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_MIN_DEPTH,
            "collision_overlap_prune_threshold": DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_THRESHOLD,
            "collision_overlap_prune_ratio_threshold": DEFAULT_RBF_COLLISION_OVERLAP_PRUNE_RATIO_THRESHOLD,
        },
        "deep_refine": {
            "depth_semantics": "lect_active_tree",
            "deep_max_boxes": DEFAULT_RBF_DEEP_MAX_BOXES,
            "deep_ffb_depth": DEFAULT_RBF_DEEP_FFB_DEPTH,
            "ffb_start_depth": DEFAULT_RBF_FFB_START_DEPTH,
            "ffb_search_mode": DEFAULT_RBF_FFB_SEARCH_MODE,
            "ffb_implementation": DEFAULT_RBF_FFB_IMPLEMENTATION,
            "domain_seed_cap": DEFAULT_RBF_DOMAIN_SEED_CAP,
            "domain_success_cap": DEFAULT_RBF_DOMAIN_SUCCESS_CAP,
            "domain_attempt_cap": DEFAULT_RBF_DOMAIN_ATTEMPT_CAP,
            "refine_timeout_ms": DEFAULT_RBF_REFINE_TIMEOUT_MS,
        },
        "connector": {
            "depth_semantics": "lect_active_tree",
            "ffb_search_mode": DEFAULT_RBF_FFB_SEARCH_MODE,
            "ffb_implementation": DEFAULT_RBF_FFB_IMPLEMENTATION,
            "segment_edges_enabled": True,
            "segment_edges_fallback_only": False,
            "segment_step": DEFAULT_RBF_AUDIT_SEGMENT_STEP,
            "enable_birrt": True,
            "max_pairs_per_gap": DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
            "per_pair_timeout_ms": DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
            "rrt_iters": DEFAULT_RBF_CONNECTOR_RRT_ITERS,
            "rrt_timeout_ms": DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS,
            "rrt_step_size": DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE,
            "rrt_goal_bias": DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS,
            "segment_resolution": DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION,
            "bridge_boxes": DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES,
            "pave_max_chain": DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN,
            "pave_steps": DEFAULT_RBF_CONNECTOR_PAVE_STEPS,
            "pave_depth": DEFAULT_RBF_CONNECTOR_PAVE_DEPTH,
            "adaptive_min_segment_fraction": DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION,
            "pave_fill_gaps": DEFAULT_RBF_CONNECTOR_PAVE_FILL_GAPS,
            "pave_require_connected_chain": DEFAULT_RBF_CONNECTOR_PAVE_REQUIRE_CONNECTED_CHAIN,
        },
        "query_bridge": {
            "depth_semantics": "lect_active_tree",
            "ffb_search_mode": DEFAULT_RBF_FFB_SEARCH_MODE,
            "ffb_implementation": DEFAULT_RBF_FFB_IMPLEMENTATION,
            "pave_depth": DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
            "ffb_start_depth": -1,
            "direct_sample_step": DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP,
            "force_selected": DEFAULT_RBF_QUERY_BRIDGE_FORCE_SELECTED,
            "forced_attempts": DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS,
            "attempt_offset": DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET,
            "no_path_retry_attempts": DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS,
            "no_path_retry_stop_on_first_success": DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS,
            "rrt_fixed_iters": DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS,
            "adaptive_fine_step": DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP,
            "adaptive_max_repair_subdivisions": DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS,
            "adaptive_max_repair_calls": DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS,
            "full_residual_overlay_when_connected": (
                DEFAULT_RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED
            ),
            "box_transition_line_deviation_penalty": DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY,
            "foreign_edge_cost_penalty": DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY,
            "query_bridge_edge_cost_penalty": DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY,
            "adaptive_all": True,
            "adaptive_max_path_length": 4.5,
            "accept_segment_fraction": 0.25,
            "accept_path_ratio": 1.50,
            "accept_path_additive": 0.75,
            "endpoint_anchor_before_bridge": DEFAULT_RBF_QUERY_ENDPOINT_ANCHOR_BEFORE_BRIDGE,
            "note": "Selected shelf queries are bridged directly with endpoint anchoring inside the bridge stage. Existing graph paths are accepted when strict audit passes, segment fraction is below the registered threshold, and path length is within the registered bound; full bridge is reserved for queries that fail these gates.",
        },
        "query": {
            "state_space": "native_joint_space",
            "strict_path_audit": True,
            "shortcut_boxes": False,
            "collision_shortcut": DEFAULT_RBF_FINAL_COLLISION_SHORTCUT,
            "final_rrt_simplify": DEFAULT_RBF_FINAL_RRT_SIMPLIFY,
            "final_rrt_simplify_timeout_ms": DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
            "final_rrt_simplify_attempts": DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
            "final_rrt_simplify_domain": "robot_joint_limits",
            "query_bridge_all": DEFAULT_RBF_QUERY_BRIDGE_ALL,
            "query_bridge_labels": DEFAULT_RBF_QUERY_BRIDGE_LABELS,
            "audit_resolution": DEFAULT_RBF_AUDIT_RESOLUTION,
            "audit_segment_step": DEFAULT_RBF_AUDIT_SEGMENT_STEP,
            "audit_collision_tolerance": DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
            "planning_time_excludes_audit": True,
        },
        "recommended_tradeoff": {
            "deep_max_boxes": DEFAULT_RBF_DEEP_MAX_BOXES,
            "validated_on": "Exp.4/5 shelf baseline seeds 0..7, partition-native adaptive leaf sweep d13, d23 external evidence, FFB start depth 32, query bridge depth 52, direct sample step 0.08 rad, repair subdivisions 1, adaptive repair priority 1, adaptive repair cap 24, forced bridge attempts 12 with attempt offset 3, fixed 1600 query-bridge RRT iterations without wall-clock cutoff, stable label-based query indices, line-deviation penalty 2.0, query-bridge edge penalty 5.0, connector pair timeout 18 ms, 10 ms main simplify budget with 8 simplify attempts",
            "success_runs": "8/8",
            "success_queries": "40/40",
            "median_offline_build_s": 0.09330,
            "median_online_solve_per_query_s": 0.02517,
            "median_online_total_per_query_s": 0.03255,
            "median_raw_segment_fraction": 0.27594,
            "mean_route_length": 2.87340,
            "source": "outputs/perf_partition_native_waypoints/default1280_regression_1780876086/exp04",
        },
    }


def shelf_d23_rbf_profile() -> dict[str, Any]:
    profile = default_rbf_profile()
    profile["profile"] = RBF_SHELF_PROFILE_NAME
    profile["cache"] = {
        "mode": "d23_warm_external_evidence",
        "depth_semantics": "lect_canonical_tree",
        "root": str(D23_CACHE_ROOT),
        "label": D23_CACHE_LABEL,
        "root_intervals": "canonical_root_internal_to_LECT",
        "active_lect_root": "canonical_primary_sector",
        "active_planning_root": "full_robot_joint_limits",
        "coverage_root": "full_robot_joint_limits",
        "split_schedule": "external_cache_manifest_prefix",
        "cache_split_schedule": "full_joint_canonical_aafk_manifest",
        "canonical_mapping_scope": "LECT_internal_only",
    }
    return profile


def robot_lectdb_depth(robot_name: str) -> int:
    return int(ROBOT_DEFAULT_LECTDB_DEPTHS.get(str(robot_name), 20))


def robot_lectdb_label(robot_name: str, *, depth: int | None = None, envelope: str = "support_hull") -> str:
    actual_depth = robot_lectdb_depth(robot_name) if depth is None else int(depth)
    robot_key = str(robot_name)
    if robot_key == "panda":
        return (
            f"tro2026_panda_full_wrist_p{actual_depth}_{envelope}_sched_"
            f"d{ROBOT_LECTDB_MAX_DEPTH}_canonical_native_stateless"
        )
    if robot_key == "ur5":
        return (
            f"tro2026_ur5_p{actual_depth}_{envelope}_sched_"
            f"d{ROBOT_LECTDB_MAX_DEPTH}_canonical_native_stateless"
        )
    return f"tro2026_{robot_key}_p{actual_depth}_{envelope}_d{ROBOT_LECTDB_MAX_DEPTH}_canonical_native_stateless"


def robot_lectdb_path(robot_name: str, *, depth: int | None = None, envelope: str = "support_hull") -> Path:
    if str(robot_name) == "iiwa":
        return D23_CACHE_ROOT / D23_CACHE_LABEL
    return ROBOT_LECTDB_CACHE_ROOT / robot_lectdb_label(robot_name, depth=depth, envelope=envelope)


def robot_lectdb_profile(robot_name: str) -> dict[str, Any]:
    depth = robot_lectdb_depth(robot_name)
    if str(robot_name) == "iiwa":
        return {
            "mode": "full_joint_d23_external_evidence",
            "depth_semantics": "lect_canonical_tree",
            "robot": "iiwa",
            "depth": 23,
            "root": str(D23_CACHE_ROOT),
            "label": D23_CACHE_LABEL,
            "path": str(D23_CACHE_ROOT / D23_CACHE_LABEL),
            "root_intervals": "canonical_root_internal_to_LECT",
            "active_lect_root": "canonical_primary_sector",
            "coverage_domain": "full_robot_joint_limits",
            "canonical_mode": True,
            "active_planning_root": "full_robot_joint_limits",
            "split_schedule": "external_cache_manifest_prefix",
            "cache_split_schedule": "full_joint_canonical_aafk_manifest",
            "canonical_mapping_scope": "LECT_internal_only",
            "symmetry_descriptor": CANONICAL_SYMMETRY_DESCRIPTOR,
        }
    return {
        "mode": "robot_native_stateless_external_evidence",
        "depth_semantics": "lect_canonical_tree",
        "robot": str(robot_name),
        "depth": depth,
        "root": str(ROBOT_LECTDB_CACHE_ROOT),
        "label": robot_lectdb_label(robot_name, depth=depth),
        "path": str(robot_lectdb_path(robot_name, depth=depth)),
        "max_depth": ROBOT_LECTDB_MAX_DEPTH,
        "canonical_mode": True,
        "active_planning_root": "full_robot_joint_limits",
        "canonical_mapping_scope": "LECT_internal_only",
        "symmetry_descriptor": CANONICAL_SYMMETRY_DESCRIPTOR,
    }


def robot_root_override_tuples(robot_name: str) -> list[tuple[float, float]] | None:
    return None


def robot_sector_expanded_root_tuples(robot_name: str, robot: Any | None = None) -> list[tuple[float, float]] | None:
    if robot is None:
        return None
    try:
        from experiments.common.sbf_import import import_sbf

        sbf = import_sbf()
        intervals = [
            (float(interval.lo), float(interval.hi))
            for interval in sbf.canonical_root_intervals_for_robot(
                robot,
                True,
                CANONICAL_SYMMETRY_DESCRIPTOR,
            )
        ]
    except Exception:
        return None
    width = intervals[0][1] - intervals[0][0]
    if width <= 1e-12:
        return intervals
    if robot is not None:
        limits = list(robot.joint_limits().limits)
        limit_lo = float(limits[0].lo)
        limit_hi = float(limits[0].hi)
    else:
        limit_lo = -2.9668
        limit_hi = 2.9668
    lo_values: list[float] = []
    hi_values: list[float] = []
    for shift in range(-16, 17):
        lo = intervals[0][0] + shift * width
        hi = intervals[0][1] + shift * width
        if hi < limit_lo - 1e-9 or lo > limit_hi + 1e-9:
            continue
        lo_values.append(max(lo, limit_lo))
        hi_values.append(min(hi, limit_hi))
    if lo_values and hi_values:
        intervals[0] = (min(lo_values), max(hi_values))
    return intervals


def rbf_budget_grid(phase: str) -> list[int]:
    if phase == "smoke":
        return [DEFAULT_RBF_DEEP_MAX_BOXES]
    return [100, 200, 400, 800]
