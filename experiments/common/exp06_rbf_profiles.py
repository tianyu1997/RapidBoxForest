from __future__ import annotations

import argparse
import copy
import sys
from typing import Any

from experiments.common.rbf_defaults import (
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
    DEFAULT_RBF_FFB_IMPLEMENTATION,
    EXP06_REGISTERED_RBF_PROFILE_NAME,
    EXP06_REGISTERED_RBF_SETTINGS,
    RBF_OFFLINE_COVERAGE_PROFILE_NAME,
    apply_offline_coverage_profile,
    default_rbf_profile,
    offline_coverage_v1_profile,
)
from experiments.common.robot_lectdb_cache import robot_split_schedule_kind


def flag_was_supplied(argv: list[str], flag: str) -> bool:
    negated = f"--no-{flag[2:]}" if flag.startswith("--") else None
    return any(
        item == flag
        or item.startswith(f"{flag}=")
        or (negated is not None and (item == negated or item.startswith(f"{negated}=")))
        for item in argv
    )


def resolved_adaptive_target_depth(args: argparse.Namespace) -> int:
    value = int(getattr(args, "adaptive_target_depth", 0))
    return value if value > 0 else int(args.leaf_max_depth)


def resolved_adaptive_grid_target_depth(args: argparse.Namespace) -> int:
    value = int(getattr(args, "adaptive_grid_target_depth", 0))
    return value if value > 0 else resolved_adaptive_target_depth(args)


def apply_hipac_improved_leaf_sweep_profile(args: argparse.Namespace,
                                            argv: list[str] | None = None) -> None:
    """Apply the validated Exp.6 HiPaC leaf-sweep profile.

    TransitionPortal is intentionally excluded from paper-facing runners until
    it is separately validated.
    """
    if not bool(getattr(args, "hipac_improved_leaf_sweep", False)):
        return
    supplied = argv or []

    def max_if_implicit(attr: str, flag: str, value: int) -> None:
        if not flag_was_supplied(supplied, flag):
            setattr(args, attr, max(int(getattr(args, attr)), int(value)))

    def max_float_if_implicit(attr: str, flag: str, value: float) -> None:
        if not flag_was_supplied(supplied, flag):
            setattr(args, attr, max(float(getattr(args, attr)), float(value)))

    max_if_implicit("leaf_max_depth", "--leaf-max-depth", 20)
    max_if_implicit("adaptive_depth_min", "--adaptive-depth-min", 14)
    max_if_implicit("adaptive_depth_max", "--adaptive-depth-max", 20)
    max_if_implicit("rbf_max_depth", "--rbf-max-depth", 110)
    max_if_implicit("deep_ffb_depth", "--deep-ffb-depth", 110)
    max_if_implicit("connector_pave_depth", "--connector-pave-depth", 110)
    max_if_implicit("query_bridge_pave_depth", "--query-bridge-pave-depth", 110)
    max_if_implicit("query_endpoint_anchor_ffb_depth", "--query-endpoint-anchor-ffb-depth", 110)
    max_if_implicit("query_bridge_rrt_fixed_iters", "--query-bridge-rrt-fixed-iters", 10000)
    max_if_implicit("query_bridge_no_path_retry_attempts", "--query-bridge-no-path-retry-attempts", 32)
    if not flag_was_supplied(supplied, "--query-bridge-no-path-retry-stop-on-first-success"):
        args.query_bridge_no_path_retry_stop_on_first_success = True
    max_float_if_implicit("query_bridge_direct_max_length", "--query-bridge-direct-max-length", 15.0)
    if not bool(getattr(args, "hipac_transition_obb_portal", False)):
        args.hipac_promote_transition_slices = False


def effective_lect_split_schedule(args: argparse.Namespace,
                                  robot_name: str,
                                  argv: list[str] | None = None) -> str:
    """Use robot-specific LECT split schedule unless the CLI overrides it."""
    supplied = argv or []
    if flag_was_supplied(supplied, "--lect-split-schedule"):
        return str(args.lect_split_schedule)
    return robot_split_schedule_kind(str(robot_name))


def apply_exp06_robot_tuned_rbf_profile(args: argparse.Namespace,
                                        robot_name: str,
                                        difficulty: str,
                                        argv: list[str] | None = None) -> argparse.Namespace:
    """Apply the registered Exp.6 RBF profile unless the CLI overrides fields."""
    if not bool(getattr(args, "rbf_robot_tuned_profile", True)):
        return args
    tuned = copy.copy(args)
    supplied = argv or []
    robot = str(robot_name).lower()
    level = str(difficulty).lower()
    settings = EXP06_REGISTERED_RBF_SETTINGS.get((robot, level))
    if settings is None:
        return tuned

    def set_if_implicit(attr: str, flag: str, value: Any) -> None:
        if not flag_was_supplied(supplied, flag):
            setattr(tuned, attr, value)

    def set_leaf_cap(depth: int) -> None:
        set_if_implicit("leaf_max_depth", "--leaf-max-depth", int(depth))
        set_if_implicit("adaptive_depth_min", "--adaptive-depth-min", int(depth))
        set_if_implicit("adaptive_depth_max", "--adaptive-depth-max", int(depth))

    if "leaf_max_depth" in settings:
        set_leaf_cap(int(settings["leaf_max_depth"]))
    field_flags = {
        "rbf_max_depth": "--rbf-max-depth",
        "deep_max_boxes": "--deep-max-boxes",
        "deep_ffb_depth": "--deep-ffb-depth",
        "connector_pave_depth": "--connector-pave-depth",
        "query_bridge_pave_depth": "--query-bridge-pave-depth",
        "query_endpoint_anchor_ffb_depth": "--query-endpoint-anchor-ffb-depth",
        "ffb_start_depth": "--ffb-start-depth",
        "ffb_binary_probe_depth": "--ffb-binary-probe-depth",
        "query_bridge_ffb_start_depth": "--query-bridge-ffb-start-depth",
        "query_bridge_edge_cost_penalty": "--query-bridge-edge-cost-penalty",
        "connector_rrt_step_size": "--connector-rrt-step-size",
        "connector_rrt_goal_bias": "--connector-rrt-goal-bias",
        "connector_rrt_local_sampling_radius": "--connector-rrt-local-sampling-radius",
        "query_bridge_direct_sample_step": "--query-bridge-direct-sample-step",
        "query_bridge_direct_segment_after_rrt": "--query-bridge-direct-segment-after-rrt",
        "query_bridge_fast_direct_segment_after_rrt": "--query-bridge-fast-direct-segment-after-rrt",
        "query_bridge_fast_direct_random_shortcut_iters": "--query-bridge-fast-direct-random-shortcut-iters",
        "query_bridge_force_selected": "--query-bridge-force-selected",
        "query_endpoint_point_anchor": "--query-endpoint-point-anchor",
        "query_bridge_full_residual_overlay_when_connected": "--query-bridge-full-residual-overlay-when-connected",
        "query_bridge_accept_segment_fraction": "--query-bridge-accept-segment-fraction",
        "query_bridge_accept_path_ratio": "--query-bridge-accept-path-ratio",
        "query_bridge_accept_path_additive": "--query-bridge-accept-path-additive",
        "query_bridge_forced_attempts": "--query-bridge-forced-attempts",
        "query_bridge_attempt_offset": "--query-bridge-attempt-offset",
        "query_bridge_rrt_fixed_iters": "--query-bridge-rrt-fixed-iters",
        "query_bridge_local_radius_schedule": "--query-bridge-local-radius-schedule",
        "query_bridge_hybridize_attempt_paths": "--query-bridge-hybridize-attempt-paths",
        "query_bridge_hybrid_max_paths": "--query-bridge-hybrid-max-paths",
        "query_bridge_hybrid_max_vertices": "--query-bridge-hybrid-max-vertices",
        "query_bridge_hybrid_max_cross_checks": "--query-bridge-hybrid-max-cross-checks",
        "query_bridge_parallel_rrt_early_stop": "--query-bridge-parallel-rrt-early-stop",
        "query_bridge_parallel_rrt_early_stop_min_successes": "--query-bridge-parallel-rrt-early-stop-min-successes",
        "query_bridge_parallel_rrt_early_stop_ratio": "--query-bridge-parallel-rrt-early-stop-ratio",
        "query_bridge_parallel_rrt_early_stop_additive": "--query-bridge-parallel-rrt-early-stop-additive",
        "query_bridge_no_path_retry_attempts": "--query-bridge-no-path-retry-attempts",
        "query_bridge_no_path_retry_stop_on_first_success": "--query-bridge-no-path-retry-stop-on-first-success",
        "query_bridge_no_path_retry_budget_iters": "--query-bridge-no-path-retry-budget-iters",
        "query_bridge_no_path_retry_budget_attempts": "--query-bridge-no-path-retry-budget-attempts",
        "query_bridge_sequential_reuse": "--query-bridge-sequential-reuse",
        "query_bridge_scene_reusable_edges": "--query-bridge-scene-reusable-edges",
        "query_bridge_direct_max_length": "--query-bridge-direct-max-length",
        "query_bridge_to_main_island": "--query-bridge-to-main-island",
        "query_bridge_failure_fallback_to_main": "--query-bridge-failure-fallback-to-main",
        "hipac_improved_leaf_sweep": "--hipac-improved-leaf-sweep",
        "hipac_online_connectivity": "--hipac-online-connectivity",
        "hipac_online_prebridge_portal": "--hipac-online-prebridge-portal",
        "segment_edge_obb_cover": "--segment-edge-obb-cover",
        "rrt_bridge_obb_cover": "--rrt-bridge-obb-cover",
        "strict_obb_bridge_cover": "--strict-obb-bridge-cover",
        "segment_edge_obb_metadata_only": "--segment-edge-obb-metadata-only",
        "segment_edge_obb_metadata_require_cover": "--segment-edge-obb-metadata-require-cover",
        "segment_edge_obb_lateral_radius": "--segment-edge-obb-lateral-radius",
        "segment_edge_obb_grow_iterations": "--segment-edge-obb-grow-iterations",
        "segment_edge_obb_binary_iterations": "--segment-edge-obb-binary-iterations",
        "segment_edge_obb_split_depth": "--segment-edge-obb-split-depth",
        "obb_max_window_segments": "--obb-max-window-segments",
        "obb_max_validations_per_window": "--obb-max-validations-per-window",
        "obb_fast_primary_orientation": "--obb-fast-primary-orientation",
        "obb_fallback_orientations_on_primary_fail": "--obb-fallback-orientations-on-primary-fail",
    }
    for attr, flag in field_flags.items():
        if attr == "leaf_max_depth":
            continue
        if attr in settings:
            set_if_implicit(attr, flag, settings[attr])
    return tuned


def apply_exp06_rbf_profiles(args: argparse.Namespace,
                             robot_name: str,
                             difficulty: str) -> argparse.Namespace:
    """Apply generic coverage defaults, then more specific robot tuning."""
    tuned = copy.copy(args)
    apply_offline_coverage_profile(tuned, getattr(args, "_argv", []))
    return apply_exp06_robot_tuned_rbf_profile(
        tuned,
        robot_name,
        difficulty,
        getattr(args, "_argv", []),
    )


def normalize_adaptive_depth_cap(args: argparse.Namespace, argv: list[str]) -> None:
    """Keep implicit adaptive-depth scans inside the requested leaf target."""
    if not bool(getattr(args, "adaptive_depth_enabled", False)):
        return
    if flag_was_supplied(argv, "--adaptive-depth-max"):
        return
    requested_cap = max(int(args.leaf_max_depth), int(resolved_adaptive_target_depth(args)))
    old_max = int(args.adaptive_depth_max)
    if old_max <= requested_cap:
        return
    args.adaptive_depth_max = requested_cap
    if int(args.adaptive_depth_min) > requested_cap:
        args.adaptive_depth_min = requested_cap
    print(
        "[exp06] capped implicit adaptive-depth-max "
        f"from {old_max} to {requested_cap}; pass --adaptive-depth-max to override",
        file=sys.stderr,
    )


def effective_rbf_profile(args: argparse.Namespace,
                          box_budgets: list[int] | None = None,
                          *,
                          split_schedule_kind: str | None = None) -> dict[str, Any]:
    profile = copy.deepcopy(default_rbf_profile())
    effective_split = str(split_schedule_kind or args.lect_split_schedule)
    inherited_profile = str(profile.get("profile", "registered_exp4_profile"))
    if bool(getattr(args, "rbf_robot_tuned_profile", False)):
        profile["profile"] = EXP06_REGISTERED_RBF_PROFILE_NAME
        profile["registered_profile"] = EXP06_REGISTERED_RBF_PROFILE_NAME
    else:
        profile["profile"] = (
            f"exp06_leaf{int(args.leaf_max_depth)}"
            f"_ffb{int(args.deep_ffb_depth)}"
            f"_bridge{int(args.query_bridge_pave_depth)}"
            f"_ead{int(args.query_endpoint_anchor_ffb_depth)}"
        )
    profile["offline_query_agnostic_build"] = True
    profile["inherits_from"] = inherited_profile
    profile["override_reason"] = (
        "Exp.6 registered random-scene RBF profile."
        if bool(getattr(args, "rbf_robot_tuned_profile", False))
        else "Exp.6 controlled depth trade-off scan on saved random-scene catalog."
    )
    profile["robot_tuned_profile"] = bool(getattr(args, "rbf_robot_tuned_profile", False))
    profile["leaf_sweep"]["leaf_start_depth"] = int(args.leaf_start_depth)
    profile["leaf_sweep"]["leaf_max_depth"] = int(args.leaf_max_depth)
    profile["leaf_sweep"]["adaptive_target_depth"] = resolved_adaptive_target_depth(args)
    profile["leaf_sweep"]["adaptive_grid_target_depth"] = resolved_adaptive_grid_target_depth(args)
    profile["leaf_sweep"]["adaptive_depth"] = {
        "enabled": bool(args.adaptive_depth_enabled),
        "min": int(args.adaptive_depth_min),
        "max": int(args.adaptive_depth_max),
        "probe_count": int(args.adaptive_depth_probe_count),
        "anchor_probe_cap": int(args.adaptive_depth_anchor_probe_cap),
        "probe_seed": int(args.adaptive_depth_probe_seed),
        "min_free_probes": int(args.adaptive_depth_min_free_probes),
        "min_covered_probes": int(args.adaptive_depth_min_covered_probes),
        "min_main_probes": int(args.adaptive_depth_min_main_probes),
        "min_main_ratio": float(args.adaptive_depth_min_main_ratio),
        "min_cells": int(args.adaptive_depth_min_cells),
        "min_main_cells": int(args.adaptive_depth_min_main_cells),
        "max_online_cells": int(args.adaptive_depth_max_online_cells),
        "max_probe_ms": float(args.adaptive_depth_max_probe_ms),
    }
    profile["offline_anchors"] = {
        "enabled": bool(args.offline_random_anchors),
        "count": int(args.offline_anchor_count),
        "candidate_count": int(args.offline_anchor_candidate_count),
        "skip_if_main_accessible": bool(args.offline_anchor_skip_if_main_accessible),
        "skip_difficulties": str(args.offline_anchor_skip_difficulties),
        "main_accessible_threshold": float(args.offline_anchor_main_accessible_threshold),
        "policy": (
            "for selected difficulties, skip anchors after adaptive build when "
            "P(main-accessible) reaches threshold; otherwise keep random anchors"
        ),
    }
    profile["offline_coverage_profile"] = str(getattr(args, "offline_coverage_profile", ""))
    profile["offline_coverage_profile_details"] = (
        offline_coverage_v1_profile()
        if str(getattr(args, "offline_coverage_profile", "")) == RBF_OFFLINE_COVERAGE_PROFILE_NAME
        else {}
    )
    profile["offline_connector"] = {
        "mode": str(getattr(args, "offline_connector_mode", "box_only")),
        "shortcut_edges": int(args.offline_shortcut_edges),
        "candidate_limit": int(args.offline_shortcut_candidate_limit),
        "min_gain_ratio": float(args.offline_shortcut_min_gain_ratio),
        "max_segment_length": float(args.offline_shortcut_max_segment_length),
    }
    profile["leaf_sweep"]["leaf_threads"] = int(args.threads)
    profile["deep_refine"]["deep_max_boxes"] = int(args.deep_max_boxes)
    profile["deep_refine"]["deep_ffb_depth"] = int(args.deep_ffb_depth)
    profile["deep_refine"]["ffb_start_depth"] = int(args.ffb_start_depth)
    profile["deep_refine"]["ffb_binary_probe_depth"] = int(args.ffb_binary_probe_depth)
    profile["deep_refine"]["ffb_search_mode"] = str(args.ffb_search_mode)
    profile["deep_refine"]["ffb_implementation"] = DEFAULT_RBF_FFB_IMPLEMENTATION
    profile["deep_refine"]["split_schedule_kind"] = effective_split
    profile["leaf_sweep"]["split_schedule_kind"] = effective_split
    profile["query_bridge"]["split_schedule_kind"] = effective_split
    profile["robot_overrides"] = {
        "ur5_ffb_start_depth": int(args.ur5_ffb_start_depth),
        "policy": "value >= 0 overrides global ffb_start_depth for all UR5 RBF stages",
    }
    profile["connector"]["pave_depth"] = int(args.connector_pave_depth)
    profile["connector"]["ffb_search_mode"] = str(args.ffb_search_mode)
    profile["connector"]["ffb_binary_probe_depth"] = int(args.ffb_binary_probe_depth)
    profile["connector"]["ffb_implementation"] = DEFAULT_RBF_FFB_IMPLEMENTATION
    profile["connector"]["max_pairs_per_gap"] = int(DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP)
    profile["connector"]["per_pair_timeout_ms"] = int(DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS)
    profile["connector"]["rrt_step_size"] = float(args.connector_rrt_step_size)
    profile["connector"]["rrt_goal_bias"] = float(args.connector_rrt_goal_bias)
    profile["connector"]["rrt_local_sampling_radius"] = float(args.connector_rrt_local_sampling_radius)
    profile["query_bridge"]["pave_depth"] = int(args.query_bridge_pave_depth)
    profile["query_bridge"]["ffb_start_depth"] = int(args.query_bridge_ffb_start_depth)
    profile["query_bridge"]["ffb_search_mode"] = str(args.ffb_search_mode)
    profile["query_bridge"]["ffb_binary_probe_depth"] = int(args.ffb_binary_probe_depth)
    profile["query_bridge"]["ffb_implementation"] = DEFAULT_RBF_FFB_IMPLEMENTATION
    profile["query_bridge"]["endpoint_anchor_ffb_depth"] = int(args.query_endpoint_anchor_ffb_depth)
    profile["query_bridge"]["all_queries"] = bool(args.query_bridge_all)
    profile["query_bridge"]["adaptive_all"] = bool(args.query_bridge_adaptive_all)
    profile["query_bridge"]["adaptive_max_path_length"] = float(args.query_bridge_adaptive_max_path_length)
    profile["query_bridge"]["direct_sample_step"] = float(args.query_bridge_direct_sample_step)
    profile["query_bridge"]["direct_max_length"] = float(args.query_bridge_direct_max_length)
    profile["query_bridge"]["full_residual_overlay_when_connected"] = bool(
        args.query_bridge_full_residual_overlay_when_connected
    )
    profile["query_bridge"]["sequential_reuse"] = bool(args.query_bridge_sequential_reuse)
    profile["query_bridge"]["scene_reusable_edges"] = bool(args.query_bridge_scene_reusable_edges)
    profile["query_bridge"]["reuse_scope"] = "scene_seed_local"
    profile["query_bridge"]["box_transition_line_deviation_penalty"] = float(args.box_transition_line_deviation_penalty)
    profile["query_bridge"]["foreign_edge_cost_penalty"] = float(args.query_foreign_edge_cost_penalty)
    profile["query_bridge"]["query_bridge_edge_cost_penalty"] = float(args.query_bridge_edge_cost_penalty)
    profile["query_bridge"]["force_selected"] = bool(args.query_bridge_force_selected)
    profile["query_bridge"]["accept_segment_fraction"] = float(args.query_bridge_accept_segment_fraction)
    profile["query_bridge"]["accept_path_ratio"] = float(args.query_bridge_accept_path_ratio)
    profile["query_bridge"]["accept_path_additive"] = float(args.query_bridge_accept_path_additive)
    profile["query_bridge"]["forced_attempts"] = int(args.query_bridge_forced_attempts)
    profile["query_bridge"]["attempt_offset"] = int(args.query_bridge_attempt_offset)
    profile["query_bridge"]["rrt_fixed_iters"] = int(args.query_bridge_rrt_fixed_iters)
    profile["query_bridge"]["local_radius_schedule"] = str(args.query_bridge_local_radius_schedule)
    profile["query_bridge"]["hybridize_attempt_paths"] = bool(args.query_bridge_hybridize_attempt_paths)
    profile["query_bridge"]["hybrid_max_paths"] = int(args.query_bridge_hybrid_max_paths)
    profile["query_bridge"]["hybrid_max_vertices"] = int(args.query_bridge_hybrid_max_vertices)
    profile["query_bridge"]["hybrid_max_cross_checks"] = int(args.query_bridge_hybrid_max_cross_checks)
    profile["query_bridge"]["parallel_rrt_early_stop"] = bool(args.query_bridge_parallel_rrt_early_stop)
    profile["query_bridge"]["parallel_rrt_early_stop_min_successes"] = int(
        args.query_bridge_parallel_rrt_early_stop_min_successes
    )
    profile["query_bridge"]["parallel_rrt_early_stop_ratio"] = float(
        args.query_bridge_parallel_rrt_early_stop_ratio
    )
    profile["query_bridge"]["parallel_rrt_early_stop_additive"] = float(
        args.query_bridge_parallel_rrt_early_stop_additive
    )
    profile["query_bridge"]["direct_segment_after_rrt"] = bool(args.query_bridge_direct_segment_after_rrt)
    profile["query_bridge"]["fast_direct_segment_after_rrt"] = bool(
        args.query_bridge_fast_direct_segment_after_rrt
    )
    profile["query_bridge"]["fast_direct_random_shortcut_iters"] = int(
        args.query_bridge_fast_direct_random_shortcut_iters
    )
    profile["query_bridge"]["endpoint_point_anchor"] = bool(args.query_endpoint_point_anchor)
    profile["query_bridge"]["no_path_retry_attempts"] = int(args.query_bridge_no_path_retry_attempts)
    profile["query_bridge"]["no_path_retry_stop_on_first_success"] = bool(
        args.query_bridge_no_path_retry_stop_on_first_success
    )
    profile["query_bridge"]["no_path_retry_budget_iters"] = str(
        getattr(args, "query_bridge_no_path_retry_budget_iters", "")
    ).strip()
    profile["query_bridge"]["no_path_retry_budget_attempts"] = str(
        getattr(args, "query_bridge_no_path_retry_budget_attempts", "")
    ).strip()
    profile["query_bridge"]["to_main_island"] = bool(args.query_bridge_to_main_island)
    profile["query_bridge"]["to_main_direct_segment_max_length"] = float(
        args.query_bridge_to_main_direct_segment_max_length
    )
    profile["query_bridge"]["failure_fallback_to_main"] = bool(args.query_bridge_failure_fallback_to_main)
    profile["query_bridge"]["endpoint_anchor_before_bridge"] = bool(args.query_endpoint_anchor_before_bridge)
    profile["hipac"] = {
        "improved_leaf_sweep": bool(args.hipac_improved_leaf_sweep),
        "portal_connectivity": bool(args.hipac_portal_connectivity),
        "portal_cell_native_validate": bool(args.hipac_portal_cell_native_validate),
        "online_connectivity": bool(args.hipac_online_connectivity),
        "online_prebridge_portal": bool(args.hipac_online_prebridge_portal),
        "promote_transition_slices": bool(args.hipac_promote_transition_slices),
    }
    profile["obb"] = {
        "segment_edge_cover": bool(args.segment_edge_obb_cover),
        "rrt_bridge_cover": bool(args.rrt_bridge_obb_cover),
        "strict_bridge_cover": bool(args.strict_obb_bridge_cover),
        "lateral_radius": float(args.segment_edge_obb_lateral_radius),
        "longitudinal_margin": float(args.segment_edge_obb_longitudinal_margin),
        "safety_epsilon": float(args.segment_edge_obb_safety_epsilon),
        "grow_iterations": int(args.segment_edge_obb_grow_iterations),
        "binary_iterations": int(args.segment_edge_obb_binary_iterations),
        "split_depth": int(args.segment_edge_obb_split_depth),
        "max_window_segments": int(args.obb_max_window_segments),
        "max_validations_per_window": int(args.obb_max_validations_per_window),
        "fast_primary_orientation": bool(args.obb_fast_primary_orientation),
        "fallback_orientations_on_primary_fail": bool(args.obb_fallback_orientations_on_primary_fail),
        "sampled_support_enabled": False,
        "clearance_sampled_support_enabled": True,
        "clearance_lateral_l1_max": 5e-3,
        "clearance_samples": 17,
        "clearance_dense_line_l1_threshold": 0.03,
        "clearance_dense_samples": 17,
        "clearance_fast_samples": 0,
        "clearance_first": False,
        "clearance_retry_attempts": 0,
        "clearance_retry_values": [],
        "clearance_retry_iters": -1,
        "clearance_retry_timeout_ms": -1.0,
    }
    profile["query"]["final_rrt_simplify_timeout_ms"] = 1000.0 * float(args.ompl_simplify_time_s)
    profile["query"]["final_rrt_simplify_time_s"] = float(args.ompl_simplify_time_s)
    if box_budgets is not None:
        profile["box_budget_grid"] = [int(value) for value in box_budgets]
    return profile
