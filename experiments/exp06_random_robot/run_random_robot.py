#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import re
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (
    DEFAULT_OUTPUT_ROOT,
    configure_thread_environment,
    csv_list,
    environment_metadata,
    run_id,
    write_json,
)
from experiments.common.metrics import mean, median, tex_num
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import DEFAULT_QUERIES_PER_SCENE, generate_catalog, make_robot, queries_for_key, scene_for_key
from experiments.common.rbf_defaults import (
    DEFAULT_OMPL_SIMPLIFY_TIME_S,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY,
    DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
    DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
    DEFAULT_RBF_CONNECTOR_PAVE_DEPTH,
    DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS,
    DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE,
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_DEEP_MAX_BOXES,
    DEFAULT_RBF_FFB_IMPLEMENTATION,
    DEFAULT_RBF_FFB_BINARY_PROBE_DEPTH,
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
    DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_MAX_DEPTH,
    DEFAULT_RBF_OFFLINE_ANCHOR_CANDIDATE_COUNT,
    DEFAULT_RBF_OFFLINE_ANCHOR_COUNT,
    DEFAULT_RBF_OFFLINE_ANCHOR_DISTANCE_MU,
    DEFAULT_RBF_OFFLINE_ANCHOR_LCA_LAMBDA,
    DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY,
    DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY,
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP,
    DEFAULT_RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED,
    DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET,
    DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS,
    DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS,
    DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES,
    DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO,
    DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT,
    DEFAULT_RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS,
    DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
    DEFAULT_RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES,
    DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS,
    DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS,
    DEFAULT_RBF_THREADS,
    ROBOT_LECTDB_CACHE_ROOT,
    EXP06_REGISTERED_RBF_PROFILE_NAME,
    EXP06_REGISTERED_RBF_SETTINGS,
    RBF_OFFLINE_COVERAGE_PROFILE_NAME,
    apply_offline_coverage_profile,
    default_rbf_profile,
    offline_coverage_v1_profile,
    rbf_budget_grid,
    robot_lectdb_profile,
    robot_joint_limit_tuples,
)
from experiments.common.rbf_leaf_rrt import QuerySpec, RBFLeafRRTOptions, run_leaf_rrt
from experiments.common.robot_lectdb_cache import (
    ensure_robot_lectdb_cache,
    robot_external_evidence_path,
    robot_split_schedule_kind,
)
from experiments.common.sbf_import import import_sbf


METHODS = ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
REGISTERED_DISTRIBUTION_CATALOG = (
    DEFAULT_OUTPUT_ROOT
    / "tro2026"
    / "exp06"
    / "distribution_q10x10_three_robot_strict_catalog.json"
)
sbf = import_sbf()


def catalog_seed_count_for_selection(path: Path, robots: list[str], difficulties: list[str]) -> int | None:
    if not Path(path).exists():
        return None
    with Path(path).open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    records = payload.get("records", [])
    selected = {(str(robot), str(difficulty)): set() for robot in robots for difficulty in difficulties}
    for record in records:
        key = (str(record.get("robot")), str(record.get("difficulty")))
        if key in selected:
            selected[key].add(int(record.get("scene_seed", -1)))
    if not selected or any(not seeds for seeds in selected.values()):
        return None
    counts: list[int] = []
    for seeds in selected.values():
        ordered = sorted(seeds)
        contiguous = 0
        for value in ordered:
            if value != contiguous:
                break
            contiguous += 1
        counts.append(contiguous)
    if not counts or min(counts) <= 0:
        return None
    return int(min(counts))


def slugify(value: Any, *, limit: int = 96) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")
    if not text:
        text = "row"
    return text[:limit]


def planned_row_fingerprint(row: dict[str, Any]) -> str:
    relevant = {
        "method": row.get("method"),
        "robot": row.get("robot"),
        "difficulty": row.get("difficulty"),
        "scene_seed": row.get("scene_seed"),
        "stage_id": row.get("stage_id"),
        "budget_s": row.get("budget_s"),
        "deep_max_boxes": row.get("deep_max_boxes"),
        "queries_per_scene": row.get("queries_per_scene"),
        "audit_segment_step": row.get("audit_segment_step"),
        "audit_collision_tolerance": row.get("audit_collision_tolerance"),
        "ompl_simplify_time_s": row.get("ompl_simplify_time_s"),
        "prm_config": row.get("prm_config"),
        "bitstar_config": row.get("bitstar_config"),
        "rbf_default_profile": row.get("rbf_default_profile"),
    }
    encoded = json.dumps(relevant, sort_keys=True, default=str, separators=(",", ":"))
    return hashlib.sha1(encoded.encode("utf-8")).hexdigest()[:16]


def planned_row_part_path(checkpoint_dir: Path, row: dict[str, Any]) -> Path:
    stem = "_".join([
        slugify(row.get("method", "method"), limit=24),
        slugify(row.get("robot", "robot"), limit=16),
        slugify(row.get("difficulty", "difficulty"), limit=16),
        f"s{int(row.get('scene_seed', 0))}",
        slugify(row.get("stage_id", "stage"), limit=72),
        planned_row_fingerprint(row),
    ])
    return checkpoint_dir / f"{stem}.json"


def load_result_part(path: Path) -> list[dict[str, Any]] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if not isinstance(payload, dict) or payload.get("status") != "complete":
        return None
    rows = payload.get("rows")
    if not isinstance(rows, list):
        return None
    return [dict(row) for row in rows if isinstance(row, dict)]


def write_result_part(path: Path, planned_row: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    payload = {
        "status": "complete",
        "fingerprint": planned_row_fingerprint(planned_row),
        "planned_row": planned_row,
        "row_count": len(rows),
        "rows": rows,
        "written_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str), encoding="utf-8")
    tmp.replace(path)


_BITSTAR_REFERENCE_SUCCESS_LABELS: dict[str, set[str]] = {}


def bitstar_reference_success_labels(args: argparse.Namespace) -> set[str]:
    path_value = getattr(args, "bitstar_supplement_failed_from_manifest", None)
    if path_value is None:
        return set()
    path = Path(path_value)
    key = str(path)
    if key in _BITSTAR_REFERENCE_SUCCESS_LABELS:
        return _BITSTAR_REFERENCE_SUCCESS_LABELS[key]
    labels: set[str] = set()
    if path.exists():
        payload = json.loads(path.read_text(encoding="utf-8"))
        for row in payload.get("rows", []):
            if not isinstance(row, dict) or str(row.get("method")) != "bitstar":
                continue
            for query in row.get("queries", []):
                if isinstance(query, dict) and bool(query.get("success")):
                    label = str(query.get("label", ""))
                    if label:
                        labels.add(label)
    _BITSTAR_REFERENCE_SUCCESS_LABELS[key] = labels
    return labels


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
        if not _flag_was_supplied(supplied, flag):
            setattr(args, attr, max(int(getattr(args, attr)), int(value)))

    def max_float_if_implicit(attr: str, flag: str, value: float) -> None:
        if not _flag_was_supplied(supplied, flag):
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
    if not _flag_was_supplied(supplied, "--query-bridge-no-path-retry-stop-on-first-success"):
        args.query_bridge_no_path_retry_stop_on_first_success = True
    max_float_if_implicit("query_bridge_direct_max_length", "--query-bridge-direct-max-length", 15.0)
    if not bool(getattr(args, "hipac_transition_obb_portal", False)):
        args.hipac_promote_transition_slices = False


def _flag_was_supplied(argv: list[str], flag: str) -> bool:
    negated = f"--no-{flag[2:]}" if flag.startswith("--") else None
    return any(
        item == flag
        or item.startswith(f"{flag}=")
        or (negated is not None and (item == negated or item.startswith(f"{negated}=")))
        for item in argv
    )


def effective_lect_split_schedule(args: argparse.Namespace,
                                  robot_name: str,
                                  argv: list[str] | None = None) -> str:
    """Use robot-specific LECT split schedule unless the CLI overrides it."""
    supplied = argv or []
    if _flag_was_supplied(supplied, "--lect-split-schedule"):
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
        if not _flag_was_supplied(supplied, flag):
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
    """Apply generic coverage defaults, then more specific robot tuning.

    The offline coverage profile is intentionally broad.  Exp.6 registered
    rows also carry robot/difficulty-specific online bridge settings, which
    must win over broad defaults when both profiles are active.
    """
    tuned = copy.copy(args)
    apply_offline_coverage_profile(tuned, getattr(args, "_argv", []))
    return apply_exp06_robot_tuned_rbf_profile(
        tuned,
        robot_name,
        difficulty,
        getattr(args, "_argv", []),
    )


def normalize_adaptive_depth_cap(args: argparse.Namespace, argv: list[str]) -> None:
    """Keep implicit adaptive-depth scans inside the requested leaf target.

    The Exp.6 fast profiles often set ``--leaf-max-depth`` /
    ``--adaptive-target-depth`` to a shallow value and leave
    ``--adaptive-depth-max`` unspecified.  In that case the runner must not
    silently expand the offline build to the parser default depth cap; doing so
    changes both build time and coverage while the stage id still looks like a
    shallow profile.  Users can still request a deeper adaptive scan by passing
    ``--adaptive-depth-max`` explicitly.
    """
    if not bool(getattr(args, "adaptive_depth_enabled", False)):
        return
    if _flag_was_supplied(argv, "--adaptive-depth-max"):
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.6 saved-catalog random multi-robot study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--force-rerun", action="store_true")
    parser.add_argument("--checkpoint-dir", type=Path, default=None)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=10)
    parser.add_argument(
        "--scene-seed-list",
        default="",
        help="Comma-separated explicit scene seeds for diagnostics. Empty keeps the standard contiguous 0..N-1 set.",
    )
    parser.add_argument("--allow-fewer-catalog-scenes", action="store_true")
    parser.add_argument(
        "--scene-profile",
        choices=[
            "bitstar_gated", "bitstar_gated_independent",
            "balanced", "balanced_independent", "balanced_probe",
            "timed_probe", "timed_probe_independent",
            "narrow_passage", "narrow_passage_independent",
            "direct_blocker",
        ],
        default="timed_probe_independent",
    )
    parser.add_argument("--max-scene-tries", type=int, default=64)
    parser.add_argument("--scene-catalog", type=Path, default=REGISTERED_DISTRIBUTION_CATALOG)
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="reuse")
    parser.add_argument("--queries-per-scene", type=int, default=DEFAULT_QUERIES_PER_SCENE)
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--methods", default="sbf_leaf_rrt")
    parser.add_argument(
        "--rbf-robot-tuned-profile",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Use the validated Exp.6 per-robot RBF profile when a setting is not "
            "explicitly supplied.  This keeps IIWA on the Exp.4-derived profile "
            "and applies UR5/Panda-specific depth and QueryBridge cost settings."
        ),
    )
    parser.add_argument("--deep-max-boxes", type=int, default=DEFAULT_RBF_DEEP_MAX_BOXES)
    parser.add_argument("--box-budgets", default="")
    parser.add_argument("--rbf-max-depth", type=int, default=DEFAULT_RBF_MAX_DEPTH)
    parser.add_argument("--offline-grower", choices=["leaf_refine", "adaptive_deep_leaf"], default="adaptive_deep_leaf")
    parser.add_argument(
        "--offline-coverage-profile",
        choices=["", RBF_OFFLINE_COVERAGE_PROFILE_NAME],
        default="",
        help="Apply a named query-agnostic offline coverage profile after robot-specific RBF defaults.",
    )
    parser.add_argument("--leaf-start-depth", type=int, default=DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--leaf-max-depth", type=int, default=14)
    parser.add_argument("--adaptive-target-depth", type=int, default=0)
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
    parser.add_argument("--adaptive-depth-max", type=int, default=18)
    parser.add_argument("--adaptive-depth-probe-count", type=int, default=2048)
    parser.add_argument("--adaptive-depth-anchor-probe-cap", type=int, default=128)
    parser.add_argument("--adaptive-depth-probe-seed", type=int, default=20260607)
    parser.add_argument("--adaptive-depth-min-free-probes", type=int, default=64)
    parser.add_argument("--adaptive-depth-min-covered-probes", type=int, default=32)
    parser.add_argument("--adaptive-depth-min-main-probes", type=int, default=24)
    parser.add_argument("--adaptive-depth-min-main-ratio", type=float, default=0.40)
    parser.add_argument("--adaptive-depth-min-cells", type=int, default=0)
    parser.add_argument("--adaptive-depth-min-main-cells", type=int, default=0)
    parser.add_argument("--adaptive-depth-max-online-cells", type=int, default=1500)
    parser.add_argument("--adaptive-depth-max-probe-ms", type=float, default=20.0)
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
    parser.add_argument("--connector-pave-depth", type=int, default=DEFAULT_RBF_CONNECTOR_PAVE_DEPTH)
    parser.add_argument("--query-bridge-pave-depth", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH)
    parser.add_argument("--query-endpoint-anchor-ffb-depth", type=int, default=0)
    parser.add_argument("--ffb-start-depth", type=int, default=DEFAULT_RBF_FFB_START_DEPTH)
    parser.add_argument("--ur5-ffb-start-depth", type=int, default=-1)
    parser.add_argument("--query-bridge-ffb-start-depth", type=int, default=-1)
    parser.add_argument("--ffb-binary-probe-depth", type=int, default=DEFAULT_RBF_FFB_BINARY_PROBE_DEPTH)
    parser.add_argument("--ffb-search-mode", default=DEFAULT_RBF_FFB_SEARCH_MODE)
    parser.add_argument(
        "--lect-split-schedule",
        default="aafk_volume_min",
        choices=[
            "aafk_volume_min",
            "aafk",
            "support_hull_volume_min",
            "support-hull-volume-min",
            "support_hull",
        ],
    )
    parser.add_argument("--connector-segment-resolution", type=int, default=None)
    parser.add_argument("--query-bridge-all", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-adaptive-all", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-adaptive-max-path-length", type=float, default=4.5)
    parser.add_argument("--query-bridge-accept-segment-fraction", type=float, default=0.25)
    parser.add_argument("--query-bridge-accept-path-ratio", type=float, default=1.50)
    parser.add_argument("--query-bridge-accept-path-additive", type=float, default=0.75)
    parser.add_argument("--query-bridge-direct-sample-step", type=float, default=DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP)
    parser.add_argument("--query-bridge-adaptive-max-repair-calls", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS)
    parser.add_argument("--query-bridge-adaptive-fine-step", type=float, default=DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP)
    parser.add_argument("--query-bridge-direct-max-length", type=float, default=6.5)
    parser.add_argument(
        "--query-bridge-full-residual-overlay-when-connected",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_FULL_RESIDUAL_OVERLAY_WHEN_CONNECTED,
    )
    parser.add_argument(
        "--box-transition-line-deviation-penalty",
        type=float,
        default=DEFAULT_RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY,
    )
    parser.add_argument(
        "--query-foreign-edge-cost-penalty",
        type=float,
        default=DEFAULT_RBF_QUERY_FOREIGN_EDGE_COST_PENALTY,
    )
    parser.add_argument("--query-bridge-edge-cost-penalty", type=float, default=DEFAULT_RBF_QUERY_BRIDGE_EDGE_COST_PENALTY)
    parser.add_argument("--connector-rrt-step-size", type=float, default=DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE)
    parser.add_argument("--connector-rrt-goal-bias", type=float, default=DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS)
    parser.add_argument("--connector-rrt-local-sampling-radius", type=float, default=0.0)
    parser.add_argument("--query-bridge-force-selected", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-sequential-reuse", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-scene-reusable-edges", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-bridge-forced-attempts", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_FORCED_ATTEMPTS)
    parser.add_argument("--query-bridge-attempt-offset", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_ATTEMPT_OFFSET)
    parser.add_argument("--query-bridge-rrt-fixed-iters", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_RRT_FIXED_ITERS)
    parser.add_argument("--query-bridge-local-radius-schedule", default=DEFAULT_RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE)
    parser.add_argument(
        "--query-bridge-parallel-rrt-early-stop",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP,
    )
    parser.add_argument(
        "--query-bridge-parallel-rrt-early-stop-min-successes",
        type=int,
        default=DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES,
    )
    parser.add_argument(
        "--query-bridge-parallel-rrt-early-stop-ratio",
        type=float,
        default=DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO,
    )
    parser.add_argument(
        "--query-bridge-parallel-rrt-early-stop-additive",
        type=float,
        default=DEFAULT_RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE,
    )
    parser.add_argument(
        "--query-bridge-direct-segment-after-rrt",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_DIRECT_SEGMENT_AFTER_RRT,
    )
    parser.add_argument(
        "--query-bridge-fast-direct-segment-after-rrt",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--query-bridge-fast-direct-random-shortcut-iters",
        type=int,
        default=DEFAULT_RBF_QUERY_BRIDGE_FAST_DIRECT_RANDOM_SHORTCUT_ITERS,
    )
    parser.add_argument(
        "--query-bridge-hybridize-attempt-paths",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS,
    )
    parser.add_argument(
        "--query-bridge-hybrid-max-paths",
        type=int,
        default=DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS,
    )
    parser.add_argument(
        "--query-bridge-hybrid-max-vertices",
        type=int,
        default=DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES,
    )
    parser.add_argument(
        "--query-bridge-hybrid-max-cross-checks",
        type=int,
        default=DEFAULT_RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS,
    )
    parser.add_argument(
        "--query-endpoint-point-anchor",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--query-bridge-no-path-retry-attempts", type=int, default=DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS)
    parser.add_argument("--query-bridge-no-path-retry-stop-on-first-success",
                        action=argparse.BooleanOptionalAction,
                        default=DEFAULT_RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS)
    parser.add_argument("--query-bridge-no-path-retry-budget-iters", default="")
    parser.add_argument("--query-bridge-no-path-retry-budget-attempts", default="")
    parser.add_argument("--query-bridge-to-main-island", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--query-bridge-failure-fallback-to-main", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--query-bridge-to-main-direct-segment-max-length", type=float, default=0.0)
    parser.add_argument("--query-endpoint-anchor-before-bridge", action=argparse.BooleanOptionalAction, default=False)
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
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--offline-random-anchors", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--offline-anchor-count", type=int, default=DEFAULT_RBF_OFFLINE_ANCHOR_COUNT)
    parser.add_argument("--offline-anchor-candidate-count", type=int, default=DEFAULT_RBF_OFFLINE_ANCHOR_CANDIDATE_COUNT)
    parser.add_argument("--offline-anchor-lca-lambda", type=float, default=DEFAULT_RBF_OFFLINE_ANCHOR_LCA_LAMBDA)
    parser.add_argument("--offline-anchor-distance-mu", type=float, default=DEFAULT_RBF_OFFLINE_ANCHOR_DISTANCE_MU)
    parser.add_argument("--offline-anchor-skip-if-main-accessible", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--offline-anchor-skip-difficulties", default="")
    parser.add_argument("--offline-anchor-main-accessible-threshold", type=float, default=0.95)
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
    parser.add_argument("--hipac-online-candidate-max-length", type=float, default=3.0)
    parser.add_argument("--hipac-online-max-resolves-per-query", type=int, default=1)
    parser.add_argument("--hipac-online-max-hidden-boxes-per-portal", type=int, default=32)
    parser.add_argument("--hipac-online-max-ffb-calls-per-portal", type=int, default=64)
    parser.add_argument("--hipac-online-prebridge-portal", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-online-prebridge-candidate-limit", type=int, default=32)
    parser.add_argument("--hipac-online-prebridge-max-pair-distance", type=float, default=1.25)
    parser.add_argument("--hipac-online-prebridge-route-distance-weight", type=float, default=1.0)
    parser.add_argument("--hipac-online-prebridge-pair-distance-weight", type=float, default=0.25)
    parser.add_argument("--hipac-transition-obb-portal", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-transition-obb-lateral-radius", type=float, default=0.01)
    parser.add_argument("--hipac-transition-obb-longitudinal-margin", type=float, default=0.0)
    parser.add_argument("--hipac-transition-obb-safety-epsilon", type=float, default=0.0)
    parser.add_argument("--segment-edge-obb-cover", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rrt-bridge-obb-cover", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--strict-obb-bridge-cover", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--segment-edge-obb-metadata-only", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--segment-edge-obb-metadata-require-cover", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--segment-edge-obb-lateral-radius", type=float, default=0.01)
    parser.add_argument("--segment-edge-obb-longitudinal-margin", type=float, default=0.0)
    parser.add_argument("--segment-edge-obb-safety-epsilon", type=float, default=0.0)
    parser.add_argument("--segment-edge-obb-grow-iterations", type=int, default=5)
    parser.add_argument("--segment-edge-obb-binary-iterations", type=int, default=5)
    parser.add_argument("--segment-edge-obb-split-depth", type=int, default=1)
    parser.add_argument("--obb-max-window-segments", type=int, default=16)
    parser.add_argument("--obb-max-validations-per-window", type=int, default=96)
    parser.add_argument("--obb-fast-primary-orientation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--obb-fallback-orientations-on-primary-fail", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-promote-transition-slices", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hipac-promote-transition-target-query-indices", default="")
    parser.add_argument("--hipac-promote-transition-min-boxes", type=int, default=8)
    parser.add_argument("--hipac-promote-transition-max-boxes", type=int, default=64)
    parser.add_argument("--hipac-promote-transition-max-attempts-per-query", type=int, default=1)
    parser.add_argument("--lect-cache-root", type=Path, default=ROBOT_LECTDB_CACHE_ROOT)
    parser.add_argument("--skip-lect-cache-ensure", action="store_true")
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--rrt-timeout-s", type=float, default=1.0)
    parser.add_argument("--rrt-range", type=float, default=0.35)
    parser.add_argument("--ompl-simplify-time-s", type=float, default=DEFAULT_OMPL_SIMPLIFY_TIME_S)
    parser.add_argument("--prm-build-s", type=float, default=20.0)
    parser.add_argument(
        "--prm-build-grid-s",
        default="",
        help=(
            "Explicit PRM cumulative build checkpoints. Empty uses "
            "--prm-build-schedule; kept for reproducing old sparse grids."
        ),
    )
    parser.add_argument(
        "--prm-build-schedule",
        choices=["progressive", "uniform", "explicit"],
        default="progressive",
        help=(
            "Cumulative PRM build checkpoint schedule. 'progressive' uses dense "
            "early checkpoints and relaxes to at most --prm-build-max-step-s; "
            "'uniform' uses --prm-build-interval-s; 'explicit' requires "
            "--prm-build-grid-s."
        ),
    )
    parser.add_argument("--prm-build-interval-s", type=float, default=0.1)
    parser.add_argument("--prm-build-max-step-s", type=float, default=0.1)
    parser.add_argument("--prm-query-s", type=float, default=1.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-planner-kind", default="prm")
    parser.add_argument("--prm-cumulative", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prm-preload-query-endpoints", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prm-early-stop-success-stall-checkpoints", type=int, default=12)
    parser.add_argument("--prm-early-stop-path-rel-tol", type=float, default=0.005)
    parser.add_argument("--prm-unresolved-query-retry-interval-s", type=float, default=0.05)
    parser.add_argument("--prm-solved-query-recheck-interval-s", type=float, default=0.1)
    parser.add_argument("--bitstar-timeout-s", type=float, default=0.5)
    parser.add_argument("--bitstar-timeout-grid-s", default="")
    parser.add_argument("--bitstar-checkpoint-grid-s", default="")
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=0.005)
    parser.add_argument(
        "--bitstar-checkpoint-schedule",
        choices=["progressive", "uniform", "explicit"],
        default="progressive",
        help=(
            "Checkpoint stage schedule for BIT* traces. 'progressive' uses dense early "
            "checkpoints and gradually relaxes to at most --bitstar-checkpoint-max-step-s; "
            "'uniform' uses --bitstar-checkpoint-interval-s; 'explicit' requires "
            "--bitstar-checkpoint-grid-s."
        ),
    )
    parser.add_argument("--bitstar-checkpoint-max-step-s", type=float, default=0.1)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=5.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--bitstar-quality-stall-checkpoints", type=int, default=0)
    parser.add_argument("--bitstar-quality-stall-rel-tol", type=float, default=0.005)
    parser.add_argument("--bitstar-supplement-failed-from-manifest", type=Path, default=None)
    return parser.parse_args()


def progressive_checkpoint_grid(timeout_s: float, *, max_step_s: float = 0.1) -> list[float]:
    timeout = max(0.0, float(timeout_s))
    if timeout <= 0.0:
        return []
    # Dense first-solution region, then relaxed checkpoints. The final segment
    # is capped by max_step_s so per-query trade-off selection is never forced
    # to wait for a coarse one-second checkpoint.
    segments = [
        (0.10, 0.005),
        (0.50, 0.010),
        (1.00, 0.020),
        (2.00, 0.050),
        (timeout, max(1e-9, min(float(max_step_s), 0.100))),
    ]
    values: list[float] = []
    current = 0.0
    for end_s, step_s in segments:
        end = min(timeout, float(end_s))
        step = max(1e-9, float(step_s))
        while current + step < end - 1e-9:
            current += step
            values.append(round(current, 9))
        if end > current + 1e-9:
            current = end
            values.append(round(current, 9))
        if current >= timeout - 1e-9:
            break
    if not values or abs(values[-1] - timeout) > 1e-9:
        values.append(round(timeout, 9))
    return sorted({float(value) for value in values if value > 0.0 and value <= timeout + 1e-9})


def progressive_bitstar_checkpoint_grid(timeout_s: float, *, max_step_s: float = 0.1) -> list[float]:
    return progressive_checkpoint_grid(timeout_s, max_step_s=max_step_s)


def prm_build_grid_from_args(args: argparse.Namespace) -> list[float]:
    build_s = max(0.0, float(getattr(args, "prm_build_s", 0.0)))
    raw_grid = str(getattr(args, "prm_build_grid_s", "")).strip()
    schedule = str(getattr(args, "prm_build_schedule", "progressive"))
    if build_s <= 0.0:
        return []
    if not raw_grid and schedule == "explicit":
        raise ValueError("--prm-build-schedule=explicit requires --prm-build-grid-s")
    if raw_grid:
        values = sorted({float(value) for value in csv_list(raw_grid) if float(value) > 0.0})
        values = [min(build_s, value) for value in values if value <= build_s + 1e-9]
        if not values or abs(values[-1] - build_s) > 1e-9:
            values.append(build_s)
        return values
    if schedule == "progressive":
        return progressive_checkpoint_grid(
            build_s,
            max_step_s=float(getattr(args, "prm_build_max_step_s", 0.1)),
        )
    interval_s = max(float(getattr(args, "prm_build_interval_s", 0.1)), 1e-9)
    values: list[float] = []
    target_s = interval_s
    while target_s < build_s - 1e-9:
        values.append(float(target_s))
        target_s += interval_s
    values.append(build_s)
    return values


def bitstar_checkpoint_grid_from_args(args: argparse.Namespace, timeout_s: float) -> list[float]:
    raw_grid = str(getattr(args, "bitstar_checkpoint_grid_s", "")).strip()
    schedule = str(getattr(args, "bitstar_checkpoint_schedule", "progressive"))
    if not raw_grid and schedule == "explicit":
        raise ValueError("--bitstar-checkpoint-schedule=explicit requires --bitstar-checkpoint-grid-s")
    if raw_grid:
        values = sorted({float(value) for value in csv_list(raw_grid) if float(value) > 0.0})
        values = [min(float(timeout_s), value) for value in values if value <= float(timeout_s) + 1e-9]
        if not values or abs(values[-1] - float(timeout_s)) > 1e-9:
            values.append(float(timeout_s))
        return values
    if schedule == "progressive":
        return progressive_bitstar_checkpoint_grid(
            float(timeout_s),
            max_step_s=float(getattr(args, "bitstar_checkpoint_max_step_s", 0.1)),
        )
    interval_s = max(float(args.bitstar_checkpoint_interval_s), 1e-9)
    values: list[float] = []
    target_s = interval_s
    while target_s < float(timeout_s) - 1e-9:
        values.append(float(target_s))
        target_s += interval_s
    values.append(float(timeout_s))
    return values


def bitstar_trace_interval_for_grid(args: argparse.Namespace, checkpoint_grid_s: list[float], timeout_s: float) -> float:
    deltas = [
        float(b) - float(a)
        for a, b in zip([0.0] + checkpoint_grid_s[:-1], checkpoint_grid_s)
        if float(b) - float(a) > 1e-9
    ]
    if deltas:
        return max(1e-9, min(deltas))
    return max(1e-9, min(float(args.bitstar_checkpoint_interval_s), float(timeout_s)))


def checkpoint_at_or_after(checkpoints: list[dict[str, Any]], target_s: float) -> dict[str, Any]:
    if not checkpoints:
        return {}
    for checkpoint in checkpoints:
        if float(checkpoint.get("checkpoint_s", 0.0) or 0.0) >= float(target_s) - 1e-9:
            return checkpoint
    return checkpoints[-1]


def fmt_float(value: float) -> str:
    return f"{float(value):g}".replace("-", "m").replace(".", "p")


def path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return math.nan
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
    return total


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def audit_path(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    *,
    start: list[float] | None = None,
    goal: list[float] | None = None,
    endpoint_tol: float = 1e-6,
    collision_tolerance: float = DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
) -> tuple[bool, float, str]:
    t0 = time.perf_counter()
    if len(path) < 2:
        return False, time.perf_counter() - t0, "empty_path"
    if start is not None and point_distance(path[0], list(start)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "start_mismatch"
    if goal is not None and point_distance(path[-1], list(goal)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "goal_mismatch"
    step = max(1e-9, float(segment_step))
    for a, b in zip(path, path[1:]):
        distance = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
        steps = max(1, int(math.ceil(distance / step)))
        for index in range(steps + 1):
            if sbf.check_config_collision(
                robot,
                obstacles,
                interpolate(a, b, index / steps),
                float(collision_tolerance),
            ):
                return False, time.perf_counter() - t0, "collision"
    return True, time.perf_counter() - t0, "passed"


def simplify_path_if_requested(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    simplify_time_s: float,
) -> tuple[list[list[float]], float, str]:
    if len(path) < 2 or simplify_time_s <= 0.0:
        return path, 0.0, "skipped"
    result = sbf.ompl_simplify_path(
        robot,
        obstacles,
        path,
        float(segment_step),
        float(simplify_time_s),
    )
    if bool(result.get("ok")):
        return [[float(value) for value in point] for point in result.get("path", [])], float(result.get("t_s", 0.0)), str(result.get("reason", "simplified"))
    return path, float(result.get("t_s", 0.0) or 0.0), str(result.get("reason", "simplify_failed"))


def run_rbf_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    args = apply_exp06_rbf_profiles(args, robot_name, difficulty)
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    queries = [
        QuerySpec(
            label=f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            start=[float(value) for value in query["start"]],
            goal=[float(value) for value in query["goal"]],
            actual_start=[float(value) for value in query["start"]],
            actual_goal=[float(value) for value in query["goal"]],
        )
        for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed))
    ]
    valid_root = robot_joint_limit_tuples(robot)
    # Canonical roots are internal to LECT.  The active database root is left
    # unset so SBF uses the canonical primary sector when appropriate; the
    # experiment/root sampling space remains the native joint-limit coverage.
    root_override = None
    anchor_skip_difficulties = {
        item.strip().lower()
        for item in str(args.offline_anchor_skip_difficulties).split(",")
        if item.strip()
    }
    scene_anchor_skip_if_main_accessible = (
        bool(args.offline_anchor_skip_if_main_accessible)
        and str(difficulty).lower() in anchor_skip_difficulties
    )
    hipac_improved = bool(args.hipac_improved_leaf_sweep)
    hipac_portal_connectivity = bool(args.hipac_portal_connectivity) or hipac_improved
    hipac_online_connectivity = bool(args.hipac_online_connectivity) or hipac_improved
    hipac_online_prebridge_portal = bool(args.hipac_online_prebridge_portal) or hipac_improved
    hipac_promote_transition_slices = (
        bool(args.hipac_promote_transition_slices) and bool(args.hipac_transition_obb_portal)
    )
    effective_ffb_start_depth = int(args.ffb_start_depth)
    if robot_name == "ur5" and int(args.ur5_ffb_start_depth) >= 0:
        effective_ffb_start_depth = int(args.ur5_ffb_start_depth)
    effective_split_schedule_kind = effective_lect_split_schedule(
        args,
        robot_name,
        getattr(args, "_argv", []),
    )
    hipac_profile_tag = (
        f"_hp{int(hipac_improved)}"
        f"_hpre{int(hipac_online_prebridge_portal)}"
        f"_askip{int(scene_anchor_skip_if_main_accessible)}"
        f"_ath{fmt_float(float(args.offline_anchor_main_accessible_threshold))}"
    )
    offline_profile_tag = ""
    if str(args.offline_coverage_profile):
        offline_profile_tag = (
            f"_oc{slugify(str(args.offline_coverage_profile), limit=18)}"
            f"_cm{slugify(str(args.offline_connector_mode), limit=12)}"
            f"_at{int(resolved_adaptive_target_depth(args))}"
            f"_tb{int(float(args.adaptive_time_budget_ms))}"
        )
    query_quality_tag = (
        f"_qbp{fmt_float(float(args.query_bridge_edge_cost_penalty))}"
        f"_rs{fmt_float(float(args.connector_rrt_step_size))}"
        f"_gb{fmt_float(float(args.connector_rrt_goal_bias))}"
        f"_lr{fmt_float(float(args.connector_rrt_local_sampling_radius))}"
        f"_fa{int(args.query_bridge_forced_attempts)}"
    )
    budget_iters_tag = str(getattr(args, "query_bridge_no_path_retry_budget_iters", "")).strip()
    budget_attempts_tag = str(getattr(args, "query_bridge_no_path_retry_budget_attempts", "")).strip()
    if budget_iters_tag or budget_attempts_tag:
        query_quality_tag += (
            f"_nbi{slugify(budget_iters_tag or '0', limit=10)}"
            f"_nba{slugify(budget_attempts_tag or '0', limit=10)}"
        )
    stage_id = (
        f"l{int(args.leaf_max_depth)}"
        f"_ffb{int(args.deep_ffb_depth)}"
        f"_fs{int(effective_ffb_start_depth)}"
        f"_sp{str(effective_split_schedule_kind).replace('-', '_')}"
        f"_ead{int(args.query_endpoint_anchor_ffb_depth)}"
        f"_b{int(args.deep_max_boxes)}"
        f"_a{int(args.offline_anchor_count)}"
        f"_c{int(args.offline_anchor_candidate_count)}"
        f"_os{int(args.offline_shortcut_edges)}"
        f"_tm{int(bool(args.query_bridge_to_main_island))}"
        f"{offline_profile_tag}"
        f"{hipac_profile_tag}"
        f"{query_quality_tag}"
    )
    active_cache_name = (
        f"rbf_{robot_name}_{difficulty}_{int(scene_seed)}"
        f"_b{int(args.deep_max_boxes)}"
        f"_l{int(args.leaf_max_depth)}"
        f"_ffb{int(args.deep_ffb_depth)}"
        f"_fs{int(effective_ffb_start_depth)}"
        f"_sp{str(effective_split_schedule_kind).replace('-', '_')}"
        f"_ead{int(args.query_endpoint_anchor_ffb_depth)}"
        f"_a{int(args.offline_anchor_count)}"
        f"_c{int(args.offline_anchor_candidate_count)}"
        f"_os{int(args.offline_shortcut_edges)}"
        f"_tm{int(bool(args.query_bridge_to_main_island))}"
        f"{offline_profile_tag}"
        f"{hipac_profile_tag}"
        f"{query_quality_tag}"
    )
    row = run_leaf_rrt(
        robot=robot,
        obstacles=list(scene.obstacles),
        queries=queries,
        database_path=args.out_dir / "active_cache" / active_cache_name,
        options=RBFLeafRRTOptions(
            seed=int(scene_seed),
            offline_grower=str(args.offline_grower),
            deep_max_boxes=int(args.deep_max_boxes),
            rbf_max_depth=int(args.rbf_max_depth),
            threads=int(args.threads),
            leaf_start_depth=int(args.leaf_start_depth),
            leaf_max_depth=int(args.leaf_max_depth),
            adaptive_target_depth=resolved_adaptive_target_depth(args),
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
            adaptive_max_free_boxes=int(args.adaptive_max_free_boxes),
            adaptive_max_unresolved_domains=int(args.adaptive_max_unresolved_domains),
            adaptive_grid_target_depth=resolved_adaptive_grid_target_depth(args),
            deep_ffb_depth=int(args.deep_ffb_depth),
            connector_pave_depth=int(args.connector_pave_depth),
            query_bridge_pave_depth=int(args.query_bridge_pave_depth),
            query_bridge_ffb_start_depth=int(args.query_bridge_ffb_start_depth),
            query_endpoint_anchor_ffb_depth=int(args.query_endpoint_anchor_ffb_depth),
            ffb_start_depth=int(effective_ffb_start_depth),
            ffb_binary_probe_depth=int(args.ffb_binary_probe_depth),
            ffb_search_mode=str(args.ffb_search_mode),
            split_schedule_kind=str(effective_split_schedule_kind),
            use_external_evidence=True,
            external_evidence_path=robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root)),
            external_evidence_verify_identity=False,
            root_override_tuples=root_override,
            coverage_override_tuples=valid_root,
            symmetry_aligned_native_root=False,
            symmetry_aligned_cache_schedule=False,
            database_canonical_mode=True,
            case_label=f"rbf_{robot_name}_{difficulty}",
            parallel_virtual_validation=True,
            leaf_threads=int(args.threads),
            canonicalize_queries=False,
            audit_collision_tolerance=float(args.audit_collision_tolerance),
            offline_coverage_profile=str(args.offline_coverage_profile),
            offline_query_agnostic_build=True,
            offline_random_anchors=bool(args.offline_random_anchors),
            offline_anchor_count=int(args.offline_anchor_count),
            offline_anchor_candidate_count=int(args.offline_anchor_candidate_count),
            offline_anchor_lca_lambda=float(args.offline_anchor_lca_lambda),
            offline_anchor_distance_mu=float(args.offline_anchor_distance_mu),
            offline_anchor_skip_if_main_accessible=bool(scene_anchor_skip_if_main_accessible),
            offline_anchor_main_accessible_threshold=float(args.offline_anchor_main_accessible_threshold),
            offline_connector_mode=str(args.offline_connector_mode),
            offline_shortcut_edges=int(args.offline_shortcut_edges),
            offline_shortcut_candidate_limit=int(args.offline_shortcut_candidate_limit),
            offline_shortcut_min_gain_ratio=float(args.offline_shortcut_min_gain_ratio),
            offline_shortcut_max_segment_length=float(args.offline_shortcut_max_segment_length),
            hipac_portal_connectivity=hipac_portal_connectivity,
            hipac_portal_cell_native_validate=bool(args.hipac_portal_cell_native_validate),
            hipac_portal_max_internal_boxes=int(args.hipac_portal_max_internal_boxes),
            hipac_portal_max_recursion_depth=int(args.hipac_portal_max_recursion_depth),
            hipac_portal_ffb_depth=int(args.hipac_portal_ffb_depth),
            hipac_portal_ffb_deadline_ms=float(args.hipac_portal_ffb_deadline_ms),
            hipac_online_connectivity=hipac_online_connectivity,
            hipac_online_before_query_bridge=bool(args.hipac_online_before_query_bridge),
            hipac_promote_query_repairs=bool(args.hipac_promote_query_repairs),
            hipac_online_candidate_max_length=float(args.hipac_online_candidate_max_length),
            hipac_online_max_resolves_per_query=int(args.hipac_online_max_resolves_per_query),
            hipac_online_max_hidden_boxes_per_portal=int(args.hipac_online_max_hidden_boxes_per_portal),
            hipac_online_max_ffb_calls_per_portal=int(args.hipac_online_max_ffb_calls_per_portal),
            hipac_online_prebridge_portal=hipac_online_prebridge_portal,
            hipac_online_prebridge_candidate_limit=int(args.hipac_online_prebridge_candidate_limit),
            hipac_online_prebridge_max_pair_distance=float(args.hipac_online_prebridge_max_pair_distance),
            hipac_online_prebridge_route_distance_weight=float(args.hipac_online_prebridge_route_distance_weight),
            hipac_online_prebridge_pair_distance_weight=float(args.hipac_online_prebridge_pair_distance_weight),
            hipac_transition_obb_portal=bool(args.hipac_transition_obb_portal),
            hipac_transition_obb_lateral_radius=float(args.hipac_transition_obb_lateral_radius),
            hipac_transition_obb_longitudinal_margin=float(args.hipac_transition_obb_longitudinal_margin),
            hipac_transition_obb_safety_epsilon=float(args.hipac_transition_obb_safety_epsilon),
            segment_edge_obb_cover=bool(args.segment_edge_obb_cover),
            rrt_bridge_obb_cover=bool(args.rrt_bridge_obb_cover),
            strict_obb_bridge_cover=bool(args.strict_obb_bridge_cover),
            segment_edge_obb_metadata_only=bool(args.segment_edge_obb_metadata_only),
            segment_edge_obb_metadata_require_cover=bool(args.segment_edge_obb_metadata_require_cover),
            segment_edge_obb_lateral_radius=float(args.segment_edge_obb_lateral_radius),
            segment_edge_obb_longitudinal_margin=float(args.segment_edge_obb_longitudinal_margin),
            segment_edge_obb_safety_epsilon=float(args.segment_edge_obb_safety_epsilon),
            segment_edge_obb_grow_iterations=int(args.segment_edge_obb_grow_iterations),
            segment_edge_obb_binary_iterations=int(args.segment_edge_obb_binary_iterations),
            segment_edge_obb_split_depth=int(args.segment_edge_obb_split_depth),
            obb_max_window_segments=int(args.obb_max_window_segments),
            obb_max_validations_per_window=int(args.obb_max_validations_per_window),
            obb_fast_primary_orientation=bool(args.obb_fast_primary_orientation),
            obb_fallback_orientations_on_primary_fail=bool(args.obb_fallback_orientations_on_primary_fail),
            hipac_promote_transition_slices=hipac_promote_transition_slices,
            hipac_promote_transition_target_query_indices=str(args.hipac_promote_transition_target_query_indices),
            hipac_promote_transition_min_boxes=int(args.hipac_promote_transition_min_boxes),
            hipac_promote_transition_max_boxes=int(args.hipac_promote_transition_max_boxes),
            hipac_promote_transition_max_attempts_per_query=int(args.hipac_promote_transition_max_attempts_per_query),
            connector_pair_timeout_ms=DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS,
            connector_max_pairs_per_gap=DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP,
            connector_rrt_step_size=float(args.connector_rrt_step_size),
            connector_rrt_goal_bias=float(args.connector_rrt_goal_bias),
            connector_rrt_local_sampling_radius=float(args.connector_rrt_local_sampling_radius),
            query_bridge_all=bool(args.query_bridge_all),
            query_bridge_adaptive_all=bool(args.query_bridge_adaptive_all),
            query_bridge_adaptive_max_path_length=float(args.query_bridge_adaptive_max_path_length),
            query_bridge_accept_segment_fraction=float(args.query_bridge_accept_segment_fraction),
            query_bridge_accept_path_ratio=float(args.query_bridge_accept_path_ratio),
            query_bridge_accept_path_additive=float(args.query_bridge_accept_path_additive),
            query_bridge_direct_sample_step=float(args.query_bridge_direct_sample_step),
            query_endpoint_anchor_before_bridge=bool(args.query_endpoint_anchor_before_bridge),
            query_bridge_adaptive_max_repair_calls=int(args.query_bridge_adaptive_max_repair_calls),
            query_bridge_adaptive_fine_step=float(args.query_bridge_adaptive_fine_step),
            query_bridge_direct_max_length=float(args.query_bridge_direct_max_length),
            query_bridge_sequential_reuse=bool(args.query_bridge_sequential_reuse),
            query_bridge_scene_reusable_edges=bool(args.query_bridge_scene_reusable_edges),
            query_bridge_force_selected=bool(args.query_bridge_force_selected),
            query_bridge_forced_attempts=int(args.query_bridge_forced_attempts),
            query_bridge_attempt_offset=int(args.query_bridge_attempt_offset),
            query_bridge_rrt_fixed_iters=int(args.query_bridge_rrt_fixed_iters),
            query_bridge_local_radius_schedule=str(args.query_bridge_local_radius_schedule),
            query_bridge_parallel_rrt_early_stop=bool(args.query_bridge_parallel_rrt_early_stop),
            query_bridge_parallel_rrt_early_stop_min_successes=int(
                args.query_bridge_parallel_rrt_early_stop_min_successes
            ),
            query_bridge_parallel_rrt_early_stop_ratio=float(
                args.query_bridge_parallel_rrt_early_stop_ratio
            ),
            query_bridge_parallel_rrt_early_stop_additive=float(
                args.query_bridge_parallel_rrt_early_stop_additive
            ),
            query_bridge_direct_segment_after_rrt=bool(args.query_bridge_direct_segment_after_rrt),
            query_bridge_fast_direct_segment_after_rrt=bool(
                args.query_bridge_fast_direct_segment_after_rrt
            ),
            query_bridge_fast_direct_random_shortcut_iters=int(
                args.query_bridge_fast_direct_random_shortcut_iters
            ),
            query_bridge_hybridize_attempt_paths=bool(args.query_bridge_hybridize_attempt_paths),
            query_bridge_hybrid_max_paths=int(args.query_bridge_hybrid_max_paths),
            query_bridge_hybrid_max_vertices=int(args.query_bridge_hybrid_max_vertices),
            query_bridge_hybrid_max_cross_checks=int(args.query_bridge_hybrid_max_cross_checks),
            query_endpoint_point_anchor=bool(args.query_endpoint_point_anchor),
            query_bridge_no_path_retry_attempts=int(args.query_bridge_no_path_retry_attempts),
            query_bridge_no_path_retry_stop_on_first_success=bool(
                args.query_bridge_no_path_retry_stop_on_first_success
            ),
            query_bridge_no_path_retry_budget_iters=str(
                getattr(args, "query_bridge_no_path_retry_budget_iters", "")
            ).strip(),
            query_bridge_no_path_retry_budget_attempts=str(
                getattr(args, "query_bridge_no_path_retry_budget_attempts", "")
            ).strip(),
            query_bridge_edge_cost_penalty=float(args.query_bridge_edge_cost_penalty),
            query_bridge_full_residual_overlay_when_connected=bool(
                args.query_bridge_full_residual_overlay_when_connected
            ),
            query_bridge_to_main_island=bool(args.query_bridge_to_main_island),
            query_bridge_failure_fallback_to_main=bool(args.query_bridge_failure_fallback_to_main),
            query_bridge_to_main_direct_segment_max_length=float(args.query_bridge_to_main_direct_segment_max_length),
            endpoint_main_target_k=int(args.endpoint_main_target_k),
            endpoint_main_coarse_step=float(args.endpoint_main_coarse_step),
            endpoint_main_fine_step=float(args.endpoint_main_fine_step),
            endpoint_main_max_ffb_calls=int(args.endpoint_main_max_ffb_calls),
            endpoint_main_max_boxes=int(args.endpoint_main_max_boxes),
            endpoint_main_residual_segment_max_length=float(args.endpoint_main_residual_segment_max_length),
            endpoint_main_lateral_offset=float(args.endpoint_main_lateral_offset),
            endpoint_main_lateral_rounds=int(args.endpoint_main_lateral_rounds),
            endpoint_main_face_epsilon=float(args.endpoint_main_face_epsilon),
            query_box_transition_line_deviation_penalty=float(args.box_transition_line_deviation_penalty),
            query_foreign_edge_cost_penalty=float(args.query_foreign_edge_cost_penalty),
            connector_segment_resolution=(
                int(args.connector_segment_resolution)
                if args.connector_segment_resolution is not None
                else RBFLeafRRTOptions().connector_segment_resolution
            ),
            final_rrt_simplify_timeout_ms=1000.0 * float(args.ompl_simplify_time_s),
            final_rrt_simplify_attempts=DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
        ),
    )
    row.update(
        {
            "method": "sbf_leaf_rrt",
            "robot": robot_name,
            "difficulty": difficulty,
            "scene_seed": int(scene_seed),
            "stage_id": stage_id,
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "adaptive_target_depth": resolved_adaptive_target_depth(args),
            "adaptive_grid_target_depth": resolved_adaptive_grid_target_depth(args),
            "deep_ffb_depth": int(args.deep_ffb_depth),
            "connector_pave_depth": int(args.connector_pave_depth),
            "query_bridge_pave_depth": int(args.query_bridge_pave_depth),
            "ffb_start_depth": int(effective_ffb_start_depth),
            "query_bridge_ffb_start_depth": int(args.query_bridge_ffb_start_depth),
            "ffb_binary_probe_depth": int(args.ffb_binary_probe_depth),
            "hipac_transition_obb_portal": bool(args.hipac_transition_obb_portal),
            "hipac_transition_obb_lateral_radius": float(args.hipac_transition_obb_lateral_radius),
            "hipac_transition_obb_longitudinal_margin": float(args.hipac_transition_obb_longitudinal_margin),
            "hipac_transition_obb_safety_epsilon": float(args.hipac_transition_obb_safety_epsilon),
            "segment_edge_obb_cover": bool(args.segment_edge_obb_cover),
            "rrt_bridge_obb_cover": bool(args.rrt_bridge_obb_cover),
            "strict_obb_bridge_cover": bool(args.strict_obb_bridge_cover),
            "segment_edge_obb_metadata_only": bool(args.segment_edge_obb_metadata_only),
            "segment_edge_obb_metadata_require_cover": bool(args.segment_edge_obb_metadata_require_cover),
            "segment_edge_obb_lateral_radius": float(args.segment_edge_obb_lateral_radius),
            "segment_edge_obb_longitudinal_margin": float(args.segment_edge_obb_longitudinal_margin),
            "segment_edge_obb_safety_epsilon": float(args.segment_edge_obb_safety_epsilon),
            "segment_edge_obb_grow_iterations": int(args.segment_edge_obb_grow_iterations),
            "segment_edge_obb_binary_iterations": int(args.segment_edge_obb_binary_iterations),
            "segment_edge_obb_split_depth": int(args.segment_edge_obb_split_depth),
            "obb_max_window_segments": int(args.obb_max_window_segments),
            "obb_max_validations_per_window": int(args.obb_max_validations_per_window),
            "obb_fast_primary_orientation": bool(args.obb_fast_primary_orientation),
            "obb_fallback_orientations_on_primary_fail": bool(args.obb_fallback_orientations_on_primary_fail),
            "query_endpoint_anchor_ffb_depth": int(args.query_endpoint_anchor_ffb_depth),
            "lect_split_schedule": str(effective_split_schedule_kind),
            "lect_split_schedule_explicit": _flag_was_supplied(
                getattr(args, "_argv", []),
                "--lect-split-schedule",
            ),
            "offline_anchor_count": int(args.offline_anchor_count),
            "offline_anchor_candidate_count": int(args.offline_anchor_candidate_count),
            "offline_anchor_skip_if_main_accessible": bool(scene_anchor_skip_if_main_accessible),
            "offline_anchor_skip_difficulties": str(args.offline_anchor_skip_difficulties),
            "offline_anchor_main_accessible_threshold": float(args.offline_anchor_main_accessible_threshold),
            "offline_coverage_profile": str(args.offline_coverage_profile),
            "offline_connector_mode": str(args.offline_connector_mode),
            "offline_shortcut_edges": int(args.offline_shortcut_edges),
            "offline_shortcut_candidate_limit": int(args.offline_shortcut_candidate_limit),
            "offline_shortcut_min_gain_ratio": float(args.offline_shortcut_min_gain_ratio),
            "offline_shortcut_max_segment_length": float(args.offline_shortcut_max_segment_length),
            "hipac_improved_leaf_sweep": hipac_improved,
            "hipac_portal_connectivity": hipac_portal_connectivity,
            "hipac_portal_cell_native_validate": bool(args.hipac_portal_cell_native_validate),
            "hipac_online_connectivity": hipac_online_connectivity,
            "hipac_online_prebridge_portal": hipac_online_prebridge_portal,
            "hipac_online_max_resolves_per_query": int(args.hipac_online_max_resolves_per_query),
            "hipac_online_max_hidden_boxes_per_portal": int(args.hipac_online_max_hidden_boxes_per_portal),
            "hipac_promote_transition_slices": hipac_promote_transition_slices,
            "query_bridge_to_main_island": bool(args.query_bridge_to_main_island),
            "query_bridge_failure_fallback_to_main": bool(args.query_bridge_failure_fallback_to_main),
            "query_bridge_to_main_direct_segment_max_length": float(args.query_bridge_to_main_direct_segment_max_length),
            "query_bridge_direct_max_length": float(args.query_bridge_direct_max_length),
            "box_transition_line_deviation_penalty": float(args.box_transition_line_deviation_penalty),
            "query_foreign_edge_cost_penalty": float(args.query_foreign_edge_cost_penalty),
            "query_bridge_sequential_reuse": bool(args.query_bridge_sequential_reuse),
            "query_bridge_scene_reusable_edges": bool(args.query_bridge_scene_reusable_edges),
            "query_bridge_reuse_scope": "scene_seed_local",
            "query_bridge_direct_sample_step": float(args.query_bridge_direct_sample_step),
            "query_bridge_full_residual_overlay_when_connected": bool(
                args.query_bridge_full_residual_overlay_when_connected
            ),
            "query_endpoint_anchor_before_bridge": bool(args.query_endpoint_anchor_before_bridge),
            "query_bridge_edge_cost_penalty": float(args.query_bridge_edge_cost_penalty),
            "connector_rrt_step_size": float(args.connector_rrt_step_size),
            "connector_rrt_goal_bias": float(args.connector_rrt_goal_bias),
            "connector_rrt_local_sampling_radius": float(args.connector_rrt_local_sampling_radius),
            "query_bridge_forced_attempts": int(args.query_bridge_forced_attempts),
            "query_bridge_attempt_offset": int(args.query_bridge_attempt_offset),
            "query_bridge_rrt_fixed_iters": int(args.query_bridge_rrt_fixed_iters),
            "query_bridge_local_radius_schedule": str(args.query_bridge_local_radius_schedule),
            "query_bridge_parallel_rrt_early_stop": bool(args.query_bridge_parallel_rrt_early_stop),
            "query_bridge_parallel_rrt_early_stop_min_successes": int(
                args.query_bridge_parallel_rrt_early_stop_min_successes
            ),
            "query_bridge_parallel_rrt_early_stop_ratio": float(
                args.query_bridge_parallel_rrt_early_stop_ratio
            ),
            "query_bridge_parallel_rrt_early_stop_additive": float(
                args.query_bridge_parallel_rrt_early_stop_additive
            ),
            "query_bridge_direct_segment_after_rrt": bool(args.query_bridge_direct_segment_after_rrt),
            "query_bridge_fast_direct_segment_after_rrt": bool(
                args.query_bridge_fast_direct_segment_after_rrt
            ),
            "query_bridge_fast_direct_random_shortcut_iters": int(
                args.query_bridge_fast_direct_random_shortcut_iters
            ),
            "query_bridge_hybridize_attempt_paths": bool(args.query_bridge_hybridize_attempt_paths),
            "query_bridge_hybrid_max_paths": int(args.query_bridge_hybrid_max_paths),
            "query_bridge_hybrid_max_vertices": int(args.query_bridge_hybrid_max_vertices),
            "query_bridge_hybrid_max_cross_checks": int(args.query_bridge_hybrid_max_cross_checks),
            "query_endpoint_point_anchor": bool(args.query_endpoint_point_anchor),
            "query_bridge_no_path_retry_attempts": int(args.query_bridge_no_path_retry_attempts),
            "query_bridge_no_path_retry_stop_on_first_success": bool(
                args.query_bridge_no_path_retry_stop_on_first_success
            ),
            "query_bridge_adaptive_fine_step": float(args.query_bridge_adaptive_fine_step),
            "query_bridge_no_path_retry_budget_iters": str(
                getattr(args, "query_bridge_no_path_retry_budget_iters", "")
            ).strip(),
            "query_bridge_no_path_retry_budget_attempts": str(
                getattr(args, "query_bridge_no_path_retry_budget_attempts", "")
            ).strip(),
            "ffb_start_depth": int(effective_ffb_start_depth),
            "ffb_binary_probe_depth": int(args.ffb_binary_probe_depth),
            "rbf_max_depth": int(args.rbf_max_depth),
            "rbf_robot_tuned_profile": bool(args.rbf_robot_tuned_profile),
            "deep_max_boxes": int(args.deep_max_boxes),
            "obstacle_count": len(scene.obstacles),
            "queries_per_scene": len(queries),
            "scene_catalog": str(args.scene_catalog or (args.out_dir / "random_scene_catalog_v6.json")),
            "lectdb": robot_lectdb_profile(robot_name),
            "active_planning_root": "full_robot_joint_limits",
            "coverage_root": "full_robot_joint_limits",
            "canonical_mapping_scope": "LECT_internal_only",
            "external_evidence_path": str(robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root))),
        }
    )
    return row


def summarize_single_query_method(
    method: str,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    scene: Any,
    planning_s: float,
    audit_s: float,
    ok: bool,
    audit_passed: bool,
    audit_status: str,
    length: float,
    path: list[list[float]],
    diagnostics: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
) -> dict[str, Any]:
    success = bool(ok) and bool(audit_passed)
    return {
        "method": method,
        "robot": robot_name,
        "difficulty": difficulty,
        "scene_seed": int(scene_seed),
        "deep_max_boxes": 0,
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        "obstacle_count": len(scene.obstacles),
        "status": "ok" if success else "failed_audit" if ok else "failed_planning",
        "success_count": 1 if success else 0,
        "query_count": 1,
        "planning_s": float(planning_s),
        "audit_s": float(audit_s),
        "path_length_mean": float(length) if success else math.nan,
        "raw_segment_fraction": 0.0 if success else math.nan,
        "final_boxes": math.nan,
        "queries": [
            {
                "label": f"{robot_name}_{difficulty}_{scene_seed}",
                "success": success,
                "audit_passed": bool(audit_passed),
                "audit_status": audit_status,
                "query_ms": float(planning_s) * 1000.0,
                "solve_ms": float(planning_s) * 1000.0,
                "simplify_ms": 0.0,
                "audit_ms": float(audit_s) * 1000.0,
                "path_length": float(length) if success else math.nan,
                "segment_fraction": 0.0 if success else math.nan,
                "waypoint_count": len(path),
            }
        ],
        "diagnostics": dict(diagnostics or {}),
    }


def summarize_query_batch_method(
    method: str,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    scene: Any,
    qrows: list[dict[str, Any]],
    *,
    offline_build_s: float = 0.0,
    online_batch_s: float | None = None,
    audit_s: float = 0.0,
    diagnostics: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
) -> dict[str, Any]:
    successes = [row for row in qrows if bool(row.get("audit_passed"))]
    query_count = max(1, len(qrows))
    online_s = (
        sum(float(row.get("query_ms", 0.0)) for row in qrows) / 1000.0
        if online_batch_s is None
        else float(online_batch_s)
    )
    online_solve_s = 0.0
    online_simplify_s = 0.0
    split_available = False
    for row in qrows:
        try:
            total_ms = float(row.get("query_ms", math.nan))
            solve_ms = float(row.get("solve_ms", math.nan))
            simplify_ms = float(row.get("simplify_ms", math.nan))
        except (TypeError, ValueError):
            continue
        if math.isfinite(solve_ms) or math.isfinite(simplify_ms):
            split_available = True
            if math.isfinite(solve_ms):
                online_solve_s += solve_ms / 1000.0
            elif math.isfinite(total_ms) and math.isfinite(simplify_ms):
                online_solve_s += max(0.0, total_ms - simplify_ms) / 1000.0
            if math.isfinite(simplify_ms):
                online_simplify_s += simplify_ms / 1000.0
    if not split_available:
        online_solve_s = online_s
        online_simplify_s = 0.0
    else:
        residual_s = online_s - online_solve_s - online_simplify_s
        if residual_s > 1e-9:
            online_solve_s += residual_s
    online_total_s = online_s
    online_s = online_solve_s
    online_per_query_s = online_solve_s / query_count
    online_total_per_query_s = online_total_s / query_count
    online_solve_per_query_s = online_solve_s / query_count
    online_simplify_per_query_s = online_simplify_s / query_count
    amortized = {
        f"amortized_s_k{k}": float(offline_build_s) / float(k) + online_per_query_s
        for k in (1, 5, 10, 20, 50)
    }
    return {
        "method": method,
        "robot": robot_name,
        "difficulty": difficulty,
        "scene_seed": int(scene_seed),
        "deep_max_boxes": 0,
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        "obstacle_count": len(scene.obstacles),
        "queries_per_scene": len(qrows),
        "status": "ok" if len(successes) == len(qrows) else "partial",
        "success_count": len(successes),
        "query_count": len(qrows),
        "planning_s": float(offline_build_s) + online_s,
        "planning_total_s": float(offline_build_s) + online_total_s,
        "build_s": float(offline_build_s),
        "offline_build_s": float(offline_build_s),
        "online_batch_s": online_s,
        "online_total_s": online_total_s,
        "online_total_batch_s": online_total_s,
        "online_solve_s": online_solve_s,
        "online_simplify_s": online_simplify_s,
        "online_per_query_s": online_per_query_s,
        "online_total_per_query_s": online_total_per_query_s,
        "online_solve_per_query_s": online_solve_per_query_s,
        "online_simplify_per_query_s": online_simplify_per_query_s,
        **amortized,
        "audit_s": float(audit_s),
        "path_length_mean": mean(row["path_length"] for row in successes),
        "raw_segment_fraction": 0.0 if successes else math.nan,
        "final_boxes": math.nan,
        "queries": qrows,
        "diagnostics": dict(diagnostics or {}),
    }


def run_rrtconnect_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    qrows: list[dict[str, Any]] = []
    audit_total_s = 0.0
    for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed)):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        result = sbf.ompl_rrt_connect_path(
            robot,
            list(scene.obstacles),
            start,
            goal,
            float(args.rrt_timeout_s) * 1000.0,
            float(args.rrt_range),
            float(args.audit_segment_step),
            float(args.ompl_simplify_time_s),
            int(args.seed_base) + int(scene_seed) * 1009 + index,
        )
        total_s = float(result.get("t_s", 0.0))
        solve_s = float(result.get("solve_s", max(0.0, total_s - float(result.get("simplify_s", 0.0)))))
        simplify_s = float(result.get("simplify_s", max(0.0, total_s - solve_s)))
        path = [[float(value) for value in point] for point in result.get("path", [])]
        audit_passed, audit_s, audit_status = audit_path(
            robot,
            list(scene.obstacles),
            path,
            float(args.audit_segment_step),
            start=start,
            goal=goal,
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_total_s += audit_s
        success = bool(result.get("ok")) and bool(audit_passed)
        qrows.append({
            "label": f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            "success": success,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_s * 1000.0,
            "path_length": path_length(path) if success else math.nan,
            "segment_fraction": 0.0 if success else math.nan,
            "waypoint_count": len(path),
        })
    return summarize_query_batch_method(
        "rrtconnect",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        qrows,
        audit_s=audit_total_s,
        diagnostics={
            "planner": "OMPL_RRTConnect",
            "timeout_s": float(args.rrt_timeout_s),
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
    )


def run_prm_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int, build_budget_s: float) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    query_records = queries_for_key(catalog, robot_name, difficulty, scene_seed)
    starts = [[float(value) for value in query["start"]] for query in query_records]
    goals = [[float(value) for value in query["goal"]] for query in query_records]
    result = sbf.ompl_prm_multiquery(
        robot,
        list(scene.obstacles),
        starts,
        goals,
        float(build_budget_s),
        float(args.prm_query_s),
        float(args.audit_segment_step),
        float(args.ompl_simplify_time_s),
        int(args.seed_base) + 10007 * int(scene_seed),
        int(args.prm_max_nearest_neighbors),
        str(args.prm_planner_kind),
        bool(args.prm_preload_query_endpoints),
        int(args.prm_early_stop_success_stall_checkpoints),
        float(args.prm_early_stop_path_rel_tol),
        float(args.prm_unresolved_query_retry_interval_s),
        float(args.prm_solved_query_recheck_interval_s),
    )
    qrows: list[dict[str, Any]] = []
    audit_total_s = 0.0
    for index, (query, qresult) in enumerate(zip(query_records, list(result.get("queries", [])), strict=False)):
        path = [[float(value) for value in point] for point in qresult.get("path", [])]
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        audit_passed, audit_s, audit_status = audit_path(
            robot,
            list(scene.obstacles),
            path,
            float(args.audit_segment_step),
            start=start,
            goal=goal,
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_total_s += audit_s
        success = bool(qresult.get("ok")) and bool(audit_passed)
        total_s = float(qresult.get("t_s", 0.0))
        solve_s = float(qresult.get("solve_s", max(0.0, total_s - float(qresult.get("simplify_s", 0.0)))))
        simplify_s = float(qresult.get("simplify_s", max(0.0, total_s - solve_s)))
        qrows.append({
            "label": f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            "success": success,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_s * 1000.0,
            "path_length": path_length(path) if success else math.nan,
            "segment_fraction": 0.0 if success else math.nan,
            "waypoint_count": len(path),
        })
    return summarize_query_batch_method(
        "prm",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        qrows,
        offline_build_s=float(result.get("build_s", 0.0)),
        audit_s=audit_total_s,
        diagnostics={
            "planner": "OMPL_PRM",
            "build_s": float(result.get("build_s", 0.0)),
            "query_s": sum(float(row.get("query_ms", 0.0)) for row in qrows) / 1000.0,
            "nodes": int(result.get("nodes", 0) or 0),
            "query_budget_s": float(args.prm_query_s),
            "max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
            "planner_kind": str(args.prm_planner_kind),
            "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
        stage_id=(
            f"{str(args.prm_planner_kind)}_build{float(build_budget_s):g}s"
            f"_k{int(args.prm_max_nearest_neighbors)}"
            f"_q{float(args.prm_query_s):g}s"
            f"_preload{int(bool(args.prm_preload_query_endpoints))}"
        ),
        budget_s=float(build_budget_s),
    )


def run_prm_scene_cumulative(
    args: argparse.Namespace,
    catalog: dict[str, Any],
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    build_checkpoints_s: list[float],
) -> list[dict[str, Any]]:
    checkpoints = sorted({float(value) for value in build_checkpoints_s if float(value) > 0.0})
    if not checkpoints:
        raise ValueError("PRM cumulative mode requires at least one positive build checkpoint")
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    query_records = queries_for_key(catalog, robot_name, difficulty, scene_seed)
    starts = [[float(value) for value in query["start"]] for query in query_records]
    goals = [[float(value) for value in query["goal"]] for query in query_records]
    result = sbf.ompl_prm_multiquery_cumulative(
        robot,
        list(scene.obstacles),
        starts,
        goals,
        checkpoints,
        float(args.prm_query_s),
        float(args.audit_segment_step),
        float(args.ompl_simplify_time_s),
        int(args.seed_base) + 10007 * int(scene_seed),
        int(args.prm_max_nearest_neighbors),
        str(args.prm_planner_kind),
        bool(args.prm_preload_query_endpoints),
        int(args.prm_early_stop_success_stall_checkpoints),
        float(args.prm_early_stop_path_rel_tol),
        float(args.prm_unresolved_query_retry_interval_s),
        float(args.prm_solved_query_recheck_interval_s),
    )
    incumbents: dict[str, dict[str, Any]] = {}
    rows: list[dict[str, Any]] = []
    for stage in result.get("stages", []):
        checkpoint_s = float(stage.get("checkpoint_s", 0.0))
        build_s = float(stage.get("build_s", checkpoint_s))
        qrows: list[dict[str, Any]] = []
        audit_total_s = 0.0
        qresult_items = list(stage.get("queries", []))
        for index, (query, qresult) in enumerate(zip(query_records, qresult_items, strict=False)):
            label = f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}"
            path = [[float(value) for value in point] for point in qresult.get("path", [])]
            start = [float(value) for value in query["start"]]
            goal = [float(value) for value in query["goal"]]
            audit_passed = False
            audit_s = 0.0
            audit_status = "not_attempted"
            raw_length = math.nan
            current = incumbents.get(label)
            if bool(qresult.get("ok")) and len(path) >= 2:
                candidate_length = path_length(path)
                should_audit = current is None or candidate_length <= float(current["path_length"]) + 1e-12
                if should_audit:
                    audit_passed, audit_s, audit_status = audit_path(
                        robot,
                        list(scene.obstacles),
                        path,
                        float(args.audit_segment_step),
                        start=start,
                        goal=goal,
                        collision_tolerance=float(args.audit_collision_tolerance),
                    )
                    audit_total_s += audit_s
                else:
                    audit_status = "skipped_not_better_than_audited_incumbent"
                if audit_passed:
                    raw_length = candidate_length
                    if current is None or raw_length <= float(current["path_length"]) + 1e-12:
                        incumbents[label] = {
                            "path_length": raw_length,
                            "waypoint_count": len(path),
                            "checkpoint_s": checkpoint_s,
                            "planner_status": str(qresult.get("status", qresult.get("reason", ""))),
                        }
            total_s = float(qresult.get("t_s", 0.0))
            solve_s = float(qresult.get("solve_s", max(0.0, total_s - float(qresult.get("simplify_s", 0.0)))))
            simplify_s = float(qresult.get("simplify_s", max(0.0, total_s - solve_s)))
            incumbent = incumbents.get(label)
            if incumbent is not None:
                qrows.append({
                    "label": label,
                    "success": True,
                    "audit_passed": True,
                    "audit_status": (
                        "current_audit_passed"
                        if math.isfinite(raw_length) and abs(raw_length - float(incumbent["path_length"])) <= 1e-12
                        else f"incumbent_from_{float(incumbent['checkpoint_s']):g}s"
                    ),
                    "query_ms": total_s * 1000.0,
                    "solve_ms": solve_s * 1000.0,
                    "simplify_ms": simplify_s * 1000.0,
                    "audit_ms": audit_s * 1000.0,
                    "path_length": float(incumbent["path_length"]),
                    "segment_fraction": 0.0,
                    "waypoint_count": int(incumbent["waypoint_count"]),
                    "planner_status": (
                        f"{str(qresult.get('status', qresult.get('reason', '')))}; "
                        f"incumbent_checkpoint={float(incumbent['checkpoint_s']):g}s"
                    ),
                })
            else:
                qrows.append({
                    "label": label,
                    "success": False,
                    "audit_passed": False,
                    "audit_status": audit_status,
                    "query_ms": total_s * 1000.0,
                    "solve_ms": solve_s * 1000.0,
                    "simplify_ms": simplify_s * 1000.0,
                    "audit_ms": audit_s * 1000.0,
                    "path_length": math.nan,
                    "segment_fraction": math.nan,
                    "waypoint_count": len(path),
                    "planner_status": str(qresult.get("status", qresult.get("reason", ""))),
                })
        rows.append(summarize_query_batch_method(
            "prm",
            robot_name,
            difficulty,
            scene_seed,
            scene,
            qrows,
            offline_build_s=build_s,
            audit_s=audit_total_s,
            diagnostics={
                "planner": "OMPL_PRM_cumulative",
                "cumulative_prm": True,
                "checkpoint_s": checkpoint_s,
                "build_checkpoints_s": checkpoints,
                "build_s": build_s,
                "nodes": int(stage.get("nodes", result.get("nodes", 0)) or 0),
                "query_budget_s": float(args.prm_query_s),
                "max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
                "planner_kind": str(args.prm_planner_kind),
                "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
                "early_stop_success_stall_checkpoints": int(args.prm_early_stop_success_stall_checkpoints),
                "early_stop_path_rel_tol": float(args.prm_early_stop_path_rel_tol),
                "unresolved_query_retry_interval_s": float(args.prm_unresolved_query_retry_interval_s),
                "solved_query_recheck_interval_s": float(args.prm_solved_query_recheck_interval_s),
                "simplify_time_s": float(args.ompl_simplify_time_s),
            },
            stage_id=(
                f"{str(args.prm_planner_kind)}_cumulative_build{checkpoint_s:g}s"
                f"_k{int(args.prm_max_nearest_neighbors)}"
                f"_q{fmt_float(float(args.prm_query_s))}s"
                f"_preload{int(bool(args.prm_preload_query_endpoints))}"
            ),
            budget_s=checkpoint_s,
        ))
    return rows


def run_bitstar_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int, timeout_s: float) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    qrows: list[dict[str, Any]] = []
    audit_total_s = 0.0
    for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed)):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        result = sbf.ompl_bitstar_path(
            robot,
            list(scene.obstacles),
            start,
            goal,
            float(timeout_s) * 1000.0,
            float(args.audit_segment_step),
            float(args.ompl_simplify_time_s),
            int(args.seed_base) + 20011 * int(scene_seed) + index,
            int(args.bitstar_samples_per_batch),
            float(args.bitstar_rewire_factor),
            bool(args.bitstar_stop_on_solution_improvement),
        )
        path = [[float(value) for value in point] for point in result.get("path", [])]
        audit_passed, audit_s, audit_status = audit_path(
            robot,
            list(scene.obstacles),
            path,
            float(args.audit_segment_step),
            start=start,
            goal=goal,
            collision_tolerance=float(args.audit_collision_tolerance),
        )
        audit_total_s += audit_s
        success = bool(result.get("ok")) and bool(audit_passed)
        total_s = float(result.get("t_s", 0.0))
        solve_s = float(result.get("solve_s", max(0.0, total_s - float(result.get("simplify_s", 0.0)))))
        simplify_s = float(result.get("simplify_s", max(0.0, total_s - solve_s)))
        qrows.append({
            "label": f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}",
            "success": success,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": total_s * 1000.0,
            "solve_ms": solve_s * 1000.0,
            "simplify_ms": simplify_s * 1000.0,
            "audit_ms": audit_s * 1000.0,
            "path_length": path_length(path) if success else math.nan,
            "segment_fraction": 0.0 if success else math.nan,
            "waypoint_count": len(path),
        })
    return summarize_query_batch_method(
        "bitstar",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        qrows,
        audit_s=audit_total_s,
        diagnostics={
            "planner": "OMPL_BITstar",
            "timeout_s": float(timeout_s),
            "samples_per_batch": int(args.bitstar_samples_per_batch),
            "rewire_factor": float(args.bitstar_rewire_factor),
            "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
            "simplify_time_s": float(args.ompl_simplify_time_s),
        },
        stage_id=(
            f"batch{int(args.bitstar_samples_per_batch)}"
            f"_rw{float(args.bitstar_rewire_factor):g}"
            f"_t{float(timeout_s):g}s"
        ),
        budget_s=float(timeout_s),
    )


def run_bitstar_scene_trace(args: argparse.Namespace,
                            catalog: dict[str, Any],
                            robot_name: str,
                            difficulty: str,
                            scene_seed: int,
                            timeout_s: float,
                            checkpoint_interval_s: float,
                            checkpoint_grid_s: list[float]) -> list[dict[str, Any]]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    query_traces: list[tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]] = []
    reference_success = bitstar_reference_success_labels(args)
    for index, query in enumerate(queries_for_key(catalog, robot_name, difficulty, scene_seed)):
        label = f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}"
        if reference_success and label in reference_success:
            continue
        result = sbf.ompl_bitstar_trace(
            robot,
            list(scene.obstacles),
            [float(value) for value in query["start"]],
            [float(value) for value in query["goal"]],
            float(timeout_s) * 1000.0,
            float(checkpoint_interval_s) * 1000.0,
            float(args.audit_segment_step),
            int(args.seed_base) + 20011 * int(scene_seed) + index,
            int(args.bitstar_samples_per_batch),
            float(args.bitstar_rewire_factor),
            bool(args.bitstar_stop_on_solution_improvement),
            int(args.bitstar_quality_stall_checkpoints),
            float(args.bitstar_quality_stall_rel_tol),
        )
        query_traces.append((
            dict(query),
            [dict(item) for item in result.get("checkpoints", [])],
            {
                "stopped_early": bool(result.get("stopped_early", False)),
                "early_stop_reason": str(result.get("early_stop_reason", "")),
                "trace_solve_s": float(result.get("solve_s", math.nan)),
            },
        ))
    rows: list[dict[str, Any]] = []
    best_by_query: dict[str, dict[str, Any]] = {}
    for target_checkpoint_s in checkpoint_grid_s:
        qrows: list[dict[str, Any]] = []
        audit_total_s = 0.0
        checkpoint_s = 0.0
        for index, (query, checkpoints, trace_meta) in enumerate(query_traces):
            checkpoint = checkpoint_at_or_after(checkpoints, target_checkpoint_s)
            checkpoint_s = max(checkpoint_s, float(checkpoint.get("checkpoint_s", target_checkpoint_s) or 0.0))
            elapsed_s = float(checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0)) or 0.0)
            solve_s = float(checkpoint.get("solve_s", checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0))) or 0.0)
            path = [[float(value) for value in point] for point in checkpoint.get("path", [])]
            path, simplify_s, simplify_status = simplify_path_if_requested(
                robot,
                list(scene.obstacles),
                path,
                float(args.audit_segment_step),
                float(args.ompl_simplify_time_s),
            )
            start = [float(value) for value in query["start"]]
            goal = [float(value) for value in query["goal"]]
            audit_passed, audit_s, audit_status = audit_path(
                robot,
                list(scene.obstacles),
                path,
                float(args.audit_segment_step),
                start=start,
                goal=goal,
                collision_tolerance=float(args.audit_collision_tolerance),
            )
            audit_total_s += audit_s
            ok = bool(checkpoint.get("ok")) and audit_passed
            label = f"{robot_name}_{difficulty}_{scene_seed}_{query.get('label', f'q{index}')}"
            length = path_length(path) if ok else math.nan
            current_row = {
                "label": label,
                "success": ok,
                "audit_passed": audit_passed,
                "audit_status": audit_status,
                "query_ms": (elapsed_s + simplify_s) * 1000.0,
                "solve_ms": solve_s * 1000.0,
                "simplify_ms": simplify_s * 1000.0,
                "audit_ms": audit_s * 1000.0,
                "path_length": length,
                "segment_fraction": 0.0 if ok else math.nan,
                "waypoint_count": len(path),
                "simplify_status": simplify_status,
                "incumbent_checkpoint_s": float(target_checkpoint_s) if ok else math.nan,
                "trace_stopped_early": bool(trace_meta.get("stopped_early", False)),
                "trace_early_stop_reason": str(trace_meta.get("early_stop_reason", "")),
                "trace_solve_ms": float(trace_meta.get("trace_solve_s", math.nan)) * 1000.0,
            }
            best = best_by_query.get(label)
            if ok and math.isfinite(length) and (best is None or length < float(best.get("path_length", math.inf))):
                best_by_query[label] = dict(current_row)
                best = best_by_query[label]
            if best is None:
                qrows.append(current_row)
            else:
                row = dict(best)
                row["query_ms"] = (elapsed_s + float(row.get("simplify_ms", 0.0)) / 1000.0) * 1000.0
                row["solve_ms"] = solve_s * 1000.0
                qrows.append(row)
        rows.append(summarize_query_batch_method(
            "bitstar",
            robot_name,
            difficulty,
            scene_seed,
            scene,
            qrows,
            audit_s=audit_total_s,
            diagnostics={
                "planner": "OMPL_BITstar_trace",
                "cumulative_bitstar": True,
                "timeout_s": float(timeout_s),
                "checkpoint_interval_s": float(checkpoint_interval_s),
                "checkpoint_grid_s": checkpoint_grid_s,
                "checkpoint_s": checkpoint_s,
                "target_checkpoint_s": float(target_checkpoint_s),
                "samples_per_batch": int(args.bitstar_samples_per_batch),
                "rewire_factor": float(args.bitstar_rewire_factor),
                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
                "quality_stall_checkpoints": int(args.bitstar_quality_stall_checkpoints),
                "quality_stall_rel_tol": float(args.bitstar_quality_stall_rel_tol),
                "simplify_time_s": float(args.ompl_simplify_time_s),
            },
            stage_id=(
                f"batch{int(args.bitstar_samples_per_batch)}"
                f"_rw{float(args.bitstar_rewire_factor):g}"
                f"_trace{float(timeout_s):g}s_t{target_checkpoint_s:g}s"
            ),
            budget_s=float(target_checkpoint_s),
        ))
    return rows


def run_baseline_scene(args: argparse.Namespace, catalog: dict[str, Any], method: str, robot: str, difficulty: str, seed: int, budget_s: float | None = None) -> dict[str, Any]:
    if method == "rrtconnect":
        row = run_rrtconnect_scene(args, catalog, robot, difficulty, seed)
        row["stage_id"] = f"timeout{float(args.rrt_timeout_s):g}s"
        row["budget_s"] = float(args.rrt_timeout_s)
        return row
    if method == "prm":
        return run_prm_scene(args, catalog, robot, difficulty, seed, float(budget_s if budget_s is not None else args.prm_build_s))
    if method == "bitstar":
        return run_bitstar_scene(args, catalog, robot, difficulty, seed, float(budget_s if budget_s is not None else args.bitstar_timeout_s))
    return {
        "method": method,
        "robot": robot,
        "difficulty": difficulty,
        "scene_seed": int(seed),
        "deep_max_boxes": 0,
        "stage_id": method,
        "budget_s": math.nan,
        "status": "external_pending",
        "success_count": 0,
        "query_count": 1,
        "planning_s": math.nan,
        "audit_s": math.nan,
        "path_length_mean": math.nan,
        "raw_segment_fraction": math.nan,
        "final_boxes": math.nan,
        "diagnostics": {"reason": "IRIS/GCS backend is not executed by this self-contained runner."},
    }


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    def timeout_cap(row: dict[str, Any]) -> float:
        diagnostics = row.get("diagnostics", {})
        if isinstance(diagnostics, dict):
            try:
                value = float(diagnostics.get("timeout_s", math.nan))
            except (TypeError, ValueError):
                value = math.nan
            if math.isfinite(value):
                return value
        return math.nan

    out: list[dict[str, Any]] = []
    keys = sorted({
        (
            row["method"],
            row["robot"],
            row["difficulty"],
            str(row.get("offline_grower", "")),
            str(row.get("stage_id", "")),
            int(row.get("deep_max_boxes", 0) or 0),
        )
        for row in rows
    })
    for method, robot, difficulty, offline_grower, stage_id, budget in keys:
        items = [
            row for row in rows
            if row["method"] == method
            and row["robot"] == robot
            and row["difficulty"] == difficulty
            and str(row.get("offline_grower", "")) == offline_grower
            and str(row.get("stage_id", "")) == stage_id
            and int(row.get("deep_max_boxes", 0) or 0) == budget
        ]
        success_items = [row for row in items if int(row.get("success_count", 0)) == int(row.get("query_count", 1))]
        success_queries = sum(int(row.get("success_count", 0)) for row in items)
        total_queries = sum(int(row.get("query_count", 0)) for row in items)
        out.append(
            {
                "method": method,
                "robot": robot,
                "difficulty": difficulty,
                "offline_grower": offline_grower,
                "stage_id": stage_id,
                "leaf_start_depth": median(row.get("leaf_start_depth", math.nan) for row in items),
                "leaf_max_depth": median(row.get("leaf_max_depth", math.nan) for row in items),
                "deep_ffb_depth": median(row.get("deep_ffb_depth", math.nan) for row in items),
                "connector_pave_depth": median(row.get("connector_pave_depth", math.nan) for row in items),
                "query_bridge_pave_depth": median(row.get("query_bridge_pave_depth", math.nan) for row in items),
                "ffb_start_depth": median(row.get("ffb_start_depth", math.nan) for row in items),
                "ffb_binary_probe_depth": median(
                    row.get("ffb_binary_probe_depth", math.nan) for row in items
                ),
                "query_bridge_ffb_start_depth": median(
                    row.get("query_bridge_ffb_start_depth", math.nan) for row in items
                ),
                "query_endpoint_anchor_ffb_depth": median(
                    row.get("query_endpoint_anchor_ffb_depth", math.nan) for row in items
                ),
                "query_bridge_sequential_reuse": median(
                    1.0 if bool(row.get("query_bridge_sequential_reuse", False)) else 0.0
                    for row in items
                ),
                "query_bridge_scene_reusable_edges": median(
                    1.0 if bool(row.get("query_bridge_scene_reusable_edges", False)) else 0.0
                    for row in items
                ),
                "query_bridge_edge_cost_penalty": median(
                    row.get("query_bridge_edge_cost_penalty", math.nan) for row in items
                ),
                "query_bridge_forced_attempts": median(
                    row.get("query_bridge_forced_attempts", math.nan) for row in items
                ),
                "query_bridge_parallel_rrt_early_stop": median(
                    1.0 if bool(row.get("query_bridge_parallel_rrt_early_stop", False)) else 0.0
                    for row in items
                ),
                "query_bridge_parallel_rrt_early_stop_min_successes": median(
                    row.get("query_bridge_parallel_rrt_early_stop_min_successes", math.nan)
                    for row in items
                ),
                "query_bridge_parallel_rrt_early_stop_ratio": median(
                    row.get("query_bridge_parallel_rrt_early_stop_ratio", math.nan)
                    for row in items
                ),
                "query_bridge_parallel_rrt_early_stop_additive": median(
                    row.get("query_bridge_parallel_rrt_early_stop_additive", math.nan)
                    for row in items
                ),
                "query_bridge_direct_segment_after_rrt": median(
                    1.0 if bool(row.get("query_bridge_direct_segment_after_rrt", False)) else 0.0
                    for row in items
                ),
                "query_bridge_fast_direct_segment_after_rrt": median(
                    1.0 if bool(row.get("query_bridge_fast_direct_segment_after_rrt", False)) else 0.0
                    for row in items
                ),
                "query_bridge_fast_direct_random_shortcut_iters": median(
                    row.get("query_bridge_fast_direct_random_shortcut_iters", 0.0)
                    for row in items
                ),
                "query_bridge_hybridize_attempt_paths": median(
                    1.0 if bool(row.get("query_bridge_hybridize_attempt_paths", False)) else 0.0
                    for row in items
                ),
                "query_bridge_hybrid_max_paths": median(
                    row.get("query_bridge_hybrid_max_paths", math.nan) for row in items
                ),
                "query_bridge_hybrid_max_vertices": median(
                    row.get("query_bridge_hybrid_max_vertices", math.nan) for row in items
                ),
                "query_bridge_hybrid_max_cross_checks": median(
                    row.get("query_bridge_hybrid_max_cross_checks", math.nan) for row in items
                ),
                "query_endpoint_point_anchor": median(
                    1.0 if bool(row.get("query_endpoint_point_anchor", False)) else 0.0
                    for row in items
                ),
                "rbf_robot_tuned_profile": median(
                    1.0 if bool(row.get("rbf_robot_tuned_profile", False)) else 0.0
                    for row in items
                ),
                "ffb_start_depth": median(row.get("ffb_start_depth", math.nan) for row in items),
                "rbf_max_depth": median(row.get("rbf_max_depth", math.nan) for row in items),
                "budget_s": median(row.get("budget_s", math.nan) for row in items),
                "timeout_cap_s": median(timeout_cap(row) for row in items) if method == "bitstar" else math.nan,
                "deep_max_boxes": budget,
                "scenes": len(items),
                "success_scenes": len(success_items),
                "queries_per_scene": median(row.get("query_count", 0) for row in items),
                "success_queries": success_queries,
                "total_queries": total_queries,
                "obstacles_median": median(row.get("obstacle_count", math.nan) for row in items),
                "measured_time_s_median": median(row.get("planning_s", math.nan) for row in items),
                "planning_s_median": median(row.get("planning_s", math.nan) for row in items),
                "offline_build_s_median": median(row.get("offline_build_s", row.get("build_s", 0.0)) for row in items),
                "offline_coverage_profile": str(items[0].get("offline_coverage_profile", "")),
                "offline_coverage_s_median": median(row.get("offline_coverage_s", math.nan) for row in items),
                "offline_connector_mode": str(items[0].get("offline_connector_mode", "")),
                "offline_connector_s_median": median(row.get("offline_connector_s", math.nan) for row in items),
                "online_batch_s_median": median(row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0))) for row in items),
                "online_total_s_median": median(row.get("online_total_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) for row in items),
                "online_solve_s_median": median(row.get("online_solve_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) for row in items),
                "online_simplify_s_median": median(row.get("online_simplify_s", 0.0) for row in items),
                "online_solve_per_query_s_median": median(
                    row.get(
                        "online_solve_per_query_s",
                        row.get("online_solve_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "online_simplify_per_query_s_median": median(
                    row.get(
                        "online_simplify_per_query_s",
                        row.get("online_simplify_s", 0.0) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "online_per_query_s_median": median(
                    row.get(
                        "online_per_query_s",
                        row.get("online_solve_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0))) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "online_total_per_query_s_median": median(
                    row.get(
                        "online_total_per_query_s",
                        row.get("online_total_s", row.get("online_batch_s", max(0.0, row.get("planning_s", 0.0) - row.get("build_s", 0.0)))) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "query_bridge_per_query_s_median": median(
                    row.get(
                        "query_bridge_per_query_s",
                        row.get("query_bridge_s", 0.0) / max(1, int(row.get("query_count", 1))),
                    )
                    for row in items
                ),
                "diag_query_bridge_batch_total_ms_median": median(row.get("diag_query_bridge_batch_total_ms", 0.0) for row in items),
                "diag_query_bridge_batch_per_query_ms_median": median(row.get("diag_query_bridge_batch_per_query_ms", 0.0) for row in items),
                "diag_query_bridge_batch_rrt_ms_total_median": median(row.get("diag_query_bridge_batch_rrt_ms_total", 0.0) for row in items),
                "diag_query_bridge_batch_probe_ms_total_median": median(row.get("diag_query_bridge_batch_probe_ms_total", 0.0) for row in items),
                "diag_query_bridge_batch_pave_ms_total_median": median(row.get("diag_query_bridge_batch_pave_ms_total", 0.0) for row in items),
                "diag_query_bridge_rrt_fixed_iters_median": median(row.get("diag_query_bridge_rrt_fixed_iters", 0.0) for row in items),
                "diag_query_bridge_hybridize_attempt_paths_tasks_median": median(row.get("diag_query_bridge_hybridize_attempt_paths_tasks", 0.0) for row in items),
                "diag_query_bridge_hybridize_attempt_paths_candidates_median": median(row.get("diag_query_bridge_hybridize_attempt_paths_candidates", 0.0) for row in items),
                "diag_query_bridge_hybridize_attempt_paths_accepts_median": median(row.get("diag_query_bridge_hybridize_attempt_paths_accepts", 0.0) for row in items),
                "diag_query_bridge_hybridize_attempt_paths_delta_median": median(row.get("diag_query_bridge_hybridize_attempt_paths_delta", 0.0) for row in items),
                "diag_query_bridge_hybridize_attempt_paths_audit_rejects_median": median(row.get("diag_query_bridge_hybridize_attempt_paths_audit_rejects", 0.0) for row in items),
                "diag_query_bridge_no_path_retry_budget_stages_median": median(row.get("diag_query_bridge_no_path_retry_budget_stages", 0.0) for row in items),
                "diag_query_bridge_no_path_retry_adaptive_attempts_median": median(row.get("diag_query_bridge_batch_no_path_retry_adaptive_attempts", 0.0) for row in items),
                "diag_query_bridge_no_path_retry_adaptive_successes_median": median(row.get("diag_query_bridge_batch_no_path_retry_adaptive_successes", 0.0) for row in items),
                "diag_query_bridge_no_path_retry_adaptive_ms_total_median": median(row.get("diag_query_bridge_batch_no_path_retry_adaptive_ms_total", 0.0) for row in items),
                "diag_query_bridge_batch_tasks_initial_median": median(row.get("diag_query_bridge_batch_tasks_initial", 0.0) for row in items),
                "diag_query_bridge_batch_tasks_attempted_median": median(row.get("diag_query_bridge_batch_tasks_attempted", 0.0) for row in items),
                "diag_query_bridge_batch_tasks_no_path_median": median(row.get("diag_query_bridge_batch_tasks_no_path", 0.0) for row in items),
                "diag_query_bridge_direct_segment_after_rrt_median": median(row.get("diag_query_bridge_direct_segment_after_rrt", 0.0) for row in items),
                "diag_query_bridge_direct_segment_after_rrt_edges_median": median(row.get("diag_query_bridge_direct_segment_after_rrt_edges", 0.0) for row in items),
                "diag_query_bridge_direct_segment_after_rrt_audit_rejects_median": median(row.get("diag_query_bridge_direct_segment_after_rrt_audit_rejects", 0.0) for row in items),
                "diag_query_bridge_direct_segment_after_rrt_add_fail_median": median(row.get("diag_query_bridge_direct_segment_after_rrt_add_fail", 0.0) for row in items),
                "diag_query_bridge_fast_direct_segment_after_rrt_median": median(row.get("diag_query_bridge_fast_direct_segment_after_rrt", 0.0) for row in items),
                "diag_query_bridge_fast_direct_segment_after_rrt_edges_median": median(row.get("diag_query_bridge_fast_direct_segment_after_rrt_edges", 0.0) for row in items),
                "diag_query_bridge_endpoint_point_anchor_attempts_median": median(row.get("diag_query_bridge_endpoint_point_anchor_attempts", 0.0) for row in items),
                "diag_query_bridge_endpoint_point_anchor_success_median": median(row.get("diag_query_bridge_endpoint_point_anchor_success", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_ms_median": median(row.get("diag_query_bridge_direct_corridor_ms", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_ms_total_median": median(row.get("diag_query_bridge_direct_corridor_ms_total", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_ffb_calls_median": median(row.get("diag_query_bridge_direct_corridor_ffb_calls", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_ffb_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_ffb_calls_total", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_all_ffb_calls_median": median(row.get("diag_query_bridge_direct_corridor_all_ffb_calls", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_all_ffb_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_all_ffb_calls_total", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_added_median": median(row.get("diag_query_bridge_direct_corridor_added", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_added_total_median": median(row.get("diag_query_bridge_direct_corridor_added_total", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_adaptive_initial_bad_fraction_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_initial_bad_fraction", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_adaptive_final_bad_fraction_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_final_bad_fraction", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_adaptive_repair_calls_total_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_calls_total", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_adaptive_repair_added_total_median": median(row.get("diag_query_bridge_direct_corridor_adaptive_repair_added_total", 0.0) for row in items),
                "diag_query_bridge_direct_corridor_partition_neighbor_candidates_median": median(row.get("diag_query_bridge_direct_corridor_partition_neighbor_candidates", 0.0) for row in items),
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
                "amortized_s_k1": median(row.get("amortized_s_k1", row.get("planning_s", math.nan)) for row in items),
                "amortized_s_k5": median(row.get("amortized_s_k5", row.get("planning_s", math.nan) / 5.0) for row in items),
                "amortized_s_k10": median(row.get("amortized_s_k10", row.get("planning_s", math.nan) / 10.0) for row in items),
                "amortized_s_k20": median(row.get("amortized_s_k20", row.get("planning_s", math.nan) / 20.0) for row in items),
                "amortized_s_k50": median(row.get("amortized_s_k50", row.get("planning_s", math.nan) / 50.0) for row in items),
                "audit_s_median": median(row.get("audit_s", math.nan) for row in items),
                "path_length_mean": mean(row.get("path_length_mean", math.nan) for row in success_items),
                "raw_segment_fraction_median": median(row.get("raw_segment_fraction", math.nan) for row in success_items),
                "adaptive_deep_leaf_s_median": median(row.get("adaptive_deep_leaf_s", math.nan) for row in items),
                "adaptive_target_depth_median": median(row.get("adaptive_target_depth", math.nan) for row in items),
                "selected_leaf_depth_median": median(row.get("selected_leaf_depth", math.nan) for row in items),
                "adaptive_depth_readiness_rate": mean(1.0 if row.get("adaptive_depth_readiness_met", False) else 0.0 for row in items),
                "adaptive_validated_median": median(row.get("adaptive_validated", math.nan) for row in items),
                "adaptive_splits_median": median(row.get("adaptive_splits", math.nan) for row in items),
                "adaptive_deferred_median": median(row.get("adaptive_deferred", math.nan) for row in items),
                "adaptive_promoted_median": median(row.get("adaptive_promoted", math.nan) for row in items),
                "adaptive_unresolved_domains_median": median(row.get("adaptive_unresolved_domains", math.nan) for row in items),
                "adaptive_merge_input_boxes_median": median(row.get("adaptive_merge_input_boxes", math.nan) for row in items),
                "adaptive_merge_output_boxes_median": median(row.get("adaptive_merge_output_boxes", math.nan) for row in items),
                "adaptive_merge_grid_ms_median": median(row.get("adaptive_merge_grid_ms", math.nan) for row in items),
                "adaptive_merge_grid_merges_median": median(row.get("adaptive_merge_grid_merges", math.nan) for row in items),
                "adaptive_merge_tree_ms_median": median(row.get("adaptive_merge_tree_ms", math.nan) for row in items),
                "adaptive_merge_tree_merges_median": median(row.get("adaptive_merge_tree_merges", math.nan) for row in items),
                "adaptive_merge_containment_ms_median": median(row.get("adaptive_merge_containment_ms", math.nan) for row in items),
                "adaptive_merge_exact_ms_median": median(row.get("adaptive_merge_exact_ms", math.nan) for row in items),
                "adaptive_partition_merge_containment_skipped_median": median(row.get("adaptive_partition_merge_containment_skipped", math.nan) for row in items),
                "adaptive_partition_merge_containment_bucket_entries_median": median(row.get("adaptive_partition_merge_containment_bucket_entries", math.nan) for row in items),
                "adaptive_partition_merge_containment_candidates_median": median(row.get("adaptive_partition_merge_containment_candidates", math.nan) for row in items),
                "adaptive_partition_merge_containment_tests_median": median(row.get("adaptive_partition_merge_containment_tests", math.nan) for row in items),
                "adaptive_partition_merge_containment_overflow_median": median(row.get("adaptive_partition_merge_containment_overflow", math.nan) for row in items),
                "adaptive_partition_merge_containment_ms_median": median(row.get("adaptive_partition_merge_containment_ms", math.nan) for row in items),
                "adaptive_partition_merge_line_ms_median": median(row.get("adaptive_partition_merge_line_ms", math.nan) for row in items),
                "adaptive_adjacency_ms_median": median(row.get("adaptive_adjacency_ms", math.nan) for row in items),
                "adaptive_adjacency_candidates_median": median(row.get("adaptive_adjacency_candidates", math.nan) for row in items),
                "adaptive_adjacency_exact_tests_median": median(row.get("adaptive_adjacency_exact_tests", math.nan) for row in items),
                "partition_cell_count_median": median(row.get("partition_cell_count", math.nan) for row in items),
                "partition_grid_cell_count_median": median(row.get("partition_grid_cell_count", math.nan) for row in items),
                "partition_non_grid_cell_count_median": median(row.get("partition_non_grid_cell_count", math.nan) for row in items),
                "partition_point_index_dims_median": median(row.get("partition_point_index_dims", math.nan) for row in items),
                "partition_point_index_entries_median": median(row.get("partition_point_index_entries", math.nan) for row in items),
                "partition_point_index_overflow_cells_median": median(row.get("partition_point_index_overflow_cells", math.nan) for row in items),
                "partition_islands_median": median(row.get("partition_islands", math.nan) for row in items),
                "partition_largest_island_median": median(row.get("partition_largest_island", math.nan) for row in items),
                "partition_overlay_edges_median": median(row.get("partition_overlay_edges", math.nan) for row in items),
                "partition_build_ms_median": median(row.get("partition_build_ms", math.nan) for row in items),
                "partition_index_rebuild_ms_median": median(row.get("partition_index_rebuild_ms", math.nan) for row in items),
                "partition_face_index_ms_median": median(row.get("partition_face_index_ms", math.nan) for row in items),
                "partition_point_index_ms_median": median(row.get("partition_point_index_ms", math.nan) for row in items),
                "partition_neighbor_cache_ms_median": median(row.get("partition_neighbor_cache_ms", math.nan) for row in items),
                "partition_island_rebuild_ms_median": median(row.get("partition_island_rebuild_ms", math.nan) for row in items),
                "partition_adjacency_candidates_median": median(row.get("partition_adjacency_candidates", math.nan) for row in items),
                "partition_adjacency_edges_median": median(row.get("partition_adjacency_edges", math.nan) for row in items),
                "partition_query_per_query_s_median": median(row.get("partition_query_per_query_s", math.nan) for row in items),
                "partition_query_total_per_query_s_median": median(row.get("partition_query_total_per_query_s", math.nan) for row in items),
                "coverage_probe_free_count_median": median(row.get("coverage_probe_free_count", math.nan) for row in items),
                "coverage_box_covered_probability_median": median(row.get("coverage_box_covered_probability", math.nan) for row in items),
                "coverage_anchor_success_probability_median": median(row.get("coverage_anchor_success_probability", math.nan) for row in items),
                "coverage_main_accessible_probability_median": median(row.get("coverage_main_accessible_probability", math.nan) for row in items),
                "offline_shortcut_s_median": median(row.get("offline_shortcut_s", math.nan) for row in items),
                "offline_shortcut_edges_requested_median": median(row.get("offline_shortcut_edges_requested", math.nan) for row in items),
                "offline_shortcut_edges_added_median": median(row.get("offline_shortcut_edges_added", math.nan) for row in items),
                "offline_shortcut_box_corridor_edges_added_median": median(row.get("offline_shortcut_box_corridor_edges_added", math.nan) for row in items),
                "offline_shortcut_segment_edges_added_median": median(row.get("offline_shortcut_segment_edges_added", math.nan) for row in items),
                "offline_shortcut_pave_boxes_added_median": median(row.get("offline_shortcut_pave_boxes_added", math.nan) for row in items),
                "offline_shortcut_pave_fail_median": median(row.get("offline_shortcut_pave_fail", math.nan) for row in items),
                "final_boxes_median": median(row.get("final_boxes", math.nan) for row in items),
                "status": "executed",
            }
        )
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "method",
        "robot",
        "difficulty",
        "offline_grower",
        "stage_id",
        "leaf_start_depth",
        "leaf_max_depth",
        "deep_ffb_depth",
        "connector_pave_depth",
        "query_bridge_pave_depth",
        "query_endpoint_anchor_ffb_depth",
        "query_bridge_ffb_start_depth",
        "query_bridge_sequential_reuse",
        "query_bridge_scene_reusable_edges",
        "query_bridge_edge_cost_penalty",
        "query_bridge_forced_attempts",
        "query_bridge_parallel_rrt_early_stop",
        "query_bridge_parallel_rrt_early_stop_min_successes",
        "query_bridge_parallel_rrt_early_stop_ratio",
        "query_bridge_parallel_rrt_early_stop_additive",
        "query_bridge_direct_segment_after_rrt",
        "query_bridge_fast_direct_segment_after_rrt",
        "query_bridge_fast_direct_random_shortcut_iters",
        "query_bridge_hybridize_attempt_paths",
        "query_bridge_hybrid_max_paths",
        "query_bridge_hybrid_max_vertices",
        "query_bridge_hybrid_max_cross_checks",
        "query_endpoint_point_anchor",
        "rbf_robot_tuned_profile",
        "ffb_start_depth",
        "ffb_binary_probe_depth",
        "rbf_max_depth",
        "budget_s",
        "timeout_cap_s",
        "deep_max_boxes",
        "scenes",
        "success_scenes",
        "queries_per_scene",
        "success_queries",
        "total_queries",
        "obstacles_median",
        "measured_time_s_median",
        "planning_s_median",
        "offline_build_s_median",
        "offline_coverage_profile",
        "offline_coverage_s_median",
        "offline_connector_mode",
        "offline_connector_s_median",
        "online_batch_s_median",
        "online_total_s_median",
        "online_solve_s_median",
        "online_simplify_s_median",
        "online_solve_per_query_s_median",
        "online_simplify_per_query_s_median",
        "online_per_query_s_median",
        "online_total_per_query_s_median",
        "query_bridge_per_query_s_median",
        "diag_query_bridge_batch_total_ms_median",
        "diag_query_bridge_batch_per_query_ms_median",
        "diag_query_bridge_batch_rrt_ms_total_median",
        "diag_query_bridge_batch_probe_ms_total_median",
        "diag_query_bridge_batch_pave_ms_total_median",
        "diag_query_bridge_rrt_fixed_iters_median",
        "diag_query_bridge_hybridize_attempt_paths_tasks_median",
        "diag_query_bridge_hybridize_attempt_paths_candidates_median",
        "diag_query_bridge_hybridize_attempt_paths_accepts_median",
        "diag_query_bridge_hybridize_attempt_paths_delta_median",
        "diag_query_bridge_hybridize_attempt_paths_audit_rejects_median",
        "diag_query_bridge_no_path_retry_budget_stages_median",
        "diag_query_bridge_no_path_retry_adaptive_attempts_median",
        "diag_query_bridge_no_path_retry_adaptive_successes_median",
        "diag_query_bridge_no_path_retry_adaptive_ms_total_median",
        "diag_query_bridge_batch_tasks_initial_median",
        "diag_query_bridge_batch_tasks_attempted_median",
        "diag_query_bridge_batch_tasks_no_path_median",
        "diag_query_bridge_direct_segment_after_rrt_median",
        "diag_query_bridge_direct_segment_after_rrt_edges_median",
        "diag_query_bridge_direct_segment_after_rrt_audit_rejects_median",
        "diag_query_bridge_direct_segment_after_rrt_add_fail_median",
        "diag_query_bridge_fast_direct_segment_after_rrt_median",
        "diag_query_bridge_fast_direct_segment_after_rrt_edges_median",
        "diag_query_bridge_endpoint_point_anchor_attempts_median",
        "diag_query_bridge_endpoint_point_anchor_success_median",
        "diag_query_bridge_direct_corridor_ms_median",
        "diag_query_bridge_direct_corridor_ms_total_median",
        "diag_query_bridge_direct_corridor_ffb_calls_median",
        "diag_query_bridge_direct_corridor_ffb_calls_total_median",
        "diag_query_bridge_direct_corridor_all_ffb_calls_median",
        "diag_query_bridge_direct_corridor_all_ffb_calls_total_median",
        "diag_query_bridge_direct_corridor_added_median",
        "diag_query_bridge_direct_corridor_added_total_median",
        "diag_query_bridge_direct_corridor_adaptive_initial_bad_fraction_median",
        "diag_query_bridge_direct_corridor_adaptive_final_bad_fraction_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_calls_total_median",
        "diag_query_bridge_direct_corridor_adaptive_repair_added_total_median",
        "diag_query_bridge_direct_corridor_partition_neighbor_candidates_median",
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
        "amortized_s_k1",
        "amortized_s_k5",
        "amortized_s_k10",
        "amortized_s_k20",
        "amortized_s_k50",
        "audit_s_median",
        "path_length_mean",
        "raw_segment_fraction_median",
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
        "partition_islands_median",
        "partition_largest_island_median",
        "partition_overlay_edges_median",
        "partition_build_ms_median",
        "partition_index_rebuild_ms_median",
        "partition_adjacency_candidates_median",
        "partition_adjacency_edges_median",
        "partition_query_per_query_s_median",
        "partition_query_total_per_query_s_median",
        "coverage_probe_free_count_median",
        "coverage_box_covered_probability_median",
        "coverage_anchor_success_probability_median",
        "coverage_main_accessible_probability_median",
        "offline_shortcut_s_median",
        "offline_shortcut_edges_requested_median",
        "offline_shortcut_edges_added_median",
        "offline_shortcut_box_corridor_edges_added_median",
        "offline_shortcut_segment_edges_added_median",
        "offline_shortcut_pave_boxes_added_median",
        "offline_shortcut_pave_fail_median",
        "final_boxes_median",
        "status",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)


def select_best_tradeoff_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Select one readable row per robot/difficulty; full budget curves stay in the figure/CSV."""
    def path_stat(row: dict[str, Any]) -> float:
        for key in ("path_length_mean", "path_length_median"):
            try:
                value = float(row.get(key, math.nan))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                return value
        return math.nan

    out: list[dict[str, Any]] = []
    robot_order = {"iiwa": 0, "ur5": 1, "panda": 2}
    difficulty_order = {"easy": 0, "medium": 1, "hard": 2}
    keys = sorted(
        {(str(row.get("robot", "")), str(row.get("difficulty", ""))) for row in rows},
        key=lambda item: (
            robot_order.get(item[0], 100),
            difficulty_order.get(item[1], 100),
            item[0],
            item[1],
        ),
    )
    for robot, difficulty in keys:
        items = [row for row in rows if str(row.get("robot", "")) == robot and str(row.get("difficulty", "")) == difficulty]
        full = [
            row for row in items
            if int(float(row.get("success_scenes", 0) or 0)) == int(float(row.get("scenes", 0) or 0))
        ]
        candidates = full or items
        if not candidates:
            continue
        finite_path = [
            path_stat(row)
            for row in candidates
            if math.isfinite(path_stat(row))
        ]
        if finite_path:
            best_path = min(finite_path)
            candidates = [
                row for row in candidates
                if math.isfinite(path_stat(row))
                and path_stat(row) <= 1.08 * best_path
            ] or candidates
        out.append(sorted(
            candidates,
            key=lambda row: (
                float(row.get("online_per_query_s_median", math.nan)) if math.isfinite(float(row.get("online_per_query_s_median", math.nan))) else 1e9,
                (
                    float(row.get("offline_build_s_median", row.get("build_s", 0.0))) / 10.0
                    + float(row.get("online_per_query_s_median", math.nan))
                    if math.isfinite(float(row.get("online_per_query_s_median", math.nan)))
                    else 1e9
                ),
                int(float(row.get("deep_max_boxes", 0) or 0)),
            ),
        )[0])
    return out


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    rows = select_best_tradeoff_rows(rows)
    lines = [
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Saved-catalog random-scene reusable-planner best trade-off points. Each obstacle scene stores fixed multi-query batches; RBF builds once per scene and online timings reuse the offline forest. Online/q excludes final simplification; Simplify/q reports the measured cost under the globally fixed 0.01~s OMPL post-processing budget. Times exclude final audit; full box-budget and amortization curves are shown in Fig.~\ref{fig:tro_random_tradeoff}.}",
        r"\label{tab:tro-random-summary}",
        r"\footnotesize",
        r"\begin{tabular}{llrrrrrrrrr}",
        r"\toprule",
        r"Robot & Difficulty & Boxes & SR & Build & Online/q & Simplify/q & Amort@10 & Path & Seg. \\",
        r"\midrule",
    ]
    for row in rows:
        sr = f"{int(row.get('success_queries', 0))}/{int(row.get('total_queries', 0))}"
        lines.append(
            f"{row.get('robot')} & {row.get('difficulty')} & {int(row.get('deep_max_boxes', 0) or 0)} & {sr} & "
            f"{tex_num(row.get('offline_build_s_median'))} & {tex_num(row.get('online_per_query_s_median'))} & "
            f"{tex_num(row.get('online_simplify_per_query_s_median'))} & "
            f"{tex_num(float(row.get('offline_build_s_median', row.get('build_s', 0.0))) / 10.0 + float(row.get('online_per_query_s_median', 0.0)))} & "
            f"{tex_num(row.get('path_length_mean', row.get('path_length_median')))} & "
            f"{tex_num(row.get('raw_segment_fraction_median'))} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\par\endgroup", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    args._argv = list(sys.argv[1:])
    apply_hipac_improved_leaf_sweep_profile(args, args._argv)
    apply_offline_coverage_profile(args, args._argv)
    normalize_adaptive_depth_cap(args, sys.argv[1:])
    configure_thread_environment(int(args.threads))
    robots = csv_list(args.robots)
    difficulties = csv_list(args.difficulties)
    methods = csv_list(args.methods)
    explicit_scene_seed_values = [
        int(item)
        for item in csv_list(args.scene_seed_list)
    ] if str(args.scene_seed_list).strip() else []
    requested_scene_seeds = 1 if args.phase == "smoke" else (10 if args.phase == "paper" else int(args.scene_seeds))
    scene_seeds = int(requested_scene_seeds)
    if str(args.scene_catalog_mode) in {"reuse", "verify"}:
        inferred_scene_seeds = catalog_seed_count_for_selection(Path(args.scene_catalog), robots, difficulties)
        if inferred_scene_seeds is not None:
            if (
                int(inferred_scene_seeds) < int(requested_scene_seeds)
                and str(args.phase) in {"paper", "full"}
                and not bool(args.allow_fewer_catalog_scenes)
            ):
                raise RuntimeError(
                    f"scene catalog {args.scene_catalog} only has {int(inferred_scene_seeds)} contiguous scene seeds "
                    f"for the selected robots/difficulties; Exp.6 {args.phase} requires {int(requested_scene_seeds)}. "
                    "Use --allow-fewer-catalog-scenes only for diagnostics."
                )
            scene_seeds = int(inferred_scene_seeds)
    box_budgets = [int(item) for item in csv_list(args.box_budgets)] if str(args.box_budgets).strip() else rbf_budget_grid(args.phase)
    prm_build_grid_s = prm_build_grid_from_args(args)
    bitstar_explicit_grid = [float(item) for item in csv_list(args.bitstar_timeout_grid_s)] if str(args.bitstar_timeout_grid_s).strip() else []
    bitstar_checkpoint_grid = [float(item) for item in csv_list(args.bitstar_checkpoint_grid_s)] if str(args.bitstar_checkpoint_grid_s).strip() else bitstar_explicit_grid
    bitstar_trace_timeout_s = max(bitstar_checkpoint_grid or bitstar_explicit_grid) if (bitstar_checkpoint_grid or bitstar_explicit_grid) else float(args.bitstar_timeout_s)
    bitstar_stage_s = bitstar_checkpoint_grid_from_args(args, bitstar_trace_timeout_s)
    args.bitstar_checkpoint_interval_s = bitstar_trace_interval_for_grid(args, bitstar_stage_s, bitstar_trace_timeout_s)
    if args.phase == "smoke":
        robots = robots[:1]
        difficulties = difficulties[:1]
        box_budgets = [int(args.deep_max_boxes)]
        prm_build_grid_s = [min(float(args.prm_build_s), 0.25)]
        bitstar_trace_timeout_s = min(float(args.bitstar_timeout_s), 0.25)
        bitstar_stage_s = [bitstar_trace_timeout_s]
        args.queries_per_scene = min(int(args.queries_per_scene), 3)
    scene_seed_values = explicit_scene_seed_values if explicit_scene_seed_values else list(range(scene_seeds))
    profile_args = copy.copy(args)
    apply_offline_coverage_profile(profile_args, getattr(args, "_argv", []))
    rbf_profile = effective_rbf_profile(profile_args, box_budgets)
    catalog_path = args.scene_catalog or (args.out_dir / "random_scene_catalog_v7.json")
    catalog_summary: dict[str, Any] = {
        "path": str(catalog_path),
        "mode": args.scene_catalog_mode,
        "records": None,
        "queries_per_scene": int(args.queries_per_scene),
    }
    catalog: dict[str, Any] | None = None
    cache_rows: list[dict[str, Any]] = []
    if "sbf_leaf_rrt" in methods:
        for robot in progress(robots, desc="exp06 lect cache", total=len(robots), disable=bool(args.dry_run or args.skip_lect_cache_ensure)):
            cache_rows.append(
                ensure_robot_lectdb_cache(
                    robot,
                    cache_root=Path(args.lect_cache_root),
                    threads=int(args.threads),
                    dry_run=bool(args.dry_run or args.skip_lect_cache_ensure),
                )
            )
    if not args.dry_run:
        catalog = generate_catalog(
            path=catalog_path,
            robots=robots,
            difficulties=difficulties,
            scene_seeds=scene_seeds,
            scene_profile=args.scene_profile,
            seed_base=int(args.seed_base),
            queries_per_scene=int(args.queries_per_scene),
            max_scene_tries=int(args.max_scene_tries),
            mode=args.scene_catalog_mode,
        )
        catalog_summary.update({
            "schema": catalog.get("schema"),
            "records": len(catalog.get("records", [])),
            "queries_per_scene": int(catalog.get("queries_per_scene", args.queries_per_scene)),
        })
    rows = []
    for method in METHODS:
        if method not in methods:
            continue
        if method == "sbf_leaf_rrt":
            budgets: list[Any] = box_budgets
        elif method == "prm":
            budgets = (
                [{
                    "cumulative": True,
                    "checkpoints": prm_build_grid_s,
                    "build_s": max(prm_build_grid_s) if prm_build_grid_s else float(args.prm_build_s),
                }]
                if bool(args.prm_cumulative)
                else prm_build_grid_s
            )
        elif method == "bitstar":
            budgets = [bitstar_trace_timeout_s]
        elif method == "rrtconnect":
            budgets = [float(args.rrt_timeout_s)]
        else:
            budgets = [None]
        for robot in robots:
            for difficulty in difficulties:
                scenario_budgets = list(budgets)
                if (
                    method == "sbf_leaf_rrt"
                    and bool(getattr(args, "rbf_robot_tuned_profile", True))
                    and not str(args.box_budgets).strip()
                ):
                    row_args_for_budget = apply_exp06_rbf_profiles(args, robot, difficulty)
                    scenario_budgets = [int(row_args_for_budget.deep_max_boxes)]
                for seed in scene_seed_values:
                    for budget in scenario_budgets:
                        stage_id = (
                            f"b{int(budget)}" if method == "sbf_leaf_rrt"
                            else (
                                (
                                    f"{str(args.prm_planner_kind)}_cumulative_trace"
                                    f"{float(budget.get('build_s', max(prm_build_grid_s))):g}s"
                                    f"_k{int(args.prm_max_nearest_neighbors)}"
                                    f"_q{fmt_float(float(args.prm_query_s))}s"
                                    f"_preload{int(bool(args.prm_preload_query_endpoints))}"
                                )
                                if isinstance(budget, dict) and bool(budget.get("cumulative"))
                                else f"{str(args.prm_planner_kind)}_build{float(budget):g}s"
                                f"_k{int(args.prm_max_nearest_neighbors)}"
                                f"_q{fmt_float(float(args.prm_query_s))}s"
                                f"_preload{int(bool(args.prm_preload_query_endpoints))}"
                            ) if method == "prm"
                            else (
                                f"batch{int(args.bitstar_samples_per_batch)}"
                                f"_rw{float(args.bitstar_rewire_factor):g}"
                                f"_trace{float(budget):g}s_dt{float(args.bitstar_checkpoint_interval_s):g}s"
                            ) if method == "bitstar"
                            else f"timeout{float(args.rrt_timeout_s):g}s" if method == "rrtconnect"
                            else method
                        )
                        row_rbf_profile = None
                        row_rbf_lectdb = None
                        row_args = args
                        if method == "sbf_leaf_rrt":
                            row_args = apply_exp06_rbf_profiles(args, robot, difficulty)
                            row_split = effective_lect_split_schedule(
                                row_args,
                                robot,
                                getattr(row_args, "_argv", []),
                            )
                            row_rbf_profile = effective_rbf_profile(
                                row_args,
                                box_budgets,
                                split_schedule_kind=row_split,
                            )
                            row_rbf_lectdb = robot_lectdb_profile(robot)
                        rows.append({
                            "method": method,
                            "robot": robot,
                            "difficulty": difficulty,
                            "scene_seed": seed,
                            "stage_id": stage_id,
                            "budget_s": (
                                float(budget.get("build_s", max(prm_build_grid_s)))
                                if method == "prm" and isinstance(budget, dict)
                                else float(budget)
                                if method != "sbf_leaf_rrt" and budget is not None
                                else None
                            ),
                            "deep_max_boxes": budget if method == "sbf_leaf_rrt" else 0,
                            "scene_catalog": str(catalog_path),
                            "queries_per_scene": int(args.queries_per_scene),
                            "audit_segment_step": float(args.audit_segment_step),
                            "audit_collision_tolerance": float(args.audit_collision_tolerance),
                            "active_planning_root": "full_robot_joint_limits",
                            "coverage_root": "full_robot_joint_limits",
                            "canonical_mapping_scope": "LECT_internal_only",
                            "offline_query_agnostic_build": bool(method == "sbf_leaf_rrt"),
                            "status": "planned" if args.dry_run else "planned_for_execution",
                            "rbf_default_profile": row_rbf_profile,
                            "rbf_robot_lectdb": row_rbf_lectdb,
                            "rbf_robot_tuned_profile": (
                                bool(getattr(row_args, "rbf_robot_tuned_profile", False))
                                if method == "sbf_leaf_rrt" else None
                            ),
                            "rbf_box_budgets": box_budgets if method == "sbf_leaf_rrt" else None,
                            "ompl_registered_profile": "exp05_registered" if method in {"prm", "bitstar"} else None,
                            "ompl_simplify_time_s": float(args.ompl_simplify_time_s) if method in {"prm", "bitstar", "rrtconnect"} else None,
                            "prm_config": {
                                "cumulative": bool(args.prm_cumulative),
                                "build_schedule": str(args.prm_build_schedule),
                                "build_interval_s": float(args.prm_build_interval_s),
                                "build_max_step_s": float(args.prm_build_max_step_s),
                                "build_s": (
                                    float(budget.get("build_s", max(prm_build_grid_s)))
                                    if isinstance(budget, dict)
                                    else float(budget)
                                ) if method == "prm" else None,
                                "build_checkpoints_s": (
                                    list(budget.get("checkpoints", prm_build_grid_s))
                                    if isinstance(budget, dict)
                                    else [float(budget)]
                                ) if method == "prm" else None,
                                "query_s": float(args.prm_query_s),
                                "max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
                                "planner_kind": str(args.prm_planner_kind),
                                "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
                                "early_stop_success_stall_checkpoints": int(args.prm_early_stop_success_stall_checkpoints),
                                "early_stop_path_rel_tol": float(args.prm_early_stop_path_rel_tol),
                                "unresolved_query_retry_interval_s": float(args.prm_unresolved_query_retry_interval_s),
                                "solved_query_recheck_interval_s": float(args.prm_solved_query_recheck_interval_s),
                            } if method == "prm" else None,
                            "bitstar_config": {
                                "timeout_s": float(budget) if method == "bitstar" else None,
                                "checkpoint_schedule": str(args.bitstar_checkpoint_schedule),
                                "checkpoint_max_step_s": float(args.bitstar_checkpoint_max_step_s),
                                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                                "checkpoint_grid_s": bitstar_stage_s,
                                "checkpoint_stage_s": bitstar_stage_s,
                                "samples_per_batch": int(args.bitstar_samples_per_batch),
                                "rewire_factor": float(args.bitstar_rewire_factor),
                                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
                                "quality_stall_checkpoints": int(args.bitstar_quality_stall_checkpoints),
                                "quality_stall_rel_tol": float(args.bitstar_quality_stall_rel_tol),
                                "supplement_failed_from_manifest": (
                                    str(args.bitstar_supplement_failed_from_manifest)
                                    if args.bitstar_supplement_failed_from_manifest is not None
                                    else None
                                ),
                            } if method == "bitstar" else None,
                            "metrics": ["success_rate", "planning_s", "audit_s", "path_length", "raw_segment_fraction"],
                        })
    checkpoint_dir = Path(args.checkpoint_dir) if args.checkpoint_dir is not None else (Path(args.out_dir) / "run_parts")
    resume_loaded_parts = 0
    resume_written_parts = 0
    run_rows: list[dict[str, Any]] = []
    if not args.dry_run and catalog is not None:
        for row in progress(rows, desc="exp06 runs", total=len(rows)):
            part_path = planned_row_part_path(checkpoint_dir, row)
            if bool(args.resume) and not bool(args.force_rerun) and part_path.exists():
                part_rows = load_result_part(part_path)
                if part_rows is not None:
                    print(
                        f"[exp06] resume hit method={row['method']} stage={row.get('stage_id')} "
                        f"robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']} rows={len(part_rows)}",
                        flush=True,
                    )
                    run_rows.extend(part_rows)
                    resume_loaded_parts += 1
                    continue
                print(f"[exp06] resume part ignored as corrupt/incomplete: {part_path}", flush=True)

            produced_rows: list[dict[str, Any]]
            if row["method"] == "sbf_leaf_rrt":
                print(f"[exp06] method=sbf_leaf_rrt robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']} budget={row['deep_max_boxes']}", flush=True)
                args.deep_max_boxes = int(row["deep_max_boxes"])
                produced_rows = [run_rbf_scene(args, catalog, str(row["robot"]), str(row["difficulty"]), int(row["scene_seed"]))]
            elif row["method"] == "bitstar":
                print(f"[exp06] method=bitstar stage={row.get('stage_id')} robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']}", flush=True)
                produced_rows = run_bitstar_scene_trace(
                    args,
                    catalog,
                    str(row["robot"]),
                    str(row["difficulty"]),
                    int(row["scene_seed"]),
                    float(row["budget_s"]) if row.get("budget_s") is not None else float(args.bitstar_timeout_s),
                    float(args.bitstar_checkpoint_interval_s),
                    bitstar_stage_s,
                )
            elif row["method"] == "prm" and bool((row.get("prm_config") or {}).get("cumulative")):
                prm_config = row.get("prm_config") or {}
                print(f"[exp06] method=prm cumulative stage={row.get('stage_id')} robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']}", flush=True)
                produced_rows = run_prm_scene_cumulative(
                    args,
                    catalog,
                    str(row["robot"]),
                    str(row["difficulty"]),
                    int(row["scene_seed"]),
                    [float(value) for value in prm_config.get("build_checkpoints_s", prm_build_grid_s)],
                )
            else:
                print(f"[exp06] method={row['method']} stage={row.get('stage_id')} robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']}", flush=True)
                produced_rows = [run_baseline_scene(
                    args,
                    catalog,
                    str(row["method"]),
                    str(row["robot"]),
                    str(row["difficulty"]),
                    int(row["scene_seed"]),
                    float(row["budget_s"]) if row.get("budget_s") is not None else None,
                )]
            write_result_part(part_path, row, produced_rows)
            resume_written_parts += 1
            run_rows.extend(produced_rows)
    summary_rows = aggregate(run_rows) if run_rows else []
    payload: dict[str, Any] = {
        "experiment": "exp06_random_robot",
        "run_id": run_id("exp06"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "thread_policy": {
            "threads": int(args.threads),
            "scope": "RBF, LECT cache prewarm, and process math libraries use --threads=8 by default; OMPL RRTConnect, PRM, and BIT* are algorithmically single-thread in this runner unless OMPL exposes planner-internal parallelism.",
        },
        "checkpointing": {
            "enabled": bool(args.resume),
            "force_rerun": bool(args.force_rerun),
            "checkpoint_dir": str(checkpoint_dir),
            "loaded_parts": int(resume_loaded_parts),
            "written_parts": int(resume_written_parts),
            "planned_parts": len(rows),
            "row_parts_complete": int(resume_loaded_parts + resume_written_parts),
        },
        "offline_query_agnostic_build": True,
        "rbf_default_profile": rbf_profile,
        "audit": {
            "segment_step": float(args.audit_segment_step),
            "collision_tolerance": float(args.audit_collision_tolerance),
            "tolerance_policy": "strict_zero_tolerance",
        },
        "ompl_baseline_profile": {
            "inherits_from": "Exp.5 registered Shelf+IIWA OMPL baseline profile",
            "simplify_time_s": float(args.ompl_simplify_time_s),
            "prm": {
                "cumulative": bool(args.prm_cumulative),
                "build_schedule": str(args.prm_build_schedule),
                "build_interval_s": float(args.prm_build_interval_s),
                "build_max_step_s": float(args.prm_build_max_step_s),
                "build_grid_s": prm_build_grid_s,
                "build_checkpoints_s": prm_build_grid_s if bool(args.prm_cumulative) else [],
                "query_s": float(args.prm_query_s),
                "max_nearest_neighbors": int(args.prm_max_nearest_neighbors),
                "planner_kind": str(args.prm_planner_kind),
                "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
                "early_stop_success_stall_checkpoints": int(args.prm_early_stop_success_stall_checkpoints),
                "early_stop_path_rel_tol": float(args.prm_early_stop_path_rel_tol),
                "unresolved_query_retry_interval_s": float(args.prm_unresolved_query_retry_interval_s),
                "solved_query_recheck_interval_s": float(args.prm_solved_query_recheck_interval_s),
            },
            "bitstar": {
                "cumulative_bitstar": True,
                "checkpoint_schedule": str(args.bitstar_checkpoint_schedule),
                "checkpoint_max_step_s": float(args.bitstar_checkpoint_max_step_s),
                "stage_s": bitstar_stage_s,
                "timeout_s": float(bitstar_trace_timeout_s),
                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                "checkpoint_grid_s": bitstar_stage_s,
                "samples_per_batch": int(args.bitstar_samples_per_batch),
                "rewire_factor": float(args.bitstar_rewire_factor),
                "stop_on_solution_improvement": bool(args.bitstar_stop_on_solution_improvement),
                "quality_stall_checkpoints": int(args.bitstar_quality_stall_checkpoints),
                "quality_stall_rel_tol": float(args.bitstar_quality_stall_rel_tol),
                "supplement_failed_from_manifest": (
                    str(args.bitstar_supplement_failed_from_manifest)
                    if args.bitstar_supplement_failed_from_manifest is not None
                    else None
                ),
            },
        },
        "lectdb_caches": cache_rows,
        "scene_catalog": catalog_summary,
        "planned_rows": rows,
        "rows": run_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "random_robot_manifest.json", payload)
    if summary_rows:
        write_csv(args.out_dir / "random_robot_summary.csv", summary_rows)
        if str(args.phase) == "paper" and any(str(row.get("method")) == "sbf_leaf_rrt" for row in summary_rows):
            write_tex(args.out_dir / "random_robot_summary_table.tex", summary_rows)
    print(f"wrote {args.out_dir / 'random_robot_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
