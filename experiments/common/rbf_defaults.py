from __future__ import annotations

from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
CANONICAL_SYMMETRY_DESCRIPTOR = "joint_symmetry_native_v1"

RBF_DEFAULT_PROFILE_NAME = "exp04_leaf_sweep_qroot_b200_l15_d34"
RBF_SHELF_PROFILE_NAME = "exp04_leaf_sweep_qroot_d23_b200_l15_d34"
RBF_DEFAULT_BACKEND = "build_leaf_sweep_refined"
RBF_DEFAULT_GROWER_MODE = "rrt"

D23_CACHE_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_d23" / "cache"
D23_CACHE_LABEL = "iiwa_shelf_endpoint_only_p23_canonical_dim0q4_fixed_root"
ROBOT_LECTDB_CACHE_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "tro2026" / "lectdb_defaults" / "cache"
ROBOT_LECTDB_MAX_DEPTH = 40
ROBOT_DEFAULT_LECTDB_DEPTHS: dict[str, int] = {
    "iiwa": 23,
    "ur5": 20,
    "panda": 20,
}
D23_ROOT_INTERVALS: list[tuple[float, float]] = [
    (0.0, 1.5707963267948966),
    (0.3194, 0.8645),
    (-0.5077, 0.5073),
    (-1.98947519, -0.33002121),
    (-0.447, 0.4473),
    (-1.34734773, 1.51007653),
    (1.262, 1.8794),
]

DEFAULT_RBF_LEAF_START_DEPTH = 10
DEFAULT_RBF_LEAF_MAX_DEPTH = 15
DEFAULT_RBF_DEEP_MAX_BOXES = 200
DEFAULT_RBF_DEEP_FFB_DEPTH = 34
DEFAULT_RBF_REFINE_TIMEOUT_MS = 800.0
DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES = 1
DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS = 1.0
DEFAULT_RBF_DOMAIN_SEED_CAP = 24
DEFAULT_RBF_DOMAIN_SUCCESS_CAP = 8
DEFAULT_RBF_DOMAIN_ATTEMPT_CAP = 160
DEFAULT_RBF_VALIDATION_BATCH_SIZE = 512
DEFAULT_RBF_THREADS = 8
DEFAULT_RBF_FFB_START_DEPTH = 15

DEFAULT_RBF_CONNECTOR_PAIR_TIMEOUT_MS = 50.0
DEFAULT_RBF_CONNECTOR_MAX_PAIRS_PER_GAP = 1
DEFAULT_RBF_CONNECTOR_RRT_ITERS = 50000
DEFAULT_RBF_CONNECTOR_RRT_TIMEOUT_MS = 2000.0
DEFAULT_RBF_CONNECTOR_RRT_STEP_SIZE = 0.25
DEFAULT_RBF_CONNECTOR_RRT_GOAL_BIAS = 0.4
DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION = 16
DEFAULT_RBF_CONNECTOR_BRIDGE_BOXES = 0
DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN = 0
DEFAULT_RBF_CONNECTOR_PAVE_STEPS = 12
DEFAULT_RBF_CONNECTOR_PAVE_DEPTH = 64

DEFAULT_RBF_AUDIT_RESOLUTION = 16
DEFAULT_RBF_AUDIT_SEGMENT_STEP = 0.01
DEFAULT_RBF_FINAL_COLLISION_SHORTCUT = True
DEFAULT_RBF_FINAL_RRT_SIMPLIFY = True
DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS = 300.0
DEFAULT_RBF_FINAL_RRT_SIMPLIFY_MAX_ITERS = 50000
DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS = 4


def root_override_intervals(sbf: Any) -> list[Any]:
    return [sbf.Interval(float(lo), float(hi)) for lo, hi in D23_ROOT_INTERVALS]


def default_rbf_profile() -> dict[str, Any]:
    return {
        "profile": RBF_DEFAULT_PROFILE_NAME,
        "backend": RBF_DEFAULT_BACKEND,
        "grower_mode": RBF_DEFAULT_GROWER_MODE,
        "cache": {
            "mode": "robot_native_stateless_external_evidence",
            "default_depths": dict(ROBOT_DEFAULT_LECTDB_DEPTHS),
            "root": str(ROBOT_LECTDB_CACHE_ROOT),
            "max_depth": ROBOT_LECTDB_MAX_DEPTH,
        },
        "leaf_sweep": {
            "leaf_start_depth": DEFAULT_RBF_LEAF_START_DEPTH,
            "leaf_max_depth": DEFAULT_RBF_LEAF_MAX_DEPTH,
            "use_virtual_topology": True,
            "parallel_virtual_validation": True,
            "store_group_results": False,
            "obstacle_cluster_gap": 1000.0,
            "validation_batch_size": DEFAULT_RBF_VALIDATION_BATCH_SIZE,
            "leaf_threads": DEFAULT_RBF_THREADS,
        },
        "deep_refine": {
            "deep_max_boxes": DEFAULT_RBF_DEEP_MAX_BOXES,
            "deep_ffb_depth": DEFAULT_RBF_DEEP_FFB_DEPTH,
            "ffb_start_depth": DEFAULT_RBF_FFB_START_DEPTH,
            "domain_seed_cap": DEFAULT_RBF_DOMAIN_SEED_CAP,
            "domain_success_cap": DEFAULT_RBF_DOMAIN_SUCCESS_CAP,
            "domain_attempt_cap": DEFAULT_RBF_DOMAIN_ATTEMPT_CAP,
            "refine_timeout_ms": DEFAULT_RBF_REFINE_TIMEOUT_MS,
            "run_rrt_grower": True,
            "rrt_grower_extra_boxes": DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES,
            "rrt_grower_timeout_ms": DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS,
        },
        "connector": {
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
        },
        "query": {
            "strict_path_audit": True,
            "shortcut_boxes": False,
            "collision_shortcut": DEFAULT_RBF_FINAL_COLLISION_SHORTCUT,
            "final_rrt_simplify": DEFAULT_RBF_FINAL_RRT_SIMPLIFY,
            "final_rrt_simplify_timeout_ms": DEFAULT_RBF_FINAL_RRT_SIMPLIFY_TIMEOUT_MS,
            "final_rrt_simplify_attempts": DEFAULT_RBF_FINAL_RRT_SIMPLIFY_ATTEMPTS,
            "audit_resolution": DEFAULT_RBF_AUDIT_RESOLUTION,
            "audit_segment_step": DEFAULT_RBF_AUDIT_SEGMENT_STEP,
            "planning_time_excludes_audit": True,
        },
        "recommended_tradeoff": {
            "deep_max_boxes": DEFAULT_RBF_DEEP_MAX_BOXES,
            "validated_on": "Exp.4 shelf full seeds 0..7",
            "success_runs": "8/8",
            "median_build_s": 0.64,
            "median_planning_s": 1.68,
            "median_raw_segment_fraction": 0.419,
            "mean_route_length": 3.29,
        },
    }


def shelf_d23_rbf_profile() -> dict[str, Any]:
    profile = default_rbf_profile()
    profile["profile"] = RBF_SHELF_PROFILE_NAME
    profile["cache"] = {
        "mode": "d23_warm_external_evidence",
        "root": str(D23_CACHE_ROOT),
        "label": D23_CACHE_LABEL,
        "root_intervals": [[lo, hi] for lo, hi in D23_ROOT_INTERVALS],
    }
    return profile


def robot_lectdb_depth(robot_name: str) -> int:
    return int(ROBOT_DEFAULT_LECTDB_DEPTHS.get(str(robot_name), 20))


def robot_lectdb_label(robot_name: str, *, depth: int | None = None, envelope: str = "support_hull") -> str:
    actual_depth = robot_lectdb_depth(robot_name) if depth is None else int(depth)
    return f"tro2026_{robot_name}_p{actual_depth}_{envelope}_d{ROBOT_LECTDB_MAX_DEPTH}_canonical_native_stateless"


def robot_lectdb_path(robot_name: str, *, depth: int | None = None, envelope: str = "support_hull") -> Path:
    if str(robot_name) == "iiwa":
        return D23_CACHE_ROOT / D23_CACHE_LABEL
    return ROBOT_LECTDB_CACHE_ROOT / robot_lectdb_label(robot_name, depth=depth, envelope=envelope)


def robot_lectdb_profile(robot_name: str) -> dict[str, Any]:
    depth = robot_lectdb_depth(robot_name)
    if str(robot_name) == "iiwa":
        coverage = robot_sector_expanded_root_tuples("iiwa")
        return {
            "mode": "restricted_d23_external_evidence",
            "robot": "iiwa",
            "depth": 23,
            "root": str(D23_CACHE_ROOT),
            "label": D23_CACHE_LABEL,
            "path": str(D23_CACHE_ROOT / D23_CACHE_LABEL),
            "root_intervals": [[lo, hi] for lo, hi in D23_ROOT_INTERVALS],
            "coverage_intervals": [[lo, hi] for lo, hi in (coverage or D23_ROOT_INTERVALS)],
            "coverage_domain": "reflected_canonical_lect_root_sections",
            "canonical_mode": True,
            "symmetry_descriptor": CANONICAL_SYMMETRY_DESCRIPTOR,
        }
    return {
        "mode": "robot_native_stateless_external_evidence",
        "robot": str(robot_name),
        "depth": depth,
        "root": str(ROBOT_LECTDB_CACHE_ROOT),
        "label": robot_lectdb_label(robot_name, depth=depth),
        "path": str(robot_lectdb_path(robot_name, depth=depth)),
        "max_depth": ROBOT_LECTDB_MAX_DEPTH,
        "canonical_mode": True,
        "symmetry_descriptor": CANONICAL_SYMMETRY_DESCRIPTOR,
    }


def robot_root_override_tuples(robot_name: str) -> list[tuple[float, float]] | None:
    if str(robot_name) == "iiwa":
        return list(D23_ROOT_INTERVALS)
    return None


def robot_sector_expanded_root_tuples(robot_name: str, robot: Any | None = None) -> list[tuple[float, float]] | None:
    if str(robot_name) == "iiwa":
        intervals = list(D23_ROOT_INTERVALS)
    else:
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
