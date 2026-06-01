#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata, namespace_dict, proc_status, run_id, write_json  # noqa: E402
from experiments.common.marcucci_anchor_guard import validate_marcucci_query_artifact  # noqa: E402
from experiments.common.shelf_iiwa_cache import (  # noqa: E402
    DEFAULT_P18_CACHE_LABEL,
    ENDPOINT_AAFK,
    ENDPOINT_ANALYTICAL,
    ENDPOINT_CRITSAMPLE,
    ENDPOINT_HIFK,
    ENDPOINT_MC,
    LECT_SPLIT_AAFK_VOLUME_MIN,
    LECT_SPLIT_AAFK_VOLUME_MIN_DIM6,
    LECT_SPLIT_SUPPORT_HULL_VOLUME_MIN,
    LECT_SPLIT_ROUND_ROBIN,
    SUPPORTED_ENDPOINT_SOURCES,
    endpoint_enum,
    normalize_endpoint_source,
)
from safe_box_forest.experiments.sbf_old import common_sbf_config as sbf_config  # noqa: E402
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    add_common_sbf_args,
    apply_dim_mask,
    compute_inert_dim_mask,
    configure_external_evidence_reuse,
    configure_standalone_sbf,
    make_aafk_volume_min_dim6_split_policy,
    make_aafk_volume_min_split_policy,
    make_support_hull_volume_min_split_policy,
    mean,
    median,
    set_online_cache_backfill,
    set_rbf_envelope,
)
from safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import final_ompl_simplify_path  # noqa: E402

sbf = sbf_config.sbf


DEFAULT_AAFK_SCHEDULE_DEPTH = 50
LATENCY_PROFILE_STABLE = "stable"
LATENCY_PROFILE_BALANCED_LOW_LATENCY = "balanced_low_latency"
LATENCY_STAGE_SELECTION_AUTO = "auto"
LATENCY_STAGE_SELECTION_ZERO_REPAIR = "zero_repair"
LATENCY_STAGE_SELECTION_ACCEPT_REPAIR = "accept_repair"

BALANCED_LOW_LATENCY_STAGE_SEQUENCE: tuple[dict[str, Any], ...] = (
    {
        "stage_id": "seed",
        "quality_min_connected_boxes": 2,
        "post_connect_extra_boxes": 0,
        "post_connect_time_budget_ms": 0.0,
        "max_boxes": 16,
    },
    {
        "stage_id": "fast",
        "quality_min_connected_boxes": 64,
        "post_connect_extra_boxes": 0,
        "post_connect_time_budget_ms": 0.0,
        "max_boxes": 96,
    },
    {
        "stage_id": "balanced",
        "quality_min_connected_boxes": 128,
        "post_connect_extra_boxes": 16,
        "post_connect_time_budget_ms": 150.0,
        "max_boxes": 160,
    },
    {
        "stage_id": "quality",
        "quality_min_connected_boxes": 192,
        "post_connect_extra_boxes": 32,
        "post_connect_time_budget_ms": 300.0,
        "max_boxes": 224,
    },
    {
        "stage_id": "fallback",
        "quality_min_connected_boxes": 256,
        "post_connect_extra_boxes": 64,
        "post_connect_time_budget_ms": 450.0,
        "max_boxes": 320,
    },
)

BALANCED_LOW_LATENCY_STAGE_IDS: tuple[str, ...] = tuple(
    str(stage["stage_id"]) for stage in BALANCED_LOW_LATENCY_STAGE_SEQUENCE
)
IIWA_JOINT_NAMES: tuple[str, ...] = tuple(f"iiwa_joint_{index}" for index in range(1, 8))


def parse_csv_ints(text: str) -> list[int]:
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def parse_csv_floats(text: str) -> list[float]:
    return [float(item.strip()) for item in str(text).split(",") if item.strip()]


def parse_interval_pairs(text: str) -> list[tuple[float, float]]:
    values = [item.strip() for item in str(text).split(";") if item.strip()]
    out: list[tuple[float, float]] = []
    for index, value in enumerate(values):
        lo_text, sep, hi_text = value.partition(":")
        if not sep:
            raise ValueError(f"interval pair #{index + 1} must use lo:hi format, got {value!r}")
        lo = float(lo_text)
        hi = float(hi_text)
        if lo > hi:
            raise ValueError(f"interval pair #{index + 1} has lo > hi: {value!r}")
        out.append((lo, hi))
    return out


def root_intervals_override(args: argparse.Namespace) -> list[Any] | None:
    raw = str(getattr(args, "lect_root_intervals", "") or "").strip()
    if not raw:
        return None
    return [sbf.Interval(float(lo), float(hi)) for lo, hi in parse_interval_pairs(raw)]


def split_joint_metrics(diagnostics: dict[str, float], dof: int) -> list[dict[str, Any]]:
    total_split_count = sum(float(diagnostics.get(f"oracle.split_dim.{index}", 0.0)) for index in range(max(0, int(dof))))
    total_split_ms = float(diagnostics.get("profile.oracle.split_node.total_ms", 0.0))
    metrics: list[dict[str, Any]] = []
    for index in range(max(0, int(dof))):
        split_count = float(diagnostics.get(f"oracle.split_dim.{index}", 0.0))
        metrics.append({
            "joint_index": int(index),
            "joint_name": IIWA_JOINT_NAMES[index] if index < len(IIWA_JOINT_NAMES) else f"joint_{index}",
            "split_count": int(split_count),
            "split_fraction": float(split_count / total_split_count) if total_split_count > 0.0 else 0.0,
            "split_time_proxy_ms": float(total_split_ms * split_count / total_split_count) if total_split_count > 0.0 else 0.0,
            "split_width_sum": float(diagnostics.get(f"oracle.split_width_sum.{index}", 0.0)),
            "split_width_max": float(diagnostics.get(f"oracle.split_width_max.{index}", 0.0)),
        })
    return metrics


def summarize_joint_split_metrics(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    joint_count = max((len(row.get("build", {}).get("joint_split_metrics", [])) for row in rows), default=0)
    summary: list[dict[str, Any]] = []
    for index in range(joint_count):
        items = [
            row["build"]["joint_split_metrics"][index]
            for row in rows
            if index < len(row.get("build", {}).get("joint_split_metrics", []))
        ]
        if not items:
            continue
        summary.append({
            "joint_index": int(index),
            "joint_name": str(items[0].get("joint_name", f"joint_{index}")),
            "split_count_median": median(float(item.get("split_count", 0.0)) for item in items),
            "split_fraction_median": median(float(item.get("split_fraction", 0.0)) for item in items),
            "split_time_proxy_median_ms": median(float(item.get("split_time_proxy_ms", 0.0)) for item in items),
            "split_width_sum_median": median(float(item.get("split_width_sum", 0.0)) for item in items),
            "split_width_max_median": median(float(item.get("split_width_max", 0.0)) for item in items),
        })
    return summary


def balanced_low_latency_stage_ids(args: argparse.Namespace) -> tuple[str, ...]:
    raw_stage_ids = getattr(args, "stage_ids", None)
    if raw_stage_ids is None:
        return BALANCED_LOW_LATENCY_STAGE_IDS
    values = [item.strip() for item in str(raw_stage_ids).split(",") if item.strip()]
    return tuple(values) if values else BALANCED_LOW_LATENCY_STAGE_IDS


def parse_stage_override(name: str, values: list[Any], cast: Any, stage_ids: tuple[str, ...]) -> tuple[Any, ...]:
    expected = len(stage_ids)
    if not values:
        if expected != len(BALANCED_LOW_LATENCY_STAGE_SEQUENCE):
            raise ValueError(
                f"{name} override expects {expected} entries for stages {stage_ids}, got 0"
            )
        return tuple(cast(stage[name]) for stage in BALANCED_LOW_LATENCY_STAGE_SEQUENCE)
    if len(values) != expected:
        raise ValueError(
            f"{name} override expects {expected} entries for stages {stage_ids}, got {len(values)}"
        )
    return tuple(cast(value) for value in values)


def balanced_low_latency_stage_sequence(args: argparse.Namespace) -> tuple[dict[str, Any], ...]:
    stage_ids = balanced_low_latency_stage_ids(args)
    quality_values = parse_stage_override(
        "quality_min_connected_boxes",
        list(getattr(args, "latency_stage_quality_min_connected_boxes", []) or []),
        int,
        stage_ids,
    )
    extra_values = parse_stage_override(
        "post_connect_extra_boxes",
        list(getattr(args, "latency_stage_post_connect_extra_boxes", []) or []),
        int,
        stage_ids,
    )
    budget_values = parse_stage_override(
        "post_connect_time_budget_ms",
        list(getattr(args, "latency_stage_post_connect_time_budget_ms", []) or []),
        float,
        stage_ids,
    )
    max_box_values = parse_stage_override(
        "max_boxes",
        list(getattr(args, "latency_stage_max_boxes", []) or []),
        int,
        stage_ids,
    )
    stages: list[dict[str, Any]] = []
    for index, stage_id in enumerate(stage_ids):
        stages.append({
            "stage_id": str(stage_id),
            "quality_min_connected_boxes": int(quality_values[index]),
            "post_connect_extra_boxes": int(extra_values[index]),
            "post_connect_time_budget_ms": float(budget_values[index]),
            "max_boxes": int(max_box_values[index]),
        })
    return tuple(stages)


def latency_stage_selection_policy(args: argparse.Namespace) -> str:
    raw_policy = str(getattr(args, "latency_stage_selection_policy", LATENCY_STAGE_SELECTION_AUTO))
    if raw_policy == LATENCY_STAGE_SELECTION_AUTO:
        if bool(getattr(args, "require_no_repair", False)):
            return LATENCY_STAGE_SELECTION_ZERO_REPAIR
        return LATENCY_STAGE_SELECTION_ACCEPT_REPAIR
    if raw_policy not in {LATENCY_STAGE_SELECTION_ZERO_REPAIR, LATENCY_STAGE_SELECTION_ACCEPT_REPAIR}:
        raise ValueError(f"unsupported latency stage selection policy {raw_policy!r}")
    return raw_policy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one Shelf+IIWA SBF case with explicit LECT DB wiring.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset="support_hull_coverage",
        rbf_envelope="support_hull",
        threads=8,
        task_batch_size=8,
        max_boxes=5000,
        timeout_ms=60000.0,
        ffb_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        rbf_max_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        connector_pave_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        component_connect_ffb_max_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        rrt_goal_bias=0.20,
        intertree_goal_bias=0.25,
        component_connect_prob=0.35,
        quality_min_connected_boxes=64,
        post_connect_extra_boxes=0,
        post_connect_time_budget_ms=450.0,
        repair_timeout_ms=750.0,
    )
    parser.add_argument("--case-name", default="shelf_sbf_case")
    parser.add_argument("--out-json", type=Path, required=True)
    parser.add_argument("--database-path", type=Path, required=True)
    parser.add_argument("--seeds-list", default="0")
    parser.add_argument("--endpoint-source", choices=list(SUPPORTED_ENDPOINT_SOURCES), default=ENDPOINT_AAFK)
    parser.add_argument("--lect-split-policy", choices=[LECT_SPLIT_AAFK_VOLUME_MIN, LECT_SPLIT_AAFK_VOLUME_MIN_DIM6, LECT_SPLIT_SUPPORT_HULL_VOLUME_MIN, LECT_SPLIT_ROUND_ROBIN], default=LECT_SPLIT_AAFK_VOLUME_MIN)
    parser.add_argument("--use-external-evidence", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-materialization", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-scoring", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--warm-cache-label", default=DEFAULT_P18_CACHE_LABEL)
    parser.add_argument(
        "--lect-root-intervals",
        default="",
        help="Optional restricted LECT root intervals in 'lo:hi;lo:hi;...' format.",
    )
    parser.add_argument("--clean-active-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine-budget-ms", type=float, default=250.0)
    parser.add_argument("--corridor-refine-max-boxes", type=int, default=48)
    parser.add_argument("--corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--corridor-refine-passes", type=int, default=2)
    parser.add_argument("--corridor-refine-start-margin-ms", type=float, default=120.0)
    parser.add_argument("--corridor-refine-defer-labels", default="CS->LB")
    parser.add_argument("--post-audit-segment-step", type=float, default=0.04)
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.0)
    parser.add_argument("--require-no-repair", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--latency-profile",
        choices=[LATENCY_PROFILE_STABLE, LATENCY_PROFILE_BALANCED_LOW_LATENCY],
        default=LATENCY_PROFILE_STABLE,
    )
    parser.add_argument(
        "--latency-stage-selection-policy",
        choices=[LATENCY_STAGE_SELECTION_AUTO, LATENCY_STAGE_SELECTION_ZERO_REPAIR, LATENCY_STAGE_SELECTION_ACCEPT_REPAIR],
        default=LATENCY_STAGE_SELECTION_ZERO_REPAIR,
    )
    parser.add_argument("--latency-stage-early-stop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--latency-stage-quality-min-connected-boxes",
        type=parse_csv_ints,
        default=[],
        help="Optional comma-separated overrides for seed,fast,balanced,quality,fallback quality_min_connected_boxes.",
    )
    parser.add_argument(
        "--latency-stage-post-connect-extra-boxes",
        type=parse_csv_ints,
        default=[],
        help="Optional comma-separated overrides for seed,fast,balanced,quality,fallback post_connect_extra_boxes.",
    )
    parser.add_argument(
        "--latency-stage-post-connect-time-budget-ms",
        type=parse_csv_floats,
        default=[],
        help="Optional comma-separated overrides for seed,fast,balanced,quality,fallback post_connect_time_budget_ms.",
    )
    parser.add_argument(
        "--latency-stage-max-boxes",
        type=parse_csv_ints,
        default=[],
        help="Optional comma-separated overrides for seed,fast,balanced,quality,fallback max_boxes.",
    )
    return parser.parse_args()


def apply_latency_profile(local_args: argparse.Namespace) -> None:
    profile = str(getattr(local_args, "latency_profile", LATENCY_PROFILE_STABLE))
    if profile == LATENCY_PROFILE_STABLE:
        return
    if profile != LATENCY_PROFILE_BALANCED_LOW_LATENCY:
        raise ValueError(f"unsupported latency profile {profile!r}")

    explicit_stage = str(getattr(local_args, "latency_stage_id", "") or "")
    local_args.max_boxes = min(int(local_args.max_boxes), 2500)
    local_args.component_connect_candidate_limit = min(int(local_args.component_connect_candidate_limit), 1)
    local_args.connector_max_pairs_per_gap = min(int(local_args.connector_max_pairs_per_gap), 4)
    local_args.connector_pair_timeout_ms = min(float(local_args.connector_pair_timeout_ms), 80.0)
    if not explicit_stage:
        local_args.quality_min_connected_boxes = min(int(local_args.quality_min_connected_boxes), 16)
        local_args.post_connect_extra_boxes = 0
        local_args.post_connect_time_budget_ms = min(float(local_args.post_connect_time_budget_ms), 0.0)
    local_args.corridor_refine_budget_ms = min(float(local_args.corridor_refine_budget_ms), 80.0)
    local_args.corridor_refine_max_boxes = min(int(local_args.corridor_refine_max_boxes), 16)
    local_args.corridor_refine_boxes_per_query = min(int(local_args.corridor_refine_boxes_per_query), 8)
    local_args.corridor_refine_passes = min(int(local_args.corridor_refine_passes), 1)


def effective_case_args(args: argparse.Namespace) -> argparse.Namespace:
    local_args = argparse.Namespace(**vars(args))
    local_args.task_batch_size = max(1, int(local_args.task_batch_size))
    apply_latency_profile(local_args)
    return local_args


def uses_balanced_low_latency_stages(args: argparse.Namespace) -> bool:
    return str(getattr(args, "latency_profile", LATENCY_PROFILE_STABLE)) == LATENCY_PROFILE_BALANCED_LOW_LATENCY


def directory_size(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def configure_endpoint(cfg: Any, endpoint_source: str) -> None:
    cfg.endpoint_source.source = endpoint_enum(endpoint_source)


def configure_lect_split(cfg: Any, robot: Any, policy: str, max_depth: int, sample_nodes_per_depth: int = 8) -> None:
    override_intervals = list(getattr(cfg.database, "root_intervals_override", []) or [])
    split_root_intervals = override_intervals if override_intervals else None
    if policy == LECT_SPLIT_AAFK_VOLUME_MIN:
        cfg.database.split_policy = make_aafk_volume_min_split_policy(
            robot,
            int(max_depth),
            root_intervals=split_root_intervals,
            sample_nodes_per_depth=sample_nodes_per_depth,
        )
    elif policy == LECT_SPLIT_AAFK_VOLUME_MIN_DIM6:
        cfg.database.split_policy = make_aafk_volume_min_dim6_split_policy(
            robot,
            int(max_depth),
            root_intervals=split_root_intervals,
            sample_nodes_per_depth=sample_nodes_per_depth,
        )
    elif policy == LECT_SPLIT_SUPPORT_HULL_VOLUME_MIN:
        cfg.database.split_policy = make_support_hull_volume_min_split_policy(
            robot,
            int(max_depth),
            root_intervals=split_root_intervals,
            sample_nodes_per_depth=sample_nodes_per_depth,
        )
    elif policy == LECT_SPLIT_ROUND_ROBIN:
        descriptor = sbf.SplitPolicyDescriptor()
        descriptor.strategy = sbf.SplitStrategy.RoundRobin
        descriptor.min_width = 0.0
        descriptor.midpoint = True
        descriptor.deterministic_tie_break = True
        cfg.database.split_policy = descriptor
    else:
        raise ValueError(f"unsupported LECT split policy {policy!r}")


def effective_lect_schedule_depth(args: argparse.Namespace) -> int:
    return max(
        int(args.rbf_max_depth),
        int(args.ffb_depth),
        int(args.connector_pave_depth),
    )


def case_config(args: argparse.Namespace, robot: Any, seed: int) -> Any:
    local_args = effective_case_args(args)
    local_args.endpoint_source = normalize_endpoint_source(local_args.endpoint_source)
    local_args.rbf_cache_root = Path(args.database_path).parent
    local_args.rbf_cache_label = Path(args.database_path).name
    cfg = configure_standalone_sbf(local_args, seed=seed, preset=str(args.preset), robot=robot)
    override_intervals = root_intervals_override(local_args)
    if override_intervals is not None:
        cfg.database.root_intervals_override = list(override_intervals)
    if bool(getattr(local_args, "ffb_auto_mask_inert", False)):
        mask_root = override_intervals if override_intervals is not None else list(robot.joint_limits().limits)
        inert_mask = compute_inert_dim_mask(robot, mask_root)
        if inert_mask and any(v == 0 for v in inert_mask):
            apply_dim_mask(cfg, inert_mask)
    set_rbf_envelope(cfg, str(args.rbf_envelope), local_args)
    configure_endpoint(cfg, str(args.endpoint_source))
    configure_lect_split(
        cfg,
        robot,
        str(args.lect_split_policy),
        effective_lect_schedule_depth(local_args),
        sample_nodes_per_depth=int(getattr(local_args, "aafk_sample_nodes_per_depth", 8)),
    )
    cfg.database.path = str(args.database_path)
    cfg.database.create_if_missing = True
    cfg.database.read_only = False
    cfg.database.verify_identity = True
    cfg.database.replay_journal = True
    cfg.database.checkpoint_after_build = False
    cfg.database.max_tree_depth = int(args.rbf_max_tree_depth)
    cfg.database.online_cache.max_nodes = int(args.rbf_online_cache_max_nodes)
    cfg.database.online_cache.max_payload_bytes = int(args.rbf_online_cache_max_payload_bytes)
    set_online_cache_backfill(cfg, allow_database_backfill=False)
    return cfg


def box_count_with_status(boxes: Iterable[Any], status: Any) -> int:
    return sum(1 for box in boxes if box.safety_status == status)


def to_float_list(values: Iterable[Any]) -> list[float]:
    return [float(value) for value in values]


def post_collision_audit(robot: Any, obstacles: list[Any], path: list[list[float]], step: float) -> dict[str, Any]:
    if len(path) < 2:
        return {"passed": False, "failed_segment_index": -1, "checked_samples": 0}
    checked = 0
    segment_step = max(float(step), 1e-6)
    for index in range(len(path) - 1):
        start = [float(value) for value in path[index]]
        goal = [float(value) for value in path[index + 1]]
        distance = math.sqrt(sum((goal[dim] - start[dim]) ** 2 for dim in range(len(start))))
        samples = max(2, int(math.ceil(distance / segment_step)))
        for sample in range(samples + 1):
            alpha = sample / samples
            point = [
                (1.0 - alpha) * start[dim] + alpha * goal[dim]
                for dim in range(len(start))
            ]
            checked += 1
            if sbf.check_config_collision(robot, obstacles, point):
                return {
                    "passed": False,
                    "failed_segment_index": int(index),
                    "checked_samples": int(checked),
                    "failed_alpha": float(alpha),
                    "failed_point": point,
                    "failed_segment_start": start,
                    "failed_segment_goal": goal,
                    "failed_segment_length": float(distance),
                }
    return {"passed": True, "failed_segment_index": -1, "checked_samples": int(checked)}


def query_payload(query: Any, result: Any, wall_s: float) -> dict[str, Any]:
    waypoints = [to_float_list(waypoint) for waypoint in result.path]
    return {
        "name": query.label,
        "from": query.start_name,
        "to": query.goal_name,
        "ok": bool(result.success),
        "audit_passed": bool(result.audit_passed),
        "audit_status": str(result.audit_status).split(".")[-1],
        "t_s": float(wall_s),
        "planning_time_ms": float(result.query_time_ms),
        "audit_time_ms": float(result.audit_time_ms),
        "repair_time_ms": float(result.repair_time_ms),
        "repair_count": int(result.repair_count),
        "remaining_unsafe_assumptions": int(result.remaining_unsafe_assumptions),
        "failed_segment_index": int(result.failed_segment_index),
        "start_box_id": int(result.start_box_id),
        "goal_box_id": int(result.goal_box_id),
        "length": float(result.path_length) if result.success else 0.0,
        "certified_box_length": float(result.certified_box_length),
        "provisional_audited_length": float(result.provisional_audited_length),
        "segment_edge_length": float(result.segment_edge_length),
        "segment_edges_used": int(result.segment_edges_used),
        "box_sequence": [int(value) for value in result.box_sequence],
        "segment_edge_sequence": [int(value) for value in result.segment_edge_sequence],
        "waypoints": waypoints,
        "waypoint_count": len(waypoints),
    }


def refine_corridors(forest: Any, queries: list[Any], args: argparse.Namespace) -> tuple[float, int, int]:
    if not bool(args.corridor_refine):
        return 0.0, 0, 0
    budget_s = max(0.0, float(args.corridor_refine_budget_ms)) / 1000.0
    max_total = max(0, int(args.corridor_refine_max_boxes))
    per_query = max(1, int(args.corridor_refine_boxes_per_query))
    if budget_s <= 0.0 or max_total <= 0:
        return 0.0, 0, 0
    t0 = time.perf_counter()
    added_total = 0
    attempts = 0
    start_margin_s = max(0.0, float(args.corridor_refine_start_margin_ms)) / 1000.0
    defer_labels = {item.strip() for item in str(args.corridor_refine_defer_labels).split(",") if item.strip()}
    ordered_queries = sorted(queries, key=lambda item: (item.label in defer_labels, item.label))
    for _ in range(max(1, int(args.corridor_refine_passes))):
        pass_added = 0
        for query in ordered_queries:
            elapsed_s = time.perf_counter() - t0
            if added_total >= max_total or elapsed_s >= budget_s:
                break
            if attempts > 0 and budget_s - elapsed_s < start_margin_s:
                break
            quota = min(per_query, max_total - added_total)
            added = int(forest.refine_query_corridor(list(query.start), list(query.goal), quota))
            attempts += 1
            added_total += added
            pass_added += added
        if pass_added == 0 or added_total >= max_total or time.perf_counter() - t0 >= budget_s:
            break
    return time.perf_counter() - t0, added_total, attempts


def run_query(forest: Any, robot: Any, obstacles: list[Any], query: Any, args: argparse.Namespace) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    query_s = time.perf_counter() - query_t0
    row = query_payload(query, result, query_s)
    initial_post_audit = post_collision_audit(
        robot,
        obstacles,
        row["waypoints"],
        float(args.post_audit_segment_step),
    )
    row["initial_post_audit"] = initial_post_audit
    row["initial_post_audit_passed"] = bool(initial_post_audit.get("passed"))
    row["pre_bridge_ok"] = bool(result.success)
    row["bridge_progress"] = 0
    row["bridge_time_s"] = 0.0
    row["post_audit_source"] = "direct_query"
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
        row["initial_post_audit"] = initial_post_audit
        row["initial_post_audit_passed"] = bool(initial_post_audit.get("passed"))
        row["pre_bridge_ok"] = bool(result.success)
        row["bridge_progress"] = int(bridge_progress)
        row["bridge_time_s"] = float(bridge_s)
        row["post_audit_source"] = "retry_after_bridge"
    final_simplify = {
        "path": [list(point) for point in row.get("waypoints", [])],
        "path_length": float(row.get("length", 0.0)) if bool(row.get("ok")) else None,
        "query_s": 0.0,
        "applied": False,
        "reason": "not_eligible",
    }
    if bool(row.get("ok")) and bool(row.get("audit_passed")):
        final_simplify = final_ompl_simplify_path(
            sbf,
            robot,
            obstacles,
            [list(point) for point in row.get("waypoints", [])],
            segment_step=float(args.audit_segment_step),
            audit_segment_step=float(args.post_audit_segment_step),
            simplify_time_s=float(getattr(args, "final_ompl_simplify_time_s", 0.0)),
        )
        row["t_s"] = float(row.get("t_s", 0.0) or 0.0) + float(final_simplify["query_s"])
        row["waypoints"] = [list(point) for point in final_simplify["path"]]
        row["waypoint_count"] = len(row["waypoints"])
        if final_simplify["path_length"] is not None:
            row["length"] = float(final_simplify["path_length"])
    row["ompl_final_simplify_time_s"] = float(final_simplify["query_s"])
    row["ompl_final_simplify_applied"] = bool(final_simplify["applied"])
    row["ompl_final_simplify_reason"] = str(final_simplify["reason"])
    final_post_audit = post_collision_audit(
        robot,
        obstacles,
        row["waypoints"],
        float(args.post_audit_segment_step),
    )
    row["post_audit"] = final_post_audit
    row["post_audit_passed"] = bool(final_post_audit.get("passed"))
    row["post_audit_mismatch"] = bool(row["audit_passed"]) and not bool(row["post_audit_passed"])
    return row


def metadata_payload(cfg: Any, args: argparse.Namespace) -> dict[str, Any]:
    split_policy = cfg.database.split_policy
    stage_sequence = balanced_low_latency_stage_sequence(args) if uses_balanced_low_latency_stages(args) else ()
    root_override = [
        [float(interval.lo), float(interval.hi)]
        for interval in list(getattr(cfg.database, "root_intervals_override", []) or [])
    ]
    return {
        "case_name": str(args.case_name),
        "preset": str(args.preset),
        "endpoint_source": str(args.endpoint_source),
        "endpoint_source_raw": str(cfg.endpoint_source.source).split(".")[-1],
        "envelope": str(args.rbf_envelope),
        "envelope_type_raw": str(cfg.envelope_type.type).split(".")[-1],
        "lect_split_policy": str(args.lect_split_policy),
        "aafk_sample_nodes_per_depth": int(getattr(args, "aafk_sample_nodes_per_depth", 8)),
        "split_policy_descriptor": sbf.split_policy_descriptor(split_policy),
        "split_policy_hash": int(sbf.split_policy_hash(split_policy)),
        "depth_dimensions": [int(value) for value in list(getattr(split_policy, "depth_dimensions", []))],
        "dimension_schedule_hash": str(getattr(split_policy, "dimension_schedule_hash", "")),
        "database_path": str(cfg.database.path),
        "external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "external_evidence_snapshot_path": str(getattr(cfg.database, "external_evidence_snapshot_path", "")),
        "external_evidence_use_snapshot": bool(getattr(cfg.database, "external_evidence_use_snapshot", False)),
        "external_evidence_auto_build_snapshot": bool(getattr(cfg.database, "external_evidence_auto_build_snapshot", True)),
        "external_evidence_materialization": bool(getattr(cfg.validation, "external_evidence_materialization", True)),
        "external_evidence_scoring": bool(getattr(cfg.validation, "external_evidence_scoring", True)),
        "external_evidence_backfill_active": bool(getattr(cfg.validation, "external_evidence_backfill_active", False)),
        "canonical_mode": bool(getattr(cfg.database, "canonical_mode", False)),
        "symmetry_descriptor": str(getattr(cfg.database, "symmetry_descriptor", "")),
        "root_intervals_override": root_override,
        "checkpoint_after_build": bool(getattr(cfg.database, "checkpoint_after_build", False)),
        "online_cache_allow_database_backfill": bool(getattr(cfg.database.online_cache, "allow_database_backfill", True)),
        "max_depth": int(args.rbf_max_depth),
        "lect_schedule_depth": effective_lect_schedule_depth(args),
        "ffb_depth": int(args.ffb_depth),
        "threads": int(args.threads),
        "task_batch_size": int(args.task_batch_size),
        "latency_profile": str(args.latency_profile),
        "latency_stage_selection_policy": latency_stage_selection_policy(args),
        "latency_stage_early_stop": bool(getattr(args, "latency_stage_early_stop", True)),
        "latency_stage_sequence": list(stage_sequence),
        "latency_stage_protocol": str(getattr(args, "latency_stage_protocol", "")),
        "latency_stage_id": str(getattr(args, "latency_stage_id", "")),
        "latency_stage_index": int(getattr(args, "latency_stage_index", -1)),
        "ffb_start_depth": int(getattr(args, "rbf_ffb_start_depth", 0)),
        "strict_path_audit": bool(getattr(args, "strict_path_audit", True)),
        "audit_resolution": int(getattr(args, "audit_resolution", 0)),
        "audit_segment_step": float(getattr(args, "audit_segment_step", 0.01)),
        "repair_on_audit_failure": bool(getattr(args, "repair_on_audit_failure", True)),
        "collision_shortcut": bool(getattr(args, "collision_shortcut", True)),
        "collision_shortcut_resolution": int(getattr(args, "collision_shortcut_resolution", 0)),
        "post_audit_segment_step": float(getattr(args, "post_audit_segment_step", 0.04)),
    }


def run_single_seed_attempt(args: argparse.Namespace,
                            robot: Any,
                            obstacles: list[Any],
                            coverage_seeds: list[list[float]],
                            queries: list[Any],
                            seed: int) -> dict[str, Any]:
    effective_args = effective_case_args(args)
    cfg = case_config(effective_args, robot, seed)
    database_path = Path(cfg.database.path)
    if bool(effective_args.clean_active_cache) and database_path.exists():
        shutil.rmtree(database_path)
    if bool(effective_args.use_external_evidence):
        warm_path = Path(effective_args.rbf_cache_root) / str(effective_args.warm_cache_label)
        if not warm_path.exists():
            raise FileNotFoundError(f"warm LECT cache does not exist: {warm_path}")
        configure_external_evidence_reuse(
            cfg,
            warm_path,
            effective_args,
            materialization=bool(effective_args.external_evidence_materialization),
            scoring=bool(effective_args.external_evidence_scoring),
            backfill_active=False,
        )
    cfg.database.create_if_missing = True

    print(
        f"[shelf-sbf-case] start case={effective_args.case_name} seed={seed} endpoint={effective_args.endpoint_source} "
        f"split={effective_args.lect_split_policy} external={bool(effective_args.use_external_evidence)}",
        flush=True,
    )
    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, coverage_seeds)
    refine_s, refine_added, refine_attempts = refine_corridors(forest, queries, effective_args)
    build_wall_s = time.perf_counter() - build_t0
    boxes = list(forest.boxes())
    query_rows = [run_query(forest, robot, obstacles, query, effective_args) for query in queries]
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    dof = len(coverage_seeds[0]) if coverage_seeds else len(queries[0].start) if queries else len(IIWA_JOINT_NAMES)
    joint_metrics = split_joint_metrics(diagnostics, int(dof))
    engine_audit_ok = all(bool(query["audit_passed"]) for query in query_rows)
    post_audit_ok = all(bool(query.get("post_audit_passed", False)) for query in query_rows)
    audit_ok = bool(engine_audit_ok and post_audit_ok)
    no_repair = all(int(query.get("repair_count", 0)) == 0 for query in query_rows)
    unique_box_count = int(len(boxes))
    has_boxes = unique_box_count > 0
    repair_only_zero_box = (not has_boxes) and any(int(query.get("repair_count", 0)) > 0 for query in query_rows)
    effective_ok = bool(audit_ok and has_boxes and (no_repair or not bool(effective_args.require_no_repair)))
    row = {
        "ok": effective_ok,
        "audit_ok": bool(audit_ok),
        "engine_audit_ok": bool(engine_audit_ok),
        "post_audit_ok": bool(post_audit_ok),
        "no_repair": bool(no_repair),
        "has_boxes": bool(has_boxes),
        "repair_only_zero_box": bool(repair_only_zero_box),
        "seed": int(seed),
        "scene": "shelf_iiwa_marcucci_combined",
        "metadata": metadata_payload(cfg, effective_args),
        "cache_path": str(database_path),
        "cache_bytes": directory_size(database_path),
        "build": {
            "wall_s": float(build_wall_s),
            "planning_s": float(profile.total_ms) / 1000.0 + float(refine_s),
            "maintenance_s": max(0.0, float(build_wall_s) - (float(profile.total_ms) / 1000.0 + float(refine_s))),
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "merge_ms": float(profile.merge_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "unique_box_count": unique_box_count,
            "certified_box_count": box_count_with_status(boxes, sbf.BoxSafetyStatus.CertifiedFree),
            "provisional_box_count": box_count_with_status(boxes, sbf.BoxSafetyStatus.ProvisionalFree),
            "strict_audit_required_box_count": sum(1 for box in boxes if bool(box.strict_audit_required)),
            "segment_edges": int(profile.segment_edges),
            "grow_adjacency_islands": int(getattr(profile, "grow_adjacency_islands", 0)),
            "grow_largest_island": int(getattr(profile, "grow_largest_island", 0)),
            "connector_islands_initial": int(diagnostics.get("connector.islands_initial", getattr(profile, "grow_adjacency_islands", 0))),
            "adjacency_islands": int(profile.adjacency_islands),
            "prebridge_time_s": float(refine_s),
            "prebridge_added_boxes": int(refine_added),
            "prebridge_attempts": int(refine_attempts),
            "latency_stage_id": str(getattr(effective_args, "latency_stage_id", "")),
            "latency_stage_index": int(getattr(effective_args, "latency_stage_index", -1)),
            "joint_split_metrics": joint_metrics,
            "diagnostics": diagnostics,
        },
        "queries": query_rows,
    }
    print(
        f"[shelf-sbf-case] done case={args.case_name} seed={seed} ok={row['ok']} "
        f"boxes={row['build']['unique_box_count']} passed={sum(1 for query in query_rows if query['audit_passed'] and query.get('post_audit_passed', False))}/{len(query_rows)} "
        f"repairs={sum(int(query.get('repair_count', 0)) for query in query_rows)} "
        f"external_hits={diagnostics.get('oracle.materialization_reused_external_evidence', 0.0):.0f}",
        flush=True,
    )
    del forest
    return row


def stage_args(base_args: argparse.Namespace, stage: dict[str, Any]) -> argparse.Namespace:
    local_args = argparse.Namespace(**vars(base_args))
    base_database_path = Path(base_args.database_path)
    stage_id = str(stage["stage_id"])
    local_args.case_name = f"{base_args.case_name}_{stage_id}"
    local_args.database_path = base_database_path.parent / f"{base_database_path.name}__{stage_id}"
    local_args.max_boxes = int(stage["max_boxes"])
    local_args.quality_min_connected_boxes = int(stage["quality_min_connected_boxes"])
    local_args.post_connect_extra_boxes = int(stage["post_connect_extra_boxes"])
    local_args.post_connect_time_budget_ms = float(stage["post_connect_time_budget_ms"])
    local_args.latency_stage_protocol = "legacy_low_box_progressive"
    local_args.latency_stage_id = stage_id
    local_args.latency_stage_index = int(stage.get("stage_index", -1))
    return local_args


def stage_record(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "stage_id": str(row["build"].get("latency_stage_id", "")),
        "stage_index": int(row["build"].get("latency_stage_index", -1)),
        "ok": bool(row.get("ok")),
        "audit_ok": bool(row.get("audit_ok")),
        "no_repair": bool(row.get("no_repair")),
        "boxes": int(row["build"].get("unique_box_count", 0)),
        "planning_s": float(row["build"].get("planning_s", 0.0)),
        "wall_s": float(row["build"].get("wall_s", 0.0)),
        "path_total": float(sum(float(query.get("length", 0.0)) for query in row.get("queries", []))),
        "repair_total": int(sum(int(query.get("repair_count", 0)) for query in row.get("queries", []))),
    }


def stage_success(row: dict[str, Any], selection_policy: str) -> bool:
    if not bool(row.get("audit_ok")) or not bool(row.get("has_boxes")):
        return False
    if selection_policy == LATENCY_STAGE_SELECTION_ACCEPT_REPAIR:
        return True
    return bool(row.get("no_repair"))


def stage_should_early_stop(args: argparse.Namespace, stage: dict[str, Any], row: dict[str, Any]) -> bool:
    if not bool(getattr(args, "latency_stage_early_stop", True)):
        return False
    selection_policy = latency_stage_selection_policy(args)
    if not stage_success(row, selection_policy):
        return False
    if selection_policy == LATENCY_STAGE_SELECTION_ACCEPT_REPAIR:
        return True
    return str(stage.get("stage_id", "")) in {"quality", "fallback"}


def stage_path_total(row: dict[str, Any]) -> float:
    return float(sum(float(query.get("length", 0.0)) for query in row.get("queries", [])))


def finalize_staged_row(rows: list[dict[str, Any]], selected_index: int) -> dict[str, Any]:
    row = rows[selected_index]
    attempted_rows = rows
    staged_records = [stage_record(item) for item in attempted_rows]
    build = row["build"]
    build["wall_s"] = float(sum(float(item["build"].get("wall_s", 0.0)) for item in attempted_rows))
    build["planning_s"] = float(sum(float(item["build"].get("planning_s", 0.0)) for item in attempted_rows))
    build["maintenance_s"] = float(sum(float(item["build"].get("maintenance_s", 0.0)) for item in attempted_rows))
    build["total_ms"] = float(sum(float(item["build"].get("total_ms", 0.0)) for item in attempted_rows))
    build["grow_ms"] = float(sum(float(item["build"].get("grow_ms", 0.0)) for item in attempted_rows))
    build["merge_ms"] = float(sum(float(item["build"].get("merge_ms", 0.0)) for item in attempted_rows))
    build["connector_ms"] = float(sum(float(item["build"].get("connector_ms", 0.0)) for item in attempted_rows))
    build["adjacency_ms"] = float(sum(float(item["build"].get("adjacency_ms", 0.0)) for item in attempted_rows))
    build["prebridge_time_s"] = float(sum(float(item["build"].get("prebridge_time_s", 0.0)) for item in attempted_rows))
    build["prebridge_added_boxes"] = int(sum(int(item["build"].get("prebridge_added_boxes", 0)) for item in attempted_rows))
    build["prebridge_attempts"] = int(sum(int(item["build"].get("prebridge_attempts", 0)) for item in attempted_rows))
    build["staged_attempt_count"] = int(len(attempted_rows))
    build["staged_selected_stage_id"] = str(build.get("latency_stage_id", ""))
    build["staged_records"] = staged_records
    row["metadata"]["latency_stage_protocol"] = "legacy_low_box_progressive"
    row["metadata"]["latency_stage_selected_id"] = str(build.get("latency_stage_id", ""))
    row["metadata"]["latency_stage_attempt_count"] = int(len(attempted_rows))
    return row


def run_seed(args: argparse.Namespace, robot: Any, obstacles: list[Any], coverage_seeds: list[list[float]], queries: list[Any], seed: int) -> dict[str, Any]:
    if not uses_balanced_low_latency_stages(args):
        return run_single_seed_attempt(args, robot, obstacles, coverage_seeds, queries, seed)

    stage_rows: list[dict[str, Any]] = []
    selection_policy = latency_stage_selection_policy(args)
    for stage_index, stage in enumerate(balanced_low_latency_stage_sequence(args)):
        stage_row = run_single_seed_attempt(
            stage_args(args, {**stage, "stage_index": stage_index}),
            robot,
            obstacles,
            coverage_seeds,
            queries,
            seed,
        )
        stage_rows.append(stage_row)
        if stage_should_early_stop(args, stage, stage_row):
            break

    successful_indices = [index for index, row in enumerate(stage_rows) if stage_success(row, selection_policy)]
    if successful_indices:
        best_index = min(
            successful_indices,
            key=lambda index: (
                stage_path_total(stage_rows[index]),
                int(stage_rows[index]["build"].get("unique_box_count", 0)),
                float(stage_rows[index]["build"].get("planning_s", 0.0)),
            ),
        )
        return finalize_staged_row(stage_rows, best_index)

    return finalize_staged_row(stage_rows, len(stage_rows) - 1)


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    query_names = [query["name"] for row in rows for query in row.get("queries", [])]
    query_names = list(dict.fromkeys(query_names))
    query_summary = []
    for name in query_names:
        items = [query for row in rows for query in row.get("queries", []) if query.get("name") == name]
        successes = [query for query in items if bool(query.get("ok"))]
        query_summary.append({
            "name": name,
            "sr": mean(1.0 if query.get("ok") else 0.0 for query in items),
            "audit_sr": mean(1.0 if query.get("audit_passed") else 0.0 for query in items),
            "query_time_median_s": median(float(query.get("t_s", 0.0)) for query in successes),
            "planning_time_median_ms": median(float(query.get("planning_time_ms", 0.0)) for query in items),
            "audit_time_median_ms": median(float(query.get("audit_time_ms", 0.0)) for query in items),
            "repair_time_median_ms": median(float(query.get("repair_time_ms", 0.0)) for query in items),
            "path_length_median": median(float(query.get("length", 0.0)) for query in successes),
            "repair_count_median": median(float(query.get("repair_count", 0.0)) for query in items),
            "ompl_final_simplify_applied_rate": mean(1.0 if query.get("ompl_final_simplify_applied") else 0.0 for query in items),
            "ompl_final_simplify_time_median_s": median(float(query.get("ompl_final_simplify_time_s", 0.0)) for query in items),
        })
    return {
        "ok": all(bool(row.get("ok")) for row in rows),
        "audit_ok": all(bool(row.get("audit_ok")) for row in rows),
        "n": len(rows),
        "build_wall_median_s": median(float(row["build"]["wall_s"]) for row in rows),
        "planning_median_s": median(float(row["build"]["planning_s"]) for row in rows),
        "unique_box_count_median": median(float(row["build"]["unique_box_count"]) for row in rows),
        "external_hits_mean": mean(float(row["build"]["diagnostics"].get("oracle.materialization_reused_external_evidence", 0.0)) for row in rows),
        "stored_endpoint_mean": mean(float(row["build"]["diagnostics"].get("oracle.materialization_stored_endpoint", 0.0)) for row in rows),
        "joint_split_metrics": summarize_joint_split_metrics(rows),
        "queries": query_summary,
    }


def main() -> int:
    args = parse_args()
    robot = sbf.load_iiwa14_robot()
    obstacles = sbf.make_combined_obstacles()
    coverage_seeds = [list(seed) for seed in sbf.make_coverage_seeds(include_extra_anchors=False)]
    queries = sbf.make_combined_queries()
    seed_values = parse_csv_ints(args.seeds_list)
    before = proc_status()
    rows = [run_seed(args, robot, obstacles, coverage_seeds, queries, seed) for seed in seed_values]
    payload = {
        "experiment": "shelf_sbf_case",
        "run_id": run_id("shelf_sbf_case"),
        "case_name": str(args.case_name),
        "ok": all(bool(row.get("ok")) for row in rows),
        "audit_ok": all(bool(row.get("audit_ok")) for row in rows),
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "proc_status_before": before,
        "proc_status_after": proc_status(),
        "summary": summarize(rows),
        "rows": rows,
        "notes": [
            "This backend is intentionally independent of safe_box_forest/experiments/rbf_only_shelf_iiwa_main.py.",
            "It consumes the Exp.3-style p18 LECT DB cache through external evidence when --use-external-evidence is enabled.",
            "AAFK endpoint source is represented by EndpointSource.IFK, matching the existing IFK_AA-backed helper metadata.",
            "A zero-box build is never considered ok, even if query repair later returns audit-passed paths.",
        ],
    }
    payload["anchor_validation"] = validate_marcucci_query_artifact(payload, artifact_path=args.out_json)
    write_json(args.out_json, payload)
    print(f"wrote {args.out_json} ok={payload['ok']}")
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
