from __future__ import annotations

import math
import os
import random
import shutil
import time
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
    DEFAULT_RBF_QUERY_BRIDGE_ADAPTIVE_FFB_DEPTHS,
    DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH,
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


@dataclass
class RBFLeafRRTOptions:
    seed: int = 0
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
    validation_batch_size: int = DEFAULT_RBF_VALIDATION_BATCH_SIZE
    ffb_start_depth: int = DEFAULT_RBF_FFB_START_DEPTH
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
    support_hull_direct_collision: bool = False
    endpoint_source: str = "ifk"
    unsafe_sampling_validation: bool = False
    use_external_evidence: bool = False
    external_evidence_live_retry_on_maybe: bool = False
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
    connector_segment_resolution: int = DEFAULT_RBF_CONNECTOR_SEGMENT_RESOLUTION
    connector_pave_max_chain: int = DEFAULT_RBF_CONNECTOR_PAVE_MAX_CHAIN
    connector_pave_steps: int = DEFAULT_RBF_CONNECTOR_PAVE_STEPS
    connector_pave_depth: int = DEFAULT_RBF_CONNECTOR_PAVE_DEPTH
    connector_adaptive_min_segment_fraction: float = DEFAULT_RBF_CONNECTOR_ADAPTIVE_MIN_SEGMENT_FRACTION
    query_bridge_pave_depth: int = DEFAULT_RBF_QUERY_BRIDGE_PAVE_DEPTH
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
    query_endpoint_anchor_before_bridge: bool = True
    query_bridge_labels: str = DEFAULT_RBF_QUERY_BRIDGE_LABELS
    query_bridge_segment_only_indices: str = ""
    query_bridge_force_indices: str = ""
    query_bridge_forced_attempts: int = 1
    query_bridge_direct_sample_step: float = 0.0
    query_bridge_repair_subdivisions: int = -1
    query_bridge_direct_max_length: float = 6.5
    query_bridge_to_main_island: bool = False
    query_bridge_to_main_direct_segment_max_length: float = 0.0
    query_bridge_to_main_box_corridor: bool = True
    endpoint_main_target_k: int = 8
    endpoint_main_coarse_step: float = 0.08
    endpoint_main_fine_step: float = 0.02
    endpoint_main_max_ffb_calls: int = 48
    endpoint_main_max_boxes: int = 64
    endpoint_main_adaptive_ffb_depths: str = "50,58,62"
    endpoint_main_residual_segment_max_length: float = 0.25
    endpoint_main_lateral_offset: float = 0.03
    endpoint_main_lateral_rounds: int = 2
    endpoint_main_face_epsilon: float = 1e-6
    allow_anchor_roots: bool = True
    use_priority_points: bool = True
    offline_query_agnostic_build: bool = True
    offline_random_anchors: bool = True
    offline_anchor_count: int = 16
    offline_anchor_candidate_count: int = 512
    offline_anchor_sampling: str = "random"
    offline_anchor_lca_lambda: float = 0.35
    offline_anchor_distance_mu: float = 0.10
    offline_shortcut_edges: int = 0
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
) -> Any:
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
            tail = list(sbf.aafk_volume_min_depth_schedule(robot, schedule_root, tail_depth, 8))
        elif len(tail) + 2 < int(max_depth):
            extra_depth = int(max_depth) - 2 - len(tail)
            extra = list(
                sbf.aafk_volume_min_depth_schedule(
                    robot,
                    schedule_root,
                    extra_depth,
                    8,
                )
            )
            tail.extend(int(dim) for dim in extra)
        schedule = [0, 0] + [int(dim) for dim in tail]
    elif root_intervals is None:
        schedule = list(sbf.aafk_volume_min_depth_schedule(robot, int(max_depth), 8))
    else:
        schedule = list(sbf.aafk_volume_min_depth_schedule(robot, list(root_intervals), int(max_depth), 8))
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


def generate_offline_anchor_points(
    robot: Any,
    obstacles: list[Any],
    options: RBFLeafRRTOptions,
) -> tuple[list[list[float]], dict[str, Any]]:
    count = max(0, int(options.offline_anchor_count))
    candidate_count = max(0, int(options.offline_anchor_candidate_count))
    if not bool(options.offline_random_anchors) or count <= 0 or candidate_count <= 0:
        return [], {
            "offline_anchor_candidates": 0,
            "offline_anchor_candidates_free": 0,
            "offline_anchor_roots_requested": 0,
            "offline_anchor_lca_depth_mean": math.nan,
            "offline_anchor_lca_depth_max": math.nan,
            "offline_anchor_min_distance_mean": math.nan,
        }
    coverage_root = _root_tuple_list(robot, options)
    tree_root = _active_tree_root_tuple_list(robot, options)
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
    )
    schedule = [int(dim) for dim in list(split_policy.depth_dimensions)]
    rng = random.Random((int(options.seed) + 1) * 1000003 + int(options.deep_max_boxes) * 9176)
    sampling = str(getattr(options, "offline_anchor_sampling", "random")).strip().lower()
    candidates: list[list[float]] = []
    halton_index = 1 + int(options.seed) * 1009 + int(options.deep_max_boxes) * 17
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
    else:
        cfg.endpoint_source.source = sbf.EndpointSource.IFK
    cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB if options.envelope == "link_aabb" else sbf.EnvelopeType.SupportHull
    if options.envelope != "link_aabb" and hasattr(cfg.envelope_type.support_hull_config, "direct_collision"):
        if hasattr(cfg.envelope_type.support_hull_config, "skip_aabb_broadphase"):
            cfg.envelope_type.support_hull_config.skip_aabb_broadphase = bool(options.support_hull_skip_aabb_broadphase)
        cfg.envelope_type.support_hull_config.direct_collision = bool(options.support_hull_direct_collision)
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
    )

    cfg.validation.enable_endpoint_evidence_cache = False
    cfg.validation.store_endpoint_evidence_cache = False
    cfg.validation.enable_worker_shared_endpoint_cache = False
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
        start = query_point(robot, query.start, canonicalize_queries)
        goal = query_point(robot, query.goal, canonicalize_queries)
        previous_active_query = os.environ.get("RBF_ACTIVE_QUERY_INDEX")
        os.environ["RBF_ACTIVE_QUERY_INDEX"] = str(query_index)
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
        query_ms = float(result.query_time_ms)
        simplify_ms = float(getattr(result, "final_simplify_time_ms", 0.0))
        solve_ms = max(0.0, query_ms - simplify_ms)
        rows.append({
            "label": query.label,
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
            "segment_fraction": (segment_length / raw_path_length) if bool(result.success) and raw_path_length > 1e-12 else math.nan,
            "box_sequence_len": len(list(result.box_sequence)),
            "segment_edges_used": int(result.segment_edges_used),
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
        if not labels and not adaptive_all and not to_main_enabled:
            return 0.0, 0, 0, {}, {}
    else:
        labels = set()
    t0 = time.perf_counter()
    added_total = 0
    attempts = 0
    timing_by_label: dict[str, float] = {}
    added_by_label: dict[str, int] = {}
    selected: list[tuple[str, list[float], list[float]]] = []
    force_selected_indices: set[int] = set()
    query_items: list[tuple[str, list[float], list[float]]] = []
    for raw in queries:
        query = query_spec(raw)
        start = query_point(robot, query.start, bool(options.canonicalize_queries))
        goal = query_point(robot, query.goal, bool(options.canonicalize_queries))
        query_items.append((str(query.label), start, goal))
    if bool(getattr(options, "query_endpoint_anchor_before_bridge", True)) and hasattr(forest, "anchor_query_endpoint_box"):
        anchor_t0 = time.perf_counter()
        anchor_added = 0
        anchor_attempts = 0
        for _label, start, goal in query_items:
            for point in (start, goal):
                anchor_attempts += 1
                box_id = int(forest.anchor_query_endpoint_box(point))
                if box_id >= 0:
                    anchor_added += 1
        timing_by_label["__endpoint_anchor__"] = time.perf_counter() - anchor_t0
        added_by_label["__endpoint_anchor__"] = anchor_added
        added_total += anchor_added
        attempts += anchor_attempts

    def forest_islands() -> list[list[int]]:
        boxes = list(forest.boxes())
        adjacency = dict(forest.adjacency())
        box_ids = [int(box.id) for box in boxes]
        box_id_set = set(box_ids)
        seen: set[int] = set()
        islands: list[list[int]] = []
        for box_id in box_ids:
            if box_id in seen:
                continue
            stack = [box_id]
            seen.add(box_id)
            island: list[int] = []
            while stack:
                current = stack.pop()
                island.append(current)
                for neighbor in adjacency.get(current, []):
                    neighbor_id = int(neighbor)
                    if neighbor_id not in box_id_set or neighbor_id in seen:
                        continue
                    seen.add(neighbor_id)
                    stack.append(neighbor_id)
            islands.append(island)
        return islands

    def closest_point_in_box(box: Any, point: list[float]) -> list[float]:
        out: list[float] = []
        for interval, value in zip(list(box.joint_intervals), point, strict=True):
            out.append(min(max(float(value), float(interval.lo)), float(interval.hi)))
        return out

    def point_in_box(box: Any, point: list[float], tolerance: float = 1e-9) -> bool:
        intervals = list(box.joint_intervals)
        if len(intervals) != len(point):
            return False
        for interval, value in zip(intervals, point, strict=True):
            if float(value) < float(interval.lo) - tolerance or float(value) > float(interval.hi) + tolerance:
                return False
        return True

    def locate_point_box_id(point: list[float]) -> int:
        candidates = [
            box for box in list(forest.boxes())
            if point_in_box(box, point)
        ]
        if not candidates:
            return -1
        candidates.sort(key=lambda box: float(getattr(box, "volume", 0.0)), reverse=True)
        return int(candidates[0].id)

    if bool(getattr(options, "query_bridge_to_main_island", False)):
        main_t0 = time.perf_counter()
        main_added = 0
        main_attempts = 0
        islands = forest_islands()
        if islands:
            islands.sort(key=len, reverse=True)
            main_ids = set(int(item) for item in islands[0])
            boxes_by_id = {int(box.id): box for box in list(forest.boxes())}
            main_boxes = [boxes_by_id[box_id] for box_id in main_ids if box_id in boxes_by_id]
            endpoints: list[tuple[str, list[float]]] = []
            for label, start, goal in query_items:
                endpoints.append((f"{label}:start", start))
                endpoints.append((f"{label}:goal", goal))
            for endpoint_label, point in endpoints:
                point_box_id = locate_point_box_id(point)
                if point_box_id in main_ids or not main_boxes:
                    continue
                best_target: list[float] | None = None
                best_dist2 = math.inf
                for box in main_boxes:
                    target = closest_point_in_box(box, point)
                    dist2 = sum((float(lhs) - float(rhs)) ** 2 for lhs, rhs in zip(point, target, strict=True))
                    if dist2 < best_dist2:
                        best_dist2 = dist2
                        best_target = target
                if best_target is None or best_dist2 <= 1e-18:
                    continue
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
                        for item in str(getattr(options, "endpoint_main_adaptive_ffb_depths", "50,58,62")).split(",")
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
                if added > 0:
                    islands = forest_islands()
                    islands.sort(key=len, reverse=True)
                    main_ids = set(int(item) for item in islands[0])
                    boxes_by_id = {int(box.id): box for box in list(forest.boxes())}
                    main_boxes = [boxes_by_id[box_id] for box_id in main_ids if box_id in boxes_by_id]
        timing_by_label["__endpoint_to_main_island__"] = time.perf_counter() - main_t0
        added_by_label["__endpoint_to_main_island__"] = main_added
        added_total += main_added
        attempts += main_attempts
    if not labels and not adaptive_all:
        return time.perf_counter() - t0, added_total, attempts, timing_by_label, added_by_label
    for label, start, goal in query_items:
        if labels and label not in labels:
            continue
        if labels or adaptive_all:
            probe = forest.query(start, goal)
            direct = point_distance(start, goal)
            raw_length = float(getattr(probe, "raw_path_length", getattr(probe, "path_length", math.inf)))
            segment_length = float(getattr(probe, "segment_edge_length", 0.0))
            segment_fraction = (
                segment_length / raw_length
                if raw_length > 1e-12 and math.isfinite(raw_length)
                else math.inf
            )
            segment_ok = segment_fraction <= float(getattr(options, "query_bridge_accept_segment_fraction", 0.0))
            path_length_value = float(getattr(probe, "path_length", math.inf))
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
            if bool(probe.success) and bool(probe.audit_passed) and segment_ok and short_enough:
                continue
            if (
                bool(getattr(options, "query_bridge_to_main_island", False)) and
                bool(probe.success) and
                bool(probe.audit_passed) and
                segment_ok and
                short_enough
            ):
                continue
        selected_index = len(selected)
        selected.append((label, start, goal))
        if to_main_enabled:
            force_selected_indices.add(selected_index)

    if (
        selected and
        hasattr(forest, "bridge_queries") and
        (len(selected) > 1 or force_selected_indices)
    ):
        starts = [item[1] for item in selected]
        goals = [item[2] for item in selected]
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
        }
        if int(options.query_bridge_forced_attempts) > 1:
            env_updates["RBF_QUERY_BRIDGE_FORCED_ATTEMPTS"] = str(int(options.query_bridge_forced_attempts))
        if float(options.query_bridge_direct_sample_step) > 0.0:
            env_updates["RBF_QUERY_BRIDGE_DIRECT_SAMPLE_STEP"] = str(float(options.query_bridge_direct_sample_step))
        if int(options.query_bridge_repair_subdivisions) >= 0:
            env_updates["RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS"] = str(int(options.query_bridge_repair_subdivisions))
        if float(options.query_bridge_direct_max_length) > 0.0:
            env_updates["RBF_QUERY_BRIDGE_DIRECT_MAX_LENGTH"] = str(float(options.query_bridge_direct_max_length))
        previous_env = {name: os.environ.get(name) for name in env_updates}
        for name, value in env_updates.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
        try:
            added_values = [int(value) for value in forest.bridge_queries(starts, goals)]
        finally:
            for name, previous_value in previous_env.items():
                if previous_value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = previous_value
        elapsed = time.perf_counter() - t0
        timing_by_label["__batch_total__"] = elapsed
        for (label, _start, _goal), added in zip(selected, added_values, strict=True):
            added_by_label[label] = added_by_label.get(label, 0) + added
            added_total += added
        attempts += len(selected)
        return elapsed, added_total, attempts, timing_by_label, added_by_label

    for label, start, goal in selected:
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
    offline_t0 = time.perf_counter()
    anchor_select_t0 = time.perf_counter()
    offline_anchor_points, offline_anchor_select_metrics = generate_offline_anchor_points(
        robot,
        list(obstacles),
        options,
    )
    offline_anchor_select_s = time.perf_counter() - anchor_select_t0
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
    if int(options.offline_shortcut_edges) > 0 and hasattr(forest, "add_offline_shortcut_edges"):
        offline_shortcut_edges_added = int(forest.add_offline_shortcut_edges(
            int(options.offline_shortcut_edges),
            int(options.offline_shortcut_candidate_limit),
            float(options.offline_shortcut_min_gain_ratio),
            float(options.offline_shortcut_max_segment_length),
        ))
    offline_shortcut_s = time.perf_counter() - offline_shortcut_t0
    offline_build_wall_s = time.perf_counter() - offline_t0
    build_final_boxes = int(build.profile.final_boxes)
    build_segment_edges = int(build.profile.segment_edges)
    corridor_refine_s, corridor_refine_added, corridor_refine_attempts = refine_corridors(
        forest,
        robot,
        query_list,
        options,
    )
    after_corridor_boxes = len(list(forest.boxes()))
    after_corridor_segment_edges = len(list(forest.segment_edges()))
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
    final_boxes = len(list(forest.boxes()))
    final_segment_edges = len(list(forest.segment_edges()))
    final_adjacency_islands = forest_adjacency_island_count(forest)
    qrows = query_rows(
        forest,
        robot,
        query_list,
        obstacles=list(obstacles),
        audit_step=float(options.audit_segment_step),
        audit_collision_tolerance=float(options.audit_collision_tolerance),
        canonicalize_queries=bool(options.canonicalize_queries),
    )
    build_for_diagnostics = forest.last_build_profile() if hasattr(forest, "last_build_profile") else build
    successes = [row for row in qrows if bool(row["audit_passed"])]
    total_len = sum(float(row["raw_path_length"]) for row in successes if math.isfinite(float(row["raw_path_length"])))
    total_seg = sum(float(row["segment_edge_length"]) for row in successes)
    diagnostics = {str(k): float(v) for k, v in dict(build_for_diagnostics.diagnostics).items()}
    qroot_pairs_total = int(diagnostics.get("leaf_refine.qroot_pairs_total", 0.0))
    qroot_uncovered_endpoints = int(diagnostics.get("leaf_refine.qroot_uncovered_endpoints", 0.0))
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
    graph_query_s = graph_solve_s + graph_simplify_s
    online_solve_s = online_adaptation_s + graph_solve_s
    online_simplify_s = graph_simplify_s
    query_s = online_solve_s + online_simplify_s
    query_count = max(1, len(qrows))
    query_bridge_per_query_s = float(query_bridge_s) / query_count
    online_per_query_s = query_s / query_count
    online_solve_per_query_s = online_solve_s / query_count
    online_simplify_per_query_s = online_simplify_s / query_count
    offline_segment_edges_added = int(getattr(build.profile, "segment_edges_added", 0))
    offline_box_edges_added = int(getattr(build.profile, "bridge_boxes_added", 0))
    amortized = {
        f"amortized_s_k{k}": offline_build_s / float(k) + online_per_query_s
        for k in (1, 5, 10, 20, 50)
    }
    external_hits = max(
        diagnostics.get("oracle.materialization_reused_external_evidence", 0.0),
        diagnostics.get("leaf_sweep.worker_oracle.materialization_reused_external_evidence", 0.0),
        diagnostics.get("grower.worker_oracle.materialization_reused_external_evidence", 0.0),
    )
    return {
        "case": options.case_label,
        "seed": int(options.seed),
        "deep_max_boxes": int(options.deep_max_boxes),
        "endpoint_source": str(options.endpoint_source),
        "unsafe_sampling_validation": bool(options.unsafe_sampling_validation),
        "status": "ok" if len(successes) == len(qrows) else "partial",
        "success_count": len(successes),
        "query_count": len(qrows),
        "planning_s": offline_build_s + query_s,
        "build_s": offline_build_s,
        "offline_build_s": offline_build_s,
        "offline_build_profile_s": offline_build_profile_s,
        "query_s": query_s,
        "online_s": query_s,
        "online_batch_s": query_s,
        "online_adaptation_s": online_adaptation_s,
        "online_solve_s": online_solve_s,
        "online_simplify_s": online_simplify_s,
        "online_per_query_s": online_per_query_s,
        "online_solve_per_query_s": online_solve_per_query_s,
        "online_simplify_per_query_s": online_simplify_per_query_s,
        "graph_query_s": graph_query_s,
        "graph_solve_s": graph_solve_s,
        "graph_simplify_s": graph_simplify_s,
        "graph_query_per_query_s": graph_query_s / query_count,
        "graph_solve_per_query_s": graph_solve_s / query_count,
        "graph_simplify_per_query_s": graph_simplify_s / query_count,
        **amortized,
        "build_wall_s": offline_build_wall_s,
        "offline_anchor_select_s": float(offline_anchor_select_s),
        "offline_query_agnostic_build": bool(options.offline_query_agnostic_build),
        "qroot_pairs_total": qroot_pairs_total,
        "qroot_uncovered_endpoints": qroot_uncovered_endpoints,
        "offline_anchor_candidates": int(offline_anchor_select_metrics.get("offline_anchor_candidates", 0)),
        "offline_anchor_candidates_free": int(offline_anchor_select_metrics.get("offline_anchor_candidates_free", 0)),
        "offline_anchor_roots_requested": int(offline_anchor_select_metrics.get("offline_anchor_roots_requested", 0)),
        "offline_anchor_roots_added": int(diagnostics.get("leaf_refine.offline_anchor_roots_added", 0.0)),
        "offline_anchor_lca_depth_mean": float(offline_anchor_select_metrics.get("offline_anchor_lca_depth_mean", math.nan)),
        "offline_anchor_lca_depth_max": float(offline_anchor_select_metrics.get("offline_anchor_lca_depth_max", math.nan)),
        "offline_anchor_min_distance_mean": float(offline_anchor_select_metrics.get("offline_anchor_min_distance_mean", math.nan)),
        "offline_anchor_box_volume_mean": float(diagnostics.get("leaf_refine.offline_anchor_box_volume_mean", 0.0)),
        "offline_anchor_box_volume_max": float(diagnostics.get("leaf_refine.offline_anchor_box_volume_max", 0.0)),
        "offline_shortcut_s": float(offline_shortcut_s),
        "offline_shortcut_edges_requested": int(options.offline_shortcut_edges),
        "offline_shortcut_edges_added": int(offline_shortcut_edges_added),
        "offline_shortcut_candidates": int(diagnostics.get("offline_shortcut.candidates", 0.0)),
        "offline_shortcut_tested_pairs": int(diagnostics.get("offline_shortcut.tested_pairs", 0.0)),
        "offline_shortcut_box_corridor_edges_added": int(diagnostics.get("offline_shortcut.box_corridor_edges_added", 0.0)),
        "offline_shortcut_segment_edges_added": int(diagnostics.get("offline_shortcut.segment_edges_added", 0.0)),
        "offline_shortcut_pave_boxes_added": int(diagnostics.get("offline_shortcut.pave_boxes_added", 0.0)),
        "offline_shortcut_pave_fail": int(diagnostics.get("offline_shortcut.pave_fail", 0.0)),
        "offline_box_edges_added": offline_box_edges_added,
        "offline_segment_edges_added": offline_segment_edges_added,
        "offline_islands_before": int(diagnostics.get("leaf_refine.offline_anchor_islands_before", 0.0)),
        "offline_islands_after": int(build.profile.adjacency_islands),
        "leaf_sweep_s": float(build.leaf_sweep_ms) / 1000.0,
        "deep_refine_s": float(build.deep_refine_ms) / 1000.0,
        "rrt_grower_s": float(getattr(build, "rrt_grower_ms", 0.0)) / 1000.0,
        "connector_s": float(build.connector_ms) / 1000.0,
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
        "audit_s": sum(float(row["audit_ms"]) for row in qrows) / 1000.0,
        "path_length_mean": mean(row["path_length"] for row in successes),
        "raw_segment_fraction": (total_seg / total_len) if total_len > 1e-12 else math.nan,
        "leaf_free_count": int(build.leaf_free_count),
        "leaf_collision_count": int(build.leaf_collision_count),
        "deep_boxes_added": int(build.deep_boxes_added),
        "rrt_grower_boxes_added": int(getattr(build, "rrt_grower_boxes_added", 0)),
        "build_final_boxes": build_final_boxes,
        "build_segment_edges": build_segment_edges,
        "after_corridor_boxes": int(after_corridor_boxes),
        "after_corridor_segment_edges": int(after_corridor_segment_edges),
        "query_bridge_boxes_added_observed": int(final_boxes - after_corridor_boxes),
        "query_bridge_segment_edges_added_observed": int(final_segment_edges - after_corridor_segment_edges),
        "final_boxes": int(final_boxes),
        "final_segment_edges": int(final_segment_edges),
        "final_adjacency_islands": int(final_adjacency_islands),
        "segment_edges": int(final_segment_edges),
        "adjacency_islands": int(build.profile.adjacency_islands),
        "external_hits": external_hits,
        "database_root_intervals": interval_pairs(forest.database_root_intervals()),
        "database_coverage_intervals": interval_pairs(forest.database_coverage_intervals()),
        "queries": qrows,
        "diagnostics": diagnostics,
    }
