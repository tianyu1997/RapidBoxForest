from __future__ import annotations

import math
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from experiments.common.metrics import mean, median
from experiments.common.rbf_defaults import (
    CANONICAL_SYMMETRY_DESCRIPTOR,
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
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_RRT_GROWER_EXTRA_BOXES,
    DEFAULT_RBF_RRT_GROWER_TIMEOUT_MS,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_REFINE_TIMEOUT_MS,
    DEFAULT_RBF_THREADS,
    DEFAULT_RBF_VALIDATION_BATCH_SIZE,
    root_override_intervals,
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
    rbf_max_depth: int = 40
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
    validation_batch_size: int = DEFAULT_RBF_VALIDATION_BATCH_SIZE
    ffb_start_depth: int = DEFAULT_RBF_FFB_START_DEPTH
    audit_resolution: int = DEFAULT_RBF_AUDIT_RESOLUTION
    audit_segment_step: float = DEFAULT_RBF_AUDIT_SEGMENT_STEP
    use_virtual_topology: bool = True
    parallel_virtual_validation: bool = False
    leaf_threads: int = 1
    envelope: str = "support_hull"
    endpoint_source: str = "ifk"
    unsafe_sampling_validation: bool = False
    use_external_evidence: bool = False
    external_evidence_path: Path | None = None
    external_evidence_verify_identity: bool = True
    use_shelf_root_override: bool = False
    root_override_tuples: list[tuple[float, float]] | None = None
    coverage_override_tuples: list[tuple[float, float]] | None = None
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
    connector_pave_fill_gaps: bool = False
    connector_pave_require_connected_chain: bool = False
    allow_anchor_roots: bool = True
    use_priority_points: bool = True
    canonicalize_queries: bool = True


def query_spec(query: Any) -> QuerySpec:
    if isinstance(query, QuerySpec):
        return query
    if isinstance(query, dict):
        return QuerySpec(
            label=str(query.get("label", query.get("name", "query"))),
            start=[float(value) for value in query.get("canonical_start", query["start"])],
            goal=[float(value) for value in query.get("canonical_goal", query["goal"])],
            actual_start=[float(value) for value in query.get("actual_start", query.get("start", []))] or None,
            actual_goal=[float(value) for value in query.get("actual_goal", query.get("goal", []))] or None,
        )
    return QuerySpec(
        label=str(getattr(query, "label", getattr(query, "name", "query"))),
        start=[float(value) for value in getattr(query, "start")],
        goal=[float(value) for value in getattr(query, "goal")],
        actual_start=getattr(query, "actual_start", None),
        actual_goal=getattr(query, "actual_goal", None),
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


def canonical_priority_points(robot: Any, queries: Iterable[Any], canonicalize: bool = True) -> list[list[float]]:
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


def make_aafk_split_policy(robot: Any, max_depth: int, root_intervals: Iterable[Any] | None = None) -> Any:
    if root_intervals is None:
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
    root_intervals = None
    if options.root_override_tuples is not None:
        root_intervals = [sbf.Interval(float(lo), float(hi)) for lo, hi in options.root_override_tuples]
        cfg.database.root_intervals_override = root_intervals
    elif options.use_shelf_root_override:
        root_intervals = root_override_intervals(sbf)
        cfg.database.root_intervals_override = root_intervals
    if options.coverage_override_tuples is not None:
        cfg.database.coverage_intervals_override = [
            sbf.Interval(float(lo), float(hi)) for lo, hi in options.coverage_override_tuples
        ]
    cfg.database.split_policy = make_aafk_split_policy(robot, int(options.rbf_max_depth), root_intervals)

    cfg.validation.enable_endpoint_evidence_cache = False
    cfg.validation.store_endpoint_evidence_cache = False
    cfg.validation.enable_worker_shared_endpoint_cache = False
    cfg.validation.external_evidence_backfill_active = False
    cfg.validation.external_evidence_materialization = True
    cfg.validation.external_evidence_scoring = True
    if options.use_external_evidence and options.external_evidence_path is not None:
        cfg.database.external_evidence_path = str(options.external_evidence_path)
        cfg.database.verify_identity = bool(options.external_evidence_verify_identity)
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
    cfg.connector.pave.find_free_box.skip_to_depth = int(options.ffb_start_depth)
    cfg.connector.pave.find_free_box.split_reserved_leaf = True
    cfg.connector.pave.find_free_box.split_unknown_leaf = True
    cfg.connector.pave.find_free_box.reject_seed_collision = False
    cfg.connector.pave.fill_gaps = bool(options.connector_pave_fill_gaps)
    cfg.connector.pave.require_connected_chain = bool(options.connector_pave_require_connected_chain)

    cfg.query.strict_path_audit = True
    cfg.query.audit_resolution = max(int(options.audit_resolution), int(options.connector_segment_resolution))
    cfg.query.audit_segment_step = float(options.audit_segment_step)
    cfg.query.shortcut_boxes = False
    cfg.query.collision_shortcut = False
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
    return cfg


def reflect_path_to_actual(query: QuerySpec, path: list[list[float]]) -> list[list[float]]:
    if query.actual_start is None:
        return path
    actual_start = [float(value) for value in query.actual_start]
    canonical_start = [float(value) for value in query.start]
    if len(actual_start) != len(canonical_start):
        return path
    delta = [a - c for a, c in zip(actual_start, canonical_start)]
    return [[float(value) + delta[index] for index, value in enumerate(point)] for point in path]


def path_collision_free(robot: Any, obstacles: list[Any], path: list[list[float]], step: float) -> bool:
    if len(path) < 2:
        return False
    for a, b in zip(path[:-1], path[1:]):
        dist = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
        steps = max(1, int(math.ceil(dist / max(float(step), 1e-9))))
        for index in range(steps + 1):
            alpha = index / steps
            q = [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]
            if sbf.check_config_collision(robot, obstacles, q):
                return False
    return True


def query_rows(
    forest: Any,
    robot: Any,
    queries: Iterable[Any],
    obstacles: list[Any] | None = None,
    audit_step: float = DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    canonicalize_queries: bool = True,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for raw in queries:
        query = query_spec(raw)
        start = query_point(robot, query.start, canonicalize_queries)
        goal = query_point(robot, query.goal, canonicalize_queries)
        result = forest.query(start, goal)
        canonical_path = result.path_as_lists() if bool(result.success) else []
        actual_path = reflect_path_to_actual(query, canonical_path)
        actual_audit_passed = bool(result.audit_passed)
        actual_audit_status = str(result.audit_status)
        if bool(result.success) and obstacles is not None:
            actual_audit_passed = path_collision_free(robot, obstacles, actual_path, audit_step)
            actual_audit_status = "actual_reflected_audit_passed" if actual_audit_passed else "actual_reflected_audit_failed"
        combined_audit_passed = bool(result.audit_passed) and bool(actual_audit_passed)
        path_length = float(result.path_length) if bool(result.success) else math.nan
        segment_length = float(result.segment_edge_length) if bool(result.success) else 0.0
        rows.append({
            "label": query.label,
            "success": bool(result.success),
            "audit_passed": combined_audit_passed,
            "canonical_audit_passed": bool(result.audit_passed),
            "actual_reflected_audit_passed": bool(actual_audit_passed),
            "query_ms": float(result.query_time_ms),
            "audit_ms": float(result.audit_time_ms),
            "path_length": path_length,
            "segment_edge_length": segment_length,
            "segment_fraction": (segment_length / path_length) if bool(result.success) and path_length > 1e-12 else math.nan,
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
    t0 = time.perf_counter()
    build = forest.build_leaf_sweep_refined(
        obstacles,
        refine_cfg,
        (
            canonical_priority_points(robot, query_list, canonicalize=bool(options.canonicalize_queries))
            if bool(options.use_priority_points)
            else []
        ),
    )
    build_wall_s = time.perf_counter() - t0
    qrows = query_rows(
        forest,
        robot,
        query_list,
        obstacles=list(obstacles),
        audit_step=float(options.audit_segment_step),
        canonicalize_queries=bool(options.canonicalize_queries),
    )
    successes = [row for row in qrows if bool(row["audit_passed"])]
    total_len = sum(float(row["path_length"]) for row in successes if math.isfinite(float(row["path_length"])))
    total_seg = sum(float(row["segment_edge_length"]) for row in successes)
    diagnostics = {str(k): float(v) for k, v in dict(build.diagnostics).items()}
    build_s = float(build.total_ms) / 1000.0
    query_s = sum(float(row["query_ms"]) for row in qrows) / 1000.0
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
        "planning_s": build_s + query_s,
        "build_s": build_s,
        "query_s": query_s,
        "build_wall_s": build_wall_s,
        "leaf_sweep_s": float(build.leaf_sweep_ms) / 1000.0,
        "deep_refine_s": float(build.deep_refine_ms) / 1000.0,
        "rrt_grower_s": float(getattr(build, "rrt_grower_ms", 0.0)) / 1000.0,
        "connector_s": float(build.connector_ms) / 1000.0,
        "audit_s": sum(float(row["audit_ms"]) for row in qrows) / 1000.0,
        "path_length_mean": mean(row["path_length"] for row in successes),
        "raw_segment_fraction": (total_seg / total_len) if total_len > 1e-12 else math.nan,
        "leaf_free_count": int(build.leaf_free_count),
        "leaf_collision_count": int(build.leaf_collision_count),
        "deep_boxes_added": int(build.deep_boxes_added),
        "rrt_grower_boxes_added": int(getattr(build, "rrt_grower_boxes_added", 0)),
        "final_boxes": int(build.profile.final_boxes),
        "segment_edges": int(build.profile.segment_edges),
        "adjacency_islands": int(build.profile.adjacency_islands),
        "external_hits": external_hits,
        "database_root_intervals": interval_pairs(forest.database_root_intervals()),
        "database_coverage_intervals": interval_pairs(forest.database_coverage_intervals()),
        "queries": qrows,
        "diagnostics": diagnostics,
    }
