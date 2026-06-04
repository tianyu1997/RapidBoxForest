#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import random
import shutil
import statistics
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata, write_json  # noqa: E402
from experiments.common.shelf_iiwa_cache import directory_size  # noqa: E402
from experiments.common import run_shelf_sbf_case as shelf  # noqa: E402
from safe_box_forest.experiments.sbf_old import common_sbf_config as sbf_config  # noqa: E402


sbf = sbf_config.sbf

D23_CACHE_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_d23" / "cache"
D23_CACHE_LABEL = "iiwa_shelf_endpoint_only_p23_canonical_dim0q4_fixed_root"
D23_ROOT_INTERVALS = ";".join([
    "0.0:1.5707963267948966",
    "0.3194:0.8645",
    "-0.5077:0.5073",
    "-1.98947519:-0.33002121",
    "-0.447:0.4473",
    "-1.34734773:1.51007653",
    "1.262:1.8794",
])
DEFAULT_GAP_TARGET_LABELS = "AS->TS,TS->CS,CS->LB"
DEFAULT_GAP_TARGET_FRACTIONS = "0.25,0.5,0.75"
DEFAULT_TARGETED_GAP_CLOUD_LABELS = ""


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))
    return float(ordered[index])


def summary_stats(values: Iterable[float]) -> dict[str, Any]:
    vals = [float(value) for value in values]
    if not vals:
        return {"n": 0}
    return {
        "n": len(vals),
        "min": min(vals),
        "p25": percentile(vals, 0.25),
        "median": float(statistics.median(vals)),
        "mean": float(statistics.mean(vals)),
        "p75": percentile(vals, 0.75),
        "max": max(vals),
        "sum": float(sum(vals)),
    }


def histogram(values: Iterable[int]) -> dict[str, int]:
    return {str(key): int(value) for key, value in sorted(Counter(int(v) for v in values).items())}


def parse_root_intervals(text: str) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    for item in text.split(";"):
        lo, hi = item.split(":")
        intervals.append((float(lo), float(hi)))
    return intervals


def estimate_box_depth(box: Any, root: list[tuple[float, float]]) -> int:
    depth = 0
    for interval, (root_lo, root_hi) in zip(list(box.joint_intervals), root):
        root_width = max(float(root_hi) - float(root_lo), 0.0)
        width = max(float(interval.hi) - float(interval.lo), 1e-300)
        if root_width <= 0.0:
            continue
        ratio = root_width / width
        if ratio <= 1.0 + 1e-9:
            continue
        depth += max(0, int(round(math.log(ratio, 2.0))))
    return int(depth)


def box_widths(box: Any) -> list[float]:
    return [float(interval.hi) - float(interval.lo) for interval in list(box.joint_intervals)]


def point_list(point: Any) -> list[float]:
    return [float(value) for value in list(point)]


def point_distance(lhs: list[float], rhs: list[float]) -> float:
    return math.sqrt(sum((a - b) * (a - b) for a, b in zip(lhs, rhs)))


def overlap_waypoint(lhs: Any, rhs: Any) -> list[float]:
    out: list[float] = []
    for lhs_interval, rhs_interval in zip(list(lhs.joint_intervals), list(rhs.joint_intervals)):
        lo = max(float(lhs_interval.lo), float(rhs_interval.lo))
        hi = min(float(lhs_interval.hi), float(rhs_interval.hi))
        out.append(0.5 * (lo + hi))
    return out


def attributed_path_lengths(
    box_sequence: list[int],
    segment_edge_sequence: list[int],
    boxes_by_id: dict[int, Any],
    segment_edges_by_id: dict[int, Any],
    start: Any,
    goal: Any,
) -> dict[str, float]:
    path: list[list[float]] = [point_list(start)]
    segment_length = 0.0
    box_length = 0.0

    def append(waypoint: list[float], *, is_segment: bool) -> None:
        nonlocal segment_length, box_length
        if path and point_distance(path[-1], waypoint) <= 1e-12:
            return
        if path:
            distance = point_distance(path[-1], waypoint)
            if is_segment:
                segment_length += distance
            else:
                box_length += distance
        path.append(waypoint)

    for index in range(1, len(box_sequence)):
        lhs = boxes_by_id.get(int(box_sequence[index - 1]))
        rhs = boxes_by_id.get(int(box_sequence[index]))
        if lhs is None or rhs is None:
            continue
        edge_id = int(segment_edge_sequence[index - 1]) if index - 1 < len(segment_edge_sequence) else -1
        if edge_id >= 0 and edge_id in segment_edges_by_id:
            edge = segment_edges_by_id[edge_id]
            edge_path = [point_list(waypoint) for waypoint in list(edge.waypoints)]
            if int(edge.source_box_id) == int(rhs.id) and int(edge.target_box_id) == int(lhs.id):
                edge_path.reverse()
            if not edge_path:
                edge_path = [point_list(lhs.center()), point_list(rhs.center())]
            for waypoint in edge_path:
                append(waypoint, is_segment=True)
            continue
        lhs_dims = len(list(lhs.joint_intervals))
        rhs_dims = len(list(rhs.joint_intervals))
        if int(lhs.parent_box_id) == int(rhs.id) and len(list(lhs.seed_config)) == lhs_dims and lhs.contains(lhs.seed_config, 1e-9):
            append(point_list(lhs.seed_config), is_segment=False)
            append(overlap_waypoint(lhs, rhs), is_segment=False)
            continue
        if int(rhs.parent_box_id) == int(lhs.id) and len(list(rhs.seed_config)) == rhs_dims and rhs.contains(rhs.seed_config, 1e-9):
            append(overlap_waypoint(lhs, rhs), is_segment=False)
            append(point_list(rhs.seed_config), is_segment=False)
            continue
        append(overlap_waypoint(lhs, rhs), is_segment=False)
    append(point_list(goal), is_segment=False)
    return {
        "path_segment_length": segment_length,
        "path_box_length": box_length,
        "reconstructed_path_length": segment_length + box_length,
    }


def enum_name(value: Any) -> str:
    text = str(value)
    return text.split(".")[-1]


def recommended_case_args(args: argparse.Namespace, seed: int) -> argparse.Namespace:
    argv = [
        "run_shelf_sbf_case.py",
        "--case-name", f"{args.case_name}_seed{seed}",
        "--out-json", str(args.out_dir / f"{args.case_name}_seed{seed}.json"),
        "--database-path", str(args.out_dir / "active_cache" / f"{args.case_name}_seed{seed}"),
        "--seeds-list", str(seed),
        "--coverage-anchor-preset", "iris8",
        "--lect-root-intervals", D23_ROOT_INTERVALS,
        "--endpoint-source", "aafk",
        "--lect-split-policy", "aafk_volume_min",
        "--rbf-envelope", "support_hull",
        "--rbf-canonical-cache",
        "--rbf-cache-root", str(args.rbf_cache_root),
        "--warm-cache-label", str(args.warm_cache_label),
        "--use-external-evidence",
        "--external-evidence-mode", "snapshot",
        "--external-evidence-materialization",
        "--external-evidence-scoring",
        "--external-evidence-auto-build-snapshot",
        "--rbf-max-depth", str(int(args.rbf_max_depth)),
        "--ffb-depth", str(int(args.ffb_depth)),
        "--rbf-ffb-start-depth", str(int(args.rbf_ffb_start_depth)),
        "--threads", str(args.threads),
        "--task-batch-size", str(args.threads),
        "--max-boxes", str(int(args.max_boxes)),
        "--timeout-ms", str(args.timeout_ms),
        "--fixed-anchor-target-preset", "iris8",
        "--random-anchor-targets", "0",
        "--anchor-target-prob", str(float(args.anchor_target_prob)),
        "--anchor-wave-targets-per-batch", str(int(args.anchor_wave_targets_per_batch)),
        "--rrt-goal-bias", str(float(args.rrt_goal_bias)),
        "--intertree-goal-bias", str(float(args.intertree_goal_bias)),
        "--unexplored-prob", str(float(args.unexplored_prob)),
        "--sample-uniform-prob", str(float(args.sample_uniform_prob)),
        "--frontier-face-memory",
        "--frontier-face-bins-per-dim", "4",
        "--frontier-face-min-attempts", "1",
        "--frontier-face-max-attempts", "12",
        "--frontier-face-area-attempt-scale", "24",
        "--frontier-face-candidate-limit", str(int(args.frontier_face_candidate_limit)),
        "--frontwave-bootstrap-boxes", str(int(args.bootstrap_boxes)),
        "--frontwave-bootstrap-depth", str(int(args.bootstrap_depth)),
        "--frontwave-bootstrap-boundary-samples", str(int(args.bootstrap_boundary_samples)),
        "--component-connect-prob", str(float(args.component_connect_prob)),
        "--component-connect-candidate-limit", str(int(args.component_connect_candidate_limit)),
        "--component-connect-ffb-depth-increment", str(int(args.component_connect_ffb_depth_increment)),
        "--component-connect-ffb-max-depth", str(int(args.ffb_depth)),
        "--component-connect-stage-normalized-linf", "0.16",
        "--component-connect-neighbor-root-bias", str(float(args.component_connect_neighbor_root_bias)),
        "--component-connect-neighbor-root-window", str(int(args.component_connect_neighbor_root_window)),
        "--no-component-connect-require-target-direction",
        "--component-connect-chain-steps", str(int(args.component_connect_chain_steps)),
        "--component-connect-chain-max-boxes", str(int(args.component_connect_chain_max_boxes)),
        "--enable-connector",
        "--connector-birrt",
        "--connector-rrt-iters", str(int(args.connector_rrt_iters)),
        "--connector-rrt-timeout-ms", str(float(args.connector_rrt_timeout_ms)),
        "--connector-rrt-step-size", str(float(args.connector_rrt_step_size)),
        "--connector-rrt-goal-bias", str(float(args.connector_rrt_goal_bias)),
        "--connector-pair-timeout-ms", str(float(args.connector_pair_timeout_ms)),
        "--connector-max-pairs-per-gap", str(int(args.connector_max_pairs_per_gap)),
        "--connector-bridge-boxes", str(int(args.connector_bridge_boxes)),
        "--connector-pave-max-chain", str(int(args.connector_pave_max_chain)),
        "--connector-pave-steps", str(int(args.connector_pave_steps)),
        "--connector-pave-depth", str(int(args.connector_pave_depth)),
        "--connector-pave-fill-gaps" if args.connector_pave_fill_gaps else "--no-connector-pave-fill-gaps",
        "--connector-pave-require-connected-chain" if args.connector_pave_require_connected_chain else "--no-connector-pave-require-connected-chain",
        "--connector-pave-gap-fill-time-budget-ms", str(float(args.connector_pave_gap_fill_time_budget_ms)),
        "--connector-pave-gap-fill-max-ffb-calls", str(int(args.connector_pave_gap_fill_max_ffb_calls)),
        "--connector-pave-gap-fill-sample-step", str(float(args.connector_pave_gap_fill_sample_step)),
        "--connector-pave-gap-fill-min-arc-gain", str(float(args.connector_pave_gap_fill_min_arc_gain)),
        "--segment-edge-policy", str(args.segment_edge_policy),
        "--collision-shortcut" if args.collision_shortcut else "--no-collision-shortcut",
        "--collision-shortcut-resolution", "24",
        "--no-repair-on-audit-failure",
        "--require-no-repair",
        "--audit-collision-tolerance", "0.002",
        "--post-audit-segment-step", "0.01",
        "--bridge-failed-queries",
        "--no-corridor-refine",
        "--final-ompl-simplify-time-s", str(float(args.final_ompl_simplify_time_s)),
        "--clean-active-cache",
    ]
    if bool(args.enable_merger):
        argv.append("--enable-merger")
    if bool(args.sample_categorical_allocation):
        argv.append("--sample-categorical-allocation")
    old_argv = sys.argv
    try:
        sys.argv = argv
        return shelf.parse_args()
    finally:
        sys.argv = old_argv


def build_forest(profile_args: argparse.Namespace,
                 case_args: argparse.Namespace,
                 seed: int,
                 extra_targets: list[list[float]] | None = None) -> tuple[Any, Any, list[Any], list[Any], Any, float]:
    effective_args = shelf.effective_case_args(case_args)
    robot = sbf.load_iiwa14_robot()
    obstacles = sbf.make_combined_obstacles()
    queries = sbf.make_combined_queries()
    coverage_seeds = shelf.coverage_seeds_for_args(effective_args, robot)
    cfg = shelf.case_config(effective_args, robot, seed)
    gap_targets = make_gap_targets(queries, profile_args.gap_target_labels, profile_args.gap_target_fractions)
    gap_targets += make_gap_target_cloud(
        robot,
        obstacles,
        queries,
        profile_args.gap_target_cloud_labels,
        count_per_query=int(profile_args.gap_target_cloud_count),
        radius=float(profile_args.gap_target_cloud_radius),
        attempts_per_target=int(profile_args.gap_target_cloud_attempts_per_target),
        seed=seed + 7919,
    )
    if extra_targets:
        gap_targets = gap_targets + [list(target) for target in extra_targets]
    if gap_targets and bool(profile_args.gap_target_as_coverage_seeds):
        coverage_seeds = list(coverage_seeds) + gap_targets
    if gap_targets and hasattr(cfg.grower, "set_fixed_anchor_targets"):
        fixed_targets = gap_targets if bool(profile_args.gap_targets_replace_fixed_anchors) else (
            [list(anchor) for anchor in sbf_config.IRIS_GCS_SHELF_ANCHOR8] + gap_targets
        )
        cfg.grower.set_fixed_anchor_targets(fixed_targets)
    database_path = Path(cfg.database.path)
    if bool(effective_args.clean_active_cache) and database_path.exists():
        shutil.rmtree(database_path)
    if bool(effective_args.use_external_evidence):
        warm_path = Path(effective_args.rbf_cache_root) / str(effective_args.warm_cache_label)
        shelf.configure_external_evidence_reuse(
            cfg,
            warm_path,
            effective_args,
            materialization=bool(effective_args.external_evidence_materialization),
            scoring=bool(effective_args.external_evidence_scoring),
            backfill_active=False,
        )
    cfg.query.shortcut_boxes = bool(profile_args.query_shortcut_boxes)
    cfg.database.create_if_missing = True
    forest = sbf.SafeBoxForest(robot, cfg)
    start = time.perf_counter()
    profile = forest.build_coverage(obstacles, coverage_seeds)
    wall_s = time.perf_counter() - start
    return forest, robot, obstacles, queries, profile, wall_s


def parse_csv_text(text: str) -> set[str]:
    return {item.strip() for item in str(text or "").split(",") if item.strip()}


def resample_polyline(points: list[list[float]], count: int) -> list[list[float]]:
    if count <= 0 or len(points) < 2:
        return []
    lengths = [0.0]
    for index in range(1, len(points)):
        lengths.append(lengths[-1] + point_distance(points[index - 1], points[index]))
    total = lengths[-1]
    if total <= 1e-12:
        return []
    targets: list[list[float]] = []
    for sample_index in range(1, count + 1):
        arc = total * (sample_index / float(count + 1))
        segment = 1
        while segment < len(lengths) and lengths[segment] < arc:
            segment += 1
        segment = min(segment, len(lengths) - 1)
        lo_arc = lengths[segment - 1]
        hi_arc = lengths[segment]
        alpha = 0.0 if hi_arc <= lo_arc else (arc - lo_arc) / (hi_arc - lo_arc)
        lhs = points[segment - 1]
        rhs = points[segment]
        targets.append([(1.0 - alpha) * a + alpha * b for a, b in zip(lhs, rhs)])
    return targets


def collect_segment_waypoint_targets(forest: Any,
                                     queries: list[Any],
                                     labels_text: str,
                                     per_edge: int) -> list[list[float]]:
    labels = parse_csv_text(labels_text)
    if not labels:
        return []
    segment_edges_by_id = {int(edge.id): edge for edge in list(forest.segment_edges())}
    targets: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for query in queries:
        if str(getattr(query, "label", "")) not in labels:
            continue
        raw = forest.query(list(query.start), list(query.goal))
        for edge_id in list(raw.segment_edge_sequence):
            edge_id = int(edge_id)
            if edge_id < 0 or edge_id not in segment_edges_by_id:
                continue
            edge = segment_edges_by_id[edge_id]
            waypoints = [point_list(waypoint) for waypoint in list(edge.waypoints)]
            for target in resample_polyline(waypoints, per_edge):
                key = tuple(round(value, 12) for value in target)
                if key in seen:
                    continue
                seen.add(key)
                targets.append(target)
    return targets


def parse_csv_floats(text: str) -> list[float]:
    return [float(item.strip()) for item in str(text or "").split(",") if item.strip()]


def make_gap_targets(queries: list[Any], labels_text: str, fractions_text: str) -> list[list[float]]:
    labels = {item.strip() for item in str(labels_text or "").split(",") if item.strip()}
    if not labels:
        return []
    fractions = [value for value in parse_csv_floats(fractions_text) if 0.0 < value < 1.0]
    targets: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for query in queries:
        label = str(getattr(query, "label", ""))
        if label not in labels:
            continue
        start = [float(value) for value in list(query.start)]
        goal = [float(value) for value in list(query.goal)]
        for frac in fractions:
            target = [(1.0 - frac) * lhs + frac * rhs for lhs, rhs in zip(start, goal)]
            key = tuple(round(value, 12) for value in target)
            if key in seen:
                continue
            seen.add(key)
            targets.append(target)
    return targets


def clip_to_root(point: list[float], root: list[tuple[float, float]]) -> list[float]:
    return [
        min(max(float(value), float(lo)), float(hi))
        for value, (lo, hi) in zip(point, root)
    ]


def make_gap_target_cloud(robot: Any,
                          obstacles: list[Any],
                          queries: list[Any],
                          labels_text: str,
                          *,
                          count_per_query: int,
                          radius: float,
                          attempts_per_target: int,
                          seed: int) -> list[list[float]]:
    labels = parse_csv_text(labels_text)
    if not labels or count_per_query <= 0:
        return []
    root = parse_root_intervals(D23_ROOT_INTERVALS)
    rng = random.Random(int(seed))
    targets: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for query in queries:
        if str(getattr(query, "label", "")) not in labels:
            continue
        start = [float(value) for value in list(query.start)]
        goal = [float(value) for value in list(query.goal)]
        accepted = 0
        attempts = 0
        while accepted < count_per_query and attempts < count_per_query * max(1, attempts_per_target):
            attempts += 1
            alpha = rng.uniform(0.12, 0.88)
            base = [(1.0 - alpha) * lhs + alpha * rhs for lhs, rhs in zip(start, goal)]
            scale = float(radius) * (0.35 + 0.65 * rng.random())
            jitter = [rng.gauss(0.0, scale) for _ in base]
            point = clip_to_root([value + delta for value, delta in zip(base, jitter)], root)
            key = tuple(round(value, 10) for value in point)
            if key in seen:
                continue
            if sbf.check_config_collision(robot, obstacles, point, 0.002):
                continue
            seen.add(key)
            targets.append(point)
            accepted += 1
    return targets


def summarize_segment_edges(segment_edges: list[Any]) -> dict[str, Any]:
    by_type: dict[str, int] = defaultdict(int)
    lengths: list[float] = []
    waypoint_counts: list[int] = []
    for edge in segment_edges:
        by_type[enum_name(edge.type)] += 1
        lengths.append(float(edge.length))
        waypoint_counts.append(len(list(edge.waypoints)))
    return {
        "count": len(segment_edges),
        "by_type": dict(sorted(by_type.items())),
        "length": summary_stats(lengths),
        "waypoint_count": summary_stats(waypoint_counts),
        "edges": [
            {
                "id": int(edge.id),
                "source_box_id": int(edge.source_box_id),
                "target_box_id": int(edge.target_box_id),
                "type": enum_name(edge.type),
                "validation": enum_name(edge.validation),
                "length": float(edge.length),
                "waypoint_count": len(list(edge.waypoints)),
                "segment_resolution": int(edge.segment_resolution),
            }
            for edge in segment_edges
        ],
    }


def summarize_seed(args: argparse.Namespace, seed: int) -> dict[str, Any]:
    case_args = recommended_case_args(args, seed)
    probe_targets: list[list[float]] = []
    probe_wall_s = 0.0
    if bool(args.probe_segment_targets):
        probe_forest, _probe_robot, _probe_obstacles, probe_queries, _probe_profile, probe_wall_s = build_forest(
            args,
            case_args,
            seed,
        )
        probe_targets = collect_segment_waypoint_targets(
            probe_forest,
            probe_queries,
            args.probe_target_labels,
            int(args.probe_targets_per_edge),
        )
        del probe_forest
    forest, robot, obstacles, queries, profile, build_wall_s = build_forest(args, case_args, seed, probe_targets)
    root = parse_root_intervals(D23_ROOT_INTERVALS)
    boxes = list(forest.boxes())
    raw_boxes = list(forest.raw_boxes()) if hasattr(forest, "raw_boxes") else boxes
    segment_edges = list(forest.segment_edges())
    boxes_by_id = {int(box.id): box for box in boxes}
    segment_edges_by_id = {int(edge.id): edge for edge in segment_edges}
    depth_by_id = {int(box.id): estimate_box_depth(box, root) for box in boxes}
    volume_by_id = {int(box.id): float(box.volume) for box in boxes}
    root_by_id = {int(box.id): int(box.root_id) for box in boxes}
    safety_by_id = {int(box.id): enum_name(box.safety_status) for box in boxes}
    widths = [box_widths(box) for box in boxes]
    width_by_dim = [
        summary_stats([item[dim] for item in widths])
        for dim in range(len(widths[0]) if widths else 0)
    ]
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    raw_query_rows: list[dict[str, Any]] = []
    final_query_rows: list[dict[str, Any]] = []
    for query in queries:
        raw = forest.query(list(query.start), list(query.goal))
        box_sequence = [int(box_id) for box_id in list(raw.box_sequence)]
        unique_box_sequence = list(dict.fromkeys(box_sequence))
        segment_edge_sequence = [int(edge_id) for edge_id in list(raw.segment_edge_sequence)]
        attributed = attributed_path_lengths(
            box_sequence,
            segment_edge_sequence,
            boxes_by_id,
            segment_edges_by_id,
            query.start,
            query.goal,
        )
        raw_path_length = float(raw.path_length)
        path_segment_length = min(float(attributed["path_segment_length"]), raw_path_length)
        path_depths = [depth_by_id[box_id] for box_id in box_sequence if box_id in depth_by_id]
        unique_path_depths = [depth_by_id[box_id] for box_id in unique_box_sequence if box_id in depth_by_id]
        path_roots = [root_by_id[box_id] for box_id in unique_box_sequence if box_id in root_by_id]
        raw_query_rows.append({
            "name": query.label,
            "success": bool(raw.success),
            "audit_passed": bool(raw.audit_passed),
            "path_length": raw_path_length,
            "segment_edge_fraction": (
                float(raw.segment_edge_length) / float(raw.path_length)
                if bool(raw.success) and float(raw.path_length) > 0.0
                else 0.0
            ),
            "stored_segment_edge_length": float(raw.segment_edge_length),
            "stored_segment_edge_fraction": (
                float(raw.segment_edge_length) / raw_path_length
                if bool(raw.success) and raw_path_length > 0.0
                else 0.0
            ),
            "path_segment_length": path_segment_length,
            "path_segment_fraction": (
                path_segment_length / raw_path_length
                if bool(raw.success) and raw_path_length > 0.0
                else 0.0
            ),
            "path_box_length": float(attributed["path_box_length"]),
            "reconstructed_path_length": float(attributed["reconstructed_path_length"]),
            "query_time_ms": float(raw.query_time_ms),
            "audit_time_ms": float(raw.audit_time_ms),
            "repair_time_ms": float(raw.repair_time_ms),
            "repair_count": int(raw.repair_count),
            "start_box_id": int(raw.start_box_id),
            "goal_box_id": int(raw.goal_box_id),
            "box_sequence_count": len(box_sequence),
            "unique_box_sequence_count": len(unique_box_sequence),
            "box_sequence": box_sequence,
            "segment_edge_sequence": segment_edge_sequence,
            "segment_edges_used": int(raw.segment_edges_used),
            "segment_edge_length": float(raw.segment_edge_length),
            "certified_box_length": float(raw.certified_box_length),
            "path_box_depth_histogram": histogram(path_depths),
            "unique_path_box_depth_histogram": histogram(unique_path_depths),
            "path_box_depth_stats": summary_stats(path_depths),
            "unique_path_box_depth_stats": summary_stats(unique_path_depths),
            "unique_path_root_count": len(set(path_roots)),
            "unique_path_roots": sorted(set(path_roots)),
            "path_box_volume_stats": summary_stats([volume_by_id[box_id] for box_id in unique_box_sequence if box_id in volume_by_id]),
        })
        final_query_rows.append(shelf.run_query(forest, robot, obstacles, query, case_args))
    del forest
    mission_box_union: set[int] = set()
    mission_root_union: set[int] = set()
    mission_depths: list[int] = []
    mission_segment_edge_ids: set[int] = set()
    mission_box_sequence_total = 0
    for row in raw_query_rows:
        mission_box_sequence_total += int(row["box_sequence_count"])
        for box_id in row["box_sequence"]:
            mission_box_union.add(int(box_id))
        for root_id in row["unique_path_roots"]:
            mission_root_union.add(int(root_id))
        for edge_id in row["segment_edge_sequence"]:
            if int(edge_id) >= 0:
                mission_segment_edge_ids.add(int(edge_id))
    for box_id in sorted(mission_box_union):
        if box_id in depth_by_id:
            mission_depths.append(depth_by_id[box_id])
    depth_values = list(depth_by_id.values())
    safety_counts = Counter(safety_by_id.values())
    root_counts = Counter(root_by_id.values())
    stage_times = {
        "build_wall_ms": build_wall_s * 1000.0,
        "profile_total_ms": float(profile.total_ms),
        "grow_ms": float(profile.grow_ms),
        "merge_ms": float(profile.merge_ms),
        "connector_ms": float(profile.connector_ms),
        "adjacency_ms": float(profile.adjacency_ms),
        "frontwave_bootstrap_ms": diagnostics.get("profile.grower.rrt.frontwave_bootstrap.total_ms", 0.0),
        "grow_loop_ms": diagnostics.get("profile.grower.rrt.loop.total_ms", 0.0),
        "connector_birrt_ms": diagnostics.get("profile.connector.birrt.total_ms", diagnostics.get("profile.connector.birrt.max_ms", 0.0)),
        "query_planning_ms_total_raw": sum(row["query_time_ms"] for row in raw_query_rows),
        "query_time_ms_total_raw": sum(row["query_time_ms"] for row in raw_query_rows),
        "audit_time_ms_total_raw": sum(row["audit_time_ms"] for row in raw_query_rows),
        "final_query_wall_ms_total": sum(float(row.get("t_s", 0.0)) * 1000.0 for row in final_query_rows),
        "final_planning_excluding_audit_ms_total": sum(
            float(row.get("planning_time_ms", 0.0))
            + float(row.get("bridge_time_s", 0.0)) * 1000.0
            + float(row.get("ompl_final_simplify_time_s", 0.0)) * 1000.0
            for row in final_query_rows
        ),
        "final_ompl_simplify_ms_total": sum(float(row.get("ompl_final_simplify_time_s", 0.0)) * 1000.0 for row in final_query_rows),
    }
    raw_path_total = sum(float(row.get("path_length", 0.0)) for row in raw_query_rows)
    stored_segment_total = sum(float(row.get("segment_edge_length", 0.0)) for row in raw_query_rows)
    path_segment_total = sum(float(row.get("path_segment_length", 0.0)) for row in raw_query_rows)
    return {
        "seed": int(seed),
        "ok": all(bool(row.get("audit_passed")) and bool(row.get("post_audit_passed")) for row in final_query_rows),
        "box_count": len(boxes),
        "raw_box_count": len(raw_boxes),
        "segment_edge_count": len(segment_edges),
        "grow_adjacency_islands": int(profile.grow_adjacency_islands),
        "grow_largest_island": int(profile.grow_largest_island),
        "adjacency_islands": int(profile.adjacency_islands),
        "connector_islands_initial": int(diagnostics.get("connector.islands_initial", profile.grow_adjacency_islands)),
        "boxes": {
            "depth_histogram": histogram(depth_values),
            "depth_stats": summary_stats(depth_values),
            "volume_stats": summary_stats(float(box.volume) for box in boxes),
            "safety_counts": dict(sorted(safety_counts.items())),
            "root_counts": {str(key): int(value) for key, value in sorted(root_counts.items())},
            "root_count": len(root_counts),
            "width_by_dim": width_by_dim,
        },
        "segment_edges": summarize_segment_edges(segment_edges),
        "mission_route": {
            "query_count": len(raw_query_rows),
            "box_sequence_total": mission_box_sequence_total,
            "unique_box_count": len(mission_box_union),
            "unique_box_fraction_of_forest": (len(mission_box_union) / len(boxes)) if boxes else 0.0,
            "unique_segment_edge_count": len(mission_segment_edge_ids),
            "unique_segment_edge_ids": sorted(mission_segment_edge_ids),
            "unique_root_count": len(mission_root_union),
            "unique_roots": sorted(mission_root_union),
            "unique_box_depth_histogram": histogram(mission_depths),
            "unique_box_depth_stats": summary_stats(mission_depths),
            "raw_path_total": raw_path_total,
            "raw_stored_segment_edge_length_total": stored_segment_total,
            "raw_stored_segment_edge_fraction_total": (stored_segment_total / raw_path_total) if raw_path_total > 0.0 else 0.0,
            "raw_path_segment_length_total": path_segment_total,
            "raw_path_segment_fraction_total": (path_segment_total / raw_path_total) if raw_path_total > 0.0 else 0.0,
            "raw_path_segment_fraction_by_query": {
                row["name"]: row["path_segment_fraction"] for row in raw_query_rows
            },
        },
        "stage_times": stage_times,
        "probe_segment_targets": {
            "enabled": bool(args.probe_segment_targets),
            "count": len(probe_targets),
            "probe_build_wall_ms": probe_wall_s * 1000.0,
            "target_labels": str(args.probe_target_labels),
            "targets_per_edge": int(args.probe_targets_per_edge),
        },
        "profile": {
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "merge_ms": float(profile.merge_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "segment_edges": int(profile.segment_edges),
            "segment_edges_added": int(profile.segment_edges_added),
            "rrt_segment_edges_added": int(profile.rrt_segment_edges_added),
            "point_gap_segment_edges_added": int(profile.point_gap_segment_edges_added),
        },
        "diagnostics_selected": {
            key: diagnostics.get(key, 0.0)
            for key in sorted(diagnostics)
            if key.startswith("connector.")
            or key.startswith("grower.")
            or key.startswith("profile.")
            or key in {
                "oracle.materialization_reused_external_evidence",
                "oracle.materializations",
                "oracle.node_validations",
                "oracle.scoring_reused_external_evidence",
            }
        },
        "raw_queries": raw_query_rows,
        "final_queries": final_query_rows,
        "final_path_total": sum(float(row.get("length", 0.0)) for row in final_query_rows),
        "raw_path_total": raw_path_total,
        "raw_stored_segment_edge_length_total": stored_segment_total,
        "raw_stored_segment_edge_fraction_total": (stored_segment_total / raw_path_total) if raw_path_total > 0.0 else 0.0,
        "raw_path_segment_length_total": path_segment_total,
        "raw_path_segment_fraction_total": (path_segment_total / raw_path_total) if raw_path_total > 0.0 else 0.0,
    }


def aggregate(seed_rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "n": len(seed_rows),
        "ok_count": sum(1 for row in seed_rows if bool(row.get("ok"))),
        "box_count": summary_stats(row["box_count"] for row in seed_rows),
        "segment_edge_count": summary_stats(row["segment_edge_count"] for row in seed_rows),
        "grow_adjacency_islands": summary_stats(row["grow_adjacency_islands"] for row in seed_rows),
        "adjacency_islands": summary_stats(row["adjacency_islands"] for row in seed_rows),
        "raw_path_total": summary_stats(row["raw_path_total"] for row in seed_rows),
        "raw_stored_segment_edge_length_total": summary_stats(row["raw_stored_segment_edge_length_total"] for row in seed_rows),
        "raw_stored_segment_edge_fraction_total": summary_stats(row["raw_stored_segment_edge_fraction_total"] for row in seed_rows),
        "raw_path_segment_length_total": summary_stats(row["raw_path_segment_length_total"] for row in seed_rows),
        "raw_path_segment_fraction_total": summary_stats(row["raw_path_segment_fraction_total"] for row in seed_rows),
        "final_path_total": summary_stats(row["final_path_total"] for row in seed_rows),
        "mission_route_unique_box_count": summary_stats(row["mission_route"]["unique_box_count"] for row in seed_rows),
        "mission_route_box_sequence_total": summary_stats(row["mission_route"]["box_sequence_total"] for row in seed_rows),
        "mission_route_unique_box_fraction": summary_stats(row["mission_route"]["unique_box_fraction_of_forest"] for row in seed_rows),
        "mission_route_unique_segment_edge_count": summary_stats(row["mission_route"]["unique_segment_edge_count"] for row in seed_rows),
        "probe_segment_target_count": summary_stats(row["probe_segment_targets"]["count"] for row in seed_rows),
        "probe_build_wall_ms": summary_stats(row["probe_segment_targets"]["probe_build_wall_ms"] for row in seed_rows),
        "stage_times_ms": {
            key: summary_stats(row["stage_times"].get(key, 0.0) for row in seed_rows)
            for key in sorted(seed_rows[0]["stage_times"]) if seed_rows
        },
        "box_depth_histogram_sum": dict(sorted((
            (depth, sum(int(row["boxes"]["depth_histogram"].get(depth, 0)) for row in seed_rows))
            for depth in sorted({depth for row in seed_rows for depth in row["boxes"]["depth_histogram"]}, key=int)
        ), key=lambda item: int(item[0]))),
        "path_box_depth_histogram_sum": dict(sorted((
            (depth, sum(
                int(query["unique_path_box_depth_histogram"].get(depth, 0))
                for row in seed_rows
                for query in row["raw_queries"]
            ))
            for depth in sorted({
                depth
                for row in seed_rows
                for query in row["raw_queries"]
                for depth in query["unique_path_box_depth_histogram"]
            }, key=int)
        ), key=lambda item: int(item[0]))),
    }


def write_markdown(path: Path, payload: dict[str, Any]) -> None:
    agg = payload["aggregate"]
    lines = [
        "# Exp04 Recommended Anchor-Segment Profile",
        "",
        f"Artifact: `{payload['out_json']}`",
        "",
        "## Aggregate",
        "",
        "| metric | median | min | max |",
        "| --- | ---: | ---: | ---: |",
    ]
    for key in ("box_count", "segment_edge_count", "grow_adjacency_islands", "adjacency_islands", "raw_path_total", "raw_path_segment_length_total", "raw_path_segment_fraction_total", "raw_stored_segment_edge_fraction_total", "final_path_total"):
        stats = agg[key]
        lines.append(f"| {key} | {stats.get('median')} | {stats.get('min')} | {stats.get('max')} |")
    for key in ("mission_route_box_sequence_total", "mission_route_unique_box_count", "mission_route_unique_box_fraction", "mission_route_unique_segment_edge_count"):
        stats = agg[key]
        lines.append(f"| {key} | {stats.get('median')} | {stats.get('min')} | {stats.get('max')} |")
    for key in ("probe_segment_target_count", "probe_build_wall_ms"):
        stats = agg[key]
        lines.append(f"| {key} | {stats.get('median')} | {stats.get('min')} | {stats.get('max')} |")
    lines.extend([
        "",
        "## Per-Seed Forest Summary",
        "",
        "| seed | boxes | roots | grow islands | largest grow island | final islands | segment edges | mission boxes | mission fraction | mission segment edges | path segment/raw path | stored segment/raw path |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for seed_row in payload["seeds"]:
        lines.append(
            f"| {seed_row['seed']} | {seed_row['box_count']} | {seed_row['boxes']['root_count']} | "
            f"{seed_row['grow_adjacency_islands']} | {seed_row['grow_largest_island']} | "
            f"{seed_row['adjacency_islands']} | {seed_row['segment_edge_count']} | "
            f"{seed_row['mission_route']['unique_box_count']} | "
            f"{seed_row['mission_route']['unique_box_fraction_of_forest']:.3f} | "
            f"{seed_row['mission_route']['unique_segment_edge_count']} | "
            f"{seed_row['mission_route']['raw_path_segment_fraction_total']:.3f} | "
            f"{seed_row['mission_route']['raw_stored_segment_edge_fraction_total']:.3f} |"
        )
    lines.extend([
        "",
        "## Stage Time Median (ms)",
        "",
        "| stage | median | min | max |",
        "| --- | ---: | ---: | ---: |",
    ])
    for key, stats in agg["stage_times_ms"].items():
        lines.append(f"| {key} | {stats.get('median'):.3f} | {stats.get('min'):.3f} | {stats.get('max'):.3f} |")
    lines.extend([
        "",
        "## Box Depth Histogram (summed over seeds)",
        "",
        json.dumps(agg["box_depth_histogram_sum"], indent=2, sort_keys=True),
        "",
        "## Unique Path Box Depth Histogram (summed over raw query routes)",
        "",
        json.dumps(agg["path_box_depth_histogram_sum"], indent=2, sort_keys=True),
        "",
        "## Per-Seed Query Summary",
        "",
        "| seed | query | raw length | path segment/raw path | stored segment/raw path | final length | boxes | unique boxes | segment edges | depth median |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for seed_row in payload["seeds"]:
        final_by_name = {row["name"]: row for row in seed_row["final_queries"]}
        for query in seed_row["raw_queries"]:
            final = final_by_name.get(query["name"], {})
            depth_median = query["unique_path_box_depth_stats"].get("median")
            lines.append(
                f"| {seed_row['seed']} | {query['name']} | {query['path_length']:.3f} | "
                f"{query['path_segment_fraction']:.3f} | {query['stored_segment_edge_fraction']:.3f} | "
                f"{float(final.get('length', 0.0)):.3f} | {query['box_sequence_count']} | "
                f"{query['unique_box_sequence_count']} | {query['segment_edges_used']} | {depth_median} |"
            )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Profile the recommended d23 + iris8 + segment + simplify shelf configuration.")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "outputs" / "new_experiments" / "exp04_anchor_segment_profile")
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--out-md", type=Path, default=None)
    parser.add_argument("--case-name", default="recommended_d23_iris8_segment_simplify005_profile")
    parser.add_argument("--seeds-list", default="0,1,2")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--max-boxes", type=int, default=1600)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--ffb-depth", type=int, default=34)
    parser.add_argument("--rbf-ffb-start-depth", type=int, default=15)
    parser.add_argument("--component-connect-ffb-depth-increment", type=int, default=14)
    parser.add_argument("--connector-pave-depth", type=int, default=28)
    parser.add_argument("--bootstrap-boxes", type=int, default=210)
    parser.add_argument("--bootstrap-depth", type=int, default=15)
    parser.add_argument("--bootstrap-boundary-samples", type=int, default=8)
    parser.add_argument("--anchor-wave-targets-per-batch", type=int, default=4)
    parser.add_argument("--frontier-face-candidate-limit", type=int, default=128)
    parser.add_argument("--anchor-target-prob", type=float, default=0.10)
    parser.add_argument("--gap-target-labels", default="")
    parser.add_argument("--gap-target-fractions", default=DEFAULT_GAP_TARGET_FRACTIONS)
    parser.add_argument("--gap-target-as-coverage-seeds", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--gap-target-cloud-labels", default=DEFAULT_TARGETED_GAP_CLOUD_LABELS)
    parser.add_argument("--gap-target-cloud-count", type=int, default=0)
    parser.add_argument("--gap-target-cloud-radius", type=float, default=0.12)
    parser.add_argument("--gap-target-cloud-attempts-per-target", type=int, default=30)
    parser.add_argument("--gap-targets-replace-fixed-anchors", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--probe-segment-targets", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--probe-target-labels", default=DEFAULT_GAP_TARGET_LABELS)
    parser.add_argument("--probe-targets-per-edge", type=int, default=3)
    parser.add_argument("--rrt-goal-bias", type=float, default=0.15)
    parser.add_argument("--intertree-goal-bias", type=float, default=0.25)
    parser.add_argument("--unexplored-prob", type=float, default=0.20)
    parser.add_argument("--sample-uniform-prob", type=float, default=0.30)
    parser.add_argument("--sample-categorical-allocation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--component-connect-prob", type=float, default=0.0)
    parser.add_argument("--component-connect-candidate-limit", type=int, default=1)
    parser.add_argument("--component-connect-neighbor-root-bias", type=float, default=1.0)
    parser.add_argument("--component-connect-neighbor-root-window", type=int, default=1)
    parser.add_argument("--component-connect-chain-steps", type=int, default=0)
    parser.add_argument("--component-connect-chain-max-boxes", type=int, default=0)
    parser.add_argument("--connector-rrt-iters", type=int, default=10000)
    parser.add_argument("--connector-rrt-timeout-ms", type=float, default=300.0)
    parser.add_argument("--connector-rrt-step-size", type=float, default=0.25)
    parser.add_argument("--connector-rrt-goal-bias", type=float, default=0.45)
    parser.add_argument("--connector-pair-timeout-ms", type=float, default=80.0)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=3)
    parser.add_argument("--connector-bridge-boxes", type=int, default=0)
    parser.add_argument("--connector-pave-max-chain", type=int, default=0)
    parser.add_argument("--connector-pave-steps", type=int, default=12)
    parser.add_argument("--connector-pave-fill-gaps", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-pave-require-connected-chain", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--connector-pave-gap-fill-time-budget-ms", type=float, default=10.0)
    parser.add_argument("--connector-pave-gap-fill-max-ffb-calls", type=int, default=32)
    parser.add_argument("--connector-pave-gap-fill-sample-step", type=float, default=0.05)
    parser.add_argument("--connector-pave-gap-fill-min-arc-gain", type=float, default=0.01)
    parser.add_argument("--segment-edge-policy", choices=["normal", "fallback_only", "off"], default="normal")
    parser.add_argument("--query-shortcut-boxes", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-shortcut", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.05)
    parser.add_argument("--enable-merger", action="store_true", default=False)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_json = args.out_json or (args.out_dir / "recommended_profile.json")
    args.out_md = args.out_md or (args.out_dir / "recommended_profile.md")
    seeds = [int(item.strip()) for item in str(args.seeds_list).split(",") if item.strip()]
    cache_path = args.rbf_cache_root / str(args.warm_cache_label)
    seed_rows = [summarize_seed(args, seed) for seed in seeds]
    payload = {
        "experiment": "exp04_recommended_anchor_segment_profile",
        "out_json": str(args.out_json),
        "params": {
            "seeds": seeds,
            "threads": int(args.threads),
            "max_boxes": int(args.max_boxes),
            "rbf_max_depth": int(args.rbf_max_depth),
            "ffb_depth": int(args.ffb_depth),
            "rbf_ffb_start_depth": int(args.rbf_ffb_start_depth),
            "component_connect_ffb_depth_increment": int(args.component_connect_ffb_depth_increment),
            "connector_pave_depth": int(args.connector_pave_depth),
            "bootstrap_boxes": int(args.bootstrap_boxes),
            "bootstrap_depth": int(args.bootstrap_depth),
            "bootstrap_boundary_samples": int(args.bootstrap_boundary_samples),
            "anchor_wave_targets_per_batch": int(args.anchor_wave_targets_per_batch),
            "frontier_face_candidate_limit": int(args.frontier_face_candidate_limit),
            "anchor_target_prob": float(args.anchor_target_prob),
            "gap_target_labels": str(args.gap_target_labels),
            "gap_target_fractions": str(args.gap_target_fractions),
            "gap_target_as_coverage_seeds": bool(args.gap_target_as_coverage_seeds),
            "gap_target_count": len(make_gap_targets(sbf.make_combined_queries(), args.gap_target_labels, args.gap_target_fractions)),
            "rrt_goal_bias": float(args.rrt_goal_bias),
            "intertree_goal_bias": float(args.intertree_goal_bias),
            "unexplored_prob": float(args.unexplored_prob),
            "sample_uniform_prob": float(args.sample_uniform_prob),
            "sample_categorical_allocation": bool(args.sample_categorical_allocation),
            "connector_rrt_iters": int(args.connector_rrt_iters),
            "connector_rrt_timeout_ms": float(args.connector_rrt_timeout_ms),
            "connector_rrt_step_size": float(args.connector_rrt_step_size),
            "connector_rrt_goal_bias": float(args.connector_rrt_goal_bias),
            "connector_pair_timeout_ms": float(args.connector_pair_timeout_ms),
            "connector_max_pairs_per_gap": int(args.connector_max_pairs_per_gap),
            "connector_bridge_boxes": int(args.connector_bridge_boxes),
            "connector_pave_max_chain": int(args.connector_pave_max_chain),
            "connector_pave_steps": int(args.connector_pave_steps),
            "connector_pave_fill_gaps": bool(args.connector_pave_fill_gaps),
            "connector_pave_require_connected_chain": bool(args.connector_pave_require_connected_chain),
            "connector_pave_gap_fill_time_budget_ms": float(args.connector_pave_gap_fill_time_budget_ms),
            "connector_pave_gap_fill_max_ffb_calls": int(args.connector_pave_gap_fill_max_ffb_calls),
            "connector_pave_gap_fill_sample_step": float(args.connector_pave_gap_fill_sample_step),
            "connector_pave_gap_fill_min_arc_gain": float(args.connector_pave_gap_fill_min_arc_gain),
            "segment_edge_policy": str(args.segment_edge_policy),
            "query_shortcut_boxes": bool(args.query_shortcut_boxes),
            "final_ompl_simplify_time_s": float(args.final_ompl_simplify_time_s),
            "enable_merger": bool(args.enable_merger),
            "root_intervals": D23_ROOT_INTERVALS,
            "cache_path": str(cache_path),
            "cache_exists": cache_path.exists(),
            "cache_bytes": directory_size(cache_path) if cache_path.exists() else 0,
            "snapshot_exists": (cache_path / "lect_snapshot").exists(),
        },
        "environment": environment_metadata(),
        "aggregate": aggregate(seed_rows),
        "seeds": seed_rows,
    }
    write_json(args.out_json, payload)
    write_markdown(args.out_md, payload)
    print(f"wrote {args.out_json}")
    print(f"wrote {args.out_md}")
    return 0 if all(row["ok"] for row in seed_rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
