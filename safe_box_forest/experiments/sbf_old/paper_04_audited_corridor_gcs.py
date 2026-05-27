#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_04_marcucci_combined import (  # noqa: E402
    ROOT,
    configure,
    parse_args as parse_exp4_args,
    query_payload,
    refine_corridors,
    sbf,
)
from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_04_merger_gcs import (  # noqa: E402
    boxes_intersect,
    interval_bounds,
    path_collision_audit,
    path_length,
)
from sbf.marcucci import make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audited corridor GCS over strict SBF query corridors.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_audited_corridor_gcs.json")
    parser.add_argument("--out-paths-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_audited_corridor_gcs_paths.json")
    parser.add_argument("--gcs-repo", type=Path, default=ROOT.parent.parent / "gcs-science-robotics")
    parser.add_argument("--query", default="all")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--quality-min-connected-boxes", type=int, default=64)
    parser.add_argument("--post-connect-time-budget-ms", type=float, default=450.0)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-merger", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--try-sbf-box-corridor", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--try-overlap-expanded-box-corridor", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--try-path-tube", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-neighbor-depth", type=int, default=0)
    parser.add_argument("--overlap-expansion-epsilon", type=float, default=1e-5)
    parser.add_argument("--tube-radius", type=float, default=0.003)
    parser.add_argument("--tube-sample-step", type=float, default=0.006)
    parser.add_argument("--max-tube-regions", type=int, default=1200)
    parser.add_argument("--gcs-relaxation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gcs-preprocessing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gcs-audit-step", type=float, default=0.04)
    parser.add_argument("--allow-failures", action="store_true")
    return parser.parse_args()


def make_exp4_args(args: argparse.Namespace) -> argparse.Namespace:
    exp4 = parse_exp4_args([])
    exp4.seed_base = int(args.seed_base)
    exp4.threads = int(args.threads)
    exp4.task_batch_size = 8
    exp4.quality_min_connected_boxes = int(args.quality_min_connected_boxes)
    exp4.post_connect_time_budget_ms = float(args.post_connect_time_budget_ms)
    exp4.corridor_refine = bool(args.corridor_refine)
    exp4.bridge_repaired_queries = bool(args.bridge_repaired_queries)
    exp4.enable_merger = bool(args.enable_merger)
    return exp4


def to_float_list(values: Any) -> list[float]:
    return [float(value) for value in values]


def run_query_with_bridge(forest: Any, query: Any, exp4_args: argparse.Namespace) -> tuple[Any, dict[str, Any]]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    initial_s = time.perf_counter() - query_t0
    bridge_s = 0.0
    retry_s = 0.0
    bridge_progress = 0
    should_bridge = (not result.success and exp4_args.bridge_failed_queries) or (
        bool(exp4_args.bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        bridge_progress = int(forest.bridge_query(list(query.start), list(query.goal)))
        bridge_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        retry_s = time.perf_counter() - retry_t0
    row = query_payload(query, result, initial_s + bridge_s + retry_s)
    row.update({
        "name": query.label,
        "bridge_progress": int(bridge_progress),
        "bridge_time_s": float(bridge_s),
        "initial_query_s": float(initial_s),
        "retry_query_s": float(retry_s),
        "box_sequence": [int(value) for value in result.box_sequence],
        "segment_edge_sequence": [int(value) for value in result.segment_edge_sequence],
        "waypoints": [to_float_list(waypoint) for waypoint in result.path],
        "waypoint_count": len(result.path),
    })
    return result, row


def global_joint_bounds(boxes: list[Any]) -> tuple[np.ndarray, np.ndarray]:
    lows: list[np.ndarray] = []
    highs: list[np.ndarray] = []
    for box in boxes:
        lo, hi = interval_bounds(box)
        lows.append(lo)
        highs.append(hi)
    if not lows:
        raise ValueError("cannot infer joint bounds without boxes")
    return np.min(np.vstack(lows), axis=0), np.max(np.vstack(highs), axis=0)


def sample_polyline(path: list[list[float]], step: float, max_regions: int) -> list[np.ndarray]:
    if len(path) < 2:
        return [np.asarray(point, dtype=float) for point in path]
    sampled: list[np.ndarray] = [np.asarray(path[0], dtype=float)]
    for index in range(len(path) - 1):
        start = np.asarray(path[index], dtype=float)
        goal = np.asarray(path[index + 1], dtype=float)
        distance = float(np.linalg.norm(goal - start))
        pieces = max(1, int(math.ceil(distance / max(float(step), 1e-6))))
        for sample in range(1, pieces + 1):
            alpha = sample / pieces
            sampled.append((1.0 - alpha) * start + alpha * goal)
    if len(sampled) <= max_regions:
        return sampled
    keep = np.linspace(0, len(sampled) - 1, max_regions)
    return [sampled[int(round(value))] for value in keep]


def solve_linear_gcs_regions(
    regions: dict[str, Any],
    edges: list[tuple[int, int]],
    source: list[float],
    target: list[float],
    args: argparse.Namespace,
) -> dict[str, Any]:
    sys.path.insert(0, str(args.gcs_repo.resolve()))
    from gcs.linear import LinearGCS
    from pydrake.solvers import CommonSolverOption, SolverOptions

    wall_t0 = time.perf_counter()
    try:
        gcs = LinearGCS(regions, edges)
        options = SolverOptions()
        options.SetOption(CommonSolverOption.kPrintToConsole, 0)
        gcs.setSolverOptions(options)
        gcs.addSourceTarget(np.asarray(source, dtype=float), np.asarray(target, dtype=float))
        waypoints, result_info = gcs.SolvePath(
            rounding=bool(args.gcs_relaxation),
            verbose=False,
            preprocessing=bool(args.gcs_preprocessing),
        )
    except Exception as exc:
        return {
            "ok": False,
            "error": str(exc),
            "solve_wall_s": float(time.perf_counter() - wall_t0),
            "vertex_count": len(regions),
            "edge_count": len(edges),
        }
    solve_s = time.perf_counter() - wall_t0
    if waypoints is None:
        return {
            "ok": False,
            "error": "SolvePath returned no waypoints",
            "solve_wall_s": float(solve_s),
            "vertex_count": len(regions),
            "edge_count": len(edges),
            "result_keys": sorted(str(key) for key in result_info.keys()),
        }
    array = np.asarray(waypoints, dtype=float)
    if array.ndim != 2:
        return {
            "ok": False,
            "error": f"unexpected waypoint array shape {array.shape}",
            "solve_wall_s": float(solve_s),
            "vertex_count": len(regions),
            "edge_count": len(edges),
        }
    if array.shape[0] == len(source):
        array = array.T
    path = [[float(value) for value in row] for row in array]
    return {
        "ok": True,
        "solve_wall_s": float(solve_s),
        "vertex_count": len(regions),
        "edge_count": len(edges),
        "length": path_length(path),
        "waypoint_count": len(path),
        "waypoints": path,
        "result_keys": sorted(str(key) for key in result_info.keys()),
    }


def make_sbf_box_corridor(forest: Any, result: Any, neighbor_depth: int) -> tuple[dict[str, Any], list[tuple[int, int]], dict[str, Any]]:
    from pydrake.geometry.optimization import HPolyhedron

    boxes = forest.boxes()
    adjacency = {int(key): [int(value) for value in values] for key, values in forest.adjacency().items()}
    box_by_id = {int(box.id): box for box in boxes}
    corridor_ids: set[int] = {int(value) for value in result.box_sequence if int(value) in box_by_id}
    frontier = set(corridor_ids)
    for _ in range(max(0, int(neighbor_depth))):
        next_frontier: set[int] = set()
        for box_id in frontier:
            for neighbor in adjacency.get(box_id, []):
                if neighbor in box_by_id and neighbor not in corridor_ids:
                    corridor_ids.add(neighbor)
                    next_frontier.add(neighbor)
        frontier = next_frontier
        if not frontier:
            break
    ordered_boxes = [box for box in boxes if int(box.id) in corridor_ids]
    local_by_id = {int(box.id): index for index, box in enumerate(ordered_boxes)}
    regions: dict[str, Any] = {}
    for index, box in enumerate(ordered_boxes):
        lo, hi = interval_bounds(box)
        regions[f"sbf_b{index}_id{int(box.id)}"] = HPolyhedron.MakeBox(lo, hi)
    edges: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    for source_id, neighbors in adjacency.items():
        if source_id not in local_by_id:
            continue
        for target_id in neighbors:
            if target_id not in local_by_id or source_id == target_id:
                continue
            source_index = local_by_id[source_id]
            target_index = local_by_id[target_id]
            if (source_index, target_index) in seen:
                continue
            if boxes_intersect(ordered_boxes[source_index], ordered_boxes[target_index]):
                edges.append((source_index, target_index))
                seen.add((source_index, target_index))
    metadata = {
        "mode": "sbf_box_corridor",
        "corridor_box_count": len(ordered_boxes),
        "corridor_ids": sorted(corridor_ids),
        "neighbor_depth": int(neighbor_depth),
    }
    return regions, edges, metadata


def ordered_query_boxes(forest: Any, result: Any) -> list[Any]:
    boxes = forest.boxes()
    box_by_id = {int(box.id): box for box in boxes}
    ordered: list[Any] = []
    last_id: int | None = None
    for value in result.box_sequence:
        box_id = int(value)
        if box_id == last_id or box_id not in box_by_id:
            continue
        ordered.append(box_by_id[box_id])
        last_id = box_id
    return ordered


def expand_bounds_to_include_point(
    lo: np.ndarray,
    hi: np.ndarray,
    point: list[float],
    lower: np.ndarray,
    upper: np.ndarray,
    epsilon: float,
) -> float:
    before_lo = lo.copy()
    before_hi = hi.copy()
    q = np.asarray(point, dtype=float)
    lo[:] = np.maximum(lower, np.minimum(lo, q - float(epsilon)))
    hi[:] = np.minimum(upper, np.maximum(hi, q + float(epsilon)))
    return float(np.sum(before_lo - lo) + np.sum(hi - before_hi))


def make_overlap_expanded_box_corridor(
    forest: Any,
    result: Any,
    source: list[float],
    target: list[float],
    lower: np.ndarray,
    upper: np.ndarray,
    epsilon: float,
) -> tuple[dict[str, Any], list[tuple[int, int]], dict[str, Any]]:
    from pydrake.geometry.optimization import HPolyhedron

    boxes = ordered_query_boxes(forest, result)
    if len(boxes) < 2:
        raise ValueError("overlap-expanded corridor requires at least two query boxes")

    bounds: list[tuple[np.ndarray, np.ndarray]] = []
    for box in boxes:
        lo, hi = interval_bounds(box)
        bounds.append((lo.copy(), hi.copy()))

    eps = max(float(epsilon), 0.0)
    gap_pair_count = 0
    gap_dim_count = 0
    max_gap = 0.0
    total_expansion = 0.0
    infeasible_pair_count = 0

    for index in range(len(bounds) - 1):
        lhs_lo, lhs_hi = bounds[index]
        rhs_lo, rhs_hi = bounds[index + 1]
        pair_had_gap = False
        for dim in range(len(lhs_lo)):
            if lhs_hi[dim] < rhs_lo[dim] - eps:
                gap = float(rhs_lo[dim] - lhs_hi[dim])
                midpoint = 0.5 * (lhs_hi[dim] + rhs_lo[dim])
                new_lhs_hi = min(float(upper[dim]), midpoint + eps)
                new_rhs_lo = max(float(lower[dim]), midpoint - eps)
                total_expansion += max(0.0, new_lhs_hi - float(lhs_hi[dim]))
                total_expansion += max(0.0, float(rhs_lo[dim]) - new_rhs_lo)
                lhs_hi[dim] = max(lhs_hi[dim], new_lhs_hi)
                rhs_lo[dim] = min(rhs_lo[dim], new_rhs_lo)
                pair_had_gap = True
                gap_dim_count += 1
                max_gap = max(max_gap, gap)
            elif rhs_hi[dim] < lhs_lo[dim] - eps:
                gap = float(lhs_lo[dim] - rhs_hi[dim])
                midpoint = 0.5 * (rhs_hi[dim] + lhs_lo[dim])
                new_rhs_hi = min(float(upper[dim]), midpoint + eps)
                new_lhs_lo = max(float(lower[dim]), midpoint - eps)
                total_expansion += max(0.0, new_rhs_hi - float(rhs_hi[dim]))
                total_expansion += max(0.0, float(lhs_lo[dim]) - new_lhs_lo)
                rhs_hi[dim] = max(rhs_hi[dim], new_rhs_hi)
                lhs_lo[dim] = min(lhs_lo[dim], new_lhs_lo)
                pair_had_gap = True
                gap_dim_count += 1
                max_gap = max(max_gap, gap)
        if pair_had_gap:
            gap_pair_count += 1
        if not bool(np.all(np.minimum(lhs_hi, rhs_hi) >= np.maximum(lhs_lo, rhs_lo) - eps)):
            infeasible_pair_count += 1

    total_expansion += expand_bounds_to_include_point(bounds[0][0], bounds[0][1], source, lower, upper, eps)
    total_expansion += expand_bounds_to_include_point(bounds[-1][0], bounds[-1][1], target, lower, upper, eps)

    regions: dict[str, Any] = {}
    for index, box in enumerate(boxes):
        lo, hi = bounds[index]
        regions[f"expanded_{index:03d}_id{int(box.id)}"] = HPolyhedron.MakeBox(lo, hi)
    edges = [(index, index + 1) for index in range(len(boxes) - 1)]
    metadata = {
        "mode": "overlap_expanded_sbf_corridor",
        "corridor_box_count": len(boxes),
        "corridor_ids": [int(box.id) for box in boxes],
        "gap_pair_count": int(gap_pair_count),
        "gap_dim_count": int(gap_dim_count),
        "infeasible_pair_count_after_expansion": int(infeasible_pair_count),
        "max_gap_before_expansion": float(max_gap),
        "total_l1_bound_expansion": float(total_expansion),
        "expansion_epsilon": float(eps),
        "safety_note": "expanded boxes are a GCS feasibility adapter and are counted only after strict path audit",
    }
    return regions, edges, metadata


def make_path_tube_corridor(
    path: list[list[float]],
    lower: np.ndarray,
    upper: np.ndarray,
    radius: float,
    sample_step: float,
    max_regions: int,
) -> tuple[dict[str, Any], list[tuple[int, int]], dict[str, Any]]:
    from pydrake.geometry.optimization import HPolyhedron

    points = sample_polyline(path, sample_step, max_regions)
    if len(points) < 2:
        raise ValueError("path tube requires at least two sampled points")
    max_observed_step = max(float(np.linalg.norm(points[index + 1] - points[index])) for index in range(len(points) - 1))
    effective_radius = max(float(radius), 0.525 * max(float(sample_step), max_observed_step))
    regions: dict[str, Any] = {}
    for index, point in enumerate(points):
        lo = np.maximum(lower, point - effective_radius)
        hi = np.minimum(upper, point + effective_radius)
        regions[f"tube_{index:03d}"] = HPolyhedron.MakeBox(lo, hi)
    edges = [(index, index + 1) for index in range(len(points) - 1)]
    metadata = {
        "mode": "path_tube",
        "tube_region_count": len(points),
        "tube_radius": float(effective_radius),
        "tube_sample_step": float(sample_step),
        "source_path_length": path_length(path),
    }
    return regions, edges, metadata


def run_gcs_attempt(
    name: str,
    regions: dict[str, Any],
    edges: list[tuple[int, int]],
    metadata: dict[str, Any],
    query: Any,
    robot: Any,
    obstacles: list[Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    row = solve_linear_gcs_regions(regions, edges, list(query.start), list(query.goal), args)
    row.update({"name": query.label, "attempt": name, "metadata": metadata})
    if row.get("ok"):
        row["audit"] = path_collision_audit(robot, obstacles, row["waypoints"], float(args.gcs_audit_step))
        row["strict_audit_passed"] = bool(row["audit"].get("passed"))
    else:
        row["audit"] = {"passed": False, "failed_segment_index": -1, "checked_samples": 0}
        row["strict_audit_passed"] = False
    return row


def main() -> int:
    args = parse_args()
    exp4_args = make_exp4_args(args)
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    if args.query != "all":
        queries = [query for query in queries if query.label == args.query]
        if not queries:
            raise ValueError(f"unknown query label {args.query!r}")
    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]

    cfg = configure(exp4_args, int(args.seed))
    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, exp4_args)
    build_s = time.perf_counter() - build_t0
    lower, upper = global_joint_bounds(forest.boxes())

    query_rows: list[dict[str, Any]] = []
    path_rows: list[dict[str, Any]] = []
    for query in queries:
        result, sbf_row = run_query_with_bridge(forest, query, exp4_args)
        sbf_audit = path_collision_audit(robot, obstacles, sbf_row["waypoints"], float(args.gcs_audit_step))
        attempts: list[dict[str, Any]] = []

        if bool(args.try_sbf_box_corridor) and sbf_row.get("ok"):
            try:
                regions, edges, metadata = make_sbf_box_corridor(forest, result, int(args.corridor_neighbor_depth))
                attempts.append(run_gcs_attempt("sbf_box_corridor", regions, edges, metadata, query, robot, obstacles, args))
            except Exception as exc:
                attempts.append({
                    "name": query.label,
                    "attempt": "sbf_box_corridor",
                    "ok": False,
                    "strict_audit_passed": False,
                    "error": str(exc),
                    "audit": {"passed": False, "failed_segment_index": -1, "checked_samples": 0},
                })

        audited_gcs = next((row for row in attempts if bool(row.get("strict_audit_passed"))), None)
        if audited_gcs is None and bool(args.try_overlap_expanded_box_corridor) and sbf_row.get("ok"):
            try:
                query_lower = np.minimum(lower, np.minimum(np.asarray(query.start, dtype=float), np.asarray(query.goal, dtype=float)))
                query_upper = np.maximum(upper, np.maximum(np.asarray(query.start, dtype=float), np.asarray(query.goal, dtype=float)))
                regions, edges, metadata = make_overlap_expanded_box_corridor(
                    forest,
                    result,
                    list(query.start),
                    list(query.goal),
                    query_lower,
                    query_upper,
                    float(args.overlap_expansion_epsilon),
                )
                expanded_row = run_gcs_attempt("overlap_expanded_sbf_corridor", regions, edges, metadata, query, robot, obstacles, args)
                attempts.append(expanded_row)
                if bool(expanded_row.get("strict_audit_passed")):
                    audited_gcs = expanded_row
            except Exception as exc:
                attempts.append({
                    "name": query.label,
                    "attempt": "overlap_expanded_sbf_corridor",
                    "ok": False,
                    "strict_audit_passed": False,
                    "error": str(exc),
                    "audit": {"passed": False, "failed_segment_index": -1, "checked_samples": 0},
                })

        if audited_gcs is None and bool(args.try_path_tube) and sbf_row.get("ok"):
            try:
                regions, edges, metadata = make_path_tube_corridor(
                    sbf_row["waypoints"],
                    lower,
                    upper,
                    float(args.tube_radius),
                    float(args.tube_sample_step),
                    int(args.max_tube_regions),
                )
                tube_row = run_gcs_attempt("audited_path_tube", regions, edges, metadata, query, robot, obstacles, args)
                attempts.append(tube_row)
                if bool(tube_row.get("strict_audit_passed")):
                    audited_gcs = tube_row
            except Exception as exc:
                attempts.append({
                    "name": query.label,
                    "attempt": "audited_path_tube",
                    "ok": False,
                    "strict_audit_passed": False,
                    "error": str(exc),
                    "audit": {"passed": False, "failed_segment_index": -1, "checked_samples": 0},
                })

        if audited_gcs is not None:
            final_source = "gcs_strict_audited"
            final_path = audited_gcs["waypoints"]
            final_audit = audited_gcs["audit"]
            final_length = audited_gcs.get("length", path_length(final_path))
        else:
            final_source = "sbf_audited_fallback"
            final_path = sbf_row["waypoints"]
            final_audit = sbf_audit
            final_length = sbf_row.get("length", path_length(final_path))

        query_row = {
            "name": query.label,
            "sbf": sbf_row,
            "sbf_audit": sbf_audit,
            "gcs_attempts": attempts,
            "gcs_success": audited_gcs is not None,
            "gcs_strict_audit_passed": audited_gcs is not None,
            "final_source": final_source,
            "final_success": bool(final_audit.get("passed")),
            "final_strict_audit_passed": bool(final_audit.get("passed")),
            "final_length": float(final_length) if final_length is not None else None,
            "final_length_over_sbf": float(final_length) / float(sbf_row["length"]) if sbf_row.get("length") else None,
            "final_audit": final_audit,
        }
        query_rows.append(query_row)
        path_rows.append({
            "name": query.label,
            "final_source": final_source,
            "final_path": final_path,
            "sbf_path": sbf_row["waypoints"],
            "gcs_attempts": [{
                "attempt": attempt.get("attempt"),
                "ok": attempt.get("ok"),
                "strict_audit_passed": attempt.get("strict_audit_passed"),
                "path": attempt.get("waypoints", []),
            } for attempt in attempts],
        })

    payload = {
        "experiment": "paper_04_audited_corridor_gcs",
        "params": {
            "quality_min_connected_boxes": int(exp4_args.quality_min_connected_boxes),
            "post_connect_time_budget_ms": float(exp4_args.post_connect_time_budget_ms),
            "corridor_refine": bool(exp4_args.corridor_refine),
            "bridge_repaired_queries": bool(exp4_args.bridge_repaired_queries),
            "enable_merger": bool(exp4_args.enable_merger),
            "try_sbf_box_corridor": bool(args.try_sbf_box_corridor),
            "try_overlap_expanded_box_corridor": bool(args.try_overlap_expanded_box_corridor),
            "try_path_tube": bool(args.try_path_tube),
            "corridor_neighbor_depth": int(args.corridor_neighbor_depth),
            "overlap_expansion_epsilon": float(args.overlap_expansion_epsilon),
            "tube_radius": float(args.tube_radius),
            "tube_sample_step": float(args.tube_sample_step),
            "max_tube_regions": int(args.max_tube_regions),
            "gcs_audit_step": float(args.gcs_audit_step),
        },
        "build": {
            "wall_s": float(build_s),
            "prebridge_time_s": float(prebridge_time_s),
            "prebridge_added_boxes": int(prebridge_added_boxes),
            "prebridge_attempts": int(prebridge_attempts),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "boxes_after_refine": len(forest.boxes()),
            "segment_edges": len(forest.segment_edges()),
            "grow_ms": float(profile.grow_ms),
            "merge_ms": float(profile.merge_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "diagnostics": {str(key): float(value) for key, value in dict(profile.diagnostics).items()},
        },
        "queries": query_rows,
        "summary": {
            "query_count": len(query_rows),
            "gcs_solver_ok_count": sum(1 for row in query_rows for attempt in row["gcs_attempts"] if attempt.get("ok")),
            "gcs_strict_audit_pass_count": sum(1 for row in query_rows if row.get("gcs_strict_audit_passed")),
            "final_success_count": sum(1 for row in query_rows if row.get("final_success")),
            "fallback_count": sum(1 for row in query_rows if row.get("final_source") == "sbf_audited_fallback"),
        },
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    args.out_paths_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_paths_json.write_text(json.dumps({"paths": path_rows}, indent=2, sort_keys=True), encoding="utf-8")

    print(json.dumps({
        "out_json": str(args.out_json),
        "out_paths_json": str(args.out_paths_json),
        "build_s": build_s,
        "boxes_after_refine": len(forest.boxes()),
        "summary": payload["summary"],
        "queries": [{
            "name": row["name"],
            "gcs_success": row["gcs_success"],
            "final_source": row["final_source"],
            "final_success": row["final_success"],
            "final_length": row["final_length"],
        } for row in query_rows],
    }, indent=2, sort_keys=True))
    if not args.allow_failures and payload["summary"]["final_success_count"] != len(query_rows):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())