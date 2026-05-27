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

from paper_04_marcucci_combined import (  # noqa: E402
    ROOT,
    configure,
    parse_args as parse_exp4_args,
    query_payload,
    refine_corridors,
    sbf,
)
from sbf.marcucci import make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build SBF with merger enabled and try GCS planning over generated boxes.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_merger_gcs.json")
    parser.add_argument("--out-paths-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_merger_gcs_paths.json")
    parser.add_argument("--out-diagnostics-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_merger_gcs_diagnostics.json")
    parser.add_argument("--gcs-repo", type=Path, default=ROOT.parent.parent / "gcs-science-robotics")
    parser.add_argument("--query", default="all")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--quality-min-connected-boxes", type=int, default=64)
    parser.add_argument("--post-connect-time-budget-ms", type=float, default=450.0)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--merger-target-boxes", type=int, default=0)
    parser.add_argument("--merger-max-rounds", type=int, default=20)
    parser.add_argument("--merger-score-threshold", type=float, default=200.0)
    parser.add_argument("--gcs-relaxation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gcs-preprocessing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gcs-audit-step", type=float, default=0.04)
    parser.add_argument("--allow-gcs-failures", action="store_true")
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
    exp4.enable_merger = True
    return exp4


def interval_bounds(box: Any) -> tuple[np.ndarray, np.ndarray]:
    lo = np.array([float(interval.lo) for interval in box.joint_intervals], dtype=float)
    hi = np.array([float(interval.hi) for interval in box.joint_intervals], dtype=float)
    return lo, hi


def boxes_intersect(lhs: Any, rhs: Any, tolerance: float = 1e-9) -> bool:
    lhs_lo, lhs_hi = interval_bounds(lhs)
    rhs_lo, rhs_hi = interval_bounds(rhs)
    return bool(np.all(np.minimum(lhs_hi, rhs_hi) >= np.maximum(lhs_lo, rhs_lo) - tolerance))


def geometric_edges(boxes: list[Any], adjacency: dict[int, list[int]]) -> list[tuple[int, int]]:
    id_to_index = {int(box.id): index for index, box in enumerate(boxes)}
    edges: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    for source_id, neighbors in adjacency.items():
        source_id = int(source_id)
        if source_id not in id_to_index:
            continue
        source_index = id_to_index[source_id]
        for target_id in neighbors:
            target_id = int(target_id)
            if target_id not in id_to_index:
                continue
            target_index = id_to_index[target_id]
            if source_index == target_index:
                continue
            key = (source_index, target_index)
            if key in seen:
                continue
            if boxes_intersect(boxes[source_index], boxes[target_index]):
                edges.append(key)
                seen.add(key)
    return edges


def path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return 0.0
    return float(sum(np.linalg.norm(np.asarray(path[index + 1]) - np.asarray(path[index])) for index in range(len(path) - 1)))


def path_collision_audit(robot: Any, obstacles: list[Any], path: list[list[float]], step: float) -> dict[str, Any]:
    if len(path) < 2:
        return {"passed": False, "failed_segment_index": -1, "checked_samples": 0}
    checked = 0
    for index in range(len(path) - 1):
        a = np.asarray(path[index], dtype=float)
        b = np.asarray(path[index + 1], dtype=float)
        distance = float(np.linalg.norm(b - a))
        samples = max(2, int(math.ceil(distance / max(float(step), 1e-6))))
        for sample in range(samples + 1):
            alpha = sample / samples
            point = (1.0 - alpha) * a + alpha * b
            checked += 1
            if sbf.check_config_collision(robot, obstacles, [float(value) for value in point]):
                return {
                    "passed": False,
                    "failed_segment_index": index,
                    "checked_samples": checked,
                    "failed_alpha": float(alpha),
                    "failed_point": [float(value) for value in point],
                    "failed_segment_start": [float(value) for value in a],
                    "failed_segment_goal": [float(value) for value in b],
                    "failed_segment_length": float(distance),
                }
    return {"passed": True, "failed_segment_index": -1, "checked_samples": checked}


def point_to_segment_distance(point: np.ndarray, start: np.ndarray, goal: np.ndarray) -> float:
    delta = goal - start
    denom = float(np.dot(delta, delta))
    if denom <= 1e-18:
        return float(np.linalg.norm(point - start))
    alpha = float(np.clip(np.dot(point - start, delta) / denom, 0.0, 1.0))
    closest = start + alpha * delta
    return float(np.linalg.norm(point - closest))


def point_to_path_distance(point: list[float] | None, path: list[list[float]]) -> float | None:
    if point is None or len(path) < 2:
        return None
    query = np.asarray(point, dtype=float)
    return min(
        point_to_segment_distance(query, np.asarray(path[index], dtype=float), np.asarray(path[index + 1], dtype=float))
        for index in range(len(path) - 1)
    )


def containing_box_ids(boxes: list[Any], point: list[float] | None, tolerance: float = 1e-9) -> list[int]:
    if point is None:
        return []
    q = np.asarray(point, dtype=float)
    ids: list[int] = []
    for box in boxes:
        lo, hi = interval_bounds(box)
        if bool(np.all(q >= lo - tolerance) and np.all(q <= hi + tolerance)):
            ids.append(int(box.id))
    return ids


def annotate_gcs_diagnostics(gcs_row: dict[str, Any], sbf_row: dict[str, Any] | None, boxes: list[Any]) -> dict[str, Any]:
    audit = gcs_row.get("audit", {})
    failed_point = audit.get("failed_point") if isinstance(audit, dict) else None
    sbf_path = sbf_row.get("waypoints", []) if sbf_row else []
    diagnostic = {
        "name": gcs_row.get("name"),
        "gcs_solver_ok": bool(gcs_row.get("ok")),
        "gcs_strict_audit_passed": bool(audit.get("passed")) if isinstance(audit, dict) else False,
        "solved_but_unsafe": bool(gcs_row.get("ok")) and not bool(audit.get("passed")) if isinstance(audit, dict) else bool(gcs_row.get("ok")),
        "gcs_length": gcs_row.get("length"),
        "sbf_length": sbf_row.get("length") if sbf_row else None,
        "failed_segment_index": audit.get("failed_segment_index") if isinstance(audit, dict) else None,
        "checked_samples": audit.get("checked_samples") if isinstance(audit, dict) else None,
        "failed_point": failed_point,
        "failed_point_containing_box_ids": containing_box_ids(boxes, failed_point),
        "failed_point_distance_to_sbf_path": point_to_path_distance(failed_point, sbf_path),
        "remaining_unsafe_assumptions": sbf_row.get("remaining_unsafe_assumptions") if sbf_row else None,
        "sbf_audit_passed": bool(sbf_row.get("audit_passed")) if sbf_row else None,
    }
    if diagnostic["gcs_length"] and diagnostic["sbf_length"]:
        diagnostic["gcs_length_over_sbf_length"] = float(diagnostic["gcs_length"]) / float(diagnostic["sbf_length"])
    return diagnostic


def solve_gcs_for_query(boxes: list[Any], edges: list[tuple[int, int]], query: Any, args: argparse.Namespace) -> dict[str, Any]:
    sys.path.insert(0, str(args.gcs_repo.resolve()))
    from gcs.linear import LinearGCS
    from pydrake.geometry.optimization import HPolyhedron
    from pydrake.solvers import CommonSolverOption, SolverOptions

    regions = {}
    for index, box in enumerate(boxes):
        lo, hi = interval_bounds(box)
        regions[f"b{index}_id{int(box.id)}"] = HPolyhedron.MakeBox(lo, hi)

    wall_t0 = time.perf_counter()
    gcs = LinearGCS(regions, edges)
    options = SolverOptions()
    options.SetOption(CommonSolverOption.kPrintToConsole, 0)
    gcs.setSolverOptions(options)
    try:
        gcs.addSourceTarget(np.asarray(query.start, dtype=float), np.asarray(query.goal, dtype=float))
        waypoints, result_info = gcs.SolvePath(rounding=bool(args.gcs_relaxation), verbose=False, preprocessing=bool(args.gcs_preprocessing))
    except Exception as exc:  # Drake/GCS errors should be recorded, not hide build results.
        return {
            "name": query.label,
            "ok": False,
            "error": str(exc),
            "solve_wall_s": float(time.perf_counter() - wall_t0),
            "vertex_count": len(regions),
            "edge_count": len(edges),
        }
    solve_s = time.perf_counter() - wall_t0
    if waypoints is None:
        return {
            "name": query.label,
            "ok": False,
            "error": "SolvePath returned no waypoints",
            "solve_wall_s": float(solve_s),
            "vertex_count": len(regions),
            "edge_count": len(edges),
            "result_keys": sorted(str(key) for key in result_info.keys()),
        }
    array = np.asarray(waypoints, dtype=float)
    if array.ndim != 2:
        raise ValueError(f"unexpected GCS waypoint array shape {array.shape}")
    if array.shape[0] == len(query.start):
        array = array.T
    path = [[float(value) for value in row] for row in array]
    return {
        "name": query.label,
        "ok": True,
        "solve_wall_s": float(solve_s),
        "vertex_count": len(regions),
        "edge_count": len(edges),
        "length": path_length(path),
        "waypoints": path,
        "waypoint_count": len(path),
        "result_keys": sorted(str(key) for key in result_info.keys()),
    }


def run_sbf_query(forest: Any, query: Any, exp4_args: argparse.Namespace) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    query_s = time.perf_counter() - query_t0
    should_bridge = (not result.success and exp4_args.bridge_failed_queries) or (
        bool(exp4_args.bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        progress = int(forest.bridge_query(list(query.start), list(query.goal)))
        bridge_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        retry_s = time.perf_counter() - retry_t0
        row = query_payload(query, result, query_s + bridge_s + retry_s)
        row["name"] = query.label
        row["bridge_progress"] = int(progress)
        row["bridge_time_s"] = float(bridge_s)
    else:
        row = query_payload(query, result, query_s)
        row["name"] = query.label
        row["bridge_progress"] = 0
        row["bridge_time_s"] = 0.0
    row["waypoints"] = [[float(value) for value in waypoint] for waypoint in result.path]
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
    cfg.enable_merger = True
    cfg.merger.target_boxes = int(args.merger_target_boxes)
    cfg.merger.max_rounds = int(args.merger_max_rounds)
    cfg.merger.score_threshold = float(args.merger_score_threshold)
    cfg.merger.n_threads = max(1, int(args.threads))
    cfg.merger.candidate_batch_size = 64
    forest = sbf.SafeBoxForest(robot, cfg)

    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, exp4_args)
    build_s = time.perf_counter() - build_t0

    sbf_rows = [run_sbf_query(forest, query, exp4_args) for query in queries]
    boxes = forest.boxes()
    adjacency = forest.adjacency()
    edges = geometric_edges(boxes, adjacency)
    gcs_rows: list[dict[str, Any]] = []
    for query in queries:
        row = solve_gcs_for_query(boxes, edges, query, args)
        if row.get("ok"):
            row["audit"] = path_collision_audit(robot, obstacles, row["waypoints"], float(args.gcs_audit_step))
        gcs_rows.append(row)
    sbf_by_name = {row["name"]: row for row in sbf_rows}
    diagnostic_rows = [annotate_gcs_diagnostics(row, sbf_by_name.get(row.get("name")), boxes) for row in gcs_rows]

    payload = {
        "experiment": "paper_04_marcucci_merger_gcs",
        "params": {
            "quality_min_connected_boxes": int(exp4_args.quality_min_connected_boxes),
            "post_connect_time_budget_ms": float(exp4_args.post_connect_time_budget_ms),
            "corridor_refine": bool(exp4_args.corridor_refine),
            "enable_merger": True,
            "merger_target_boxes": int(args.merger_target_boxes),
            "merger_max_rounds": int(args.merger_max_rounds),
            "merger_score_threshold": float(args.merger_score_threshold),
            "gcs_relaxation": bool(args.gcs_relaxation),
            "gcs_preprocessing": bool(args.gcs_preprocessing),
        },
        "build": {
            "wall_s": float(build_s),
            "prebridge_time_s": float(prebridge_time_s),
            "prebridge_added_boxes": int(prebridge_added_boxes),
            "prebridge_attempts": int(prebridge_attempts),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "boxes_after_refine": len(boxes),
            "segment_edges": len(forest.segment_edges()),
            "merge_ms": float(profile.merge_ms),
            "grow_ms": float(profile.grow_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "adjacency_edges_geometric_for_gcs": len(edges),
            "diagnostics": {str(key): float(value) for key, value in dict(profile.diagnostics).items()},
        },
        "sbf_queries": sbf_rows,
        "gcs_queries": gcs_rows,
        "diagnostics": diagnostic_rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    args.out_paths_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_paths_json.write_text(json.dumps({"gcs_paths": gcs_rows, "sbf_paths": sbf_rows}, indent=2, sort_keys=True), encoding="utf-8")
    args.out_diagnostics_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_diagnostics_json.write_text(json.dumps({
        "experiment": "paper_04_marcucci_merger_gcs_diagnostics",
        "interpretation": "Direct GCS over provisional SBF boxes is a solved-but-unsafe control unless strict audit passes.",
        "build": payload["build"],
        "diagnostics": diagnostic_rows,
    }, indent=2, sort_keys=True), encoding="utf-8")
    summary = {
        "out_json": str(args.out_json),
        "out_paths_json": str(args.out_paths_json),
        "out_diagnostics_json": str(args.out_diagnostics_json),
        "build_s": build_s,
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "boxes_after_refine": len(boxes),
        "gcs_success_count": sum(1 for row in gcs_rows if row.get("ok")),
        "gcs_audit_pass_count": sum(1 for row in gcs_rows if row.get("audit", {}).get("passed")),
        "gcs_queries": [{"name": row["name"], "ok": row.get("ok"), "length": row.get("length"), "audit": row.get("audit")} for row in gcs_rows],
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    if not args.allow_gcs_failures and summary["gcs_success_count"] == 0:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())