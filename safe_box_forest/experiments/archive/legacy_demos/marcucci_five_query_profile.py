#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any


def _bootstrap_imports() -> Path:
    root = Path(__file__).resolve().parents[3]
    for candidate in (root / "python", root / "build" / "python", root / "build_py310" / "python"):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root


ROOT = _bootstrap_imports()

import sbf
from sbf.marcucci import make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Profile standalone SBF on the full Marcucci scene with all five query pairs.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "marcucci_five_query" / "critsample_link_grower_only.json")
    parser.add_argument("--attempt", default="critsample_link_grower_only")
    parser.add_argument("--endpoint-source", choices=["ifk", "critsample"], default="critsample")
    parser.add_argument("--envelope", choices=["link", "support_hull"], default="link")
    parser.add_argument("--max-boxes", type=int, default=3000)
    parser.add_argument("--timeout-ms", type=float, default=120000.0)
    parser.add_argument("--ffb-depth", type=int, default=80)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--task-batch-size", type=int, default=1)
    parser.add_argument("--component-connect-prob", type=float, default=0.55)
    parser.add_argument("--component-connect-candidate-limit", type=int, default=4)
    parser.add_argument("--component-connect-island-aware", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--component-connect-frontier-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--component-connect-staged-growth", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--component-connect-stage-normalized-linf", type=float, default=0.35)
    parser.add_argument("--component-connect-neighbor-root-bias", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--component-connect-neighbor-root-window", type=int, default=1)
    parser.add_argument("--component-connect-max-parent-failures", type=int, default=8)
    parser.add_argument("--component-connect-ffb-depth-increment", type=int, default=40)
    parser.add_argument("--component-connect-ffb-max-depth", type=int, default=120)
    parser.add_argument("--component-connect-depth-after-unknown-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extra-random-roots", type=int, default=0)
    parser.add_argument("--root-seed-candidate-count", type=int, default=128)
    parser.add_argument("--root-seed-min-normalized-linf", type=float, default=0.25)
    parser.add_argument("--root-seed-max-lca-depth", type=int, default=2)
    parser.add_argument("--path-guided-seeds", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--path-guided-only-seeds", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--path-guided-root-only", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--path-guided-query-indices", default="all")
    parser.add_argument("--path-guided-relax-root-filter", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--path-guided-rrt-iters", type=int, default=30000)
    parser.add_argument("--path-guided-rrt-timeout-ms", type=float, default=4000.0)
    parser.add_argument("--path-guided-rrt-step-size", type=float, default=0.16)
    parser.add_argument("--path-guided-rrt-goal-bias", type=float, default=0.30)
    parser.add_argument("--path-guided-rrt-segment-resolution", type=int, default=16)
    parser.add_argument("--path-guided-rrt-attempts", type=int, default=3)
    parser.add_argument("--path-guided-sample-step", type=float, default=0.35)
    parser.add_argument("--path-guided-max-samples-per-query", type=int, default=24)
    parser.add_argument("--path-guided-seed-base", type=int, default=20260504)
    parser.add_argument("--path-guided-coverage-tol", type=float, default=0.0)
    parser.add_argument("--path-guided-uncovered-limit", type=int, default=12)
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--bridge-ffb-depth", type=int, default=160)
    parser.add_argument("--bridge-max-chain", type=int, default=250)
    parser.add_argument("--bridge-steps-per-waypoint", type=int, default=12)
    parser.add_argument("--rrt-goal-bias", type=float, default=0.2)
    parser.add_argument("--intertree-goal-bias", type=float, default=0.2)
    parser.add_argument("--unexplored-prob", type=float, default=0.45)
    parser.add_argument("--step-ratio", type=float, default=0.08)
    parser.add_argument("--stop-after-connect", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-merger", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--trace-path", type=Path)
    parser.add_argument("--trace-max-events", type=int, default=50000)
    parser.add_argument("--trace-face-candidate-limit", type=int, default=8)
    parser.add_argument("--root-pair-gap-limit", type=int, default=80)
    parser.add_argument("--allow-failures", action="store_true")
    return parser.parse_args()


def configure(args: argparse.Namespace) -> sbf.SBFConfig:
    config = sbf.SBFConfig()
    config.enable_merger = bool(args.enable_merger)
    config.enable_connector = False
    config.endpoint_source.source = sbf.EndpointSource.IFK if args.endpoint_source == "ifk" else sbf.EndpointSource.CritSample
    envelope_map = {
        "link": sbf.EnvelopeType.LinkIAABB,
        "support_hull": sbf.EnvelopeType.SupportHull,
    }
    config.envelope_type.type = envelope_map[args.envelope]
    config.envelope_type.n_subdivisions = 4

    config.runtime.mode = sbf.ExecutionMode.Parallel if args.threads > 1 else sbf.ExecutionMode.Inline
    config.runtime.n_threads = max(1, int(args.threads))
    config.runtime.batch_size = max(1, int(args.task_batch_size))
    config.runtime.parallel_threshold = 1

    config.grower.mode = sbf.GrowerMode.RRT
    config.grower.max_boxes = int(args.max_boxes)
    config.grower.timeout_ms = float(args.timeout_ms)
    config.grower.max_consecutive_miss = 6000
    config.grower.rng_seed = 20260504
    config.grower.n_threads = max(1, int(args.threads))
    config.grower.task_batch_size = max(1, int(args.task_batch_size))
    config.grower.parallel_threshold = 1
    config.grower.worker_local_ffb = True
    config.grower.rrt_goal_bias = float(args.rrt_goal_bias)
    config.grower.intertree_goal_bias = float(args.intertree_goal_bias)
    config.grower.sustained_goal_bias_cap = min(0.25, float(args.intertree_goal_bias))
    config.grower.high_goal_bias_pulse_period = 8
    config.grower.expand_all_roots_per_sample = True
    config.grower.extra_random_roots = int(args.extra_random_roots)
    config.grower.root_seed_candidate_count = int(args.root_seed_candidate_count)
    config.grower.root_seed_min_normalized_linf = float(args.root_seed_min_normalized_linf)
    if args.path_guided_seeds and args.path_guided_relax_root_filter:
        config.grower.root_seed_max_lca_depth = -1
    else:
        config.grower.root_seed_max_lca_depth = int(args.root_seed_max_lca_depth)
    config.grower.root_seed_include_user_seeds = True
    config.grower.component_connect_prob = float(args.component_connect_prob)
    config.grower.component_connect_candidate_limit = int(args.component_connect_candidate_limit)
    config.grower.component_connect_island_aware = bool(args.component_connect_island_aware)
    config.grower.component_connect_frontier_cache = bool(args.component_connect_frontier_cache)
    config.grower.component_connect_staged_growth = bool(args.component_connect_staged_growth)
    config.grower.component_connect_stage_normalized_linf = float(args.component_connect_stage_normalized_linf)
    neighbor_root_bias = bool(args.path_guided_seeds) if args.component_connect_neighbor_root_bias is None else bool(args.component_connect_neighbor_root_bias)
    config.grower.component_connect_neighbor_root_bias = neighbor_root_bias
    config.grower.component_connect_neighbor_root_window = int(args.component_connect_neighbor_root_window)
    config.grower.component_connect_max_parent_failures = int(args.component_connect_max_parent_failures)
    config.grower.component_connect_adaptive_ffb = True
    config.grower.component_connect_ffb_depth_increment = int(args.component_connect_ffb_depth_increment)
    config.grower.component_connect_ffb_max_depth = int(args.component_connect_ffb_max_depth)
    config.grower.component_connect_depth_after_unknown_only = bool(args.component_connect_depth_after_unknown_only)
    config.grower.rrt_step_ratio = float(args.step_ratio)
    config.grower.unexplored_sample_prob = float(args.unexplored_prob)
    config.grower.connect_mode = True
    config.grower.stop_after_connect = bool(args.stop_after_connect)
    config.grower.post_connect_extra_boxes = 0
    config.grower.find_free_box.max_depth = int(args.ffb_depth)
    config.grower.find_free_box.deadline_ms = 0.0
    config.grower.find_free_box.split_reserved_leaf = True
    config.grower.find_free_box.split_unknown_leaf = True
    config.grower.find_free_box.reject_seed_collision = False
    config.grower.n_boundary_samples = 8
    config.grower.goal_face_bias = 0.75
    if args.trace_path is not None:
        config.grower.trace_enabled = True
        config.grower.trace_path = str(args.trace_path)
        config.grower.trace_max_events = int(args.trace_max_events)
        config.grower.trace_face_candidate_limit = int(args.trace_face_candidate_limit)

    config.query.nearest_if_outside = False
    config.query.shortcut_boxes = True

    config.connector.rrt.max_iters = int(args.path_guided_rrt_iters)
    config.connector.rrt.timeout_ms = float(args.path_guided_rrt_timeout_ms)
    config.connector.rrt.step_size = float(args.path_guided_rrt_step_size)
    config.connector.rrt.goal_bias = float(args.path_guided_rrt_goal_bias)
    config.connector.rrt.segment_resolution = int(args.path_guided_rrt_segment_resolution)
    config.connector.pave.find_free_box.max_depth = int(args.bridge_ffb_depth)
    config.connector.pave.find_free_box.split_reserved_leaf = True
    config.connector.pave.find_free_box.split_unknown_leaf = True
    config.connector.pave.find_free_box.reject_seed_collision = False
    config.connector.pave.max_chain = int(args.bridge_max_chain)
    config.connector.pave.max_steps_per_waypoint = int(args.bridge_steps_per_waypoint)
    return config


def profile_to_dict(profile: sbf.BuildProfile) -> dict[str, Any]:
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    return {
        "total_ms": float(profile.total_ms),
        "grow_ms": float(profile.grow_ms),
        "merge_ms": float(profile.merge_ms),
        "connector_ms": float(profile.connector_ms),
        "adjacency_ms": float(profile.adjacency_ms),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "bridge_boxes_added": int(profile.bridge_boxes_added),
        "connector_attempted_pairs": int(profile.connector_attempted_pairs),
        "connector_connected": bool(profile.connector_connected),
        "adjacency_islands": int(profile.adjacency_islands),
        "diagnostics": diagnostics,
        "function_profile": function_profile(diagnostics),
    }


def function_profile(diagnostics: dict[str, float]) -> dict[str, dict[str, float]]:
    prefix = "profile."
    suffix = ".calls"
    names = sorted(
        key[len(prefix):-len(suffix)]
        for key in diagnostics
        if key.startswith(prefix) and key.endswith(suffix)
    )
    out: dict[str, dict[str, float]] = {}
    for name in names:
        calls = diagnostics.get(f"profile.{name}.calls", 0.0)
        total_ms = diagnostics.get(f"profile.{name}.total_ms", 0.0)
        out[name] = {
            "calls": calls,
            "total_ms": total_ms,
            "mean_ms": total_ms / calls if calls > 0.0 else 0.0,
            "min_ms": diagnostics.get(f"profile.{name}.min_ms", 0.0),
            "max_ms": diagnostics.get(f"profile.{name}.max_ms", 0.0),
        }
    return out


def component_sizes(adjacency: dict[int, list[int]]) -> list[int]:
    unseen = set(int(node) for node in adjacency)
    sizes: list[int] = []
    while unseen:
        root = unseen.pop()
        queue: deque[int] = deque([root])
        size = 0
        while queue:
            current = queue.popleft()
            size += 1
            for neighbor in adjacency.get(current, []):
                neighbor = int(neighbor)
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    queue.append(neighbor)
        sizes.append(size)
    return sorted(sizes, reverse=True)


def parse_query_indices(text: str, n_queries: int) -> set[int]:
    value = text.strip().lower()
    if not value or value == "all":
        return set(range(n_queries))
    indices: set[int] = set()
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            lhs, rhs = token.split("-", 1)
            start = int(lhs)
            stop = int(rhs)
            if stop < start:
                start, stop = stop, start
            indices.update(range(start, stop + 1))
        else:
            indices.add(int(token))
    return {index for index in indices if 0 <= index < n_queries}


def point_key(point: list[float] | tuple[float, ...], digits: int = 7) -> tuple[float, ...]:
    return tuple(round(float(value), digits) for value in point)


def path_length_values(path: list[list[float]]) -> float:
    return sum(math.dist(lhs, rhs) for lhs, rhs in zip(path, path[1:]))


def resample_path(path: list[list[float]], sample_step: float, max_samples: int) -> list[list[float]]:
    if not path:
        return []
    step = max(float(sample_step), 1e-9)
    samples: list[list[float]] = [[float(value) for value in path[0]]]
    for start, goal in zip(path, path[1:]):
        distance = math.dist(start, goal)
        n_steps = max(1, int(math.ceil(distance / step)))
        for step_index in range(1, n_steps + 1):
            alpha = float(step_index) / float(n_steps)
            samples.append([
                float((1.0 - alpha) * lhs + alpha * rhs)
                for lhs, rhs in zip(start, goal)
            ])
    deduped: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for point in samples:
        key = point_key(point)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(point)
    if max_samples > 0 and len(deduped) > max_samples:
        if max_samples == 1:
            return [deduped[0]]
        last = len(deduped) - 1
        picked: list[list[float]] = []
        picked_keys: set[tuple[float, ...]] = set()
        for index in range(max_samples):
            source_index = int(round(index * last / float(max_samples - 1)))
            point = deduped[source_index]
            key = point_key(point)
            if key in picked_keys:
                continue
            picked_keys.add(key)
            picked.append(point)
        return picked
    return deduped


def make_path_guided_seeds(robot: sbf.Robot,
                           obstacles: list[Any],
                           queries: list[Any],
                           args: argparse.Namespace) -> tuple[list[list[float]], list[dict[str, Any]]]:
    cfg = sbf.RRTConnectConfig()
    cfg.max_iters = int(args.path_guided_rrt_iters)
    cfg.timeout_ms = float(args.path_guided_rrt_timeout_ms)
    cfg.step_size = float(args.path_guided_rrt_step_size)
    cfg.goal_bias = float(args.path_guided_rrt_goal_bias)
    cfg.segment_resolution = int(args.path_guided_rrt_segment_resolution)

    selected_indices = parse_query_indices(args.path_guided_query_indices, len(queries))
    report: list[dict[str, Any]] = []
    seeds: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for index, query in enumerate(queries):
        if index not in selected_indices:
            continue
        wall_start = time.perf_counter()
        path: list[list[float]] = []
        attempts = max(1, int(args.path_guided_rrt_attempts))
        successful_attempt = -1
        for attempt in range(attempts):
            seed = int(args.path_guided_seed_base) + 997 * index + attempt
            candidate = sbf.rrt_connect_path(robot, obstacles, list(query.start), list(query.goal), cfg, seed)
            if candidate:
                path = [[float(value) for value in waypoint] for waypoint in candidate]
                successful_attempt = attempt
                break
        samples = resample_path(path, args.path_guided_sample_step, args.path_guided_max_samples_per_query)
        added = 0
        for sample in samples:
            key = point_key(sample)
            if key in seen:
                continue
            seen.add(key)
            seeds.append(sample)
            added += 1
        report.append({
            "pair_idx": index,
            "label": query.label,
            "rrt_success": bool(path),
            "rrt_attempt": successful_attempt,
            "rrt_waypoints": len(path),
            "rrt_path_length": path_length_values(path),
            "rrt_wall_ms": (time.perf_counter() - wall_start) * 1000.0,
            "sampled_seeds": len(samples),
            "unique_seed_additions": added,
            "samples": samples,
        })
    return seeds, report


def box_contains_point(box: Any, point: list[float], tolerance: float = 0.0) -> bool:
    return all(
        float(interval.lo) - tolerance <= float(value) <= float(interval.hi) + tolerance
        for interval, value in zip(box.joint_intervals, point)
    )


def point_covered(boxes: list[Any], point: list[float], tolerance: float = 0.0) -> bool:
    return any(box_contains_point(box, point, tolerance) for box in boxes)


def contiguous_spans(indices: list[int]) -> list[dict[str, int]]:
    if not indices:
        return []
    spans: list[dict[str, int]] = []
    start = indices[0]
    previous = indices[0]
    for index in indices[1:]:
        if index == previous + 1:
            previous = index
            continue
        spans.append({"start": start, "end": previous, "count": previous - start + 1})
        start = index
        previous = index
    spans.append({"start": start, "end": previous, "count": previous - start + 1})
    return spans


def path_guided_coverage(report: list[dict[str, Any]],
                         boxes: list[Any],
                         tolerance: float = 0.0,
                         uncovered_limit: int = 12) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    total_samples = 0
    covered_samples = 0
    limit = max(0, int(uncovered_limit))
    for row in report:
        samples = row.get("samples", [])
        covered_flags = [point_covered(boxes, sample, tolerance) for sample in samples]
        count = len(covered_flags)
        covered = sum(1 for flag in covered_flags if flag)
        total_samples += count
        covered_samples += covered
        first_uncovered = next((idx for idx, flag in enumerate(covered_flags) if not flag), -1)
        uncovered_indices = [idx for idx, flag in enumerate(covered_flags) if not flag]
        limited_indices = uncovered_indices[:limit]
        rows.append({
            "pair_idx": row["pair_idx"],
            "label": row["label"],
            "rrt_success": row["rrt_success"],
            "sampled_seeds": count,
            "covered_samples": covered,
            "all_samples_covered": count > 0 and covered == count,
            "first_uncovered_index": first_uncovered,
            "uncovered_count": count - covered,
            "uncovered_indices": limited_indices,
            "uncovered_samples": [samples[idx] for idx in limited_indices],
            "uncovered_spans": contiguous_spans(uncovered_indices),
        })
    return {
        "total_samples": total_samples,
        "covered_samples": covered_samples,
        "all_samples_covered": total_samples > 0 and covered_samples == total_samples,
        "queries": rows,
    }


def flatten_uncovered_points(coverage: dict[str, Any]) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    for row in coverage.get("queries", []):
        for sample_index, sample in zip(row.get("uncovered_indices", []), row.get("uncovered_samples", [])):
            points.append({
                "pair_idx": int(row["pair_idx"]),
                "label": row["label"],
                "sample_index": int(sample_index),
                "q": sample,
            })
    return points


def quantile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    alpha = position - lower
    return (1.0 - alpha) * ordered[lower] + alpha * ordered[upper]


def summarize_values(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    total = sum(float(value) for value in values)
    return {
        "count": len(values),
        "min": min(values),
        "p05": quantile(values, 0.05),
        "p10": quantile(values, 0.10),
        "p25": quantile(values, 0.25),
        "median": quantile(values, 0.50),
        "p75": quantile(values, 0.75),
        "p90": quantile(values, 0.90),
        "p95": quantile(values, 0.95),
        "max": max(values),
        "mean": total / len(values),
    }


def box_size_summary(boxes: list[Any]) -> dict[str, Any]:
    if not boxes:
        return {"count": 0, "width_by_dim": []}
    n_dims = len(boxes[0].joint_intervals)
    widths_by_dim: list[list[float]] = [[] for _ in range(n_dims)]
    volumes: list[float] = []
    log10_volumes: list[float] = []
    max_widths: list[float] = []
    mean_widths: list[float] = []
    for box in boxes:
        widths = [float(interval.hi - interval.lo) for interval in box.joint_intervals]
        for dim, width in enumerate(widths):
            widths_by_dim[dim].append(width)
        volume = 1.0
        for width in widths:
            volume *= max(width, 0.0)
        volumes.append(volume)
        log10_volumes.append(math.log10(max(volume, 1e-300)))
        max_widths.append(max(widths) if widths else 0.0)
        mean_widths.append(sum(widths) / len(widths) if widths else 0.0)
    return {
        "count": len(boxes),
        "width_by_dim": [summarize_values(values) for values in widths_by_dim],
        "volume": summarize_values(volumes),
        "log10_volume": summarize_values(log10_volumes),
        "max_width": summarize_values(max_widths),
        "mean_width": summarize_values(mean_widths),
    }


def serializable_path_guided_report(report: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in report:
        rows.append({key: value for key, value in row.items() if key != "samples"})
    return rows


def box_gap_squared(lhs: Any, rhs: Any) -> float:
    gap_sq = 0.0
    for lhs_interval, rhs_interval in zip(lhs.joint_intervals, rhs.joint_intervals):
        gap = 0.0
        if lhs_interval.hi < rhs_interval.lo:
            gap = float(rhs_interval.lo - lhs_interval.hi)
        elif rhs_interval.hi < lhs_interval.lo:
            gap = float(lhs_interval.lo - rhs_interval.hi)
        gap_sq += gap * gap
    return gap_sq


def root_pair_gaps(boxes: list[Any], limit: int = 0) -> list[dict[str, Any]]:
    by_root: dict[int, list[Any]] = {}
    for box in boxes:
        by_root.setdefault(int(box.root_id), []).append(box)
    roots = sorted(by_root)
    rows: list[dict[str, Any]] = []
    for outer_index, lhs_root in enumerate(roots):
        for rhs_root in roots[outer_index + 1:]:
            best_gap_sq = float("inf")
            best_pair = (-1, -1)
            for lhs in by_root[lhs_root]:
                for rhs in by_root[rhs_root]:
                    gap_sq = box_gap_squared(lhs, rhs)
                    if gap_sq < best_gap_sq:
                        best_gap_sq = gap_sq
                        best_pair = (int(lhs.id), int(rhs.id))
            rows.append({
                "lhs_root": lhs_root,
                "rhs_root": rhs_root,
                "gap": best_gap_sq ** 0.5 if best_gap_sq < float("inf") else float("inf"),
                "gap_sq": best_gap_sq,
                "box_pair": list(best_pair),
            })
    rows.sort(key=lambda row: (row["gap"], row["lhs_root"], row["rhs_root"]))
    if limit > 0:
        return rows[:limit]
    return rows


def query_to_dict(index: int, query, result: sbf.QueryResult, wall_ms: float) -> dict[str, Any]:
    return {
        "pair_idx": index,
        "label": query.label,
        "start_name": query.start_name,
        "goal_name": query.goal_name,
        "success": bool(result.success),
        "start_box_id": int(result.start_box_id),
        "goal_box_id": int(result.goal_box_id),
        "box_sequence": [int(value) for value in result.box_sequence],
        "box_sequence_len": len(result.box_sequence),
        "waypoints": [[float(value) for value in waypoint] for waypoint in result.path],
        "path_length": float(result.path_length),
        "query_time_ms": float(result.query_time_ms),
        "query_wall_ms": float(wall_ms),
    }


def main() -> int:
    args = parse_args()
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    base_seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]
    seeds = [] if args.path_guided_only_seeds else list(base_seeds)
    path_guided_report: list[dict[str, Any]] = []
    path_guided_seed_count = 0
    if args.path_guided_seeds:
        guided_seeds, path_guided_report = make_path_guided_seeds(robot, obstacles, queries, args)
        path_guided_seed_count = len(guided_seeds)
        seeds.extend(guided_seeds)
    config = configure(args)
    if args.path_guided_root_only:
        config.grower.max_boxes = max(1, len(seeds))
        config.grower.max_consecutive_miss = 0
        config.grower.connect_mode = False
        config.grower.component_connect_prob = 0.0
    forest = sbf.SafeBoxForest(robot, config)

    build_wall_start = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    build_wall_ms = (time.perf_counter() - build_wall_start) * 1000.0

    query_entries: list[dict[str, Any]] = []
    for index, query in enumerate(queries):
        query_wall_start = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        query_wall_ms = (time.perf_counter() - query_wall_start) * 1000.0
        entry = query_to_dict(index, query, result, query_wall_ms)
        if not result.success and args.bridge_failed_queries:
            bridge_wall_start = time.perf_counter()
            added = forest.bridge_query(list(query.start), list(query.goal))
            bridge_wall_ms = (time.perf_counter() - bridge_wall_start) * 1000.0
            retry_wall_start = time.perf_counter()
            retry = forest.query(list(query.start), list(query.goal))
            retry_wall_ms = (time.perf_counter() - retry_wall_start) * 1000.0
            entry["bridge_added_boxes"] = int(added)
            entry["bridge_wall_ms"] = float(bridge_wall_ms)
            entry["after_bridge_success"] = bool(retry.success)
            entry["after_bridge_box_sequence_len"] = len(retry.box_sequence)
            entry["after_bridge_path_length"] = float(retry.path_length)
            entry["after_bridge_query_wall_ms"] = float(retry_wall_ms)
        query_entries.append(entry)

    adjacency = {int(node): [int(value) for value in neighbors] for node, neighbors in dict(forest.adjacency()).items()}
    boxes = forest.boxes()
    root_boxes = [box for box in boxes if int(box.parent_box_id) < 0]
    root_hist: dict[int, int] = {}
    for box in boxes:
        root_hist[int(box.root_id)] = root_hist.get(int(box.root_id), 0) + 1

    coverage_payload = path_guided_coverage(
        path_guided_report,
        boxes,
        args.path_guided_coverage_tol,
        args.path_guided_uncovered_limit,
    ) if path_guided_report else {
        "total_samples": 0,
        "covered_samples": 0,
        "all_samples_covered": False,
        "queries": [],
    }
    root_only_coverage_payload = path_guided_coverage(
        path_guided_report,
        root_boxes,
        args.path_guided_coverage_tol,
        args.path_guided_uncovered_limit,
    ) if path_guided_report else {
        "total_samples": 0,
        "covered_samples": 0,
        "all_samples_covered": False,
        "queries": [],
    }

    path_guided_payload = {
        "enabled": bool(args.path_guided_seeds),
        "rrt_config": {
            "max_iters": args.path_guided_rrt_iters,
            "timeout_ms": args.path_guided_rrt_timeout_ms,
            "step_size": args.path_guided_rrt_step_size,
            "goal_bias": args.path_guided_rrt_goal_bias,
            "segment_resolution": args.path_guided_rrt_segment_resolution,
            "attempts": args.path_guided_rrt_attempts,
        },
        "sample_step": args.path_guided_sample_step,
        "max_samples_per_query": args.path_guided_max_samples_per_query,
        "coverage_tolerance": args.path_guided_coverage_tol,
        "uncovered_report_limit": args.path_guided_uncovered_limit,
        "n_seed_additions": path_guided_seed_count,
        "queries": serializable_path_guided_report(path_guided_report),
        "coverage": coverage_payload,
        "root_only_coverage": root_only_coverage_payload,
        "depth_cap_points": flatten_uncovered_points(root_only_coverage_payload),
        "n_root_boxes": len(root_boxes),
    }

    successful = [entry for entry in query_entries if entry["success"]]
    successful_after_bridge = [
        entry for entry in query_entries
        if entry["success"] or bool(entry.get("after_bridge_success", False))
    ]
    payload = {
        "attempt": args.attempt,
        "config": {
            "endpoint_source": args.endpoint_source,
            "envelope": args.envelope,
            "max_boxes": args.max_boxes,
            "timeout_ms": args.timeout_ms,
            "ffb_depth": args.ffb_depth,
            "threads": args.threads,
            "task_batch_size": args.task_batch_size,
            "component_connect_prob": args.component_connect_prob,
            "component_connect_candidate_limit": args.component_connect_candidate_limit,
            "component_connect_island_aware": args.component_connect_island_aware,
            "component_connect_frontier_cache": args.component_connect_frontier_cache,
            "component_connect_staged_growth": args.component_connect_staged_growth,
            "component_connect_stage_normalized_linf": args.component_connect_stage_normalized_linf,
            "component_connect_neighbor_root_bias": config.grower.component_connect_neighbor_root_bias,
            "component_connect_neighbor_root_window": args.component_connect_neighbor_root_window,
            "component_connect_max_parent_failures": args.component_connect_max_parent_failures,
            "component_connect_ffb_depth_increment": args.component_connect_ffb_depth_increment,
            "component_connect_ffb_max_depth": args.component_connect_ffb_max_depth,
            "component_connect_depth_after_unknown_only": args.component_connect_depth_after_unknown_only,
            "extra_random_roots": args.extra_random_roots,
            "root_seed_candidate_count": args.root_seed_candidate_count,
            "root_seed_min_normalized_linf": args.root_seed_min_normalized_linf,
            "root_seed_max_lca_depth": config.grower.root_seed_max_lca_depth,
            "path_guided_seeds": args.path_guided_seeds,
            "path_guided_only_seeds": args.path_guided_only_seeds,
            "path_guided_root_only": args.path_guided_root_only,
            "path_guided_query_indices": args.path_guided_query_indices,
            "path_guided_relax_root_filter": args.path_guided_relax_root_filter,
            "bridge_failed_queries": args.bridge_failed_queries,
            "bridge_ffb_depth": args.bridge_ffb_depth,
            "bridge_max_chain": args.bridge_max_chain,
            "bridge_steps_per_waypoint": args.bridge_steps_per_waypoint,
            "rrt_goal_bias": args.rrt_goal_bias,
            "intertree_goal_bias": args.intertree_goal_bias,
            "unexplored_prob": args.unexplored_prob,
            "step_ratio": args.step_ratio,
            "stop_after_connect": args.stop_after_connect,
            "enable_merger": args.enable_merger,
            "enable_connector": False,
            "trace_path": str(args.trace_path) if args.trace_path is not None else None,
            "trace_max_events": args.trace_max_events,
            "trace_face_candidate_limit": args.trace_face_candidate_limit,
            "root_pair_gap_limit": args.root_pair_gap_limit,
        },
        "scene": {
            "name": "combined",
            "n_obstacles": len(obstacles),
            "n_query_pairs": len(queries),
            "seed_names": ["AS", "TS", "CS", "LB", "RB"],
            "n_seeds": len(seeds),
            "n_base_seeds": 0 if args.path_guided_only_seeds else len(base_seeds),
            "n_path_guided_seeds": path_guided_seed_count,
        },
        "path_guided": path_guided_payload,
        "build_wall_ms": build_wall_ms,
        "build_profile": profile_to_dict(profile),
        "graph": {
            "component_sizes": component_sizes(adjacency),
            "root_hist": {str(root): count for root, count in sorted(root_hist.items())},
            "root_pair_gaps": root_pair_gaps(boxes, int(args.root_pair_gap_limit)),
            "box_size_summary": box_size_summary(boxes),
            "root_box_size_summary": box_size_summary(root_boxes),
        },
        "queries": query_entries,
        "summary": {
            "n_success": len(successful),
            "n_fail": len(query_entries) - len(successful),
            "all_success": len(successful) == len(query_entries),
            "n_success_after_bridge": len(successful_after_bridge),
            "n_fail_after_bridge": len(query_entries) - len(successful_after_bridge),
            "all_success_after_bridge": len(successful_after_bridge) == len(query_entries),
            "path_lengths": {entry["label"]: entry["path_length"] for entry in query_entries},
        },
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    if len(successful) != len(query_entries) and not args.allow_failures:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())