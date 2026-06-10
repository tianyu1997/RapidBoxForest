#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import math
import random
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Sequence

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import csv_list, environment_metadata
from experiments.common.generate_mapped_workspace_catalog import (
    CSpaceBox,
    clip_interval,
    dedupe_obstacles,
    difficulty_params,
    direct_obstruction_fraction,
    flatten_cspace_box,
    interpolate,
    l2,
    make_local_hyper_gate,
    make_path_collision_cells,
    map_cspace_boxes_to_workspace,
    obstacle_volume,
    resolved_allowed_link_idxs,
    resolved_subbox_cap,
    resolved_wall_dim,
    sample_local_free_pair,
    single_obstacle_direct_hits,
    subdivide_cspace_box,
)
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import (
    BITSTAR_PROBE_CHECKPOINT_INTERVAL_S,
    BITSTAR_PROBE_REWIRE_FACTOR,
    BITSTAR_PROBE_SAMPLES_PER_BATCH,
    BITSTAR_PROBE_TIMEOUT_S,
    CATALOG_SCHEMA,
    LECT_SAMPLE_DOMAIN,
    bitstar_median_window_s,
    canonical_root_intervals,
    interval_pairs,
    make_robot,
    obstacle_bounds,
    obstacle_from_bounds,
    q_in_intervals,
    q_in_lect_root,
    query_record,
    robot_joint_limit_intervals,
    sample_free_pair_with_canonical_record,
    sector_expanded_lect_root_intervals,
    timed_probe_window_s,
)
from experiments.common.rbf_defaults import robot_joint_limit_tuples
from experiments.common.sbf_import import import_sbf

sbf = import_sbf()
DIFFICULTY_ORDER = ("easy", "medium", "hard")
DEFAULT_RRT_MEDIAN_WINDOWS = "easy:0.0-0.003,medium:0.003-0.007,hard:0.007-0.050"
DEFAULT_BITSTAR_MEDIAN_WINDOWS = "easy:0.0-0.050,medium:0.050-0.100,hard:0.100-0.500"
TIME_FLOOR_S = 0.005


def finite_median(values: Sequence[float]) -> float:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return statistics.median(finite) if finite else math.nan


def in_window(value: float, window: tuple[float, float]) -> bool:
    return math.isfinite(float(value)) and float(window[0]) - 1e-12 <= float(value) <= float(window[1]) + 1e-12


def query_max_l2_limit(value: float) -> float:
    raw = float(value)
    if raw <= 0.0 or not math.isfinite(raw):
        return math.inf
    return raw


def parse_window_map(raw: str) -> dict[str, tuple[float, float]]:
    out: dict[str, tuple[float, float]] = {}
    for item in str(raw).split(","):
        text = item.strip()
        if not text:
            continue
        name, value = text.split(":", 1)
        lo_text, hi_text = value.split("-", 1)
        lo = float(lo_text)
        hi = math.inf if hi_text.strip().lower() in {"inf", "infty", "infinite"} else float(hi_text)
        if hi <= lo:
            raise ValueError(f"invalid median window {text!r}")
        out[name.strip().lower()] = (lo, hi)
    return out


def window_for(windows: dict[str, tuple[float, float]], difficulty: str, fallback: tuple[float, float]) -> tuple[float, float]:
    return windows.get(str(difficulty).lower(), fallback)


def strict_path_collision_free(robot: Any, obstacles: list[Any], path: Sequence[Sequence[float]], step: float) -> bool:
    if len(path) < 2:
        return False
    for lhs, rhs in zip(path, path[1:]):
        distance = l2(lhs, rhs)
        samples = max(1, int(math.ceil(distance / max(1e-9, float(step)))))
        for index in range(samples + 1):
            if sbf.check_config_collision(robot, obstacles, interpolate(lhs, rhs, index / samples)):
                return False
    return True


def bitstar_first_solution_probe_strict(
    *,
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    timeout_s: float,
    checkpoint_interval_s: float,
    segment_step: float,
    samples_per_batch: int,
    rewire_factor: float,
) -> dict[str, Any]:
    result = sbf.ompl_bitstar_trace(
        robot,
        obstacles,
        start,
        goal,
        float(timeout_s) * 1000.0,
        float(checkpoint_interval_s) * 1000.0,
        float(segment_step),
        int(seed),
        int(samples_per_batch),
        float(rewire_factor),
        False,
    )
    checkpoints = [dict(item) for item in result.get("checkpoints", [])]
    first: dict[str, Any] | None = None
    for checkpoint in checkpoints:
        if not bool(checkpoint.get("ok")):
            continue
        path = [[float(value) for value in point] for point in checkpoint.get("path", [])]
        if strict_path_collision_free(robot, obstacles, path, float(segment_step)):
            first = checkpoint
            break
    first_checkpoint_s = math.nan if first is None else float(first.get("checkpoint_s", math.nan))
    first_elapsed_s = math.nan if first is None else float(first.get("elapsed_s", first_checkpoint_s))
    return {
        "planner": "OMPL_BITstar_trace",
        "ok": first is not None and math.isfinite(first_checkpoint_s),
        "status": str(result.get("status", "")) if first is not None else "no_valid_solution_before_timeout",
        "timeout_s": float(timeout_s),
        "checkpoint_interval_s": float(checkpoint_interval_s),
        "segment_step": float(segment_step),
        "seed": int(seed),
        "samples_per_batch": int(samples_per_batch),
        "rewire_factor": float(rewire_factor),
        "first_success_checkpoint_s": first_checkpoint_s,
        "first_success_elapsed_s": first_elapsed_s,
        "checkpoint_count": len(checkpoints),
        "success_checkpoint_count": sum(1 for row in checkpoints if bool(row.get("ok"))),
    }


def rrtconnect_first_solution_probe_strict(
    *,
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    timeout_s: float,
    segment_step: float,
) -> dict[str, Any]:
    result = sbf.ompl_rrt_connect_path(
        robot,
        obstacles,
        start,
        goal,
        float(timeout_s) * 1000.0,
        0.35,
        float(segment_step),
        0.0,
        int(seed),
    )
    path = [[float(value) for value in point] for point in result.get("path", [])]
    solve_s = float(result.get("solve_s", result.get("t_s", math.nan)) or math.nan)
    raw_ok = math.isfinite(solve_s) and len(path) >= 2
    strict_ok = raw_ok and math.isfinite(solve_s) and strict_path_collision_free(robot, obstacles, path, float(segment_step))
    return {
        "planner": "OMPL_RRTConnect",
        "ok": bool(strict_ok),
        "status": str(result.get("status", "")) if raw_ok else "no_solution_before_timeout",
        "reason": str(result.get("reason", "")),
        "raw_ok": bool(result.get("ok")),
        "raw_path_present": bool(len(path) >= 2),
        "raw_path_length": int(len(path)),
        "exact_solution": bool(result.get("exact_solution", False)),
        "first_success_s": solve_s if strict_ok else math.nan,
        "raw_solve_s": solve_s,
        "path": path if strict_ok else [],
        "timeout_s": float(timeout_s),
        "range": 0.35,
        "segment_step": float(segment_step),
        "simplify_time_s": 0.0,
        "nodes": int(result.get("nodes", 0) or 0),
    }


def bitstar_first_solution_path_probe_strict(
    *,
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    seed: int,
    timeout_s: float,
    segment_step: float,
    samples_per_batch: int,
    rewire_factor: float,
) -> dict[str, Any]:
    result = sbf.ompl_bitstar_path(
        robot,
        obstacles,
        start,
        goal,
        float(timeout_s) * 1000.0,
        float(segment_step),
        0.0,
        int(seed),
        int(samples_per_batch),
        float(rewire_factor),
        True,
    )
    path = [[float(value) for value in point] for point in result.get("path", [])]
    solve_s = float(result.get("solve_s", result.get("t_s", math.nan)) or math.nan)
    ok = (
        bool(result.get("ok"))
        and math.isfinite(solve_s)
        and strict_path_collision_free(robot, obstacles, path, float(segment_step))
    )
    return {
        "planner": "OMPL_BITstar_path_first_solution",
        "ok": bool(ok),
        "status": str(result.get("status", "")) if ok else "no_valid_solution_before_timeout",
        "timeout_s": float(timeout_s),
        "checkpoint_interval_s": 0.0,
        "segment_step": float(segment_step),
        "seed": int(seed),
        "samples_per_batch": int(samples_per_batch),
        "rewire_factor": float(rewire_factor),
        "first_success_checkpoint_s": solve_s if ok else math.nan,
        "first_success_elapsed_s": solve_s if ok else math.nan,
        "checkpoint_count": 1,
        "success_checkpoint_count": 1 if ok else 0,
    }


def strip_probe_paths(summary: dict[str, Any]) -> dict[str, Any]:
    out = dict(summary)
    probes = []
    for raw in out.get("probes", []):
        probe = dict(raw)
        probe.pop("path", None)
        probes.append(probe)
    if probes:
        out["probes"] = probes
    return out


def first_solution_summary(
    *,
    robot: Any,
    obstacles: list[Any],
    queries: list[dict[str, Any]],
    difficulty: str,
    seed: int,
    planner_seeds: int,
    rrt_window: tuple[float, float],
    bitstar_window: tuple[float, float],
    rrt_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
) -> dict[str, Any]:
    rrt_lo, rrt_hi = rrt_window
    bit_lo, bit_hi = bitstar_window
    rrt_probes: list[dict[str, Any]] = []
    bit_probes: list[dict[str, Any]] = []
    rrt_call_timeout_s = max(float(rrt_timeout_s), float(rrt_hi) if math.isfinite(float(rrt_hi)) else 0.0)
    for query_index, query in enumerate(queries):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        for seed_index in range(max(1, int(planner_seeds))):
            probe_seed = int(seed) + 1009 * query_index + 65537 * seed_index
            rrt = rrtconnect_first_solution_probe_strict(
                robot=robot,
                obstacles=obstacles,
                start=start,
                goal=goal,
                seed=probe_seed,
                timeout_s=float(rrt_call_timeout_s),
                segment_step=float(audit_step),
            )
            if bool(rrt.get("ok")) and not strict_path_collision_free(
                robot,
                obstacles,
                [[float(value) for value in point] for point in rrt.get("path", [])],
                float(audit_step),
            ):
                rrt["ok"] = False
                rrt["status"] = "strict_audit_failed"
                rrt["first_success_s"] = math.nan
            rrt["query_index"] = int(query_index)
            rrt["seed_index"] = int(seed_index)
            rrt_probes.append(rrt)
            if str(bitstar_probe_mode).lower() == "trace":
                bit = bitstar_first_solution_probe_strict(
                    robot=robot,
                    obstacles=obstacles,
                    start=start,
                    goal=goal,
                    seed=probe_seed + 524287,
                    timeout_s=float(bitstar_timeout_s),
                    checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
                    segment_step=float(audit_step),
                    samples_per_batch=int(bitstar_samples_per_batch),
                    rewire_factor=float(bitstar_rewire_factor),
                )
            else:
                bit = bitstar_first_solution_path_probe_strict(
                    robot=robot,
                    obstacles=obstacles,
                    start=start,
                    goal=goal,
                    seed=probe_seed + 524287,
                    timeout_s=float(bitstar_timeout_s),
                    segment_step=float(audit_step),
                    samples_per_batch=int(bitstar_samples_per_batch),
                    rewire_factor=float(bitstar_rewire_factor),
                )
            bit["query_index"] = int(query_index)
            bit["seed_index"] = int(seed_index)
            bit_probes.append(bit)
    rrt_success_times = [
        float(row.get("first_success_s", math.nan))
        for row in rrt_probes
        if bool(row.get("ok")) and math.isfinite(float(row.get("first_success_s", math.nan)))
    ]
    bit_success_times = [
        float(row.get("first_success_elapsed_s", math.nan))
        for row in bit_probes
        if bool(row.get("ok")) and math.isfinite(float(row.get("first_success_elapsed_s", math.nan)))
    ]
    bit_success_checkpoint_times = [
        float(row.get("first_success_checkpoint_s", math.nan))
        for row in bit_probes
        if bool(row.get("ok")) and math.isfinite(float(row.get("first_success_checkpoint_s", math.nan)))
    ]
    rrt_timeout_value = float(rrt_call_timeout_s)
    bit_timeout_value = float(bitstar_timeout_s)
    rrt_times = [
        float(row.get("first_success_s", rrt_timeout_value))
        if bool(row.get("ok")) and math.isfinite(float(row.get("first_success_s", math.nan)))
        else rrt_timeout_value
        for row in rrt_probes
    ]
    bit_times = [
        float(row.get("first_success_elapsed_s", bit_timeout_value))
        if bool(row.get("ok")) and math.isfinite(float(row.get("first_success_elapsed_s", math.nan)))
        else bit_timeout_value
        for row in bit_probes
    ]
    bit_checkpoint_times = [
        float(row.get("first_success_checkpoint_s", bit_timeout_value))
        if bool(row.get("ok")) and math.isfinite(float(row.get("first_success_checkpoint_s", math.nan)))
        else bit_timeout_value
        for row in bit_probes
    ]
    rrt_median = finite_median(rrt_times)
    bit_median = finite_median(bit_times)
    bit_checkpoint_median = finite_median(bit_checkpoint_times)
    min_success = max(0.0, min(1.0, float(min_success_fraction)))
    rrt_success_fraction = float(len(rrt_success_times)) / float(len(rrt_probes)) if rrt_probes else 0.0
    bit_success_fraction = float(len(bit_success_times)) / float(len(bit_probes)) if bit_probes else 0.0
    rrt_ok = rrt_success_fraction >= min_success - 1e-12 and in_window(rrt_median, (rrt_lo, rrt_hi))
    bit_ok = bit_success_fraction >= min_success - 1e-12 and in_window(bit_median, (bit_lo, bit_hi))
    return {
        "planner": "OMPL_RRTConnect+BITstar",
        "policy": "shared_query_median_first_solution_time_windows",
        "aggregation": "all shared queries times all planner seeds",
        "ok": bool(rrt_ok and bit_ok),
        "difficulty": str(difficulty),
        "query_count": int(len(queries)),
        "planner_seed_count": int(max(1, planner_seeds)),
        "rrtconnect": {
            "planner": "OMPL_RRTConnect",
            "ok": bool(rrt_ok),
            "success_count": int(len(rrt_success_times)),
            "query_count": int(len(rrt_probes)),
            "success_fraction": float(rrt_success_fraction),
            "median_first_success_s": float(rrt_median),
            "mean_first_success_s": float(sum(rrt_times) / len(rrt_times)) if rrt_times else math.nan,
            "min_first_success_s": float(min(rrt_times)) if rrt_times else math.nan,
            "max_first_success_s": float(max(rrt_times)) if rrt_times else math.nan,
            "censored_timeout_s": float(rrt_timeout_value),
            "window_s": [float(rrt_lo), float(rrt_hi)],
            "timeout_s": float(rrt_call_timeout_s),
            "probes": [strip_probe_paths(row) for row in rrt_probes],
        },
        "bitstar": {
            "planner": "OMPL_BITstar_trace",
            "ok": bool(bit_ok),
            "success_count": int(len(bit_success_times)),
            "query_count": int(len(bit_probes)),
            "success_fraction": float(bit_success_fraction),
            "median_first_success_s": float(bit_median),
            "median_first_success_checkpoint_s": float(bit_checkpoint_median),
            "success_checkpoint_count": int(len(bit_success_checkpoint_times)),
            "mean_first_success_s": float(sum(bit_times) / len(bit_times)) if bit_times else math.nan,
            "min_first_success_s": float(min(bit_times)) if bit_times else math.nan,
            "max_first_success_s": float(max(bit_times)) if bit_times else math.nan,
            "mean_first_success_checkpoint_s": float(sum(bit_checkpoint_times) / len(bit_checkpoint_times)) if bit_checkpoint_times else math.nan,
            "censored_timeout_s": float(bit_timeout_value),
            "window_s": [float(bit_lo), float(bit_hi)],
            "timeout_s": float(bitstar_timeout_s),
            "checkpoint_interval_s": float(bitstar_checkpoint_interval_s),
            "probe_mode": str(bitstar_probe_mode),
            "samples_per_batch": int(bitstar_samples_per_batch),
            "rewire_factor": float(bitstar_rewire_factor),
            "probes": bit_probes,
        },
    }


def summary_with_windows(
    summary: dict[str, Any],
    *,
    difficulty: str,
    rrt_window: tuple[float, float],
    bitstar_window: tuple[float, float],
    min_success_fraction: float,
) -> dict[str, Any]:
    out = copy.deepcopy(summary)
    min_success = max(0.0, min(1.0, float(min_success_fraction)))
    rrt = out.get("rrtconnect", {})
    bit = out.get("bitstar", {})
    rrt_median = float(rrt.get("median_first_success_s", math.nan))
    bit_median = float(bit.get("median_first_success_s", math.nan))
    rrt_success = float(rrt.get("success_fraction", 0.0))
    bit_success = float(bit.get("success_fraction", 0.0))
    rrt_ok = rrt_success >= min_success - 1e-12 and in_window(rrt_median, rrt_window)
    bit_ok = bit_success >= min_success - 1e-12 and in_window(bit_median, bitstar_window)
    out["difficulty"] = str(difficulty)
    out["ok"] = bool(rrt_ok and bit_ok)
    out["rrtconnect"]["ok"] = bool(rrt_ok)
    out["rrtconnect"]["window_s"] = [float(rrt_window[0]), float(rrt_window[1])]
    out["bitstar"]["ok"] = bool(bit_ok)
    out["bitstar"]["window_s"] = [float(bitstar_window[0]), float(bitstar_window[1])]
    return out


def prefix_counts(count: int, *, fine_until: int, mid_step: int, coarse_step: int) -> list[int]:
    values = {0, 1, 2, 3, 4, 5}
    values.update(range(6, min(count, int(fine_until)) + 1, max(1, int(mid_step))))
    values.update(range(min(count, int(fine_until)) + max(1, int(mid_step)), min(count, 250) + 1, max(1, int(coarse_step))))
    values.update(range(275, count + 1, max(1, 2 * int(coarse_step))))
    values.add(int(count))
    return sorted(value for value in values if 0 <= value <= int(count))


def quantile(values: Sequence[float], q: float) -> float:
    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return float(finite[0])
    pos = max(0.0, min(1.0, float(q))) * float(len(finite) - 1)
    lo_index = int(math.floor(pos))
    hi_index = int(math.ceil(pos))
    if lo_index == hi_index:
        return float(finite[lo_index])
    frac = pos - float(lo_index)
    return float((1.0 - frac) * finite[lo_index] + frac * finite[hi_index])


def _planner_times_from_summary(summary: dict[str, Any], planner_key: str) -> list[float]:
    planner = dict(summary.get(planner_key, {}))
    timeout = float(planner.get("censored_timeout_s", planner.get("timeout_s", math.nan)) or math.nan)
    probes = planner.get("probes", [])
    field = "first_success_s" if planner_key == "rrtconnect" else "first_success_elapsed_s"
    times: list[float] = []
    for row in probes:
        raw = float(row.get(field, math.nan))
        if bool(row.get("ok")) and math.isfinite(raw):
            times.append(raw)
        elif math.isfinite(timeout):
            times.append(timeout)
    if not times:
        median = float(planner.get("median_first_success_s", math.nan))
        if math.isfinite(median):
            times.append(median)
        elif math.isfinite(timeout):
            times.append(timeout)
    return [float(value) for value in times]


def _planner_distribution_metrics(summary: dict[str, Any], planner_key: str) -> dict[str, Any]:
    planner = dict(summary.get(planner_key, {}))
    times = _planner_times_from_summary(summary, planner_key)
    median = quantile(times, 0.5)
    return {
        "times_s": times,
        "success_fraction": float(planner.get("success_fraction", 0.0)),
        "median_s": float(median),
        "q25_s": float(quantile(times, 0.25)),
        "q75_s": float(quantile(times, 0.75)),
        "min_s": float(min(times)) if times else math.nan,
        "max_s": float(max(times)) if times else math.nan,
        "log_median": float(math.log(max(float(median), TIME_FLOOR_S))) if math.isfinite(median) else math.nan,
    }


def distribution_metrics_from_summary(summary: dict[str, Any], min_success_fraction: float) -> dict[str, Any]:
    rrt = _planner_distribution_metrics(summary, "rrtconnect")
    bitstar = _planner_distribution_metrics(summary, "bitstar")
    composite_samples: list[float] = []
    for rrt_time, bitstar_time in zip(rrt.get("times_s", []), bitstar.get("times_s", [])):
        composite_samples.append(
            float(
                math.exp(
                    0.5
                    * (
                        math.log(max(float(rrt_time), TIME_FLOOR_S))
                        + math.log(max(float(bitstar_time), TIME_FLOOR_S))
                    )
                )
            )
        )
    if not composite_samples:
        rrt_median = float(rrt.get("median_s", math.nan))
        bitstar_median = float(bitstar.get("median_s", math.nan))
        if math.isfinite(rrt_median) and math.isfinite(bitstar_median):
            composite_samples = [
                float(
                    math.exp(
                        0.5
                        * (
                            math.log(max(rrt_median, TIME_FLOOR_S))
                            + math.log(max(bitstar_median, TIME_FLOOR_S))
                        )
                    )
                )
            ]
    min_success = max(0.0, min(1.0, float(min_success_fraction)))
    composite_median = quantile(composite_samples, 0.5)
    selectable = (
        float(rrt.get("success_fraction", 0.0)) >= min_success - 1e-12
        and float(bitstar.get("success_fraction", 0.0)) >= min_success - 1e-12
        and math.isfinite(composite_median)
    )
    return {
        "policy": "distribution_separation_v1",
        "selectable": bool(selectable),
        "rrtconnect": rrt,
        "bitstar": bitstar,
        "composite": {
            "times_s": composite_samples,
            "median_s": float(composite_median),
            "q25_s": float(quantile(composite_samples, 0.25)),
            "q75_s": float(quantile(composite_samples, 0.75)),
            "log_median": float(math.log(max(float(composite_median), TIME_FLOOR_S))) if math.isfinite(composite_median) else math.nan,
        },
    }


def _normalize_scan_scores(scan_rows: list[dict[str, Any]]) -> None:
    for planner_key in ("rrtconnect", "bitstar"):
        values = [
            float(row["metrics"][planner_key].get("log_median", math.nan))
            for row in scan_rows
            if bool(row.get("metrics", {}).get("selectable")) and math.isfinite(float(row["metrics"][planner_key].get("log_median", math.nan)))
        ]
        lo = min(values) if values else math.nan
        hi = max(values) if values else math.nan
        span = hi - lo if math.isfinite(lo) and math.isfinite(hi) else math.nan
        for row in scan_rows:
            value = float(row.get("metrics", {}).get(planner_key, {}).get("log_median", math.nan))
            if math.isfinite(value) and math.isfinite(span) and span > 1e-12:
                row["metrics"][planner_key]["normalized_log_median"] = float((value - lo) / span)
            elif math.isfinite(value):
                row["metrics"][planner_key]["normalized_log_median"] = 0.0
            else:
                row["metrics"][planner_key]["normalized_log_median"] = math.nan
    for row in scan_rows:
        rrt_norm = float(row.get("metrics", {}).get("rrtconnect", {}).get("normalized_log_median", math.nan))
        bit_norm = float(row.get("metrics", {}).get("bitstar", {}).get("normalized_log_median", math.nan))
        row["metrics"]["composite"]["normalized_log_median"] = (
            float(0.5 * (rrt_norm + bit_norm))
            if math.isfinite(rrt_norm) and math.isfinite(bit_norm)
            else math.nan
        )


def _planner_strong_separation(low: dict[str, Any], high: dict[str, Any], planner_key: str) -> bool:
    low_q75 = float(low.get("metrics", {}).get(planner_key, {}).get("q75_s", math.nan))
    high_q25 = float(high.get("metrics", {}).get(planner_key, {}).get("q25_s", math.nan))
    return math.isfinite(low_q75) and math.isfinite(high_q25) and low_q75 < high_q25


def _distribution_triple_score(
    easy: dict[str, Any],
    medium: dict[str, Any],
    hard: dict[str, Any],
    *,
    medium_ratio: float,
    hard_ratio: float,
    hard_not_faster_factor: float,
    require_strong_planner: bool,
) -> tuple[bool, float, dict[str, Any]]:
    e = easy["metrics"]
    m = medium["metrics"]
    h = hard["metrics"]
    if not (bool(e.get("selectable")) and bool(m.get("selectable")) and bool(h.get("selectable"))):
        return False, -math.inf, {"reason": "not_selectable"}
    e_med = float(e["composite"]["median_s"])
    m_med = float(m["composite"]["median_s"])
    h_med = float(h["composite"]["median_s"])
    if not (math.isfinite(e_med) and math.isfinite(m_med) and math.isfinite(h_med)):
        return False, -math.inf, {"reason": "nonfinite_composite"}
    if not (e_med < m_med < h_med):
        return False, -math.inf, {"reason": "composite_median_not_ordered"}
    m_ratio = m_med / max(e_med, TIME_FLOOR_S)
    h_ratio = h_med / max(m_med, TIME_FLOOR_S)
    if m_ratio < float(medium_ratio) - 1e-12:
        return False, -math.inf, {"reason": "medium_ratio_too_low", "medium_over_easy": float(m_ratio)}
    if h_ratio < float(hard_ratio) - 1e-12:
        return False, -math.inf, {"reason": "hard_ratio_too_low", "hard_over_medium": float(h_ratio)}
    if not (
        float(e["composite"]["q75_s"]) < float(m["composite"]["median_s"])
        and float(m["composite"]["q75_s"]) < float(h["composite"]["median_s"])
    ):
        return False, -math.inf, {"reason": "composite_quantile_overlap"}
    for planner_key in ("rrtconnect", "bitstar"):
        e_planner = float(e[planner_key]["median_s"])
        m_planner = float(m[planner_key]["median_s"])
        h_planner = float(h[planner_key]["median_s"])
        if math.isfinite(e_planner) and math.isfinite(m_planner):
            if m_planner < float(hard_not_faster_factor) * e_planner:
                return False, -math.inf, {"reason": f"{planner_key}_medium_faster_than_easy"}
        if math.isfinite(m_planner) and math.isfinite(h_planner):
            if h_planner < float(hard_not_faster_factor) * m_planner:
                return False, -math.inf, {"reason": f"{planner_key}_hard_faster_than_medium"}
    strong_planners = [
        planner_key
        for planner_key in ("rrtconnect", "bitstar")
        if _planner_strong_separation(easy, medium, planner_key)
        and _planner_strong_separation(medium, hard, planner_key)
    ]
    if require_strong_planner and not strong_planners:
        return False, -math.inf, {"reason": "no_strong_reference_planner"}
    norm_gap = (
        float(m["composite"].get("normalized_log_median", 0.0))
        - float(e["composite"].get("normalized_log_median", 0.0))
        + float(h["composite"].get("normalized_log_median", 0.0))
        - float(m["composite"].get("normalized_log_median", 0.0))
    )
    score = (
        math.log(max(m_ratio, 1e-12))
        + math.log(max(h_ratio, 1e-12))
        + 0.5 * float(norm_gap)
        + 0.25 * float(len(strong_planners))
    )
    details = {
        "medium_over_easy": float(m_ratio),
        "hard_over_medium": float(h_ratio),
        "strong_reference_planners": strong_planners,
        "score": float(score),
    }
    return True, float(score), details


def select_distribution_prefixes_from_scan(
    scan_rows: list[dict[str, Any]],
    *,
    medium_ratio: float = 5.0,
    hard_ratio: float = 2.0,
    hard_not_faster_factor: float = 1.0,
    require_strong_planner: bool = True,
) -> dict[str, Any]:
    rows = [row for row in scan_rows if bool(row.get("metrics", {}).get("selectable"))]
    rows.sort(key=lambda row: int(row["count"]))
    _normalize_scan_scores(rows)
    best: tuple[float, dict[str, dict[str, Any]], dict[str, Any]] | None = None
    best_reject: dict[str, Any] | None = None
    for easy_index, easy in enumerate(rows):
        for medium_index in range(easy_index + 1, len(rows)):
            medium = rows[medium_index]
            for hard in rows[medium_index + 1:]:
                if not (int(easy["count"]) < int(medium["count"]) < int(hard["count"])):
                    if best_reject is None:
                        best_reject = {
                            "easy_count": int(easy["count"]),
                            "medium_count": int(medium["count"]),
                            "hard_count": int(hard["count"]),
                            "reason": "prefix_counts_not_strictly_nested",
                        }
                    continue
                ok, score, details = _distribution_triple_score(
                    easy,
                    medium,
                    hard,
                    medium_ratio=float(medium_ratio),
                    hard_ratio=float(hard_ratio),
                    hard_not_faster_factor=float(hard_not_faster_factor),
                    require_strong_planner=bool(require_strong_planner),
                )
                if not ok:
                    if best_reject is None:
                        best_reject = {
                            "easy_count": int(easy["count"]),
                            "medium_count": int(medium["count"]),
                            "hard_count": int(hard["count"]),
                            **details,
                        }
                    continue
                selected = {"easy": easy, "medium": medium, "hard": hard}
                if best is None or score > best[0]:
                    best = (float(score), selected, details)
    if best is None:
        raise RuntimeError(
            "no distribution-separated prefix triple; "
            f"selectable={len(rows)}/{len(scan_rows)}; first_reject={best_reject}"
        )
    score, selected, details = best
    return {
        "policy": "distribution_separation_v1",
        "score": float(score),
        "selection_metrics": details,
        "prefix_counts": {name: int(row["count"]) for name, row in selected.items()},
        "selected_rows": selected,
        "scan_rows": scan_rows,
        "criteria": {
            "medium_over_easy_min": float(medium_ratio),
            "hard_over_medium_min": float(hard_ratio),
            "hard_not_faster_factor": float(hard_not_faster_factor),
            "strict_prefix_nesting": True,
            "require_strong_reference_planner": bool(require_strong_planner),
        },
    }


def compact_prefix_scan_row(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "count": int(row["count"]),
        "direct_obstruction_fraction_mean": float(row.get("direct_mean", math.nan)),
        "metrics": copy.deepcopy(row.get("metrics", {})),
    }


def query_direct_mean(robot: Any, obstacles: list[Any], queries: list[dict[str, Any]], samples: int) -> float:
    values = [
        direct_obstruction_fraction(
            sbf,
            robot,
            obstacles,
            query["start"],
            query["goal"],
            int(samples),
        )
        for query in queries
    ]
    return float(sum(values) / len(values)) if values else math.nan


def obstacle_score(robot: Any, bounds: Sequence[float], queries: list[dict[str, Any]], samples: int) -> tuple[int, float]:
    obstacle = obstacle_from_bounds(bounds)
    hits = 0
    for query in queries:
        hits += single_obstacle_direct_hits(
            sbf,
            robot,
            obstacle,
            query["start"],
            query["goal"],
            int(samples),
        )
    return int(hits), float(obstacle_volume(bounds))


def ordered_obstacle_bounds(
    *,
    robot: Any,
    bounds: list[list[float]],
    queries: list[dict[str, Any]],
    samples: int,
    strategy: str,
    max_obstacles: int,
    seed: int,
) -> list[list[float]]:
    key = str(strategy).strip().lower()
    if key in {"source_order", "source", "as_generated"}:
        ordered = [[float(value) for value in item] for item in bounds]
        if int(max_obstacles) > 0:
            ordered = ordered[: int(max_obstacles)]
        return ordered
    scored = [
        (obstacle_score(robot, item, queries, int(samples)), item)
        for item in bounds
    ]
    if key == "random":
        rng = random.Random(int(seed) + 37_156_667)
        shuffled = [item for _score, item in scored]
        rng.shuffle(shuffled)
        ordered = shuffled
    elif key == "direct_asc":
        ordered = [item for _score, item in sorted(scored, key=lambda pair: (pair[0][0], pair[0][1]))]
    elif key == "volume_asc":
        ordered = [item for _score, item in sorted(scored, key=lambda pair: (pair[0][1], pair[0][0]))]
    elif key == "volume_desc":
        ordered = [item for _score, item in sorted(scored, key=lambda pair: (pair[0][1], pair[0][0]), reverse=True)]
    elif key == "mixed":
        direct = [item for _score, item in sorted(scored, key=lambda pair: (pair[0][0], pair[0][1]), reverse=True)]
        volume = [item for _score, item in sorted(scored, key=lambda pair: (pair[0][1], pair[0][0]))]
        seen: set[tuple[float, ...]] = set()
        ordered = []
        while direct or volume:
            for source in (volume, direct):
                if not source:
                    continue
                item = source.pop(0)
                key_tuple = tuple(round(float(value), 9) for value in item)
                if key_tuple in seen:
                    continue
                seen.add(key_tuple)
                ordered.append(item)
    else:
        ordered = [item for _score, item in sorted(scored, key=lambda pair: (pair[0][0], pair[0][1]), reverse=True)]
    if int(max_obstacles) > 0:
        ordered = ordered[: int(max_obstacles)]
    return [[float(value) for value in item] for item in ordered]


def probe_guided_obstacle_bounds(
    *,
    robot_name: str,
    robot: Any,
    bounds: list[list[float]],
    queries: list[dict[str, Any]],
    samples: int,
    max_obstacles: int,
    seed: int,
    planner_seeds: int,
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    candidate_pool: int,
    max_prefix: int,
) -> tuple[list[list[float]], list[dict[str, Any]]]:
    direct_order = ordered_obstacle_bounds(
        robot=robot,
        bounds=bounds,
        queries=queries,
        samples=int(samples),
        strategy="mixed",
        max_obstacles=int(max_obstacles),
        seed=int(seed),
    )
    remaining = [[float(value) for value in item] for item in direct_order]
    selected: list[list[float]] = []
    trace: list[dict[str, Any]] = []
    skipped_failures: list[list[float]] = []
    rng = random.Random(int(seed) + 9_191_717)
    max_steps = min(
        len(remaining),
        int(max_obstacles) if int(max_obstacles) > 0 else len(remaining),
        int(max_prefix) if int(max_prefix) > 0 else len(remaining),
    )
    for step in progress(range(max_steps), desc=f"{robot_name} probe-greedy-order"):
        pool_size = max(1, int(candidate_pool))
        pool = remaining[:pool_size]
        if len(remaining) > pool_size:
            extra_count = min(pool_size, len(remaining) - pool_size)
            pool.extend(rng.sample(remaining[pool_size:], extra_count))
        best_item: list[float] | None = None
        best_measurement: dict[str, Any] | None = None
        best_score = -math.inf
        failed_this_round: list[list[float]] = []
        for item in pool:
            obstacles = [obstacle_from_bounds(bounds_item) for bounds_item in selected + [item]]
            if any(
                sbf.check_config_collision(robot, obstacles, query["start"])
                or sbf.check_config_collision(robot, obstacles, query["goal"])
                for query in queries
            ):
                failed_this_round.append(item)
                continue
            measurement = first_solution_summary(
                robot=robot,
                obstacles=obstacles,
                queries=queries,
                difficulty="probe_greedy_order",
                seed=int(seed) + 65_537 * (step + 1) + 17_071 * len(trace),
                planner_seeds=int(planner_seeds),
                rrt_window=(0.0, math.inf),
                bitstar_window=(0.0, math.inf),
                rrt_timeout_s=float(rrt_probe_timeout_s),
                bitstar_timeout_s=float(bitstar_timeout_s),
                bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
                audit_step=float(audit_step),
                min_success_fraction=float(min_success_fraction),
                bitstar_probe_mode=str(bitstar_probe_mode),
                bitstar_samples_per_batch=int(bitstar_samples_per_batch),
                bitstar_rewire_factor=float(bitstar_rewire_factor),
            )
            rrt = measurement["rrtconnect"]
            bit = measurement["bitstar"]
            rrt_success = float(rrt.get("success_fraction", 0.0))
            bit_success = float(bit.get("success_fraction", 0.0))
            if rrt_success < float(min_success_fraction) - 1e-12 or bit_success < float(min_success_fraction) - 1e-12:
                failed_this_round.append(item)
                continue
            rrt_s = float(rrt.get("median_first_success_s", math.nan))
            bit_s = float(bit.get("median_first_success_s", math.nan))
            if not math.isfinite(rrt_s) or not math.isfinite(bit_s):
                failed_this_round.append(item)
                continue
            direct_hits, volume = obstacle_score(robot, item, queries, int(samples))
            # BIT* is the harder gate to shape for UR5; RRTConnect and direct
            # obstruction break ties without accepting infeasible prefixes.
            score = 100.0 * bit_s + 10.0 * rrt_s + 1e-4 * float(direct_hits) + 1e-6 * float(volume)
            if score > best_score:
                best_score = float(score)
                best_item = item
                best_measurement = measurement
        if best_item is None or best_measurement is None:
            for item in failed_this_round:
                if item in remaining:
                    remaining.remove(item)
                    skipped_failures.append(item)
            if not remaining:
                break
            continue
        selected.append(best_item)
        if best_item in remaining:
            remaining.remove(best_item)
        rrt = best_measurement["rrtconnect"]
        bit = best_measurement["bitstar"]
        trace.append(
            {
                "prefix_count": int(len(selected)),
                "rrt_median_first_success_s": float(rrt.get("median_first_success_s", math.nan)),
                "bitstar_median_first_success_s": float(bit.get("median_first_success_s", math.nan)),
                "rrt_success_fraction": float(rrt.get("success_fraction", 0.0)),
                "bitstar_success_fraction": float(bit.get("success_fraction", 0.0)),
                "candidate_pool": int(len(pool)),
                "skipped_failures_so_far": int(len(skipped_failures)),
            }
        )
    ordered = selected + remaining + skipped_failures
    if int(max_obstacles) > 0:
        ordered = ordered[: int(max_obstacles)]
    return [[float(value) for value in item] for item in ordered], trace


def cspace_box_around_point(
    q: Sequence[float],
    limits: Sequence[tuple[float, float]],
    half_width: float,
) -> CSpaceBox | None:
    intervals: list[tuple[float, float]] = []
    for dim_index, value in enumerate(q):
        lo = max(float(limits[dim_index][0]), float(value) - float(half_width))
        hi = min(float(limits[dim_index][1]), float(value) + float(half_width))
        if hi <= lo + 1e-9:
            return None
        intervals.append((lo, hi))
    return CSpaceBox(intervals)


def path_blocking_obstacle_bounds(
    *,
    robot_name: str,
    robot: Any,
    limits: Sequence[tuple[float, float]],
    queries: list[dict[str, Any]],
    seed: int,
    planner_seeds: int,
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    direct_obstruction_samples: int,
    max_obstacles: int,
    max_prefix: int,
    candidate_pool: int,
    samples_per_path: int,
    box_half_width: float,
    use_inflated: bool,
    workspace_margin: float,
    endpoint_source: str,
    n_samples_crit: int,
    endpoint_threads: int,
    n_subdivisions: int,
    workspace_aabb_shrink: float,
    min_active_link_idx: int,
    max_active_link_idx: int,
    allowed_link_idxs: Sequence[int],
    query_count: int,
    initial_obstacles: list[list[float]] | None = None,
) -> tuple[list[list[float]], list[dict[str, Any]]]:
    selected: list[list[float]] = [
        [float(value) for value in item]
        for item in (initial_obstacles or [])
    ]
    trace: list[dict[str, Any]] = []
    seen: set[tuple[float, ...]] = {
        tuple(round(float(value), 5) for value in item)
        for item in selected
    }
    rng = random.Random(int(seed) + 24_676_219)
    max_steps = min(
        int(max_obstacles) if int(max_obstacles) > 0 else int(max_prefix),
        int(max_prefix) if int(max_prefix) > 0 else int(max_obstacles),
    )
    if max_steps <= 0:
        max_steps = int(max_obstacles) if int(max_obstacles) > 0 else 40
    active_queries = queries[: max(1, min(len(queries), int(query_count)))]

    initial_count = len(selected)
    for step in progress(range(max_steps), desc=f"{robot_name} path-block-order"):
        current_obstacles = [obstacle_from_bounds(item) for item in selected]
        measurement = first_solution_summary(
            robot=robot,
            obstacles=current_obstacles,
            queries=active_queries,
            difficulty="path_blocking_current",
            seed=int(seed) + 3571 * (step + 1),
            planner_seeds=int(planner_seeds),
            rrt_window=(0.0, math.inf),
            bitstar_window=(0.0, math.inf),
            rrt_timeout_s=float(rrt_probe_timeout_s),
            bitstar_timeout_s=float(bitstar_timeout_s),
            bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
            audit_step=float(audit_step),
            min_success_fraction=float(min_success_fraction),
            bitstar_probe_mode=str(bitstar_probe_mode),
            bitstar_samples_per_batch=int(bitstar_samples_per_batch),
            bitstar_rewire_factor=float(bitstar_rewire_factor),
        )
        rrt = measurement["rrtconnect"]
        bit = measurement["bitstar"]
        if (
            float(rrt.get("success_fraction", 0.0)) < float(min_success_fraction) - 1e-12
            or float(bit.get("success_fraction", 0.0)) < float(min_success_fraction) - 1e-12
        ):
            break

        cspace_candidates: list[CSpaceBox] = []
        for query_index, query in enumerate(active_queries):
            probe = rrtconnect_first_solution_probe_strict(
                robot=robot,
                obstacles=current_obstacles,
                start=[float(value) for value in query["start"]],
                goal=[float(value) for value in query["goal"]],
                seed=int(seed) + 65_537 * (step + 1) + 1009 * query_index,
                timeout_s=float(rrt_probe_timeout_s),
                segment_step=float(audit_step),
            )
            if not bool(probe.get("ok")):
                continue
            path = [[float(value) for value in point] for point in probe.get("path", [])]
            if len(path) < 3:
                continue
            sample_count = max(1, int(samples_per_path))
            for sample_index in range(1, sample_count + 1):
                alpha = float(sample_index) / float(sample_count + 1)
                path_pos = alpha * (len(path) - 1)
                lo_index = min(len(path) - 2, max(0, int(math.floor(path_pos))))
                frac = path_pos - lo_index
                q = interpolate(path[lo_index], path[lo_index + 1], frac)
                box = cspace_box_around_point(q, limits, float(box_half_width))
                if box is not None:
                    cspace_candidates.append(box)
        if not cspace_candidates:
            break

        mapped = map_cspace_boxes_to_workspace(
            robot_name=robot_name,
            boxes=cspace_candidates,
            use_inflated=bool(use_inflated),
            workspace_margin=float(workspace_margin),
            endpoint_source=str(endpoint_source),
            n_samples_crit=int(n_samples_crit),
            endpoint_threads=int(endpoint_threads),
            n_subdivisions=int(n_subdivisions),
            workspace_aabb_shrink=float(workspace_aabb_shrink),
            min_active_link_idx=int(min_active_link_idx),
            max_active_link_idx=int(max_active_link_idx),
            allowed_link_idxs=allowed_link_idxs,
        )
        bounds = dedupe_obstacles(mapped.get("obstacles", []), precision=5)
        filtered: list[list[float]] = []
        for item in bounds:
            key = tuple(round(float(value), 5) for value in item)
            if key in seen:
                continue
            obstacle = obstacle_from_bounds(item)
            if any(
                sbf.check_config_collision(robot, [obstacle], query["start"])
                or sbf.check_config_collision(robot, [obstacle], query["goal"])
                for query in queries
            ):
                continue
            filtered.append([float(value) for value in item])
        if not filtered:
            break
        scored = [
            (obstacle_score(robot, item, queries, int(direct_obstruction_samples)), item)
            for item in filtered
        ]
        scored.sort(key=lambda pair: (pair[0][0], pair[0][1]), reverse=True)
        pool = [item for _score, item in scored[: max(1, int(candidate_pool))]]
        if len(scored) > len(pool):
            tail = [item for _score, item in scored[len(pool):]]
            pool.extend(rng.sample(tail, min(max(1, int(candidate_pool)), len(tail))))

        best_item: list[float] | None = None
        best_measurement: dict[str, Any] | None = None
        best_score = -math.inf
        rejected = 0
        for item in pool:
            obstacles = [obstacle_from_bounds(bounds_item) for bounds_item in selected + [item]]
            measurement_next = first_solution_summary(
                robot=robot,
                obstacles=obstacles,
                queries=active_queries,
                difficulty="path_blocking_candidate",
                seed=int(seed) + 97_409 * (step + 1) + 7919 * rejected,
                planner_seeds=int(planner_seeds),
                rrt_window=(0.0, math.inf),
                bitstar_window=(0.0, math.inf),
                rrt_timeout_s=float(rrt_probe_timeout_s),
                bitstar_timeout_s=float(bitstar_timeout_s),
                bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
                audit_step=float(audit_step),
                min_success_fraction=float(min_success_fraction),
                bitstar_probe_mode=str(bitstar_probe_mode),
                bitstar_samples_per_batch=int(bitstar_samples_per_batch),
                bitstar_rewire_factor=float(bitstar_rewire_factor),
            )
            rrt_next = measurement_next["rrtconnect"]
            bit_next = measurement_next["bitstar"]
            if (
                float(rrt_next.get("success_fraction", 0.0)) < float(min_success_fraction) - 1e-12
                or float(bit_next.get("success_fraction", 0.0)) < float(min_success_fraction) - 1e-12
            ):
                rejected += 1
                continue
            rrt_s = float(rrt_next.get("median_first_success_s", math.nan))
            bit_s = float(bit_next.get("median_first_success_s", math.nan))
            if not math.isfinite(rrt_s) or not math.isfinite(bit_s):
                rejected += 1
                continue
            direct_hits, volume = obstacle_score(robot, item, queries, int(direct_obstruction_samples))
            score = 100.0 * bit_s + 10.0 * rrt_s + 1e-4 * float(direct_hits) + 1e-6 * float(volume)
            if score > best_score:
                best_score = float(score)
                best_item = item
                best_measurement = measurement_next
        if best_item is None or best_measurement is None:
            break
        selected.append(best_item)
        seen.add(tuple(round(float(value), 5) for value in best_item))
        rrt_next = best_measurement["rrtconnect"]
        bit_next = best_measurement["bitstar"]
        trace.append(
            {
                "prefix_count": int(len(selected)),
                "added_count": int(len(selected) - initial_count),
                "rrt_median_first_success_s": float(rrt_next.get("median_first_success_s", math.nan)),
                "bitstar_median_first_success_s": float(bit_next.get("median_first_success_s", math.nan)),
                "rrt_success_fraction": float(rrt_next.get("success_fraction", 0.0)),
                "bitstar_success_fraction": float(bit_next.get("success_fraction", 0.0)),
                "candidate_pool": int(len(pool)),
                "candidate_rejected": int(rejected),
            }
        )
    return [[float(value) for value in item] for item in selected], trace


def make_multi_wall_hyper_gate(
    limits: Sequence[tuple[float, float]],
    *,
    robot_name: str,
    rng: random.Random,
    scene_try: int,
    wall_dim: int,
    wall_count: int,
) -> tuple[list[float], list[float], list[CSpaceBox], dict[str, Any]]:
    dim = len(limits)
    wall_dim = min(max(0, int(wall_dim)), dim - 1)
    params = difficulty_params(robot_name, "hard", rng, scene_try)
    gate_count = min(int(params["gate_count"]), max(1, dim - 1))
    gate_dims = [index for index in range(dim) if index != wall_dim][:gate_count]
    nominal: list[float] = []
    for lo, hi in limits:
        span = float(hi) - float(lo)
        nominal.append(rng.uniform(float(lo) + 0.25 * span, float(hi) - 0.25 * span))
    start = list(nominal)
    goal = list(nominal)
    wall_span = float(limits[wall_dim][1]) - float(limits[wall_dim][0])
    wall_delta = min(2.40, max(0.90, 0.42 * wall_span))
    start[wall_dim] = max(float(limits[wall_dim][0]) + 0.20, nominal[wall_dim] - wall_delta)
    goal[wall_dim] = min(float(limits[wall_dim][1]) - 0.20, nominal[wall_dim] + wall_delta)
    span_along = float(goal[wall_dim]) - float(start[wall_dim])
    count = max(1, int(wall_count))
    gate_half = 0.5 * float(params["gate_width"])
    local_radius = float(params["local_radius"])
    obstacles: list[CSpaceBox] = []
    gate_centers_by_wall: list[dict[int, float]] = []
    for wall_index in range(count):
        alpha = float(wall_index + 1) / float(count + 1)
        wall_center = float(start[wall_dim]) + alpha * span_along
        centers: dict[int, float] = {}
        for gate_dim_index, gate_dim in enumerate(gate_dims):
            lo, hi = limits[gate_dim]
            span = float(hi) - float(lo)
            direction = 1.0 if (wall_index + gate_dim_index) % 2 == 0 else -1.0
            center = nominal[gate_dim] + direction * float(params["gate_offset_fraction"]) * span
            center = min(float(hi) - gate_half - 1e-6, max(float(lo) + gate_half + 1e-6, center))
            centers[gate_dim] = center
        gate_centers_by_wall.append(centers)
        for gate_dim in gate_dims:
            center = centers[gate_dim]
            for side_lo, side_hi in (
                (nominal[gate_dim] - local_radius, center - gate_half),
                (center + gate_half, nominal[gate_dim] + local_radius),
            ):
                clipped_side = clip_interval(side_lo, side_hi, limits[gate_dim])
                if clipped_side is None:
                    continue
                intervals: list[tuple[float, float]] = []
                for dim_index, limit in enumerate(limits):
                    if dim_index == wall_dim:
                        interval = clip_interval(
                            wall_center - float(params["wall_half_width"]),
                            wall_center + float(params["wall_half_width"]),
                            limit,
                        )
                    elif dim_index == gate_dim:
                        interval = clipped_side
                    else:
                        radius = local_radius if dim_index in gate_dims else 0.65 * local_radius
                        interval = clip_interval(nominal[dim_index] - radius, nominal[dim_index] + radius, limit)
                    if interval is None:
                        break
                    intervals.append(interval)
                if len(intervals) == dim:
                    obstacles.append(CSpaceBox(intervals))
    metadata = {
        "generator": "multi_wall_hyper_gate",
        "wall_dim": int(wall_dim),
        "wall_count": int(count),
        "gate_dims": gate_dims,
        "gate_centers": {str(k): float(v) for k, v in gate_centers_by_wall[0].items()} if gate_centers_by_wall else {},
        "gate_centers_by_wall": [
            {str(k): float(v) for k, v in centers.items()}
            for centers in gate_centers_by_wall
        ],
        "nominal": [float(value) for value in nominal],
        **{key: float(value) if isinstance(value, float) else int(value) for key, value in params.items()},
    }
    return start, goal, obstacles, metadata


def scaled_gate_gap_boxes(boxes: Sequence[CSpaceBox], metadata: dict[str, Any], scale: float) -> list[CSpaceBox]:
    factor = max(1e-3, min(1.0, float(scale)))
    if factor >= 1.0 - 1e-12:
        return [CSpaceBox(list(box.intervals)) for box in boxes]
    centers = {int(key): float(value) for key, value in dict(metadata.get("gate_centers", {})).items()}
    old_half = 0.5 * float(metadata.get("gate_width", 0.0) or 0.0)
    new_half = max(1e-5, old_half * factor)
    if old_half <= new_half + 1e-12 or not centers:
        return [CSpaceBox(list(box.intervals)) for box in boxes]
    out: list[CSpaceBox] = []
    for box in boxes:
        intervals = list(box.intervals)
        for dim_index, center in centers.items():
            if dim_index < 0 or dim_index >= len(intervals):
                continue
            lo, hi = intervals[dim_index]
            # These side slabs are the C-space obstacles adjacent to the gate.
            # Moving the inner face toward the gate center narrows the free gap
            # before workspace-envelope mapping.
            if float(hi) <= center - old_half + 1e-9:
                intervals[dim_index] = (float(lo), max(float(lo) + 1e-9, center - new_half))
            elif float(lo) >= center + old_half - 1e-9:
                intervals[dim_index] = (min(float(hi) - 1e-9, center + new_half), float(hi))
        out.append(CSpaceBox(intervals))
    return out


def first_strict_prefix_triple(
    candidates: dict[str, list[tuple[int, dict[str, Any], float]]],
) -> dict[str, tuple[int, dict[str, Any], float]] | None:
    for easy in candidates["easy"]:
        for medium in candidates["medium"]:
            if medium[0] <= easy[0]:
                continue
            for hard in candidates["hard"]:
                if hard[0] <= medium[0]:
                    continue
                return {"easy": easy, "medium": medium, "hard": hard}
    return None


def sample_shared_queries(
    *,
    robot: Any,
    obstacles: list[Any],
    start: list[float],
    goal: list[float],
    count: int,
    seed: int,
    min_l2: float,
    max_l2: float,
    local_radius: float,
    max_tries: int,
) -> list[dict[str, Any]]:
    queries = [query_record(label="q0", robot=robot, start=start, goal=goal)]
    rng = random.Random(int(seed) + 1299709)
    attempts = 0
    while len(queries) < int(count) and attempts < int(max_tries):
        attempts += 1
        try:
            cand_start, cand_goal, shift, sector = sample_local_free_pair(
                sbf=sbf,
                robot=robot,
                obstacles=obstacles,
                rng=rng,
                base_start=start,
                base_goal=goal,
                radius=float(local_radius),
                min_l2=float(min_l2),
                max_l2=float(max_l2),
                max_tries=64,
            )
        except RuntimeError:
            continue
        if any(
            l2(cand_start, query["start"]) + l2(cand_goal, query["goal"]) < 0.10
            for query in queries
        ):
            continue
        queries.append(
            query_record(
                label=f"q{len(queries)}",
                robot=robot,
                start=cand_start,
                goal=cand_goal,
                symmetry_shift=shift,
                symmetry_sector=sector,
            )
        )
    if len(queries) < int(count):
        raise RuntimeError(f"could only sample {len(queries)}/{count} shared queries")
    return queries


def sample_hard_probe_queries(
    *,
    robot: Any,
    obstacles: list[Any],
    count: int,
    seed: int,
    min_l2: float,
    max_l2: float,
    clearance_margin_m: float,
    max_tries: int,
    rrt_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    bitstar_min_s: float,
    bitstar_max_s: float,
    rrt_min_s: float,
    rrt_max_s: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    direct_obstruction_samples: int,
) -> list[dict[str, Any]]:
    rng = random.Random(int(seed) + 6_700_417)
    queries: list[dict[str, Any]] = []
    attempts = 0
    reject_counts = {
        "sample_failed": 0,
        "duplicate": 0,
        "direct_unblocked": 0,
        "rrt_failed": 0,
        "rrt_out_of_window": 0,
        "bitstar_failed": 0,
        "bitstar_out_of_window": 0,
    }
    rrt_observed: list[float] = []
    bitstar_observed: list[float] = []
    for attempts in progress(
        range(1, max(0, int(max_tries)) + 1),
        desc="hard-query",
    ):
        if len(queries) >= int(count):
            break
        try:
            start, goal, _cstart, _cgoal, shift, sector = sample_free_pair_with_canonical_record(
                robot,
                list(obstacles),
                rng,
                min_l2=float(min_l2),
                max_l2=float(max_l2),
                clearance_margin_m=float(clearance_margin_m),
                max_tries=96,
            )
        except RuntimeError:
            reject_counts["sample_failed"] += 1
            continue
        if any(
            l2(start, query["start"]) + l2(goal, query["goal"]) < 0.20
            for query in queries
        ):
            reject_counts["duplicate"] += 1
            continue
        direct = direct_obstruction_fraction(
            sbf,
            robot,
            obstacles,
            start,
            goal,
            int(direct_obstruction_samples),
        )
        if direct <= 0.0:
            reject_counts["direct_unblocked"] += 1
            continue
        probe_seed = int(seed) + 131_071 * attempts
        rrt_timeout = max(float(rrt_timeout_s), float(rrt_max_s) if math.isfinite(float(rrt_max_s)) else 0.0)
        rrt = rrtconnect_first_solution_probe_strict(
            robot=robot,
            obstacles=obstacles,
            start=start,
            goal=goal,
            seed=probe_seed,
            timeout_s=float(rrt_timeout),
            segment_step=float(audit_step),
        )
        rrt_s = float(rrt.get("first_success_s", math.nan))
        if bool(rrt.get("ok")) and math.isfinite(rrt_s):
            rrt_observed.append(float(rrt_s))
        if not bool(rrt.get("ok")):
            reject_counts["rrt_failed"] += 1
            continue
        if not in_window(rrt_s, (float(rrt_min_s), float(rrt_max_s))):
            reject_counts["rrt_out_of_window"] += 1
            continue

        if str(bitstar_probe_mode).lower() == "trace":
            bit = bitstar_first_solution_probe_strict(
                robot=robot,
                obstacles=obstacles,
                start=start,
                goal=goal,
                seed=probe_seed + 524287,
                timeout_s=float(bitstar_timeout_s),
                checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
                segment_step=float(audit_step),
                samples_per_batch=int(bitstar_samples_per_batch),
                rewire_factor=float(bitstar_rewire_factor),
            )
        else:
            bit = bitstar_first_solution_path_probe_strict(
                robot=robot,
                obstacles=obstacles,
                start=start,
                goal=goal,
                seed=probe_seed + 524287,
                timeout_s=float(bitstar_timeout_s),
                segment_step=float(audit_step),
                samples_per_batch=int(bitstar_samples_per_batch),
                rewire_factor=float(bitstar_rewire_factor),
            )
        bit_s = float(bit.get("first_success_elapsed_s", math.nan))
        bit_checkpoint_s = float(bit.get("first_success_checkpoint_s", math.nan))
        if bool(bit.get("ok")) and math.isfinite(bit_s):
            bitstar_observed.append(float(bit_s))
        if not bool(bit.get("ok")):
            reject_counts["bitstar_failed"] += 1
            continue
        if not in_window(bit_s, (float(bitstar_min_s), float(bitstar_max_s))):
            reject_counts["bitstar_out_of_window"] += 1
            continue
        rrt_summary = dict(rrt)
        rrt_summary.pop("path", None)
        probe = {
            "planner": "OMPL_RRTConnect+BITstar",
            "policy": "single_query_hard_probe_prefilter",
            "ok": True,
            "difficulty": "hard_query_prefilter",
            "query_count": 1,
            "planner_seed_count": 1,
            "rrtconnect": {
                "planner": "OMPL_RRTConnect",
                "ok": True,
                "success_count": 1,
                "query_count": 1,
                "success_fraction": 1.0,
                "median_first_success_s": float(rrt_s),
                "window_s": [float(rrt_min_s), float(rrt_max_s)],
                "timeout_s": float(rrt_timeout),
                "probes": [rrt_summary],
            },
            "bitstar": {
                "planner": str(bit.get("planner", "OMPL_BITstar")),
                "ok": True,
                "success_count": 1,
                "query_count": 1,
                "success_fraction": 1.0,
                "median_first_success_s": float(bit_s),
                "median_first_success_checkpoint_s": float(bit_checkpoint_s),
                "window_s": [float(bitstar_min_s), float(bitstar_max_s)],
                "timeout_s": float(bitstar_timeout_s),
                "checkpoint_interval_s": float(bitstar_checkpoint_interval_s),
                "probe_mode": str(bitstar_probe_mode),
                "samples_per_batch": int(bitstar_samples_per_batch),
                "rewire_factor": float(bitstar_rewire_factor),
                "probes": [dict(bit)],
            },
        }
        record = query_record(
            label=f"q{len(queries)}",
            robot=robot,
            start=start,
            goal=goal,
            symmetry_shift=shift,
            symmetry_sector=sector,
            difficulty_probe=probe,
        )
        record["direct_obstruction_fraction"] = float(direct)
        queries.append(record)
        print(
            f"[hard-query] accepted q{len(queries) - 1}: "
            f"rrt={rrt_s:.6f}s bitstar={bit_s:.6f}s checkpoint={bit_checkpoint_s:.6f}s",
            flush=True,
        )
    if len(queries) < int(count):
        rrt_text = (
            f"rrt_observed_median={finite_median(rrt_observed):.6g}, "
            f"rrt_observed_min={min(rrt_observed):.6g}, rrt_observed_max={max(rrt_observed):.6g}"
            if rrt_observed
            else "rrt_observed=none"
        )
        bit_text = (
            f"bitstar_observed_median={finite_median(bitstar_observed):.6g}, "
            f"bitstar_observed_min={min(bitstar_observed):.6g}, bitstar_observed_max={max(bitstar_observed):.6g}"
            if bitstar_observed
            else "bitstar_observed=none"
        )
        raise RuntimeError(
            f"could only sample {len(queries)}/{count} hard-probed shared queries "
            f"from {attempts} attempts; rejects={reject_counts}; {rrt_text}; {bit_text}"
        )
    return queries


def augment_prefix_matching_queries(
    *,
    robot: Any,
    bounds_ordered: list[list[float]],
    prefixes: dict[str, tuple[int, dict[str, Any], float]],
    queries: list[dict[str, Any]],
    target_count: int,
    seed: int,
    min_l2: float,
    max_l2: float,
    local_radius: float,
    clearance_margin_m: float,
    max_tries: int,
    planner_seeds: int,
    rrt_windows: dict[str, tuple[float, float]],
    bitstar_windows: dict[str, tuple[float, float]],
    post_rrt_windows: dict[str, tuple[float, float]] | None = None,
    post_bitstar_windows: dict[str, tuple[float, float]] | None = None,
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    check_mode: str = "all",
    local_sample_prob: float = 0.85,
    group_min_success_fraction: float = 0.5,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    if len(queries) >= int(target_count):
        return queries[: int(target_count)], {}
    rng = random.Random(int(seed) + 31_415_927)
    out = [dict(query) for query in queries]
    rejects: dict[str, int] = {}

    def reject(reason: str) -> None:
        rejects[reason] = int(rejects.get(reason, 0)) + 1

    hard_count = int(prefixes["hard"][0])
    hard_obstacles = [obstacle_from_bounds(item) for item in bounds_ordered[:hard_count]]
    active_rrt_windows = post_rrt_windows if post_rrt_windows is not None else rrt_windows
    active_bitstar_windows = post_bitstar_windows if post_bitstar_windows is not None else bitstar_windows
    attempts = 0
    for attempts in progress(
        range(1, max(0, int(max_tries)) + 1),
        desc="post-query",
    ):
        if len(out) >= int(target_count):
            break
        try:
            if out and rng.random() < max(0.0, min(1.0, float(local_sample_prob))):
                base = out[rng.randrange(len(out))]
                start, goal, shift, sector = sample_local_free_pair(
                    sbf=sbf,
                    robot=robot,
                    obstacles=hard_obstacles,
                    rng=rng,
                    base_start=base["start"],
                    base_goal=base["goal"],
                    radius=float(local_radius),
                    min_l2=float(min_l2),
                    max_l2=float(max_l2),
                    max_tries=96,
                )
            else:
                start, goal, _cstart, _cgoal, shift, sector = sample_free_pair_with_canonical_record(
                    robot,
                    hard_obstacles,
                    rng,
                    min_l2=float(min_l2),
                    max_l2=float(max_l2),
                    clearance_margin_m=float(clearance_margin_m),
                    max_tries=96,
                )
        except RuntimeError:
            reject("sample_failed")
            continue
        if any(
            l2(start, query["start"]) + l2(goal, query["goal"]) < 0.10
            for query in out
        ):
            reject("duplicate")
            continue

        per_difficulty: dict[str, Any] = {}
        mode = str(check_mode).strip().lower()
        if mode != "none":
            ok = True
            difficulties = DIFFICULTY_ORDER if mode in {"all", "group"} else ("hard",)
            for difficulty in difficulties:
                count = int(prefixes[difficulty][0])
                obstacles = [obstacle_from_bounds(item) for item in bounds_ordered[:count]]
                check_queries = (
                    [*out, {"start": start, "goal": goal}]
                    if mode == "group"
                    else [{"start": start, "goal": goal}]
                )
                measurement = first_solution_summary(
                    robot=robot,
                    obstacles=obstacles,
                    queries=check_queries,
                    difficulty=f"post_query_{difficulty}",
                    seed=int(seed) + 65_537 * attempts + 1009 * len(out),
                    planner_seeds=int(planner_seeds),
                    rrt_window=(0.0, math.inf),
                    bitstar_window=(0.0, math.inf),
                    rrt_timeout_s=float(rrt_probe_timeout_s),
                    bitstar_timeout_s=float(bitstar_timeout_s),
                    bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
                    audit_step=float(audit_step),
                    min_success_fraction=(
                        float(group_min_success_fraction)
                        if mode == "group"
                        else 1.0
                    ),
                    bitstar_probe_mode=str(bitstar_probe_mode),
                    bitstar_samples_per_batch=int(bitstar_samples_per_batch),
                    bitstar_rewire_factor=float(bitstar_rewire_factor),
                )
                checked = summary_with_windows(
                    measurement,
                    difficulty=difficulty,
                    rrt_window=window_for(active_rrt_windows, difficulty, (0.0, math.inf)),
                    bitstar_window=window_for(active_bitstar_windows, difficulty, (0.0, math.inf)),
                    min_success_fraction=(
                        float(group_min_success_fraction)
                        if mode == "group"
                        else 1.0
                    ),
                )
                per_difficulty[difficulty] = checked
                if not bool(checked.get("ok")):
                    ok = False
                    reject(f"{difficulty}_window_failed")
                    break
            if not ok:
                continue
        record = query_record(
            label=f"q{len(out)}",
            robot=robot,
            start=start,
            goal=goal,
            symmetry_shift=shift,
            symmetry_sector=sector,
            difficulty_probe={"per_difficulty": per_difficulty},
        )
        out.append(record)
        print(f"[post-query] accepted q{len(out) - 1} check_mode={check_mode}", flush=True)
    if len(out) < int(target_count):
        raise RuntimeError(
            f"could only post-sample {len(out)}/{target_count} prefix-matching queries; rejects={rejects}"
        )
    return out, rejects


def build_record(
    *,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    generator_seed: int,
    queries: list[dict[str, Any]],
    obstacle_bounds_list: list[list[float]],
    difficulty_probe: dict[str, Any],
    source_cspace: dict[str, Any],
    mapping: dict[str, Any],
    direct_mean: float,
) -> dict[str, Any]:
    robot = make_robot(robot_name)
    first = queries[0]
    canonical_start = [float(value) for value in first["canonical_start"]]
    canonical_goal = [float(value) for value in first["canonical_goal"]]
    mapping_payload = dict(mapping)
    mapping_payload["obstacle_prefix_difficulty"] = str(difficulty)
    mapping_payload["obstacle_prefix_count"] = int(len(obstacle_bounds_list))
    return {
        "schema": CATALOG_SCHEMA,
        "robot": str(robot_name),
        "difficulty": str(difficulty),
        "scene_seed": int(scene_seed),
        "generator_seed": int(generator_seed),
        "scene_profile": "prefix_cspace_mapped_workspace_median_gated",
        "queries_per_scene": int(len(queries)),
        "queries": [dict(query) for query in queries],
        "start": [float(value) for value in first["start"]],
        "goal": [float(value) for value in first["goal"]],
        "canonical_start": canonical_start,
        "canonical_goal": canonical_goal,
        "symmetry_shift": int(first.get("symmetry_shift", 0)),
        "symmetry_sector": int(first.get("symmetry_sector", 0)),
        "obstacles": [[float(value) for value in bounds] for bounds in obstacle_bounds_list],
        "sample_domain": LECT_SAMPLE_DOMAIN,
        "canonical_cache": True,
        "lect_root_intervals": interval_pairs(canonical_root_intervals(robot)),
        "planning_root_intervals": interval_pairs(robot_joint_limit_intervals(robot)),
        "sector_expanded_root_intervals": interval_pairs(sector_expanded_lect_root_intervals(robot)),
        "canonical_start_in_lect_root": q_in_intervals(canonical_start, canonical_root_intervals(robot)),
        "canonical_goal_in_lect_root": q_in_intervals(canonical_goal, canonical_root_intervals(robot)),
        "actual_start_in_lect_root": q_in_lect_root(robot, first["start"]),
        "actual_goal_in_lect_root": q_in_lect_root(robot, first["goal"]),
        "endpoint_clearance_margin_m": 0.0,
        "fixed_robot_clearance_margin_m": 0.0,
        "max_query_l2": float("nan"),
        "direct_segment_blocked": bool(direct_mean > 0.0),
        "direct_obstruction_fraction_mean": float(direct_mean),
        "segment_resolution": int(mapping.get("direct_obstruction_samples", 96)),
        "difficulty_probe": difficulty_probe,
        "incremental_scene": {
            "shared_query_set": True,
            "obstacle_prefix_difficulty": str(difficulty),
            "obstacle_prefix_count": int(len(obstacle_bounds_list)),
        },
        "source_cspace": source_cspace,
        "workspace_mapping": mapping_payload,
    }


def find_prefixes(
    *,
    robot_name: str,
    robot: Any,
    bounds_ordered: list[list[float]],
    queries: list[dict[str, Any]],
    seed: int,
    planner_seeds: int,
    direct_obstruction_samples: int,
    rrt_windows: dict[str, tuple[float, float]],
    bitstar_windows: dict[str, tuple[float, float]],
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    prefix_fine_until: int,
    prefix_mid_step: int,
    prefix_coarse_step: int,
) -> dict[str, tuple[int, dict[str, Any], float]]:
    candidates: dict[str, list[tuple[int, dict[str, Any], float]]] = {key: [] for key in DIFFICULTY_ORDER}
    observed: list[dict[str, Any]] = []
    for count in progress(
        prefix_counts(
            len(bounds_ordered),
            fine_until=int(prefix_fine_until),
            mid_step=int(prefix_mid_step),
            coarse_step=int(prefix_coarse_step),
        ),
        desc=f"{robot_name} prefix-count",
    ):
        obstacles = [obstacle_from_bounds(bounds) for bounds in bounds_ordered[:count]]
        if any(
            sbf.check_config_collision(robot, obstacles, query["start"])
            or sbf.check_config_collision(robot, obstacles, query["goal"])
            for query in queries
        ):
            continue
        measurement = first_solution_summary(
            robot=robot,
            obstacles=obstacles,
            queries=queries,
            difficulty="unclassified",
            seed=int(seed) + 104729 * count,
            planner_seeds=int(planner_seeds),
            rrt_window=(0.0, math.inf),
            bitstar_window=(0.0, math.inf),
            rrt_timeout_s=float(rrt_probe_timeout_s),
            bitstar_timeout_s=float(bitstar_timeout_s),
            bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
            audit_step=float(audit_step),
            min_success_fraction=float(min_success_fraction),
            bitstar_probe_mode=str(bitstar_probe_mode),
            bitstar_samples_per_batch=int(bitstar_samples_per_batch),
            bitstar_rewire_factor=float(bitstar_rewire_factor),
        )
        observed.append(
            {
                "count": int(count),
                "rrt": float(measurement["rrtconnect"]["median_first_success_s"]),
                "rrt_success_fraction": float(measurement["rrtconnect"]["success_fraction"]),
                "rrt_probe_head": [
                    {
                        "ok": bool(row.get("ok")),
                        "status": str(row.get("status", "")),
                        "reason": str(row.get("reason", "")),
                        "first_success_s": float(row.get("first_success_s", math.nan)),
                        "raw_solve_s": float(row.get("raw_solve_s", math.nan)),
                        "path_len": len(row.get("path", [])),
                        "raw_path_length": int(row.get("raw_path_length", 0)),
                    }
                    for row in measurement["rrtconnect"].get("probes", [])[:2]
                ],
                "bitstar": float(measurement["bitstar"]["median_first_success_s"]),
                "bitstar_checkpoint": float(measurement["bitstar"].get("median_first_success_checkpoint_s", math.nan)),
                "bitstar_success_fraction": float(measurement["bitstar"]["success_fraction"]),
            }
        )
        for difficulty in DIFFICULTY_ORDER:
            probe = summary_with_windows(
                measurement,
                difficulty=difficulty,
                rrt_window=window_for(rrt_windows, difficulty, timed_probe_window_s(difficulty, strict_time=False)),
                bitstar_window=window_for(bitstar_windows, difficulty, bitstar_median_window_s(difficulty)),
                min_success_fraction=float(min_success_fraction),
            )
            if not bool(probe.get("ok")):
                continue
            direct_mean = query_direct_mean(robot, obstacles, queries, int(direct_obstruction_samples))
            candidates[difficulty].append((count, probe, direct_mean))
        accepted = first_strict_prefix_triple(candidates)
        if accepted is not None:
            return accepted
    details = {
        key: [
            {
                "count": item[0],
                "rrt": item[1]["rrtconnect"]["median_first_success_s"],
                "bitstar": item[1]["bitstar"]["median_first_success_s"],
                "bitstar_checkpoint": item[1]["bitstar"].get("median_first_success_checkpoint_s", math.nan),
                "direct": item[2],
            }
            for item in value[:5]
        ]
        for key, value in candidates.items()
    }
    raise RuntimeError(f"no strict easy<medium<hard prefix triple; candidates={details}; observed={observed}")


def distribution_prefix_scan_rows(
    *,
    robot_name: str,
    robot: Any,
    bounds_ordered: list[list[float]],
    queries: list[dict[str, Any]],
    counts: Sequence[int],
    desc_suffix: str,
    seed: int,
    planner_seeds: int,
    direct_obstruction_samples: int,
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
) -> list[dict[str, Any]]:
    scan_rows: list[dict[str, Any]] = []
    for count in progress(counts, desc=f"{robot_name} {desc_suffix}"):
        obstacles = [obstacle_from_bounds(bounds) for bounds in bounds_ordered[: int(count)]]
        if any(
            sbf.check_config_collision(robot, obstacles, query["start"])
            or sbf.check_config_collision(robot, obstacles, query["goal"])
            for query in queries
        ):
            continue
        measurement = first_solution_summary(
            robot=robot,
            obstacles=obstacles,
            queries=queries,
            difficulty="unclassified",
            seed=int(seed) + 104729 * int(count),
            planner_seeds=int(planner_seeds),
            rrt_window=(0.0, math.inf),
            bitstar_window=(0.0, math.inf),
            rrt_timeout_s=float(rrt_probe_timeout_s),
            bitstar_timeout_s=float(bitstar_timeout_s),
            bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
            audit_step=float(audit_step),
            min_success_fraction=float(min_success_fraction),
            bitstar_probe_mode=str(bitstar_probe_mode),
            bitstar_samples_per_batch=int(bitstar_samples_per_batch),
            bitstar_rewire_factor=float(bitstar_rewire_factor),
        )
        metrics = distribution_metrics_from_summary(measurement, float(min_success_fraction))
        direct_mean = query_direct_mean(robot, obstacles, queries, int(direct_obstruction_samples))
        scan_rows.append(
            {
                "count": int(count),
                "measurement": measurement,
                "metrics": metrics,
                "direct_mean": float(direct_mean),
            }
        )
    return scan_rows


def distribution_prefix_output_from_selection(selection: dict[str, Any]) -> dict[str, tuple[int, dict[str, Any], float]]:
    compact_scan = [compact_prefix_scan_row(row) for row in selection["scan_rows"]]
    out: dict[str, tuple[int, dict[str, Any], float]] = {}
    for difficulty in DIFFICULTY_ORDER:
        row = selection["selected_rows"][difficulty]
        probe = copy.deepcopy(row["measurement"])
        probe["policy"] = "distribution_separation_v1"
        probe["difficulty"] = str(difficulty)
        probe["ok"] = True
        probe["distribution_metrics"] = copy.deepcopy(row["metrics"])
        probe["distribution_selection"] = {
            "score": float(selection["score"]),
            "criteria": copy.deepcopy(selection["criteria"]),
            "selection_metrics": copy.deepcopy(selection["selection_metrics"]),
            "prefix_counts": copy.deepcopy(selection["prefix_counts"]),
        }
        probe["distribution_prefix_scan"] = compact_scan
        out[difficulty] = (int(row["count"]), probe, float(row.get("direct_mean", math.nan)))
    return out


def prefix_count_neighbors(all_counts: Sequence[int], selected_counts: Sequence[int], radius: int) -> list[int]:
    ordered = [int(value) for value in all_counts]
    selected = {int(value) for value in selected_counts}
    out: set[int] = set()
    for index, count in enumerate(ordered):
        if count not in selected:
            continue
        lo = max(0, index - max(0, int(radius)))
        hi = min(len(ordered), index + max(0, int(radius)) + 1)
        out.update(ordered[lo:hi])
    return sorted(out)


def find_distribution_prefixes(
    *,
    robot_name: str,
    robot: Any,
    bounds_ordered: list[list[float]],
    queries: list[dict[str, Any]],
    seed: int,
    planner_seeds: int,
    direct_obstruction_samples: int,
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    prefix_fine_until: int,
    prefix_mid_step: int,
    prefix_coarse_step: int,
    distribution_medium_ratio: float,
    distribution_hard_ratio: float,
    distribution_hard_not_faster_factor: float,
    distribution_require_strong_planner: bool,
) -> dict[str, tuple[int, dict[str, Any], float]]:
    counts = prefix_counts(
        len(bounds_ordered),
        fine_until=int(prefix_fine_until),
        mid_step=int(prefix_mid_step),
        coarse_step=int(prefix_coarse_step),
    )
    scan_rows = distribution_prefix_scan_rows(
        robot_name=robot_name,
        robot=robot,
        bounds_ordered=bounds_ordered,
        queries=queries,
        counts=counts,
        desc_suffix="distribution-prefix-count",
        seed=int(seed),
        planner_seeds=int(planner_seeds),
        direct_obstruction_samples=int(direct_obstruction_samples),
        rrt_probe_timeout_s=float(rrt_probe_timeout_s),
        bitstar_timeout_s=float(bitstar_timeout_s),
        bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
        audit_step=float(audit_step),
        min_success_fraction=float(min_success_fraction),
        bitstar_probe_mode=str(bitstar_probe_mode),
        bitstar_samples_per_batch=int(bitstar_samples_per_batch),
        bitstar_rewire_factor=float(bitstar_rewire_factor),
    )
    selection = select_distribution_prefixes_from_scan(
        scan_rows,
        medium_ratio=float(distribution_medium_ratio),
        hard_ratio=float(distribution_hard_ratio),
        hard_not_faster_factor=float(distribution_hard_not_faster_factor),
        require_strong_planner=bool(distribution_require_strong_planner),
    )
    return distribution_prefix_output_from_selection(selection)


def find_distribution_prefixes_two_stage(
    *,
    robot_name: str,
    robot: Any,
    bounds_ordered: list[list[float]],
    queries: list[dict[str, Any]],
    seed: int,
    stage_a_planner_seeds: int,
    stage_b_planner_seeds: int,
    direct_obstruction_samples: int,
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    prefix_fine_until: int,
    prefix_mid_step: int,
    prefix_coarse_step: int,
    stage_b_neighbor_radius: int,
    distribution_medium_ratio: float,
    distribution_hard_ratio: float,
    distribution_hard_not_faster_factor: float,
    distribution_require_strong_planner: bool,
) -> dict[str, tuple[int, dict[str, Any], float]]:
    all_counts = prefix_counts(
        len(bounds_ordered),
        fine_until=int(prefix_fine_until),
        mid_step=int(prefix_mid_step),
        coarse_step=int(prefix_coarse_step),
    )
    stage_a_rows = distribution_prefix_scan_rows(
        robot_name=robot_name,
        robot=robot,
        bounds_ordered=bounds_ordered,
        queries=queries,
        counts=all_counts,
        desc_suffix="stage-a-prefix-count",
        seed=int(seed),
        planner_seeds=max(1, int(stage_a_planner_seeds)),
        direct_obstruction_samples=int(direct_obstruction_samples),
        rrt_probe_timeout_s=float(rrt_probe_timeout_s),
        bitstar_timeout_s=float(bitstar_timeout_s),
        bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
        audit_step=float(audit_step),
        min_success_fraction=float(min_success_fraction),
        bitstar_probe_mode=str(bitstar_probe_mode),
        bitstar_samples_per_batch=int(bitstar_samples_per_batch),
        bitstar_rewire_factor=float(bitstar_rewire_factor),
    )
    stage_a_selection = select_distribution_prefixes_from_scan(
        stage_a_rows,
        medium_ratio=float(distribution_medium_ratio),
        hard_ratio=float(distribution_hard_ratio),
        hard_not_faster_factor=float(distribution_hard_not_faster_factor),
        require_strong_planner=bool(distribution_require_strong_planner),
    )
    selected_counts = [
        int(stage_a_selection["prefix_counts"][difficulty])
        for difficulty in DIFFICULTY_ORDER
    ]
    stage_b_counts = prefix_count_neighbors(
        all_counts,
        selected_counts,
        radius=int(stage_b_neighbor_radius),
    )
    stage_b_rows = distribution_prefix_scan_rows(
        robot_name=robot_name,
        robot=robot,
        bounds_ordered=bounds_ordered,
        queries=queries,
        counts=stage_b_counts,
        desc_suffix="stage-b-prefix-count",
        seed=int(seed) + 7_340_039,
        planner_seeds=max(1, int(stage_b_planner_seeds)),
        direct_obstruction_samples=int(direct_obstruction_samples),
        rrt_probe_timeout_s=float(rrt_probe_timeout_s),
        bitstar_timeout_s=float(bitstar_timeout_s),
        bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
        audit_step=float(audit_step),
        min_success_fraction=float(min_success_fraction),
        bitstar_probe_mode=str(bitstar_probe_mode),
        bitstar_samples_per_batch=int(bitstar_samples_per_batch),
        bitstar_rewire_factor=float(bitstar_rewire_factor),
    )
    stage_b_selection = select_distribution_prefixes_from_scan(
        stage_b_rows,
        medium_ratio=float(distribution_medium_ratio),
        hard_ratio=float(distribution_hard_ratio),
        hard_not_faster_factor=float(distribution_hard_not_faster_factor),
        require_strong_planner=bool(distribution_require_strong_planner),
    )
    stage_b_selection["two_stage_prefix_confirmation"] = {
        "stage_a_planner_seeds": int(stage_a_planner_seeds),
        "stage_b_planner_seeds": int(stage_b_planner_seeds),
        "stage_a_prefix_counts": copy.deepcopy(stage_a_selection["prefix_counts"]),
        "stage_b_counts_scanned": [int(value) for value in stage_b_counts],
        "stage_b_neighbor_radius": int(stage_b_neighbor_radius),
        "stage_a_scan": [compact_prefix_scan_row(row) for row in stage_a_rows],
    }
    out = distribution_prefix_output_from_selection(stage_b_selection)
    for _difficulty, (_count, probe, _direct_mean) in out.items():
        probe["distribution_selection"]["two_stage_prefix_confirmation"] = copy.deepcopy(
            stage_b_selection["two_stage_prefix_confirmation"]
        )
    return out


def _nearest_count(counts: Sequence[int], target: int) -> int:
    if not counts:
        return 0
    return int(min(counts, key=lambda value: (abs(int(value) - int(target)), int(value))))


def candidate_prefix_probe(
    *,
    difficulty: str,
    count: int,
    prefix_counts_by_difficulty: dict[str, int],
    query_count: int,
) -> dict[str, Any]:
    return {
        "planner": "candidate_prefix_generator",
        "policy": "candidate_unvalidated_prefixes",
        "aggregation": "no planner probes; run augment_prefix_catalog_queries.py for strict confirmation",
        "ok": False,
        "difficulty": str(difficulty),
        "query_count": int(query_count),
        "planner_seed_count": 0,
        "candidate_prefix_counts": {
            key: int(value) for key, value in prefix_counts_by_difficulty.items()
        },
        "rrtconnect": {
            "planner": "OMPL_RRTConnect",
            "ok": False,
            "success_count": 0,
            "query_count": 0,
            "success_fraction": 0.0,
            "median_first_success_s": math.nan,
            "mean_first_success_s": math.nan,
            "min_first_success_s": math.nan,
            "max_first_success_s": math.nan,
            "censored_timeout_s": math.nan,
            "window_s": [math.nan, math.nan],
            "timeout_s": math.nan,
            "probes": [],
        },
        "bitstar": {
            "planner": "OMPL_BITstar_trace",
            "ok": False,
            "success_count": 0,
            "query_count": 0,
            "success_fraction": 0.0,
            "median_first_success_s": math.nan,
            "median_first_success_checkpoint_s": math.nan,
            "success_checkpoint_count": 0,
            "mean_first_success_s": math.nan,
            "min_first_success_s": math.nan,
            "max_first_success_s": math.nan,
            "mean_first_success_checkpoint_s": math.nan,
            "censored_timeout_s": math.nan,
            "window_s": [math.nan, math.nan],
            "timeout_s": math.nan,
            "checkpoint_interval_s": math.nan,
            "probe_mode": "none",
            "samples_per_batch": 0,
            "rewire_factor": math.nan,
            "probes": [],
        },
        "candidate_note": (
            "This record is a cheap ordered-obstacle/query candidate. "
            "It is not a paper scene until distribution selection succeeds."
        ),
        "candidate_prefix_count": int(count),
    }


def find_candidate_prefixes(
    *,
    bounds_ordered: list[list[float]],
    queries: list[dict[str, Any]],
    prefix_fine_until: int,
    prefix_mid_step: int,
    prefix_coarse_step: int,
) -> dict[str, tuple[int, dict[str, Any], float]]:
    counts = prefix_counts(
        len(bounds_ordered),
        fine_until=int(prefix_fine_until),
        mid_step=int(prefix_mid_step),
        coarse_step=int(prefix_coarse_step),
    )
    if not counts:
        counts = [0]
    max_count = int(max(counts))
    prefix_by_difficulty = {
        "easy": 0,
        "medium": _nearest_count(counts, max(1, int(round(0.5 * max_count)))),
        "hard": max_count,
    }
    if prefix_by_difficulty["medium"] in {prefix_by_difficulty["easy"], prefix_by_difficulty["hard"]}:
        positives = [int(value) for value in counts if int(value) > 0]
        if len(positives) >= 2:
            prefix_by_difficulty["medium"] = positives[len(positives) // 2]
        elif positives:
            prefix_by_difficulty["medium"] = positives[0]
    out: dict[str, tuple[int, dict[str, Any], float]] = {}
    for difficulty in DIFFICULTY_ORDER:
        count = int(prefix_by_difficulty[difficulty])
        out[difficulty] = (
            count,
            candidate_prefix_probe(
                difficulty=difficulty,
                count=count,
                prefix_counts_by_difficulty=prefix_by_difficulty,
                query_count=len(queries),
            ),
            math.nan,
        )
    return out


def find_prefixes_by_mode(
    *,
    selection_mode: str,
    robot_name: str,
    robot: Any,
    bounds_ordered: list[list[float]],
    queries: list[dict[str, Any]],
    seed: int,
    planner_seeds: int,
    direct_obstruction_samples: int,
    rrt_windows: dict[str, tuple[float, float]],
    bitstar_windows: dict[str, tuple[float, float]],
    rrt_probe_timeout_s: float,
    bitstar_timeout_s: float,
    bitstar_checkpoint_interval_s: float,
    audit_step: float,
    min_success_fraction: float,
    bitstar_probe_mode: str,
    bitstar_samples_per_batch: int,
    bitstar_rewire_factor: float,
    prefix_fine_until: int,
    prefix_mid_step: int,
    prefix_coarse_step: int,
    distribution_medium_ratio: float,
    distribution_hard_ratio: float,
    distribution_hard_not_faster_factor: float,
    distribution_require_strong_planner: bool,
    prefix_confirm_mode: str = "single_stage",
    prefix_stage_a_planner_seeds: int = 1,
    prefix_stage_b_planner_seeds: int = 3,
    prefix_stage_b_neighbor_radius: int = 1,
) -> dict[str, tuple[int, dict[str, Any], float]]:
    mode = str(selection_mode).strip().lower()
    if mode == "candidate":
        return find_candidate_prefixes(
            bounds_ordered=bounds_ordered,
            queries=queries,
            prefix_fine_until=int(prefix_fine_until),
            prefix_mid_step=int(prefix_mid_step),
            prefix_coarse_step=int(prefix_coarse_step),
        )
    if mode == "distribution" and str(prefix_confirm_mode).strip().lower() == "two_stage":
        return find_distribution_prefixes_two_stage(
            robot_name=robot_name,
            robot=robot,
            bounds_ordered=bounds_ordered,
            queries=queries,
            seed=int(seed),
            stage_a_planner_seeds=int(prefix_stage_a_planner_seeds),
            stage_b_planner_seeds=int(prefix_stage_b_planner_seeds),
            direct_obstruction_samples=int(direct_obstruction_samples),
            rrt_probe_timeout_s=float(rrt_probe_timeout_s),
            bitstar_timeout_s=float(bitstar_timeout_s),
            bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
            audit_step=float(audit_step),
            min_success_fraction=float(min_success_fraction),
            bitstar_probe_mode=str(bitstar_probe_mode),
            bitstar_samples_per_batch=int(bitstar_samples_per_batch),
            bitstar_rewire_factor=float(bitstar_rewire_factor),
            prefix_fine_until=int(prefix_fine_until),
            prefix_mid_step=int(prefix_mid_step),
            prefix_coarse_step=int(prefix_coarse_step),
            stage_b_neighbor_radius=int(prefix_stage_b_neighbor_radius),
            distribution_medium_ratio=float(distribution_medium_ratio),
            distribution_hard_ratio=float(distribution_hard_ratio),
            distribution_hard_not_faster_factor=float(distribution_hard_not_faster_factor),
            distribution_require_strong_planner=bool(distribution_require_strong_planner),
        )
    if mode == "distribution":
        return find_distribution_prefixes(
            robot_name=robot_name,
            robot=robot,
            bounds_ordered=bounds_ordered,
            queries=queries,
            seed=int(seed),
            planner_seeds=int(planner_seeds),
            direct_obstruction_samples=int(direct_obstruction_samples),
            rrt_probe_timeout_s=float(rrt_probe_timeout_s),
            bitstar_timeout_s=float(bitstar_timeout_s),
            bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
            audit_step=float(audit_step),
            min_success_fraction=float(min_success_fraction),
            bitstar_probe_mode=str(bitstar_probe_mode),
            bitstar_samples_per_batch=int(bitstar_samples_per_batch),
            bitstar_rewire_factor=float(bitstar_rewire_factor),
            prefix_fine_until=int(prefix_fine_until),
            prefix_mid_step=int(prefix_mid_step),
            prefix_coarse_step=int(prefix_coarse_step),
            distribution_medium_ratio=float(distribution_medium_ratio),
            distribution_hard_ratio=float(distribution_hard_ratio),
            distribution_hard_not_faster_factor=float(distribution_hard_not_faster_factor),
            distribution_require_strong_planner=bool(distribution_require_strong_planner),
        )
    return find_prefixes(
        robot_name=robot_name,
        robot=robot,
        bounds_ordered=bounds_ordered,
        queries=queries,
        seed=int(seed),
        planner_seeds=int(planner_seeds),
        direct_obstruction_samples=int(direct_obstruction_samples),
        rrt_windows=rrt_windows,
        bitstar_windows=bitstar_windows,
        rrt_probe_timeout_s=float(rrt_probe_timeout_s),
        bitstar_timeout_s=float(bitstar_timeout_s),
        bitstar_checkpoint_interval_s=float(bitstar_checkpoint_interval_s),
        audit_step=float(audit_step),
        min_success_fraction=float(min_success_fraction),
        bitstar_probe_mode=str(bitstar_probe_mode),
        bitstar_samples_per_batch=int(bitstar_samples_per_batch),
        bitstar_rewire_factor=float(bitstar_rewire_factor),
        prefix_fine_until=int(prefix_fine_until),
        prefix_mid_step=int(prefix_mid_step),
        prefix_coarse_step=int(prefix_coarse_step),
    )


def build_robot_scene_group(args: argparse.Namespace, robot_name: str, scene_seed: int) -> list[dict[str, Any]]:
    robot = make_robot(robot_name)
    limits = robot_joint_limit_tuples(robot)
    robot_offset = {"iiwa": 0, "ur5": 1, "panda": 2}.get(str(robot_name).lower(), 7) * 1_000_003
    wall_dim = resolved_wall_dim(int(args.wall_dim), robot_name, len(limits))
    subbox_cap = resolved_subbox_cap(int(args.max_subboxes_per_cspace_box), robot_name)
    allowed_link_idxs = resolved_allowed_link_idxs(str(args.allowed_link_idxs), robot_name)
    last_error: Exception | None = None
    for scene_try in range(max(1, int(args.max_scene_tries))):
        generator_seed = int(args.seed_base) + 1009 * int(scene_seed) + robot_offset + 104729 * int(scene_try)
        rng = random.Random(generator_seed)
        try:
            if int(args.wall_count) > 1:
                start, goal, cspace_obstacles, cspace_meta = make_multi_wall_hyper_gate(
                    limits,
                    robot_name=robot_name,
                    rng=rng,
                    scene_try=scene_try,
                    wall_dim=wall_dim,
                    wall_count=int(args.wall_count),
                )
            else:
                start, goal, cspace_obstacles, cspace_meta = make_local_hyper_gate(
                    limits,
                    robot_name=robot_name,
                    difficulty="hard",
                    rng=rng,
                    scene_try=scene_try,
                    wall_dim=wall_dim,
                )
            cspace_obstacles = scaled_gate_gap_boxes(
                cspace_obstacles,
                cspace_meta,
                float(args.gate_gap_scale),
            )
            cspace_meta = dict(cspace_meta)
            cspace_meta["gate_gap_scale"] = float(args.gate_gap_scale)
            subboxes: list[CSpaceBox] = []
            if str(args.map_mode) == "path_cells":
                subboxes = make_path_collision_cells(
                    limits,
                    cspace_obstacles,
                    start,
                    goal,
                    samples=int(args.direct_obstruction_samples),
                    half_width=float(args.path_cell_half_width),
                    max_cells=int(args.max_path_cells),
                )
            else:
                for box in cspace_obstacles:
                    subboxes.extend(
                        subdivide_cspace_box(
                            box,
                            max_width=float(args.cspace_subdivide_width),
                            max_boxes=max(1, int(subbox_cap)),
                        )
                    )
            if not subboxes:
                raise RuntimeError("no cspace subboxes for workspace mapping")
            mapped = map_cspace_boxes_to_workspace(
                robot_name=robot_name,
                boxes=subboxes,
                use_inflated=bool(args.use_inflated_envelope),
                workspace_margin=float(args.workspace_margin),
                endpoint_source=str(args.endpoint_source),
                n_samples_crit=int(args.n_samples_crit),
                endpoint_threads=int(args.endpoint_threads),
                n_subdivisions=int(args.envelope_subdivisions),
                workspace_aabb_shrink=float(args.workspace_aabb_shrink),
                min_active_link_idx=int(args.min_active_link_idx),
                max_active_link_idx=int(args.max_active_link_idx),
                allowed_link_idxs=allowed_link_idxs,
            )
            bounds = dedupe_obstacles(mapped.get("obstacles", []), precision=int(args.dedupe_precision))
            endpoint_safe: list[list[float]] = []
            for item in bounds:
                obstacle = obstacle_from_bounds(item)
                if sbf.check_config_collision(robot, [obstacle], start) or sbf.check_config_collision(robot, [obstacle], goal):
                    continue
                endpoint_safe.append([float(value) for value in item])
            if not endpoint_safe:
                raise RuntimeError("mapping produced no endpoint-safe obstacles")
            q0_scored = [
                (
                    (
                        single_obstacle_direct_hits(
                            sbf,
                            robot,
                            obstacle_from_bounds(item),
                            start,
                            goal,
                            int(args.direct_obstruction_samples),
                        ),
                        obstacle_volume(item),
                    ),
                    item,
                )
                for item in endpoint_safe
            ]
            q0_scored.sort(key=lambda pair: (pair[0][0], pair[0][1]), reverse=True)
            query_cap = int(args.query_sampling_obstacle_cap)
            query_bounds = (
                [item for _score, item in q0_scored[:query_cap]]
                if query_cap > 0
                else [item for _score, item in q0_scored]
            )
            full_obstacles = [obstacle_from_bounds(item) for item in query_bounds]
            initial_query_count = int(args.initial_queries_per_scene)
            if initial_query_count <= 0:
                initial_query_count = int(args.queries_per_scene)
            initial_query_count = max(1, min(int(args.queries_per_scene), int(initial_query_count)))
            if str(args.query_sampling_mode) == "hard_probe":
                hard_query_count = int(args.hard_query_prefix_count)
                provisional_bounds = (
                    query_bounds[:hard_query_count]
                    if hard_query_count > 0
                    else query_bounds
                )
                if not provisional_bounds:
                    raise RuntimeError("hard_probe query sampling has no provisional obstacles")
                queries = sample_hard_probe_queries(
                    robot=robot,
                    obstacles=[obstacle_from_bounds(item) for item in provisional_bounds],
                    count=initial_query_count,
                    seed=generator_seed,
                    min_l2=float(args.query_min_l2),
                    max_l2=query_max_l2_limit(float(args.query_max_l2)),
                    clearance_margin_m=float(args.query_clearance_margin_m),
                    max_tries=int(args.query_max_tries),
                    rrt_timeout_s=float(args.rrt_probe_timeout_s),
                    bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                    bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                    audit_step=float(args.strict_audit_step),
                    bitstar_min_s=float(args.hard_query_bitstar_min_s),
                    bitstar_max_s=float(args.hard_query_bitstar_max_s),
                    rrt_min_s=float(args.hard_query_rrt_min_s),
                    rrt_max_s=float(args.hard_query_rrt_max_s),
                    bitstar_probe_mode=str(args.bitstar_probe_mode),
                    bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                    bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                    direct_obstruction_samples=int(args.direct_obstruction_samples),
                )
            else:
                queries = sample_shared_queries(
                    robot=robot,
                    obstacles=full_obstacles,
                    start=start,
                    goal=goal,
                    count=initial_query_count,
                    seed=generator_seed,
                    min_l2=float(args.query_min_l2),
                    max_l2=query_max_l2_limit(float(args.query_max_l2)),
                    local_radius=float(args.query_local_radius),
                    max_tries=int(args.query_max_tries),
                )
            query_endpoint_safe: list[list[float]] = []
            for item in endpoint_safe:
                obstacle = obstacle_from_bounds(item)
                if any(
                    sbf.check_config_collision(robot, [obstacle], query["start"])
                    or sbf.check_config_collision(robot, [obstacle], query["goal"])
                    for query in queries
                ):
                    continue
                query_endpoint_safe.append(item)
            if not query_endpoint_safe:
                raise RuntimeError("no obstacles remain endpoint-safe for all shared queries")
            max_obstacles = int(args.max_workspace_obstacles)
            probe_greedy_trace: list[dict[str, Any]] = []
            if str(args.obstacle_order) == "path_blocking":
                initial_path_blocking_obstacles: list[list[float]] | None = None
                if int(args.path_blocking_initial_mapped_obstacles) != 0:
                    initial_cap = int(args.path_blocking_initial_mapped_obstacles)
                    if str(args.path_blocking_initial_order).strip().lower() == "query_sampling":
                        initial_ordered = [
                            [float(value) for value in item]
                            for item in query_bounds[:max_obstacles if max_obstacles > 0 else len(query_bounds)]
                        ]
                    else:
                        initial_ordered = ordered_obstacle_bounds(
                            robot=robot,
                            bounds=query_endpoint_safe,
                            queries=queries,
                            samples=int(args.direct_obstruction_samples),
                            strategy=str(args.path_blocking_initial_order),
                            max_obstacles=max_obstacles,
                            seed=generator_seed,
                        )
                    initial_path_blocking_obstacles = (
                        initial_ordered[:initial_cap]
                        if initial_cap > 0
                        else initial_ordered
                    )
                ordered, probe_greedy_trace = path_blocking_obstacle_bounds(
                    robot_name=robot_name,
                    robot=robot,
                    limits=limits,
                    queries=queries,
                    seed=generator_seed,
                    planner_seeds=int(args.planner_seeds),
                    rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
                    bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                    bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                    audit_step=float(args.strict_audit_step),
                    min_success_fraction=float(args.min_probe_success_fraction),
                    bitstar_probe_mode=str(args.bitstar_probe_mode),
                    bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                    bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                    direct_obstruction_samples=int(args.direct_obstruction_samples),
                    max_obstacles=max_obstacles,
                    max_prefix=int(args.path_blocking_max_prefix),
                    candidate_pool=int(args.path_blocking_candidate_pool),
                    samples_per_path=int(args.path_blocking_samples_per_path),
                    box_half_width=float(args.path_blocking_box_half_width),
                    use_inflated=bool(args.use_inflated_envelope),
                    workspace_margin=float(args.workspace_margin),
                    endpoint_source=str(args.endpoint_source),
                    n_samples_crit=int(args.n_samples_crit),
                    endpoint_threads=int(args.endpoint_threads),
                    n_subdivisions=int(args.envelope_subdivisions),
                    workspace_aabb_shrink=float(args.workspace_aabb_shrink),
                    min_active_link_idx=int(args.min_active_link_idx),
                    max_active_link_idx=int(args.max_active_link_idx),
                    allowed_link_idxs=allowed_link_idxs,
                    query_count=int(args.path_blocking_query_count),
                    initial_obstacles=initial_path_blocking_obstacles,
                )
            elif str(args.obstacle_order) == "probe_greedy":
                ordered, probe_greedy_trace = probe_guided_obstacle_bounds(
                    robot_name=robot_name,
                    robot=robot,
                    bounds=query_endpoint_safe,
                    queries=queries,
                    samples=int(args.direct_obstruction_samples),
                    max_obstacles=max_obstacles,
                    seed=generator_seed,
                    planner_seeds=int(args.planner_seeds),
                    rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
                    bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                    bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                    audit_step=float(args.strict_audit_step),
                    min_success_fraction=float(args.min_probe_success_fraction),
                    bitstar_probe_mode=str(args.bitstar_probe_mode),
                    bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                    bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                    candidate_pool=int(args.probe_greedy_candidate_pool),
                    max_prefix=int(args.probe_greedy_max_prefix),
                )
            else:
                ordered = ordered_obstacle_bounds(
                    robot=robot,
                    bounds=query_endpoint_safe,
                    queries=queries,
                    samples=int(args.direct_obstruction_samples),
                    strategy=str(args.obstacle_order),
                    max_obstacles=max_obstacles,
                    seed=generator_seed,
                )
            prefixes = find_prefixes_by_mode(
                selection_mode=str(args.prefix_selection_mode),
                robot_name=robot_name,
                robot=robot,
                bounds_ordered=ordered,
                queries=queries,
                seed=generator_seed,
                planner_seeds=int(args.planner_seeds),
                direct_obstruction_samples=int(args.direct_obstruction_samples),
                rrt_windows=parse_window_map(str(args.rrt_median_windows)),
                bitstar_windows=parse_window_map(str(args.bitstar_median_windows)),
                rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
                bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                audit_step=float(args.strict_audit_step),
                min_success_fraction=float(args.min_probe_success_fraction),
                bitstar_probe_mode=str(args.bitstar_probe_mode),
                bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                prefix_fine_until=int(args.prefix_fine_until),
                prefix_mid_step=int(args.prefix_mid_step),
                prefix_coarse_step=int(args.prefix_coarse_step),
                distribution_medium_ratio=float(args.distribution_medium_ratio),
                distribution_hard_ratio=float(args.distribution_hard_ratio),
                distribution_hard_not_faster_factor=float(args.distribution_hard_not_faster_factor),
                distribution_require_strong_planner=bool(args.distribution_require_strong_planner),
                prefix_confirm_mode=str(args.prefix_confirm_mode),
                prefix_stage_a_planner_seeds=int(args.prefix_stage_a_planner_seeds),
                prefix_stage_b_planner_seeds=int(args.prefix_stage_b_planner_seeds),
                prefix_stage_b_neighbor_radius=int(args.prefix_stage_b_neighbor_radius),
            )
            post_query_rejections: dict[str, int] = {}
            if bool(args.post_sample_matching_queries) and len(queries) < int(args.queries_per_scene):
                queries, post_query_rejections = augment_prefix_matching_queries(
                    robot=robot,
                    bounds_ordered=ordered,
                    prefixes=prefixes,
                    queries=queries,
                    target_count=int(args.queries_per_scene),
                    seed=generator_seed,
                    min_l2=float(args.query_min_l2),
                    max_l2=query_max_l2_limit(float(args.query_max_l2)),
                    local_radius=float(args.query_local_radius),
                    clearance_margin_m=float(args.query_clearance_margin_m),
                    max_tries=int(args.post_query_max_tries),
                    planner_seeds=int(args.post_query_planner_seeds),
                    rrt_windows=parse_window_map(str(args.rrt_median_windows)),
                    bitstar_windows=parse_window_map(str(args.bitstar_median_windows)),
                    post_rrt_windows=(
                        parse_window_map(str(args.post_query_rrt_median_windows))
                        if str(args.post_query_rrt_median_windows).strip()
                        else None
                    ),
                    post_bitstar_windows=(
                        parse_window_map(str(args.post_query_bitstar_median_windows))
                        if str(args.post_query_bitstar_median_windows).strip()
                        else None
                    ),
                    rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
                    bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                    bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                    audit_step=float(args.strict_audit_step),
                    bitstar_probe_mode=str(args.bitstar_probe_mode),
                    bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                    bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                    check_mode=str(args.post_query_check_mode),
                    local_sample_prob=float(args.post_query_local_prob),
                    group_min_success_fraction=float(args.min_probe_success_fraction),
                )
                prefixes = find_prefixes_by_mode(
                    selection_mode=str(args.prefix_selection_mode),
                    robot_name=robot_name,
                    robot=robot,
                    bounds_ordered=ordered,
                    queries=queries,
                    seed=generator_seed + 4_194_301,
                    planner_seeds=int(args.planner_seeds),
                    direct_obstruction_samples=int(args.direct_obstruction_samples),
                    rrt_windows=parse_window_map(str(args.rrt_median_windows)),
                    bitstar_windows=parse_window_map(str(args.bitstar_median_windows)),
                    rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
                    bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                    bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                    audit_step=float(args.strict_audit_step),
                    min_success_fraction=float(args.min_probe_success_fraction),
                    bitstar_probe_mode=str(args.bitstar_probe_mode),
                    bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                    bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                    prefix_fine_until=int(args.prefix_fine_until),
                    prefix_mid_step=int(args.prefix_mid_step),
                    prefix_coarse_step=int(args.prefix_coarse_step),
                    distribution_medium_ratio=float(args.distribution_medium_ratio),
                    distribution_hard_ratio=float(args.distribution_hard_ratio),
                    distribution_hard_not_faster_factor=float(args.distribution_hard_not_faster_factor),
                    distribution_require_strong_planner=bool(args.distribution_require_strong_planner),
                    prefix_confirm_mode=str(args.prefix_confirm_mode),
                    prefix_stage_a_planner_seeds=int(args.prefix_stage_a_planner_seeds),
                    prefix_stage_b_planner_seeds=int(args.prefix_stage_b_planner_seeds),
                    prefix_stage_b_neighbor_radius=int(args.prefix_stage_b_neighbor_radius),
                )
            source_cspace = {
                "generator": "local_hyper_gate_hard_then_workspace_prefix",
                "obstacles": [{"intervals": flatten_cspace_box(box)} for box in cspace_obstacles],
                "subbox_count": int(len(subboxes)),
                "map_mode": str(args.map_mode),
                "metadata": cspace_meta,
            }
            mapping = {
                "policy": "cspace_box_to_link_iaabb_workspace_aabb_sorted_prefix",
                "endpoint_source": str(args.endpoint_source),
                "n_samples_crit": int(args.n_samples_crit),
                "endpoint_threads": int(args.endpoint_threads),
                "use_inflated_envelope": bool(args.use_inflated_envelope),
                "workspace_margin": float(args.workspace_margin),
                "workspace_aabb_shrink": float(args.workspace_aabb_shrink),
                "min_active_link_idx": int(args.min_active_link_idx),
                "max_active_link_idx": int(args.max_active_link_idx),
                "allowed_link_idxs": list(allowed_link_idxs),
                "wall_dim": int(wall_dim),
                "envelope_subdivisions": int(args.envelope_subdivisions),
                "map_mode": str(args.map_mode),
                "path_cell_half_width": float(args.path_cell_half_width),
                "max_path_cells": int(args.max_path_cells),
                "mapped_obstacle_count": int(len(endpoint_safe)),
                "query_endpoint_safe_obstacle_count": int(len(query_endpoint_safe)),
                "ordered_obstacle_count": int(len(ordered)),
                "ordered_obstacles": [[float(value) for value in item] for item in ordered],
                "prefix_selection_mode": str(args.prefix_selection_mode),
                "prefix_confirm_mode": str(args.prefix_confirm_mode),
                "prefix_stage_a_planner_seeds": int(args.prefix_stage_a_planner_seeds),
                "prefix_stage_b_planner_seeds": int(args.prefix_stage_b_planner_seeds),
                "prefix_stage_b_neighbor_radius": int(args.prefix_stage_b_neighbor_radius),
                "distribution_separation": {
                    "medium_over_easy_min": float(args.distribution_medium_ratio),
                    "hard_over_medium_min": float(args.distribution_hard_ratio),
                    "hard_not_faster_factor": float(args.distribution_hard_not_faster_factor),
                    "require_strong_reference_planner": bool(args.distribution_require_strong_planner),
                    "time_floor_s": float(TIME_FLOOR_S),
                },
                "query_sampling_obstacle_count": int(len(query_bounds)),
                "query_sampling_mode": str(args.query_sampling_mode),
                "initial_queries_per_scene": int(initial_query_count),
                "post_sample_matching_queries": bool(args.post_sample_matching_queries),
                "post_query_max_tries": int(args.post_query_max_tries),
                "post_query_planner_seeds": int(args.post_query_planner_seeds),
                "post_query_check_mode": str(args.post_query_check_mode),
                "post_query_local_sample_probability": float(args.post_query_local_prob),
                "post_query_rrt_median_windows": str(args.post_query_rrt_median_windows),
                "post_query_bitstar_median_windows": str(args.post_query_bitstar_median_windows),
                "post_query_rejections": post_query_rejections,
                "query_min_l2": float(args.query_min_l2),
                "query_max_l2": None if math.isinf(query_max_l2_limit(float(args.query_max_l2))) else float(args.query_max_l2),
                "query_max_l2_unbounded": math.isinf(query_max_l2_limit(float(args.query_max_l2))),
                "hard_query_prefix_count": int(args.hard_query_prefix_count),
                "hard_query_bitstar_min_s": float(args.hard_query_bitstar_min_s),
                "bitstar_samples_per_batch": int(args.bitstar_samples_per_batch),
                "bitstar_rewire_factor": float(args.bitstar_rewire_factor),
                "direct_obstruction_samples": int(args.direct_obstruction_samples),
                "obstacle_order": str(args.obstacle_order),
                "probe_greedy_candidate_pool": int(args.probe_greedy_candidate_pool),
                "probe_greedy_max_prefix": int(args.probe_greedy_max_prefix),
                "path_blocking_candidate_pool": int(args.path_blocking_candidate_pool),
                "path_blocking_max_prefix": int(args.path_blocking_max_prefix),
                "path_blocking_samples_per_path": int(args.path_blocking_samples_per_path),
                "path_blocking_box_half_width": float(args.path_blocking_box_half_width),
                "path_blocking_query_count": int(args.path_blocking_query_count),
                "path_blocking_initial_mapped_obstacles": int(args.path_blocking_initial_mapped_obstacles),
                "path_blocking_initial_order": str(args.path_blocking_initial_order),
                "probe_greedy_trace": probe_greedy_trace,
                "envelope_time_us": float(mapped.get("envelope_time_us", math.nan)),
            }
            rows = []
            for difficulty in DIFFICULTY_ORDER:
                count, probe, direct_mean = prefixes[difficulty]
                rows.append(
                    build_record(
                        robot_name=robot_name,
                        difficulty=difficulty,
                        scene_seed=scene_seed,
                        generator_seed=generator_seed,
                        queries=queries,
                        obstacle_bounds_list=ordered[:count],
                        difficulty_probe=probe,
                        source_cspace=source_cspace,
                        mapping=mapping,
                        direct_mean=direct_mean,
                    )
                )
            return rows
        except Exception as exc:
            last_error = exc
    raise RuntimeError(f"could not build prefix mapped scene for {robot_name}/{scene_seed}: {last_error}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate strict prefix Exp.6 scenes with RRT/BIT* median gates.")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--scene-seeds", type=int, default=1)
    parser.add_argument("--scene-seed-start", type=int, default=0)
    parser.add_argument("--queries-per-scene", type=int, default=10)
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--max-scene-tries", type=int, default=24)
    parser.add_argument("--planner-seeds", type=int, default=1)
    parser.add_argument("--rrt-median-windows", default=DEFAULT_RRT_MEDIAN_WINDOWS)
    parser.add_argument("--bitstar-median-windows", default=DEFAULT_BITSTAR_MEDIAN_WINDOWS)
    parser.add_argument("--rrt-probe-timeout-s", type=float, default=0.25)
    parser.add_argument("--bitstar-probe-timeout-s", type=float, default=BITSTAR_PROBE_TIMEOUT_S)
    parser.add_argument("--bitstar-probe-checkpoint-interval-s", type=float, default=BITSTAR_PROBE_CHECKPOINT_INTERVAL_S)
    parser.add_argument("--bitstar-probe-mode", choices=("path", "trace"), default="path")
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=BITSTAR_PROBE_SAMPLES_PER_BATCH)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=BITSTAR_PROBE_REWIRE_FACTOR)
    parser.add_argument("--strict-audit-step", type=float, default=0.01)
    parser.add_argument("--min-probe-success-fraction", type=float, default=0.5)
    parser.add_argument("--prefix-fine-until", type=int, default=80)
    parser.add_argument("--prefix-mid-step", type=int, default=5)
    parser.add_argument("--prefix-coarse-step", type=int, default=25)
    parser.add_argument("--prefix-selection-mode", choices=("distribution", "window", "candidate"), default="distribution")
    parser.add_argument("--prefix-confirm-mode", choices=("single_stage", "two_stage"), default="single_stage")
    parser.add_argument("--prefix-stage-a-planner-seeds", type=int, default=1)
    parser.add_argument("--prefix-stage-b-planner-seeds", type=int, default=3)
    parser.add_argument("--prefix-stage-b-neighbor-radius", type=int, default=1)
    parser.add_argument("--distribution-medium-ratio", type=float, default=5.0)
    parser.add_argument("--distribution-hard-ratio", type=float, default=2.0)
    parser.add_argument("--distribution-hard-not-faster-factor", type=float, default=1.0)
    parser.add_argument("--distribution-require-strong-planner", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--query-min-l2", type=float, default=0.8)
    parser.add_argument("--query-max-l2", type=float, default=math.inf, help="Maximum start-goal L2 distance; <=0 or inf disables the upper bound.")
    parser.add_argument("--initial-queries-per-scene", type=int, default=0)
    parser.add_argument("--post-sample-matching-queries", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--post-query-max-tries", type=int, default=4096)
    parser.add_argument("--post-query-planner-seeds", type=int, default=1)
    parser.add_argument(
        "--post-query-check-mode",
        choices=("all", "hard", "group", "none"),
        default="all",
        help="Whether each post-sampled query is gated individually, by hard only, by the growing query-set median, or only by final prefix search.",
    )
    parser.add_argument("--post-query-local-prob", type=float, default=0.85)
    parser.add_argument("--post-query-rrt-median-windows", default="")
    parser.add_argument("--post-query-bitstar-median-windows", default="")
    parser.add_argument("--query-local-radius", type=float, default=0.22)
    parser.add_argument("--query-clearance-margin-m", type=float, default=0.0)
    parser.add_argument("--query-max-tries", type=int, default=4096)
    parser.add_argument("--query-sampling-mode", choices=("local", "hard_probe"), default="local")
    parser.add_argument("--hard-query-prefix-count", type=int, default=0)
    parser.add_argument("--hard-query-bitstar-min-s", type=float, default=0.05)
    parser.add_argument("--hard-query-bitstar-max-s", type=float, default=0.75)
    parser.add_argument("--hard-query-rrt-min-s", type=float, default=0.003)
    parser.add_argument("--hard-query-rrt-max-s", type=float, default=0.25)
    parser.add_argument("--wall-dim", type=int, default=-1)
    parser.add_argument("--wall-count", type=int, default=1)
    parser.add_argument("--gate-gap-scale", type=float, default=1.0)
    parser.add_argument("--map-mode", choices=("full", "path_cells"), default="full")
    parser.add_argument("--path-cell-half-width", type=float, default=0.08)
    parser.add_argument("--max-path-cells", type=int, default=96)
    parser.add_argument("--cspace-subdivide-width", type=float, default=0.22)
    parser.add_argument("--max-subboxes-per-cspace-box", type=int, default=-1)
    parser.add_argument("--max-workspace-obstacles", type=int, default=450)
    parser.add_argument(
        "--obstacle-order",
        choices=(
            "direct_desc",
            "direct_asc",
            "volume_asc",
            "volume_desc",
            "random",
            "mixed",
            "source_order",
            "probe_greedy",
            "path_blocking",
        ),
        default="direct_desc",
    )
    parser.add_argument("--probe-greedy-candidate-pool", type=int, default=8)
    parser.add_argument("--probe-greedy-max-prefix", type=int, default=40)
    parser.add_argument("--path-blocking-candidate-pool", type=int, default=8)
    parser.add_argument("--path-blocking-max-prefix", type=int, default=40)
    parser.add_argument("--path-blocking-samples-per-path", type=int, default=4)
    parser.add_argument("--path-blocking-box-half-width", type=float, default=0.08)
    parser.add_argument("--path-blocking-query-count", type=int, default=1)
    parser.add_argument(
        "--path-blocking-initial-mapped-obstacles",
        type=int,
        default=0,
        help="Seed path-blocking with mapped obstacles before appending path blockers; negative keeps all mapped obstacles.",
    )
    parser.add_argument("--path-blocking-initial-order", default="mixed")
    parser.add_argument("--query-sampling-obstacle-cap", type=int, default=0)
    parser.add_argument("--dedupe-precision", type=int, default=5)
    parser.add_argument("--endpoint-source", default="critsample")
    parser.add_argument("--n-samples-crit", type=int, default=1000)
    parser.add_argument("--endpoint-threads", type=int, default=1)
    parser.add_argument("--envelope-subdivisions", type=int, default=1)
    parser.add_argument("--use-inflated-envelope", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--workspace-margin", type=float, default=0.0)
    parser.add_argument("--workspace-aabb-shrink", type=float, default=0.18)
    parser.add_argument("--min-active-link-idx", type=int, default=0)
    parser.add_argument("--max-active-link-idx", type=int, default=1000000)
    parser.add_argument("--allowed-link-idxs", default="auto")
    parser.add_argument("--direct-obstruction-samples", type=int, default=96)
    parser.add_argument("--summary-json", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    t0 = time.perf_counter()
    robots = csv_list(args.robots)
    rrt_windows = parse_window_map(str(args.rrt_median_windows))
    bitstar_windows = parse_window_map(str(args.bitstar_median_windows))
    records: list[dict[str, Any]] = []
    scene_seed_start = int(args.scene_seed_start)
    scene_seed_count = max(1, int(args.scene_seeds))
    keys = [
        (robot, seed)
        for robot in robots
        for seed in range(scene_seed_start, scene_seed_start + scene_seed_count)
    ]
    for robot_name, scene_seed in progress(keys, total=len(keys), desc="prefix-mapped-catalog"):
        group = build_robot_scene_group(args, robot_name, int(scene_seed))
        records.extend(group)
        for row in group:
            probe = row["difficulty_probe"]
            print(
                "[prefix-mapped]",
                row["robot"],
                row["difficulty"],
                row["scene_seed"],
                f"obs={len(row['obstacles'])}",
                f"queries={len(row['queries'])}",
                f"rrt={float(probe['rrtconnect']['median_first_success_s']):.4f}",
                f"bitstar={float(probe['bitstar']['median_first_success_s']):.4f}",
                flush=True,
            )
    payload = {
        "schema": CATALOG_SCHEMA,
        "scene_profile": "prefix_cspace_mapped_workspace_median_gated",
        "robots": robots,
        "difficulties": list(DIFFICULTY_ORDER),
        "scene_seeds": int(scene_seed_count),
        "scene_seed_start": int(scene_seed_start),
        "queries_per_scene": int(args.queries_per_scene),
        "seed_base": int(args.seed_base),
        "records": records,
        "partial": False,
        "generation_s": time.perf_counter() - t0,
        "generation_policy": {
            "obstacle_prefixes": True,
            "shared_queries_across_difficulties": True,
            "difficulty_gate": (
                "distribution-separated RRTConnect/BIT* observed first strict solution times"
                if str(args.prefix_selection_mode) == "distribution"
                else (
                    "unvalidated candidate prefixes; strict confirmation must be run with augment_prefix_catalog_queries.py"
                    if str(args.prefix_selection_mode) == "candidate"
                    else "RRTConnect and BIT* observed-elapsed median first strict solution over shared queries and planner seeds"
                )
            ),
            "planner_seed_count": int(args.planner_seeds),
            "prefix_selection_mode": str(args.prefix_selection_mode),
            "prefix_confirm_mode": str(args.prefix_confirm_mode),
            "prefix_stage_a_planner_seeds": int(args.prefix_stage_a_planner_seeds),
            "prefix_stage_b_planner_seeds": int(args.prefix_stage_b_planner_seeds),
            "prefix_stage_b_neighbor_radius": int(args.prefix_stage_b_neighbor_radius),
            "distribution_separation": {
                "medium_over_easy_min": float(args.distribution_medium_ratio),
                "hard_over_medium_min": float(args.distribution_hard_ratio),
                "hard_not_faster_factor": float(args.distribution_hard_not_faster_factor),
                "require_strong_reference_planner": bool(args.distribution_require_strong_planner),
                "time_floor_s": float(TIME_FLOOR_S),
            },
            "rrtconnect_windows_s": {
                key: list(window_for(rrt_windows, key, timed_probe_window_s(key, strict_time=False)))
                for key in DIFFICULTY_ORDER
            },
            "bitstar_windows_s": {
                key: list(window_for(bitstar_windows, key, bitstar_median_window_s(key)))
                for key in DIFFICULTY_ORDER
            },
            "bitstar_timeout_s": float(args.bitstar_probe_timeout_s),
            "rrtconnect_timeout_s": float(args.rrt_probe_timeout_s),
            "bitstar_checkpoint_interval_s": float(args.bitstar_probe_checkpoint_interval_s),
            "bitstar_probe_mode": str(args.bitstar_probe_mode),
            "bitstar_samples_per_batch": int(args.bitstar_samples_per_batch),
            "bitstar_rewire_factor": float(args.bitstar_rewire_factor),
            "strict_audit_step": float(args.strict_audit_step),
            "minimum_probe_success_fraction": float(args.min_probe_success_fraction),
            "query_min_l2": float(args.query_min_l2),
            "query_max_l2": None if math.isinf(query_max_l2_limit(float(args.query_max_l2))) else float(args.query_max_l2),
            "query_max_l2_unbounded": math.isinf(query_max_l2_limit(float(args.query_max_l2))),
            "obstacle_count_policy": (
                "searched prefix counts; accept easy/medium/hard only when first-solution time distributions are separated"
                if str(args.prefix_selection_mode) == "distribution"
                else (
                    "cheap candidate counts only; final counts selected by later strict distribution confirmation"
                    if str(args.prefix_selection_mode) == "candidate"
                    else "searched prefix count; no fixed count target; accept only if RRTConnect and BIT* median first-solution windows pass"
                )
            ),
            "prefix_search": {
                "fine_until": int(args.prefix_fine_until),
                "mid_step": int(args.prefix_mid_step),
                "coarse_step": int(args.prefix_coarse_step),
                "max_workspace_obstacles_cap": int(args.max_workspace_obstacles),
            },
        },
        "environment": environment_metadata(),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    summary = {
        "catalog": str(args.out),
        "records": len(records),
        "robots": robots,
        "scene_seeds": int(args.scene_seeds),
        "scene_seed_start": int(scene_seed_start),
        "queries_per_scene": int(args.queries_per_scene),
        "generation_s": payload["generation_s"],
    }
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
