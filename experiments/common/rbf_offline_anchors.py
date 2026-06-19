from __future__ import annotations

import math
import random
from typing import Any

from experiments.common.metrics import mean
from experiments.common.rbf_defaults import robot_joint_limit_tuples, robot_symmetry_aligned_root_tuples
from experiments.common.rbf_split_policy import (
    make_aafk_split_policy,
    make_aafk_split_policy_from_cache_prefix,
    normalized_split_schedule_kind,
    read_lect_cache_depth_dimensions,
)
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


def _coverage_root_tuple_list(robot: Any, options: Any) -> list[tuple[float, float]]:
    if options.coverage_override_tuples is not None:
        return [(float(lo), float(hi)) for lo, hi in options.coverage_override_tuples]
    return robot_joint_limit_tuples(robot)


def _active_tree_root_tuple_list(robot: Any, options: Any) -> list[tuple[float, float]]:
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


def empty_offline_anchor_metrics(reason: str) -> dict[str, Any]:
    return {
        "offline_anchor_candidates": 0,
        "offline_anchor_candidates_free": 0,
        "offline_anchor_roots_requested": 0,
        "offline_anchor_lca_depth_mean": math.nan,
        "offline_anchor_lca_depth_max": math.nan,
        "offline_anchor_min_distance_mean": math.nan,
        "offline_anchor_skip_reason": str(reason),
        "offline_anchor_skip_p_main_accessible": math.nan,
    }


def generate_offline_anchor_points(
    robot: Any,
    obstacles: list[Any],
    options: Any,
) -> tuple[list[list[float]], dict[str, Any]]:
    count = max(0, int(options.offline_anchor_count))
    candidate_count = max(0, int(options.offline_anchor_candidate_count))
    if not bool(options.offline_random_anchors) or count <= 0 or candidate_count <= 0:
        return [], empty_offline_anchor_metrics("disabled")
    coverage_root = _coverage_root_tuple_list(robot, options)
    tree_root = _active_tree_root_tuple_list(robot, options)
    cache_schedule = (
        read_lect_cache_depth_dimensions(options.external_evidence_path)
        if (bool(options.use_external_evidence) or bool(options.symmetry_aligned_cache_schedule))
        and options.external_evidence_path is not None
        else []
    )
    split_schedule_kind = normalized_split_schedule_kind(getattr(options, "split_schedule_kind", "aafk_volume_min"))
    if cache_schedule and split_schedule_kind == "aafk_volume_min":
        split_policy = make_aafk_split_policy_from_cache_prefix(
            robot,
            int(options.rbf_max_depth),
            cache_schedule,
            [sbf.Interval(float(lo), float(hi)) for lo, hi in tree_root],
        )
    else:
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
            split_schedule_kind=split_schedule_kind,
        )
    schedule = [int(dim) for dim in list(split_policy.depth_dimensions)]
    rng = random.Random((int(options.seed) + 1) * 1000003)
    sampling = str(getattr(options, "offline_anchor_sampling", "random")).strip().lower()
    candidates: list[list[float]] = []
    halton_index = 1 + int(options.seed) * 1009
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
        "offline_anchor_skip_reason": "",
        "offline_anchor_skip_p_main_accessible": math.nan,
    }
    return selected, metrics
