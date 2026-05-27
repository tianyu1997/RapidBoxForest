#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence


def _bootstrap_imports() -> tuple[Path, Path]:
    root = Path(__file__).resolve().parents[1]
    lie_root = root.parent / "link-interval-envelope"
    build_dir = os.environ.get("LIE_BUILD_DIR")
    candidates: list[Path] = []
    if build_dir:
        build_path = Path(build_dir)
        candidates.extend((
            build_path / "python",
            build_path / "python" / "link_interval_envelope" / "Release",
            build_path / "python" / "link_interval_envelope" / "RelWithDebInfo",
        ))
    temp_build = Path(os.environ.get("TEMP", "")) / "lie_py310_force"
    for default_build in (temp_build, lie_root / "build_lie_py_force", lie_root / "build_py310", lie_root / "build"):
        candidates.extend((
            default_build / "python",
            default_build / "python" / "link_interval_envelope" / "Release",
            default_build / "python" / "link_interval_envelope" / "RelWithDebInfo",
        ))
    candidates.append(lie_root / "python")
    for candidate in reversed(candidates):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root, lie_root


ROOT, LIE_ROOT = _bootstrap_imports()

try:
    import link_interval_envelope as lie
except ImportError as exc:  # pragma: no cover - only used when the extension is absent.
    raise SystemExit(
        "link_interval_envelope is required for this weak-dependency runner. "
        "Build link-interval-envelope first, or set LIE_BUILD_DIR to its build directory."
    ) from exc


IntervalBox = tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class AABB:
    x0: float
    y0: float
    z0: float
    x1: float
    y1: float
    z1: float

    @property
    def bounds(self) -> list[float]:
        return [self.x0, self.y0, self.z0, self.x1, self.y1, self.z1]


@dataclass(frozen=True)
class ObstacleGroup:
    name: str
    carving_obstacles: tuple[AABB, ...]
    validation_obstacles: tuple[AABB, ...]


@dataclass
class Cell:
    id: int
    intervals: IntervalBox
    depth: int
    parent: int = -1
    split_dim: int = -1
    split_value: float = 0.0
    collision_mask: tuple[bool, ...] = field(default_factory=tuple)

    @property
    def volume(self) -> float:
        volume = 1.0
        for lo, hi in self.intervals:
            volume *= max(0.0, hi - lo)
        return volume


@dataclass
class SplitCandidate:
    parent: Cell
    dim: int
    children: tuple[Cell, Cell]
    count_delta: int
    volume_delta: float
    accepted: bool
    reason: str = ""


class IdAllocator:
    def __init__(self, start: int = 0) -> None:
        self.next_id = int(start)

    def alloc(self) -> int:
        value = self.next_id
        self.next_id += 1
        return value


class EnvelopeOracle:
    def __init__(self, robot: Any, args: argparse.Namespace) -> None:
        self.robot = robot
        self.args = args
        self._link_box_cache: dict[IntervalBox, list[AABB]] = {}
        self.envelope_calls = 0
        self.envelope_time_s = 0.0

    def ensure_cells(self, cells: Sequence[Cell]) -> None:
        missing = [cell for cell in cells if cell.intervals not in self._link_box_cache]
        if not missing:
            return
        t0 = time.perf_counter()
        results = lie.compute_envelope_batch(
            self.robot,
            [cell.intervals for cell in missing],
            endpoint_source=self.args.endpoint_source,
            envelope_type=self.args.envelope_type,
            n_subdivisions=self.args.n_subdivisions,
            n_samples_crit=self.args.n_samples_crit,
            endpoint_threads=self.args.endpoint_threads,
            parallel_min_combos=self.args.parallel_min_combos,
            n_threads=self.args.batch_threads,
            include_endpoint_iaabbs=False,
        )
        self.envelope_time_s += time.perf_counter() - t0
        self.envelope_calls += len(missing)
        for cell, result in zip(missing, results):
            flat = [float(value) for value in result["envelope"].get("inflated_link_iaabbs_flat", [])]
            self._link_box_cache[cell.intervals] = [
                AABB(flat[i], flat[i + 1], flat[i + 2], flat[i + 3], flat[i + 4], flat[i + 5])
                for i in range(0, len(flat), 6)
            ]

    def cell_collides_obstacles(self, cell: Cell, obstacles: Sequence[AABB]) -> bool:
        if not obstacles:
            return False
        self.ensure_cells([cell])
        return any(aabb_overlap(link_box, obstacle) for link_box in self._link_box_cache[cell.intervals] for obstacle in obstacles)

    def update_collision_masks(self, cells: Sequence[Cell], groups: Sequence[ObstacleGroup]) -> None:
        self.ensure_cells(cells)
        for cell in cells:
            link_boxes = self._link_box_cache[cell.intervals]
            mask = []
            for group in groups:
                mask.append(any(aabb_overlap(link_box, obstacle) for link_box in link_boxes for obstacle in group.carving_obstacles))
            cell.collision_mask = tuple(mask)


def make_aabb(cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> AABB:
    return AABB(cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz)


def aggregate_aabb(obstacles: Sequence[AABB], padding: float = 0.0) -> AABB:
    return AABB(
        min(obstacle.x0 for obstacle in obstacles) - padding,
        min(obstacle.y0 for obstacle in obstacles) - padding,
        min(obstacle.z0 for obstacle in obstacles) - padding,
        max(obstacle.x1 for obstacle in obstacles) + padding,
        max(obstacle.y1 for obstacle in obstacles) + padding,
        max(obstacle.z1 for obstacle in obstacles) + padding,
    )


def make_shelves_obstacles() -> list[AABB]:
    ox, oy, oz = 0.85, 0.0, 0.4
    obstacles: list[AABB] = []

    def add(lx: float, ly: float, lz: float, fx: float, fy: float, fz: float) -> None:
        obstacles.append(make_aabb(ox + lx, oy + ly, oz + lz, fx / 2.0, fy / 2.0, fz / 2.0))

    add(0.0, 0.292, 0.0, 0.3, 0.016, 0.783)
    add(0.0, -0.292, 0.0, 0.3, 0.016, 0.783)
    add(0.0, 0.0, 0.3995, 0.3, 0.6, 0.016)
    add(0.0, 0.0, -0.13115, 0.3, 0.6, 0.016)
    add(0.0, 0.0, 0.13115, 0.3, 0.6, 0.016)
    return obstacles


def make_bins_obstacles() -> list[AABB]:
    obstacles: list[AABB] = []

    def add_bin(bx: float, by: float, bz: float) -> None:
        def add(lx: float, ly: float, lz: float, fx: float, fy: float, fz: float) -> None:
            obstacles.append(make_aabb(bx - ly, by + lx, bz + lz, fy / 2.0, fx / 2.0, fz / 2.0))

        add(0.22, 0.0, 0.105, 0.05, 0.63, 0.21)
        add(-0.22, 0.0, 0.105, 0.05, 0.63, 0.21)
        add(0.0, 0.29, 0.105, 0.49, 0.05, 0.21)
        add(0.0, -0.29, 0.105, 0.49, 0.05, 0.21)
        add(0.0, 0.0, 0.0075, 0.49, 0.63, 0.015)

    add_bin(0.0, -0.6, 0.0)
    add_bin(0.0, 0.6, 0.0)
    return obstacles


def make_table_obstacles() -> list[AABB]:
    return [make_aabb(0.4, 0.0, -0.25, 2.5 / 2.0, 2.5 / 2.0, 0.2 / 2.0)]


def make_group(name: str, validation: Sequence[AABB], carving_mode: str, padding: float) -> ObstacleGroup:
    validation_tuple = tuple(validation)
    carving = (aggregate_aabb(validation_tuple, padding),) if carving_mode == "aggregate" else validation_tuple
    return ObstacleGroup(name=name, carving_obstacles=tuple(carving), validation_obstacles=validation_tuple)


def make_grouped_obstacles(carving_mode: str, padding: float) -> list[ObstacleGroup]:
    bins = make_bins_obstacles()
    return [
        make_group("shelf", make_shelves_obstacles(), carving_mode, padding),
        make_group("bin_negative_y", bins[:5], carving_mode, padding),
        make_group("bin_positive_y", bins[5:], carving_mode, padding),
        make_group("table", make_table_obstacles(), carving_mode, padding),
    ]


def aabb_overlap(lhs: AABB, rhs: AABB, tolerance: float = 0.0) -> bool:
    return (
        lhs.x0 <= rhs.x1 + tolerance and rhs.x0 <= lhs.x1 + tolerance and
        lhs.y0 <= rhs.y1 + tolerance and rhs.y0 <= lhs.y1 + tolerance and
        lhs.z0 <= rhs.z1 + tolerance and rhs.z0 <= lhs.z1 + tolerance
    )


def cell_width(cell: Cell, dim: int) -> float:
    lo, hi = cell.intervals[dim]
    return max(0.0, hi - lo)


def intervals_overlap(lhs: IntervalBox, rhs: IntervalBox, tolerance: float = 0.0) -> bool:
    return all(lo_l <= hi_r + tolerance and lo_r <= hi_l + tolerance for (lo_l, hi_l), (lo_r, hi_r) in zip(lhs, rhs))


def cell_contains_point(cell: Cell, point: Sequence[float], tolerance: float = 0.0) -> bool:
    return all(lo - tolerance <= value <= hi + tolerance for value, (lo, hi) in zip(point, cell.intervals))


def cell_from_center(center: Sequence[float], half_widths: Sequence[float], limits: IntervalBox, ids: IdAllocator, depth: int = 0, parent: int = -1) -> Cell:
    intervals = []
    for value, half_width, (lo, hi) in zip(center, half_widths, limits):
        intervals.append((max(lo, value - half_width), min(hi, value + half_width)))
    return Cell(ids.alloc(), tuple(intervals), depth, parent)


def root_intervals(robot: Any) -> IntervalBox:
    limits = robot.joint_limits().limits
    return tuple((float(interval.lo), float(interval.hi)) for interval in limits)


def split_cell(cell: Cell, dim: int, ids: IdAllocator) -> tuple[Cell, Cell]:
    lo, hi = cell.intervals[dim]
    mid = 0.5 * (lo + hi)
    left_intervals = [tuple(interval) for interval in cell.intervals]
    right_intervals = [tuple(interval) for interval in cell.intervals]
    left_intervals[dim] = (lo, mid)
    right_intervals[dim] = (mid, hi)
    return (
        Cell(ids.alloc(), tuple(left_intervals), cell.depth + 1, cell.id, dim, mid),
        Cell(ids.alloc(), tuple(right_intervals), cell.depth + 1, cell.id, dim, mid),
    )


def candidate_dims(cell: Cell, max_candidate_dims: int, min_width: float) -> list[int]:
    dims = [(dim, cell_width(cell, dim)) for dim in range(len(cell.intervals))]
    dims = [(dim, width) for dim, width in dims if width >= 2.0 * min_width]
    dims.sort(key=lambda item: item[1], reverse=True)
    if max_candidate_dims > 0:
        dims = dims[:max_candidate_dims]
    return [dim for dim, _ in dims]


def group_collision_count(cells: Sequence[Cell]) -> int:
    return sum(sum(1 for hit in cell.collision_mask if hit) for cell in cells)


def grouped_collision_volume(cells: Sequence[Cell]) -> float:
    return sum(cell.volume * sum(1 for hit in cell.collision_mask if hit) for cell in cells)


def unique_collision_count(cells: Sequence[Cell]) -> int:
    return sum(1 for cell in cells if any(cell.collision_mask))


def unique_collision_volume(cells: Sequence[Cell]) -> float:
    return sum(cell.volume for cell in cells if any(cell.collision_mask))


def evaluate_split(
    cell: Cell,
    dim: int,
    ids: IdAllocator,
    oracle: EnvelopeOracle,
    groups: Sequence[ObstacleGroup],
    min_gain_abs: float,
    min_gain_per_added_abs: float,
    allow_count_increase: bool,
    exploratory_initial_splits: bool,
    max_exploratory_count_delta: int,
) -> SplitCandidate:
    snapshot = ids.next_id
    children = split_cell(cell, dim, ids)
    oracle.update_collision_masks(children, groups)
    parent_count = group_collision_count([cell])
    child_count = group_collision_count(children)
    parent_volume = grouped_collision_volume([cell])
    child_volume = grouped_collision_volume(children)
    count_delta = child_count - parent_count
    volume_delta = parent_volume - child_volume
    accepted = False
    reason = ""
    if count_delta <= 0 and volume_delta > min_gain_abs:
        accepted = True
        reason = "count_nonincreasing_volume_gain"
    elif allow_count_increase and count_delta > 0 and volume_delta > min_gain_per_added_abs * count_delta:
        accepted = True
        reason = "volume_gain_per_added_collision"
    elif exploratory_initial_splits and child_count > 0 and (max_exploratory_count_delta < 0 or count_delta <= max_exploratory_count_delta):
        accepted = True
        reason = "count_first_exploration"
    if not accepted:
        ids.next_id = snapshot
    return SplitCandidate(cell, dim, children, count_delta, volume_delta, accepted, reason)


def candidate_key(candidate: SplitCandidate) -> tuple[int, int, float, float, int]:
    no_count_increase = 1 if candidate.count_delta <= 0 else 0
    return (
        no_count_increase,
        -candidate.count_delta,
        candidate.volume_delta,
        candidate.parent.volume,
        -candidate.parent.id,
    )


def build_obstacle_aware_initial_cells(
    oracle: EnvelopeOracle,
    intervals: IntervalBox,
    groups: Sequence[ObstacleGroup],
    args: argparse.Namespace,
) -> tuple[list[Cell], list[dict[str, Any]], IdAllocator]:
    ids = IdAllocator()
    root = Cell(ids.alloc(), intervals, 0)
    oracle.update_collision_masks([root], groups)
    leaves = [root]
    events: list[dict[str, Any]] = []
    root_volume = max(root.volume, 1e-300)
    min_gain_abs = float(args.min_volume_gain_ratio) * root_volume
    min_gain_per_added_abs = float(args.min_volume_gain_per_added_collision_ratio) * root_volume

    while len(leaves) + 1 <= args.max_initial_cells:
        best: SplitCandidate | None = None
        for leaf in leaves:
            if leaf.depth >= args.max_initial_depth or not any(leaf.collision_mask):
                continue
            for dim in candidate_dims(leaf, args.max_candidate_dims, args.min_cell_width):
                candidate = evaluate_split(
                    leaf,
                    dim,
                    ids,
                    oracle,
                    groups,
                    min_gain_abs,
                    min_gain_per_added_abs,
                    bool(args.allow_count_increase),
                    bool(args.exploratory_initial_splits),
                    int(args.max_exploratory_count_delta),
                )
                if not candidate.accepted:
                    continue
                if best is None or candidate_key(candidate) > candidate_key(best):
                    best = candidate
        if best is None:
            break
        leaves.remove(best.parent)
        leaves.extend(best.children)
        events.append({
            "parent": best.parent.id,
            "dim": best.dim,
            "split_value": best.children[0].split_value,
            "depth": best.parent.depth,
            "count_delta": best.count_delta,
            "volume_delta": best.volume_delta,
            "reason": best.reason,
            "leaf_count_after": len(leaves),
        })
    return leaves, events, ids


def all_validation_obstacles(groups: Sequence[ObstacleGroup]) -> list[AABB]:
    return [obstacle for group in groups for obstacle in group.validation_obstacles]


def expanded_cell(cell: Cell, limits: IntervalBox, dim: int, side: int, grow_factor: float, min_width: float, ids: IdAllocator) -> Cell | None:
    intervals = [tuple(interval) for interval in cell.intervals]
    lo, hi = intervals[dim]
    width = max(hi - lo, min_width)
    delta = max(width * max(0.0, grow_factor - 1.0), min_width)
    limit_lo, limit_hi = limits[dim]
    if side < 0:
        new_lo = max(limit_lo, lo - delta)
        if new_lo >= lo:
            return None
        intervals[dim] = (new_lo, hi)
    else:
        new_hi = min(limit_hi, hi + delta)
        if new_hi <= hi:
            return None
        intervals[dim] = (lo, new_hi)
    return Cell(ids.alloc(), tuple(intervals), cell.depth + 1, cell.id, dim, 0.5 * (intervals[dim][0] + intervals[dim][1]))


def grow_obstacle_free_sample_cell(
    seed: Sequence[float],
    limits: IntervalBox,
    obstacles: Sequence[AABB],
    oracle: EnvelopeOracle,
    groups: Sequence[ObstacleGroup],
    ids: IdAllocator,
    args: argparse.Namespace,
) -> Cell | None:
    root_widths = [hi - lo for lo, hi in limits]
    half_widths = [max(args.sample_box_initial_radius_ratio * width, args.min_cell_width) for width in root_widths]
    cell = cell_from_center(seed, half_widths, limits, ids)
    if oracle.cell_collides_obstacles(cell, obstacles):
        if not args.sample_keep_colliding:
            return None
        oracle.update_collision_masks([cell], groups)
        return cell

    for _ in range(max(0, int(args.sample_box_grow_steps))):
        best: Cell | None = None
        best_gain = 0.0
        dims = candidate_dims(cell, 0, args.min_cell_width)
        for dim in dims:
            for side in (-1, 1):
                candidate = expanded_cell(cell, limits, dim, side, args.sample_box_grow_factor, args.min_cell_width, ids)
                if candidate is None:
                    continue
                if oracle.cell_collides_obstacles(candidate, obstacles):
                    continue
                gain = candidate.volume - cell.volume
                if gain > best_gain:
                    best = candidate
                    best_gain = gain
        if best is None:
            break
        cell = best

    oracle.update_collision_masks([cell], groups)
    return cell


def sample_centers(limits: IntervalBox, args: argparse.Namespace) -> list[tuple[float, ...]]:
    rng = random.Random(int(args.initial_rng_seed))
    centers: list[tuple[float, ...]] = []
    midpoint = tuple(0.5 * (lo + hi) for lo, hi in limits)
    zero = tuple(min(max(0.0, lo), hi) for lo, hi in limits)
    centers.extend((midpoint, zero))
    for _ in range(max(0, int(args.initial_samples))):
        centers.append(tuple(rng.uniform(lo, hi) for lo, hi in limits))
    return centers


def build_sampled_initial_cells(
    oracle: EnvelopeOracle,
    intervals: IntervalBox,
    groups: Sequence[ObstacleGroup],
    args: argparse.Namespace,
) -> tuple[list[Cell], list[dict[str, Any]], IdAllocator]:
    ids = IdAllocator()
    obstacles = all_validation_obstacles(groups)
    cells: list[Cell] = []
    events: list[dict[str, Any]] = []
    attempts = 0
    skipped_inside_existing = 0
    skipped_overlap = 0
    skipped_colliding_seed = 0

    for center in sample_centers(intervals, args):
        if len(cells) >= args.max_initial_cells:
            break
        attempts += 1
        if any(cell_contains_point(cell, center) for cell in cells):
            skipped_inside_existing += 1
            continue
        cell = grow_obstacle_free_sample_cell(center, intervals, obstacles, oracle, groups, ids, args)
        if cell is None:
            skipped_colliding_seed += 1
            continue
        if not args.sample_allow_overlap and any(intervals_overlap(cell.intervals, existing.intervals) for existing in cells):
            skipped_overlap += 1
            continue
        cells.append(cell)
        events.append({
            "id": cell.id,
            "attempt": attempts,
            "depth": cell.depth,
            "volume": cell.volume,
            "collision_mask": list(cell.collision_mask),
            "center": list(center),
        })

    events.insert(0, {
        "attempts": attempts,
        "accepted": len(cells),
        "skipped_inside_existing": skipped_inside_existing,
        "skipped_overlap": skipped_overlap,
        "skipped_colliding_seed": skipped_colliding_seed,
    })
    return cells, events, ids


def choose_local_split(cell: Cell, ids: IdAllocator, oracle: EnvelopeOracle, obstacles: Sequence[AABB], args: argparse.Namespace) -> tuple[Cell, Cell] | None:
    best_children: tuple[Cell, Cell] | None = None
    best_key: tuple[int, float, float] | None = None
    for dim in candidate_dims(cell, args.max_candidate_dims, args.min_cell_width):
        snapshot = ids.next_id
        children = split_cell(cell, dim, ids)
        colliding = [oracle.cell_collides_obstacles(child, obstacles) for child in children]
        count = sum(1 for hit in colliding if hit)
        volume = sum(child.volume for child, hit in zip(children, colliding) if hit)
        key = (-count, -volume, cell_width(cell, dim))
        if best_key is None or key > best_key:
            best_key = key
            best_children = children
        else:
            ids.next_id = snapshot
    return best_children


def local_regrow(
    domains: Sequence[Cell],
    scene_obstacles: Sequence[AABB],
    oracle: EnvelopeOracle,
    groups: Sequence[ObstacleGroup],
    ids: IdAllocator,
    args: argparse.Namespace,
) -> tuple[list[Cell], list[Cell]]:
    accepted: list[Cell] = []
    unresolved: list[Cell] = []
    for domain in domains:
        stack = [domain]
        domain_depth_limit = domain.depth + args.local_regrow_depth
        while stack:
            cell = stack.pop()
            if not oracle.cell_collides_obstacles(cell, scene_obstacles):
                oracle.update_collision_masks([cell], groups)
                accepted.append(cell)
                continue
            if cell.depth >= domain_depth_limit:
                unresolved.append(cell)
                continue
            children = choose_local_split(cell, ids, oracle, scene_obstacles, args)
            if children is None:
                unresolved.append(cell)
                continue
            stack.extend(children)
    return accepted, unresolved


def simulate_subtractive(
    initial_cells: Sequence[Cell],
    groups: Sequence[ObstacleGroup],
    oracle: EnvelopeOracle,
    ids: IdAllocator,
    args: argparse.Namespace,
) -> tuple[list[Cell], list[dict[str, Any]], dict[str, Any]]:
    active = list(initial_cells)
    scene_obstacles: list[AABB] = []
    group_rows: list[dict[str, Any]] = []
    for group in groups:
        scene_obstacles.extend(group.carving_obstacles)
        removed: list[Cell] = []
        kept: list[Cell] = []
        for cell in active:
            if oracle.cell_collides_obstacles(cell, group.carving_obstacles):
                removed.append(cell)
            else:
                kept.append(cell)
        regrown, unresolved = local_regrow(removed, scene_obstacles, oracle, groups, ids, args)
        active = kept + regrown
        group_rows.append({
            "name": group.name,
            "removed_count": len(removed),
            "removed_volume": sum(cell.volume for cell in removed),
            "regrown_count": len(regrown),
            "regrown_volume": sum(cell.volume for cell in regrown),
            "unresolved_count": len(unresolved),
            "unresolved_volume": sum(cell.volume for cell in unresolved),
            "active_count_after": len(active),
            "active_volume_after": sum(cell.volume for cell in active),
        })

    validation_obstacles = [obstacle for group in groups for obstacle in group.validation_obstacles]
    final_pruned = [cell for cell in active if oracle.cell_collides_obstacles(cell, validation_obstacles)]
    final_cells = [cell for cell in active if cell not in final_pruned]
    final_row = {
        "validation_pruned_count": len(final_pruned),
        "validation_pruned_volume": sum(cell.volume for cell in final_pruned),
        "final_count": len(final_cells),
        "final_volume": sum(cell.volume for cell in final_cells),
    }
    return final_cells, group_rows, final_row


def cell_summary(cells: Sequence[Cell]) -> dict[str, Any]:
    volumes = sorted(cell.volume for cell in cells)
    if not volumes:
        return {"count": 0, "volume_sum": 0.0}
    mid = len(volumes) // 2
    median = volumes[mid] if len(volumes) % 2 else 0.5 * (volumes[mid - 1] + volumes[mid])
    return {
        "count": len(cells),
        "volume_sum": sum(volumes),
        "volume_min": volumes[0],
        "volume_median": median,
        "volume_max": volumes[-1],
        "unique_collision_count": unique_collision_count(cells),
        "unique_collision_volume": unique_collision_volume(cells),
        "group_collision_count": group_collision_count(cells),
        "group_collision_volume": grouped_collision_volume(cells),
    }


def cell_payload(cell: Cell) -> dict[str, Any]:
    return {
        "id": cell.id,
        "parent": cell.parent,
        "depth": cell.depth,
        "split_dim": cell.split_dim,
        "split_value": cell.split_value,
        "volume": cell.volume,
        "collision_mask": list(cell.collision_mask),
        "intervals": [[lo, hi] for lo, hi in cell.intervals],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone weak-dependency subtractive SBF prototype using only link_interval_envelope.")
    parser.add_argument("--robot-json", type=Path, default=ROOT / "python" / "sbf" / "data" / "iiwa14.json")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "lie_subtractive_weak.json")
    parser.add_argument("--endpoint-source", choices=["ifk", "crit", "analytical", "gcpc", "mc"], default="ifk")
    parser.add_argument("--envelope-type", choices=["link_iaabb", "kdop", "support_hull"], default="link_iaabb")
    parser.add_argument("--n-subdivisions", type=int, default=4)
    parser.add_argument("--n-samples-crit", type=int, default=1000)
    parser.add_argument("--endpoint-threads", type=int, default=1)
    parser.add_argument("--parallel-min-combos", type=int, default=0)
    parser.add_argument("--batch-threads", type=int, default=0)
    parser.add_argument("--carving-mode", choices=["aggregate", "exact"], default="aggregate")
    parser.add_argument("--carving-padding", type=float, default=0.0)
    parser.add_argument("--initial-strategy", choices=["split", "sample"], default="sample")
    parser.add_argument("--initial-samples", type=int, default=512)
    parser.add_argument("--initial-rng-seed", type=int, default=17)
    parser.add_argument("--max-initial-cells", type=int, default=96)
    parser.add_argument("--max-initial-depth", type=int, default=8)
    parser.add_argument("--max-candidate-dims", type=int, default=0)
    parser.add_argument("--min-cell-width", type=float, default=1e-6)
    parser.add_argument("--min-volume-gain-ratio", type=float, default=1e-4)
    parser.add_argument("--min-volume-gain-per-added-collision-ratio", type=float, default=0.02)
    parser.add_argument("--allow-count-increase", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--exploratory-initial-splits", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-exploratory-count-delta", type=int, default=8)
    parser.add_argument("--sample-box-initial-radius-ratio", type=float, default=1e-4)
    parser.add_argument("--sample-box-grow-factor", type=float, default=1.8)
    parser.add_argument("--sample-box-grow-steps", type=int, default=24)
    parser.add_argument("--sample-keep-colliding", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--sample-allow-overlap", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--local-regrow-depth", type=int, default=4)
    parser.add_argument("--initial-only", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--emit-cells", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--emit-split-events", type=int, default=64)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot = lie.Robot.from_json(str(args.robot_json))
    groups = make_grouped_obstacles(args.carving_mode, args.carving_padding)
    oracle = EnvelopeOracle(robot, args)
    t0 = time.perf_counter()
    intervals = root_intervals(robot)
    if args.initial_strategy == "split":
        initial_cells, split_events, ids = build_obstacle_aware_initial_cells(oracle, intervals, groups, args)
    else:
        initial_cells, split_events, ids = build_sampled_initial_cells(oracle, intervals, groups, args)
    initial_s = time.perf_counter() - t0

    final_cells: list[Cell] = []
    group_rows: list[dict[str, Any]] = []
    final_validation: dict[str, Any] = {}
    subtractive_s = 0.0
    if not args.initial_only:
        t1 = time.perf_counter()
        final_cells, group_rows, final_validation = simulate_subtractive(initial_cells, groups, oracle, ids, args)
        subtractive_s = time.perf_counter() - t1

    payload: dict[str, Any] = {
        "schema_version": 1,
        "experiment": "lie_subtractive_weak",
        "description": "Weak-dependency subtractive SBF prototype using only link_interval_envelope envelopes and AABB obstacles.",
        "robot_json": str(args.robot_json),
        "group_order": [group.name for group in groups],
        "carving_mode": args.carving_mode,
        "initial_objective": {
            "strategy": args.initial_strategy,
            "priority": "group collision box count first, grouped collision volume second",
            "allow_count_increase": bool(args.allow_count_increase),
            "exploratory_initial_splits": bool(args.exploratory_initial_splits),
            "max_exploratory_count_delta": int(args.max_exploratory_count_delta),
            "min_volume_gain_ratio": float(args.min_volume_gain_ratio),
            "min_volume_gain_per_added_collision_ratio": float(args.min_volume_gain_per_added_collision_ratio),
            "sample_box_initial_radius_ratio": float(args.sample_box_initial_radius_ratio),
            "sample_box_grow_factor": float(args.sample_box_grow_factor),
            "sample_box_grow_steps": int(args.sample_box_grow_steps),
            "sample_keep_colliding": bool(args.sample_keep_colliding),
            "sample_allow_overlap": bool(args.sample_allow_overlap),
        },
        "timing_s": {
            "initial_build": initial_s,
            "subtractive": subtractive_s,
            "envelope_total": oracle.envelope_time_s,
        },
        "envelope_calls": oracle.envelope_calls,
        "initial": {
            "summary": cell_summary(initial_cells),
            "split_events": split_events[:max(0, int(args.emit_split_events))],
            "split_event_count": len(split_events),
        },
        "subtractive_groups": group_rows,
        "final_validation": final_validation,
    }
    if args.emit_cells:
        payload["initial"]["cells"] = [cell_payload(cell) for cell in initial_cells]
        payload["final_cells"] = [cell_payload(cell) for cell in final_cells]

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "out_json": str(args.out_json),
        "initial": payload["initial"]["summary"],
        "final_validation": final_validation,
        "envelope_calls": oracle.envelope_calls,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())