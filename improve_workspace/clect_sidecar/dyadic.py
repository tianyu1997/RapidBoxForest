"""Dyadic cell addressing and mixed-depth grid predicates.

This is the sidecar implementation of the sparse dyadic address overlay from
`docs/improve.md`.  A cell is represented by per-dimension dyadic levels and
indices, so deep local refinement does not require materializing a heap-style
binary path.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import floor
from typing import Iterable, Sequence


@dataclass(frozen=True, order=True)
class Interval:
    lo: float
    hi: float

    def __post_init__(self) -> None:
        if self.hi < self.lo:
            raise ValueError(f"invalid interval [{self.lo}, {self.hi}]")

    @property
    def width(self) -> float:
        return self.hi - self.lo

    @property
    def center(self) -> float:
        return 0.5 * (self.lo + self.hi)

    def contains(self, value: float, tol: float = 0.0) -> bool:
        return self.lo - tol <= value <= self.hi + tol

    def overlaps_or_touches(self, other: "Interval", tol: float = 0.0) -> bool:
        return self.lo <= other.hi + tol and other.lo <= self.hi + tol

    def strictly_overlaps(self, other: "Interval", tol: float = 0.0) -> bool:
        return self.lo < other.hi - tol and other.lo < self.hi - tol

    def is_subset_of(self, other: "Interval", tol: float = 0.0) -> bool:
        return self.lo >= other.lo - tol and self.hi <= other.hi + tol


JointBox = tuple[Interval, ...]


def box_volume(box: JointBox) -> float:
    volume = 1.0
    for interval in box:
        volume *= max(0.0, interval.width)
    return volume


def box_center(box: JointBox) -> tuple[float, ...]:
    return tuple(interval.center for interval in box)


def boxes_touch_or_overlap(a: JointBox, b: JointBox, tol: float = 1e-12) -> bool:
    if len(a) != len(b):
        return False
    return all(x.overlaps_or_touches(y, tol) for x, y in zip(a, b))


def box_contains_point(box: JointBox, point: Sequence[float], tol: float = 0.0) -> bool:
    return len(box) == len(point) and all(i.contains(float(v), tol) for i, v in zip(box, point))


def box_is_subset(a: JointBox, b: JointBox, tol: float = 0.0) -> bool:
    return len(a) == len(b) and all(x.is_subset_of(y, tol) for x, y in zip(a, b))


@dataclass(frozen=True)
class DyadicAddress:
    """Per-dimension dyadic address.

    Dimension j covers the integer grid range [index_j, index_j + 1) at level
    level_j.  Splitting dimension k increments only level_k.
    """

    levels: tuple[int, ...]
    indices: tuple[int, ...]

    def __post_init__(self) -> None:
        if len(self.levels) != len(self.indices):
            raise ValueError("levels and indices have different dimensions")
        for level, index in zip(self.levels, self.indices):
            if level < 0:
                raise ValueError("negative dyadic level")
            if index < 0 or index >= (1 << level):
                raise ValueError(f"index {index} outside level {level}")

    @classmethod
    def root(cls, dims: int) -> "DyadicAddress":
        return cls((0,) * dims, (0,) * dims)

    @property
    def dims(self) -> int:
        return len(self.levels)

    @property
    def depth(self) -> int:
        return sum(self.levels)

    @property
    def max_level(self) -> int:
        return max(self.levels) if self.levels else 0

    def split(self, dim: int) -> tuple["DyadicAddress", "DyadicAddress"]:
        if dim < 0 or dim >= self.dims:
            raise IndexError(dim)
        levels = list(self.levels)
        left_indices = list(self.indices)
        right_indices = list(self.indices)
        levels[dim] += 1
        left_indices[dim] *= 2
        right_indices[dim] = right_indices[dim] * 2 + 1
        return (
            DyadicAddress(tuple(levels), tuple(left_indices)),
            DyadicAddress(tuple(levels), tuple(right_indices)),
        )

    def interval_box(self, root: JointBox) -> JointBox:
        if len(root) != self.dims:
            raise ValueError("root dimensionality mismatch")
        out: list[Interval] = []
        for interval, level, index in zip(root, self.levels, self.indices):
            scale = 1 << level
            width = interval.width / scale
            lo = interval.lo + index * width
            out.append(Interval(lo, lo + width))
        return tuple(out)

    def contains_point(self, root: JointBox, point: Sequence[float], tol: float = 0.0) -> bool:
        return box_contains_point(self.interval_box(root), point, tol)

    def is_ancestor_of(self, other: "DyadicAddress") -> bool:
        if self.dims != other.dims:
            return False
        for level, index, other_level, other_index in zip(
            self.levels, self.indices, other.levels, other.indices
        ):
            if level > other_level:
                return False
            shift = other_level - level
            if other_index >> shift != index:
                return False
        return True

    def lca(self, other: "DyadicAddress") -> "DyadicAddress":
        if self.dims != other.dims:
            raise ValueError("dimension mismatch")
        levels: list[int] = []
        indices: list[int] = []
        for a_level, a_index, b_level, b_index in zip(
            self.levels, self.indices, other.levels, other.indices
        ):
            level = min(a_level, b_level)
            a = a_index >> (a_level - level)
            b = b_index >> (b_level - level)
            while level > 0 and a != b:
                a >>= 1
                b >>= 1
                level -= 1
            levels.append(level)
            indices.append(a if a == b else 0)
        return DyadicAddress(tuple(levels), tuple(indices))


def jump_cell_containing(root: JointBox, point: Sequence[float], levels: Sequence[int]) -> DyadicAddress:
    """Return the dyadic cell at `levels` that contains `point`.

    Values on the upper boundary are clamped into the last cell, matching the
    plan's `floor(...), clamp` rule.
    """

    if len(root) != len(point) or len(root) != len(levels):
        raise ValueError("dimension mismatch")
    indices: list[int] = []
    for interval, value, level in zip(root, point, levels):
        if level < 0:
            raise ValueError("negative level")
        scale = 1 << level
        if interval.width <= 0.0:
            raise ValueError("zero-width root interval")
        t = (float(value) - interval.lo) / interval.width
        index = int(floor(t * scale))
        indices.append(max(0, min(scale - 1, index)))
    return DyadicAddress(tuple(int(x) for x in levels), tuple(indices))


def split_schedule_cells(root_dims: int, depth: int, schedule: Iterable[int] | None = None) -> list[DyadicAddress]:
    """Generate depth-synchronous start cells as dyadic addresses."""

    if depth < 0:
        raise ValueError("negative depth")
    cells = [DyadicAddress.root(root_dims)]
    if schedule is None:
        schedule_values = [i % root_dims for i in range(depth)]
    else:
        schedule_values = list(schedule)
        if len(schedule_values) < depth:
            raise ValueError("schedule shorter than depth")
    for dim in schedule_values[:depth]:
        next_cells: list[DyadicAddress] = []
        for cell in cells:
            next_cells.extend(cell.split(dim))
        cells = next_cells
    return cells

