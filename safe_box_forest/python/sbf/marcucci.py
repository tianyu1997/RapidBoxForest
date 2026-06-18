from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from ._sbf_cpp import Obstacle, Robot


@dataclass(frozen=True)
class QueryPair:
    label: str
    start_name: str
    goal_name: str
    start: tuple[float, ...]
    goal: tuple[float, ...]


def _q(values: Iterable[float]) -> tuple[float, ...]:
    return tuple(float(v) for v in values)


ANCHORS: dict[str, tuple[float, ...]] = {
    "C": _q([0.0, 0.2, 0.0, -2.09, 0.0, -0.3, 1.5707963267948966]),
    "L": _q([0.8, 0.7, 0.0, -1.6, 0.0, 0.0, 1.5707963267948966]),
    "R": _q([-0.8, 0.7, 0.0, -1.6, 0.0, 0.0, 1.5707963267948966]),
    "AS": _q([6.42e-05, 0.4719533, -0.0001493, -0.6716735, 0.0001854, 0.4261696, 1.5706922]),
    "TS": _q([-1.55e-04, 0.3972726, 0.0002196, -1.3674756, 0.0002472, -0.1929518, 1.5704688]),
    "CS": _q([-1.76e-04, 0.6830279, 0.0002450, -1.6478229, 2.09e-05, -0.7590545, 1.5706263]),
    "LB": _q([1.3326656, 0.7865932, 0.3623384, -1.4916529, -0.3192509, 0.9217325, 1.7911904]),
    "RB": _q([-1.3324624, 0.7866478, -0.3626562, -1.4916528, 0.3195340, 0.9217833, 1.3502090]),
}


def package_data_dir() -> Path:
    return Path(__file__).resolve().parent / "data"


def iiwa14_robot_json() -> Path:
    return package_data_dir() / "iiwa14.json"


def load_iiwa14_robot() -> Robot:
    return Robot.from_json(str(iiwa14_robot_json()))


def make_aabb(cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> Obstacle:
    return Obstacle(cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz)


def make_shelves_obstacles() -> list[Obstacle]:
    ox, oy, oz = 0.85, 0.0, 0.4
    obstacles: list[Obstacle] = []

    def add(lx: float, ly: float, lz: float, fx: float, fy: float, fz: float) -> None:
        obstacles.append(make_aabb(ox + lx, oy + ly, oz + lz, fx / 2.0, fy / 2.0, fz / 2.0))

    add(0.0, 0.292, 0.0, 0.3, 0.016, 0.783)
    add(0.0, -0.292, 0.0, 0.3, 0.016, 0.783)
    add(0.0, 0.0, 0.3995, 0.3, 0.6, 0.016)
    add(0.0, 0.0, -0.13115, 0.3, 0.6, 0.016)
    add(0.0, 0.0, 0.13115, 0.3, 0.6, 0.016)
    return obstacles


def make_bins_obstacles() -> list[Obstacle]:
    obstacles: list[Obstacle] = []

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


def make_table_obstacles() -> list[Obstacle]:
    return [make_aabb(0.4, 0.0, -0.25, 2.5 / 2.0, 2.5 / 2.0, 0.2 / 2.0)]


def make_combined_obstacles() -> list[Obstacle]:
    return [*make_shelves_obstacles(), *make_bins_obstacles(), *make_table_obstacles()]


def make_combined_queries() -> list[QueryPair]:
    pairs = [("AS", "TS"), ("TS", "CS"), ("CS", "LB"), ("LB", "RB"), ("RB", "AS")]
    return [QueryPair(f"{start}->{goal}", start, goal, ANCHORS[start], ANCHORS[goal]) for start, goal in pairs]


def make_coverage_seeds(include_extra_anchors: bool = False) -> list[tuple[float, ...]]:
    names = ["AS", "TS", "CS", "LB", "RB"]
    if include_extra_anchors:
        names.extend(["C", "L", "R"])
    return [ANCHORS[name] for name in names]
