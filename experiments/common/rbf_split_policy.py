from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

from experiments.common.rbf_defaults import CANONICAL_SYMMETRY_DESCRIPTOR
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


def serialize_depth_dimensions(depth_dimensions: Iterable[int]) -> str:
    return ",".join(str(int(dim)) for dim in depth_dimensions)


def normalized_split_schedule_kind(kind: str) -> str:
    key = str(kind or "aafk_volume_min").strip().lower().replace("-", "_")
    if key in {"support_hull", "support_hull_volume", "support_hull_volume_min", "sh", "sh_volume_min"}:
        return "support_hull_volume_min"
    if key in {"aafk", "aafk_volume", "aafk_volume_min", "endpoint_aafk", "ifk", "ifk_volume_min"}:
        return "aafk_volume_min"
    raise ValueError(f"unknown LECT split schedule kind: {kind}")


def volume_min_depth_schedule(
    robot: Any,
    root_intervals: Iterable[Any] | None,
    depth: int,
    sample_count: int,
    split_schedule_kind: str,
) -> list[int]:
    kind = normalized_split_schedule_kind(split_schedule_kind)
    schedule_fn = (
        sbf.support_hull_volume_min_depth_schedule
        if kind == "support_hull_volume_min"
        else sbf.aafk_volume_min_depth_schedule
    )
    if root_intervals is None:
        return [int(dim) for dim in schedule_fn(robot, int(depth), int(sample_count))]
    return [int(dim) for dim in schedule_fn(robot, list(root_intervals), int(depth), int(sample_count))]


def read_lect_cache_depth_dimensions(cache_path: Path | None) -> list[int]:
    if cache_path is None:
        return []
    manifest_path = Path(cache_path) / "manifest.json"
    if not manifest_path.exists():
        return []
    for line in manifest_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if not line.startswith("split_depth_dimensions="):
            continue
        raw = line.split("=", 1)[1].strip()
        return [int(token) for token in raw.split(",") if token.strip()]
    return []


def parse_int_csv(raw: Any) -> list[int]:
    return [int(token.strip()) for token in str(raw or "").split(",") if token.strip()]


def parse_float_csv(raw: Any) -> list[float]:
    return [float(token.strip()) for token in str(raw or "").split(",") if token.strip()]


def make_aafk_split_policy(
    robot: Any,
    max_depth: int,
    root_intervals: Iterable[Any] | None = None,
    *,
    force_dim0_first_two: bool = False,
    forced_tail_schedule: Iterable[int] | None = None,
    split_schedule_kind: str = "aafk_volume_min",
) -> Any:
    split_schedule_kind = normalized_split_schedule_kind(split_schedule_kind)
    tail = [int(dim) for dim in (forced_tail_schedule or [])]
    if force_dim0_first_two:
        schedule_root = list(root_intervals) if root_intervals is not None else list(
            sbf.canonical_root_intervals_for_robot(
                robot,
                True,
                CANONICAL_SYMMETRY_DESCRIPTOR,
            )
        )
        if schedule_root:
            # The first two binary splits cover the four dim0 symmetry sectors.
            # The remaining schedule should match the per-sector shelf/root
            # resolution, not the widened native dim0 hull.
            schedule_root[0] = sbf.Interval(0.0, 0.5 * math.pi)
        if not tail:
            tail_depth = max(0, int(max_depth) - 2)
            tail = volume_min_depth_schedule(robot, schedule_root, tail_depth, 8, split_schedule_kind)
        elif len(tail) + 2 < int(max_depth):
            extra_depth = int(max_depth) - 2 - len(tail)
            extra = volume_min_depth_schedule(robot, schedule_root, extra_depth, 8, split_schedule_kind)
            tail.extend(int(dim) for dim in extra)
        schedule = [0, 0] + [int(dim) for dim in tail]
    else:
        schedule = volume_min_depth_schedule(robot, root_intervals, int(max_depth), 8, split_schedule_kind)
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


def make_aafk_split_policy_from_cache_prefix(
    robot: Any,
    max_depth: int,
    cache_depth_dimensions: Iterable[int],
    root_intervals: Iterable[Any] | None = None,
) -> Any:
    """Build an active split policy whose prefix exactly matches an external LECT cache.

    Exact evidence reuse is interval-keyed, so changing the dimension schedule
    by even one level makes every lookup miss.  When a warm cache is provided,
    the active tree must therefore inherit the cache schedule prefix and only
    generate a local tail beyond the cached depth.
    """
    schedule = [int(dim) for dim in cache_depth_dimensions]
    max_depth = int(max_depth)
    if len(schedule) < max_depth:
        tail_depth = max_depth - len(schedule)
        if root_intervals is None:
            tail = list(sbf.aafk_volume_min_depth_schedule(robot, tail_depth, 8))
        else:
            tail = list(sbf.aafk_volume_min_depth_schedule(robot, list(root_intervals), tail_depth, 8))
        schedule.extend(int(dim) for dim in tail)
    schedule = schedule[:max_depth]
    if len(schedule) < max_depth:
        raise RuntimeError(f"cached AAFK schedule has {len(schedule)} entries, expected {max_depth}")
    split_policy = sbf.SplitPolicyDescriptor()
    split_policy.strategy = sbf.SplitStrategy.AAFKVolumeMin
    split_policy.min_width = 0.0
    split_policy.midpoint = True
    split_policy.deterministic_tie_break = True
    split_policy.depth_dimensions = [int(dim) for dim in schedule]
    split_policy.dimension_schedule_hash = str(sbf.stable_hash(serialize_depth_dimensions(schedule)))
    return split_policy
