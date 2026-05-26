"""Self-contained link interval envelope package."""
from __future__ import annotations

import importlib
import json
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence

try:
    _cpp = importlib.import_module("link_interval_envelope._link_interval_envelope_cpp")
except ImportError:
    _cpp = importlib.import_module("_link_interval_envelope_cpp")

Interval = _cpp.Interval
JointLimits = _cpp.JointLimits
Robot = _cpp.Robot
EndpointSource = _cpp.EndpointSource
EnvelopeType = _cpp.EnvelopeType
EndpointSourceConfig = _cpp.EndpointSourceConfig
EnvelopeTypeConfig = _cpp.EnvelopeTypeConfig
GridConfig = _cpp.GridConfig
GcpcCache = _cpp.GcpcCache
FKState = _cpp.FKState

__version__ = "0.1.0"


def load_robot(robot: str | Path | Mapping[str, Any] | Any) -> Any:
    if isinstance(robot, (str, Path)):
        return Robot.from_json(str(robot))
    if isinstance(robot, Mapping):
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8", delete=False) as tmp:
            json.dump(robot, tmp)
            tmp_path = tmp.name
        try:
            return Robot.from_json(tmp_path)
        finally:
            Path(tmp_path).unlink(missing_ok=True)
    return robot


def make_intervals(intervals: Sequence[Sequence[float]] | Sequence[Any]) -> list[Any]:
    out: list[Any] = []
    for item in intervals:
        if hasattr(item, "lo") and hasattr(item, "hi"):
            out.append(item)
        else:
            lo, hi = item  # type: ignore[misc]
            out.append(Interval(float(lo), float(hi)))
    return out


def _normalize_hifk_depth(hifk_max_depth: int | str) -> int:
    if isinstance(hifk_max_depth, str):
        key = hifk_max_depth.strip().lower()
        if key == "auto":
            return -1
        return int(key)
    return int(hifk_max_depth)


def recommend_hifk_depth(
    robot_or_intervals: str | Path | Mapping[str, Any] | Any | Sequence[Sequence[float]] | Sequence[Any],
    intervals: Sequence[Sequence[float]] | Sequence[Any] | None = None,
    *,
    max_depth_cap: int = 5,
) -> int:
    if intervals is None:
        return int(_cpp.recommend_hifk_depth(make_intervals(robot_or_intervals), int(max_depth_cap)))
    robot_obj = load_robot(robot_or_intervals)
    return int(_cpp.recommend_hifk_depth_for_robot(robot_obj, make_intervals(intervals), int(max_depth_cap)))


def make_endpoint_config(
    source: str | Any = "ifk",
    *,
    n_samples_crit: int = 1000,
    endpoint_threads: int = 1,
    parallel_min_combos: int = 0,
    max_phase_analytical: int = 3,
    bypass_narrow_skip: bool = False,
    gcpc_match_analytical: bool = False,
    hifk_max_depth: int | str = 9,
    hifk_n_threads: int = 1,
    hifk_vol_ratio_thresh: float = 0.0,
) -> Any:
    cfg = EndpointSourceConfig()
    if isinstance(source, str):
        key = source.strip().lower().replace("-", "_")
        mapping = {
            "ifk": EndpointSource.IFK,
            "crit": EndpointSource.CritSample,
            "critsample": EndpointSource.CritSample,
            "crit_sample": EndpointSource.CritSample,
            "analytical": EndpointSource.Analytical,
            "gcpc": EndpointSource.GCPC,
            "mc": EndpointSource.MC,
            "ifk_aa": EndpointSource.IFK,
            "hifk": EndpointSource.HIFK,
            "hifk_aa": EndpointSource.HIFK,
        }
        if key not in mapping:
            raise ValueError(f"unknown endpoint source: {source!r}")
        cfg.source = mapping[key]
    else:
        cfg.source = source
    cfg.n_samples_crit = int(n_samples_crit)
    cfg.n_threads = int(endpoint_threads)
    cfg.parallel_min_combos = int(parallel_min_combos)
    cfg.max_phase_analytical = int(max_phase_analytical)
    cfg.bypass_narrow_skip = bool(bypass_narrow_skip)
    cfg.gcpc_match_analytical = bool(gcpc_match_analytical)
    cfg.hifk_max_depth = _normalize_hifk_depth(hifk_max_depth)
    cfg.hifk_n_threads = int(hifk_n_threads)
    cfg.hifk_vol_ratio_thresh = float(hifk_vol_ratio_thresh)
    return cfg


def compute_endpoint_iaabb_info(
    robot: str | Path | Mapping[str, Any] | Any,
    intervals: Sequence[Sequence[float]] | Sequence[Any],
    *,
    endpoint_config: Any | None = None,
    gcpc_cache: Any | None = None,
    output_mode: str = "full",
) -> dict[str, Any]:
    robot_obj = load_robot(robot)
    interval_objs = make_intervals(intervals)
    cfg = endpoint_config if endpoint_config is not None else make_endpoint_config()
    return _cpp.compute_endpoint_iaabb_info(
        robot_obj,
        interval_objs,
        cfg,
        gcpc_cache,
        output_mode,
    )


def make_envelope_config(
    envelope: str | Any = "link_iaabb",
    *,
    n_subdivisions: int = 1,
    voxel_delta: float = 0.05,
) -> Any:
    cfg = EnvelopeTypeConfig()
    if isinstance(envelope, str):
        key = envelope.strip().lower().replace("-", "_")
        mapping = {
            "link_iaabb": EnvelopeType.LinkIAABB,
            "linkiaabb": EnvelopeType.LinkIAABB,
            "link_iaabb_grid": EnvelopeType.LinkIAABB_Grid,
            "linkiaabb_grid": EnvelopeType.LinkIAABB_Grid,
            "hull16_grid": EnvelopeType.Hull16_Grid,
            "hull16": EnvelopeType.Hull16_Grid,
        }
        if key not in mapping:
            raise ValueError(f"unknown envelope type: {envelope!r}")
        cfg.type = mapping[key]
    else:
        cfg.type = envelope
    cfg.n_subdivisions = max(1, int(n_subdivisions))
    cfg.grid_config.voxel_delta = float(voxel_delta)
    return cfg


def _aabb_pair(flat: Sequence[float], offset: int) -> list[list[float]]:
    box = [float(v) for v in flat[offset:offset + 6]]
    return [[box[0], box[3]], [box[1], box[4]], [box[2], box[5]]]


def _group_centres(flat: Sequence[float]) -> list[list[float]]:
    return [[float(flat[i]), float(flat[i + 1]), float(flat[i + 2])] for i in range(0, len(flat), 3)]


def _flatten_numbers(values: Any) -> list[float]:
    if isinstance(values, (int, float)):
        return [float(values)]
    out: list[float] = []
    for item in values:
        out.extend(_flatten_numbers(item))
    return out


def normalize_result(
    raw: Mapping[str, Any],
    *,
    intervals: Sequence[Sequence[float]] | None = None,
    robot_path: str | None = None,
    include_endpoint_iaabbs: bool = True,
) -> dict[str, Any]:
    n_active = int(raw["n_active_links"])
    n_sub = int(raw.get("n_subdivisions", 1))
    active_map = [int(v) for v in raw.get("active_link_map", [])]
    active_radii = [float(v) for v in raw.get("active_link_radii", [])]
    link_flat = [float(v) for v in raw.get("link_iaabbs", [])]
    inflated_flat = [float(v) for v in raw.get("inflated_link_iaabbs", [])]

    links = []
    for ci in range(n_active):
        for si in range(n_sub):
            idx = ci * n_sub + si
            links.append({
                "link_idx": active_map[ci] if ci < len(active_map) else ci,
                "active_link_idx": ci,
                "subdivision": si,
                "radius": active_radii[ci] if ci < len(active_radii) else 0.0,
                "raw_aabb": _aabb_pair(link_flat, idx * 6),
                "inflated_aabb": _aabb_pair(inflated_flat, idx * 6),
            })

    grid = dict(raw.get("grid", {}))
    if "centres_flat" in grid:
        grid["centres"] = _group_centres(grid.pop("centres_flat"))

    endpoint: dict[str, Any] = {
        "source": str(raw.get("endpoint_source", raw.get("source", ""))),
        "is_safe": bool(raw.get("endpoint_is_safe", raw.get("is_safe", False))),
        "n_pruned_links": int(raw.get("n_pruned_links", 0)),
        "shape": [n_active, 2, 6],
    }
    if include_endpoint_iaabbs and "endpoint_iaabbs" in raw:
        endpoint["endpoint_iaabbs_flat"] = [float(v) for v in raw["endpoint_iaabbs"]]

    midpoint_flat = [float(v) for v in raw.get("midpoint_endpoint_positions", [])]
    midpoint_links = []
    for ci in range(n_active):
        base = ci * 2 * 3
        if base + 5 >= len(midpoint_flat):
            continue
        midpoint_links.append({
            "link_idx": active_map[ci] if ci < len(active_map) else ci,
            "proximal": midpoint_flat[base:base + 3],
            "distal": midpoint_flat[base + 3:base + 6],
        })

    return {
        "schema": "link_interval_envelope.v1",
        "robot": {
            "name": str(raw.get("robot_name", "")),
            "source_path": robot_path,
            "n_joints": int(raw.get("n_joints", 0)),
            "n_active_links": n_active,
            "active_link_map": active_map,
            "active_link_radii": active_radii,
            "midpoint_links": midpoint_links,
        },
        "intervals": [[float(a), float(b)] for a, b in intervals] if intervals is not None else None,
        "endpoint": endpoint,
        "diagnostics": {
            "combo_count": int(raw.get("combo_count", 0)),
            "enumerate_threads": int(raw.get("enumerate_threads", 1)),
            "enumerate_time_us": float(raw.get("enumerate_time_us", 0.0)),
            "changed_dim": int(raw.get("changed_dim", -1)),
            "parallel_min_combos_used": int(raw.get("parallel_min_combos_used", 0)),
            "enumerate_chunk_size": int(raw.get("enumerate_chunk_size", 0)),
            "enumerate_chunk_count": int(raw.get("enumerate_chunk_count", 0)),
            "candidate_dirty_count": int(raw.get("candidate_dirty_count", 0)),
            "predh_rebuild_count": int(raw.get("predh_rebuild_count", 0)),
            "endpoint_cache_reused": bool(raw.get("endpoint_cache_reused", False)),
        },
        "envelope": {
            "type": str(raw.get("envelope_type", "")),
            "n_subdivisions": n_sub,
            "shape": [n_active, n_sub, 6],
            "active_link_map": active_map,
            "active_link_radii": active_radii,
            "link_iaabbs_flat": link_flat,
            "inflated_link_iaabbs_flat": inflated_flat,
            "links": links,
            "grid": grid,
        },
        "timing_us": {
            "endpoint": float(raw.get("endpoint_time_us", 0.0)),
            "envelope": float(raw.get("envelope_time_us", 0.0)),
            "total": float(raw.get("total_time_us", raw.get("endpoint_time_us", 0.0) + raw.get("envelope_time_us", 0.0))),
        },
    }


def compute_envelope(
    robot: str | Path | Mapping[str, Any] | Any,
    intervals: Sequence[Sequence[float]] | Sequence[Any],
    *,
    endpoint_source: str | Any = "ifk",
    envelope_type: str | Any = "link_iaabb",
    n_subdivisions: int = 1,
    voxel_delta: float = 0.05,
    n_samples_crit: int = 1000,
    endpoint_threads: int = 1,
    parallel_min_combos: int = 0,
    max_phase_analytical: int = 3,
    bypass_narrow_skip: bool = False,
    gcpc_match_analytical: bool = False,
    hifk_max_depth: int | str = 9,
    hifk_n_threads: int = 1,
    hifk_vol_ratio_thresh: float = 0.0,
    gcpc_cache: Any | None = None,
    include_voxels: str = "none",
    include_endpoint_iaabbs: bool = True,
) -> dict[str, Any]:
    robot_obj = load_robot(robot)
    interval_objs = make_intervals(intervals)
    interval_pairs = [[float(iv.lo), float(iv.hi)] for iv in interval_objs]
    endpoint_config = make_endpoint_config(
        endpoint_source,
        n_samples_crit=n_samples_crit,
        endpoint_threads=endpoint_threads,
        parallel_min_combos=parallel_min_combos,
        max_phase_analytical=max_phase_analytical,
        bypass_narrow_skip=bypass_narrow_skip,
        gcpc_match_analytical=gcpc_match_analytical,
        hifk_max_depth=hifk_max_depth,
        hifk_n_threads=hifk_n_threads,
        hifk_vol_ratio_thresh=hifk_vol_ratio_thresh,
    )
    envelope_config = make_envelope_config(
        envelope_type,
        n_subdivisions=n_subdivisions,
        voxel_delta=voxel_delta,
    )
    raw = _cpp.compute_envelope_info(
        robot_obj,
        interval_objs,
        endpoint_config,
        envelope_config,
        gcpc_cache,
        include_voxels,
    )
    return normalize_result(
        raw,
        intervals=interval_pairs,
        robot_path=str(robot) if isinstance(robot, (str, Path)) else None,
        include_endpoint_iaabbs=include_endpoint_iaabbs,
    )


def compute_envelope_batch(
    robot: str | Path | Mapping[str, Any] | Any,
    interval_boxes: Sequence[Sequence[Sequence[float]]] | Sequence[Sequence[Any]],
    *,
    endpoint_source: str | Any = "ifk",
    envelope_type: str | Any = "link_iaabb",
    n_subdivisions: int = 1,
    voxel_delta: float = 0.05,
    n_samples_crit: int = 1000,
    endpoint_threads: int = 1,
    parallel_min_combos: int = 0,
    hifk_max_depth: int | str = 9,
    hifk_n_threads: int = 1,
    hifk_vol_ratio_thresh: float = 0.0,
    n_threads: int = 0,
    include_voxels: str = "none",
    include_endpoint_iaabbs: bool = True,
) -> list[dict[str, Any]]:
    robot_obj = load_robot(robot)
    endpoint_config = make_endpoint_config(
        endpoint_source,
        n_samples_crit=n_samples_crit,
        endpoint_threads=endpoint_threads,
        parallel_min_combos=parallel_min_combos,
        hifk_max_depth=hifk_max_depth,
        hifk_n_threads=hifk_n_threads,
        hifk_vol_ratio_thresh=hifk_vol_ratio_thresh,
    )
    envelope_config = make_envelope_config(
        envelope_type,
        n_subdivisions=n_subdivisions,
        voxel_delta=voxel_delta,
    )
    interval_objs = [make_intervals(box) for box in interval_boxes]
    interval_pairs = [
        [[float(iv.lo), float(iv.hi)] for iv in box]
        for box in interval_objs
    ]
    raw_items = _cpp.compute_envelope_batch_info(
        robot_obj,
        interval_objs,
        endpoint_config,
        envelope_config,
        int(n_threads),
        include_voxels,
    )
    robot_path = str(robot) if isinstance(robot, (str, Path)) else None
    return [
        normalize_result(
            raw,
            intervals=interval_pairs[idx],
            robot_path=robot_path,
            include_endpoint_iaabbs=include_endpoint_iaabbs,
        )
        for idx, raw in enumerate(raw_items)
    ]


def compute_from_endpoint_iaabbs(
    robot: str | Path | Mapping[str, Any] | Any,
    endpoint_iaabbs: Sequence[float],
    *,
    envelope_type: str | Any = "link_iaabb",
    n_subdivisions: int = 1,
    voxel_delta: float = 0.05,
    include_voxels: str = "none",
) -> dict[str, Any]:
    robot_obj = load_robot(robot)
    envelope_config = make_envelope_config(
        envelope_type,
        n_subdivisions=n_subdivisions,
        voxel_delta=voxel_delta,
    )
    raw = _cpp.compute_link_envelope_from_endpoints(
        robot_obj,
        _flatten_numbers(endpoint_iaabbs),
        envelope_config,
        include_voxels,
    )
    return normalize_result(raw, robot_path=str(robot) if isinstance(robot, (str, Path)) else None)


class IncrementalEnvelopeComputer:
    """Stateful envelope computer for nearby interval boxes.

    The first call builds the endpoint state. Later calls infer the changed joint
    dimension when exactly one interval changed, or accept ``changed_dim``
    explicitly. CritSample reuses critical candidates, precomputed DH matrices,
    and unchanged endpoint caches; AA-backed IFK/HIFK remain one-shot endpoint
    evaluations inside this wrapper.
    """

    def __init__(
        self,
        robot: str | Path | Mapping[str, Any] | Any,
        *,
        endpoint_source: str | Any = "ifk",
        envelope_type: str | Any = "link_iaabb",
        n_subdivisions: int = 1,
        voxel_delta: float = 0.05,
        n_samples_crit: int = 1000,
        endpoint_threads: int = 1,
        parallel_min_combos: int = 0,
        max_phase_analytical: int = 3,
        bypass_narrow_skip: bool = False,
        gcpc_match_analytical: bool = False,
        hifk_max_depth: int | str = 9,
        hifk_n_threads: int = 1,
        hifk_vol_ratio_thresh: float = 0.0,
        gcpc_cache: Any | None = None,
    ) -> None:
        self.robot = load_robot(robot)
        self.robot_path = str(robot) if isinstance(robot, (str, Path)) else None
        self._gcpc_cache = gcpc_cache
        endpoint_config = make_endpoint_config(
            endpoint_source,
            n_samples_crit=n_samples_crit,
            endpoint_threads=endpoint_threads,
            parallel_min_combos=parallel_min_combos,
            max_phase_analytical=max_phase_analytical,
            bypass_narrow_skip=bypass_narrow_skip,
            gcpc_match_analytical=gcpc_match_analytical,
            hifk_max_depth=hifk_max_depth,
            hifk_n_threads=hifk_n_threads,
            hifk_vol_ratio_thresh=hifk_vol_ratio_thresh,
        )
        if gcpc_cache is not None:
            endpoint_config.set_gcpc_cache(gcpc_cache)
        envelope_config = make_envelope_config(
            envelope_type,
            n_subdivisions=n_subdivisions,
            voxel_delta=voxel_delta,
        )
        self._context = _cpp.IncrementalEnvelopeContext(
            self.robot,
            endpoint_config,
            envelope_config,
        )

    @property
    def fk_state(self) -> Any:
        return self._context.fk_state()

    def reset(self) -> None:
        self._context.reset()

    def has_valid_fk(self) -> bool:
        return bool(self._context.has_valid_fk())

    def compute(
        self,
        intervals: Sequence[Sequence[float]] | Sequence[Any],
        *,
        changed_dim: int = -1,
        include_voxels: str = "none",
        include_endpoint_iaabbs: bool = True,
    ) -> dict[str, Any]:
        interval_objs = make_intervals(intervals)
        interval_pairs = [[float(iv.lo), float(iv.hi)] for iv in interval_objs]
        raw = self._context.compute(interval_objs, include_voxels, int(changed_dim))
        result = normalize_result(
            raw,
            intervals=interval_pairs,
            robot_path=self.robot_path,
            include_endpoint_iaabbs=include_endpoint_iaabbs,
        )
        result["incremental"] = {
            "changed_dim": int(raw.get("changed_dim", -1)),
            "used_incremental_fk": bool(raw.get("used_incremental_fk", False)),
            "used_source_incremental_state": bool(raw.get("used_source_incremental_state", False)),
            "reused_fk": bool(raw.get("reused_fk", False)),
            "reused_endpoint_cache": bool(raw.get("reused_endpoint_cache", False)),
            "fk_valid": bool(raw.get("fk_valid", False)),
        }
        return result


def write_json(result: Mapping[str, Any], path: str | Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(result, indent=2), encoding="utf-8")


__all__ = [
    "Interval",
    "JointLimits",
    "Robot",
    "EndpointSource",
    "EnvelopeType",
    "EndpointSourceConfig",
    "EnvelopeTypeConfig",
    "GridConfig",
    "GcpcCache",
    "FKState",
    "IncrementalEnvelopeComputer",
    "compute_endpoint_iaabb_info",
    "compute_envelope",
    "compute_envelope_batch",
    "compute_from_endpoint_iaabbs",
    "load_robot",
    "make_intervals",
    "make_endpoint_config",
    "make_envelope_config",
    "write_json",
]
