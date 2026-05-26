#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.spatial import ConvexHull, QhullError


ROOT = Path(__file__).resolve().parents[1]
LIE_ROOT = ROOT.parent / "link_interval_envelope"


def bootstrap_imports() -> None:
    paths = [
        LIE_ROOT / "build_py310" / "python",
        ROOT / "build_py310" / "python",
        ROOT / "build" / "python",
        LIE_ROOT / "python",
        ROOT / "python",
    ]
    for path in reversed(paths):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        if path.exists():
            sys.path.insert(0, text)


bootstrap_imports()

import link_interval_envelope as lie  # noqa: E402
def make_aabb_bounds(cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> list[float]:
    return [cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz]

def make_marcucci_combined_obstacle_bounds() -> list[list[float]]:
    obstacles: list[list[float]] = []

    ox, oy, oz = 0.85, 0.0, 0.4

    def add_shelf(lx: float, ly: float, lz: float, fx: float, fy: float, fz: float) -> None:
        obstacles.append(make_aabb_bounds(ox + lx, oy + ly, oz + lz, fx / 2.0, fy / 2.0, fz / 2.0))

    add_shelf(0.0, 0.292, 0.0, 0.3, 0.016, 0.783)
    add_shelf(0.0, -0.292, 0.0, 0.3, 0.016, 0.783)
    add_shelf(0.0, 0.0, 0.3995, 0.3, 0.6, 0.016)
    add_shelf(0.0, 0.0, -0.13115, 0.3, 0.6, 0.016)
    add_shelf(0.0, 0.0, 0.13115, 0.3, 0.6, 0.016)

    def add_bin(bx: float, by: float, bz: float) -> None:
        def add(lx: float, ly: float, lz: float, fx: float, fy: float, fz: float) -> None:
            obstacles.append(make_aabb_bounds(bx - ly, by + lx, bz + lz, fy / 2.0, fx / 2.0, fz / 2.0))

        add(0.22, 0.0, 0.105, 0.05, 0.63, 0.21)
        add(-0.22, 0.0, 0.105, 0.05, 0.63, 0.21)
        add(0.0, 0.29, 0.105, 0.49, 0.05, 0.21)
        add(0.0, -0.29, 0.105, 0.49, 0.05, 0.21)
        add(0.0, 0.0, 0.0075, 0.49, 0.63, 0.015)

    add_bin(0.0, -0.6, 0.0)
    add_bin(0.0, 0.6, 0.0)

    obstacles.append(make_aabb_bounds(0.4, 0.0, -0.25, 2.5 / 2.0, 2.5 / 2.0, 0.2 / 2.0))
    return obstacles


DEFAULT_BOX_TABLE = ROOT / "outputs" / "paper" / "epiaabb_pipeline_standalone_n400_fixed_widths_endpoint_only_fixed_boxes.json"
DEFAULT_VARIANTS = "link_s4,kdop26_s4,support_hull_nokdop_s4"
D32_NODES = 10_000_000_000


def robot_json_path() -> Path:
    for candidate in [
        LIE_ROOT / "examples" / "data" / "iiwa14.json",
        ROOT / "python" / "sbf" / "data" / "iiwa14.json",
    ]:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("iiwa14.json was not found in standalone package data paths")


def sha256_json(payload: Any) -> str:
    data = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def parse_variant(text: str) -> dict[str, Any]:
    key = text.strip()
    if not key:
        raise ValueError("empty variant")
    if key.startswith("link_s"):
        n_sub = int(float(key.removeprefix("link_s")))
        return {
            "key": key,
            "label": f"Crit+LinkIAABB S={n_sub}",
            "endpoint_source": "critsample",
            "envelope_type": "link_iaabb",
            "n_subdivisions": n_sub,
            "support_hull_keep_kdop": False,
            "volume_reference_variant": None,
            "voxel_delta": None,
            "grid_pad_policy": "strict_half_diagonal",
            "custom_safety_pad": 0.0,
            "diagnostic": False,
        }
    if key.startswith("hull_d"):
        body = key.removeprefix("hull_d")
        pad_policy = "strict_half_diagonal"
        if body.endswith("_nopad"):
            body = body.removesuffix("_nopad")
            pad_policy = "no_extra_pad"
        delta = float(body)
        suffix = " no-pad" if pad_policy == "no_extra_pad" else " strict-pad"
        return {
            "key": key,
            "label": f"Crit+HullGrid d={delta:g}{suffix}",
            "endpoint_source": "critsample",
            "envelope_type": "hull_grid",
            "n_subdivisions": 1,
            "support_hull_keep_kdop": False,
            "volume_reference_variant": None,
            "voxel_delta": delta,
            "grid_pad_policy": pad_policy,
            "custom_safety_pad": 0.0,
            "diagnostic": pad_policy == "no_extra_pad",
        }
    if key.startswith("kdop26_s"):
        n_sub = int(float(key.removeprefix("kdop26_s")))
        return {
            "key": key,
            "label": f"Crit+KDOP26 S={n_sub}",
            "endpoint_source": "critsample",
            "envelope_type": "kdop26",
            "n_subdivisions": n_sub,
            "support_hull_keep_kdop": False,
            "volume_reference_variant": None,
            "voxel_delta": None,
            "grid_pad_policy": "strict_half_diagonal",
            "custom_safety_pad": 0.0,
            "diagnostic": False,
        }
    if key.startswith("support_hull_nokdop_s") or key.startswith("support_hull_plain_s"):
        prefix = "support_hull_nokdop_s" if key.startswith("support_hull_nokdop_s") else "support_hull_plain_s"
        n_sub = int(float(key.removeprefix(prefix)))
        return {
            "key": key,
            "label": f"Crit+SupportHull S={n_sub}",
            "endpoint_source": "critsample",
            "envelope_type": "support_hull",
            "n_subdivisions": n_sub,
            "support_hull_keep_kdop": False,
            "volume_reference_variant": None,
            "voxel_delta": None,
            "grid_pad_policy": "strict_half_diagonal",
            "custom_safety_pad": 0.0,
            "diagnostic": False,
        }
    if key == "ifk_link_s4":
        return {
            "key": key,
            "label": "IFK+LinkIAABB S=4",
            "endpoint_source": "ifk",
            "envelope_type": "link_iaabb",
            "n_subdivisions": 4,
            "support_hull_keep_kdop": False,
            "volume_reference_variant": None,
            "voxel_delta": None,
            "grid_pad_policy": "strict_half_diagonal",
            "custom_safety_pad": 0.0,
            "diagnostic": True,
        }
    if key == "ifk_hull_d0.04":
        return {
            "key": key,
            "label": "IFK+HullGrid d=0.04 strict-pad",
            "endpoint_source": "ifk",
            "envelope_type": "hull_grid",
            "n_subdivisions": 1,
            "support_hull_keep_kdop": False,
            "volume_reference_variant": None,
            "voxel_delta": 0.04,
            "grid_pad_policy": "strict_half_diagonal",
            "custom_safety_pad": 0.0,
            "diagnostic": True,
        }
    raise ValueError(f"unknown variant '{text}'")


def load_box_table(path: Path, max_boxes_per_width: int | None = None) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    table = json.loads(path.read_text(encoding="utf-8"))
    if "width_bins" not in table:
        raise ValueError(f"box table {path} is missing width_bins")
    boxes: list[dict[str, Any]] = []
    for width_record in table["width_bins"]:
        width = float(width_record.get("fixed_width", width_record.get("width_lo", 0.0)))
        source_boxes = list(width_record.get("boxes", []))
        if max_boxes_per_width is not None:
            source_boxes = source_boxes[:max(0, int(max_boxes_per_width))]
        for box in source_boxes:
            pairs = [[float(lo), float(hi)] for lo, hi in box["intervals"]]
            boxes.append({
                "width": width,
                "width_label": str(width_record.get("width_bin", f"{width:g}")),
                "box_id": str(box.get("box_id", f"{width:g}:{len(boxes):06d}")),
                "intervals": pairs,
            })
    return table, boxes


def stats(values: Iterable[float]) -> dict[str, float]:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return {"mean": 0.0, "median": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


def box_volume(flat: list[float], offset: int) -> float:
    box = flat[offset:offset + 6]
    if len(box) != 6:
        return 0.0
    return (
        max(0.0, box[3] - box[0])
        * max(0.0, box[4] - box[1])
        * max(0.0, box[5] - box[2])
    )


def flat_boxes(flat: list[float]) -> list[tuple[float, float, float, float, float, float]]:
    boxes: list[tuple[float, float, float, float, float, float]] = []
    for offset in range(0, len(flat), 6):
        box = flat[offset:offset + 6]
        if len(box) != 6:
            continue
        lo_x, lo_y, lo_z, hi_x, hi_y, hi_z = [float(value) for value in box]
        if hi_x > lo_x and hi_y > lo_y and hi_z > lo_z:
            boxes.append((lo_x, lo_y, lo_z, hi_x, hi_y, hi_z))
    return boxes


def union_area_yz(boxes: list[tuple[float, float, float, float, float, float]]) -> float:
    ys = sorted({box[1] for box in boxes} | {box[4] for box in boxes})
    if len(ys) < 2:
        return 0.0
    area = 0.0
    for y0, y1 in zip(ys[:-1], ys[1:]):
        dy = y1 - y0
        if dy <= 0.0:
            continue
        y_mid = 0.5 * (y0 + y1)
        intervals = sorted((box[2], box[5]) for box in boxes if box[1] <= y_mid <= box[4])
        if not intervals:
            continue
        covered = 0.0
        cur_lo, cur_hi = intervals[0]
        for lo, hi in intervals[1:]:
            if lo <= cur_hi:
                cur_hi = max(cur_hi, hi)
            else:
                covered += max(0.0, cur_hi - cur_lo)
                cur_lo, cur_hi = lo, hi
        covered += max(0.0, cur_hi - cur_lo)
        area += dy * covered
    return area


def union_volume_3d(boxes: list[tuple[float, float, float, float, float, float]]) -> float:
    xs = sorted({box[0] for box in boxes} | {box[3] for box in boxes})
    if len(xs) < 2:
        return 0.0
    volume = 0.0
    for x0, x1 in zip(xs[:-1], xs[1:]):
        dx = x1 - x0
        if dx <= 0.0:
            continue
        x_mid = 0.5 * (x0 + x1)
        active = [box for box in boxes if box[0] <= x_mid <= box[3]]
        if active:
            volume += dx * union_area_yz(active)
    return float(volume)


def link_union_volume(flat: list[float], n_subdivisions: int) -> float:
    boxes = flat_boxes(flat)
    n_sub = max(1, int(n_subdivisions))
    if not boxes:
        return 0.0
    if n_sub <= 1:
        return float(sum((box[3] - box[0]) * (box[4] - box[1]) * (box[5] - box[2]) for box in boxes))
    total = 0.0
    for offset in range(0, len(boxes), n_sub):
        total += union_volume_3d(boxes[offset:offset + n_sub])
    return float(total)


def link_additive_volume(flat: list[float]) -> float:
    return float(sum(box_volume(flat, offset) for offset in range(0, len(flat), 6)))


_KDOP_AXES: list[tuple[float, float, float]] = [
    (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    (1.0, 1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 0.0, 1.0),
    (1.0, 0.0, -1.0), (0.0, 1.0, 1.0), (0.0, 1.0, -1.0),
    (1.0, 1.0, 1.0), (1.0, 1.0, -1.0), (1.0, -1.0, 1.0), (-1.0, 1.0, 1.0),
]
_KDOP_AXES_NP: "np.ndarray | None" = None


def _kdop_axes_np() -> "np.ndarray":
    global _KDOP_AXES_NP
    if _KDOP_AXES_NP is None:
        raw = np.asarray(_KDOP_AXES, dtype=float)
        norms = np.linalg.norm(raw, axis=1, keepdims=True)
        _KDOP_AXES_NP = raw / norms
    return _KDOP_AXES_NP


def _kdop_volume_for_link(
    aabb: tuple[float, float, float, float, float, float],
    record: list[float],
    radius: float = 0.0,
    n_mc: int = 4096,
    rng: "np.random.RandomState | None" = None,
) -> float:
    """Monte Carlo estimate of the uninflated KDOP polytope volume inside the link AABB."""
    if rng is None:
        rng = np.random.RandomState(42)
    lo = np.asarray(aabb[:3], dtype=float)
    hi = np.asarray(aabb[3:], dtype=float)
    aabb_vol = float(np.prod(np.maximum(0.0, hi - lo)))
    if aabb_vol <= 0.0:
        return 0.0
    pts = lo + rng.random_sample((n_mc, 3)) * (hi - lo)
    axes = _kdop_axes_np()
    proj = pts @ axes.T  # (n_mc, 13)
    inside = np.ones(n_mc, dtype=bool)
    for k in range(13):
        lo_k = float(record[2 * k]) + radius
        hi_k = float(record[2 * k + 1]) - radius
        if hi_k < lo_k:
            return 0.0
        inside &= (proj[:, k] >= lo_k) & (proj[:, k] <= hi_k)
    return aabb_vol * float(np.sum(inside)) / n_mc


def kdop_volume(result: dict[str, Any], n_mc: int = 4096) -> float:
    """Estimate total uninflated KDOP envelope volume via Monte Carlo sampling."""
    envelope = result.get("envelope", {})
    kdop = envelope.get("kdop", {})
    intervals_flat = [float(v) for v in kdop.get("intervals_flat", [])]
    n_axes = int(kdop.get("n_axes", 0))
    if n_axes != 13 or not intervals_flat:
        return 0.0
    aabb_flat = [float(v) for v in envelope.get("link_iaabbs_flat", [])]
    n_active = len(intervals_flat) // (n_axes * 2)
    if n_active == 0 or len(aabb_flat) < n_active * 6:
        return 0.0
    n_sub = max(1, int(envelope.get("n_subdivisions", 1)))
    radii = [float(v) for v in result.get("robot", {}).get("active_link_radii", [])]
    rng = np.random.RandomState(42)
    total = 0.0
    for i in range(n_active):
        aabb = tuple(aabb_flat[i * 6: i * 6 + 6])
        record = intervals_flat[i * n_axes * 2: i * n_axes * 2 + n_axes * 2]
        link_idx = i // n_sub
        radius = radii[link_idx] if link_idx < len(radii) else 0.0
        total += _kdop_volume_for_link(aabb, record, radius=radius, n_mc=n_mc, rng=rng)
    return total


def aabb_corners(box: list[float]) -> list[list[float]]:
    return [[x, y, z]
            for x in (box[0], box[3])
            for y in (box[1], box[4])
            for z in (box[2], box[5])]


def support_hull_volume_for_record(record: list[float]) -> float:
    points = np.asarray(aabb_corners(record[:6]) + aabb_corners(record[6:12]), dtype=float)
    points = np.unique(points, axis=0)
    if points.shape[0] < 4:
        return 0.0
    try:
        return float(ConvexHull(points).volume)
    except QhullError:
        return 0.0


def support_hull_volume(result: dict[str, Any]) -> float:
    envelope = result.get("envelope", {})
    support_hulls_flat = [float(v) for v in envelope.get("support_hulls_flat", [])]
    stride = 13
    if not support_hulls_flat or len(support_hulls_flat) < stride:
        return 0.0
    cached = envelope.get("_support_hull_volume_uninflated")
    if cached is not None:
        return float(cached)
    total = 0.0
    for offset in range(0, len(support_hulls_flat), stride):
        record = support_hulls_flat[offset:offset + stride]
        if len(record) == stride:
            total += support_hull_volume_for_record(record)
    envelope["_support_hull_volume_uninflated"] = float(total)
    return float(total)


def envelope_volume(result: dict[str, Any]) -> float:
    envelope = result.get("envelope", {})
    grid = envelope.get("grid", {})
    if grid.get("has_grid"):
        return float(grid.get("occupied_volume", 0.0))
    env_type = str(envelope.get("type", "")).upper()
    if env_type in {"SUPPORTHULL", "SUPPORT_HULL"}:
        v = support_hull_volume(result)
        if v > 0.0:
            return v
    if env_type in {"KDOP", "KDOP26"}:
        v = kdop_volume(result)
        if v > 0.0:
            return v
    flat = [float(value) for value in envelope.get("link_iaabbs_flat", [])]
    return link_additive_volume(flat)


def raw_link_volume(result: dict[str, Any]) -> float:
    envelope = result.get("envelope", {})
    grid = envelope.get("grid", {})
    if grid.get("has_grid"):
        return float(grid.get("occupied_volume", 0.0))
    flat = [float(value) for value in envelope.get("link_iaabbs_flat", [])]
    return link_additive_volume(flat)


def compact_payload_bytes(result: dict[str, Any]) -> float:
    envelope = result.get("envelope", {})
    grid = envelope.get("grid", {})
    if grid.get("has_grid"):
        bricks = float(grid.get("n_bricks", 0.0))
        voxels = float(grid.get("n_occupied", 0.0))
        return float(32.0 + 4.0 * max(0.0, bricks) + max(0.0, voxels) / 16.0)
    link_iaabbs_flat = envelope.get("link_iaabbs_flat", [])
    env_type = str(envelope.get("type", "")).lower()
    if link_iaabbs_flat and env_type in {"linkiaabb", "link_iaabb"}:
        return float(len(link_iaabbs_flat) * 4)

    # KDOP payload: n_active_links * n_subdivisions * kdop_n_axes * 2 floats.
    kdop = envelope.get("kdop", {})
    kdop_intervals_flat = kdop.get("intervals_flat", [])
    if kdop_intervals_flat and str(envelope.get("type", "")).upper() in {"KDOP", "KDOP26"}:
        return float(len(kdop_intervals_flat) * 4)

    support_hulls_flat = envelope.get("support_hulls_flat", [])
    if support_hulls_flat and env_type in {"supporthull", "support_hull"}:
        return float((len(support_hulls_flat) + len(kdop_intervals_flat)) * 4)
    return 0.0


def cache_payload_label(bytes_per_node: float) -> str:
    if bytes_per_node >= 1024.0 * 1024.0:
        return f"{bytes_per_node / (1024.0 * 1024.0):.2f} MB"
    if bytes_per_node >= 1024.0:
        return f"{bytes_per_node / 1024.0:.2f} KB"
    return f"{bytes_per_node:.1f} B"


def d32_time_hours(t_eval_us: float, n_nodes: int) -> float:
    return float(t_eval_us) * float(n_nodes) / 1e6 / 3600.0


def d32_disk_gb(bytes_per_node: float, n_nodes: int) -> float:
    return float(bytes_per_node) * float(n_nodes) / 1e9


def summarize_results(
    results: list[dict[str, Any]],
    variant: dict[str, Any],
    d32_nodes: int,
    *,
    volume_results: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    volume_source = volume_results if volume_results is not None else results
    volumes = [envelope_volume(item) for item in volume_source]
    raw_volumes = [raw_link_volume(item) for item in volume_source]
    endpoint_us = [float(item.get("timing_us", {}).get("endpoint", 0.0)) for item in results]
    envelope_us = [float(item.get("timing_us", {}).get("envelope", 0.0)) for item in results]
    total_us = [float(item.get("timing_us", {}).get("total", 0.0)) for item in results]
    combo_counts = [float(item.get("diagnostics", {}).get("combo_count", 0.0)) for item in results]
    enum_threads = [float(item.get("diagnostics", {}).get("enumerate_threads", 1.0)) for item in results]
    payloads = [compact_payload_bytes(item) for item in results]
    grids = [item.get("envelope", {}).get("grid", {}) for item in results]
    voxels = [float(grid.get("n_occupied", 0.0)) for grid in grids]
    bricks = [float(grid.get("n_bricks", 0.0)) for grid in grids]
    safety_pads = [float(grid.get("safety_pad", 0.0)) for grid in grids]

    volume_stats = stats(volumes)
    raw_volume_stats = stats(raw_volumes)
    endpoint_stats = stats(endpoint_us)
    envelope_stats = stats(envelope_us)
    total_stats = stats(total_us)
    payload_stats = stats(payloads)
    voxel_stats = stats(voxels)
    brick_stats = stats(bricks)
    thread_stats = stats(enum_threads)
    combo_stats = stats(combo_counts)
    safety_pad_stats = stats(safety_pads)
    t_eval_us = envelope_stats["mean"]
    return {
        "variant": variant["key"],
        "label": variant["label"],
        "endpoint_source": variant["endpoint_source"],
        "envelope_type": variant["envelope_type"],
        "n_subdivisions": int(variant["n_subdivisions"]),
        "support_hull_keep_kdop": bool(variant.get("support_hull_keep_kdop", False)),
        "volume_reference_variant": variant.get("volume_reference_variant"),
        "voxel_delta": variant["voxel_delta"],
        "grid_pad_policy": variant["grid_pad_policy"],
        "diagnostic": bool(variant["diagnostic"]),
        "n_boxes": len(results),
        "volume_mean": volume_stats["mean"],
        "volume_median": volume_stats["median"],
        "volume_std": volume_stats["std"],
        "raw_link_volume_mean": raw_volume_stats["mean"],
        "endpoint_us_mean": endpoint_stats["mean"],
        "envelope_us_mean": envelope_stats["mean"],
        "total_us_mean": total_stats["mean"],
        "t_eval_us_mean": envelope_stats["mean"],
        "t_eval_us_median": envelope_stats["median"],
        "t_eval_us_std": envelope_stats["std"],
        "combo_count_mean": combo_stats["mean"],
        "enumerate_threads_mean": thread_stats["mean"],
        "enumerate_threads_max": thread_stats["max"],
        "voxel_count_mean": voxel_stats["mean"],
        "voxel_brick_count_mean": brick_stats["mean"],
        "grid_safety_pad_mean": safety_pad_stats["mean"],
        "compact_payload_bytes_mean": payload_stats["mean"],
        "compact_payload_label": cache_payload_label(payload_stats["mean"]),
        "d32_nodes": int(d32_nodes),
        "d32_eval_time_h": d32_time_hours(t_eval_us, d32_nodes),
        "d32_disk_gb": d32_disk_gb(payload_stats["mean"], d32_nodes),
    }


def marcucci_collision_obstacle_bounds() -> list[list[float]]:
    return make_marcucci_combined_obstacle_bounds()


def collision_mode_for_variant(variant: dict[str, Any]) -> str:
    envelope_type = str(variant.get("envelope_type", "")).lower()
    if envelope_type == "link_iaabb":
        return "aabb_only"
    if envelope_type == "kdop26":
        return "kdop_only"
    if envelope_type == "support_hull":
        return "kdop_then_support_hull" if bool(variant.get("support_hull_keep_kdop", False)) else "support_hull_only"
    return "auto"


def summarize_collision_results(
    robot_path: Path,
    results: list[dict[str, Any]],
    variant: dict[str, Any],
    obstacle_bounds: list[list[float]],
) -> dict[str, Any]:
    collision_runs: list[dict[str, Any]] = []
    for result in results:
        endpoint = (result.get("endpoint") or {}).get("endpoint_iaabbs_flat")
        if endpoint is None:
            raise ValueError("collision benchmark requires endpoint_iaabbs_flat in batch results")
        collision_runs.append(
            lie.compute_collision_from_endpoint_iaabbs(
                str(robot_path),
                endpoint,
                obstacle_bounds,
                envelope_type=variant["envelope_type"],
                n_subdivisions=int(variant["n_subdivisions"]),
                voxel_delta=float(variant["voxel_delta"] or 0.05),
                kdop_directions="dop26",
                support_hull_keep_kdop=bool(variant.get("support_hull_keep_kdop", False)),
                collision_mode=collision_mode_for_variant(variant),
                count_all_pairs=True,
                include_voxels="none",
            )
        )
    collision_us = stats(float(item.get("timing_us", {}).get("collision", 0.0)) for item in collision_runs)
    total_us = stats(float(item.get("timing_us", {}).get("total", 0.0)) for item in collision_runs)
    definitely_free = [1.0 if bool((item.get("collision") or {}).get("is_definitely_free", False)) else 0.0 for item in collision_runs]
    maybe_pairs = stats(float((item.get("collision") or {}).get("maybe_pairs", 0.0)) for item in collision_runs)
    kdop_tests = stats(float((item.get("collision") or {}).get("kdop_tests", 0.0)) for item in collision_runs)
    gjk_tests = stats(float((item.get("collision") or {}).get("gjk_tests", 0.0)) for item in collision_runs)
    return {
        "collision_mode": collision_mode_for_variant(variant),
        "collision_us_mean": collision_us["mean"],
        "collision_us_median": collision_us["median"],
        "collision_us_std": collision_us["std"],
        "collision_total_us_mean": total_us["mean"],
        "collision_total_us_median": total_us["median"],
        "collision_result_definitely_free_sr": float(sum(definitely_free) / len(definitely_free)) if definitely_free else 0.0,
        "collision_maybe_pairs_mean": maybe_pairs["mean"],
        "collision_kdop_tests_mean": kdop_tests["mean"],
        "collision_gjk_tests_mean": gjk_tests["mean"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone Exp.2 envelope-only LinkIAABB/KDOP26/SupportHull runner.")
    parser.add_argument("--boxes-json", type=Path, default=DEFAULT_BOX_TABLE)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "link_envelope_pipeline_standalone_envelope_only.json")
    parser.add_argument("--variants", default=DEFAULT_VARIANTS)
    parser.add_argument("--endpoint-threads", type=int, default=0)
    parser.add_argument("--batch-threads", type=int, default=1)
    parser.add_argument("--parallel-min-combos", type=int, default=0)
    parser.add_argument("--max-boxes-per-width", type=int, default=None)
    parser.add_argument("--include-ifk-controls", action="store_true", default=False)
    parser.add_argument("--include-collision-benchmark", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--collision-obstacle-set", choices=["marcucci_combined"], default="marcucci_combined")
    parser.add_argument("--d32-nodes", type=int, default=D32_NODES)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot_path = robot_json_path()
    robot = lie.Robot.from_json(str(robot_path))
    box_table, boxes = load_box_table(args.boxes_json, args.max_boxes_per_width)
    variants = [parse_variant(item) for item in args.variants.split(",") if item.strip()]
    if args.include_ifk_controls:
        seen = {variant["key"] for variant in variants}
        for key in ["ifk_link_s4"]:
            if key not in seen:
                variants.append(parse_variant(key))

    rows: list[dict[str, Any]] = []
    rows_by_width: list[dict[str, Any]] = []
    result_rows_by_variant: dict[str, list[dict[str, Any]]] = {}
    batch_wall_s_by_variant: dict[str, float] = {}
    interval_boxes = [box["intervals"] for box in boxes]
    width_values = sorted({float(box["width"]) for box in boxes})
    obstacle_bounds = marcucci_collision_obstacle_bounds() if args.include_collision_benchmark else []

    for variant in variants:
        t0 = time.perf_counter()
        results = lie.compute_envelope_batch(
            robot,
            interval_boxes,
            endpoint_source=variant["endpoint_source"],
            envelope_type=variant["envelope_type"],
            n_subdivisions=int(variant["n_subdivisions"]),
            voxel_delta=float(variant["voxel_delta"] or 0.05),
            grid_pad_policy=variant["grid_pad_policy"],
            custom_safety_pad=float(variant["custom_safety_pad"]),
            endpoint_threads=int(args.endpoint_threads),
            parallel_min_combos=int(args.parallel_min_combos),
            n_threads=int(args.batch_threads),
            support_hull_keep_kdop=bool(variant.get("support_hull_keep_kdop", False)),
            include_voxels="none",
            include_endpoint_iaabbs=bool(args.include_collision_benchmark),
        )
        wall_s = time.perf_counter() - t0
        result_rows_by_variant[variant["key"]] = results
        batch_wall_s_by_variant[variant["key"]] = float(wall_s)
        print(
            f"[done] {variant['key']}: boxes={len(results)}, "
            f"envelope={stats(float(item.get('timing_us', {}).get('envelope', 0.0)) for item in results)['mean']:.2f} us, "
            f"payload={cache_payload_label(stats(compact_payload_bytes(item) for item in results)['mean'])}"
        )

    for variant in variants:
        results = result_rows_by_variant[variant["key"]]
        volume_results = results
        volume_reference_variant = variant.get("volume_reference_variant")
        if volume_reference_variant is not None:
            if volume_reference_variant not in result_rows_by_variant:
                raise ValueError(
                    f"{variant['key']} requires {volume_reference_variant} to estimate SupportHull volume under the same geometry"
                )
            volume_results = result_rows_by_variant[volume_reference_variant]
        row = summarize_results(results, variant, int(args.d32_nodes), volume_results=volume_results)
        row["batch_wall_s"] = batch_wall_s_by_variant[variant["key"]]
        if args.include_collision_benchmark:
            row.update(summarize_collision_results(robot_path, results, variant, obstacle_bounds))
        rows.append(row)

        for width in width_values:
            selected = [result for result, box in zip(results, boxes) if float(box["width"]) == width]
            selected_volume = [result for result, box in zip(volume_results, boxes) if float(box["width"]) == width]
            width_row = summarize_results(selected, variant, int(args.d32_nodes), volume_results=selected_volume)
            width_row["fixed_width"] = float(width)
            width_row["width_label"] = next(box["width_label"] for box in boxes if float(box["width"]) == width)
            if args.include_collision_benchmark:
                width_row.update(summarize_collision_results(robot_path, selected, variant, obstacle_bounds))
            rows_by_width.append(width_row)

    baseline = next((row for row in rows if row["variant"] == "link_s1"), None)
    if baseline is None:
        baseline = next((row for row in rows if row["variant"] == "link_s4"), None)
    baseline_volume = float(baseline["volume_mean"]) if baseline else 0.0
    for row in rows:
        row["ratio_to_link_s1"] = float(row["volume_mean"] / baseline_volume) if baseline_volume > 0.0 else 0.0
    for row in rows_by_width:
        base_width = next(
            (candidate for candidate in rows_by_width
             if candidate["variant"] in {"link_s1", "link_s4"}
             and abs(float(candidate["fixed_width"]) - float(row["fixed_width"])) < 1e-12),
            None,
        )
        base_volume = float(base_width["volume_mean"]) if base_width else 0.0
        row["ratio_to_link_s1"] = float(row["volume_mean"] / base_volume) if base_volume > 0.0 else 0.0

    payload = {
        "experiment": "exp2_link_envelope_pipeline_envelope_only",
        "source_protocol": "standalone_link_interval_envelope_no_v6_runtime_dependency",
        "source_script": str(Path(__file__).resolve()),
        "robot": "iiwa14",
        "robot_json": str(robot_path),
        "box_table_json": str(args.boxes_json),
        "box_table_sha256": sha256_json(box_table),
        "box_protocol": box_table.get("protocol", "unknown"),
        "fixed_widths": [float(value) for value in width_values],
        "n_boxes_total": len(boxes),
        "n_boxes_per_width": int(len(boxes) / max(1, len(width_values))),
        "endpoint_threads": int(args.endpoint_threads),
        "batch_threads": int(args.batch_threads),
        "parallel_min_combos": int(args.parallel_min_combos),
        "d32_nodes": int(args.d32_nodes),
        "volume_model": "All representation volumes are uninflated record-wise envelope volumes under the same short-link split. LinkIAABB sums each retained sub-AABB volume; KDOP26 uses Monte Carlo slab-volume estimates after removing the link-radius slab pad; SupportHull sums exact Conv(proximal endpoint AABB union distal endpoint AABB) volumes per short-link record and ignores the stored link radius.",
        "d32_time_model": "envelope_only_compute_extrapolation: mean envelope_us * d32_nodes; endpoint enumeration is excluded from the representation microbenchmark",
        "d32_disk_model": "compact payload estimate after short-link pruning: LinkIAABB=6 floats per retained sub-box; KDOP26=26 floats per retained sub-box; pure SupportHull=13 support floats per retained sub-box",
        "collision_benchmark_enabled": bool(args.include_collision_benchmark),
        "collision_obstacle_set": args.collision_obstacle_set if args.include_collision_benchmark else None,
        "collision_metric": "envelope-only obstacle test using all surviving link/obstacle pairs; bottom-level collision time excludes endpoint enumeration and envelope construction",
        "collision_mode_policy": "LinkIAABB=aabb_only, KDOP26=kdop_only, pure SupportHull=support_hull_only",
        "support_hull_volume_policy": "Pure SupportHull volume is computed directly from each uninflated short-link endpoint-AABB convex hull. KDOP axes match the C++ DOP26 order, and link-radius expansion is applied only by the collision test.",
        "incremental_fk_policy": "not used in main t_eval; fixed boxes are independent, so parent-FK reuse would mix grower-local behavior into the envelope microkernel",
        "t_read_policy": "not part of Exp.2 cold t_eval; grower/cache-hit reuse is reported by Exp.3 diagnostics",
        "grid_pad_policy_main": "legacy hull-grid parser retained for archaeology; default paper variants exclude grid rows",
        "rows": rows,
        "rows_by_width": rows_by_width,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "rows": len(rows), "rows_by_width": len(rows_by_width)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())