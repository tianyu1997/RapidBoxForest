#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import random
import statistics
import sys
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.spatial import ConvexHull, QhullError

REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (
    REPO_ROOT,
    REPO_ROOT / "build-leaf-sweep" / "python",
    REPO_ROOT / "build" / "python",
):
    if candidate.exists() and str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, namespace_dict, run_id, write_csv, write_json
from experiments.common.progress import progress

import link_interval_envelope as lie


METHODS = [
    ("LinkIAABB", "link_iaabb"),
    ("SupportHull", "support_hull"),
]
DEFAULT_WIDTHS = "0.02,0.05,0.10,0.20,0.30,0.50"
ROBOT_PATH = REPO_ROOT / "link_interval_envelope" / "examples" / "data" / "iiwa14.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.2 S=1 link-envelope representation study with IFK_AA endpoints.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp02")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--widths", default=DEFAULT_WIDTHS)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--collision-repeats", type=int, default=5)
    parser.add_argument("--seed", type=int, default=6200)
    parser.add_argument("--robot-json", type=Path, default=ROBOT_PATH)
    return parser.parse_args()


def phase_samples(args: argparse.Namespace) -> int:
    if args.phase == "smoke":
        return min(int(args.samples), 5)
    if args.phase == "pilot":
        return min(int(args.samples), 200)
    return int(args.samples)


def parse_widths(args: argparse.Namespace) -> list[float]:
    widths = [float(item) for item in str(args.widths).split(",") if item.strip()]
    return widths[:1] if args.phase == "smoke" else widths


def sample_boxes(robot: Any, width: float, count: int, seed: int) -> list[list[Any]]:
    rng = random.Random(seed)
    limits = list(robot.joint_limits().limits)
    boxes: list[list[Any]] = []
    for _ in range(count):
        intervals = []
        for limit in limits:
            lo = float(limit.lo)
            hi = float(limit.hi)
            span = min(float(width), max(0.0, hi - lo))
            if span <= 0.0:
                intervals.append(lie.Interval(lo, hi))
                continue
            center = rng.uniform(lo + 0.5 * span, hi - 0.5 * span)
            intervals.append(lie.Interval(center - 0.5 * span, center + 0.5 * span))
        boxes.append(intervals)
    return boxes


def box_volume(flat: Iterable[float]) -> float:
    data = [float(value) for value in flat]
    total = 0.0
    for offset in range(0, len(data), 6):
        dx = max(0.0, data[offset + 3] - data[offset + 0])
        dy = max(0.0, data[offset + 4] - data[offset + 1])
        dz = max(0.0, data[offset + 5] - data[offset + 2])
        total += dx * dy * dz
    return total


def make_aabb_bounds(cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> list[float]:
    return [cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz]


def marcucci_combined_obstacle_bounds() -> list[float]:
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
    return [value for box in obstacles for value in box]


def shifted_obstacle_bounds(bounds: list[float], dx: float = 20.0) -> list[float]:
    shifted = list(bounds)
    for offset in range(0, len(shifted), 6):
        shifted[offset + 0] += dx
        shifted[offset + 3] += dx
    return shifted


def center_from_aabb(box: list[float]) -> tuple[float, float, float]:
    return (
        0.5 * (float(box[0]) + float(box[3])),
        0.5 * (float(box[1]) + float(box[4])),
        0.5 * (float(box[2]) + float(box[5])),
    )


def known_colliding_obstacle_bounds(result: dict[str, Any], method: str, template_bounds: list[float]) -> list[float]:
    bounds = list(template_bounds)
    if method == "support_hull":
        hulls = [float(value) for value in result.get("support_hulls", [])]
        if len(hulls) >= 13:
            cx, cy, cz = center_from_aabb(hulls[:6])
        else:
            link_boxes = [float(value) for value in result.get("link_iaabbs", [])]
            cx, cy, cz = center_from_aabb(link_boxes[:6])
    else:
        link_boxes = [float(value) for value in result.get("link_iaabbs", [])]
        cx, cy, cz = center_from_aabb(link_boxes[:6])
    bounds[:6] = make_aabb_bounds(cx, cy, cz, 0.005, 0.005, 0.005)
    return bounds


def aabb_corners(box: list[float]) -> list[list[float]]:
    return [
        [x, y, z]
        for x in (box[0], box[3])
        for y in (box[1], box[4])
        for z in (box[2], box[5])
    ]


def support_hull_record_volume(record: list[float]) -> float:
    points = np.asarray(aabb_corners(record[:6]) + aabb_corners(record[6:12]), dtype=float)
    points = np.unique(points, axis=0)
    if points.shape[0] < 4:
        return 0.0
    try:
        return float(ConvexHull(points).volume)
    except QhullError:
        return 0.0


def support_hull_volume(flat: Iterable[float]) -> float:
    data = [float(value) for value in flat]
    stride = 13
    total = 0.0
    for offset in range(0, len(data), stride):
        record = data[offset:offset + stride]
        if len(record) == stride:
            total += support_hull_record_volume(record)
    return total


def payload_bytes(result: dict[str, Any]) -> int:
    total = 0
    for key in ("link_iaabbs", "inflated_link_iaabbs", "support_hulls"):
        value = result.get(key)
        if value is not None:
            total += len(list(value)) * 4
    return total


def median(values: Iterable[float]) -> float:
    vals = [float(value) for value in values if math.isfinite(float(value))]
    return float(statistics.median(vals)) if vals else math.nan


def mean(values: Iterable[float]) -> float:
    vals = [float(value) for value in values if math.isfinite(float(value))]
    return float(statistics.mean(vals)) if vals else math.nan


def envelope_config(method: str) -> Any:
    if method == "aabb_to_support_hull":
        cfg = lie.make_envelope_config("support_hull", n_subdivisions=1)
    else:
        cfg = lie.make_envelope_config(method, n_subdivisions=1)
    prefilter_attr = "keep_" + "k" + "dop"
    if hasattr(cfg, "support_hull_config") and hasattr(cfg.support_hull_config, prefilter_attr):
        setattr(cfg.support_hull_config, prefilter_attr, False)
    return cfg


def batch_method_results(robot: Any, boxes: list[list[Any]], method: str, endpoint_cfg: Any) -> list[dict[str, Any]]:
    return list(lie._cpp.compute_envelope_batch_info(
        robot,
        boxes,
        endpoint_cfg,
        envelope_config(method),
        1,
        "arrays",
    ))


def collision_mode_for_method(method: str) -> str:
    return "aabb_only" if method == "link_iaabb" else "support_hull_only"


def collision_records(
    robot: Any,
    boxes: list[list[Any]],
    method: str,
    endpoint_cfg: Any,
    obstacle_bounds_by_box: list[list[float]],
    *,
    repeats: int,
    count_all_pairs: bool,
) -> list[dict[str, float]]:
    cfg = envelope_config(method)
    if method == "support_hull" and hasattr(cfg, "support_hull_config"):
        cfg.support_hull_config.direct_collision = True
    mode = collision_mode_for_method(method)
    records: list[dict[str, float]] = []
    use_link_aabb_broadphase = method == "link_iaabb"
    repeat_count = max(1, int(repeats))
    for intervals, obstacle_bounds in zip(boxes, obstacle_bounds_by_box):
        for _ in range(repeat_count):
            collision = lie._cpp.compute_envelope_collision_info(
                robot,
                intervals,
                endpoint_cfg,
                cfg,
                obstacle_bounds,
                mode,
                "summary",
                use_link_aabb_broadphase,
                count_all_pairs,
            )
            records.append({
                "collision_time_us": float(collision["collision_time_us"]),
                "link_aabb_tests": float(collision.get("link_aabb_tests", 0.0)),
                "link_aabb_rejects": float(collision.get("link_aabb_rejects", 0.0)),
                "gjk_tests": float(collision.get("gjk_tests", 0.0)),
                "gjk_iterations": float(collision.get("gjk_iterations", 0.0)),
                "maybe_pairs": float(collision.get("maybe_pairs", 0.0)),
            })
    return records


def summarize_collision_records(prefix: str, records: list[dict[str, float]]) -> dict[str, Any]:
    collision_times = [float(record["collision_time_us"]) for record in records]
    gjk_tests = [float(record["gjk_tests"]) for record in records]
    gjk_iterations = [float(record["gjk_iterations"]) for record in records]
    maybe_pairs = [float(record["maybe_pairs"]) for record in records]
    stem = "collision" if prefix == "collision" else f"{prefix}_collision"
    return {
        f"{stem}_us_median": median(collision_times),
        f"{stem}_us_mean": mean(collision_times),
        f"{stem}_gjk_tests_median": median(gjk_tests),
        f"{stem}_gjk_tests_mean": mean(gjk_tests),
        f"{stem}_gjk_iterations_median": median(gjk_iterations),
        f"{stem}_gjk_iterations_mean": mean(gjk_iterations),
        f"{stem}_maybe_pairs_median": median(maybe_pairs),
        f"{stem}_maybe_pairs_mean": mean(maybe_pairs),
    }


def summarize_method(
    label: str,
    method: str,
    results: list[dict[str, Any]],
    free_collisions: list[dict[str, float]],
    colliding_collisions: list[dict[str, float]],
) -> dict[str, Any]:
    volumes: list[float] = []
    envelope_times: list[float] = []
    bytes_values: list[float] = []
    for result in results:
        envelope_us = float(result["envelope_time_us"])
        envelope_times.append(envelope_us)
        if method == "support_hull":
            volumes.append(support_hull_volume(result.get("support_hulls", [])))
        else:
            volumes.append(box_volume(result.get("link_iaabbs", [])))
        bytes_values.append(float(payload_bytes(result)))
    all_collisions = free_collisions + colliding_collisions
    return {
        "envelope": label,
        "split_count": 1,
        "endpoint": "IFK_AA",
        "volume_m3_median": median(volumes),
        "volume_m3_mean": mean(volumes),
        "envelope_us_median": median(envelope_times),
        "envelope_us_mean": mean(envelope_times),
        **summarize_collision_records("free", free_collisions),
        **summarize_collision_records("colliding", colliding_collisions),
        **summarize_collision_records("collision", all_collisions),
        "payload_bytes_median": median(bytes_values),
        "samples": len(results),
        "collision_repeats": len(all_collisions) / max(1, 2 * len(results)),
    }


def run_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    robot = lie.Robot.from_json(str(args.robot_json))
    endpoint_cfg = lie.make_endpoint_config("ifk_aa")
    base_obstacle_bounds = marcucci_combined_obstacle_bounds()
    free_obstacle_bounds = shifted_obstacle_bounds(base_obstacle_bounds)
    rows: list[dict[str, Any]] = []
    for width in progress(parse_widths(args), desc="exp02 widths"):
        boxes = sample_boxes(robot, width, phase_samples(args), int(args.seed) + int(round(width * 10000)))
        results_by_method = {
            method: batch_method_results(robot, boxes, method, endpoint_cfg)
            for _label, method in progress(METHODS, desc=f"exp02 envelope w={width:g}", total=len(METHODS))
        }
        free_obstacles_by_box = [free_obstacle_bounds for _ in boxes]
        free_collisions_by_method = {
            method: collision_records(
                robot,
                boxes,
                method,
                endpoint_cfg,
                free_obstacles_by_box,
                repeats=int(args.collision_repeats),
                count_all_pairs=True,
            )
            for _label, method in progress(METHODS, desc=f"exp02 free collision w={width:g}", total=len(METHODS))
        }
        colliding_obstacles_by_method = {
            method: [
                known_colliding_obstacle_bounds(result, method, free_obstacle_bounds)
                for result in results_by_method[method]
            ]
            for _label, method in METHODS
        }
        colliding_collisions_by_method = {
            method: collision_records(
                robot,
                boxes,
                method,
                endpoint_cfg,
                colliding_obstacles_by_method[method],
                repeats=int(args.collision_repeats),
                count_all_pairs=True,
            )
            for _label, method in progress(METHODS, desc=f"exp02 colliding collision w={width:g}", total=len(METHODS))
        }
        for label, method in METHODS:
            rows.append({
                "width": float(width),
                **summarize_method(
                    label,
                    method,
                    results_by_method[method],
                    free_collisions_by_method[method],
                    colliding_collisions_by_method[method],
                ),
            })
    return rows


CSV_FIELDS = [
    "width",
    "envelope",
    "endpoint",
    "split_count",
    "samples",
    "collision_repeats",
    "volume_m3_median",
    "volume_m3_mean",
    "envelope_us_median",
    "envelope_us_mean",
    "free_collision_us_median",
    "free_collision_us_mean",
    "colliding_collision_us_median",
    "colliding_collision_us_mean",
    "collision_us_median",
    "collision_us_mean",
    "free_collision_gjk_tests_median",
    "free_collision_gjk_tests_mean",
    "colliding_collision_gjk_tests_median",
    "colliding_collision_gjk_tests_mean",
    "collision_gjk_tests_median",
    "collision_gjk_tests_mean",
    "free_collision_gjk_iterations_median",
    "free_collision_gjk_iterations_mean",
    "colliding_collision_gjk_iterations_median",
    "colliding_collision_gjk_iterations_mean",
    "collision_gjk_iterations_median",
    "collision_gjk_iterations_mean",
    "free_collision_maybe_pairs_median",
    "free_collision_maybe_pairs_mean",
    "colliding_collision_maybe_pairs_median",
    "colliding_collision_maybe_pairs_mean",
    "collision_maybe_pairs_median",
    "collision_maybe_pairs_mean",
    "payload_bytes_median",
]


def tex_num(value: Any, digits: int = 3) -> str:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(x):
        return "--"
    return f"{x:.{digits}f}"


def envelope_label(value: Any) -> str:
    return str(value).replace("LinkIAABB", "Link AABB").replace("_", r"\_")


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Link-envelope representation comparison over fixed joint-box widths.}",
        r"\label{tab:tro-link-envelope}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{2.0pt}",
        r"\renewcommand{\arraystretch}{0.94}",
        r"\begin{tabular}{@{}llrrr@{}}",
        r"\toprule",
        r"Width & Envelope & $V$ (m$^3$) & \shortstack{Env.\\($\mu$s)} & \shortstack{Test\\($\mu$s)} \\",
        r"\midrule",
    ]
    last_width: float | None = None
    for row in rows:
        width = float(row["width"])
        if last_width is not None and abs(width - last_width) > 1e-12:
            lines.append(r"\addlinespace")
        lines.append(
            f"{tex_num(width, 2)} & {envelope_label(row['envelope'])} & "
            f"{tex_num(row.get('volume_m3_mean'), 6)} & "
            f"{tex_num(row.get('envelope_us_mean'), 1)} & "
            f"{tex_num(row.get('collision_us_mean'), 1)} \\\\"
        )
        last_width = width
    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\par\vspace{0.1ex}",
        r"{\scriptsize\emph{Notes:} Means over fixed-width boxes. Test time pools far-separated probes and local obstacle-overlap probes; build time excludes endpoint time.\par}",
        r"\par\endgroup",
        "",
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def planned_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    return [
        {
            "width": width,
            "envelope": label,
            "endpoint": "IFK_AA",
            "split_count": 1,
            "status": "planned",
            "metrics": ["envelope_time", "collision_time", "payload_bytes", "aabb_volume"],
        }
        for width in parse_widths(args)
        for label, _method in METHODS
    ]


def main() -> int:
    args = parse_args()
    csv_path = args.out_dir / "link_envelope_summary.csv"
    tex_path = REPO_ROOT / "paper" / "generated" / "tab_tro_link_envelope.tex"
    rows = planned_rows(args) if args.dry_run else run_rows(args)
    if not args.dry_run:
        write_csv(csv_path, rows, CSV_FIELDS)
        write_tex(tex_path, rows)
    payload = {
        "experiment": "exp02_link_envelope",
        "run_id": run_id("exp02"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "params": namespace_dict(args),
        "summary_csv": str(csv_path) if not args.dry_run else None,
        "table": str(tex_path) if not args.dry_run else None,
        "timing_policy": (
            "Table reports envelope materialization separately from collision testing. "
            "Collision timing is averaged over repeated calls in far-separated and local obstacle-overlap "
            "obstacle cases. The colliding case disables early exit by scanning all obstacle/link pairs. "
            "LinkIAABB uses AABB overlap tests; SupportHull uses direct GJK with AABB broadphases disabled. "
            "Median and GJK counters remain in the CSV/manifest for audit."
        ),
        "migrated_archive_features": [
            "batch envelope materialization",
            "Marcucci combined obstacle set for collision benchmark",
        ],
        "rows": rows,
    }
    write_json(args.out_dir / "link_envelope_manifest.json", payload)
    print(f"wrote {args.out_dir / 'link_envelope_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
