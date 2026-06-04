#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import random
import statistics
import sys
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (
    REPO_ROOT,
    REPO_ROOT / "build-leaf-sweep" / "python",
    REPO_ROOT / "build" / "python",
):
    if candidate.exists() and str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, namespace_dict, run_id, write_json
from experiments.common.progress import progress

import link_interval_envelope as lie


METHODS = [
    ("IFK_AA", "ifk_aa", 0),
    ("HIFK_3", "hifk", 3),
    ("HIFK_5", "hifk", 5),
    ("CritSample", "critsample", 0),
    ("Analytical", "analytical", 0),
    ("MC", "mc", 0),
]
GAP_REFERENCE_SOURCES = ("CritSample", "Analytical", "MC")
DEFAULT_WIDTHS = "0.02,0.05,0.10,0.20,0.30,0.50"
ROBOT_PATH = REPO_ROOT / "link_interval_envelope" / "examples" / "data" / "iiwa14.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.1 certified endpoint-envelope source study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp01")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--widths", default=DEFAULT_WIDTHS)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--ref-samples", type=int, default=50000, help="MC samples at the reference width.")
    parser.add_argument("--ref-width", type=float, default=0.35)
    parser.add_argument("--min-mc-samples", type=int, default=1000)
    parser.add_argument("--max-mc-samples", type=int, default=10000000)
    parser.add_argument("--mc-density-rho", type=float, default=None, help="Samples per radian of geometric-mean joint width; defaults to ref_samples/ref_width.")
    parser.add_argument("--analytical-max-phase", type=int, default=3)
    parser.add_argument("--bypass-narrow-skip", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--seed", type=int, default=6100)
    parser.add_argument("--robot-json", type=Path, default=ROBOT_PATH)
    parser.add_argument("--endpoint-threads", type=int, default=1)
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


def sample_boxes(robot: Any, width: float, count: int, seed: int, args: argparse.Namespace) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    limits = list(robot.joint_limits().limits)
    rho = float(args.mc_density_rho) if args.mc_density_rho is not None else float(args.ref_samples) / float(args.ref_width)
    boxes: list[dict[str, Any]] = []
    for _ in range(count):
        intervals = []
        widths = []
        for limit in limits:
            lo = float(limit.lo)
            hi = float(limit.hi)
            span = min(float(width), max(0.0, hi - lo))
            if span <= 0.0:
                intervals.append(lie.Interval(lo, hi))
                widths.append(max(0.0, hi - lo))
                continue
            center = rng.uniform(lo + 0.5 * span, hi - 0.5 * span)
            intervals.append(lie.Interval(center - 0.5 * span, center + 0.5 * span))
            widths.append(span)
        geo_mean_width = math.prod(max(0.0, item) for item in widths) ** (1.0 / max(1, len(widths)))
        n_mc = int(max(
            int(args.min_mc_samples),
            min(int(args.max_mc_samples), round(rho * geo_mean_width)),
        ))
        boxes.append({
            "intervals": intervals,
            "width_geo_mean": float(geo_mean_width),
            "n_mc": n_mc,
        })
    return boxes


def endpoint_volume(endpoint_iaabbs: Iterable[float]) -> float:
    data = [float(value) for value in endpoint_iaabbs]
    total = 0.0
    for offset in range(0, len(data), 6):
        dx = max(0.0, data[offset + 3] - data[offset + 0])
        dy = max(0.0, data[offset + 4] - data[offset + 1])
        dz = max(0.0, data[offset + 5] - data[offset + 2])
        total += dx * dy * dz
    return total


def endpoint_extent(endpoint_iaabbs: Iterable[float]) -> tuple[list[float], list[float]]:
    data = [float(value) for value in endpoint_iaabbs]
    if not data:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    lo = [math.inf, math.inf, math.inf]
    hi = [-math.inf, -math.inf, -math.inf]
    for offset in range(0, len(data), 6):
        for axis in range(3):
            lo[axis] = min(lo[axis], data[offset + axis])
            hi[axis] = max(hi[axis], data[offset + 3 + axis])
    return lo, hi


def median(values: Iterable[float]) -> float:
    vals = [float(value) for value in values if math.isfinite(float(value))]
    return float(statistics.median(vals)) if vals else math.nan


def mean(values: Iterable[float]) -> float:
    vals = [float(value) for value in values if math.isfinite(float(value))]
    return float(statistics.mean(vals)) if vals else math.nan


def certified_source(label: str) -> bool:
    return label in {"IFK_AA", "HIFK_3", "HIFK_5", "Analytical"}


def run_endpoint(robot: Any, box: dict[str, Any], label: str, source: str, hifk_depth: int, args: argparse.Namespace) -> dict[str, Any]:
    cfg = lie.make_endpoint_config(
        source,
        n_samples_crit=int(box["n_mc"]),
        endpoint_threads=int(args.endpoint_threads),
        max_phase_analytical=int(args.analytical_max_phase),
        bypass_narrow_skip=bool(args.bypass_narrow_skip),
        hifk_max_depth=int(hifk_depth),
        hifk_n_threads=int(args.endpoint_threads),
    )
    result = lie.compute_endpoint_iaabb_info(
        robot,
        box["intervals"],
        endpoint_config=cfg,
        output_mode="arrays",
    )
    flat = [float(value) for value in result["endpoint_iaabbs"]]
    return {
        "source": label,
        "volume": endpoint_volume(flat),
        "time_us": float(result["endpoint_time_us"]),
        "extent": endpoint_extent(flat),
        "combo_count": float(result.get("combo_count", 0.0)),
        "enumerate_threads": float(result.get("enumerate_threads", 1.0)),
    }


def run_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    robot = lie.Robot.from_json(str(args.robot_json))
    rows: list[dict[str, Any]] = []
    for width in progress(parse_widths(args), desc="exp01 widths"):
        boxes = sample_boxes(robot, width, phase_samples(args), int(args.seed) + int(round(width * 10000)), args)
        trials: list[dict[str, Any]] = []
        for box in progress(boxes, desc=f"exp01 w={width:g}", total=len(boxes)):
            values: dict[str, dict[str, Any]] = {}
            for label, source, depth in METHODS:
                values[label] = run_endpoint(robot, box, label, source, depth, args)
            trials.append({"box": box, "values": values})
        width_rows: list[dict[str, Any]] = []
        for label, _source, _depth in METHODS:
            volumes: list[float] = []
            times: list[float] = []
            gaps: list[float] = []
            combo_counts: list[float] = []
            enum_threads: list[float] = []
            for trial in trials:
                values = trial["values"]
                current = values[label]
                reference_extents = [
                    values[source_label]["extent"]
                    for source_label in GAP_REFERENCE_SOURCES
                    if source_label in values
                ]
                volumes.append(float(current["volume"]))
                times.append(float(current["time_us"]))
                combo_counts.append(float(current.get("combo_count", 0.0)))
                enum_threads.append(float(current.get("enumerate_threads", 1.0)))
                if reference_extents:
                    ref_width = []
                    for axis in range(3):
                        ref_lo = min(extent[0][axis] for extent in reference_extents)
                        ref_hi = max(extent[1][axis] for extent in reference_extents)
                        ref_width.append(ref_hi - ref_lo)
                    cur_lo, cur_hi = current["extent"]
                    cur_width = [cur_hi[axis] - cur_lo[axis] for axis in range(3)]
                    negative = [max(0.0, ref_width[axis] - cur_width[axis]) for axis in range(3)]
                    gaps.append(max(negative))
            width_rows.append({
                "width": float(width),
                "source": label,
                "safe": certified_source(label),
                "samples": len(boxes),
                "mc_samples_median": median(box["n_mc"] for box in boxes),
                "mc_samples_min": min(box["n_mc"] for box in boxes) if boxes else math.nan,
                "mc_samples_max": max(box["n_mc"] for box in boxes) if boxes else math.nan,
                "volume_m3_median": median(volumes),
                "volume_m3_mean": mean(volumes),
                "endpoint_us_median": median(times),
                "endpoint_us_mean": mean(times),
                "max_negative_gap": -max(gaps) if gaps else math.nan,
                "max_gap_to_sampling_union": max(gaps) if gaps else math.nan,
                "combo_count_median": median(combo_counts),
                "enumerate_threads_median": median(enum_threads),
            })
        ifk_volume = next(row["volume_m3_median"] for row in width_rows if row["source"] == "IFK_AA")
        for row in width_rows:
            row["rel_volume_vs_ifk"] = float(row["volume_m3_median"]) / ifk_volume if ifk_volume > 1e-18 else math.nan
            rows.append(row)
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "width", "source", "safe", "samples",
        "mc_samples_median", "mc_samples_min", "mc_samples_max",
        "volume_m3_median", "volume_m3_mean",
        "endpoint_us_median", "endpoint_us_mean",
        "rel_volume_vs_ifk",
        "max_negative_gap", "max_gap_to_sampling_union",
        "combo_count_median", "enumerate_threads_median",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)


def tex_num(value: Any, digits: int = 3) -> str:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(x):
        return "--"
    return f"{x:.{digits}f}"


def tex_bool(value: Any) -> str:
    if isinstance(value, bool):
        return "Y" if value else "N"
    text = str(value).strip().lower()
    return "Y" if text in {"1", "true", "yes", "y"} else "N"


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Endpoint AABB source comparison at fixed joint-box widths. Cert. marks certificate-backed sources. MC uses width-density sampling, and Max neg. gap reports the worst per-axis shortfall against the CritSample/Analytical/MC sampling-union reference.}",
        r"\label{tab:tro-endpoint-envelope}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{2.0pt}",
        r"\renewcommand{\arraystretch}{0.94}",
        r"\begin{tabular}{@{}llrrrr@{}}",
        r"\toprule",
        r"Width & Source & Cert. & $V_{\mathrm{ep}}$ (m$^3$) & Time ($\mu$s) & Max neg. gap \\",
        r"\midrule",
    ]
    last_width: float | None = None
    for row in rows:
        width = float(row["width"])
        source = str(row["source"]).replace("_", r"\_")
        if last_width is not None and abs(width - last_width) > 1e-12:
            lines.append(r"\addlinespace")
        lines.append(
            f"{tex_num(width, 2)} & {source} & "
            f"{tex_bool(row.get('safe'))} & "
            f"{tex_num(row.get('volume_m3_median'), 6)} & "
            f"{tex_num(row.get('endpoint_us_median'), 1)} & "
            f"{tex_num(row.get('max_negative_gap'), 4)} \\\\"
        )
        last_width = width
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def planned_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    return [
        {
            "width": width,
            "source": label,
            "status": "planned",
            "metrics": ["certified", "endpoint_volume", "endpoint_time", "relative_volume"],
        }
        for width in parse_widths(args)
        for label, _source, _depth in METHODS
    ]


def main() -> int:
    args = parse_args()
    csv_path = args.out_dir / "endpoint_envelope_summary.csv"
    tex_path = REPO_ROOT / "paper" / "generated" / "tab_tro_endpoint_envelope.tex"
    rows = planned_rows(args) if args.dry_run else run_rows(args)
    if not args.dry_run:
        write_csv(csv_path, rows)
        write_tex(tex_path, rows)
    payload = {
        "experiment": "exp01_endpoint_envelope",
        "run_id": run_id("exp01"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "params": namespace_dict(args),
        "mc_sampling_mode": "density_by_geometric_mean_width",
        "gap_reference_sources": list(GAP_REFERENCE_SOURCES),
        "summary_csv": str(csv_path) if not args.dry_run else None,
        "table": str(tex_path) if not args.dry_run else None,
        "rows": rows,
    }
    write_json(args.out_dir / "endpoint_envelope_manifest.json", payload)
    print(f"wrote {args.out_dir / 'endpoint_envelope_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
