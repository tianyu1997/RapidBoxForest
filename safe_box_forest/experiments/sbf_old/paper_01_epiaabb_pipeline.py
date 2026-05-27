#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parents[1]


def detect_lie_root() -> Path:
    candidates = []
    env_value = None
    try:
        import os

        env_value = os.environ.get("RBF_ENVELOPE_MODULE_DIR")
    except Exception:
        env_value = None
    if env_value:
        candidates.append(Path(env_value))
    candidates.extend([
        ROOT.parent / "link_interval_envelope",
        ROOT.parent / "link-interval-envelope",
    ])
    for candidate in candidates:
        if (candidate / "CMakeLists.txt").exists():
            return candidate
    return ROOT.parent / "link-interval-envelope"


LIE_ROOT = detect_lie_root()


def bootstrap_imports() -> None:
    lie_build_paths = sorted(LIE_ROOT.glob("build*/python"))
    if lie_build_paths:
        paths: list[Path] = [
            ROOT / "python",
            ROOT / "build_py310" / "python",
            *lie_build_paths,
        ]
    else:
        paths = [
            ROOT / "python",
            ROOT / "build_py310" / "python",
            LIE_ROOT / "python",
        ]
    for path in paths:
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        if path.exists():
            sys.path.insert(0, text)


bootstrap_imports()

import link_interval_envelope as lie  # noqa: E402


DEFAULT_WIDTH_BINS = [
    ("W1_0.001_0.05", 0.001, 0.05),
    ("W2_0.05_0.1", 0.05, 0.10),
    ("W3_0.1_0.2", 0.10, 0.20),
    ("W4_0.2_0.5", 0.20, 0.50),
]
DEFAULT_FIXED_WIDTHS = [0.02, 0.05, 0.10, 0.20, 0.30, 0.50]
DEFAULT_WIDTH_LABELS = {
    "W1_0.001_0.05": "0.001-0.05",
    "W2_0.05_0.1": "0.05-0.10",
    "W3_0.1_0.2": "0.10-0.20",
    "W4_0.2_0.5": "0.20-0.50",
}
SOURCE_SPECS = {
    "IFK": {"source": "ifk"},
    "HIFK_3": {"source": "hifk", "hifk_max_depth": 3, "hifk_n_threads": 1},
    "HIFK_5": {"source": "hifk", "hifk_max_depth": 5, "hifk_n_threads": 1},
    "CritSample": {"source": "critsample"},
    "Analytical": {"source": "analytical"},
    "MC": {"source": "mc"},
}
SOURCES = ["IFK", "HIFK_3", "HIFK_5", "CritSample", "Analytical", "MC"]
GAP_REFERENCE_SOURCES = ("CritSample", "Analytical", "MC")


def format_fixed_width(width: float) -> str:
    return f"{float(width):.6g}"


def format_width_label(width_lo: float, width_hi: float, name: str = "") -> str:
    if name in DEFAULT_WIDTH_LABELS:
        return DEFAULT_WIDTH_LABELS[name]
    return f"{width_lo:.3g}-{width_hi:.3g}"


def parse_width_bins(spec: str, subdivide: int) -> list[tuple[str, float, float]]:
    bins: list[tuple[str, float, float]] = []
    if spec.strip().lower() in {"default", "v6"}:
        bins = list(DEFAULT_WIDTH_BINS)
    else:
        for index, item in enumerate(spec.split(","), start=1):
            text = item.strip()
            if not text:
                continue
            if ":" in text:
                lo_text, hi_text = text.split(":", 1)
            elif "-" in text:
                lo_text, hi_text = text.split("-", 1)
            else:
                raise ValueError(f"width bin must be lo:hi or lo-hi, got {text!r}")
            lo = float(lo_text)
            hi = float(hi_text)
            if hi <= lo:
                raise ValueError(f"width bin upper bound must be > lower bound, got {text!r}")
            bins.append((f"W{index}_{lo:g}_{hi:g}", lo, hi))
    if subdivide <= 1:
        return bins
    refined: list[tuple[str, float, float]] = []
    for name, lo, hi in bins:
        edges = np.linspace(lo, hi, subdivide + 1)
        for sub_index in range(subdivide):
            sub_lo = float(edges[sub_index])
            sub_hi = float(edges[sub_index + 1])
            refined.append((f"{name}_S{sub_index + 1}", sub_lo, sub_hi))
    return refined


def parse_fixed_widths(spec: str) -> list[tuple[str, float, float]]:
    text = spec.strip()
    if text.lower() in {"default", "paper"}:
        values = list(DEFAULT_FIXED_WIDTHS)
    else:
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if not values:
        raise ValueError("fixed width list is empty")
    widths: list[tuple[str, float, float]] = []
    for width in values:
        if width <= 0.0:
            raise ValueError(f"fixed width must be positive, got {width}")
        label = format_fixed_width(width)
        widths.append((f"FW_{label.replace('.', 'p')}", float(width), float(width)))
    return widths


def certified_source(source: str) -> bool:
    return source in {"IFK", "HIFK_3", "HIFK_5", "Analytical"}


def robot_json_path() -> Path:
    for candidate in [
        LIE_ROOT / "examples" / "data" / "iiwa14.json",
        ROOT / "python" / "sbf" / "data" / "iiwa14.json",
    ]:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("iiwa14.json was not found in standalone package data paths")


def paired_centers(robot: Any, rng: np.random.RandomState, max_width: float) -> list[float]:
    centers: list[float] = []
    for limit in robot.joint_limits().limits:
        span = float(limit.hi - limit.lo)
        margin = min(0.5 * max_width, 0.45 * span)
        lo = float(limit.lo) + margin
        hi = float(limit.hi) - margin
        if hi <= lo:
            lo = float(limit.lo)
            hi = float(limit.hi)
        centers.append(float(rng.uniform(lo, hi)))
    return centers


def intervals_from_center_widths(robot: Any, centers: list[float], widths: list[float]) -> list[Any]:
    intervals: list[Any] = []
    for center, width, limit in zip(centers, widths, robot.joint_limits().limits):
        span = float(limit.hi - limit.lo)
        target_width = min(float(width), 0.95 * span)
        lo = max(float(limit.lo), float(center) - 0.5 * target_width)
        hi = min(float(limit.hi), float(center) + 0.5 * target_width)
        if hi - lo < target_width:
            if lo <= float(limit.lo):
                hi = min(float(limit.hi), lo + target_width)
            elif hi >= float(limit.hi):
                lo = max(float(limit.lo), hi - target_width)
        intervals.append(lie.Interval(float(lo), float(hi)))
    return intervals


def interval_pairs(intervals: Iterable[Any]) -> list[list[float]]:
    return [[float(interval.lo), float(interval.hi)] for interval in intervals]


def intervals_from_pairs(pairs: Iterable[Iterable[float]]) -> list[Any]:
    return [lie.Interval(float(pair[0]), float(pair[1])) for pair in pairs]


def box_widths_from_pairs(pairs: Iterable[Iterable[float]]) -> list[float]:
    return [max(0.0, float(pair[1]) - float(pair[0])) for pair in pairs]


def random_intervals(robot: Any, rng: np.random.RandomState, width_lo: float, width_hi: float) -> list[Any]:
    intervals: list[Any] = []
    for limit in robot.joint_limits().limits:
        width = float(rng.uniform(width_lo, width_hi))
        lo_max = max(float(limit.lo), float(limit.hi) - width)
        lo = float(rng.uniform(float(limit.lo), lo_max))
        hi = min(float(limit.hi), lo + width)
        intervals.append(lie.Interval(float(lo), float(hi)))
    return intervals


def sample_widths(n_dof: int,
                  rng: np.random.RandomState,
                  width_lo: float,
                  width_hi: float,
                  mode: str) -> list[float]:
    if mode == "per-box":
        width = float(rng.uniform(width_lo, width_hi))
        return [width] * n_dof
    return [float(value) for value in rng.uniform(width_lo, width_hi, size=n_dof)]


def box_table_digest(box_table: dict[str, Any]) -> str:
    data = json.dumps(box_table, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def default_box_table_path(out_json: Path) -> Path:
    return out_json.with_name(f"{out_json.stem}_fixed_boxes.json")


def build_fixed_box_table(robot: Any,
                          width_bins: list[tuple[str, float, float]],
                          args: argparse.Namespace,
                          rho: float) -> dict[str, Any]:
    n_dof = len(robot.joint_limits().limits)
    max_width = max(width_hi for _, _, width_hi in width_bins)
    paired_rng = np.random.RandomState(int(args.base_seed))
    paired_center_table = [paired_centers(robot, paired_rng, max_width) for _ in range(int(args.n_boxes))]

    table_bins: list[dict[str, Any]] = []
    for bin_index, (bin_name, width_lo, width_hi) in enumerate(width_bins):
        is_fixed_width = bool(abs(float(width_hi) - float(width_lo)) <= 1e-15)
        width_label = format_fixed_width(width_lo) if is_fixed_width else format_width_label(width_lo, width_hi, bin_name)
        rng = np.random.RandomState(int(args.base_seed) + bin_index)
        boxes: list[dict[str, Any]] = []
        for sample_index in range(int(args.n_boxes)):
            if args.independent_bins:
                intervals = random_intervals(robot, rng, width_lo, width_hi)
            else:
                widths = sample_widths(n_dof, rng, width_lo, width_hi, args.width_sampling)
                intervals = intervals_from_center_widths(robot, paired_center_table[sample_index], widths)
            pairs = interval_pairs(intervals)
            widths_actual = box_widths_from_pairs(pairs)
            geo_mean_width = float(np.prod(widths_actual) ** (1.0 / float(n_dof)))
            n_mc = int(np.clip(round(rho * geo_mean_width), int(args.min_samples), int(args.max_samples)))
            boxes.append({
                "box_id": f"{bin_name}:{sample_index:06d}",
                "sample_index": int(sample_index),
                "intervals": pairs,
                "widths": widths_actual,
                "width_geo_mean": float(geo_mean_width),
                "n_mc": int(n_mc),
            })
        table_bins.append({
            "name": bin_name,
            "width_bin": width_label,
            "fixed_width": float(width_lo) if is_fixed_width else None,
            "width_lo": float(width_lo),
            "width_hi": float(width_hi),
            "n_boxes": int(args.n_boxes),
            "boxes": boxes,
        })

    return {
        "version": 1,
        "protocol": "fixed_precomputed_same_boxes_per_source",
        "generation": "seeded_fixed_widths" if all(abs(hi - lo) <= 1e-15 for _, lo, hi in width_bins) else "seeded_random_widths_inside_bins",
        "base_seed": int(args.base_seed),
        "n_boxes_per_bin": int(args.n_boxes),
        "width_sampling": args.width_sampling,
        "independent_bins": bool(args.independent_bins),
        "width_bins": table_bins,
    }


def load_fixed_box_table(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    table = payload.get("box_table", payload) if isinstance(payload, dict) else payload
    if not isinstance(table, dict) or "width_bins" not in table:
        raise ValueError(f"fixed box table at {path} is missing width_bins")
    return table


def write_fixed_box_table(path: Path, box_table: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(box_table, indent=2, sort_keys=True), encoding="utf-8")


def stats(values: list[float]) -> dict[str, float]:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {"mean": 0.0, "median": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


def joint_box_volume(intervals: Iterable[Any]) -> float:
    volume = 1.0
    for interval in intervals:
        volume *= max(0.0, float(interval.hi - interval.lo))
    return float(volume)


def endpoint_volume_sum(flat_iaabbs: list[float]) -> float:
    volume = 0.0
    for offset in range(0, len(flat_iaabbs), 6):
        box = flat_iaabbs[offset:offset + 6]
        if len(box) != 6:
            continue
        volume += max(0.0, box[3] - box[0]) * max(0.0, box[4] - box[1]) * max(0.0, box[5] - box[2])
    return float(volume)


def extent_from_iaabbs(flat_iaabbs: list[float]) -> tuple[np.ndarray, np.ndarray]:
    arr = np.asarray(flat_iaabbs, dtype=float)
    if arr.size == 0:
        zero = np.array([0.0, 0.0, 0.0], dtype=float)
        return zero, zero
    arr = arr.reshape((-1, 6))
    return np.min(arr[:, 0:3], axis=0), np.max(arr[:, 3:6], axis=0)


def compute_endpoint(robot: Any,
                     intervals: list[Any],
                     source: str,
                     n_mc_samples: int,
                     args: argparse.Namespace) -> tuple[float, float, tuple[np.ndarray, np.ndarray], dict[str, float]]:
    spec = dict(SOURCE_SPECS[source])
    source_name = str(spec.pop("source"))
    endpoint_config = lie.make_endpoint_config(
        source_name,
        n_samples_crit=int(n_mc_samples),
        endpoint_threads=int(args.endpoint_threads),
        max_phase_analytical=int(args.analytical_max_phase),
        bypass_narrow_skip=bool(args.bypass_narrow_skip),
        **spec,
    )
    result = lie.compute_endpoint_iaabb_info(
        robot,
        intervals,
        endpoint_config=endpoint_config,
        output_mode="arrays",
    )
    flat = [float(value) for value in result.get("endpoint_iaabbs", [])]
    instrumentation = {
        "combo_count": float(result.get("combo_count", 0.0)),
        "enumerate_threads": float(result.get("enumerate_threads", 1.0)),
        "parallel_min_combos_used": float(result.get("parallel_min_combos_used", 0.0)),
        "enumerate_chunk_count": float(result.get("enumerate_chunk_count", 0.0)),
    }
    return (
        endpoint_volume_sum(flat),
        float(result.get("endpoint_time_us", 0.0)),
        extent_from_iaabbs(flat),
        instrumentation,
    )


def compare_to_v6(payload: dict[str, Any], v6_path: Path | None) -> dict[str, Any] | None:
    if v6_path is None or not v6_path.exists():
        return None
    v6_payload = json.loads(v6_path.read_text(encoding="utf-8"))
    v6_rows = {(row.get("width_bin"), row.get("source")): row for row in v6_payload.get("rows", [])}
    rows: list[dict[str, Any]] = []
    for row in payload.get("rows", []):
        key = (row.get("width_bin"), row.get("source"))
        reference = v6_rows.get(key)
        if reference is None:
            continue
        comparison = {
            "width_bin": row["width_bin"],
            "source": row["source"],
            "volume_mean_v6": float(reference.get("volume_mean", 0.0)),
            "volume_mean_standalone": float(row.get("volume_mean", 0.0)),
            "time_us_mean_v6": float(reference.get("time_us_mean", 0.0)),
            "time_us_mean_standalone": float(row.get("time_us_mean", 0.0)),
            "time_us_median_v6": float(reference.get("time_us_median", 0.0)),
            "time_us_median_standalone": float(row.get("time_us_median", 0.0)),
            "max_negative_gap_v6": float(reference.get("max_negative_gap", 0.0)),
            "max_negative_gap_standalone": float(row.get("max_negative_gap", 0.0)),
        }
        if comparison["volume_mean_v6"] != 0.0:
            comparison["volume_ratio_standalone_over_v6"] = comparison["volume_mean_standalone"] / comparison["volume_mean_v6"]
        if comparison["time_us_mean_v6"] != 0.0:
            comparison["time_ratio_standalone_over_v6"] = comparison["time_us_mean_standalone"] / comparison["time_us_mean_v6"]
        rows.append(comparison)
    return {
        "v6_path": str(v6_path),
        "n_boxes_per_bin_v6": int(v6_payload.get("n_boxes_per_bin", 0)),
        "n_boxes_per_bin_standalone": int(payload.get("n_boxes_per_bin", 0)),
        "rows": rows,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone v6-aligned Exp.1 endpoint iAABB pipeline.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "epiaabb_pipeline_standalone.json")
    parser.add_argument("--v6-json", type=Path, default=REPO_ROOT / "cpp" / "v6" / "experiments" / "results_paper" / "epiaabb_pipeline.json")
    parser.add_argument("--fixed-widths", default="default", help="Comma-separated fixed widths; use 'none' to fall back to --width-bins.")
    parser.add_argument("--width-bins", default="v6", help="Use 'v6' or comma-separated lo:hi bins, e.g. 0.001:0.025,0.025:0.05")
    parser.add_argument("--subdivide-width-bins", type=int, default=1, help="Split each requested bin into this many equal sub-bins.")
    parser.add_argument("--n-boxes", type=int, default=400, help="Interval boxes per width bin.")
    parser.add_argument("--width-sampling", choices=["per-joint", "per-box"], default="per-joint")
    parser.add_argument("--ref-samples", type=int, default=50000, help="MC samples at reference width 0.35 rad.")
    parser.add_argument("--min-samples", type=int, default=1000)
    parser.add_argument("--max-samples", type=int, default=10000000)
    parser.add_argument("--rho", type=float, default=None)
    parser.add_argument("--base-seed", type=int, default=6100)
    parser.add_argument("--fixed-boxes-json", type=Path, default=None, help="Load a precomputed fixed interval-box table.")
    parser.add_argument("--save-fixed-boxes-json", type=Path, default=None, help="Write the fixed interval-box table; defaults to <out-json stem>_fixed_boxes.json.")
    parser.add_argument("--endpoint-threads", type=int, default=0, help="endpoint source worker threads; 0 selects automatic threading")
    parser.add_argument("--analytical-max-phase", type=int, default=3)
    parser.add_argument("--bypass-narrow-skip", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--independent-bins", action="store_true", help="Use random centers per bin instead of paired centers shared by all bins.")
    parser.add_argument("--sources", default=",".join(SOURCES))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    fixed_width_text = str(args.fixed_widths).strip()
    if fixed_width_text.lower() in {"", "none", "off", "false"}:
        width_bins = parse_width_bins(args.width_bins, max(1, int(args.subdivide_width_bins)))
        width_protocol = "width_bins"
    else:
        width_bins = parse_fixed_widths(fixed_width_text)
        width_protocol = "fixed_widths"
    sources = [item.strip() for item in args.sources.split(",") if item.strip()]
    unknown_sources = sorted(set(sources) - set(SOURCES))
    if unknown_sources:
        raise ValueError(f"unknown endpoint sources: {unknown_sources}")

    robot_path = robot_json_path()
    robot = lie.Robot.from_json(str(robot_path))
    n_dof = len(robot.joint_limits().limits)
    ref_width = 0.35
    rho = float(args.rho) if args.rho is not None else float(args.ref_samples) / ref_width
    if args.fixed_boxes_json is not None:
        box_table = load_fixed_box_table(args.fixed_boxes_json)
        box_table_source = str(args.fixed_boxes_json)
    else:
        box_table = build_fixed_box_table(robot, width_bins, args, rho)
        box_table_source = "generated"
    box_table_sha256 = box_table_digest(box_table)
    box_table_path = args.save_fixed_boxes_json or default_box_table_path(args.out_json)
    write_fixed_box_table(box_table_path, box_table)

    rows: list[dict[str, Any]] = []
    width_bin_payload: list[dict[str, Any]] = []
    for bin_record in box_table.get("width_bins", []):
        bin_name = str(bin_record["name"])
        width_label = str(bin_record["width_bin"])
        width_lo = float(bin_record["width_lo"])
        width_hi = float(bin_record["width_hi"])
        boxes = list(bin_record.get("boxes", []))
        width_bin_payload.append({
            "name": bin_name,
            "width_bin": width_label,
            "fixed_width": bin_record.get("fixed_width"),
            "width_lo": float(width_lo),
            "width_hi": float(width_hi),
            "n_boxes": int(len(boxes)),
        })

        trials: list[dict[str, Any]] = []
        mc_samples_used: list[int] = []
        width_geo_used: list[float] = []
        for box in boxes:
            pairs = [[float(lo), float(hi)] for lo, hi in box["intervals"]]
            geo_mean_width = float(box.get("width_geo_mean", np.prod(box_widths_from_pairs(pairs)) ** (1.0 / float(n_dof))))
            n_mc = int(box.get("n_mc", np.clip(round(rho * geo_mean_width), int(args.min_samples), int(args.max_samples))))
            width_geo_used.append(float(geo_mean_width))
            mc_samples_used.append(n_mc)
            trials.append({
                "box_id": str(box.get("box_id", f"{bin_name}:{len(trials):06d}")),
                "interval_pairs": pairs,
                "n_mc": n_mc,
                "values": {},
                "extents": {},
            })

        for source in sources:
            for trial in trials:
                intervals = intervals_from_pairs(trial["interval_pairs"])
                volume, time_us, extent, instrumentation = compute_endpoint(robot, intervals, source, int(trial["n_mc"]), args)
                trial["values"][source] = {"volume": volume, "time_us": time_us, **instrumentation}
                trial["extents"][source] = extent

        acc: dict[str, dict[str, list[float]]] = {
            source: {
                "volume": [],
                "time_us": [],
                "dvol_mc": [],
                "max_gap_to_sampling_union": [],
                "enumerate_threads": [],
                "combo_count": [],
            }
            for source in sources
        }
        for trial in trials:
            reference_extents = [
                trial["extents"][source][1] - trial["extents"][source][0]
                for source in GAP_REFERENCE_SOURCES
                if source in trial["extents"]
            ]
            if "MC" not in trial["values"] or not reference_extents:
                continue
            mc_volume = float(trial["values"]["MC"]["volume"])
            reference_extent = np.maximum.reduce(reference_extents)
            for source, values in trial["values"].items():
                cur_lo, cur_hi = trial["extents"][source]
                extent_gap = (cur_hi - cur_lo) - reference_extent
                negative = extent_gap[extent_gap < 0.0]
                max_gap_to_sampling_union = float(np.max(np.abs(negative))) if negative.size > 0 else 0.0
                acc[source]["volume"].append(float(values["volume"]))
                acc[source]["time_us"].append(float(values["time_us"]))
                acc[source]["dvol_mc"].append(float(values["volume"]) - mc_volume)
                acc[source]["max_gap_to_sampling_union"].append(max_gap_to_sampling_union)
                acc[source]["enumerate_threads"].append(float(values.get("enumerate_threads", 1.0)))
                acc[source]["combo_count"].append(float(values.get("combo_count", 0.0)))

        for source in sources:
            if not acc[source]["volume"]:
                continue
            volume_stats = stats(acc[source]["volume"])
            time_stats = stats(acc[source]["time_us"])
            gap_stats = stats(acc[source]["max_gap_to_sampling_union"])
            thread_stats = stats(acc[source]["enumerate_threads"])
            combo_stats = stats(acc[source]["combo_count"])
            rows.append({
                "width_bin": width_label,
                "fixed_width": bin_record.get("fixed_width"),
                "source": source,
                "volume_mean": volume_stats["mean"],
                "volume_median": volume_stats["median"],
                "volume_std": volume_stats["std"],
                "volume_min": volume_stats["min"],
                "volume_max": volume_stats["max"],
                "time_us_mean": time_stats["mean"],
                "time_us_median": time_stats["median"],
                "time_us_std": time_stats["std"],
                "time_us_min": time_stats["min"],
                "time_us_max": time_stats["max"],
                "enumerate_threads_mean": thread_stats["mean"],
                "enumerate_threads_median": thread_stats["median"],
                "enumerate_threads_max": thread_stats["max"],
                "combo_count_mean": combo_stats["mean"],
                "combo_count_max": combo_stats["max"],
                "max_gap_to_sampling_union": gap_stats["max"],
                "mean_gap_to_sampling_union": gap_stats["mean"],
                "median_gap_to_sampling_union": gap_stats["median"],
                "max_negative_gap": -gap_stats["max"],
                "max_negative_gap_reference": "per_axis_max_extent_of_CritSample_Analytical_MC",
                "gap_reference_sources": list(GAP_REFERENCE_SOURCES),
                "certified": certified_source(source),
            })

        print(
            f"[done] {bin_name} ({width_label}): boxes={len(trials)}, "
            f"width_gmean={float(np.mean(width_geo_used)):.5f}, "
            f"MC samples mean={float(np.mean(mc_samples_used)):.1f}, "
            f"min={int(np.min(mc_samples_used))}, max={int(np.max(mc_samples_used))}"
        )

    n_boxes_per_bin = int(box_table.get("n_boxes_per_bin", int(args.n_boxes)))

    payload: dict[str, Any] = {
        "experiment": "epiaabb_pipeline",
        "robot": "iiwa14",
        "robot_json": str(robot_path),
        "n_boxes_per_bin": n_boxes_per_bin,
        "mc_sampling_mode": "width_proportional",
        "mc_samples": int(args.ref_samples),
        "mc_reference_samples": int(args.ref_samples),
        "mc_reference_width": ref_width,
        "mc_min_samples": int(args.min_samples),
        "mc_max_samples": int(args.max_samples),
        "endpoint_threads": int(args.endpoint_threads),
        "analytical_max_phase": int(args.analytical_max_phase),
        "mc_density_rho": float(rho),
        "max_negative_gap_reference": "per_axis_max_extent_of_CritSample_Analytical_MC",
        "bypass_narrow_skip": bool(args.bypass_narrow_skip),
        "interval_protocol": str(box_table.get("protocol", "fixed_precomputed_same_boxes_per_source")),
        "box_table_source": box_table_source,
        "box_table_json": str(box_table_path),
        "box_table_sha256": box_table_sha256,
        "box_table_version": int(box_table.get("version", 0)),
        "box_table_generation": str(box_table.get("generation", "unknown")),
        "width_protocol": width_protocol,
        "fixed_widths": [float(lo) for _, lo, hi in width_bins if abs(hi - lo) <= 1e-15],
        "width_sampling": args.width_sampling,
        "gap_reference_sources": list(GAP_REFERENCE_SOURCES),
        "source_protocol": "standalone_link_interval_envelope_no_v6_runtime_dependency",
        "source_script": str(Path(__file__).resolve()),
        "width_bins": width_bin_payload,
        "rows": rows,
    }
    payload["comparison_to_v6"] = compare_to_v6(payload, args.v6_json)
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "rows": len(rows)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())