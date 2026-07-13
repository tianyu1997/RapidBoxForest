#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import re
import math
import os
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata, write_json  # noqa: E402
from experiments.common.iris_gcs_dispatch import (  # noqa: E402
    shelf_iris_json_to_run_rows,
    shelf_iris_summary_rows,
)
from experiments.common.metrics import mean, median  # noqa: E402
from experiments.common.rbf_defaults import (  # noqa: E402
    DEFAULT_RBF_SHELF_BOX_BUDGET,
    EXP06_REGISTERED_RBF_PROFILE_NAME,
)


REQUIRED_TABLES = {
    "tab_tro_endpoint_envelope.tex": "Endpoint-envelope source comparison over fixed joint-interval side lengths.",
    "tab_tro_link_envelope.tex": "Link-envelope representation comparison over fixed joint-interval side lengths.",
    "tab_tro_shelf_ablation.tex": "Shelf+IIWA RBF profile and mechanism ablations.",
    "tab_tro_shelf_cross_algorithm.tex": "Shelf+IIWA cross-algorithm comparison.",
    "tab_tro_random_summary.tex": "Random multi-robot summary.",
}

REQUIRED_FIGURES = {
    "fig_tro_shelf_tradeoff.pdf": "Shelf+IIWA RBF five-query amortized-time quality/segment trade-off curve.",
    "fig_tro_shelf_tradeoff.png": "Shelf+IIWA RBF five-query amortized-time quality/segment trade-off curve preview.",
    "fig_tro_shelf_cross_tradeoff.pdf": "Shelf+IIWA five-query amortized-time trade-off with cross-algorithm context.",
    "fig_tro_shelf_cross_tradeoff.png": "Shelf+IIWA five-query amortized-time trade-off with cross-algorithm context preview.",
    "fig_tro_random_tradeoff.pdf": "Random-scene fixed-K=5 reuse-horizon accounting curves.",
    "fig_tro_random_tradeoff.png": "Random-scene fixed-K=5 reuse-horizon accounting preview.",
}

REGISTERED_EXP05_RRTCONNECT_SUMMARY = REPO_ROOT / "outputs" / "tro2026" / "exp05_full_joint_rrtconnect_s0_7" / "shelf_cross_algorithm_summary.csv"
REGISTERED_EXP05_RRTCONNECT_MANIFEST = REPO_ROOT / "outputs" / "tro2026" / "exp05_full_joint_rrtconnect_s0_7" / "shelf_cross_algorithm_manifest.json"
REGISTERED_EXP05_PRM_OPTIMIZED_SUMMARY = REPO_ROOT / "outputs" / "tro2026_current_improve" / "exp05" / "current_prm_optimized" / "shelf_cross_algorithm_summary.csv"
REGISTERED_EXP05_PRM_OPTIMIZED_MANIFEST = REPO_ROOT / "outputs" / "tro2026_current_improve" / "exp05" / "current_prm_optimized" / "shelf_cross_algorithm_manifest.json"
REGISTERED_EXP05_IRIS_GCS_ANYTIME = REPO_ROOT / "outputs" / "new_experiments" / "tro2026" / "exp05" / "iris_np_gcs_anytime.json"
REGISTERED_EXP04_QUERY_MANIFEST = REPO_ROOT / "outputs" / "tro2026" / "exp04" / "shelf_leaf_rrt_manifest.json"
EXP04_D23_CACHE_RECORDS = "16,777,215"
EXP04_D23_CACHE_BYTES = "13,683,310,249"
EXP04_D23_CACHE_GIB = "12.74"
EXP04_D23_PREWARM_S = "1008.600"
EXP04_D23_CACHE_ARTIFACT = Path(
    "outputs/new_experiments/exp04_full_root_d23/"
    "d23_prewarm_full_root_ifk_volume_min.json"
)
GOLD_POINT_RELATIVE_PATH_IMPROVEMENT_THRESHOLD = 0.01

METHOD_STYLE = {
    "sbf_leaf_rrt": {"label": "RBF", "color": "#1f77b4", "marker": "o"},
    "iris_np_gcs": {"label": "IRIS/GCS", "color": "#ff7f0e", "marker": "D"},
    "prm": {"label": "PRM", "color": "#2ca02c", "marker": "s"},
    "rrtconnect": {"label": "RRT-Connect", "color": "#d62728", "marker": "x"},
    "bitstar": {"label": "BIT*", "color": "#9467bd", "marker": "^"},
}

REGISTERED_EXP06_BASELINE_CONTEXT = {
    ("iiwa", "easy"): {
        "iris_np_gcs": {"total_s": 76.400, "path_length": 5.46},
        "prm": {"total_s": 2.411, "path_length": 3.28},
        "rrtconnect": {"total_s": 0.001, "path_length": 2.56},
        "bitstar": {"total_s": 1.006, "path_length": 2.36},
    },
    ("iiwa", "medium"): {
        "iris_np_gcs": {"total_s": 137.700, "path_length": 5.56},
        "prm": {"total_s": 2.408, "path_length": 3.48},
        "rrtconnect": {"total_s": 0.001, "path_length": 2.85},
        "bitstar": {"total_s": 1.005, "path_length": 2.61},
    },
    ("iiwa", "hard"): {
        "iris_np_gcs": {"total_s": 187.500, "path_length": 5.46},
        "prm": {"total_s": 5.239, "path_length": 3.34},
        "rrtconnect": {"total_s": 0.001, "path_length": 2.77},
        "bitstar": {"total_s": 1.006, "path_length": 2.89},
    },
    ("ur5", "easy"): {
        "iris_np_gcs": {"total_s": 18.140, "path_length": 7.62},
        "prm": {"total_s": 1.416, "path_length": 5.17},
        "rrtconnect": {"total_s": 0.002, "path_length": 4.92},
        "bitstar": {"total_s": 1.006, "path_length": 4.69},
    },
    ("ur5", "medium"): {
        "iris_np_gcs": {"total_s": 29.160, "path_length": 7.60},
        "prm": {"total_s": 0.671, "path_length": 7.26},
        "rrtconnect": {"total_s": 0.007, "path_length": 16.58},
        "bitstar": {"total_s": 1.006, "path_length": 5.02},
    },
    ("ur5", "hard"): {
        "iris_np_gcs": {"total_s": 40.670, "path_length": 7.62},
        "prm": {"total_s": 0.920, "path_length": 6.21},
        "rrtconnect": {"total_s": 0.003, "path_length": 9.52},
        "bitstar": {"total_s": 1.005, "path_length": 4.66},
    },
    ("panda", "easy"): {
        "iris_np_gcs": {"total_s": 18.130, "path_length": 5.82},
        "prm": {"total_s": 1.420, "path_length": 4.64},
        "rrtconnect": {"total_s": 0.001, "path_length": 4.38},
        "bitstar": {"total_s": 1.005, "path_length": 4.39},
    },
    ("panda", "medium"): {
        "iris_np_gcs": {"total_s": 30.120, "path_length": 5.87},
        "prm": {"total_s": 1.419, "path_length": 4.82},
        "rrtconnect": {"total_s": 0.001, "path_length": 4.44},
        "bitstar": {"total_s": 1.005, "path_length": 4.90},
    },
    ("panda", "hard"): {
        "iris_np_gcs": {"total_s": 44.300, "path_length": 5.79},
        "prm": {"total_s": 1.420, "path_length": 4.73},
        "rrtconnect": {"total_s": 0.001, "path_length": 3.90},
        "bitstar": {"total_s": 1.005, "path_length": 4.08},
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate TRO2026 paper tables/manifest from current artifacts.")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "outputs" / "new_experiments" / "tro2026")
    parser.add_argument("--paper-dir", type=Path, default=REPO_ROOT / "paper")
    parser.add_argument("--allow-placeholders", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--include-exp06-current-baselines",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Merge exp06/current_ompl_baselines into Table VI/figure generation. "
            "Enabled by default because the registered Exp.6 OMPL baselines use the saved catalog "
            "and the global fixed-step validation/final-simplification policy."
        ),
    )
    return parser.parse_args()


def placeholder_table(caption: str, label: str) -> str:
    return "\n".join([
        r"% Auto-generated placeholder from current self-contained asset pipeline.",
        r"\begingroup",
        r"\centering",
        rf"\captionof{{table}}{{{caption} Full artifact pending; this placeholder is generated by the current self-contained asset pipeline.}}",
        rf"\label{{{label}}}",
        r"\begin{tabular}{lc}",
        r"\toprule",
        r"Metric & Value \\",
        r"\midrule",
        r"Status & Pending \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\par\endgroup",
        "",
    ])


_FILE_SHA256_CACHE: dict[tuple[str, int, int], str] = {}


def file_sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    stat = path.stat()
    cache_key = (str(path.resolve()), int(stat.st_size), int(stat.st_mtime_ns))
    cached = _FILE_SHA256_CACHE.get(cache_key)
    if cached is not None:
        return cached
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    value = digest.hexdigest()
    _FILE_SHA256_CACHE[cache_key] = value
    return value


def file_sha256_if_reasonable(path: Path | None, max_bytes: int = 256 * 1024 * 1024) -> str | None:
    if path is None or not path.exists():
        return None
    try:
        if path.stat().st_size > max_bytes:
            return "skipped_large_file"
    except OSError:
        return None
    return file_sha256(path)


def manifest_path(path: Path | None) -> str | None:
    if path is None:
        return None
    try:
        return str(path.resolve().relative_to(REPO_ROOT.resolve()))
    except ValueError:
        return str(path)


def relativize_manifest(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: relativize_manifest(item) for key, item in value.items()}
    if isinstance(value, list):
        return [relativize_manifest(item) for item in value]
    if isinstance(value, str):
        repo_prefix = str(REPO_ROOT.resolve())
        if value == repo_prefix:
            return "."
        if value.startswith(repo_prefix + os.sep):
            return value[len(repo_prefix) + 1:]
    return value


def tex_num(value: Any, digits: int = 3) -> str:
    if value is None:
        return "--"
    if isinstance(value, str) and value.strip() == "":
        return "--"
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if x != x:
        return "--"
    return f"{x:.{digits}f}"


def tex_qrange_ms(lo_s: Any, hi_s: Any) -> str:
    if lo_s is None or hi_s is None:
        return "--"
    if isinstance(lo_s, str) and lo_s.strip() == "":
        return "--"
    if isinstance(hi_s, str) and hi_s.strip() == "":
        return "--"
    try:
        lo = float(lo_s)
        hi = float(hi_s)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(lo) or not math.isfinite(hi):
        return "--"
    return f"\\([{1000.0 * lo:.2f},{1000.0 * hi:.2f}]\\)"


def tex_qrange_s(lo_s: Any, hi_s: Any) -> str:
    if lo_s is None or hi_s is None:
        return "--"
    if isinstance(lo_s, str) and lo_s.strip() == "":
        return "--"
    if isinstance(hi_s, str) and hi_s.strip() == "":
        return "--"
    try:
        lo = float(lo_s)
        hi = float(hi_s)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(lo) or not math.isfinite(hi):
        return "--"
    return f"\\([{lo:.3f}, {hi:.3f}]\\)"


def tex_iqr_or_qrange_s(values: Any, lo_s: Any, hi_s: Any) -> str:
    vals = finite_values(values)
    if vals:
        return tex_iqr(vals, 3)
    return tex_qrange_s(lo_s, hi_s)


def tex_iqr_or_qrange_ms(values: Any, lo_s: Any, hi_s: Any) -> str:
    vals = finite_values(values)
    if vals:
        return tex_iqr([1000.0 * value for value in vals], 2)
    return tex_qrange_ms(lo_s, hi_s)


def tex_int_commas(value: Any) -> str:
    x = as_float(value)
    if not math.isfinite(x):
        return "--"
    return f"{int(round(x)):,}"


def tex_speedup_range(lo_value: Any, hi_value: Any) -> str:
    lo = as_float(lo_value)
    hi = as_float(hi_value)
    if not math.isfinite(lo) or not math.isfinite(hi):
        return "--"
    return f"{lo:.1f}--{hi:.1f}$\\times$"


def tex_speedup_iqr_or_range(values: Any, lo_value: Any, hi_value: Any) -> str:
    vals = finite_values(values)
    if vals:
        return rf"{tex_iqr(vals, 1)}$\times$"
    return tex_speedup_range(lo_value, hi_value)


def tex_sci(value: Any, digits: int = 2) -> str:
    if value is None:
        return "--"
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(x):
        return "--"
    if abs(x) < 1e-15:
        return "0"
    mantissa, exponent = f"{x:.{digits}e}".split("e")
    return rf"\ensuremath{{{mantissa}\mathrm{{e}}{{{int(exponent)}}}}}"


def tex_fixed(value: Any, digits: int = 6) -> str:
    if value is None:
        return "--"
    try:
        x = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(x):
        return "--"
    if abs(x) < 0.5 * (10 ** -digits):
        x = 0.0
    return f"{x:.{digits}f}"


def as_float(value: Any, default: float = math.nan) -> float:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def finite_values(values: Any) -> list[float]:
    if values is None:
        return []
    if isinstance(values, (str, bytes)) or not isinstance(values, (list, tuple)):
        values = [values]
    out: list[float] = []
    for value in values:
        x = as_float(value)
        if math.isfinite(x):
            out.append(x)
    return out


def percentile_value(values: Any, q: float) -> float:
    vals = sorted(finite_values(values))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    pos = max(0.0, min(1.0, float(q))) * (len(vals) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    alpha = pos - lo
    return vals[lo] * (1.0 - alpha) + vals[hi] * alpha


def tex_iqr(values: Any, digits: int = 3) -> str:
    vals = finite_values(values)
    if not vals:
        return "--"
    med = percentile_value(vals, 0.50)
    q1 = percentile_value(vals, 0.25)
    q3 = percentile_value(vals, 0.75)
    return rf"{tex_num(med, digits)} [{tex_num(q1, digits)}, {tex_num(q3, digits)}]"


def tex_iqr_stacked(values: Any, digits: int = 3) -> str:
    vals = finite_values(values)
    if not vals:
        return "--"
    med = percentile_value(vals, 0.50)
    q1 = percentile_value(vals, 0.25)
    q3 = percentile_value(vals, 0.75)
    return (
        rf"\shortstack{{{tex_num(med, digits)}\\[-0.2ex]"
        rf"{{\scriptsize [{tex_num(q1, digits)}, {tex_num(q3, digits)}]}}}}"
    )


def tex_mean_iqr_stacked(values: Any, digits: int = 3) -> str:
    vals = finite_values(values)
    if not vals:
        return "--"
    avg = sum(vals) / len(vals)
    q1 = percentile_value(vals, 0.25)
    q3 = percentile_value(vals, 0.75)
    return (
        rf"\shortstack{{{tex_num(avg, digits)}\\[-0.2ex]"
        rf"{{\scriptsize [{tex_num(q1, digits)}, {tex_num(q3, digits)}]}}}}"
    )


def tex_time_tail(values: Any, digits: int = 3) -> str:
    return tex_iqr(values, digits)


def tex_median_scalar(values: Any, digits: int = 3) -> str:
    vals = finite_values(values)
    if not vals:
        return "--"
    return tex_num(percentile_value(vals, 0.50), digits)


def tex_median_int(values: Any) -> str:
    vals = finite_values(values)
    if not vals:
        return "--"
    return tex_num(percentile_value(vals, 0.50), 0)


def normalized_gap_values(values: Any, reference: float) -> list[float]:
    ref = as_float(reference)
    if not math.isfinite(ref) or ref <= 0.0:
        return []
    return [
        value / ref
        for value in finite_values(values)
        if math.isfinite(value) and value > 0.0
    ]


def normalized_gap_from_label_values(path_by_label: dict[str, list[float]], refs: dict[str, float]) -> list[float]:
    out: list[float] = []
    for label, values in path_by_label.items():
        out.extend(normalized_gap_values(values, refs.get(label, math.nan)))
    return out


def update_min_ref(refs: dict[str, float], label: str, values: Any) -> None:
    finite = [value for value in finite_values(values) if value > 0.0]
    if not label or not finite:
        return
    candidate = min(finite)
    current = refs.get(label, math.nan)
    if not math.isfinite(current) or candidate < current:
        refs[label] = candidate


def query_path_refs_from_runs(
    runs: list[dict[str, Any]],
    predicate: Any | None = None,
) -> dict[str, float]:
    refs: dict[str, float] = {}
    for run in runs:
        if predicate is not None and not predicate(run):
            continue
        for query in run.get("queries", []):
            if not query_success(query):
                continue
            update_min_ref(refs, str(query.get("label", "")), query.get("path_length"))
    return refs


def query_path_refs_from_methods(methods: list[dict[str, Any]]) -> dict[str, float]:
    refs: dict[str, float] = {}
    for method in methods:
        queries = method.get("queries", {})
        if not isinstance(queries, dict):
            continue
        for label, stats in queries.items():
            if not isinstance(stats, dict):
                continue
            update_min_ref(refs, str(label), stats.get("path_values", stats.get("path")))
    return refs


def query_success(query: dict[str, Any]) -> bool:
    return bool(query.get("success", query.get("audit_passed", False))) and bool(query.get("audit_passed", True))


def query_solve_seconds(query: dict[str, Any]) -> float:
    solve_ms = as_float(query.get("solve_ms"))
    if not math.isfinite(solve_ms):
        query_ms = as_float(query.get("query_ms"))
        simplify_ms = as_float(query.get("simplify_ms"), 0.0)
        if math.isfinite(query_ms):
            solve_ms = max(0.0, query_ms - (simplify_ms if math.isfinite(simplify_ms) else 0.0))
    return solve_ms / 1000.0 if math.isfinite(solve_ms) else math.nan


def online_query_time(row: dict[str, Any]) -> float:
    """Online query time excluding fixed final simplification."""
    method = str(row.get("method", "")).lower()
    build = as_float(row.get("offline_build_s_median", row.get("build_s")), 0.0)
    for key in (
        "online_solve_per_query_s_median",
        "online_solve_per_query_s",
        "online_per_query_s_median",
        "online_per_query_s",
        "query_s_median",
        "query_s",
    ):
        value = as_float(row.get(key))
        if math.isfinite(value):
            if method == "prm" and value <= 1e-9:
                amortized = as_float(row.get("amortized_s_k5"))
                if math.isfinite(amortized) and math.isfinite(build):
                    derived = amortized - build / 5.0
                    if derived > 1e-9:
                        return derived
                continue
            return value
    return math.nan


def online_total_query_time(row: dict[str, Any]) -> float:
    """Online query time including fixed final simplification, for diagnostics only."""
    for key in ("online_total_per_query_s_median", "online_per_query_s_median", "query_s_median"):
        value = as_float(row.get(key))
        if math.isfinite(value):
            return value
    return math.nan


def amortized_query_time(row: dict[str, Any], k: int) -> float:
    build = as_float(row.get("offline_build_s_median", row.get("build_s")), 0.0)
    online = online_query_time(row)
    if math.isfinite(online):
        return build / float(k) + online
    return math.nan


def method_time(row: dict[str, Any]) -> float:
    reported_amortized = as_float(row.get("amortized_s_k5"))
    if math.isfinite(reported_amortized):
        return reported_amortized
    amortized = amortized_query_time(row, 5)
    if math.isfinite(amortized):
        return amortized
    return as_float(row.get("planning_s_median", row.get("build_s")))


def measured_time_key(row: dict[str, Any]) -> float:
    """Actual measured planning time used for all paper trade-off axes/selection."""
    value = method_time(row)
    return value if math.isfinite(value) else 1e9


def random_display_time(row: dict[str, Any]) -> float:
    """Exp.6 display time for per-query-selected OMPL rows."""
    display_values = finite_values(row.get("_display_time_values"))
    if display_values:
        display_med = median(display_values)
        if display_med is not None and math.isfinite(float(display_med)):
            return float(display_med)
    return method_time(row)


def is_full_success(row: dict[str, Any]) -> bool:
    success = as_float(row.get("success_queries", row.get("success_runs", row.get("success_scenes"))), 0.0)
    total = as_float(row.get("total_queries", row.get("runs", row.get("scenes"))), 0.0)
    return total > 0.0 and success >= total


def select_quality_plateau_index(
    quality_values: list[float],
    *,
    start_index: int = 0,
    relative_improvement_threshold: float = GOLD_POINT_RELATIVE_PATH_IMPROVEMENT_THRESHOLD,
) -> int | None:
    """Select the last substantial quality-improvement point after start.

    Lower values are better. Starting from the first point with validated success
    on the saved query set, move
    forward only when a later checkpoint improves the current displayed path
    quality by at least the relative threshold.  This prevents a gold marker
    from drifting to a much later checkpoint for a visually negligible gain.
    """
    if not quality_values:
        return None
    start_index = max(0, min(start_index, len(quality_values) - 1))
    finite_indices = [
        index for index in range(start_index, len(quality_values))
        if math.isfinite(as_float(quality_values[index]))
    ]
    if not finite_indices:
        return None
    selected = finite_indices[0]
    while True:
        current_quality = as_float(quality_values[selected])
        if not math.isfinite(current_quality) or current_quality <= 0.0:
            return selected
        threshold_quality = current_quality * (1.0 - relative_improvement_threshold)
        next_index = None
        for index in range(selected + 1, len(quality_values)):
            candidate_quality = as_float(quality_values[index])
            if math.isfinite(candidate_quality) and candidate_quality <= threshold_quality:
                next_index = index
                break
        if next_index is None:
            return selected
        selected = next_index


AMORTIZATION_QUERY_COUNTS = [1, 5, 10, 20, 50]
PANEL_TITLE_FONTSIZE = 9.4
AXIS_LABEL_FONTSIZE = 8.9
TICK_LABEL_FONTSIZE = 8.0
LEGEND_FONTSIZE = 8.1
LINE_WIDTH = 1.35
POINT_SIZE = 32
SELECTED_POINT_SIZE = 78
SELECTED_LINE_WIDTH = 1.65


def configure_matplotlib_for_ieee(matplotlib_module: Any) -> None:
    """Use submission-friendly embedded fonts for generated vector figures."""
    matplotlib_module.rcParams.update({
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "font.family": "serif",
        "font.serif": [
            "Times New Roman",
            "Liberation Serif",
            "Times",
            "DejaVu Serif",
        ],
        # DejaVu math glyphs embed as standards-compliant TrueType outlines.
        # OTF math glyphs trigger a false CID type/file mismatch in Poppler
        # when Matplotlib's PDF backend is configured for fonttype 42. The
        # Times-compatible Liberation Serif text face is TrueType as well.
        "mathtext.fontset": "dejavuserif",
    })


def plot_query_amortization_panel(
    ax: Any,
    methods: list[dict[str, Any]],
    *,
    title: str = "(b) reuse-horizon display",
    show_xlabel: bool = True,
    show_ylabel: bool = True,
    font_scale: float = 1.0,
) -> None:
    plotted = False
    for method in methods:
        label = str(method.get("label", method.get("method", "")))
        style = METHOD_STYLE.get(str(method.get("method", "")), {})
        color = method.get("color", style.get("color", "0.35"))
        marker = method.get("marker", style.get("marker", "o"))
        build_s = as_float(method.get("build_s"), 0.0)
        per_query_s = as_float(method.get("per_query_s"), 0.0)
        if not math.isfinite(build_s) or not math.isfinite(per_query_s):
            continue
        ys = [
            max(1e-5, (max(0.0, build_s) + count * max(0.0, per_query_s)) / count)
            for count in AMORTIZATION_QUERY_COUNTS
        ]
        ax.plot(
            AMORTIZATION_QUERY_COUNTS,
            ys,
            marker=marker,
            markersize=3.3,
            linewidth=LINE_WIDTH,
            color=color,
            label=label,
            alpha=0.92,
        )
        plotted = True
    if not plotted:
        ax.text(0.5, 0.5, "amortization rows missing", ha="center", va="center", fontsize=7.0)
        ax.axis("off")
        return
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("number of queries" if show_xlabel else "")
    ax.set_ylabel("reported task coordinate (s)" if show_ylabel else "")
    ax.set_title(title, fontsize=PANEL_TITLE_FONTSIZE * font_scale)
    ax.grid(True, which="both", alpha=0.24)
    ax.tick_params(labelsize=TICK_LABEL_FONTSIZE * font_scale)
    ax.xaxis.label.set_size(AXIS_LABEL_FONTSIZE * font_scale)
    ax.yaxis.label.set_size(AXIS_LABEL_FONTSIZE * font_scale)


def use_compact_log_x_ticks(ax: Any, *, numticks: int = 3) -> None:
    from matplotlib.ticker import LogLocator, NullFormatter

    ax.xaxis.set_major_locator(LogLocator(base=10.0, numticks=numticks))
    ax.xaxis.set_minor_formatter(NullFormatter())


def set_padded_linear_ylim(ax: Any, values: list[float], *, min_pad: float = 0.08) -> None:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return
    low = min(finite)
    high = max(finite)
    span = max(high - low, min_pad)
    ax.set_ylim(low - 0.08 * span, high + 0.10 * span)


def finite_min(values: Any) -> float:
    finite = finite_values(values)
    return min(finite) if finite else math.nan


def ratio_to_ref(value: Any, reference: Any) -> float:
    x = as_float(value)
    ref = as_float(reference)
    if not math.isfinite(x) or not math.isfinite(ref) or ref <= 0.0:
        return math.nan
    return x / ref


def generate_pipeline_figure(pdf_path: Path, png_path: Path) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

    fig, ax = plt.subplots(1, 1, figsize=(11.6, 3.15))
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")

    palette = {
        "geom": "#e7f0f7",
        "cache": "#edf3e8",
        "scene": "#f7efe4",
        "query": "#f1edf7",
        "edge": "#38444f",
        "muted": "#6a737d",
    }

    def add_box(
        x: float,
        y: float,
        w: float,
        h: float,
        text: str,
        *,
        facecolor: str,
        fontsize: float = 8.5,
        weight: str = "normal",
    ) -> None:
        patch = FancyBboxPatch(
            (x, y),
            w,
            h,
            boxstyle="round,pad=0.012,rounding_size=0.012",
            linewidth=0.85,
            edgecolor=palette["edge"],
            facecolor=facecolor,
        )
        ax.add_patch(patch)
        ax.text(
            x + w / 2.0,
            y + h / 2.0,
            text,
            ha="center",
            va="center",
            fontsize=fontsize,
            fontweight=weight,
            color="#111827",
            linespacing=1.08,
        )

    def arrow(
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        *,
        style: str = "-|>",
        rad: float = 0.0,
        color: str | None = None,
    ) -> None:
        ax.add_patch(
            FancyArrowPatch(
                (x1, y1),
                (x2, y2),
                arrowstyle=style,
                mutation_scale=10.5,
                linewidth=0.9,
                color=color or palette["edge"],
                connectionstyle=f"arc3,rad={rad}",
            )
        )

    ax.text(0.035, 0.92, "Geometric box validation", fontsize=9.3, fontweight="bold")
    ax.text(0.545, 0.92, "Reusable planning layers", fontsize=9.3, fontweight="bold")

    add_box(0.035, 0.66, 0.145, 0.17, "Joint box $I$\nrounded links", facecolor=palette["geom"])
    add_box(0.215, 0.66, 0.165, 0.17, "Endpoint envelopes\n$E_a(I), E_b(I)$", facecolor=palette["geom"])
    add_box(0.415, 0.66, 0.165, 0.17, "Link envelope\nconv$(E_a \\cup E_b)$ + r", facecolor=palette["geom"])
    add_box(0.615, 0.66, 0.155, 0.17, "Scene validation\nagainst obstacles", facecolor=palette["scene"])
    add_box(0.815, 0.66, 0.145, 0.17, "Accepted box\nor blocker set", facecolor=palette["scene"])

    arrow(0.18, 0.745, 0.215, 0.745)
    arrow(0.38, 0.745, 0.415, 0.745)
    arrow(0.58, 0.745, 0.615, 0.745)
    arrow(0.77, 0.745, 0.815, 0.745)

    add_box(
        0.235,
        0.43,
        0.330,
        0.12,
        "LECT evidence cache: kinematics-keyed endpoint/link records\n(no obstacle labels or imported collision-free certificates)",
        facecolor=palette["cache"],
        fontsize=7.8,
    )
    arrow(0.315, 0.66, 0.315, 0.55, color=palette["muted"])
    arrow(0.500, 0.55, 0.500, 0.66, color=palette["muted"])

    add_box(0.075, 0.18, 0.165, 0.17, "Grow / refine\ncandidate boxes", facecolor=palette["scene"])
    add_box(0.290, 0.18, 0.165, 0.17, "Scene-level\nbox forest", facecolor=palette["scene"])
    add_box(0.505, 0.18, 0.165, 0.17, "Typed corridor graph\n$E_{\\cap}, E_s, E_\\pi$", facecolor=palette["scene"])
    add_box(0.715, 0.18, 0.120, 0.17, "Query path\nretrieval", facecolor=palette["query"])
    add_box(0.875, 0.18, 0.085, 0.17, "Strict\naudit", facecolor=palette["query"])

    arrow(0.240, 0.265, 0.290, 0.265)
    arrow(0.455, 0.265, 0.505, 0.265)
    arrow(0.670, 0.265, 0.715, 0.265)
    arrow(0.835, 0.265, 0.875, 0.265)

    add_box(
        0.645,
        0.43,
        0.315,
        0.12,
        "Obstacle edit: invalidate dirty boxes, promote cached blockers,\nthen refill locally under the same scene-validation policy",
        facecolor="#f9f2dc",
        fontsize=7.55,
    )
    arrow(0.725, 0.43, 0.590, 0.35, rad=0.05, color=palette["muted"])

    ax.text(
        0.035,
        0.055,
        "Certificate boundary: only paths contained in conservatively validated boxes inherit the box-corridor certificate; repair, segments, and smoothing require query validation.",
        fontsize=7.4,
        color="#333333",
    )
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.015)
    fig.savefig(png_path, dpi=240, bbox_inches="tight", pad_inches=0.015)
    plt.close(fig)


def current_random_context_from_rows(rows: list[dict[str, Any]]) -> dict[tuple[str, str], dict[str, dict[str, float]]]:
    out: dict[tuple[str, str], dict[str, dict[str, float]]] = {}
    expected_queries: dict[tuple[str, str], float] = {}
    methods = sorted({
        str(row.get("method", ""))
        for row in rows
        if str(row.get("method", "")) and str(row.get("method", "")) != "sbf_leaf_rrt"
    })
    scenarios = sorted({
        (str(row.get("robot", "")).lower(), str(row.get("difficulty", "")).lower())
        for row in rows
        if str(row.get("robot", "")) and str(row.get("difficulty", ""))
    })
    for robot, difficulty in scenarios:
        totals = [
            as_float(row.get("total_queries", row.get("scenes")), 0.0)
            for row in rows
            if str(row.get("robot", "")).lower() == robot
            and str(row.get("difficulty", "")).lower() == difficulty
        ]
        finite_totals = [value for value in totals if math.isfinite(value) and value > 0.0]
        expected_queries[(robot, difficulty)] = max(finite_totals) if finite_totals else 0.0
    for robot, difficulty in scenarios:
        for method in methods:
            items = [
                row for row in rows
                if str(row.get("method", "")) == method
                and str(row.get("robot", "")).lower() == robot
                and str(row.get("difficulty", "")).lower() == difficulty
            ]
            if method == "bitstar":
                per_query_items = [
                    row for row in items
                    if str(row.get("source", "")) == "per_query_seed_tradeoff_from_manifest"
                ]
                if per_query_items:
                    # Table VI declares the per-scene-query 8% selector.  Keep
                    # that registered aggregate row separate from the
                    # checkpoint-level curve selector used only by Fig. 5.
                    items = per_query_items
            full = []
            for row in items:
                success = as_float(row.get("success_queries", row.get("success_scenes")), 0.0)
                total = as_float(row.get("total_queries", row.get("scenes")), 0.0)
                plan = random_display_time(row)
                query_s = online_query_time(row)
                path_len = path_length_stat(row)
                if method == "prm" and not math.isfinite(query_s):
                    continue
                expected = expected_queries.get((robot, difficulty), 0.0)
                if (
                    total > 0
                    and success >= total
                    and (expected <= 0.0 or total >= expected)
                    and math.isfinite(plan)
                    and math.isfinite(path_len)
                ):
                    full.append(row)
            if not full:
                continue
            ordered_full = sorted(
                full,
                key=lambda row: (
                    random_display_time(row),
                    path_length_stat(row) if math.isfinite(path_length_stat(row)) else 1e9,
                ),
            )
            quality_values = [
                path_length_stat(row) if math.isfinite(path_length_stat(row)) else math.nan
                for row in ordered_full
            ]
            chosen_index = select_quality_plateau_index(quality_values, start_index=0)
            chosen = ordered_full[chosen_index if chosen_index is not None else 0]
            build_s = as_float(chosen.get("offline_build_s_median", chosen.get("build_s")), 0.0)
            query_s = online_query_time(chosen)
            if not math.isfinite(query_s):
                query_s = method_time(chosen)
            total_s = random_display_time(chosen)
            path_len = path_length_stat(chosen)
            build_values = (
                chosen.get("_build_values")
                or finite_values(0.0 if method in {"rrtconnect", "bitstar"} else build_s)
            )
            query_values = chosen.get("_online_solve_values") or finite_values(query_s)
            display_values = (
                chosen.get("_display_time_values")
                or chosen.get("_online_solve_values")
                or finite_values(total_s)
            )
            if build_values:
                build_s = percentile_value(build_values, 0.50)
            if query_values:
                query_s = percentile_value(query_values, 0.50)
            if display_values:
                total_s = percentile_value(display_values, 0.50)
            if method in {"rrtconnect", "bitstar"}:
                total_s = query_s
            out.setdefault((robot, difficulty), {})[method] = {
                "build_s": 0.0 if method in {"rrtconnect", "bitstar"} else build_s,
                "query_s": query_s,
                "total_s": total_s,
                "path_length": path_len,
                "build_values": build_values,
                "query_values": query_values,
                "time_values": display_values,
                "path_values": chosen.get("_path_values") or [],
                "path_by_label": chosen.get("_path_by_label") or {},
                "success": int(as_float(chosen.get("success_queries", chosen.get("success_scenes")), 0.0)),
                "total": int(as_float(chosen.get("total_queries", chosen.get("scenes")), 0.0)),
                "source": "current_saved_catalog",
                "stage_id": str(chosen.get("stage_id", "")),
                "measured_time_s": total_s,
            }
    return out


def annotate_summary_rows_with_manifest_distributions(
    summary_rows: list[dict[str, Any]],
    manifest_payload: dict[str, Any],
) -> None:
    if not isinstance(manifest_payload, dict):
        return
    manifest_rows = manifest_payload.get("rows", [])
    if not isinstance(manifest_rows, list) or not manifest_rows:
        return

    def norm_text(value: Any) -> str:
        return str(value or "").lower()

    def norm_budget(value: Any) -> int | None:
        raw = str(value or "")
        if not raw or raw in {"0", "0.0"}:
            return None
        try:
            return int(float(raw))
        except (TypeError, ValueError):
            return None

    exact_index: dict[tuple[str, str, str, str, int], list[dict[str, Any]]] = {}
    loose_index: dict[tuple[str, str, str, str], list[dict[str, Any]]] = {}
    for run in manifest_rows:
        loose_key = (
            norm_text(run.get("method")),
            norm_text(run.get("robot")),
            norm_text(run.get("difficulty")),
            norm_text(run.get("stage_id")),
        )
        loose_index.setdefault(loose_key, []).append(run)
        run_budget = norm_budget(run.get("deep_max_boxes"))
        if run_budget is not None:
            exact_index.setdefault((*loose_key, run_budget), []).append(run)

    def matches(summary: dict[str, Any], run: dict[str, Any]) -> bool:
        for field in ("method", "robot", "difficulty", "stage_id"):
            expected = str(summary.get(field, ""))
            if expected and str(run.get(field, "")) != expected:
                return False
        summary_budget = str(summary.get("deep_max_boxes", ""))
        if summary_budget and summary_budget not in {"0", "0.0"}:
            if int(float(run.get("deep_max_boxes", summary_budget) or summary_budget)) != int(float(summary_budget)):
                return False
        return True

    for summary in summary_rows:
        loose_key = (
            norm_text(summary.get("method")),
            norm_text(summary.get("robot")),
            norm_text(summary.get("difficulty")),
            norm_text(summary.get("stage_id")),
        )
        summary_budget = norm_budget(summary.get("deep_max_boxes"))
        if summary_budget is None:
            candidates = loose_index.get(loose_key, [])
        else:
            candidates = exact_index.get((*loose_key, summary_budget), [])
            if not candidates:
                candidates = loose_index.get(loose_key, [])
        matched = [run for run in candidates if matches(summary, run)]
        if not matched:
            continue
        build_values: list[float] = []
        online_solve_values: list[float] = []
        simplify_values: list[float] = []
        path_values: list[float] = []
        segment_values: list[float] = []
        path_by_label: dict[str, list[float]] = {}
        amortized_k10_values: list[float] = []
        display_time_values: list[float] = []
        method = str(summary.get("method", ""))
        for run in matched:
            build = as_float(run.get("offline_build_s", run.get("build_s")))
            if math.isfinite(build):
                build_values.append(build)
            amortized = as_float(run.get("amortized_s_k10"))
            if math.isfinite(amortized):
                amortized_k10_values.append(amortized)
            display_time = method_time(run)
            if math.isfinite(display_time):
                display_time_values.append(display_time)
            if method == "sbf_leaf_rrt":
                online_scene_q = online_query_time(run)
                if math.isfinite(online_scene_q):
                    online_solve_values.append(online_scene_q)
                simplify_scene_q = as_float(run.get("online_simplify_per_query_s"))
                if math.isfinite(simplify_scene_q):
                    simplify_values.append(simplify_scene_q)
            for query in run.get("queries", []):
                if not query_success(query):
                    continue
                if method != "sbf_leaf_rrt":
                    solve_ms = as_float(query.get("solve_ms"))
                    if math.isfinite(solve_ms):
                        online_solve_values.append(solve_ms / 1000.0)
                    simplify_ms = as_float(query.get("simplify_ms"))
                    if math.isfinite(simplify_ms):
                        simplify_values.append(simplify_ms / 1000.0)
                path_values.append(as_float(query.get("path_length")))
                segment_values.append(as_float(query.get("segment_fraction")))
                label = str(query.get("label", ""))
                if label:
                    path_by_label.setdefault(label, []).append(as_float(query.get("path_length")))
        summary["_build_values"] = build_values
        summary["_online_solve_values"] = online_solve_values
        summary["_online_simplify_values"] = simplify_values
        summary["_amortized_k10_values"] = amortized_k10_values
        summary["_display_time_values"] = display_time_values or online_solve_values
        summary["_path_values"] = path_values
        summary["_path_by_label"] = path_by_label
        summary["_segment_values"] = segment_values


def _select_query_tradeoff_candidate(candidates: list[dict[str, Any]], *, path_slack: float = 1.08) -> dict[str, Any] | None:
    finite_path = [
        as_float(candidate.get("path"))
        for candidate in candidates
        if math.isfinite(as_float(candidate.get("path")))
    ]
    if not finite_path:
        return None
    best_path = min(finite_path)
    viable = [
        candidate for candidate in candidates
        if math.isfinite(as_float(candidate.get("path")))
        and as_float(candidate.get("path")) <= path_slack * best_path
        and math.isfinite(as_float(candidate.get("display_s")))
    ]
    if not viable:
        viable = [
            candidate for candidate in candidates
            if math.isfinite(as_float(candidate.get("display_s")))
        ]
    if not viable:
        return None
    return sorted(
        viable,
        key=lambda candidate: (
            as_float(candidate.get("display_s")),
            as_float(candidate.get("path")),
            as_float(candidate.get("budget_s")),
        ),
    )[0]


def exp06_per_query_tradeoff_rows(
    manifest_payload: dict[str, Any],
    *,
    methods: set[str],
    path_slack: float = 1.08,
) -> list[dict[str, Any]]:
    """Create Exp.6 PRM/BIT* rows by selecting trade-off per query/seed.

    Summary checkpoint rows charge one stage to all queries. For paper Table IV
    and the random-scene context markers, reusable PRM and anytime BIT* instead
    use the best trade-off candidate after fixed-step validation independently for each
    saved query.
    """
    if not isinstance(manifest_payload, dict):
        return []
    run_rows = manifest_payload.get("rows", [])
    if not isinstance(run_rows, list):
        return []

    all_query_keys: dict[tuple[str, str, str], set[tuple[int, str]]] = {}
    all_scene_labels: dict[tuple[str, str, str, int], set[str]] = {}
    candidates: dict[tuple[str, str, str, int, str], list[dict[str, Any]]] = {}
    query_count_by_run: dict[tuple[str, str, str, int, str], int] = {}

    for run in run_rows:
        method = str(run.get("method", ""))
        if method not in methods:
            continue
        robot = str(run.get("robot", "")).lower()
        difficulty = str(run.get("difficulty", "")).lower()
        if not robot or not difficulty:
            continue
        scene_seed = int(as_float(run.get("scene_seed"), 0.0))
        queries = run.get("queries", [])
        if not isinstance(queries, list):
            continue
        query_count = max(1, int(as_float(run.get("query_count"), len(queries) or 1)))
        build_s = as_float(run.get("offline_build_s", run.get("build_s")), 0.0)
        stage_id = str(run.get("stage_id", ""))
        budget_s = as_float(run.get("budget_s", run.get("timeout_cap_s")))
        for query_index, query in enumerate(queries):
            label = str(query.get("label", f"q{query_index}"))
            scenario_key = (method, robot, difficulty)
            scene_key = (method, robot, difficulty, scene_seed)
            query_key = (method, robot, difficulty, scene_seed, label)
            all_query_keys.setdefault(scenario_key, set()).add((scene_seed, label))
            all_scene_labels.setdefault(scene_key, set()).add(label)
            query_count_by_run[query_key] = query_count
            if not query_success(query):
                continue
            solve_s = query_solve_seconds(query)
            path = as_float(query.get("path_length"))
            if not math.isfinite(solve_s) or not math.isfinite(path):
                continue
            display_s = solve_s
            if method == "prm":
                display_s = build_s / float(query_count) + solve_s
            candidates.setdefault(query_key, []).append({
                "build_s": 0.0 if method in {"bitstar", "rrtconnect"} else build_s,
                "budget_s": budget_s,
                "display_s": display_s,
                "label": label,
                "path": path,
                "query_count": query_count,
                "scene_seed": scene_seed,
                "segment": as_float(query.get("segment_fraction")),
                "solve_s": solve_s,
                "stage_id": stage_id,
            })

    selected_by_scenario: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    selected_scene_labels: dict[tuple[str, str, str, int], set[str]] = {}
    for query_key, items in candidates.items():
        method, _robot, _difficulty, _scene_seed, _label = query_key
        if method == "prm":
            last_measured_solve = math.nan
            for item in sorted(items, key=lambda candidate: as_float(candidate.get("budget_s"))):
                solve_s = as_float(item.get("solve_s"))
                if math.isfinite(solve_s) and solve_s > 0.0:
                    last_measured_solve = solve_s
                    continue
                if not math.isfinite(last_measured_solve):
                    continue
                item["solve_s"] = last_measured_solve
                item["display_s"] = (
                    as_float(item.get("build_s"), 0.0) / max(1.0, as_float(item.get("query_count"), 1.0))
                    + last_measured_solve
                )
        selected = _select_query_tradeoff_candidate(items, path_slack=path_slack)
        if selected is None:
            continue
        method, robot, difficulty, scene_seed, label = query_key
        selected_by_scenario.setdefault((method, robot, difficulty), []).append(selected)
        selected_scene_labels.setdefault((method, robot, difficulty, scene_seed), set()).add(label)

    out: list[dict[str, Any]] = []
    for scenario_key, all_keys in sorted(all_query_keys.items()):
        method, robot, difficulty = scenario_key
        selected = selected_by_scenario.get(scenario_key, [])
        selected_count = len(selected)
        total_count = len(all_keys)
        scene_keys = [
            key for key in all_scene_labels
            if key[:3] == scenario_key
        ]
        success_scenes = sum(
            1 for key in scene_keys
            if all_scene_labels.get(key, set())
            and all_scene_labels.get(key, set()).issubset(selected_scene_labels.get(key, set()))
        )
        build_values = [as_float(item.get("build_s")) for item in selected if math.isfinite(as_float(item.get("build_s")))]
        display_values = [as_float(item.get("display_s")) for item in selected if math.isfinite(as_float(item.get("display_s")))]
        online_values = [as_float(item.get("solve_s")) for item in selected if math.isfinite(as_float(item.get("solve_s")))]
        path_values = [as_float(item.get("path")) for item in selected if math.isfinite(as_float(item.get("path")))]
        segment_values = [as_float(item.get("segment")) for item in selected if math.isfinite(as_float(item.get("segment")))]
        path_by_label: dict[str, list[float]] = {}
        stage_counts: dict[str, int] = {}
        for item in selected:
            label = str(item.get("label", ""))
            path = as_float(item.get("path"))
            if label and math.isfinite(path):
                path_by_label.setdefault(label, []).append(path)
            stage = str(item.get("stage_id", ""))
            if stage:
                stage_counts[stage] = stage_counts.get(stage, 0) + 1
        out.append({
            "method": method,
            "robot": robot,
            "difficulty": difficulty,
            "stage_id": f"{method}_per_query_seed_tradeoff",
            "budget_s": median(item.get("budget_s") for item in selected) if selected else math.nan,
            "source": "per_query_seed_tradeoff_from_manifest",
            "tradeoff_policy": f"per_query_seed_path_slack_{path_slack:g}",
            "selected_stage_counts": stage_counts,
            "scenes": len(scene_keys),
            "success_scenes": success_scenes,
            "queries_per_scene": median(query_count_by_run.values()) if query_count_by_run else math.nan,
            "success_queries": selected_count,
            "total_queries": total_count,
            "offline_build_s_median": median(build_values),
            "build_s": median(build_values),
            "online_solve_per_query_s_median": median(online_values),
            "online_per_query_s_median": median(online_values),
            "planning_s_median": median(display_values),
            "measured_time_s_median": median(display_values),
            "path_length_mean": mean(path_values),
            "path_length_median": median(path_values),
            "raw_segment_fraction_median": median(segment_values),
            "_build_values": build_values,
            "_display_time_values": display_values,
            "_online_solve_values": online_values,
            "_path_by_label": path_by_label,
            "_path_values": path_values,
            "_segment_values": segment_values,
        })
    return out


def exp06_prm_cumulative_curve_rows(manifest_payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Build full-catalog PRM curve checkpoints directly from the manifest.

    The cumulative PRM runner stores incumbent paths for unchanged queries in
    the manifest, while the summary CSV can collapse or omit late repeats.
    Fig. 5 needs the full cumulative envelope so the late roadmap plateau
    remains visible.  Table selection still uses the normal summary and
    per-query trade-off rows.
    """
    if not isinstance(manifest_payload, dict):
        return []
    run_rows = manifest_payload.get("rows", [])
    if not isinstance(run_rows, list):
        return []

    scenario_runs: dict[tuple[str, str], list[dict[str, Any]]] = {}
    scenario_query_keys: dict[tuple[str, str], set[tuple[str, str]]] = {}
    scenario_scenes: dict[tuple[str, str], set[str]] = {}
    for run in run_rows:
        if str(run.get("method", "")) != "prm":
            continue
        robot = str(run.get("robot", "")).lower()
        difficulty = str(run.get("difficulty", "")).lower()
        stage_id = str(run.get("stage_id", ""))
        if not robot or not difficulty or not stage_id:
            continue
        scenario_key = (robot, difficulty)
        scenario_runs.setdefault(scenario_key, []).append(run)
        scene_seed = str(run.get("scene_seed", ""))
        if scene_seed:
            scenario_scenes.setdefault(scenario_key, set()).add(scene_seed)
        queries = run.get("queries", [])
        if not isinstance(queries, list):
            continue
        for query_index, query in enumerate(queries):
            label = str(query.get("label", f"q{query_index}"))
            scenario_query_keys.setdefault(scenario_key, set()).add((scene_seed, label))

    out: list[dict[str, Any]] = []
    for (robot, difficulty), runs in sorted(scenario_runs.items()):
        expected_keys = scenario_query_keys.get((robot, difficulty), set())
        if not expected_keys:
            continue
        by_stage: dict[str, list[dict[str, Any]]] = {}
        stage_budget: dict[str, float] = {}
        for run in runs:
            stage_id = str(run.get("stage_id", ""))
            by_stage.setdefault(stage_id, []).append(run)
            budget = as_float(run.get("budget_s"))
            if math.isfinite(budget):
                stage_budget[stage_id] = budget

        incumbent_path: dict[tuple[str, str], float] = {}
        incumbent_segment: dict[tuple[str, str], float] = {}
        incumbent_solve: dict[tuple[str, str], float] = {}
        for stage_id, group in sorted(
            by_stage.items(),
            key=lambda item: (stage_budget.get(item[0], math.inf), item[0]),
        ):
            build_values: list[float] = []
            query_count_values: list[float] = []
            for run in group:
                scene_seed = str(run.get("scene_seed", ""))
                query_count = as_float(run.get("query_count"), math.nan)
                if math.isfinite(query_count):
                    query_count_values.append(query_count)
                build = as_float(run.get("offline_build_s", run.get("build_s")))
                if math.isfinite(build):
                    build_values.append(build)
                queries = run.get("queries", [])
                if not isinstance(queries, list):
                    continue
                for query_index, query in enumerate(queries):
                    label = str(query.get("label", f"q{query_index}"))
                    key = (scene_seed, label)
                    if not query_success(query):
                        continue
                    path = as_float(query.get("path_length"))
                    if math.isfinite(path):
                        previous = incumbent_path.get(key, math.inf)
                        incumbent_path[key] = min(previous, path)
                    segment = as_float(query.get("segment_fraction"))
                    if math.isfinite(segment):
                        incumbent_segment[key] = segment
                    solve_s = query_solve_seconds(query)
                    if math.isfinite(solve_s):
                        incumbent_solve[key] = solve_s

            if not expected_keys.issubset(incumbent_path.keys()):
                continue
            path_values = [incumbent_path[key] for key in sorted(expected_keys)]
            segment_values = [
                incumbent_segment.get(key, 0.0)
                for key in sorted(expected_keys)
            ]
            solve_values = [
                incumbent_solve.get(key, 0.0)
                for key in sorted(expected_keys)
            ]
            path_by_label: dict[str, list[float]] = {}
            for _scene_seed, label in sorted(expected_keys):
                path_by_label.setdefault(label, []).append(incumbent_path[(_scene_seed, label)])
            budget = stage_budget.get(stage_id, math.nan)
            build_for_display = (
                budget
                if math.isfinite(budget)
                else percentile_value(build_values, 0.50)
            )
            solve_for_display = percentile_value(solve_values, 0.50) if solve_values else 0.0
            display_s = max(0.0, build_for_display) / 5.0 + max(0.0, solve_for_display)
            out.append({
                "method": "prm",
                "robot": robot,
                "difficulty": difficulty,
                "stage_id": stage_id,
                "source": "prm_cumulative_curve_from_manifest",
                "budget_s": budget,
                "scenes": len(scenario_scenes.get((robot, difficulty), set())),
                "success_scenes": len(scenario_scenes.get((robot, difficulty), set())),
                "queries_per_scene": median(query_count_values),
                "success_queries": len(expected_keys),
                "total_queries": len(expected_keys),
                "offline_build_s_median": percentile_value(build_values, 0.50),
                "build_s": build_for_display,
                "online_solve_per_query_s_median": solve_for_display,
                "online_per_query_s_median": solve_for_display,
                "planning_s_median": display_s,
                "measured_time_s_median": display_s,
                "amortized_s_k5": display_s,
                "path_length_mean": mean(path_values),
                "path_length_median": percentile_value(path_values, 0.50),
                "raw_segment_fraction_median": percentile_value(segment_values, 0.50),
                "_build_values": build_values,
                "_display_time_values": [display_s],
                "_online_solve_values": solve_values,
                "_path_by_label": path_by_label,
                "_path_values": path_values,
                "_segment_values": segment_values,
            })
    return out


def current_random_curves_from_rows(rows: list[dict[str, Any]]) -> dict[tuple[str, str], dict[str, list[dict[str, Any]]]]:
    def sparse_cumulative_prm_points(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Keep the PRM cumulative roadmap curve readable in Fig. 5.

        The cumulative PRM runner emits many closely spaced build checkpoints
        (often 50--100 per scenario).  Plotting every checkpoint makes the
        displayed trade-off unreadable and overstates the visual weight of PRM
        relative to RBF/BIT*.  The table selection still sees the full data; the
        figure keeps only a small set of Pareto-improving checkpoints sampled
        on a log-time scale.
        """
        if len(points) <= 8:
            return points
        sorted_points = sorted(points, key=lambda item: item["total_s"])
        pareto: list[tuple[int, dict[str, Any]]] = []
        best_path = math.inf
        for index, point in enumerate(sorted_points):
            path = as_float(point.get("path_length"))
            if not math.isfinite(path):
                continue
            if not pareto or path < best_path * (1.0 - 1e-3):
                pareto.append((index, point))
                best_path = min(best_path, path)
        source = pareto if len(pareto) >= 2 else list(enumerate(sorted_points))
        selected_indices: set[int] = set()

        def add_nearest(target_s: float, *, from_all: bool = False) -> None:
            candidates_source = list(enumerate(sorted_points)) if from_all else source
            valid = [
                (index, point)
                for index, point in candidates_source
                if index not in selected_indices and as_float(point.get("total_s")) > 0.0
            ]
            if not valid:
                return
            log_target = math.log(max(target_s, 1e-9))
            index, _point = min(
                valid,
                key=lambda item: abs(math.log(max(as_float(item[1].get("total_s")), 1e-9)) - log_target),
            )
            selected_indices.add(index)

        def add_stage_build(target_build_s: float) -> None:
            """Keep visible late cumulative PRM checkpoints when present.

            PRM is a reusable roadmap, so the late curve is important even when
            the incumbent changes only slightly.  Select by encoded build
            checkpoint instead of display time because total_s also includes
            query cost and is not strictly monotone in the roadmap build budget.
            """
            pattern = re.compile(rf"build{re.escape(f'{target_build_s:g}')}s(?:_|$)")
            matches = [
                (index, point)
                for index, point in enumerate(sorted_points)
                if pattern.search(str(point.get("stage_id", "")))
            ]
            if not matches:
                return
            index, _point = min(
                matches,
                key=lambda item: as_float(item[1].get("total_s"), math.inf),
            )
            selected_indices.add(index)

        # Always show the first checkpoint with validated success on the saved query set,
        # the best path point,
        # and the final/biggest-budget checkpoint on the cumulative curve.  The
        # final checkpoint must come from the original sorted curve, not only
        # the Pareto-improving subset; otherwise the visibly flat tail of a
        # cumulative PRM roadmap disappears.
        selected_indices.add(source[0][0])
        best_index, _best_point = min(
            source,
            key=lambda item: as_float(item[1].get("path_length"), math.inf),
        )
        selected_indices.add(best_index)
        selected_indices.add(len(sorted_points) - 1)
        # Keep the last cumulative checkpoints even if they do not improve the
        # incumbent.  This makes the late roadmap saturation/plateau visible
        # instead of visually truncating PRM at its last Pareto improvement.
        for tail_index in range(max(0, len(sorted_points) - 5), len(sorted_points)):
            selected_indices.add(tail_index)

        min_s = min(as_float(point.get("total_s")) for point in sorted_points)
        max_s = max(as_float(point.get("total_s")) for point in sorted_points)
        if math.isfinite(min_s) and math.isfinite(max_s) and min_s > 0.0 and max_s > min_s:
            # Five Pareto-biased interior samples keep the early quality
            # changes readable.
            for step in range(1, 6):
                fraction = step / 6.0
                add_nearest(math.exp(math.log(min_s) * (1.0 - fraction) + math.log(max_s) * fraction))
            # Add a few samples from the complete cumulative time axis, not
            # just the Pareto-improving subset.  PRM often plateaus after the
            # roadmap has saturated; without these late checkpoints the figure
            # falsely suggests the curve stops before the flat tail.
            for fraction in (0.78, 0.88, 0.96):
                add_nearest(
                    math.exp(math.log(min_s) * (1.0 - fraction) + math.log(max_s) * fraction),
                    from_all=True,
                )
        for target_build_s in (2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0, 20.0):
            add_stage_build(target_build_s)
        return [
            sorted_points[index]
            for index in sorted(selected_indices, key=lambda item: sorted_points[item]["total_s"])
        ]

    out: dict[tuple[str, str], dict[str, list[dict[str, Any]]]] = {}
    expected_queries: dict[tuple[str, str], float] = {}
    for row in rows:
        robot = str(row.get("robot", "")).lower()
        difficulty = str(row.get("difficulty", "")).lower()
        if not robot or not difficulty:
            continue
        total = as_float(row.get("total_queries", row.get("scenes")), 0.0)
        if math.isfinite(total) and total > 0.0:
            key = (robot, difficulty)
            expected_queries[key] = max(expected_queries.get(key, 0.0), total)
    for row in rows:
        method = str(row.get("method", ""))
        if method not in {"prm", "bitstar"}:
            continue
        if str(row.get("source", "")) == "per_query_seed_tradeoff_from_manifest":
            continue
        robot = str(row.get("robot", "")).lower()
        difficulty = str(row.get("difficulty", "")).lower()
        if not robot or not difficulty:
            continue
        success = as_float(row.get("success_queries", row.get("success_scenes")), 0.0)
        total = as_float(row.get("total_queries", row.get("scenes")), 0.0)
        plan = random_display_time(row)
        if method == "bitstar":
            # BIT* rows in Exp.6 are anytime checkpoint traces.  The
            # figure should show the checkpoint budget used to obtain that
            # quality, not the median first-solution time of the queries that
            # happened to solve before the checkpoint.  Using the latter
            # collapses late checkpoints into a near-single x coordinate and
            # makes the BIT* curve look like an isolated point.
            checkpoint_s = as_float(row.get("budget_s"))
            if math.isfinite(checkpoint_s) and checkpoint_s > 0.0:
                plan = checkpoint_s
        online = online_query_time(row)
        if not math.isfinite(online):
            online_values = finite_values(row.get("_online_solve_values"))
            online = percentile_value(online_values, 0.50) if online_values else plan
        if method == "prm" and not math.isfinite(online_query_time(row)):
            continue
        path_len = path_length_stat(row)
        expected = expected_queries.get((robot, difficulty), 0.0)
        full_catalog_success = (
            total > 0.0
            and success >= total
            and (expected <= 0.0 or total >= expected)
        )
        # PRM and BIT* are cumulative/anytime traces.  Some checkpoints are
        # stored for only a subset of the saved queries, but they are still
        # useful for showing the full time-quality envelope.  Table/context
        # selection still uses full catalog success through
        # current_random_context_from_rows().
        bitstar_partial_checkpoint = (
            method == "bitstar"
            and total > 0.0
            and success > 0.0
            and (expected <= 0.0 or total >= expected)
        )
        prm_partial_checkpoint = (
            method == "prm"
            and total > 0.0
            and success > 0.0
            and (expected <= 0.0 or total >= expected)
        )
        if (
            total <= 0
            or not (full_catalog_success or bitstar_partial_checkpoint or prm_partial_checkpoint)
            or not math.isfinite(plan)
            or not math.isfinite(path_len)
        ):
            continue
        out.setdefault((robot, difficulty), {}).setdefault(method, []).append(
            {
                "total_s": plan,
                "online_s": online,
                "path_length": path_len,
                "measured_time_s": plan,
                "stage_id": str(row.get("stage_id", "")),
                "full_success": full_catalog_success,
                "success": success,
                "total": total,
                "path_by_label": row.get("_path_by_label") or row.get("path_by_label") or {},
                "path_values": row.get("_path_values") or row.get("path_values") or [],
            }
        )
    # Some cumulative PRM checkpoints are stored as one row per saved scene
    # (10 queries each) rather than one scenario-level 100-query summary row.
    # Aggregate those rows for the figure so the cumulative roadmap curve keeps
    # its late, usually flat, high-build tail.  This is display-only; table
    # context selection still uses scenario-level rows with validated success on
    # the saved query set.
    prm_groups: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in rows:
        if str(row.get("method", "")) != "prm":
            continue
        robot = str(row.get("robot", "")).lower()
        difficulty = str(row.get("difficulty", "")).lower()
        stage_id = str(row.get("stage_id", ""))
        if not robot or not difficulty or not stage_id:
            continue
        expected = expected_queries.get((robot, difficulty), 0.0)
        total = as_float(row.get("total_queries", row.get("scenes")), 0.0)
        success = as_float(row.get("success_queries", row.get("success_scenes")), 0.0)
        if expected <= 0.0 or total <= 0.0 or total >= expected or success < total:
            continue
        plan = random_display_time(row)
        online = online_query_time(row)
        path_len = path_length_stat(row)
        if not (math.isfinite(plan) and math.isfinite(online) and math.isfinite(path_len)):
            continue
        prm_groups.setdefault((robot, difficulty, stage_id), []).append(row)
    existing_prm_stages: dict[tuple[str, str], set[str]] = {}
    for key, scenario in out.items():
        existing_prm_stages[key] = {
            str(point.get("stage_id", ""))
            for point in scenario.get("prm", [])
            if str(point.get("stage_id", ""))
        }
    for (robot, difficulty, stage_id), group in prm_groups.items():
        expected = expected_queries.get((robot, difficulty), 0.0)
        success_total = sum(as_float(row.get("success_queries", row.get("success_scenes")), 0.0) for row in group)
        query_total = sum(as_float(row.get("total_queries", row.get("scenes")), 0.0) for row in group)
        if expected <= 0.0 or query_total < expected or success_total < query_total:
            continue
        if stage_id in existing_prm_stages.get((robot, difficulty), set()):
            continue
        plans = [random_display_time(row) for row in group if math.isfinite(random_display_time(row))]
        onlines = [online_query_time(row) for row in group if math.isfinite(online_query_time(row))]
        path_values: list[float] = []
        path_by_label: dict[str, list[float]] = {}
        for row in group:
            values = finite_values(row.get("_path_values") or row.get("path_values"))
            if values:
                path_values.extend(values)
            else:
                scalar = path_length_stat(row)
                if math.isfinite(scalar):
                    path_values.append(scalar)
            labels = row.get("_path_by_label") or row.get("path_by_label") or {}
            if isinstance(labels, dict):
                for label, values_for_label in labels.items():
                    finite = finite_values(values_for_label)
                    if finite:
                        path_by_label.setdefault(str(label), []).extend(finite)
        if not plans or not onlines or not path_values:
            continue
        out.setdefault((robot, difficulty), {}).setdefault("prm", []).append(
            {
                "total_s": percentile_value(plans, 0.50),
                "online_s": percentile_value(onlines, 0.50),
                "path_length": mean(path_values),
                "measured_time_s": percentile_value(plans, 0.50),
                "stage_id": stage_id,
                "full_success": True,
                "success": success_total,
                "total": query_total,
                "path_by_label": path_by_label,
                "path_values": path_values,
            }
        )
    for scenario in out.values():
        for method, points in scenario.items():
            points = sorted(points, key=lambda item: item["total_s"])
            if method == "prm":
                scenario[method] = sparse_cumulative_prm_points(points)
            else:
                scenario[method] = points
    return out


def merged_random_context(rows: list[dict[str, Any]]) -> tuple[dict[tuple[str, str], dict[str, dict[str, float]]], bool]:
    current_context = current_random_context_from_rows(rows)
    if current_context:
        return current_context, True
    merged = {
        key: {method: dict(value) for method, value in methods.items()}
        for key, methods in REGISTERED_EXP06_BASELINE_CONTEXT.items()
    }
    return merged, False


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def find_exp04_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "shelf_leaf_rrt_summary.csv",
        out_dir / "exp04" / "shelf_leaf_rrt_summary.csv",
        out_dir / "exp04_shelf_leaf_rrt" / "shelf_leaf_rrt_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/shelf_leaf_rrt_summary.csv"))
    return matches[0] if matches else None


def find_exp04_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "shelf_leaf_rrt_manifest.json",
        out_dir / "exp04" / "shelf_leaf_rrt_manifest.json",
        out_dir / "exp04_shelf_leaf_rrt" / "shelf_leaf_rrt_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/shelf_leaf_rrt_manifest.json"))
    return matches[0] if matches else None


def find_exp01_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "endpoint_envelope_summary.csv",
        out_dir / "exp01" / "endpoint_envelope_summary.csv",
        out_dir / "exp01_endpoint_envelope" / "endpoint_envelope_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/endpoint_envelope_summary.csv"))
    return matches[0] if matches else None


def find_exp02_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "link_envelope_summary.csv",
        out_dir / "exp02" / "link_envelope_summary.csv",
        out_dir / "exp02_link_envelope" / "link_envelope_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/link_envelope_summary.csv"))
    return matches[0] if matches else None


def generate_exp01_table(path: Path, rows: list[dict[str, Any]]) -> None:
    width_order = ["0.02", "0.05", "0.10", "0.20", "0.30", "0.50"]
    source_order = ["IFK-AA", "HIFK-3", "HIFK-5", "Critical-sample", "Analytical", "MC"]

    def width_key(value: Any) -> str:
        try:
            return f"{float(value):.2f}"
        except (TypeError, ValueError):
            return str(value).strip()

    def source_label(value: Any) -> str:
        return (
            str(value)
            .replace("IFK_AA", "IFK-AA")
            .replace("HIFK_3", "HIFK-3")
            .replace("HIFK_5", "HIFK-5")
            .replace("CritSample", "Critical-sample")
            .replace("_", r"\_")
        )

    selected = {
        (source_label(row.get("source", "")), width_key(row.get("width"))): row
        for row in rows
        if width_key(row.get("width")) in width_order
    }
    analytical_volume = {
        width: as_float(selected.get(("Analytical", width), {}).get("volume_m3_median"))
        for width in width_order
    }

    def metric_cells(source: str, field: str, *, fixed: int | None = None, digits: int | None = None) -> str:
        cells: list[str] = []
        for width in width_order:
            row = selected.get((source, width))
            if row is None:
                cells.append("--")
            elif fixed is not None:
                cells.append(tex_fixed(row.get(field), fixed))
            else:
                cells.append(tex_num(row.get(field), digits or 2))
        return " & ".join(cells)

    def volume_ratio_cells(source: str) -> str:
        cells: list[str] = []
        for width in width_order:
            row = selected.get((source, width))
            ref = analytical_volume.get(width, math.nan)
            value = as_float(row.get("volume_m3_median")) if row is not None else math.nan
            if not math.isfinite(value) or not math.isfinite(ref) or ref <= 0.0:
                cells.append("--")
            else:
                cells.append(f"{value / ref:.2f}")
        return " & ".join(cells)

    def gap_mm_cells(source: str) -> str:
        cells: list[str] = []
        for width in width_order:
            row = selected.get((source, width))
            if row is None:
                cells.append("--")
                continue
            if source in {"IFK-AA", "HIFK-3", "HIFK-5"}:
                cells.append("--")
                continue
            gap_mm = 1000.0 * as_float(row.get("max_negative_gap"))
            if not math.isfinite(gap_mm):
                cells.append("--")
            else:
                cells.append(tex_fixed(gap_mm, 2))
        return " & ".join(cells)

    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Endpoint-envelope evaluation over joint-interval side lengths (radians)}",
        r"\label{tab:tro-endpoint-envelope}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{1.4pt}",
        r"\renewcommand{\arraystretch}{0.86}",
        r"\resizebox{\textwidth}{!}{%",
        r"\begin{tabular}{@{}l*{6}{r}@{\hspace{0.55em}}*{6}{r}@{\hspace{0.55em}}*{6}{r}@{}}",
        r"\toprule",
        r" & \multicolumn{6}{c}{Volume ratio} & \multicolumn{6}{c}{Time ($\mu$s)} & \multicolumn{6}{c}{Gap (mm)} \\",
        r"\cmidrule(lr){2-7}\cmidrule(lr){8-13}\cmidrule(l){14-19}",
        r"Source & 0.02 & 0.05 & 0.10 & 0.20 & 0.30 & 0.50 & 0.02 & 0.05 & 0.10 & 0.20 & 0.30 & 0.50 & 0.02 & 0.05 & 0.10 & 0.20 & 0.30 & 0.50 \\",
        r"\midrule",
    ]
    for source in source_order:
        if not any((source, width) in selected for width in width_order):
            continue
        label = "Monte Carlo" if source == "MC" else source
        volume = volume_ratio_cells(source)
        time_us = metric_cells(source, "endpoint_us_median", digits=2)
        gap = gap_mm_cells(source)
        lines.append(f"{label} & {volume} & {time_us} & {gap} \\\\")
    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"}",
        r"\par\vspace{0.1ex}",
        r"{\footnotesize\emph{Notes:} 1000 boxes/width; medians; volume ratios use Analytical as the denominator. Gap is the negative largest per-axis width shortfall to the per-box reference extent hull; more negative is worse. Only IFK-AA and HIFK are candidates for the theoretical conservative branch.\par}",
        r"\par\endgroup",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp02_table(path: Path, rows: list[dict[str, Any]]) -> None:
    width_order = ["0.02", "0.05", "0.10", "0.20", "0.30", "0.50"]
    envelope_order = ["Link AABB", "SupportHull"]

    def width_key(value: Any) -> str:
        try:
            return f"{float(value):.2f}"
        except (TypeError, ValueError):
            return str(value).strip()

    def envelope_label(value: Any) -> str:
        return str(value).replace("LinkIAABB", "Link AABB").replace("_", r"\_")

    def tex_sig_num(value: Any, sig: int = 3) -> str:
        if value is None:
            return "--"
        if isinstance(value, str) and value.strip() == "":
            return "--"
        try:
            x = float(value)
        except (TypeError, ValueError):
            return str(value)
        if x != x:
            return "--"
        return f"{x:.{sig}g}"

    selected = {
        (envelope_label(row.get("envelope", "")), width_key(row.get("width"))): row
        for row in rows
        if width_key(row.get("width")) in width_order
    }

    def metric_cells(envelope: str, field: str, digits: int) -> str:
        cells: list[str] = []
        for width in width_order:
            row = selected.get((envelope, width))
            cells.append("--" if row is None else tex_num(row.get(field), digits))
        return " & ".join(cells)

    def volume_cells(envelope: str) -> str:
        cells: list[str] = []
        for width in width_order:
            row = selected.get((envelope, width))
            cells.append("--" if row is None else tex_sig_num(row.get("volume_m3_mean"), 3))
        return " & ".join(cells)

    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Link-envelope evaluation over joint-interval side lengths (radians)}",
        r"\label{tab:tro-link-envelope}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{1.4pt}",
        r"\renewcommand{\arraystretch}{0.84}",
        r"\resizebox{\textwidth}{!}{%",
        r"\begin{tabular}{@{}l*{6}{r}@{\hspace{0.55em}}*{6}{r}@{\hspace{0.55em}}*{6}{r}@{}}",
        r"\toprule",
        r" & \multicolumn{6}{c}{Core volume (m$^3$)} & \multicolumn{6}{c}{Build ($\mu$s)} & \multicolumn{6}{c}{Test ($\mu$s)} \\",
        r"\cmidrule(lr){2-7}\cmidrule(lr){8-13}\cmidrule(l){14-19}",
        r"Envelope & 0.02 & 0.05 & 0.10 & 0.20 & 0.30 & 0.50 & 0.02 & 0.05 & 0.10 & 0.20 & 0.30 & 0.50 & 0.02 & 0.05 & 0.10 & 0.20 & 0.30 & 0.50 \\",
        r"\midrule",
    ]
    for envelope in envelope_order:
        if not any((envelope, width) in selected for width in width_order):
            continue
        volume = volume_cells(envelope)
        build = metric_cells(envelope, "envelope_us_mean", 3)
        test = metric_cells(envelope, "collision_us_mean", 3)
        lines.append(f"{envelope} & {volume} & {build} & {test} \\\\")
    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"}",
        r"\par\vspace{0.35ex}",
        r"{\footnotesize\emph{Notes:} Means over 1000 boxes/width and five repeats; no early exits. AABB uses overlap; SupportHull uses direct GJK\@; Build excludes endpoint construction.\par}",
        r"\par\endgroup",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def find_exp05_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "shelf_cross_algorithm_summary.csv",
        out_dir / "exp05" / "shelf_cross_algorithm_summary.csv",
        out_dir / "exp05_shelf_cross_algorithm" / "shelf_cross_algorithm_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/shelf_cross_algorithm_summary.csv"))
    return matches[0] if matches else None


def find_exp05_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "shelf_cross_algorithm_manifest.json",
        out_dir / "exp05" / "shelf_cross_algorithm_manifest.json",
        out_dir / "exp05_shelf_cross_algorithm" / "shelf_cross_algorithm_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/shelf_cross_algorithm_manifest.json"))
    return matches[0] if matches else None


def find_exp05_rbf_single_query_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "rbf_single_query_online" / "shelf_rbf_single_query_online_summary.csv",
        out_dir / "rbf_single_query_online" / "shelf_rbf_single_query_online_summary.csv",
        out_dir / "exp05_rbf_single_query_online_b100_fixed1600_a12_step0p08_20260609" / "shelf_rbf_single_query_online_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/shelf_rbf_single_query_online_summary.csv"))
    return matches[-1] if matches else None


def find_exp05_rbf_single_query_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "rbf_single_query_online" / "shelf_rbf_single_query_online_manifest.json",
        out_dir / "rbf_single_query_online" / "shelf_rbf_single_query_online_manifest.json",
        out_dir / "exp05_rbf_single_query_online_b100_fixed1600_a12_step0p08_20260609" / "shelf_rbf_single_query_online_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/shelf_rbf_single_query_online_manifest.json"))
    return matches[-1] if matches else None


def find_exp05_current_baseline_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "current_ompl_baselines_001" / "shelf_cross_algorithm_summary.csv",
        out_dir / "current_ompl_baselines_001" / "shelf_cross_algorithm_summary.csv",
        out_dir / "exp05" / "current_ompl_baselines" / "shelf_cross_algorithm_summary.csv",
        out_dir / "current_ompl_baselines" / "shelf_cross_algorithm_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp05_current_baseline_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "current_ompl_baselines_001" / "shelf_cross_algorithm_manifest.json",
        out_dir / "current_ompl_baselines_001" / "shelf_cross_algorithm_manifest.json",
        out_dir / "exp05" / "current_ompl_baselines" / "shelf_cross_algorithm_manifest.json",
        out_dir / "current_ompl_baselines" / "shelf_cross_algorithm_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp05_prm_optimized_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "current_prm_optimized" / "shelf_cross_algorithm_summary.csv",
        out_dir / "current_prm_optimized" / "shelf_cross_algorithm_summary.csv",
        out_dir / "exp05" / "prm_confirm_build_quality" / "shelf_cross_algorithm_summary.csv",
        out_dir / "prm_confirm_build_quality" / "shelf_cross_algorithm_summary.csv",
        REGISTERED_EXP05_PRM_OPTIMIZED_SUMMARY,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp05_prm_optimized_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "current_prm_optimized" / "shelf_cross_algorithm_manifest.json",
        out_dir / "current_prm_optimized" / "shelf_cross_algorithm_manifest.json",
        out_dir / "exp05" / "prm_confirm_build_quality" / "shelf_cross_algorithm_manifest.json",
        out_dir / "prm_confirm_build_quality" / "shelf_cross_algorithm_manifest.json",
        REGISTERED_EXP05_PRM_OPTIMIZED_MANIFEST,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp05_current_iris_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "current_iris_gcs" / "shelf_cross_algorithm_summary.csv",
        out_dir / "current_iris_gcs" / "shelf_cross_algorithm_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp05_current_iris_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "current_iris_gcs" / "shelf_cross_algorithm_manifest.json",
        out_dir / "current_iris_gcs" / "shelf_cross_algorithm_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp05_bitstar_trace_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "bitstar_trace10_001" / "shelf_cross_algorithm_summary.csv",
        out_dir / "exp05_bitstar_trace10_001" / "shelf_cross_algorithm_summary.csv",
        out_dir / "bitstar_trace10_001" / "shelf_cross_algorithm_summary.csv",
        out_dir / "exp05" / "bitstar_trace10" / "shelf_cross_algorithm_summary.csv",
        out_dir / "exp05_bitstar_trace10" / "shelf_cross_algorithm_summary.csv",
        out_dir / "bitstar_trace10" / "shelf_cross_algorithm_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp05_bitstar_trace_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp05" / "bitstar_trace10_001" / "shelf_cross_algorithm_manifest.json",
        out_dir / "exp05_bitstar_trace10_001" / "shelf_cross_algorithm_manifest.json",
        out_dir / "bitstar_trace10_001" / "shelf_cross_algorithm_manifest.json",
        out_dir / "exp05" / "bitstar_trace10" / "shelf_cross_algorithm_manifest.json",
        out_dir / "exp05_bitstar_trace10" / "shelf_cross_algorithm_manifest.json",
        out_dir / "bitstar_trace10" / "shelf_cross_algorithm_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "random_robot_summary.csv",
        out_dir / "exp06" / "random_robot_summary.csv",
        out_dir / "exp06_random_robot" / "random_robot_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/random_robot_summary.csv"))
    return matches[0] if matches else None


def find_exp06_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "random_robot_manifest.json",
        out_dir / "exp06" / "random_robot_manifest.json",
        out_dir / "exp06_random_robot" / "random_robot_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/random_robot_manifest.json"))
    return matches[0] if matches else None


def find_exp06_rbf_registered_override_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / EXP06_REGISTERED_RBF_PROFILE_NAME / "random_robot_summary.csv",
        out_dir / "exp06" / EXP06_REGISTERED_RBF_PROFILE_NAME / "random_robot_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_rbf_registered_override_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / EXP06_REGISTERED_RBF_PROFILE_NAME / "random_robot_manifest.json",
        out_dir / "exp06" / EXP06_REGISTERED_RBF_PROFILE_NAME / "random_robot_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_current_baseline_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "rrtconnect_q10x10_saved_catalog_perquery" / "random_robot_summary.csv",
        out_dir / "rrtconnect_q10x10_saved_catalog_perquery" / "random_robot_summary.csv",
        out_dir / "exp06" / "current_ompl_baselines" / "random_robot_summary.csv",
        out_dir / "current_ompl_baselines" / "random_robot_summary.csv",
        out_dir / "exp06" / "current_ompl_baselines_001" / "random_robot_summary.csv",
        out_dir / "current_ompl_baselines_001" / "random_robot_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_current_baseline_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "rrtconnect_q10x10_saved_catalog_perquery" / "random_robot_manifest.json",
        out_dir / "rrtconnect_q10x10_saved_catalog_perquery" / "random_robot_manifest.json",
        out_dir / "exp06" / "current_ompl_baselines" / "random_robot_manifest.json",
        out_dir / "current_ompl_baselines" / "random_robot_manifest.json",
        out_dir / "exp06" / "current_ompl_baselines_001" / "random_robot_manifest.json",
        out_dir / "current_ompl_baselines_001" / "random_robot_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_ompl_curve_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv",
        out_dir / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv",
        out_dir / "exp06" / "ompl_prm_cumulative_20s_sparse_noearly_v1" / "random_robot_summary.csv",
        out_dir / "ompl_prm_cumulative_20s_sparse_noearly_v1" / "random_robot_summary.csv",
        out_dir / "exp06" / "ompl_tradeoff_curves" / "random_robot_summary.csv",
        out_dir / "ompl_tradeoff_curves" / "random_robot_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_ompl_curve_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_manifest.json",
        out_dir / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_manifest.json",
        out_dir / "exp06" / "ompl_prm_cumulative_20s_sparse_noearly_v1" / "random_robot_manifest.json",
        out_dir / "ompl_prm_cumulative_20s_sparse_noearly_v1" / "random_robot_manifest.json",
        out_dir / "exp06" / "ompl_tradeoff_curves" / "random_robot_manifest.json",
        out_dir / "ompl_tradeoff_curves" / "random_robot_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_ompl_curve_supplements(out_dir: Path) -> list[tuple[Path, Path | None]]:
    combined = out_dir / "exp06" / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv"
    combined_flat = out_dir / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv"
    if combined.exists() or combined_flat.exists():
        return []
    candidates = [
        (
            out_dir / "exp06" / "ompl_tradeoff_curves_prm60_iiwa_hard" / "random_robot_summary.csv",
            out_dir / "exp06" / "ompl_tradeoff_curves_prm60_iiwa_hard" / "random_robot_manifest.json",
        ),
        (
            out_dir / "ompl_tradeoff_curves_prm60_iiwa_hard" / "random_robot_summary.csv",
            out_dir / "ompl_tradeoff_curves_prm60_iiwa_hard" / "random_robot_manifest.json",
        ),
    ]
    out: list[tuple[Path, Path | None]] = []
    for summary, manifest in candidates:
        if summary.exists():
            out.append((summary, manifest if manifest.exists() else None))
    return out


def find_exp06_bitstar_trace_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv",
        out_dir / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv",
        out_dir / "exp06" / "bitstar_trace05_001" / "random_robot_summary.csv",
        out_dir / "exp06_bitstar_trace05_001" / "random_robot_summary.csv",
        out_dir / "bitstar_trace05_001" / "random_robot_summary.csv",
        out_dir / "exp06" / "bitstar_trace05" / "random_robot_summary.csv",
        out_dir / "exp06_bitstar_trace05" / "random_robot_summary.csv",
        out_dir / "bitstar_trace05" / "random_robot_summary.csv",
        out_dir / "exp06" / "bitstar_trace10" / "random_robot_summary.csv",
        out_dir / "exp06_bitstar_trace10" / "random_robot_summary.csv",
        out_dir / "bitstar_trace10" / "random_robot_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_bitstar_trace_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_manifest.json",
        out_dir / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_manifest.json",
        out_dir / "exp06" / "bitstar_trace05_001" / "random_robot_manifest.json",
        out_dir / "exp06_bitstar_trace05_001" / "random_robot_manifest.json",
        out_dir / "bitstar_trace05_001" / "random_robot_manifest.json",
        out_dir / "exp06" / "bitstar_trace05" / "random_robot_manifest.json",
        out_dir / "exp06_bitstar_trace05" / "random_robot_manifest.json",
        out_dir / "bitstar_trace05" / "random_robot_manifest.json",
        out_dir / "exp06" / "bitstar_trace10" / "random_robot_manifest.json",
        out_dir / "exp06_bitstar_trace10" / "random_robot_manifest.json",
        out_dir / "bitstar_trace10" / "random_robot_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_bitstar_trace_supplements(out_dir: Path) -> list[tuple[Path, Path | None]]:
    combined = out_dir / "exp06" / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv"
    combined_flat = out_dir / "ompl_prm_bitstar_60s_quality_stall_v2" / "random_robot_summary.csv"
    if combined.exists() or combined_flat.exists():
        return []
    quality_candidates = [
        (
            out_dir / "exp06" / "bitstar_trace60_iiwa_quality" / "random_robot_summary.csv",
            out_dir / "exp06" / "bitstar_trace60_iiwa_quality" / "random_robot_manifest.json",
        ),
        (
            out_dir / "bitstar_trace60_iiwa_quality" / "random_robot_summary.csv",
            out_dir / "bitstar_trace60_iiwa_quality" / "random_robot_manifest.json",
        ),
    ]
    quality_out: list[tuple[Path, Path | None]] = []
    for summary, manifest in quality_candidates:
        if summary.exists():
            quality_out.append((summary, manifest if manifest.exists() else None))
    if quality_out:
        return quality_out

    candidates = [
        (
            out_dir / "exp06" / "bitstar_trace60_iiwa" / "random_robot_summary.csv",
            out_dir / "exp06" / "bitstar_trace60_iiwa" / "random_robot_manifest.json",
        ),
        (
            out_dir / "bitstar_trace60_iiwa" / "random_robot_summary.csv",
            out_dir / "bitstar_trace60_iiwa" / "random_robot_manifest.json",
        ),
    ]
    out: list[tuple[Path, Path | None]] = []
    for summary, manifest in candidates:
        if summary.exists():
            out.append((summary, manifest if manifest.exists() else None))
    return out


def find_exp06_iris_summaries(out_dir: Path) -> list[Path]:
    candidates = [
        out_dir / "exp06" / "current_iris_gcs" / "random_robot_iris_gcs_summary.csv",
        out_dir / "current_iris_gcs" / "random_robot_iris_gcs_summary.csv",
    ]
    out: list[Path] = []
    seen: set[Path] = set()
    for candidate in candidates:
        if candidate.exists() and candidate not in seen:
            out.append(candidate)
            seen.add(candidate)
    return out


def find_supporting_import_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "supporting_table_import_manifest.json",
        out_dir / "supporting_imports" / "supporting_table_import_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/supporting_table_import_manifest.json"))
    return matches[0] if matches else None


def load_json_file(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {}
    try:
        size = path.stat().st_size
    except OSError:
        size = 0
    # Exp.6 cumulative PRM/BIT* traces intentionally keep per-query checkpoint
    # data so Table VI/Fig.5 can use the same exact scene-query references as
    # RBF and RRT-Connect. The complete labels include robot, difficulty, scene
    # seed, and query index. Those manifests are large
    # (currently about 1.4 GB), but silently skipping them falls back to
    # scene-level scalar path means and makes the baseline path quality
    # incomparable.  Keep a hard cap to avoid accidental multi-GB scratch
    # files, but allow the registered trace manifest.
    max_bytes = 2 * 1024 * 1024 * 1024
    if size > max_bytes:
        return {
            "status": "skipped_large_manifest",
            "_skipped_path": str(path),
            "_skipped_bytes": size,
            "_skip_reason": f"larger_than_{max_bytes}_bytes",
        }
    import json

    return json.loads(path.read_text(encoding="utf-8"))


def collect_shelf_query_path_refs(
    out_dir: Path,
    *,
    extra_manifests: list[dict[str, Any]] | None = None,
) -> dict[str, float]:
    """Collect query-level Shelf+IIWA path references across available methods."""
    refs: dict[str, float] = {}

    def merge_manifest(manifest: dict[str, Any] | None) -> None:
        if not isinstance(manifest, dict):
            return
        manifest_refs = query_path_refs_from_runs(manifest.get("rows", []))
        for query_label, ref in manifest_refs.items():
            update_min_ref(refs, query_label, ref)

    for manifest in extra_manifests or []:
        merge_manifest(manifest)

    registered_iris_payload = load_json_file(REGISTERED_EXP05_IRIS_GCS_ANYTIME)
    if isinstance(registered_iris_payload, dict):
        merge_manifest({"rows": shelf_iris_json_to_run_rows(registered_iris_payload)})

    candidate_paths = [
        find_exp04_manifest(out_dir),
        find_exp05_manifest(out_dir),
        find_exp05_current_baseline_manifest(out_dir),
        find_exp05_prm_optimized_manifest(out_dir),
        find_exp05_current_iris_manifest(out_dir),
        find_exp05_bitstar_trace_manifest(out_dir),
        find_exp05_rbf_single_query_manifest(out_dir),
        REGISTERED_EXP05_RRTCONNECT_MANIFEST if REGISTERED_EXP05_RRTCONNECT_MANIFEST.exists() else None,
        REGISTERED_EXP04_QUERY_MANIFEST if REGISTERED_EXP04_QUERY_MANIFEST.exists() else None,
    ]
    seen: set[Path] = set()
    for path in candidate_paths:
        if path is None:
            continue
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        merge_manifest(load_json_file(path))
    return refs


def manifest_ompl_simplify_time_s(manifest: dict[str, Any]) -> float:
    """Return the declared OMPL final-simplify budget when an artifact records it."""
    if not isinstance(manifest, dict):
        return math.nan
    baseline = manifest.get("baseline_execution")
    if isinstance(baseline, dict):
        value = as_float(baseline.get("ompl_simplify_time_s"))
        if math.isfinite(value):
            return value
    profile = manifest.get("rbf_default_profile")
    if isinstance(profile, dict):
        query = profile.get("query")
        if isinstance(query, dict):
            value = as_float(query.get("final_rrt_simplify_time_s"))
            if math.isfinite(value):
                return value
            value_ms = as_float(query.get("final_rrt_simplify_timeout_ms"))
            if math.isfinite(value_ms):
                return value_ms / 1000.0
    return math.nan


def manifest_uses_required_simplify(manifest: dict[str, Any], target_s: float = 0.01) -> bool:
    value = manifest_ompl_simplify_time_s(manifest)
    return math.isfinite(value) and abs(value - float(target_s)) <= 1e-9


QUERY_ORDER = ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]
QUERY_LABELS = {
    "AS->TS": r"AS$\rightarrow$TS",
    "TS->CS": r"TS$\rightarrow$CS",
    "CS->LB": r"CS$\rightarrow$LB",
    "LB->RB": r"LB$\rightarrow$RB",
    "RB->AS": r"RB$\rightarrow$AS",
}


def finite_or_inf(value: Any) -> float:
    return as_float(value, 1e9)


def full_success(row: dict[str, Any]) -> bool:
    success = int(float(row.get("success_queries", row.get("success_scenes", row.get("success_runs", 0))) or 0))
    total = int(float(row.get("total_queries", row.get("scenes", row.get("runs", 0))) or 0))
    return total > 0 and success == total


def path_length_stat(row: dict[str, Any]) -> float:
    for key in ("path_length_mean", "path_length_median", "route_length_mean", "route_length_median"):
        if key in row:
            value = as_float(row.get(key))
            if math.isfinite(value):
                return value
    return math.nan


def select_tradeoff_row(rows: list[dict[str, Any]], *, budget_field: str = "budget_s") -> dict[str, Any] | None:
    full = [row for row in rows if full_success(row)]
    candidates = full
    finite_path = [
        path_length_stat(row)
        for row in candidates
        if math.isfinite(path_length_stat(row))
    ]
    if finite_path:
        best_path = min(finite_path)
        candidates = [
            row for row in candidates
            if math.isfinite(path_length_stat(row))
            and path_length_stat(row) <= 1.08 * best_path
        ] or candidates
    if not candidates:
        return None
    return sorted(
        candidates,
        key=lambda row: (
            measured_time_key(row),
            path_length_stat(row) if math.isfinite(path_length_stat(row)) else 1e9,
            finite_or_inf(row.get("deep_max_boxes")),
        ),
    )[0]


def first_full_success_row(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    full = [
        row for row in rows
        if full_success(row) and math.isfinite(measured_time_key(row))
    ]
    if not full:
        return None
    return sorted(
        full,
        key=lambda row: (
            measured_time_key(row),
            path_length_stat(row) if math.isfinite(path_length_stat(row)) else 1e9,
        ),
    )[0]


def select_registered_rbf_budget_row(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Select the Exp.4 registered RBF budget for cross-experiment tables."""
    registered = [
        row for row in rows
        if int(float(row.get("deep_max_boxes", -1) or -1)) == DEFAULT_RBF_SHELF_BOX_BUDGET
        and is_full_success(row)
    ]
    if registered:
        return sorted(registered, key=measured_time_key)[0]
    return select_tradeoff_row(rows, budget_field="deep_max_boxes")


def query_stats_from_runs(
    run_rows: list[dict[str, Any]],
    predicate: Any,
) -> dict[str, dict[str, float]]:
    buckets: dict[str, dict[str, list[float]]] = {
        label: {"query_s": [], "path": [], "segment": []}
        for label in QUERY_ORDER
    }
    for row in run_rows:
        if not predicate(row):
            continue
        for query in row.get("queries", []):
            label = str(query.get("label", ""))
            if label not in buckets or not query_success(query):
                continue
            solve_ms = as_float(query.get("solve_ms"))
            if not math.isfinite(solve_ms):
                solve_ms = as_float(query.get("query_ms")) - as_float(query.get("simplify_ms"), 0.0)
            buckets[label]["query_s"].append(solve_ms / 1000.0)
            buckets[label]["path"].append(as_float(query.get("path_length")))
            buckets[label]["segment"].append(as_float(query.get("segment_fraction")))
    out: dict[str, dict[str, float]] = {}
    for label, values in buckets.items():
        out[label] = {
            "query_s": median(values["query_s"]),
            "query_s_values": values["query_s"],
            "path": median(values["path"]),
            "path_mean": mean(values["path"]),
            "path_values": values["path"],
            "segment": median(values["segment"]),
            "segment_values": values["segment"],
        }
    return out


def rbf_registered_batch_query_stats(
    run_rows: list[dict[str, Any]],
    predicate: Any,
) -> dict[str, dict[str, float]]:
    """Per-query path stats with the registered RBF batch online/q timing.

    Exp.4/5 RBF is a reusable two-stage planner.  The per-query objects record
    the final graph lookup time in ``query_ms``, which is intentionally tiny
    after the batch online bridge stage has updated the forest.  Table III
    should report the registered online solve time excluding simplification,
    i.e. the batch online time divided by query count.
    """
    buckets: dict[str, dict[str, list[float]]] = {
        label: {"query_s": [], "path": [], "segment": []}
        for label in QUERY_ORDER
    }
    for row in run_rows:
        if not predicate(row):
            continue
        online_q = online_query_time(row)
        if not math.isfinite(online_q):
            query_count = max(1.0, as_float(row.get("query_count"), 1.0))
            online_q = as_float(row.get("online_solve_s", row.get("query_s"))) / query_count
        for query in row.get("queries", []):
            label = str(query.get("label", ""))
            if label not in buckets or not query_success(query):
                continue
            buckets[label]["query_s"].append(online_q)
            buckets[label]["path"].append(as_float(query.get("path_length")))
            buckets[label]["segment"].append(as_float(query.get("segment_fraction")))
    out: dict[str, dict[str, float]] = {}
    for label, values in buckets.items():
        out[label] = {
            "query_s": median(values["query_s"]),
            "query_s_values": values["query_s"],
            "path": median(values["path"]),
            "path_mean": mean(values["path"]),
            "path_values": values["path"],
            "segment": median(values["segment"]),
            "segment_values": values["segment"],
        }
    return out


def rbf_registered_per_query_online_stats(
    run_rows: list[dict[str, Any]],
    predicate: Any,
) -> dict[str, dict[str, float]]:
    """Per-query RBF stats with query-specific online bridge timing.

    The RBF query entries contain the final partition lookup in ``query_ms``.
    For Table III we instead report each query's online solve cost, which is
    recorded by the query-bridge task diagnostics in query order.
    """
    buckets: dict[str, dict[str, list[float]]] = {
        label: {"query_s": [], "path": [], "segment": []}
        for label in QUERY_ORDER
    }
    for row in run_rows:
        if not predicate(row):
            continue
        batch_online_q = online_query_time(row)
        for query in row.get("queries", []):
            label = str(query.get("label", ""))
            if label not in buckets or not query_success(query):
                continue
            query_index = int(as_float(query.get("query_index"), QUERY_ORDER.index(label)))
            task_ms = as_float(row.get(f"diag_query_bridge_task{query_index}_total_ms"))
            query_s = task_ms / 1000.0 if math.isfinite(task_ms) else batch_online_q
            buckets[label]["query_s"].append(query_s)
            buckets[label]["path"].append(as_float(query.get("path_length")))
            buckets[label]["segment"].append(as_float(query.get("segment_fraction")))
    out: dict[str, dict[str, float]] = {}
    for label, values in buckets.items():
        out[label] = {
            "query_s": median(values["query_s"]),
            "query_s_values": values["query_s"],
            "path": median(values["path"]),
            "path_mean": mean(values["path"]),
            "path_values": values["path"],
            "segment": median(values["segment"]),
            "segment_values": values["segment"],
        }
    return out


def rbf_single_query_online_stats(
    run_rows: list[dict[str, Any]],
) -> dict[str, dict[str, float]]:
    """RBF per-query stats from the single-query online fallback artifact.

    The registered Exp.4 summary can be available without per-query manifest
    rows.  Table III still needs RBF b100 query-level entries, so use the
    single-query online manifest when the registered manifest is empty.
    """
    buckets: dict[str, dict[str, list[float]]] = {
        label: {"query_s": [], "path": [], "segment": []}
        for label in QUERY_ORDER
    }
    for row in run_rows:
        if str(row.get("method")) not in {"", "sbf_leaf_rrt"}:
            continue
        online_s = online_query_time(row)
        queries = row.get("queries", [])
        if queries:
            for query in queries:
                label = str(query.get("label", ""))
                if label not in buckets or not query_success(query):
                    continue
                query_s = online_s if math.isfinite(online_s) else query_solve_seconds(query)
                if math.isfinite(query_s):
                    buckets[label]["query_s"].append(query_s)
                path_len = as_float(query.get("path_length"))
                if math.isfinite(path_len):
                    buckets[label]["path"].append(path_len)
                segment = as_float(query.get("segment_fraction"))
                if math.isfinite(segment):
                    buckets[label]["segment"].append(segment)
            continue

        # Summary-only fallback, used only if a manifest is unavailable.
        label = str(row.get("query_label", ""))
        if label not in buckets or not full_success(row):
            continue
        if math.isfinite(online_s):
            buckets[label]["query_s"].append(online_s)
        path_len = path_length_stat(row)
        if math.isfinite(path_len):
            buckets[label]["path"].append(path_len)
        segment = as_float(row.get("raw_segment_fraction_median"))
        if math.isfinite(segment):
            buckets[label]["segment"].append(segment)

    out: dict[str, dict[str, float]] = {}
    for label, values in buckets.items():
        out[label] = {
            "query_s": median(values["query_s"]),
            "query_s_values": values["query_s"],
            "path": median(values["path"]),
            "path_mean": mean(values["path"]),
            "path_values": values["path"],
            "segment": median(values["segment"]),
            "segment_values": values["segment"],
        }
    return out


def bitstar_first_full_success_query_stats(
    run_rows: list[dict[str, Any]],
) -> tuple[dict[str, dict[str, float]], int, int]:
    bitstar_rows = [
        row for row in run_rows
        if str(row.get("method")) == "bitstar" and row.get("queries")
    ]
    by_stage: dict[str, list[dict[str, Any]]] = {}
    for row in bitstar_rows:
        by_stage.setdefault(str(row.get("stage_id", "")), []).append(row)

    def stage_time(stage_id: str, rows: list[dict[str, Any]]) -> float:
        values = [as_float(row.get("budget_s", row.get("planning_s"))) for row in rows]
        finite = [value for value in values if math.isfinite(value)]
        if finite:
            return median(finite)
        match = re.search(r"_t([0-9.]+)s", stage_id)
        return float(match.group(1)) if match else math.inf

    stats: dict[str, dict[str, float]] = {}
    success_total = 0
    run_total = 0
    ordered_stages = sorted(by_stage.items(), key=lambda item: stage_time(item[0], item[1]))
    for label in QUERY_ORDER:
        selected: dict[str, dict[str, float]] | None = None
        selected_total = 0
        for _stage_id, rows in ordered_stages:
            query_s: list[float] = []
            paths: list[float] = []
            segments: list[float] = []
            ok = 0
            total = 0
            for row in rows:
                for query in row.get("queries", []):
                    if str(query.get("label", "")) != label:
                        continue
                    total += 1
                    if not (bool(query.get("success")) and bool(query.get("audit_passed", True))):
                        continue
                    ok += 1
                    solve_ms = as_float(query.get("solve_ms"))
                    if not math.isfinite(solve_ms):
                        solve_ms = as_float(query.get("query_ms")) - as_float(query.get("simplify_ms"), 0.0)
                    query_s.append(solve_ms / 1000.0)
                    paths.append(as_float(query.get("path_length")))
                    segments.append(as_float(query.get("segment_fraction")))
            if total > 0 and ok == total:
                selected = {
                    "query_s": median(query_s),
                    "query_s_values": query_s,
                    "path": median(paths),
                    "path_mean": mean(paths),
                    "path_values": paths,
                    "segment": median(segments),
                    "segment_values": segments,
                }
                selected_total = total
                break
        if selected is None:
            stats[label] = {"query_s": math.nan, "path": math.nan, "segment": math.nan}
        else:
            stats[label] = selected
            success_total += selected_total
            run_total += selected_total
    return stats, success_total, run_total


def ompl_per_query_tradeoff_stats(
    run_rows: list[dict[str, Any]],
    *,
    method: str,
    path_slack: float = 1.01,
) -> tuple[dict[str, dict[str, float]], int, int]:
    """Select an OMPL checkpoint independently for each shelf query.

    PRM and BIT* converge at different rates on the five shelf queries.  A
    single global checkpoint can therefore misrepresent both online time and
    path quality.  For Table III, select the fastest full-seed-success
    checkpoint for each query whose mean path is within ``path_slack`` of that
    query's best checkpoint with validated success on the saved five-query set.
    """
    method_rows = [
        row for row in run_rows
        if str(row.get("method")) == method and row.get("queries")
    ]
    by_stage: dict[str, list[dict[str, Any]]] = {}
    for row in method_rows:
        by_stage.setdefault(str(row.get("stage_id", "")), []).append(row)

    def stage_time(stage_id: str, rows: list[dict[str, Any]]) -> float:
        values = [as_float(row.get("budget_s", row.get("planning_s"))) for row in rows]
        finite = [value for value in values if math.isfinite(value)]
        if finite:
            return median(finite)
        match = re.search(r"_t([0-9.]+)s", stage_id)
        return float(match.group(1)) if match else math.inf

    stats: dict[str, dict[str, float]] = {}
    success_total = 0
    run_total = 0
    ordered_stages = sorted(by_stage.items(), key=lambda item: stage_time(item[0], item[1]))
    for label in QUERY_ORDER:
        candidates: list[dict[str, Any]] = []
        for stage_id, rows in ordered_stages:
            query_s: list[float] = []
            paths: list[float] = []
            segments: list[float] = []
            builds: list[float] = []
            ok = 0
            total = 0
            for row in rows:
                build_s = as_float(row.get("offline_build_s", row.get("build_s")))
                if math.isfinite(build_s):
                    builds.append(build_s)
                for query in row.get("queries", []):
                    if str(query.get("label", "")) != label:
                        continue
                    total += 1
                    if not (bool(query.get("success")) and bool(query.get("audit_passed", True))):
                        continue
                    ok += 1
                    solve_ms = as_float(query.get("solve_ms"))
                    if not math.isfinite(solve_ms):
                        solve_ms = as_float(query.get("query_ms")) - as_float(query.get("simplify_ms"), 0.0)
                    query_s.append(solve_ms / 1000.0)
                    paths.append(as_float(query.get("path_length")))
                    segments.append(as_float(query.get("segment_fraction")))
            if total > 0 and ok == total and finite_values(paths):
                candidates.append({
                    "stage_id": stage_id,
                    "budget_s": stage_time(stage_id, rows),
                    "build_s": median(builds),
                    "total": total,
                    "query_s": median(query_s),
                    "query_s_values": query_s,
                    "path": median(paths),
                    "path_mean": mean(paths),
                    "path_values": paths,
                    "segment": median(segments),
                    "segment_values": segments,
                })
        if not candidates:
            stats[label] = {"query_s": math.nan, "path": math.nan, "segment": math.nan}
            continue
        best_path = min(
            candidate["path_mean"]
            for candidate in candidates
            if math.isfinite(as_float(candidate.get("path_mean")))
        )
        eligible = [
            candidate for candidate in candidates
            if math.isfinite(as_float(candidate.get("path_mean")))
            and as_float(candidate.get("path_mean")) <= float(path_slack) * best_path
        ] or candidates
        selected = sorted(
            eligible,
            key=lambda candidate: (
                as_float(candidate.get("query_s"), math.inf),
                as_float(candidate.get("path_mean"), math.inf),
                as_float(candidate.get("budget_s"), math.inf),
            ),
        )[0]
        stats[label] = {
            "query_s": selected["query_s"],
            "query_s_values": selected["query_s_values"],
            "path": selected["path"],
            "path_mean": selected["path_mean"],
            "path_values": selected["path_values"],
            "segment": selected["segment"],
            "segment_values": selected["segment_values"],
            "selected_stage_id": selected["stage_id"],
            "selected_budget_s": selected["budget_s"],
            "selected_build_s": selected["build_s"],
        }
        success_total += int(selected.get("total", 0) or 0)
        run_total += int(selected.get("total", 0) or 0)
    return stats, success_total, run_total


def bitstar_per_query_tradeoff_stats(
    run_rows: list[dict[str, Any]],
    *,
    path_slack: float = 1.01,
) -> tuple[dict[str, dict[str, float]], int, int]:
    return ompl_per_query_tradeoff_stats(run_rows, method="bitstar", path_slack=path_slack)


def prm_per_query_tradeoff_stats(
    run_rows: list[dict[str, Any]],
    *,
    path_slack: float = 1.01,
) -> tuple[dict[str, dict[str, float]], int, int]:
    return ompl_per_query_tradeoff_stats(run_rows, method="prm", path_slack=path_slack)


def rbf_single_query_stats_from_summary(
    rows: list[dict[str, Any]],
) -> tuple[dict[str, dict[str, float]], int, int, float]:
    stats: dict[str, dict[str, float]] = {
        label: {"query_s": math.nan, "path": math.nan, "segment": math.nan}
        for label in QUERY_ORDER
    }
    success = 0
    total = 0
    builds: list[float] = []
    for row in rows:
        label = str(row.get("query_label", ""))
        if label not in stats:
            continue
        success += int(float(row.get("success_queries", 0) or 0))
        total += int(float(row.get("total_queries", row.get("runs", 0)) or 0))
        build = as_float(row.get("offline_build_s_median"))
        if math.isfinite(build):
            builds.append(build)
        stats[label] = {
            "query_s": as_float(
                row.get(
                    "online_solve_wall_s_median",
                    row.get("online_per_query_s_median", row.get("online_solve_per_query_s_median")),
                )
            ),
            "query_s_values": finite_values(row.get(
                "online_solve_wall_s_median",
                row.get("online_per_query_s_median", row.get("online_solve_per_query_s_median")),
            )),
            "path": as_float(row.get("path_length_median", row.get("path_length_mean"))),
            "path_values": finite_values(row.get("path_length_median", row.get("path_length_mean"))),
            "segment": as_float(row.get("raw_segment_fraction_median")),
            "segment_values": finite_values(row.get("raw_segment_fraction_median")),
        }
    return stats, success, total, median(builds)


def rbf_single_query_stats_from_manifest(
    manifest: dict[str, Any],
    *,
    deep_boxes: int,
) -> tuple[dict[str, dict[str, Any]], int, int, float]:
    stats: dict[str, dict[str, Any]] = {
        label: {"query_s": math.nan, "query_s_values": [], "path": math.nan, "path_values": [], "segment": math.nan, "segment_values": []}
        for label in QUERY_ORDER
    }
    rows = manifest.get("rows", []) if isinstance(manifest, dict) else []
    builds: list[float] = []
    success = 0
    total = 0
    for row in rows:
        if int(float(row.get("deep_max_boxes", deep_boxes) or deep_boxes)) != deep_boxes:
            continue
        build = as_float(row.get("offline_build_s", row.get("build_s")))
        if math.isfinite(build):
            builds.append(build)
        queries = row.get("queries", [])
        if not queries:
            continue
        query = queries[0]
        label = str(query.get("label", row.get("single_query_label", "")))
        if label not in stats:
            continue
        total += 1
        if not query_success(query):
            continue
        success += 1
        query_s = as_float(row.get("online_solve_wall_s", row.get("online_solve_s", row.get("online_solve_per_query_s"))))
        if not math.isfinite(query_s):
            task_ms = as_float(row.get("diag_query_bridge_task0_total_ms"))
            graph_solve = as_float(row.get("graph_solve_s"), 0.0)
            query_s = task_ms / 1000.0 + graph_solve if math.isfinite(task_ms) else as_float(query.get("solve_ms")) / 1000.0
        stats[label]["query_s_values"].append(query_s)
        stats[label]["path_values"].append(as_float(query.get("path_length")))
        stats[label]["segment_values"].append(as_float(query.get("segment_fraction")))
    for label, item in stats.items():
        item["query_s"] = median(item["query_s_values"])
        item["path"] = median(item["path_values"])
        item["path_mean"] = mean(item["path_values"])
        item["segment"] = median(item["segment_values"])
    return stats, success, total, median(builds)


def format_method_header(
    label: str,
    build_s: Any | None = None,
    sr: str | None = None,
    *,
    simplify_s: Any | None = None,
    time_label: str = "Build",
    build_note: str | None = None,
) -> str:
    details: list[str] = []
    build = as_float(build_s)
    if math.isfinite(build):
        details.append(rf"{time_label} {tex_num(build, 3)}\,s")
    elif build_note:
        details.append(build_note)
    simplify = as_float(simplify_s)
    if math.isfinite(simplify):
        details.append(rf"Simp {tex_num(simplify, 3)}\,s")
    if details:
        detail_lines = r"\\[-0.2ex]".join(
            rf"{{\normalfont\fontsize{{6.6}}{{7.1}}\selectfont {detail}}}"
            for detail in details
        )
        return rf"\textbf{{\footnotesize \shortstack{{{label}\\[-0.2ex]{detail_lines}}}}}"
    return rf"\textbf{{\footnotesize \shortstack{{{label}}}}}"


def grouped_query_table(
    path: Path,
    *,
    caption: str,
    label: str,
    methods: list[dict[str, Any]],
    notes: str | None = None,
    include_segment: bool = False,
    time_unit: str = "s",
    normalize_path_by_query: bool = False,
    show_time_tail: bool = False,
    path_refs_override: dict[str, float] | None = None,
    tabcolsep: str = "1.55pt",
    arraystretch: str = "0.96",
    extrarowheight: str | None = None,
    notes_font: str = r"\scriptsize",
    notes_vspace: str = "0.25ex",
    compact_time_path_cells: bool = False,
    stacked_iqr_cells: bool = False,
    ) -> None:
    time_scale = 1000.0 if time_unit == "ms" else 1.0
    per_method_cols = 1 if compact_time_path_cells else (3 if include_segment else 2)
    group_spec = "c" if compact_time_path_cells else ("rrr" if include_segment else "rr")
    spaced_groups: list[str] = []
    for method_index, _method in enumerate(methods):
        if method_index:
            spaced_groups.append(r"@{\hspace{0.45em}}")
        spaced_groups.append(group_spec)
    colspec = "@{}l" + "".join(spaced_groups) + "@{}"
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        rf"\captionof{{table}}{{{caption}}}",
        rf"\label{{{label}}}",
        r"\footnotesize",
        rf"\setlength{{\tabcolsep}}{{{tabcolsep}}}",
        rf"\renewcommand{{\arraystretch}}{{{arraystretch}}}",
        rf"\setlength{{\extrarowheight}}{{{extrarowheight}}}" if extrarowheight else "",
        r"\resizebox{\textwidth}{!}{%",
        rf"\begin{{tabular}}{{{colspec}}}",
        r"\toprule",
        "  & "
        + " & ".join(
            rf"\multicolumn{{{per_method_cols}}}{{c}}{{{format_method_header(str(method['label']), method.get('build_s'), None, simplify_s=method.get('simplify_s'), time_label=str(method.get('time_label', 'Build')), build_note=method.get('build_note'))}}}"
            for method in methods
        )
        + r" \\",
    ]
    cmid = []
    start = 2
    for _method in methods:
        end = start + per_method_cols - 1
        cmid.append(rf"\cmidrule(lr){{{start}-{end}}}")
        start = end + 1
    lines.append("".join(cmid))
    default_time_metric = rf"$T$ ({time_unit})"
    metric_cols = []
    for method in methods:
        time_metric = str(method.get("time_metric_label") or default_time_metric)
        path_metric = r"$L_q/L_{\mathrm{ref},q}$" if normalize_path_by_query else "$L$"
        if compact_time_path_cells:
            metric_cols.append(rf"\shortstack{{{time_metric}\\[-0.2ex]{path_metric}}}")
        else:
            metric_cols.append(f"{time_metric} & {path_metric} & Seg." if include_segment else f"{time_metric} & {path_metric}")
    lines.append("Query & " + " & ".join(metric_cols) + r" \\")
    lines.append(r"\midrule")
    path_refs: dict[str, float] = {}
    if normalize_path_by_query:
        path_refs = dict(path_refs_override or {})
        fallback_refs = query_path_refs_from_methods(methods)
        for query_label, ref in fallback_refs.items():
            current = path_refs.get(query_label, math.nan)
            if not math.isfinite(current) or ref < current:
                path_refs[query_label] = ref
    for query_label in QUERY_ORDER:
        query_cell = QUERY_LABELS[query_label]
        if stacked_iqr_cells:
            query_cell = rf"\shortstack{{{query_cell}\\[-0.2ex]{{\scriptsize\strut}}}}"
        cells = [query_cell]
        for method in methods:
            stats = method.get("queries", {}).get(query_label, {})
            time_values = finite_values(stats.get("query_s_values"))
            if time_values:
                scaled = [value * time_scale for value in time_values]
                time_cell = (
                    tex_iqr_stacked(scaled, 3)
                    if stacked_iqr_cells else
                    (tex_time_tail(scaled, 3) if show_time_tail else tex_iqr(scaled, 3))
                )
            else:
                fallback_time = as_float(stats.get("query_s")) * time_scale
                time_cell = (
                    tex_iqr_stacked(fallback_time, 3)
                    if stacked_iqr_cells else
                    (tex_time_tail(fallback_time, 3) if show_time_tail else tex_iqr(fallback_time, 3))
                )
            path_values = stats.get("path_values", stats.get("path"))
            if normalize_path_by_query:
                normalized_path = normalized_gap_values(path_values, path_refs.get(query_label, math.nan))
                path_cell = tex_iqr_stacked(normalized_path, 2) if stacked_iqr_cells else tex_iqr(normalized_path, 2)
            else:
                path_cell = tex_iqr_stacked(path_values, 2) if stacked_iqr_cells else tex_iqr(path_values, 2)
            if compact_time_path_cells:
                cells.append(rf"\shortstack{{{time_cell}\\[-0.2ex]{{\scriptsize {path_cell}}}}}")
            else:
                cells.extend([time_cell, path_cell])
            if include_segment:
                cells.append(tex_iqr(stats.get("segment_values", stats.get("segment")), 2))
        lines.append(" & ".join(cells) + r" \\")
    lines.extend([r"\bottomrule", r"\end{tabular}%", r"}"])
    if notes:
        lines.extend([
            rf"\par\vspace{{{notes_vspace}}}",
            rf"{{{notes_font}\emph{{Notes:}} {notes}\par}}",
        ])
    lines.extend([r"\par\endgroup", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def exp04_case_distributions(
    manifest_payload: dict[str, Any],
    *,
    case: str,
    deep_boxes: int,
) -> dict[str, Any]:
    rows = manifest_payload.get("rows", []) if isinstance(manifest_payload, dict) else []
    build_values: list[float] = []
    online_values: list[float] = []
    amortized_values: list[float] = []
    path_values: list[float] = []
    segment_values: list[float] = []
    audit_per_query_values: list[float] = []
    build_final_boxes_values: list[float] = []
    final_boxes_values: list[float] = []
    final_segment_edges_values: list[float] = []
    box_contained_success = 0
    query_count = 0
    path_by_label: dict[str, list[float]] = {}
    for run in rows:
        if str(run.get("case")) != case:
            continue
        if int(float(run.get("deep_max_boxes", deep_boxes) or deep_boxes)) != deep_boxes:
            continue
        for key, target in (
            ("build_final_boxes", build_final_boxes_values),
            ("final_boxes", final_boxes_values),
            ("final_segment_edges", final_segment_edges_values),
            ("segment_edges", final_segment_edges_values),
        ):
            value = as_float(run.get(key))
            if math.isfinite(value):
                target.append(value)
                if key == "final_segment_edges":
                    break
        audit_s = as_float(run.get("audit_s"))
        query_records = run.get("queries", [])
        run_query_count = len(query_records) if query_records else as_float(run.get("queries_per_scene"))
        if math.isfinite(audit_s) and math.isfinite(run_query_count) and run_query_count > 0:
            audit_per_query_values.append(audit_s / run_query_count)
        build = as_float(run.get("offline_build_s", run.get("build_s")))
        if math.isfinite(build):
            build_values.append(build)
        online_q = online_query_time(run)
        run_query_online_values: list[float] = []
        for query in query_records:
            query_count += 1
            if not query_success(query):
                continue
            label = str(query.get("label", ""))
            query_index = int(as_float(query.get("query_index"), QUERY_ORDER.index(label) if label in QUERY_ORDER else -1))
            task_ms = as_float(run.get(f"diag_query_bridge_task{query_index}_total_ms"))
            if math.isfinite(task_ms):
                query_online_s = task_ms / 1000.0
            else:
                query_online_s = online_q
            if math.isfinite(query_online_s):
                run_query_online_values.append(query_online_s)
            if (
                as_float(query.get("segment_edges_used"), 0.0) == 0.0
                and as_float(query.get("segment_fraction"), 0.0) == 0.0
            ):
                box_contained_success += 1
            path_len = as_float(query.get("path_length"))
            if math.isfinite(path_len):
                path_values.append(path_len)
                if label:
                    path_by_label.setdefault(label, []).append(path_len)
            segment = as_float(query.get("segment_fraction"))
            if math.isfinite(segment):
                segment_values.append(segment)
        if run_query_online_values:
            online_values.extend(run_query_online_values)
            if math.isfinite(build):
                amortized_values.extend([build / 5.0 + value for value in run_query_online_values])
        else:
            if math.isfinite(online_q):
                online_values.append(online_q)
            amortized = as_float(run.get("amortized_s_k5"))
            if math.isfinite(amortized):
                amortized_values.append(amortized)
    return {
        "build_values": build_values,
        "online_values": online_values,
        "amortized_values": amortized_values,
        "path_values": path_values,
        "path_by_label": path_by_label,
        "segment_values": segment_values,
        "audit_per_query_values": audit_per_query_values,
        "build_final_boxes_values": build_final_boxes_values,
        "final_boxes_values": final_boxes_values,
        "final_segment_edges_values": final_segment_edges_values,
        "box_contained_success": box_contained_success,
        "query_count": query_count,
    }


def generate_exp04_table(
    path: Path,
    rows: list[dict[str, Any]],
    manifest_payload: dict[str, Any] | None = None,
    *,
    global_path_refs: dict[str, float] | None = None,
) -> None:
    labels = {
        "baseline_d23_aafk_support_hull_8t": r"\rbf{} baseline",
        "critsample_d23_cache": "Critical-sample",
        "no_cache_full_root_ts": "Full-root",
        "critsample_support_hull": "Critical-sample",
        "critsample_support_hull_unsafe": "Critical-sample",
        "no_external_lect": "HIFK-5/no cache",
        "support_hull_no_aabb": "SupportHull-only",
        "link_aabb": "Link-AABB",
        "single_thread": "1 thread",
    }
    table_rows = [
        row
        for row in rows
        if int(float(row.get("deep_max_boxes", -1) or -1)) == DEFAULT_RBF_SHELF_BOX_BUDGET
    ]
    if not table_rows:
        table_rows = rows
    table_rows = [
        row for row in table_rows
        if full_success(row)
    ]
    distributions: dict[int, dict[str, Any]] = {}
    for index, row in enumerate(table_rows):
        case = str(row.get("case", ""))
        deep_boxes = int(float(row.get("deep_max_boxes", DEFAULT_RBF_SHELF_BOX_BUDGET) or DEFAULT_RBF_SHELF_BOX_BUDGET))
        distributions[index] = exp04_case_distributions(
            manifest_payload or {},
            case=case,
            deep_boxes=deep_boxes,
        )

    path_refs = dict(global_path_refs or {})
    local_path_refs = query_path_refs_from_runs(
        (manifest_payload or {}).get("rows", []) if isinstance(manifest_payload, dict) else [],
        lambda run: str(run.get("case", "")) in labels,
    )
    for query_label, value in local_path_refs.items():
        update_min_ref(path_refs, query_label, value)
    for dist in distributions.values():
        for query_label, values in dist.get("path_by_label", {}).items():
            update_min_ref(path_refs, str(query_label), values)
    scalar_path_ref = min(
        [
            path_length_stat(row)
            for row in table_rows
            if math.isfinite(path_length_stat(row))
        ],
        default=math.nan,
    )

    def dist_or_scalar(dist: dict[str, Any], key: str, scalar: Any) -> Any:
        values = finite_values(dist.get(key))
        return values if values else scalar

    def gap_cell(dist: dict[str, Any], scalar_path: Any) -> str:
        path_by_label = dist.get("path_by_label", {})
        if isinstance(path_by_label, dict) and path_by_label:
            gaps = normalized_gap_from_label_values(path_by_label, path_refs)
            if gaps:
                return tex_iqr(gaps, 2)
        return tex_iqr(normalized_gap_values(dist.get("path_values") or scalar_path, scalar_path_ref), 2)

    def cache_cell(row: dict[str, Any]) -> str:
        hits = as_float(row.get("external_reused_hits_median", row.get("external_hits_median")))
        return "d23 replay" if math.isfinite(hits) and hits > 0.0 else "live mat."

    def box_contained_cell(dist: dict[str, Any]) -> str:
        success = int(dist.get("box_contained_success", 0) or 0)
        total = int(dist.get("query_count", 0) or 0)
        return f"{success}/{total}" if total > 0 else "--"

    def boxes_cell(dist: dict[str, Any], row: dict[str, Any]) -> str:
        build_boxes = dist_or_scalar(dist, "build_final_boxes_values", row.get("build_final_boxes_median"))
        final_boxes = dist_or_scalar(dist, "final_boxes_values", row.get("final_boxes_median"))
        return f"{tex_median_int(build_boxes)}/{tex_median_int(final_boxes)}"

    def segment_witness_cell(dist: dict[str, Any], row: dict[str, Any]) -> str:
        segment_fraction = dist_or_scalar(dist, "segment_values", row.get("raw_segment_fraction_median"))
        segment_edges = dist_or_scalar(
            dist,
            "final_segment_edges_values",
            row.get("final_segment_edges_median", row.get("segment_edges_median")),
        )
        return f"{tex_median_scalar(segment_fraction, 2)}/{tex_median_int(segment_edges)}"

    def audit_per_query_cell(dist: dict[str, Any], row: dict[str, Any]) -> str:
        values = finite_values(dist.get("audit_per_query_values"))
        if values:
            return tex_iqr_stacked(values, 3)
        audit_s = as_float(row.get("audit_s_median"))
        queries = as_float(row.get("queries_per_scene"), 5.0)
        if math.isfinite(audit_s) and math.isfinite(queries) and queries > 0.0:
            return tex_num(audit_s / queries, 3)
        return "--"

    replay_hit_values = [
        value for value in (
            as_float(row.get("external_reused_hits_median", row.get("external_hits_median")))
            for row in table_rows
        )
        if math.isfinite(value) and value > 0.0
    ]
    replay_hit_note = tex_int_commas(percentile_value(replay_hit_values, 0.50)) if replay_hit_values else "--"

    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Shelf+IIWA \rbf{} warm-d23 baseline and registered profile controls}",
        r"\label{tab:tro-shelf-ablation}",
        r"\footnotesize",
        r"\setlength{\tabcolsep}{1.8pt}",
        r"\renewcommand{\arraystretch}{1.25}",
        r"\begin{tabular}{@{}lccc@{}}",
        r"\toprule",
        r"Case & Build & Batch timestamp & $L_q/L_{\mathrm{ref},q}$ \\",
        r"\midrule",
    ]
    for index, row in enumerate(table_rows):
        dist = distributions.get(index, {})
        label = labels.get(str(row.get("case", "")), str(row.get("case", ""))).replace("_", r"\_")
        lines.append(
            f"{label} & "
            f"{tex_iqr(dist_or_scalar(dist, 'build_values', row.get('offline_build_s_median', row.get('build_s_median', row.get('build_s')))), 3)} & "
            f"{tex_iqr(dist_or_scalar(dist, 'online_values', online_query_time(row)), 3)} & "
            f"{gap_cell(dist, path_length_stat(row))} \\\\"
        )
    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\par\vspace{0.1ex}",
        r"{\footnotesize\emph{Notes:} Seconds; medians [25\%, 75\%]; b100, 8 seeds. Batch timestamp: 40 within-batch completion times, not single-query latency. Baseline: IFK-AA+d23+Link-AABB+SupportHull/GJK\@. Registered caches: none (HIFK-5); d23 (Critical-sample); IFK-AA+d23 (Link-AABB and SupportHull-only). Critical-sample is non-certifying; its outputs remain validation-required.\par}",
        r"\par\endgroup",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp04_query_table(path: Path, rows: list[dict[str, Any]], manifest: dict[str, Any]) -> None:
    labels = {
        "baseline_d23_aafk_support_hull_8t": r"\rbf{} baseline",
        "critsample_d23_cache": "Critical sample",
        "critsample_support_hull": "Critical sample",
        "critsample_support_hull_unsafe": "Critical sample",
        "link_aabb": "Link AABB",
        "no_external_lect": "HIFK-5/no cache",
        "single_thread": "1 thread",
    }
    order = [
        "baseline_d23_aafk_support_hull_8t",
        "critsample_d23_cache",
        "critsample_support_hull_unsafe",
        "critsample_support_hull",
        "link_aabb",
        "no_external_lect",
        "single_thread",
    ]
    run_rows = manifest.get("rows", []) if isinstance(manifest, dict) else []
    methods: list[dict[str, Any]] = []
    for case in order:
        case_rows = [row for row in rows if str(row.get("case")) == case]
        selected = select_tradeoff_row(case_rows, budget_field="deep_max_boxes")
        if selected is None:
            continue
        deep_boxes = int(float(selected.get("deep_max_boxes", 0) or 0))
        runs = int(float(selected.get("runs", 0) or 0))
        success = int(float(selected.get("success_runs", 0) or 0))
        query_stats = query_stats_from_runs(
            run_rows,
            lambda row, case=case, deep_boxes=deep_boxes: (
                str(row.get("case")) == case
                and int(float(row.get("deep_max_boxes", 0) or 0)) == deep_boxes
            ),
        )
        methods.append({
            "label": rf"{labels.get(case, case)} b{deep_boxes}",
            "build_s": selected.get("build_s_median", selected.get("planning_s_median")),
            "sr": f"{success}/{runs}",
            "queries": query_stats,
        })
    grouped_query_table(
        path,
        caption=(
            r"Shelf+IIWA RBF mechanism rows from Fig.~\ref{fig:tro_shelf_tradeoff}, "
            r"shown by query. Each row contributes the design point selected "
            r"from its five-query amortized-time trade-off curve; the full curves remain the primary evidence. "
            r"Query path entries report $L_q/L_{\mathrm{ref},q}$ intervals computed only from successful fixed 0.01~rad validated paths for the same saved Shelf query."
        ),
        label="tab:tro-shelf-ablation",
        methods=methods,
        include_segment=True,
    )


def generate_exp05_table(
    path: Path,
    rows: list[dict[str, Any]],
    *,
    rbf_manifest: dict[str, Any],
    baseline_manifest: dict[str, Any],
    rbf_single_query_summary_rows: list[dict[str, Any]] | None = None,
    rbf_single_query_manifest: dict[str, Any] | None = None,
) -> None:
    order = ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
    def best_rbf_row() -> dict[str, Any] | None:
        rbf_rows = [row for row in rows if str(row.get("method")) == "sbf_leaf_rrt"]
        return select_registered_rbf_budget_row(rbf_rows)

    def selected_method_row(method: str) -> dict[str, Any] | None:
        items = [row for row in rows if str(row.get("method")) == method]
        if method == "prm":
            return select_tradeoff_row(items, budget_field="budget_s")
        full = [
            row for row in items
            if int(float(row.get("success_runs", 0) or 0)) == int(float(row.get("runs", 0) or 0))
        ]
        candidates = full or items
        finite_path = [
            path_length_stat(row)
            for row in candidates
            if math.isfinite(path_length_stat(row))
        ]
        if finite_path:
            best_path = min(finite_path)
            candidates = [
                row for row in candidates
                if math.isfinite(path_length_stat(row))
                and path_length_stat(row) <= 1.08 * best_path
            ] or candidates
        if not candidates:
            return None
        return sorted(
            candidates,
            key=lambda row: (
                measured_time_key(row),
                path_length_stat(row) if math.isfinite(path_length_stat(row)) else 1e9,
            ),
        )[0]

    row_by_method: dict[str, dict[str, Any]] = {}
    for method in order:
        if method == "sbf_leaf_rrt":
            continue
        selected = selected_method_row(method)
        if selected is not None:
            row_by_method[method] = selected
    selected_rbf = best_rbf_row()
    if selected_rbf is not None:
        row_by_method["sbf_leaf_rrt"] = selected_rbf
    labels = {
        "sbf_leaf_rrt": r"\rbf{}",
        "iris_np_gcs": "IRIS/GCS+post",
        "prm": "PRM",
        "rrtconnect": "RRT-Connect",
        "bitstar": "BIT*",
    }
    rbf_runs = rbf_manifest.get("rows", []) if isinstance(rbf_manifest, dict) else []
    rbf_single_runs = (
        rbf_single_query_manifest.get("rows", [])
        if isinstance(rbf_single_query_manifest, dict)
        else []
    )
    baseline_runs = baseline_manifest.get("rows", []) if isinstance(baseline_manifest, dict) else []
    methods: list[dict[str, Any]] = []
    for method in order:
        row = row_by_method.get(method)
        if row is None:
            continue
        runs = int(float(row.get("total_queries", row.get("runs", 0)) or 0))
        success = int(float(row.get("success_queries", row.get("success_runs", 0)) or 0))
        time_metric_label = None
        if method == "sbf_leaf_rrt":
            deep_boxes = int(float(row.get("deep_max_boxes", 0) or 0))
            query_stats = rbf_registered_per_query_online_stats(
                rbf_runs,
                lambda run, deep_boxes=deep_boxes: (
                    (
                        str(run.get("case")) == "baseline_d23_aafk_support_hull_8t"
                        or str(run.get("method")) == "sbf_leaf_rrt"
                    )
                    and int(float(run.get("deep_max_boxes", 0) or 0)) == deep_boxes
                ),
            )
            if not any(finite_values(stats.get("query_s_values")) for stats in query_stats.values()):
                query_stats = rbf_single_query_online_stats(rbf_single_runs)
            time_metric_label = r"Batch timestamp (s)"
            label = rf"{labels[method]} b{deep_boxes}"
            build_s = row.get("offline_build_s_median", row.get("build_s", row.get("planning_s_median")))
            time_label = "Build"
        elif method == "iris_np_gcs":
            stage_id = str(row.get("stage_id", method))
            query_stats = query_stats_from_runs(
                baseline_runs,
                lambda run, stage_id=stage_id: (
                    str(run.get("method")) == "iris_np_gcs" and str(run.get("stage_id", "iris_np_gcs")) == stage_id
                ),
            )
            label = labels[method]
            build_s = math.nan if method in {"rrtconnect", "bitstar"} else row.get("offline_build_s_median", row.get("build_s", row.get("planning_s_median")))
        else:
            build_s = math.nan
            stage_id = str(row.get("stage_id", method))
            if method == "bitstar":
                query_stats, bitstar_success, bitstar_total = bitstar_per_query_tradeoff_stats(baseline_runs)
                if bitstar_total > 0:
                    success = bitstar_success
                    runs = bitstar_total
            elif method == "prm":
                query_stats, prm_success, prm_total = prm_per_query_tradeoff_stats(baseline_runs)
                if prm_total > 0:
                    success = prm_success
                    runs = prm_total
                    selected_builds = [
                        as_float(stats.get("selected_build_s"))
                        for stats in query_stats.values()
                        if math.isfinite(as_float(stats.get("selected_build_s")))
                    ]
                    if selected_builds:
                        build_s = median(selected_builds)
            else:
                query_stats = query_stats_from_runs(
                    baseline_runs,
                    lambda run, method=method, stage_id=stage_id: (
                        str(run.get("method")) == method and str(run.get("stage_id", method)) == stage_id
                    ),
                )
            label = labels[method]
            if method != "prm" or not math.isfinite(as_float(build_s)):
                build_s = math.nan if method in {"rrtconnect", "bitstar"} else row.get("offline_build_s_median", row.get("build_s", row.get("planning_s_median")))
        if method != "sbf_leaf_rrt":
            time_label = "Build"
        methods.append({
            "label": label,
            "build_s": build_s,
            "build_note": "No reusable build" if method in {"rrtconnect", "bitstar"} else None,
            "simplify_s": None,
            "time_label": time_label,
            "time_metric_label": time_metric_label,
            "sr": f"{success}/{runs}",
            "queries": query_stats,
        })
    global_path_refs = query_path_refs_from_runs(rbf_runs + baseline_runs)
    if isinstance(rbf_single_query_manifest, dict):
        single_refs = query_path_refs_from_runs(rbf_single_query_manifest.get("rows", []))
        for query_label, ref in single_refs.items():
            current = global_path_refs.get(query_label, math.nan)
            if not math.isfinite(current) or ref < current:
                global_path_refs[query_label] = ref
    fallback_refs = query_path_refs_from_methods(methods)
    for query_label, ref in fallback_refs.items():
        current = global_path_refs.get(query_label, math.nan)
        if not math.isfinite(current) or ref < current:
            global_path_refs[query_label] = ref
    grouped_query_table(
        path,
        caption=(
            r"Shelf+IIWA comparison at method-specific, in-sample operating points"
        ),
        label="tab:tro-shelf-cross-algorithm",
        methods=methods,
        notes=(
            r"Seconds; medians [25\%, 75\%], eight seeds. "
            r"\rbf{} Batch timestamp: 40 within-batch completions from eight five-query runs; other \(T\) values are per query. "
            r"Times are method-local; postprocessing is excluded."
        ),
        include_segment=False,
        time_unit="s",
        normalize_path_by_query=True,
        show_time_tail=True,
        path_refs_override=global_path_refs,
        tabcolsep="1.6pt",
        arraystretch="1.18",
        extrarowheight="1.2pt",
        notes_font=r"\footnotesize",
        notes_vspace="0.05ex",
        compact_time_path_cells=False,
        stacked_iqr_cells=True,
    )


def generate_exp05_figure(pdf_path: Path, png_path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    configure_matplotlib_for_ieee(matplotlib)
    import matplotlib.pyplot as plt

    method_order = ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
    method_rows = {
        method: sorted(
            [
                row for row in rows
                if str(row.get("method")) == method
                and is_full_success(row)
            ],
            key=lambda row: (measured_time_key(row), finite_or_inf(row.get("deep_max_boxes"))),
        )
        for method in method_order
    }
    selected_rows = {}
    for method in method_order:
        if not method_rows.get(method):
            continue
        if method == "sbf_leaf_rrt":
            selected_rows[method] = select_registered_rbf_budget_row(method_rows[method])
        else:
            selected_rows[method] = select_tradeoff_row(method_rows[method], budget_field="budget_s")

    # This figure is included in a partial-width minipage. Author its text
    # slightly larger so the final IEEE-scale labels do not fall below the
    # surrounding caption size.
    exp05_font_scale = 1.4
    fig, axes = plt.subplots(1, 2, figsize=(7.85, 2.45))
    ax = axes[0]
    path_ref = finite_min([
        path_length_stat(row)
        for items in method_rows.values()
        for row in items
    ])
    path_values: list[float] = []
    for method in method_order:
        items = method_rows.get(method, [])
        if not items:
            continue
        style = METHOD_STYLE.get(method, {"label": method, "color": "0.3", "marker": "o"})
        xs = [method_time(row) for row in items]
        ys = [ratio_to_ref(path_length_stat(row), path_ref) for row in items]
        path_values.extend(ys)
        if len(items) > 1:
            ax.plot(xs, ys, "-", color=style["color"], alpha=0.62, linewidth=LINE_WIDTH)
        ax.scatter(xs, ys, marker=style["marker"], color=style["color"], s=POINT_SIZE, alpha=0.78, label=style["label"])
        selected = selected_rows.get(method)
        first = first_full_success_row(items)
        if first is not None:
            ax.scatter(
                [method_time(first)],
                [ratio_to_ref(path_length_stat(first), path_ref)],
                facecolors="none",
                edgecolors="black",
                linewidths=0.9,
                s=28,
                zorder=4,
            )
        if selected is not None:
            ax.scatter(
                [method_time(selected)],
                [ratio_to_ref(path_length_stat(selected), path_ref)],
                facecolors="none",
                edgecolors="#d4a017",
                linewidths=SELECTED_LINE_WIDTH,
                s=SELECTED_POINT_SIZE,
                zorder=5,
            )
    ax.set_xscale("log")
    ax.set_xlabel("method-specific budget (s)")
    ax.set_ylabel(r"mean-path ratio $\bar L/\bar L_{\mathrm{ref}}$")
    ax.set_title(
        "(a) within-method budget / quality",
        fontsize=PANEL_TITLE_FONTSIZE * exp05_font_scale,
    )
    ax.grid(True, which="both", alpha=0.24)
    ax.tick_params(labelsize=TICK_LABEL_FONTSIZE * exp05_font_scale)
    ax.xaxis.label.set_size(AXIS_LABEL_FONTSIZE * exp05_font_scale)
    ax.yaxis.label.set_size(AXIS_LABEL_FONTSIZE * exp05_font_scale)
    set_padded_linear_ylim(ax, path_values)

    ax = axes[1]
    amortization_methods: list[dict[str, Any]] = []
    for method in method_order:
        row = selected_rows.get(method)
        if row is None:
            continue
        label = str(row.get("method_label") or METHOD_STYLE.get(method, {}).get("label", method))
        build_s = row.get("offline_build_s_median", row.get("build_s", 0.0))
        per_query_s = online_query_time(row)
        if not math.isfinite(per_query_s):
            per_query_s = method_time(row)
        if method in {"rrtconnect", "bitstar"}:
            build_s = 0.0
        amortization_methods.append({
            "method": method,
            "label": label,
            "build_s": build_s,
            "per_query_s": per_query_s,
        })
    plot_query_amortization_panel(
        ax,
        amortization_methods,
        title="(b) reuse-horizon display",
        font_scale=exp05_font_scale,
    )

    handles, labels = axes[0].get_legend_handles_labels()
    dedup: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        dedup.setdefault(label, handle)
    fig.legend(dedup.values(), dedup.keys(), loc="upper center", bbox_to_anchor=(0.5, 0.995),
               ncol=5, frameon=False, fontsize=LEGEND_FONTSIZE * exp05_font_scale)
    fig.subplots_adjust(left=0.105, right=0.99, top=0.73, bottom=0.22, wspace=0.24)
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.01)
    fig.savefig(png_path, dpi=240, bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)


def generate_exp04_figure(
    pdf_path: Path,
    png_path: Path,
    rows: list[dict[str, Any]],
    manifest_payload: dict[str, Any] | None = None,
    *,
    global_path_refs: dict[str, float] | None = None,
) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    configure_matplotlib_for_ieee(matplotlib)
    import matplotlib.pyplot as plt

    case_order = [
        ("baseline_d23_aafk_support_hull_8t", "RBF", "#1f77b4", "o"),
        ("critsample_d23_cache", "Critical sample", "#17becf", "v"),
        ("critsample_support_hull_unsafe", "Critical sample", "#17becf", "v"),
        ("critsample_support_hull", "Critical sample", "#17becf", "v"),
        ("link_aabb", "Link AABB", "#2ca02c", "s"),
        ("no_external_lect", "HIFK-5/no cache", "#ff7f0e", "D"),
        ("single_thread", "1T", "#9467bd", "^"),
    ]
    exp04_font_scale = 1.70
    selected_rows: dict[str, dict[str, Any]] = {}
    fig, ax = plt.subplots(1, 1, figsize=(4.12, 2.16))
    allowed_cases = {case for case, _label, _color, _marker in case_order}
    path_refs = dict(global_path_refs or {})
    local_path_refs = query_path_refs_from_runs(
        (manifest_payload or {}).get("rows", []) if isinstance(manifest_payload, dict) else [],
        lambda run: str(run.get("case", "")) in allowed_cases,
    )
    for query_label, value in local_path_refs.items():
        update_min_ref(path_refs, query_label, value)
    scalar_path_ref = finite_min([
        path_length_stat(row)
        for row in rows
        if str(row.get("case")) in allowed_cases and full_success(row)
    ])
    path_values: list[float] = []

    def row_path_ratio(row: dict[str, Any]) -> float:
        case = str(row.get("case"))
        deep_boxes = int(float(row.get("deep_max_boxes", DEFAULT_RBF_SHELF_BOX_BUDGET) or DEFAULT_RBF_SHELF_BOX_BUDGET))
        dist = exp04_case_distributions(
            manifest_payload or {},
            case=case,
            deep_boxes=deep_boxes,
        )
        path_by_label = dist.get("path_by_label", {})
        if isinstance(path_by_label, dict) and path_by_label and path_refs:
            ratios = normalized_gap_from_label_values(path_by_label, path_refs)
            if ratios:
                return percentile_value(ratios, 0.50)
        ratios = normalized_gap_values(dist.get("path_values") or path_length_stat(row), scalar_path_ref)
        if ratios:
            return percentile_value(ratios, 0.50)
        return ratio_to_ref(path_length_stat(row), scalar_path_ref)

    def row_figure_time(row: dict[str, Any]) -> float:
        case = str(row.get("case"))
        deep_boxes = int(float(row.get("deep_max_boxes", DEFAULT_RBF_SHELF_BOX_BUDGET) or DEFAULT_RBF_SHELF_BOX_BUDGET))
        dist = exp04_case_distributions(
            manifest_payload or {},
            case=case,
            deep_boxes=deep_boxes,
        )
        values = finite_values(dist.get("amortized_values"))
        if values:
            return percentile_value(values, 0.50)
        return method_time(row)

    for case, label, color, marker in case_order:
        items = sorted(
            [row for row in rows if str(row.get("case")) == case and full_success(row)],
            key=row_figure_time,
        )
        if not items:
            continue
        selected = select_tradeoff_row(items, budget_field="deep_max_boxes")
        if selected is not None:
            selected_rows[case] = selected
        xs = [row_figure_time(row) for row in items]
        ys = [row_path_ratio(row) for row in items]
        path_values.extend(ys)
        ax.plot(xs, ys, "-", color=color, alpha=0.62, linewidth=LINE_WIDTH)
        ax.scatter(xs, ys, marker=marker, color=color, s=POINT_SIZE, alpha=0.78, label=label)
        selected_ratio = math.nan if selected is None else row_path_ratio(selected)
        if selected is not None and math.isfinite(selected_ratio):
            ax.scatter(
                [row_figure_time(selected)],
                [selected_ratio],
                facecolors="none",
                edgecolors="#d4a017",
                linewidths=SELECTED_LINE_WIDTH,
                s=SELECTED_POINT_SIZE,
                zorder=5,
            )
    ax.set_xscale("log")
    ax.set_xticks([0.2, 0.4, 0.8])
    ax.set_xticklabels(["0.2", "0.4", "0.8"])
    from matplotlib.ticker import NullFormatter
    ax.xaxis.set_minor_formatter(NullFormatter())
    ax.set_xlabel("Build/5 + batch task (s)")
    ax.set_ylabel(r"path-length ratio $L_q/L_{\mathrm{ref},q}$")
    ax.grid(True, which="both", alpha=0.24)
    ax.tick_params(labelsize=TICK_LABEL_FONTSIZE * exp04_font_scale)
    ax.xaxis.label.set_size(AXIS_LABEL_FONTSIZE * exp04_font_scale)
    ax.yaxis.label.set_size(AXIS_LABEL_FONTSIZE * exp04_font_scale)
    set_padded_linear_ylim(ax, path_values)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 1.03),
               ncol=5, frameon=False,
               fontsize=LEGEND_FONTSIZE * exp04_font_scale,
               handlelength=1.1)
    fig.subplots_adjust(left=0.16, right=0.985, top=0.835, bottom=0.225)
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.01)
    fig.savefig(png_path, dpi=240, bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)


def generate_exp04_assets(generated: Path, out_dir: Path) -> dict[str, Any]:
    summary = find_exp04_summary(out_dir)
    if summary is None:
        return {"status": "missing", "summary": None}
    rows = read_csv_rows(summary)
    manifest_path = find_exp04_manifest(out_dir)
    manifest = load_json_file(manifest_path)
    global_path_refs = collect_shelf_query_path_refs(out_dir, extra_manifests=[manifest])
    table_path = generated / "tab_tro_shelf_ablation.tex"
    pdf_path = generated / "fig_tro_shelf_tradeoff.pdf"
    png_path = generated / "fig_tro_shelf_tradeoff.png"
    generate_exp04_table(table_path, rows, manifest, global_path_refs=global_path_refs)
    generate_exp04_figure(
        pdf_path,
        png_path,
        rows,
        manifest,
        global_path_refs=global_path_refs,
    )
    return {
        "status": "generated",
        "summary": str(summary),
        "summary_sha256": file_sha256(summary),
        "manifest": str(manifest_path) if manifest_path is not None else None,
        "manifest_sha256": file_sha256(manifest_path) if manifest_path is not None else None,
        "global_path_refs": global_path_refs,
        "rows": len(rows),
        "table": str(table_path),
        "figure_pdf": str(pdf_path),
        "figure_png": str(png_path),
    }


def generate_exp05_assets(generated: Path, out_dir: Path) -> dict[str, Any]:
    summary = find_exp05_summary(out_dir)
    if summary is None:
        return {"status": "missing", "summary": None}
    rows = read_csv_rows(summary)
    rbf_manifest_path = find_exp05_manifest(out_dir) or find_exp04_manifest(out_dir)
    rbf_manifest = load_json_file(rbf_manifest_path)
    if isinstance(rbf_manifest, dict):
        for run in rbf_manifest.get("rows", []):
            if str(run.get("method")) == "sbf_leaf_rrt":
                run["case"] = "baseline_d23_aafk_support_hull_8t"
    rbf_query_manifest_path: Path | None = None
    if (
        (not isinstance(rbf_manifest, dict) or not rbf_manifest.get("rows"))
        and REGISTERED_EXP04_QUERY_MANIFEST.exists()
    ):
        rbf_query_manifest_path = REGISTERED_EXP04_QUERY_MANIFEST
        rbf_manifest = load_json_file(rbf_query_manifest_path)
    current_baseline_summary = find_exp05_current_baseline_summary(out_dir)
    current_baseline_manifest = find_exp05_current_baseline_manifest(out_dir)
    prm_optimized_summary = find_exp05_prm_optimized_summary(out_dir)
    prm_optimized_manifest = find_exp05_prm_optimized_manifest(out_dir)
    current_iris_summary = find_exp05_current_iris_summary(out_dir)
    current_iris_manifest = find_exp05_current_iris_manifest(out_dir)
    bitstar_trace_summary = find_exp05_bitstar_trace_summary(out_dir)
    bitstar_trace_manifest = find_exp05_bitstar_trace_manifest(out_dir)
    rbf_single_query_summary = find_exp05_rbf_single_query_summary(out_dir)
    rbf_single_query_manifest = find_exp05_rbf_single_query_manifest(out_dir)
    rbf_single_query_rows = read_csv_rows(rbf_single_query_summary) if rbf_single_query_summary is not None else []
    rbf_single_query_manifest_payload = load_json_file(rbf_single_query_manifest)
    baseline_manifest = load_json_file(current_baseline_manifest)
    baseline_manifest_ok = manifest_uses_required_simplify(baseline_manifest)
    if current_baseline_summary is not None and current_baseline_summary != summary and baseline_manifest_ok:
        baseline_methods = {"rrtconnect", "prm", "bitstar"}
        current_baseline_rows = [
            row for row in read_csv_rows(current_baseline_summary)
            if str(row.get("method")) in baseline_methods
        ]
        if current_baseline_rows:
            rows = [row for row in rows if str(row.get("method")) not in baseline_methods]
            rows.extend(current_baseline_rows)
    prm_optimized_manifest_payload = load_json_file(prm_optimized_manifest)
    if prm_optimized_summary is not None:
        prm_rows = [
            row for row in read_csv_rows(prm_optimized_summary)
            if str(row.get("method")) == "prm"
        ]
        if prm_rows:
            rows = [row for row in rows if str(row.get("method")) != "prm"]
            rows.extend(prm_rows)
    current_iris_manifest_payload = load_json_file(current_iris_manifest)
    if current_iris_summary is not None:
        current_iris_rows = [
            row for row in read_csv_rows(current_iris_summary)
            if str(row.get("method")) == "iris_np_gcs"
        ]
        if current_iris_rows:
            rows = [row for row in rows if str(row.get("method")) != "iris_np_gcs"]
            rows.extend(current_iris_rows)
    registered_iris_payload = load_json_file(REGISTERED_EXP05_IRIS_GCS_ANYTIME)
    registered_iris_run_rows: list[dict[str, Any]] = []
    registered_iris_summary_rows: list[dict[str, Any]] = []
    if isinstance(registered_iris_payload, dict):
        registered_iris_run_rows = shelf_iris_json_to_run_rows(registered_iris_payload)
        registered_iris_summary_rows = shelf_iris_summary_rows(registered_iris_run_rows)
    if registered_iris_summary_rows:
        rows = [row for row in rows if str(row.get("method")) != "iris_np_gcs"]
        rows.extend(registered_iris_summary_rows)
        current_iris_manifest_payload = {
            "rows": registered_iris_run_rows,
            "source": str(REGISTERED_EXP05_IRIS_GCS_ANYTIME),
        }
    bitstar_trace_manifest_payload = load_json_file(bitstar_trace_manifest)
    if bitstar_trace_summary is not None:
        bitstar_rows = [
            row for row in read_csv_rows(bitstar_trace_summary)
            if str(row.get("method")) == "bitstar"
        ]
        if bitstar_rows:
            rows = [row for row in rows if str(row.get("method")) != "bitstar"]
            rows.extend(bitstar_rows)
    registered_rrt_rows: list[dict[str, Any]] = []
    registered_rrt_manifest_rows: list[dict[str, Any]] = []
    if not any(str(row.get("method")) == "rrtconnect" for row in rows) and REGISTERED_EXP05_RRTCONNECT_SUMMARY.exists():
        registered_rrt_rows = [
            row for row in read_csv_rows(REGISTERED_EXP05_RRTCONNECT_SUMMARY)
            if str(row.get("method")) == "rrtconnect" and is_full_success(row)
        ]
        rows.extend(registered_rrt_rows)
    if isinstance(baseline_manifest, dict):
        manifest_rows = baseline_manifest.setdefault("rows", [])
        if isinstance(current_iris_manifest_payload, dict):
            iris_manifest_rows = [
                row for row in current_iris_manifest_payload.get("rows", [])
                if str(row.get("method")) == "iris_np_gcs"
            ] if isinstance(current_iris_manifest_payload, dict) else []
            if iris_manifest_rows:
                manifest_rows[:] = [
                    row for row in manifest_rows
                    if str(row.get("method")) != "iris_np_gcs"
                ]
                manifest_rows.extend(iris_manifest_rows)
        if prm_optimized_manifest is not None:
            prm_manifest_rows = [
                row for row in prm_optimized_manifest_payload.get("rows", [])
                if str(row.get("method")) == "prm"
            ] if isinstance(prm_optimized_manifest_payload, dict) else []
            if prm_manifest_rows:
                manifest_rows[:] = [
                    row for row in manifest_rows
                    if str(row.get("method")) != "prm"
                ]
                manifest_rows.extend(prm_manifest_rows)
        if (
            bitstar_trace_manifest is not None
        ):
            bitstar_manifest_rows = [
                row for row in bitstar_trace_manifest_payload.get("rows", [])
                if str(row.get("method")) == "bitstar"
            ] if isinstance(bitstar_trace_manifest_payload, dict) else []
            if bitstar_manifest_rows:
                manifest_rows[:] = [
                    row for row in manifest_rows
                    if str(row.get("method")) != "bitstar"
                ]
                manifest_rows.extend(bitstar_manifest_rows)
        if (
            not any(str(row.get("method")) == "rrtconnect" for row in manifest_rows)
            and REGISTERED_EXP05_RRTCONNECT_MANIFEST.exists()
        ):
            registered_rrt_manifest = load_json_file(REGISTERED_EXP05_RRTCONNECT_MANIFEST)
            registered_rrt_manifest_rows = [
                row for row in registered_rrt_manifest.get("rows", [])
                if str(row.get("method")) == "rrtconnect"
            ] if isinstance(registered_rrt_manifest, dict) else []
            manifest_rows.extend(registered_rrt_manifest_rows)
    exp04_summary = find_exp04_summary(out_dir)
    exp04_manifest_path = find_exp04_manifest(out_dir)
    exp04_manifest_payload = load_json_file(exp04_manifest_path)
    if exp04_summary is not None:
        exp04_rows = [
            row for row in read_csv_rows(exp04_summary)
            if str(row.get("case")) == "baseline_d23_aafk_support_hull_8t"
        ]
        if exp04_rows:
            non_rbf_rows = [row for row in rows if str(row.get("method")) != "sbf_leaf_rrt"]
            registered_rbf_rows: list[dict[str, Any]] = []
            for row in exp04_rows:
                deep_boxes = int(float(row.get("deep_max_boxes", DEFAULT_RBF_SHELF_BOX_BUDGET) or DEFAULT_RBF_SHELF_BOX_BUDGET))
                dist = exp04_case_distributions(
                    exp04_manifest_payload if isinstance(exp04_manifest_payload, dict) else {},
                    case="baseline_d23_aafk_support_hull_8t",
                    deep_boxes=deep_boxes,
                )
                online_values = finite_values(dist.get("online_values"))
                amortized_values = finite_values(dist.get("amortized_values"))
                online_median = (
                    percentile_value(online_values, 0.50)
                    if online_values else
                    as_float(row.get("online_per_query_s_median"))
                )
                amortized_median = (
                    percentile_value(amortized_values, 0.50)
                    if amortized_values else
                    as_float(row.get("amortized_s_k5"))
                )
                registered_rbf_rows.append({
                    "method": "sbf_leaf_rrt",
                    "method_label": "RBF",
                    "deep_max_boxes": row.get("deep_max_boxes"),
                    "runs": row.get("runs"),
                    "success_runs": row.get("success_runs"),
                    "source": "current_exp04_registered_profile",
                    "build_s": row.get("build_s_median", row.get("planning_s_median")),
                    "query_s_median": row.get("query_s_median", ""),
                    "offline_build_s_median": row.get("offline_build_s_median", row.get("build_s_median", row.get("planning_s_median"))),
                    "online_batch_s_median": row.get("online_batch_s_median", ""),
                    "online_total_s_median": row.get("online_total_s_median", ""),
                    "online_per_query_s_median": online_median,
                    "online_total_per_query_s_median": row.get("online_total_per_query_s_median", ""),
                    "online_solve_s_median": row.get("online_solve_s_median", ""),
                    "online_simplify_s_median": row.get("online_simplify_s_median", ""),
                    "online_solve_per_query_s_median": online_median,
                    "online_simplify_per_query_s_median": row.get("online_simplify_per_query_s_median", ""),
                    "amortized_s_k1": row.get("amortized_s_k1", ""),
                    "amortized_s_k5": amortized_median,
                    "amortized_s_k10": row.get("amortized_s_k10", ""),
                    "amortized_s_k20": row.get("amortized_s_k20", ""),
                    "amortized_s_k50": row.get("amortized_s_k50", ""),
                    "planning_s_median": row.get("planning_s_median"),
                    "audit_s_median": row.get("audit_s_median"),
                    "path_length_mean": row.get("path_length_mean", row.get("path_length_median")),
                    "raw_segment_fraction_median": row.get("raw_segment_fraction_median"),
                    "status": "exp04_registered_profile",
                })
            rows = non_rbf_rows + registered_rbf_rows
    table_path = generated / "tab_tro_shelf_cross_algorithm.tex"
    pdf_path = generated / "fig_tro_shelf_cross_tradeoff.pdf"
    png_path = generated / "fig_tro_shelf_cross_tradeoff.png"
    generate_exp05_table(
        table_path,
        rows,
        rbf_manifest=exp04_manifest_payload if exp04_summary is not None else rbf_manifest,
        baseline_manifest=baseline_manifest,
        rbf_single_query_summary_rows=rbf_single_query_rows,
        rbf_single_query_manifest=rbf_single_query_manifest_payload,
    )
    generate_exp05_figure(pdf_path, png_path, rows)
    bitstar_table_stats: dict[str, dict[str, float]] = {}
    bitstar_table_success = 0
    bitstar_table_total = 0
    prm_table_stats: dict[str, dict[str, float]] = {}
    prm_table_success = 0
    prm_table_total = 0
    if isinstance(baseline_manifest, dict):
        prm_table_stats, prm_table_success, prm_table_total = (
            prm_per_query_tradeoff_stats(baseline_manifest.get("rows", []))
        )
        bitstar_table_stats, bitstar_table_success, bitstar_table_total = (
            bitstar_per_query_tradeoff_stats(baseline_manifest.get("rows", []))
        )
    prm_selected_stages = {
        label: str(stats.get("selected_stage_id", ""))
        for label, stats in prm_table_stats.items()
        if stats.get("selected_stage_id")
    }
    bitstar_selected_stages = {
        label: str(stats.get("selected_stage_id", ""))
        for label, stats in bitstar_table_stats.items()
        if stats.get("selected_stage_id")
    }
    source_payload = {
        "status": "generated",
        "summary": str(summary),
        "summary_sha256": file_sha256(summary),
        "current_baseline_summary": str(current_baseline_summary) if current_baseline_summary is not None else None,
        "current_baseline_summary_sha256": file_sha256(current_baseline_summary) if current_baseline_summary is not None else None,
        "current_baseline_manifest": str(current_baseline_manifest) if current_baseline_manifest is not None else None,
        "current_baseline_manifest_sha256": file_sha256(current_baseline_manifest) if current_baseline_manifest is not None else None,
        "prm_optimized_summary": str(prm_optimized_summary) if prm_optimized_summary is not None else None,
        "prm_optimized_summary_sha256": file_sha256(prm_optimized_summary) if prm_optimized_summary is not None else None,
        "prm_optimized_manifest": str(prm_optimized_manifest) if prm_optimized_manifest is not None else None,
        "prm_optimized_manifest_sha256": file_sha256(prm_optimized_manifest) if prm_optimized_manifest is not None else None,
        "current_iris_summary": str(current_iris_summary) if current_iris_summary is not None else None,
        "current_iris_summary_sha256": file_sha256(current_iris_summary) if current_iris_summary is not None else None,
        "current_iris_manifest": str(current_iris_manifest) if current_iris_manifest is not None else None,
        "current_iris_manifest_sha256": file_sha256(current_iris_manifest) if current_iris_manifest is not None else None,
        "registered_iris_gcs_anytime": str(REGISTERED_EXP05_IRIS_GCS_ANYTIME),
        "registered_iris_gcs_anytime_sha256": file_sha256(REGISTERED_EXP05_IRIS_GCS_ANYTIME),
        "registered_iris_gcs_runs": len(registered_iris_run_rows),
        "current_iris_context_policy": "registered_repeated_eight_seed_anytime_artifact",
        "bitstar_trace_summary": str(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_summary_sha256": file_sha256(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_manifest": str(bitstar_trace_manifest) if bitstar_trace_manifest is not None else None,
        "bitstar_trace_manifest_sha256": file_sha256(bitstar_trace_manifest) if bitstar_trace_manifest is not None else None,
        "registered_rrtconnect_rows": len(registered_rrt_rows),
        "registered_rrtconnect_manifest_rows": len(registered_rrt_manifest_rows),
        "rbf_manifest": str(rbf_manifest_path) if rbf_manifest_path is not None else None,
        "rbf_manifest_sha256": file_sha256(rbf_manifest_path) if rbf_manifest_path is not None else None,
        "rbf_manifest_for_table": str(exp04_manifest_path) if exp04_summary is not None else str(rbf_manifest_path) if rbf_manifest_path is not None else None,
        "rbf_manifest_for_table_sha256": file_sha256(exp04_manifest_path) if exp04_summary is not None else file_sha256(rbf_manifest_path) if rbf_manifest_path is not None else None,
        "rbf_manifest_policy": "current_exp04_registered_profile",
        "table_rbf_time_policy": "registered_exp04_build_with_parallel_five_query_task_diagnostics_and_legacy_single_query_fallback_only_if_missing",
        "table_prm_tradeoff_policy": "per_query_fastest_full_seed_checkpoint_within_1p01x_best_query_path_mean",
        "table_prm_tradeoff_path_slack": 1.01,
        "table_prm_success": prm_table_success,
        "table_prm_total": prm_table_total,
        "table_prm_selected_stages": prm_selected_stages,
        "table_bitstar_tradeoff_policy": "per_query_fastest_full_seed_checkpoint_within_1p01x_best_query_path_mean",
        "table_bitstar_tradeoff_path_slack": 1.01,
        "table_bitstar_success": bitstar_table_success,
        "table_bitstar_total": bitstar_table_total,
        "table_bitstar_selected_stages": bitstar_selected_stages,
        "rbf_single_query_summary": str(rbf_single_query_summary) if rbf_single_query_summary is not None else None,
        "rbf_single_query_summary_sha256": file_sha256(rbf_single_query_summary) if rbf_single_query_summary is not None else None,
        "rbf_single_query_manifest": str(rbf_single_query_manifest) if rbf_single_query_manifest is not None else None,
        "rbf_single_query_manifest_sha256": file_sha256(rbf_single_query_manifest) if rbf_single_query_manifest is not None else None,
        "exp04_registered_summary": str(exp04_summary) if exp04_summary is not None else None,
        "exp04_registered_summary_sha256": file_sha256(exp04_summary) if exp04_summary is not None else None,
        "rows": len(rows),
        "table": str(table_path),
        "figure_pdf": str(pdf_path),
        "figure_png": str(png_path),
    }
    if registered_rrt_rows:
        source_payload["registered_rrtconnect_summary"] = str(REGISTERED_EXP05_RRTCONNECT_SUMMARY)
    else:
        source_payload["registered_rrtconnect_summary_policy"] = (
            "not_used_current_saved_baseline_rows_available"
        )
    if registered_rrt_manifest_rows:
        source_payload["registered_rrtconnect_manifest"] = str(REGISTERED_EXP05_RRTCONNECT_MANIFEST)
    else:
        source_payload["registered_rrtconnect_manifest_policy"] = (
            "not_used_current_saved_baseline_manifest_available"
        )
    if rbf_query_manifest_path is not None:
        source_payload["rbf_query_manifest"] = str(rbf_query_manifest_path)
        source_payload["rbf_query_manifest_sha256"] = file_sha256(rbf_query_manifest_path)
    else:
        source_payload["rbf_query_manifest_policy"] = "not_used_rbf_manifest_had_rows"
    return source_payload


def select_best_budget_rows(rows: list[dict[str, Any]], group_fields: list[str]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []

    def quality_plateau_row(candidates: list[dict[str, Any]]) -> dict[str, Any]:
        ordered = sorted(
            candidates,
            key=lambda row: (
                measured_time_key(row),
                path_length_stat(row) if math.isfinite(path_length_stat(row)) else 1e9,
                int(float(row.get("deep_max_boxes", 0) or 0)),
            ),
        )
        quality_values = [
            path_length_stat(row) if math.isfinite(path_length_stat(row)) else math.nan
            for row in ordered
        ]
        selected_index = select_quality_plateau_index(quality_values, start_index=0)
        return ordered[selected_index if selected_index is not None else 0]

    keys = sorted({tuple(str(row.get(field, "")) for field in group_fields) for row in rows})
    for key in keys:
        items = [
            row for row in rows
            if tuple(str(row.get(field, "")) for field in group_fields) == key
        ]
        full = [row for row in items if full_success(row)]
        candidates = full or items
        if not candidates:
            continue
        registered = [
            row for row in candidates
            if str(row.get("paper_selected_profile", "")).strip()
            or str(row.get("registered_profile", "")).lower() in {"1", "true", "yes"}
        ]
        if registered:
            out.append(quality_plateau_row(registered))
            continue
        out.append(quality_plateau_row(candidates))
    return out


def generate_rbf_budget_figure(pdf_path: Path,
                               png_path: Path,
                               rows: list[dict[str, Any]],
                               title: str,
                               group_fields: list[str]) -> None:
    if not rows:
        return
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    configure_matplotlib_for_ieee(matplotlib)
    import matplotlib.pyplot as plt

    def group_label(row: dict[str, Any]) -> str:
        if not group_fields:
            return "RBF"
        return "/".join(str(row.get(field, "")) for field in group_fields)

    groups = sorted({group_label(row) for row in rows})
    fig, axes = plt.subplots(1, 2, figsize=(6.6, 2.65), constrained_layout=True)
    path_ref = finite_min([path_length_stat(row) for row in rows])
    metrics = [
        ("raw_segment_fraction_median", "Raw segment fraction"),
        ("path_length_mean", r"Path-length ratio $L_q/L_{\mathrm{ref},q}$"),
    ]
    for group in groups:
        items = sorted([row for row in rows if group_label(row) == group], key=measured_time_key)
        x = [method_time(row) for row in items]
        for axis, (field, ylabel) in zip(axes, metrics):
            y = [
                ratio_to_ref(path_length_stat(row), path_ref)
                if field == "path_length_mean"
                else float(row.get(field, "nan"))
                for row in items
            ]
            axis.plot(x, y, marker="o", linewidth=1.5, label=group)
            for bx, by, row in zip(x, y, items):
                ok = int(float(row.get("success_scenes", row.get("success_runs", 0)) or 0))
                n = int(float(row.get("scenes", row.get("runs", 0)) or 0))
                if n > 0 and ok < n:
                    axis.plot([bx], [by], marker="x", color="black", markersize=6, mew=1.4)
    for axis, (_, ylabel) in zip(axes, metrics):
        axis.set_xscale("log")
        axis.set_xlabel("amortized time / query @5 (s, log)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25, linewidth=0.6)
    axes[0].axhline(0.4, color="0.35", linestyle="--", linewidth=1.0)
    if len(groups) <= 9:
        axes[0].legend(fontsize=6, frameon=False)
    fig.suptitle(title, fontsize=10)
    fig.savefig(pdf_path)
    fig.savefig(png_path, dpi=220)
    plt.close(fig)


def generate_exp06_table(path: Path, rows: list[dict[str, Any]]) -> None:
    reusable_context_methods = ["prm"]
    online_context_methods = ["rrtconnect", "bitstar"]
    rbf_rows = [row for row in rows if str(row.get("method")) == "sbf_leaf_rrt"]
    selected_rows = select_best_budget_rows(rbf_rows, ["robot", "difficulty"])
    context, has_current_baselines = merged_random_context(rows)
    selected_by_key = {
        (str(row.get("robot")).lower(), str(row.get("difficulty")).lower()): row
        for row in selected_rows
    }
    robot_order = ["iiwa", "ur5", "panda"]
    difficulty_order = ["medium", "hard"]
    robot_labels = {"iiwa": "IIWA", "ur5": "UR5", "panda": "Panda"}
    scenario_items: list[tuple[str, str, str, dict[str, Any], dict[str, dict[str, Any]]]] = []
    for robot in robot_order:
        for difficulty in difficulty_order:
            row = selected_by_key.get((robot, difficulty))
            if row is None:
                continue
            scenario = f"{robot_labels.get(robot, robot.upper())}-{difficulty.capitalize()}"
            scenario_items.append((scenario, robot, difficulty, row, context.get((robot, difficulty), {})))

    scenario_refs: dict[str, dict[str, float]] = {}
    scenario_scalar_refs: dict[str, float] = {}
    for scenario, robot, difficulty, row, scenario_context in scenario_items:
        label_candidates: dict[str, list[float]] = {}
        scalar_candidates: list[float] = []

        def collect_refs(path_by_label: Any, path_values: Any, scalar_path: Any) -> None:
            if isinstance(path_by_label, dict) and path_by_label:
                for label, values in path_by_label.items():
                    finite = [value for value in finite_values(values) if value > 0.0]
                    if finite:
                        label_candidates.setdefault(str(label), []).append(min(finite))
            finite_path_values = finite_values(path_values)
            positive_path_values = [value for value in finite_path_values if value > 0.0]
            if positive_path_values:
                scalar_candidates.append(min(positive_path_values))
                return
            scalar = as_float(scalar_path)
            if math.isfinite(scalar) and scalar > 0.0:
                scalar_candidates.append(scalar)

        for candidate in rows:
            if (
                str(candidate.get("robot")).lower() == robot
                and str(candidate.get("difficulty")).lower() == difficulty
            ):
                collect_refs(
                    candidate.get("_path_by_label") or candidate.get("path_by_label"),
                    candidate.get("_path_values") or candidate.get("path_values"),
                    path_length_stat(candidate),
                )
        for method in reusable_context_methods + online_context_methods:
            item = scenario_context.get(method, {})
            collect_refs(item.get("path_by_label"), item.get("path_values"), item.get("path_length"))
        scenario_refs[scenario] = {
            label: min(values)
            for label, values in label_candidates.items()
            if values
        }
        scenario_scalar_refs[scenario] = min(scalar_candidates) if scalar_candidates else math.nan

    def gap_cell(path_by_label: Any, path_values: Any, scalar_path: Any, scenario: str) -> str:
        refs = scenario_refs.get(scenario, {})
        if isinstance(path_by_label, dict):
            gaps = normalized_gap_from_label_values(path_by_label, refs)
            if gaps:
                return tex_iqr_stacked(gaps, 2)
        values = path_values if finite_values(path_values) else scalar_path
        return tex_iqr_stacked(
            normalized_gap_values(values, scenario_scalar_refs.get(scenario, math.nan)),
            2,
        )

    caption = r"\captionof{table}{Random-scene results at method-specific, in-sample operating points}"
    path_metric = r"$L/L_{\mathrm{ref},\lambda}$" if has_current_baselines else r"$L/L_{\mathrm{ref,scn}}$"
    notes = (
        r"Seconds; medians [25\%, 75\%]. \rbf{} Task/q: ten-query scene-adaptation-plus-query average; OMPL \(\mathrm{Online/q}\): 100 queries/condition. "
        r"BIT* is selected per instance; UR5 Medium uses query-local edges; postprocessing is excluded."
    )
    if not has_current_baselines:
        path_metric = r"$L/L_{\mathrm{ref,scn}}$"
        notes = (
            r"Entries are medians [first quartile, third quartile]; timing columns are in seconds. Separately recorded final-return simplification and the data-integrity check are excluded; planning-internal collision and region-validation checks remain included. "
            r"\(L/L_{\mathrm{ref,scn}}\) is the scenario-level empirical 0.01~rad reference ratio. "
            r"RBF = RapidBoxForest; PRM = probabilistic roadmap; RRT-Connect = bidirectional RRT; BIT* = Batch Informed Trees."
        )

    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        caption,
        r"\label{tab:tro-random-summary}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{1.3pt}",
        r"\renewcommand{\arraystretch}{1.18}",
        r"\setlength{\extrarowheight}{1.2pt}",
        r"\resizebox{\textwidth}{!}{%",
        r"\begin{tabular}{lccc@{\hspace{0.55em}}ccc@{\hspace{0.55em}}cc@{\hspace{0.55em}}cc}",
        r"\toprule",
        r"Scenario & \multicolumn{3}{c}{\rbf{}} & \multicolumn{3}{c}{PRM} & \multicolumn{2}{c}{RRT-Connect} & \multicolumn{2}{c}{BIT*} \\",
        r"\cmidrule(lr){2-4}\cmidrule(lr){5-7}\cmidrule(lr){8-9}\cmidrule(lr){10-11}",
        rf" & Build & Task/q & {path_metric} & Build & \onlineq{{}} & {path_metric} & \onlineq{{}} & {path_metric} & Selected \onlineq{{}} & {path_metric} \\",
        r"\midrule",
    ]
    for scenario, _robot, _difficulty, row, scenario_context in scenario_items:
        online_q = online_query_time(row)
        if not math.isfinite(online_q):
            online_q = as_float(row.get("planning_s_median", row.get("measured_time_s_median")))
        cells = [
            scenario,
            tex_iqr_stacked(row.get("_build_values", row.get("offline_build_s_median", row.get("build_s"))), 3),
            tex_iqr_stacked(row.get("_online_solve_values", online_q), 3),
            gap_cell(row.get("_path_by_label"), row.get("_path_values"), path_length_stat(row), scenario),
        ]
        for method in reusable_context_methods:
            item = scenario_context.get(method, {})
            cells.extend([
                tex_iqr_stacked(item.get("build_values") or item.get("build_s"), 3),
                tex_iqr_stacked(item.get("query_values") or item.get("query_s"), 3),
                gap_cell(item.get("path_by_label"), item.get("path_values"), item.get("path_length"), scenario),
            ])
        for method in online_context_methods:
            item = scenario_context.get(method, {})
            cells.extend([
                tex_iqr_stacked(item.get("query_values") or item.get("query_s"), 3),
                gap_cell(item.get("path_by_label"), item.get("path_values"), item.get("path_length"), scenario),
            ])
        lines.append(" & ".join(cells) + r" \\")
    lines.extend([
        r"\bottomrule",
        r"\end{tabular}%",
        r"}",
        r"\par\vspace{0.1ex}",
        rf"{{\footnotesize\emph{{Notes:}} {notes}\par}}",
        r"\par\endgroup",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp06_figure(pdf_path: Path, png_path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    configure_matplotlib_for_ieee(matplotlib)
    import matplotlib.pyplot as plt

    context, _has_current_baselines = merged_random_context(rows)
    current_curves = current_random_curves_from_rows(rows)
    robot_order = ["iiwa", "ur5", "panda"]
    difficulty_order = ["medium", "hard"]
    robot_labels = {"iiwa": "IIWA", "ur5": "UR5", "panda": "Panda"}
    rbf_selected = {
        (str(row.get("robot")).lower(), str(row.get("difficulty")).lower()): row
        for row in select_best_budget_rows(
            [row for row in rows if str(row.get("method")) == "sbf_leaf_rrt"],
            ["robot", "difficulty"],
        )
    }

    def figure_time(row: dict[str, Any], k: int = 5) -> float:
        """Primary Exp.6 figure time: fixed-K reusable amortization."""
        reported = as_float(row.get(f"amortized_s_k{k}"))
        if math.isfinite(reported):
            return reported
        return amortized_query_time(row, k)

    def context_figure_time(item: dict[str, Any], k: int = 5) -> float:
        """Primary Exp.6 figure time for merged baseline/reference points."""
        value = as_float(item.get("total_s", item.get("measured_time_s")))
        if math.isfinite(value):
            return value
        build = as_float(item.get("build_s"), 0.0)
        query = as_float(item.get("query_s", item.get("online_s", item.get("total_s"))))
        if math.isfinite(query):
            return max(0.0, build) / float(k) + query
        return math.nan

    def scenario_path_refs(
        robot: str,
        difficulty: str,
        scenario_context: dict[str, dict[str, Any]],
        scenario_curves: dict[str, list[dict[str, Any]]],
    ) -> tuple[dict[str, float], float]:
        """Build the exact scene-query references used by the Exp.6 table."""
        label_candidates: dict[str, list[float]] = {}
        scalar_candidates: list[float] = []

        def collect_refs(path_by_label: Any, path_values: Any, scalar_path: Any) -> None:
            if isinstance(path_by_label, dict) and path_by_label:
                for label, values in path_by_label.items():
                    finite = [value for value in finite_values(values) if value > 0.0]
                    if finite:
                        label_candidates.setdefault(str(label), []).append(min(finite))
            finite_path_values = finite_values(path_values)
            positive_path_values = [value for value in finite_path_values if value > 0.0]
            if positive_path_values:
                scalar_candidates.append(min(positive_path_values))
                return
            scalar = as_float(scalar_path)
            if math.isfinite(scalar) and scalar > 0.0:
                scalar_candidates.append(scalar)

        for candidate in rows:
            if (
                str(candidate.get("robot")).lower() == robot
                and str(candidate.get("difficulty")).lower() == difficulty
            ):
                collect_refs(
                    candidate.get("_path_by_label") or candidate.get("path_by_label"),
                    candidate.get("_path_values") or candidate.get("path_values"),
                    path_length_stat(candidate),
                )
        for item in scenario_context.values():
            collect_refs(item.get("path_by_label"), item.get("path_values"), item.get("path_length"))
        for method_points in scenario_curves.values():
            for point in method_points:
                collect_refs(point.get("path_by_label"), point.get("path_values"), point.get("path_length"))
        refs = {
            label: min(values)
            for label, values in label_candidates.items()
            if values
        }
        scalar_ref = min(scalar_candidates) if scalar_candidates else math.nan
        return refs, scalar_ref

    def figure_path_ratio(
        path_by_label: Any,
        path_values: Any,
        scalar_path: Any,
        refs: dict[str, float],
        scalar_ref: float,
    ) -> float:
        """Single-point y value aligned with the table's query-matched median semantics."""
        if isinstance(path_by_label, dict):
            ratios = normalized_gap_from_label_values(path_by_label, refs)
            if ratios:
                return percentile_value(ratios, 0.50)
        values = path_values if finite_values(path_values) else scalar_path
        ratios = normalized_gap_values(values, scalar_ref)
        if ratios:
            return percentile_value(ratios, 0.50)
        return ratio_to_ref(scalar_path, scalar_ref)

    # Author the 3x3 panel on a wide canvas so its full-width IEEE placement
    # enlarges the panels without consuming a nearly square float page.
    # Scale only its typography so the included vector artwork retains
    # publication-size labels after LaTeX shrinks it to the text width.
    random_font_scale = 3.0
    fig, axes = plt.subplots(
        len(robot_order),
        len(difficulty_order) + 1,
        figsize=(21.6, 7.9),
    )
    for row_index, robot in enumerate(robot_order):
        for col_index, difficulty in enumerate(difficulty_order):
            axis = axes[row_index][col_index]
            items = sorted(
                [
                    row for row in rows
                    if str(row.get("method")) == "sbf_leaf_rrt"
                    and str(row.get("robot")).lower() == robot
                    and str(row.get("difficulty")).lower() == difficulty
                    and full_success(row)
                ],
                key=lambda row: int(float(row.get("deep_max_boxes", 0) or 0)),
            )
            scenario_context = context.get((robot, difficulty), {})
            scenario_curves = current_curves.get((robot, difficulty), {})
            path_refs, scalar_path_ref = scenario_path_refs(robot, difficulty, scenario_context, scenario_curves)
            panel_path_values: list[float] = []
            if items:
                xs = [figure_time(row, 5) for row in items]
                ys = [
                    figure_path_ratio(
                        row.get("_path_by_label"),
                        row.get("_path_values"),
                        path_length_stat(row),
                        path_refs,
                        scalar_path_ref,
                    )
                    for row in items
                ]
                panel_path_values.extend(ys)
                axis.plot(xs, ys, "-", color=METHOD_STYLE["sbf_leaf_rrt"]["color"],
                          alpha=0.60, linewidth=LINE_WIDTH)
                axis.scatter(xs, ys, marker=METHOD_STYLE["sbf_leaf_rrt"]["marker"],
                             color=METHOD_STYLE["sbf_leaf_rrt"]["color"], s=POINT_SIZE, alpha=0.82)
                first = first_full_success_row(items)
                if first is not None:
                    axis.scatter(
                        [figure_time(first, 5)],
                        [
                            figure_path_ratio(
                                first.get("_path_by_label"),
                                first.get("_path_values"),
                                path_length_stat(first),
                                path_refs,
                                scalar_path_ref,
                            )
                        ],
                        facecolors="none",
                        edgecolors="black",
                        linewidths=0.9,
                        s=28,
                        zorder=4,
                    )
                selected = rbf_selected.get((robot, difficulty))
                if selected is not None:
                    axis.scatter(
                        [figure_time(selected, 5)],
                        [
                            figure_path_ratio(
                                selected.get("_path_by_label"),
                                selected.get("_path_values"),
                                path_length_stat(selected),
                                path_refs,
                                scalar_path_ref,
                            )
                        ],
                        facecolors="none",
                        edgecolors="#d4a017",
                        linewidths=SELECTED_LINE_WIDTH,
                        s=SELECTED_POINT_SIZE,
                        zorder=5,
                    )
            plotted_curve_xy: dict[str, tuple[list[float], list[float], list[dict[str, Any]]]] = {}
            for method in ["prm", "bitstar"]:
                points = scenario_curves.get(method, [])
                if len(points) < 2:
                    continue
                style = METHOD_STYLE[method]
                xs = [point["total_s"] for point in points]
                ys = [
                    figure_path_ratio(
                        point.get("path_by_label"),
                        point.get("path_values"),
                        point.get("path_length"),
                        path_refs,
                        scalar_path_ref,
                    )
                    for point in points
                ]
                plotted_curve_xy[method] = (xs, ys, points)
                panel_path_values.extend(ys)
                if method == "bitstar":
                    axis.plot(xs, ys, "-", color=style["color"], alpha=0.74, linewidth=LINE_WIDTH)
                    marker_indices = sorted({
                        0,
                        *[
                            min(range(len(xs)), key=lambda index: abs(xs[index] - target))
                            for target in (0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)
                            if min(xs) <= target <= max(xs)
                        ],
                    })
                    axis.scatter(
                        [xs[index] for index in marker_indices],
                        [ys[index] for index in marker_indices],
                        marker=style["marker"],
                        color=style["color"],
                        s=POINT_SIZE,
                        alpha=0.62,
                    )
                    full_indices = [
                        index for index, point in enumerate(points)
                        if bool(point.get("full_success", True))
                    ]
                    if full_indices:
                        first_full = full_indices[0]
                        axis.scatter(
                            [xs[first_full]],
                            [ys[first_full]],
                            facecolors="none",
                            edgecolors="black",
                            linewidths=0.8,
                            s=24,
                            zorder=4,
                        )
                else:
                    # PRM is plotted as a cumulative incumbent curve: the
                    # roadmap quality stays fixed between build checkpoints
                    # and only changes when a later checkpoint improves the
                    # incumbent path.  A straight line visually invents
                    # continuous quality improvement and hides the late
                    # plateau behavior.
                    axis.step(
                        xs,
                        ys,
                        where="post",
                        color=style["color"],
                        alpha=0.52,
                        linewidth=LINE_WIDTH,
                    )
                    axis.scatter(xs, ys, marker=style["marker"], color=style["color"], s=POINT_SIZE, alpha=0.58)
                if method != "bitstar":
                    full_indices = [
                        index for index, point in enumerate(points)
                        if bool(point.get("full_success", True))
                    ]
                    first_full = full_indices[0] if full_indices else 0
                    axis.scatter(
                        [xs[first_full]],
                        [ys[first_full]],
                        facecolors="none",
                        edgecolors="black",
                        linewidths=0.8,
                        s=24,
                        zorder=4,
                    )
            for method in ["prm", "rrtconnect", "bitstar"]:
                item = scenario_context.get(method)
                if not item:
                    continue
                if scenario_curves.get(method):
                    continue
                style = METHOD_STYLE[method]
                item_ratio = figure_path_ratio(
                    item.get("path_by_label"),
                    item.get("path_values"),
                    item.get("path_length"),
                    path_refs,
                    scalar_path_ref,
                )
                panel_path_values.append(item_ratio)
                context_x = context_figure_time(item, 5)
                axis.scatter([context_x], [item_ratio],
                             marker=style["marker"], color=style["color"], s=POINT_SIZE, alpha=0.82)
            for method in ["prm", "rrtconnect", "bitstar"]:
                item = scenario_context.get(method)
                if not item:
                    continue
                curve_xy = plotted_curve_xy.get(method)
                if curve_xy is not None:
                    xs, ys, points = curve_xy
                    full_indices = [
                        index for index, point in enumerate(points)
                        if bool(point.get("full_success", True))
                    ]
                    first_full_index = full_indices[0] if full_indices else 0
                    candidate_indices = [
                        index for index in range(first_full_index, len(points))
                        if math.isfinite(ys[index]) and math.isfinite(xs[index])
                    ]
                    if not candidate_indices:
                        continue
                    # The displayed/gold point is on the actual displayed
                    # curve, constrained to lie at or after the first
                    # point with validated success on the saved query set (black).
                    # It advances only while path
                    # quality decreases by the configured relative threshold,
                    # preventing negligible late improvements from moving the
                    # displayed point to the far end of the curve.
                    selected_index = select_quality_plateau_index(
                        ys,
                        start_index=first_full_index,
                    )
                    if selected_index is None:
                        continue
                    context_x = xs[selected_index]
                    item_ratio = ys[selected_index]
                else:
                    context_x = context_figure_time(item, 5)
                    if not math.isfinite(context_x):
                        continue
                    item_ratio = figure_path_ratio(
                        item.get("path_by_label"),
                        item.get("path_values"),
                        item.get("path_length"),
                        path_refs,
                        scalar_path_ref,
                    )
                    if not math.isfinite(item_ratio):
                        continue
                axis.scatter(
                    [context_x],
                    [item_ratio],
                    facecolors="none",
                    edgecolors="#d4a017",
                    linewidths=SELECTED_LINE_WIDTH,
                    s=SELECTED_POINT_SIZE,
                    zorder=6,
                )
            axis.set_xscale("log")
            use_compact_log_x_ticks(axis)
            axis.set_xlabel("method-specific budget (s)" if row_index == len(robot_order) - 1 else "")
            if row_index == 0:
                axis.set_title(
                    difficulty.capitalize(),
                    fontsize=PANEL_TITLE_FONTSIZE * random_font_scale,
                )
            if col_index == 0:
                axis.set_ylabel(robot_labels[robot] + "\n" + r"$L/L_{\mathrm{ref},\lambda}$")
            axis.grid(True, which="both", alpha=0.22)
            axis.tick_params(labelsize=TICK_LABEL_FONTSIZE * random_font_scale)
            axis.xaxis.label.set_size(AXIS_LABEL_FONTSIZE * random_font_scale)
            axis.yaxis.label.set_size(AXIS_LABEL_FONTSIZE * random_font_scale)
            set_padded_linear_ylim(axis, panel_path_values, min_pad=0.04)

        amort_methods: list[dict[str, Any]] = []
        for method in ["sbf_leaf_rrt", "prm", "rrtconnect", "bitstar"]:
            build_values: list[float] = []
            query_values: list[float] = []
            for difficulty in difficulty_order:
                if method == "sbf_leaf_rrt":
                    selected = rbf_selected.get((robot, difficulty))
                    if selected is None:
                        continue
                    selected_build_values = finite_values(
                        selected.get("_build_values", selected.get("offline_build_s_median"))
                    )
                    selected_query_values = finite_values(
                        selected.get("_online_solve_values", online_query_time(selected))
                    )
                    build_values.append(
                        percentile_value(selected_build_values, 0.50)
                        if selected_build_values else
                        as_float(selected.get("offline_build_s_median"), 0.0)
                    )
                    query_values.append(
                        percentile_value(selected_query_values, 0.50)
                        if selected_query_values else
                        online_query_time(selected)
                    )
                else:
                    item = context.get((robot, difficulty), {}).get(method)
                    if not item:
                        continue
                    item_build_values = finite_values(item.get("build_values", item.get("build_s")))
                    item_query_values = finite_values(item.get("query_values", item.get("query_s")))
                    build_values.append(
                        percentile_value(item_build_values, 0.50)
                        if item_build_values else
                        as_float(item.get("build_s"), 0.0)
                    )
                    query_values.append(
                        percentile_value(item_query_values, 0.50)
                        if item_query_values else
                        as_float(item.get("query_s"), item.get("total_s", 0.0))
                    )
            if build_values and query_values:
                style = METHOD_STYLE[method]
                amort_methods.append({
                    "method": method,
                    "label": style["label"],
                    "build_s": mean(build_values) or 0.0,
                    "per_query_s": mean(query_values) or 0.0,
                })
        amort_axis = axes[row_index][len(difficulty_order)]
        plot_query_amortization_panel(
            amort_axis,
            amort_methods,
            title="Reuse horizon (medium/hard mean)" if row_index == 0 else "",
            show_xlabel=row_index == len(robot_order) - 1,
            show_ylabel=False,
            font_scale=random_font_scale,
        )

    handles = []
    labels = []
    for method in ["sbf_leaf_rrt", "prm", "rrtconnect", "bitstar"]:
        style = METHOD_STYLE[method]
        handles.append(
            plt.Line2D([0], [0], color=style["color"], marker=style["marker"],
                       linestyle="-" if method == "sbf_leaf_rrt" else "None",
                       markersize=4.4, linewidth=LINE_WIDTH)
        )
        labels.append(style["label"])
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=4,
        frameon=False,
        fontsize=max(LEGEND_FONTSIZE, 7.5) * random_font_scale,
        handlelength=2.4,
        handletextpad=0.4,
        columnspacing=0.85,
        labelspacing=0.45,
    )
    fig.subplots_adjust(left=0.065, right=0.996, top=0.900, bottom=0.050, hspace=0.25, wspace=0.12)
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.01)
    fig.savefig(png_path, dpi=220, bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)


def generate_exp06_assets(generated: Path, out_dir: Path, *, include_current_baselines: bool = False) -> dict[str, Any]:
    summary = find_exp06_summary(out_dir)
    if summary is None:
        return {"status": "missing", "summary": None}
    rows = read_csv_rows(summary)
    summary_manifest = find_exp06_manifest(out_dir)
    summary_manifest_payload = load_json_file(summary_manifest)
    annotate_summary_rows_with_manifest_distributions(rows, summary_manifest_payload)
    rbf_override_summary = find_exp06_rbf_registered_override_summary(out_dir)
    rbf_override_manifest = find_exp06_rbf_registered_override_manifest(out_dir)
    rbf_override_manifest_payload = load_json_file(rbf_override_manifest)
    rbf_override_rows: list[dict[str, Any]] = []
    if rbf_override_summary is not None:
        rbf_override_rows = [
            row for row in read_csv_rows(rbf_override_summary)
            if str(row.get("method")) == "sbf_leaf_rrt" and is_full_success(row)
        ]
        annotate_summary_rows_with_manifest_distributions(
            rbf_override_rows,
            rbf_override_manifest_payload,
        )
        override_profile = (
            rbf_override_summary.parent.name
            if rbf_override_summary is not None
            else EXP06_REGISTERED_RBF_PROFILE_NAME
        )
        for row in rbf_override_rows:
            row["paper_selected_profile"] = override_profile
            row["source"] = override_profile
        if rbf_override_rows:
            # A registered RBF override is the sole paper-facing Exp.6 RBF
            # source.  Remove all older RBF budget/profile rows before table,
            # figure, and L*_q reference construction; otherwise retired
            # diagnostic rows can define the denominator for the registered OBB
            # profile while not being displayed as the selected method.
            rows = [
                row for row in rows
                if str(row.get("method", "")) != "sbf_leaf_rrt"
            ]
            rows.extend(rbf_override_rows)
    if include_current_baselines:
        rows = [row for row in rows if str(row.get("method")) == "sbf_leaf_rrt"]
    baseline_summary = find_exp06_current_baseline_summary(out_dir) if include_current_baselines else None
    baseline_manifest = find_exp06_current_baseline_manifest(out_dir) if include_current_baselines else None
    curve_summary = find_exp06_ompl_curve_summary(out_dir)
    curve_manifest = find_exp06_ompl_curve_manifest(out_dir)
    curve_supplements = find_exp06_ompl_curve_supplements(out_dir)
    bitstar_trace_summary = find_exp06_bitstar_trace_summary(out_dir)
    bitstar_trace_manifest = find_exp06_bitstar_trace_manifest(out_dir)
    bitstar_trace_supplements = find_exp06_bitstar_trace_supplements(out_dir)
    combined_prm_bitstar_manifest = (
        curve_manifest is not None
        and bitstar_trace_manifest is not None
        and curve_manifest.resolve() == bitstar_trace_manifest.resolve()
    )
    combined_prm_bitstar_summary = (
        curve_summary is not None
        and bitstar_trace_summary is not None
        and curve_summary.resolve() == bitstar_trace_summary.resolve()
    )
    iris_summaries = find_exp06_iris_summaries(out_dir)
    baseline_rows: list[dict[str, Any]] = []
    baseline_manifest_payload = load_json_file(baseline_manifest)
    if baseline_summary is not None and manifest_uses_required_simplify(baseline_manifest_payload):
        baseline_rows = [
            row for row in read_csv_rows(baseline_summary)
            if str(row.get("method")) == "rrtconnect"
        ]
        annotate_summary_rows_with_manifest_distributions(baseline_rows, baseline_manifest_payload)
    iris_rows: list[dict[str, Any]] = []
    accepted_iris_summaries: list[Path] = []
    curve_rows: list[dict[str, Any]] = []
    curve_rows_for_figure: list[dict[str, Any]] = []
    per_query_tradeoff_rows: list[dict[str, Any]] = []
    curve_manifest_payload = load_json_file(curve_manifest)
    prm_cumulative_curve_rows: list[dict[str, Any]] = []
    if curve_summary is not None:
        raw_curve_rows = [
            row for row in read_csv_rows(curve_summary)
            if str(row.get("method")) in {"prm", "bitstar"}
        ]
        annotate_summary_rows_with_manifest_distributions(raw_curve_rows, curve_manifest_payload)
        curve_rows = [row for row in raw_curve_rows if is_full_success(row)]
        prm_cumulative_curve_rows = exp06_prm_cumulative_curve_rows(curve_manifest_payload)
        curve_rows_for_figure.extend(
            row for row in raw_curve_rows
            if str(row.get("method")) != "prm"
        )
        curve_rows_for_figure.extend(prm_cumulative_curve_rows)
        tradeoff_methods = {"bitstar"} if combined_prm_bitstar_manifest else set()
        per_query_tradeoff_rows.extend(
            exp06_per_query_tradeoff_rows(curve_manifest_payload, methods=tradeoff_methods)
        )
    accepted_curve_supplements: list[Path] = []
    for supplement_summary, supplement_manifest in curve_supplements:
        supplement_manifest_payload = load_json_file(supplement_manifest)
        raw_supplement_rows = [
            row for row in read_csv_rows(supplement_summary)
            if str(row.get("method")) in {"prm", "bitstar"}
        ]
        annotate_summary_rows_with_manifest_distributions(raw_supplement_rows, supplement_manifest_payload)
        supplement_rows = [row for row in raw_supplement_rows if is_full_success(row)]
        if supplement_rows:
            accepted_curve_supplements.append(supplement_summary)
            curve_rows.extend(supplement_rows)
        if raw_supplement_rows:
            curve_rows_for_figure.extend(raw_supplement_rows)
    bitstar_trace_manifest_payload = (
        curve_manifest_payload
        if combined_prm_bitstar_manifest else
        load_json_file(bitstar_trace_manifest)
    )
    bitstar_rows: list[dict[str, Any]] = []
    bitstar_rows_for_figure: list[dict[str, Any]] = []
    if (
        not combined_prm_bitstar_summary
        and bitstar_trace_summary is not None
        and manifest_uses_required_simplify(bitstar_trace_manifest_payload)
    ):
        raw_bitstar_rows = [
            row for row in read_csv_rows(bitstar_trace_summary)
            if str(row.get("method")) == "bitstar"
        ]
        annotate_summary_rows_with_manifest_distributions(raw_bitstar_rows, bitstar_trace_manifest_payload)
        bitstar_rows_for_figure.extend(raw_bitstar_rows)
        bitstar_rows = [row for row in raw_bitstar_rows if is_full_success(row)]
        per_query_tradeoff_rows.extend(
            exp06_per_query_tradeoff_rows(bitstar_trace_manifest_payload, methods={"bitstar"})
        )
    accepted_bitstar_trace_supplements: list[Path] = []
    for supplement_summary, supplement_manifest in bitstar_trace_supplements:
        supplement_manifest_payload = load_json_file(supplement_manifest)
        if not manifest_uses_required_simplify(supplement_manifest_payload):
            continue
        raw_supplement_rows = [
            row for row in read_csv_rows(supplement_summary)
            if str(row.get("method")) == "bitstar"
        ]
        annotate_summary_rows_with_manifest_distributions(raw_supplement_rows, supplement_manifest_payload)
        supplement_rows = [row for row in raw_supplement_rows if is_full_success(row)]
        if supplement_rows:
            accepted_bitstar_trace_supplements.append(supplement_summary)
            bitstar_rows.extend(supplement_rows)
            bitstar_rows_for_figure.extend(raw_supplement_rows)
            per_query_tradeoff_rows.extend(
                exp06_per_query_tradeoff_rows(supplement_manifest_payload, methods={"bitstar"})
            )
    if rbf_override_rows:
        figure_base_rows = [
            row for row in rows
            if str(row.get("method")) != "sbf_leaf_rrt"
            or str(row.get("paper_selected_profile", "")).strip()
        ]
    else:
        figure_base_rows = rows
    table_rows = rows + baseline_rows + curve_rows + bitstar_rows + per_query_tradeoff_rows + iris_rows
    figure_rows = (
        figure_base_rows
        + baseline_rows
        + curve_rows_for_figure
        + bitstar_rows_for_figure
        + per_query_tradeoff_rows
        + iris_rows
    )
    table_path = generated / "tab_tro_random_summary.tex"
    pdf_path = generated / "fig_tro_random_tradeoff.pdf"
    png_path = generated / "fig_tro_random_tradeoff.png"
    generate_exp06_table(table_path, table_rows)
    generate_exp06_figure(pdf_path, png_path, figure_rows)
    return {
        "status": "generated",
        "summary": str(summary),
        "summary_sha256": file_sha256(summary),
        "manifest": str(summary_manifest) if summary_manifest is not None else None,
        "manifest_sha256": file_sha256(summary_manifest) if summary_manifest is not None else None,
        "manifest_status": (
            summary_manifest_payload.get("status")
            if isinstance(summary_manifest_payload, dict) else None
        ),
        "manifest_merge_policy": (
            summary_manifest_payload.get("merge_policy") or "none"
            if isinstance(summary_manifest_payload, dict) else "not_available"
        ),
        "rbf_registered_override_summary": (
            str(rbf_override_summary) if rbf_override_summary is not None else None
        ),
        "rbf_registered_override_summary_sha256": (
            file_sha256(rbf_override_summary) if rbf_override_summary is not None else None
        ),
        "rbf_registered_override_manifest": (
            str(rbf_override_manifest) if rbf_override_manifest is not None else None
        ),
        "rbf_registered_override_manifest_sha256": (
            file_sha256(rbf_override_manifest) if rbf_override_manifest is not None else None
        ),
        "rbf_registered_override_manifest_role": (
            "timing_path_quality_and_audited_success_only; no aggregate certificate coverage claim without query-level final provenance"
            if rbf_override_manifest is not None else None
        ),
        "rbf_certificate_coverage_policy": "not_reported_current_artifact_lacks_query_level_final_certificate_provenance",
        "rbf_registered_override_rows": len(rbf_override_rows),
        "rbf_registered_override_plot_policy": (
            "plot_only_registered_override_rbf_rows"
            if rbf_override_rows else
            "plot_all_rbf_budget_rows"
        ),
        "include_current_baselines": include_current_baselines,
        "current_baseline_summary": str(baseline_summary) if baseline_summary is not None else None,
        "current_baseline_summary_sha256": file_sha256(baseline_summary) if baseline_summary is not None else None,
        "current_baseline_manifest": str(baseline_manifest) if baseline_manifest is not None else None,
        "current_baseline_manifest_sha256": file_sha256(baseline_manifest) if baseline_manifest is not None else None,
        "current_baseline_manifest_uses_required_simplify": manifest_uses_required_simplify(baseline_manifest_payload),
        "current_baseline_rows": len(baseline_rows),
        "current_iris_summaries": [str(path) for path in iris_summaries],
        "current_iris_summary_sha256": {str(path): file_sha256(path) for path in iris_summaries},
        "current_iris_accepted_summaries": [str(path) for path in accepted_iris_summaries],
        "current_iris_context_policy": "not_available_no_registered_full_catalog_result_under_common_reporting_protocol",
        "current_iris_rows": len(iris_rows),
        "ompl_curve_summary": str(curve_summary) if curve_summary is not None else None,
        "ompl_curve_summary_sha256": file_sha256(curve_summary) if curve_summary is not None else None,
        "ompl_curve_manifest": str(curve_manifest) if curve_manifest is not None else None,
        "ompl_curve_manifest_sha256": file_sha256(curve_manifest) if curve_manifest is not None else None,
        "ompl_curve_supplements": [str(path) for path in accepted_curve_supplements],
        "ompl_curve_supplement_sha256": {str(path): file_sha256(path) for path in accepted_curve_supplements},
        "ompl_per_query_tradeoff_policy": "bitstar_per_query_seed_select_fastest_within_1p08x_best_path_prm_figure_uses_manifest_cumulative_incumbent_curve",
        "ompl_per_query_tradeoff_rows": len(per_query_tradeoff_rows),
        "bitstar_trace_summary": str(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_summary_sha256": file_sha256(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_manifest": str(bitstar_trace_manifest) if bitstar_trace_manifest is not None else None,
        "bitstar_trace_manifest_sha256": file_sha256(bitstar_trace_manifest) if bitstar_trace_manifest is not None else None,
        "bitstar_trace_manifest_uses_required_simplify": manifest_uses_required_simplify(bitstar_trace_manifest_payload),
        "bitstar_trace_supplements": [str(path) for path in accepted_bitstar_trace_supplements],
        "bitstar_trace_supplement_sha256": {str(path): file_sha256(path) for path in accepted_bitstar_trace_supplements},
        "ompl_curve_rows_100pct": len(curve_rows),
        "registered_baseline_context": not bool(baseline_rows or curve_rows or iris_rows),
        "registered_baseline_context_policy": (
            "inline_registered_context_constants_used_only_when_current_baselines_are_absent"
        ),
        "rows": len(table_rows),
        "figure_rows": len(figure_rows),
        "table": str(table_path),
        "figure_pdf": str(pdf_path),
        "figure_png": str(png_path),
    }


def main() -> int:
    args = parse_args()
    generated = args.paper_dir / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, Any] = {
        "artifact": "tro2026_paper_assets",
        "environment": environment_metadata(),
        "source_root": str(args.out_dir),
        "assets": {},
        "sources": {},
        "placeholder_mode": bool(args.allow_placeholders),
    }
    manifest["sources"]["exp04_d23_cache_artifact"] = {
        "status": "found" if EXP04_D23_CACHE_ARTIFACT.exists() else "missing",
        "path": str(EXP04_D23_CACHE_ARTIFACT),
        "sha256": file_sha256(EXP04_D23_CACHE_ARTIFACT),
        "records": EXP04_D23_CACHE_RECORDS,
        "cache_bytes": EXP04_D23_CACHE_BYTES,
        "cache_gib": EXP04_D23_CACHE_GIB,
        "prewarm_s": EXP04_D23_PREWARM_S,
    }
    exp01_summary = find_exp01_summary(args.out_dir)
    exp01_rows = read_csv_rows(exp01_summary) if exp01_summary is not None else []
    if exp01_summary is not None:
        generate_exp01_table(generated / "tab_tro_endpoint_envelope.tex", exp01_rows)
    manifest["sources"]["exp01_endpoint_envelope"] = {
        "status": "generated" if exp01_summary is not None else "missing",
        "summary": str(exp01_summary) if exp01_summary is not None else None,
        "summary_sha256": file_sha256(exp01_summary) if exp01_summary is not None else None,
        "rows": len(exp01_rows),
        "registered_widths": sorted({str(row.get("width")) for row in exp01_rows}),
        "provenance_policy": "complete_result_summary_csv; later dry-run manifest excluded",
        "table": str(generated / "tab_tro_endpoint_envelope.tex"),
    }
    exp02_summary = find_exp02_summary(args.out_dir)
    exp02_rows = read_csv_rows(exp02_summary) if exp02_summary is not None else []
    if exp02_summary is not None:
        generate_exp02_table(generated / "tab_tro_link_envelope.tex", exp02_rows)
    manifest["sources"]["exp02_link_envelope"] = {
        "status": "generated" if exp02_summary is not None else "missing",
        "summary": str(exp02_summary) if exp02_summary is not None else None,
        "summary_sha256": file_sha256(exp02_summary) if exp02_summary is not None else None,
        "rows": len(exp02_rows),
        "registered_widths": sorted({str(row.get("width")) for row in exp02_rows}),
        "provenance_policy": "complete_result_summary_csv; later dry-run manifest excluded",
        "table": str(generated / "tab_tro_link_envelope.tex"),
    }
    exp04 = generate_exp04_assets(generated, args.out_dir)
    manifest["sources"]["exp04_shelf_leaf_rrt"] = exp04
    exp05 = generate_exp05_assets(generated, args.out_dir)
    manifest["sources"]["exp05_shelf_cross_algorithm"] = exp05
    exp06 = generate_exp06_assets(
        generated,
        args.out_dir,
        include_current_baselines=bool(args.include_exp06_current_baselines),
    )
    manifest["sources"]["exp06_random_robot"] = exp06
    supporting_manifest_path = find_supporting_import_manifest(args.out_dir)
    supporting_payload = load_json_file(supporting_manifest_path)
    manifest["sources"]["supporting_table_imports"] = {
        "status": supporting_payload.get("status", "missing"),
        "manifest": str(supporting_manifest_path) if supporting_manifest_path else None,
        "manifest_sha256": file_sha256(supporting_manifest_path) if supporting_manifest_path else None,
        "rows": supporting_payload.get("rows", []),
    }
    imported_supporting = {
        Path(str(row.get("target", ""))).name
        for row in supporting_payload.get("rows", [])
        if row.get("status") == "pass"
    }
    generated_tables = set(imported_supporting)
    if exp01_summary is not None:
        generated_tables.add("tab_tro_endpoint_envelope.tex")
    if exp02_summary is not None:
        generated_tables.add("tab_tro_link_envelope.tex")
    if exp04.get("status") == "generated":
        generated_tables.add("tab_tro_shelf_ablation.tex")
    if exp05.get("status") == "generated":
        generated_tables.add("tab_tro_shelf_cross_algorithm.tex")
    if exp06.get("status") == "generated":
        generated_tables.add("tab_tro_random_summary.tex")
    for filename, caption in REQUIRED_TABLES.items():
        path = generated / filename
        label = "tab:" + filename.removesuffix(".tex").replace("_", "-")
        is_placeholder = filename not in generated_tables
        if is_placeholder and not args.allow_placeholders:
            raise RuntimeError(
                f"required table {filename} was not generated from current sources; "
                "rerun with --allow-placeholders only for draft builds"
            )
        if args.allow_placeholders and is_placeholder:
            path.write_text(placeholder_table(caption, label), encoding="utf-8")
        manifest["assets"][filename] = {
            "path": str(path),
            "exists": path.exists(),
            "placeholder": is_placeholder,
            "sha256": file_sha256(path),
        }
    for filename, caption in REQUIRED_FIGURES.items():
        path = generated / filename
        manifest["assets"][filename] = {
            "path": str(path),
            "exists": path.exists(),
            "placeholder": False,
            "caption": caption,
            "sha256": file_sha256(path),
        }
    manifest = relativize_manifest(manifest)
    write_json(generated / "tro_table_generation_manifest.json", manifest)
    print(f"wrote {generated / 'tro_table_generation_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
