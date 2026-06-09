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
from experiments.common.metrics import mean, median  # noqa: E402
from experiments.common.rbf_defaults import DEFAULT_RBF_SHELF_BOX_BUDGET  # noqa: E402


REQUIRED_TABLES = {
    "tab_tro_endpoint_envelope.tex": "Endpoint envelope source study.",
    "tab_tro_link_envelope.tex": "S=1 link envelope representation study.",
    "tab_tro_lect_performance.tex": "LECT operation and memory study.",
    "tab_tro_shelf_ablation.tex": "Shelf+IIWA SBF ablation.",
    "tab_tro_shelf_cross_algorithm.tex": "Shelf+IIWA cross-algorithm comparison.",
    "tab_tro_random_summary.tex": "Random multi-robot summary.",
    "tab_tro_dynamic_update.tex": "Dynamic update summary.",
}

REQUIRED_FIGURES = {
    "fig_tro_shelf_tradeoff.pdf": "Shelf+IIWA SBF measured-time quality/segment trade-off curve.",
    "fig_tro_shelf_tradeoff.png": "Shelf+IIWA SBF measured-time quality/segment trade-off curve preview.",
    "fig_tro_shelf_cross_tradeoff.pdf": "Shelf+IIWA measured-time trade-off with cross-algorithm context.",
    "fig_tro_shelf_cross_tradeoff.png": "Shelf+IIWA measured-time trade-off with cross-algorithm context preview.",
    "fig_tro_random_tradeoff.pdf": "Random-scene measured-time trade-off curve.",
    "fig_tro_random_tradeoff.png": "Random-scene measured-time trade-off curve preview.",
}

OLD_TRO_PAPER_ROOT = Path("/home/tian/桌面/box_aabb/cpp/SBF/doc/paper/tro_rewrite_2026")
OLD_RANDOM_TABLE = OLD_TRO_PAPER_ROOT / "generated" / "tab_tro_main_random_best_tradeoff.tex"
REGISTERED_EXP05_RRTCONNECT_SUMMARY = REPO_ROOT / "outputs" / "tro2026" / "exp05_full_joint_rrtconnect_s0_7" / "shelf_cross_algorithm_summary.csv"
REGISTERED_EXP05_RRTCONNECT_MANIFEST = REPO_ROOT / "outputs" / "tro2026" / "exp05_full_joint_rrtconnect_s0_7" / "shelf_cross_algorithm_manifest.json"
REGISTERED_EXP04_QUERY_MANIFEST = REPO_ROOT / "outputs" / "tro2026" / "exp04" / "shelf_leaf_rrt_manifest.json"

METHOD_STYLE = {
    "sbf_leaf_rrt": {"label": "RBF", "color": "#1f77b4", "marker": "o"},
    "iris_np_gcs": {"label": "IRIS-NP+GCS", "color": "#ff7f0e", "marker": "D"},
    "prm": {"label": "PRM", "color": "#2ca02c", "marker": "s"},
    "rrtconnect": {"label": "RRTConnect", "color": "#d62728", "marker": "x"},
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
    parser.add_argument("--allow-placeholders", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--include-exp06-current-baselines",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Merge exp06/current_ompl_baselines into Table VI/figure generation. "
            "Enabled by default because the registered Exp.6 OMPL baselines use the saved catalog "
            "and the global strict-audit/final-simplify policy."
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


def file_sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def as_float(value: Any, default: float = math.nan) -> float:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def online_query_time(row: dict[str, Any]) -> float:
    """Online query time excluding fixed final simplification."""
    for key in ("online_solve_per_query_s_median", "online_per_query_s_median", "query_s_median"):
        value = as_float(row.get(key))
        if math.isfinite(value):
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


def is_full_success(row: dict[str, Any]) -> bool:
    success = as_float(row.get("success_queries", row.get("success_runs", row.get("success_scenes"))), 0.0)
    total = as_float(row.get("total_queries", row.get("runs", row.get("scenes"))), 0.0)
    return total > 0.0 and success >= total


AMORTIZATION_QUERY_COUNTS = [1, 5, 10, 20, 50]
PANEL_TITLE_FONTSIZE = 7.6
AXIS_LABEL_FONTSIZE = 7.0
TICK_LABEL_FONTSIZE = 6.2
LEGEND_FONTSIZE = 6.6
LINE_WIDTH = 0.95
POINT_SIZE = 14
SELECTED_POINT_SIZE = 42
SELECTED_LINE_WIDTH = 1.2


def plot_query_amortization_panel(
    ax: Any,
    methods: list[dict[str, Any]],
    *,
    title: str = "(b) amortization",
    show_xlabel: bool = True,
    show_ylabel: bool = True,
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
            markersize=3.0,
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
    ax.set_ylabel("amortized time / query (s)" if show_ylabel else "")
    ax.set_title(title, fontsize=PANEL_TITLE_FONTSIZE)
    ax.grid(True, which="both", alpha=0.24)
    ax.tick_params(labelsize=TICK_LABEL_FONTSIZE)
    ax.xaxis.label.set_size(AXIS_LABEL_FONTSIZE)
    ax.yaxis.label.set_size(AXIS_LABEL_FONTSIZE)


def set_padded_linear_ylim(ax: Any, values: list[float], *, min_pad: float = 0.08) -> None:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return
    low = min(finite)
    high = max(finite)
    span = max(high - low, min_pad)
    ax.set_ylim(low - 0.08 * span, high + 0.10 * span)


def parse_old_random_context(table_path: Path = OLD_RANDOM_TABLE) -> dict[tuple[str, str], dict[str, dict[str, float]]]:
    """Read old TRO random best-point table as contextual baseline markers.

    The current Exp.6 RBF rows use the new saved v5 catalog. These imported
    rows are therefore only plotted/reported as old common-rule context, not as
    same-catalog head-to-head trials.
    """
    if not table_path.exists():
        return {}
    text = table_path.read_text(encoding="utf-8")
    out: dict[tuple[str, str], dict[str, dict[str, float]]] = {}
    methods = ["sbf_old", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
    for line in text.splitlines():
        if "&" not in line:
            continue
        match = re.match(r"\s*(IIWA|UR5|Panda)-(Easy|Medium|Hard)\s*&\s*(.*)\\\\", line)
        if not match:
            continue
        robot = match.group(1).lower()
        difficulty = match.group(2).lower()
        values = [cell.strip() for cell in match.group(3).split("&")]
        if len(values) < 13:
            continue
        # SBF/IRIS/PRM are Build, Query, Path; RRTConnect/BIT* are Query, Path.
        offsets = {
            "sbf_old": (0, 1, 2),
            "iris_np_gcs": (3, 4, 5),
            "prm": (6, 7, 8),
            "rrtconnect": (None, 9, 10),
            "bitstar": (None, 11, 12),
        }
        scenario: dict[str, dict[str, float]] = {}
        for method, (build_index, query_index, path_index) in offsets.items():
            build_s = 0.0 if build_index is None else as_float(values[build_index])
            query_s = as_float(values[query_index])
            path_len = as_float(values[path_index])
            if math.isfinite(query_s) and math.isfinite(path_len):
                scenario[method] = {
                    "build_s": build_s,
                    "query_s": query_s,
                    "total_s": build_s + query_s,
                    "path_length": path_len,
                }
        out[(robot, difficulty)] = scenario
    return out


def current_random_context_from_rows(rows: list[dict[str, Any]]) -> dict[tuple[str, str], dict[str, dict[str, float]]]:
    out: dict[tuple[str, str], dict[str, dict[str, float]]] = {}
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
        for method in methods:
            items = [
                row for row in rows
                if str(row.get("method", "")) == method
                and str(row.get("robot", "")).lower() == robot
                and str(row.get("difficulty", "")).lower() == difficulty
            ]
            full = []
            for row in items:
                success = as_float(row.get("success_queries", row.get("success_scenes")), 0.0)
                total = as_float(row.get("total_queries", row.get("scenes")), 0.0)
                plan = method_time(row)
                path_len = path_length_stat(row)
                if total > 0 and success >= total and math.isfinite(plan) and math.isfinite(path_len):
                    full.append(row)
            if not full:
                continue
            best_path = min(path_length_stat(row) for row in full)
            candidates = [
                row for row in full
                if path_length_stat(row) <= 1.08 * best_path
            ] or full
            chosen = sorted(
                candidates,
                key=lambda row: (
                    measured_time_key(row),
                    path_length_stat(row) if math.isfinite(path_length_stat(row)) else 1e9,
                ),
            )[0]
            build_s = as_float(chosen.get("offline_build_s_median", chosen.get("build_s")), 0.0)
            query_s = online_query_time(chosen)
            if not math.isfinite(query_s):
                query_s = method_time(chosen)
            total_s = method_time(chosen)
            path_len = path_length_stat(chosen)
            out.setdefault((robot, difficulty), {})[method] = {
                "build_s": 0.0 if method in {"rrtconnect", "bitstar"} else build_s,
                "query_s": query_s,
                "total_s": total_s,
                "path_length": path_len,
                "source": "current_saved_catalog",
                "stage_id": str(chosen.get("stage_id", "")),
                "measured_time_s": total_s,
            }
    return out


def current_random_curves_from_rows(rows: list[dict[str, Any]]) -> dict[tuple[str, str], dict[str, list[dict[str, float]]]]:
    def sparse_bitstar_points(points: list[dict[str, float]]) -> list[dict[str, float]]:
        if len(points) <= 8:
            return points
        targets = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5]
        out_points: list[dict[str, float]] = []
        used: set[int] = set()
        for target in targets:
            candidates = [
                (index, point)
                for index, point in enumerate(points)
                if index not in used and point["total_s"] >= target - 1e-9
            ]
            if not candidates:
                candidates = [
                    (index, point)
                    for index, point in enumerate(points)
                    if index not in used
                ]
            if not candidates:
                continue
            index, point = min(candidates, key=lambda item: abs(item[1]["total_s"] - target))
            used.add(index)
            out_points.append(point)
        return sorted(out_points, key=lambda item: item["total_s"])

    out: dict[tuple[str, str], dict[str, list[dict[str, float]]]] = {}
    for row in rows:
        method = str(row.get("method", ""))
        if method not in {"prm", "bitstar"}:
            continue
        robot = str(row.get("robot", "")).lower()
        difficulty = str(row.get("difficulty", "")).lower()
        if not robot or not difficulty:
            continue
        success = as_float(row.get("success_queries", row.get("success_scenes")), 0.0)
        total = as_float(row.get("total_queries", row.get("scenes")), 0.0)
        plan = method_time(row)
        path_len = path_length_stat(row)
        if total <= 0 or success < total or not math.isfinite(plan) or not math.isfinite(path_len):
            continue
        out.setdefault((robot, difficulty), {}).setdefault(method, []).append(
            {
                "total_s": plan,
                "path_length": path_len,
                "measured_time_s": plan,
            }
        )
    for scenario in out.values():
        for method, points in scenario.items():
            points = sorted(points, key=lambda item: item["total_s"])
            scenario[method] = sparse_bitstar_points(points) if method == "bitstar" else points
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


def find_exp03_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "lect_performance_summary.csv",
        out_dir / "exp03" / "lect_performance_summary.csv",
        out_dir / "exp03_lect_performance" / "lect_performance_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/lect_performance_summary.csv"))
    return matches[0] if matches else None


def generate_exp01_table(path: Path, rows: list[dict[str, Any]]) -> None:
    selected_widths = {"0.02", "0.1", "0.10", "0.5", "0.50"}
    table_rows = [
        row for row in rows
        if str(row.get("width", "")).strip() in selected_widths
    ] or rows
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Endpoint envelope source study at representative interval widths. Vol. is median workspace envelope volume; time is endpoint-envelope construction only. Negative gap is measured against the sampling-union reference.}",
        r"\label{tab:tro-endpoint-envelope}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{1.7pt}",
        r"\begin{tabular}{llrrrr}",
        r"\toprule",
        r"Width & Source & Safe & Vol. & Time ($\mu$s) & Neg. gap \\",
        r"\midrule",
    ]
    for row in table_rows:
        safe = "Y" if str(row.get("safe", "")).lower() == "true" else "N"
        width = tex_num(row.get("width"), 2)
        source = str(row.get("source", "")).replace("_", r"\_")
        volume = tex_sci(row.get("volume_m3_median"), 2)
        time_us = tex_num(row.get("endpoint_us_median"), 2)
        gap = tex_sci(row.get("max_negative_gap"), 1)
        lines.append(f"{width} & {source} & {safe} & {volume} & {time_us} & {gap} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\par\endgroup", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp02_table(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Link envelope representation study. Env. reports envelope construction time only; Coll. reports obstacle collision time.}",
        r"\label{tab:tro-link-envelope}",
        r"\footnotesize",
        r"\setlength{\tabcolsep}{2.2pt}",
        r"\begin{tabular}{llrrrr}",
        r"\toprule",
        r"Width & Envelope & Splits & Vol. & Env. ($\mu$s) & Coll. ($\mu$s) \\",
        r"\midrule",
    ]
    for row in rows:
        width = tex_num(row.get("width"), 2)
        envelope = str(row.get("envelope", "")).replace("_", r"\_")
        splits = int(float(row.get("split_count", 0) or 0))
        volume = tex_num(row.get("volume_m3_median"), 3)
        env_us = tex_num(row.get("envelope_us_median"), 3)
        coll_us = tex_num(row.get("collision_us_median"), 3)
        lines.append(f"{width} & {envelope} & {splits} & {volume} & {env_us} & {coll_us} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\par\endgroup", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp03_table(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{LECT snapshot/cache operation costs. Exact lookup and endpoint lookup are measured on the persisted d23 snapshot.}",
        r"\label{tab:tro-lect-performance}",
        r"\footnotesize",
        r"\setlength{\tabcolsep}{2.4pt}",
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Operation & Ops & Avg. ($\mu$s) & Nodes & Evidence \\",
        r"\midrule",
    ]
    for row in rows:
        operation = str(row.get("operation", "")).replace("_", r"\_")
        ops = int(float(row.get("operations", 0) or 0))
        avg_us = tex_num(row.get("avg_us_per_op"), 3)
        nodes = int(float(row.get("nodes", 0) or 0))
        evidence = int(float(row.get("evidence", 0) or 0))
        lines.append(f"{operation} & {ops} & {avg_us} & {nodes} & {evidence} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\par\endgroup", ""])
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


def find_exp06_current_baseline_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "current_ompl_baselines_001" / "random_robot_summary.csv",
        out_dir / "current_ompl_baselines_001" / "random_robot_summary.csv",
        out_dir / "exp06" / "current_ompl_baselines" / "random_robot_summary.csv",
        out_dir / "current_ompl_baselines" / "random_robot_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_current_baseline_manifest(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "current_ompl_baselines_001" / "random_robot_manifest.json",
        out_dir / "current_ompl_baselines_001" / "random_robot_manifest.json",
        out_dir / "exp06" / "current_ompl_baselines" / "random_robot_manifest.json",
        out_dir / "current_ompl_baselines" / "random_robot_manifest.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_ompl_curve_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "exp06" / "ompl_tradeoff_curves" / "random_robot_summary.csv",
        out_dir / "ompl_tradeoff_curves" / "random_robot_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_exp06_bitstar_trace_summary(out_dir: Path) -> Path | None:
    candidates = [
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


def find_exp06_iris_summaries(out_dir: Path) -> list[Path]:
    candidates = [
        out_dir / "exp06" / "current_iris_gcs_adaptive" / "random_robot_iris_gcs_summary.csv",
        out_dir / "exp06" / "current_iris_gcs" / "random_robot_iris_gcs_summary.csv",
        out_dir / "exp06" / "current_iris_gcs_r16" / "random_robot_iris_gcs_summary.csv",
        out_dir / "exp06" / "current_iris_gcs_r24_it4" / "random_robot_iris_gcs_summary.csv",
        out_dir / "exp06" / "current_iris_gcs_nopre" / "random_robot_iris_gcs_summary.csv",
        out_dir / "current_iris_gcs_adaptive" / "random_robot_iris_gcs_summary.csv",
        out_dir / "current_iris_gcs" / "random_robot_iris_gcs_summary.csv",
        out_dir / "current_iris_gcs_r16" / "random_robot_iris_gcs_summary.csv",
        out_dir / "current_iris_gcs_r24_it4" / "random_robot_iris_gcs_summary.csv",
        out_dir / "current_iris_gcs_nopre" / "random_robot_iris_gcs_summary.csv",
    ]
    out: list[Path] = []
    seen: set[Path] = set()
    for candidate in candidates:
        if candidate.exists() and candidate not in seen:
            out.append(candidate)
            seen.add(candidate)
    return out


def find_exp07_summary(out_dir: Path) -> Path | None:
    candidates = [
        out_dir / "dynamic_update_summary.csv",
        out_dir / "exp07" / "dynamic_update_summary.csv",
        out_dir / "exp07_dynamic_update" / "dynamic_update_summary.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(out_dir.glob("**/dynamic_update_summary.csv"))
    return matches[0] if matches else None


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
    import json

    return json.loads(path.read_text(encoding="utf-8"))


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
            if label not in buckets or not bool(query.get("success", query.get("audit_passed", False))):
                continue
            buckets[label]["query_s"].append(as_float(query.get("query_ms")) / 1000.0)
            buckets[label]["path"].append(as_float(query.get("path_length")))
            buckets[label]["segment"].append(as_float(query.get("segment_fraction")))
    out: dict[str, dict[str, float]] = {}
    for label, values in buckets.items():
        out[label] = {
            "query_s": median(values["query_s"]),
            "path": mean(values["path"]),
            "segment": median(values["segment"]),
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
                    query_s.append(as_float(query.get("query_ms")) / 1000.0)
                    paths.append(as_float(query.get("path_length")))
                    segments.append(as_float(query.get("segment_fraction")))
            if total > 0 and ok == total:
                selected = {
                    "query_s": median(query_s),
                    "path": mean(paths),
                    "segment": median(segments),
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
            "path": as_float(row.get("path_length_mean", row.get("path_length_median"))),
            "segment": as_float(row.get("raw_segment_fraction_median")),
        }
    return stats, success, total, median(builds)


def old_shelf_iris_query_stats() -> dict[str, dict[str, float]]:
    old_table = Path("/home/tian/桌面/box_aabb/cpp/SBF/doc/paper/tro_rewrite_2026/generated/tab_tro_main_shelf_best_tradeoff.tex")
    if not old_table.exists():
        return {}
    stats: dict[str, dict[str, float]] = {}
    for line in old_table.read_text(encoding="utf-8").splitlines():
        if r"\rightarrow" not in line or "&" not in line:
            continue
        cells = [cell.strip() for cell in line.rstrip("\\").split("&")]
        if len(cells) < 5:
            continue
        raw_label = cells[0]
        label = None
        for key, display in QUERY_LABELS.items():
            if display == raw_label:
                label = key
                break
        if label is None:
            continue
        stats[label] = {
            "query_s": as_float(cells[3]),
            "path": as_float(cells[4]),
            "segment": math.nan,
        }
    return stats


def format_method_header(
    label: str,
    build_s: Any | None = None,
    sr: str | None = None,
    *,
    simplify_s: Any | None = None,
    time_label: str = "Build",
) -> str:
    details: list[str] = []
    build = as_float(build_s)
    if math.isfinite(build):
        details.append(rf"{time_label} {tex_num(build, 3)}\,s")
    simplify = as_float(simplify_s)
    if math.isfinite(simplify):
        details.append(rf"Simp {tex_num(simplify, 3)}\,s")
    if sr:
        details.append(rf"SR {sr}")
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
    include_segment: bool = False,
    time_unit: str = "s",
) -> None:
    time_scale = 1000.0 if time_unit == "ms" else 1.0
    per_method_cols = 3 if include_segment else 2
    colspec = "@{}l" + ("rrr|" if include_segment else "rr|") * len(methods)
    colspec = colspec.rstrip("|") + "@{}"
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        rf"\captionof{{table}}{{{caption}}}",
        rf"\label{{{label}}}",
        r"\footnotesize",
        r"\setlength{\tabcolsep}{1.55pt}",
        r"\renewcommand{\arraystretch}{0.96}",
        rf"\begin{{tabular}}{{{colspec}}}",
        r"\toprule",
        "  & "
        + " & ".join(
            rf"\multicolumn{{{per_method_cols}}}{{c}}{{{format_method_header(str(method['label']), method.get('build_s'), method.get('sr'), simplify_s=method.get('simplify_s'), time_label=str(method.get('time_label', 'Build')))}}}"
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
    default_time_metric = rf"$T_{{50}}$ ({time_unit})"
    metric_cols = []
    for method in methods:
        time_metric = str(method.get("time_metric_label") or default_time_metric)
        metric_cols.append(f"{time_metric} & $L_\\mu$ & Seg." if include_segment else f"{time_metric} & $L_\\mu$")
    lines.append("Query & " + " & ".join(metric_cols) + r" \\")
    lines.append(r"\midrule")
    for query_label in QUERY_ORDER:
        cells = [QUERY_LABELS[query_label]]
        for method in methods:
            stats = method.get("queries", {}).get(query_label, {})
            cells.append(tex_num(as_float(stats.get("query_s")) * time_scale))
            cells.append(tex_num(stats.get("path"), 2))
            if include_segment:
                cells.append(tex_num(stats.get("segment"), 2))
        lines.append(" & ".join(cells) + r" \\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\par\endgroup", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp04_table(path: Path, rows: list[dict[str, Any]]) -> None:
    labels = {
        "baseline_d23_aafk_support_hull_8t": "RBF-SH d23",
        "critsample_d23_cache": "CritSample d23",
        "no_cache_full_root_ts": "No-cache full-root",
        "critsample_support_hull": "CritSample endpoints",
        "critsample_support_hull_unsafe": "CritSample",
        "no_external_lect": "No LECT replay, live d17",
        "support_hull_no_aabb": "SH w/o broadphase",
        "link_aabb": "Link AABB",
        "single_thread": "No LECT, 1 thread",
    }
    table_rows = [
        row
        for row in rows
        if int(float(row.get("deep_max_boxes", -1) or -1)) == DEFAULT_RBF_SHELF_BOX_BUDGET
    ]
    if not table_rows:
        table_rows = rows
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Shelf+IIWA RBF ablation at the selected b100 point. Times are in seconds; Path is the success-only mean and Seg. is the raw segment fraction.}",
        r"\label{tab:tro-shelf-ablation}",
        r"\begin{tabular}{lrrrrrrr}",
        r"\toprule",
        r"Case & Build & Batch/q & Amort@5 & Path & Seg. & Boxes & SR \\",
        r"\midrule",
    ]
    for row in table_rows:
        runs = int(float(row.get("total_queries", row.get("runs", 0)) or 0))
        success = int(float(row.get("success_queries", row.get("success_runs", 0)) or 0))
        label = labels.get(str(row.get("case", "")), str(row.get("case", ""))).replace("_", r"\_")
        path_length = path_length_stat(row) if success == runs else None
        segment_fraction = row.get("raw_segment_fraction_median") if success == runs else None
        lines.append(
            f"{label} & {tex_num(row.get('offline_build_s_median', row.get('build_s_median', row.get('build_s'))))} & "
            f"{tex_num(online_query_time(row))} & "
            f"{tex_num(row.get('amortized_s_k5', amortized_query_time(row, 5)))} & "
            f"{tex_num(path_length)} & {tex_num(segment_fraction)} & "
            f"{tex_num(row.get('final_boxes_median'), 1)} & {success}/{runs} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\par\endgroup", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp04_query_table(path: Path, rows: list[dict[str, Any]], manifest: dict[str, Any]) -> None:
    labels = {
        "baseline_d23_aafk_support_hull_8t": "RBF-SH",
        "critsample_support_hull": "CritSample",
        "critsample_support_hull_unsafe": "CritSample",
        "link_aabb": "Link AABB",
        "no_external_lect": "No LECT replay, live d17",
        "single_thread": "1 thread",
    }
    order = [
        "baseline_d23_aafk_support_hull_8t",
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
            r"Shelf+IIWA RBF ablation rows from Fig.~\ref{fig:tro_shelf_tradeoff}, "
            r"reported by query. Each ablation contributes the selected design point "
            r"selected from its measured-time trade-off curve; the full curves remain the primary evidence. "
            r"Query path entries are success-only means over fixed 0.01 joint-space strict-audit paths."
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
) -> None:
    order = ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
    def best_rbf_row() -> dict[str, Any] | None:
        rbf_rows = [row for row in rows if str(row.get("method")) == "sbf_leaf_rrt"]
        return select_registered_rbf_budget_row(rbf_rows)

    def selected_method_row(method: str) -> dict[str, Any] | None:
        items = [row for row in rows if str(row.get("method")) == method]
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
        "sbf_leaf_rrt": "RBF",
        "iris_np_gcs": "IRIS-NP+GCS",
        "prm": "PRM",
        "rrtconnect": "RRTConnect",
        "bitstar": "BIT*",
    }
    rbf_runs = rbf_manifest.get("rows", []) if isinstance(rbf_manifest, dict) else []
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
            single_query_rows = [
                item for item in (rbf_single_query_summary_rows or [])
                if int(float(item.get("deep_max_boxes", deep_boxes) or deep_boxes)) == deep_boxes
            ]
            if single_query_rows:
                query_stats, success, runs, _single_build_s = rbf_single_query_stats_from_summary(single_query_rows)
            else:
                query_stats = query_stats_from_runs(
                    rbf_runs,
                    lambda run, deep_boxes=deep_boxes: (
                        (
                            str(run.get("case")) == "baseline_d23_aafk_support_hull_8t"
                            or str(run.get("method")) == "sbf_leaf_rrt"
                        )
                        and int(float(run.get("deep_max_boxes", 0) or 0)) == deep_boxes
                    ),
                )
            time_metric_label = r"$T_{50}$ (s)"
            label = rf"{labels[method]} b{deep_boxes}"
            build_s = row.get("offline_build_s_median", row.get("build_s", row.get("planning_s_median")))
        elif method == "iris_np_gcs":
            stage_id = str(row.get("stage_id", method))
            query_stats = query_stats_from_runs(
                baseline_runs,
                lambda run, stage_id=stage_id: (
                    str(run.get("method")) == "iris_np_gcs" and str(run.get("stage_id", "iris_np_gcs")) == stage_id
                ),
            )
            if not any(math.isfinite(as_float(value.get("path"))) for value in query_stats.values()):
                query_stats = old_shelf_iris_query_stats()
            label = labels[method]
            build_s = 0.0 if method in {"rrtconnect", "bitstar"} else row.get("offline_build_s_median", row.get("build_s", row.get("planning_s_median")))
        else:
            stage_id = str(row.get("stage_id", method))
            if method == "bitstar":
                query_stats, bitstar_success, bitstar_total = bitstar_first_full_success_query_stats(baseline_runs)
                if bitstar_total > 0:
                    success = bitstar_success
                    runs = bitstar_total
            else:
                query_stats = query_stats_from_runs(
                    baseline_runs,
                    lambda run, method=method, stage_id=stage_id: (
                        str(run.get("method")) == method and str(run.get("stage_id", method)) == stage_id
                    ),
                )
            label = labels[method]
            build_s = 0.0 if method in {"rrtconnect", "bitstar"} else row.get("offline_build_s_median", row.get("build_s", row.get("planning_s_median")))
        time_label = "Build"
        methods.append({
            "label": label,
            "build_s": build_s,
            "simplify_s": None,
            "time_label": time_label,
            "time_metric_label": time_metric_label,
            "sr": f"{success}/{runs}",
            "queries": query_stats,
        })
    grouped_query_table(
        path,
        caption=(
            r"Shelf+IIWA per-query cross-algorithm comparison. $T_{50}$ is median online solve time in seconds; $L_\mu$ is success-only mean path length. RBF uses independent single-query runs and the selected Exp.~4 profile. Full curves appear in \Cref{fig:tro_shelf_cross_tradeoff}."
        ),
        label="tab:tro-shelf-cross-algorithm",
        methods=methods,
        include_segment=False,
        time_unit="s",
    )


def generate_exp05_figure(pdf_path: Path, png_path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
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

    fig, axes = plt.subplots(1, 2, figsize=(5.25, 2.15))
    ax = axes[0]
    path_values: list[float] = []
    for method in method_order:
        items = method_rows.get(method, [])
        if not items:
            continue
        style = METHOD_STYLE.get(method, {"label": method, "color": "0.3", "marker": "o"})
        xs = [method_time(row) for row in items]
        ys = [path_length_stat(row) for row in items]
        path_values.extend(ys)
        if len(items) > 1:
            ax.plot(xs, ys, "-", color=style["color"], alpha=0.62, linewidth=LINE_WIDTH)
        ax.scatter(xs, ys, marker=style["marker"], color=style["color"], s=POINT_SIZE, alpha=0.78, label=style["label"])
        selected = selected_rows.get(method)
        first = first_full_success_row(items)
        if first is not None:
            ax.scatter(
                [method_time(first)],
                [path_length_stat(first)],
                facecolors="none",
                edgecolors="black",
                linewidths=0.9,
                s=28,
                zorder=4,
            )
        if selected is not None:
            ax.scatter(
                [method_time(selected)],
                [path_length_stat(selected)],
                facecolors="none",
                edgecolors="#d4a017",
                linewidths=SELECTED_LINE_WIDTH,
                s=SELECTED_POINT_SIZE,
                zorder=5,
            )
    ax.set_xscale("log")
    ax.set_xlabel("amortized time / query @5 (s, log)")
    ax.set_ylabel("mean audited path length")
    ax.set_title("(a) time / path", fontsize=PANEL_TITLE_FONTSIZE)
    ax.grid(True, which="both", alpha=0.24)
    ax.tick_params(labelsize=TICK_LABEL_FONTSIZE)
    ax.xaxis.label.set_size(AXIS_LABEL_FONTSIZE)
    ax.yaxis.label.set_size(AXIS_LABEL_FONTSIZE)
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
    plot_query_amortization_panel(ax, amortization_methods, title="(b) amortization")

    handles, labels = axes[0].get_legend_handles_labels()
    dedup: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        dedup.setdefault(label, handle)
    fig.legend(dedup.values(), dedup.keys(), loc="upper center", bbox_to_anchor=(0.5, 1.02),
               ncol=5, frameon=False, fontsize=LEGEND_FONTSIZE)
    fig.subplots_adjust(left=0.12, right=0.985, top=0.86, bottom=0.20, wspace=0.34)
    fig.savefig(pdf_path)
    fig.savefig(png_path, dpi=220)
    plt.close(fig)


def generate_exp04_figure(pdf_path: Path, png_path: Path, rows: list[dict[str, Any]]) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    case_order = [
        ("baseline_d23_aafk_support_hull_8t", "RBF-SH", "#1f77b4", "o"),
        ("critsample_support_hull_unsafe", "Crit.", "#17becf", "v"),
        ("critsample_support_hull", "Crit.", "#17becf", "v"),
        ("link_aabb", "Link AABB", "#2ca02c", "s"),
        ("no_external_lect", "No LECT live d17", "#ff7f0e", "D"),
        ("single_thread", "1T", "#9467bd", "^"),
    ]
    selected_rows: dict[str, dict[str, Any]] = {}
    fig, ax = plt.subplots(1, 1, figsize=(3.25, 2.20))
    path_values: list[float] = []
    for case, label, color, marker in case_order:
        items = sorted(
            [row for row in rows if str(row.get("case")) == case and full_success(row)],
            key=measured_time_key,
        )
        if not items:
            continue
        selected = select_tradeoff_row(items, budget_field="deep_max_boxes")
        if selected is not None:
            selected_rows[case] = selected
        xs = [method_time(row) for row in items]
        ys = [path_length_stat(row) for row in items]
        path_values.extend(ys)
        ax.plot(xs, ys, "-", color=color, alpha=0.62, linewidth=LINE_WIDTH)
        ax.scatter(xs, ys, marker=marker, color=color, s=POINT_SIZE, alpha=0.78, label=label)
        selected_path = None if selected is None else path_length_stat(selected)
        if selected is not None and math.isfinite(selected_path):
            ax.scatter(
                [method_time(selected)],
                [selected_path],
                facecolors="none",
                edgecolors="#d4a017",
                linewidths=SELECTED_LINE_WIDTH,
                s=SELECTED_POINT_SIZE,
                zorder=5,
            )
    ax.set_xscale("log")
    ax.set_xlabel("amortized time / query @5 (s, log)")
    ax.set_ylabel("mean audited path length")
    ax.grid(True, which="both", alpha=0.24)
    ax.tick_params(labelsize=TICK_LABEL_FONTSIZE)
    ax.xaxis.label.set_size(AXIS_LABEL_FONTSIZE)
    ax.yaxis.label.set_size(AXIS_LABEL_FONTSIZE)
    set_padded_linear_ylim(ax, path_values)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 1.02),
               ncol=5, frameon=False, fontsize=LEGEND_FONTSIZE, handlelength=1.1)
    fig.subplots_adjust(left=0.21, right=0.97, top=0.88, bottom=0.21)
    fig.savefig(pdf_path)
    fig.savefig(png_path, dpi=220)
    plt.close(fig)


def generate_exp04_assets(generated: Path, out_dir: Path) -> dict[str, Any]:
    summary = find_exp04_summary(out_dir)
    if summary is None:
        return {"status": "missing", "summary": None}
    rows = read_csv_rows(summary)
    manifest_path = find_exp04_manifest(out_dir)
    manifest = load_json_file(manifest_path)
    table_path = generated / "tab_tro_shelf_ablation.tex"
    pdf_path = generated / "fig_tro_shelf_tradeoff.pdf"
    png_path = generated / "fig_tro_shelf_tradeoff.png"
    generate_exp04_table(table_path, rows)
    generate_exp04_figure(
        pdf_path,
        png_path,
        rows,
    )
    return {
        "status": "generated",
        "summary": str(summary),
        "summary_sha256": file_sha256(summary),
        "manifest": str(manifest_path) if manifest_path is not None else None,
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
    current_iris_summary = find_exp05_current_iris_summary(out_dir)
    current_iris_manifest = find_exp05_current_iris_manifest(out_dir)
    bitstar_trace_summary = find_exp05_bitstar_trace_summary(out_dir)
    bitstar_trace_manifest = find_exp05_bitstar_trace_manifest(out_dir)
    rbf_single_query_summary = find_exp05_rbf_single_query_summary(out_dir)
    rbf_single_query_manifest = find_exp05_rbf_single_query_manifest(out_dir)
    rbf_single_query_rows = read_csv_rows(rbf_single_query_summary) if rbf_single_query_summary is not None else []
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
    current_iris_manifest_payload = load_json_file(current_iris_manifest)
    if current_iris_summary is not None:
        current_iris_rows = [
            row for row in read_csv_rows(current_iris_summary)
            if str(row.get("method")) == "iris_np_gcs"
        ]
        if current_iris_rows:
            rows = [row for row in rows if str(row.get("method")) != "iris_np_gcs"]
            rows.extend(current_iris_rows)
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
        if current_iris_manifest is not None:
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
    if exp04_summary is not None:
        exp04_rows = [
            row for row in read_csv_rows(exp04_summary)
            if str(row.get("case")) == "baseline_d23_aafk_support_hull_8t"
        ]
        if exp04_rows:
            non_rbf_rows = [row for row in rows if str(row.get("method")) != "sbf_leaf_rrt"]
            rows = non_rbf_rows + [
                {
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
                    "online_per_query_s_median": row.get("online_per_query_s_median", ""),
                    "online_total_per_query_s_median": row.get("online_total_per_query_s_median", ""),
                    "online_solve_s_median": row.get("online_solve_s_median", ""),
                    "online_simplify_s_median": row.get("online_simplify_s_median", ""),
                    "online_solve_per_query_s_median": row.get("online_solve_per_query_s_median", ""),
                    "online_simplify_per_query_s_median": row.get("online_simplify_per_query_s_median", ""),
                    "amortized_s_k1": row.get("amortized_s_k1", ""),
                    "amortized_s_k5": row.get("amortized_s_k5", ""),
                    "amortized_s_k10": row.get("amortized_s_k10", ""),
                    "amortized_s_k20": row.get("amortized_s_k20", ""),
                    "amortized_s_k50": row.get("amortized_s_k50", ""),
                    "planning_s_median": row.get("planning_s_median"),
                    "audit_s_median": row.get("audit_s_median"),
                    "path_length_mean": row.get("path_length_mean", row.get("path_length_median")),
                    "raw_segment_fraction_median": row.get("raw_segment_fraction_median"),
                    "status": "exp04_registered_profile",
                }
                for row in exp04_rows
            ]
    table_path = generated / "tab_tro_shelf_cross_algorithm.tex"
    pdf_path = generated / "fig_tro_shelf_cross_tradeoff.pdf"
    png_path = generated / "fig_tro_shelf_cross_tradeoff.png"
    generate_exp05_table(
        table_path,
        rows,
        rbf_manifest=rbf_manifest,
        baseline_manifest=baseline_manifest,
        rbf_single_query_summary_rows=rbf_single_query_rows,
    )
    generate_exp05_figure(pdf_path, png_path, rows)
    return {
        "status": "generated",
        "summary": str(summary),
        "summary_sha256": file_sha256(summary),
        "current_baseline_summary": str(current_baseline_summary) if current_baseline_summary is not None else None,
        "current_baseline_manifest": str(current_baseline_manifest) if current_baseline_manifest is not None else None,
        "current_iris_summary": str(current_iris_summary) if current_iris_summary is not None else None,
        "current_iris_summary_sha256": file_sha256(current_iris_summary) if current_iris_summary is not None else None,
        "current_iris_manifest": str(current_iris_manifest) if current_iris_manifest is not None else None,
        "current_iris_manifest_sha256": file_sha256(current_iris_manifest) if current_iris_manifest is not None else None,
        "current_iris_context_policy": "legacy_shelf_context_restored_without_0p01_simplify_manifest_filter",
        "bitstar_trace_summary": str(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_summary_sha256": file_sha256(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_manifest": str(bitstar_trace_manifest) if bitstar_trace_manifest is not None else None,
        "bitstar_trace_manifest_sha256": file_sha256(bitstar_trace_manifest) if bitstar_trace_manifest is not None else None,
        "registered_rrtconnect_summary": str(REGISTERED_EXP05_RRTCONNECT_SUMMARY) if registered_rrt_rows else None,
        "registered_rrtconnect_rows": len(registered_rrt_rows),
        "registered_rrtconnect_manifest": str(REGISTERED_EXP05_RRTCONNECT_MANIFEST) if registered_rrt_manifest_rows else None,
        "registered_rrtconnect_manifest_rows": len(registered_rrt_manifest_rows),
        "rbf_manifest": str(rbf_manifest_path) if rbf_manifest_path is not None else None,
        "rbf_query_manifest": str(rbf_query_manifest_path) if rbf_query_manifest_path is not None else None,
        "rbf_query_manifest_sha256": file_sha256(rbf_query_manifest_path) if rbf_query_manifest_path is not None else None,
        "rbf_single_query_summary": str(rbf_single_query_summary) if rbf_single_query_summary is not None else None,
        "rbf_single_query_summary_sha256": file_sha256(rbf_single_query_summary) if rbf_single_query_summary is not None else None,
        "rbf_single_query_manifest": str(rbf_single_query_manifest) if rbf_single_query_manifest is not None else None,
        "rbf_single_query_manifest_sha256": file_sha256(rbf_single_query_manifest) if rbf_single_query_manifest is not None else None,
        "exp04_registered_summary": str(exp04_summary) if exp04_summary is not None else None,
        "rows": len(rows),
        "table": str(table_path),
        "figure_pdf": str(pdf_path),
        "figure_png": str(png_path),
    }


def select_best_budget_rows(rows: list[dict[str, Any]], group_fields: list[str]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
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
        out.append(sorted(
            candidates,
            key=lambda row: (
                online_query_time(row) if math.isfinite(online_query_time(row)) else measured_time_key(row),
                amortized_query_time(row, 10) if math.isfinite(amortized_query_time(row, 10)) else measured_time_key(row),
                path_length_stat(row) if math.isfinite(path_length_stat(row)) else 1e9,
                int(float(row.get("deep_max_boxes", 0) or 0)),
            ),
        )[0])
    return out


def generate_sbf_budget_figure(pdf_path: Path,
                               png_path: Path,
                               rows: list[dict[str, Any]],
                               title: str,
                               group_fields: list[str]) -> None:
    if not rows:
        return
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def group_label(row: dict[str, Any]) -> str:
        if not group_fields:
            return "RBF"
        return "/".join(str(row.get(field, "")) for field in group_fields)

    groups = sorted({group_label(row) for row in rows})
    fig, axes = plt.subplots(1, 2, figsize=(6.6, 2.65), constrained_layout=True)
    metrics = [
        ("raw_segment_fraction_median", "Raw segment fraction"),
        ("path_length_mean", "Mean audited path length"),
    ]
    for group in groups:
        items = sorted([row for row in rows if group_label(row) == group], key=measured_time_key)
        x = [method_time(row) for row in items]
        for axis, (field, ylabel) in zip(axes, metrics):
            y = [path_length_stat(row) if field == "path_length_mean" else float(row.get(field, "nan")) for row in items]
            axis.plot(x, y, marker="o", linewidth=1.5, label=group)
            for bx, by, row in zip(x, y, items):
                ok = int(float(row.get("success_scenes", row.get("success_runs", 0)) or 0))
                n = int(float(row.get("scenes", row.get("runs", 0)) or 0))
                if n > 0 and ok < n:
                    axis.plot([bx], [by], marker="x", color="black", markersize=6, mew=1.4)
    for axis, (_, ylabel) in zip(axes, metrics):
        axis.set_xscale("log")
        axis.set_xlabel("Measured planning time (s)")
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
    rbf_rows = [row for row in rows if str(row.get("method")) == "sbf_leaf_rrt"]
    selected_rows = select_best_budget_rows(rbf_rows, ["robot", "difficulty"])
    context, has_current_baselines = merged_random_context(rows)
    selected_by_key = {
        (str(row.get("robot")).lower(), str(row.get("difficulty")).lower()): row
        for row in selected_rows
    }
    robot_order = ["iiwa", "ur5", "panda"]
    difficulty_order = ["easy", "medium", "hard"]
    lines = [
        r"% Auto-generated from current trade-off artifacts.",
        r"\begingroup",
        r"\centering",
            (
            r"\captionof{table}{Saved-catalog random-scene best trade-off points. Time columns are medians in seconds; $L_\mu$ is success-only mean path length. RBF Online/q excludes final simplification. Full curves appear in \Cref{fig:tro_random_tradeoff}.}"
            if has_current_baselines else
            r"\captionof{table}{Saved-catalog random-scene best trade-off points. Time columns are medians in seconds; $L_\mu$ is success-only mean path length. RBF Online/q excludes final simplification. Full curves appear in \Cref{fig:tro_random_tradeoff}.}"
        ),
        r"\label{tab:tro-random-summary}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{1.35pt}",
        r"\begin{tabular}{lrrrrrrr|rr|rr|rr|rr}",
        r"\toprule",
        r"Scenario & \multicolumn{7}{c|}{RBF} & \multicolumn{2}{c|}{IRIS-NP+GCS} & \multicolumn{2}{c|}{PRM} & \multicolumn{2}{c|}{RRTConnect} & \multicolumn{2}{c}{BIT*} \\",
        r"\cmidrule(lr){2-8}\cmidrule(lr){9-10}\cmidrule(lr){11-12}\cmidrule(lr){13-14}\cmidrule(lr){15-16}",
        r" & Build$_{50}$ & Online/q$_{50}$ & Simplify/q$_{50}$ & Amort@10$_{50}$ & $L_\mu$ & Boxes & SR & $T_{50}$ (s) & $L_\mu$ & $T_{50}$ (s) & $L_\mu$ & $T_{50}$ (s) & $L_\mu$ & $T_{50}$ (s) & $L_\mu$ \\",
        r"\midrule",
    ]
    for robot in robot_order:
        for difficulty in difficulty_order:
            row = selected_by_key.get((robot, difficulty))
            if row is None:
                continue
            scenario = f"{robot.upper()}-{difficulty.capitalize()}"
            scenario_context = context.get((robot, difficulty), {})
            cells = []
            for method in ["iris_np_gcs", "prm", "rrtconnect", "bitstar"]:
                item = scenario_context.get(method, {})
                cells.extend([tex_num(item.get("total_s")), tex_num(item.get("path_length"), 2)])
            online_q = online_query_time(row)
            if not math.isfinite(online_q):
                online_q = as_float(row.get("planning_s_median", row.get("measured_time_s_median")))
            amortized_k10 = amortized_query_time(row, 10)
            if not math.isfinite(amortized_k10):
                amortized_k10 = online_q
            success = int(float(row.get("success_queries", row.get("success_runs", 0)) or 0))
            total = int(float(row.get("total_queries", row.get("runs", 0)) or 0))
            lines.append(
                f"{scenario} & {tex_num(row.get('offline_build_s_median', row.get('build_s')))} & "
                f"{tex_num(online_q)} & {tex_num(row.get('online_simplify_per_query_s_median'))} & "
                f"{tex_num(amortized_k10)} & "
                f"{tex_num(path_length_stat(row), 2)} & {int(float(row.get('deep_max_boxes', 0) or 0))} & "
                f"{success}/{total} & "
                + " & ".join(cells)
                + r" \\"
            )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\par\endgroup", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp06_figure(pdf_path: Path, png_path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    context, _has_current_baselines = merged_random_context(rows)
    current_curves = current_random_curves_from_rows(rows)
    robot_order = ["iiwa", "ur5", "panda"]
    difficulty_order = ["easy", "medium", "hard"]
    robot_labels = {"iiwa": "IIWA", "ur5": "UR5", "panda": "Panda"}
    rbf_selected = {
        (str(row.get("robot")).lower(), str(row.get("difficulty")).lower()): row
        for row in select_best_budget_rows(
            [row for row in rows if str(row.get("method")) == "sbf_leaf_rrt"],
            ["robot", "difficulty"],
        )
    }

    fig, axes = plt.subplots(
        len(robot_order),
        len(difficulty_order) + 1,
        figsize=(11.2, 6.9),
    )
    for row_index, robot in enumerate(robot_order):
        robot_path_values: list[float] = []
        for difficulty in difficulty_order:
            robot_path_values.extend([
                path_length_stat(row)
                for row in rows
                if str(row.get("method")) == "sbf_leaf_rrt"
                and str(row.get("robot")).lower() == robot
                and str(row.get("difficulty")).lower() == difficulty
            ])
            robot_path_values.extend([
                item["path_length"]
                for item in context.get((robot, difficulty), {}).values()
                if math.isfinite(item.get("path_length", math.nan))
            ])
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
            if items:
                xs = [amortized_query_time(row, 10) for row in items]
                ys = [path_length_stat(row) for row in items]
                axis.plot(xs, ys, "-", color=METHOD_STYLE["sbf_leaf_rrt"]["color"],
                          alpha=0.60, linewidth=LINE_WIDTH)
                axis.scatter(xs, ys, marker=METHOD_STYLE["sbf_leaf_rrt"]["marker"],
                             color=METHOD_STYLE["sbf_leaf_rrt"]["color"], s=POINT_SIZE, alpha=0.82)
                first = first_full_success_row(items)
                if first is not None:
                    axis.scatter(
                        [amortized_query_time(first, 10)],
                        [path_length_stat(first)],
                        facecolors="none",
                        edgecolors="black",
                        linewidths=0.9,
                        s=28,
                        zorder=4,
                    )
                selected = rbf_selected.get((robot, difficulty))
                if selected is not None:
                    axis.scatter(
                        [amortized_query_time(selected, 10)],
                        [path_length_stat(selected)],
                        facecolors="none",
                        edgecolors="#d4a017",
                        linewidths=SELECTED_LINE_WIDTH,
                        s=SELECTED_POINT_SIZE,
                        zorder=5,
                    )
            scenario_context = context.get((robot, difficulty), {})
            scenario_curves = current_curves.get((robot, difficulty), {})
            for method in ["prm", "bitstar"]:
                points = scenario_curves.get(method, [])
                if len(points) < 2:
                    continue
                style = METHOD_STYLE[method]
                xs = [point["total_s"] for point in points]
                ys = [point["path_length"] for point in points]
                axis.plot(xs, ys, "-", color=style["color"], alpha=0.42, linewidth=LINE_WIDTH)
                axis.scatter(xs, ys, marker=style["marker"], color=style["color"], s=POINT_SIZE, alpha=0.58)
                axis.scatter(
                    [xs[0]],
                    [ys[0]],
                    facecolors="none",
                    edgecolors="black",
                    linewidths=0.8,
                    s=24,
                    zorder=4,
                )
            for method in ["iris_np_gcs", "prm", "rrtconnect", "bitstar"]:
                item = scenario_context.get(method)
                if not item:
                    continue
                style = METHOD_STYLE[method]
                axis.scatter([item["total_s"]], [item["path_length"]],
                             marker=style["marker"], color=style["color"], s=POINT_SIZE, alpha=0.82)
            axis.set_xscale("log")
            axis.set_xlabel("amortized/online time per query (s, log)" if row_index == len(robot_order) - 1 else "")
            if row_index == 0:
                axis.set_title(difficulty.capitalize(), fontsize=PANEL_TITLE_FONTSIZE)
            if col_index == 0:
                axis.set_ylabel(f"{robot_labels[robot]}\nmean audited path length")
            axis.grid(True, which="both", alpha=0.22)
            axis.tick_params(labelsize=TICK_LABEL_FONTSIZE)
            axis.xaxis.label.set_size(AXIS_LABEL_FONTSIZE)
            axis.yaxis.label.set_size(AXIS_LABEL_FONTSIZE)
            set_padded_linear_ylim(axis, robot_path_values)

        amort_methods: list[dict[str, Any]] = []
        for method in ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]:
            build_values: list[float] = []
            query_values: list[float] = []
            for difficulty in difficulty_order:
                if method == "sbf_leaf_rrt":
                    selected = rbf_selected.get((robot, difficulty))
                    if selected is None:
                        continue
                    build_values.append(as_float(selected.get("offline_build_s_median"), 0.0))
                    query_values.append(online_query_time(selected))
                else:
                    item = context.get((robot, difficulty), {}).get(method)
                    if not item:
                        continue
                    build_values.append(as_float(item.get("build_s"), 0.0))
                    query_values.append(as_float(item.get("query_s"), item.get("total_s", 0.0)))
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
            title="Amortization" if row_index == 0 else "",
            show_xlabel=row_index == len(robot_order) - 1,
            show_ylabel=False,
        )

    handles = []
    labels = []
    for method in ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]:
        style = METHOD_STYLE[method]
        handles.append(
            plt.Line2D([0], [0], color=style["color"], marker=style["marker"],
                       linestyle="-" if method == "sbf_leaf_rrt" else "None",
                       markersize=4, linewidth=LINE_WIDTH)
        )
        labels.append(style["label"])
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.0),
        ncol=5,
        frameon=False,
        fontsize=max(LEGEND_FONTSIZE, 7.2),
        handlelength=2.4,
        handletextpad=0.4,
        columnspacing=0.85,
        labelspacing=0.45,
    )
    fig.subplots_adjust(left=0.072, right=0.992, top=0.94, bottom=0.055, hspace=0.18, wspace=0.16)
    fig.savefig(pdf_path)
    fig.savefig(png_path, dpi=220)
    plt.close(fig)


def generate_exp06_assets(generated: Path, out_dir: Path, *, include_current_baselines: bool = False) -> dict[str, Any]:
    summary = find_exp06_summary(out_dir)
    if summary is None:
        return {"status": "missing", "summary": None}
    rows = read_csv_rows(summary)
    baseline_summary = find_exp06_current_baseline_summary(out_dir) if include_current_baselines else None
    baseline_manifest = find_exp06_current_baseline_manifest(out_dir) if include_current_baselines else None
    curve_summary = find_exp06_ompl_curve_summary(out_dir)
    bitstar_trace_summary = find_exp06_bitstar_trace_summary(out_dir)
    bitstar_trace_manifest = find_exp06_bitstar_trace_manifest(out_dir)
    iris_summaries = find_exp06_iris_summaries(out_dir)
    baseline_rows: list[dict[str, Any]] = []
    baseline_manifest_payload = load_json_file(baseline_manifest)
    if baseline_summary is not None and manifest_uses_required_simplify(baseline_manifest_payload):
        baseline_rows = [
            row for row in read_csv_rows(baseline_summary)
            if str(row.get("method")) != "sbf_leaf_rrt"
        ]
    iris_rows: list[dict[str, Any]] = []
    accepted_iris_summaries: list[Path] = []
    for iris_summary in iris_summaries:
        accepted_iris_summaries.append(iris_summary)
        iris_rows.extend(
            row for row in read_csv_rows(iris_summary)
            if str(row.get("method")) == "iris_np_gcs" and is_full_success(row)
        )
    curve_rows: list[dict[str, Any]] = []
    if curve_summary is not None:
        curve_rows = [
            row for row in read_csv_rows(curve_summary)
            if str(row.get("method")) in {"prm", "bitstar"} and is_full_success(row)
        ]
    bitstar_trace_manifest_payload = load_json_file(bitstar_trace_manifest)
    if bitstar_trace_summary is not None and manifest_uses_required_simplify(bitstar_trace_manifest_payload):
        bitstar_rows = [
            row for row in read_csv_rows(bitstar_trace_summary)
            if str(row.get("method")) == "bitstar" and is_full_success(row)
        ]
        if bitstar_rows:
            curve_rows = [row for row in curve_rows if str(row.get("method")) != "bitstar"]
            curve_rows.extend(bitstar_rows)
    table_rows = rows + baseline_rows + curve_rows + iris_rows
    figure_rows = rows + baseline_rows + curve_rows + iris_rows
    table_path = generated / "tab_tro_random_summary.tex"
    pdf_path = generated / "fig_tro_random_tradeoff.pdf"
    png_path = generated / "fig_tro_random_tradeoff.png"
    generate_exp06_table(table_path, table_rows)
    generate_exp06_figure(pdf_path, png_path, figure_rows)
    return {
        "status": "generated",
        "summary": str(summary),
        "summary_sha256": file_sha256(summary),
        "include_current_baselines": include_current_baselines,
        "current_baseline_summary": str(baseline_summary) if baseline_summary is not None else None,
        "current_baseline_summary_sha256": file_sha256(baseline_summary) if baseline_summary is not None else None,
        "current_baseline_manifest": str(baseline_manifest) if baseline_manifest is not None else None,
        "current_baseline_manifest_uses_required_simplify": manifest_uses_required_simplify(baseline_manifest_payload),
        "current_baseline_rows": len(baseline_rows),
        "current_iris_summaries": [str(path) for path in iris_summaries],
        "current_iris_summary_sha256": {str(path): file_sha256(path) for path in iris_summaries},
        "current_iris_accepted_summaries": [str(path) for path in accepted_iris_summaries],
        "current_iris_context_policy": "legacy_random_context_restored_without_0p01_simplify_manifest_filter",
        "current_iris_rows": len(iris_rows),
        "ompl_curve_summary": str(curve_summary) if curve_summary is not None else None,
        "ompl_curve_summary_sha256": file_sha256(curve_summary) if curve_summary is not None else None,
        "bitstar_trace_summary": str(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_summary_sha256": file_sha256(bitstar_trace_summary) if bitstar_trace_summary is not None else None,
        "bitstar_trace_manifest": str(bitstar_trace_manifest) if bitstar_trace_manifest is not None else None,
        "bitstar_trace_manifest_uses_required_simplify": manifest_uses_required_simplify(bitstar_trace_manifest_payload),
        "ompl_curve_rows_100pct": len(curve_rows),
        "registered_baseline_context": not bool(baseline_rows or curve_rows or iris_rows),
        "old_random_context_table": str(OLD_RANDOM_TABLE) if OLD_RANDOM_TABLE.exists() else None,
        "rows": len(table_rows),
        "figure_rows": len(figure_rows),
        "table": str(table_path),
        "figure_pdf": str(pdf_path),
        "figure_png": str(png_path),
    }


def generate_exp07_table(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Saved-catalog dynamic-update results. Update time is compared with a fresh warm rebuild on the target scene; final audit time is excluded.}",
        r"\label{tab:tro-dynamic-update}",
        r"\footnotesize",
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Transition & SR & Update (s) & Warm (s) & Speedup \\",
        r"\midrule",
    ]
    for row in rows:
        sr = f"{int(float(row.get('success_runs', 0) or 0))}/{int(float(row.get('runs', 0) or 0))}"
        lines.append(
            f"{row.get('transition')} & {sr} & {tex_num(row.get('update_s_median'))} & "
            f"{tex_num(row.get('warm_rebuild_s_median'))} & {tex_num(row.get('speedup_median'))} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_exp07_assets(generated: Path, out_dir: Path) -> dict[str, Any]:
    summary = find_exp07_summary(out_dir)
    if summary is None:
        return {"status": "missing", "summary": None}
    rows = read_csv_rows(summary)
    table_path = generated / "tab_tro_dynamic_update.tex"
    generate_exp07_table(table_path, rows)
    return {
        "status": "generated",
        "summary": str(summary),
        "summary_sha256": file_sha256(summary),
        "rows": len(rows),
        "table": str(table_path),
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
    exp01_summary = find_exp01_summary(args.out_dir)
    if exp01_summary is not None:
        generate_exp01_table(generated / "tab_tro_endpoint_envelope.tex", read_csv_rows(exp01_summary))
    manifest["sources"]["exp01_endpoint_envelope"] = {
        "status": "generated" if exp01_summary is not None else "missing",
        "summary": str(exp01_summary) if exp01_summary is not None else None,
        "summary_sha256": file_sha256(exp01_summary) if exp01_summary is not None else None,
        "table": str(generated / "tab_tro_endpoint_envelope.tex"),
    }
    exp02_summary = find_exp02_summary(args.out_dir)
    if exp02_summary is not None:
        generate_exp02_table(generated / "tab_tro_link_envelope.tex", read_csv_rows(exp02_summary))
    manifest["sources"]["exp02_link_envelope"] = {
        "status": "generated" if exp02_summary is not None else "missing",
        "summary": str(exp02_summary) if exp02_summary is not None else None,
        "summary_sha256": file_sha256(exp02_summary) if exp02_summary is not None else None,
        "table": str(generated / "tab_tro_link_envelope.tex"),
    }
    exp03_summary = find_exp03_summary(args.out_dir)
    if exp03_summary is not None:
        generate_exp03_table(generated / "tab_tro_lect_performance.tex", read_csv_rows(exp03_summary))
    manifest["sources"]["exp03_lect_performance"] = {
        "status": "generated" if exp03_summary is not None else "missing",
        "summary": str(exp03_summary) if exp03_summary is not None else None,
        "summary_sha256": file_sha256(exp03_summary) if exp03_summary is not None else None,
        "table": str(generated / "tab_tro_lect_performance.tex"),
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
    exp07 = generate_exp07_assets(generated, args.out_dir)
    manifest["sources"]["exp07_dynamic_update"] = exp07
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
    if exp03_summary is not None:
        generated_tables.add("tab_tro_lect_performance.tex")
    if exp04.get("status") == "generated":
        generated_tables.add("tab_tro_shelf_ablation.tex")
    if exp05.get("status") == "generated":
        generated_tables.add("tab_tro_shelf_cross_algorithm.tex")
    if exp06.get("status") == "generated":
        generated_tables.add("tab_tro_random_summary.tex")
    if exp07.get("status") == "generated":
        generated_tables.add("tab_tro_dynamic_update.tex")
    for filename, caption in REQUIRED_TABLES.items():
        path = generated / filename
        label = "tab:" + filename.removesuffix(".tex").replace("_", "-")
        is_placeholder = filename not in generated_tables
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
    write_json(generated / "tro_table_generation_manifest.json", manifest)
    print(f"wrote {generated / 'tro_table_generation_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
