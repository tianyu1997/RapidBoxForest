#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parents[1]
DEFAULT_OUT_DIR = ROOT / "doc" / "paper" / "tro_rewrite_2026" / "generated"
PAPER_OUTPUTS = ROOT / "outputs" / "paper"
# Keep the paper-facing LECT reuse table pinned to the archived ~250-box artifact.
# The mutable default JSON is reused by later reruns (for example 512-box studies)
# and must not silently replace the paper table inputs.
MAIN_LECT_REUSE_ARTIFACT = "tro2026_iiwa_lect_incremental_reuse_boxes256_paper.json"
SBF_SH_LABEL = "SBF-SH"
SBF_SH_DEFINITION = "AABB--KDOP26--SupportHull cascade"
FULL_SUCCESS_THRESHOLD = 0.999
PATH_TRADEOFF_TOL = 0.08
PATH_DOMINATION_EPS = 1e-4
PATH_LOG_TIME_PENALTY_RAD = 0.16
PATH_LOG_TIME_OFFSET_S = 0.5
TABLE_FONT_REGULAR = r"\small"
TABLE_FONT_DENSE = r"\footnotesize"
TABLE_HEADER_FONT = r"\footnotesize"
TABLE_CAPTION_FONT = r"\small"
PATH_QUALITY_SCORE_WEIGHT = 2.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate TRO rewrite LaTeX tables from standalone SBF paper outputs.")
    parser.add_argument("--outputs", type=Path, default=PAPER_OUTPUTS)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--mode", choices=["main", "appendix", "all"], default="main", help="Generate compressed main-text tables, appendix/detail tables, or both.")
    parser.add_argument("--strict-missing", action="store_true", help="Fail if a generated table has no backing artifact rows.")
    parser.add_argument("--manifest", type=Path, default=None, help="Optional JSON manifest for generated and missing tables.")
    parser.add_argument("--placeholder", action="store_true", help="Generate blank main-text placeholder tables without reading experiment artifacts.")
    return parser.parse_args()


def import_pyplot() -> Any:
    import matplotlib

    matplotlib.use("Agg")
    matplotlib.rcParams.update({
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    import matplotlib.pyplot as plt

    return plt


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))

def anytime_summary_points(payload: dict[str, Any] | None) -> list[dict[str, Any]]:
    if not isinstance(payload, dict):
        return []
    records = payload.get("records")
    if isinstance(records, list) and records:
        try:
            from common_anytime_tradeoff import aggregate_stage_records
        except Exception:
            try:
                import importlib.util

                helper_path = ROOT / "experiments" / "common_anytime_tradeoff.py"
                spec = importlib.util.spec_from_file_location("tro_common_anytime_tradeoff", helper_path)
                if spec is not None and spec.loader is not None:
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                    aggregate_stage_records = module.aggregate_stage_records
                else:
                    aggregate_stage_records = None
            except Exception:
                aggregate_stage_records = None
        if aggregate_stage_records is not None:
            try:
                summary = aggregate_stage_records(records)
                points = list(((summary or {}).get("points", [])))
                if points:
                    return points
            except Exception:
                pass
    return list(((payload.get("summary", {}) or {}).get("points", [])))


def first_existing(root: Path, names: Iterable[str]) -> tuple[Path | None, dict[str, Any] | None]:
    for name in names:
        path = root / name
        payload = load_json(path)
        if payload is not None:
            return path, payload
    return None, None


def mean(values: Iterable[float]) -> float | None:
    rows = [float(value) for value in values if value is not None]
    return statistics.fmean(rows) if rows else None


def median(values: Iterable[float]) -> float | None:
    rows = [float(value) for value in values if value is not None]
    return statistics.median(rows) if rows else None


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def tex_escape(value: Any) -> str:
    text = str(value)
    return (
        text.replace("\\", r"\textbackslash{}")
        .replace("&", r"\&")
        .replace("%", r"\%")
        .replace("$", r"\$")
        .replace("#", r"\#")
        .replace("_", r"\_")
        .replace("{", r"\{")
        .replace("}", r"\}")
    )


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "--"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return str(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return tex_escape(value)
    if abs(number) >= 1000:
        return f"{number:.1f}"
    if abs(number) >= 100:
        return f"{number:.2f}"
    if abs(number) >= 10:
        return f"{number:.2f}"
    if abs(number) >= 1:
        return f"{number:.{digits}f}"
    if abs(number) >= 1e-3:
        return f"{number:.{digits + 1}f}"
    if number == 0:
        return "0"
    return f"{number:.6f}"


def is_grid_row(row: dict[str, Any]) -> bool:
    text = " ".join(str(row.get(key, "")) for key in ("variant", "label", "envelope_type", "method"))
    return "grid" in text.lower() or "hull_d" in text.lower()


def method_label(value: Any) -> str:
    text = str(value)
    replacements = {
        "SBF AABB+KDOP26+SupportHull": SBF_SH_LABEL,
        "AABB+KDOP26+SupportHull": SBF_SH_LABEL,
        "Crit+KDOP26+SupportHull": SBF_SH_LABEL,
        "SBF (AABB/KDOP/SupportHull)": SBF_SH_LABEL,
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def variant_label(name: Any) -> str:
    labels = {
        "ifk_strict": "IFK+LinkIAABB",
        "ifk_linkiaabb": "IFK+LinkIAABB",
        "ifk_kdop26": "IFK+KDOP26",
        "ifk_support_hull": "IFK+SupportHull",
        "ifk_sh": "IFK+SH",
        "crit_link_coverage": "Crit+LinkIAABB",
        "crit_linkiaabb": "Crit+LinkIAABB",
        "kdop26_coverage": "Crit+KDOP26",
        "crit_kdop26": "Crit+KDOP26",
        "crit_support_hull": "Crit+SupportHull",
        "crit_sh": "Crit+SH",
        "support_hull_coverage": SBF_SH_LABEL,
        "coverage_hybrid": "Crit+HullGrid",
    }
    return labels.get(str(name), str(name))


def table(
    caption: str,
    label: str,
    columns: str,
    header: list[str],
    rows: list[list[Any]],
    *,
    font_command: str = TABLE_FONT_REGULAR,
    tabcolsep: float | None = None,
    resize_width: str | None = None,
    float_spec: str = "[t]",
    arraystretch: float | None = None,
) -> str:
    body = []
    body.append(rf"\begin{{table}}{float_spec}")
    body.append(r"\centering")
    body.append(r"\caption{" + caption + r"}")
    body.append(r"\label{" + label + r"}")
    body.append(font_command)
    if tabcolsep is not None:
        body.append(rf"\setlength{{\tabcolsep}}{{{tabcolsep:.2f}pt}}")
    if arraystretch is not None:
        body.append(rf"\renewcommand{{\arraystretch}}{{{arraystretch:.2f}}}")
    if resize_width is not None:
        body.append(rf"\resizebox{{{resize_width}}}{{!}}{{%")
    body.append(r"\begin{tabular}{" + columns + r"}")
    body.append(r"\toprule")
    body.append(" & ".join(header) + r" \\")
    body.append(r"\midrule")
    if rows:
        for row in rows:
            body.append(" & ".join(tex_escape(item) if isinstance(item, str) else fmt(item) for item in row) + r" \\")
    else:
        body.append(r"\multicolumn{" + str(len(header)) + r"}{c}{No result artifact available.} \\")
    body.append(r"\bottomrule")
    body.append(r"\end{tabular}")
    if resize_width is not None:
        body.append(r"}")
    body.append(r"\end{table}")
    body.append("")
    return "\n".join(body)


def table_star(
    caption: str,
    label: str,
    columns: str,
    header: list[str],
    rows: list[list[Any]],
    *,
    font_command: str = TABLE_FONT_REGULAR,
    tabcolsep: float | None = None,
    arraystretch: float | None = None,
    resize_width: str | None = None,
    float_spec: str = "[t]",
) -> str:
    body = []
    body.append(rf"\begin{{table*}}{float_spec}")
    body.append(r"\centering")
    body.append(r"\caption{" + caption + r"}")
    body.append(r"\label{" + label + r"}")
    body.append(font_command)
    if tabcolsep is not None:
        body.append(rf"\setlength{{\tabcolsep}}{{{tabcolsep:.2f}pt}}")
    if arraystretch is not None:
        body.append(rf"\renewcommand{{\arraystretch}}{{{arraystretch:.2f}}}")
    if resize_width is not None:
        body.append(rf"\resizebox{{{resize_width}}}{{!}}{{%")
    body.append(r"\begin{tabular}{" + columns + r"}")
    body.append(r"\toprule")
    body.append(" & ".join(header) + r" \\")
    body.append(r"\midrule")
    if rows:
        for row in rows:
            body.append(" & ".join(tex_escape(item) if isinstance(item, str) else fmt(item) for item in row) + r" \\")
    else:
        body.append(r"\multicolumn{" + str(len(header)) + r"}{c}{No result artifact available.} \\")
    body.append(r"\bottomrule")
    body.append(r"\end{tabular}")
    if resize_width is not None:
        body.append(r"}")
    body.append(r"\end{table*}")
    body.append("")
    return "\n".join(body)


def fixed_caption_table(
    caption: str,
    label: str,
    columns: str,
    header: list[str],
    rows: list[list[Any]],
    *,
    font_command: str = TABLE_FONT_REGULAR,
    tabcolsep: float | None = None,
    arraystretch: float | None = None,
    resize_width: str | None = None,
) -> str:
    body: list[str] = []
    body.append(r"\begingroup")
    body.append(r"\centering")
    body.append(r"\captionof{table}{" + caption + r"}")
    body.append(r"\label{" + label + r"}")
    body.append(font_command)
    if tabcolsep is not None:
        body.append(rf"\setlength{{\tabcolsep}}{{{tabcolsep:.2f}pt}}")
    if arraystretch is not None:
        body.append(rf"\renewcommand{{\arraystretch}}{{{arraystretch:.2f}}}")
    if resize_width is not None:
        body.append(rf"\resizebox{{{resize_width}}}{{!}}{{%")
    body.append(r"\begin{tabular}{" + columns + r"}")
    body.append(r"\toprule")
    body.append(" & ".join(header) + r" \\")
    body.append(r"\midrule")
    if rows:
        for row in rows:
            body.append(" & ".join(tex_escape(item) if isinstance(item, str) else fmt(item) for item in row) + r" \\")
    else:
        body.append(r"\multicolumn{" + str(len(header)) + r"}{c}{No result artifact available.} \\")
    body.append(r"\bottomrule")
    body.append(r"\end{tabular}")
    if resize_width is not None:
        body.append(r"}")
    body.append(r"\par")
    body.append(r"\endgroup")
    body.append("")
    return "\n".join(body)


def appendix_table(
    caption: str,
    label: str,
    columns: str,
    header: list[str],
    rows: list[list[Any]],
    *,
    tabcolsep: float = 2.6,
    font_command: str = TABLE_FONT_DENSE,
    arraystretch: float = 0.95,
    float_spec: str = "[t]",
) -> str:
    return table(
        caption,
        label,
        columns,
        header,
        rows,
        font_command=font_command,
        tabcolsep=tabcolsep,
        arraystretch=arraystretch,
        float_spec=float_spec,
    )


def table_has_missing_artifact(content: str) -> bool:
    return "No result artifact available." in content


def placeholder_table(caption: str, label: str, columns: str, header: list[str]) -> str:
    return table(caption + " Data are reserved for the final experiment run.", label, columns, header, [])


def query_label(name: str) -> str:
    return str(name).replace("->", r"$\rightarrow$")


def fmt_fixed(value: Any, digits: int = 3) -> str:
    if value is None:
        return "--"
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return tex_escape(value)


def fmt_sci(value: Any, digits: int = 2) -> str:
    if value is None:
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return tex_escape(value)
    if number == 0.0:
        return "0"
    return f"{number:.{digits}e}"


def fmt_main_time(value: Any) -> str:
    """Format main-table seconds without scientific notation."""
    if value is None:
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return tex_escape(value)
    if number == 0.0:
        return "0"
    abs_number = abs(number)
    if abs_number >= 100:
        return f"{number:.0f}"
    if abs_number >= 10:
        return f"{number:.1f}"
    if abs_number >= 1:
        return f"{number:.2f}"
    if abs_number >= 1e-2:
        return f"{number:.3f}"
    if abs_number >= 1e-3:
        return f"{number:.4f}"
    return f"{number:.5f}"


def fmt_build(value: Any) -> str:
    if value is None:
        return "--"
    number = float(value)
    if abs(number) < 5e-4:
        return "0"
    return f"{number:.3f}"


def build_header(label: str, build_s: Any, *, extra: str | None = None) -> str:
    build_label = "Build --" if build_s is None else f"Build {fmt_main_time(build_s)}\\,s"
    suffix = build_label if extra is None else f"{build_label}, {extra}"
    return rf"\shortstack{{{label}\\[-0.2ex]{{\normalfont\fontsize{{7.1}}{{7.8}}\selectfont {suffix}}}}}"


def canonical_shelf_queries() -> list[str]:
    return ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]


def sbf_build_s(payload: dict[str, Any] | None) -> float | None:
    if not payload:
        return None
    build = payload.get("build", {})
    for key in ("median_s", "mean_s"):
        value = build.get(key)
        if value is not None:
            return float(value)
    return None


def sbf_query_stats(payload: dict[str, Any] | None) -> dict[str, dict[str, float | None]]:
    if not payload:
        return {}
    stats: dict[str, dict[str, float | None]] = {}
    for query in payload.get("queries", []):
        name = str(query.get("name") or "")
        if not name:
            continue
        sr = query.get("sr")
        stats[name] = {
            "sr": 100.0 * float(sr) if sr is not None else None,
            "query_time_s_median": float(query["t_med_s"]) if query.get("t_med_s") is not None else None,
            "query_path_rad_median": float(query["len_med"]) if query.get("len_med") is not None else None,
        }
    return stats


def baseline_summary(payload: dict[str, Any] | None) -> dict[str, Any]:
    if not payload:
        return {}
    summary = payload.get("summary")
    if isinstance(summary, dict):
        return summary
    build_samples = [float(row["build_s"]) for row in payload.get("seed_trials", []) if row.get("build_s") is not None]
    return {"build_s_median": median(build_samples), "build_s_mean": mean(build_samples)}


def baseline_query_stats(
    payload: dict[str, Any] | None,
    *,
    fixed_budget_s: float | None = None,
) -> dict[str, dict[str, float | None]]:
    if not payload:
        return {}
    accum: dict[str, dict[str, Any]] = {}
    for query in payload.get("queries", []):
        name = str(query.get("name") or query.get("query") or "")
        if name:
            accum.setdefault(name, {"total": 0, "success": 0, "times": [], "paths": []})
    for trial in payload.get("seed_trials", []):
        for query in trial.get("queries", []):
            name = str(query.get("query") or query.get("name") or "")
            if not name:
                continue
            bucket = accum.setdefault(name, {"total": 0, "success": 0, "times": [], "paths": []})
            bucket["total"] += 1
            if not query.get("success"):
                continue
            bucket["success"] += 1
            if fixed_budget_s is not None:
                bucket["times"].append(float(fixed_budget_s))
            elif query.get("time_s") is not None:
                bucket["times"].append(float(query["time_s"]))
            if query.get("path_length") is not None:
                bucket["paths"].append(float(query["path_length"]))
    return {
        name: {
            "sr": (100.0 * bucket["success"] / bucket["total"]) if bucket["total"] else None,
            "query_time_s_median": median(bucket["times"]),
            "query_path_rad_median": median(bucket["paths"]),
        }
        for name, bucket in accum.items()
    }


def query_baseline_table(outputs: Path) -> str:
    _, sbf_payload = first_existing(outputs, [
        "tro2026_exp04_marcucci_full.json",
        "tro2026_exp04_marcucci_support_hull_full.json",
        "marcucci_corridor_refine_selfedge_s10.json",
        "marcucci_combined_standalone.json",
        "marcucci_combined_smoke.json",
    ])
    iris_np = load_json(outputs / "marcucci_iris_np_gcs.json")
    ompl_prm = load_json(outputs / "marcucci_ompl_prm.json")
    ompl_bitstar = load_json(outputs / "marcucci_ompl_bitstar_budget.json")
    rrt_connect = load_json(outputs / "tro2026_exp04_rrt_connect_full.json") or load_json(outputs / "marcucci_rrt_connect_baseline.json")

    query_names = [str(row.get("name")) for row in (sbf_payload or {}).get("queries", []) if row.get("name")]
    if not query_names:
        query_names = canonical_shelf_queries()

    method_specs = [
        {
            "label": build_header(SBF_SH_LABEL, sbf_build_s(sbf_payload), extra="cold"),
            "stats": sbf_query_stats(sbf_payload),
            "columns": [r"SR (\%)", "Query (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"IRIS-NP+GCS", baseline_summary(iris_np).get("build_s_median")),
            "stats": baseline_query_stats(iris_np),
            "columns": [r"SR (\%)", "Query (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"PRM", baseline_summary(ompl_prm).get("build_s_median")),
            "stats": baseline_query_stats(ompl_prm),
            "columns": [r"SR (\%)", "Query (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"RRTConnect", baseline_summary(rrt_connect).get("build_s_median")),
            "stats": baseline_query_stats(rrt_connect),
            "columns": [r"SR (\%)", "Query (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": r"\shortstack{BIT*\\fixed timeout}",
            "stats": baseline_query_stats(ompl_bitstar),
            "columns": [r"SR (\%)", "Query (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
    ]

    colspec = "@{}l" + "|".join("r" * len(spec["columns"]) for spec in method_specs) + "@{}"
    group_header = " & ".join(
        [" "] + [rf"\multicolumn{{{len(spec['columns'])}}}{{c}}{{\textbf{{{TABLE_HEADER_FONT} {spec['label']}}}}}" for spec in method_specs]
    )
    column_header = " & ".join(["Query"] + [" & ".join(spec["columns"]) for spec in method_specs])
    cmidrules = []
    current_col = 2
    for spec in method_specs:
        width = len(spec["columns"])
        cmidrules.append(rf"\cmidrule(lr){{{current_col}-{current_col + width - 1}}}")
        current_col += width

    row_end = " " + chr(92) * 2
    rows: list[str] = []
    for name in query_names:
        values = [query_label(name)]
        for spec in method_specs:
            stats = spec["stats"].get(name, {})
            for key in spec["keys"]:
                values.append(fmt_fixed(stats.get(key), 1 if key == "sr" else 3))
        rows.append(" & ".join(values) + row_end)

    return "\n".join([
        "% Auto-generated from SBF Exp.4 outputs and baseline artifacts.",
        "% SBF reports a cold scene build followed by online queries on the freshly built forest; baseline rows keep their method-specific build/query accounting.",
        r"\begin{table}[!t]",
        r"\centering",
            rf"\caption{{Shelf+IIWA baseline comparison relative to Marcucci~\cite{{marcucci2023motion}}, generated from current-version experiment artifacts. {SBF_SH_LABEL} reports a cold scene build of the {SBF_SH_DEFINITION} followed by collision-checked online queries. Drake IRIS-NP+GCS is rerun by the current SBF baseline script. OMPL PRM uses cumulative shared-roadmap build/query accounting. OMPL RRTConnect uses one max-timeout query attempt per seed/query and returns on connection. BIT* uses one fixed-timeout invocation per seed/query and reports the audited monotone incumbent checkpoint trace. OMPL planning, final simplify, and fixed-resolution final audit all use the same 0.01 joint-space segment step.}}",
        r"\label{tab:query}",
        TABLE_FONT_DENSE,
        r"\setlength{\tabcolsep}{1.5pt}",
        rf"\begin{{tabular}}{{{colspec}}}",
        r"\toprule",
        group_header + row_end,
        "".join(cmidrules),
        column_header + row_end,
        r"\midrule",
        "% --- DATA BEGIN ---",
        *rows,
        "% --- DATA END ---",
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        "",
    ])

def write_marcucci_baseline_plot(outputs: Path, out_dir: Path) -> None:
    try:
        plt = import_pyplot()
    except Exception as exc:
        print(f"[tables] skip Marcucci baseline plot: {exc}")
        return

    _, sbf_payload = first_existing(outputs, [
        "tro2026_exp04_marcucci_full.json",
        "tro2026_exp04_marcucci_support_hull_full.json",
        "marcucci_corridor_refine_selfedge_s10.json",
        "marcucci_combined_standalone.json",
        "marcucci_combined_smoke.json",
    ])
    iris_np = load_json(outputs / "marcucci_iris_np_gcs.json")
    ompl_prm = load_json(outputs / "marcucci_ompl_prm.json")
    ompl_bitstar = load_json(outputs / "marcucci_ompl_bitstar_budget.json")
    rrt_connect = load_json(outputs / "tro2026_exp04_rrt_connect_full.json") or load_json(outputs / "marcucci_rrt_connect_baseline.json")

    specs = [
        ("SBF", sbf_query_stats(sbf_payload), "#0b6e4f"),
        ("IRIS-NP+GCS", baseline_query_stats(iris_np), "#7b2cbf"),
        ("OMPL PRM", baseline_query_stats(ompl_prm), "#1f77b4"),
        ("OMPL RRTConnect", baseline_query_stats(rrt_connect), "#d95f02"),
        ("OMPL BIT*", baseline_query_stats(ompl_bitstar), "#555555"),
    ]
    query_names = [str(row.get("name")) for row in (sbf_payload or {}).get("queries", []) if row.get("name")]
    if not query_names:
        query_names = canonical_shelf_queries()
    markers = ["o", "s", "^", "D", "P"]

    fig, ax = plt.subplots(figsize=(3.4, 2.45))
    for method_label, stats, color in specs:
        xs: list[float] = []
        ys: list[float] = []
        marker_items: list[str] = []
        for index, name in enumerate(query_names):
            item = stats.get(name, {})
            sr = item.get("sr")
            time_s = item.get("query_time_s_median")
            length = item.get("query_path_rad_median")
            if sr is None or float(sr) <= 0.0 or time_s is None or length is None:
                continue
            xs.append(float(time_s))
            ys.append(float(length))
            marker_items.append(markers[index % len(markers)])
        for x, y, marker in zip(xs, ys, marker_items):
            ax.scatter(x, y, marker=marker, s=34, color=color, edgecolors="black", linewidths=0.35, alpha=0.9)
        if xs:
            ax.scatter([], [], marker="o", s=34, color=color, edgecolors="black", linewidths=0.35, label=method_label)

    for index, name in enumerate(query_names):
        ax.scatter([], [], marker=markers[index % len(markers)], s=28, color="#eeeeee", edgecolors="black", linewidths=0.35, label=query_label(name))
    ax.set_xscale("log")
    ax.set_xlabel("successful query time (s)")
    ax.set_ylabel("path length (rad)")
    ax.grid(True, which="both", linewidth=0.35, alpha=0.35)
    ax.legend(fontsize=5.5, ncol=2, frameon=False, loc="best", handletextpad=0.4, columnspacing=0.8)
    fig.tight_layout(pad=0.4)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_exp4_marcucci_baselines.pdf")
    plt.close(fig)


def experiment_matrix_table() -> str:
    rows = [
        ["Exp.1", "Endpoint interval sources", "cost--tightness trade-off", "Main endpoint-evidence comparison"],
        ["Exp.2", "Link-envelope representations", "representation trade-off", "Supplementary Appendix envelope comparison table"],
        ["Exp.3", "LECT reuse and depth sensitivity", "reuse efficiency and refinement depth", "Main LECT reuse table and supplementary depth-sensitivity table"],
        ["Exp.4", "Shelf+IIWA workload", "figure-level efficiency and path quality", "Main Shelf+IIWA figure and tabulated-slice table"],
        ["Exp.5", "Grow-stop ablation", "build/path-quality trade-off", "Supplementary Appendix grow-stop table"],
        ["Exp.6", "GCS composition audit", "fixed-resolution audit pass rate", "Supplementary Appendix GCS audit table"],
        ["Exp.7", "Protected merger study", "direct versus protected safety", "Supplementary Appendix merger-control table"],
        ["Exp.8", "Cross-robot random scenes", "transfer across robots and difficulties", "Supplementary Appendix random-scene table"],
        ["Exp.9", "Dynamic scene update", "localized deletion and rebuild cost", "Supplementary Appendix dynamic-update table"],
        ["Exp.10", "Parallel scaling", "speedup and efficiency boundary", "Supplementary Appendix parallel-scaling table"],
        ["Exp.11", "Soundness audit suite", "paper-wide fixed-resolution audit accounting", "Supplementary Appendix soundness-audit table"],
        ["Exp.12", "Random-scene baselines", "cross-robot figure-level comparison", "Main random-scene figure and tabulated-slice table"],
        ["Exp.13", "Mechanism diagnostics", "claim-boundary diagnostics", "Supplementary Appendix mechanism-diagnostic table"],
    ]
    return table_star(
        "Experiment coverage and evidence map. Each row identifies a study, the phenomenon it tests, and where the corresponding evidence is used in the manuscript. Unless the study itself is the parallel-scaling sweep, all reported algorithms use eight worker threads.",
        "tab:tro_experiment_matrix",
        "@{}p{0.09\\textwidth}p{0.18\\textwidth}p{0.24\\textwidth}p{0.29\\textwidth}@{}",
        ["Study", "Focus", "Measured phenomenon", "Manuscript use"],
        rows,
        font_command=TABLE_FONT_REGULAR,
        tabcolsep=3.2,
        arraystretch=1.02,
        resize_width=None,
    )


def write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def endpoint_table(outputs: Path) -> str:
    _, payload = first_existing(outputs, [
        "tro2026_exp01_endpoint_full.json",
        "epiaabb_pipeline_standalone_n400_fixed_widths_endpoint_only.json",
        "epiaabb_pipeline_standalone.json",
        "smoke_exp1.json",
    ])
    rows: list[list[Any]] = []
    if payload:
        grouped: dict[str, list[dict[str, Any]]] = {}
        for row in payload.get("rows", []):
            grouped.setdefault(str(row.get("source")), []).append(row)
        order = ["IFK", "CritSample", "Analytical", "MC"]
        for source in order:
            items = grouped.get(source, [])
            if not items:
                continue
            rows.append([
                source,
                mean(item.get("time_us_mean") for item in items),
                mean(item.get("volume_mean") for item in items),
                mean(item.get("mean_gap_to_sampling_union") for item in items),
                "yes" if any(bool(item.get("certified")) for item in items) else "no",
            ])
    return table(
        "Endpoint-interval source microbenchmark averaged over the sampled width bins.",
        "tab:tro_endpoint_sources",
        "lrrrr",
        ["Source", "Time (us)", "Volume", "Mean gap", "Certified"],
        rows,
    )


LINK_REPRESENTATION_VARIANT_ORDER = [
    "link_s4",
    "kdop26_s4",
    "support_hull_nokdop_s4",
]

LINK_REPRESENTATION_STYLES = {
    "link_s4": ("LinkIAABB", "#1f77b4", "s", "-", True),
    "kdop26_s4": ("KDOP26", "#d95f02", "^", "--", True),
    "support_hull_nokdop_s4": ("SupportHull", "#7b2cbf", "D", "-.", True),
}


def link_representation_display_label(variant: Any) -> str:
    style = LINK_REPRESENTATION_STYLES.get(str(variant))
    return style[0] if style is not None else str(variant)


def link_representation_summary_rows(outputs: Path) -> list[dict[str, Any]]:
    _, payload = first_existing(outputs, [
        "tro2026_exp01_link_repr_widthwise.json",
        "tro2026_exp01_link_repr_s4_collision.json",
        "_tmp_link_repr_smoke.json",
    ])
    if not payload:
        return []
    grouped: dict[str, dict[str, Any]] = {}
    for row in payload.get("rows", []):
        variant = str(row.get("variant"))
        if variant not in LINK_REPRESENTATION_VARIANT_ORDER:
            continue
        if bool(row.get("diagnostic")) or is_grid_row(row):
            continue
        endpoint_source = str(row.get("endpoint_source", "critsample")).lower()
        if endpoint_source not in {"", "critsample"}:
            continue
        grouped[variant] = row
    return [grouped[variant] for variant in LINK_REPRESENTATION_VARIANT_ORDER if variant in grouped]


def link_representation_rows(outputs: Path) -> tuple[list[float], dict[tuple[float, str], dict[str, Any]]]:
    _, payload = first_existing(outputs, [
        "tro2026_exp01_link_repr_widthwise.json",
        "tro2026_exp01_link_repr_s4_collision.json",
        "_tmp_link_repr_smoke.json",
    ])
    grouped: dict[tuple[float, str], dict[str, Any]] = {}
    if not payload:
        return [], grouped
    for row in payload.get("rows_by_width", []):
        variant = str(row.get("variant"))
        if variant not in LINK_REPRESENTATION_VARIANT_ORDER:
            continue
        if bool(row.get("diagnostic")) or is_grid_row(row):
            continue
        endpoint_source = str(row.get("endpoint_source", "critsample")).lower()
        if endpoint_source not in {"", "critsample"}:
            continue
        width = safe_float(row.get("fixed_width"), float("nan"))
        if not math.isfinite(width):
            continue
        grouped[(width, variant)] = row
    width_order = sorted({width for width, _ in grouped})
    return width_order, grouped


def link_representation_widthwise_table(outputs: Path) -> str:
    width_order, grouped = link_representation_rows(outputs)
    body: list[str] = []
    for width in width_order:
        for variant in LINK_REPRESENTATION_VARIANT_ORDER:
            row = grouped.get((width, variant))
            if row is None:
                continue
            label = link_representation_display_label(variant)
            body.append(" & ".join([
                tex_escape(fmt_fixed(width, 2)),
                tex_escape(label),
                tex_escape(fmt(row.get("volume_mean"), 3)),
                tex_escape(fmt_fixed(row.get("t_eval_us_mean"), 1)),
                tex_escape(fmt_fixed(row.get("collision_us_mean"), 1)),
            ]) + r" \\")
        if width != width_order[-1]:
            body.append(r"\addlinespace")
    if not body:
        body.append(r"\multicolumn{5}{c}{No result artifact available.} \\")
    return "\n".join([
        "% Auto-generated from the width-wise CritSample link-envelope representation artifact.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Width-wise CritSample link-envelope comparison under the same $S=4$ short-link split.}",
        r"\label{tab:tro_link_envelopes}",
        r"\label{tab:tro_link_envelope_widthwise}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{2.2pt}",
        r"\renewcommand{\arraystretch}{0.82}",
        r"\begin{tabular}{@{}llrrr@{}}",
        r"\toprule",
        r"Width & Envelope & $V$ (m$^3$) & Eval ($\mu$s) & Collision ($\mu$s) \\",
        r"\midrule",
        *body,
        r"\bottomrule",
        r"\end{tabular}",
        r"\par",
        r"\endgroup",
        "",
    ])


def lect_table(outputs: Path) -> str:
    _, payload = first_existing(outputs, [
        "tro2026_exp02_lect_probe_protocolmix_full_s10.json",
        "tro2026_exp02_lect_full.json",
        "marcucci_envelope_build_standalone.json",
        "marcucci_envelope_build_protocol_optimized.json",
        "smoke_exp3_standalone.json",
    ])
    rows: list[list[Any]] = []
    if payload:
        summaries = payload.get("summaries", {})
        preferred = ["crit_link_coverage", "kdop26_coverage", "support_hull_coverage"]
        variants = [variant for variant in preferred if variant in summaries]
        variants.extend(variant for variant in sorted(summaries) if variant not in variants and not is_grid_row({"method": variant}))

        def item(variant: str, protocol: str) -> dict[str, Any]:
            value = summaries.get(variant, {}).get(protocol)
            return value if isinstance(value, dict) else {}

        def speedup(variant: str) -> str:
            cold = item(variant, "cold").get("build_mean_s")
            warm = item(variant, "warm").get("build_mean_s")
            try:
                if float(warm) > 0.0:
                    return f"{float(cold) / float(warm):.2f}x"
            except (TypeError, ValueError):
                pass
            return "--"

        metric_rows: list[tuple[str, callable[[str], Any]]] = [
            ("Cold build (s)", lambda variant: item(variant, "cold").get("build_mean_s")),
            ("Warm build (s)", lambda variant: item(variant, "warm").get("build_mean_s")),
            ("Cross-scene build (s)", lambda variant: item(variant, "cross_scene").get("build_mean_s")),
            ("Warm/cold speedup", speedup),
            ("Warm cache (MB)", lambda variant: (item(variant, "warm").get("cache_file_bytes_mean") or 0.0) / 1e6),
            ("Boxes", lambda variant: item(variant, "cold").get("box_count_mean")),
        ]
        for label, getter in metric_rows:
            rows.append([label, *[getter(variant) for variant in variants]])
        headers = ["Protocol / metric", *[variant_label(variant) for variant in variants]]
    else:
        headers = ["Protocol / metric", "Crit+LinkIAABB", "Crit+KDOP26", SBF_SH_LABEL]
    return appendix_table(
        "LECT build/cache reuse under cold, warm, and cross-scene protocols. Columns correspond to envelope methods, and rows report protocol-level settings and outcomes. Warm and cross-scene runs reuse only kinematics-keyed envelope evidence, followed by scene revalidation.",
        "tab:tro_lect_reuse",
        "l" + "r" * (len(headers) - 1),
        headers,
        rows,
    )


def main_lect_reuse_table(outputs: Path) -> str:
    payload = load_json(outputs / MAIN_LECT_REUSE_ARTIFACT)
    summaries = (payload or {}).get("summaries", {})
    params = (payload or {}).get("params", {})
    variant = "support_hull_coverage" if "support_hull_coverage" in summaries else next(iter(summaries), None)

    def scenario_label(key: str) -> str:
        labels = {
            "shelf_iiwa": "Shelf+IIWA",
            "shelf_iiwa_marcucci_combined": "Shelf+IIWA",
            "random_obstacle_iiwa": "Random IIWA",
            "easy": "Easy",
            "medium": "Medium",
            "hard": "Hard",
        }
        return labels.get(key, key.replace("_", " ").title())

    available_scenarios = list((summaries.get(variant, {}) or {}).keys()) if variant is not None else []
    configured_scenarios = params.get("scenarios") or params.get("difficulties") or []
    if isinstance(configured_scenarios, str):
        configured_scenarios = [configured_scenarios]
    scenario_order = [str(item) for item in configured_scenarios if str(item) in available_scenarios]
    scenario_order.extend(item for item in available_scenarios if item not in scenario_order)

    def phase(variant: str, difficulty: str, name: str) -> dict[str, Any]:
        value = (((summaries.get(variant, {}) or {}).get(difficulty, {}) or {}).get(name, {}) or {})
        return value if isinstance(value, dict) else {}

    def build_stat(summary: dict[str, Any]) -> Any:
        return summary.get("build_median_s", summary.get("build_mean_s"))

    def hit_percent(summary: dict[str, Any], regime: str) -> float | None:
        if regime == "replay":
            materializations = safe_float(summary.get("materializations_mean"))
            cached_envelopes = safe_float(summary.get("materialization_reused_cached_envelope_mean"))
            if materializations > 0.0 and cached_envelopes > 0.0:
                return 100.0 * cached_envelopes / materializations
            value = summary.get("lect_endpoint_hit_rate_mean")
        else:
            value = summary.get("lect_prewarm_hit_rate_mean", summary.get("lect_materialization_hit_rate_mean"))
        return None if value is None else 100.0 * safe_float(value)

    def speedup_cell(value: Any) -> str:
        if value is None:
            return "--"
        return rf"{safe_float(value):.2f}$\times$"

    def budget_sort_key(label: str, row: dict[str, Any]) -> float:
        try:
            return float(row.get("budget_ms"))
        except (TypeError, ValueError):
            pass
        return float("inf")

    rows: list[str] = []
    if variant is not None:
        for difficulty in scenario_order:
            block = (summaries.get(variant, {}) or {}).get(difficulty, {}) or {}
            cold = phase(variant, difficulty, "cold")
            warm_populate = phase(variant, difficulty, "warm_populate")
            warm_budget = phase(variant, difficulty, "warm_budget")
            warm_budgets = block.get("warm_budgets", block.get("cross_budgets", {}))
            replay_populate = phase(variant, difficulty, "replay_populate")
            replay = phase(variant, difficulty, "replay") or phase(variant, difficulty, "warm")
            if not cold and not warm_budget and not replay:
                continue
            if rows:
                rows.append(r"\midrule")
            rows.append(" & ".join([
                "Cold",
                "--",
                fmt_fixed(hit_percent(cold, "cold"), 1),
                fmt_fixed(build_stat(cold), 3),
                speedup_cell(1.0),
            ]) + r" \\")
            if isinstance(warm_budgets, dict) and warm_budgets:
                for _, budget_row in sorted(warm_budgets.items(), key=lambda item: budget_sort_key(item[0], item[1])):
                    prewarm = budget_row.get("prewarm", {}) if isinstance(budget_row, dict) else {}
                    reuse = budget_row.get("reuse", {}) if isinstance(budget_row, dict) else {}
                    if not reuse:
                        continue
                    rows.append(" & ".join([
                        "Warm",
                        fmt_fixed(build_stat(prewarm), 3),
                        fmt_fixed(hit_percent(reuse, "warm"), 1),
                        fmt_fixed(build_stat(reuse), 3),
                        speedup_cell(budget_row.get("warm_speedup_vs_cold", budget_row.get("cross_speedup_vs_cold"))),
                    ]) + r" \\")
            elif warm_budget:
                rows.append(" & ".join([
                    "Warm",
                    fmt_fixed(build_stat(warm_populate), 3),
                    fmt_fixed(hit_percent(warm_budget, "warm"), 1),
                    fmt_fixed(build_stat(warm_budget), 3),
                    speedup_cell(block.get("warm_speedup_vs_cold", block.get("cross_speedup_vs_cold"))),
                ]) + r" \\")
            if replay:
                rows.append(" & ".join([
                    "Replay",
                    fmt_fixed(build_stat(replay_populate), 3),
                    fmt_fixed(hit_percent(replay, "replay"), 1),
                    fmt_fixed(build_stat(replay), 3),
                    speedup_cell(block.get("replay_speedup_vs_cold")),
                ]) + r" \\")
    if not rows:
        rows.append(r"\multicolumn{5}{c}{No result artifact available.} \\")
    scenario_caption = scenario_label(scenario_order[0]) if scenario_order else "IIWA"
    return "\n".join([
        "% Auto-generated from the IIWA incremental LECT reuse artifact.",
        r"\begingroup",
        r"\centering",
        rf"\captionof{{table}}{{{scenario_caption} LECT reuse under the paper-facing CritSample endpoint source and an approximately 250-box target regime. Warm rows use blind random-obstacle prewarm scenes; Cold and Warm replay the same validation/split route. LECT hit reports external prewarm materialization hits for Warm and cached-envelope hits for Replay. Speedup charges only the target build.}}",
        r"\label{tab:tro_main_lect_reuse}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{1.4pt}",
        r"\renewcommand{\arraystretch}{0.82}",
        r"\begin{tabular}{@{}lrrrr@{}}",
        r"\toprule",
        r"Regime & Off./pop. (s) & LECT hit (\%) & Build (s) & Speedup \\",
        r"\midrule",
        *rows,
        r"\bottomrule",
        r"\end{tabular}",
        r"\par",
        r"\endgroup",
        "",
    ])


def main_lect_cache_footprint_table(outputs: Path) -> str:
    del outputs
    row_lines = [
        r"Crit+AABB$_{4}$ & 1.31 & 0.63 & 114.0 & 0.72 & 0.62 & 1.17 & 646.0 \\",
        r"IFK+AABB$_{4}$ & 0.67 & 0.67 & 114.0 & 0.39 & 0.39 & 1.00 & 646.0 \\",
    ]
    return "\n".join([
        "% Static v6 reference panel requested for the experiment opening.",
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{LECT cache reuse and storage footprint for retained envelope records. Matched-route replay reruns the same route. Cross-scene reuse evaluates retained evidence in a different scene.}",
        r"\label{tab:tro_main_lect_cache_footprint}",
        TABLE_FONT_DENSE,
        r"\setlength{\tabcolsep}{2.0pt}",
        r"\begin{tabular}{@{}lrrrrrrr@{}}",
        r"\toprule",
        r"Group & \multicolumn{3}{c}{Matched route} & \multicolumn{4}{c}{Cross scene} \\",
        r"\cmidrule(lr){2-4} \cmidrule(lr){5-8}",
        r" & Cold (s) & Warm (s) & MB & Cold (s) & Warm (s) & Speedup & MB \\",
        r"\midrule",
        *row_lines,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        "",
    ])


def ffb_depth_table(outputs: Path) -> str:
    rows: list[list[Any]] = []
    depth_files = [
        (40, "tro2026_exp03_ffb_depth_40_default_after_fk_s3.json"),
        (60, "tro2026_exp03_ffb_depth_60_s3.json"),
        (90, "tro2026_exp03_ffb_depth_90_s3.json"),
        (120, "tro2026_exp03_ffb_depth_120_s3.json"),
        (160, "tro2026_exp03_ffb_depth_160_s3.json"),
    ]
    for depth, name in depth_files:
        payload = load_json(outputs / name)
        if not payload:
            continue
        summaries = payload.get("summaries", {})

        def avg(protocol: str, key: str) -> float | None:
            return mean(
                item.get(protocol, {}).get(key)
                for item in summaries.values()
                if item.get(protocol)
            )

        cold_score = avg("cold", "scoring_candidate_dirty_count_mean")
        same_audit = avg("cold", "audited_query_sr")
        cross_audit = avg("cross_scene", "audited_query_sr")
        rows.append([
            depth,
            avg("cold", "build_mean_s"),
            avg("warm", "build_mean_s"),
            avg("cross_scene", "build_mean_s"),
            f"{cold_score / 1000.0:.1f}k" if cold_score is not None else "--",
            f"{same_audit:.2f} / {cross_audit:.2f}" if same_audit is not None and cross_audit is not None else "--",
        ])
    return appendix_table(
        "FFB depth-cap sensitivity in the Exp.3 Marcucci cache protocol. Rows average LinkIAABB, KDOP26, and SupportHull over three seeds; warm and cross-scene rows use LECT evidence replay with domain-local worker snapshots. The D=40 row is a lower-depth follow-up with the FK-aware split filter enabled.",
        "tab:tro_ffb_depth_sweep",
        "rrrrrr",
        [r"$D_{\max}$", "Cold (s)", "Warm (s)", "Cross (s)", "Cold score cand.", "Audit SR same/cross"],
        rows,
    )


def marcucci_table(outputs: Path) -> str:
    _, payload = first_existing(outputs, [
        "tro2026_exp04_marcucci_full.json",
        "marcucci_corridor_refine_selfedge_s10.json",
        "marcucci_combined_standalone.json",
        "marcucci_combined_smoke.json",
    ])
    rows: list[list[Any]] = []
    if payload:
        for row in payload.get("queries", []):
            rows.append([
                row.get("name"),
                row.get("sr"),
                row.get("audit_sr"),
                row.get("t_med_s"),
                row.get("len_med"),
                row.get("repair_count_med"),
            ])
        build = payload.get("build", {})
        rows.append([
            "Build summary",
            "--",
            "--",
            build.get("mean_s"),
            build.get("mean_unique_box_count"),
            build.get("mean_segment_edge_count"),
        ])
    return table(
        "Shelf+IIWA end-to-end SBF result. The build row reports build time, mean boxes, and segment edges.",
        "tab:tro_marcucci_e2e",
        "lrrrrr",
        ["Item", "SR", "Audit SR", "Time (s)", "Length/Boxes", "Repairs/Edges"],
        rows,
    )


def _load_marcucci_variant(outputs: Path, names: list[str]) -> dict[str, Any] | None:
    _, payload = first_existing(outputs, names)
    return payload


def _query_values(payload: dict[str, Any], key: str) -> list[float]:
    values: list[float] = []
    for row in payload.get("queries", []):
        value = row.get(key)
        if value is not None:
            values.append(float(value))
    return values


def marcucci_envelope_variant_table(outputs: Path) -> str:
    variants = [
        (
            "LinkIAABB",
            ["tro2026_exp04_marcucci_full.json", "marcucci_corridor_refine_selfedge_s10.json", "marcucci_combined_standalone.json"],
        ),
        (
            "KDOP26",
            ["tro2026_exp04_marcucci_kdop26_full.json", "marcucci_full_kdop26_s5.json"],
        ),
        (
            "SupportHull",
            ["tro2026_exp04_marcucci_support_hull_full.json", "marcucci_full_support_hull_s5.json"],
        ),
    ]
    rows: list[list[Any]] = []
    for label, names in variants:
        payload = _load_marcucci_variant(outputs, names)
        if not payload:
            continue
        build = payload.get("build", {})
        query_count = len(payload.get("queries", []))
        audit_values = _query_values(payload, "audit_sr")
        sr_values = _query_values(payload, "sr")
        query_times = _query_values(payload, "t_med_s")
        lengths = _query_values(payload, "len_med")
        rows.append([
            label,
            build.get("mean_s"),
            build.get("mean_unique_box_count"),
            statistics.fmean(sr_values) if sr_values else None,
            statistics.fmean(audit_values) if audit_values else None,
            statistics.median(query_times) if query_times else None,
            statistics.median(lengths) if lengths else None,
            query_count,
        ])
    return appendix_table(
        f"End-to-end Marcucci envelope variants under the same SBF build/query protocol. {SBF_SH_LABEL} is the SupportHull row with retained KDOP26 slabs and collision-checked path post-processing.",
        "tab:tro_marcucci_envelope_variants",
        "lrrrrrrr",
        ["Envelope", "Build (s)", "Boxes", "SR", "Audit SR", "Med. query (s)", "Med. path", "Queries"],
        rows,
    )


def grower_table(outputs: Path) -> str:
    payload = load_json(outputs / "tro2026_exp03_grower_full.json") or load_json(outputs / "marcucci_grower_tradeoff.json")
    selected_payload = load_json(outputs / "tro2026_exp04_marcucci_full.json") or load_json(outputs / "tro2026_exp04_marcucci_support_hull_full.json")
    rows: list[list[Any]] = []
    if payload:
        wanted = {0, 64, 128, 320, 512}
        selected_floor = int(safe_float((selected_payload or {}).get("params", {}).get("quality_min_connected_boxes"), 64.0))
        wanted.add(selected_floor)
        for item in payload.get("settings", []):
            floor = int(item.get("quality_min_connected_boxes", -1))
            if floor not in wanted:
                continue
            rows.append([
                f"{floor} (selected)" if floor == selected_floor else floor,
                item.get("build_mean_s"),
                item.get("box_mean"),
                item.get("total_length_median"),
                item.get("repair_total_median"),
                item.get("audit_sr"),
            ])
    return appendix_table(
        "Grow-stop trade-off on the Marcucci scene without an imposed 0.5 s build cap. The selected floor is used by subsequent paper-facing SBF runs.",
        "tab:tro_grower_tradeoff",
        "rrrrrr",
        ["Quality floor", "Build (s)", "Boxes", "Path sum", "Repairs", "Audit SR"],
        rows,
    )


def write_sbf_time_quality_tradeoff_plot(outputs: Path, out_dir: Path) -> None:
    payload = load_json(outputs / "tro2026_exp03_grower_full.json") or load_json(outputs / "marcucci_grower_tradeoff.json")
    if not payload:
        return
    try:
        plt = import_pyplot()
    except Exception as exc:
        print(f"[tables] skip SBF time-quality plot: {exc}")
        return
    selected_payload = load_json(outputs / "tro2026_exp04_marcucci_full.json") or load_json(outputs / "tro2026_exp04_marcucci_support_hull_full.json")
    selected_floor = int(safe_float((selected_payload or {}).get("params", {}).get("quality_min_connected_boxes"), -1.0))
    settings = [
        row for row in payload.get("settings", [])
        if row.get("build_mean_s") is not None and row.get("total_length_median") is not None
    ]
    if not settings:
        return
    settings = sorted(settings, key=lambda row: float(row.get("build_mean_s") or 0.0))
    xs = [float(row["build_mean_s"]) for row in settings]
    ys = [float(row["total_length_median"]) for row in settings]
    repairs = [float(row.get("repair_total_median") or 0.0) for row in settings]
    floors = [int(row.get("quality_min_connected_boxes", -1)) for row in settings]
    sizes = [36.0 + 8.0 * min(6.0, max(0.0, repair)) for repair in repairs]

    fig, ax = plt.subplots(figsize=(3.35, 2.35))
    scatter = ax.scatter(xs, ys, c=repairs, s=sizes, cmap="viridis_r", edgecolors="black", linewidths=0.35, zorder=3)
    ax.plot(xs, ys, color="#59636e", linewidth=0.75, alpha=0.65, zorder=2)
    for x_value, y_value, floor in zip(xs, ys, floors):
        ax.annotate(str(floor), (x_value, y_value), textcoords="offset points", xytext=(3, 3), fontsize=6.0)
    if selected_floor in floors:
        index = floors.index(selected_floor)
        ax.scatter([xs[index]], [ys[index]], marker="*", s=120, color="#c62828", edgecolors="black", linewidths=0.45, zorder=4)
    ax.set_xlabel("cold build time (s)")
    ax.set_ylabel("total path length (rad)")
    ax.grid(True, linewidth=0.35, alpha=0.35)
    cbar = fig.colorbar(scatter, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("repair count", fontsize=6.0)
    cbar.ax.tick_params(labelsize=5.5)
    ax.tick_params(labelsize=6.5)
    ax.xaxis.label.set_size(7.0)
    ax.yaxis.label.set_size(7.0)
    fig.tight_layout(pad=0.35)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_tro_sbf_time_quality_tradeoff.pdf")
    plt.close(fig)


ANYTIME_METHOD_STYLES = {
    "sbf_cold": (SBF_SH_LABEL, "#0b6e4f", "o"),
    "sbf_scene_stage_support_hull_coverage": (SBF_SH_LABEL, "#0b6e4f", "o"),
    "sbf_warm_support_hull_coverage": (r"SBF-SH warm", "#0b6e4f", "o"),
    "ompl_prm": (r"PRM", "#1f77b4", "s"),
    "ompl_bitstar": (r"BIT*", "#555555", "^"),
    "ompl_rrtconnect": (r"RRTConnect", "#d95f02", "D"),
    "drake_iris_np_gcs": (r"IRIS-NP+GCS", "#7b2cbf", "P"),
}

ANYTIME_MARKER_OFFSETS = {
    "sbf": (0.0, 0.0),
    "sbf_cold": (0.0, 0.0),
    "sbf_scene_stage_support_hull_coverage": (0.0, 0.0),
    "drake_iris_np_gcs": (-2.8, 1.9),
    "ompl_prm": (2.8, -1.8),
    "ompl_bitstar": (1.8, 2.0),
    "ompl_rrtconnect": (-1.8, -1.7),
}

BITSTAR_DISPLAY_MARKER_BUDGET = 5


def anytime_style(method: Any) -> tuple[str, str, str]:
    text = str(method)
    if text in ANYTIME_METHOD_STYLES:
        return ANYTIME_METHOD_STYLES[text]
    if text.startswith("sbf_warm_"):
        return (r"SBF warm", "#0b6e4f", "o")
    return (tex_escape(text), "#666666", "o")


def anytime_marker_size(audit_sr: Any) -> float:
    sr = max(0.0, min(1.0, safe_float(audit_sr, 0.0)))
    return 8.0 + 28.0 * sr


def anytime_plot_x(value: Any, floor: float = 1e-4) -> float:
    return max(floor, safe_float(value, floor))


def anytime_checkpoint_seconds(row: dict[str, Any]) -> float | None:
    params = row.get("params") if isinstance(row.get("params"), dict) else {}
    for key in ("checkpoint_s", "timeout_s"):
        value = row.get(key, params.get(key))
        if value is not None:
            seconds = safe_float(value, float("nan"))
            return seconds if math.isfinite(seconds) else None
    stage_id = str(row.get("stage_id", ""))
    if stage_id.startswith("t") and stage_id.endswith("s"):
        seconds = safe_float(stage_id[1:-1], float("nan"))
        return seconds if math.isfinite(seconds) else None
    return None


def anytime_marker_key(point: dict[str, Any]) -> tuple[str, int, str]:
    return (str(point.get("method")), int(point.get("stage_index", -1)), str(point.get("stage_id", "")))


def selected_marker_keys(outputs: Path, scenario_key: str) -> set[tuple[str, int, str]]:
    return {
        anytime_marker_key(row)
        for row in best_tradeoff_points(outputs)
        if str(row.get("scenario_key")) == str(scenario_key)
    }


def anytime_marker_rows(method: str, rows: list[dict[str, Any]], forced_keys: set[tuple[str, int, str]] | None = None) -> list[dict[str, Any]]:
    if method != "ompl_bitstar" or len(rows) <= BITSTAR_DISPLAY_MARKER_BUDGET:
        return rows

    forced = forced_keys or set()
    selected_indices = {0, len(rows) - 1}
    for index, row in enumerate(rows):
        if anytime_marker_key(row) in forced:
            selected_indices.add(index)

    time_positions: list[float] = []
    for row in rows:
        seconds = anytime_checkpoint_seconds(row)
        if seconds is None or not math.isfinite(seconds) or seconds <= 0.0:
            seconds = anytime_plot_x(row.get("total_s"))
        time_positions.append(math.log(max(1e-6, float(seconds))))

    preferred_indices = [
        index for index, row in enumerate(rows)
        if index not in selected_indices and bool(row.get("display_promoted", row.get("promoted", False)))
    ]
    if len(preferred_indices) < BITSTAR_DISPLAY_MARKER_BUDGET - len(selected_indices):
        preferred_indices = [index for index in range(len(rows)) if index not in selected_indices]

    while preferred_indices and len(selected_indices) < BITSTAR_DISPLAY_MARKER_BUDGET:
        best_index = max(
            preferred_indices,
            key=lambda index: (
                min(abs(time_positions[index] - time_positions[chosen]) for chosen in selected_indices),
                1 if bool(rows[index].get("display_promoted", rows[index].get("promoted", False))) else 0,
            ),
        )
        selected_indices.add(best_index)
        preferred_indices.remove(best_index)

    return [row for index, row in enumerate(rows) if index in selected_indices]


def positive_range(values: list[float], *, pad: float = 1.35) -> tuple[float, float] | None:
    positives = [float(value) for value in values if float(value) > 0.0]
    if not positives:
        return None
    lo = min(positives)
    hi = max(positives)
    if abs(hi - lo) <= 1e-12:
        return lo / pad, hi * pad
    return lo / pad, hi * pad


def linear_range(values: list[float], *, pad_fraction: float = 0.16) -> tuple[float, float] | None:
    rows = [float(value) for value in values if value is not None]
    if not rows:
        return None
    lo = min(rows)
    hi = max(rows)
    span = hi - lo
    if span <= 1e-12:
        pad = max(0.08, 0.04 * max(1.0, abs(lo)))
    else:
        pad = max(0.04, span * pad_fraction)
    return lo - pad, hi + pad


def robust_linear_range(values: list[float], *, pad_fraction: float = 0.18, whisker: float = 3.0) -> tuple[float, float] | None:
    rows = sorted(float(value) for value in values if value is not None)
    if len(rows) < 8:
        return linear_range(rows, pad_fraction=pad_fraction)

    def percentile(frac: float) -> float:
        pos = (len(rows) - 1) * frac
        lo_index = int(pos)
        hi_index = min(len(rows) - 1, lo_index + 1)
        alpha = pos - lo_index
        return rows[lo_index] * (1.0 - alpha) + rows[hi_index] * alpha

    q1 = percentile(0.25)
    q3 = percentile(0.75)
    iqr = max(1e-9, q3 - q1)
    lo = max(rows[0], q1 - whisker * iqr)
    hi = min(rows[-1], q3 + whisker * iqr)
    if hi <= lo + 1e-9:
        return linear_range(rows, pad_fraction=pad_fraction)
    span = hi - lo
    pad = max(0.04, span * pad_fraction)
    return lo - pad, hi + pad


def centered_linear_range(values: list[float], *, pad_fraction: float = 0.08) -> tuple[float, float] | None:
    rows = sorted(float(value) for value in values if value is not None)
    if not rows:
        return None
    center = rows[len(rows) // 2] if len(rows) % 2 == 1 else 0.5 * (rows[len(rows) // 2 - 1] + rows[len(rows) // 2])
    half_span = max(center - rows[0], rows[-1] - center)
    if half_span <= 1e-12:
        half_span = max(0.08, 0.04 * max(1.0, abs(center)))
    half_span *= 1.0 + pad_fraction
    lo = center - half_span
    hi = center + half_span
    if lo < 0.0 and rows[0] >= 0.0:
        lo = 0.0
        hi = max(hi, rows[-1] + max(0.04, (rows[-1] - rows[0]) * pad_fraction))
    return lo, hi


def point_has_full_success(point: dict[str, Any]) -> bool:
    success_count = point.get("success_count")
    task_count = point.get("task_count")
    if success_count is not None and task_count is not None and safe_float(task_count) > 0:
        return safe_float(success_count) >= safe_float(task_count) - 1e-9
    return safe_float(point.get("audit_sr"), 0.0) >= FULL_SUCCESS_THRESHOLD


def prepare_display_points(
    points: list[dict[str, Any]],
    *,
    require_full_success: bool = False,
    compress_full_success_iris: bool = False,
    allow_partial_success_methods: set[str] | None = None,
    epsilon_path: float = 1e-6,
) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for point in points:
        if point.get("path_length") is None or point.get("total_s") is None:
            continue
        method = str(point.get("method"))
        normalized_method = normalize_anytime_method(method)
        allow_partial_success = (
            allow_partial_success_methods is not None
            and normalized_method in allow_partial_success_methods
        )
        if require_full_success and not allow_partial_success and not point_has_full_success(point):
            continue
        candidates.append(dict(point))
    by_method: dict[str, list[dict[str, Any]]] = {}
    for point in candidates:
        by_method.setdefault(str(point.get("method")), []).append(point)
    out: list[dict[str, Any]] = []
    for method, rows in sorted(by_method.items()):
        rows = sorted(rows, key=lambda row: int(row.get("stage_index", 0)))
        if compress_full_success_iris and method == "drake_iris_np_gcs":
            kept: list[dict[str, Any]] = []
            best_path = float("inf")
            best_sr = -1.0
            has_full_success = any(point_has_full_success(row) for row in rows)
            for row in rows:
                sr = safe_float(row.get("audit_sr"), 0.0)
                path = safe_float(row.get("path_length"), float("inf"))
                if has_full_success:
                    if point_has_full_success(row) and (not kept or path < best_path - float(epsilon_path)):
                        kept.append(row)
                        best_path = path
                    continue
                improved_sr = sr > best_sr + 1e-9
                improved_path = path < best_path - float(epsilon_path)
                if not kept or improved_sr or (abs(sr - best_sr) <= 1e-9 and improved_path):
                    kept.append(row)
                    best_sr = max(best_sr, sr)
                    best_path = min(best_path, path)
            rows = kept
        best_path = float("inf")
        for row in rows:
            path = safe_float(row.get("path_length"), float("inf"))
            promoted = path < best_path - float(epsilon_path)
            row["display_promoted"] = bool(promoted)
            if promoted:
                row["display_no_improvement_reason"] = None
                best_path = path
            else:
                row["display_no_improvement_reason"] = "display_incumbent_not_improved"
            out.append(row)
    return out


def grouped_anytime_points(
    points: list[dict[str, Any]],
    *,
    require_full_success: bool = False,
    compress_full_success_iris: bool = False,
    allow_partial_success_methods: set[str] | None = None,
) -> dict[str, list[dict[str, Any]]]:
    by_method: dict[str, list[dict[str, Any]]] = {}
    for point in prepare_display_points(
        points,
        require_full_success=require_full_success,
        compress_full_success_iris=compress_full_success_iris,
        allow_partial_success_methods=allow_partial_success_methods,
    ):
        by_method.setdefault(str(point.get("method")), []).append(point)
    return by_method


def displayed_anytime_y_range(
    points: list[dict[str, Any]],
    *,
    require_full_success: bool = False,
    compress_full_success_iris: bool = False,
    allow_partial_success_methods: set[str] | None = None,
    pad_fraction: float = 0.035,
) -> tuple[float, float] | None:
    displayed = prepare_display_points(
        points,
        require_full_success=require_full_success,
        compress_full_success_iris=compress_full_success_iris,
        allow_partial_success_methods=allow_partial_success_methods,
    )
    y_values = [safe_float(row.get("path_length")) for row in displayed if row.get("path_length") is not None]
    return linear_range(y_values, pad_fraction=pad_fraction)


def draw_anytime_marker(
    ax: Any,
    row: dict[str, Any],
    x_value: float,
    y_value: float,
    *,
    label: str,
    color: str,
    marker: str,
    zorder: int,
    marker_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    sr = safe_float(row.get("audit_sr"), 0.0)
    promoted = bool(row.get("display_promoted", row.get("promoted", True)))
    transform = ax.transData
    if marker_offset != (0.0, 0.0):
        from matplotlib.transforms import ScaledTranslation

        transform = transform + ScaledTranslation(marker_offset[0] / 72.0, marker_offset[1] / 72.0, ax.figure.dpi_scale_trans)
    ax.scatter(
        [x_value],
        [y_value],
        marker=marker,
        s=anytime_marker_size(sr),
        facecolors=color if promoted else "white",
        edgecolors=color,
        linewidths=0.48 if promoted else 0.62,
        alpha=0.90 if sr >= 0.999 else 0.58,
        zorder=zorder,
        label=label,
        transform=transform,
    )
    if sr < 0.999:
        ax.annotate(f"{100.0 * sr:.0f}%", (x_value, y_value), textcoords="offset points", xytext=(3, 3), fontsize=5.4)


def plot_anytime_points(
    ax: Any,
    points: list[dict[str, Any]],
    *,
    title: str | None = None,
    show_legend: bool = True,
    show_xlabel: bool = True,
    show_ylabel: bool = True,
    require_full_success: bool = False,
    compress_full_success_iris: bool = False,
    allow_partial_success_methods: set[str] | None = None,
    separate_markers: bool = False,
    method_order: list[str] | None = None,
    forced_marker_keys: set[tuple[str, int, str]] | None = None,
) -> None:
    by_method = grouped_anytime_points(
        points,
        require_full_success=require_full_success,
        compress_full_success_iris=compress_full_success_iris,
        allow_partial_success_methods=allow_partial_success_methods,
    )
    all_x: list[float] = []
    all_y: list[float] = []
    scaled_translation = None
    if method_order is None:
        ordered_methods = sorted(by_method)
    else:
        ordered_methods = [method for method in method_order if method in by_method]
        ordered_methods.extend(method for method in sorted(by_method) if method not in ordered_methods)
    for method_index, method in enumerate(ordered_methods):
        rows = by_method[method]
        label, color, marker = anytime_style(method)
        rows = sorted(rows, key=lambda row: int(row.get("stage_index", 0)))
        marker_rows = anytime_marker_rows(method, rows, forced_marker_keys)
        line_rows = marker_rows if method == "ompl_bitstar" else rows
        xs = [anytime_plot_x(row.get("total_s")) for row in line_rows]
        ys = [safe_float(row.get("path_length")) for row in line_rows]
        all_x.extend(xs)
        all_y.extend(ys)
        line_zorder = 2 + method_index
        marker_zorder = 3 + method_index
        if len(xs) > 1:
            line_kwargs: dict[str, Any] = {"color": color, "linewidth": 0.82, "alpha": 0.74, "zorder": line_zorder}
            offset = ANYTIME_MARKER_OFFSETS.get(method, (0.0, 0.0)) if separate_markers else (0.0, 0.0)
            if offset != (0.0, 0.0):
                if scaled_translation is None:
                    from matplotlib.transforms import ScaledTranslation

                    scaled_translation = ScaledTranslation
                line_kwargs["transform"] = ax.transData + scaled_translation(offset[0] / 72.0, offset[1] / 72.0, ax.figure.dpi_scale_trans)
            ax.plot(xs, ys, **line_kwargs)
        marker_xs = [anytime_plot_x(row.get("total_s")) for row in marker_rows]
        marker_ys = [safe_float(row.get("path_length")) for row in marker_rows]
        for row, x_value, y_value in zip(marker_rows, marker_xs, marker_ys):
            offset = ANYTIME_MARKER_OFFSETS.get(method, (0.0, 0.0)) if separate_markers else (0.0, 0.0)
            draw_anytime_marker(ax, row, x_value, y_value, label=label, color=color, marker=marker, zorder=marker_zorder, marker_offset=offset)
    ax.set_xscale("log")
    if show_xlabel:
        ax.set_xlabel("time / budget (s)")
    if show_ylabel:
        ax.set_ylabel("mean audited path length (rad)")
    else:
        ax.set_ylabel("")
    if title:
        ax.set_title(title, fontsize=7.2)
    ax.grid(True, which="both", linewidth=0.35, alpha=0.35)
    ax.tick_params(labelsize=6.2)
    ax.xaxis.label.set_size(7.0)
    ax.yaxis.label.set_size(7.0)
    x_range = positive_range(all_x)
    if x_range:
        ax.set_xlim(*x_range)
    y_range = centered_linear_range(all_y)
    if y_range:
        ax.set_ylim(*y_range)
    if show_legend:
        handles, labels = ax.get_legend_handles_labels()
        unique: dict[str, Any] = {}
        for handle, label in zip(handles, labels):
            unique.setdefault(label, handle)
        ax.legend(unique.values(), unique.keys(), fontsize=5.6, ncol=min(5, len(unique)), frameon=False, loc="best", handletextpad=0.35, columnspacing=0.7)


def draw_selected_tradeoff_points(ax: Any, outputs: Path, scenario_key: str, *, separate_markers: bool = False) -> None:
    rows = [row for row in best_tradeoff_points(outputs) if str(row.get("scenario_key")) == str(scenario_key)]
    if not rows:
        return
    from matplotlib.transforms import ScaledTranslation

    label_used = False
    for row in rows:
        if row.get("path_length") is None or row.get("total_s") is None:
            continue
        if not point_has_full_success(row):
            continue
        method = str(row.get("method"))
        normalized = str(row.get("normalized_method", normalize_anytime_method(method)))
        offset = ANYTIME_MARKER_OFFSETS.get(method, ANYTIME_MARKER_OFFSETS.get(normalized, (0.0, 0.0))) if separate_markers else (0.0, 0.0)
        transform = ax.transData
        if offset != (0.0, 0.0):
            transform = transform + ScaledTranslation(offset[0] / 72.0, offset[1] / 72.0, ax.figure.dpi_scale_trans)
        ax.scatter(
            [anytime_plot_x(row.get("total_s"))],
            [safe_float(row.get("path_length"))],
            marker="o",
            s=78.0,
            facecolors="none",
            edgecolors="#f0b000",
            linewidths=1.05,
            zorder=20,
            transform=transform,
            label="selected trade-off" if not label_used else None,
        )
        label_used = True


def plot_query_amortization_panel(
    ax: Any,
    outputs: Path,
    *,
    scenario_key: str | None = "Shelf+IIWA",
    scenario_keys: Iterable[str] | None = None,
    title: str = "Query amortization",
    show_xlabel: bool = True,
    show_ylabel: bool = True,
) -> None:
    rows = query_amortization_rows_from_best(outputs, scenario_key=scenario_key, scenario_keys=scenario_keys)
    if not rows:
        ax.text(0.5, 0.5, "best trade-off rows missing", ha="center", va="center", fontsize=7.0)
        ax.axis("off")
        return
    label_to_style = {
        "sbf": (SBF_SH_LABEL, "#0b6e4f", "o"),
        "drake_iris_np_gcs": (r"IRIS-NP+GCS", "#7b2cbf", "P"),
        "ompl_prm": (r"PRM", "#1f77b4", "s"),
        "ompl_bitstar": (r"BIT*", "#555555", "^"),
        "ompl_rrtconnect": (r"RRTConnect", "#d95f02", "D"),
    }
    by_method: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        by_method.setdefault(str(row.get("method")), []).append(row)
    for method, group in sorted(by_method.items()):
        label, color, marker = label_to_style.get(method, (tex_escape(method), "#666666", "o"))
        group = sorted(group, key=lambda row: int(row.get("queries", 0) or 0))
        xs = [int(row.get("queries", 0) or 0) for row in group]
        ys = [max(1e-5, safe_float(row.get("amortized_s_per_query"), 0.0)) for row in group]
        ax.plot(xs, ys, marker=marker, markersize=3.8, linewidth=1.05, color=color, label=label, alpha=0.92)
    ax.set_xscale("log")
    ax.set_yscale("log")
    if show_xlabel:
        ax.set_xlabel("number of queries")
    else:
        ax.set_xlabel("")
    if show_ylabel:
        ax.set_ylabel("amortized time / query (s)")
    else:
        ax.set_ylabel("")
    ax.set_title(title, fontsize=7.2)
    ax.grid(True, which="both", linewidth=0.35, alpha=0.35)
    ax.tick_params(labelsize=6.2)
    ax.xaxis.label.set_size(7.0)
    ax.yaxis.label.set_size(7.0)


def shelf_drake_anytime_points(outputs: Path) -> list[dict[str, Any]]:
    anytime = load_json(outputs / "tro2026_shelf_iris_np_gcs_anytime.json")
    points = anytime_summary_points(anytime)
    if any(point_has_full_success(point) and point.get("path_length") is not None for point in points):
        return points
    payload = load_json(outputs / "marcucci_iris_np_gcs.json")
    if not payload:
        return []
    queries = payload.get("queries", [])
    successes = [row for row in queries if safe_float(row.get("sr"), 0.0) > 0.0]
    path_s = sum(safe_float(row.get("len_med")) for row in successes)
    query_s = sum(safe_float(row.get("t_med_s")) for row in successes)
    task_count = len(queries) or int((payload.get("summary", {}) or {}).get("n_queries", 0) or 0)
    success_count = sum(int(safe_float(row.get("success_count"), 0.0)) for row in queries)
    build_s = safe_float((payload.get("summary", {}) or {}).get("build_s_median"))
    if path_s <= 0.0 or task_count <= 0:
        return []
    return [{
        "method": "drake_iris_np_gcs",
        "stage_index": 0,
        "stage_id": "reference",
        "build_s": build_s,
        "query_s": query_s,
        "total_s": build_s + query_s,
        "path_length": path_s / max(1, len(successes)),
        "path_length_total": path_s,
        "audit_sr": float(success_count) / max(1, task_count),
        "seed_count": int(payload.get("seeds", 1) or 1),
        "task_count": task_count,
        "success_count": success_count,
        "protocol": payload.get("source_protocol", "current_drake_iris_np_gcs_reference"),
        "promoted": True,
    }]


def random_drake_anytime_points(outputs: Path) -> dict[str, list[dict[str, Any]]]:
    anytime = load_json(outputs / RANDOM_IRIS_ANYTIME_ARTIFACT)
    panels = (anytime or {}).get("panels", {})
    if panels:
        out: dict[str, list[dict[str, Any]]] = {}
        for key, panel in panels.items():
            out[str(key)] = anytime_summary_points(panel if isinstance(panel, dict) else None)
        return out
    return {}


METHOD_ORDER = ["sbf", "drake_iris_np_gcs", "ompl_prm", "ompl_bitstar", "ompl_rrtconnect"]
METHOD_CELL_LABELS = {
    "sbf": SBF_SH_LABEL,
    "drake_iris_np_gcs": "IRIS+GCS",
    "ompl_prm": "PRM",
    "ompl_bitstar": "BIT*",
    "ompl_rrtconnect": "RRTConnect",
}

SELECTED_TRADEOFF_STAGE_OVERRIDES: dict[tuple[str, str], int] = {}

RANDOM_ANYTIME_FULL_ARTIFACT = "tro2026_random_anytime_tradeoff_full_unbiased_strictaudit_rrtgrid_sbfopt_20260512.json"
RANDOM_SBF_MAIN_ARTIFACT = RANDOM_ANYTIME_FULL_ARTIFACT
RANDOM_IRIS_ANYTIME_ARTIFACT = "tro2026_random_iris_np_gcs_anytime_extended_grid_strictaudit_20260512.json"


def normalize_anytime_method(method: Any) -> str:
    text = str(method)
    if text.startswith("sbf_"):
        return "sbf"
    return text


def scenario_label_from_key(key: str) -> str:
    if key == "Shelf+IIWA":
        return key
    robot, _, difficulty = key.partition(":")
    robot_label = robot.upper() if robot != "panda" else "Panda"
    return f"{robot_label}-{difficulty.capitalize()}"


def random_sbf_main_points(outputs: Path, key: str) -> list[dict[str, Any]]:
    payload = load_json(outputs / RANDOM_SBF_MAIN_ARTIFACT)
    panel = ((payload or {}).get("panels", {}) or {}).get(str(key), {})
    points = anytime_summary_points(panel if isinstance(panel, dict) else None)
    return [dict(point) for point in points if normalize_anytime_method(point.get("method")) == "sbf"]


def collect_anytime_stage_points(outputs: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    shelf = load_json(outputs / "tro2026_shelf_anytime_tradeoff_full.json")
    if shelf:
        for point in anytime_summary_points(shelf) + shelf_drake_anytime_points(outputs):
            row = dict(point)
            row["scenario_key"] = "Shelf+IIWA"
            row["scenario"] = "Shelf+IIWA"
            row["difficulty"] = "shelf"
            row["normalized_method"] = normalize_anytime_method(row.get("method"))
            rows.append(row)
    random_payload = load_json(outputs / RANDOM_ANYTIME_FULL_ARTIFACT)
    panels = (random_payload or {}).get("panels", {})
    drake_points = random_drake_anytime_points(outputs)
    for key in sorted(set(panels) | set(drake_points)):
        panel = panels.get(key, {})
        points = anytime_summary_points(panel if isinstance(panel, dict) else None)
        sbf_points = random_sbf_main_points(outputs, str(key))
        if sbf_points:
            points = [point for point in points if normalize_anytime_method(point.get("method")) != "sbf"]
            points.extend(sbf_points)
        points.extend(drake_points.get(key, []))
        _, _, difficulty = str(key).partition(":")
        for point in points:
            row = dict(point)
            row["scenario_key"] = str(key)
            row["scenario"] = scenario_label_from_key(str(key))
            row["difficulty"] = difficulty
            row["normalized_method"] = normalize_anytime_method(row.get("method"))
            rows.append(row)
    return rows


def pareto_tradeoff_points(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    full = [
        point
        for point in points
        if point.get("total_s") is not None
        and point.get("path_length") is not None
        and point_has_full_success(point)
    ]
    frontier: list[dict[str, Any]] = []
    for point in full:
        time_s = safe_float(point.get("total_s"), float("inf"))
        path = safe_float(point.get("path_length"), float("inf"))
        dominated = False
        for other in full:
            other_time = safe_float(other.get("total_s"), float("inf"))
            other_path = safe_float(other.get("path_length"), float("inf"))
            if (
                other_time <= time_s + 1e-12
                and other_path <= path + PATH_DOMINATION_EPS
                and (other_time < time_s - 1e-12 or other_path < path - PATH_DOMINATION_EPS)
            ):
                dominated = True
                break
        if not dominated:
            frontier.append(point)
    return sorted(frontier, key=lambda point: safe_float(point.get("total_s"), float("inf")))


def choose_best_tradeoff_point(points: list[dict[str, Any]], *, success_only: bool = False) -> dict[str, Any] | None:
    usable = [point for point in points if point.get("total_s") is not None]
    if not usable:
        return None
    if success_only:
        return min(
            usable,
            key=lambda point: (-safe_float(point.get("audit_sr"), 0.0), safe_float(point.get("total_s"), float("inf")), safe_float(point.get("path_length"), float("inf"))),
        )
    full = pareto_tradeoff_points(usable)
    if not full:
        best = choose_best_tradeoff_point(usable, success_only=True)
        if best is not None:
            best = dict(best)
            best["best_metric"] = "success_lt100"
        return best
    best_path = min(safe_float(point.get("path_length"), float("inf")) for point in full)
    path_limit = best_path * (1.0 + PATH_TRADEOFF_TOL) if best_path > 0 else best_path + PATH_TRADEOFF_TOL
    candidates = [point for point in full if safe_float(point.get("path_length"), float("inf")) <= path_limit]
    def weighted_score(point: dict[str, Any]) -> float:
        time_s = safe_float(point.get("total_s"), float("inf"))
        path = safe_float(point.get("path_length"), float("inf"))
        return path + PATH_LOG_TIME_PENALTY_RAD * math.log(PATH_LOG_TIME_OFFSET_S + max(0.0, time_s))

    best = min(
        candidates,
        key=lambda point: (
            weighted_score(point),
            safe_float(point.get("path_length"), float("inf")),
            safe_float(point.get("total_s"), float("inf")),
        ),
    )
    best = dict(best)
    best["best_metric"] = "scenario_pareto_log_time_path_utility_100"
    best["best_path_limit"] = path_limit
    best["best_quality_score"] = weighted_score(best)
    best["best_log_time_penalty_rad"] = PATH_LOG_TIME_PENALTY_RAD
    best["best_log_time_offset_s"] = PATH_LOG_TIME_OFFSET_S
    return best


def best_tradeoff_points(outputs: Path) -> list[dict[str, Any]]:
    rows = collect_anytime_stage_points(outputs)
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault((str(row.get("scenario_key")), str(row.get("normalized_method"))), []).append(row)
    best_rows: list[dict[str, Any]] = []
    for (scenario_key, method), points in sorted(grouped.items(), key=lambda item: (item[0][0], METHOD_ORDER.index(item[0][1]) if item[0][1] in METHOD_ORDER else 99)):
        if method not in METHOD_ORDER:
            continue
        difficulty = str(points[0].get("difficulty", ""))
        override_stage_index = SELECTED_TRADEOFF_STAGE_OVERRIDES.get((scenario_key, method))
        best = None
        if override_stage_index is not None:
            override_candidates = [
                dict(point)
                for point in points
                if int(point.get("stage_index", -1)) == int(override_stage_index)
                and point.get("path_length") is not None
                and point.get("total_s") is not None
                and point_has_full_success(point)
            ]
            if override_candidates:
                best = min(
                    override_candidates,
                    key=lambda point: (safe_float(point.get("total_s"), float("inf")), safe_float(point.get("path_length"), float("inf"))),
                )
                best["best_metric"] = "manual_stage_override"
                best["selected_override_stage_index"] = int(override_stage_index)
        if best is None:
            best = choose_best_tradeoff_point(points, success_only=False)
        if best is None:
            continue
        best["scenario_key"] = scenario_key
        best["scenario"] = points[0].get("scenario", scenario_label_from_key(scenario_key))
        best["difficulty"] = difficulty
        best["normalized_method"] = method
        best_rows.append(best)
    return best_rows


def query_amortization_rows_from_best(
    outputs: Path,
    *,
    scenario_key: str | None = "Shelf+IIWA",
    scenario_keys: Iterable[str] | None = None,
) -> list[dict[str, Any]]:
    counts = [1, 5, 10, 20, 50]
    rows: list[dict[str, Any]] = []
    selected_keys = set(str(key) for key in scenario_keys) if scenario_keys is not None else None
    grouped: dict[str, list[dict[str, Any]]] = {}
    for point in best_tradeoff_points(outputs):
        key = str(point.get("scenario_key"))
        if selected_keys is not None:
            if key not in selected_keys:
                continue
        elif scenario_key is not None and key != str(scenario_key):
            continue
        method = str(point.get("normalized_method"))
        grouped.setdefault(method, []).append(point)
    for method, points in sorted(grouped.items(), key=lambda item: METHOD_ORDER.index(item[0]) if item[0] in METHOD_ORDER else 99):
        build_values: list[float] = []
        per_query_values: list[float] = []
        for point in points:
            seed_count = max(1.0, safe_float(point.get("seed_count"), 1.0))
            tasks_per_seed = max(1.0, safe_float(point.get("task_count"), 1.0) / seed_count)
            build_values.append(safe_float(point.get("build_s"), 0.0))
            per_query_values.append(safe_float(point.get("query_s"), 0.0) / tasks_per_seed)
        build_s = mean(build_values) or 0.0
        per_query_s = mean(per_query_values) or 0.0
        for count in counts:
            total_s = build_s + count * per_query_s
            rows.append({
                "method": method,
                "queries": count,
                "build_s": build_s,
                "per_query_s": per_query_s,
                "total_s": total_s,
                "amortized_s_per_query": total_s / count,
            })
    return rows


def write_shelf_anytime_tradeoff_plot(outputs: Path, out_dir: Path) -> None:
    payload = load_json(outputs / "tro2026_shelf_anytime_tradeoff_full.json")
    if not payload:
        return
    points = anytime_summary_points(payload)
    points.extend(shelf_drake_anytime_points(outputs))
    if not points:
        return
    try:
        plt = import_pyplot()
    except Exception as exc:
        print(f"[tables] skip shelf anytime plot: {exc}")
        return
    fig, (ax, amort_ax) = plt.subplots(1, 2, figsize=(7.12, 3.18), gridspec_kw={"width_ratios": [1.18, 1.0], "wspace": 0.24})
    plot_anytime_points(
        ax,
        points,
        show_legend=False,
        show_xlabel=True,
        require_full_success=True,
        separate_markers=True,
        method_order=["sbf", "ompl_prm", "ompl_rrtconnect", "ompl_bitstar", "drake_iris_np_gcs"],
        forced_marker_keys=selected_marker_keys(outputs, "Shelf+IIWA"),
    )
    draw_selected_tradeoff_points(ax, outputs, "Shelf+IIWA", separate_markers=True)
    ax.set_ylabel("mean audited path length (rad)")
    ax.set_title("Shelf+IIWA anytime trade-off", fontsize=8.1)
    plot_query_amortization_panel(amort_ax, outputs)
    amort_ax.set_title("Query amortization", fontsize=8.1)
    for current_ax in (ax, amort_ax):
        current_ax.tick_params(labelsize=6.9)
        current_ax.xaxis.label.set_size(7.7)
        current_ax.yaxis.label.set_size(7.7)
    handles, labels = ax.get_legend_handles_labels()
    unique: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        unique.setdefault(label, handle)
    h2, l2 = amort_ax.get_legend_handles_labels()
    for handle, label in zip(h2, l2):
        unique.setdefault(label, handle)
    if unique:
        fig.legend(unique.values(), unique.keys(), loc="upper center", bbox_to_anchor=(0.5, 0.998), ncol=len(unique), fontsize=6.1, frameon=False, handletextpad=0.35, columnspacing=0.78)
    fig.subplots_adjust(left=0.074, right=0.986, bottom=0.17, top=0.81, wspace=0.26)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_tro_shelf_anytime_tradeoff.pdf")
    plt.close(fig)


def write_random_anytime_tradeoff_plot(outputs: Path, out_dir: Path) -> None:
    payload = load_json(outputs / RANDOM_ANYTIME_FULL_ARTIFACT)
    if not payload:
        return
    panels = (payload or {}).get("panels", {})
    drake_points = random_drake_anytime_points(outputs)
    if not panels and not drake_points:
        return
    try:
        plt = import_pyplot()
    except Exception as exc:
        print(f"[tables] skip random anytime plot: {exc}")
        return
    difficulty_order = ["easy", "medium", "hard"]
    robot_order = ["iiwa", "ur5", "panda"]
    robots_present = [
        robot for robot in robot_order
        if any(f"{robot}:{difficulty}" in panels or f"{robot}:{difficulty}" in drake_points for difficulty in difficulty_order)
    ]
    if not robots_present:
        return
    fig = plt.figure(figsize=(7.2, 1.90 * len(robots_present) + 0.72))
    grid = fig.add_gridspec(len(robots_present), 4, width_ratios=[1.0, 1.0, 1.0, 1.05], hspace=0.28, wspace=0.24)
    axes = []
    for row_index, robot in enumerate(robots_present):
        for col, difficulty in enumerate(difficulty_order):
            ax = fig.add_subplot(grid[row_index, col])
            axes.append(ax)
            key = f"{robot}:{difficulty}"
            panel = panels.get(key, {})
            points = anytime_summary_points(panel if isinstance(panel, dict) else None)
            sbf_points = random_sbf_main_points(outputs, key)
            if sbf_points:
                points = [point for point in points if normalize_anytime_method(point.get("method")) != "sbf"]
                points.extend(sbf_points)
            if key in drake_points:
                points.extend(drake_points[key])
            title = f"{robot.upper() if robot != 'panda' else 'Panda'}-{difficulty.capitalize()}"
            plot_anytime_points(
                ax,
                points,
                title=title,
                show_legend=False,
                show_xlabel=row_index == len(robots_present) - 1,
                show_ylabel=col == 0,
                require_full_success=True,
                compress_full_success_iris=True,
                separate_markers=True,
                method_order=["sbf", "ompl_prm", "ompl_rrtconnect", "ompl_bitstar", "drake_iris_np_gcs"],
                forced_marker_keys=selected_marker_keys(outputs, key),
            )
            draw_selected_tradeoff_points(ax, outputs, key, separate_markers=True)
            panel_y_range = displayed_anytime_y_range(
                points,
                require_full_success=True,
                compress_full_success_iris=True,
                pad_fraction=0.035,
            )
            if panel_y_range:
                ax.set_ylim(*panel_y_range)
        amort_ax = fig.add_subplot(grid[row_index, 3])
        robot_scenarios = [
            f"{robot}:{difficulty}" for difficulty in difficulty_order
            if f"{robot}:{difficulty}" in panels or f"{robot}:{difficulty}" in drake_points
        ]
        plot_query_amortization_panel(
            amort_ax,
            outputs,
            scenario_key=None,
            scenario_keys=robot_scenarios,
            title=f"{robot.upper() if robot != 'panda' else 'Panda'} amortization",
            show_xlabel=row_index == len(robots_present) - 1,
            show_ylabel=False,
        )
    handles, labels = axes[0].get_legend_handles_labels()
    unique: dict[str, Any] = {}
    for ax in axes[1:]:
        h, l = ax.get_legend_handles_labels()
        handles.extend(h)
        labels.extend(l)
    for handle, label in zip(handles, labels):
        unique.setdefault(label, handle)
    if unique:
        fig.legend(unique.values(), unique.keys(), loc="upper center", bbox_to_anchor=(0.5, 0.995), ncol=len(unique), fontsize=6.0, frameon=False, handletextpad=0.35, columnspacing=0.85)
    fig.subplots_adjust(left=0.052, right=0.996, bottom=0.10, top=0.91, hspace=0.38, wspace=0.28)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_tro_random_anytime_tradeoff.pdf", bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)


def plot_success_points(ax: Any, points: list[dict[str, Any]], *, title: str) -> None:
    by_method: dict[str, list[dict[str, Any]]] = {}
    for point in points:
        if point.get("total_s") is None:
            continue
        by_method.setdefault(str(point.get("method")), []).append(point)
    all_x: list[float] = []
    for method, rows in sorted(by_method.items()):
        label, color, marker = anytime_style(method)
        rows = sorted(rows, key=lambda row: int(row.get("stage_index", 0)))
        xs = [anytime_plot_x(row.get("total_s")) for row in rows]
        ys = [100.0 * safe_float(row.get("audit_sr"), 0.0) for row in rows]
        all_x.extend(xs)
        if len(xs) > 1:
            ax.plot(xs, ys, color=color, linewidth=0.95, alpha=0.82, zorder=2)
        ax.scatter(xs, ys, marker=marker, s=26.0, facecolors=color, edgecolors=color, linewidths=0.55, alpha=0.9, zorder=3, label=label)
    ax.set_xscale("log")
    ax.set_ylim(-3.0, 103.0)
    ax.set_xlabel("time / budget (s)")
    ax.set_ylabel("fixed-step audit success rate (%)")
    ax.set_title(title, fontsize=7.2)
    ax.grid(True, which="both", linewidth=0.35, alpha=0.35)
    ax.tick_params(labelsize=6.2)
    ax.xaxis.label.set_size(7.0)
    ax.yaxis.label.set_size(7.0)
    x_range = positive_range(all_x)
    if x_range:
        ax.set_xlim(*x_range)


def write_random_hard_success_plot(outputs: Path, out_dir: Path) -> None:
    payload = load_json(outputs / RANDOM_ANYTIME_FULL_ARTIFACT)
    panels = (payload or {}).get("panels", {})
    drake_points = random_drake_anytime_points(outputs)
    keys = [key for key in ["iiwa:hard", "ur5:hard", "panda:hard"] if key in panels or key in drake_points]
    if not any(key in panels or key in drake_points for key in keys):
        return
    try:
        plt = import_pyplot()
    except Exception as exc:
        print(f"[tables] skip random hard success plot: {exc}")
        return
    fig, axes = plt.subplots(1, len(keys), figsize=(3.15 * len(keys), 2.65), sharey=True)
    if len(keys) == 1:
        axes = [axes]
    for ax, key in zip(axes, keys):
        panel = panels.get(key, {})
        points = list(((panel.get("summary", {}) or {}).get("points", [])) if isinstance(panel, dict) else [])
        points.extend(drake_points.get(key, []))
        title = scenario_label_from_key(key)
        plot_success_points(ax, points, title=title)
    handles, labels = axes[0].get_legend_handles_labels()
    unique: dict[str, Any] = {}
    for ax in axes[1:]:
        h, l = ax.get_legend_handles_labels()
        handles.extend(h)
        labels.extend(l)
    for handle, label in zip(handles, labels):
        unique.setdefault(label, handle)
    if unique:
        fig.legend(unique.values(), unique.keys(), loc="upper center", bbox_to_anchor=(0.5, 0.995), ncol=len(unique), fontsize=6.0, frameon=False, handletextpad=0.35, columnspacing=0.85)
    fig.subplots_adjust(left=0.085, right=0.985, bottom=0.20, top=0.76, wspace=0.12)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_tro_random_hard_success.pdf")
    plt.close(fig)


def gcs_table(outputs: Path) -> str:
    direct = load_json(outputs / "marcucci_merger_gcs.json")
    audited = load_json(outputs / "tro2026_exp07_gcs_full.json") or load_json(outputs / "marcucci_audited_corridor_gcs.json")
    rows: list[list[Any]] = []
    direct_by_name = {row.get("name"): row for row in direct.get("gcs_queries", [])} if direct else {}
    audited_rows = audited.get("queries", []) if audited else []
    for row in audited_rows:
        direct_row = direct_by_name.get(row.get("name"), {})
        expanded = next((attempt for attempt in row.get("gcs_attempts", []) if attempt.get("attempt") == "overlap_expanded_sbf_corridor"), {})
        rows.append([
            row.get("name"),
            bool(direct_row.get("ok")),
            bool(direct_row.get("audit", {}).get("passed")),
            bool(expanded.get("ok")),
            bool(expanded.get("strict_audit_passed")),
            bool(row.get("final_strict_audit_passed", row.get("gcs_strict_audit_passed"))),
            row.get("final_source"),
            row.get("final_length"),
        ])
    return appendix_table(
        "GCS composition result. Expanded boxes repair LinearGCS overlap feasibility but are counted only after the fixed-resolution final audit.",
        "tab:tro_audited_gcs",
        "lrrrrrrr",
        ["Query", "Direct solved", "Direct audit", "Expanded solved", "Expanded audit", "Final audit", "Final source", "Length (rad)"],
        rows,
    )


def merger_table(outputs: Path) -> str:
    study = load_json(outputs / "marcucci_protected_merger_study.json")
    rows: list[list[Any]] = []
    if study:
        for item in study.get("rows", []):
            rows.append([
                item.get("mode"),
                item.get("region_policy"),
                item.get("solve_count"),
                item.get("audit_pass_count"),
                item.get("solved_unsafe_count"),
                item.get("max_gap_before_expansion"),
            ])
        return appendix_table(
            "Merger and protected-GCS study. Merging or expanding boxes changes GCS feasibility, not the fixed-resolution safety acceptance result.",
            "tab:tro_merger_control",
            "llrrrr",
            ["Mode", "Regions", "Solve", "Audit", "Unsafe", "Max gap"],
            rows,
        )

    payload = load_json(outputs / "marcucci_merger_gcs.json")
    if payload:
        build = payload.get("build", {})
        rows.append([
            "Direct merger+GCS",
            build.get("raw_boxes"),
            build.get("final_boxes"),
            build.get("boxes_after_refine"),
            build.get("merge_ms"),
            build.get("adjacency_edges_geometric_for_gcs"),
            sum(1 for row in payload.get("gcs_queries", []) if row.get("audit", {}).get("passed")),
        ])
    return appendix_table(
        "Merger and direct-GCS control. Audit pass count is the safety metric.",
        "tab:tro_merger_control",
        "lrrrrrr",
        ["Run", "Raw", "Merged", "Refined", "Merge (ms)", "GCS edges", "Audit pass"],
        rows,
    )


def random_scene_table(outputs: Path) -> str:
    _, payload = first_existing(outputs, ["tro2026_exp05_random_sbf_full.json", "exp5_random_robot_scenes_standalone.json", "smoke_exp5.json"])
    rows: list[list[Any]] = []
    if payload:
        for item in payload.get("summary", []):
            if str(item.get("method")) != "support_hull_coverage":
                continue
            rows.append([
                item.get("robot"),
                item.get("difficulty"),
                SBF_SH_LABEL,
                item.get("build_mean_s"),
                item.get("box_count_mean"),
                item.get("query_sr"),
                item.get("audit_sr"),
            ])
    return appendix_table(
        "Cross-robot random-scene standalone SBF evaluation.",
        "tab:tro_random_scenes",
        "lllrrrr",
        ["Robot", "Difficulty", "Method", "Build (s)", "Boxes", "Query SR", "Audit SR"],
        rows,
    )


def random_rrt_baseline_table(outputs: Path) -> str:
    sbf_payload = load_json(outputs / "tro2026_exp05_random_sbf_full.json") or load_json(outputs / "exp5_random_robot_scenes_standalone.json")
    rrt_payload = load_json(outputs / "tro2026_exp05_random_rrt_full.json") or load_json(outputs / "random_scene_rrt_connect_baseline.json")
    rows: list[list[Any]] = []
    groups: set[tuple[str, str]] = set()
    if sbf_payload:
        for item in sbf_payload.get("summary", []):
            if str(item.get("method")) == "support_hull_coverage":
                groups.add((str(item.get("robot")), str(item.get("difficulty"))))
    if rrt_payload:
        for item in rrt_payload.get("summary", []):
            groups.add((str(item.get("robot")), str(item.get("difficulty"))))
    sbf_rows = sbf_payload.get("rows", []) if sbf_payload else []
    rrt_summary = {
        (str(item.get("robot")), str(item.get("difficulty"))): item
        for item in (rrt_payload.get("summary", []) if rrt_payload else [])
    }
    for robot_name, difficulty in sorted(groups):
        candidates = [
            item for item in (sbf_payload.get("summary", []) if sbf_payload else [])
            if str(item.get("robot")) == robot_name and str(item.get("difficulty")) == difficulty
            and str(item.get("method")) == "support_hull_coverage"
        ]
        if candidates:
            best = max(candidates, key=lambda item: (float(item.get("audit_sr") or 0.0), float(item.get("query_sr") or 0.0), -float(item.get("build_mean_s") or 0.0)))
            raw = [
                item for item in sbf_rows
                if str(item.get("robot")) == robot_name and str(item.get("difficulty")) == difficulty and str(item.get("method")) == str(best.get("method"))
            ]
            success_rows = [item for item in raw if item.get("query", {}).get("ok")]
            rows.append([
                robot_name,
                difficulty,
                SBF_SH_LABEL,
                best.get("build_mean_s"),
                mean(item.get("query", {}).get("t_s") for item in raw),
                best.get("audit_sr"),
                mean(item.get("query", {}).get("length") for item in success_rows),
                best.get("box_count_mean"),
            ])
        rrt = rrt_summary.get((robot_name, difficulty))
        if rrt:
            rows.append([
                robot_name,
                difficulty,
                "OMPL RRTConnect",
                0,
                rrt.get("query_mean_s"),
                rrt.get("audit_sr"),
                rrt.get("success_length_mean"),
                "--",
            ])
    return appendix_table(
        "Raw OMPL RRTConnect single-query baseline on the random-scene protocol. It uses the SBF collision checker, no reusable build, and no shortcut simplification by default.",
        "tab:tro_random_rrt_baseline",
        "lllrrrrr",
        ["Robot", "Difficulty", "Planner", "Build (s)", "Query (s)", "Audit SR", "Path", "Boxes"],
        rows,
    )


def rebuild_table(outputs: Path) -> str:
    _, payload = first_existing(outputs, ["tro2026_exp09_random_dynamic_rebuild.json", "tro2026_exp06_dynamic_full.json", "obstacle_rebuild_standalone.json", "obstacle_rebuild_standalone_1seed.json"])
    rows: list[list[Any]] = []
    if payload and payload.get("experiment") == "exp09_random_dynamic_rebuild":
        sbf_stage = payload.get("sbf_stage") or {}
        stage_id = sbf_stage.get("stage_id", "selected")
        for item in payload.get("summary", []):
            rows.append([
                f"{item.get('from_stage')}->{item.get('to_stage')}",
                item.get("update_median_s"),
                item.get("warm_build_median_s", item.get("cold_build_median_s")),
                item.get("speedup_vs_warm_median", item.get("speedup_vs_cold_median")),
            ])
        return table(
            "Random-scene dynamic obstacle updates on matched easy/medium/hard edit paths. Warm denotes an isolated source-scene prewarm followed by a fresh target-scene rebuild, and speedup denotes the ratio of warm median to incremental median.",
            "tab:tro_dynamic_rebuild",
            "@{}lrrr@{}",
            ["Transition", "Incr. (s)", "Warm (s)", "Speedup"],
            rows,
            font_command=TABLE_FONT_DENSE,
            tabcolsep=2.6,
            arraystretch=0.95,
            resize_width="0.70\\columnwidth",
            float_spec="[t]",
        )
    if payload:
        summary = payload.get("summary", {})
        rows.append([
            payload.get("added_obstacle"),
            summary.get("build_time_s_median"),
            summary.get("boxes_before_median"),
            summary.get("boxes_removed_median"),
            summary.get("removal_ratio_median"),
            summary.get("rebuild_time_s_median"),
            summary.get("post_update_query_sr_median"),
            summary.get("post_update_audit_sr_median"),
        ])
    return table(
        "Dynamic obstacle insertion and localized box deletion/rebuild cost with post-update query/audit accounting when available.",
        "tab:tro_dynamic_rebuild",
        "@{}lrrrrrrr@{}",
        ["Obstacle", "Initial build (s)", "Boxes", "Removed", "Removal", "Rebuild (s)", "Query SR", "Audit SR"],
        rows,
        font_command=TABLE_FONT_DENSE,
        tabcolsep=2.6,
        arraystretch=0.95,
        resize_width="0.98\\columnwidth",
        float_spec="[t]",
    )


def parallel_table(outputs: Path) -> str:
    payload = load_json(outputs / "tro2026_exp08_parallel_full.json") or load_json(outputs / "parallel_scaling_standalone.json")
    rows: list[list[Any]] = []
    if payload:
        for item in payload.get("summary", []):
            rows.append([
                item.get("threads"),
                item.get("build_mean_s"),
                item.get("speedup"),
                item.get("efficiency"),
                item.get("box_count_mean"),
                item.get("audit_sr"),
            ])
    return appendix_table(
        "Parallel build scaling for the Marcucci end-to-end pipeline.",
        "tab:tro_parallel_scaling",
        "rrrrrr",
        ["Threads", "Build (s)", "Speedup", "Efficiency", "Boxes", "Audit SR"],
        rows,
    )


def mechanism_diagnostics_table(outputs: Path) -> str:
    payload = load_json(outputs / "tro_mechanism_diagnostics.json")
    rows: list[list[Any]] = []
    if payload:
        for item in payload.get("rows", []):
            rows.append([
                item.get("scope"),
                item.get("metric"),
                item.get("value"),
                item.get("interpretation"),
            ])
    return appendix_table(
        "Mechanism diagnostics for claim boundaries, failure modes, and negative controls.",
        "tab:tro_mechanism_diagnostics",
        "llll",
        ["Scope", "Metric", "Value", "Interpretation"],
        rows,
        tabcolsep=2.2,
    )


def audit_suite_table(outputs: Path) -> str:
    rows: list[list[Any]] = []
    suite = load_json(outputs / "tro2026_safety_accounting_full.json") or load_json(outputs / "paper_soundness_audit_suite.json")
    if suite:
        for row in suite.get("rows", []):
            rows.append([
                row.get("scope"),
                row.get("path_count"),
                row.get("audit_sr"),
                row.get("repair_or_fallback_events"),
                row.get("solved_unsafe_count"),
                row.get("remaining_unsafe_assumptions"),
            ])
        return appendix_table(
            "Paper-wide soundness audit suite. Solved-but-unsafe paths are counted separately and are never reported as planning success.",
            "tab:tro_soundness_audit",
            "lrrrrl",
            ["Scope", "Paths", "Audit SR", "Repair/Fallback", "Solved unsafe", "Residual assumption"],
            rows,
        )

    marcucci = load_json(outputs / "marcucci_corridor_refine_selfedge_s10.json")
    if marcucci:
        queries = marcucci.get("queries", [])
        rows.append([
            "SBF Marcucci 10-seed",
            len(queries),
            mean(row.get("audit_sr") for row in queries),
            median(row.get("repair_count_med") for row in queries),
            0,
        ])
    direct = load_json(outputs / "marcucci_merger_gcs.json")
    if direct:
        rows.append([
            "Direct GCS control",
            len(direct.get("gcs_queries", [])),
            mean(1.0 if row.get("audit", {}).get("passed") else 0.0 for row in direct.get("gcs_queries", [])),
            "--",
            len([row for row in direct.get("gcs_queries", []) if row.get("ok") and not row.get("audit", {}).get("passed")]),
        ])
    audited = load_json(outputs / "marcucci_audited_corridor_gcs.json")
    if audited:
        rows.append([
            "Audited corridor GCS",
            audited.get("summary", {}).get("query_count"),
            audited.get("summary", {}).get("gcs_strict_audit_pass_count", 0) / max(1, audited.get("summary", {}).get("query_count", 1)),
            audited.get("summary", {}).get("fallback_count"),
            0,
        ])
    return appendix_table(
        "Soundness audit suite. Solved-but-unsafe paths are counted separately and are never reported as planning success.",
        "tab:tro_soundness_audit",
        "lrrrr",
        ["Pipeline", "Paths", "Audit SR", "Repair/Fallback", "Solved unsafe"],
        rows,
    )


def validation_profile_table(outputs: Path) -> str:
    payload = load_json(outputs / "tro2026_exp14_validation_profiles.json")
    rows: list[list[Any]] = []
    if payload:
        for row in payload.get("rows", []):
            accounting = row.get("length_accounting", {})
            rows.append([
                row.get("label", row.get("profile")),
                row.get("evidence"),
                row.get("median_build_s"),
                row.get("median_box_count"),
                row.get("median_certified_box_count"),
                row.get("median_provisional_box_count"),
                row.get("query_sr"),
                row.get("audit_sr"),
                row.get("repair_event_count"),
                accounting.get("segment_edge_fraction"),
            ])
    return appendix_table(
        "Planner-level validation-profile ablation. Query SR denotes candidate-path success, and Audit SR denotes fixed-resolution audited final-path success after allowed repair.",
        "tab:tro_validation_profiles",
        "llrrrrrrrr",
        ["Profile", "Evidence", "Build (s)", "Boxes", "Cert.", "Prov.", "Query SR", "Audit SR", "Repairs", "Seg. frac."],
        rows,
    )


def query_amortization_table(outputs: Path) -> str:
    payload = load_json(outputs / "tro2026_exp15_query_amortization.json")
    rows: list[list[Any]] = []
    if payload:
        selected_counts = {1, 5, 20, 50}
        for row in payload.get("rows", []):
            count = int(row.get("queries", 0) or 0)
            if count not in selected_counts:
                continue
            rows.append([
                row.get("method"),
                count,
                row.get("build_s"),
                row.get("per_query_s"),
                row.get("audit_s"),
                row.get("amortized_s_per_query"),
                row.get("success_rate"),
            ])
    return table(
        "Multi-query amortization from existing build/query artifacts. SBF rows include reusable build time and final-audit time.",
        "tab:tro_query_amortization",
        "lrrrrrr",
        ["Method", "Queries", "Build (s)", "Query (s)", "Audit (s)", "Amort. (s/q)", "Audit SR"],
        rows,
    )


def evidence_validation_rows(outputs: Path) -> tuple[list[float], dict[tuple[float, str], dict[str, Any]]]:
    payload = load_json(outputs / "tro2026_exp01_endpoint_full.json")
    grouped: dict[tuple[float, str], dict[str, Any]] = {}
    width_order: list[float] = []
    if payload:
        for row in payload.get("rows", []):
            try:
                key = (float(row.get("fixed_width")), str(row.get("source")))
            except (TypeError, ValueError):
                continue
            grouped[key] = row
        width_order = sorted({key[0] for key in grouped})
    return width_order, grouped


def main_evidence_validation_table(outputs: Path) -> str:
    source_order = ["IFK", "HIFK_3", "HIFK_5", "CritSample", "Analytical", "MC"]
    width_order, grouped = evidence_validation_rows(outputs)

    body: list[str] = []
    for width in width_order:
        for source in source_order:
            row = grouped.get((width, source))
            if row is None:
                continue
            body.append(" & ".join([
                tex_escape(fmt_fixed(width, 2)),
                tex_escape(source),
                tex_escape(fmt(row.get("volume_mean"), 3)),
                tex_escape(fmt_fixed(row.get("time_us_mean"), 1)),
                tex_escape(fmt_fixed(row.get("max_gap_to_sampling_union"), 4)),
            ]) + r" \\")
            if source == source_order[-1] and width != width_order[-1]:
                body.append(r"\addlinespace")
    if not body:
        body.append(r"\multicolumn{5}{c}{No result artifact available.} \\")
    return "\n".join([
        "% Auto-generated from the endpoint interval-envelope artifact.",
        r"\begingroup",
        r"\centering",
        r"\captionof{table}{Endpoint-interval AABB source comparison at fixed widths.}",
        r"\label{tab:tro_main_evidence_validation}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{2.0pt}",
        r"\renewcommand{\arraystretch}{0.76}",
        r"\begin{tabular}{@{}llrrr@{}}",
        r"\toprule",
        r"Width & Source & $V$ (m$^3$) & Mean ($\mu$s) & Max gap (m) \\",
        r"\midrule",
        *body,
        r"\bottomrule",
        r"\end{tabular}",
        r"\par",
        r"\endgroup",
        "",
    ])


def write_evidence_validation_tradeoff_plot(outputs: Path, out_dir: Path) -> None:
    width_order, grouped = evidence_validation_rows(outputs)
    if not width_order:
        return
    plt = import_pyplot()
    source_order = ["IFK", "HIFK_3", "HIFK_5", "CritSample", "Analytical", "MC"]
    styles = {
        "IFK": ("#0b6e4f", "o"),
        "HIFK_3": ("#1b9e77", "X"),
        "HIFK_5": ("#2a9d8f", "P"),
        "CritSample": ("#1f77b4", "s"),
        "Analytical": ("#7b2cbf", "^"),
        "MC": ("#d95f02", "D"),
    }
    fig, axes = plt.subplots(1, 2, figsize=(3.45, 1.9), sharex=True)
    volume_ax, time_ax = axes
    for source in source_order:
        xs: list[float] = []
        volumes: list[float] = []
        times: list[float] = []
        for width in width_order:
            row = grouped.get((width, source))
            if row is None:
                continue
            xs.append(width)
            volumes.append(safe_float(row.get("volume_mean")))
            times.append(safe_float(row.get("time_us_mean")))
        if not xs:
            continue
        color, marker = styles[source]
        line_style = "-" if source in {"IFK", "HIFK_3", "HIFK_5"} else "--"
        volume_ax.plot(
            xs,
            volumes,
            marker=marker,
            color=color,
            linewidth=1.1,
            markersize=3.2,
            linestyle=line_style,
            label=source,
        )
        time_ax.plot(
            xs,
            times,
            marker=marker,
            color=color,
            linewidth=1.1,
            markersize=3.2,
            linestyle=line_style,
        )
    for ax, ylabel, title in [
        (volume_ax, "AABB volume", "Volume objective\n(certified lower; samples higher)"),
        (time_ax, "mean time (us)", "Cost"),
    ]:
        ax.set_yscale("log")
        ax.set_xlabel("width")
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=6.5)
        ax.grid(True, which="both", linewidth=0.35, alpha=0.35)
        ax.tick_params(labelsize=6)
        ax.xaxis.label.set_size(6.5)
        ax.yaxis.label.set_size(6.5)
    volume_ax.legend(fontsize=5.2, frameon=False, loc="upper left", handletextpad=0.35, borderaxespad=0.2)
    fig.tight_layout(pad=0.35, w_pad=0.7)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_tro_evidence_validation_tradeoff.pdf", bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)


PROFILE_SELECTION_METHOD_ORDER = [
    "sbf_scene_stage_ifk_strict",
    "sbf_scene_stage_crit_link_coverage",
    "sbf_scene_stage_kdop26_coverage",
    "sbf_scene_stage_support_hull_coverage",
]

PROFILE_SELECTION_LABELS = {
    "sbf_scene_stage_ifk_strict": "IFK+LinkIAABB",
    "sbf_scene_stage_crit_link_coverage": "Crit+LinkIAABB",
    "sbf_scene_stage_kdop26_coverage": "Crit+KDOP26",
    "sbf_scene_stage_support_hull_coverage": "Crit+SH",
}


def normalize_profile_selection_method(method: Any) -> str:
    text = str(method)
    if text.startswith("sbf_warm_"):
        return "sbf_scene_stage_" + text[len("sbf_warm_"):]
    return text


def main_random_profile_selection_table(outputs: Path) -> str:
    payload = load_json(outputs / "tro2026_random_profile_selection.json")
    panels = (payload or {}).get("panels", {})
    by_scenario_method: dict[tuple[str, str], dict[str, Any]] = {}
    for scenario_key, panel in panels.items():
        points = list(((panel.get("summary", {}) or {}).get("points", [])) if isinstance(panel, dict) else [])
        grouped: dict[str, list[dict[str, Any]]] = {}
        for point in points:
            method = normalize_profile_selection_method(point.get("method"))
            if method in PROFILE_SELECTION_METHOD_ORDER:
                grouped.setdefault(method, []).append(point)
        for method, method_points in grouped.items():
            best = choose_best_tradeoff_point(method_points, success_only=False)
            if best is not None:
                by_scenario_method[(str(scenario_key), method)] = best
    scenario_keys = sorted({key for key, _ in by_scenario_method}, key=scenario_sort_key)
    row_lines: list[str] = []
    for scenario_key in scenario_keys:
        cells = [best_tradeoff_cell(by_scenario_method.get((scenario_key, method))) for method in PROFILE_SELECTION_METHOD_ORDER]
        row_lines.append(tex_escape(scenario_label_from_key(scenario_key)) + " & " + " & ".join(cells) + r" \\")
    header = ["Scene"] + [PROFILE_SELECTION_LABELS[method] for method in PROFILE_SELECTION_METHOD_ORDER]
    if not row_lines:
        row_lines = [r"\multicolumn{5}{c}{No result artifact available.} \\"]
    return "\n".join([
        "% Auto-generated from current random profile-selection artifacts.",
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Common-rule tabulated SBF evidence-profile rows on balanced random-obstacle scenes. Easy, medium, and hard correspond to 4, 8, and 12 obstacles with side-length scales 0.12, 0.16, and 0.20, respectively. The table uses UR5 and Panda with five scene seeds per robot and difficulty. Cells follow the same full-success best-trade-off rule as the figures and provide a compact numeric slice rather than the primary figure-level evidence. SR denotes strict final success.}",
        r"\label{tab:tro_main_random_profile_selection}",
        TABLE_FONT_DENSE,
        r"\setlength{\tabcolsep}{2.4pt}",
        r"\resizebox{\textwidth}{!}{%",
        r"\begin{tabular}{@{}lcccc@{}}",
        r"\toprule",
        " & ".join(header) + r" \\",
        r"\midrule",
        *row_lines,
        r"\bottomrule",
        r"\end{tabular}}",
        r"\end{table*}",
        "",
    ])


def appendix_shelf_benchmark_table(outputs: Path) -> str:
    content = query_baseline_table(outputs)
    return content.replace(r"\label{tab:query}", r"\label{tab:tro_appendix_shelf_benchmark}")


def safety_fallback_text(outputs: Path) -> str:
    direct = load_json(outputs / "marcucci_merger_gcs.json")
    audited = load_json(outputs / "tro2026_exp07_gcs_full.json") or load_json(outputs / "marcucci_audited_corridor_gcs.json")
    direct_queries = direct.get("gcs_queries", []) if direct else []
    queries = audited.get("queries", []) if audited else []
    if not queries and not direct_queries:
        return "% Auto-generated by experiments/tro2026_generate_tables.py\nNo safety fallback artifact is available for the main-text safety check.\n"
    return "\n".join([
        "% Auto-generated by experiments/tro2026_generate_tables.py",
        "This acceptance-boundary principle is not a standalone benchmark: provisional GCS compositions and other external compositions are not credited unless they pass the fixed-resolution final audit, and the protected interface exposes only final audited paths.",
        "",
    ])


def appendix_generalization_table(outputs: Path) -> str:
    random_sbf = load_json(outputs / "tro2026_exp05_random_sbf_full.json") or load_json(outputs / "exp5_random_robot_scenes_standalone.json")
    random_rrt = load_json(outputs / "tro2026_exp05_random_rrt_full.json") or load_json(outputs / "random_scene_rrt_connect_baseline.json")
    sbf_summary = {}
    sbf_detail: dict[tuple[str, str], dict[str, list[float]]] = {}
    if random_sbf:
        for item in random_sbf.get("summary", []) if isinstance(random_sbf.get("summary"), list) else []:
            if str(item.get("method")) == "support_hull_coverage":
                sbf_summary[(str(item.get("robot")), str(item.get("difficulty")))] = item
        for item in random_sbf.get("rows", []) if isinstance(random_sbf.get("rows"), list) else []:
            key = (str(item.get("robot")), str(item.get("difficulty")))
            query = item.get("query", {}) if isinstance(item.get("query"), dict) else {}
            bucket = sbf_detail.setdefault(key, {"times": [], "paths": []})
            time_s = query.get("t_s")
            if time_s is None and query.get("planning_time_ms") is not None:
                time_s = safe_float(query.get("planning_time_ms")) / 1000.0
            if time_s is not None:
                bucket["times"].append(safe_float(time_s))
            if bool(query.get("audit_passed", False)) and query.get("length") is not None:
                bucket["paths"].append(safe_float(query.get("length")))
    rrt_summary = {}
    if random_rrt:
        for item in random_rrt.get("summary", []) if isinstance(random_rrt.get("summary"), list) else []:
            rrt_summary[(str(item.get("robot")), str(item.get("difficulty")))] = item
    ompl_stats = random_baseline_current_stats(outputs)
    keys = sorted(
        set(sbf_summary) | set(rrt_summary) | {(robot, difficulty) for robot, difficulty, _ in ompl_stats},
        key=lambda key: ({"ur5": 0, "panda": 1}.get(key[0], 99), {"easy": 0, "medium": 1, "hard": 2}.get(key[1], 99)),
    )

    method_specs = [
        {"key": "sbf", "label": r"\shortstack{SBF-SH\\cold build}", "columns": ["Build", "Query", "Path", "SR"]},
        {"key": "drake_iris_np_gcs", "label": r"\shortstack{IRIS-NP+GCS\\random scene}", "columns": ["Build", "Query", "Path", "SR"]},
        {"key": "ompl_prm", "label": r"\shortstack{PRM\\shared roadmap}", "columns": ["Build", "Query", "Path", "SR"]},
        {"key": "rrt_connect", "label": r"\shortstack{RRTConnect\\build=0\,s}", "columns": ["Query", "Path", "SR"]},
        {"key": "ompl_bitstar", "label": r"\shortstack{BIT*\\fixed timeout}", "columns": ["Query", "Path", "SR"]},
    ]

    def sr_percent(value: Any) -> float | None:
        if value is None:
            return None
        return 100.0 * safe_float(value)

    def baseline_value(robot: str, difficulty: str, method: str, field: str) -> Any:
        return ompl_stats.get((robot, difficulty, method), {}).get(field)

    row_end = " " + chr(92) * 2
    row_lines: list[str] = []
    for robot, difficulty in keys:
        label = f"{robot.upper() if robot != 'panda' else 'Panda'}-{difficulty.capitalize()}"
        sbf_item = sbf_summary.get((robot, difficulty), {})
        rrt_item = rrt_summary.get((robot, difficulty), {})
        detail = sbf_detail.get((robot, difficulty), {"times": [], "paths": []})
        values = [label]
        values.extend([
            fmt_fixed(sbf_item.get("build_median_s", sbf_item.get("build_mean_s"))),
            fmt_fixed(median(detail["times"])),
            fmt_fixed(median(detail["paths"])),
            fmt_fixed(sr_percent(sbf_item.get("audit_sr", sbf_item.get("query_sr"))), 1),
        ])
        values.extend([
            fmt_fixed(baseline_value(robot, difficulty, "drake_iris_np_gcs", "build_s")),
            fmt_fixed(baseline_value(robot, difficulty, "drake_iris_np_gcs", "query_s")),
            fmt_fixed(baseline_value(robot, difficulty, "drake_iris_np_gcs", "path")),
            fmt_fixed(sr_percent(baseline_value(robot, difficulty, "drake_iris_np_gcs", "sr")), 1),
        ])
        values.extend([
            fmt_fixed(baseline_value(robot, difficulty, "ompl_prm", "build_s")),
            fmt_fixed(baseline_value(robot, difficulty, "ompl_prm", "query_s")),
            fmt_fixed(baseline_value(robot, difficulty, "ompl_prm", "path")),
            fmt_fixed(sr_percent(baseline_value(robot, difficulty, "ompl_prm", "sr")), 1),
        ])
        values.extend([
            fmt_fixed(rrt_item.get("query_median_s", rrt_item.get("query_mean_s"))),
            fmt_fixed(rrt_item.get("success_length_median", rrt_item.get("success_length_mean"))),
            fmt_fixed(sr_percent(rrt_item.get("audit_sr", rrt_item.get("sr"))), 1),
        ])
        values.extend([
            fmt_fixed(baseline_value(robot, difficulty, "ompl_bitstar", "query_s")),
            fmt_fixed(baseline_value(robot, difficulty, "ompl_bitstar", "path")),
            fmt_fixed(sr_percent(baseline_value(robot, difficulty, "ompl_bitstar", "sr")), 1),
        ])
        row_lines.append(" & ".join(values) + row_end)

    group_header = " & ".join(
        [" "] + [rf"\multicolumn{{{len(spec['columns'])}}}{{c}}{{\textbf{{{TABLE_HEADER_FONT} {spec['label']}}}}}" for spec in method_specs]
    )
    column_header = " & ".join(["Scene"] + [" & ".join(spec["columns"]) for spec in method_specs])
    cmidrules = []
    current_col = 2
    for spec in method_specs:
        width = len(spec["columns"])
        cmidrules.append(rf"\cmidrule(lr){{{current_col}-{current_col + width - 1}}}")
        current_col += width
    colspec = "@{}l" + "|".join("r" * len(spec["columns"]) for spec in method_specs) + "@{}"

    return "\n".join([
        "% Auto-generated from current Exp.5 random-scene artifacts only.",
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{UR5/Panda random-obstacle generalization under current-version experiment artifacts. SBF-SH reports cold per-scene builds and charged online queries. Drake IRIS-NP+GCS reconstructs the same current parametric random-scene robot geometry and charges guide-seed generation to build time. OMPL PRM uses cumulative shared-roadmap build/query accounting. OMPL RRTConnect uses one max-timeout query attempt per trial and returns on connection, while BIT* uses one fixed-timeout invocation per trial with an audited monotone incumbent checkpoint trace. OMPL planning, final simplify, and fixed-resolution final audit all use the same 0.01 joint-space segment step. SR is reported in percent.}",
        r"\label{tab:tro_appendix_generalization}",
        TABLE_FONT_DENSE,
        r"\setlength{\tabcolsep}{1.2pt}",
        r"\resizebox{\textwidth}{!}{%",
        rf"\begin{{tabular}}{{{colspec}}}",
        r"\toprule",
        group_header + row_end,
        "".join(cmidrules),
        column_header + row_end,
        r"\midrule",
        "% --- DATA BEGIN ---",
        *row_lines,
        "% --- DATA END ---",
        r"\bottomrule",
        r"\end{tabular}}",
        r"\end{table*}",
        "",
    ])


def random_baseline_current_stats(outputs: Path) -> dict[tuple[str, str, str], dict[str, Any]]:
    payloads = [payload for payload in [
        load_json(outputs / "tro2026_exp05_random_ompl_full.json") or load_json(outputs / "tro2026_exp05_random_all_baselines.json"),
        load_json(outputs / "tro2026_exp05_random_iris_np_gcs_full.json"),
    ] if payload]
    stats: dict[tuple[str, str, str], dict[str, Any]] = {}
    if not payloads:
        return stats

    def metric(summary: dict[str, Any], key: str) -> Any:
        values = summary.get(key) or {}
        if not isinstance(values, dict):
            return None
        return values.get("median", values.get("mean"))

    for payload in payloads:
        groups = (payload.get("aggregation") or {}).get("groups", [])
        for group in groups:
            robot = str(group.get("robot", ""))
            difficulty = str(group.get("difficulty", ""))
            for method, summary in (group.get("methods") or {}).items():
                if method not in {"ompl_prm", "ompl_bitstar", "drake_iris_np_gcs"}:
                    continue
                stats[(robot, difficulty, method)] = {
                    "build_s": metric(summary, "build_time_s"),
                    "query_s": metric(summary, "query_time_s"),
                    "path": metric(summary, "path_length"),
                    "sr": summary.get("audit_success_rate", summary.get("success_rate")),
                    "n": summary.get("n_runs"),
                }
    return stats


def scenario_sort_key(label: str) -> tuple[int, int, str]:
    if label == "Shelf+IIWA":
        return (0, 0, label)
    robot, _, difficulty = label.partition(":")
    return (
        {"iiwa": 1, "ur5": 2, "panda": 3}.get(robot, 9),
        {"easy": 0, "medium": 1, "hard": 2}.get(difficulty, 9),
        label,
    )


def scenario_family_key(label: str) -> str:
    if label == "Shelf+IIWA":
        return label
    robot, _, _ = label.partition(":")
    return robot


def scenario_family_caption(label: str) -> str:
    if label == "Shelf+IIWA":
        return "Shelf+IIWA"
    return f"{scenario_label_from_key(label + ':easy').split('-')[0]} random scenes"


def difficulty_label(label: str) -> str:
    if label == "Shelf+IIWA":
        return "Shelf"
    _, _, difficulty = label.partition(":")
    return difficulty.capitalize()


def best_tradeoff_cell(point: dict[str, Any] | None) -> str:
    if point is None:
        return "--"
    stage = tex_escape(point.get("stage_id", "--"))
    total_s = fmt_fixed(point.get("total_s"), 2)
    sr = 100.0 * safe_float(point.get("audit_sr"), 0.0)
    metric = str(point.get("best_metric", ""))
    if metric in {"path_100", "quality_weighted_path_100"}:
        return rf"\shortstack{{{stage}\\{total_s}\,s\\L={fmt_fixed(point.get('path_length'), 2)}}}"
    return rf"\shortstack{{{stage}\\{total_s}\,s\\SR={fmt_fixed(sr, 0)}\%}}"


BEST_TRADEOFF_TABLE_METHOD_ORDER = ["sbf", "drake_iris_np_gcs", "ompl_prm", "ompl_rrtconnect", "ompl_bitstar"]
BEST_TRADEOFF_TABLE_LABELS = {
    "sbf": "SBF-SH",
    "drake_iris_np_gcs": "IRIS-NP+GCS",
    "ompl_prm": "PRM",
    "ompl_rrtconnect": "RRTConnect",
    "ompl_bitstar": "BIT*",
}
MAIN_TRADEOFF_TABLE_COLUMNS_BY_METHOD = {
    "sbf": ["Build (s)", "Query (s)", "Path"],
    "drake_iris_np_gcs": ["Build (s)", "Query (s)", "Path"],
    "ompl_prm": ["Build (s)", "Query (s)", "Path"],
    "ompl_rrtconnect": ["Query (s)", "Path"],
    "ompl_bitstar": ["Query (s)", "Path"],
}
BEST_TRADEOFF_TABLE_COLUMNS_BY_METHOD = {
    "sbf": ["Build (s)", "Query (s)", "Path", "SR"],
    "drake_iris_np_gcs": ["Build (s)", "Query (s)", "Path", "SR"],
    "ompl_prm": ["Build (s)", "Query (s)", "Path", "SR"],
    "ompl_rrtconnect": ["Query (s)", "Path", "SR"],
    "ompl_bitstar": ["Query (s)", "Path", "SR"],
}


def format_method_header(label: str) -> str:
    stripped = label.lstrip()
    return label if stripped.startswith(r"\shortstack") else rf"\shortstack{{{label}}}"


def best_tradeoff_placeholder_span(columns_by_method: dict[str, list[str]] | None = None) -> int:
    columns = columns_by_method or BEST_TRADEOFF_TABLE_COLUMNS_BY_METHOD
    return 1 + sum(len(columns[method]) for method in BEST_TRADEOFF_TABLE_METHOD_ORDER)


def best_tradeoff_grouped_row_lines(outputs: Path, scenario_keys: list[str], columns_by_method: dict[str, list[str]] | None = None) -> list[str]:
    best = best_tradeoff_points(outputs)
    by_scenario_method = {(str(row.get("scenario_key")), str(row.get("normalized_method"))): row for row in best}
    columns = columns_by_method or BEST_TRADEOFF_TABLE_COLUMNS_BY_METHOD
    row_lines: list[str] = []
    row_end = " " + chr(92) * 2
    for scenario_key in scenario_keys:
        values = [tex_escape(scenario_label_from_key(scenario_key))]
        for method in BEST_TRADEOFF_TABLE_METHOD_ORDER:
            point = by_scenario_method.get((scenario_key, method))
            method_columns = columns[method]
            if point is None:
                values.extend(["--"] * len(method_columns))
                continue
            timing = {
                "Build": fmt_main_time(point.get("build_s")),
                "Query": fmt_main_time(point.get("query_s")),
                "Total": fmt_main_time(point.get("total_s")),
                "Path": fmt_fixed(point.get("path_length"), 2) if point.get("path_length") is not None else "--",
                "Build (s)": fmt_main_time(point.get("build_s")),
                "Query (s)": fmt_main_time(point.get("query_s")),
                "Total (s)": fmt_main_time(point.get("total_s")),
                "Path (rad)": fmt_fixed(point.get("path_length"), 2) if point.get("path_length") is not None else "--",
                "SR": fmt_fixed(100.0 * safe_float(point.get("audit_sr"), 0.0), 0),
            }
            values.extend([timing[column] for column in method_columns])
        row_lines.append(" & ".join(values) + row_end)
    if not row_lines:
        row_lines.append(r"\multicolumn{" + str(best_tradeoff_placeholder_span(columns)) + r"}{c}{No result artifact available.} \\")
    return row_lines


def shelf_query_rows_from_best(outputs: Path, columns_by_method: dict[str, list[str]] | None = None) -> list[str]:
    best = [row for row in best_tradeoff_points(outputs) if str(row.get("scenario_key")) == "Shelf+IIWA"]
    by_method = {str(row.get("normalized_method")): row for row in best}
    columns = columns_by_method or BEST_TRADEOFF_TABLE_COLUMNS_BY_METHOD
    payload_cache = {
        "shelf": load_json(outputs / "tro2026_shelf_anytime_tradeoff_full.json") or {},
        "drake": load_json(outputs / "tro2026_shelf_iris_np_gcs_anytime.json") or {},
        "drake_reference": load_json(outputs / "marcucci_iris_np_gcs.json") or {},
    }

    def records_for(point: dict[str, Any] | None) -> list[dict[str, Any]]:
        if point is None:
            return []
        source = payload_cache["drake"] if str(point.get("normalized_method")) == "drake_iris_np_gcs" else payload_cache["shelf"]
        records = source.get("records", []) if isinstance(source, dict) else []
        indices = point.get("raw_record_indices") or []
        out = []
        for raw_index in indices:
            index = int(raw_index)
            if 0 <= index < len(records) and isinstance(records[index], dict):
                out.append(records[index])
        return out

    def selected_tasks(record: dict[str, Any]) -> list[dict[str, Any]]:
        if str(record.get("protocol")) == "single_run_max_timeout":
            return [task for task in record.get("raw_tasks", []) if isinstance(task, dict)]
        tasks = record.get("incumbent_tasks") or record.get("raw_tasks") or []
        return [task for task in tasks if isinstance(task, dict)]

    def query_stats(tasks: list[dict[str, Any]]) -> dict[str, Any]:
        successes = [task for task in tasks if bool(task.get("audit_passed")) and task.get("path_length") is not None]
        return {
            "audit_passed": bool(successes),
            "sr": float(len(successes)) / max(1, len(tasks)),
            "query_s": mean(task.get("query_s") for task in successes),
            "path_length": mean(task.get("path_length") for task in successes),
            "success_count": len(successes),
            "trial_count": len(tasks),
        }

    task_maps: dict[str, dict[str, dict[str, Any]]] = {}
    for method in BEST_TRADEOFF_TABLE_METHOD_ORDER:
        point = by_method.get(method)
        if method == "drake_iris_np_gcs" and point is not None and str(point.get("stage_id")) == "reference":
            queries = (payload_cache["drake_reference"] or {}).get("queries") or []
            task_maps[method] = {
                str(task.get("name")): {
                    "audit_passed": safe_float(task.get("sr"), 0.0) >= FULL_SUCCESS_THRESHOLD,
                    "sr": safe_float(task.get("sr"), 0.0),
                    "query_s": task.get("t_med_s", task.get("t_mean_s")),
                    "path_length": task.get("len_med", task.get("len_mean")),
                }
                for task in queries
                if isinstance(task, dict)
            }
            continue
        grouped_tasks: dict[str, list[dict[str, Any]]] = {}
        for record in records_for(point):
            for task in selected_tasks(record):
                grouped_tasks.setdefault(str(task.get("name")), []).append(task)
        task_maps[method] = {name: query_stats(tasks) for name, tasks in grouped_tasks.items()}

    row_lines: list[str] = []
    row_end = " " + chr(92) * 2
    for query_name in canonical_shelf_queries():
        values = [query_label(query_name)]
        for method in BEST_TRADEOFF_TABLE_METHOD_ORDER:
            point = by_method.get(method)
            task = task_maps.get(method, {}).get(query_name)
            method_columns = columns[method]
            if point is None or task is None:
                values.extend(["--"] * len(method_columns))
                continue
            success = bool(task.get("audit_passed"))
            query_s = safe_float(task.get("query_s"), 0.0) if success else None
            build_s = safe_float(point.get("build_s"), 0.0)
            total_s = None if query_s is None else build_s + query_s
            timing = {
                "Build": fmt_main_time(build_s),
                "Query": fmt_main_time(query_s) if success else "--",
                "Total": fmt_main_time(total_s) if success else "--",
                "Path": fmt_fixed(task.get("path_length"), 2) if success else "--",
                "Build (s)": fmt_main_time(build_s),
                "Query (s)": fmt_main_time(query_s) if success else "--",
                "5-query (s)": fmt_main_time(query_s) if success else "--",
                "Total (s)": fmt_main_time(total_s) if success else "--",
                "Path (rad)": fmt_fixed(task.get("path_length"), 2) if success else "--",
                "SR": fmt_fixed(100.0 * safe_float(task.get("sr"), 0.0), 0),
            }
            values.extend([timing[column] for column in method_columns])
        row_lines.append(" & ".join(values) + row_end)
    if not row_lines:
        row_lines.append(r"\multicolumn{" + str(best_tradeoff_placeholder_span(columns)) + r"}{c}{No result artifact available.} \\")
    return row_lines


def best_tradeoff_grouped_table(
    caption: str,
    label: str,
    row_lines: list[str],
    *,
    tabcolsep: float = 1.35,
    first_column: str = "Scenario",
    font_command: str = TABLE_FONT_DENSE,
    method_labels: dict[str, str] | None = None,
    columns_by_method: dict[str, list[str]] | None = None,
    float_spec: str = "[t]",
) -> str:
    labels = method_labels or BEST_TRADEOFF_TABLE_LABELS
    columns = columns_by_method or BEST_TRADEOFF_TABLE_COLUMNS_BY_METHOD
    row_end = " " + chr(92) * 2
    group_header = " & ".join(
        [" "] + [
            rf"\multicolumn{{{len(columns[method])}}}{{c}}{{\textbf{{{TABLE_HEADER_FONT} {format_method_header(labels[method])}}}}}"
            for method in BEST_TRADEOFF_TABLE_METHOD_ORDER
        ]
    )
    column_header = " & ".join([first_column] + [" & ".join(columns[method]) for method in BEST_TRADEOFF_TABLE_METHOD_ORDER])
    cmidrules = []
    current_col = 2
    for method in BEST_TRADEOFF_TABLE_METHOD_ORDER:
        width = len(columns[method])
        cmidrules.append(rf"\cmidrule(lr){{{current_col}-{current_col + width - 1}}}")
        current_col += width
    colspec = "@{}l" + "|".join("r" * len(columns[method]) for method in BEST_TRADEOFF_TABLE_METHOD_ORDER) + "@{}"
    return "\n".join([
        "% Auto-generated from current anytime trade-off artifacts.",
        rf"\begin{{table*}}{float_spec}",
        r"\centering",
        r"\caption{" + caption + r"}",
        r"\label{" + label + r"}",
        font_command,
        rf"\setlength{{\tabcolsep}}{{{tabcolsep:.2f}pt}}",
        r"\renewcommand{\arraystretch}{0.96}",
        rf"\begin{{tabular}}{{{colspec}}}",
        r"\toprule",
        group_header + row_end,
        "".join(cmidrules),
        column_header + row_end,
        r"\midrule",
        *row_lines,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table*}",
        "",
    ])


def main_shelf_best_tradeoff_table(outputs: Path) -> str:
    columns_by_method = {
        method: ["Query (s)", "Path"]
        for method in BEST_TRADEOFF_TABLE_METHOD_ORDER
    }
    best = [row for row in best_tradeoff_points(outputs) if str(row.get("scenario_key")) == "Shelf+IIWA"]
    by_method = {str(row.get("normalized_method")): row for row in best}
    method_labels = dict(BEST_TRADEOFF_TABLE_LABELS)
    for method in ["sbf", "drake_iris_np_gcs", "ompl_prm"]:
        point = by_method.get(method)
        if point is not None:
            method_labels[method] = build_header(BEST_TRADEOFF_TABLE_LABELS[method], point.get("build_s"))
    rows = shelf_query_rows_from_best(outputs, columns_by_method=columns_by_method)
    table_text = best_tradeoff_grouped_table(
        r"Common-rule tabulated Shelf+IIWA rows from Fig.~\ref{fig:tro_shelf_anytime_tradeoff}, reported by query. Gold rings in Fig.~4 mark these rows only to expose detailed numeric values; the full curves remain the primary evidence. RRTConnect uses one max-timeout attempt per seed/query and returns on connection; BIT* uses one fixed-timeout invocation per seed/query with an audited monotone incumbent checkpoint trace. OMPL planning, final simplify, and fixed-resolution final audit use the same 0.01 joint-space segment step. Query and path entries are success-only averages over paths that pass this fixed-resolution audit.",
        "tab:tro_main_shelf_best_tradeoff",
        rows,
        tabcolsep=1.55,
        first_column="Query",
        method_labels=method_labels,
        columns_by_method=columns_by_method,
    )
    return (
        table_text
        .replace(r"\begin{table*}[t]", r"\begingroup", 1)
        .replace(r"\caption{", r"\captionof{table}{", 1)
        .replace(r"\end{table*}", r"\par\endgroup", 1)
    )


def main_random_best_tradeoff_table(outputs: Path) -> str:
    best = best_tradeoff_points(outputs)
    scenario_keys = sorted({str(row.get("scenario_key")) for row in best if str(row.get("scenario_key")) != "Shelf+IIWA"}, key=scenario_sort_key)
    rows = best_tradeoff_grouped_row_lines(outputs, scenario_keys, columns_by_method=MAIN_TRADEOFF_TABLE_COLUMNS_BY_METHOD)
    caption = r"Common-rule tabulated random-scene rows from Fig.~\ref{fig:tro_random_anytime_tradeoff}. Gold rings identify these rows only to expose detailed numeric values; the full curves remain the primary evidence. Each method and scenario is evaluated on its full curve, and the table lists one readable slice because the complete checkpoint data are too extensive to print in full. Among points within 8\% of the shortest audited path, the rule minimizes a single path/log-time utility. BIT* rows are taken from the audited monotone incumbent checkpoint trace under the same 0.01 OMPL planning/simplify and fixed-resolution final-audit segment step."
    columns = MAIN_TRADEOFF_TABLE_COLUMNS_BY_METHOD
    row_end = " " + chr(92) * 2
    group_header = " & ".join(
        [" "] + [
            rf"\multicolumn{{{len(columns[method])}}}{{c}}{{\textbf{{{TABLE_HEADER_FONT} {format_method_header(BEST_TRADEOFF_TABLE_LABELS[method])}}}}}"
            for method in BEST_TRADEOFF_TABLE_METHOD_ORDER
        ]
    )
    column_header = " & ".join(["Scenario"] + [" & ".join(columns[method]) for method in BEST_TRADEOFF_TABLE_METHOD_ORDER])
    cmidrules = []
    current_col = 2
    for method in BEST_TRADEOFF_TABLE_METHOD_ORDER:
        width = len(columns[method])
        cmidrules.append(rf"\cmidrule(lr){{{current_col}-{current_col + width - 1}}}")
        current_col += width
    colspec = "@{}l" + "|".join("r" * len(columns[method]) for method in BEST_TRADEOFF_TABLE_METHOD_ORDER) + "@{}"
    return "\n".join([
        "% Auto-generated from current anytime trade-off artifacts.",
        r"\begin{minipage}{\textwidth}",
        r"\centering",
        r"\refstepcounter{table}",
        r"\makeatletter\def\@currentlabelname{Common-rule tabulated random-scene rows}\makeatother",
        r"\addcontentsline{lot}{table}{\protect\numberline{\Roman{table}}Common-rule tabulated random-scene rows from Fig.~\ref{fig:tro_random_anytime_tradeoff}.}",
        r"\label{tab:tro_main_random_best_tradeoff}",
        r"{" + TABLE_CAPTION_FONT + r"\textsc{Table~\Roman{table}}\par}",
        r"{" + TABLE_CAPTION_FONT + " " + caption + r"\par}",
        r"\vspace{0.2em}",
        TABLE_FONT_DENSE,
        r"\setlength{\tabcolsep}{0.72pt}",
        r"\renewcommand{\arraystretch}{0.96}",
        rf"\begin{{tabular}}{{{colspec}}}",
        r"\toprule",
        group_header + row_end,
        "".join(cmidrules),
        column_header + row_end,
        r"\midrule",
        *rows,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{minipage}",
        "",
    ])


def appendix_anytime_full_table(outputs: Path) -> str:
    rows = collect_anytime_stage_points(outputs)
    rows = sorted(
        rows,
        key=lambda row: (
            scenario_sort_key(str(row.get("scenario_key"))),
            int(row.get("stage_index", 0)),
        ),
    )
    grouped_rows: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped_rows.setdefault(scenario_family_key(str(row.get("scenario_key"))), []).append(row)

    parts = ["% Auto-generated from current anytime trade-off artifacts."]
    family_order = ["Shelf+IIWA", "iiwa", "ur5", "panda"]
    for family_index, family_key in enumerate(family_order):
        family = grouped_rows.get(family_key, [])
        if family_key != "Shelf+IIWA" and not family:
            continue

        is_shelf = family_key == "Shelf+IIWA"
        caption = "Full anytime-stage data grouped by workload family. Path values are included for traceability and are not used for path-quality comparison when SR is below 100\\%."
        if family_index > 0:
            caption = f"Full anytime-stage data, continued ({scenario_family_caption(family_key)})."

        body: list[str] = []
        for row in family:
            cells: list[str] = []
            if not is_shelf:
                cells.append(tex_escape(difficulty_label(str(row.get("scenario_key", "")))))
            cells.extend([
                tex_escape(METHOD_CELL_LABELS.get(str(row.get("normalized_method")), str(row.get("method")))),
                tex_escape(row.get("stage_id")),
                fmt_fixed(row.get("build_s"), 3),
                fmt_fixed(row.get("query_s"), 3),
                fmt_fixed(row.get("total_s"), 3),
                fmt_fixed(100.0 * safe_float(row.get("audit_sr"), 0.0), 1),
                fmt_fixed(row.get("path_length"), 3),
                "yes" if bool(row.get("promoted")) else "no",
            ])
            body.append(" & ".join(cells) + r" \\")

        if not body:
            colspan = 8 if is_shelf else 9
            body.append(rf"\multicolumn{{{colspan}}}{{c}}{{No result artifact available.}} \\")

        colspec = r"@{}llrrrrrl@{}" if is_shelf else r"@{}lllrrrrrl@{}"
        header = (
            r"Method & Stage & Build (s) & Query (s) & Total (s) & SR (\%) & Path & Prom. \\")
        if not is_shelf:
            header = (
                r"Difficulty & Method & Stage & Build (s) & Query (s) & Total (s) & SR (\%) & Path & Prom. \\")

        parts.extend([
            r"\begin{table*}[t]",
            r"\centering",
            (r"\addtocounter{table}{-1}" if family_index > 0 else r""),
            r"\caption{" + caption + r"}",
            (r"\label{tab:tro_appendix_anytime_full}" if family_index == 0 else r""),
            TABLE_FONT_DENSE,
            (r"\setlength{\tabcolsep}{1.45pt}" if is_shelf else r"\setlength{\tabcolsep}{1.55pt}"),
            r"\renewcommand{\arraystretch}{0.94}",
            r"\resizebox{\textwidth}{!}{%",
            r"\begin{tabular}{" + colspec + r"}",
            r"\toprule",
            header,
            r"\midrule",
            *body,
            r"\bottomrule",
            r"\end{tabular}}",
            r"\end{table*}",
            "",
        ])
    return "\n".join(parts)


def implementation_optimization_table(outputs: Path) -> str:
    payload = load_json(outputs / "tro2026_exp17_implementation_optimization_plan.json")
    rows: list[list[Any]] = []
    if payload:
        priority_order = {"P0": 0, "P1": 1, "P2": 2, "P3": 3}
        items = sorted(
            payload.get("rows", []),
            key=lambda row: (priority_order.get(str(row.get("priority")), 99), str(row.get("package")), str(row.get("id"))),
        )
        for row in items:
            rows.append([
                row.get("package"),
                row.get("id"),
                row.get("priority"),
                row.get("target"),
                row.get("artifact"),
                "yes" if row.get("artifact_exists") else "no",
                row.get("primary_metric"),
                row.get("validation_gate"),
            ])
    return table(
        "Implementation-optimization matrix for LECT, link-interval envelopes, and SBF. The artifact column lists the JSON file that must validate each optimization before the result is used in the paper.",
        "tab:tro_implementation_optimization_plan",
        "llllllll",
        ["Pkg.", "ID", "Prio.", "Target", "Artifact", "Ready", "Metric", "Gate"],
        rows,
    )


def macros(outputs: Path) -> str:
    audited = load_json(outputs / "tro2026_exp07_gcs_full.json") or load_json(outputs / "marcucci_audited_corridor_gcs.json")
    direct = load_json(outputs / "marcucci_merger_gcs.json")
    marcucci = load_json(outputs / "tro2026_exp04_marcucci_full.json") or load_json(outputs / "tro2026_exp04_marcucci_support_hull_full.json") or load_json(outputs / "marcucci_corridor_refine_selfedge_s10.json")
    suite = load_json(outputs / "tro2026_safety_accounting_full.json") or load_json(outputs / "paper_soundness_audit_suite.json")
    lines = [
        "% Auto-generated by experiments/tro2026_generate_tables.py",
        r"\newcommand{\TroAuditedGcsAuditPass}{--/--}",
        r"\newcommand{\TroExpandedGcsAuditPass}{--/--}",
        r"\newcommand{\TroDirectGcsAuditPass}{--/--}",
        r"\newcommand{\TroSbfBuildMean}{--}",
        r"\newcommand{\TroSbfBoxMean}{--}",
        r"\newcommand{\TroPaperAuditPass}{--/--}",
    ]
    if audited:
        summary = audited.get("summary", {})
        lines.append(r"\renewcommand{\TroAuditedGcsAuditPass}{" + fmt(summary.get("gcs_strict_audit_pass_count"), 0) + r"/" + fmt(summary.get("query_count"), 0) + r"}")
        expanded_pass = 0
        expanded_solved = 0
        for row in audited.get("queries", []):
            for attempt in row.get("gcs_attempts", []):
                if attempt.get("attempt") == "overlap_expanded_sbf_corridor":
                    expanded_solved += 1 if attempt.get("ok") else 0
                    expanded_pass += 1 if attempt.get("strict_audit_passed") else 0
        lines.append(r"\renewcommand{\TroExpandedGcsAuditPass}{" + fmt(expanded_pass, 0) + r"/" + fmt(expanded_solved, 0) + r"}")
    if direct:
        pass_count = sum(1 for row in direct.get("gcs_queries", []) if row.get("audit", {}).get("passed"))
        solved = sum(1 for row in direct.get("gcs_queries", []) if row.get("ok"))
        lines.append(r"\renewcommand{\TroDirectGcsAuditPass}{" + fmt(pass_count, 0) + r"/" + fmt(solved, 0) + r"}")
    if marcucci:
        build = marcucci.get("build", {})
        lines.append(r"\renewcommand{\TroSbfBuildMean}{" + fmt(build.get("mean_s")) + r"}")
        lines.append(r"\renewcommand{\TroSbfBoxMean}{" + fmt(build.get("mean_unique_box_count"), 1) + r"}")
    if suite:
        summary = suite.get("summary", {})
        lines.append(r"\renewcommand{\TroPaperAuditPass}{" + str(int(summary.get("audit_pass_count", 0))) + r"/" + str(int(summary.get("path_count", 0))) + r"}")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    outputs = args.outputs
    out_dir = args.out_dir
    main_writers = {
        "tab_tro_main_lect_cache_footprint.tex": main_lect_cache_footprint_table(outputs),
        "tab_tro_main_evidence_validation.tex": main_evidence_validation_table(outputs),
        "tab_tro_main_lect_reuse.tex": main_lect_reuse_table(outputs),
        "tab_tro_main_shelf_best_tradeoff.tex": main_shelf_best_tradeoff_table(outputs),
        "tab_tro_main_random_best_tradeoff.tex": main_random_best_tradeoff_table(outputs),
        "tab_tro_link_envelope_widthwise.tex": link_representation_widthwise_table(outputs),
        "tab_tro_dynamic_rebuild.tex": rebuild_table(outputs),
        "text_tro_safety_fallback.tex": safety_fallback_text(outputs),
        "tro_macros.tex": macros(outputs),
    }
    if args.placeholder:
        main_writers = {
            "tab_tro_main_lect_cache_footprint.tex": main_lect_cache_footprint_table(outputs),
            "tab_tro_main_evidence_validation.tex": placeholder_table(
                "Shelf+IIWA evidence-profile efficiency and validation boundary over the five canonical queries.",
                "tab:tro_main_evidence_validation",
                "llrrrrrr",
                ["Source", "Envelope", "Boundary", "Build", "Boxes", "Query", "Repair", "Final SR"],
            ),
            "tab_tro_main_lect_reuse.tex": placeholder_table(
                "LECT evidence reuse on the Shelf+IIWA protocol.",
                "tab:tro_main_lect_reuse",
                "lllllllll",
                ["Pipeline", "Cold", "Warm", "Cross", "Warm speedup", "Cross speedup", "Cache MB", "Boxes", "SR same/cross"],
            ),
            "tab_tro_main_shelf_best_tradeoff.tex": placeholder_table(
                "Best Shelf+IIWA trade-off points selected from the full-success curve.",
                "tab:tro_main_shelf_best_tradeoff",
                "lrrrrrrrrrrrrrrrrrrrr",
                ["Scenario", "SBF Build", "SBF Query", "SBF Total", "SBF Path", "IRIS Build", "IRIS Query", "IRIS Total", "IRIS Path", "PRM Build", "PRM Query", "PRM Total", "PRM Path", "RRT Build", "RRT Query", "RRT Total", "RRT Path", "BIT* Build", "BIT* Query", "BIT* Total", "BIT* Path"],
            ),
            "tab_tro_main_random_best_tradeoff.tex": placeholder_table(
                "Best random-scene trade-off points selected from the full-success panels.",
                "tab:tro_main_random_best_tradeoff",
                "lrrrrrrrrrrrrrrrrrrrr",
                ["Scenario", "SBF Build", "SBF Query", "SBF Total", "SBF Path", "IRIS Build", "IRIS Query", "IRIS Total", "IRIS Path", "PRM Build", "PRM Query", "PRM Total", "PRM Path", "RRT Build", "RRT Query", "RRT Total", "RRT Path", "BIT* Build", "BIT* Query", "BIT* Total", "BIT* Path"],
            ),
            "tab_tro_link_envelope_widthwise.tex": link_representation_widthwise_table(outputs),
            "tab_tro_dynamic_rebuild.tex": rebuild_table(outputs),
            "text_tro_safety_fallback.tex": safety_fallback_text(outputs),
            "tro_macros.tex": macros(outputs),
        }
    appendix_writers: dict[str, str] = {
        "tab_tro_experiment_matrix.tex": experiment_matrix_table(),
        "tab_tro_link_envelopes.tex": link_representation_widthwise_table(outputs),
        "tab_tro_grower_tradeoff.tex": grower_table(outputs),
        "tab_tro_audited_gcs.tex": gcs_table(outputs),
        "tab_tro_merger_control.tex": merger_table(outputs),
        "tab_tro_random_scenes.tex": random_scene_table(outputs),
        "tab_tro_random_rrt_baseline.tex": random_rrt_baseline_table(outputs),
        "tab_tro_parallel_scaling.tex": parallel_table(outputs),
        "tab_tro_mechanism_diagnostics.tex": mechanism_diagnostics_table(outputs),
        "tab_tro_soundness_audit.tex": audit_suite_table(outputs),
    }
    if args.mode == "main":
        writers = main_writers
    elif args.mode == "appendix":
        writers = appendix_writers
    else:
        writers = {**appendix_writers, **main_writers}
    missing = sorted(name for name, content in writers.items() if table_has_missing_artifact(content))
    manifest = {
        "mode": args.mode,
        "out_dir": str(out_dir),
        "generated": sorted(writers),
        "missing_artifact_tables": missing,
    }
    if args.strict_missing and missing:
        raise SystemExit("Missing result artifacts for: " + ", ".join(missing))
    for name, content in writers.items():
        write(out_dir / name, content)
    manifest_path = args.manifest or out_dir / "tro_table_generation_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    if not args.placeholder:
        write_evidence_validation_tradeoff_plot(outputs, out_dir)
        write_marcucci_baseline_plot(outputs, out_dir)
        write_sbf_time_quality_tradeoff_plot(outputs, out_dir)
        write_shelf_anytime_tradeoff_plot(outputs, out_dir)
        write_random_anytime_tradeoff_plot(outputs, out_dir)
        write_random_hard_success_plot(outputs, out_dir)
    for obsolete_name in [
        "tab_tro_marcucci_e2e.tex",
        "tab_tro_query_amortization.tex",
        "tab_tro_main_safety_gcs.tex",
        "tab_tro_main_systems_summary.tex",
        "tab_tro_main_shelf_benchmark.tex",
        "tab_tro_main_generalization.tex",
        "tab_tro_main_best_tradeoff.tex",
        "tab_tro_main_random_profile_selection.tex",
        "tab_tro_appendix_shelf_benchmark.tex",
        "tab_tro_appendix_generalization.tex",
        "tab_tro_endpoint_sources.tex",
        "tab_query.tex",
        "tab_tro_appendix_anytime_full.tex",
        "tab_tro_lect_reuse.tex",
        "tab_tro_ffb_depth_sweep.tex",
        "tab_tro_marcucci_envelope_variants.tex",
        "tab_tro_validation_profiles.tex",
        "tab_tro_implementation_optimization_plan.tex",
        "fig_tro_link_representation_tradeoff.pdf",
        "tro_experiment_section.tex",
    ]:
        obsolete = out_dir / obsolete_name
        if obsolete.exists():
            obsolete.unlink()
    print(json.dumps({"out_dir": str(out_dir), "files": sorted(writers), "missing_artifact_tables": missing, "manifest": str(manifest_path)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())