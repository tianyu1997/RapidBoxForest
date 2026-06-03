#!/usr/bin/env python3
"""Generate paper-facing Exp.4-6 tables and trade-off figures.

The current manuscript uses reduced, current-state artifacts rather than the
older full-matrix paper artifacts. This script keeps the presentation coherent:
Exp.4 is shown as a leaf-refine trade-off, while Exp.6 is shown as an anytime
curve with a separate fast design-point table by difficulty.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: E402
except ModuleNotFoundError:
    matplotlib = None
    plt = None


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "paper" / "sbf_old" / "generated"
EXP4 = ROOT / "outputs" / "new_experiments" / "exp04_leaf_refine_ablation_opt_box200_full_20260603"
EXP6_SBF = ROOT / "outputs" / "new_experiments" / "exp06_random_robot_sbf_rrt_s1_20260603" / "random_anytime_tradeoff.json"
EXP6_BASE = ROOT / "outputs" / "new_experiments" / "exp06_random_robot_baselines_s1_20260603"
EXP6_IRIS = ROOT / "outputs" / "new_experiments" / "exp06_iris_reduced_s1_r5_20260603" / "iris_np_gcs_random_anytime.json"
EXP5_SHELF = ROOT / "outputs" / "new_experiments" / "exp05_shelf_cross_algorithm_split_s3_20260603"
EXP5_SBF = EXP5_SHELF / "sbf_leaf_refine_d23_box200" / "leaf_refine_tradeoff_summary.json"
IRIS_PREFERRED_STAGE = "r20"


DIFFICULTIES = ("easy", "medium", "hard")
ROBOTS = ("iiwa", "ur5", "panda")
METHOD_STYLE = {
    "sbf": {"label": "SBF stages", "color": "#1f77b4", "marker": "o"},
    "prm": {"label": "PRM", "color": "#2ca02c", "marker": "s"},
    "bitstar": {"label": "BIT*", "color": "#9467bd", "marker": "^"},
    "rrtconnect": {"label": "RRTConnect", "color": "#d62728", "marker": "x"},
    "iris": {"label": "IRIS-NP+GCS", "color": "#ff7f0e", "marker": "D"},
}


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def infer_best_iris_stage(path: Path, preferred: str | None = None) -> str:
    """Pick the highest-available IRIS stage id, preferring a configured budget.

    This keeps the table/figure generation robust when the current artifact changes
    from r5 to r20-like budgets.
    """
    data = load_json(path)
    stage_ids = {
        str(row.get("stage_id"))
        for panel in data.get("panels", {}).values()
        for row in panel.get("records", [])
        if row.get("stage_id") is not None
    }
    if not stage_ids:
        return preferred if preferred else "r5"
    preferred = preferred.strip() if isinstance(preferred, str) else None
    if preferred and preferred in stage_ids:
        return preferred
    ranked: list[tuple[int, str]] = []
    for stage_id in stage_ids:
        sid = str(stage_id)
        if sid.startswith("r") and sid[1:].isdigit():
            ranked.append((int(sid[1:]), sid))
    if not ranked:
        return sorted(stage_ids)[-1]
    ranked.sort()
    if preferred and preferred.startswith("r") and preferred[1:].isdigit():
        target = int(preferred[1:])
        for value, sid in reversed(ranked):
            if value <= target:
                return sid
        return ranked[-1][1]
    return ranked[-1][1]


IRIS_REPORTED_STAGE = infer_best_iris_stage(EXP6_IRIS, preferred=IRIS_PREFERRED_STAGE)


def median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else float("nan")


def panel_key(robot: str, difficulty: str) -> str:
    return f"{robot}:{difficulty}"


def records(path: Path, robot: str, difficulty: str) -> list[dict[str, Any]]:
    data = load_json(path)
    return list(data["panels"][panel_key(robot, difficulty)]["records"])


def successful_time_path(record: dict[str, Any]) -> tuple[float, float] | None:
    if int(record.get("incumbent_success_count", 0) or 0) <= 0:
        return None
    time_s = record.get("cumulative_total_s")
    length = record.get("incumbent_mean_length")
    if time_s is None or length is None:
        return None
    return float(time_s), float(length)


def chosen_record(path: Path, robot: str, difficulty: str, stage: str | None = None) -> dict[str, Any]:
    rows = records(path, robot, difficulty)
    if stage is not None:
        rows = [row for row in rows if str(row.get("stage_id")) == stage]
    if not rows:
        raise RuntimeError(f"missing {path} {robot}:{difficulty} stage={stage}")
    return rows[-1]


def format_float(value: float, digits: int = 3) -> str:
    if not math.isfinite(value):
        return "--"
    if abs(value) < 0.1:
        return f"{value:.4f}"
    return f"{value:.{digits}f}"


def generate_exp4_figure() -> None:
    if plt is None:
        return
    rows: list[dict[str, Any]] = []
    with (EXP4 / "leaf_refine_ablation_summary.csv").open("r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append(row)

    ok_rows = [row for row in rows if int(row["ok_count"]) > 0 and row["route_length_median"]]
    names = [row["name"] for row in ok_rows]
    short = {
        "baseline_d23_sh_8t_leaf8_14_box200_d28": "d23 box200",
        "baseline_d23_sh_8t_leaf8_14_box400_d28": "d23 box400",
        "baseline_d23_sh_8t_leaf8_14_box500_d24": "d23 box500/d24",
        "nocache_sh_8t_leaf8_14_box200_d28": "no-cache\nSH",
        "nocache_aabb_8t_leaf8_14_box200_d28": "no-cache\nAABB",
        "nocache_sh_1t_leaf8_14_box200_d28": "1t",
        "nocache_sh_8t_serial_leaf_box200_d28": "serial",
    }
    labels = [short.get(name, name) for name in names]
    plan = [float(row["planning_ms_median"]) for row in ok_rows]
    length = [float(row["route_length_median"]) for row in ok_rows]
    segment = [float(row["segment_fraction_median"]) for row in ok_rows]
    leaf = [float(row["leaf_sweep_ms_median"]) for row in ok_rows]
    refine = [float(row["deep_refine_ms_median"]) for row in ok_rows]
    conn = [float(row["connector_ms_median"]) for row in ok_rows]

    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.5), constrained_layout=True)
    ax = axes[0]
    ax.scatter(plan, length, s=45, color="#1f77b4")
    offsets = {
        "1t": (-22, 8),
        "serial": (4, 10),
        "no-cache\nSH": (4, 8),
        "no-cache\nAABB": (4, -12),
    }
    for x, y, label in zip(plan, length, labels):
        if label in {"1t", "serial"}:
            continue
        ax.annotate(label, (x, y), xytext=offsets.get(label, (4, 4)),
                    textcoords="offset points", fontsize=7)
    ax.set_xlabel("planning time (ms)")
    ax.set_ylabel("audited path length")
    ax.set_title("(a) time / path trade-off")
    ax.grid(True, alpha=0.25)

    ax = axes[1]
    xs = list(range(len(labels)))
    ax.bar(xs, leaf, label="leaf", color="#9ecae1")
    ax.bar(xs, refine, bottom=leaf, label="refine", color="#3182bd")
    ax.bar(xs, conn, bottom=[a + b for a, b in zip(leaf, refine)], label="connector", color="#08519c")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=7)
    ax.set_ylabel("stage time (ms)")
    ax.set_title("(b) time breakdown")
    ax.legend(fontsize=7, frameon=False)
    ax.grid(True, axis="y", alpha=0.25)

    ax = axes[2]
    ax.scatter(plan, segment, s=45, color="#d62728")
    for x, y, label in zip(plan, segment, labels):
        if label in {"1t", "serial"}:
            continue
        ax.annotate(label, (x, y), xytext=offsets.get(label, (4, 4)),
                    textcoords="offset points", fontsize=7)
    ax.set_xlabel("planning time (ms)")
    ax.set_ylabel("raw segment fraction")
    ax.set_ylim(bottom=-0.01)
    ax.set_title("(c) segment fallback")
    ax.grid(True, alpha=0.25)

    for suffix in ("pdf", "png"):
        fig.savefig(OUT / f"fig_tro_leaf_refine_tradeoff.{suffix}", dpi=220)
    plt.close(fig)


def generate_random_figure() -> None:
    if plt is None:
        return
    prm_path = EXP6_BASE / "random_prm_anytime.json"
    bit_path = EXP6_BASE / "random_bitstar_anytime.json"
    rrt_path = EXP6_BASE / "random_rrtconnect_anytime.json"

    fig, axes = plt.subplots(1, 3, figsize=(11.5, 4.15), sharey=False)
    for ax, difficulty in zip(axes, DIFFICULTIES):
        for robot_index, robot in enumerate(ROBOTS):
            jitter = 1.0 + (robot_index - 1) * 0.015

            sbf_points = [
                successful_time_path(row)
                for row in records(EXP6_SBF, robot, difficulty)
                if str(row.get("method", "")).startswith("sbf_")
            ]
            sbf_points = [point for point in sbf_points if point is not None]
            if sbf_points:
                xs = [p[0] * jitter for p in sbf_points]
                ys = [p[1] for p in sbf_points]
                ax.plot(xs, ys, "-", color=METHOD_STYLE["sbf"]["color"], alpha=0.45, linewidth=1.2)
                ax.scatter(xs, ys, marker=METHOD_STYLE["sbf"]["marker"], s=22,
                           color=METHOD_STYLE["sbf"]["color"], alpha=0.7)

            for key, path, stage, size in [
                ("prm", prm_path, None, 24),
                ("bitstar", bit_path, None, 24),
                ("rrtconnect", rrt_path, "timeout10s", 32),
                ("iris", EXP6_IRIS, IRIS_REPORTED_STAGE, 30),
            ]:
                rows = records(path, robot, difficulty)
                if stage is not None:
                    rows = [row for row in rows if str(row.get("stage_id")) == stage]
                points = [successful_time_path(row) for row in rows]
                points = [point for point in points if point is not None]
                if not points:
                    continue
                style = METHOD_STYLE[key]
                if len(points) > 1:
                    ax.plot([p[0] * jitter for p in points], [p[1] for p in points],
                            "-", color=style["color"], alpha=0.45, linewidth=1.0)
                ax.scatter([p[0] * jitter for p in points], [p[1] for p in points],
                           marker=style["marker"], s=size, color=style["color"], alpha=0.82)

            # Label each robot near the SBF seed point.
            if sbf_points:
                ax.annotate(robot.upper(), (sbf_points[0][0] * jitter, sbf_points[0][1]),
                            xytext=(3, 3), textcoords="offset points", fontsize=7,
                            color=METHOD_STYLE["sbf"]["color"])

        ax.set_xscale("log")
        ax.set_xlabel("charged time (s, log)")
        ax.set_title(difficulty.capitalize())
        ax.grid(True, which="both", alpha=0.22)
    axes[0].set_ylabel("audited path length")

    handles = []
    labels = []
    for key in ("sbf", "iris", "prm", "bitstar", "rrtconnect"):
        style = METHOD_STYLE[key]
        handle = plt.Line2D([0], [0], color=style["color"], marker=style["marker"],
                            linestyle="-" if key in {"sbf", "prm", "bitstar"} else "None",
                            markersize=5, linewidth=1.2)
        handles.append(handle)
        labels.append(style["label"])
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.995),
               ncol=5, frameon=False, fontsize=8)
    fig.subplots_adjust(left=0.065, right=0.99, top=0.83, bottom=0.15, wspace=0.12)
    for suffix in ("pdf", "png"):
        fig.savefig(OUT / f"fig_tro_random_reduced_tradeoff.{suffix}", dpi=220)
    plt.close(fig)


def aggregate_by_difficulty(path: Path, stage: str | None) -> dict[str, tuple[int, int, float, float]]:
    out: dict[str, tuple[int, int, float, float]] = {}
    for difficulty in DIFFICULTIES:
        times: list[float] = []
        lengths: list[float] = []
        success = 0
        total = 0
        for robot in ROBOTS:
            row = chosen_record(path, robot, difficulty, stage)
            total += int(row.get("task_count", 1) or 1)
            if int(row.get("incumbent_success_count", 0) or 0) > 0:
                success += int(row.get("incumbent_success_count", 0) or 0)
                times.append(float(row["cumulative_total_s"]))
                lengths.append(float(row["incumbent_mean_length"]))
        out[difficulty] = (success, total, median(times), median(lengths))
    return out


def write_random_tables() -> None:
    rows = [
        ("SBF seed stage", "seed", EXP6_SBF, "seed"),
        ("IRIS-NP+GCS", IRIS_REPORTED_STAGE, EXP6_IRIS, IRIS_REPORTED_STAGE),
        ("PRM", "build1s", EXP6_BASE / "random_prm_anytime.json", "build1s"),
        ("BIT*", "t2s", EXP6_BASE / "random_bitstar_anytime.json", "t2s"),
        ("RRTConnect", "10s", EXP6_BASE / "random_rrtconnect_anytime.json", "timeout10s"),
    ]
    lines = [
        "\\begin{table*}[t]",
        "\\centering",
        "\\caption{Reduced random-scene design-point summary by difficulty. Each difficulty aggregates one IIWA, one UR5, and one Panda task. SBF uses the first strict-audit-successful seed stage; later SBF stages are shown in Fig.~\\ref{fig:tro_random_reduced_tradeoff} as an anytime/quality curve rather than as the baseline row. Times follow each method's artifact accounting. Len. is the strict-audit-passing incumbent mean path length under the same per-query convention for all methods.}",
        "\\label{tab:tro_random_reduced}",
        "\\footnotesize",
        "\\setlength{\\tabcolsep}{3.0pt}",
        "\\begin{tabular}{llrr|rr|rr|rr}",
        "\\toprule",
        "Method & Stage & Audit & Median s & \\multicolumn{2}{c|}{Easy} & \\multicolumn{2}{c|}{Medium} & \\multicolumn{2}{c}{Hard} \\\\",
        " & & & & s & Len. & s & Len. & s & Len. \\\\",
        "\\midrule",
    ]
    for method, stage_label, path, stage in rows:
        agg = aggregate_by_difficulty(path, stage)
        success = sum(v[0] for v in agg.values())
        total = sum(v[1] for v in agg.values())
        all_times = [v[2] for v in agg.values() if math.isfinite(v[2])]
        med_time = median(all_times)
        cells = []
        for diff in DIFFICULTIES:
            _, _, t, l = agg[diff]
            cells.extend([format_float(t), format_float(l)])
        lines.append(
            f"{method} & {stage_label} & {success}/{total} & {format_float(med_time)} & "
            + " & ".join(cells)
            + " \\\\"
        )
    lines.extend(["\\bottomrule", "\\end{tabular}", "\\end{table*}", ""])
    (OUT / "tab_tro_random_reduced.tex").write_text("\n".join(lines), encoding="utf-8")

    # Separate SBF-stage diagnostic table for analysis notes, not currently
    # included in the paper main text.
    stage_rows = []
    for stage in ("seed", "fast", "balanced", "quality", "high"):
        times: list[float] = []
        lengths: list[float] = []
        improved = 0
        for difficulty in DIFFICULTIES:
            for robot in ROBOTS:
                row = chosen_record(EXP6_SBF, robot, difficulty, stage)
                if int(row.get("incumbent_success_count", 0) or 0) > 0:
                    times.append(float(row["cumulative_total_s"]))
                    lengths.append(float(row["incumbent_mean_length"]))
                if bool(row.get("improved")):
                    improved += 1
        stage_rows.append((stage, median(times), max(times), median(lengths), improved))
    stage_lines = [
        "\\begin{table}[t]",
        "\\centering",
        "\\caption{SBF random-scene cumulative stage diagnostic. The high stage is a curve endpoint, not the selected baseline row.}",
        "\\label{tab:tro_random_sbf_stage_diagnostic}",
        "\\footnotesize",
        "\\begin{tabular}{lrrrr}",
        "\\toprule",
        "Stage & Median s & Max s & Length & Improved panels \\\\",
        "\\midrule",
    ]
    for stage, med_t, max_t, med_l, improved in stage_rows:
        stage_lines.append(
            f"{stage} & {format_float(med_t)} & {format_float(max_t)} & {format_float(med_l)} & {improved}/9 \\\\"
        )
    stage_lines.extend(["\\bottomrule", "\\end{tabular}", "\\end{table}", ""])
    (OUT / "tab_tro_random_sbf_stage_diagnostic.tex").write_text("\n".join(stage_lines), encoding="utf-8")


def success_only_mean_path(lengths: list[float]) -> float:
    return sum(lengths) / len(lengths) if lengths else float("nan")


def shelf_sbf_stats(path: Path) -> tuple[int, int, float, float]:
    data = load_json(path)
    case = data["cases"][0]
    per_seed_time_s: list[float] = []
    per_seed_mean_length: list[float] = []
    success = 0
    total = 0
    for seed in case["seeds"]:
        per_seed_time_s.append(float(seed["total_ms"]) / 1000.0)
        lengths: list[float] = []
        for query in seed["queries"]:
            total += 1
            if bool(query.get("success")) and bool(query.get("audit_passed")):
                success += 1
                lengths.append(float(query["path_length"]))
        if lengths:
            per_seed_mean_length.append(success_only_mean_path(lengths))
    return success, total, median(per_seed_time_s), median(per_seed_mean_length)


def shelf_anytime_stats(path: Path, stage: str) -> tuple[int, int, float, float]:
    data = load_json(path)
    rows = [row for row in data["records"] if str(row.get("stage_id")) == stage]
    if not rows:
        raise RuntimeError(f"missing shelf anytime stage {stage} in {path}")
    per_seed_time_s: list[float] = []
    per_seed_mean_length: list[float] = []
    success = 0
    total = 0
    for row in rows:
        total += int(row.get("task_count", 0) or 0)
        success += int(row.get("incumbent_success_count", 0) or 0)
        per_seed_time_s.append(float(row["cumulative_total_s"]))
        lengths = [
            float(task["path_length"])
            for task in row.get("incumbent_tasks", [])
            if task.get("path_length") is not None
        ]
        if lengths:
            per_seed_mean_length.append(success_only_mean_path(lengths))
    return success, total, median(per_seed_time_s), median(per_seed_mean_length)


def write_shelf_cross_algorithm_table() -> None:
    rows = [
        ("SBF leaf-refine d23 box200", *shelf_sbf_stats(EXP5_SBF)),
        ("PRM shared roadmap, 5 s grid", *shelf_anytime_stats(EXP5_SHELF / "legacy_prm_anytime.json", "build5s")),
        ("BIT*, 5 s checkpoint", *shelf_anytime_stats(EXP5_SHELF / "legacy_bitstar_anytime.json", "t5s")),
        ("RRTConnect, 10 s timeout", *shelf_anytime_stats(EXP5_SHELF / "legacy_rrtconnect_anytime.json", "timeout10s")),
    ]
    lines = [
        "\\begin{table}[t]",
        "\\centering",
        "\\caption{Shelf+IIWA cross-algorithm sanity check over three seeds and five fixed queries. Times follow each method's artifact semantics: SBF reports median planning time excluding audit, PRM/BIT* report cumulative audited incumbent time, and RRTConnect reports query-only raw total time. Path length is now reported under a common success-only per-query mean: each seed first averages the final strict-audit-passing path lengths of its successful queries, then the table reports the median of those seed means.}",
        "\\label{tab:tro_shelf_cross_algorithm}",
        "\\footnotesize",
        "\\setlength{\\tabcolsep}{2.4pt}",
        "\\begin{tabular}{lrrr}",
        "\\toprule",
        "Method & Audit & Time s & Mean Len. \\\\",
        "\\midrule",
    ]
    for method, success, total, time_s, length in rows:
        lines.append(f"{method} & {success}/{total} & {format_float(time_s)} & {format_float(length)} \\\\")
    lines.extend(["\\bottomrule", "\\end{tabular}", "\\end{table}", ""])
    (OUT / "tab_tro_shelf_cross_algorithm.tex").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    generate_exp4_figure()
    write_shelf_cross_algorithm_table()
    generate_random_figure()
    write_random_tables()
    print(f"wrote paper assets to {OUT}")


if __name__ == "__main__":
    main()
