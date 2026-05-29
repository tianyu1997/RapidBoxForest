#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from safe_box_forest.experiments.sbf_old.tro2026_generate_tables import (  # noqa: E402
    anytime_marker_size,
    anytime_plot_x,
    centered_linear_range,
    import_pyplot,
    positive_range,
)


DEFAULT_OUTPUT_DIR = REPO_ROOT / "outputs" / "new_experiments" / "exp04_depth48_diagnosis_20260528"
DEFAULT_BASELINE_D40 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_full_ablation_20260528" / "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json"
DEFAULT_BASELINE_D48 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_split_compare_depth48_20260528" / "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json"
DEFAULT_BASELINE_D48_BOOSTED = REPO_ROOT / "outputs" / "new_experiments" / "exp04_depth48_boosted_schedule_20260528" / "baseline_depth48_boosted_schedule.json"
DEFAULT_ROUND_ROBIN_D48 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_split_compare_depth48_20260528" / "round_robin_split_policy.json"

CURVE_ORDER = [
    "baseline_d40",
    "baseline_d48",
    "baseline_d48_boosted",
    "round_robin_d48",
]

CURVE_STYLES = {
    "baseline_d40": {"label": "Baseline d40", "color": "#0b6e4f", "marker": "o"},
    "baseline_d48": {"label": "Baseline d48", "color": "#c17c00", "marker": "s"},
    "baseline_d48_boosted": {"label": "Baseline d48 boosted", "color": "#b23a48", "marker": "^"},
    "round_robin_d48": {"label": "Round-robin d48", "color": "#235789", "marker": "D"},
}

QUERY_ORDER = ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]
STAGE_LABELS = {
    "seed": "seed",
    "fast": "fast",
    "balanced": "balanced",
    "quality": "quality",
    "high": "high",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the Exp.4 depth diagnosis trade-off curve using the old TRO plotting convention.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--baseline-d40", type=Path, default=DEFAULT_BASELINE_D40)
    parser.add_argument("--baseline-d48", type=Path, default=DEFAULT_BASELINE_D48)
    parser.add_argument("--baseline-d48-boosted", type=Path, default=DEFAULT_BASELINE_D48_BOOSTED)
    parser.add_argument("--round-robin-d48", type=Path, default=DEFAULT_ROUND_ROBIN_D48)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def artifact_paths(args: argparse.Namespace) -> dict[str, Path]:
    return {
        "baseline_d40": args.baseline_d40,
        "baseline_d48": args.baseline_d48,
        "baseline_d48_boosted": args.baseline_d48_boosted,
        "round_robin_d48": args.round_robin_d48,
    }


def extract_points(payload: dict[str, Any]) -> list[dict[str, Any]]:
    summary = payload.get("summary") or {}
    points = [dict(point) for point in list(summary.get("points") or [])]
    points.sort(key=lambda row: int(row.get("stage_index", 0)))
    return points


def final_high_row(payload: dict[str, Any]) -> dict[str, Any]:
    rows = list(payload.get("raw_stage_rows") or [])
    for row in rows:
        if str(row.get("stage_id")) == "high":
            return dict(row.get("row") or {})
    return dict((rows[-1].get("row") if rows else {}) or {})


def final_query_lengths(payload: dict[str, Any]) -> dict[str, float]:
    high = final_high_row(payload)
    queries = list(high.get("queries") or [])
    return {
        str(query.get("name")): float(query.get("length", 0.0))
        for query in queries
        if query.get("name") is not None
    }


def stage_improvements(payload: dict[str, Any]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for record in list(payload.get("records") or []):
        out.append({
            "stage_id": str(record.get("stage_id")),
            "cumulative_total_s": float(record.get("cumulative_total_s", 0.0)),
            "incumbent_total_length": float(record.get("incumbent_total_length", 0.0)),
            "improved_tasks": list(record.get("improved_tasks") or []),
        })
    return out


def promoted_marker_style(point: dict[str, Any]) -> bool:
    if "promoted" in point:
        return bool(point.get("promoted"))
    return not bool(point.get("no_improvement_reason"))


def plot_curve(ax: Any, name: str, points: list[dict[str, Any]]) -> None:
    style = CURVE_STYLES[name]
    xs = [anytime_plot_x(point.get("total_s")) for point in points]
    ys = [float(point.get("path_length", 0.0)) for point in points]
    ax.plot(xs, ys, color=style["color"], linewidth=0.95, alpha=0.78)
    for point, x_value, y_value in zip(points, xs, ys):
        promoted = promoted_marker_style(point)
        ax.scatter(
            [x_value],
            [y_value],
            marker=style["marker"],
            s=anytime_marker_size(point.get("audit_sr", 1.0)),
            facecolors=style["color"] if promoted else "white",
            edgecolors=style["color"],
            linewidths=0.55 if promoted else 0.72,
            alpha=0.92,
            label=style["label"] if point is points[0] else None,
            zorder=3,
        )
        ax.annotate(
            STAGE_LABELS.get(str(point.get("stage_id")), str(point.get("stage_id"))),
            (x_value, y_value),
            textcoords="offset points",
            xytext=(3, 3),
            fontsize=6.0,
            color=style["color"],
        )


def build_summary(payloads: dict[str, dict[str, Any]], paths: dict[str, Path]) -> dict[str, Any]:
    summary: dict[str, Any] = {"artifacts": {}, "diagnosis": {}}
    for name in CURVE_ORDER:
        payload = payloads[name]
        points = extract_points(payload)
        final = points[-1]
        high = final_high_row(payload)
        summary["artifacts"][name] = {
            "label": CURVE_STYLES[name]["label"],
            "path": str(paths[name]),
            "final_total_s": float(final.get("total_s", 0.0)),
            "final_mean_path_length": float(final.get("path_length", 0.0)),
            "final_total_path_length": float(final.get("path_length_total", 0.0)),
            "all_stage_repair_total": int(sum(int((row.get("row") or {}).get("repair_total", 0)) for row in list(payload.get("raw_stage_rows") or []))),
            "all_stage_post_audit_all": bool(all(bool(query.get("post_audit_passed", False)) for row in list(payload.get("raw_stage_rows") or []) for query in list((row.get("row") or {}).get("queries") or []))),
            "final_query_lengths": final_query_lengths(payload),
            "stage_improvements": stage_improvements(payload),
            "points": points,
            "high_ok": bool(high.get("ok", False)),
        }

    base40 = summary["artifacts"]["baseline_d40"]
    base48 = summary["artifacts"]["baseline_d48"]
    boosted48 = summary["artifacts"]["baseline_d48_boosted"]
    round48 = summary["artifacts"]["round_robin_d48"]
    query_deltas = {}
    for query_name in QUERY_ORDER:
        base40_length = float(base40["final_query_lengths"].get(query_name, 0.0))
        base48_length = float(base48["final_query_lengths"].get(query_name, 0.0))
        query_deltas[query_name] = {
            "baseline40": base40_length,
            "baseline48": base48_length,
            "delta_48_minus_40": base48_length - base40_length,
            "round_robin48": float(round48["final_query_lengths"].get(query_name, 0.0)),
            "boosted48": float(boosted48["final_query_lengths"].get(query_name, 0.0)),
        }

    summary["diagnosis"] = {
        "baseline48_vs_baseline40_query_deltas": query_deltas,
        "baseline48_missing_improvements_vs_d40": {
            "baseline40_stage_improvements": base40["stage_improvements"],
            "baseline48_stage_improvements": base48["stage_improvements"],
        },
        "interpretation": [
            "Depth-48 baseline degrades even though all five queries still pass post-audit with zero repair.",
            "The degradation is not explained by insufficient d40_r4 stage budget alone: a larger depth-48 schedule does not recover the curve and instead worsens the final audited total path length.",
            "Round-robin at the same depth-48 cache and audit protocol restores a much shorter zero-repair curve, so the dominant interaction is AAFKVolumeMin split behavior at deeper trees rather than depth by itself.",
        ],
    }
    return summary


def write_tradeoff_figure(payloads: dict[str, dict[str, Any]], out_dir: Path) -> None:
    plt = import_pyplot()
    fig, ax = plt.subplots(1, 1, figsize=(6.6, 4.2))
    all_x: list[float] = []
    all_y: list[float] = []
    for name in CURVE_ORDER:
        points = extract_points(payloads[name])
        plot_curve(ax, name, points)
        all_x.extend(anytime_plot_x(point.get("total_s")) for point in points)
        all_y.extend(float(point.get("path_length", 0.0)) for point in points)
    ax.set_xscale("log")
    ax.set_xlabel("time / budget (s)")
    ax.set_ylabel("mean audited path length (rad)")
    ax.set_title("Shelf+IIWA depth diagnosis trade-off", fontsize=9.0)
    ax.grid(True, which="both", linewidth=0.35, alpha=0.35)
    ax.tick_params(labelsize=7.0)
    x_range = positive_range(all_x)
    if x_range:
        ax.set_xlim(*x_range)
    y_range = centered_linear_range(all_y)
    if y_range:
        ax.set_ylim(*y_range)
    handles, labels = ax.get_legend_handles_labels()
    unique: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        unique.setdefault(label, handle)
    ax.legend(unique.values(), unique.keys(), fontsize=6.7, frameon=False, loc="best", handletextpad=0.35, columnspacing=0.9)
    fig.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_exp04_depth_tradeoff_curve.pdf", bbox_inches="tight", pad_inches=0.02)
    fig.savefig(out_dir / "fig_exp04_depth_tradeoff_curve.png", dpi=220, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    paths = artifact_paths(args)
    payloads = {name: load_json(path) for name, path in paths.items()}
    write_tradeoff_figure(payloads, args.out_dir)
    summary = build_summary(payloads, paths)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "exp04_depth_tradeoff_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({
        "out_dir": str(args.out_dir),
        "figure_pdf": str(args.out_dir / "fig_exp04_depth_tradeoff_curve.pdf"),
        "figure_png": str(args.out_dir / "fig_exp04_depth_tradeoff_curve.png"),
        "summary_json": str(args.out_dir / "exp04_depth_tradeoff_summary.json"),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())