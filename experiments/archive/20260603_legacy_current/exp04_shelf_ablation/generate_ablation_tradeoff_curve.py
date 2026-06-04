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


DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "new_experiments" / "exp04_ablation_tradeoff_native_root_pair1_20260528"
DEFAULT_E4_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_native_root_pair1_20260528"

CURVE_SPECS = {
    "baseline": {
        "path_name": "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json",
        "label": "Baseline",
        "color": "#0b6e4f",
        "marker": "o",
    },
    "no_cache": {
        "path_name": "no_lect_cache_online_envelopes.json",
        "label": "No LECT cache",
        "color": "#235789",
        "marker": "s",
    },
    "critsample": {
        "path_name": "critsample_endpoint_support_hull.json",
        "label": "CritSample",
        "color": "#b23a48",
        "marker": "^",
    },
    "aabb": {
        "path_name": "aabb_envelope_only.json",
        "label": "AABB only",
        "color": "#c17c00",
        "marker": "D",
    },
    "single_thread": {
        "path_name": "single_thread.json",
        "label": "Single thread",
        "color": "#7b2cbf",
        "marker": "P",
    },
    "round_robin": {
        "path_name": "round_robin_split_policy.json",
        "label": "Round-robin",
        "color": "#495057",
        "marker": "X",
    },
}

CURVE_ORDER = ["baseline", "no_cache", "critsample", "aabb", "single_thread", "round_robin"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the complete Exp.4 ablation trade-off figure using the old TRO curve convention.")
    parser.add_argument("--e4-root", type=Path, default=DEFAULT_E4_ROOT)
    parser.add_argument("--round-robin", "--round-robin-d48", dest="round_robin", type=Path, default=None)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--plot-incomplete-task-sets", action="store_true", help="Plot points whose native audited incumbent set covers only a subset of tasks.")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def artifact_paths(args: argparse.Namespace) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for name, spec in CURVE_SPECS.items():
        if name == "round_robin" and args.round_robin is not None:
            paths[name] = args.round_robin
        else:
            paths[name] = args.e4_root / str(spec["path_name"])
    return paths


def extract_points(payload: dict[str, Any]) -> list[dict[str, Any]]:
    points = [dict(point) for point in list(((payload.get("summary") or {}).get("points") or []))]
    points.sort(key=lambda row: int(row.get("stage_index", 0)))
    return points


def complete_task_set(point: dict[str, Any]) -> bool:
    task_count = int(point.get("task_count", 0) or 0)
    success_count = int(point.get("success_count", 0) or 0)
    return task_count > 0 and success_count == task_count and float(point.get("audit_sr", 0.0) or 0.0) >= 1.0


def comparable_points(points: list[dict[str, Any]], *, include_incomplete: bool) -> list[dict[str, Any]]:
    if include_incomplete:
        return list(points)
    return [point for point in points if complete_task_set(point)]


def plot_curve(ax: Any, name: str, points: list[dict[str, Any]], *, include_incomplete: bool) -> None:
    spec = CURVE_SPECS[name]
    points = comparable_points(points, include_incomplete=include_incomplete)
    if not points:
        return
    xs = [anytime_plot_x(point.get("total_s")) for point in points]
    ys = [float(point.get("path_length", 0.0)) for point in points]
    ax.plot(xs, ys, color=spec["color"], linewidth=0.95, alpha=0.78)
    for index, (point, x_value, y_value) in enumerate(zip(points, xs, ys)):
        promoted = bool(point.get("promoted", not point.get("no_improvement_reason")))
        ax.scatter(
            [x_value],
            [y_value],
            marker=str(spec["marker"]),
            s=anytime_marker_size(point.get("audit_sr", 1.0)),
            facecolors=str(spec["color"]) if promoted else "white",
            edgecolors=str(spec["color"]),
            linewidths=0.55 if promoted else 0.75,
            alpha=0.92,
            zorder=3,
            label=str(spec["label"]) if index == 0 else None,
        )


def summary_entry(path: Path, payload: dict[str, Any], *, label: str) -> dict[str, Any]:
    points = extract_points(payload)
    final = points[-1]
    complete_points = comparable_points(points, include_incomplete=False)
    raw_queries = [
        query
        for row in list(payload.get("raw_stage_rows") or [])
        for query in list((row.get("row") or {}).get("queries") or [])
    ]
    repair_total = int(sum(int(query.get("repair_count", 0)) for query in raw_queries))
    native_success_total = int(sum(
        int(query.get("repair_count", 0)) == 0
        and int(query.get("segment_edges_used", 0)) > 0
        and len(query.get("box_sequence") or []) > 0
        for query in raw_queries
    ))
    final_audit_sr = float(final.get("audit_sr", 0.0))
    return {
        "label": label,
        "path": str(path),
        "stage_count": len(points),
        "comparable_stage_count": len(complete_points),
        "has_comparable_curve": bool(complete_points),
        "incomplete_stage_ids": [str(point.get("stage_id")) for point in points if not complete_task_set(point)],
        "final_total_s": float(final.get("total_s", 0.0)),
        "final_mean_path_length": float(final.get("path_length", 0.0)),
        "final_total_path_length": float(final.get("path_length_total", 0.0)),
        "final_audit_sr": final_audit_sr,
        "final_success_count": int(final.get("success_count", 0)),
        "final_task_count": int(final.get("task_count", 0)),
        "final_complete_task_set": final_audit_sr >= 1.0,
        "all_stage_post_audit_all": bool(all(bool(query.get("post_audit_passed", False)) for row in list(payload.get("raw_stage_rows") or []) for query in list((row.get("row") or {}).get("queries") or []))),
        "all_stage_repair_total": repair_total,
        "all_stage_native_success_total": native_success_total,
        "all_stage_query_total": len(raw_queries),
    }


def write_figure(payloads: dict[str, dict[str, Any]], out_dir: Path, *, include_incomplete: bool) -> None:
    plt = import_pyplot()
    fig, ax = plt.subplots(1, 1, figsize=(7.0, 4.5))
    all_x: list[float] = []
    all_y: list[float] = []
    for name in CURVE_ORDER:
        points = comparable_points(extract_points(payloads[name]), include_incomplete=include_incomplete)
        plot_curve(ax, name, points, include_incomplete=True)
        all_x.extend(anytime_plot_x(point.get("total_s")) for point in points)
        all_y.extend(float(point.get("path_length", 0.0)) for point in points)
    ax.set_xscale("log")
    ax.set_xlabel("time / budget (s)")
    ax.set_ylabel("mean audited path length (rad)")
    ax.set_title("Exp.4 ablation trade-off", fontsize=9.0)
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
    ax.legend(unique.values(), unique.keys(), fontsize=6.8, frameon=False, loc="best", handletextpad=0.35, columnspacing=0.85)
    fig.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "fig_exp04_ablation_tradeoff_curve.pdf", bbox_inches="tight", pad_inches=0.02)
    fig.savefig(out_dir / "fig_exp04_ablation_tradeoff_curve.png", dpi=220, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    paths = artifact_paths(args)
    payloads = {name: load_json(path) for name, path in paths.items()}
    write_figure(payloads, args.out_dir, include_incomplete=bool(args.plot_incomplete_task_sets))
    summary = {
        "plot_incomplete_task_sets": bool(args.plot_incomplete_task_sets),
        "included_curves": {
            name: summary_entry(paths[name], payloads[name], label=str(CURVE_SPECS[name]["label"]))
            for name in CURVE_ORDER
        },
        "omitted_rows": [
            {
                "name": "aabb_to_support_hull_chain",
                "reason": "Merged into the current support_hull pure-GJK path; not a distinct executable configuration.",
            },
            {
                "name": "round_robin_split_policy_d40",
                "reason": "Replaced by the current fingerprint-isolated round_robin_split_policy row from the same Exp.4 root.",
            },
            {
                "name": "baseline_d48",
                "reason": "Removed per request; this figure focuses on the complete Exp.4 comparison set rather than the depth diagnosis plot.",
            },
            {
                "name": "baseline_d48_boosted",
                "reason": "Removed per request; this figure focuses on the complete Exp.4 comparison set rather than the depth diagnosis plot.",
            },
        ],
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.out_dir / "exp04_ablation_tradeoff_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({
        "figure_pdf": str(args.out_dir / "fig_exp04_ablation_tradeoff_curve.pdf"),
        "figure_png": str(args.out_dir / "fig_exp04_ablation_tradeoff_curve.png"),
        "summary_json": str(summary_path),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())