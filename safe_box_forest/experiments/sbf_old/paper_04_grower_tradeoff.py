#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_04_marcucci_combined import (  # noqa: E402
    ROOT,
    configure,
    median,
    parse_args as parse_exp4_args,
    query_payload,
    refine_corridors,
    sbf,
)
from sbf.marcucci import make_combined_obstacles, make_combined_queries, make_coverage_seeds, load_iiwa14_robot  # noqa: E402


def parse_grid(text: str) -> list[int]:
    values = []
    for token in text.split(","):
        token = token.strip()
        if token:
            values.append(int(token))
    return sorted(dict.fromkeys(values))


def mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep grower quality floor and summarize build-time/path-quality trade-off.")
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_grower_tradeoff.json")
    parser.add_argument("--out-csv", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_grower_tradeoff.csv")
    parser.add_argument("--out-md", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_grower_tradeoff.md")
    parser.add_argument("--out-plot", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_grower_tradeoff.png")
    parser.add_argument("--quality-grid", default="0,16,32,64,96,128,160,224,320,512")
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--task-batch-size", type=int, default=8)
    parser.add_argument("--post-connect-time-budget-ms", type=float, default=0.0, help="0 disables the post-connect time cap")
    parser.add_argument("--max-boxes", type=int, default=5000)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--repair-weight", type=float, default=0.08)
    return parser.parse_args()


def make_exp4_args(args: argparse.Namespace, quality_floor: int, seed: int) -> argparse.Namespace:
    exp4 = parse_exp4_args([])
    exp4.seed_base = int(args.seed_base)
    exp4.threads = int(args.threads)
    exp4.task_batch_size = int(args.task_batch_size)
    exp4.max_boxes = max(int(args.max_boxes), int(quality_floor) + 32)
    exp4.ffb_depth = int(args.ffb_depth)
    exp4.quality_min_connected_boxes = int(quality_floor)
    exp4.post_connect_time_budget_ms = float(args.post_connect_time_budget_ms)
    exp4.corridor_refine = bool(args.corridor_refine)
    exp4.bridge_repaired_queries = bool(args.bridge_repaired_queries)
    exp4.bridge_failed_queries = bool(args.bridge_failed_queries)
    exp4.seeds = 1
    exp4.seed_base = int(args.seed_base) + int(seed)
    return exp4


def run_query_with_optional_bridge(forest: Any, query: Any, exp4_args: argparse.Namespace) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    query_s = time.perf_counter() - query_t0
    should_bridge = (not result.success and exp4_args.bridge_failed_queries) or (
        bool(exp4_args.bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        progress = int(forest.bridge_query(list(query.start), list(query.goal)))
        bridge_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        retry_s = time.perf_counter() - retry_t0
        row = query_payload(query, result, query_s + bridge_s + retry_s)
        row["name"] = query.label
        row["bridge_progress"] = int(progress)
        row["bridge_time_s"] = float(bridge_s)
    else:
        row = query_payload(query, result, query_s)
        row["name"] = query.label
        row["bridge_progress"] = 0
        row["bridge_time_s"] = 0.0
    return row


def summarize_setting(quality_floor: int, rows: list[dict[str, Any]], query_labels: list[str]) -> dict[str, Any]:
    trial_totals = []
    trial_repairs = []
    trial_query_times = []
    for trial in rows:
        successes = [row for row in trial["queries"] if row.get("ok")]
        trial_totals.append(sum(float(row.get("length", 0.0)) for row in successes))
        trial_repairs.append(sum(float(row.get("repair_count", 0.0)) for row in trial["queries"]))
        trial_query_times.append(sum(float(row.get("t_s", 0.0)) for row in successes))

    per_query = []
    for label in query_labels:
        values = []
        repairs = []
        audit = []
        sr = []
        for trial in rows:
            for row in trial["queries"]:
                if row["name"] == label:
                    sr.append(1.0 if row.get("ok") else 0.0)
                    audit.append(1.0 if row.get("audit_passed") else 0.0)
                    repairs.append(float(row.get("repair_count", 0.0)))
                    if row.get("ok"):
                        values.append(float(row.get("length", 0.0)))
                    break
        per_query.append({
            "name": label,
            "len_med": median(values),
            "repair_med": median(repairs),
            "sr": mean(sr),
            "audit_sr": mean(audit),
        })

    total_length_med = median(trial_totals)
    repair_total_med = median(trial_repairs)
    audit_values = [1.0 if row.get("audit_passed") else 0.0 for trial in rows for row in trial["queries"]]
    sr_values = [1.0 if row.get("ok") else 0.0 for trial in rows for row in trial["queries"]]
    build_values = [float(trial["build_s"]) for trial in rows]
    grow_values = [float(trial["build_profile"].get("grow_ms", 0.0)) / 1000.0 for trial in rows]
    out = {
        "quality_min_connected_boxes": int(quality_floor),
        "trial_count": len(rows),
        "build_mean_s": mean(build_values),
        "build_median_s": median(build_values),
        "grow_mean_s": mean(grow_values),
        "box_mean": mean([float(trial["n_boxes"]) for trial in rows]),
        "box_median": median([float(trial["n_boxes"]) for trial in rows]),
        "segment_edge_mean": mean([float(trial["segment_edge_count"]) for trial in rows]),
        "audit_sr": mean(audit_values),
        "sr": mean(sr_values),
        "total_length_median": total_length_med,
        "total_query_time_median_s": median(trial_query_times),
        "repair_total_median": repair_total_med,
        "queries": per_query,
    }
    return out


def choose_balance(settings: list[dict[str, Any]], repair_weight: float) -> dict[str, Any] | None:
    feasible = [row for row in settings if float(row.get("sr") or 0.0) >= 0.999 and float(row.get("audit_sr") or 0.0) >= 0.999]
    if not feasible:
        return None
    for row in feasible:
        path_term = float(row.get("total_length_median") or math.inf)
        repair_term = float(row.get("repair_total_median") or 0.0) / 5.0
        row["quality_objective"] = path_term + float(repair_weight) * repair_term
    build_values = [float(row["build_mean_s"]) for row in feasible]
    quality_values = [float(row["quality_objective"]) for row in feasible]
    build_min, build_max = min(build_values), max(build_values)
    quality_min, quality_max = min(quality_values), max(quality_values)
    best = None
    best_score = math.inf
    for row in feasible:
        build_norm = 0.0 if build_max <= build_min else (float(row["build_mean_s"]) - build_min) / (build_max - build_min)
        quality_norm = 0.0 if quality_max <= quality_min else (float(row["quality_objective"]) - quality_min) / (quality_max - quality_min)
        score = math.hypot(build_norm, quality_norm)
        row["balance_score"] = score
        if score < best_score:
            best = row
            best_score = score
    return best


def pareto_frontier(settings: list[dict[str, Any]]) -> list[dict[str, Any]]:
    candidates = [row for row in settings if row.get("quality_objective") is not None]
    frontier: list[dict[str, Any]] = []
    best_quality = math.inf
    for row in sorted(candidates, key=lambda item: float(item["build_mean_s"])):
        quality = float(row["quality_objective"])
        if quality < best_quality - 1e-12:
            frontier.append(row)
            best_quality = quality
    return frontier


def write_csv(path: Path, settings: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "quality_min_connected_boxes",
        "build_mean_s",
        "build_median_s",
        "grow_mean_s",
        "box_mean",
        "audit_sr",
        "sr",
        "total_length_median",
        "repair_total_median",
        "quality_objective",
        "balance_score",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in settings:
            writer.writerow({key: row.get(key) for key in fieldnames})


def write_markdown(path: Path, settings: list[dict[str, Any]], balance: dict[str, Any] | None, frontier: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Marcucci Grower Build/Quality Trade-off",
        "",
        "This sweep disables the 0.5 s build cap by setting `post_connect_time_budget_ms=0` unless otherwise specified.",
        "Path quality is measured by total median path length plus a small repair-count penalty.",
        "",
    ]
    if balance is not None:
        lines.extend([
            "## Balance Point",
            "",
            f"- quality floor: `{int(balance['quality_min_connected_boxes'])}` boxes",
            f"- build mean: `{float(balance['build_mean_s']):.6f}` s",
            f"- mean boxes: `{float(balance['box_mean']):.1f}`",
            f"- total length median: `{float(balance['total_length_median']):.6f}`",
            f"- repair total median: `{float(balance['repair_total_median']):.2f}`",
            "",
        ])
    lines.extend([
        "## Summary Table",
        "",
        "| Quality floor | Build mean (s) | Mean boxes | Audit SR | Total len med | Repair med sum | Objective |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in settings:
        lines.append(
            f"| {int(row['quality_min_connected_boxes'])} | "
            f"{float(row['build_mean_s']):.6f} | "
            f"{float(row['box_mean']):.1f} | "
            f"{float(row['audit_sr']):.3f} | "
            f"{float(row['total_length_median']):.6f} | "
            f"{float(row['repair_total_median']):.2f} | "
            f"{float(row.get('quality_objective', math.nan)):.4f} |"
        )
    lines.extend(["", "## Pareto Frontier", ""])
    if frontier:
        lines.extend(["| Quality floor | Build mean (s) | Total len med | Repair med sum |", "| ---: | ---: | ---: | ---: |"])
        for row in frontier:
            lines.append(
                f"| {int(row['quality_min_connected_boxes'])} | {float(row['build_mean_s']):.6f} | "
                f"{float(row['total_length_median']):.6f} | {float(row['repair_total_median']):.2f} |"
            )
    else:
        lines.append("No feasible Pareto frontier was found.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_plot(path: Path, settings: list[dict[str, Any]], balance: dict[str, Any] | None) -> None:
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    xs = [float(row["build_mean_s"]) for row in settings]
    ys = [float(row["total_length_median"]) for row in settings]
    repairs = [float(row["repair_total_median"]) for row in settings]
    sizes = [max(30.0, float(row["box_mean"]) * 0.9) for row in settings]
    labels = [str(int(row["quality_min_connected_boxes"])) for row in settings]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    scatter = axes[0].scatter(xs, ys, c=repairs, s=sizes, cmap="viridis_r", edgecolors="black", linewidths=0.5)
    for x_value, y_value, label in zip(xs, ys, labels):
        axes[0].annotate(label, (x_value, y_value), textcoords="offset points", xytext=(4, 4), fontsize=8)
    axes[0].set_xlabel("Build mean (s)")
    axes[0].set_ylabel("Total median path length")
    axes[0].set_title("Path Quality vs Build Time")
    fig.colorbar(scatter, ax=axes[0], label="Median total repair count")

    axes[1].plot(xs, repairs, marker="o", color="#7a3db8")
    for x_value, y_value, label in zip(xs, repairs, labels):
        axes[1].annotate(label, (x_value, y_value), textcoords="offset points", xytext=(4, 4), fontsize=8)
    axes[1].set_xlabel("Build mean (s)")
    axes[1].set_ylabel("Median total repair count")
    axes[1].set_title("Repair Reliance vs Build Time")
    if balance is not None:
        bx = float(balance["build_mean_s"])
        by = float(balance["total_length_median"])
        axes[0].scatter([bx], [by], marker="*", s=220, color="#d62728", edgecolors="black", linewidths=0.6, zorder=5)
        axes[1].scatter([bx], [float(balance["repair_total_median"])], marker="*", s=220, color="#d62728", edgecolors="black", linewidths=0.6, zorder=5)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    quality_grid = parse_grid(args.quality_grid)
    robot = load_iiwa14_robot()
    obstacles = make_combined_obstacles()
    queries = make_combined_queries()
    query_labels = [query.label for query in queries]
    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]

    trials_by_quality: dict[int, list[dict[str, Any]]] = {quality: [] for quality in quality_grid}
    for quality_floor in quality_grid:
        for seed_index in range(max(1, int(args.seeds))):
            exp4_args = make_exp4_args(args, quality_floor, seed_index)
            cfg = configure(exp4_args, seed_index)
            forest = sbf.SafeBoxForest(robot, cfg)
            build_t0 = time.perf_counter()
            profile = forest.build_coverage(obstacles, seeds)
            prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, exp4_args)
            build_s = time.perf_counter() - build_t0
            query_rows = [run_query_with_optional_bridge(forest, query, exp4_args) for query in queries]
            trials_by_quality[quality_floor].append({
                "seed_index": int(seed_index),
                "build_s": float(build_s),
                "prebridge_time_s": float(prebridge_time_s),
                "prebridge_added_boxes": int(prebridge_added_boxes),
                "prebridge_attempts": int(prebridge_attempts),
                "n_boxes": len(forest.boxes()),
                "segment_edge_count": len(forest.segment_edges()),
                "build_profile": {
                    "total_ms": float(profile.total_ms),
                    "grow_ms": float(profile.grow_ms),
                    "merge_ms": float(profile.merge_ms),
                    "connector_ms": float(profile.connector_ms),
                    "adjacency_ms": float(profile.adjacency_ms),
                    "raw_boxes": int(profile.raw_boxes),
                    "final_boxes": int(profile.final_boxes),
                    "segment_edges": int(profile.segment_edges),
                    "adjacency_islands": int(profile.adjacency_islands),
                    "diagnostics": {str(key): float(value) for key, value in dict(profile.diagnostics).items()},
                },
                "queries": query_rows,
            })

    settings = [summarize_setting(quality, trials_by_quality[quality], query_labels) for quality in quality_grid]
    balance = choose_balance(settings, float(args.repair_weight))
    frontier = pareto_frontier(settings)
    payload = {
        "experiment": "paper_04_marcucci_grower_tradeoff",
        "params": {
            "quality_grid": quality_grid,
            "seeds": int(args.seeds),
            "post_connect_time_budget_ms": float(args.post_connect_time_budget_ms),
            "corridor_refine": bool(args.corridor_refine),
            "bridge_repaired_queries": bool(args.bridge_repaired_queries),
            "repair_weight": float(args.repair_weight),
        },
        "settings": settings,
        "balance_point": balance,
        "pareto_frontier": [int(row["quality_min_connected_boxes"]) for row in frontier],
        "trials_by_quality": trials_by_quality,
    }

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_csv(args.out_csv, settings)
    write_markdown(args.out_md, settings, balance, frontier)
    write_plot(args.out_plot, settings, balance)
    print(json.dumps({
        "out_json": str(args.out_json),
        "out_csv": str(args.out_csv),
        "out_md": str(args.out_md),
        "out_plot": str(args.out_plot),
        "balance_point": None if balance is None else {
            "quality_min_connected_boxes": int(balance["quality_min_connected_boxes"]),
            "build_mean_s": float(balance["build_mean_s"]),
            "box_mean": float(balance["box_mean"]),
            "total_length_median": float(balance["total_length_median"]),
            "repair_total_median": float(balance["repair_total_median"]),
        },
        "settings": [{
            "quality": int(row["quality_min_connected_boxes"]),
            "build_mean_s": row["build_mean_s"],
            "box_mean": row["box_mean"],
            "total_length_median": row["total_length_median"],
            "repair_total_median": row["repair_total_median"],
        } for row in settings],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())