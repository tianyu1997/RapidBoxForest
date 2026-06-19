#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median
from experiments.common.progress import progress
from experiments.common.rbf_defaults import (
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_OFFLINE_ANCHOR_CANDIDATE_COUNT,
    DEFAULT_RBF_OFFLINE_ANCHOR_COUNT,
    DEFAULT_RBF_OFFLINE_ANCHOR_DISTANCE_MU,
    DEFAULT_RBF_OFFLINE_ANCHOR_LCA_LAMBDA,
    DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS,
    DEFAULT_RBF_OMPL_SIMPLIFY_TIME_S,
    DEFAULT_RBF_SHELF_BOX_BUDGET,
    DEFAULT_RBF_THREADS,
)
from experiments.exp05_shelf_cross_algorithm.run_shelf_cross_algorithm import (
    configure_thread_environment,
    run_sbf,
    sbf,
    shelf_queries,
)


def csv_ints(raw: str) -> list[int]:
    return [int(item.strip()) for item in str(raw).split(",") if item.strip()]


def safe_label(label: str) -> str:
    return (
        str(label)
        .replace("->", "_to_")
        .replace("/", "_")
        .replace(" ", "_")
        .replace(":", "_")
    )


def finite(value: Any, default: float = math.nan) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def success_query(row: dict[str, Any]) -> dict[str, Any]:
    queries = list(row.get("queries", []))
    return dict(queries[0]) if queries else {}


def summarize_by_query(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    labels = sorted({str(row.get("single_query_label", "")) for row in rows})
    out: list[dict[str, Any]] = []
    for label in labels:
        items = [row for row in rows if str(row.get("single_query_label", "")) == label]
        qitems = [success_query(row) for row in items]
        success_items = [
            (row, q)
            for row, q in zip(items, qitems, strict=True)
            if bool(q.get("success")) and bool(q.get("audit_passed"))
        ]
        online_solve = [finite(row.get("online_per_query_s")) for row in items]
        online_total = [finite(row.get("online_total_per_query_s")) for row in items]
        simplify = [finite(row.get("online_simplify_per_query_s")) for row in items]
        query_graph = [finite(q.get("query_ms")) / 1000.0 for q in qitems]
        qbridge = [finite(row.get("query_bridge_s")) for row in items]
        build = [finite(row.get("offline_build_s", row.get("build_s"))) for row in items]
        path_lengths = [finite(q.get("path_length")) for _row, q in success_items]
        raw_lengths = [finite(q.get("raw_path_length")) for _row, q in success_items]
        seg_fracs = [finite(q.get("segment_fraction")) for _row, q in success_items]
        seg_edges = [int(float(q.get("segment_edges_used", 0) or 0)) for _row, q in success_items]
        out.append({
            "method": "sbf_leaf_rrt",
            "method_label": "RBF",
            "stage_id": f"b{int(items[0].get('deep_max_boxes', 0) or 0)}_{safe_label(label)}",
            "query_label": label,
            "deep_max_boxes": int(items[0].get("deep_max_boxes", 0) or 0),
            "runs": len(items),
            "success_runs": sum(
                1
                for row in items
                if int(row.get("success_count", 0) or 0) == int(row.get("query_count", 0) or 0)
            ),
            "success_queries": len(success_items),
            "total_queries": len(items),
            "offline_build_s_median": median(build),
            "online_solve_wall_s_median": median(online_solve),
            "online_total_wall_s_median": median(online_total),
            "simplify_s_median": median(simplify),
            "query_graph_s_median": median(query_graph),
            "query_bridge_s_median": median(qbridge),
            "amortized_s_k1": median(
                finite(row.get("offline_build_s", row.get("build_s")), 0.0)
                + finite(row.get("online_per_query_s"), 0.0)
                for row in items
            ),
            "amortized_s_k5": median(
                finite(row.get("offline_build_s", row.get("build_s")), 0.0) / 5.0
                + finite(row.get("online_per_query_s"), 0.0)
                for row in items
            ),
            "path_length_mean": mean(path_lengths),
            "path_length_median": median(path_lengths),
            "raw_path_length_mean": mean(raw_lengths),
            "raw_segment_fraction_median": median(seg_fracs),
            "segment_edges_used_median": statistics.median(seg_edges) if seg_edges else math.nan,
            "source": "single_query_online_wall_clock",
            "status": "executed",
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_report(path: Path, summary: list[dict[str, Any]], rows: list[dict[str, Any]]) -> None:
    lines = [
        "# Exp.5 RBF Single-Query Online Timing",
        "",
        "This artifact reruns RBF one shelf query at a time. The reported online solve wall-clock time is not derived from a batch-parallel run.",
        "",
        "- `online_solve_wall_s_median` excludes final audit and excludes the fixed OMPL simplify budget.",
        "- `online_total_wall_s_median` includes the fixed 0.01 s OMPL simplify step.",
        "- Each raw run has `query_count=1`.",
        "",
        "| Query | SR | Build median (s) | Online solve median (ms) | Total median (ms) | Path mean | Seg. median |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            "| {query_label} | {success_queries}/{total_queries} | {build:.3f} | {online_ms:.3f} | "
            "{total_ms:.3f} | {path:.3f} | {seg:.3f} |".format(
                query_label=row["query_label"],
                success_queries=int(row["success_queries"]),
                total_queries=int(row["total_queries"]),
                build=finite(row["offline_build_s_median"]),
                online_ms=1000.0 * finite(row["online_solve_wall_s_median"]),
                total_ms=1000.0 * finite(row["online_total_wall_s_median"]),
                path=finite(row["path_length_mean"]),
                seg=finite(row["raw_segment_fraction_median"]),
            )
        )
    all_online = [finite(row.get("online_per_query_s")) for row in rows]
    all_success = sum(int(row.get("success_count", 0) or 0) for row in rows)
    lines.extend([
        "",
        f"Overall success: {all_success}/{len(rows)}.",
        f"Overall single-query online solve median: {1000.0 * median(all_online):.3f} ms.",
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rerun Exp.5 RBF as independent single-query online measurements.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / run_id("exp05_rbf_single_query_online"))
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--budget", type=int, default=DEFAULT_RBF_SHELF_BOX_BUDGET)
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--ompl-simplify-time-s", type=float, default=DEFAULT_RBF_OMPL_SIMPLIFY_TIME_S)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    parser.add_argument("--offline-random-anchors", action=argparse.BooleanOptionalAction, default=DEFAULT_RBF_OFFLINE_RANDOM_ANCHORS)
    parser.add_argument("--offline-anchor-count", type=int, default=DEFAULT_RBF_OFFLINE_ANCHOR_COUNT)
    parser.add_argument("--offline-anchor-candidate-count", type=int, default=DEFAULT_RBF_OFFLINE_ANCHOR_CANDIDATE_COUNT)
    parser.add_argument("--offline-anchor-lca-lambda", type=float, default=DEFAULT_RBF_OFFLINE_ANCHOR_LCA_LAMBDA)
    parser.add_argument("--offline-anchor-distance-mu", type=float, default=DEFAULT_RBF_OFFLINE_ANCHOR_DISTANCE_MU)
    parser.add_argument("--query-bridge-to-main-island", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    configure_thread_environment(int(args.threads))
    args.out_dir.mkdir(parents=True, exist_ok=True)

    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = shelf_queries(robot)
    seeds = csv_ints(args.seeds)

    rows: list[dict[str, Any]] = []
    total = len(seeds) * len(queries)
    for seed in progress(seeds, desc="exp05 rbf single-query seeds", total=len(seeds)):
        for query in progress(queries, desc=f"seed={seed} query", total=len(queries)):
            label = str(query["label"])
            run_args = argparse.Namespace(**vars(args))
            run_args.out_dir = args.out_dir / "runs" / f"seed{int(seed)}_{safe_label(label)}"
            row = run_sbf(int(seed), run_args, robot, obstacles, [query], int(args.budget))
            row["single_query_online"] = True
            row["single_query_label"] = label
            row["single_query_seed"] = int(seed)
            row["stage_id"] = f"b{int(args.budget)}_{safe_label(label)}"
            row["measurement_note"] = "independent one-query wall-clock run; not split from batch-parallel artifact"
            if int(row.get("query_count", 0) or 0) != 1:
                raise RuntimeError(f"single-query invariant failed for {label}, seed {seed}: query_count={row.get('query_count')}")
            rows.append(row)
            print(
                f"[{len(rows)}/{total}] seed={seed} query={label} "
                f"online={1000.0 * finite(row.get('online_per_query_s')):.3f} ms "
                f"total={1000.0 * finite(row.get('online_total_per_query_s')):.3f} ms "
                f"success={row.get('success_count')}/{row.get('query_count')}",
                flush=True,
            )

    summary = summarize_by_query(rows)
    write_json(
        args.out_dir / "shelf_rbf_single_query_online_manifest.json",
        {
            "experiment": "exp05_rbf_single_query_online",
            "measurement_semantics": {
                "query_count_per_run": 1,
                "online_solve_wall_s": "wall-clock online stage for one query; excludes final audit and fixed OMPL simplify",
                "online_total_wall_s": "online_solve_wall_s plus measured fixed 0.01 s OMPL simplify",
                "not_derived_from_batch_parallel_artifact": True,
            },
            "args": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
            "environment": environment_metadata(),
            "rows": rows,
            "summary": summary,
        },
    )
    write_csv(args.out_dir / "shelf_rbf_single_query_online_runs.csv", rows)
    write_csv(args.out_dir / "shelf_rbf_single_query_online_summary.csv", summary)
    write_report(args.out_dir / "shelf_rbf_single_query_online_summary.md", summary, rows)
    print(f"wrote {args.out_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
