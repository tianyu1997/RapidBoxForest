#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.rbf_defaults import DEFAULT_RBF_SHELF_BOX_BUDGET, DEFAULT_RBF_THREADS


RUNNER = REPO_ROOT / "experiments" / "exp04_shelf_leaf_rrt" / "run_shelf_leaf_rrt.py"
BASELINE_CASE = "baseline_d23_aafk_support_hull_8t"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a one-seed Exp.4 baseline efficiency comparison between the "
            "legacy fixed/refine path and the optimized adaptive partition path."
        )
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp04_optimized_baseline_compare",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--box-budget", type=int, default=DEFAULT_RBF_SHELF_BOX_BUDGET)
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full"], default="smoke")
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--leaf-start-depth", type=int, default=8)
    parser.add_argument("--leaf-max-depth", type=int, default=13)
    parser.add_argument("--adaptive-target-depth", type=int, default=13)
    parser.add_argument("--adaptive-grid-target-depth", type=int, default=13)
    parser.add_argument("--adaptive-time-budget-ms", type=float, default=60000.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def base_command(args: argparse.Namespace, out_dir: Path) -> list[str]:
    return [
        sys.executable,
        str(RUNNER),
        "--out-dir",
        str(out_dir),
        "--phase",
        str(args.phase),
        "--only",
        BASELINE_CASE,
        "--seeds",
        str(int(args.seed)),
        "--box-budgets",
        str(int(args.box_budget)),
        "--threads",
        str(int(args.threads)),
        "--timeout-ms",
        str(float(args.timeout_ms)),
        "--leaf-start-depth",
        str(int(args.leaf_start_depth)),
        "--leaf-max-depth",
        str(int(args.leaf_max_depth)),
        "--adaptive-target-depth",
        str(int(args.adaptive_target_depth)),
        "--adaptive-grid-target-depth",
        str(int(args.adaptive_grid_target_depth)),
        "--adaptive-time-budget-ms",
        str(float(args.adaptive_time_budget_ms)),
    ]


def variants(args: argparse.Namespace) -> list[dict[str, Any]]:
    return [
        {
            "variant": "legacy_leaf_refine_box_graph",
            "description": "Exp.4 baseline case using legacy build_leaf_sweep_refined and box graph query backend.",
            "extra_args": [
                "--offline-grower",
                "leaf_refine",
                "--adaptive-planning-backend",
                "box_graph",
                "--no-adaptive-depth-enabled",
            ],
        },
        {
            "variant": "optimized_adaptive_partition",
            "description": "Exp.4 baseline case using adaptive_deep_leaf and partition_native backend.",
            "extra_args": [
                "--offline-grower",
                "adaptive_deep_leaf",
                "--adaptive-planning-backend",
                "partition_native",
                "--adaptive-depth-enabled",
            ],
        },
    ]


def run_variant(args: argparse.Namespace, variant: dict[str, Any]) -> dict[str, Any]:
    out_dir = args.out_dir / str(variant["variant"])
    command = base_command(args, out_dir) + list(variant["extra_args"])
    if args.dry_run:
        return {
            "variant": variant["variant"],
            "description": variant["description"],
            "out_dir": str(out_dir),
            "command": command,
            "returncode": 0,
            "elapsed_s": 0.0,
            "dry_run": True,
        }
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    t0 = time.perf_counter()
    proc = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed = time.perf_counter() - t0
    summary_path = out_dir / "shelf_leaf_rrt_summary.csv"
    manifest_path = out_dir / "shelf_leaf_rrt_manifest.json"
    summary_rows = read_csv_rows(summary_path) if summary_path.exists() else []
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    return {
        "variant": variant["variant"],
        "description": variant["description"],
        "out_dir": str(out_dir),
        "command": command,
        "returncode": int(proc.returncode),
        "elapsed_s": elapsed,
        "stdout_tail": proc.stdout[-4000:],
        "stderr_tail": proc.stderr[-4000:],
        "summary_path": str(summary_path),
        "manifest_path": str(manifest_path),
        "summary": summary_rows,
        "manifest_status": manifest.get("status"),
    }


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def numeric(row: dict[str, Any], key: str) -> float:
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def compact_row(result: dict[str, Any]) -> dict[str, Any]:
    summary = result.get("summary") or []
    row = summary[0] if summary else {}
    return {
        "variant": result["variant"],
        "returncode": result["returncode"],
        "success_queries": row.get("success_queries", ""),
        "total_queries": row.get("total_queries", ""),
        "offline_grower": row.get("offline_grower", ""),
        "offline_build_s": numeric(row, "offline_build_s_median"),
        "online_per_query_s": numeric(row, "online_per_query_s_median"),
        "online_total_per_query_s": numeric(row, "online_total_per_query_s_median"),
        "query_bridge_per_query_s": numeric(row, "query_bridge_per_query_s_median"),
        "partition_query_per_query_s": numeric(row, "partition_query_per_query_s_median"),
        "path_length_mean": numeric(row, "path_length_mean"),
        "raw_segment_fraction": numeric(row, "raw_segment_fraction_median"),
        "final_boxes": numeric(row, "final_boxes_median"),
        "partition_cells": numeric(row, "partition_cell_count_median"),
        "partition_islands": numeric(row, "partition_islands_median"),
        "coverage_main_accessible_probability": numeric(row, "coverage_main_accessible_probability_median"),
        "external_hits": numeric(row, "external_hits_median"),
        "external_reused_hits": numeric(row, "external_reused_hits_median"),
        "external_exact_hits": numeric(row, "external_exact_hits_median"),
        "external_exact_misses": numeric(row, "external_exact_misses_median"),
        "elapsed_s": float(result["elapsed_s"]),
        "out_dir": result["out_dir"],
    }


def ratio(numerator: float, denominator: float) -> float | None:
    if denominator != denominator or abs(denominator) < 1e-15:
        return None
    if numerator != numerator:
        return None
    return numerator / denominator


def write_compare_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_compare_md(path: Path, rows: list[dict[str, Any]], speedups: dict[str, float | None]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Exp.4 Optimized Baseline Efficiency Compare",
        "",
        "| Variant | SR | Build s | Online/q s | Total/q s | Path | Seg. | Boxes | Partition cells | Ext. reused | Ext. exact |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        sr = f"{row['success_queries']}/{row['total_queries']}"
        lines.append(
            f"| {row['variant']} | {sr} | {row['offline_build_s']:.6g} | "
            f"{row['online_per_query_s']:.6g} | {row['online_total_per_query_s']:.6g} | "
            f"{row['path_length_mean']:.6g} | {row['raw_segment_fraction']:.6g} | "
            f"{row['final_boxes']:.6g} | {row['partition_cells']:.6g} | "
            f"{row['external_reused_hits']:.6g} | {row['external_exact_hits']:.6g} |"
        )
    lines.extend(["", "## Speedups (legacy / optimized)", ""])
    for key, value in speedups.items():
        rendered = "n/a" if value is None else f"{value:.4g}x"
        lines.append(f"- `{key}`: {rendered}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    results = [run_variant(args, variant) for variant in variants(args)]
    compact = [compact_row(result) for result in results]
    by_variant = {row["variant"]: row for row in compact}
    legacy = by_variant.get("legacy_leaf_refine_box_graph", {})
    optimized = by_variant.get("optimized_adaptive_partition", {})
    speedups = {
        "offline_build_s": ratio(float(legacy.get("offline_build_s", float("nan"))), float(optimized.get("offline_build_s", float("nan")))),
        "online_per_query_s": ratio(float(legacy.get("online_per_query_s", float("nan"))), float(optimized.get("online_per_query_s", float("nan")))),
        "online_total_per_query_s": ratio(float(legacy.get("online_total_per_query_s", float("nan"))), float(optimized.get("online_total_per_query_s", float("nan")))),
        "elapsed_s": ratio(float(legacy.get("elapsed_s", float("nan"))), float(optimized.get("elapsed_s", float("nan")))),
    }
    payload = {
        "experiment": "exp04_optimized_baseline_compare",
        "run_id": run_id("exp04_opt_compare"),
        "environment": environment_metadata(),
        "config": {
            "seed": int(args.seed),
            "box_budget": int(args.box_budget),
            "threads": int(args.threads),
            "phase": str(args.phase),
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "adaptive_target_depth": int(args.adaptive_target_depth),
            "adaptive_grid_target_depth": int(args.adaptive_grid_target_depth),
        },
        "results": results,
        "compact": compact,
        "speedups_legacy_over_optimized": speedups,
    }
    write_json(args.out_dir / "optimized_baseline_compare.json", payload)
    if compact:
        write_compare_csv(args.out_dir / "optimized_baseline_compare.csv", compact)
        write_compare_md(args.out_dir / "optimized_baseline_compare.md", compact, speedups)
    print(f"wrote {args.out_dir / 'optimized_baseline_compare.json'}")
    return 0 if all(int(result["returncode"]) == 0 for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
