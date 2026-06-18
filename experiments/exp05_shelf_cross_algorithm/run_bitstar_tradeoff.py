#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, write_json
from experiments.common.progress import progress
from experiments.common.rbf_defaults import DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE, DEFAULT_RBF_AUDIT_SEGMENT_STEP
from experiments.exp05_shelf_cross_algorithm import run_bitstar_per_query


def csv_floats(raw: str) -> list[float]:
    return [float(item.strip()) for item in str(raw).split(",") if item.strip()]


def summarize_rows(rows: list[dict[str, Any]], stage_id: str) -> dict[str, Any]:
    seeds = sorted({int(row["seed"]) for row in rows})
    success_rows = [row for row in rows if bool(row.get("audit_passed"))]
    success_runs = 0
    for seed in seeds:
        seed_rows = [row for row in rows if int(row.get("seed", -1)) == seed]
        if len(seed_rows) == 5 and all(bool(row.get("audit_passed")) for row in seed_rows):
            success_runs += 1
    planning_by_seed = []
    for seed in seeds:
        seed_rows = [row for row in rows if int(row.get("seed", -1)) == seed]
        planning_by_seed.append(sum(float(row.get("planning_s", 0.0) or 0.0) for row in seed_rows))
    return {
        "method": "bitstar",
        "stage_id": stage_id,
        "runs": len(seeds),
        "success_runs": success_runs,
        "query_rows": len(rows),
        "success_queries": len(success_rows),
        "planning_s_mean_per_run": sum(planning_by_seed) / max(1, len(planning_by_seed)),
        "planning_s_median_per_run": sorted(planning_by_seed)[len(planning_by_seed) // 2] if planning_by_seed else math.nan,
        "path_length_mean": (
            sum(float(row["path_length"]) for row in success_rows) / len(success_rows)
            if success_rows else math.nan
        ),
        "failure_statuses": ";".join(
            sorted({f"{row.get('label')}:{row.get('audit_status')}:{row.get('planner_status')}" for row in rows if not row.get("audit_passed")})
        ),
    }


def run_one(args: argparse.Namespace, timeout_s: float) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    subdir = Path(args.out_dir) / f"t{timeout_s:g}s_batch{int(args.samples_per_batch)}_rw{float(args.rewire_factor):g}"
    child_args = argparse.Namespace(
        out_dir=subdir,
        seeds=args.seeds,
        query_indices=args.query_indices,
        timeout_s=float(timeout_s),
        samples_per_batch=int(args.samples_per_batch),
        rewire_factor=float(args.rewire_factor),
        wall_timeout_factor=float(args.wall_timeout_factor),
        audit_segment_step=float(args.audit_segment_step),
        audit_collision_tolerance=float(args.audit_collision_tolerance),
        simplify_time_s=float(args.simplify_time_s),
        stop_on_solution_improvement=bool(args.stop_on_solution_improvement),
        use_k_nearest=int(args.use_k_nearest),
        pruning=int(args.pruning),
        prune_threshold_fraction=float(args.prune_threshold_fraction),
        delay_rewiring_until_initial_solution=int(args.delay_rewiring_until_initial_solution),
        just_in_time_sampling=int(args.just_in_time_sampling),
        drop_samples_on_prune=int(args.drop_samples_on_prune),
        approximate_solutions=int(args.approximate_solutions),
        strict_queue_ordering=int(args.strict_queue_ordering),
        cascading_rewirings=int(args.cascading_rewirings),
        initial_inflation_factor=float(args.initial_inflation_factor),
        inflation_scaling_parameter=float(args.inflation_scaling_parameter),
        truncation_scaling_parameter=float(args.truncation_scaling_parameter),
        allowed_failed_sampling_attempts=int(args.allowed_failed_sampling_attempts),
        append=False,
        worker=False,
        seed=0,
        query_index=0,
    )
    run_bitstar_per_query.parent(child_args)
    manifest = json.loads((subdir / "bitstar_per_query_manifest.json").read_text(encoding="utf-8"))
    rows = [dict(row) for row in manifest.get("rows", [])]
    stage_id = rows[0].get("stage_id", f"t{timeout_s:g}s") if rows else f"t{timeout_s:g}s"
    return summarize_rows(rows, str(stage_id)), rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.5 BIT* timeout trade-off runner using fork watchdog workers.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp05_bitstar_tradeoff")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--query-indices", default="0,1,2,3,4")
    parser.add_argument("--timeout-grid-s", default="0.05,0.1,0.2,0.5,1,2,5")
    parser.add_argument("--samples-per-batch", type=int, default=100)
    parser.add_argument("--rewire-factor", type=float, default=5.0)
    parser.add_argument("--wall-timeout-factor", type=float, default=1.5)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--simplify-time-s", type=float, default=0.01)
    parser.add_argument("--stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-k-nearest", type=int, default=-1)
    parser.add_argument("--pruning", type=int, default=-1)
    parser.add_argument("--prune-threshold-fraction", type=float, default=-1.0)
    parser.add_argument("--delay-rewiring-until-initial-solution", type=int, default=-1)
    parser.add_argument("--just-in-time-sampling", type=int, default=-1)
    parser.add_argument("--drop-samples-on-prune", type=int, default=-1)
    parser.add_argument("--approximate-solutions", type=int, default=-1)
    parser.add_argument("--strict-queue-ordering", type=int, default=-1)
    parser.add_argument("--cascading-rewirings", type=int, default=-1)
    parser.add_argument("--initial-inflation-factor", type=float, default=-1.0)
    parser.add_argument("--inflation-scaling-parameter", type=float, default=-1.0)
    parser.add_argument("--truncation-scaling-parameter", type=float, default=-1.0)
    parser.add_argument("--allowed-failed-sampling-attempts", type=int, default=-1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    summaries: list[dict[str, Any]] = []
    all_rows: list[dict[str, Any]] = []
    timeouts = csv_floats(args.timeout_grid_s)
    for timeout_s in progress(timeouts, desc="bitstar tradeoff", total=len(timeouts)):
        summary, rows = run_one(args, timeout_s)
        summaries.append(summary)
        all_rows.extend(rows)

    write_json(args.out_dir / "bitstar_tradeoff_manifest.json", {"summaries": summaries, "rows": all_rows})
    with (args.out_dir / "bitstar_tradeoff_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "method",
            "stage_id",
            "runs",
            "success_runs",
            "query_rows",
            "success_queries",
            "planning_s_mean_per_run",
            "planning_s_median_per_run",
            "path_length_mean",
            "failure_statuses",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in summaries:
            writer.writerow(row)
    print(json.dumps(summaries, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
