#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import sys
import time
import traceback
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, write_csv as write_csv_rows, write_json
from experiments.common.path_tools import audit_path, path_length
from experiments.common.rbf_defaults import (
    DEFAULT_OMPL_SIMPLIFY_TIME_S,
    DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
)
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


def query_specs() -> list[dict[str, Any]]:
    return [
        {
            "index": index,
            "label": str(query.label),
            "start": [float(value) for value in query.start],
            "goal": [float(value) for value in query.goal],
        }
        for index, query in enumerate(list(sbf.make_combined_queries()))
    ]


def run_query_once(args: argparse.Namespace) -> dict[str, Any]:
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    query = query_specs()[int(args.query_index)]
    t0 = time.perf_counter()
    raw = sbf.ompl_bitstar_path(
        robot,
        obstacles,
        list(query["start"]),
        list(query["goal"]),
        float(args.timeout_s) * 1000.0,
        float(args.audit_segment_step),
        0.0,
        int(args.seed) * 2003 + int(args.query_index),
        int(args.samples_per_batch),
        float(args.rewire_factor),
        bool(args.stop_on_solution_improvement),
        int(args.use_k_nearest),
        int(args.pruning),
        float(args.prune_threshold_fraction),
        int(args.delay_rewiring_until_initial_solution),
        int(args.just_in_time_sampling),
        int(args.drop_samples_on_prune),
        int(args.approximate_solutions),
        int(args.strict_queue_ordering),
        int(args.cascading_rewirings),
        float(args.initial_inflation_factor),
        float(args.inflation_scaling_parameter),
        float(args.truncation_scaling_parameter),
        int(args.allowed_failed_sampling_attempts),
    )
    path = [[float(value) for value in point] for point in raw.get("path", [])]
    simplify_s = 0.0
    if float(args.simplify_time_s) > 0.0 and len(path) >= 2:
        simplified = sbf.ompl_simplify_path(
            robot,
            obstacles,
            path,
            float(args.audit_segment_step),
            float(args.simplify_time_s),
        )
        simplify_s = float(simplified.get("t_s", 0.0) or 0.0)
        maybe_path = [[float(value) for value in point] for point in simplified.get("path", [])]
        if bool(simplified.get("ok")) and len(maybe_path) >= 2:
            path = maybe_path
    audit_passed, audit_s, audit_status = audit_path(
        sbf,
        robot,
        obstacles,
        path,
        float(args.audit_segment_step),
        start=list(query["start"]),
        goal=list(query["goal"]),
        collision_tolerance=float(args.audit_collision_tolerance),
    )
    ok = bool(raw.get("ok")) and audit_passed
    row = {
        "method": "bitstar",
        "seed": int(args.seed),
        "query_index": int(args.query_index),
        "label": query["label"],
        "success": ok,
        "audit_passed": audit_passed,
        "audit_status": audit_status,
        "planner_status": str(raw.get("status", raw.get("reason", ""))),
        "planning_s": float(raw.get("solve_s", raw.get("t_s", time.perf_counter() - t0))) + simplify_s,
        "solve_s": float(raw.get("solve_s", raw.get("t_s", time.perf_counter() - t0))),
        "simplify_s": simplify_s,
        "audit_s": audit_s,
        "path_length": path_length(path) if ok else math.nan,
        "waypoint_count": len(path),
        "timeout_s": float(args.timeout_s),
        "samples_per_batch": int(args.samples_per_batch),
        "rewire_factor": float(args.rewire_factor),
        "use_k_nearest": int(args.use_k_nearest),
        "pruning": int(args.pruning),
        "prune_threshold_fraction": float(args.prune_threshold_fraction),
        "delay_rewiring_until_initial_solution": int(args.delay_rewiring_until_initial_solution),
        "just_in_time_sampling": int(args.just_in_time_sampling),
        "drop_samples_on_prune": int(args.drop_samples_on_prune),
        "approximate_solutions": int(args.approximate_solutions),
        "strict_queue_ordering": int(args.strict_queue_ordering),
        "cascading_rewirings": int(args.cascading_rewirings),
        "initial_inflation_factor": float(args.initial_inflation_factor),
        "inflation_scaling_parameter": float(args.inflation_scaling_parameter),
        "truncation_scaling_parameter": float(args.truncation_scaling_parameter),
        "allowed_failed_sampling_attempts": int(args.allowed_failed_sampling_attempts),
        "iterations": int(raw.get("iterations", 0) or 0),
        "batches": int(raw.get("batches", 0) or 0),
    }
    return row


def worker(args: argparse.Namespace) -> int:
    print(json.dumps(run_query_once(args), sort_keys=True), flush=True)
    return 0


def multiprocessing_worker(args: argparse.Namespace, queue: Any) -> None:
    try:
        queue.put({"ok": True, "row": run_query_once(args)})
    except BaseException:
        queue.put({"ok": False, "error": traceback.format_exc()})


def append_jsonl(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=True) + "\n")


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    seeds = sorted({int(row["seed"]) for row in rows})
    seed_success = []
    for seed in seeds:
        items = [row for row in rows if int(row["seed"]) == seed]
        seed_success.append(len(items) == 5 and all(bool(row.get("audit_passed")) for row in items))
    successes = [row for row in rows if bool(row.get("audit_passed"))]
    return {
        "method": "bitstar",
        "stage_id": rows[0].get("stage_id", "bitstar") if rows else "bitstar",
        "runs": len(seeds),
        "success_runs": sum(1 for item in seed_success if item),
        "query_rows": len(rows),
        "success_queries": len(successes),
        "planning_s_mean_per_run": (
            sum(float(row.get("planning_s", 0.0)) for row in rows) / max(1, len(seeds))
            if rows else math.nan
        ),
        "path_length_mean": (
            sum(float(row["path_length"]) for row in successes) / len(successes)
            if successes else math.nan
        ),
    }


def parent(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    jsonl_path = out_dir / "bitstar_per_query_rows.jsonl"
    if jsonl_path.exists() and not bool(args.append):
        jsonl_path.unlink()
    rows: list[dict[str, Any]] = []
    query_indices = [
        int(item.strip())
        for item in str(args.query_indices).split(",")
        if item.strip()
    ]
    for seed_text in str(args.seeds).split(","):
        if not seed_text.strip():
            continue
        seed = int(seed_text)
        for query_index in query_indices:
            worker_args = argparse.Namespace(**vars(args))
            worker_args.worker = True
            worker_args.seed = seed
            worker_args.query_index = query_index
            wall_timeout = max(5.0, float(args.timeout_s) * float(args.wall_timeout_factor) + 5.0)
            print(f"[bitstar-per-query] seed={seed} query={query_index} wall_timeout={wall_timeout:g}s", flush=True)
            context = mp.get_context("fork")
            queue = context.Queue(maxsize=1)
            process = context.Process(target=multiprocessing_worker, args=(worker_args, queue))
            process.start()
            process.join(wall_timeout)
            if process.is_alive():
                process.terminate()
                process.join(1.0)
                if process.is_alive():
                    process.kill()
                    process.join(1.0)
                row = {
                    "method": "bitstar",
                    "seed": seed,
                    "query_index": query_index,
                    "label": query_specs()[query_index]["label"],
                    "success": False,
                    "audit_passed": False,
                    "audit_status": "wall_timeout",
                    "planner_status": "wall_timeout",
                    "planning_s": float(wall_timeout),
                    "audit_s": 0.0,
                    "path_length": math.nan,
                    "timeout_s": float(args.timeout_s),
                    "samples_per_batch": int(args.samples_per_batch),
                    "rewire_factor": float(args.rewire_factor),
                    "use_k_nearest": int(args.use_k_nearest),
                    "pruning": int(args.pruning),
                    "prune_threshold_fraction": float(args.prune_threshold_fraction),
                    "delay_rewiring_until_initial_solution": int(args.delay_rewiring_until_initial_solution),
                    "just_in_time_sampling": int(args.just_in_time_sampling),
                    "drop_samples_on_prune": int(args.drop_samples_on_prune),
                    "approximate_solutions": int(args.approximate_solutions),
                    "strict_queue_ordering": int(args.strict_queue_ordering),
                    "cascading_rewirings": int(args.cascading_rewirings),
                    "initial_inflation_factor": float(args.initial_inflation_factor),
                    "inflation_scaling_parameter": float(args.inflation_scaling_parameter),
                    "truncation_scaling_parameter": float(args.truncation_scaling_parameter),
                    "allowed_failed_sampling_attempts": int(args.allowed_failed_sampling_attempts),
                }
            else:
                payload = queue.get() if not queue.empty() else {"ok": False, "error": "worker exited without result"}
                if bool(payload.get("ok")):
                    row = dict(payload.get("row", {}))
                else:
                    row = {
                        "method": "bitstar",
                        "seed": seed,
                        "query_index": query_index,
                        "label": query_specs()[query_index]["label"],
                        "success": False,
                        "audit_passed": False,
                        "audit_status": "worker_failed",
                        "planner_status": str(payload.get("error", ""))[-2000:],
                        "planning_s": math.nan,
                        "audit_s": 0.0,
                        "path_length": math.nan,
                        "timeout_s": float(args.timeout_s),
                        "samples_per_batch": int(args.samples_per_batch),
                        "rewire_factor": float(args.rewire_factor),
                        "use_k_nearest": int(args.use_k_nearest),
                        "pruning": int(args.pruning),
                        "prune_threshold_fraction": float(args.prune_threshold_fraction),
                        "delay_rewiring_until_initial_solution": int(args.delay_rewiring_until_initial_solution),
                        "just_in_time_sampling": int(args.just_in_time_sampling),
                        "drop_samples_on_prune": int(args.drop_samples_on_prune),
                        "approximate_solutions": int(args.approximate_solutions),
                        "strict_queue_ordering": int(args.strict_queue_ordering),
                        "cascading_rewirings": int(args.cascading_rewirings),
                        "initial_inflation_factor": float(args.initial_inflation_factor),
                        "inflation_scaling_parameter": float(args.inflation_scaling_parameter),
                        "truncation_scaling_parameter": float(args.truncation_scaling_parameter),
                        "allowed_failed_sampling_attempts": int(args.allowed_failed_sampling_attempts),
                    }
            row["stage_id"] = (
                f"t{float(args.timeout_s):g}s_batch{int(args.samples_per_batch)}_rw{float(args.rewire_factor):g}"
                f"_kn{int(args.use_k_nearest)}_delay{int(args.delay_rewiring_until_initial_solution)}"
                f"_jit{int(args.just_in_time_sampling)}_infl{float(args.initial_inflation_factor):g}"
            )
            append_jsonl(jsonl_path, row)
            rows.append(row)
    summary = summarize(rows)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_json(out_dir / "bitstar_per_query_manifest.json", {"rows": rows, "summary": summary})
    write_csv_rows(out_dir / "bitstar_per_query_summary.csv", [summary], sorted(summary))
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.5 BIT* per-query subprocess runner with outer watchdog.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp05_bitstar_per_query")
    parser.add_argument("--seeds", default="0")
    parser.add_argument("--query-indices", default="0,1,2,3,4")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--samples-per-batch", type=int, default=100)
    parser.add_argument("--rewire-factor", type=float, default=5.0)
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
    parser.add_argument("--wall-timeout-factor", type=float, default=1.8)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--simplify-time-s", type=float, default=DEFAULT_OMPL_SIMPLIFY_TIME_S)
    parser.add_argument("--stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--query-index", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return worker(args) if bool(args.worker) else parent(args)


if __name__ == "__main__":
    raise SystemExit(main())
