#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import sys
import time
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    environment_metadata,
    namespace_dict,
    run_id,
    write_json,
)
from experiments.common.metrics import median, percentile  # noqa: E402
from experiments.common.progress import progress  # noqa: E402
from experiments.common.random_scene_catalog import (  # noqa: E402
    make_robot,
    normalize_obstacles,
    obstacle_from_bounds,
    sample_free_pair,
)
from experiments.exp07_dynamic_update import run_dynamic_update as dyn  # noqa: E402


CATALOG_SCHEMA = "tro2026_exp07_update_replan_diagnostic_v1"


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def set_thread_env(threads: int) -> None:
    value = str(max(1, int(threads)))
    for key in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
    ):
        os.environ[key] = value


def finite(value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def q1(values: Iterable[Any]) -> float | None:
    return percentile(values, 0.25)


def q3(values: Iterable[Any]) -> float | None:
    return percentile(values, 0.75)


def fmt(value: Any, digits: int = 3) -> str:
    x = finite(value)
    return "--" if not math.isfinite(x) else f"{x:.{digits}f}"


def interval(values: Iterable[Any], digits: int = 3) -> str:
    vals = [finite(value) for value in values]
    vals = [value for value in vals if math.isfinite(value)]
    if not vals:
        return "--"
    return f"{fmt(median(vals), digits)} [{fmt(q1(vals), digits)}, {fmt(q3(vals), digits)}]"


def query_payload(phase: str, query_index: int, start: list[float], goal: list[float], result: Any) -> dict[str, Any]:
    success = bool(getattr(result, "success", False))
    query_ms = float(getattr(result, "query_time_ms", 0.0))
    simplify_ms = float(getattr(result, "final_simplify_time_ms", 0.0))
    path_length = float(getattr(result, "path_length", math.nan)) if success else math.nan
    raw_path_length = float(getattr(result, "raw_path_length", path_length)) if success else math.nan
    segment_length = float(getattr(result, "segment_edge_length", 0.0)) if success else 0.0
    return {
        "phase": phase,
        "query_index": int(query_index),
        "success": success,
        "audit_passed": bool(getattr(result, "audit_passed", False)),
        "query_s": query_ms / 1000.0,
        "solve_s": max(0.0, query_ms - simplify_ms) / 1000.0,
        "simplify_s": simplify_ms / 1000.0,
        "audit_s": float(getattr(result, "audit_time_ms", 0.0)) / 1000.0,
        "audit_complete_s": query_ms / 1000.0 + float(getattr(result, "audit_time_ms", 0.0)) / 1000.0,
        "path_length": path_length,
        "raw_path_length": raw_path_length,
        "segment_edge_length": segment_length,
        "segment_fraction": (segment_length / raw_path_length) if success and raw_path_length > 1e-12 else math.nan,
        "segment_edges_used": int(getattr(result, "segment_edges_used", 0)),
        "box_sequence_len": len(list(getattr(result, "box_sequence", []))),
        "waypoint_count": len(result.path_as_lists()) if hasattr(result, "path_as_lists") else 0,
        "audit_status": str(getattr(result, "audit_status", "")),
        "start": start,
        "goal": goal,
    }


def run_queries(forest: Any, phase: str, queries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query_index, query in enumerate(queries):
        start = [float(value) for value in query["start"]]
        goal = [float(value) for value in query["goal"]]
        previous_active_query = os.environ.get("RBF_ACTIVE_QUERY_INDEX")
        os.environ["RBF_ACTIVE_QUERY_INDEX"] = str(query_index)
        try:
            result = forest.query(start, goal)
        finally:
            if previous_active_query is None:
                os.environ.pop("RBF_ACTIVE_QUERY_INDEX", None)
            else:
                os.environ["RBF_ACTIVE_QUERY_INDEX"] = previous_active_query
        rows.append(query_payload(phase, query_index, start, goal, result))
    return rows


def obstacle_records_sha(records: list[dict[str, Any]]) -> str:
    text = json.dumps(records, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(text).hexdigest()


def sample_queries_for_record(args: argparse.Namespace, record: dict[str, Any]) -> list[dict[str, Any]]:
    robot_name = str(record["robot"])
    seed = int(record["seed"])
    robot = make_robot(robot_name)
    obstacles = normalize_obstacles(obstacle_from_bounds(bounds) for bounds in record["obstacles"])
    rng = random.Random(int(args.query_seed_base) + 1009 * seed + 7919 * len(robot_name))
    queries: list[dict[str, Any]] = []
    for query_index in range(int(args.queries_per_scene)):
        start, goal = sample_free_pair(
            robot,
            obstacles[: int(record["max_obstacles"])],
            rng,
            min_l2=float(args.query_min_l2),
            max_l2=float(args.query_max_l2),
            max_tries=int(args.query_max_tries),
            canonical=False,
        )
        queries.append({
            "label": f"dyn{query_index}",
            "start": [float(value) for value in start],
            "goal": [float(value) for value in goal],
        })
    return queries


def load_or_create_catalog(args: argparse.Namespace) -> dict[str, Any]:
    path = Path(args.query_catalog or (args.out_dir / "update_replan_catalog.json"))
    if str(args.query_catalog_mode) == "reuse":
        if not path.exists():
            raise FileNotFoundError(path)
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("schema") != CATALOG_SCHEMA:
            raise RuntimeError(f"catalog schema mismatch: {payload.get('schema')} != {CATALOG_SCHEMA}")
        return payload

    obstacle_payload = dyn.load_or_create_catalog(args)
    records: list[dict[str, Any]] = []
    for record in obstacle_payload.get("records", []):
        copied = dict(record)
        copied["queries"] = sample_queries_for_record(args, copied)
        records.append(copied)
    payload = {
        "schema": CATALOG_SCHEMA,
        "source_obstacle_schema": obstacle_payload.get("schema"),
        "source_obstacle_sha256": obstacle_payload.get("obstacle_sha256"),
        "query_seed_base": int(args.query_seed_base),
        "queries_per_scene": int(args.queries_per_scene),
        "records": records,
    }
    payload["catalog_sha256"] = obstacle_records_sha(records)
    write_json(path, payload)
    return payload


def timed_profile_s(callable_obj: Any) -> tuple[Any, float]:
    start = time.perf_counter()
    profile = callable_obj()
    return profile, time.perf_counter() - start


def run_record(args: argparse.Namespace, record: dict[str, Any]) -> dict[str, Any]:
    robot_name = str(record["robot"])
    seed = int(record["seed"])
    robot = make_robot(robot_name)
    obstacles = normalize_obstacles(obstacle_from_bounds(bounds) for bounds in record["obstacles"])
    min_count = int(record.get("min_obstacles", args.min_obstacles))
    max_count = int(record.get("max_obstacles", args.max_obstacles))
    queries = list(record.get("queries", []))

    forest, opt = dyn.make_forest(args, robot, robot_name, seed, f"exp07_replan_{robot_name}_seed{seed}_incremental")
    initial_build = dyn.build_adaptive(args, forest, obstacles[:min_count], opt)
    initial_queries = run_queries(forest, "source_initial", queries)

    insert_profile, insert_wall_s = timed_profile_s(lambda: forest.add_obstacles_and_rebuild(obstacles[min_count:max_count]))
    insert_profile_row = dyn.profile_row(insert_profile)
    insert_queries = run_queries(forest, "after_insert_update", queries)

    target_forest, target_opt = dyn.make_forest(args, robot, robot_name, seed, f"exp07_replan_{robot_name}_seed{seed}_target_rebuild")
    target_build = dyn.build_adaptive(args, target_forest, obstacles[:max_count], target_opt)
    target_rebuild_queries = run_queries(target_forest, "target_fresh_rebuild", queries)

    remove_profile, remove_wall_s = timed_profile_s(lambda: forest.remove_obstacle_suffix_and_regrow(min_count))
    remove_profile_row = dyn.profile_row(remove_profile)
    remove_queries = run_queries(forest, "after_remove_update", queries)

    return {
        "robot": robot_name,
        "seed": seed,
        "min_obstacles": min_count,
        "max_obstacles": max_count,
        "query_count": len(queries),
        "initial_build_s": initial_build["wall_s"],
        "initial_build_boxes": initial_build["forest_boxes"],
        "initial_build_segment_edges": initial_build["forest_segment_edges"],
        "target_build_s": target_build["wall_s"],
        "target_build_boxes": target_build["forest_boxes"],
        "target_build_segment_edges": target_build["forest_segment_edges"],
        "insert_update_s": insert_wall_s,
        "insert_update_profile_s": insert_profile_row["total_ms"] / 1000.0,
        "insert_boxes_after": insert_profile_row["boxes_after"],
        "insert_segment_edges_added": insert_profile_row["segment_edges_added"],
        "insert_update": insert_profile_row,
        "remove_update_s": remove_wall_s,
        "remove_update_profile_s": remove_profile_row["total_ms"] / 1000.0,
        "remove_boxes_after": remove_profile_row["boxes_after"],
        "remove_segment_edges_added": remove_profile_row["segment_edges_added"],
        "remove_update": remove_profile_row,
        "source_initial_queries": initial_queries,
        "after_insert_update_queries": insert_queries,
        "target_fresh_rebuild_queries": target_rebuild_queries,
        "after_remove_update_queries": remove_queries,
    }


def flatten_query_rows(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for record in records:
        for key in (
            "source_initial_queries",
            "after_insert_update_queries",
            "target_fresh_rebuild_queries",
            "after_remove_update_queries",
        ):
            for query in record.get(key, []):
                row = {
                    "robot": record["robot"],
                    "seed": record["seed"],
                    "min_obstacles": record["min_obstacles"],
                    "max_obstacles": record["max_obstacles"],
                }
                row.update({k: v for k, v in query.items() if k not in {"start", "goal"}})
                rows.append(row)
    return rows


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    query_rows = flatten_query_rows(records)
    phase_rows = {
        phase: [row for row in query_rows if row.get("phase") == phase]
        for phase in sorted({str(row.get("phase")) for row in query_rows})
    }
    phases: dict[str, Any] = {}
    for phase, rows in phase_rows.items():
        phases[phase] = {
            "n": len(rows),
            "success": sum(1 for row in rows if bool(row.get("success"))),
            "audit_passed": sum(1 for row in rows if bool(row.get("audit_passed"))),
            "query_s_median": median(row.get("query_s") for row in rows),
            "query_s_q1": q1(row.get("query_s") for row in rows),
            "query_s_q3": q3(row.get("query_s") for row in rows),
            "audit_s_median": median(row.get("audit_s") for row in rows),
            "audit_s_q1": q1(row.get("audit_s") for row in rows),
            "audit_s_q3": q3(row.get("audit_s") for row in rows),
            "segment_fraction_median": median(row.get("segment_fraction") for row in rows),
        }
    insert_totals = []
    rebuild_totals = []
    remove_totals = []
    for record in records:
        insert_query_median = median(row.get("query_s") for row in record.get("after_insert_update_queries", [])) or 0.0
        rebuild_query_median = median(row.get("query_s") for row in record.get("target_fresh_rebuild_queries", [])) or 0.0
        remove_query_median = median(row.get("query_s") for row in record.get("after_remove_update_queries", [])) or 0.0
        insert_totals.append(float(record.get("insert_update_s", 0.0)) + insert_query_median)
        rebuild_totals.append(float(record.get("target_build_s", 0.0)) + rebuild_query_median)
        remove_totals.append(float(record.get("remove_update_s", 0.0)) + remove_query_median)
    return {
        "records": len(records),
        "queries": len(query_rows),
        "initial_build_s": interval(record.get("initial_build_s") for record in records),
        "initial_build_boxes": interval((record.get("initial_build_boxes") for record in records), digits=0),
        "target_build_s": interval(record.get("target_build_s") for record in records),
        "target_build_boxes": interval((record.get("target_build_boxes") for record in records), digits=0),
        "insert_update_s": interval(record.get("insert_update_s") for record in records),
        "insert_boxes_after": interval((record.get("insert_boxes_after") for record in records), digits=0),
        "remove_update_s": interval(record.get("remove_update_s") for record in records),
        "remove_boxes_after": interval((record.get("remove_boxes_after") for record in records), digits=0),
        "insert_update_plus_query_s": interval(insert_totals),
        "target_rebuild_plus_query_s": interval(rebuild_totals),
        "remove_update_plus_query_s": interval(remove_totals),
        "phases": phases,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary_md(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# Exp.7 update-and-replan diagnostic",
        "",
        "| Metric | median [Q1, Q3] |",
        "|---|---:|",
        f"| Initial source build | {summary['initial_build_s']} |",
        f"| Initial source boxes | {summary['initial_build_boxes']} |",
        f"| Insert update | {summary['insert_update_s']} |",
        f"| Boxes after insert update | {summary['insert_boxes_after']} |",
        f"| Insert update + post-update query | {summary['insert_update_plus_query_s']} |",
        f"| Target fresh rebuild | {summary['target_build_s']} |",
        f"| Target fresh-rebuild boxes | {summary['target_build_boxes']} |",
        f"| Target rebuild + query | {summary['target_rebuild_plus_query_s']} |",
        f"| Remove update | {summary['remove_update_s']} |",
        f"| Boxes after remove update | {summary['remove_boxes_after']} |",
        f"| Remove update + post-update query | {summary['remove_update_plus_query_s']} |",
        "",
        "| Phase | audit success | query s | audit s | seg. frac. median |",
        "|---|---:|---:|---:|---:|",
    ]
    for phase, row in summary.get("phases", {}).items():
        lines.append(
            f"| {phase} | {row['audit_passed']}/{row['n']} | "
            f"{interval([row.get('query_s_median'), row.get('query_s_q1'), row.get('query_s_q3')])} | "
            f"{interval([row.get('audit_s_median'), row.get('audit_s_q1'), row.get('audit_s_q3')])} | "
            f"{fmt(row.get('segment_fraction_median'), 3)} |"
        )
    lines.extend([
        "",
        "Scope: diagnostic only. Queries are sampled free in the target scene; the runner compares update+query against a fresh target rebuild+query under the current Exp.7 adaptive profile.",
        "",
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.7 update + post-update query diagnostic.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp07_update_replan_diagnostic")
    parser.add_argument("--phase", choices=["smoke", "paper"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    parser.add_argument("--query-catalog", type=Path, default=None)
    parser.add_argument("--query-catalog-mode", choices=["auto", "reuse"], default="auto")
    parser.add_argument("--robots", default="iiwa")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--seed-base", type=int, default=9207)
    parser.add_argument("--query-seed-base", type=int, default=20260613)
    parser.add_argument("--queries-per-scene", type=int, default=2)
    parser.add_argument("--query-min-l2", type=float, default=0.8)
    parser.add_argument("--query-max-l2", type=float, default=4.0)
    parser.add_argument("--query-max-tries", type=int, default=3000)
    parser.add_argument("--min-obstacles", type=int, default=10)
    parser.add_argument("--max-obstacles", type=int, default=15)
    parser.add_argument("--obstacle-profile", choices=["random", "narrow", "mixed"], default="random")
    parser.add_argument("--obstacle-scale", type=float, default=0.12)
    parser.add_argument("--min-obstacle-separation", type=float, default=0.01)
    parser.add_argument("--max-obstacle-tries", type=int, default=10000)
    parser.add_argument("--deep-max-boxes", type=int, default=dyn.DEFAULT_EXP07_BOX_BUDGET)
    parser.add_argument("--rbf-max-depth", type=int, default=dyn.DEFAULT_RBF_MAX_DEPTH)
    parser.add_argument("--leaf-start-depth", type=int, default=dyn.DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--leaf-max-depth", type=int, default=dyn.DEFAULT_LEAF_MAX_DEPTH)
    parser.add_argument("--adaptive-target-depth", type=int, default=dyn.DEFAULT_ADAPTIVE_TARGET_DEPTH)
    parser.add_argument("--deep-ffb-depth", type=int, default=dyn.DEFAULT_RBF_DEEP_FFB_DEPTH)
    parser.add_argument("--ffb-start-depth", type=int, default=16)
    parser.add_argument("--ffb-search-mode", default=dyn.DEFAULT_RBF_FFB_SEARCH_MODE)
    parser.add_argument("--adaptive-time-budget-ms", type=float, default=60000.0)
    parser.add_argument("--adaptive-node-budget", type=int, default=0)
    parser.add_argument("--adaptive-defer-min-depth", type=int, default=16)
    parser.add_argument("--adaptive-overlap-depth-threshold", type=float, default=0.05)
    parser.add_argument("--adaptive-overlap-depth-min-threshold", type=float, default=0.01)
    parser.add_argument("--adaptive-overlap-depth-decay-per-depth", type=float, default=0.002)
    parser.add_argument("--adaptive-seed-probe-count", type=int, default=1024)
    parser.add_argument("--adaptive-seed-anchor-probe-cap", type=int, default=64)
    parser.add_argument("--adaptive-depth-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--adaptive-depth-min", type=int, default=10)
    parser.add_argument("--adaptive-depth-probe-count", type=int, default=1024)
    parser.add_argument("--adaptive-depth-anchor-probe-cap", type=int, default=64)
    parser.add_argument("--adaptive-depth-probe-seed", type=int, default=20260607)
    parser.add_argument("--adaptive-depth-min-free-probes", type=int, default=64)
    parser.add_argument("--adaptive-depth-min-covered-probes", type=int, default=0)
    parser.add_argument("--adaptive-depth-min-main-probes", type=int, default=0)
    parser.add_argument("--adaptive-depth-min-main-ratio", type=float, default=0.0)
    parser.add_argument("--adaptive-depth-min-cells", type=int, default=200)
    parser.add_argument("--adaptive-depth-min-main-cells", type=int, default=1)
    parser.add_argument("--adaptive-depth-max-online-cells", type=int, default=320)
    parser.add_argument("--adaptive-depth-max-probe-ms", type=float, default=10.0)
    parser.add_argument("--adaptive-fast-virtual-checkpoint-mode", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--adaptive-max-merge-ms", type=float, default=1500.0)
    parser.add_argument("--adaptive-max-merge-rounds", type=int, default=2)
    parser.add_argument("--adaptive-max-merge-input-boxes", type=int, default=20000)
    parser.add_argument("--validation-batch-size", type=int, default=512)
    parser.add_argument("--enable-merger", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--use-virtual-topology", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dirty-region-padding", type=float, default=0.0)
    parser.add_argument("--local-regrow-box-limit", type=int, default=dyn.DEFAULT_EXP07_LOCAL_REGROW_BOX_LIMIT)
    parser.add_argument("--local-regrow-timeout-ms", type=float, default=1000.0)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--lect-cache-root", type=Path, default=dyn.ROBOT_LECTDB_CACHE_ROOT)
    parser.add_argument("--skip-lect-cache-ensure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.phase == "smoke":
        args.seeds = ",".join(str(seed) for seed in parse_int_list(args.seeds)[:1])
        args.queries_per_scene = min(int(args.queries_per_scene), 1)
    set_thread_env(int(args.threads))

    if args.dry_run:
        records: list[dict[str, Any]] = []
        catalog_payload: dict[str, Any] | None = None
    else:
        catalog_payload = load_or_create_catalog(args)
        records = list(catalog_payload.get("records", []))

    if not args.skip_lect_cache_ensure:
        for robot_name in progress([item for item in str(args.robots).split(",") if item], desc="exp07 replan cache"):
            dyn.ensure_robot_lectdb_cache(
                robot_name,
                cache_root=Path(args.lect_cache_root),
                threads=int(args.threads),
                dry_run=bool(args.dry_run),
            )

    run_rows: list[dict[str, Any]] = []
    if not args.dry_run:
        for record in progress(records, desc="exp07 update+query", total=len(records)):
            print(
                f"[exp07-replan] robot={record['robot']} seed={record['seed']} "
                f"{record['min_obstacles']}->{record['max_obstacles']} q={len(record.get('queries', []))}",
                flush=True,
            )
            run_rows.append(run_record(args, record))

    query_rows = flatten_query_rows(run_rows)
    summary = summarize(run_rows) if run_rows else {}
    payload = {
        "experiment": "exp07_update_replan_diagnostic",
        "run_id": run_id("exp07_update_replan"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "args": namespace_dict(args),
        "environment": environment_metadata(),
        "catalog": None if catalog_payload is None else {
            "schema": catalog_payload.get("schema"),
            "records": len(catalog_payload.get("records", [])),
            "catalog_sha256": catalog_payload.get("catalog_sha256"),
            "source_obstacle_sha256": catalog_payload.get("source_obstacle_sha256"),
        },
        "scope": "diagnostic update + post-update query/audit; not integrated into main dynamic-update table",
        "rows": run_rows,
        "query_rows": query_rows,
        "summary": summary,
    }
    write_json(args.out_dir / "update_replan_diagnostic.json", payload)
    if query_rows:
        write_csv(args.out_dir / "update_replan_queries.csv", query_rows)
    if run_rows:
        compact_rows = [
            {
                "robot": row["robot"],
                "seed": row["seed"],
                "min_obstacles": row["min_obstacles"],
                "max_obstacles": row["max_obstacles"],
                "query_count": row["query_count"],
                "initial_build_s": row["initial_build_s"],
                "initial_build_boxes": row["initial_build_boxes"],
                "insert_update_s": row["insert_update_s"],
                "insert_boxes_after": row["insert_boxes_after"],
                "target_build_s": row["target_build_s"],
                "target_build_boxes": row["target_build_boxes"],
                "remove_update_s": row["remove_update_s"],
                "remove_boxes_after": row["remove_boxes_after"],
            }
            for row in run_rows
        ]
        write_csv(args.out_dir / "update_replan_runs.csv", compact_rows)
        write_summary_md(args.out_dir / "update_replan_summary.md", summary)
    print(f"wrote {args.out_dir / 'update_replan_diagnostic.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
