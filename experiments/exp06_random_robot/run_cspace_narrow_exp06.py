#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Any

import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.cspace_narrow_benchmark import CSpaceBox, flatten_box, l2, segment_valid
from experiments.common.experiment_io import csv_list, environment_metadata, namespace_dict, run_id, write_json
from experiments.common.progress import progress
from experiments.common.rbf_defaults import default_rbf_profile
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


def csv_floats(raw: str) -> list[float]:
    return [float(item) for item in csv_list(raw)]


def path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return math.nan
    return sum(l2(path[index], path[index + 1]) for index in range(len(path) - 1))


def load_catalog(path: Path) -> dict[str, Any]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if str(payload.get("schema")) != "tro2026_cspace_narrow_catalog_v2":
        raise ValueError(f"unsupported C-space catalog schema: {payload.get('schema')!r}")
    return payload


def obstacles_from_record(record: dict[str, Any]) -> list[CSpaceBox]:
    return [
        CSpaceBox([(float(lo), float(hi)) for lo, hi in obstacle["intervals"]])
        for obstacle in record.get("obstacles", [])
    ]


def flattened_obstacles(record: dict[str, Any]) -> list[list[float]]:
    if record.get("flattened_obstacles"):
        return [[float(value) for value in row] for row in record["flattened_obstacles"]]
    return [flatten_box(obstacle) for obstacle in obstacles_from_record(record)]


def audit_path(record: dict[str, Any], path: list[list[float]], step: float) -> tuple[bool, float, str]:
    t0 = time.perf_counter()
    if len(path) < 2:
        return False, time.perf_counter() - t0, "empty_path"
    obstacles = obstacles_from_record(record)
    for index in range(len(path) - 1):
        if not segment_valid(path[index], path[index + 1], obstacles, float(step)):
            return False, time.perf_counter() - t0, "collision"
    return True, time.perf_counter() - t0, "passed"


def method_time(row: dict[str, Any]) -> float:
    return float(row.get("planning_s", math.nan))


def make_query_row(
    *,
    method: str,
    record: dict[str, Any],
    seed: int,
    stage_id: str,
    budget_s: float,
    planner_result: dict[str, Any],
    audit_step: float,
    offline_build_s: float = 0.0,
    diagnostics: dict[str, Any] | None = None,
) -> dict[str, Any]:
    path = [[float(value) for value in point] for point in planner_result.get("path", [])]
    audit_ok, audit_s, audit_status = audit_path(record, path, audit_step)
    success = bool(planner_result.get("ok")) and bool(audit_ok)
    solve_s = float(planner_result.get("solve_s", planner_result.get("t_s", 0.0)) or 0.0)
    simplify_s = float(planner_result.get("simplify_s", 0.0) or 0.0)
    online_s = float(planner_result.get("t_s", solve_s + simplify_s) or 0.0)
    return {
        "method": method,
        "robot": str(record["robot"]),
        "difficulty": str(record["difficulty"]),
        "scene_seed": int(seed),
        "stage_id": str(stage_id),
        "budget_s": float(budget_s),
        "success_count": 1 if success else 0,
        "query_count": 1,
        "planning_s": float(offline_build_s) + online_s,
        "offline_build_s": float(offline_build_s),
        "online_s": online_s,
        "online_solve_s": solve_s,
        "online_simplify_s": simplify_s,
        "audit_s": audit_s,
        "path_length_mean": path_length(path) if success else math.nan,
        "raw_segment_fraction": 0.0 if success else math.nan,
        "final_boxes": math.nan,
        "obstacle_count": len(record.get("obstacles", [])),
        "gate_dims": ",".join(str(value) for value in record.get("gate_dims", [])),
        "gate_width": float(record.get("gate_width", math.nan)),
        "wall_half_width": float(record.get("wall_half_width", math.nan)),
        "status": "executed",
        "diagnostics": diagnostics or {},
    }


def run_rrt(record: dict[str, Any], seed: int, args: argparse.Namespace) -> dict[str, Any]:
    result = sbf.ompl_cspace_rrt_connect_path(
        [[float(lo), float(hi)] for lo, hi in record["limits"]],
        flattened_obstacles(record),
        [float(value) for value in record["start"]],
        [float(value) for value in record["goal"]],
        float(args.rrt_timeout_s) * 1000.0,
        float(args.rrt_range),
        float(args.audit_segment_step),
        float(args.ompl_simplify_time_s),
        int(seed),
    )
    return make_query_row(
        method="rrtconnect",
        record=record,
        seed=seed,
        stage_id=f"timeout{float(args.rrt_timeout_s):g}s",
        budget_s=float(args.rrt_timeout_s),
        planner_result=dict(result),
        audit_step=float(args.audit_segment_step),
        diagnostics={"planner": "OMPL_CSpace_RRTConnect", "range": float(args.rrt_range)},
    )


def run_prm(record: dict[str, Any], seed: int, build_s: float, args: argparse.Namespace) -> dict[str, Any]:
    result = dict(sbf.ompl_cspace_prm_multiquery(
        [[float(lo), float(hi)] for lo, hi in record["limits"]],
        flattened_obstacles(record),
        [[float(value) for value in record["start"]]],
        [[float(value) for value in record["goal"]]],
        float(build_s),
        float(args.prm_query_s),
        float(args.audit_segment_step),
        float(args.ompl_simplify_time_s),
        int(seed),
        int(args.prm_max_nearest_neighbors),
        str(args.prm_planner_kind),
        bool(args.prm_preload_query_endpoints),
    ))
    queries = [dict(item) for item in result.get("queries", [])]
    qresult = queries[0] if queries else {"ok": False, "status": "no_query", "path": []}
    return make_query_row(
        method="prm",
        record=record,
        seed=seed,
        stage_id=(
            f"{str(args.prm_planner_kind)}_build{float(build_s):g}s"
            f"_k{int(args.prm_max_nearest_neighbors)}"
            f"_q{float(args.prm_query_s):g}s"
            f"_preload{int(bool(args.prm_preload_query_endpoints))}"
        ),
        budget_s=float(build_s),
        planner_result=qresult,
        audit_step=float(args.audit_segment_step),
        offline_build_s=float(result.get("build_s", 0.0) or 0.0),
        diagnostics={
            "planner": result.get("planner", "OMPL_CSpace_PRM"),
            "nodes": int(result.get("nodes", 0) or 0),
            "query_budget_s": float(args.prm_query_s),
            "preload_query_endpoints": bool(args.prm_preload_query_endpoints),
        },
    )


def checkpoint_at_or_after(checkpoints: list[dict[str, Any]], target_s: float) -> dict[str, Any]:
    if not checkpoints:
        return {}
    for checkpoint in checkpoints:
        if float(checkpoint.get("checkpoint_s", 0.0) or 0.0) >= float(target_s) - 1e-9:
            return checkpoint
    return checkpoints[-1]


def run_bitstar_trace(record: dict[str, Any], seed: int, args: argparse.Namespace, stages: list[float]) -> list[dict[str, Any]]:
    result = dict(sbf.ompl_cspace_bitstar_trace(
        [[float(lo), float(hi)] for lo, hi in record["limits"]],
        flattened_obstacles(record),
        [float(value) for value in record["start"]],
        [float(value) for value in record["goal"]],
        float(args.bitstar_timeout_s) * 1000.0,
        float(args.bitstar_checkpoint_interval_s) * 1000.0,
        float(args.audit_segment_step),
        int(seed),
        int(args.bitstar_samples_per_batch),
        float(args.bitstar_rewire_factor),
        False,
    ))
    checkpoints = [dict(item) for item in result.get("checkpoints", [])]
    rows: list[dict[str, Any]] = []
    best: dict[str, Any] | None = None
    for stage_s in stages:
        checkpoint = checkpoint_at_or_after(checkpoints, float(stage_s))
        path = [[float(value) for value in point] for point in checkpoint.get("path", [])]
        audit_ok, audit_s, audit_status = audit_path(record, path, float(args.audit_segment_step))
        ok = bool(checkpoint.get("ok")) and bool(audit_ok)
        current_length = path_length(path) if ok else math.nan
        current = {
            "ok": ok,
            "path": path,
            "solve_s": float(checkpoint.get("solve_s", checkpoint.get("elapsed_s", stage_s)) or 0.0),
            "simplify_s": 0.0,
            "t_s": float(checkpoint.get("t_s", checkpoint.get("elapsed_s", stage_s)) or 0.0),
            "status": checkpoint.get("status", "unknown"),
            "path_length": current_length,
        }
        if ok and math.isfinite(current_length) and (best is None or current_length < float(best.get("path_length", math.inf))):
            best = dict(current)
        effective = best if best is not None else current
        row = make_query_row(
            method="bitstar",
            record=record,
            seed=seed,
            stage_id=f"batch{int(args.bitstar_samples_per_batch)}_rw{float(args.bitstar_rewire_factor):g}_trace{float(args.bitstar_timeout_s):g}s_t{float(stage_s):g}s",
            budget_s=float(stage_s),
            planner_result=effective,
            audit_step=float(args.audit_segment_step),
            diagnostics={
                "planner": "OMPL_CSpace_BITstar_trace",
                "timeout_s": float(args.bitstar_timeout_s),
                "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
                "target_checkpoint_s": float(stage_s),
                "audit_status": audit_status,
                "audit_s": audit_s,
            },
        )
        rows.append(row)
    return rows


def median(values: list[float]) -> float:
    clean = sorted(value for value in values if math.isfinite(float(value)))
    if not clean:
        return math.nan
    mid = len(clean) // 2
    if len(clean) % 2:
        return clean[mid]
    return 0.5 * (clean[mid - 1] + clean[mid])


def mean(values: list[float]) -> float:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    return sum(clean) / len(clean) if clean else math.nan


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    keys = sorted({(row["method"], row["robot"], row["difficulty"], row["stage_id"]) for row in rows})
    out: list[dict[str, Any]] = []
    for method, robot, difficulty, stage_id in keys:
        items = [
            row for row in rows
            if row["method"] == method
            and row["robot"] == robot
            and row["difficulty"] == difficulty
            and row["stage_id"] == stage_id
        ]
        success_items = [row for row in items if int(row.get("success_count", 0)) == int(row.get("query_count", 1))]
        out.append({
            "method": method,
            "robot": robot,
            "difficulty": difficulty,
            "stage_id": stage_id,
            "budget_s": median([float(row.get("budget_s", math.nan)) for row in items]),
            "scenes": len(items),
            "success_scenes": len(success_items),
            "success_queries": sum(int(row.get("success_count", 0)) for row in items),
            "total_queries": sum(int(row.get("query_count", 0)) for row in items),
            "planning_s_median": median([method_time(row) for row in items]),
            "planning_s_mean": mean([method_time(row) for row in items]),
            "offline_build_s_median": median([float(row.get("offline_build_s", 0.0)) for row in items]),
            "online_s_median": median([float(row.get("online_s", math.nan)) for row in items]),
            "audit_s_median": median([float(row.get("audit_s", math.nan)) for row in items]),
            "path_length_mean": mean([float(row.get("path_length_mean", math.nan)) for row in success_items]),
            "raw_segment_fraction_median": median([float(row.get("raw_segment_fraction", math.nan)) for row in success_items]),
            "gate_width_median": median([float(row.get("gate_width", math.nan)) for row in items]),
            "status": "executed",
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.6 C-space narrow-passage OMPL baseline runner.")
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--methods", default="rrtconnect,prm,bitstar")
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--robots", default="")
    parser.add_argument("--difficulties", default="")
    parser.add_argument("--audit-segment-step", type=float, default=0.01)
    parser.add_argument("--ompl-simplify-time-s", type=float, default=0.01)
    parser.add_argument("--rrt-timeout-s", type=float, default=1.0)
    parser.add_argument("--rrt-range", type=float, default=0.35)
    parser.add_argument("--prm-build-grid-s", default="0.05,0.1,0.2,0.5,1,2,5")
    parser.add_argument("--prm-query-s", type=float, default=4.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-planner-kind", default="prm")
    parser.add_argument("--prm-preload-query-endpoints", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bitstar-timeout-s", type=float, default=5.0)
    parser.add_argument("--bitstar-checkpoint-grid-s", default="0.02,0.04,0.06,0.08,0.1,0.12,0.16,0.2,0.3,0.5,0.75,1,1.5,2,3,4,5")
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=0.02)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    catalog = load_catalog(args.catalog)
    methods = set(csv_list(args.methods))
    robots = set(csv_list(args.robots)) if str(args.robots).strip() else None
    difficulties = set(csv_list(args.difficulties)) if str(args.difficulties).strip() else None
    records = [
        dict(record) for record in catalog.get("records", [])
        if (robots is None or str(record.get("robot")) in robots)
        and (difficulties is None or str(record.get("difficulty")) in difficulties)
    ]
    seeds = list(range(max(1, int(args.seeds))))
    prm_grid = csv_floats(args.prm_build_grid_s)
    bitstar_grid = [value for value in csv_floats(args.bitstar_checkpoint_grid_s) if value <= float(args.bitstar_timeout_s) + 1e-9]
    if not bitstar_grid or abs(bitstar_grid[-1] - float(args.bitstar_timeout_s)) > 1e-9:
        bitstar_grid.append(float(args.bitstar_timeout_s))

    planned: list[tuple[str, dict[str, Any], int, float | None]] = []
    for record in records:
        for seed in seeds:
            if "rrtconnect" in methods:
                planned.append(("rrtconnect", record, seed, None))
            if "prm" in methods:
                for budget in prm_grid:
                    planned.append(("prm", record, seed, float(budget)))
            if "bitstar" in methods:
                planned.append(("bitstar", record, seed, None))

    if args.dry_run:
        print({"planned": len(planned), "records": len(records), "seeds": len(seeds), "methods": sorted(methods)})
        return 0

    rows: list[dict[str, Any]] = []
    for method, record, seed, budget in progress(planned, total=len(planned), desc="exp06 cspace"):
        if method == "rrtconnect":
            rows.append(run_rrt(record, seed, args))
        elif method == "prm":
            rows.append(run_prm(record, seed, float(budget), args))
        elif method == "bitstar":
            rows.extend(run_bitstar_trace(record, seed, args, bitstar_grid))
        else:
            raise ValueError(f"unsupported method {method!r}")

    summary = aggregate(rows)
    unsupported = [
        {
            "method": "sbf_leaf_rrt",
            "status": "unsupported_cspace_obstacles",
            "reason": "Current RBF/LECT collision oracle certifies robot-link envelopes against workspace AABB obstacles; this C-space catalog uses synthetic joint-space AABB obstacles.",
            "inherited_rbf_profile": default_rbf_profile(),
        },
        {
            "method": "iris_np_gcs",
            "status": "unsupported_cspace_obstacles",
            "reason": "Current IRIS/GCS runner builds Drake workspace collision constraints; this C-space catalog uses synthetic joint-space AABB obstacles.",
        },
    ]
    manifest = {
        "run_id": run_id("exp06_cspace_narrow"),
        "experiment": "exp06_cspace_narrow",
        "catalog": str(args.catalog),
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "rbf_profile_inherited_from_exp04_exp05": default_rbf_profile(),
        "unsupported_methods": unsupported,
        "rows": rows,
        "summary": summary,
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.out_dir / "cspace_narrow_manifest.json", manifest)
    write_csv(args.out_dir / "cspace_narrow_rows.csv", rows)
    write_csv(args.out_dir / "cspace_narrow_summary.csv", summary)
    print({
        "out_dir": str(args.out_dir),
        "rows": len(rows),
        "summary_rows": len(summary),
        "unsupported_methods": [row["method"] for row in unsupported],
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
