#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median, tex_num
from experiments.common.progress import progress
from experiments.common.rbf_defaults import (
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_DEEP_MAX_BOXES,
    rbf_budget_grid,
    shelf_d23_rbf_profile,
)
from experiments.common.rbf_leaf_rrt import RBFLeafRRTOptions, canonical_q, run_leaf_rrt
from experiments.common.sbf_import import import_sbf
from experiments.exp05_shelf_cross_algorithm.import_old_shelf_baselines import import_old_baselines


sbf = import_sbf()
METHODS = ["sbf_leaf_rrt", "rrtconnect", "prm", "bitstar", "iris_np_gcs"]
IMPORTED_BASELINE_METHODS = {"iris_np_gcs"}
METHOD_LABELS = {
    "sbf_leaf_rrt": "RBF",
    "iris_np_gcs": "IRIS-NP+GCS",
    "prm": "PRM",
    "rrtconnect": "RRTConnect",
    "bitstar": "BIT*",
}


def csv_items(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def path_length(path: Iterable[Iterable[float]]) -> float:
    pts = [[float(value) for value in point] for point in path]
    if len(pts) < 2:
        return math.nan
    total = 0.0
    for a, b in zip(pts, pts[1:]):
        total += math.sqrt(sum((x - y) * (x - y) for x, y in zip(a, b)))
    return total


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def audit_path(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    segment_step: float,
    *,
    start: list[float] | None = None,
    goal: list[float] | None = None,
    endpoint_tol: float = 1e-6,
) -> tuple[bool, float, str]:
    t0 = time.perf_counter()
    if len(path) < 2:
        return False, time.perf_counter() - t0, "empty_path"
    if start is not None and point_distance(path[0], list(start)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "start_mismatch"
    if goal is not None and point_distance(path[-1], list(goal)) > float(endpoint_tol):
        return False, time.perf_counter() - t0, "goal_mismatch"
    step = max(1e-9, float(segment_step))
    for a, b in zip(path, path[1:]):
        distance = math.sqrt(sum((x - y) * (x - y) for x, y in zip(a, b)))
        steps = max(1, int(math.ceil(distance / step)))
        for index in range(steps + 1):
            if sbf.check_config_collision(robot, obstacles, interpolate(a, b, index / steps)):
                return False, time.perf_counter() - t0, "collision"
    return True, time.perf_counter() - t0, "passed"


def shelf_queries(robot: Any) -> list[dict[str, Any]]:
    queries = []
    for query in list(sbf.make_combined_queries()):
        start = [float(value) for value in query.start]
        goal = [float(value) for value in query.goal]
        queries.append({
            "label": str(query.label),
            "start": start,
            "goal": goal,
            "actual_start": start,
            "actual_goal": goal,
            "canonical_start": canonical_q(robot, start),
            "canonical_goal": canonical_q(robot, goal),
        })
    return queries


def run_sbf(seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]], budget: int) -> dict[str, Any]:
    budget = int(budget)
    options = RBFLeafRRTOptions(
        seed=int(seed),
        deep_max_boxes=budget,
        use_external_evidence=True,
        external_evidence_path=Path(args.rbf_cache_root) / str(args.warm_cache_label),
        use_shelf_root_override=True,
        case_label="sbf_leaf_rrt",
        threads=int(args.threads),
        parallel_virtual_validation=True,
        leaf_threads=int(args.threads),
    )
    row = run_leaf_rrt(
        robot=robot,
        obstacles=obstacles,
        queries=queries,
        database_path=args.out_dir / "active_cache" / f"sbf_seed{seed}_box{budget}",
        options=options,
    )
    row["method"] = "sbf_leaf_rrt"
    row["planner"] = "RBF build_leaf_sweep_refined"
    row["deep_max_boxes"] = budget
    row["stage_id"] = f"b{budget}"
    row["budget_s"] = math.nan
    return row


def run_rrtconnect(seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]]) -> dict[str, Any]:
    qrows: list[dict[str, Any]] = []
    planning_s = 0.0
    audit_s = 0.0
    for index, query in enumerate(progress(queries, desc=f"exp05 rrt seed={seed}", total=len(queries))):
        result = sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            list(query["start"]),
            list(query["goal"]),
            float(args.rrt_timeout_s) * 1000.0,
            float(args.rrt_range),
            float(args.audit_segment_step),
            0.0,
            int(seed) * 1009 + index,
        )
        planning_s += float(result.get("t_s", 0.0))
        path = [[float(value) for value in point] for point in result.get("path", [])]
        audit_passed, audit_time_s, audit_status = audit_path(
            robot,
            obstacles,
            path,
            float(args.audit_segment_step),
            start=list(query["start"]),
            goal=list(query["goal"]),
        )
        audit_s += audit_time_s
        ok = bool(result.get("ok")) and audit_passed
        length = path_length(path) if ok else math.nan
        qrows.append({
            "label": query["label"],
            "success": ok,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": float(result.get("t_s", 0.0)) * 1000.0,
            "audit_ms": audit_time_s * 1000.0,
            "path_length": length,
            "segment_edge_length": 0.0,
            "segment_fraction": 0.0 if ok else math.nan,
            "waypoint_count": len(path),
            "planner_status": str(result.get("status", result.get("reason", ""))),
        })
    return summarize_method_run(
        "rrtconnect",
        seed,
        planning_s,
        audit_s,
        qrows,
        {"planner": "OMPL_RRTConnect", "timeout_s": float(args.rrt_timeout_s)},
        stage_id=f"timeout{float(args.rrt_timeout_s):g}s",
        budget_s=float(args.rrt_timeout_s),
    )


def run_bitstar_trace(seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    timeout_s = float(args.bitstar_timeout_s)
    interval_s = float(args.bitstar_checkpoint_interval_s)
    query_traces: list[tuple[dict[str, Any], list[dict[str, Any]]]] = []
    for index, query in enumerate(progress(queries, desc=f"exp05 bitstar seed={seed}", total=len(queries))):
        result = sbf.ompl_bitstar_trace(
            robot,
            obstacles,
            list(query["start"]),
            list(query["goal"]),
            timeout_s * 1000.0,
            interval_s * 1000.0,
            float(args.audit_segment_step),
            int(seed) * 2003 + index,
            int(args.bitstar_samples_per_batch),
            float(args.bitstar_rewire_factor),
            bool(args.bitstar_stop_on_solution_improvement),
        )
        query_traces.append((query, [dict(item) for item in result.get("checkpoints", [])]))
    stage_count = max((len(checkpoints) for _query, checkpoints in query_traces), default=0)
    rows: list[dict[str, Any]] = []
    for stage_index in range(stage_count):
        qrows: list[dict[str, Any]] = []
        audit_s = 0.0
        checkpoint_s = 0.0
        for query, checkpoints in query_traces:
            checkpoint = checkpoints[min(stage_index, len(checkpoints) - 1)] if checkpoints else {}
            checkpoint_s = max(checkpoint_s, float(checkpoint.get("checkpoint_s", (stage_index + 1) * interval_s) or 0.0))
            path = [[float(value) for value in point] for point in checkpoint.get("path", [])]
            audit_passed, audit_time_s, audit_status = audit_path(
                robot,
                obstacles,
                path,
                float(args.audit_segment_step),
                start=list(query["start"]),
                goal=list(query["goal"]),
            )
            audit_s += audit_time_s
            ok = bool(checkpoint.get("ok")) and audit_passed
            length = path_length(path) if ok else math.nan
            qrows.append({
                "label": query["label"],
                "success": ok,
                "audit_passed": audit_passed,
                "audit_status": audit_status,
                "query_ms": float(checkpoint.get("elapsed_s", checkpoint.get("t_s", 0.0)) or 0.0) * 1000.0,
                "audit_ms": audit_time_s * 1000.0,
                "path_length": length,
                "segment_edge_length": 0.0,
                "segment_fraction": 0.0 if ok else math.nan,
                "waypoint_count": len(path),
                "planner_status": str(checkpoint.get("status", checkpoint.get("reason", ""))),
                "iterations": int(checkpoint.get("iterations", 0) or 0),
                "batches": int(checkpoint.get("batches", 0) or 0),
                "checkpoint_s": checkpoint_s,
            })
        row = summarize_method_run(
            "bitstar",
            seed,
            sum(float(query.get("query_ms", 0.0)) for query in qrows) / 1000.0,
            audit_s,
            qrows,
            {
                "planner": "OMPL_BITstar_trace",
                "timeout_s": timeout_s,
                "checkpoint_interval_s": interval_s,
                "checkpoint_s": checkpoint_s,
            },
            stage_id=f"t{checkpoint_s:g}s",
            budget_s=checkpoint_s,
        )
        rows.append(row)
    return rows


def run_prm(seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]], build_s: float | None = None) -> dict[str, Any]:
    build_s = float(args.prm_build_s if build_s is None else build_s)
    starts = [list(query["start"]) for query in queries]
    goals = [list(query["goal"]) for query in queries]
    t0 = time.perf_counter()
    result = sbf.ompl_prm_multiquery(
        robot,
        obstacles,
        starts,
        goals,
        build_s,
        float(args.prm_query_s),
        float(args.audit_segment_step),
        0.0,
        int(seed),
        int(args.prm_max_nearest_neighbors),
    )
    planning_s = time.perf_counter() - t0
    qrows: list[dict[str, Any]] = []
    audit_s = 0.0
    qresult_items = list(result.get("queries", []))
    for query, qresult in zip(progress(queries, desc=f"exp05 prm audit seed={seed}", total=len(queries)), qresult_items):
        path = [[float(value) for value in point] for point in qresult.get("path", [])]
        audit_passed, audit_time_s, audit_status = audit_path(
            robot,
            obstacles,
            path,
            float(args.audit_segment_step),
            start=list(query["start"]),
            goal=list(query["goal"]),
        )
        audit_s += audit_time_s
        ok = bool(qresult.get("ok")) and audit_passed
        qrows.append({
            "label": query["label"],
            "success": ok,
            "audit_passed": audit_passed,
            "audit_status": audit_status,
            "query_ms": float(qresult.get("t_s", 0.0)) * 1000.0,
            "audit_ms": audit_time_s * 1000.0,
            "path_length": path_length(path) if ok else math.nan,
            "segment_edge_length": 0.0,
            "segment_fraction": 0.0 if ok else math.nan,
            "waypoint_count": len(path),
            "planner_status": str(qresult.get("status", qresult.get("reason", ""))),
        })
    row = summarize_method_run(
        "prm",
        seed,
        planning_s,
        audit_s,
        qrows,
        {
            "planner": "OMPL_PRM",
            "build_s": float(result.get("build_s", 0.0)),
            "nodes": int(result.get("nodes", 0)),
        },
        stage_id=f"build{build_s:g}s",
        budget_s=build_s,
    )
    row["build_s"] = float(result.get("build_s", build_s))
    return row


def summarize_method_run(
    method: str,
    seed: int,
    planning_s: float,
    audit_s: float,
    qrows: list[dict[str, Any]],
    extra: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
) -> dict[str, Any]:
    successes = [row for row in qrows if bool(row["audit_passed"])]
    return {
        "method": method,
        "seed": int(seed),
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        "status": "ok" if len(successes) == len(qrows) else "partial",
        "success_count": len(successes),
        "query_count": len(qrows),
        "planning_s": float(planning_s),
        "audit_s": float(audit_s),
        "path_length_mean": mean(row["path_length"] for row in successes),
        "raw_segment_fraction": 0.0,
        "queries": qrows,
        "diagnostics": dict(extra or {}),
    }


def external_pending_row(method: str, seed: int, reason: str) -> dict[str, Any]:
    return {
        "method": method,
        "seed": int(seed),
        "stage_id": method,
        "budget_s": math.nan,
        "status": "external_pending",
        "success_count": 0,
        "query_count": 0,
        "planning_s": math.nan,
        "audit_s": math.nan,
        "path_length_mean": math.nan,
        "raw_segment_fraction": math.nan,
        "queries": [],
        "diagnostics": {"reason": reason},
    }


def run_method(method: str, seed: int, args: argparse.Namespace, robot: Any, obstacles: list[Any], queries: list[dict[str, Any]], budget_s: float | None = None) -> dict[str, Any]:
    if method == "sbf_leaf_rrt":
        return run_sbf(seed, args, robot, obstacles, queries, int(args.sbf_box_budget))
    if method == "rrtconnect":
        return run_rrtconnect(seed, args, robot, obstacles, queries)
    if method == "prm":
        return run_prm(seed, args, robot, obstacles, queries, budget_s)
    if method == "bitstar":
        raise ValueError("BIT* uses run_bitstar_trace() so one fixed-timeout run can emit all checkpoints")
    if method == "iris_np_gcs":
        return external_pending_row(method, seed, "IRIS/GCS backend requires external dependency and is not executed by smoke runner.")
    raise ValueError(f"unknown method {method!r}")


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    keys = sorted({
        (str(row["method"]), str(row.get("stage_id", row["method"])), str(row.get("deep_max_boxes", "")))
        for row in rows
    })
    for method, stage_id, budget in keys:
        items = [
            row for row in rows
            if str(row["method"]) == method
            and str(row.get("stage_id", row["method"])) == stage_id
            and str(row.get("deep_max_boxes", "")) == budget
            and row.get("status") != "external_pending"
        ]
        pending = [
            row for row in rows
            if str(row["method"]) == method
            and str(row.get("stage_id", row["method"])) == stage_id
            and str(row.get("deep_max_boxes", "")) == budget
            and row.get("status") == "external_pending"
        ]
        if not items:
            out.append({
                "method": method,
                "stage_id": stage_id,
                "budget_s": None,
                "deep_max_boxes": budget,
                "runs": len(pending),
                "success_runs": 0,
                "planning_s_median": None,
                "audit_s_median": None,
                "path_length_mean": None,
                "raw_segment_fraction_median": None,
                "status": "external_pending" if pending else "missing",
            })
            continue
        out.append({
            "method": method,
            "method_label": METHOD_LABELS.get(method, method),
            "stage_id": stage_id,
            "budget_s": median(row.get("budget_s", math.nan) for row in items),
            "deep_max_boxes": budget if method == "sbf_leaf_rrt" else 0,
            "runs": len(items),
            "success_runs": sum(1 for row in items if int(row["success_count"]) == int(row["query_count"])),
            "source": "current_execution",
            "build_s": median(row.get("build_s", row["planning_s"]) for row in items),
            "query_s_median": median(median(q.get("query_ms", math.nan) / 1000.0 for q in row.get("queries", [])) for row in items),
            "planning_s_median": median(row["planning_s"] for row in items),
            "audit_s_median": median(row["audit_s"] for row in items),
            "path_length_mean": mean(row["path_length_mean"] for row in items if int(row["success_count"]) == int(row["query_count"])),
            "raw_segment_fraction_median": median(row["raw_segment_fraction"] for row in items if int(row["success_count"]) == int(row["query_count"])),
            "status": "executed",
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["method", "method_label", "stage_id", "budget_s", "deep_max_boxes", "runs", "success_runs", "source", "build_s", "query_s_median", "planning_s_median", "audit_s_median", "path_length_mean", "raw_segment_fraction_median", "status"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    def path_stat(row: dict[str, Any]) -> float:
        for key in ("path_length_mean", "path_length_median"):
            try:
                value = float(row.get(key, math.nan))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                return value
        return math.nan

    def selected_rows() -> list[dict[str, Any]]:
        def finite_value(value: Any, fallback: float = 1e9) -> float:
            try:
                out = float(value)
            except (TypeError, ValueError):
                return fallback
            return out if math.isfinite(out) else fallback

        rbf_rows = [
            row for row in rows
            if str(row.get("method")) == "sbf_leaf_rrt"
            and int(row.get("success_runs", 0) or 0) == int(row.get("runs", 0) or 0)
        ]
        selected_rbf = None
        if rbf_rows:
            selected_rbf = sorted(
                rbf_rows,
                key=lambda row: (
                    float(row.get("planning_s_median") or row.get("build_s") or 1e9),
                    int(float(row.get("deep_max_boxes", 0) or 0)),
                ),
            )[0]
        row_by_method: dict[str, dict[str, Any]] = {}
        for method in ["iris_np_gcs", "prm", "rrtconnect", "bitstar"]:
            items = [row for row in rows if str(row.get("method")) == method]
            full = [
                row for row in items
                if int(row.get("success_runs", 0) or 0) == int(row.get("runs", 0) or 0)
            ]
            candidates = full or items
            finite_path = [
                path_stat(row)
                for row in candidates
                if math.isfinite(path_stat(row))
            ]
            if finite_path:
                best_path = min(finite_path)
                candidates = [
                    row for row in candidates
                    if math.isfinite(path_stat(row))
                    and path_stat(row) <= 1.08 * best_path
                ] or candidates
            if candidates:
                row_by_method[method] = sorted(
                    candidates,
                    key=lambda row: (
                        finite_value(row.get("planning_s_median")),
                        finite_value(row.get("budget_s")),
                    ),
                )[0]
        if selected_rbf is not None:
            row_by_method["sbf_leaf_rrt"] = selected_rbf
        return [
            row_by_method[method]
            for method in ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
            if method in row_by_method
        ]

    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Shelf+IIWA cross-algorithm comparison under a common fixed-step final audit. RBF uses the leaf-sweep--RRT grower profile; non-RBF rows use the old TRO baseline budgets and are either current reruns or audited imported artifacts. Planning excludes final audit.}",
        r"\label{tab:tro-shelf-cross-algorithm}",
        r"\footnotesize",
        r"\setlength{\tabcolsep}{3.5pt}",
        r"\begin{tabular}{lrrrrr}",
        r"\toprule",
        r"Method & SR & Build/Plan (s) & Query (s) & Audit (s) & Len. \\",
        r"\midrule",
    ]
    for row in selected_rows():
        sr = f"{int(row.get('success_runs', 0))}/{int(row.get('runs', 0))}"
        method = str(row.get("method_label", row.get("method", ""))).replace("_", r"\_")
        if str(row.get("method")) == "sbf_leaf_rrt":
            method = rf"{method} (b{int(float(row.get('deep_max_boxes', 0) or 0))})"
        lines.append(
            f"{method} & {sr} & "
            f"{tex_num(row.get('build_s', row.get('planning_s_median')))} & {tex_num(row.get('query_s_median'))} & "
            f"{tex_num(row.get('audit_s_median'))} & "
            f"{tex_num(path_stat(row))} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table*}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.5 Shelf+IIWA cross-algorithm study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp05")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--methods", default=",".join(METHODS))
    parser.add_argument("--rerun-baselines", action=argparse.BooleanOptionalAction, default=True, help="Rerun OMPL baselines instead of importing old audited artifacts. IRIS-NP+GCS may still be imported unless explicitly supported.")
    parser.add_argument("--old-paper-root", type=Path, default=Path("/home/tian/桌面/box_aabb/cpp/SBF/doc/paper/tro_rewrite_2026"))
    parser.add_argument("--old-output-root", type=Path, default=Path("/home/tian/桌面/box_aabb/cpp/SBF/outputs/paper"))
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--sbf-box-budget", type=int, default=DEFAULT_RBF_DEEP_MAX_BOXES)
    parser.add_argument("--sbf-box-budgets", default=",".join(str(item) for item in rbf_budget_grid("pilot")))
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    parser.add_argument("--rrt-timeout-s", type=float, default=10.0)
    parser.add_argument("--rrt-range", type=float, default=0.35)
    parser.add_argument("--bitstar-timeout-s", type=float, default=5.0)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=1.0)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=-1)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=-1.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--prm-build-s", type=float, default=0.25)
    parser.add_argument("--prm-build-grid-s", default="0.25,0.5,1,2,5")
    parser.add_argument("--prm-query-s", type=float, default=1.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=32)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    seeds = [int(item) for item in csv_items(args.seeds)]
    methods = [item for item in csv_items(args.methods) if item in METHODS]
    sbf_budgets = [int(item) for item in csv_items(args.sbf_box_budgets)]
    prm_build_grid_s = [float(item) for item in csv_items(args.prm_build_grid_s)] if str(args.prm_build_grid_s).strip() else [float(args.prm_build_s)]
    if args.phase == "smoke":
        seeds = seeds[:1]
        sbf_budgets = [int(args.sbf_box_budget)]
        prm_build_grid_s = [min(float(args.prm_build_s), 0.25)]
        args.bitstar_timeout_s = min(float(args.bitstar_timeout_s), 0.25)
        args.bitstar_checkpoint_interval_s = min(float(args.bitstar_checkpoint_interval_s), float(args.bitstar_timeout_s))
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = shelf_queries(robot)
    planned_rows = []
    for method in methods:
        if method == "sbf_leaf_rrt":
            budgets: list[Any] = sbf_budgets
        elif method == "prm" and args.rerun_baselines:
            budgets = prm_build_grid_s
        elif method == "bitstar":
            budgets = [float(args.bitstar_timeout_s)]
        elif method == "rrtconnect" and args.rerun_baselines:
            budgets = [float(args.rrt_timeout_s)]
        else:
            budgets = [None]
        for seed in seeds:
            for budget in budgets:
                stage_id = (
                    f"b{int(budget)}" if method == "sbf_leaf_rrt"
                    else f"build{float(budget):g}s" if method == "prm" and budget is not None
                    else f"trace{float(budget):g}s" if method == "bitstar" and budget is not None
                    else f"timeout{float(args.rrt_timeout_s):g}s" if method == "rrtconnect" and args.rerun_baselines
                    else method
                )
                planned_rows.append(
        {
            "method": method,
            "seed": seed,
            "stage_id": stage_id,
            "budget_s": float(budget) if method != "sbf_leaf_rrt" and budget is not None else None,
            "deep_max_boxes": budget,
            "scene": "marcucci_shelf_iiwa",
            "query_set": "AS_TS_CS_LB_RB_canonical",
            "audit_segment_step": float(args.audit_segment_step),
            "execution_policy": "import_old_audited_artifact" if method in IMPORTED_BASELINE_METHODS and not args.rerun_baselines else "current_execution",
            "rbf_default_profile": shelf_d23_rbf_profile() if method == "sbf_leaf_rrt" else None,
            "box_budgets": rbf_budget_grid(args.phase) if method == "sbf_leaf_rrt" else None,
        }
                )
    rows: list[dict[str, Any]] = []
    import_payload: dict[str, Any] | None = None
    if not args.dry_run:
        if any(method in IMPORTED_BASELINE_METHODS for method in methods):
            import_payload = import_old_baselines(args.out_dir, args.old_paper_root, args.old_output_root)
            if import_payload["audit"]["status"] != "reusable":
                raise RuntimeError("old baseline reuse audit failed; rerun with --rerun-baselines or inspect old_shelf_baseline_reuse_audit.json")
        executable_rows = [
            planned
            for planned in planned_rows
            if str(planned["method"]) not in IMPORTED_BASELINE_METHODS
        ]
        for planned in progress(executable_rows, desc="exp05 runs", total=len(executable_rows)):
            if (
                str(planned["method"]) in IMPORTED_BASELINE_METHODS
            ):
                continue
            print(f"[exp05] method={planned['method']} stage={planned.get('stage_id')} seed={planned['seed']} budget={planned.get('deep_max_boxes')}", flush=True)
            if str(planned["method"]) == "sbf_leaf_rrt":
                rows.append(run_sbf(int(planned["seed"]), args, robot, obstacles, queries, int(planned["deep_max_boxes"])))
            elif str(planned["method"]) == "bitstar":
                rows.extend(run_bitstar_trace(int(planned["seed"]), args, robot, obstacles, queries))
            else:
                rows.append(run_method(
                    str(planned["method"]),
                    int(planned["seed"]),
                    args,
                    robot,
                    obstacles,
                    queries,
                    float(planned["budget_s"]) if planned.get("budget_s") is not None else None,
                ))
    summary = aggregate(rows) if rows else []
    if import_payload is not None:
        imported_methods = set(methods) & IMPORTED_BASELINE_METHODS
        summary.extend(row for row in import_payload["summary"] if row["method"] in imported_methods)
    payload: dict[str, Any] = {
        "experiment": "exp05_shelf_cross_algorithm",
        "run_id": run_id("exp05"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "rbf_default_profile": shelf_d23_rbf_profile(),
        "baseline_reuse_audit": import_payload["audit"] if import_payload is not None else None,
        "planned_rows": planned_rows,
        "rows": rows,
        "summary": summary,
    }
    write_json(args.out_dir / "shelf_cross_algorithm_manifest.json", payload)
    if summary:
        write_csv(args.out_dir / "shelf_cross_algorithm_summary.csv", summary)
        if any(str(row.get("method")) == "sbf_leaf_rrt" for row in summary):
            write_tex(REPO_ROOT / "paper" / "generated" / "tab_tro_shelf_cross_algorithm.tex", summary)
    print(f"wrote {args.out_dir / 'shelf_cross_algorithm_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
