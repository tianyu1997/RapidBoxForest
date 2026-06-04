#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, csv_list, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median, tex_num
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import generate_catalog, make_robot, scene_for_key
from experiments.common.rbf_defaults import (
    DEFAULT_RBF_AUDIT_SEGMENT_STEP,
    DEFAULT_RBF_DEEP_MAX_BOXES,
    ROBOT_LECTDB_CACHE_ROOT,
    default_rbf_profile,
    rbf_budget_grid,
    robot_lectdb_profile,
    robot_sector_expanded_root_tuples,
)
from experiments.common.rbf_leaf_rrt import QuerySpec, RBFLeafRRTOptions, run_leaf_rrt
from experiments.common.robot_lectdb_cache import ensure_robot_lectdb_cache, robot_external_evidence_path
from experiments.common.sbf_import import import_sbf


METHODS = ["sbf_leaf_rrt", "iris_np_gcs", "prm", "rrtconnect", "bitstar"]
sbf = import_sbf()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.6 saved-catalog random multi-robot study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=50)
    parser.add_argument("--scene-profile", choices=["balanced", "balanced_independent", "balanced_probe", "legacy"], default="balanced_independent")
    parser.add_argument("--max-scene-tries", type=int, default=64)
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--methods", default="sbf_leaf_rrt")
    parser.add_argument("--deep-max-boxes", type=int, default=DEFAULT_RBF_DEEP_MAX_BOXES)
    parser.add_argument("--box-budgets", default="")
    parser.add_argument("--connector-segment-resolution", type=int, default=None)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--lect-cache-root", type=Path, default=ROBOT_LECTDB_CACHE_ROOT)
    parser.add_argument("--skip-lect-cache-ensure", action="store_true")
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--rrt-timeout-s", type=float, default=1.0)
    parser.add_argument("--rrt-range", type=float, default=0.35)
    parser.add_argument("--prm-build-s", type=float, default=2.0)
    parser.add_argument("--prm-build-grid-s", default="0.25,0.5,1,2,5")
    parser.add_argument("--prm-query-s", type=float, default=1.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=32)
    parser.add_argument("--bitstar-timeout-s", type=float, default=2.0)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=1.0)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=1.1)
    return parser.parse_args()


def path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return math.nan
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
    return total


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def audit_path(robot: Any, obstacles: list[Any], path: list[list[float]], segment_step: float) -> tuple[bool, float, str]:
    t0 = time.perf_counter()
    if len(path) < 2:
        return False, time.perf_counter() - t0, "empty_path"
    step = max(1e-9, float(segment_step))
    for a, b in zip(path, path[1:]):
        distance = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
        steps = max(1, int(math.ceil(distance / step)))
        for index in range(steps + 1):
            if sbf.check_config_collision(robot, obstacles, interpolate(a, b, index / steps)):
                return False, time.perf_counter() - t0, "collision"
    return True, time.perf_counter() - t0, "passed"


def run_rbf_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    query = QuerySpec(
        label=f"{robot_name}_{difficulty}_{scene_seed}",
        start=list(scene.start),
        goal=list(scene.goal),
        actual_start=list(scene.start),
        actual_goal=list(scene.goal),
    )
    root_override = robot_sector_expanded_root_tuples(robot_name, robot)
    row = run_leaf_rrt(
        robot=robot,
        obstacles=list(scene.obstacles),
        queries=[query],
        database_path=args.out_dir / "active_cache" / f"rbf_{robot_name}_{difficulty}_{scene_seed}",
        options=RBFLeafRRTOptions(
            seed=int(scene_seed),
            deep_max_boxes=int(args.deep_max_boxes),
            threads=int(args.threads),
            use_external_evidence=True,
            external_evidence_path=robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root)),
            external_evidence_verify_identity=root_override is None,
            root_override_tuples=root_override,
            coverage_override_tuples=root_override,
            database_canonical_mode=True,
            case_label=f"rbf_{robot_name}_{difficulty}",
            parallel_virtual_validation=False,
            leaf_threads=1,
            canonicalize_queries=False,
            connector_segment_resolution=(
                int(args.connector_segment_resolution)
                if args.connector_segment_resolution is not None
                else RBFLeafRRTOptions().connector_segment_resolution
            ),
        ),
    )
    row.update(
        {
            "method": "sbf_leaf_rrt",
            "robot": robot_name,
            "difficulty": difficulty,
            "scene_seed": int(scene_seed),
            "obstacle_count": len(scene.obstacles),
            "scene_catalog": str(args.scene_catalog or (args.out_dir / "random_scene_catalog.json")),
            "lectdb": robot_lectdb_profile(robot_name),
            "external_evidence_path": str(robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root))),
        }
    )
    return row


def summarize_single_query_method(
    method: str,
    robot_name: str,
    difficulty: str,
    scene_seed: int,
    scene: Any,
    planning_s: float,
    audit_s: float,
    ok: bool,
    audit_passed: bool,
    audit_status: str,
    length: float,
    path: list[list[float]],
    diagnostics: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
) -> dict[str, Any]:
    success = bool(ok) and bool(audit_passed)
    return {
        "method": method,
        "robot": robot_name,
        "difficulty": difficulty,
        "scene_seed": int(scene_seed),
        "deep_max_boxes": 0,
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        "obstacle_count": len(scene.obstacles),
        "status": "ok" if success else "failed_audit" if ok else "failed_planning",
        "success_count": 1 if success else 0,
        "query_count": 1,
        "planning_s": float(planning_s),
        "audit_s": float(audit_s),
        "path_length_mean": float(length) if success else math.nan,
        "raw_segment_fraction": 0.0 if success else math.nan,
        "final_boxes": math.nan,
        "queries": [
            {
                "label": f"{robot_name}_{difficulty}_{scene_seed}",
                "success": success,
                "audit_passed": bool(audit_passed),
                "audit_status": audit_status,
                "query_ms": float(planning_s) * 1000.0,
                "audit_ms": float(audit_s) * 1000.0,
                "path_length": float(length) if success else math.nan,
                "segment_fraction": 0.0 if success else math.nan,
                "waypoint_count": len(path),
            }
        ],
        "diagnostics": dict(diagnostics or {}),
    }


def run_rrtconnect_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    result = sbf.ompl_rrt_connect_path(
        robot,
        list(scene.obstacles),
        list(scene.start),
        list(scene.goal),
        float(args.rrt_timeout_s) * 1000.0,
        float(args.rrt_range),
        float(args.audit_segment_step),
        0.0,
        int(args.seed_base) + int(scene_seed),
    )
    path = [[float(value) for value in point] for point in result.get("path", [])]
    audit_passed, audit_s, audit_status = audit_path(robot, list(scene.obstacles), path, float(args.audit_segment_step))
    ok = bool(result.get("ok"))
    return summarize_single_query_method(
        "rrtconnect",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        float(result.get("t_s", 0.0)),
        audit_s,
        ok,
        audit_passed,
        audit_status,
        path_length(path),
        path,
        {"planner": "OMPL_RRTConnect", "status": str(result.get("status", "")), "nodes": int(result.get("nodes", 0) or 0)},
    )


def run_prm_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int, build_budget_s: float) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    result = sbf.ompl_prm_multiquery(
        robot,
        list(scene.obstacles),
        [list(scene.start)],
        [list(scene.goal)],
        float(build_budget_s),
        float(args.prm_query_s),
        float(args.audit_segment_step),
        0.0,
        int(args.seed_base) + 10007 * int(scene_seed),
        int(args.prm_max_nearest_neighbors),
    )
    qresult = list(result.get("queries", []))[0] if list(result.get("queries", [])) else {}
    path = [[float(value) for value in point] for point in qresult.get("path", [])]
    audit_passed, audit_s, audit_status = audit_path(robot, list(scene.obstacles), path, float(args.audit_segment_step))
    planning_s = float(result.get("build_s", 0.0)) + float(qresult.get("t_s", 0.0))
    return summarize_single_query_method(
        "prm",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        planning_s,
        audit_s,
        bool(qresult.get("ok")),
        audit_passed,
        audit_status,
        path_length(path),
        path,
        {
            "planner": "OMPL_PRM",
            "build_s": float(result.get("build_s", 0.0)),
            "query_s": float(qresult.get("t_s", 0.0)),
            "status": str(qresult.get("status", "")),
            "nodes": int(result.get("nodes", 0) or 0),
        },
        stage_id=f"build{float(build_budget_s):g}s",
        budget_s=float(build_budget_s),
    )


def run_bitstar_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int, timeout_s: float) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    result = sbf.ompl_bitstar_path(
        robot,
        list(scene.obstacles),
        list(scene.start),
        list(scene.goal),
        float(timeout_s) * 1000.0,
        float(args.audit_segment_step),
        0.0,
        int(args.seed_base) + 20011 * int(scene_seed),
        int(args.bitstar_samples_per_batch),
        float(args.bitstar_rewire_factor),
        False,
    )
    path = [[float(value) for value in point] for point in result.get("path", [])]
    audit_passed, audit_s, audit_status = audit_path(robot, list(scene.obstacles), path, float(args.audit_segment_step))
    return summarize_single_query_method(
        "bitstar",
        robot_name,
        difficulty,
        scene_seed,
        scene,
        float(result.get("t_s", 0.0)),
        audit_s,
        bool(result.get("ok")),
        audit_passed,
        audit_status,
        path_length(path),
        path,
        {
            "planner": "OMPL_BITstar",
            "status": str(result.get("status", "")),
            "iterations": int(result.get("iterations", 0) or 0),
            "batches": int(result.get("batches", 0) or 0),
        },
        stage_id=f"t{float(timeout_s):g}s",
        budget_s=float(timeout_s),
    )


def run_baseline_scene(args: argparse.Namespace, catalog: dict[str, Any], method: str, robot: str, difficulty: str, seed: int, budget_s: float | None = None) -> dict[str, Any]:
    if method == "rrtconnect":
        row = run_rrtconnect_scene(args, catalog, robot, difficulty, seed)
        row["stage_id"] = f"timeout{float(args.rrt_timeout_s):g}s"
        row["budget_s"] = float(args.rrt_timeout_s)
        return row
    if method == "prm":
        return run_prm_scene(args, catalog, robot, difficulty, seed, float(budget_s if budget_s is not None else args.prm_build_s))
    if method == "bitstar":
        return run_bitstar_scene(args, catalog, robot, difficulty, seed, float(budget_s if budget_s is not None else args.bitstar_timeout_s))
    return {
        "method": method,
        "robot": robot,
        "difficulty": difficulty,
        "scene_seed": int(seed),
        "deep_max_boxes": 0,
        "stage_id": method,
        "budget_s": math.nan,
        "status": "external_pending",
        "success_count": 0,
        "query_count": 1,
        "planning_s": math.nan,
        "audit_s": math.nan,
        "path_length_mean": math.nan,
        "raw_segment_fraction": math.nan,
        "final_boxes": math.nan,
        "diagnostics": {"reason": "IRIS/GCS backend is not executed by this self-contained runner."},
    }


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    keys = sorted({
        (
            row["method"],
            row["robot"],
            row["difficulty"],
            str(row.get("stage_id", "")),
            int(row.get("deep_max_boxes", 0) or 0),
        )
        for row in rows
    })
    for method, robot, difficulty, stage_id, budget in keys:
        items = [
            row for row in rows
            if row["method"] == method
            and row["robot"] == robot
            and row["difficulty"] == difficulty
            and str(row.get("stage_id", "")) == stage_id
            and int(row.get("deep_max_boxes", 0) or 0) == budget
        ]
        success_items = [row for row in items if int(row.get("success_count", 0)) == int(row.get("query_count", 1))]
        out.append(
            {
                "method": method,
                "robot": robot,
                "difficulty": difficulty,
                "stage_id": stage_id,
                "budget_s": median(row.get("budget_s", math.nan) for row in items),
                "deep_max_boxes": budget,
                "scenes": len(items),
                "success_scenes": len(success_items),
                "obstacles_median": median(row.get("obstacle_count", math.nan) for row in items),
                "planning_s_median": median(row.get("planning_s", math.nan) for row in items),
                "audit_s_median": median(row.get("audit_s", math.nan) for row in items),
                "path_length_mean": mean(row.get("path_length_mean", math.nan) for row in success_items),
                "raw_segment_fraction_median": median(row.get("raw_segment_fraction", math.nan) for row in success_items),
                "final_boxes_median": median(row.get("final_boxes", math.nan) for row in items),
                "status": "executed",
            }
        )
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "method",
        "robot",
        "difficulty",
        "stage_id",
        "budget_s",
        "deep_max_boxes",
        "scenes",
        "success_scenes",
        "obstacles_median",
        "planning_s_median",
        "audit_s_median",
        "path_length_mean",
        "raw_segment_fraction_median",
        "final_boxes_median",
        "status",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)


def select_best_tradeoff_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Select one readable row per robot/difficulty; full budget curves stay in the figure/CSV."""
    def path_stat(row: dict[str, Any]) -> float:
        for key in ("path_length_mean", "path_length_median"):
            try:
                value = float(row.get(key, math.nan))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                return value
        return math.nan

    out: list[dict[str, Any]] = []
    keys = sorted({(str(row.get("robot", "")), str(row.get("difficulty", ""))) for row in rows})
    for robot, difficulty in keys:
        items = [row for row in rows if str(row.get("robot", "")) == robot and str(row.get("difficulty", "")) == difficulty]
        full = [
            row for row in items
            if int(float(row.get("success_scenes", 0) or 0)) == int(float(row.get("scenes", 0) or 0))
        ]
        candidates = full or items
        if not candidates:
            continue
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
        out.append(sorted(
            candidates,
            key=lambda row: (
                float(row.get("planning_s_median", math.nan)) if math.isfinite(float(row.get("planning_s_median", math.nan))) else 1e9,
                int(float(row.get("deep_max_boxes", 0) or 0)),
            ),
        )[0])
    return out


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    rows = select_best_tradeoff_rows(rows)
    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Saved-catalog random-scene RBF best trade-off points. Planning time excludes final audit; full box-budget curves are shown in Fig.~\ref{fig:tro_random_tradeoff}.}",
        r"\label{tab:tro-random-summary}",
        r"\footnotesize",
        r"\begin{tabular}{llrrrrrrr}",
        r"\toprule",
        r"Robot & Difficulty & Boxes & SR & Obst. & Plan (s) & Audit (s) & Len. & Seg. \\",
        r"\midrule",
    ]
    for row in rows:
        sr = f"{int(row.get('success_scenes', 0))}/{int(row.get('scenes', 0))}"
        lines.append(
            f"{row.get('robot')} & {row.get('difficulty')} & {int(row.get('deep_max_boxes', 0) or 0)} & {sr} & "
            f"{tex_num(row.get('obstacles_median'), 1)} & {tex_num(row.get('planning_s_median'))} & "
            f"{tex_num(row.get('audit_s_median'))} & {tex_num(row.get('path_length_mean', row.get('path_length_median')))} & "
            f"{tex_num(row.get('raw_segment_fraction_median'))} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table*}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    scene_seeds = 1 if args.phase == "smoke" else (8 if args.phase == "paper" else int(args.scene_seeds))
    robots = csv_list(args.robots)
    difficulties = csv_list(args.difficulties)
    methods = csv_list(args.methods)
    box_budgets = [int(item) for item in csv_list(args.box_budgets)] if str(args.box_budgets).strip() else rbf_budget_grid(args.phase)
    prm_build_grid_s = [float(item) for item in csv_list(args.prm_build_grid_s)] if str(args.prm_build_grid_s).strip() else [float(args.prm_build_s)]
    bitstar_stage_s: list[float] = []
    interval_s = max(1e-9, float(args.bitstar_checkpoint_interval_s))
    target = interval_s
    while target < float(args.bitstar_timeout_s) - 1e-9:
        bitstar_stage_s.append(float(target))
        target += interval_s
    bitstar_stage_s.append(float(args.bitstar_timeout_s))
    if args.phase == "smoke":
        robots = robots[:1]
        difficulties = difficulties[:1]
        box_budgets = [int(args.deep_max_boxes)]
        prm_build_grid_s = [min(float(args.prm_build_s), 0.25)]
        bitstar_stage_s = [min(float(args.bitstar_timeout_s), 0.25)]
    catalog_path = args.scene_catalog or (args.out_dir / "random_scene_catalog.json")
    catalog_summary: dict[str, Any] = {"path": str(catalog_path), "mode": args.scene_catalog_mode, "records": None}
    catalog: dict[str, Any] | None = None
    cache_rows: list[dict[str, Any]] = []
    if "sbf_leaf_rrt" in methods:
        for robot in progress(robots, desc="exp06 lect cache", total=len(robots), disable=bool(args.dry_run or args.skip_lect_cache_ensure)):
            cache_rows.append(
                ensure_robot_lectdb_cache(
                    robot,
                    cache_root=Path(args.lect_cache_root),
                    threads=int(args.threads),
                    dry_run=bool(args.dry_run or args.skip_lect_cache_ensure),
                )
            )
    if not args.dry_run:
        catalog = generate_catalog(
            path=catalog_path,
            robots=robots,
            difficulties=difficulties,
            scene_seeds=scene_seeds,
            scene_profile=args.scene_profile,
            seed_base=int(args.seed_base),
            max_scene_tries=int(args.max_scene_tries),
            mode=args.scene_catalog_mode,
        )
        catalog_summary.update({"schema": catalog.get("schema"), "records": len(catalog.get("records", []))})
    rows = []
    for method in METHODS:
        if method not in methods:
            continue
        if method == "sbf_leaf_rrt":
            budgets: list[Any] = box_budgets
        elif method == "prm":
            budgets = prm_build_grid_s
        elif method == "bitstar":
            budgets = bitstar_stage_s
        elif method == "rrtconnect":
            budgets = [float(args.rrt_timeout_s)]
        else:
            budgets = [None]
        for robot in robots:
            for difficulty in difficulties:
                for seed in range(scene_seeds):
                    for budget in budgets:
                        stage_id = (
                            f"b{int(budget)}" if method == "sbf_leaf_rrt"
                            else f"build{float(budget):g}s" if method == "prm"
                            else f"t{float(budget):g}s" if method == "bitstar"
                            else f"timeout{float(args.rrt_timeout_s):g}s" if method == "rrtconnect"
                            else method
                        )
                        rows.append({
                            "method": method,
                            "robot": robot,
                            "difficulty": difficulty,
                            "scene_seed": seed,
                            "stage_id": stage_id,
                            "budget_s": float(budget) if method != "sbf_leaf_rrt" and budget is not None else None,
                            "deep_max_boxes": budget if method == "sbf_leaf_rrt" else 0,
                            "scene_catalog": str(catalog_path),
                            "status": "planned" if args.dry_run else "planned_for_execution",
                            "rbf_default_profile": default_rbf_profile() if method == "sbf_leaf_rrt" else None,
                            "rbf_robot_lectdb": robot_lectdb_profile(robot) if method == "sbf_leaf_rrt" else None,
                            "rbf_box_budgets": box_budgets if method == "sbf_leaf_rrt" else None,
                            "metrics": ["success_rate", "planning_s", "audit_s", "path_length", "raw_segment_fraction"],
                        })
    run_rows: list[dict[str, Any]] = []
    if not args.dry_run and catalog is not None:
        for row in progress(rows, desc="exp06 runs", total=len(rows)):
            if row["method"] == "sbf_leaf_rrt":
                print(f"[exp06] method=sbf_leaf_rrt robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']} budget={row['deep_max_boxes']}", flush=True)
                args.deep_max_boxes = int(row["deep_max_boxes"])
                run_rows.append(run_rbf_scene(args, catalog, str(row["robot"]), str(row["difficulty"]), int(row["scene_seed"])))
            else:
                print(f"[exp06] method={row['method']} stage={row.get('stage_id')} robot={row['robot']} difficulty={row['difficulty']} seed={row['scene_seed']}", flush=True)
                run_rows.append(run_baseline_scene(
                    args,
                    catalog,
                    str(row["method"]),
                    str(row["robot"]),
                    str(row["difficulty"]),
                    int(row["scene_seed"]),
                    float(row["budget_s"]) if row.get("budget_s") is not None else None,
                ))
    summary_rows = aggregate(run_rows) if run_rows else []
    payload: dict[str, Any] = {
        "experiment": "exp06_random_robot",
        "run_id": run_id("exp06"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "rbf_default_profile": default_rbf_profile(),
        "lectdb_caches": cache_rows,
        "scene_catalog": catalog_summary,
        "planned_rows": rows,
        "rows": run_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "random_robot_manifest.json", payload)
    if summary_rows:
        write_csv(args.out_dir / "random_robot_summary.csv", summary_rows)
        if any(str(row.get("method")) == "sbf_leaf_rrt" for row in summary_rows):
            write_tex(REPO_ROOT / "paper" / "generated" / "tab_tro_random_summary.tex", summary_rows)
    print(f"wrote {args.out_dir / 'random_robot_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
