#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "rbf_matplotlib"))

REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (REPO_ROOT.parent, REPO_ROOT):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, csv_list, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import generate_catalog, make_robot, scene_for_key
from experiments.common.rbf_defaults import DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE, DEFAULT_RBF_AUDIT_SEGMENT_STEP
from experiments.common.sbf_import import import_sbf

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import (  # noqa: E402
    final_ompl_simplify_path,
)
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_rrt_connect import (  # noqa: E402
    path_length as list_path_length,
)
from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_12_random_scene_iris_np_gcs_baseline import (  # noqa: E402
    DEFAULT_IRIS_NP,
    build_drake_random_scene,
    build_regions,
    configure_threads,
    guide_path,
    region_seed_points,
    solve_regions_gcs,
)
from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_16_random_iris_np_gcs_anytime import (  # noqa: E402
    repair_sbf_audit_path,
)


sbf = import_sbf()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.6 saved-catalog IRIS-NP+GCS runner.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06" / "current_iris_gcs")
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=8)
    parser.add_argument("--scene-catalog", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp06" / "random_scene_catalog.json")
    parser.add_argument("--scene-catalog-mode", choices=["reuse", "verify", "auto", "generate"], default="reuse")
    parser.add_argument("--scene-profile", choices=["balanced", "balanced_independent", "balanced_probe", "legacy"], default="balanced_independent")
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--max-scene-tries", type=int, default=64)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--budget-s", type=float, default=240.0)
    parser.add_argument("--max-region-seeds", type=int, default=8)
    parser.add_argument("--iteration-limit", type=int, default=DEFAULT_IRIS_NP["iteration_limit"])
    parser.add_argument("--relative-termination-threshold", type=float, default=DEFAULT_IRIS_NP["relative_termination_threshold"])
    parser.add_argument("--query-time-limit-s", type=float, default=90.0)
    parser.add_argument("--rounding-max-paths", type=int, default=24)
    parser.add_argument("--rounding-max-trials", type=int, default=240)
    parser.add_argument("--gcs-preprocessing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-rounding", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--allow-repair", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--segment-step", type=float, default=0.06)
    parser.add_argument("--audit-segment-step", type=float, default=DEFAULT_RBF_AUDIT_SEGMENT_STEP)
    parser.add_argument("--audit-collision-tolerance", type=float, default=DEFAULT_RBF_AUDIT_COLLISION_TOLERANCE)
    parser.add_argument("--guide-timeout-ms", type=float, default=750.0)
    parser.add_argument("--guide-range", type=float, default=0.35)
    parser.add_argument("--guide-simplify-time-s", type=float, default=0.0)
    parser.add_argument("--sbf-repair-timeout-ms", type=float, default=5000.0)
    parser.add_argument("--sbf-repair-range", type=float, default=0.35)
    parser.add_argument("--sbf-repair-simplify-time-s", type=float, default=0.05)
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.05)
    parser.add_argument("--resume", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rerun-failed", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def audit_path(
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    *,
    segment_step: float,
    start: list[float],
    goal: list[float],
    endpoint_tol: float = 1e-6,
    collision_tolerance: float = 0.0,
) -> tuple[bool, str]:
    if len(path) < 2:
        return False, "empty_path"
    if point_distance(path[0], start) > endpoint_tol:
        return False, "start_mismatch"
    if point_distance(path[-1], goal) > endpoint_tol:
        return False, "goal_mismatch"
    step = max(1e-9, float(segment_step))
    for a, b in zip(path, path[1:]):
        pieces = max(1, int(math.ceil(point_distance(a, b) / step)))
        for index in range(pieces + 1):
            q = interpolate(a, b, index / pieces)
            if sbf.check_config_collision(robot, obstacles, q, float(collision_tolerance)):
                return False, "collision"
    return True, "passed"


def make_iris_args(args: argparse.Namespace) -> SimpleNamespace:
    return SimpleNamespace(
        iteration_limit=int(args.iteration_limit),
        relative_termination_threshold=float(args.relative_termination_threshold),
        guide_timeout_ms=float(args.guide_timeout_ms),
        guide_range=float(args.guide_range),
        guide_simplify_time_s=float(args.guide_simplify_time_s),
        segment_step=float(args.segment_step),
        allow_repair=bool(args.allow_repair),
        query_time_limit_s=float(args.query_time_limit_s),
        rounding_max_paths=int(args.rounding_max_paths),
        rounding_max_trials=int(args.rounding_max_trials),
        gcs_preprocessing=bool(args.gcs_preprocessing),
        use_rounding=bool(args.use_rounding),
        audit_segment_step=float(args.audit_segment_step),
        sbf_repair_timeout_ms=float(args.sbf_repair_timeout_ms),
        sbf_repair_range=float(args.sbf_repair_range),
        sbf_repair_simplify_time_s=float(args.sbf_repair_simplify_time_s),
        final_ompl_simplify_time_s=float(args.final_ompl_simplify_time_s),
    )


def run_scene(args: argparse.Namespace, catalog: dict[str, Any], robot_name: str, difficulty: str, scene_seed: int) -> dict[str, Any]:
    scene = scene_for_key(catalog, robot_name, difficulty, scene_seed)
    robot = make_robot(robot_name)
    iris_args = make_iris_args(args)
    planner_seed = int(args.seed_base) + 73856093 * int(scene_seed) + 19349663 * (1 + len(robot_name) + len(difficulty))
    print(f"[exp06-iris] robot={robot_name} difficulty={difficulty} seed={scene_seed}", flush=True)
    guide, guide_s, guide_result = guide_path(iris_args, robot, list(scene.obstacles), list(scene.start), list(scene.goal), planner_seed)
    robot_diagram, plant, model_instance, checker = build_drake_random_scene(robot_name, list(scene.obstacles))
    raw_seed_points = [np.asarray(scene.start, dtype=float), np.asarray(scene.goal, dtype=float)]
    raw_seed_points.extend(region_seed_points(guide, int(args.max_region_seeds)))
    seed_points = []
    seen_seed_keys: set[tuple[float, ...]] = set()
    for point in raw_seed_points:
        key = tuple(round(float(value), 9) for value in point)
        if key in seen_seed_keys:
            continue
        seen_seed_keys.add(key)
        if checker.CheckConfigCollisionFree(point):
            seed_points.append(np.asarray(point, dtype=float))
    if not seed_points:
        seed_points = [np.asarray(scene.start, dtype=float), np.asarray(scene.goal, dtype=float)]
    regions, timings, failures = build_regions(
        iris_args,
        robot_diagram,
        plant,
        model_instance,
        checker,
        seed_points,
        planner_seed,
        float(args.budget_s),
    )
    result = solve_regions_gcs(
        np.asarray(scene.start, dtype=float),
        np.asarray(scene.goal, dtype=float),
        regions,
        seed=planner_seed,
        checker=checker,
        robot=robot,
        query_time_limit_s=float(args.query_time_limit_s),
        allow_repair=bool(args.allow_repair),
        rounding_max_paths=int(args.rounding_max_paths),
        rounding_max_trials=int(args.rounding_max_trials),
        gcs_preprocessing=bool(args.gcs_preprocessing),
        use_rounding=bool(args.use_rounding),
    )
    path = [list(point) for point in result.get("path", [])]
    drake_success = bool(result.get("success")) and len(path) >= 2
    repair: dict[str, Any] = {}
    audit_ok, audit_status = audit_path(
        robot,
        list(scene.obstacles),
        path,
        segment_step=float(args.audit_segment_step),
        start=list(scene.start),
        goal=list(scene.goal),
        collision_tolerance=float(args.audit_collision_tolerance),
    ) if drake_success else (False, str(result.get("note", "gcs_failed")))
    if drake_success and not audit_ok:
        repair = repair_sbf_audit_path(iris_args, robot, list(scene.obstacles), path, planner_seed + 10007)
        if repair.get("ok"):
            path = [list(point) for point in repair.get("path", [])]
            audit_ok, audit_status = audit_path(
                robot,
                list(scene.obstacles),
                path,
                segment_step=float(args.audit_segment_step),
                start=list(scene.start),
                goal=list(scene.goal),
                collision_tolerance=float(args.audit_collision_tolerance),
            )
    if drake_success and audit_ok and float(args.final_ompl_simplify_time_s) > 0.0:
        simplified = final_ompl_simplify_path(
            sbf,
            robot,
            list(scene.obstacles),
            path,
            segment_step=float(args.segment_step),
            audit_segment_step=float(args.audit_segment_step),
            simplify_time_s=float(args.final_ompl_simplify_time_s),
        )
        path = [list(point) for point in simplified.get("path", path)]
        simplify_s = float(simplified.get("query_s", 0.0) or 0.0)
        simplify_reason = str(simplified.get("reason", ""))
    else:
        simplify_s = 0.0
        simplify_reason = "not_attempted"
    if drake_success and audit_ok:
        audit_ok, audit_status = audit_path(
            robot,
            list(scene.obstacles),
            path,
            segment_step=float(args.audit_segment_step),
            start=list(scene.start),
            goal=list(scene.goal),
            collision_tolerance=float(args.audit_collision_tolerance),
        )
    success = bool(drake_success and audit_ok)
    build_s = float(guide_s) + sum(float(value) for value in timings)
    query_s = float(result.get("time_s", 0.0) or 0.0) + float(repair.get("time_s", 0.0) or 0.0) + simplify_s
    length = list_path_length(path) if success else math.nan
    print(
        f"[exp06-iris] done robot={robot_name} difficulty={difficulty} seed={scene_seed} "
        f"regions={len(regions)} success={success} build_s={build_s:.3f} query_s={query_s:.3f}",
        flush=True,
    )
    return {
        "method": "iris_np_gcs",
        "robot": robot_name,
        "difficulty": difficulty,
        "scene_seed": int(scene_seed),
        "stage_id": f"budget{float(args.budget_s):g}_r{int(args.max_region_seeds)}_it{int(args.iteration_limit)}",
        "budget_s": float(args.budget_s),
        "deep_max_boxes": 0,
        "obstacle_count": len(scene.obstacles),
        "status": "ok" if success else "failed_audit" if drake_success else "failed_planning",
        "success_count": 1 if success else 0,
        "query_count": 1,
        "planning_s": build_s + query_s,
        "build_s": build_s,
        "query_s": query_s,
        "audit_s": 0.0,
        "path_length_mean": length,
        "raw_segment_fraction": 0.0 if success else math.nan,
        "final_boxes": math.nan,
        "queries": [{
            "label": f"{robot_name}_{difficulty}_{scene_seed}",
            "success": success,
            "audit_passed": audit_ok,
            "audit_status": audit_status,
            "query_ms": query_s * 1000.0,
            "audit_ms": 0.0,
            "path_length": length,
            "segment_fraction": 0.0 if success else math.nan,
            "waypoint_count": len(path),
        }],
        "diagnostics": {
            "guide_s": guide_s,
            "guide_ok": bool(guide_result.get("ok")),
            "region_build_s": [float(value) for value in timings],
            "failed_region_seeds": failures,
            "n_regions": len(regions),
            "gcs_result": {key: value for key, value in result.items() if key != "path"},
            "sbf_repair": repair,
            "final_ompl_simplify_time_s": float(args.final_ompl_simplify_time_s),
            "final_ompl_simplify_reason": simplify_reason,
            "scene_catalog": str(args.scene_catalog),
        },
    }


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    keys = sorted({(row["robot"], row["difficulty"], row["stage_id"]) for row in rows})
    for robot, difficulty, stage_id in keys:
        items = [row for row in rows if row["robot"] == robot and row["difficulty"] == difficulty and row["stage_id"] == stage_id]
        success_items = [row for row in items if int(row.get("success_count", 0)) == int(row.get("query_count", 1))]
        out.append({
            "method": "iris_np_gcs",
            "robot": robot,
            "difficulty": difficulty,
            "stage_id": stage_id,
            "budget_s": median(row.get("budget_s", math.nan) for row in items),
            "deep_max_boxes": 0,
            "scenes": len(items),
            "success_scenes": len(success_items),
            "obstacles_median": median(row.get("obstacle_count", math.nan) for row in items),
            "planning_s_median": median(row.get("planning_s", math.nan) for row in items),
            "audit_s_median": median(row.get("audit_s", math.nan) for row in items),
            "path_length_mean": mean(row.get("path_length_mean", math.nan) for row in success_items),
            "raw_segment_fraction_median": median(row.get("raw_segment_fraction", math.nan) for row in success_items),
            "final_boxes_median": math.nan,
            "status": "executed",
        })
    return out


def write_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "method", "robot", "difficulty", "stage_id", "budget_s", "deep_max_boxes", "scenes",
        "success_scenes", "obstacles_median", "planning_s_median", "audit_s_median",
        "path_length_mean", "raw_segment_fraction_median", "final_boxes_median", "status",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)


def row_key(row: dict[str, Any]) -> tuple[str, str, int]:
    return (str(row.get("robot")), str(row.get("difficulty")), int(row.get("scene_seed", -1)))


def planned_key(item: tuple[str, str, int]) -> tuple[str, str, int]:
    robot, difficulty, seed = item
    return (str(robot), str(difficulty), int(seed))


def load_checkpoint(out_dir: Path) -> list[dict[str, Any]]:
    manifest_path = out_dir / "random_robot_iris_gcs_manifest.json"
    if not manifest_path.exists():
        return []
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return []
    rows = payload.get("rows", [])
    return rows if isinstance(rows, list) else []


def write_checkpoint(args: argparse.Namespace,
                     catalog: dict[str, Any],
                     planned: list[tuple[str, str, int]],
                     rows: list[dict[str, Any]],
                     run_id_value: str,
                     status: str) -> None:
    summary = aggregate(rows) if rows else []
    payload = {
        "experiment": "exp06_saved_catalog_iris_gcs",
        "run_id": run_id_value,
        "status": status,
        "environment": environment_metadata(),
        "params": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "scene_catalog": {"path": str(args.scene_catalog), "schema": catalog.get("schema"), "records": len(catalog.get("records", []))},
        "planned_rows": planned,
        "rows": rows,
        "summary": summary,
    }
    out_dir = Path(args.out_dir)
    write_json(out_dir / "random_robot_iris_gcs_manifest.json", payload)
    if summary:
        write_summary_csv(out_dir / "random_robot_iris_gcs_summary.csv", summary)


def main() -> int:
    args = parse_args()
    configure_threads(int(args.threads))
    robots = csv_list(args.robots)
    difficulties = csv_list(args.difficulties)
    catalog = generate_catalog(
        path=Path(args.scene_catalog),
        robots=robots,
        difficulties=difficulties,
        scene_seeds=int(args.scene_seeds),
        scene_profile=str(args.scene_profile),
        seed_base=int(args.seed_base),
        max_scene_tries=int(args.max_scene_tries),
        mode=str(args.scene_catalog_mode),
    )
    planned = [
        (robot, difficulty, seed)
        for robot in robots
        for difficulty in difficulties
        for seed in range(int(args.scene_seeds))
    ]
    out_dir = Path(args.out_dir)
    run_id_value = run_id("exp06_iris")
    rows: list[dict[str, Any]] = load_checkpoint(out_dir) if bool(args.resume) and not args.dry_run else []
    if bool(args.rerun_failed):
        rows = [
            row for row in rows
            if int(row.get("success_count", 0)) == int(row.get("query_count", 1))
        ]
    seen = {row_key(row) for row in rows}
    if not args.dry_run:
        for robot, difficulty, seed in progress(planned, desc="exp06 iris", total=len(planned)):
            if planned_key((robot, difficulty, seed)) in seen:
                continue
            rows.append(run_scene(args, catalog, robot, difficulty, seed))
            seen.add(planned_key((robot, difficulty, seed)))
            write_checkpoint(args, catalog, planned, rows, run_id_value, "partial")
    write_checkpoint(args, catalog, planned, rows, run_id_value, "dry_run" if args.dry_run else "executed")
    print(f"wrote {Path(args.out_dir) / 'random_robot_iris_gcs_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
