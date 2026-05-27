#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import aggregate_stage_records, final_ompl_simplify_path, incumbent_stage_record, path_passes_post_audit, task_result, update_incumbents  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_rrt_connect import path_length as list_path_length, segment_free  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import ROOT, sbf, write_json  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import DEFAULT_RANDOM_DIFFICULTIES, DEFAULT_RANDOM_ROBOTS, DEFAULT_RANDOM_SCENE_SEEDS, ENDPOINT_CLEARANCE_MARGIN_M, FIXED_ROBOT_CLEARANCE_MARGIN_M, SEGMENT_RESOLUTION, make_random_scene, make_robot, scene_profile_requires_balanced_probe  # noqa: E402
from RapidBoxForest.safe_box_forest.experiments.sbf_old.paper_12_random_scene_iris_np_gcs_baseline import (  # noqa: E402
    build_drake_random_scene,
    build_regions,
    configure_threads,
    guide_path,
    parse_csv,
    region_seed_points,
    sbf_audit_path,
    solve_regions_gcs,
)


def parse_int_grid(raw: str) -> list[int]:
    return [int(float(item)) for item in parse_csv(raw)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prefix-budget IRIS-NP+GCS anytime curve for random SBF scenes.")
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--difficulties", default=DEFAULT_RANDOM_DIFFICULTIES)
    parser.add_argument("--scene-seeds", type=int, default=DEFAULT_RANDOM_SCENE_SEEDS)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--logical-threads", type=int, default=8)
    parser.add_argument("--budget-s", type=float, default=420.0)
    parser.add_argument("--stage-region-counts", default="3,5,7,9,12,16,20")
    parser.add_argument("--iteration-limit", type=int, default=8)
    parser.add_argument("--relative-termination-threshold", type=float, default=1e-2)
    parser.add_argument("--query-time-limit-s", type=float, default=90.0)
    parser.add_argument("--rounding-max-paths", type=int, default=24)
    parser.add_argument("--rounding-max-trials", type=int, default=240)
    parser.add_argument("--gcs-preprocessing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-rounding", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--allow-repair", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--segment-step", type=float, default=0.06)
    parser.add_argument("--audit-segment-step", type=float, default=0.01, help="Independent dense post-hoc path audit step in joint-space radians. Use <=0 to reuse --segment-step.")
    parser.add_argument("--guide-timeout-ms", type=float, default=750.0)
    parser.add_argument("--guide-range", type=float, default=0.35)
    parser.add_argument("--guide-simplify-time-s", type=float, default=0.0)
    parser.add_argument("--sbf-repair-timeout-ms", type=float, default=5000.0)
    parser.add_argument("--sbf-repair-range", type=float, default=0.35)
    parser.add_argument("--sbf-repair-simplify-time-s", type=float, default=0.05)
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.01)
    parser.add_argument("--epsilon-path", type=float, default=1e-6)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "tro2026_random_iris_np_gcs_anytime_strictaudit_20260511.json")
    parser.add_argument("--checkpoint-json", type=Path, default=None)
    parser.add_argument("--resume", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def prefix_build_s(guide_s: float, timings: list[float], target_regions: int) -> float:
    if not timings:
        return float(guide_s)
    return float(guide_s) + sum(float(value) for value in timings[: max(1, min(int(target_regions), len(timings)))])


def guide_seed_budget(path: list[list[float]], max_regions: int) -> int:
    max_regions = max(1, int(max_regions))
    if len(path) <= max_regions:
        return len(path)
    # Avoid dropping narrow connector seeds when the guide only barely exceeds
    # the largest prefix budget.
    if len(path) <= max_regions + 4:
        return len(path)
    return max_regions


def unified_final_simplify_enabled(args: argparse.Namespace) -> bool:
    return float(getattr(args, "final_ompl_simplify_time_s", 0.0)) > 0.0


def audit_segment_step(args: argparse.Namespace) -> float:
    value = float(getattr(args, "audit_segment_step", 0.0) or 0.0)
    return value if value > 0.0 else float(args.segment_step)


def repair_sbf_audit_path(args: argparse.Namespace, robot: Any, obstacles: list[Any], path: list[list[float]], seed: int) -> dict[str, Any]:
    import time

    strict_step = audit_segment_step(args)
    if len(path) < 2:
        return {"ok": False, "time_s": 0.0, "path": [], "note": "empty_path"}
    if sbf.check_config_collision(robot, obstacles, list(path[0])) or sbf.check_config_collision(robot, obstacles, list(path[-1])):
        return {"ok": False, "time_s": 0.0, "path": [], "note": "terminal_endpoint_collision"}
    repaired = [list(path[0])]
    repair_time_s = 0.0
    repaired_segments = 0
    skipped_collision_waypoints = 0
    index = 0
    while index < len(path) - 1:
        start = list(repaired[-1])
        next_index = index + 1
        while next_index < len(path) - 1 and sbf.check_config_collision(robot, obstacles, list(path[next_index])):
            skipped_collision_waypoints += 1
            next_index += 1
        goal = list(path[next_index])
        if segment_free(robot, obstacles, start, goal, strict_step):
            repaired.append(goal)
            index = next_index
            continue
        t0 = time.perf_counter()
        raw = dict(sbf.ompl_rrt_connect_path(
            robot,
            obstacles,
            start,
            goal,
            float(args.sbf_repair_timeout_ms),
            float(args.sbf_repair_range),
            strict_step,
            0.0 if unified_final_simplify_enabled(args) else float(args.sbf_repair_simplify_time_s),
            int(seed) + 7919 * index,
        ))
        repair_time_s += float(raw.get("t_s", time.perf_counter() - t0) or 0.0)
        local_path = [list(point) for point in raw.get("path", [])]
        ok = bool(raw.get("ok")) and len(local_path) >= 2
        if not ok:
            return {"ok": False, "time_s": repair_time_s, "path": repaired, "note": str(raw.get("reason", "sbf_rrt_repair_failed")), "repaired_segments": repaired_segments, "skipped_collision_waypoints": skipped_collision_waypoints}
        if not all(segment_free(robot, obstacles, local_path[j], local_path[j + 1], strict_step) for j in range(len(local_path) - 1)):
            return {"ok": False, "time_s": repair_time_s, "path": repaired, "note": "sbf_rrt_repair_audit_failed", "repaired_segments": repaired_segments, "skipped_collision_waypoints": skipped_collision_waypoints}
        repaired.extend(local_path[1:])
        repaired_segments += 1
        index = next_index
    if not all(segment_free(robot, obstacles, repaired[j], repaired[j + 1], strict_step) for j in range(len(repaired) - 1)):
        return {"ok": False, "time_s": repair_time_s, "path": repaired, "note": "final_audit_failed", "repaired_segments": repaired_segments, "skipped_collision_waypoints": skipped_collision_waypoints}
    return {"ok": True, "time_s": repair_time_s, "path": repaired, "path_length": list_path_length(repaired), "repaired_segments": repaired_segments, "skipped_collision_waypoints": skipped_collision_waypoints}


def run_case(args: argparse.Namespace, robot_name: str, difficulty: str, scene_seed: int, trial: int, stages: list[int]) -> list[dict[str, Any]]:
    print(f"[random-iris-anytime] robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial}", flush=True)
    robot = make_robot(robot_name)
    scene = make_random_scene(robot_name, difficulty, int(args.seed_base) + 1009 * scene_seed, scene_profile=args.scene_profile)
    strict_step = audit_segment_step(args)
    planner_seed = int(args.seed_base) + 73856093 * scene_seed + 19349663 * trial
    guide, guide_s, guide_result = guide_path(args, robot, scene.obstacles, scene.start, scene.goal, planner_seed)
    robot_diagram, plant, model_instance, checker = build_drake_random_scene(robot_name, scene.obstacles)
    seed_points = [
        point
        for point in region_seed_points(guide, guide_seed_budget(guide, max(stages)))
        if checker.CheckConfigCollisionFree(point)
    ]
    if not seed_points:
        seed_points = [np.asarray(scene.start, dtype=float), np.asarray(scene.goal, dtype=float)]
    regions, timings, failures = build_regions(args, robot_diagram, plant, model_instance, checker, seed_points, planner_seed, float(args.budget_s))

    seed_index = scene_seed * max(1, int(args.trials)) + trial
    task_name = f"{robot_name}:{difficulty}:{scene_seed}:trial{trial}"
    incumbents: dict[str, dict[str, Any]] = {}
    records: list[dict[str, Any]] = []
    for stage_index, target in enumerate(stages):
        prefix = regions[: min(int(target), len(regions))]
        result = solve_regions_gcs(
            np.asarray(scene.start, dtype=float),
            np.asarray(scene.goal, dtype=float),
            prefix,
            seed=planner_seed + 101 * stage_index,
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
        audit_passed = path_passes_post_audit(
            sbf,
            robot,
            scene.obstacles,
            path,
            segment_step=strict_step,
            start=list(scene.start),
            goal=list(scene.goal),
        ) if drake_success else False
        repair: dict[str, Any] = {}
        if drake_success and not audit_passed:
            repair = repair_sbf_audit_path(args, robot, scene.obstacles, path, planner_seed + 10007 * stage_index)
            if repair.get("ok"):
                path = [list(point) for point in repair.get("path", [])]
                audit_passed = True
        query_s = float(result.get("time_s", 0.0) or 0.0) + float(repair.get("time_s", 0.0) or 0.0)
        if drake_success and audit_passed:
            final_simplify = final_ompl_simplify_path(
                sbf,
                robot,
                scene.obstacles,
                path,
                segment_step=float(args.segment_step),
                audit_segment_step=strict_step,
                simplify_time_s=float(args.final_ompl_simplify_time_s),
                epsilon_path=float(args.epsilon_path),
            )
            path = [list(point) for point in final_simplify["path"]]
            query_s += float(final_simplify["query_s"])
            audit_passed = path_passes_post_audit(
                sbf,
                robot,
                scene.obstacles,
                path,
                segment_step=strict_step,
                start=list(scene.start),
                goal=list(scene.goal),
            )
        path_length = list_path_length(path) if drake_success and audit_passed else None
        task = task_result(
            name=task_name,
            ok=bool(drake_success and audit_passed),
            audit_passed=bool(drake_success and audit_passed),
            path_length=path_length,
            query_s=query_s,
            reason=None if drake_success and audit_passed else str(repair.get("note") or result.get("note", "audit_failed" if drake_success else "gcs_failed")),
            extra={
                "raw": {key: value for key, value in result.items() if key != "path"},
                "repair": repair,
                "n_regions": len(prefix),
                "guide_ok": bool(guide_result.get("ok")),
                "waypoints": path,
                "waypoint_count": len(path),
                "ompl_final_simplify_time_s": float(final_simplify["query_s"]) if drake_success and audit_passed else 0.0,
                "ompl_final_simplify_applied": bool(final_simplify["applied"]) if drake_success and audit_passed else False,
                "ompl_final_simplify_reason": str(final_simplify["reason"]) if drake_success and audit_passed else "disabled",
            },
        )
        incumbents, improved = update_incumbents(incumbents, [task], epsilon_path=float(args.epsilon_path))
        build_s = prefix_build_s(float(guide_s), [float(value) for value in timings], int(target))
        records.append(incumbent_stage_record(
            method="drake_iris_np_gcs",
            stage_id=f"r{target}",
            stage_index=stage_index,
            seed_index=seed_index,
            task_count=1,
            cumulative_build_s=build_s,
            cumulative_query_s=query_s,
            stage_build_s=build_s,
            stage_query_s=query_s,
            raw_tasks=[task],
            incumbents=incumbents,
            improved_tasks=improved,
            params={
                "robot": robot_name,
                "difficulty": difficulty,
                "scene_seed": int(scene_seed),
                "trial": int(trial),
                "target_regions": int(target),
                "available_regions": len(regions),
                "failed_region_seeds": failures,
            },
            protocol="independent_prefix_exit_condition",
        ))
    return records


def aggregate_panels(records: list[dict[str, Any]], epsilon_path: float) -> dict[str, Any]:
    panels: dict[str, dict[str, Any]] = {}
    for record in records:
        params = record.get("params", {})
        key = f"{params.get('robot')}:{params.get('difficulty')}"
        panels.setdefault(key, {"robot": params.get("robot"), "difficulty": params.get("difficulty"), "records": []})["records"].append(record)
    for panel in panels.values():
        panel["summary"] = aggregate_stage_records(panel["records"], epsilon_path=epsilon_path)
    return panels


def case_key(robot_name: str, difficulty: str, scene_seed: int, trial: int) -> tuple[str, str, int, int]:
    return (str(robot_name), str(difficulty), int(scene_seed), int(trial))


def completed_case_keys(records: list[dict[str, Any]], expected_stages: int) -> set[tuple[str, str, int, int]]:
    grouped: dict[tuple[str, str, int, int], set[int]] = {}
    for record in records:
        params = record.get("params", {})
        key = case_key(params.get("robot"), params.get("difficulty"), params.get("scene_seed", -1), params.get("trial", -1))
        grouped.setdefault(key, set()).add(int(record.get("stage_index", -1)))
    return {key for key, stages in grouped.items() if len(stages) >= int(expected_stages)}


def make_payload(args: argparse.Namespace, records: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "experiment": "tro2026_random_iris_np_gcs_anytime",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_random_iris_np_gcs_prefix_exit_tradeoff",
        "note": "IRIS regions are grown once per scene from guide-path seeds. Prefix region counts emulate earlier/later IRIS exit conditions; GCS is solved independently at each prefix and endpoint-aware strict SBF path audit is applied before a path is counted.",
        "params": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "scene_filter_protocol": {
            "fixed_robot_exclusion_margin_m": float(FIXED_ROBOT_CLEARANCE_MARGIN_M),
            "endpoint_clearance_margin_m": float(ENDPOINT_CLEARANCE_MARGIN_M),
            "direct_segment_required_blocked": True,
            "direct_segment_resolution": int(SEGMENT_RESOLUTION),
            "balanced_probe_required": scene_profile_requires_balanced_probe(str(args.scene_profile)),
            "path_audit_segment_step": float(audit_segment_step(args)),
        },
        "panels": aggregate_panels(records, float(args.epsilon_path)),
        "records": records,
    }


def main() -> int:
    args = parse_args()
    configure_threads(int(args.logical_threads))
    stages = parse_int_grid(args.stage_region_counts)
    checkpoint_json = args.checkpoint_json or args.out_json.with_suffix(args.out_json.suffix + ".checkpoint.json")
    records: list[dict[str, Any]] = []
    if bool(args.resume) and checkpoint_json.exists():
        checkpoint = json.loads(checkpoint_json.read_text(encoding="utf-8"))
        records = list(checkpoint.get("records", []))
        print(json.dumps({"resume_checkpoint": str(checkpoint_json), "records": len(records), "completed_cases": len(completed_case_keys(records, len(stages)))}, sort_keys=True), flush=True)
    completed = completed_case_keys(records, len(stages))
    for robot_name in parse_csv(args.robots):
        for difficulty in parse_csv(args.difficulties):
            for scene_seed in range(max(1, int(args.scene_seeds))):
                for trial in range(max(1, int(args.trials))):
                    key = case_key(robot_name, difficulty, scene_seed, trial)
                    if key in completed:
                        print(f"[random-iris-anytime] skip completed robot={robot_name} difficulty={difficulty} scene_seed={scene_seed} trial={trial}", flush=True)
                        continue
                    records.extend(run_case(args, robot_name, difficulty, scene_seed, trial, stages))
                    completed.add(key)
                    checkpoint_payload = make_payload(args, records)
                    checkpoint_payload["checkpoint_complete_cases"] = len(completed)
                    write_json(checkpoint_json, checkpoint_payload)
                    print(json.dumps({"checkpoint_json": str(checkpoint_json), "records": len(records), "completed_cases": len(completed)}, sort_keys=True), flush=True)
    payload = make_payload(args, records)
    write_json(args.out_json, payload)
    print(json.dumps({"out_json": str(args.out_json), "records": len(records), "panels": sorted(payload["panels"])}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())