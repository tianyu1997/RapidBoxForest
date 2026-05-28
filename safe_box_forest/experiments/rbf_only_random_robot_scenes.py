from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path
from typing import Any

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import (
    RBF_LIFELONG_PRESET,
    RBF_ONLY_OUTPUT_ROOT,
    add_common_sbf_args,
    configure_external_evidence_reuse,
    configure_standalone_sbf,
    mean,
    median,
    query_result_payload,
    rbf_lifelong_config_metadata,
    set_online_cache_backfill,
    write_json,
)
from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_scene_sampling import (
    DEFAULT_RANDOM_DIFFICULTIES,
    DEFAULT_RANDOM_ROBOTS,
    make_random_scene,
    make_robot,
)

import sbf


def parse_csv(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def directory_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def far_obstacle() -> Any:
    return sbf.Obstacle(100.0, 100.0, 100.0, 101.0, 101.0, 101.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the RBF-only E4 random robot scenes pilot/main experiment.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset=RBF_LIFELONG_PRESET,
        rbf_envelope="link",
        rbf_prewarm_depth=18,
        max_boxes=20000,
        timeout_ms=120000.0,
        threads=8,
        task_batch_size=8,
        worker_local_ffb=True,
        enable_connector=True,
        connector_bridge_boxes=0,
        ffb_depth=160,
        component_connect_ffb_max_depth=200,
        post_connect_extra_boxes=2000,
        quality_min_connected_boxes=512,
        post_connect_time_budget_ms=5000.0,
        repair_timeout_ms=1500.0,
    )
    parser.add_argument("--robots", default=DEFAULT_RANDOM_ROBOTS)
    parser.add_argument("--difficulties", default=DEFAULT_RANDOM_DIFFICULTIES)
    parser.add_argument("--scene-seeds", type=int, default=5)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--modes", default="warm_d18")
    parser.add_argument("--iiwa-warm-cache-label", default="e5_lifelong_cache_link_d18_smoke")
    parser.add_argument("--clean-prewarm-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--clean-cold-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clean-warm-active-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--out-json", type=Path, default=RBF_ONLY_OUTPUT_ROOT / "e4_random_robot_scenes.json")
    return parser.parse_args()


def warm_cache_label(args: argparse.Namespace, robot_name: str) -> str:
    if robot_name == "iiwa":
        return str(args.iiwa_warm_cache_label)
    return f"e4_lifelong_cache_link_d{int(args.rbf_prewarm_depth)}_{robot_name}"


def make_config(args: argparse.Namespace, robot: Any, cache_label: str, seed: int) -> Any:
    local_args = argparse.Namespace(**vars(args))
    local_args.rbf_cache_label = cache_label
    cfg = configure_standalone_sbf(local_args, seed=seed, preset=RBF_LIFELONG_PRESET, robot=robot)
    set_online_cache_backfill(cfg, allow_database_backfill=True)
    return cfg


def ensure_warm_cache(args: argparse.Namespace, robot_name: str, robot: Any) -> dict[str, Any]:
    cache_label = warm_cache_label(args, robot_name)
    cache_path = Path(args.rbf_cache_root) / cache_label
    if robot_name == "iiwa" and cache_path.exists() and not args.clean_prewarm_cache:
        return {
            "ok": True,
            "robot": robot_name,
            "cache_label": cache_label,
            "cache_path": str(cache_path),
            "cache_bytes": directory_size_bytes(cache_path),
            "reused_existing": True,
        }

    cfg = make_config(args, robot, cache_label, seed=0)
    if args.clean_prewarm_cache and cache_path.exists():
        shutil.rmtree(cache_path)
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    result = dict(forest.prewarm_lifelong_cache(int(args.rbf_prewarm_depth), [far_obstacle()]))
    wall_s = time.perf_counter() - t0
    payload = {
        "ok": bool(result.get("ok")),
        "robot": robot_name,
        "cache_label": cache_label,
        "cache_path": str(cache_path),
        "cache_bytes": directory_size_bytes(cache_path),
        "wall_s": float(wall_s),
        "result": result,
        "metadata": rbf_lifelong_config_metadata(cfg, args),
        "reused_existing": False,
    }
    del forest
    return payload


def failure_query_payload(reason: str) -> dict[str, Any]:
    return {
        "ok": False,
        "success": False,
        "audit_passed": False,
        "repair_count": 0,
        "t_s": 0.0,
        "reason": reason,
    }


def run_case(args: argparse.Namespace,
             robot_name: str,
             difficulty: str,
             scene_seed: int,
             mode: str,
             prewarm_info: dict[str, Any]) -> dict[str, Any]:
    print(f"[e4] start robot={robot_name} difficulty={difficulty} seed={scene_seed} mode={mode}", flush=True)
    try:
        scene = make_random_scene(robot_name, difficulty, int(args.seed_base) + 1009 * scene_seed, scene_profile=args.scene_profile)
        robot = make_robot(robot_name)
    except RuntimeError as exc:
        reason = f"scene_generation_failed:{exc}"
        print(f"[e4] skipped robot={robot_name} difficulty={difficulty} seed={scene_seed} mode={mode} reason={reason}", flush=True)
        return {
            "ok": False,
            "robot": robot_name,
            "difficulty": difficulty,
            "scene_seed": int(scene_seed),
            "mode": mode,
            "scene_valid": False,
            "failure_reason": reason,
            "build": {"wall_s": 0.0, "planning_s": 0.0, "maintenance_s": 0.0, "diagnostics": {}},
            "query": failure_query_payload(reason),
        }

    cache_label = f"e4_{mode}_{robot_name}_{difficulty}_seed{scene_seed}"
    cfg = make_config(args, robot, cache_label, seed=scene_seed)
    cache_path = Path(cfg.database.path)
    if mode == "cold" and args.clean_cold_cache and cache_path.exists():
        shutil.rmtree(cache_path)
    if mode == "warm_d18":
        warm_path = Path(prewarm_info["cache_path"])
        if not warm_path.exists():
            raise FileNotFoundError(f"warm d18 cache does not exist: {warm_path}")
        if args.clean_warm_active_cache and cache_path.exists():
            shutil.rmtree(cache_path)
        configure_external_evidence_reuse(cfg, warm_path, args, backfill_active=False)
    cfg.database.create_if_missing = True

    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(scene.obstacles, [scene.start, scene.goal])
    build_wall_s = time.perf_counter() - build_t0
    planning_s = float(profile.total_ms) / 1000.0
    query_t0 = time.perf_counter()
    query = forest.query(scene.start, scene.goal)
    query_payload = query_result_payload(f"{robot_name}:{difficulty}:{scene_seed}", query, time.perf_counter() - query_t0)
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    row = {
        "ok": bool(query_payload["audit_passed"]),
        "robot": robot_name,
        "difficulty": difficulty,
        "scene_seed": int(scene_seed),
        "mode": mode,
        "scene_valid": True,
        "metadata": rbf_lifelong_config_metadata(cfg, args),
        "warm_cache_label": prewarm_info["cache_label"],
        "cache_path": str(cache_path),
        "external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "cache_bytes": directory_size_bytes(cache_path),
        "endpoint_clearance_margin_m": float(scene.endpoint_clearance_margin_m),
        "direct_segment_blocked": bool(scene.direct_segment_blocked),
        "segment_resolution": int(scene.segment_resolution),
        "obstacle_count": len(scene.obstacles),
        "build": {
            "wall_s": float(build_wall_s),
            "planning_s": planning_s,
            "maintenance_s": max(0.0, float(build_wall_s) - planning_s),
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "connector_ms": float(profile.connector_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "segment_edges": int(profile.segment_edges),
            "adjacency_islands": int(profile.adjacency_islands),
            "diagnostics": diagnostics,
        },
        "query": query_payload,
    }
    print(
        f"[e4] done robot={robot_name} difficulty={difficulty} seed={scene_seed} mode={mode} "
        f"ok={row['ok']} planning_s={planning_s:.3f} "
        f"external_hits={diagnostics.get('oracle.materialization_reused_external_evidence', 0.0):.0f}",
        flush=True,
    )
    del forest
    return row


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in rows:
        if not row.get("scene_valid", False):
            continue
        groups.setdefault((str(row["robot"]), str(row["difficulty"]), str(row["mode"])), []).append(row)
    summaries: list[dict[str, Any]] = []
    for (robot, difficulty, mode), items in sorted(groups.items()):
        summaries.append({
            "robot": robot,
            "difficulty": difficulty,
            "mode": mode,
            "n": len(items),
            "build_mean_s": mean(float(item["build"]["wall_s"]) for item in items),
            "build_median_s": median(float(item["build"]["wall_s"]) for item in items),
            "planning_mean_s": mean(float(item["build"]["planning_s"]) for item in items),
            "audit_sr": mean(1.0 if item["query"]["audit_passed"] else 0.0 for item in items),
            "query_median_s": median(float(item["query"]["t_s"]) for item in items),
            "repair_mean": mean(float(item["query"]["repair_count"]) for item in items),
            "external_hits_mean": mean(float(item["build"]["diagnostics"].get("oracle.materialization_reused_external_evidence", 0.0)) for item in items),
        })
    return summaries


def main() -> int:
    args = parse_args()
    robots = parse_csv(args.robots)
    difficulties = parse_csv(args.difficulties)
    modes = parse_csv(args.modes)

    warm_cache_info: dict[str, dict[str, Any]] = {}
    for robot_name in robots:
        if "warm_d18" not in modes:
            break
        robot = make_robot(robot_name)
        warm_cache_info[robot_name] = ensure_warm_cache(args, robot_name, robot)

    rows: list[dict[str, Any]] = []
    for robot_name in robots:
        for difficulty in difficulties:
            for scene_seed in range(max(1, int(args.scene_seeds))):
                for mode in modes:
                    info = warm_cache_info.get(robot_name, {
                        "cache_label": "",
                        "cache_path": "",
                    })
                    rows.append(run_case(args, robot_name, difficulty, scene_seed, mode, info))

    payload = {
        "ok": all(bool(row.get("ok", False)) for row in rows if row.get("scene_valid", True)),
        "experiment": "e4_random_robot_scenes",
        "args": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "warm_cache": warm_cache_info,
        "rows": rows,
        "summary": summarize(rows),
    }
    write_json(args.out_json, payload)
    print(f"[e4] wrote {args.out_json} ok={payload['ok']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())