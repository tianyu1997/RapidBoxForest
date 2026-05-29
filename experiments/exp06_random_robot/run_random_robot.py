#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    csv_list,
    default_sbf_subprocess_env,
    environment_metadata,
    namespace_dict,
    run_command,
    run_id,
    write_json,
)
from experiments.common.cache_maintenance import prune_directory_children  # noqa: E402
from experiments.common.anytime_defaults import UNIFIED_SBF_ANYTIME_STAGE_IDS  # noqa: E402
from experiments.common.lect_db_dispatch import (  # noqa: E402
    build_random_anytime_command,
    build_random_iris_anytime_command,
)
from experiments.common.random_robot_cache import (  # noqa: E402
    DEFAULT_RANDOM_CACHE_RUN_ID,
    DEFAULT_RANDOM_P18_ENVELOPE,
    DEFAULT_RANDOM_P18_MAX_DEPTH,
    DEFAULT_RANDOM_P18_PREWARM_DEPTH,
    DEFAULT_RANDOM_P18_THREADS,
    ensure_random_robot_p18_cache,
    seed_scene_stage_eval_caches_from_p18,
)


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp06_random_robot"
    parser = argparse.ArgumentParser(description="Run Experiment 6 as a random-scene anytime trade-off dispatcher.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--methods", default="sbf,iris_np,prm,rrtconnect,bitstar")
    parser.add_argument("--scene-seeds", type=int, default=5)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--cache-run-id", default=DEFAULT_RANDOM_CACHE_RUN_ID)
    parser.add_argument("--prewarm-depth", type=int, default=DEFAULT_RANDOM_P18_PREWARM_DEPTH)
    parser.add_argument("--prewarm-threads", type=int, default=DEFAULT_RANDOM_P18_THREADS)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def command_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    methods = set(csv_list(args.methods))
    rows: list[dict[str, Any]] = []
    anytime_baselines = [method for method in ("prm", "bitstar", "rrtconnect") if method in methods]
    anytime_sbf_methods = "support_hull_coverage" if "sbf" in methods else ""
    if "sbf" in methods or anytime_baselines:
        rows.append({
            "name": "random_anytime_tradeoff",
            "methods": (["sbf"] if "sbf" in methods else []) + anytime_baselines,
            "kind": "random_anytime",
            "description": "Random-scene anytime trade-off artifact for SBF, PRM, BIT*, and/or RRTConnect under the unified d40 SBF schedule.",
            "command": build_random_anytime_command(
                python_executable=sys.executable,
                out_json=args.out_dir / "random_anytime_tradeoff.json",
                robots=str(args.robots),
                difficulties=str(args.difficulties),
                scene_seeds=int(args.scene_seeds),
                scene_profile=str(args.scene_profile),
                threads=int(args.threads),
                trials=int(args.trials),
                methods=anytime_sbf_methods,
                baseline_methods=",".join("rrt" if method == "rrtconnect" else method for method in anytime_baselines),
                cache_root=args.rbf_cache_root,
                cache_run_id=str(args.cache_run_id),
                clear_cache=False,
            ),
        })
    if "iris_np" in methods:
        rows.append({
            "name": "iris_np_gcs_random_anytime",
            "methods": ["iris_np"],
            "kind": "random_iris_anytime",
            "description": "IRIS-NP+GCS random-scene prefix anytime artifact.",
            "command": build_random_iris_anytime_command(
                python_executable=sys.executable,
                out_json=args.out_dir / "iris_np_gcs_random_anytime.json",
                robots=str(args.robots),
                difficulties=str(args.difficulties),
                scene_seeds=int(args.scene_seeds),
                scene_profile=str(args.scene_profile),
                threads=int(args.threads),
                trials=int(args.trials),
            ),
        })
    return rows


def prepare_random_sbf_caches(args: argparse.Namespace) -> dict[str, Any]:
    methods = set(csv_list(args.methods))
    if "sbf" not in methods:
        return {
            "enabled": False,
            "root": str(args.rbf_cache_root),
            "p18": [],
            "prune": {
                "path": str(args.rbf_cache_root),
                "kept": [],
                "removed": [],
                "removed_count": 0,
                "dry_run": bool(args.dry_run or not args.execute),
            },
            "seed_eval": {"dry_run": bool(args.dry_run or not args.execute), "seeded_namespace_count": 0, "seeded_namespaces": []},
        }
    robot_names = csv_list(args.robots)
    difficulty_names = csv_list(args.difficulties)
    p18_rows = [
        ensure_random_robot_p18_cache(
            cache_root=args.rbf_cache_root,
            robot_name=robot_name,
            prewarm_depth=int(args.prewarm_depth),
            envelope=DEFAULT_RANDOM_P18_ENVELOPE,
            prewarm_threads=int(args.prewarm_threads),
            max_depth=DEFAULT_RANDOM_P18_MAX_DEPTH,
            dry_run=bool(args.dry_run or not args.execute),
        )
        for robot_name in robot_names
    ]
    keep_names = [str(row["cache_label"]) for row in p18_rows]
    prune_summary: dict[str, Any] = {
        "path": str(args.rbf_cache_root),
        "kept": keep_names,
        "removed": [],
        "removed_count": 0,
        "dry_run": bool(args.dry_run or not args.execute),
    }
    if args.execute and not args.dry_run:
        prune_summary = prune_directory_children(args.rbf_cache_root, keep_names)
    seed_eval = seed_scene_stage_eval_caches_from_p18(
        cache_root=args.rbf_cache_root,
        cache_run_id=str(args.cache_run_id),
        robot_names=robot_names,
        method_names=["support_hull_coverage"],
        stage_ids=UNIFIED_SBF_ANYTIME_STAGE_IDS,
        difficulties=difficulty_names,
        scene_seeds=int(args.scene_seeds),
        p18_cache_labels={str(row["robot"]): str(row["cache_label"]) for row in p18_rows},
        dry_run=bool(args.dry_run or not args.execute),
    )
    return {
        "enabled": True,
        "root": str(args.rbf_cache_root),
        "cache_run_id": str(args.cache_run_id),
        "p18": p18_rows,
        "prune": prune_summary,
        "seed_eval": seed_eval,
    }


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "random_robot_manifest.json")
    cache_info = prepare_random_sbf_caches(args)
    rows = command_rows(args)
    run_records = []
    extra_env = default_sbf_subprocess_env()
    if args.execute:
        for row in rows:
            run_records.append({
                "name": row["name"],
                "measurement": run_command(row["command"], dry_run=bool(args.dry_run), extra_env=extra_env),
            })

    payload = {
        "experiment": "exp06_random_robot",
        "plan_file": "experiments/06_random_robot_plan.md",
        "run_id": run_id("exp06"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": cache_info,
        "commands": rows,
        "runs": run_records,
        "notes": [
            "Experiment 6 now dispatches random-scene anytime artifacts instead of separate one-shot baseline scripts.",
            "The random anytime backend uses the unified d40 SBF schedule for its SBF method and keeps PRM/BIT*/RRTConnect in the same anytime artifact when requested.",
            "IRIS-NP+GCS remains a separate random prefix-anytime artifact because it uses a different backend and accounting model.",
            "Random SBF runs seed every scene-stage namespace from a canonical native p18 cache per robot, then write subsequent warm updates into the evaluation cache namespaces.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
