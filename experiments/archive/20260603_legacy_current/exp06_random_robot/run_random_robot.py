#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (REPO_ROOT, REPO_ROOT.parent):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

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
from experiments.common.random_scene_catalog import generate_catalog  # noqa: E402
from experiments.common.random_robot_cache import (  # noqa: E402
    DEFAULT_RANDOM_CACHE_RUN_ID,
    DEFAULT_RANDOM_P18_ENVELOPE,
    DEFAULT_RANDOM_P18_MAX_DEPTH,
    DEFAULT_RANDOM_P18_PREWARM_DEPTH,
    DEFAULT_RANDOM_P18_THREADS,
    ensure_random_robot_p18_cache,
    seed_scene_stage_eval_caches_from_p18,
)


LOCAL_IRIS_PYTHON = Path("/home/tian/miniconda3/envs/sbf/bin/python")


def default_iris_python() -> str:
    return str(LOCAL_IRIS_PYTHON if LOCAL_IRIS_PYTHON.exists() else Path(sys.executable))


def iris_dependency_status(python_executable: str) -> dict[str, Any]:
    missing: list[str] = []
    if not Path(python_executable).exists():
        missing.append(str(python_executable))
    env = os.environ.copy()
    pythonpath_entries = [
        str(REPO_ROOT / "build-exp04" / "python"),
        str(REPO_ROOT.parent),
    ]
    existing = env.get("PYTHONPATH", "")
    if existing:
        pythonpath_entries.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(pythonpath_entries)
    probe = None
    if not missing:
        probe = subprocess.run(
            [str(python_executable), "-c", "import numpy, pydrake, sbf"],
            cwd=str(REPO_ROOT),
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )
        if probe.returncode != 0:
            missing.append("iris_import_probe_failed")
    return {
        "ok": not missing,
        "missing": missing,
        "python": str(python_executable),
        "stdout_tail": (probe.stdout[-2000:] if probe is not None else ""),
        "stderr_tail": (probe.stderr[-2000:] if probe is not None else ""),
    }


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
    parser.add_argument("--prm-build-grid-s", default="0.25,0.5,1,2,5")
    parser.add_argument("--prm-query-budget-s", type=float, default=1.0)
    parser.add_argument("--bitstar-timeout-s", type=float, default=5.0)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=1.0)
    parser.add_argument("--rrt-timeout-ms", type=float, default=10000.0)
    parser.add_argument("--iris-python", default=default_iris_python())
    parser.add_argument("--iris-budget-s", type=float, default=420.0)
    parser.add_argument("--iris-stage-region-counts", default="3,5,7,9,12,16,20")
    parser.add_argument("--iris-iteration-limit", type=int, default=8)
    parser.add_argument("--iris-query-time-limit-s", type=float, default=90.0)
    parser.add_argument("--iris-rounding-max-paths", type=int, default=24)
    parser.add_argument("--iris-rounding-max-trials", type=int, default=240)
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument(
        "--scene-catalog-mode",
        choices=["auto", "generate", "reuse"],
        default="auto",
        help="random scene catalog lifecycle: auto=generate if needed, generate=always regenerate, reuse=require existing catalog.",
    )
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
    scene_catalog = args.scene_catalog if args.scene_catalog is not None else args.out_dir / "random_scene_catalog.json"

    anytime_baselines = [method for method in ("prm", "bitstar", "rrtconnect") if method in methods]
    if "sbf" in methods:
        rows.append({
            "name": "random_sbf_anytime",
            "methods": ["sbf"],
            "kind": "random_anytime",
            "description": "Random-scene staged SBF anytime artifact under the unified d40 SBF schedule.",
            "command": build_random_anytime_command(
                python_executable=sys.executable,
                out_json=args.out_dir / "random_sbf_anytime.json",
                robots=str(args.robots),
                difficulties=str(args.difficulties),
                scene_seeds=int(args.scene_seeds),
                scene_profile=str(args.scene_profile),
                threads=int(args.threads),
                trials=int(args.trials),
                methods="support_hull_coverage",
                baseline_methods="",
                cache_root=args.rbf_cache_root,
                cache_run_id=str(args.cache_run_id),
                clear_cache=False,
                prewarm_depth=int(args.prewarm_depth),
                scene_catalog=scene_catalog,
                scene_catalog_mode=str(args.scene_catalog_mode),
                prm_build_grid_s=str(args.prm_build_grid_s),
                prm_query_budget_s=float(args.prm_query_budget_s),
                bitstar_timeout_s=float(args.bitstar_timeout_s),
                bitstar_checkpoint_interval_s=float(args.bitstar_checkpoint_interval_s),
                rrt_timeout_ms=float(args.rrt_timeout_ms),
            ),
        })
    for method in anytime_baselines:
        legacy_name = "rrt" if method == "rrtconnect" else method
        rows.append({
            "name": f"random_{method}_anytime",
            "methods": [method],
            "kind": "random_anytime",
            "description": f"Random-scene {method} anytime artifact.",
            "command": build_random_anytime_command(
                python_executable=sys.executable,
                out_json=args.out_dir / f"random_{method}_anytime.json",
                robots=str(args.robots),
                difficulties=str(args.difficulties),
                scene_seeds=int(args.scene_seeds),
                scene_profile=str(args.scene_profile),
                threads=int(args.threads),
                trials=int(args.trials),
                methods="",
                baseline_methods=legacy_name,
                cache_root=args.rbf_cache_root,
                cache_run_id=str(args.cache_run_id),
                clear_cache=False,
                prewarm_depth=int(args.prewarm_depth),
                scene_catalog=scene_catalog,
                scene_catalog_mode=str(args.scene_catalog_mode),
                prm_build_grid_s=str(args.prm_build_grid_s),
                prm_query_budget_s=float(args.prm_query_budget_s),
                bitstar_timeout_s=float(args.bitstar_timeout_s),
                bitstar_checkpoint_interval_s=float(args.bitstar_checkpoint_interval_s),
                rrt_timeout_ms=float(args.rrt_timeout_ms),
            ),
        })
    if "iris_np" in methods:
        deps = iris_dependency_status(str(args.iris_python))
        row = {
            "name": "iris_np_gcs_random_anytime",
            "methods": ["iris_np"],
            "kind": "random_iris_anytime" if deps["ok"] else "skipped_dependency",
            "description": "IRIS-NP+GCS random-scene prefix anytime artifact.",
            "dependency_status": deps,
        }
        if deps["ok"]:
            row["command"] = build_random_iris_anytime_command(
                python_executable=str(args.iris_python),
                out_json=args.out_dir / "iris_np_gcs_random_anytime.json",
                robots=str(args.robots),
                difficulties=str(args.difficulties),
                scene_seeds=int(args.scene_seeds),
                scene_profile=str(args.scene_profile),
                threads=int(args.threads),
                trials=int(args.trials),
                budget_s=float(args.iris_budget_s),
                stage_region_counts=str(args.iris_stage_region_counts),
                iteration_limit=int(args.iris_iteration_limit),
                query_time_limit_s=float(args.iris_query_time_limit_s),
                rounding_max_paths=int(args.iris_rounding_max_paths),
                rounding_max_trials=int(args.iris_rounding_max_trials),
            )
        rows.append(row)
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
    seed_eval = {
        "dry_run": bool(args.dry_run or not args.execute),
        "seeded_namespace_count": 0,
        "seeded_namespaces": [],
        "mode": "external_read_only_prewarm",
    }
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
    if args.scene_catalog is None:
        args.scene_catalog = args.out_dir / "random_scene_catalog.json"
    scene_catalog_summary: dict[str, Any] = {
        "path": str(args.scene_catalog),
        "mode": str(args.scene_catalog_mode),
        "generated_or_verified": False,
    }
    if args.execute and not args.dry_run:
        catalog = generate_catalog(
            path=args.scene_catalog,
            robots=csv_list(args.robots),
            difficulties=csv_list(args.difficulties),
            scene_seeds=int(args.scene_seeds),
            scene_profile=str(args.scene_profile),
            seed_base=9176,
            mode=str(args.scene_catalog_mode),
        )
        scene_catalog_summary.update({
            "generated_or_verified": True,
            "schema": catalog.get("schema"),
            "records": len(catalog.get("records", [])),
            "robots": catalog.get("robots"),
            "difficulties": catalog.get("difficulties"),
            "scene_seeds": catalog.get("scene_seeds"),
        })
    cache_info = prepare_random_sbf_caches(args)
    rows = command_rows(args)
    run_records = []
    extra_env = default_sbf_subprocess_env()
    if args.execute:
        for row in rows:
            if row.get("kind") == "skipped_dependency":
                run_records.append({
                    "name": row["name"],
                    "measurement": {
                        "dry_run": bool(args.dry_run),
                        "returncode": None,
                        "skipped": True,
                        "reason": "missing_dependency",
                        "dependency_status": row.get("dependency_status", {}),
                    },
                })
                continue
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
        "scene_catalog": scene_catalog_summary,
        "commands": rows,
        "runs": run_records,
        "notes": [
            "Experiment 6 dispatches random-scene anytime artifacts instead of separate one-shot baseline scripts.",
            "The random anytime backend uses the unified d40 SBF schedule for its SBF method and writes PRM/BIT*/RRTConnect into separate artifacts when requested.",
            "IRIS-NP+GCS remains a separate random prefix-anytime artifact because it uses a different backend and accounting model.",
            "Random SBF runs seed every scene-stage namespace from a canonical native p20 cache per robot, then writes warm planning updates into the evaluation cache namespaces.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
