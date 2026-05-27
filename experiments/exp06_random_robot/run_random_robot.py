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
    environment_metadata,
    namespace_dict,
    run_command,
    run_id,
    write_json,
)


SBF_RANDOM = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_05_random_robot_scenes.py"
RRT_RANDOM = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_12_random_scene_rrt_baseline.py"
OMPL_RANDOM = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_12_random_scene_ompl_baselines.py"
IRIS_RANDOM = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_12_random_scene_iris_np_gcs_baseline.py"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp06_random_robot"
    parser = argparse.ArgumentParser(description="Run Experiment 6 random-scene cross-robot dispatcher.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--methods", default="sbf,iris_np,prm,rrtconnect,bitstar")
    parser.add_argument("--scene-seeds", type=int, default=5)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def common_scene_args(args: argparse.Namespace) -> list[str]:
    return [
        "--robots",
        str(args.robots),
        "--difficulties",
        str(args.difficulties),
        "--scene-seeds",
        str(args.scene_seeds),
        "--scene-profile",
        str(args.scene_profile),
    ]


def commands(args: argparse.Namespace) -> list[dict[str, Any]]:
    methods = set(csv_list(args.methods))
    rows: list[dict[str, Any]] = []
    scene_args = common_scene_args(args)
    if "sbf" in methods:
        rows.append({
            "name": "sbf_random_robot",
            "methods": ["sbf"],
            "command": [
                sys.executable,
                str(SBF_RANDOM),
                *scene_args,
                "--methods",
                "support_hull_coverage",
                "--out-json",
                str(args.out_dir / "sbf_random_robot.json"),
            ],
        })
    if "rrtconnect" in methods:
        rows.append({
            "name": "rrtconnect_random_robot",
            "methods": ["rrtconnect"],
            "command": [
                sys.executable,
                str(RRT_RANDOM),
                *scene_args,
                "--trials",
                str(args.trials),
                "--segment-step",
                "0.01",
                "--out-json",
                str(args.out_dir / "rrtconnect_random_robot.json"),
            ],
        })
    ompl_methods = [method for method in ("prm", "bitstar") if method in methods]
    if ompl_methods:
        rows.append({
            "name": "ompl_random_robot",
            "methods": ompl_methods,
            "command": [
                sys.executable,
                str(OMPL_RANDOM),
                *scene_args,
                "--methods",
                ",".join(ompl_methods),
                "--trials",
                str(args.trials),
                "--segment-step",
                "0.01",
                "--out-json",
                str(args.out_dir / "ompl_random_robot.json"),
            ],
        })
    if "iris_np" in methods:
        rows.append({
            "name": "iris_np_gcs_random_robot",
            "methods": ["iris_np"],
            "command": [
                sys.executable,
                str(IRIS_RANDOM),
                *scene_args,
                "--trials",
                "1",
                "--segment-step",
                "0.01",
                "--out-json",
                str(args.out_dir / "iris_np_gcs_random_robot.json"),
            ],
        })
    return rows


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "random_robot_manifest.json")
    rows = commands(args)
    run_records = []
    if args.execute:
        for row in rows:
            run_records.append({"name": row["name"], "measurement": run_command(row["command"], dry_run=bool(args.dry_run))})
    payload = {
        "experiment": "exp06_random_robot",
        "run_id": run_id("exp06"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "commands": rows,
        "runs": run_records,
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
