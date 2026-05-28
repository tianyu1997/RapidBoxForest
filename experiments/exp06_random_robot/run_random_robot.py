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
from experiments.common.shelf_iiwa_cache import (  # noqa: E402
    DEFAULT_P18_CACHE_LABEL,
    cache_file_sizes,
    directory_size,
    ensure_p18_prewarm_summary,
    load_json,
    read_manifest,
    snapshot_path_for_cache,
    snapshot_summary,
)


SBF_RANDOM = REPO_ROOT / "safe_box_forest" / "experiments" / "rbf_only_random_robot_scenes.py"
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
    parser.add_argument("--prewarm-json", type=Path, default=output_dir / "p18_prewarm.json")
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--prewarm-envelope", choices=["link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--iiwa-warm-cache-label", default=DEFAULT_P18_CACHE_LABEL)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--external-evidence-mode", choices=["legacy", "snapshot"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
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
                "--modes",
                "warm_d18",
                "--rbf-cache-root",
                str(args.rbf_cache_root),
                "--iiwa-warm-cache-label",
                str(args.iiwa_warm_cache_label),
                "--external-evidence-mode",
                str(args.external_evidence_mode),
                "--out-json",
                str(args.out_dir / "sbf_random_robot.json"),
            ],
        })
        if args.external_evidence_auto_build_snapshot:
            rows[-1]["command"].append("--external-evidence-auto-build-snapshot")
        else:
            rows[-1]["command"].append("--no-external-evidence-auto-build-snapshot")
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
    cache_path = args.rbf_cache_root / str(args.iiwa_warm_cache_label)
    prewarm_summary = load_json(args.prewarm_json)
    extra_env = default_sbf_subprocess_env()
    needs_prewarm = any("sbf" in row.get("methods", []) for row in rows)
    if args.execute and needs_prewarm:
        prewarm_summary = ensure_p18_prewarm_summary(
            args.prewarm_json,
            cache_path=cache_path,
            prewarm_depth=int(args.prewarm_depth),
            envelope=str(args.prewarm_envelope),
            prewarm_threads=int(args.prewarm_threads),
            clean_cache=bool(args.clean_cache),
            dry_run=bool(args.dry_run),
        )
    if args.execute:
        for row in rows:
            measurement_env = extra_env if "sbf" in row.get("methods", []) else None
            run_records.append({"name": row["name"], "measurement": run_command(row["command"], dry_run=bool(args.dry_run), extra_env=measurement_env)})
    payload = {
        "experiment": "exp06_random_robot",
        "run_id": run_id("exp06"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": {
            "path": str(cache_path),
            "bytes": directory_size(cache_path),
            "file_sizes": cache_file_sizes(cache_path),
            "manifest": read_manifest(cache_path / "manifest.json"),
            "snapshot": prewarm_summary.get("snapshot", snapshot_summary(snapshot_path_for_cache(cache_path))),
            "prewarm_json_path": str(args.prewarm_json),
            "prewarm_summary": prewarm_summary,
        },
        "commands": rows,
        "runs": run_records,
        "notes": [
            "The IIWA SBF row reuses a shared Shelf+IIWA p18 endpoint-only cache under the experiment output root.",
            "LECT DB stores endpoint-envelope payloads only; link envelopes are reconstructed online from endpoint envelopes.",
            "Warm-cache snapshots are published by the writer after checkpoint; reused caches backfill missing snapshots before SBF execution.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
