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
from experiments.common.lect_db_dispatch import (  # noqa: E402
    RANDOM_IRIS,
    RANDOM_OMPL,
    RANDOM_RRTCONNECT,
    build_random_sbf_command,
    ensure_shelf_cache,
    shelf_cache_payload,
)
from experiments.common.shelf_iiwa_cache import DEFAULT_P18_CACHE_LABEL, load_json  # noqa: E402


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp06_random_robot"
    parser = argparse.ArgumentParser(description="Run Experiment 6 from the random robot plan.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--methods", default="sbf,iris_np,prm,rrtconnect,bitstar")
    parser.add_argument("--scene-seeds", type=int, default=5)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--scene-profile", choices=["balanced", "legacy"], default="balanced")
    parser.add_argument("--prewarm-json", type=Path, default=output_dir / "p18_prewarm.json")
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--prewarm-envelope", choices=["link", "support_hull"], default="support_hull")
    parser.add_argument("--rbf-envelope", choices=["link", "support_hull"], default="support_hull")
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--iiwa-warm-cache-label", default=DEFAULT_P18_CACHE_LABEL)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--external-evidence-mode", choices=["legacy", "snapshot"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def scene_args(args: argparse.Namespace) -> list[str]:
    return [
        "--robots",
        str(args.robots),
        "--difficulties",
        str(args.difficulties),
        "--scene-seeds",
        str(max(1, int(args.scene_seeds))),
        "--scene-profile",
        str(args.scene_profile),
    ]


def command_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    methods = set(csv_list(args.methods))
    rows: list[dict[str, Any]] = []
    if "sbf" in methods:
        rows.append({
            "name": "sbf_sh_warm_aafk_random_robot",
            "methods": ["sbf"],
            "kind": "sbf_baseline",
            "description": "SBF-SH baseline: warm + AAFK + SupportHull + 8 threads + AAFKVolumeMin",
            "command": build_random_sbf_command(
                python_executable=sys.executable,
                out_json=args.out_dir / "sbf_sh_warm_aafk_random_robot.json",
                robots=str(args.robots),
                difficulties=str(args.difficulties),
                scene_seeds=int(args.scene_seeds),
                scene_profile=str(args.scene_profile),
                rbf_cache_root=args.rbf_cache_root,
                iiwa_warm_cache_label=str(args.iiwa_warm_cache_label),
                external_evidence_mode=str(args.external_evidence_mode),
                external_evidence_auto_build_snapshot=bool(args.external_evidence_auto_build_snapshot),
                threads=int(args.threads),
                prewarm_depth=int(args.prewarm_depth),
                rbf_envelope=str(args.rbf_envelope),
            ),
        })
    if "rrtconnect" in methods:
        rows.append({
            "name": "rrtconnect_random_robot",
            "methods": ["rrtconnect"],
            "kind": "legacy_rrtconnect",
            "description": "RRTConnect with final audit and 0.01 joint-space segment step",
            "command": [
                sys.executable,
                str(RANDOM_RRTCONNECT),
                *scene_args(args),
                "--trials",
                str(max(1, int(args.trials))),
                "--segment-step",
                "0.01",
                "--out-json",
                str(args.out_dir / "rrtconnect_random_robot.json"),
            ],
        })
    ompl_methods = [method for method in ("prm", "bitstar") if method in methods]
    if ompl_methods:
        rows.append({
            "name": "ompl_prm_bitstar_random_robot",
            "methods": ompl_methods,
            "kind": "legacy_ompl",
            "description": "PRM and BIT* random-scene baselines with 0.01 joint-space segment step",
            "command": [
                sys.executable,
                str(RANDOM_OMPL),
                *scene_args(args),
                "--methods",
                ",".join(ompl_methods),
                "--trials",
                str(max(1, int(args.trials))),
                "--segment-step",
                "0.01",
                "--out-json",
                str(args.out_dir / "ompl_prm_bitstar_random_robot.json"),
            ],
        })
    if "iris_np" in methods:
        rows.append({
            "name": "iris_np_gcs_random_robot",
            "methods": ["iris_np"],
            "kind": "legacy_iris_np_gcs",
            "description": "IRIS-NP+GCS random-scene baseline with final audit",
            "command": [
                sys.executable,
                str(RANDOM_IRIS),
                *scene_args(args),
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
    rows = command_rows(args)
    cache_path = args.rbf_cache_root / str(args.iiwa_warm_cache_label)
    needs_prewarm = any(row.get("kind") == "sbf_baseline" for row in rows)
    if args.execute and needs_prewarm:
        prewarm_summary = ensure_shelf_cache(
            prewarm_json=args.prewarm_json,
            cache_path=cache_path,
            prewarm_depth=int(args.prewarm_depth),
            envelope=str(args.prewarm_envelope),
            prewarm_threads=int(args.prewarm_threads),
            clean_cache=bool(args.clean_cache),
            dry_run=bool(args.dry_run),
        )
    else:
        prewarm_summary = load_json(args.prewarm_json)

    run_records = []
    extra_env = default_sbf_subprocess_env()
    if args.execute:
        for row in rows:
            measurement_env = extra_env if row.get("kind") == "sbf_baseline" else None
            run_records.append({
                "name": row["name"],
                "measurement": run_command(row["command"], dry_run=bool(args.dry_run), extra_env=measurement_env),
            })

    payload = {
        "experiment": "exp06_random_robot",
        "plan_file": "experiments/06_random_robot_plan.md",
        "run_id": run_id("exp06"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": shelf_cache_payload(cache_path, args.prewarm_json, prewarm_summary),
        "commands": rows,
        "runs": run_records,
        "notes": [
            "This is the fresh /experiments Exp.6 dispatcher; it keeps the random-scene backend as a reusable execution engine.",
            "The IIWA SBF row reuses the Exp.3-style p18 LECT DB cache; UR5 and Panda warm caches are built by the random backend because no shelf Exp.3 cache exists for those robots.",
            "The SBF command shape is warm_d18 + AAFK + SupportHull + 8 threads + AAFKVolumeMin through the random SBF backend defaults.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
