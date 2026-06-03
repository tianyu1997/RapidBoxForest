#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
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
from experiments.common.anytime_defaults import UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH  # noqa: E402
from experiments.common.lect_db_dispatch import (  # noqa: E402
    build_current_shelf_sbf_anytime_command,
    build_legacy_shelf_anytime_command,
    build_shelf_iris_anytime_command,
    ensure_shelf_cache,
    shelf_cache_payload,
)
from experiments.common.shelf_iiwa_cache import DEFAULT_P18_CACHE_LABEL, load_json  # noqa: E402


SBF_METHOD_NAME = "sbf_leaf_refine_d23_box200"
DEFAULT_GCS_REPO = REPO_ROOT / "gcs-science-robotics"
LOCAL_GCS_REPO = Path("/home/tian/桌面/box_aabb/gcs-science-robotics")
LOCAL_IRIS_PYTHON = Path("/home/tian/miniconda3/envs/sbf/bin/python")


def default_iris_python() -> str:
    return str(LOCAL_IRIS_PYTHON if LOCAL_IRIS_PYTHON.exists() else Path(sys.executable))


def default_gcs_repo() -> Path:
    return LOCAL_GCS_REPO if LOCAL_GCS_REPO.is_dir() else DEFAULT_GCS_REPO


def iris_dependency_status(python_executable: str, gcs_repo: Path) -> dict[str, Any]:
    missing: list[str] = []
    if not Path(python_executable).exists():
        missing.append(str(python_executable))
    if not Path(gcs_repo).is_dir():
        missing.append(str(gcs_repo))
    env = os.environ.copy()
    pythonpath_entries = [
        str(REPO_ROOT / "build-exp04" / "python"),
        str(REPO_ROOT.parent),
        str(gcs_repo),
    ]
    existing = env.get("PYTHONPATH", "")
    if existing:
        pythonpath_entries.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(pythonpath_entries)
    probe = None
    if not missing:
        probe = subprocess.run(
            [
                str(python_executable),
                "-c",
                "import numpy, pydrake, sbf; from gcs.bezier import BezierGCS",
            ],
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
        "gcs_repo": str(gcs_repo),
        "stdout_tail": (probe.stdout[-2000:] if probe is not None else ""),
        "stderr_tail": (probe.stderr[-2000:] if probe is not None else ""),
    }


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp05_shelf_cross_algorithm"
    parser = argparse.ArgumentParser(description="Run Experiment 5 as a shelf cross-algorithm anytime trade-off dispatcher.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--prewarm-json", type=Path, default=output_dir / "p18_prewarm.json")
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--prewarm-max-depth", type=int, default=UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--prewarm-envelope", choices=["link", "support_hull"], default="support_hull")
    parser.add_argument("--methods", default="sbf,iris_np,prm,rrtconnect,bitstar")
    parser.add_argument("--sbf-backend", choices=["leaf_refine", "current_anytime"], default="leaf_refine")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=10000.0)
    parser.add_argument("--rrt-trials", type=int, default=10)
    parser.add_argument("--prm-build-grid-s", default="0.25,0.5,1,2,5")
    parser.add_argument("--prm-query-budget-s", type=float, default=1.0)
    parser.add_argument("--bitstar-timeout-s", type=float, default=5.0)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=1.0)
    parser.add_argument("--rrt-timeout-ms", type=float, default=10000.0)
    parser.add_argument("--iris-python", default=default_iris_python())
    parser.add_argument("--iris-gcs-repo", type=Path, default=default_gcs_repo())
    parser.add_argument("--iris-budget-s", type=float, default=600.0)
    parser.add_argument("--iris-stage-region-counts", default="2,4,6,8")
    parser.add_argument("--iris-iteration-limit", type=int, default=3)
    parser.add_argument("--iris-query-time-limit-s", type=float, default=30.0)
    parser.add_argument("--iris-rounding-max-paths", type=int, default=10)
    parser.add_argument("--iris-rounding-max-trials", type=int, default=100)
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--warm-cache-label", default=DEFAULT_P18_CACHE_LABEL)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--external-evidence-mode", choices=["legacy", "snapshot"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def seeds_list(count: int) -> str:
    return ",".join(str(index) for index in range(max(1, int(count))))


def sbf_command(args: argparse.Namespace) -> list[str]:
    if str(args.sbf_backend) == "leaf_refine":
        return [
            sys.executable,
            str(REPO_ROOT / "experiments" / "exp04_shelf_ablation" / "run_leaf_refine_tradeoff.py"),
            "--out-dir",
            str(args.out_dir / SBF_METHOD_NAME),
            "--seeds-list",
            seeds_list(int(args.seeds)),
            "--leaf-depths",
            "8:14",
            "--deep-max-boxes-list",
            "200",
            "--deep-ffb-depth-list",
            "28",
            "--refine-timeout-ms-list",
            "600",
            "--leaf-threads",
            str(int(args.threads)),
            "--threads",
            str(int(args.threads)),
        ]
    return build_current_shelf_sbf_anytime_command(
        python_executable=sys.executable,
        out_json=args.out_dir / f"{SBF_METHOD_NAME}.json",
        database_path=args.rbf_cache_root / SBF_METHOD_NAME,
        case_name=SBF_METHOD_NAME,
        threads=int(args.threads),
        seeds=int(args.seeds),
        timeout_ms=float(args.timeout_ms),
        rbf_cache_root=args.rbf_cache_root,
        warm_cache_label=str(args.warm_cache_label),
        external_evidence_mode=str(args.external_evidence_mode),
        external_evidence_auto_build_snapshot=bool(args.external_evidence_auto_build_snapshot),
        clean_active_cache=True,
    )


def legacy_ompl_methods_arg(methods: set[str]) -> str:
    requested = []
    if "prm" in methods:
        requested.append("prm")
    if "bitstar" in methods:
        requested.append("bitstar")
    if "rrtconnect" in methods:
        requested.append("rrt")
    return ",".join(requested)


def command_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    methods = set(csv_list(args.methods))
    rows: list[dict[str, Any]] = []
    if "sbf" in methods:
        rows.append({
            "name": SBF_METHOD_NAME,
            "methods": ["sbf"],
            "kind": "sbf_leaf_refine" if str(args.sbf_backend) == "leaf_refine" else "sbf_anytime_current",
            "command": sbf_command(args),
        })
    for method, legacy_name in (("prm", "prm"), ("bitstar", "bitstar"), ("rrtconnect", "rrt")):
        if method not in methods:
            continue
        rows.append({
            "name": f"legacy_{method}_anytime",
            "methods": [method],
            "kind": "legacy_shelf_anytime",
            "command": build_legacy_shelf_anytime_command(
                python_executable=sys.executable,
                out_json=args.out_dir / f"legacy_{method}_anytime.json",
                methods=legacy_name,
                threads=int(args.threads),
                seeds=int(args.seeds),
                prm_build_grid_s=str(args.prm_build_grid_s),
                prm_query_budget_s=float(args.prm_query_budget_s),
                bitstar_timeout_s=float(args.bitstar_timeout_s),
                bitstar_checkpoint_interval_s=float(args.bitstar_checkpoint_interval_s),
                rrt_timeout_ms=float(args.rrt_timeout_ms),
            ),
        })
    if "iris_np" in methods:
        deps = iris_dependency_status(str(args.iris_python), Path(args.iris_gcs_repo))
        row = {
            "name": "iris_np_gcs_anytime",
            "methods": ["iris_np"],
            "kind": "shelf_iris_anytime" if deps["ok"] else "skipped_dependency",
            "dependency_status": deps,
        }
        if deps["ok"]:
            row["command"] = build_shelf_iris_anytime_command(
                python_executable=str(args.iris_python),
                out_json=args.out_dir / "iris_np_gcs_anytime.json",
                seeds=int(args.seeds),
                threads=int(args.threads),
                gcs_repo=Path(args.iris_gcs_repo),
                budget_s=float(args.iris_budget_s),
                stage_region_counts=str(args.iris_stage_region_counts),
                iteration_limit=int(args.iris_iteration_limit),
                query_time_limit_s=float(args.iris_query_time_limit_s),
                rounding_max_paths=int(args.iris_rounding_max_paths),
                rounding_max_trials=int(args.iris_rounding_max_trials),
            )
        rows.append(row)
    return rows


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "shelf_cross_algorithm_manifest.json")
    rows = command_rows(args)
    cache_path = args.rbf_cache_root / str(args.warm_cache_label)
    prune_summary: dict[str, Any] = {
        "path": str(args.rbf_cache_root),
        "kept": [str(args.warm_cache_label)],
        "removed": [],
        "removed_count": 0,
        "dry_run": bool(args.dry_run or not args.execute),
    }
    if args.execute and not args.dry_run:
        prune_summary = prune_directory_children(args.rbf_cache_root, [str(args.warm_cache_label)])
    needs_prewarm = any(row.get("kind") == "sbf_anytime_current" for row in rows)
    if args.execute and needs_prewarm:
        prewarm_summary = ensure_shelf_cache(
            prewarm_json=args.prewarm_json,
            cache_path=cache_path,
            prewarm_depth=int(args.prewarm_depth),
            envelope=str(args.prewarm_envelope),
            prewarm_threads=int(args.prewarm_threads),
            max_depth=int(args.prewarm_max_depth),
            clean_cache=bool(args.clean_cache),
            dry_run=bool(args.dry_run),
        )
    else:
        prewarm_summary = load_json(args.prewarm_json)

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
            measurement_env = extra_env
            run_records.append({
                "name": row["name"],
                "measurement": run_command(row["command"], dry_run=bool(args.dry_run), extra_env=measurement_env),
            })

    payload = {
        "experiment": "exp05_shelf_cross_algorithm",
        "plan_file": "experiments/05_shelf_cross_algorithm_plan.md",
        "run_id": run_id("exp05"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": {
            **shelf_cache_payload(cache_path, args.prewarm_json, prewarm_summary),
            "prune": prune_summary,
        },
        "commands": rows,
        "runs": run_records,
        "notes": [
            "Experiment 5 is now a pure anytime dispatcher: current shelf SBF anytime, legacy OMPL shelf anytime, and IRIS-NP+GCS prefix anytime are emitted as separate artifacts.",
            "By default, the SBF row uses the optimized Exp.4 leaf-refine d23 baseline. The old unified d40 shelf anytime backend remains available with --sbf-backend current_anytime.",
            "PRM, BIT*, and RRTConnect remain on the legacy shelf anytime backend; IRIS-NP+GCS remains on its dedicated prefix anytime backend.",
            "Shelf cache preparation keeps only the canonical native p18 cache under the cache root, and the shelf SBF backend runs without checkpointing online cache writes back into LECT.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
