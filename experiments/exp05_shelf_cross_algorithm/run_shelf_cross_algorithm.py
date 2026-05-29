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
from experiments.common.anytime_defaults import UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH  # noqa: E402
from experiments.common.lect_db_dispatch import (  # noqa: E402
    build_current_shelf_sbf_anytime_command,
    build_legacy_shelf_anytime_command,
    build_shelf_iris_anytime_command,
    ensure_shelf_cache,
    shelf_cache_payload,
)
from experiments.common.shelf_iiwa_cache import DEFAULT_P18_CACHE_LABEL, load_json  # noqa: E402


SBF_METHOD_NAME = "sbf_current_anytime_d40_r4"


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
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=10000.0)
    parser.add_argument("--rrt-trials", type=int, default=10)
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--warm-cache-label", default=DEFAULT_P18_CACHE_LABEL)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--external-evidence-mode", choices=["legacy", "snapshot"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def sbf_command(args: argparse.Namespace) -> list[str]:
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
            "kind": "sbf_anytime_current",
            "command": sbf_command(args),
        })
    baseline_methods = legacy_ompl_methods_arg(methods)
    if baseline_methods:
        rows.append({
            "name": "legacy_ompl_anytime_tradeoff",
            "methods": [method for method in ["prm", "bitstar", "rrtconnect"] if method in methods],
            "kind": "legacy_shelf_anytime",
            "command": build_legacy_shelf_anytime_command(
                python_executable=sys.executable,
                out_json=args.out_dir / "legacy_ompl_anytime_tradeoff.json",
                methods=baseline_methods,
                threads=int(args.threads),
                seeds=int(args.seeds),
            ),
        })
    if "iris_np" in methods:
        rows.append({
            "name": "iris_np_gcs_anytime",
            "methods": ["iris_np"],
            "kind": "shelf_iris_anytime",
            "command": build_shelf_iris_anytime_command(
                python_executable=sys.executable,
                out_json=args.out_dir / "iris_np_gcs_anytime.json",
                seeds=int(args.seeds),
                threads=int(args.threads),
            ),
        })
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
            "The SBF row uses the refined unified d40 schedule through experiments/common/run_shelf_sbf_anytime.py and the Exp.3 p18 warm cache.",
            "PRM, BIT*, and RRTConnect remain on the legacy shelf anytime backend; IRIS-NP+GCS remains on its dedicated prefix anytime backend.",
            "Shelf cache preparation keeps only the canonical native p18 cache under the cache root, and the shelf SBF backend runs without checkpointing online cache writes back into LECT.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
