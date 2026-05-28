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
    SHELF_BASELINES,
    SHELF_RRTCONNECT,
    build_shelf_sbf_case_command,
    ensure_shelf_cache,
    shelf_cache_payload,
)
from experiments.common.shelf_iiwa_cache import DEFAULT_P18_CACHE_LABEL, load_json  # noqa: E402


SBF_METHOD_NAME = "sbf_warm_aafk_support_hull"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp05_shelf_cross_algorithm"
    parser = argparse.ArgumentParser(description="Run Experiment 5 from the shelf cross-algorithm plan.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--prewarm-json", type=Path, default=output_dir / "p18_prewarm.json")
    parser.add_argument("--prewarm-depth", type=int, default=18)
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
    return build_shelf_sbf_case_command(
        python_executable=sys.executable,
        out_json=args.out_dir / f"{SBF_METHOD_NAME}.json",
        database_path=args.rbf_cache_root / SBF_METHOD_NAME,
        case_name=SBF_METHOD_NAME,
        endpoint_source="aafk",
        lect_split_policy="aafk_volume_min",
        envelope="support_hull",
        threads=int(args.threads),
        seeds=int(args.seeds),
        timeout_ms=float(args.timeout_ms),
        use_external_evidence=True,
        rbf_cache_root=args.rbf_cache_root,
        warm_cache_label=str(args.warm_cache_label),
        external_evidence_mode=str(args.external_evidence_mode),
        external_evidence_auto_build_snapshot=bool(args.external_evidence_auto_build_snapshot),
        external_evidence_materialization=True,
        external_evidence_scoring=True,
        keep_kdop=True,
        clean_active_cache=True,
    )


def baseline_methods_arg(methods: set[str]) -> str:
    requested = []
    if "iris_np" in methods:
        requested.append("iris_np")
    if "prm" in methods or "bitstar" in methods:
        requested.append("ompl")
    return ",".join(requested)


def command_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    methods = set(csv_list(args.methods))
    rows: list[dict[str, Any]] = []
    if "sbf" in methods:
        rows.append({
            "name": SBF_METHOD_NAME,
            "methods": ["sbf"],
            "kind": "sbf_baseline",
            "command": sbf_command(args),
        })
    baseline_methods = baseline_methods_arg(methods)
    if baseline_methods:
        command = [
            sys.executable,
            str(SHELF_BASELINES),
            "--quick",
            "--source",
            "live",
            "--methods",
            baseline_methods,
            "--out-dir",
            str(args.out_dir / "baselines"),
            "--seeds",
            str(max(1, int(args.seeds))),
            "--timeout",
            str(int(float(args.timeout_ms) / 1000.0)),
            "--logical-threads",
            str(max(1, int(args.threads))),
        ]
        if args.dry_run:
            command.append("--dry-run")
        rows.append({
            "name": "iris_prm_bitstar_baselines",
            "methods": [method for method in ["iris_np", "prm", "bitstar"] if method in methods],
            "kind": "legacy_shelf_baselines",
            "command": command,
        })
    if "rrtconnect" in methods:
        rows.append({
            "name": "rrtconnect",
            "methods": ["rrtconnect"],
            "kind": "legacy_rrtconnect",
            "command": [
                sys.executable,
                str(SHELF_RRTCONNECT),
                "--trials",
                str(max(1, int(args.rrt_trials))),
                "--timeout-ms",
                str(float(args.timeout_ms)),
                "--segment-step",
                "0.01",
                "--out-json",
                str(args.out_dir / "rrtconnect.json"),
            ],
        })
    return rows


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "shelf_cross_algorithm_manifest.json")
    rows = command_rows(args)
    cache_path = args.rbf_cache_root / str(args.warm_cache_label)
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
        "experiment": "exp05_shelf_cross_algorithm",
        "plan_file": "experiments/05_shelf_cross_algorithm_plan.md",
        "run_id": run_id("exp05"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": shelf_cache_payload(cache_path, args.prewarm_json, prewarm_summary),
        "commands": rows,
        "runs": run_records,
        "notes": [
            "The SBF row is exactly the Exp.4 baseline command shape and shares the Exp.3 p18 LECT DB cache path.",
            "IRIS-NP+GCS, PRM, BIT*, and RRTConnect are dispatched through the existing shelf baseline backends.",
            "OMPL segment-step is fixed to 0.01 for RRTConnect; PRM/BIT* use the legacy OMPL dispatcher defaults recorded in their outputs.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
