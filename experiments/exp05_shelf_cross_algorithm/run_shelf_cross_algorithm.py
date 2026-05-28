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


SBF_COMBINED = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_marcucci_combined.py"
SBF_SHELF_MAIN = REPO_ROOT / "safe_box_forest" / "experiments" / "rbf_only_shelf_iiwa_main.py"
BASELINES = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_baselines_marcucci.py"
RRTCONNECT = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_rrt_connect_baseline.py"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp05_shelf_cross_algorithm"
    parser = argparse.ArgumentParser(description="Run Experiment 5 shelf cross-algorithm dispatcher.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--prewarm-json", type=Path, default=output_dir / "p18_prewarm.json")
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--prewarm-envelope", choices=["link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--methods", default="sbf,iris_np,prm,rrtconnect,bitstar")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--rrt-trials", type=int, default=10)
    parser.add_argument("--timeout-ms", type=float, default=10000.0)
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--warm-cache-label", default=DEFAULT_P18_CACHE_LABEL)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--external-evidence-mode", choices=["legacy", "snapshot"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def seeds_list_arg(seed_count: int) -> str:
    return ",".join(str(index) for index in range(max(1, int(seed_count))))


def shelf_baseline_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        str(SBF_SHELF_MAIN),
        "--out-json",
        str(args.out_dir / "sbf_support_hull.json"),
        "--modes",
        "warm_d18",
        "--seeds-list",
        seeds_list_arg(args.seeds),
        "--threads",
        "8",
        "--timeout-ms",
        str(args.timeout_ms),
        "--rbf-prewarm-depth",
        "18",
        "--rbf-envelope",
        "support_hull",
        "--rbf-cache-root",
        str(args.rbf_cache_root),
        "--warm-cache-label",
        str(args.warm_cache_label),
        "--external-evidence-mode",
        str(args.external_evidence_mode),
        "--no-support-hull-keep-kdop",
    ]
    if args.external_evidence_auto_build_snapshot:
        command.append("--external-evidence-auto-build-snapshot")
    else:
        command.append("--no-external-evidence-auto-build-snapshot")
    return command


def commands(args: argparse.Namespace) -> list[dict[str, Any]]:
    methods = set(csv_list(args.methods))
    rows: list[dict[str, Any]] = []
    if "sbf" in methods:
        rows.append({
            "name": "sbf_support_hull",
            "methods": ["sbf"],
            "command": shelf_baseline_command(args),
        })
    baseline_methods = []
    if "iris_np" in methods:
        baseline_methods.append("iris_np")
    if "prm" in methods or "bitstar" in methods:
        baseline_methods.append("ompl")
    if baseline_methods:
        rows.append({
            "name": "iris_prm_bitstar_baselines",
            "methods": baseline_methods,
            "command": [
                sys.executable,
                str(BASELINES),
                "--quick",
                "--methods",
                ",".join(baseline_methods),
                "--out-dir",
                str(args.out_dir / "baselines"),
                "--seeds",
                str(args.seeds),
                "--dry-run" if args.dry_run else "--source",
                "live" if not args.dry_run else "",
            ],
        })
        rows[-1]["command"] = [part for part in rows[-1]["command"] if part != ""]
    if "rrtconnect" in methods:
        rows.append({
            "name": "rrtconnect",
            "methods": ["rrtconnect"],
            "command": [
                sys.executable,
                str(RRTCONNECT),
                "--trials",
                str(args.rrt_trials),
                "--timeout-ms",
                str(args.timeout_ms),
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
    rows = commands(args)
    run_records = []
    cache_path = args.rbf_cache_root / str(args.warm_cache_label)
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
        "experiment": "exp05_shelf_cross_algorithm",
        "run_id": run_id("exp05"),
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
            "The SBF row reuses a shared Shelf+IIWA p18 endpoint-only cache under the experiment output root.",
            "LECT DB stores endpoint-envelope payloads only; link envelopes are reconstructed online from endpoint envelopes.",
            "Warm-cache snapshots are published by the writer after checkpoint; reused caches backfill missing snapshots before SBF execution.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
