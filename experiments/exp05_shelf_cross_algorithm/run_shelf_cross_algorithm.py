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


SBF_COMBINED = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_marcucci_combined.py"
BASELINES = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_baselines_marcucci.py"
RRTCONNECT = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_rrt_connect_baseline.py"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp05_shelf_cross_algorithm"
    parser = argparse.ArgumentParser(description="Run Experiment 5 shelf cross-algorithm dispatcher.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--methods", default="sbf,iris_np,prm,rrtconnect,bitstar")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--rrt-trials", type=int, default=10)
    parser.add_argument("--timeout-ms", type=float, default=10000.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def commands(args: argparse.Namespace) -> list[dict[str, Any]]:
    methods = set(csv_list(args.methods))
    rows: list[dict[str, Any]] = []
    if "sbf" in methods:
        rows.append({
            "name": "sbf_support_hull",
            "methods": ["sbf"],
            "command": [
                sys.executable,
                str(SBF_COMBINED),
                "--out-json",
                str(args.out_dir / "sbf_support_hull.json"),
                "--preset",
                "ifk_strict",
                "--envelope",
                "support_hull",
                "--threads",
                "8",
                "--seeds",
                str(args.seeds),
                "--strict-path-audit",
            ],
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
    if args.execute:
        for row in rows:
            run_records.append({"name": row["name"], "measurement": run_command(row["command"], dry_run=bool(args.dry_run))})
    payload = {
        "experiment": "exp05_shelf_cross_algorithm",
        "run_id": run_id("exp05"),
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
