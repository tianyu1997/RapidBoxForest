#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT_DIR = ROOT / "outputs" / "paper"
IRIS_SCRIPT = ROOT / "experiments" / "paper_04_iris_np_gcs_baseline.py"
OMPL_SCRIPT = ROOT / "experiments" / "paper_04_ompl_baselines.py"

METHOD_OUTPUTS = {
    "iris_np": ["marcucci_iris_np_gcs.json"],
    "ompl": ["marcucci_ompl_prm.json", "marcucci_ompl_bitstar_budget.json"],
}


def parse_methods(raw: str) -> list[str]:
    methods = [item.strip() for item in raw.split(",") if item.strip()]
    unknown = [item for item in methods if item not in METHOD_OUTPUTS]
    if unknown:
        raise ValueError(f"unknown baseline methods: {unknown}; choices={sorted(METHOD_OUTPUTS)}")
    return methods


def run_child(cmd: list[str], cwd: Path) -> float:
    t0 = time.perf_counter()
    subprocess.run(cmd, check=True, cwd=cwd)
    return time.perf_counter() - t0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Current SBF Exp.4 Marcucci baseline dispatcher.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true")
    mode.add_argument("--full", action="store_true")
    parser.add_argument("--source", choices=["live"], default="live", help="Only live current-version runs are allowed; historical artifacts are intentionally disabled.")
    parser.add_argument("--methods", default="iris_np,ompl")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--logical-threads", type=int, default=8)
    parser.add_argument("--bitstar-budget-s", type=float, default=10.0)
    parser.add_argument("--bitstar-restarts", type=int, default=5)
    parser.add_argument("--bitstar-simplify-time-s", type=float, default=0.2)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=-1)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=-1.0)
    parser.add_argument("--bitstar-stop-on-solution-improvement", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prm-build-budget-s", type=float, default=40.0)
    parser.add_argument("--prm-query-budget-s", type=float, default=2.0)
    parser.add_argument("--prm-max-nearest-neighbors", type=int, default=128)
    parser.add_argument("--prm-simplify-time-s", type=float, default=0.10)
    parser.add_argument("--prm-roadmap-retries", type=int, default=3)
    parser.add_argument("--iris-budget-s", type=float, default=800.0)
    parser.add_argument("--iris-query-time-limit-s", type=float, default=120.0)
    parser.add_argument("--iris-iteration-limit", type=int, default=10)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def mode_flag(args: argparse.Namespace) -> str:
    return "--full" if args.full else "--quick"


def existing_outputs(out_dir: Path, methods: list[str]) -> list[dict[str, str]]:
    outputs: list[dict[str, str]] = []
    for method in methods:
        for name in METHOD_OUTPUTS[method]:
            path = out_dir / name
            if path.exists():
                outputs.append({"method": method, "target": str(path)})
    return outputs


def main() -> int:
    args = parse_args()
    methods = parse_methods(args.methods)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    commands: list[list[str]] = []
    common = [mode_flag(args), "--logical-threads", str(args.logical_threads)]
    if args.seeds is not None:
        common += ["--seeds", str(args.seeds)]
    if args.timeout is not None:
        common += ["--timeout", str(args.timeout)]

    if "iris_np" in methods:
        commands.append([
            sys.executable,
            str(IRIS_SCRIPT),
            *common,
            "--out",
            str(args.out_dir / "marcucci_iris_np_gcs.json"),
            "--budget-s",
            str(args.iris_budget_s),
            "--query-time-limit-s",
            str(args.iris_query_time_limit_s),
            "--iteration-limit",
            str(args.iris_iteration_limit),
        ])
    if "ompl" in methods:
        commands.append([
            sys.executable,
            str(OMPL_SCRIPT),
            *common,
            "--out-dir",
            str(args.out_dir),
            "--methods",
            "prm,bitstar_budget",
            "--bitstar-budget-s",
            str(args.bitstar_budget_s),
            "--bitstar-restarts",
            str(args.bitstar_restarts),
            "--bitstar-simplify-time-s",
            str(args.bitstar_simplify_time_s),
            "--bitstar-samples-per-batch",
            str(args.bitstar_samples_per_batch),
            "--bitstar-rewire-factor",
            str(args.bitstar_rewire_factor),
            "--prm-build-budget-s",
            str(args.prm_build_budget_s),
            "--prm-query-budget-s",
            str(args.prm_query_budget_s),
            "--prm-max-nearest-neighbors",
            str(args.prm_max_nearest_neighbors),
            "--prm-simplify-time-s",
            str(args.prm_simplify_time_s),
            "--prm-roadmap-retries",
            str(args.prm_roadmap_retries),
            "--bitstar-stop-on-solution-improvement" if args.bitstar_stop_on_solution_improvement else "--no-bitstar-stop-on-solution-improvement",
        ])

    run_records: list[dict[str, Any]] = []
    for cmd in commands:
        if args.dry_run:
            print("$", " ".join(cmd))
            continue
        wall_s = run_child(cmd, ROOT)
        run_records.append({"command": cmd, "wall_s": wall_s})
    outputs = existing_outputs(args.out_dir, methods)
    print(json.dumps({"source": "live_current_sbf", "methods": methods, "runs": run_records, "outputs": outputs}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())