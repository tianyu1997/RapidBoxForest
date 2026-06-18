#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    csv_list,
    environment_metadata,
    run_id,
    write_json,
)
from experiments.common.progress import progress  # noqa: E402


EXPERIMENTS = {
    "exp01": ("Endpoint envelope", REPO_ROOT / "experiments" / "exp01_endpoint_envelope" / "run_endpoint_envelope.py"),
    "exp02": ("Link envelope", REPO_ROOT / "experiments" / "exp02_link_envelope" / "run_link_envelope.py"),
    "exp03": ("LECT performance", REPO_ROOT / "experiments" / "exp03_lect_performance" / "run_lect_performance.py"),
    "exp04": ("Shelf leaf RRT", REPO_ROOT / "experiments" / "exp04_shelf_leaf_rrt" / "run_shelf_leaf_rrt.py"),
    "exp05": ("Shelf cross algorithm", REPO_ROOT / "experiments" / "exp05_shelf_cross_algorithm" / "run_shelf_cross_algorithm.py"),
    "exp06": ("Random robot", REPO_ROOT / "experiments" / "exp06_random_robot" / "run_random_robot.py"),
    "exp07": ("Dynamic update", REPO_ROOT / "experiments" / "exp07_dynamic_update" / "run_dynamic_update.py"),
    "appendix": ("Appendix sweeps", REPO_ROOT / "experiments" / "appendix_sweeps" / "run_appendix_sweeps.py"),
}

PUBLIC_SMOKE_EXPERIMENTS = ("exp01", "exp02")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Unified TRO2026 experiment dispatcher.")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--only", default="all", help="Comma-separated experiment ids or all.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    return parser.parse_args()


def selected_experiments(args: argparse.Namespace) -> list[str]:
    wanted = set(csv_list(args.only))
    if not wanted or "all" in wanted:
        if str(args.phase) == "smoke":
            return [item for item in PUBLIC_SMOKE_EXPERIMENTS if item in EXPERIMENTS]
        return list(EXPERIMENTS)
    unknown = sorted(item for item in wanted if item not in EXPERIMENTS)
    if unknown:
        raise ValueError(f"unknown experiment ids: {unknown}")
    return [item for item in EXPERIMENTS if item in wanted]


def command_for(args: argparse.Namespace, exp_id: str) -> list[str]:
    script = EXPERIMENTS[exp_id][1]
    out_dir = args.out_dir / exp_id
    command = [sys.executable, str(script), "--out-dir", str(out_dir), "--phase", str(args.phase)]
    if args.dry_run:
        command.append("--dry-run")
    if args.scene_catalog is not None and exp_id in {"exp06", "exp07"}:
        command.extend(["--scene-catalog", str(args.scene_catalog)])
    if exp_id in {"exp06", "exp07"}:
        command.extend(["--scene-catalog-mode", str(args.scene_catalog_mode)])
    return command


def run_command(command: list[str]) -> dict[str, Any]:
    result = subprocess.run(command, cwd=str(REPO_ROOT), text=True, check=False)
    return {
        "command": command,
        "returncode": result.returncode,
        "stdout_tail": "streamed live by dispatcher",
        "stderr_tail": "streamed live by dispatcher",
    }


def main() -> int:
    args = parse_args()
    exp_ids = selected_experiments(args)
    commands = [
        {
            "id": exp_id,
            "name": EXPERIMENTS[exp_id][0],
            "command": command_for(args, exp_id),
        }
        for exp_id in exp_ids
    ]
    runs: list[dict[str, Any]] = []
    if args.execute:
        for item in progress(commands, desc="tro2026 experiments", total=len(commands)):
            runs.append({"id": item["id"], "measurement": run_command(list(item["command"]))})
    payload = {
        "experiment": "tro2026_dispatch",
        "run_id": run_id("tro2026"),
        "phase": str(args.phase),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "selection_policy": (
            "public_smoke_fast_core" if str(args.phase) == "smoke" and "all" in set(csv_list(args.only))
            else "explicit_or_full"
        ),
        "public_smoke_experiments": list(PUBLIC_SMOKE_EXPERIMENTS),
        "environment": environment_metadata(),
        "commands": commands,
        "runs": runs,
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.out_dir / f"tro2026_{args.phase}_manifest.json", payload)
    print(f"wrote {args.out_dir / f'tro2026_{args.phase}_manifest.json'}")
    return 0 if all(run.get("measurement", {}).get("returncode", 0) == 0 for run in runs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
