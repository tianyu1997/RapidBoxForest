#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import csv_list  # noqa: E402
from experiments.common.progress import progress  # noqa: E402
from experiments.common.rbf_defaults import (  # noqa: E402
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_LEAF_MAX_DEPTH,
    DEFAULT_RBF_LEAF_START_DEPTH,
    DEFAULT_RBF_MAX_DEPTH,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan Exp.6 RBF leaf/FFB depth settings.")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "outputs" / "new_experiments" / "exp06_rbf_depth_scan")
    parser.add_argument("--scene-catalog", type=Path, default=REPO_ROOT / "outputs" / "new_experiments" / "tro2026" / "exp06" / "random_scene_catalog.json")
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=3)
    parser.add_argument("--box-budget", type=int, default=400)
    parser.add_argument("--leaf-start-depth", type=int, default=DEFAULT_RBF_LEAF_START_DEPTH)
    parser.add_argument("--leaf-max-depths", default=f"10,12,{DEFAULT_RBF_LEAF_MAX_DEPTH},16")
    parser.add_argument("--ffb-depths", default=f"48,56,{DEFAULT_RBF_DEEP_FFB_DEPTH}")
    parser.add_argument("--rbf-max-depth", type=int, default=DEFAULT_RBF_MAX_DEPTH)
    parser.add_argument("--query-bridge-adaptive-max-path-length", type=float, default=4.5)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def config_label(leaf_max: int, ffb_depth: int, budget: int, seeds: int) -> str:
    return f"leaf{leaf_max}_ffb{ffb_depth}_b{budget}_s{seeds}"


def main() -> int:
    args = parse_args()
    leaf_depths = [int(item) for item in csv_list(args.leaf_max_depths)]
    ffb_depths = [int(item) for item in csv_list(args.ffb_depths)]
    configs = [(leaf, ffb) for leaf in leaf_depths for ffb in ffb_depths]
    all_rows: list[dict[str, Any]] = []
    runner = REPO_ROOT / "experiments" / "exp06_random_robot" / "run_random_robot.py"
    for leaf_max, ffb_depth in progress(configs, desc="exp06 depth scan", total=len(configs), disable=bool(args.dry_run)):
        label = config_label(leaf_max, ffb_depth, int(args.box_budget), int(args.scene_seeds))
        out_dir = Path(args.out_dir) / label
        cmd = [
            sys.executable,
            str(runner),
            "--phase", "pilot",
            "--methods", "sbf_leaf_rrt",
            "--robots", str(args.robots),
            "--difficulties", str(args.difficulties),
            "--scene-seeds", str(int(args.scene_seeds)),
            "--box-budgets", str(int(args.box_budget)),
            "--scene-catalog-mode", "reuse",
            "--scene-catalog", str(args.scene_catalog),
            "--skip-lect-cache-ensure",
            "--threads", str(int(args.threads)),
            "--rbf-max-depth", str(max(int(args.rbf_max_depth), int(ffb_depth))),
            "--leaf-start-depth", str(int(args.leaf_start_depth)),
            "--leaf-max-depth", str(int(leaf_max)),
            "--deep-ffb-depth", str(int(ffb_depth)),
            "--connector-pave-depth", str(int(ffb_depth)),
            "--query-bridge-pave-depth", str(int(ffb_depth)),
            "--query-bridge-all",
            "--query-bridge-adaptive-all",
            "--query-bridge-adaptive-max-path-length", str(float(args.query_bridge_adaptive_max_path_length)),
            "--out-dir", str(out_dir),
        ]
        print(f"[scan] {label}", flush=True)
        if not args.dry_run:
            subprocess.run(cmd, cwd=str(REPO_ROOT), check=True)
            summary_path = out_dir / "random_robot_summary.csv"
            for row in read_rows(summary_path):
                row.update({
                    "scan_label": label,
                    "leaf_start_depth": int(args.leaf_start_depth),
                    "leaf_max_depth": int(leaf_max),
                    "ffb_depth": int(ffb_depth),
                    "box_budget": int(args.box_budget),
                    "scene_seeds_requested": int(args.scene_seeds),
                })
                all_rows.append(row)
    if not args.dry_run:
        write_rows(Path(args.out_dir) / "exp06_rbf_depth_scan_summary.csv", all_rows)
        print(f"wrote {Path(args.out_dir) / 'exp06_rbf_depth_scan_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
