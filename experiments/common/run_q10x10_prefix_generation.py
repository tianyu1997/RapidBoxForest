#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import csv_list
from experiments.common.progress import progress


DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "new_experiments" / "tro2026" / "exp06"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the Exp.6 strict q10x10 prefix catalog.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--scene-seeds", type=int, default=10)
    parser.add_argument("--queries-per-scene", type=int, default=10)
    parser.add_argument("--max-attempts", type=int, default=8)
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--timeout-s", type=float, default=1800.0)
    parser.add_argument("--candidate-only", action="store_true")
    parser.add_argument("--confirm-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def robot_candidate_args(robot: str) -> list[str]:
    key = str(robot).lower()
    common = [
        "--query-sampling-mode", "local",
        "--query-min-l2", "0.8",
        "--endpoint-source", "critsample",
        "--prefix-selection-mode", "candidate",
        "--planner-seeds", "1",
        "--rrt-probe-timeout-s", "0.5",
        "--bitstar-probe-timeout-s", "0.5",
        "--bitstar-probe-checkpoint-interval-s", "0.005",
        "--bitstar-probe-mode", "trace",
        "--bitstar-samples-per-batch", "100",
        "--strict-audit-step", "0.01",
        "--min-probe-success-fraction", "0.5",
    ]
    if key == "ur5":
        return common + [
            "--query-local-radius", "0.22",
            "--obstacle-order", "mixed",
            "--max-workspace-obstacles", "100",
            "--n-samples-crit", "220",
            "--allowed-link-idxs", "2,3,4",
            "--workspace-aabb-shrink", "0.5",
            "--prefix-fine-until", "8",
            "--prefix-mid-step", "2",
            "--prefix-coarse-step", "20",
        ]
    if key == "iiwa":
        return common + [
            "--rrt-probe-timeout-s", "0.2",
            "--bitstar-probe-timeout-s", "0.2",
            "--query-sampling-mode", "hard_probe",
            "--query-max-tries", "4096",
            "--hard-query-prefix-count", "120",
            "--hard-query-bitstar-min-s", "0.02",
            "--hard-query-bitstar-max-s", "0.50",
            "--hard-query-rrt-min-s", "0.003",
            "--hard-query-rrt-max-s", "0.20",
            "--obstacle-order", "path_blocking",
            "--path-blocking-max-prefix", "40",
            "--path-blocking-candidate-pool", "4",
            "--path-blocking-query-count", "3",
            "--path-blocking-box-half-width", "0.14",
            "--max-workspace-obstacles", "180",
            "--n-samples-crit", "500",
            "--allowed-link-idxs", "2,3,4,5,6,7",
            "--workspace-aabb-shrink", "0.0",
            "--prefix-fine-until", "12",
            "--prefix-mid-step", "1",
            "--prefix-coarse-step", "20",
        ]
    if key == "panda":
        return common + [
            "--query-local-radius", "0.18",
            "--obstacle-order", "path_blocking",
            "--path-blocking-max-prefix", "40",
            "--path-blocking-candidate-pool", "8",
            "--path-blocking-query-count", "3",
            "--max-workspace-obstacles", "120",
            "--n-samples-crit", "220",
            "--allowed-link-idxs", "2,3,4,5,6,7",
            "--workspace-aabb-shrink", "0.18",
            "--prefix-fine-until", "12",
            "--prefix-mid-step", "1",
            "--prefix-coarse-step", "20",
        ]
    raise ValueError(f"unknown robot profile: {robot}")


def confirm_args() -> list[str]:
    return [
        "--queries-per-scene", "10",
        "--prefix-selection-mode", "distribution",
        "--prefix-confirm-mode", "two_stage",
        "--prefix-stage-a-planner-seeds", "1",
        "--prefix-stage-b-planner-seeds", "3",
        "--prefix-stage-b-neighbor-radius", "1",
        "--prefix-fine-until", "12",
        "--prefix-mid-step", "1",
        "--prefix-coarse-step", "10",
        "--planner-seeds", "3",
        "--rrt-probe-timeout-s", "1.0",
        "--bitstar-probe-timeout-s", "1.0",
        "--bitstar-probe-checkpoint-interval-s", "0.005",
        "--bitstar-probe-mode", "trace",
        "--bitstar-samples-per-batch", "100",
        "--strict-audit-step", "0.01",
        "--min-probe-success-fraction", "1.0",
        "--distribution-hard-not-faster-factor", "1.0",
    ]


def run_command(cmd: list[str], *, timeout_s: float, dry_run: bool) -> bool:
    print("[q10x10]", " ".join(cmd), flush=True)
    if dry_run:
        return True
    result = subprocess.run(cmd, cwd=REPO_ROOT, timeout=float(timeout_s), check=False)
    return result.returncode == 0


def main() -> int:
    args = parse_args()
    parts_dir = args.out_dir / "q10x10_parts"
    parts_dir.mkdir(parents=True, exist_ok=True)
    robots = csv_list(args.robots)
    tasks = [(robot, seed) for robot in robots for seed in range(max(1, int(args.scene_seeds)))]
    for robot, scene_seed in progress(tasks, total=len(tasks), desc="q10x10 scene groups"):
        final_confirm = parts_dir / f"{robot}_seed{scene_seed}_confirm.json"
        final_summary = parts_dir / f"{robot}_seed{scene_seed}_confirm_summary.json"
        if final_confirm.exists() and not args.force:
            continue
        success = False
        for attempt in range(max(1, int(args.max_attempts))):
            candidate = parts_dir / f"{robot}_seed{scene_seed}_candidate_attempt{attempt}.json"
            candidate_summary = parts_dir / f"{robot}_seed{scene_seed}_candidate_attempt{attempt}_summary.json"
            confirm = parts_dir / f"{robot}_seed{scene_seed}_confirm_attempt{attempt}.json"
            confirm_summary = parts_dir / f"{robot}_seed{scene_seed}_confirm_attempt{attempt}_summary.json"
            if not args.confirm_only and (args.force or not candidate.exists()):
                candidate_cmd = [
                    sys.executable,
                    "experiments/common/generate_prefix_mapped_workspace_catalog.py",
                    "--out", str(candidate),
                    "--summary-json", str(candidate_summary),
                    "--robots", robot,
                    "--scene-seed-start", str(scene_seed),
                    "--scene-seeds", "1",
                    "--queries-per-scene", str(args.queries_per_scene),
                    "--initial-queries-per-scene", str(args.queries_per_scene),
                    "--seed-base", str(int(args.seed_base) + 10_000_019 * attempt),
                    "--max-scene-tries", "1",
                    *robot_candidate_args(robot),
                ]
                if not run_command(candidate_cmd, timeout_s=float(args.timeout_s), dry_run=bool(args.dry_run)):
                    continue
            if args.candidate_only:
                success = True
                break
            if not candidate.exists() and not args.dry_run:
                continue
            confirm_cmd = [
                sys.executable,
                "experiments/common/augment_prefix_catalog_queries.py",
                "--input", str(candidate),
                "--out", str(confirm),
                "--summary-json", str(confirm_summary),
                *confirm_args(),
            ]
            if run_command(confirm_cmd, timeout_s=float(args.timeout_s), dry_run=bool(args.dry_run)):
                if not args.dry_run:
                    shutil.copyfile(confirm, final_confirm)
                    shutil.copyfile(confirm_summary, final_summary)
                success = True
                break
        if not success:
            raise RuntimeError(f"failed to confirm {robot}/seed{scene_seed} after {args.max_attempts} attempts")
    if not args.candidate_only:
        final_catalog = args.out_dir / "distribution_q10x10_three_robot_strict_catalog.json"
        assemble_cmd = [
            sys.executable,
            "experiments/common/assemble_q10x10_prefix_catalog.py",
            "--parts-dir", str(parts_dir),
            "--out", str(final_catalog),
            "--report-json", str(args.out_dir / "distribution_q10x10_three_robot_strict_report.json"),
            "--report-md", str(args.out_dir / "distribution_q10x10_three_robot_strict_report.md"),
            "--robots", ",".join(robots),
            "--scene-seeds", str(args.scene_seeds),
            "--queries-per-scene", str(args.queries_per_scene),
        ]
        if not run_command(assemble_cmd, timeout_s=120.0, dry_run=bool(args.dry_run)):
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
