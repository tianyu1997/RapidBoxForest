#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import csv_list
from experiments.common.generate_prefix_mapped_workspace_catalog import (
    distribution_prefix_scan_rows,
    select_distribution_prefixes_from_scan,
)
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import make_robot


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


def robot_candidate_args(robot: str, attempt: int = 0) -> list[str]:
    key = str(robot).lower()
    attempt = int(attempt)
    common = [
        "--query-sampling-mode", "local",
        "--query-min-l2", "0.8",
        "--endpoint-source", "critsample",
        "--prefix-selection-mode", "candidate",
        "--planner-seeds", "1",
        "--rrt-probe-timeout-s", "0.5",
        "--bitstar-probe-timeout-s", "0.5",
        "--bitstar-probe-checkpoint-interval-s", "0.005",
        "--bitstar-probe-mode", "path",
        "--bitstar-samples-per-batch", "100",
        "--strict-audit-step", "0.01",
        "--min-probe-success-fraction", "0.5",
    ]
    if key == "ur5":
        profiles = [
            [
                "--query-local-radius", "0.22",
                "--obstacle-order", "mixed",
                "--max-workspace-obstacles", "100",
                "--n-samples-crit", "220",
                "--allowed-link-idxs", "2,3,4",
                "--workspace-aabb-shrink", "0.5",
            ],
            [
                "--query-local-radius", "0.16",
                "--wall-count", "8",
                "--gate-gap-scale", "0.12",
                "--cspace-subdivide-width", "0.08",
                "--max-subboxes-per-cspace-box", "192",
                "--obstacle-order", "source_order",
                "--max-workspace-obstacles", "180",
                "--n-samples-crit", "360",
                "--allowed-link-idxs", "2,3,4,5",
                "--workspace-aabb-shrink", "0.8",
            ],
        ]
        return common + profiles[attempt % len(profiles)] + [
            "--prefix-fine-until", "80",
            "--prefix-mid-step", "10",
            "--prefix-coarse-step", "50",
        ]
    if key == "iiwa":
        profiles = [
            [
                "--query-local-radius", "0.06",
                "--wall-count", "8",
                "--gate-gap-scale", "0.12",
                "--cspace-subdivide-width", "0.08",
                "--max-subboxes-per-cspace-box", "256",
                "--obstacle-order", "source_order",
                "--max-workspace-obstacles", "120",
                "--n-samples-crit", "500",
                "--allowed-link-idxs", "4,6,7",
                "--workspace-aabb-shrink", "1.0",
            ],
            [
                "--query-local-radius", "0.06",
                "--wall-count", "8",
                "--gate-gap-scale", "0.12",
                "--cspace-subdivide-width", "0.08",
                "--max-subboxes-per-cspace-box", "256",
                "--obstacle-order", "source_order",
                "--max-workspace-obstacles", "1000",
                "--n-samples-crit", "500",
                "--allowed-link-idxs", "2,3,4,5,6,7",
                "--workspace-aabb-shrink", "1.0",
            ],
            [
                "--query-local-radius", "0.10",
                "--wall-count", "8",
                "--gate-gap-scale", "0.10",
                "--cspace-subdivide-width", "0.08",
                "--max-subboxes-per-cspace-box", "256",
                "--obstacle-order", "source_order",
                "--max-workspace-obstacles", "240",
                "--n-samples-crit", "500",
                "--allowed-link-idxs", "2,3,4,5,6,7",
                "--workspace-aabb-shrink", "0.9",
            ],
        ]
        return common + profiles[attempt % len(profiles)] + [
            "--prefix-fine-until", "120",
            "--prefix-mid-step", "5",
            "--prefix-coarse-step", "50",
        ]
    if key == "panda":
        profiles = [
            [
                "--query-local-radius", "0.22",
                "--obstacle-order", "mixed",
                "--max-workspace-obstacles", "100",
                "--n-samples-crit", "220",
                "--allowed-link-idxs", "2,3,4",
                "--workspace-aabb-shrink", "0.5",
            ],
            [
                "--query-local-radius", "0.18",
                "--wall-count", "8",
                "--gate-gap-scale", "0.12",
                "--cspace-subdivide-width", "0.08",
                "--max-subboxes-per-cspace-box", "192",
                "--obstacle-order", "source_order",
                "--max-workspace-obstacles", "180",
                "--n-samples-crit", "360",
                "--allowed-link-idxs", "2,3,4,5",
                "--workspace-aabb-shrink", "0.5",
            ],
            [
                "--query-local-radius", "0.26",
                "--obstacle-order", "mixed",
                "--max-workspace-obstacles", "120",
                "--n-samples-crit", "220",
                "--allowed-link-idxs", "2,3,4,5,6,7",
                "--workspace-aabb-shrink", "0.35",
            ],
        ]
        return common + profiles[attempt % len(profiles)] + [
            "--prefix-fine-until", "80",
            "--prefix-mid-step", "10",
            "--prefix-coarse-step", "50",
        ]
    raise ValueError(f"unknown robot profile: {robot}")


def confirm_args() -> list[str]:
    return [
        "--queries-per-scene", "10",
        "--prefix-selection-mode", "distribution",
        "--prefix-confirm-mode", "two_stage",
        "--prefix-stage-a-planner-seeds", "1",
        "--prefix-stage-b-planner-seeds", "1",
        "--prefix-stage-b-neighbor-radius", "1",
        "--prefix-fine-until", "120",
        "--prefix-mid-step", "5",
        "--prefix-coarse-step", "50",
        "--planner-seeds", "1",
        "--rrt-probe-timeout-s", "0.25",
        "--bitstar-probe-timeout-s", "0.25",
        "--bitstar-probe-checkpoint-interval-s", "0.005",
        "--bitstar-probe-mode", "path",
        "--bitstar-samples-per-batch", "100",
        "--strict-audit-step", "0.01",
        "--min-probe-success-fraction", "0.3",
        "--distribution-medium-ratio", "1.0",
        "--distribution-hard-ratio", "1.0",
        "--distribution-min-medium-count", "1",
        "--distribution-min-hard-count", "2",
        "--distribution-hard-not-faster-factor", "0.6",
        "--no-distribution-require-strong-planner",
    ]


def run_command(cmd: list[str], *, timeout_s: float, dry_run: bool) -> bool:
    print("[q10x10]", " ".join(cmd), flush=True)
    if dry_run:
        return True
    result = subprocess.run(cmd, cwd=REPO_ROOT, timeout=float(timeout_s), check=False)
    return result.returncode == 0


def iiwa_candidate_screen_passes(candidate: Path) -> bool:
    payload = json.loads(candidate.read_text(encoding="utf-8"))
    records = [dict(row) for row in payload.get("records", [])]
    hard = next((row for row in records if str(row.get("difficulty")) == "hard"), None)
    if hard is None:
        print(f"[q10x10] screen failed: no hard record in {candidate}", flush=True)
        return False
    ordered = hard.get("workspace_mapping", {}).get("ordered_obstacles", [])
    queries = hard.get("queries", [])
    if len(ordered) < 20 or len(queries) < 10:
        print(
            f"[q10x10] screen failed: ordered={len(ordered)} queries={len(queries)}",
            flush=True,
        )
        return False
    robot = make_robot("iiwa")
    candidate_counts = [
        0,
        max(1, len(ordered) // 8),
        max(2, len(ordered) // 4),
        max(3, len(ordered) // 2),
        20,
        40,
        75,
        120,
        240,
        475,
        775,
        len(ordered),
    ]
    counts = sorted({int(count) for count in candidate_counts if 0 <= int(count) <= len(ordered)})
    rows = distribution_prefix_scan_rows(
        robot_name="iiwa",
        robot=robot,
        bounds_ordered=ordered,
        queries=queries,
        counts=counts,
        desc_suffix="candidate-screen",
        seed=20260609,
        planner_seeds=1,
        direct_obstruction_samples=96,
        rrt_probe_timeout_s=1.0,
        bitstar_timeout_s=1.0,
        bitstar_checkpoint_interval_s=0.005,
        audit_step=0.01,
        min_success_fraction=0.3,
        bitstar_probe_mode="path",
        bitstar_samples_per_batch=100,
        bitstar_rewire_factor=5.0,
    )
    for row in rows:
        metrics = row["metrics"]
        print(
            "[q10x10] screen",
            f"count={row['count']}",
            f"direct={float(row.get('direct_mean', 0.0)):.3f}",
            f"comp={float(metrics['composite']['median_s']):.4f}",
            f"rrt={float(metrics['rrtconnect']['median_s']):.4f}",
            f"bit={float(metrics['bitstar']['median_s']):.4f}",
            flush=True,
        )
    try:
        selection = select_distribution_prefixes_from_scan(
            rows,
            medium_ratio=1.0,
            hard_ratio=1.0,
            hard_not_faster_factor=0.6,
            require_strong_planner=False,
            min_medium_count=1,
            min_hard_count=2,
        )
        print(f"[q10x10] screen passed: {selection['prefix_counts']}", flush=True)
        return True
    except RuntimeError as exc:
        print(f"[q10x10] screen failed: {exc}", flush=True)
        return False


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
                    *robot_candidate_args(robot, attempt),
                ]
                if not run_command(candidate_cmd, timeout_s=float(args.timeout_s), dry_run=bool(args.dry_run)):
                    continue
            if args.candidate_only:
                success = True
                break
            if not candidate.exists() and not args.dry_run:
                continue
            if not args.dry_run and robot == "iiwa" and not iiwa_candidate_screen_passes(candidate):
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
            "--min-probe-success-fraction", "0.3",
        ]
        if not run_command(assemble_cmd, timeout_s=120.0, dry_run=bool(args.dry_run)):
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
