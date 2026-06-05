#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import write_json
from experiments.common.rbf_defaults import (
    CANONICAL_SYMMETRY_DESCRIPTOR,
    D23_CACHE_ROOT,
    D23_CACHE_LABEL,
    D23_NATIVE_DIM0_ROOT_INTERVALS,
    D23_ROOT_INTERVALS,
    robot_sector_expanded_root_tuples,
)
from experiments.common.rbf_leaf_rrt import make_aafk_split_policy
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


def far_obstacle() -> Any:
    return sbf.Obstacle(100.0, 100.0, 100.0, 101.0, 101.0, 101.0)


def directory_size(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def make_config(cache_path: Path, *, max_depth: int, threads: int, root_mode: str) -> Any:
    robot = sbf.load_iiwa14_robot()
    cfg = sbf.SBFConfig()
    cfg.enable_connector = False
    cfg.endpoint_source.source = sbf.EndpointSource.IFK
    cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
    cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
    cfg.validation.accept_unsafe_free = False
    cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly

    cfg.database.path = str(cache_path)
    cfg.database.create_if_missing = True
    cfg.database.max_tree_depth = int(max_depth)
    cfg.database.canonical_mode = True
    cfg.database.symmetry_descriptor = CANONICAL_SYMMETRY_DESCRIPTOR
    cfg.database.online_cache.allow_database_backfill = True
    if root_mode == "shelf_dim0q4":
        root_tuples = list(D23_ROOT_INTERVALS)
        coverage_tuples = robot_sector_expanded_root_tuples("iiwa", robot) or root_tuples
        root_intervals = [sbf.Interval(float(lo), float(hi)) for lo, hi in root_tuples]
        cfg.database.root_intervals_override = root_intervals
        cfg.database.coverage_intervals_override = [
            sbf.Interval(float(lo), float(hi)) for lo, hi in coverage_tuples
        ]
        cfg.database.split_policy = make_aafk_split_policy(robot, int(max_depth), root_intervals)
    elif root_mode == "shelf_native_dim0_forced_q4":
        root_tuples = list(D23_NATIVE_DIM0_ROOT_INTERVALS)
        coverage_tuples = robot_sector_expanded_root_tuples("iiwa", robot) or list(D23_ROOT_INTERVALS)
        root_intervals = [sbf.Interval(float(lo), float(hi)) for lo, hi in root_tuples]
        cfg.database.root_intervals_override = root_intervals
        cfg.database.coverage_intervals_override = [
            sbf.Interval(float(lo), float(hi)) for lo, hi in coverage_tuples
        ]
        cfg.database.split_policy = make_aafk_split_policy(
            robot,
            int(max_depth),
            root_intervals,
            force_dim0_first_two=True,
        )
    elif root_mode == "full_root":
        cfg.database.split_policy = make_aafk_split_policy(robot, int(max_depth), None)
    else:
        raise ValueError(f"unsupported root mode: {root_mode}")

    n_threads = max(1, int(threads))
    cfg.runtime.mode = sbf.ExecutionMode.Parallel if n_threads > 1 else sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = n_threads
    cfg.runtime.batch_size = n_threads
    cfg.runtime.parallel_threshold = 1
    cfg.grower.n_threads = n_threads
    cfg.grower.task_batch_size = n_threads
    return robot, cfg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build Exp4 IIWA d23 canonical LECT cache. The --depth value is "
            "canonical LECT tree depth, not native/all-sector tree depth."
        )
    )
    parser.add_argument("--cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--cache-label", default=D23_CACHE_LABEL)
    parser.add_argument("--depth", type=int, default=23)
    parser.add_argument("--max-depth", type=int, default=40)
    parser.add_argument("--root-mode", choices=["shelf_dim0q4", "shelf_native_dim0_forced_q4", "full_root"], default="shelf_dim0q4")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--clean", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--verify", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--publish-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--streaming", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--resident-cap", type=int, default=500_000)
    parser.add_argument("--checkpoint-seconds", type=float, default=30.0)
    parser.add_argument("--progress", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--out-json", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if int(args.depth) > 23:
        raise SystemExit(
            "Refusing to prewarm depth > 23 for Exp4 cache. Depth is canonical LECT tree depth; use depth <= 23."
        )
    cache_path = Path(args.cache_root) / str(args.cache_label)
    if args.root_mode == "full_root":
        root_tag = "full_root"
    elif args.root_mode == "shelf_native_dim0_forced_q4":
        root_tag = "native_dim0_full_forced_dim0q4_shelf_tail"
    else:
        root_tag = "canonical_dim0q4_fixed_root"
    out_json = args.out_json or (cache_path.parent.parent / f"d{int(args.depth)}_prewarm_{root_tag}_aafk_volume_min.json")
    if bool(args.clean) and cache_path.exists():
        shutil.rmtree(cache_path)
    cache_path.parent.mkdir(parents=True, exist_ok=True)

    if bool(args.streaming):
        os.environ["SBF_PREWARM_STREAMING"] = "1"
        os.environ["SBF_PREWARM_RESIDENT_CAP"] = str(max(1, int(args.resident_cap)))
    else:
        os.environ.pop("SBF_PREWARM_STREAMING", None)
        os.environ.pop("SBF_PREWARM_RESIDENT_CAP", None)
    if float(args.checkpoint_seconds) > 0.0:
        os.environ["SBF_PREWARM_CHECKPOINT_SECONDS"] = str(float(args.checkpoint_seconds))
    else:
        os.environ.pop("SBF_PREWARM_CHECKPOINT_SECONDS", None)
    os.environ["SBF_PREWARM_PROGRESS"] = "1" if bool(args.progress) else "0"
    os.environ.setdefault("SBF_PREWARM_VERIFY", "0")
    os.environ.setdefault("SBF_PREWARM_VERIFY_STRICT", "0")

    print(
        "prewarm config: "
        f"depth={int(args.depth)} max_depth={int(args.max_depth)} threads={int(args.threads)} "
        f"root_mode={args.root_mode} "
        f"streaming={bool(args.streaming)} resident_cap={int(args.resident_cap)} "
        f"checkpoint_s={float(args.checkpoint_seconds)} progress={bool(args.progress)} "
        f"cache={cache_path}",
        file=sys.stderr,
        flush=True,
    )

    robot, cfg = make_config(cache_path, max_depth=int(args.max_depth), threads=int(args.threads), root_mode=str(args.root_mode))
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    prewarm = dict(forest.prewarm_lifelong_cache(int(args.depth), [far_obstacle()]))
    wall_s = time.perf_counter() - t0
    verify_ok = bool(forest.database_verify(True)) if bool(args.verify) else True
    snapshot_ok = bool(forest.database_wait_for_snapshot_publish()) if bool(args.publish_snapshot) else False
    summary = {
        "ok": bool(prewarm.get("ok")) and verify_ok,
        "cache_path": str(cache_path),
        "cache_root": str(args.cache_root),
        "cache_label": str(args.cache_label),
        "depth": int(args.depth),
        "max_depth": int(args.max_depth),
        "threads": int(args.threads),
        "canonical_mode": True,
        "depth_semantics": "canonical_lect_tree",
        "symmetry_descriptor": CANONICAL_SYMMETRY_DESCRIPTOR,
        "root_mode": str(args.root_mode),
        "root_intervals": (
            "shelf_task_root_dim0_minus_pi_to_pi"
            if args.root_mode == "shelf_native_dim0_forced_q4"
            else ("shelf_task_root_primary_sector" if args.root_mode == "shelf_dim0q4" else "robot_joint_limits_with_canonical_symmetry")
        ),
        "coverage_intervals": (
            "shelf_task_root_reflected_dim0_all_sectors"
            if args.root_mode in {"shelf_dim0q4", "shelf_native_dim0_forced_q4"}
            else "full_robot_joint_limits"
        ),
        "wall_s": wall_s,
        "prewarm": prewarm,
        "verify_ok": verify_ok,
        "snapshot_ok": snapshot_ok,
        "snapshot_path": str(cache_path / "lect_snapshot"),
        "snapshot_exists": (cache_path / "lect_snapshot").exists(),
        "cache_bytes": directory_size(cache_path),
    }
    write_json(out_json, summary)
    print(f"prewarm ok={summary['ok']} cache={cache_path} wall_s={wall_s:.3f} bytes={summary['cache_bytes']}")
    print(f"snapshot_ok={snapshot_ok} snapshot={summary['snapshot_path']}")
    if not summary["ok"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
