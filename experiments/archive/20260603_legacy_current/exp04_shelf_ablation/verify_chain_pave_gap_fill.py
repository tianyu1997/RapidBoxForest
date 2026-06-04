#!/usr/bin/env python3
"""Verify the production chain_pave gap-filling fix on the REAL IIWA-14 + shelf
baseline scene (exp04_shelf_ablation).

We grow an RBFPlanningForest with build(start, goal, obstacles) so start/goal
boxes exist, then run forest.debug_chain_pave(start, goal, ...) twice:

    A) fill_segment_gaps = False  -> original behaviour (stops at first gap)
    B) fill_segment_gaps = True   -> new behaviour (bisect + insert middle boxes)

For each run we densify the BiRRT bridge waypoints debug_chain_pave actually used
and measure the fraction of densified samples that lie inside SOME committed box
(start box, goal box, or any chain box). Gap filling should raise coverage and
the number of committed boxes.

Read-only w.r.t. any production DB (throwaway cache root).

Usage:
    PYTHONPATH=$PWD/build-rbf-only-exec/python:$PWD \\
    python3 experiments/exp04_shelf_ablation/verify_chain_pave_gap_fill.py \\
        --queries all --out-dir outputs/logs/verify_chain_pave_gap_fill
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    RBF_LIFELONG_PRESET,
    add_common_sbf_args,
    configure_standalone_sbf,
    make_aafk_volume_min_split_policy,
    set_online_cache_backfill,
)

import sbf  # noqa: E402


def make_base_args(max_depth: int, ffb_start_depth: int) -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    add_common_sbf_args(parser)
    args = parser.parse_args([])
    args.preset = RBF_LIFELONG_PRESET
    args.rbf_envelope = "support_hull"
    args.threads = 1
    args.task_batch_size = 1
    args.seed_base = 0
    args.max_boxes = 4096
    args.timeout_ms = 60000.0
    args.rbf_ffb_start_depth = int(ffb_start_depth)
    args.rbf_canonical_cache = True
    args.endpoint_source = "aafk"
    args.rbf_max_depth = int(max_depth)
    return args


def build_forest(robot: Any, db_path: Path, max_depth: int,
                 sample_nodes_per_depth: int, ffb_start_depth: int) -> Any:
    root = list(sbf.canonical_root_intervals_for_robot(robot, True, "joint_symmetry_native_v1"))
    args = make_base_args(max_depth, ffb_start_depth)
    args.rbf_cache_root = db_path.parent
    args.rbf_cache_label = db_path.name
    cfg = configure_standalone_sbf(args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    cfg.database.path = str(db_path)
    cfg.database.root_intervals_override = list(root)
    cfg.database.split_policy = make_aafk_volume_min_split_policy(
        robot, int(max_depth), root_intervals=root,
        sample_nodes_per_depth=int(sample_nodes_per_depth),
    )
    try:
        cfg.database.max_tree_depth = int(max_depth) + 1
    except Exception:
        pass
    set_online_cache_backfill(cfg, True)
    return sbf.SafeBoxForest(robot, cfg)


def densify(path: list[list[float]], step: float) -> list[np.ndarray]:
    if len(path) < 2:
        return [np.asarray(p, dtype=float) for p in path]
    out: list[np.ndarray] = []
    pts = [np.asarray(p, dtype=float) for p in path]
    for a, b in zip(pts[:-1], pts[1:]):
        seg = b - a
        dist = float(np.linalg.norm(seg))
        n = max(1, int(np.ceil(dist / max(1e-9, step))))
        for i in range(n):
            out.append(a + (i / n) * seg)
    out.append(pts[-1])
    return out


def box_contains(intervals: list, q: np.ndarray, tol: float) -> bool:
    for d, iv in enumerate(intervals):
        lo, hi = (iv[0], iv[1]) if not hasattr(iv, "lo") else (iv.lo, iv.hi)
        if q[d] < lo - tol or q[d] > hi + tol:
            return False
    return True


def coverage(boxes: list[list], samples: list[np.ndarray], tol: float) -> float:
    if not samples:
        return 1.0
    covered = 0
    for q in samples:
        if any(box_contains(b, q, tol) for b in boxes):
            covered += 1
    return covered / len(samples)


def run_one(forest: Any, start: list[float], goal: list[float], *,
            fill: bool, max_chain: int, max_depth: int, sample_step: float,
            tol: float, adjacency_tolerance: float,
            gap_fill_sample_step: float = 0.01,
            gap_fill_time_budget_ms: float = 10.0,
            gap_fill_max_ffb_calls: int = 32,
            gap_fill_min_arc_gain: float = 0.01,
            robot: Any = None, obstacles: Any = None) -> dict[str, Any]:
    res = dict(forest.debug_chain_pave(
        list(map(float, start)), list(map(float, goal)),
        max_chain=max_chain, max_depth=max_depth,
        fill_segment_gaps=bool(fill),
        max_gap_fill_steps=20, gap_fill_min_step=1e-5,
        adjacency_tolerance=adjacency_tolerance,
        gap_fill_sample_step=gap_fill_sample_step,
        gap_fill_time_budget_ms=gap_fill_time_budget_ms,
        gap_fill_max_ffb_calls=gap_fill_max_ffb_calls,
        gap_fill_min_arc_gain=gap_fill_min_arc_gain,
    ))
    waypoints = [list(map(float, w)) for w in res.get("waypoints", [])]
    samples = densify(waypoints, sample_step) if waypoints else []
    boxes: list[list] = []
    if res.get("start_box"):
        boxes.append(res["start_box"])
    if res.get("goal_box"):
        boxes.append(res["goal_box"])
    # Prefer the FULL forest box set: chain_pave covers many path points by
    # REUSING pre-existing forest boxes (committed during build), which are
    # absent from `committed_boxes`. Measuring against `committed_boxes` alone
    # under-reports coverage (treats reused-box coverage as a gap). Fall back to
    # committed_boxes only for older bindings without `all_boxes`.
    all_boxes = res.get("all_boxes")
    if all_boxes:
        boxes.extend(all_boxes)
    else:
        boxes.extend(res.get("committed_boxes", []))
    # Classify uncovered samples: a sample that is itself in collision can NEVER
    # be inside a free box, so it is a measurement artifact of linearly
    # interpolating the BiRRT waypoints (not a gap-fill failure).
    uncovered = [q for q in samples if not any(box_contains(b, q, tol) for b in boxes)]
    uncovered_collision = 0
    uncovered_free = 0
    ffb_fail = 0
    ffb_box_missing = 0
    ffb_box_ok = 0
    if fill and robot is not None and obstacles is not None:
        opts = sbf.FindFreeBoxOptions()
        opts.max_depth = max_depth
        opts.reject_seed_collision = False
        for q in uncovered:
            if sbf.check_config_collision(robot, obstacles, list(map(float, q))):
                uncovered_collision += 1
                continue
            uncovered_free += 1
            try:
                fr = forest.debug_find_free_box(list(map(float, q)), list(obstacles), opts, True)
            except Exception:
                fr = None
            if not fr or not fr.get("found"):
                ffb_fail += 1
            elif box_contains(list(fr.get("intervals", [])), q, tol):
                ffb_box_ok += 1
            else:
                ffb_box_missing += 1
    n = len(samples)
    cov = (n - len(uncovered)) / n if n else 1.0
    cov_free = (n - uncovered_free) / n if n else 1.0
    return {
        "fill": fill,
        "bridge_found": bool(res.get("bridge_found")),
        "audit_passed": bool(res.get("audit_passed")),
        "added": int(res.get("added", 0)),
        "fast_gap_fill_ffb_calls": int(res.get("fast_gap_fill_ffb_calls", 0)),
        "fast_gap_fill_ms": float(res.get("fast_gap_fill_ms", 0.0)),
        "n_committed_boxes": len(res.get("committed_boxes", [])),
        "n_samples": n,
        "coverage": cov,
        "n_uncovered": len(uncovered),
        "n_uncovered_collision": uncovered_collision,
        "n_uncovered_free": uncovered_free,
        "ffb_fail": ffb_fail,
        "ffb_box_missing": ffb_box_missing,
        "ffb_box_ok": ffb_box_ok,
        "coverage_of_free": cov_free,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--queries", default="all")
    parser.add_argument("--max-depth", type=int, default=120)
    parser.add_argument("--ffb-start-depth", type=int, default=8)
    parser.add_argument("--sample-nodes-per-depth", type=int, default=8)
    parser.add_argument("--max-chain", type=int, default=4096)
    parser.add_argument("--sample-step", type=float, default=0.05)
    parser.add_argument("--adjacency-tolerance", type=float, default=1e-9)
    parser.add_argument("--gap-fill-sample-step", type=float, default=0.05)
    parser.add_argument("--gap-fill-time-budget-ms", type=float, default=10.0)
    parser.add_argument("--gap-fill-max-ffb-calls", type=int, default=32)
    parser.add_argument("--gap-fill-min-arc-gain", type=float, default=0.01)
    parser.add_argument("--tol", type=float, default=1e-6)
    parser.add_argument("--out-dir", default="outputs/logs/verify_chain_pave_gap_fill")
    args = parser.parse_args()

    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    if args.queries != "all":
        wanted = {s.strip() for s in args.queries.split(",")}
        queries = [q for q in queries if str(getattr(q, "label", "")) in wanted]

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    for qi, q in enumerate(queries):
        label = str(getattr(q, "label", f"q{qi}"))
        start = list(map(float, q.start))
        goal = list(map(float, q.goal))
        cache_root = Path(tempfile.mkdtemp(prefix=f"gapfill_{label}_"))
        try:
            db_path = cache_root / "db"
            forest = build_forest(robot, db_path, args.max_depth,
                                  args.sample_nodes_per_depth, args.ffb_start_depth)
            forest.build(start, goal, obstacles)
            for fill in (False, True):
                t0 = time.time()
                r = run_one(forest, start, goal, fill=fill,
                            max_chain=args.max_chain, max_depth=args.max_depth,
                            sample_step=args.sample_step, tol=args.tol,
                            adjacency_tolerance=args.adjacency_tolerance,
                            gap_fill_sample_step=args.gap_fill_sample_step,
                            gap_fill_time_budget_ms=args.gap_fill_time_budget_ms,
                            gap_fill_max_ffb_calls=args.gap_fill_max_ffb_calls,
                            gap_fill_min_arc_gain=args.gap_fill_min_arc_gain,
                            robot=robot, obstacles=obstacles)
                r["query"] = label
                r["wall_s"] = round(time.time() - t0, 2)
                rows.append(r)
                print(f"[{label}] fill={fill!s:5}  added={r['added']:3d}  "
                      f"boxes={r['n_committed_boxes']:3d}  "
                      f"coverage={r['coverage']*100:6.2f}%  "
                      f"free_cov={r['coverage_of_free']*100:6.2f}%  "
                      f"unc(coll/free)={r['n_uncovered_collision']}/{r['n_uncovered_free']}  "
                      f"ffb(fail/miss/ok)={r['ffb_fail']}/{r['ffb_box_missing']}/{r['ffb_box_ok']}  "
                      f"samples={r['n_samples']:4d}  fast={r['fast_gap_fill_ffb_calls']}/"
                      f"{r['fast_gap_fill_ms']:.2f}ms  audit={r['audit_passed']}  "
                      f"({r['wall_s']}s)")
        finally:
            shutil.rmtree(cache_root, ignore_errors=True)

    summary = {
        "scene": "iiwa14 + make_combined_obstacles (exp04 baseline)",
        "config": {
            "max_depth": args.max_depth,
            "ffb_start_depth": args.ffb_start_depth,
            "sample_nodes_per_depth": args.sample_nodes_per_depth,
            "max_chain": args.max_chain,
            "sample_step": args.sample_step,
        },
        "rows": rows,
    }
    out_file = out_dir / "gap_fill_result.json"
    out_file.write_text(json.dumps(summary, indent=2))
    print(f"\nWrote {out_file}")

    # aggregate
    def avg(fill: bool, key: str) -> float:
        vals = [r[key] for r in rows if r["fill"] is fill]
        return sum(vals) / len(vals) if vals else float("nan")
    print("\n=== aggregate ===")
    print(f"fill=False  mean coverage={avg(False,'coverage')*100:6.2f}%  "
          f"mean boxes={avg(False,'n_committed_boxes'):.1f}")
    print(f"fill=True   mean coverage={avg(True,'coverage')*100:6.2f}%  "
          f"mean boxes={avg(True,'n_committed_boxes'):.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
