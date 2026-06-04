#!/usr/bin/env python3
"""Verify why chain_pave cannot fully tile a BiRRT segment bridge with boxes on
the REAL IIWA-14 + shelf scene used by exp04_shelf_ablation (baseline group).

Baseline group reproduced here = the exact config of
`baseline_warm_aafk_support_hull_8t_aafk_volume_min`:
  - endpoint_source = AAFK (safe channel)
  - envelope        = support_hull
  - lect split      = aafk_volume_min (8 sample nodes / depth)
  - canonical root intervals (joint_symmetry_native_v1)

Pipeline mirrored:
  connector BiRRT (sbf.rrt_connect_path) produces a polyline "segment bridge",
  then connector.chain_pave_along_path() walks the bridge and tries to certify a
  free box at each step via FindFreeBoxService::find (the canonical kd-tree
  descent). We reproduce the *descent* with forest.debug_find_free_box() -- the
  same code path chain_pave uses -- sampling densely along each real bridge and
  sweeping the FFB max_depth.

What we measure per query/seed:
  * coverage_by_depth: fraction of bridge samples that get a certified-free box
  * residual failures at the deepest depth: fail_code histogram and the rate of
    hit_unknown_depth_cap (a node that straddles the C-space obstacle boundary
    and can NEVER be certified Free at any finite depth -> the structural cause)
  * adjacency_breaks: among consecutive certified samples, how many box pairs are
    NOT face-adjacent (chain_pave rejects diagonal-only neighbours, so even fully
    certified neighbours can fail to *chain*).

Read-only w.r.t. any production DB: debug_find_free_box runs against a throwaway
cache root built fresh here.

Usage:
    PYTHONPATH=$PWD/build-rbf-only-exec/python:$PWD \\
    python3 experiments/exp04_shelf_ablation/chain_pave_coverage_shelf.py \\
        --out-dir outputs/logs/chain_pave_coverage_shelf
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Iterable

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


# --------------------------------------------------------------------------- #
# Scene + baseline forest (mirrors exp04 baseline group)
# --------------------------------------------------------------------------- #
def make_base_args(max_depth: int, ffb_start_depth: int) -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    add_common_sbf_args(parser)
    args = parser.parse_args([])
    args.preset = RBF_LIFELONG_PRESET
    args.rbf_envelope = "support_hull"
    args.threads = 1
    args.task_batch_size = 1
    args.seed_base = 0
    args.max_boxes = 1
    args.timeout_ms = 1.0
    args.rbf_ffb_start_depth = int(ffb_start_depth)
    args.rbf_canonical_cache = True
    args.endpoint_source = "aafk"
    args.rbf_max_depth = int(max_depth)
    return args


def build_baseline_forest(
    robot: Any,
    db_path: Path,
    max_depth: int,
    sample_nodes_per_depth: int,
    ffb_start_depth: int,
) -> tuple[Any, Any, list[Any]]:
    """Build the exp04 baseline forest at the deepest sweep depth; we then vary
    only opts.max_depth per probe so a single oracle serves every depth."""
    root = list(sbf.canonical_root_intervals_for_robot(robot, True, "joint_symmetry_native_v1"))
    args = make_base_args(max_depth, ffb_start_depth)
    args.rbf_cache_root = db_path.parent
    args.rbf_cache_label = db_path.name
    cfg = configure_standalone_sbf(args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    cfg.database.path = str(db_path)
    cfg.database.root_intervals_override = list(root)
    cfg.database.split_policy = make_aafk_volume_min_split_policy(
        robot,
        int(max_depth),
        root_intervals=root,
        sample_nodes_per_depth=int(sample_nodes_per_depth),
    )
    try:
        cfg.database.max_tree_depth = int(max_depth) + 1
    except Exception:
        pass
    set_online_cache_backfill(cfg, True)
    forest = sbf.SafeBoxForest(robot, cfg)
    return forest, cfg, root


# --------------------------------------------------------------------------- #
# BiRRT segment bridge
# --------------------------------------------------------------------------- #
def make_rrt_config(timeout_ms: float, max_iters: int, step_size: float,
                    goal_bias: float, segment_resolution: int) -> Any:
    config = sbf.RRTConnectConfig()
    config.timeout_ms = float(timeout_ms)
    config.max_iters = int(max_iters)
    config.step_size = float(step_size)
    config.goal_bias = float(goal_bias)
    config.segment_resolution = int(segment_resolution)
    config.local_sampling_radius = 0.0
    return config


def birrt_bridge(robot: Any, obstacles: list[Any], start: list[float],
                 goal: list[float], config: Any, seed: int) -> list[list[float]]:
    path = sbf.rrt_connect_path(robot, obstacles, list(start), list(goal), config, int(seed))
    return [list(map(float, p)) for p in path] if path else []


def densify(path: list[list[float]], step: float) -> list[list[float]]:
    if len(path) < 2:
        return [list(p) for p in path]
    out: list[list[float]] = []
    pts = [np.asarray(p, dtype=float) for p in path]
    for a, b in zip(pts[:-1], pts[1:]):
        seg = b - a
        dist = float(np.linalg.norm(seg))
        n = max(1, int(np.ceil(dist / max(1e-9, step))))
        for i in range(n):
            out.append((a + (i / n) * seg).tolist())
    out.append(pts[-1].tolist())
    return out


# --------------------------------------------------------------------------- #
# Coverage probe (the exact FFB descent chain_pave uses)
# --------------------------------------------------------------------------- #
def probe_point(forest: Any, obstacles: list[Any], q: list[float], opts: Any,
                max_depth: int) -> dict[str, Any]:
    opts.max_depth = int(max_depth)
    res = dict(forest.debug_find_free_box(list(map(float, q)), obstacles, opts))
    intervals = res.get("intervals", None)
    iv = None
    if intervals is not None:
        iv = []
        for it in intervals:
            if hasattr(it, "lo"):
                iv.append([float(it.lo), float(it.hi)])
            else:
                iv.append([float(it[0]), float(it[1])])
    return {
        "found": bool(res.get("found", False)),
        "fail_code": int(res.get("fail_code", -1)),
        "hit_unknown_depth_cap": bool(res.get("hit_unknown_depth_cap", False)),
        "hit_reserved_depth_cap": bool(res.get("hit_reserved_depth_cap", False)),
        "seed_collision": bool(res.get("seed_collision", False)),
        "seed_in_domain": bool(res.get("seed_in_domain", True)),
        "node": int(res.get("node", -1)),
        "splits": int(res.get("splits", 0)),
        "effective_max_depth": int(res.get("effective_max_depth", -1)),
        "intervals": iv,
    }


def boxes_face_adjacent(iv_a: list[list[float]], iv_b: list[list[float]], tol: float) -> bool:
    """Relaxed adjacency: two boxes count as adjacent as long as their CLOSURES
    intersect (share at least one point), including diagonal corner/edge touches.
    Equivalent to: no dimension is strictly separated. This is more permissive
    than the C++ boxes_connected face-adjacency rule on purpose, to test whether
    accepting diagonal neighbours removes the chain breaks."""
    if iv_a is None or iv_b is None:
        return False
    for (alo, ahi), (blo, bhi) in zip(iv_a, iv_b):
        # strictly separated in this dim -> closures do not intersect
        if ahi < blo - tol or bhi < alo - tol:
            return False
    return True


# --------------------------------------------------------------------------- #
# Per query / seed
# --------------------------------------------------------------------------- #
def run_query(forest: Any, opts: Any, obstacles: list[Any], query: Any,
              rrt_cfg: Any, seed: int, args: argparse.Namespace,
              depths: list[int]) -> dict[str, Any]:
    label = str(getattr(query, "label", getattr(query, "name", "query")))
    start = list(map(float, query.start))
    goal = list(map(float, query.goal))

    t0 = time.perf_counter()
    path = birrt_bridge(args._robot, obstacles, start, goal, rrt_cfg, seed)
    bridge_ms = (time.perf_counter() - t0) * 1000.0
    if not path:
        return {"label": label, "seed": int(seed), "bridge_found": False,
                "bridge_ms": bridge_ms}

    samples = densify(path, float(args.sample_step))
    max_depth = max(depths)

    # Probe each sample at the deepest depth once to get the full record, plus a
    # per-depth found flag (cheaper: probe across all depths only the found map).
    per_sample: list[dict[str, Any]] = []
    coverage_by_depth = {int(d): 0 for d in depths}
    for q in samples:
        # Sweep shallow->deep; once found at a depth it's found at all deeper too,
        # but a node straddling the boundary never becomes Free, so we must probe
        # each depth to capture the saturation curve.
        rec_deep = probe_point(forest, obstacles, q, opts, max_depth)
        found_depth = None
        for d in depths:
            r = probe_point(forest, obstacles, q, opts, d)
            if r["found"]:
                coverage_by_depth[int(d)] += 1
                if found_depth is None:
                    found_depth = int(d)
        per_sample.append({
            "found_deep": rec_deep["found"],
            "found_min_depth": found_depth,
            "fail_code": rec_deep["fail_code"],
            "hit_unknown_depth_cap": rec_deep["hit_unknown_depth_cap"],
            "hit_reserved_depth_cap": rec_deep["hit_reserved_depth_cap"],
            "seed_collision": rec_deep["seed_collision"],
            "node": rec_deep["node"],
            "splits": rec_deep["splits"],
            "intervals": rec_deep["intervals"],
        })

    n = len(samples)
    covered_deep = [s for s in per_sample if s["found_deep"]]
    uncovered_deep = [s for s in per_sample if not s["found_deep"]]

    # residual failure analysis at deepest depth
    fail_hist: dict[str, int] = {}
    unknown_cap = 0
    for s in uncovered_deep:
        key = str(s["fail_code"])
        fail_hist[key] = fail_hist.get(key, 0) + 1
        if s["hit_unknown_depth_cap"]:
            unknown_cap += 1

    # adjacency breaks among consecutive certified samples
    adjacency = {str(tol): {"pairs": 0, "breaks": 0} for tol in args.adjacency_tols}
    for a, b in zip(per_sample[:-1], per_sample[1:]):
        if a["found_deep"] and b["found_deep"]:
            for tol in args.adjacency_tols:
                adjacency[str(tol)]["pairs"] += 1
                if not boxes_face_adjacent(a["intervals"], b["intervals"], float(tol)):
                    adjacency[str(tol)]["breaks"] += 1

    return {
        "label": label,
        "seed": int(seed),
        "bridge_found": True,
        "bridge_ms": bridge_ms,
        "path_waypoints": len(path),
        "n_samples": n,
        "coverage_by_depth": {str(d): coverage_by_depth[int(d)] / n for d in depths},
        "covered_count_deep": len(covered_deep),
        "uncovered_count_deep": len(uncovered_deep),
        "coverage_deep": len(covered_deep) / n if n else 0.0,
        "fail_code_hist_uncovered": fail_hist,
        "hit_unknown_depth_cap_uncovered": unknown_cap,
        "adjacency": adjacency,
    }


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", type=Path,
                        default=REPO_ROOT / "outputs" / "logs" / "chain_pave_coverage_shelf")
    parser.add_argument("--queries", default="all",
                        help="Comma-separated query labels (e.g. AS->TS) or all.")
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--sample-step", type=float, default=0.05,
                        help="C-space resampling step (rad) along the bridge.")
    parser.add_argument("--min-depth", type=int, default=16)
    parser.add_argument("--max-depth", type=int, default=44)
    parser.add_argument("--depth-step", type=int, default=4)
    parser.add_argument("--aafk-sample-nodes-per-depth", type=int, default=8)
    parser.add_argument("--ffb-start-depth", type=int, default=0)
    parser.add_argument("--adjacency-tols", type=float, nargs="+", default=[1e-9, 1e-6, 1e-3])
    # BiRRT (production-mirroring) defaults
    parser.add_argument("--rrt-timeout-ms", type=float, default=5000.0)
    parser.add_argument("--rrt-max-iters", type=int, default=200000)
    parser.add_argument("--rrt-step-size", type=float, default=0.5)
    parser.add_argument("--rrt-goal-bias", type=float, default=0.2)
    parser.add_argument("--rrt-segment-resolution", type=int, default=32)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    depths = list(range(int(args.min_depth), int(args.max_depth) + 1, int(args.depth_step)))
    args.out_dir.mkdir(parents=True, exist_ok=True)

    robot = sbf.load_iiwa14_robot()
    args._robot = robot
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    if str(args.queries) != "all":
        wanted = {item.strip() for item in str(args.queries).split(",") if item.strip()}
        queries = [q for q in queries if str(getattr(q, "label", "")) in wanted]

    db_root = Path(tempfile.mkdtemp(prefix="chain_pave_cov_shelf_"))
    db_path = db_root / "baseline_db"
    forest, cfg, root = build_baseline_forest(
        robot, db_path, max(depths), int(args.aafk_sample_nodes_per_depth),
        int(args.ffb_start_depth))
    opts = cfg.grower.find_free_box

    rrt_cfg = make_rrt_config(args.rrt_timeout_ms, args.rrt_max_iters,
                              args.rrt_step_size, args.rrt_goal_bias,
                              args.rrt_segment_resolution)

    rows: list[dict[str, Any]] = []
    for query in queries:
        for seed in args.seeds:
            row = run_query(forest, opts, obstacles, query, rrt_cfg, int(seed),
                            args, depths)
            rows.append(row)
            tag = f"{row['label']} seed={seed}"
            if not row["bridge_found"]:
                print(f"[cov-shelf] {tag}: bridge NOT found ({row['bridge_ms']:.0f} ms)")
                continue
            print(f"[cov-shelf] {tag}: n={row['n_samples']} "
                  f"cov_deep={row['coverage_deep']:.3f} "
                  f"uncovered={row['uncovered_count_deep']} "
                  f"unknown_cap={row['hit_unknown_depth_cap_uncovered']} "
                  f"fail_hist={row['fail_code_hist_uncovered']}")

    payload = {
        "experiment": "exp04_chain_pave_coverage_shelf",
        "scene": "iiwa14 + make_combined_obstacles",
        "baseline_group": "baseline_warm_aafk_support_hull_8t_aafk_volume_min",
        "params": {
            "depths": depths,
            "sample_step": float(args.sample_step),
            "aafk_sample_nodes_per_depth": int(args.aafk_sample_nodes_per_depth),
            "ffb_start_depth": int(args.ffb_start_depth),
            "adjacency_tols": [float(t) for t in args.adjacency_tols],
            "rrt": {
                "timeout_ms": float(args.rrt_timeout_ms),
                "max_iters": int(args.rrt_max_iters),
                "step_size": float(args.rrt_step_size),
                "goal_bias": float(args.rrt_goal_bias),
                "segment_resolution": int(args.rrt_segment_resolution),
            },
            "seeds": [int(s) for s in args.seeds],
        },
        "rows": rows,
    }
    out_json = args.out_dir / "coverage_results.json"
    out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(f"[cov-shelf] wrote {out_json}")

    shutil.rmtree(db_root, ignore_errors=True)

    # ---- aggregate summary --------------------------------------------------
    found_rows = [r for r in rows if r.get("bridge_found")]
    if found_rows:
        print("\n=== SUMMARY (baseline, iiwa+shelf) ===")
        for d in depths:
            vals = [r["coverage_by_depth"][str(d)] for r in found_rows]
            print(f"  depth={d:>2}: mean_coverage={np.mean(vals):.3f}")
        tot_unc = sum(r["uncovered_count_deep"] for r in found_rows)
        tot_unk = sum(r["hit_unknown_depth_cap_uncovered"] for r in found_rows)
        print(f"  deepest depth: uncovered_samples={tot_unc}, "
              f"of which hit_unknown_depth_cap={tot_unk} "
              f"({(tot_unk / tot_unc * 100.0) if tot_unc else 0.0:.1f}%)")
        for tol in args.adjacency_tols:
            pairs = sum(r["adjacency"][str(tol)]["pairs"] for r in found_rows)
            breaks = sum(r["adjacency"][str(tol)]["breaks"] for r in found_rows)
            print(f"  adjacency tol={tol:g}: breaks={breaks}/{pairs} "
                  f"({(breaks / pairs * 100.0) if pairs else 0.0:.1f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
