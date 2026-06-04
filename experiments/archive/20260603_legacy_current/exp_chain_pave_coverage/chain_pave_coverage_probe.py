#!/usr/bin/env python3
"""Verify why chain_pave cannot fully tile a BiRRT segment bridge with boxes.

Hypothesis (from reading safe_box_forest/src/connector.cpp + find_free_box.cpp):

  chain_pave_along_path() covers a BiRRT bridge by repeatedly calling
  FindFreeBoxService::find() on interpolated seeds and committing the returned
  *certified-free canonical LECT box*. A point can only be covered if the
  descent reaches a canonical node that validate_node() certifies as
  DefinitelyFree. Two hard walls prevent full coverage even with huge max_depth:

    (1) effective_max_depth = min(options.max_depth, oracle.max_tree_depth()-1)
        -> the tree depth itself caps box resolution.
    (2) A node straddling the C-space obstacle boundary is Unknown/Occupied and
        can NEVER be certified Free at any *finite* depth, because certification
        is conservative (the whole swept box must be collision-free). The seed
        descent then hits hit_unknown_depth_cap (fail_code == 2).
    (3) chain_pave additionally requires each new box to be face-adjacent to the
        previous one (boxes_connected, adjacency_tolerance = 1e-9). Diagonal-only
        neighbours are rejected, breaking the chain even when both endpoints
        individually obtain a box.

This script measures, on a controllable 2-DOF planar robot with a narrow
passage:

  * coverage fraction of the bridge as a function of max_depth (Phase C),
  * the C-space clearance of covered vs uncovered samples (Phase D),
  * chain face-adjacency continuity under several adjacency tolerances (Phase E),
  * how coverage recovers as the passage is widened (Phase F).

It is read-only with respect to the database (debug_find_free_box rebuilds a
fresh oracle from the obstacle scene on every call).

Run:
  source .venv/bin/activate
  PYTHONPATH=$PWD/build-rbf-only-exec/python:$PWD \
    python3 experiments/exp_chain_pave_coverage/chain_pave_coverage_probe.py \
    --out-dir outputs/logs/chain_pave_coverage
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

PI = math.pi

try:
    import sbf
except ImportError as exc:  # pragma: no cover - import guard
    sys.stderr.write(
        "Failed to import sbf. Set PYTHONPATH to the built python dir, e.g.\n"
        "  PYTHONPATH=$PWD/build-rbf-only-exec/python:$PWD\n"
        f"Original error: {exc}\n"
    )
    raise


# --------------------------------------------------------------------------- #
# Scene + robot construction (2-DOF planar arm, controllable narrow passage)
# --------------------------------------------------------------------------- #
def make_planar_robot(link_1: float = 1.0, link_2: float = 1.0, radius: float = 0.02) -> "sbf.Robot":
    dh0 = sbf.DHParam()
    dh0.alpha, dh0.a, dh0.d, dh0.theta, dh0.joint_type = 0.0, float(link_1), 0.0, 0.0, 0
    dh1 = sbf.DHParam()
    dh1.alpha, dh1.a, dh1.d, dh1.theta, dh1.joint_type = 0.0, float(link_2), 0.0, 0.0, 0
    limits = sbf.JointLimits()
    limits.limits = [sbf.Interval(-PI, PI), sbf.Interval(-PI, PI)]
    return sbf.Robot("planar_2dof", [dh0, dh1], limits, None, [radius, radius])


def make_passage_obstacles(gap: float, cx: float = 0.95, hx: float = 0.22, hz: float = 0.20) -> list["sbf.Obstacle"]:
    """Two slabs forming a vertical gate of half-height `gap` around y=0.

    Smaller `gap` -> narrower workspace corridor -> narrower C-space passage.
    """
    big = 1.6
    obstacles = [
        # upper slab: y in [gap, gap+big]
        sbf.Obstacle(cx - hx, gap, -hz, cx + hx, gap + big, hz),
        # lower slab: y in [-(gap+big), -gap]
        sbf.Obstacle(cx - hx, -(gap + big), -hz, cx + hx, -gap, hz),
    ]
    return obstacles


def build_config() -> "sbf.SBFConfig":
    config = sbf.SBFConfig()
    config.enable_merger = False
    config.enable_connector = False
    config.endpoint_source.source = sbf.EndpointSource.CritSample
    config.envelope_type.type = sbf.EnvelopeType.LinkIAABB
    config.envelope_type.n_subdivisions = 4
    config.runtime.mode = sbf.ExecutionMode.Inline
    config.runtime.n_threads = 1
    # deep tree so oracle.max_tree_depth() does not become the binding limit
    config.database.max_tree_depth = 64
    config.query.nearest_if_outside = False
    return config


# --------------------------------------------------------------------------- #
# Collision helpers
# --------------------------------------------------------------------------- #
def make_collision_fn(robot, obstacles):
    """Return free(q)->bool, calibrated so the returned value is True iff q is
    collision-free.

    check_config_collision() returns True when the configuration is *in
    collision* (verified empirically). We calibrate against q=[pi, 0], where the
    2-link arm points to (-2, 0), far from the gate at x~0.95, hence guaranteed
    collision-free for every gap setting.
    """
    raw = lambda q: bool(sbf.check_config_collision(robot, obstacles, list(map(float, q))))
    folded_free = [PI, 0.0]
    free_value = raw(folded_free)  # value 'raw' returns for a known-free pose
    return lambda q: raw(q) == free_value


def straight_line_collides(free_fn, a: np.ndarray, b: np.ndarray, n: int = 64) -> bool:
    for i in range(n + 1):
        t = i / n
        if not free_fn(a + t * (b - a)):
            return True
    return False


def find_query(free_fn, rng: np.random.Generator, tries: int = 4000):
    """Find a non-trivial query: both endpoints free, but the straight line
    between them collides (so a BiRRT bridge is genuinely required)."""
    free_pool = []
    while len(free_pool) < 60 and tries > 0:
        q = rng.uniform(-PI, PI, size=2)
        tries -= 1
        if free_fn(q):
            free_pool.append(q)
    rng.shuffle(free_pool)
    for i in range(len(free_pool)):
        for j in range(i + 1, len(free_pool)):
            a, b = free_pool[i], free_pool[j]
            if 0.6 < float(np.linalg.norm(a - b)) < 3.5 and straight_line_collides(free_fn, a, b):
                return a, b
    return None, None


def clearance(free_fn, q, max_r: float = 0.6, n_dirs: int = 16, n_steps: int = 24) -> float:
    """Approximate C-space clearance: radius of the largest collision-free ball
    around q (probed along n_dirs directions). Cheap but monotone in true
    clearance, which is all we need to separate boundary samples from interior.
    """
    if not free_fn(q):
        return 0.0
    best = max_r
    for k in range(n_dirs):
        ang = 2.0 * PI * k / n_dirs
        d = np.array([math.cos(ang), math.sin(ang)])
        hit = max_r
        for s in range(1, n_steps + 1):
            r = max_r * s / n_steps
            if not free_fn(q + r * d):
                hit = r
                break
        best = min(best, hit)
    return best


# --------------------------------------------------------------------------- #
# BiRRT bridge + dense resampling
# --------------------------------------------------------------------------- #
def birrt_bridge(robot, obstacles, start, goal, seed: int) -> list[list[float]]:
    cfg = sbf.RRTConnectConfig()
    cfg.timeout_ms = 2000.0
    cfg.max_iters = 200000
    cfg.step_size = 0.25
    cfg.goal_bias = 0.2
    cfg.segment_resolution = 64
    path = sbf.rrt_connect_path(robot, obstacles, list(start), list(goal), cfg, seed)
    return [list(map(float, wp)) for wp in path]


def densify(path: list[list[float]], step: float) -> list[np.ndarray]:
    """Resample a polyline at ~`step` spacing (the continuous bridge geometry)."""
    if len(path) < 2:
        return [np.array(p, dtype=float) for p in path]
    out: list[np.ndarray] = []
    for a, b in zip(path[:-1], path[1:]):
        a = np.array(a, dtype=float)
        b = np.array(b, dtype=float)
        seg = b - a
        length = float(np.linalg.norm(seg))
        n = max(1, int(math.ceil(length / step)))
        for i in range(n):
            out.append(a + seg * (i / n))
    out.append(np.array(path[-1], dtype=float))
    return out


# --------------------------------------------------------------------------- #
# Coverage probe via debug_find_free_box (the exact descent chain_pave uses)
# --------------------------------------------------------------------------- #
def probe_point(forest, obstacles, q: np.ndarray, max_depth: int) -> dict[str, Any]:
    opts = sbf.FindFreeBoxOptions()
    opts.max_depth = int(max_depth)
    opts.split_unknown_leaf = True
    opts.split_reserved_leaf = True
    opts.reject_seed_collision = False
    res = forest.debug_find_free_box(list(map(float, q)), obstacles, opts, True)
    return {
        "found": bool(res["found"]),
        "fail_code": int(res["fail_code"]),
        "hit_unknown_depth_cap": bool(res.get("hit_unknown_depth_cap", False)),
        "hit_reserved_depth_cap": bool(res.get("hit_reserved_depth_cap", False)),
        "seed_collision": bool(res.get("seed_collision", False)),
        "node": int(res.get("node", -1)),
        "depth": int(res.get("decisions", 0)),
        "intervals": [[float(lo), float(hi)] for lo, hi in res.get("intervals", [])],
    }


def boxes_face_adjacent(iv_a: list[list[float]], iv_b: list[list[float]], tol: float) -> bool:
    """Mirror rbf::boxes_connected: need >=1 touching dim, no separated dim."""
    if not iv_a or not iv_b or len(iv_a) != len(iv_b):
        return False
    shared = 0
    overlap = 0
    nd = len(iv_a)
    for (alo, ahi), (blo, bhi) in zip(iv_a, iv_b):
        lo = max(alo, blo)
        hi = min(ahi, bhi)
        if hi < lo - tol:
            return False
        if hi - lo < tol:
            shared += 1
        else:
            overlap += 1
    return shared >= 1 or overlap == nd


# --------------------------------------------------------------------------- #
# Phases
# --------------------------------------------------------------------------- #
def run_scene(args, gap: float, seed: int) -> dict[str, Any]:
    robot = make_planar_robot()
    obstacles = make_passage_obstacles(gap)
    free_fn = make_collision_fn(robot, obstacles)

    # start/goal: reach into the gate from opposite sides so a bridge must thread
    # the narrow corridor. Validated free, else nudge.
    start = np.array([0.55, -0.35], dtype=float)
    goal = np.array([0.05, 0.55], dtype=float)
    if not (free_fn(start) and free_fn(goal)):
        # fall back to a guaranteed-free pair away from the gate
        start = np.array([PI - 0.3, 0.2], dtype=float)
        goal = np.array([-PI + 0.3, -0.2], dtype=float)

    path = birrt_bridge(robot, obstacles, start, goal, seed)
    if len(path) < 2:
        return {"gap": gap, "seed": seed, "bridge_found": False}

    samples = densify(path, args.sample_step)
    clears = [clearance(free_fn, q) for q in samples]

    forest = sbf.SafeBoxForest(robot, build_config())

    depths = list(range(args.min_depth, args.max_depth + 1, args.depth_step))
    coverage_by_depth: dict[int, dict[str, Any]] = {}
    # cache the deepest-probe per-sample result for clearance + adjacency analysis
    deepest_results: list[dict[str, Any]] = []
    for d in depths:
        covered = 0
        unknown_cap = 0
        occupied = 0
        seed_coll = 0
        per_sample = []
        for q in samples:
            r = probe_point(forest, obstacles, q, d)
            per_sample.append(r)
            if r["found"]:
                covered += 1
            elif r["seed_collision"]:
                seed_coll += 1
            elif r["hit_unknown_depth_cap"]:
                unknown_cap += 1
            elif r["fail_code"] == 3:
                occupied += 1
        coverage_by_depth[d] = {
            "covered": covered,
            "total": len(samples),
            "fraction": covered / len(samples),
            "fail_unknown_depth_cap": unknown_cap,
            "fail_occupied": occupied,
            "seed_collision": seed_coll,
        }
        if d == depths[-1]:
            deepest_results = per_sample

    # Phase D: clearance separation (covered vs uncovered at deepest probe)
    cov_clear = [c for c, r in zip(clears, deepest_results) if r["found"]]
    unc_clear = [c for c, r in zip(clears, deepest_results) if not r["found"]]

    # Phase E: chain face-adjacency continuity at the deepest probe.
    # Walk the bridge; between consecutive *covered* samples, check whether their
    # certified boxes are face-adjacent (what chain_pave requires to commit).
    adjacency_breaks = {}
    for tol in args.adjacency_tols:
        breaks = 0
        pairs = 0
        prev = None
        for r in deepest_results:
            if not r["found"]:
                prev = None
                continue
            if prev is not None:
                pairs += 1
                if not boxes_face_adjacent(prev["intervals"], r["intervals"], tol):
                    breaks += 1
            prev = r
        adjacency_breaks[f"{tol:.0e}"] = {"pairs": pairs, "breaks": breaks}

    return {
        "gap": gap,
        "seed": seed,
        "bridge_found": True,
        "bridge_waypoints": len(path),
        "bridge_samples": len(samples),
        "start": start.tolist(),
        "goal": goal.tolist(),
        "coverage_by_depth": {str(k): v for k, v in coverage_by_depth.items()},
        "clearance": {
            "covered_mean": float(np.mean(cov_clear)) if cov_clear else None,
            "covered_min": float(np.min(cov_clear)) if cov_clear else None,
            "uncovered_mean": float(np.mean(unc_clear)) if unc_clear else None,
            "uncovered_max": float(np.max(unc_clear)) if unc_clear else None,
            "n_covered": len(cov_clear),
            "n_uncovered": len(unc_clear),
        },
        "adjacency_breaks": adjacency_breaks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/logs/chain_pave_coverage"))
    parser.add_argument("--gaps", type=float, nargs="+", default=[0.06, 0.10, 0.18, 0.30])
    parser.add_argument("--seeds", type=int, nargs="+", default=[1, 7, 13])
    parser.add_argument("--sample-step", type=float, default=0.02)
    parser.add_argument("--min-depth", type=int, default=8)
    parser.add_argument("--max-depth", type=int, default=44)
    parser.add_argument("--depth-step", type=int, default=4)
    parser.add_argument("--adjacency-tols", type=float, nargs="+", default=[1e-9, 1e-6, 1e-3])
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for gap in args.gaps:
        for seed in args.seeds:
            print(f"[run] gap={gap:.3f} seed={seed} ...", flush=True)
            res = run_scene(args, gap, seed)
            results.append(res)
            if res.get("bridge_found"):
                deep = max(int(k) for k in res["coverage_by_depth"])
                cov = res["coverage_by_depth"][str(deep)]
                print(
                    f"    bridge_samples={res['bridge_samples']} "
                    f"coverage@d{deep}={cov['fraction']:.3f} "
                    f"unknown_cap={cov['fail_unknown_depth_cap']} "
                    f"adj_breaks(1e-9)={res['adjacency_breaks']['1e-09']['breaks']}"
                    f"/{res['adjacency_breaks']['1e-09']['pairs']}",
                    flush=True,
                )
            else:
                print("    bridge NOT found (skipped)", flush=True)

    out_json = args.out_dir / "coverage_results.json"
    out_json.write_text(json.dumps({"args": {
        "gaps": args.gaps, "seeds": args.seeds, "sample_step": args.sample_step,
        "min_depth": args.min_depth, "max_depth": args.max_depth,
        "depth_step": args.depth_step, "adjacency_tols": args.adjacency_tols,
    }, "results": results}, indent=2))
    print(f"\nWrote {out_json}")

    # ---- aggregate summary across seeds, per gap ----
    print("\n=== summary: deepest-probe coverage vs passage gap ===")
    by_gap: dict[float, list[dict[str, Any]]] = {}
    for r in results:
        if r.get("bridge_found"):
            by_gap.setdefault(r["gap"], []).append(r)
    for gap in sorted(by_gap):
        rs = by_gap[gap]
        deep = max(int(k) for k in rs[0]["coverage_by_depth"])
        fr = np.mean([r["coverage_by_depth"][str(deep)]["fraction"] for r in rs])
        ucap = np.mean([r["coverage_by_depth"][str(deep)]["fail_unknown_depth_cap"] for r in rs])
        brk = np.mean([r["adjacency_breaks"]["1e-09"]["breaks"] for r in rs])
        prs = np.mean([r["adjacency_breaks"]["1e-09"]["pairs"] for r in rs])
        print(
            f"  gap={gap:5.3f}  coverage@d{deep}={fr:6.3f}  "
            f"unknown_depth_cap(mean)={ucap:6.1f}  "
            f"chain_face_adj_breaks(1e-9)={brk:.1f}/{prs:.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
