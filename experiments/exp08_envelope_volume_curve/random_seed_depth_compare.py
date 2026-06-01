#!/usr/bin/env python3
"""Compare FFB depth-to-first-free-box between the AAFK and support-hull split
schedules across many random seeds drawn uniformly from the restricted ROOT box.

Each (policy, seed) pair runs on its own fresh, empty LECTDatabase (no external
evidence) so the measured depth is the pure "root -> first free box" depth for
that seed under that split schedule. Both policies see the SAME random seeds.

Run (sbf-only interpreter):
    PYTHONPATH=$PWD/build-rbf-only-exec/python:$PWD \
        .venv/bin/python3 experiments/exp08_envelope_volume_curve/random_seed_depth_compare.py \
        --seeds 120 --max-depth 40
"""
from __future__ import annotations

import argparse
import json
import random
import statistics
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

ROOT = ("0.0:1.5707963267948966;0.3194:0.8645;-0.5077:0.5073;"
        "-1.98947519:-0.33002121;-0.447:0.4473;-1.34734773:1.51007653;1.262:1.8794")


def parse_root(root: str) -> list[tuple[float, float]]:
    out = []
    for part in root.split(";"):
        lo, hi = part.split(":")
        out.append((float(lo), float(hi)))
    return out


def build_forest(sbf, case, policy: str, db_path: str):
    sys.argv = [
        "prog", "--case-name", "depth_probe", "--out-json", "/dev/null",
        "--database-path", db_path,
        "--endpoint-source", "aafk", "--lect-split-policy", policy,
        "--preset", "support_hull_coverage", "--rbf-envelope", "support_hull",
        "--rbf-max-depth", "40", "--ffb-depth", "40", "--connector-pave-depth", "40",
        "--component-connect-ffb-max-depth", "40", "--rbf-ffb-start-depth", "8",
        "--lect-root-intervals", ROOT,
        "--no-use-external-evidence", "--no-external-evidence-materialization",
        "--no-external-evidence-scoring",
        "--rbf-canonical-cache", "--seeds", "1",
    ]
    args = case.parse_args()
    robot = sbf.load_iiwa14_robot()
    cfg = case.case_config(args, robot, seed=20260504)
    return sbf.SafeBoxForest(robot, cfg), robot


def ffb_depth(forest, sbf, seed_values, obstacles, max_depth: int):
    opt = sbf.FindFreeBoxOptions()
    opt.max_depth = max_depth
    opt.split_reserved_leaf = True
    opt.split_unknown_leaf = True
    opt.reject_seed_collision = False
    res = forest.debug_find_free_box(list(seed_values), obstacles, opt, False)
    if isinstance(res, dict):
        trace = res.get("trace") or []
        found = bool(res.get("found", res.get("ok", False)))
        depth = trace[-1].get("depth") if trace else res.get("depth")
    else:
        trace = getattr(res, "trace", []) or []
        found = bool(getattr(res, "found", False))
        depth = trace[-1]["depth"] if trace else getattr(res, "depth", None)
    return found, depth


def main() -> None:
    import sbf  # noqa: PLC0415
    from experiments.common import run_shelf_sbf_case as case  # noqa: PLC0415

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, default=120)
    ap.add_argument("--max-depth", type=int, default=40)
    ap.add_argument("--rng-seed", type=int, default=20260601)
    ap.add_argument("--out-dir", type=Path,
                    default=REPO_ROOT / "outputs" / "logs" / "split_policy_depth_seeds")
    args = ap.parse_args()

    bounds = parse_root(ROOT)
    rng = random.Random(args.rng_seed)
    seeds = [[rng.uniform(lo, hi) for lo, hi in bounds] for _ in range(args.seeds)]

    obstacles = sbf.make_combined_obstacles()

    records = []
    tmp = Path(tempfile.mkdtemp(prefix="rbf_depth_probe_"))
    try:
        for policy in ("aafk_volume_min", "support_hull_volume_min"):
            for i, sv in enumerate(seeds):
                db = tmp / f"{policy}_{i}"
                forest, _ = build_forest(sbf, case, policy, str(db))
                found, depth = ffb_depth(forest, sbf, sv, obstacles, args.max_depth)
                records.append({"policy": policy, "seed_idx": i,
                                "found": found, "depth": depth})
                del forest
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)

    # Pair up per seed where both policies found a free box.
    by_idx = {}
    for r in records:
        by_idx.setdefault(r["seed_idx"], {})[r["policy"]] = r

    aafk_d, hull_d, deltas = [], [], []
    paired = 0
    hull_deeper = hull_same = hull_shallower = 0
    for idx in sorted(by_idx):
        a = by_idx[idx].get("aafk_volume_min")
        h = by_idx[idx].get("support_hull_volume_min")
        if not (a and h and a["found"] and h["found"]
                and a["depth"] is not None and h["depth"] is not None):
            continue
        paired += 1
        aafk_d.append(a["depth"])
        hull_d.append(h["depth"])
        d = h["depth"] - a["depth"]
        deltas.append(d)
        if d > 0:
            hull_deeper += 1
        elif d == 0:
            hull_same += 1
        else:
            hull_shallower += 1

    summary = {
        "seeds_requested": args.seeds,
        "paired_found": paired,
        "aafk_depth_mean": statistics.mean(aafk_d) if aafk_d else None,
        "hull_depth_mean": statistics.mean(hull_d) if hull_d else None,
        "aafk_depth_median": statistics.median(aafk_d) if aafk_d else None,
        "hull_depth_median": statistics.median(hull_d) if hull_d else None,
        "delta_mean": statistics.mean(deltas) if deltas else None,
        "hull_deeper": hull_deeper,
        "hull_same": hull_same,
        "hull_shallower": hull_shallower,
    }

    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "depth_seeds.json").write_text(
        json.dumps({"summary": summary, "records": records}, indent=2))

    print("== Random-seed FFB depth: AAFK vs support-hull split schedule ==")
    print(f"seeds requested      : {args.seeds}")
    print(f"paired (both found)  : {paired}")
    if paired:
        print(f"AAFK depth  mean/median : {summary['aafk_depth_mean']:.3f} / "
              f"{summary['aafk_depth_median']}")
        print(f"HULL depth  mean/median : {summary['hull_depth_mean']:.3f} / "
              f"{summary['hull_depth_median']}")
        print(f"mean depth delta (HULL-AAFK): {summary['delta_mean']:+.3f}")
        print(f"HULL deeper / same / shallower : "
              f"{hull_deeper} / {hull_same} / {hull_shallower}")
    print(f"\n[json] {args.out_dir / 'depth_seeds.json'}")


if __name__ == "__main__":
    main()
