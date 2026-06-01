#!/usr/bin/env python3
"""Controlled A/B for the reflected-Gray prewarm leaf ordering.

Runs the lifelong-cache prewarm twice at the same depth -- once with the
node-id (left-to-right) leaf order and once with the reflected-Gray order --
in two separate caches, and reports for each:
  * incremental-FK reuse rate (source_incremental_state / materializations)
  * prewarm wall time
  * persisted cache size and record counts

Because the incremental endpoint result is provably identical to a full pass,
both orders must produce the same materialization count, parent-hull count,
evidence count and cache size; only the wall time and reuse rate differ.

    PYTHONPATH=$PWD/build-rbf-only-exec/python:$PWD \
        .venv/bin/python3 experiments/exp08_envelope_volume_curve/prewarm_gray_ab.py --depth 14
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

ROOT = ("0.0:1.5707963267948966;0.3194:0.8645;-0.5077:0.5073;"
        "-1.98947519:-0.33002121;-0.447:0.4473;-1.34734773:1.51007653;1.262:1.8794")


def run_one(sc, sbf, cache_path: Path, depth: int, max_depth: int, gray: bool) -> dict:
    if cache_path.exists():
        shutil.rmtree(cache_path)
    prewarm_args = sc.make_prewarm_config_args(
        cache_path, depth, "support_hull", 8,
        max_depth, sc.ENDPOINT_AAFK, sc.LECT_SPLIT_AAFK_VOLUME_MIN, ROOT, True,
    )
    robot = sbf.load_iiwa14_robot()
    cfg = sc.configure_standalone_sbf(prewarm_args, seed=0,
                                      preset=sc.RBF_LIFELONG_PRESET, robot=robot)
    root_override = sc.parse_lect_root_intervals(ROOT)
    if root_override is not None:
        cfg.database.root_intervals_override = list(root_override)
    sc.configure_cache_endpoint(cfg, sc.ENDPOINT_AAFK)
    sc.configure_cache_split_policy(cfg, robot, sc.LECT_SPLIT_AAFK_VOLUME_MIN,
                                    int(max_depth), root_intervals=root_override)
    sc.set_online_cache_backfill(cfg, True)
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    res = dict(forest.prewarm_lifelong_cache(int(depth), [sc.far_obstacle()],
                                             gray_leaf_order=bool(gray)))
    res["wall_s_outer"] = time.perf_counter() - t0
    res["verify_ok"] = bool(forest.database_verify(True))
    del forest
    res["cache_bytes"] = sc.directory_size(cache_path)
    return res


def main() -> None:
    import experiments.common.shelf_iiwa_cache as sc  # noqa: PLC0415
    import sbf  # noqa: PLC0415

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--depth", type=int, default=14)
    ap.add_argument("--max-depth", type=int, default=40)
    ap.add_argument("--cache-root", type=Path,
                    default=REPO_ROOT / "outputs" / "logs" / "prewarm_gray_ab")
    args = ap.parse_args()
    args.cache_root.mkdir(parents=True, exist_ok=True)

    out = {}
    for label, gray in (("node_id", False), ("gray", True)):
        res = run_one(sc, sbf, args.cache_root / f"p{args.depth}_{label}",
                      args.depth, args.max_depth, gray)
        mat = int(res.get("materializations", 0)) or 1
        out[label] = {
            "gray_leaf_order": res.get("gray_leaf_order"),
            "verify_ok": res.get("verify_ok"),
            "wall_s_prewarm": round(float(res.get("wall_s_outer", 0.0)), 3),
            "materializations": int(res.get("materializations", 0)),
            "source_incremental_state": int(res.get("source_incremental_state", 0)),
            "incremental_hit_rate": round(int(res.get("source_incremental_state", 0)) / mat, 4),
            "parent_hulls_built": int(res.get("parent_hulls_built", 0)),
            "evidence_after": int(res.get("evidence_after", 0)),
            "cache_bytes": int(res.get("cache_bytes", 0)),
        }

    n, g = out["node_id"], out["gray"]
    out["outputs_identical"] = (
        n["materializations"] == g["materializations"]
        and n["parent_hulls_built"] == g["parent_hulls_built"]
        and n["evidence_after"] == g["evidence_after"]
        and n["cache_bytes"] == g["cache_bytes"]
    )
    if g["wall_s_prewarm"] > 0:
        out["speedup_x"] = round(n["wall_s_prewarm"] / g["wall_s_prewarm"], 3)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
