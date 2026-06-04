"""Compare Plan C (build-driven lazy-deepening prewarm) against the blanket layer-N prewarm.

The existing ``prewarm_lifelong_cache(depth)`` materialises the *entire* uniform
``2**depth`` layer (e.g. 262144 nodes at depth 18) regardless of whether the
adaptive build ever descends there, so the warmed boxes sit at the wrong depth
and the held-out build keeps materialising the deep (depth 19-40) boxes it
actually visits.  Plan C instead warms only the subtrees a representative
``build_coverage`` descends into (``run_build_driven_prewarm``).

This script warms both caches, then runs an identical held-out ``build_coverage``
with each cache attached as external evidence (snapshot, backfill off) and reports
the external-evidence hit rate and grow time, so we can quantify how much the
build-driven prewarm raises the cache hit rate over the blanket layer prewarm.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.shelf_iiwa_cache import (  # noqa: E402
    configure_cache_endpoint,
    configure_cache_split_policy,
    make_prewarm_config_args,
    run_build_driven_prewarm,
    run_p18_prewarm,
)
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    RBF_LIFELONG_PRESET,
    configure_external_evidence_reuse,
    configure_standalone_sbf,
    set_online_cache_backfill,
)

import sbf  # noqa: E402

ENDPOINT_AAFK = "AAFK"
LECT_SPLIT_AAFK_VOLUME_MIN = "aafk_volume_min"
WORKER_PREFIX = "grower.worker_oracle."


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--max-depth", type=int, default=40)
    parser.add_argument("--blanket-depth", type=int, default=18)
    parser.add_argument(
        "--cache-root",
        type=Path,
        default=REPO_ROOT / "outputs" / "new_experiments" / "exp04_prewarm_hit_rate" / "cache",
    )
    parser.add_argument(
        "--out-json",
        type=Path,
        default=REPO_ROOT / "outputs" / "new_experiments" / "exp04_prewarm_hit_rate" / "comparison.json",
    )
    parser.add_argument("--clean", action="store_true", help="rebuild both warm caches from scratch")
    return parser.parse_args()


def measure_held_out_build(
    warm_path: Path,
    threads: int,
    max_depth: int,
) -> dict[str, float]:
    """Run a fresh build_coverage with ``warm_path`` attached as external evidence."""
    robot = sbf.load_iiwa14_robot()
    measure_args = make_prewarm_config_args(
        warm_path / "_held_out_active",
        prewarm_depth=max_depth,
        envelope="link",
        prewarm_threads=threads,
        max_depth=max_depth,
        endpoint_source=ENDPOINT_AAFK,
        lect_split_policy=LECT_SPLIT_AAFK_VOLUME_MIN,
        canonical_mode=True,
    )
    cfg = configure_standalone_sbf(measure_args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    configure_cache_endpoint(cfg, ENDPOINT_AAFK)
    configure_cache_split_policy(cfg, robot, LECT_SPLIT_AAFK_VOLUME_MIN, max_depth)
    # Attach the warm cache as read-only external evidence; no local backfill so the
    # only cross-build reuse path is the prewarmed snapshot under test.
    configure_external_evidence_reuse(cfg, warm_path, measure_args, backfill_active=False)
    set_online_cache_backfill(cfg, False)
    cfg.database.create_if_missing = True

    obstacles = sbf.make_combined_obstacles()
    coverage_seeds = [list(seed) for seed in sbf.make_coverage_seeds(include_extra_anchors=False)]

    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, coverage_seeds)
    wall_s = time.perf_counter() - t0
    diag = {str(k): float(v) for k, v in dict(profile.diagnostics).items()}
    del forest

    mat = diag.get(WORKER_PREFIX + "materializations", 0.0)
    reused_external = diag.get(WORKER_PREFIX + "materialization_reused_external_evidence", 0.0)
    reused_shared = diag.get(WORKER_PREFIX + "materialization_reused_shared_endpoint_cache", 0.0)
    lookups = mat + reused_external
    return {
        "grow_ms": float(getattr(profile, "grow_ms", 0.0)),
        "wall_s": wall_s,
        "materializations": mat,
        "reused_external": reused_external,
        "reused_shared": reused_shared,
        "endpoint_lookups": lookups,
        "external_hit_rate": (reused_external / lookups) if lookups > 0 else 0.0,
    }


def main() -> int:
    args = parse_args()
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    blanket_cache = args.cache_root / "blanket_p18_canonical"
    plan_c_cache = args.cache_root / "build_driven_canonical"

    print("[prewarm-cmp] warming blanket layer cache ...", flush=True)
    blanket = run_p18_prewarm(
        blanket_cache,
        prewarm_depth=args.blanket_depth,
        envelope="link",
        prewarm_threads=args.threads,
        max_depth=args.max_depth,
        clean_cache=args.clean,
        dry_run=False,
        endpoint_source=ENDPOINT_AAFK,
        lect_split_policy=LECT_SPLIT_AAFK_VOLUME_MIN,
        canonical_mode=True,
    )
    print(
        f"[prewarm-cmp] blanket ok={blanket['ok']} bytes={blanket['cache_bytes']} "
        f"wall_s={blanket['prewarm'].get('wall_s_outer', 0.0):.1f}",
        flush=True,
    )

    print("[prewarm-cmp] warming build-driven cache (Plan C) ...", flush=True)
    plan_c = run_build_driven_prewarm(
        plan_c_cache,
        envelope="link",
        prewarm_threads=args.threads,
        max_depth=args.max_depth,
        clean_cache=args.clean,
        endpoint_source=ENDPOINT_AAFK,
        lect_split_policy=LECT_SPLIT_AAFK_VOLUME_MIN,
        canonical_mode=True,
    )
    print(
        f"[prewarm-cmp] plan_c ok={plan_c['ok']} bytes={plan_c['cache_bytes']} "
        f"wall_s={plan_c['prewarm'].get('wall_s_outer', 0.0):.1f} "
        f"mat={plan_c['prewarm'].get('materializations', 0.0):.0f}",
        flush=True,
    )

    print("[prewarm-cmp] measuring held-out build with blanket cache ...", flush=True)
    blanket_measure = measure_held_out_build(blanket_cache, args.threads, args.max_depth)
    print(
        f"[prewarm-cmp] blanket  grow_ms={blanket_measure['grow_ms']:.1f} "
        f"mat={blanket_measure['materializations']:.0f} "
        f"reused_external={blanket_measure['reused_external']:.0f} "
        f"hit_rate={blanket_measure['external_hit_rate']*100:.1f}%",
        flush=True,
    )

    print("[prewarm-cmp] measuring held-out build with build-driven cache ...", flush=True)
    plan_c_measure = measure_held_out_build(plan_c_cache, args.threads, args.max_depth)
    print(
        f"[prewarm-cmp] plan_c   grow_ms={plan_c_measure['grow_ms']:.1f} "
        f"mat={plan_c_measure['materializations']:.0f} "
        f"reused_external={plan_c_measure['reused_external']:.0f} "
        f"hit_rate={plan_c_measure['external_hit_rate']*100:.1f}%",
        flush=True,
    )

    payload = {
        "params": {
            "threads": args.threads,
            "max_depth": args.max_depth,
            "blanket_depth": args.blanket_depth,
            "endpoint_source": ENDPOINT_AAFK,
            "lect_split_policy": LECT_SPLIT_AAFK_VOLUME_MIN,
        },
        "blanket_prewarm": {k: v for k, v in blanket.items() if k != "metadata"},
        "plan_c_prewarm": {k: v for k, v in plan_c.items() if k != "metadata"},
        "held_out_blanket": blanket_measure,
        "held_out_plan_c": plan_c_measure,
    }
    args.out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"[prewarm-cmp] wrote {args.out_json}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
