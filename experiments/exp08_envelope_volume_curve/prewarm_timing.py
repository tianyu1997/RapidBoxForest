#!/usr/bin/env python3
"""Measure prewarm wall-time, peak RSS and cache size for a single prewarm depth.

One prewarm per process so resource.getrusage peak RSS is attributable to this
depth only. Prints a JSON line on stdout.

    PYTHONPATH=$PWD/build-rbf-only-exec/python:$PWD \
        .venv/bin/python3 experiments/exp08_envelope_volume_curve/prewarm_timing.py --depth 18
"""
from __future__ import annotations

import argparse
import json
import resource
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

ROOT = ("0.0:1.5707963267948966;0.3194:0.8645;-0.5077:0.5073;"
        "-1.98947519:-0.33002121;-0.447:0.4473;-1.34734773:1.51007653;1.262:1.8794")


def main() -> None:
    from experiments.common import shelf_iiwa_cache as sc  # noqa: PLC0415

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--depth", type=int, required=True)
    ap.add_argument("--threads", type=int, default=8)
    ap.add_argument("--max-depth", type=int, default=40)
    ap.add_argument("--cache-root", type=Path,
                    default=REPO_ROOT / "outputs" / "logs" / "prewarm_timing" / "cache")
    args = ap.parse_args()

    cache_path = args.cache_root / f"prewarm_p{args.depth}"
    args.cache_root.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    res = sc.run_p18_prewarm(
        cache_path=cache_path,
        prewarm_depth=args.depth,
        envelope="support_hull",
        prewarm_threads=args.threads,
        max_depth=args.max_depth,
        clean_cache=True,
        dry_run=False,
        endpoint_source=sc.ENDPOINT_AAFK,
        lect_split_policy=sc.LECT_SPLIT_AAFK_VOLUME_MIN,
        lect_root_intervals=ROOT,
        canonical_mode=True,
    )
    wall_total = time.perf_counter() - t0
    peak_rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0

    prewarm = res.get("prewarm", {})
    out = {
        "depth": args.depth,
        "threads": args.threads,
        "ok": res.get("ok"),
        "wall_s_total": round(wall_total, 3),
        "wall_s_prewarm": round(float(prewarm.get("wall_s_outer", 0.0)), 3),
        "peak_rss_mb": round(peak_rss_mb, 1),
        "cache_bytes": res.get("cache_bytes"),
        "cache_mb": round((res.get("cache_bytes") or 0) / 1e6, 2),
        "prewarm_keys": sorted(prewarm.keys()),
        "prewarm_counts": {k: v for k, v in prewarm.items()
                           if isinstance(v, (int, float)) and "wall" not in k},
    }
    print(json.dumps(out))


if __name__ == "__main__":
    main()
