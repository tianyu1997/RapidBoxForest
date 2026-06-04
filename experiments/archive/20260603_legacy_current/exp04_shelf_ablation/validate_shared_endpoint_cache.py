"""Validate Plan A (process-level cross-query shared endpoint cache).

The shared endpoint cache (lect_database::SharedEndpointEvidenceCache) is owned
by the persistent forest and is preserved across ``reset_oracle`` while the
local online cache is cleared.  ``build_coverage`` calls ``reset_oracle`` at the
start, so rebuilding the *same* coverage on one forest cleanly isolates the
shared cache: pass #2's workers miss the (cleared) online cache yet hit the
(preserved) shared cache for every canonical box pass #1 materialized.

This script measures, per pass:
  * grower wall time (grow_ms)
  * worker-oracle materializations / reused_shared / stored_shared
  * shared-cache size / bytes / evictions

A working Plan A shows pass #2 with reused_shared ~= pass #1 materializations and
a markedly lower grow_ms, with cache evictions = 0 (within the OOM budget).
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
)
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    RBF_LIFELONG_PRESET,
    configure_standalone_sbf,
    set_online_cache_backfill,
)

import sbf  # noqa: E402

ENDPOINT_AAFK = "AAFK"
LECT_SPLIT_AAFK_VOLUME_MIN = "aafk_volume_min"

WORKER_PREFIX = "grower.worker_oracle."
COUNTER_KEYS = [
    "materializations",
    "materialization_reused_endpoint_cache",
    "materialization_reused_external_evidence",
    "materialization_reused_shared_endpoint_cache",
    "materialization_stored_shared_endpoint_cache",
    "materialization_endpoint_wall_time_us",
]
SHARED_KEYS = [
    "oracle.shared_endpoint_cache_size",
    "oracle.shared_endpoint_cache_bytes",
    "oracle.shared_endpoint_cache_evictions",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--max-depth", type=int, default=40)
    parser.add_argument("--passes", type=int, default=3)
    parser.add_argument(
        "--cache-root",
        type=Path,
        default=REPO_ROOT / "outputs" / "new_experiments" / "exp04_shared_cache_validate" / "cache",
    )
    parser.add_argument(
        "--out-json",
        type=Path,
        default=REPO_ROOT / "outputs" / "new_experiments" / "exp04_shared_cache_validate" / "plan_a_validation.json",
    )
    return parser.parse_args()


def collect(profile: Any) -> dict[str, float]:
    diag = {str(k): float(v) for k, v in dict(profile.diagnostics).items()}
    row: dict[str, float] = {
        "grow_ms": float(getattr(profile, "grow_ms", diag.get("forest.build_coverage", 0.0))),
        "total_ms": float(profile.total_ms),
    }
    for key in COUNTER_KEYS:
        row[key] = diag.get(WORKER_PREFIX + key, 0.0)
    for key in SHARED_KEYS:
        row[key] = diag.get(key, 0.0)
    return row


def main() -> int:
    args = parse_args()
    cache_path = args.cache_root / "shared_cache_validate_p18_canonical"
    args.out_json.parent.mkdir(parents=True, exist_ok=True)

    prewarm_args = make_prewarm_config_args(
        cache_path,
        prewarm_depth=18,
        envelope="link",
        prewarm_threads=args.threads,
        max_depth=args.max_depth,
        endpoint_source=ENDPOINT_AAFK,
        lect_split_policy=LECT_SPLIT_AAFK_VOLUME_MIN,
        canonical_mode=True,
    )
    robot = sbf.load_iiwa14_robot()
    cfg = configure_standalone_sbf(prewarm_args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    configure_cache_endpoint(cfg, ENDPOINT_AAFK)
    configure_cache_split_policy(cfg, robot, LECT_SPLIT_AAFK_VOLUME_MIN, args.max_depth)
    # Cold start: no external warm snapshot, no database backfill, so the shared
    # endpoint cache is the only cross-pass reuse path under test.
    set_online_cache_backfill(cfg, False)
    cfg.database.create_if_missing = True

    obstacles = sbf.make_combined_obstacles()
    coverage_seeds = [list(seed) for seed in sbf.make_coverage_seeds(include_extra_anchors=False)]

    forest = sbf.SafeBoxForest(robot, cfg)
    rows: list[dict[str, Any]] = []
    for pass_index in range(args.passes):
        t0 = time.perf_counter()
        profile = forest.build_coverage(obstacles, coverage_seeds)
        wall_s = time.perf_counter() - t0
        row = collect(profile)
        row["pass"] = pass_index
        row["wall_s"] = wall_s
        rows.append(row)
        print(
            f"[plan-a] pass={pass_index} grow_ms={row['grow_ms']:.1f} "
            f"mat={row['materializations']:.0f} "
            f"reused_shared={row['materialization_reused_shared_endpoint_cache']:.0f} "
            f"stored_shared={row['materialization_stored_shared_endpoint_cache']:.0f} "
            f"reused_local={row['materialization_reused_endpoint_cache']:.0f} "
            f"cache_size={row['oracle.shared_endpoint_cache_size']:.0f} "
            f"evict={row['oracle.shared_endpoint_cache_evictions']:.0f}",
            flush=True,
        )
    del forest

    payload = {
        "params": {
            "threads": args.threads,
            "max_depth": args.max_depth,
            "passes": args.passes,
            "endpoint_source": ENDPOINT_AAFK,
            "lect_split_policy": LECT_SPLIT_AAFK_VOLUME_MIN,
        },
        "rows": rows,
    }
    args.out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"[plan-a] wrote {args.out_json}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
