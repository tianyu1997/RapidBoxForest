#!/usr/bin/env python3
"""Build the shelf IIWA d23 warm LECT cache used by Exp04 baseline."""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import write_json  # noqa: E402
from experiments.common.lect_db_dispatch import ensure_shelf_cache  # noqa: E402
from experiments.common.shelf_iiwa_cache import publish_shelf_cache_snapshot  # noqa: E402
from experiments.exp04_shelf_ablation.run_shelf_ablation import (  # noqa: E402
    DEFAULT_BASELINE_WARM_PREWARM_DEPTH,
    FIXED_SHELF_ROOT_INTERVALS,
    warm_cache_paths,
    warm_cache_ready,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prewarm the Exp04 baseline d23 shelf cache.")
    parser.add_argument("--prewarm-depth", type=int, default=DEFAULT_BASELINE_WARM_PREWARM_DEPTH)
    parser.add_argument("--prewarm-max-depth", type=int, default=64)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--snapshot-only",
        action="store_true",
        help="Skip prewarm/verify; only publish lect_snapshot from an existing on-disk cache.",
    )
    parser.add_argument("--lect-root-intervals", default=FIXED_SHELF_ROOT_INTERVALS)
    return parser.parse_args()


def apply_prewarm_env_defaults() -> None:
    os.environ.setdefault("SBF_PREWARM_VERIFY", "0")
    os.environ.setdefault("SBF_PREWARM_VERIFY_STRICT", "0")
    os.environ.setdefault("SBF_PREWARM_SNAPSHOT", "1")
    os.environ.setdefault("SBF_PREWARM_PROGRESS", "1")
    os.environ.pop("SBF_PREWARM_STREAMING", None)
    os.environ.pop("SBF_DISABLE_BULK_PREWARM", None)


def main() -> int:
    args = parse_args()
    apply_prewarm_env_defaults()
    label, cache_root, prewarm_json = warm_cache_paths(int(args.prewarm_depth))
    cache_path = cache_root / label
    prewarm_json.parent.mkdir(parents=True, exist_ok=True)

    if bool(args.snapshot_only):
        if not cache_path.is_dir():
            raise SystemExit(f"cache does not exist: {cache_path}")
        summary = publish_shelf_cache_snapshot(
            cache_path=cache_path,
            prewarm_depth=int(args.prewarm_depth),
            envelope="support_hull",
            prewarm_threads=int(args.prewarm_threads),
            max_depth=int(args.prewarm_max_depth),
            endpoint_source="aafk",
            lect_split_policy="aafk_volume_min",
            lect_root_intervals=str(args.lect_root_intervals),
            canonical_mode=True,
        )
        write_json(prewarm_json, summary)
    else:
        summary = ensure_shelf_cache(
            prewarm_json=prewarm_json,
            cache_path=cache_path,
            prewarm_depth=int(args.prewarm_depth),
            envelope="support_hull",
            prewarm_threads=int(args.prewarm_threads),
            max_depth=int(args.prewarm_max_depth),
            endpoint_source="aafk",
            lect_split_policy="aafk_volume_min",
            lect_root_intervals=str(args.lect_root_intervals),
            canonical_mode=True,
            clean_cache=bool(args.clean_cache),
            dry_run=False,
        )
    ready, reason = warm_cache_ready(cache_path)
    print(f"prewarm ok={summary.get('ok')} cache={cache_path} bytes={summary.get('cache_bytes')}")
    print(f"snapshot wait_ok={summary.get('snapshot', {}).get('wait_ok')}")
    if not ready:
        raise SystemExit(reason)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
