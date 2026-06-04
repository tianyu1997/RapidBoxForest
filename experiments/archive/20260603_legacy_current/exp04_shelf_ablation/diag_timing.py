#!/usr/bin/env python3
"""Dump fine-grained materialization / envelope timing counters at a stage.

Goal: understand WHY external-evidence cache and envelope choice do not speed up
the fast stage. We look at where build time actually goes:
  - endpoint materialization (what the cache can skip)
  - envelope compute (what AABB vs support_hull changes)
  - envelope collision
  - external lookup overhead (cost the cache ADDS)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from analyze_ablation_timings import GROUP_SPECS, load_json, stage_rows  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_E4 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_seeds5_20260528"

GROUPS = ["baseline", "no_cache", "aabb", "critsample"]

KEYS = [
    "materializations",
    "materialization_reused_external_evidence",
    "materialization_reused_endpoint_cache",
    "materialization_reused_shared_endpoint_cache",
    "materialization_skipped_endpoint_cache",
    "materialization_stored_endpoint",
    "materialization_endpoint_wall_time_us",
    "materialization_endpoint_time_us",
    "materialization_envelope_compute_time_us",
    "materialization_envelope_collision_time_us",
    "materialization_envelope_time_us",
    "materialization_external_lookup_time_us",
    "materialization_external_read_time_us",
    "materialization_cache_lookup_time_us",
    "materialization_cache_read_time_us",
    "envelope_collision_queries",
    "certified_free",
    "collision_possible",
]


def find_key(diag: dict, suffix: str) -> float:
    # diagnostics keys are prefixed e.g. "oracle.<suffix>" or "grower.worker_oracle.<suffix>"
    total = 0.0
    found = False
    for k, v in diag.items():
        if k.endswith("." + suffix) or k == suffix:
            try:
                total += float(v)
                found = True
            except (TypeError, ValueError):
                pass
    return total if found else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--e4-root", type=Path, default=DEFAULT_E4)
    ap.add_argument("--stage", default="fast")
    ap.add_argument("--dump-keys", action="store_true", help="print all matching diag keys for baseline")
    ap.add_argument("--filter", default="material,envelope",
                    help="comma-separated substrings to match when dumping keys")
    args = ap.parse_args()
    stage = str(args.stage)
    filters = [f.strip() for f in str(args.filter).split(",") if f.strip()]

    diags = {}
    for name in GROUPS:
        spec = GROUP_SPECS[name]
        if str(spec.get("root")) != "e4":
            continue
        path = args.e4_root / str(spec["path_name"])
        if not path.exists():
            continue
        payload = load_json(path)
        row = stage_rows(payload)[stage]
        diags[name] = dict((row.get("build") or {}).get("diagnostics") or {})

    names = list(diags)

    if args.dump_keys and names:
        print(f"All diag keys matching {filters} (baseline):")
        for k in sorted(diags[names[0]]):
            if any(f in k for f in filters):
                print(f"  {k} = {diags[names[0]][k]}")
        print()

    print(f"stage = {stage!r}  (timings in us, cumulative over all queries)\n")
    print(f"{'counter':<46}" + "".join(f"{GROUP_SPECS[n]['label']:>15}" for n in names))
    print("-" * (46 + 15 * len(names)))
    for suffix in KEYS:
        vals = [find_key(diags[n], suffix) for n in names]
        print(f"{suffix:<46}" + "".join(f"{v:>15.1f}" for v in vals))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
