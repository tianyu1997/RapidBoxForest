#!/usr/bin/env python3
"""Why do no_cache and AABB show NO optimization vs baseline at the fast stage?

Dumps build sub-timings, box counts, external-evidence reuse counters, frontier
cache hit/insert stats and split-dim distribution for baseline / no_cache / aabb
at a chosen stage (default 'fast'), so we can see whether the cache and envelope
features are even active / differentiating.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from analyze_ablation_timings import GROUP_SPECS, load_json, summarize_artifact  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_E4 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_seeds5_20260528"

GROUPS = ["baseline", "no_cache", "aabb", "critsample"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--e4-root", type=Path, default=DEFAULT_E4)
    ap.add_argument("--depth48-root", type=Path, default=DEFAULT_E4)
    ap.add_argument("--stage", default="fast")
    args = ap.parse_args()
    stage = str(args.stage)

    specs = dict(GROUP_SPECS)

    arts = {}
    for name in GROUPS:
        spec = specs[name]
        root = args.e4_root if str(spec["root"]) == "e4" else args.depth48_root
        path = root / str(spec["path_name"])
        if not path.exists():
            continue
        arts[name] = summarize_artifact(name, path, load_json(path))

    print(f"stage = {stage!r}\n")

    def row(label, vals, fmt="{:>14.3f}"):
        print(f"{label:<32}" + "".join(fmt.format(v) if isinstance(v, float) else f"{v:>14}" for v in vals))

    names = list(arts)
    print(f"{'metric':<32}" + "".join(f"{arts[n]['label']:>14}" for n in names))
    print("-" * (32 + 14 * len(names)))

    def get(n, *path):
        s = arts[n]["stages"][stage]
        for p in path:
            s = s[p]
        return s

    row("cumulative_total_s", [float(get(n, "cumulative_total_s")) for n in names])
    row("cumulative_build_s", [float(get(n, "cumulative_build_s")) for n in names])
    row("cumulative_query_s", [float(get(n, "cumulative_query_s")) for n in names])
    row("incumbent_total_length", [float(get(n, "incumbent_total_length")) for n in names])
    print()
    row("build.unique_box_count", [int(get(n, "build", "unique_box_count")) for n in names])
    row("build.certified_box_count", [int(get(n, "build", "certified_box_count")) for n in names])
    row("build.grow_ms", [float(get(n, "build", "grow_ms")) for n in names])
    row("build.connector_ms", [float(get(n, "build", "connector_ms")) for n in names])
    row("build.planning_s", [float(get(n, "build", "planning_s")) for n in names])
    row("build.maintenance_s", [float(get(n, "build", "maintenance_s")) for n in names])
    print()
    row("ext.worker_mat_reused", [float(get(n, "external_evidence", "worker_materialization_reused")) for n in names])
    row("ext.worker_scoring_reused", [float(get(n, "external_evidence", "worker_scoring_reused")) for n in names])
    row("ext.oracle_mat_reused", [float(get(n, "external_evidence", "oracle_materialization_reused")) for n in names])
    row("ext.oracle_scoring_reused", [float(get(n, "external_evidence", "oracle_scoring_reused")) for n in names])
    print()
    row("frontier.covered_cache_hits", [float(get(n, "frontier", "covered_cache_hits")) for n in names])
    row("frontier.covered_cache_inserts", [float(get(n, "frontier", "covered_cache_inserts")) for n in names])
    row("frontier.uncovered_cache_hits", [float(get(n, "frontier", "uncovered_cache_hits")) for n in names])
    row("frontier.uncovered_cache_inserts", [float(get(n, "frontier", "uncovered_cache_inserts")) for n in names])
    print()
    for n in names:
        sc = get(n, "split", "dim_counts")
        print(f"{arts[n]['label']:<20} split dim_counts: {dict(sorted(sc.items()))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
