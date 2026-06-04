#!/usr/bin/env python3
"""Focused baseline-vs-others comparison for exp04 (native-fs run).

Reuses the artifact summarizer from analyze_ablation_timings.py but skips the
optional aabb_no_external probe (not generated in this run). Prints a compact
table of final metrics and per-group deltas relative to baseline so we can see
whether baseline dominates every ablation group.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from analyze_ablation_timings import (  # noqa: E402
    GROUP_SPECS,
    QUERY_ORDER,
    load_json,
    summarize_artifact,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_E4 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_full_ablation_20260528"
DEFAULT_D48 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_split_compare_depth48_20260528"

# Groups to compare against baseline (skip aabb_no_external probe).
COMPARE_ORDER = [
    "baseline",
    "no_cache",
    "aabb",
    "critsample",
    "single_thread",
    "baseline_d48",
    "round_robin_d48",
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--e4-root", type=Path, default=DEFAULT_E4)
    ap.add_argument("--depth48-root", type=Path, default=DEFAULT_D48)
    ap.add_argument("--stage", default="fast", choices=["seed", "fast", "balanced", "quality", "high"])
    args = ap.parse_args()
    stage = str(args.stage)

    arts: dict[str, dict] = {}
    for name in COMPARE_ORDER:
        spec = GROUP_SPECS[name]
        root = args.e4_root if str(spec["root"]) == "e4" else args.depth48_root
        path = root / str(spec["path_name"])
        arts[name] = summarize_artifact(name, path, load_json(path))

    def pick(name: str) -> dict:
        s = arts[name]["stages"][stage]
        return {
            "total_s": float(s["cumulative_total_s"]),
            "build_s": float(s["cumulative_build_s"]),
            "query_s": float(s["cumulative_query_s"]),
            "total_path_length": float(s["incumbent_total_length"]),
            "mean_path_length": float(s["incumbent_mean_length"]),
            "incumbent_queries": s["incumbent_queries"],
        }

    base = arts["baseline"]
    bf = pick("baseline")

    print("=" * 100)
    print(f"EXP04 baseline vs others  (stage = {stage!r})")
    print("=" * 100)
    header = f"{'group':<22}{'total_s':>10}{'build_s':>10}{'query_s':>10}{'path_len':>12}{'mean_len':>10}"
    print(header)
    print("-" * len(header))
    for name in COMPARE_ORDER:
        f = pick(name)
        print(f"{arts[name]['label']:<22}{f['total_s']:>10.3f}{f['build_s']:>10.3f}"
              f"{f['query_s']:>10.3f}{f['total_path_length']:>12.4f}{f['mean_path_length']:>10.4f}")

    print()
    print("=" * 100)
    print(f"Delta vs baseline ({stage})  (negative path_len = group BETTER/shorter; positive total_s = group SLOWER)")
    print("=" * 100)
    hdr2 = f"{'group':<22}{'d_total_s':>12}{'d_build_s':>12}{'d_path_len':>14}{'verdict':>16}"
    print(hdr2)
    print("-" * len(hdr2))
    for name in COMPARE_ORDER:
        if name == "baseline":
            continue
        f = pick(name)
        d_total = f["total_s"] - bf["total_s"]
        d_build = f["build_s"] - bf["build_s"]
        d_path = f["total_path_length"] - bf["total_path_length"]
        # baseline "wins" if group is not strictly shorter path AND not faster
        better_path = d_path < -1e-6
        verdict = "GROUP BETTER" if better_path else ("baseline>=" )
        print(f"{arts[name]['label']:<22}{d_total:>12.3f}{d_build:>12.3f}{d_path:>14.5f}{verdict:>16}")

    print()
    print(f"Per-query {stage} incumbent length (rad):")
    qh = f"{'group':<22}" + "".join(f"{q:>12}" for q in QUERY_ORDER)
    print(qh)
    print("-" * len(qh))
    for name in COMPARE_ORDER:
        q = pick(name)["incumbent_queries"]
        print(f"{arts[name]['label']:<22}" + "".join(f"{float(q.get(k,0.0)):>12.4f}" for k in QUERY_ORDER))

    print()
    print(f"Per-query delta vs baseline ({stage}) (negative = group shorter):")
    print(qh)
    print("-" * len(qh))
    bq = pick("baseline")["incumbent_queries"]
    for name in COMPARE_ORDER:
        if name == "baseline":
            continue
        q = pick(name)["incumbent_queries"]
        print(f"{arts[name]['label']:<22}" + "".join(
            f"{float(q.get(k,0.0))-float(bq.get(k,0.0)):>12.4f}" for k in QUERY_ORDER))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
