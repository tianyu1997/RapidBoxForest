#!/usr/bin/env python3
"""Hybrid (aafk_volume_min_dim6) verification vs baseline & round-robin.

Compares the new AAFKVolumeMin+dim6 hybrid split policy against the pure
AAFKVolumeMin baseline and the round-robin policy at both d40 and d48, to test
whether guaranteeing coverage of the starved wrist DOF (dim_6) lets the
volume-min baseline match/beat round-robin.
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
DEFAULT_E4 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_hybrid_d40_20260528"
DEFAULT_D48 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_hybrid_d48_20260528"

ORDER = [
    "baseline",
    "round_robin_d48",  # rendered from d48 root regardless; labelled below
    "hybrid_d40",
    "baseline_d48",
    "hybrid_d48",
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--e4-root", type=Path, default=DEFAULT_E4)
    ap.add_argument("--depth48-root", type=Path, default=DEFAULT_D48)
    args = ap.parse_args()

    # d40 group set
    d40 = ["baseline", "round_robin_d40", "hybrid_d40"]
    d48 = ["baseline_d48", "round_robin_d48", "hybrid_d48"]

    # round_robin at d40 isn't in GROUP_SPECS; synthesize from e4 root.
    specs = dict(GROUP_SPECS)
    specs["round_robin_d40"] = {
        "path_name": "round_robin_split_policy.json",
        "label": "Round-robin d40",
        "root": "e4",
    }

    arts: dict[str, dict] = {}
    for name in d40 + d48:
        spec = specs[name]
        root = args.e4_root if str(spec["root"]) == "e4" else args.depth48_root
        path = root / str(spec["path_name"])
        arts[name] = summarize_artifact(name, path, load_json(path), spec_override=spec)

    def block(title: str, names: list[str], base_key: str) -> None:
        print("=" * 96)
        print(title)
        print("=" * 96)
        header = f"{'group':<20}{'total_s':>10}{'build_s':>10}{'query_s':>10}{'path_len':>12}{'d_path':>12}{'verdict':>14}"
        print(header)
        print("-" * len(header))
        bp = arts[base_key]["final"]["total_path_length"]
        for name in names:
            f = arts[name]["final"]
            d = f["total_path_length"] - bp
            verdict = "" if name == base_key else ("SHORTER" if d < -1e-6 else "longer/eq")
            print(f"{arts[name]['label']:<20}{f['total_s']:>10.3f}{f['build_s']:>10.3f}"
                  f"{f['query_s']:>10.3f}{f['total_path_length']:>12.4f}{d:>12.4f}{verdict:>14}")
        print()
        print("Per-query final incumbent length (rad):")
        qh = f"{'group':<20}" + "".join(f"{q:>12}" for q in QUERY_ORDER)
        print(qh)
        print("-" * len(qh))
        for name in names:
            q = arts[name]["final"]["incumbent_queries"]
            print(f"{arts[name]['label']:<20}" + "".join(f"{float(q.get(k,0.0)):>12.4f}" for k in QUERY_ORDER))
        print()

    block("HYBRID vs BASELINE vs ROUND-ROBIN  @ d40 (final stage 'high')", d40, "baseline")
    block("HYBRID vs BASELINE vs ROUND-ROBIN  @ d48 (final stage 'high')", d48, "baseline_d48")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
