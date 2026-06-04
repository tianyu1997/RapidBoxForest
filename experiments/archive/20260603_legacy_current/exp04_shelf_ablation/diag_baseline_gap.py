#!/usr/bin/env python3
"""Deep diagnostic: per-stage incumbent progression + split-dim usage for the
groups that beat baseline, to find WHY baseline underperforms."""
from __future__ import annotations
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from analyze_ablation_timings import (  # noqa: E402
    GROUP_SPECS, STAGE_ORDER, load_json, summarize_artifact,
)
REPO = Path(__file__).resolve().parents[2]
E4 = REPO / "outputs/new_experiments/exp04_shelf_ablation_full_ablation_20260528"
D48 = REPO / "outputs/new_experiments/exp04_split_compare_depth48_20260528"

GROUPS = ["baseline", "single_thread", "round_robin_d48", "baseline_d48"]

def root_for(name):
    return E4 if str(GROUP_SPECS[name]["root"]) == "e4" else D48

arts = {}
for n in GROUPS:
    p = root_for(n) / str(GROUP_SPECS[n]["path_name"])
    arts[n] = summarize_artifact(n, p, load_json(p))

print("=== Per-stage cumulative total path length (incumbent) ===")
hdr = f"{'group':<18}" + "".join(f"{s:>10}" for s in STAGE_ORDER)
print(hdr); print("-"*len(hdr))
for n in GROUPS:
    st = arts[n]["stages"]
    print(f"{arts[n]['label']:<18}" + "".join(
        f"{st[s]['incumbent_total_length']:>10.3f}" for s in STAGE_ORDER))

print("\n=== Per-stage cumulative build_s / query_s ===")
for n in GROUPS:
    st = arts[n]["stages"]
    print(f"{arts[n]['label']}")
    print("  build_s :", " ".join(f"{s}={st[s]['cumulative_build_s']:.3f}" for s in STAGE_ORDER))
    print("  query_s :", " ".join(f"{s}={st[s]['cumulative_query_s']:.3f}" for s in STAGE_ORDER))

print("\n=== 'high' stage split dim fractions + zero axes ===")
for n in GROUPS:
    sp = arts[n]["stages"]["high"]["split"]
    fr = sp["dim_fractions"]
    print(f"{arts[n]['label']:<18} zero_axes={sp['zero_axes']}")
    print("   " + " ".join(f"d{i}={fr.get(f'dim_{i}',0):.3f}" for i in range(7)))

print("\n=== 'high' stage component-connect ===")
for n in GROUPS:
    cc = arts[n]["stages"]["high"]["component_connect"]
    print(f"{arts[n]['label']:<18} attempts={cc['attempts']:.0f} succ_rate={cc['success_rate']} "
          f"connected_root_pairs_max={cc['connected_root_pairs_max']:.0f} staged_targets={cc['staged_targets']:.0f}")

print("\n=== 'high' unique/certified box counts ===")
for n in GROUPS:
    b = arts[n]["stages"]["high"]["build"]
    print(f"{arts[n]['label']:<18} unique={b['unique_box_count']} certified={b['certified_box_count']}")
