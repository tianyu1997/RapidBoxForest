# TRO 2026 Experiment Workspace

This directory contains the current paper-facing experiment framework for
`paper/sbf_tro_2026.tex`. Legacy development scripts are not part of the clean
public export and are no longer paper-facing entry points.

## Current Entry Points

```bash
python3 experiments/run_tro2026.py --phase smoke --dry-run
python3 experiments/run_tro2026.py --phase smoke --execute
python3 experiments/common/generate_random_scene_catalog.py --out outputs/tro2026/catalogs/random_scene_catalog.json --scene-seeds 50 --mode generate
python3 experiments/generate_tro2026_paper_assets.py --out-dir outputs/tro2026 --paper-dir paper
```

The default public smoke suite intentionally runs only the lightweight core
mechanism checks (`exp01`, `exp02`). Heavier planning, random-scene, Drake/IRIS,
and dynamic-update studies should be requested explicitly with `--only` once
their required caches, catalogs, and optional dependencies are available.
Smoke execute mode still requires the `link_interval_envelope` Python module to
be importable; use dry-run mode in a source-only checkout.

## Experiment Modules

1. `exp01_endpoint_envelope/` — endpoint envelope source study.
2. `exp02_link_envelope/` — `S=1` link envelope representation study.
3. `exp03_lect_performance/` — LECT operation and memory study.
4. `exp04_shelf_leaf_rrt/` — Shelf+IIWA leaf-sweep + partition-native query-bridge study.
5. `exp05_shelf_cross_algorithm/` — Shelf+IIWA cross-algorithm comparison.
6. `exp06_random_robot/` — saved-catalog random multi-robot study.
7. `exp07_dynamic_update/` — current dynamic-update study.
8. `appendix_sweeps/` — parameter and sensitivity sweeps.

## Required Policy

Paper-facing scripts must not import or execute bundled historical SBF
experiment trees. If a legacy baseline is needed for context, it must be passed
as an external artifact or script path and recorded in the output manifest.

Exp.5 may import non-RBF baseline values from the old TRO artifact only through
`exp05_shelf_cross_algorithm/import_old_shelf_baselines.py`. The importer
records source paths, SHA256 hashes, scene/query checks, and the fixed `0.01`
audit-step evidence. Old SBF rows are excluded because current RBF rows are
regenerated with the leaf-sweep + partition-native profile.

Exp.6 current RBF, PRM, RRTConnect, and BIT* rows must come from the saved v5
random-scene catalog. The paper asset generator may still read the old TRO
random best-point table for IRIS-NP+GCS context until the Drake/GCS pipeline is
self-contained in the current runner; imported rows must be described as
context, not current-catalog trials.

All random-scene experiments must consume a saved catalog in full runs. Audit
time is reported separately from planning time.

The default RBF method is the registered Exp.4 algorithm profile
`exp04_partition_leaf13_d23_fixed800_online25ms`: `build_leaf_sweep_refined`
with offline leaf sweep + adaptive deep refine and partition-native online planning,
strict final audit, and fixed time reporting.
Shelf+IIWA baseline rows add the d23 warm-cached external evidence override.
The authoritative parameters live in
`experiments/common/rbf_defaults.py`.
