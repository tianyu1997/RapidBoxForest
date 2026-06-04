# TRO 2026 Experiment Workspace

This directory contains the current paper-facing experiment framework for
`paper/sbf_tro_2026.tex`. Legacy scripts were archived under
`experiments/archive/20260603_legacy_current/` and are no longer paper-facing
entry points.

## Current Entry Points

```bash
python3 experiments/run_tro2026.py --phase smoke --dry-run
python3 experiments/run_tro2026.py --phase smoke --execute
python3 experiments/common/generate_random_scene_catalog.py --out outputs/tro2026/catalogs/random_scene_catalog.json --scene-seeds 50 --mode generate
python3 experiments/generate_tro2026_paper_assets.py --out-dir outputs/tro2026 --paper-dir paper
```

## Experiment Modules

1. `exp01_endpoint_envelope/` — endpoint envelope source study.
2. `exp02_link_envelope/` — `S=1` link envelope representation study.
3. `exp03_lect_performance/` — LECT operation and memory study.
4. `exp04_shelf_leaf_rrt/` — Shelf+IIWA leaf-sweep + RRT grower study.
5. `exp05_shelf_cross_algorithm/` — Shelf+IIWA cross-algorithm comparison.
6. `exp06_random_robot/` — saved-catalog random multi-robot study.
7. `exp07_dynamic_update/` — current dynamic-update study.
8. `appendix_sweeps/` — parameter and sensitivity sweeps.

## Required Policy

Paper-facing scripts must not import or execute
`safe_box_forest/experiments/sbf_old`. Historical files may mention that path
only inside `experiments/archive/`.

Exp.5 may import non-RBF baseline values from the old TRO artifact only through
`exp05_shelf_cross_algorithm/import_old_shelf_baselines.py`. The importer
records source paths, SHA256 hashes, scene/query checks, and the fixed `0.01`
audit-step evidence. Old SBF rows are excluded because current RBF rows are
regenerated with the leaf-sweep + RRT grower profile.

Exp.6 current RBF, PRM, RRTConnect, and BIT* rows must come from the saved v5
random-scene catalog. The paper asset generator may still read the old TRO
random best-point table for IRIS-NP+GCS context until the Drake/GCS pipeline is
self-contained in the current runner; imported rows must be described as
context, not current-catalog trials.

All random-scene experiments must consume a saved catalog in full runs. Audit
time is reported separately from planning time.

The default RBF method is the registered Exp.4 algorithm profile
`exp04_leaf_sweep_rrt_b100_d34`: `build_leaf_sweep_refined` with shallow leaf
sweep followed by restricted deep refinement, a bounded RRT grower
(`25` extra boxes, `100 ms` cap), and strict no-post final audit. The connector
uses at most one `50 ms` BiRRT pair attempt per gap.
Shelf+IIWA baseline rows add the `exp04_leaf_sweep_rrt_d23_b100_d34` warm d23
evidence-cache override. The authoritative parameters live in
`experiments/common/rbf_defaults.py`.
