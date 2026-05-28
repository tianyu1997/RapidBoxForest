# Experiment script layout

For a repository-level rerun guide, start with `../docs/EXPERIMENT_REPRODUCTION.md`.

The TRO 2026 experiment scripts are organized into three tiers.

## Main compressed TRO 2026 pipeline

These are the preferred entry points for the compressed main-text experiment design:

- `tro2026_main_01_evidence_validation.py` — strict/provisional evidence-profile ablation.
- `tro2026_main_02_query_amortization.py` — multi-query amortization JSON/CSV/figure generation.
- `tro2026_main_03_implementation_optimization.py` — implementation-optimization matrix for LECT, link-interval-envelope, and SBF.
- `tro2026_generate_tables.py` — compressed main-text and appendix table generator.

Compatibility wrappers are kept for older commands:

- `paper_14_validation_profile_ablation.py`
- `paper_15_query_amortization.py`
- `paper_generate_tro_tables.py`

## Warm-reuse default

The current warm-d18 experiment runners use a writable active cache plus a
separate read-only external evidence source. By default that external source is
opened as a LECT read snapshot, with the snapshot stored under
`<external_evidence_path>/lect_snapshot` and auto-built on first use.

Common runners that import `sbf_old/common_sbf_config.py` now accept:

- `--external-evidence-mode {legacy,snapshot}`
- `--external-evidence-auto-build-snapshot` / `--no-external-evidence-auto-build-snapshot`

Use `legacy` only when you need to compare against the old read-only database
open path. The default `snapshot` mode is the paper-facing best-performance
configuration for warm external evidence reuse.

## Supporting paper experiment runners

These scripts still produce detailed artifacts consumed by the main generator or appendix tables:

- `paper_01_epiaabb_pipeline.py`
- `paper_02_link_envelope_pipeline.py`
- `paper_03_marcucci_envelope_build.py`
- `paper_04_marcucci_combined.py`
- `paper_04_audited_corridor_gcs.py`
- `paper_04_baselines_marcucci.py`
- `paper_04_grower_tradeoff.py`
- `paper_04_merger_gcs.py`
- `paper_04_rrt_connect_baseline.py`
- `paper_05_random_robot_scenes.py`
- `paper_06_obstacle_rebuild.py`
- `paper_07_merger_protected_study.py`
- `paper_07_parallel_scaling.py`
- `paper_09_random_dynamic_rebuild.py`
- `paper_11_soundness_audit_suite.py`
- `paper_12_random_scene_rrt_baseline.py`
- `paper_13_mechanism_diagnostics.py`

## Archived exploratory scripts

Ad-hoc demos and legacy exploratory scripts live under `archive/legacy_demos/`.
