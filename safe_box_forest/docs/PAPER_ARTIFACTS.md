# TRO 2026 Paper Artifact Map

This repository keeps the current SBF TRO paper and its experiment artifacts in
one place so the source code, generated tables, and committed JSON remain easy
to cross-check.

## Manuscript Source

- `doc/paper/tro_rewrite_2026/sbf_tro_2026.tex` - main paper entry point.
- `doc/paper/tro_rewrite_2026/references.bib` - bibliography.
- `doc/paper/tro_rewrite_2026/generated/` - generated LaTeX snippets and figure PDFs included by the paper.

## Experiment And Table Generation Pipeline

- `experiments/tro2026_generate_tables.py` - authoritative main-text table and figure generator.
- `doc/paper/tro_rewrite_2026/generated/tro_table_generation_manifest_main.json` - main-paper generation manifest.
- `doc/paper/tro_rewrite_2026/generated/tro_table_generation_manifest_all.json` - broader generation manifest used for extended outputs.

The generator consumes paper-facing JSON artifacts and writes updated `.tex`
tables, macros, and selected figure PDFs into `doc/paper/tro_rewrite_2026/generated/`.

## Artifact Inputs

- `outputs/paper/` - committed JSON outputs from the paper experiment runners.
- `experiments/paper_14_shelf_anytime_tradeoff.py` - Shelf+IIWA anytime artifact source.
- `experiments/paper_15_random_anytime_tradeoff.py` - balanced random-scene anytime artifact source.
- `experiments/tro2026_main_01_evidence_validation.py` - evidence-profile ablation source.
- `experiments/tro2026_main_02_query_amortization.py` - amortization study source.
- `experiments/tro2026_main_03_implementation_optimization.py` - implementation optimization source.

## Recommended Public-Release Workflow

1. Build and validate the repository with `bash tests/run_all.sh`.
2. Regenerate tables and figures with `experiments/tro2026_generate_tables.py`.
3. Compile `doc/paper/tro_rewrite_2026/sbf_tro_2026.tex` twice with XeLaTeX.
4. If you reran heavy experiments, keep the updated JSON under `outputs/paper/` together with the regenerated paper assets.

For a command-oriented walkthrough, see `docs/EXPERIMENT_REPRODUCTION.md`.