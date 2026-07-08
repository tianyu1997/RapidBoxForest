# Paper Asset Manifest

Paper-facing assets are generated into `paper/generated/` and `paper/figures/`.
Old TRO artifacts may be used only as external historical references while
porting or contextualizing baselines. They are not bundled in the clean public
export by default.

## Required Main Assets

1. `paper/generated/tab_tro_endpoint_envelope.tex`
2. `paper/generated/tab_tro_link_envelope.tex`
3. `paper/generated/fig_tro_shelf_tradeoff.pdf`
4. `paper/generated/tab_tro_shelf_ablation.tex`
5. `paper/generated/tab_tro_shelf_cross_algorithm.tex`
6. `paper/generated/fig_tro_random_tradeoff.pdf`
7. `paper/generated/tab_tro_random_summary.tex`
8. `paper/generated/tro_table_generation_manifest.json`

## Manifest Requirements

The generated JSON manifest must include:

1. source artifact paths and hashes;
2. runner command lines;
3. git SHA and environment metadata;
4. table row-selection rule;
5. audit segment step and simplify policy;
6. whether planning time excludes audit time;
7. scene catalog path for Exp.6.

When a generated figure or table includes context imported from the old TRO
paper artifacts, the manifest must record the imported source path. Such rows
must be described as historical/common-rule context unless the current runner
has rerun the method on the active saved catalog.
