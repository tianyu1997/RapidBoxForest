# RapidBoxForest Documentation

This directory contains workspace-level documentation. Module-specific usage
and API details live beside each module.

## Reading Order

1. `../README.md` - repository purpose, layout, build flags, and experiment
   entry points.
2. `ARCHITECTURE.md` - code framework, package boundaries, dependency flow, and
   generated-file policy.
3. `../link_interval_envelope/README.md` - envelope computation package.
4. `../lect_database/README.md` - LECT database, snapshots, and online cache.
5. `../safe_box_forest/README.md` - planner facade, query pipeline, and Python
   bindings.
6. `../experiments/README.md` - current experiment runners and output layout.

## Active Planning Notes

The files in this directory that end in `_PLAN.md` or include dated status
snapshots are engineering planning notes from the current development cycle.
They are useful for implementation context, but they are not the canonical API
or build documentation.

## Archive Policy

- `archive/` keeps superseded planning notes that may still explain historical
  decisions.
- `paper/` and `safe_box_forest/experiments/sbf_old/` keep reproduction
  artifacts for older TRO-oriented workflows.
- Generated outputs belong under ignored paths such as `build*/`, `outputs/`,
  `**/outputs/`, `__pycache__/`, and `.sbf_lect_database/`.

Do not add local build caches, CMake output, Python bytecode, or LaTeX
intermediate files to git. Keep source, scripts, committed result manifests,
figures, and PDFs only when they are required for reproducibility.
