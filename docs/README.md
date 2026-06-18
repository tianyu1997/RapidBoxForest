# RapidBoxForest Documentation

This directory contains workspace-level documentation. Module-specific usage
and API details live beside each module.

## Reading Order

1. `../README.md` - repository purpose, layout, build flags, and experiment
   entry points.
2. `ARCHITECTURE.md` - code framework, package boundaries, dependency flow, and
   generated-file policy.
3. `REPRODUCIBILITY.md` - build, smoke-test, experiment, cache, and paper-asset
   reproduction workflow.
4. `OPEN_SOURCE_RELEASE_CHECKLIST.md` - release-readiness checks before
   publishing the repository.
5. `../scripts/export_public_release.py` - allowlist-based clean public source
   tree exporter.
6. `../scripts/check_public_release.py` - public release tree validator.
7. `../link_interval_envelope/README.md` - envelope computation package.
8. `../lect_database/README.md` - LECT database, snapshots, and online cache.
9. `../safe_box_forest/README.md` - planner facade, query pipeline, and Python
   bindings.
10. `../experiments/README.md` - current experiment runners and output layout.

## Active Planning Notes

The files in this directory that end in `_PLAN.md` or include dated status
snapshots are engineering planning notes from the current development cycle.
They are useful for implementation context, but they are not the canonical API
or build documentation.

## Archive Policy

- `archive/` keeps superseded planning notes that may still explain historical
  decisions in the private development checkout.
- Clean public exports exclude historical archives by default; paper-facing
  reproduction should use the current top-level `experiments/` runners.
- Generated outputs belong under ignored paths such as `build*/`, `outputs/`,
  `**/outputs/`, `__pycache__/`, and `.sbf_lect_database/`.

Do not add local build caches, CMake output, Python bytecode, or LaTeX
intermediate files to git. Keep source, scripts, committed result manifests,
figures, and PDFs only when they are required for reproducibility.
