# RapidBoxForest Documentation

This directory contains public workspace-level documentation. Module-specific
usage and API details live beside each module.

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

## Development Notes

The tracked documentation tree contains only public API, architecture, release,
and reproduction material. Development plans, dated status snapshots,
exploratory algorithm notes, and paper-drafting notes are not canonical
documentation and should not be committed under `docs/`.

Prototype source trees must be upstreamed into the main modules instead of kept
as parallel workspaces. In particular, `improve_workspace/` is treated as a
forbidden sidecar: the release-readiness checks fail if it exists in the source
tree, even though `.gitignore` also prevents it from being accidentally
committed.

## Archive Policy

- Clean public exports exclude historical archives and the private `paper/`
  directory by default; paper-facing reproduction should use the current
  top-level `experiments/` runners from the development checkout.
- Generated outputs belong under ignored paths such as `build*/`, `outputs/`,
  `**/outputs/`, `__pycache__/`, and `.sbf_lect_database/`.

Do not add local build caches, CMake output, Python bytecode, or LaTeX
intermediate files to git. Keep source, scripts, committed result manifests,
figures, and PDFs only when they are required for reproducibility.
