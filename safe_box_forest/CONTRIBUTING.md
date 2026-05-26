# Contributing To SafeBoxForest

Thanks for contributing to the standalone SBF repository.

## Scope

This repository is the public standalone package for SBF. Keep changes local to
this repository unless the work explicitly requires coordinated updates in
`LECT` or `link_interval_envelope`.

## Before Opening A Pull Request

1. Build and run the default validation workflow with `bash tests/run_all.sh`.
2. If your change touches experiment scripts or paper-facing outputs, read `docs/EXPERIMENT_REPRODUCTION.md` and rerun the affected generator or experiment entry point.
3. If your change updates the TRO manuscript under `doc/paper/tro_rewrite_2026`, rebuild `sbf_tro_2026.tex` with XeLaTeX so the checked-in PDF matches the source.

## Change Guidelines

- Prefer focused pull requests that solve one problem at a time.
- Keep public APIs, experiment assumptions, and file formats backward compatible unless the change is intentionally breaking and clearly documented.
- Do not commit large temporary outputs, scratch logs, or machine-local paths.
- When changing experiment behavior, document the affected entry point and output path in the pull request description.

## Reporting Reproduction Changes

For any paper-facing change, include these details in the pull request:

- which script or manifest was rerun,
- which output files changed,
- which validation command you executed,
- and whether the final paper PDF was rebuilt.

## Questions

If you are unsure whether a change belongs in SBF or in one of its sibling
dependencies, open an issue first and describe the intended public API surface.