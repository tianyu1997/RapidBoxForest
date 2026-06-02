# Repository Guidelines

## Current Paper Goal

The active goal is to finish the TRO manuscript at `paper/sbf_tro_2026.tex`.
The code framework and basic paper structure are in place; current work is in
the experiment section, especially Experiment 4 shelf ablation. Exp.4 should
use the baseline with the d23 warm cache, while non-baseline comparison groups
run without cache unless explicitly changed for a new study. Exp.7 and Exp.8
are no longer part of the active experiment plan.

## Project Structure & Module Organization

RapidBoxForest is a workspace monorepo with one-way dependencies:
`safe_box_forest -> lect_database -> link_interval_envelope`. The root
`CMakeLists.txt` only wires modules together.

- `link_interval_envelope/`: interval FK, endpoint evidence, envelope APIs,
  Python facade, examples, and tests.
- `lect_database/`: persistent LECT tree, evidence storage, online cache, SBF
  adapter, tools, and tests.
- `safe_box_forest/`: planner facade, grower/free-box/connector/query stages,
  Python bindings, package-local experiments, and tests.
- `experiments/`: current top-level experiment runners and shared helpers.
- `docs/` and module `docs/`: architecture, API notes, plans, and reproduction
  guides. `paper/` keeps manuscript and reproducibility artifacts.

## Build, Test, and Development Commands

```bash
cmake -S . -B build -DRBF_BUILD_ENVELOPE=ON -DRBF_BUILD_LECT_DATABASE=ON -DRBF_BUILD_SBF=ON -DRBF_BUILD_TESTS=ON -DRBF_WITH_PYTHON=OFF
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Use module wrappers for focused validation:

```bash
bash link_interval_envelope/tests/run_all.sh
bash safe_box_forest/tests/run_all.sh
python3 experiments/exp04_shelf_ablation/run_shelf_ablation.py --dry-run
```

Enable optional pieces with `RBF_WITH_PYTHON=ON`, `RBF_BUILD_EXPERIMENTS=ON`,
or `RBF_BUILD_TOOLS=ON`.

## Coding Style & Naming Conventions

Use C++20, CMake targets, and existing module APIs before adding abstractions.
C++ code uses 4-space indentation in LECT/envelope files and tab-oriented
indentation in several SBF headers; match the surrounding file. Prefer
`snake_case` for functions/variables, `PascalCase` for types, and descriptive
config structs such as `RBFPlanningConfig`. Python scripts use clear
`snake_case` names and argparse-style CLI flags.

## Testing Guidelines

C++ tests are under each module's `tests/` directory and run through CTest.
Python API smoke tests are named `test_python_api.py`. Add focused regression
tests for public API changes, database persistence changes, connector behavior,
and experiment dispatcher changes. For long experiments, first run `--dry-run`
or `--smoke --dry-run` and document output paths.

## Commit & Pull Request Guidelines

Commit history is mixed, but prefer concise imperative subjects, for example
`Add shelf ablation diagnostic script` or `Refactor read snapshot metrics`.
Keep commits scoped to one module or workflow when possible.

Pull requests should describe the change, affected modules, validation commands,
and changed artifacts. For paper-facing or experiment changes, list rerun
scripts, manifests, output files, and whether PDFs/tables were regenerated.

## Generated Files & Configuration

Do not commit local build directories, CMake caches, `__pycache__/`, `.pyc`,
`.sbf_lect_database/`, `outputs/`, scratch logs, or LaTeX intermediates. Keep
archived scripts and paper assets only when needed for reproducibility.
