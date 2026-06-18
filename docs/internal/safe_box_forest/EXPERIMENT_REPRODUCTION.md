# SBF Experiment Reproduction Guide

This guide covers the SBF module inside the `RapidBoxForest` repository.

## 1. Prerequisites

Expected sibling layout:

```text
workspace/
  link_interval_envelope/
  lect_database/
  safe_box_forest/
```

Required tools:

- CMake and a C++20-capable compiler.
- Python 3 with the packages needed by this repository's Python bindings and experiment scripts.
- XeLaTeX if you want to rebuild the paper PDF.

The workspace root CMake entrypoint wires the module dependencies in order.

## 2. Build And Validate The Repository

From the repository root:

```bash
BUILD_DIR="$PWD/build_py310" \
SBF_BUILD_EXPERIMENTS=ON \
PYTHON_EXECUTABLE="$(command -v python)" \
bash tests/run_all.sh
```

This configures the project, builds the C++ targets, builds the Python module,
runs CTest, and checks that the standalone package does not pull headers back
from `cpp/v6`.

If you only want the library build without experiment binaries, omit
`SBF_BUILD_EXPERIMENTS=ON`.

## 3. Regenerate The Current TRO 2026 Paper From Versioned Artifacts

The repository already tracks the paper source, generated table manifests, and
the JSON artifacts consumed by the table generator. After the build above,
regenerate the main paper tables and figures with:

```bash
PYTHONPATH="$PWD/build_py310/python:$PWD/python:$PWD/experiments" \
python experiments/tro2026_generate_tables.py \
  --mode main \
  --manifest doc/paper/tro_rewrite_2026/generated/tro_table_generation_manifest_main.json
```

Then compile the paper:

```bash
cd doc/paper/tro_rewrite_2026
xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
```

The manuscript entry point is `doc/paper/tro_rewrite_2026/sbf_tro_2026.tex`.

## 4. Full Experiment Rerun Entry Points

The heavy experiments are not hidden behind one monolithic script. Use the
numbered runners in `experiments/` according to the artifact you want to
reproduce.

Recommended entry points:

- `tro2026_main_01_evidence_validation.py` - strict/provisional evidence-profile ablation.
- `tro2026_main_02_query_amortization.py` - amortization study.
- `tro2026_main_03_implementation_optimization.py` - implementation optimization matrix.
- `paper_14_shelf_anytime_tradeoff.py` - Shelf+IIWA anytime trade-off data.
- `paper_15_random_anytime_tradeoff.py` - random-scene anytime trade-off data.
- `paper_16_shelf_iris_np_gcs_anytime.py` and `paper_16_random_iris_np_gcs_anytime.py` - IRIS-NP+GCS anytime traces.
- `tro2026_generate_tables.py` - final LaTeX table and figure generation from artifacts.

To inspect the arguments for any experiment script:

```bash
python experiments/<script_name>.py --help
```

`experiments/README.md` describes the current script tiers and which commands
are preferred over older compatibility wrappers.

## 5. Output Locations

- `outputs/paper/` - paper-facing JSON artifacts emitted by experiment runners.
- `doc/paper/tro_rewrite_2026/generated/` - generated LaTeX tables and figure files.
- `doc/paper/tro_rewrite_2026/` - manuscript source and final PDF.

As a rule, rerun experiment scripts first, rerun `tro2026_generate_tables.py`
second, and rebuild the PDF last.