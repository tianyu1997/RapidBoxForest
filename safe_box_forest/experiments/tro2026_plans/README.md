# TRO 2026 Experiment Plan Index

This folder contains the paper-facing experiment design for the TRO rewrite. The old `paper_*.py` scripts remain implementation references; final paper numbers should be regenerated under the frozen protocol described here.

## Global Execution Environment

Run experiments from `cpp/SBF` unless a document says otherwise. This workspace currently has two relevant ABI profiles:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
export PYTHON_LIE=/home/tian/miniconda3/envs/sbf/bin/python3.10
export PYTHON_SBF=/home/tian/miniconda3/bin/python

# Use this profile for Exp.1 link-interval-envelope microbenchmarks.
export PYTHONPATH_LIE="$PWD/../link_interval_envelope/build_py310/python:$PWD/../link_interval_envelope/python${PYTHONPATH:+:$PYTHONPATH}"

# Use this profile for Exp.2-Exp.8 and final safety accounting.
export SBF_BUILD_DIR="$PWD/build_perf"
export BUILD_DIR="$PWD/build_perf"
export PYTHONPATH="$PWD/build_perf/python:$PWD/python:$PWD/experiments${PYTHONPATH:+:$PYTHONPATH}"
```

This split is required because the current SBF implementation is available as cpython-313 in `build_perf`, while the standalone link-interval-envelope package is currently available as cpython-310 in `build_py310`.

Use smoke commands first to validate schema and nonempty artifacts. Full commands should only be run after smoke artifacts parse, audit semantics are checked, and baseline budgets are frozen.

## Formal Experiment Order

### RBF-Only Updated Track

The RBF-only track is frozen in `rbf_only_lifelong_cache_execution_plan.md`. For that track, the Lifelong/canonical cache mechanism is executed before the main experiments, and later RBF experiments use that mechanism by default unless a row is explicitly marked as a cache ablation.

1. `exp01_link_envelope_foundation.md` - endpoint and link envelope microbenchmarks.
2. `exp02_lect_reuse_cache_attribution.md` - LECT reuse and cache attribution.
3. `exp03_sbf_construction_ablation.md` - grow-stop, connector, merger, and repair ablations.
4. `exp04_shelf_iiwa_main_benchmark.md` - main Marcucci shelf+IIWA benchmark and multi-query amortization.
5. `exp05_cross_robot_random_scenes.md` - UR5/Panda randomized scene generalization.
6. `exp06_dynamic_scene_update_reuse.md` - obstacle edits and local update/rebuild.
7. `exp07_gcs_composition_boundary.md` - GCS negative controls and protected adapter.
8. `exp08_parallel_resource_scaling.md` - thread/resource scaling and root-cause profiling.

## Cross-Cutting Documents

- `00_protocol_gates.md` - artifact schema, statistics, and audit policy gates.
- `final_safety_accounting.md` - final paper-wide safety accounting and failure taxonomy.

## Paper Integration Rule

Each result used in the paper must come from a JSON artifact. Do not hand-edit numerical values in LaTeX tables. Solver success is not planner success unless final strict collision audit passes.
