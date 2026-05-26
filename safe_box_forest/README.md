# SafeBoxForest Standalone Package

This package contains the SafeBoxForest (SBF) build/query pipeline. It is
independent from `cpp/v6` and uses the consolidated `LECTDatabase` storage,
online-cache, and SBF adapter targets plus the standalone `link_interval_envelope`
package.

Public repository:
https://github.com/tianyu1997/SafeBoxForest

The package separates SBF into explicit rings:

- Growers: `RrtGrower`, `FrontwaveGrower`
- Free-box construction: `FindFreeBoxService`
- Graph: `BoxGraph` utilities
- Merger: `Consolidator`
- Connector: RRT-Connect, chain paving, island connector
- Query: `CorridorQuery`
- Facade: `SafeBoxForest`
- Runtime: `StageContext`, `RuntimeConfig`, `ThreadPoolExecutor`

The default `SBFConfig` is the paper-facing `SBF-SH` profile: CritSample
endpoint evidence, SupportHull envelopes with retained KDOP26 slabs, a 64-box
connected quality floor, strict path audit, and collision-checked path
post-processing.

## Documentation

- `docs/README.md` - documentation index and reading order.
- `docs/EXPERIMENT_REPRODUCTION.md` - build, test, paper, and experiment rerun guide.
- `docs/PAPER_ARTIFACTS.md` - manuscript, generated tables, and result-artifact map.
- `experiments/README.md` - experiment script layout and preferred entry points.

## Release Metadata

- `LICENSE` - MIT license for the standalone SBF repository.
- `CONTRIBUTING.md` - contribution and review checklist.
- `CITATION.cff` - software citation metadata for GitHub and downstream users.

## Dependencies

Expected sibling layout:

```text
workspace/
	SBF/
	LECTDatabase/
	link_interval_envelope/
```

Override dependency paths with `SBF_LECTDATABASE_SOURCE_DIR` and
`SBF_LINK_INTERVAL_ENVELOPE_SOURCE_DIR` when using another layout.

## Build And Test

```bash
bash tests/run_all.sh
```

Useful knobs:

```bash
BUILD_DIR=build_release CMAKE_BUILD_TYPE=Release bash tests/run_all.sh
SBF_BUILD_EXPERIMENTS=ON bash tests/run_all.sh
```

The test script builds C++, builds the Python extension, runs CTest, and checks
that this package does not include v6-only headers or paths.

## TRO 2026 Paper Quickstart

If you want to regenerate the current TRO paper from the committed JSON and
generated manifests, use one build directory consistently and then rerun the
table generator plus XeLaTeX:

```bash
BUILD_DIR="$PWD/build_py310" \
SBF_BUILD_EXPERIMENTS=ON \
PYTHON_EXECUTABLE="$(command -v python)" \
bash tests/run_all.sh

PYTHONPATH="$PWD/build_py310/python:$PWD/python:$PWD/experiments" \
python experiments/tro2026_generate_tables.py \
	--mode main \
	--manifest doc/paper/tro_rewrite_2026/generated/tro_table_generation_manifest_main.json

cd doc/paper/tro_rewrite_2026
xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
```

For a full rerun of the heavy paper experiments, start from
`docs/EXPERIMENT_REPRODUCTION.md` and `experiments/README.md` rather than
calling ad hoc scripts directly.

## C++ Quickstart

```cpp
#include <SBF/sbf.h>

sbf::Robot robot = sbf::Robot::from_json("robot.json");

sbf::SBFConfig config;
config.grower.mode = sbf::GrowerConfig::Mode::RRT;
config.runtime.mode = sbf::ExecutionMode::Parallel;
config.runtime.n_threads = 4;

sbf::SafeBoxForest forest(robot, config);
Eigen::VectorXd start = Eigen::VectorXd::Zero(robot.n_joints());
Eigen::VectorXd goal = Eigen::VectorXd::Zero(robot.n_joints());

sbf::BuildProfile profile = forest.build(start, goal, {});
sbf::QueryResult query = forest.query(start, goal);
```

Downstream CMake projects can use the installed package target:

```cmake
find_package(SBF REQUIRED)
target_link_libraries(my_target PRIVATE SBF::core)
```

The CMake package export is opt-in with `-DSBF_INSTALL_CMAKE_PACKAGE=ON` and
expects installed `LECT` and `link_interval_envelope` packages. The default
sibling-source workflow keeps the export disabled and uses `add_subdirectory`.

## Python Quickstart

After a CMake build, the package is available at `${BUILD_DIR}/python`:

```bash
PYTHONPATH=build/python python - <<'PY'
import sbf

robot = sbf.Robot.from_json("robot.json")
config = sbf.SBFConfig()
config.runtime.mode = sbf.ExecutionMode.Parallel
config.runtime.n_threads = 4

forest = sbf.SafeBoxForest(robot, config)
profile = forest.build([0.0] * robot.n_joints(), [0.1] * robot.n_joints(), [])
result = forest.query([0.0] * robot.n_joints(), [0.1] * robot.n_joints())
print(profile.final_boxes, result.success)
PY
```

## Marcucci Shelf Demo

The standalone package includes a Drake/Meshcat demo for the Marcucci
shelf+IIWA combined scene. It ports the 16 AABB obstacles, five canonical query
pairs, and IIWA14 model data into the SBF package, then exports timing JSON and
a Meshcat HTML visualization.

```bash
PYTHONPATH=build_py310/python:python \
python experiments/archive/legacy_demos/marcucci_shelf_demo.py \
	--query AS->TS \
	--out-dir outputs/marcucci_shelf_demo
```

Outputs:

- `marcucci_shelf_sbf_run.json`: build/query timings and diagnostics.
- `paths.json`: path waypoints in the same shape as the legacy visualizer.
- `marcucci_shelf_sbf_path.html`: Drake Meshcat scene with the SBF path trace.

Set `GCS_REPO=/path/to/gcs-science-robotics` or pass `--gcs-repo` if the Drake
model repository is not beside this workspace.

## Parallelism Contract

SBF is serial by default. Parallel execution is enabled only through
`RuntimeConfig` or per-stage thread knobs. Worker tasks compute proposals,
free-box searches, or merge validations; mutable graph updates, box id
assignment, and oracle reservations are committed on the master thread.

For LECT-backed oracles, mutable worker-local FFB uses exclusive domain roots
and commits back through node-id remapping. Merger validation uses read-only
worker sessions. This keeps tree/materializer/counter state isolated from
parallel writes while preserving deterministic reductions.