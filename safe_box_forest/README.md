
# SafeBoxForest Module

This module contains the SafeBoxForest (SBF) build/query pipeline. It depends on
the sibling `lect_database` module, which in turn depends on the sibling
`link_interval_envelope` module.

In the public RapidBoxForest release this module is normally built through the
workspace root.  Use the top-level README and `docs/REPRODUCIBILITY.md` for
paper-facing builds and experiment reruns.

The package separates SBF into explicit rings:

- Growers: `RrtGrower`, `FrontwaveGrower`
- Free-box construction: `FindFreeBoxService`
- Graph: `BoxGraph` utilities
- Merger: `Consolidator`
- Connector: RRT-Connect, chain paving, island connector
- Query: `CorridorQuery`
- Facade: `SafeBoxForest`
- Runtime: `StageContext`, `RuntimeConfig`, `ThreadPoolExecutor`

Paper-facing configurations are defined by the top-level experiment profiles
under `experiments/common/`.  Do not infer the paper configuration from a raw
default-constructed planner config.

## Documentation

- `docs/README.md` - documentation index and reading order.
- `../experiments/README.md` - paper experiment script layout and preferred
  entry points.
- `../docs/REPRODUCIBILITY.md` - workspace-level build, paper, and experiment
  reproduction guide.

## Release Metadata

- `LICENSE` - MIT license for the standalone SBF repository.
- `CONTRIBUTING.md` - contribution and review checklist.
- `CITATION.cff` - software citation metadata for GitHub and downstream users.

## Dependencies

Expected sibling layout:

```text
workspace/
	link_interval_envelope/
	lect_database/
	safe_box_forest/
```

For standalone SBF development outside this workspace, override the database
module path with `SBF_LECT_DATABASE_MODULE_DIR`.

## Build And Test

```bash
bash tests/run_all.sh
```

Useful knobs:

```bash
BUILD_DIR=build_release CMAKE_BUILD_TYPE=Release bash tests/run_all.sh
SBF_BUILD_EXPERIMENTS=ON bash tests/run_all.sh
```

The test script configures the workspace root, builds C++, builds the Python
extension, runs CTest, and checks that this module does not include forbidden
private headers or paths.

## TRO 2026 Paper Quickstart

Use the workspace-level dispatcher and paper asset generator rather than
package-local historical scripts:

```bash
python3 ../experiments/run_tro2026.py --phase smoke --dry-run
python3 ../experiments/generate_tro2026_paper_assets.py \
  --out-dir ../outputs/new_experiments/tro2026 \
  --paper-dir ../paper
```

For the complete public workflow, start from `../docs/REPRODUCIBILITY.md`.

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

## Paper Experiments

The clean public release uses the top-level TRO dispatcher for paper-facing
experiments:

```bash
python3 ../experiments/run_tro2026.py --phase smoke --dry-run
```

Historical Drake/Meshcat demos may exist in the private development checkout,
but they are excluded from the default clean public export and are not the
recommended reproduction entry point.

## Parallelism Contract

SBF is serial by default. Parallel execution is enabled only through
`RuntimeConfig` or per-stage thread knobs. Worker tasks compute proposals,
free-box searches, or merge validations; mutable graph updates, box id
assignment, and oracle reservations are committed on the master thread.

For LECT-backed oracles, mutable worker-local FFB uses exclusive domain roots
and commits back through node-id remapping. Merger validation uses read-only
worker sessions. This keeps tree/materializer/counter state isolated from
parallel writes while preserving deterministic reductions.
