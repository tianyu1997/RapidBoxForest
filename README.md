# RapidBoxForest

RapidBoxForest is a workspace-style monorepo for a safe motion-planning stack
built around interval link envelopes, a persistent LECT database, and the
SafeBoxForest planner.

The repository is organised as three first-party modules with one-way
dependencies:

```text
safe_box_forest -> lect_database -> link_interval_envelope
```

The top-level project owns the integrated CMake entry point, shared
documentation, experiments, and paper artifacts. Implementation code remains in
the module directories.

## Repository Layout

```text
.
|-- CMakeLists.txt                  integrated workspace build
|-- docs/                           workspace architecture and active plans
|-- experiments/                    current experiment runners and protocols
|-- link_interval_envelope/         interval FK and link-envelope package
|-- lect_database/                  persistent LECT database and SBF adapter
|-- safe_box_forest/                planner, query pipeline, and bindings
`-- paper/                          manuscript sources and generated figures
```

## Modules

- `link_interval_envelope/` computes conservative per-link envelopes from robot
  models and joint-interval boxes. It exposes C++ targets, Python bindings, a
  batch API, and an incremental context for nearby interval queries.
- `lect_database/` persists the LECT split tree, evidence records, read
  snapshots, and online envelope cache. It also owns the SBF-facing scene,
  collision checker, and database-backed oracle adapter.
- `safe_box_forest/` builds and queries SafeBoxForest planning forests. Its
  pipeline combines growers, free-box search, graph construction, merger,
  connector, corridor query, dynamic update, and optional Python bindings.

See `docs/ARCHITECTURE.md` for the code framework and package boundaries.

## Build

Configure the full workspace from the repository root:

```bash
cmake -S . -B build \
  -DRBF_BUILD_ENVELOPE=ON \
  -DRBF_BUILD_LECT_DATABASE=ON \
  -DRBF_BUILD_SBF=ON \
  -DRBF_BUILD_TESTS=ON \
  -DRBF_WITH_PYTHON=OFF

cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Useful options:

- `RBF_BUILD_SBF=ON` enables the planner module. It is off by default for a
  smaller dependency surface.
- `RBF_WITH_PYTHON=ON` builds Python extensions for enabled modules.
- `RBF_BUILD_EXPERIMENTS=ON` builds optional C++ experiment drivers.
- `RBF_BUILD_TOOLS=ON` builds LECTDatabase command-line tools.

Each module can also be configured from its own directory; module READMEs
document the standalone workflows.

## Experiments

Current experiment entry points live under `experiments/` and default to writing
generated results under `outputs/new_experiments/`.

```bash
python3 experiments/exp01_endpoint_envelope/run_endpoint_envelope.py --smoke --dry-run
python3 experiments/exp02_link_envelope_s1/run_link_envelope_s1.py --smoke --dry-run
python3 experiments/exp03_lect_microbench/run_lect_microbench.py --dry-run
python3 experiments/exp04_shelf_ablation/run_shelf_ablation.py --dry-run
python3 experiments/exp05_shelf_cross_algorithm/run_shelf_cross_algorithm.py --dry-run
python3 experiments/exp06_random_robot/run_random_robot.py --dry-run
```

Use `experiments/README.md` and `experiments/00_experiment_principles.md` as the
entry point before running long jobs.

## Documentation

- `docs/README.md` - workspace documentation index and cleanup policy.
- `docs/ARCHITECTURE.md` - code framework, module targets, and data flow.
- `link_interval_envelope/README.md` - envelope package usage.
- `lect_database/README.md` - database and online-cache package usage.
- `safe_box_forest/README.md` - planner package usage.

## Generated And Archived Files

Build directories, CMake caches, Python caches, local database caches, and
experiment outputs are generated files and should stay untracked. Historical
scripts under `safe_box_forest/experiments/sbf_old/` and manuscripts under
`paper/sbf_old/` are retained as archival reproduction material unless a
specific replacement is added.
