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
documentation, and experiments. The private development checkout also keeps
paper artifacts, but the public source export omits `paper/`. Implementation
code remains in the module directories.

## Repository Layout

```text
.
|-- CMakeLists.txt                  integrated workspace build
|-- docs/                           public workspace architecture and release docs
|-- experiments/                    current experiment runners and protocols
|-- link_interval_envelope/         interval FK and link-envelope package
|-- lect_database/                  persistent LECT database and SBF adapter
`-- safe_box_forest/                planner, query pipeline, and bindings
```

The private development checkout also keeps `paper/` with manuscript sources
and generated figures. The clean public source export intentionally omits that
directory.

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

For source builds in a restricted-network environment, install Eigen,
nlohmann_json, and pybind11 first, then configure with
`-DFETCHCONTENT_FULLY_DISCONNECTED=ON` to avoid implicit dependency downloads.
See `docs/REPRODUCIBILITY.md` for the Python-extension smoke workflow.

Conda users can create the recommended build-and-experiment environment with:

```bash
conda env create -f environment.yml
conda activate rapidboxforest
```

## Experiments

Current experiment entry points live under `experiments/` and default to writing
generated results under `outputs/new_experiments/`.

```bash
python3 experiments/run_tro2026.py --phase smoke --dry-run
python3 experiments/run_tro2026.py --phase smoke --execute
python3 experiments/run_tro2026.py --phase smoke --only exp04 --dry-run
```

The default smoke execute path imports the `link_interval_envelope` Python
module. Use dry-run mode for a source-only checkout, or build/install the Python
extension before using `--execute`.

Use `experiments/README.md` and `experiments/00_experiment_principles.md` as the
entry point before running long jobs.

For a clean reproduction workflow, including smoke runs, saved random-scene
catalogs, cache policy, and paper table/figure generation, see
`docs/REPRODUCIBILITY.md`.

## Documentation

- `docs/README.md` - workspace documentation index and cleanup policy.
- `docs/ARCHITECTURE.md` - code framework, module targets, and data flow.
- `docs/REPRODUCIBILITY.md` - build, smoke-test, experiment, and paper-asset
  reproduction workflow.
- `link_interval_envelope/README.md` - envelope package usage.
- `lect_database/README.md` - database and online-cache package usage.
- `safe_box_forest/README.md` - planner package usage.

## Citation And License

RapidBoxForest is released under the MIT License; see `LICENSE`. If you use the
software in research, cite the repository metadata in `CITATION.cff` and the
associated TRO manuscript once it is published.

## Generated And Archived Files

Build directories, CMake caches, Python caches, local database caches, and
experiment outputs are generated files and should stay untracked. The private
development checkout may contain historical scripts and old manuscripts for
engineering reference, but the clean public export excludes those archives by
default. Paper-facing reproduction should use the current top-level
`experiments/` runners.

For a public release, prefer exporting a clean source tree instead of publishing
the full development history:

```bash
python3 scripts/export_public_release.py \
  --out-dir /tmp/RapidBoxForest-public \
  --force
python3 scripts/check_public_release.py \
  /tmp/RapidBoxForest-public \
  --run-smoke-dry-run
python3 scripts/check_release_readiness.py \
  --repo-root . \
  --public-tree /tmp/RapidBoxForest-public
```

The export keeps source code, public documentation, experiment runners, release
scripts, and metadata while excluding `paper/`, local outputs, caches, build
products, scratch workspaces, and historical archives by default.

To produce a checked source archive for upload or for initializing a new public
repository:

```bash
python3 scripts/package_public_release.py \
  --out-dir /tmp/RapidBoxForest-release \
  --repo-url https://github.com/<owner>/RapidBoxForest \
  --version v0.1.0 \
  --release-date YYYY-MM-DD \
  --strict-metadata \
  --force
```

This writes `RapidBoxForest-public.tar.gz` and a companion schema-v2 package
manifest with the archive SHA256, exported file counts, tar member count,
duplicate-entry count, `release_tools_checked`, and `cache_archives_checked`;
the package step also verifies that the archive matches the manifest and has no
duplicate tar entries. Add `--doi` when that identifier is available. Large
LECT caches are local/generated artifacts and are ignored by git; pass
`--cache-manifest` only when validating a separately prepared local cache
artifact bundle.

After the public repository URL is known, update citation metadata before the
final strict release check:

```bash
python3 scripts/set_public_release_urls.py \
  --repo-url https://github.com/<owner>/RapidBoxForest
```

To initialize a fresh public git repository from the clean export:

```bash
python3 scripts/init_public_repository.py \
  --out-dir /tmp/RapidBoxForest-public-git \
  --repo-url https://github.com/<owner>/RapidBoxForest \
  --version v0.1.0 \
  --release-date YYYY-MM-DD \
  --strict-metadata \
  --force
```

When `--repo-url` is provided, the script stamps citation metadata, refreshes
`PUBLIC_RELEASE_MANIFEST.json`, runs strict citation validation, and stages the
exported files. Add `--doi` when that metadata is available. Add `--commit`
after configuring git identity locally if you want it to create the initial
commit.

After building Python bindings from the exported tree into an out-of-tree build
directory, the same checker can validate executable smoke tests:

```bash
PYBIND11_CMAKE_DIR="$(python3 -c 'import pybind11; print(pybind11.get_cmake_dir())')"
cmake -S /tmp/RapidBoxForest-public \
  -B /tmp/RapidBoxForest-python-smoke-build \
  -DRBF_BUILD_ENVELOPE=ON \
  -DRBF_BUILD_LECT_DATABASE=OFF \
  -DRBF_BUILD_SBF=OFF \
  -DRBF_BUILD_TESTS=OFF \
  -DRBF_WITH_PYTHON=ON \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  "-DCMAKE_PREFIX_PATH=${PYBIND11_CMAKE_DIR}"
cmake --build /tmp/RapidBoxForest-python-smoke-build -j"$(nproc)"

python3 /tmp/RapidBoxForest-public/scripts/check_public_release.py \
  /tmp/RapidBoxForest-public \
  --pythonpath /tmp/RapidBoxForest-python-smoke-build/python \
  --check-python-extension \
  --run-smoke-execute
```

If a XeLaTeX-capable TeX distribution is available, manuscript compilation is
checked from the private development checkout, where `paper/` is present:

```bash
cd paper
latexmk -xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
```
