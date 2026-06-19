# Reproducibility Guide

This guide describes the paper-facing workflow for rebuilding the code,
running smoke checks, regenerating experiment artifacts, and reproducing the
tables and figures used by `paper/sbf_tro_2026.tex`.

## Scope

The development repository intentionally separates three kinds of artifacts:

1. source code, experiment runners, generated paper assets, and manifests that
   can be versioned in the development checkout;
2. local build products and Python/LaTeX intermediates, which are ignored;
3. large experiment outputs and LECT evidence caches under `outputs/`, which
   are reproducible but not suitable for normal git history.

The source of truth for generated paper assets is
`paper/generated/tro_table_generation_manifest.json`. It records source
artifact paths, hashes, selected rows, and table/figure provenance.

For public release, export a clean source tree rather than publishing the full
development repository history. The public source tree intentionally omits the
`paper/` directory; manuscript sources and generated paper assets are validated
from the private development checkout.

```bash
python3 scripts/export_public_release.py \
  --out-dir /tmp/RapidBoxForest-public \
  --force
```

The export is allowlist-based. It keeps the current modules, paper-facing
experiments, documentation, license, citation metadata, and release scripts. It
excludes `paper/`, local outputs, LECT caches, build products, scratch
workspaces, and historical archives by default.

After exporting, run the release-readiness audit:

```bash
python3 scripts/check_release_readiness.py \
  --repo-root . \
  --public-tree /tmp/RapidBoxForest-public
```

For a lightweight regression check of the release helpers themselves, run:

```bash
python3 scripts/self_test_release_tools.py
```

This creates a temporary fake cache artifact, verifies the cache packager and
checker, and confirms that key negative cases fail as expected without using
real LECT caches.

Use strict mode only after the public repository URL is available. If the
private development checkout keeps template citation metadata, run strict
readiness from the initialized public git tree that was created by
`init_public_repository.py`. Strict mode also verifies that all files listed in
`PUBLIC_RELEASE_MANIFEST.json` are git-tracked:

```bash
python3 /tmp/RapidBoxForest-public-git/scripts/check_release_readiness.py \
  --repo-root /tmp/RapidBoxForest-public-git \
  --public-tree /tmp/RapidBoxForest-public-git \
  --package-manifest /tmp/RapidBoxForest-release/RapidBoxForest-public.package.json \
  --strict
```

To create a checked source archive from the same export policy:

```bash
python3 scripts/package_public_release.py \
  --out-dir /tmp/RapidBoxForest-release \
  --repo-url https://github.com/<owner>/RapidBoxForest \
  --version v0.1.0 \
  --release-date YYYY-MM-DD \
  --strict-metadata \
  --force
```

The package step writes `RapidBoxForest-public.tar.gz` and
`RapidBoxForest-public.package.json`; the latter records the archive SHA256,
public manifest SHA256, exported source-file count, tar member count,
duplicate-entry count, and whether checks were run. The package step also
validates the archive against the package manifest, including archive SHA256,
tar member uniqueness, exported file counts, and optional cache-manifest SHA256.
The source package manifest uses schema version 2, which requires explicit
`release_tools_checked` and `cache_archives_checked` fields. It records
`release_tools_checked=true` when the exported tree's release helper self-tests
were run.
The `--strict-metadata` flag requires a real repository URL, version, and
release date for final release packages. Add `--doi` when that identifier is
available; the package step refreshes `PUBLIC_RELEASE_MANIFEST.json` after
writing citation metadata and runs strict citation validation when `--repo-url`
is provided. Large LECT caches are local/generated artifacts by default; omit
`--cache-manifest` and `--cache-archive-dir` for the normal source release. If a
separate local cache bundle is being checked, pass `--cache-manifest` and
optionally `--cache-archive-dir` to validate and record its SHA256 metadata.

To check an existing package independently:

```bash
python3 scripts/check_public_package.py \
  /tmp/RapidBoxForest-release/RapidBoxForest-public.package.json \
  --require-release-tools-checked \
  --strict-metadata
```

After creating the public repository, write its URL into the citation metadata:

```bash
python3 scripts/set_public_release_urls.py \
  --repo-url https://github.com/<owner>/RapidBoxForest
```

If a DOI has already been minted for the software archive, add it in the same
step:

```bash
python3 scripts/set_public_release_urls.py \
  --repo-url https://github.com/<owner>/RapidBoxForest \
  --doi 10.xxxx/zenodo.xxxxxxx
```

To create a fresh public git repository directory from the clean export:

```bash
python3 scripts/init_public_repository.py \
  --out-dir /tmp/RapidBoxForest-public-git \
  --repo-url https://github.com/<owner>/RapidBoxForest \
  --version v0.1.0 \
  --release-date YYYY-MM-DD \
  --strict-metadata \
  --force
```

The script writes the repository URL into citation metadata when `--repo-url`
is provided, refreshes `PUBLIC_RELEASE_MANIFEST.json` for the final tree,
validates the exported tree with strict citation checks, initializes `main`,
stages all files, and optionally sets `origin`. The `--strict-metadata` flag
requires a repository URL, version, and release date. Add `--doi` when that
identifier is available. It does not push. Add `--commit` only after local git
identity is configured.

## Environment

The core workspace requires a C++20 compiler and CMake. The Python experiment
scripts use the standard library plus the packages listed in
`requirements-experiments.txt`.

Recommended system packages for a source build:

```bash
sudo apt-get install cmake g++ libeigen3-dev nlohmann-json3-dev pybind11-dev
```

or, in a conda environment:

```bash
conda env create -f environment.yml
conda activate rapidboxforest
```

For an existing Python environment, install the Python-only experiment
dependencies with:

```bash
python3 -m pip install -r requirements-experiments.txt
```

Then build the C++ workspace:

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

For paper-facing runs, use a single experiment process at a time and the
standard eight-thread budget:

```bash
export OMP_NUM_THREADS=8
export OPENBLAS_NUM_THREADS=8
export MKL_NUM_THREADS=8
export NUMEXPR_NUM_THREADS=8
export VECLIB_MAXIMUM_THREADS=8
```

Final OMPL simplification is fixed at `0.01` seconds for reported methods.
Planning time excludes final audit time.

## Smoke Validation

Start with dry runs in a disposable output directory:

```bash
python3 experiments/run_tro2026.py \
  --phase smoke \
  --dry-run \
  --out-dir outputs/repro_smoke
```

Executing the Exp.1/Exp.2 smoke checks requires the
`link_interval_envelope` Python module to be importable. Build the Python
extension or install the package before running `--execute`:

```bash
cmake -S . -B /tmp/RapidBoxForest-python-smoke-build \
  -DRBF_BUILD_ENVELOPE=ON \
  -DRBF_BUILD_LECT_DATABASE=OFF \
  -DRBF_BUILD_SBF=OFF \
  -DRBF_BUILD_TESTS=OFF \
  -DRBF_WITH_PYTHON=ON
cmake --build /tmp/RapidBoxForest-python-smoke-build -j"$(nproc)"

python3 experiments/run_tro2026.py \
  --phase smoke \
  --execute \
  --out-dir outputs/repro_smoke

python3 scripts/export_public_release.py \
  --out-dir /tmp/RapidBoxForest-public \
  --force

python3 scripts/check_public_release.py \
  /tmp/RapidBoxForest-public \
  --pythonpath /tmp/RapidBoxForest-python-smoke-build/python \
  --check-release-tools \
  --check-python-extension \
  --run-smoke-execute
```

In offline or restricted-network environments, pass
`-DFETCHCONTENT_FULLY_DISCONNECTED=ON` to make CMake fail fast unless Eigen,
nlohmann_json, and pybind11 are already installed and discoverable.

The default public smoke suite runs the lightweight core mechanism checks
(`exp01`, `exp02`). It deliberately avoids heavy external baselines, random
catalog sweeps, and cache-dependent planning rows. To smoke-test a specific
later experiment after preparing its dependencies, pass `--only`, for example
`--only exp04`.

To regenerate paper assets from an existing artifact directory:

```bash
python3 experiments/generate_tro2026_paper_assets.py \
  --out-dir outputs/new_experiments/tro2026 \
  --paper-dir paper
```

Validate the generated paper provenance manifest from a source checkout:

```bash
python3 scripts/check_paper_result_sources.py \
  --manifest paper/generated/tro_table_generation_manifest.json \
  --repo-root . \
  --verify-local
```

This check verifies the generated paper assets, source-artifact references,
hash syntax, and any locally present source CSV/JSON files. For a complete
artifact bundle that includes `outputs/`, make the check strict:

```bash
python3 scripts/check_paper_result_sources.py \
  --manifest paper/generated/tro_table_generation_manifest.json \
  --repo-root . \
  --verify-local \
  --require-local \
  --require-source-hashes
```

Compile the manuscript with:

```bash
cd paper
latexmk -xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
```

Manuscript compilation is verified from the development checkout, where
`paper/` is present:

```bash
cd paper
latexmk -xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
```

## Full Experiment Workflow

The unified dispatcher is the preferred public entry point:

```bash
python3 experiments/run_tro2026.py \
  --phase paper \
  --execute \
  --out-dir outputs/new_experiments/tro2026
```

Long random-scene experiments must consume a saved catalog. Generate or verify
the catalog before a full run:

```bash
python3 experiments/common/generate_random_scene_catalog.py \
  --out outputs/new_experiments/tro2026/catalogs/random_scene_catalog.json \
  --mode generate

python3 experiments/run_tro2026.py \
  --phase paper \
  --only exp06,exp07 \
  --scene-catalog outputs/new_experiments/tro2026/catalogs/random_scene_catalog.json \
  --scene-catalog-mode verify \
  --execute \
  --out-dir outputs/new_experiments/tro2026
```

Large LECT caches are generated under `outputs/` and are intentionally not
tracked or synchronized to the public git remote. The Exp.4 Shelf+IIWA
warm-cache artifact is capped at canonical depth 23. If a cache is missing,
regenerate it with the experiment-specific cache builder. Keep local cache
paths, archives, and checksum manifests outside git.

The public repository includes `docs/cache_artifacts.example.json` as the
template for optional local cache artifact checks. It records the artifact id, robot,
envelope, endpoint source, canonical depth, split schedule, expected unpack
path, rebuild command, archive checksum, and unpacked-directory checksum. Keep
the example file in git with `TODO` placeholders; create a filled manifest only
when you need to validate a local cache bundle.

Validate the template from a source checkout with:

```bash
python3 scripts/check_cache_artifacts.py \
  docs/cache_artifacts.example.json \
  --allow-placeholders
```

After regenerating the real caches, validate the filled manifest and expected
unpack paths with:

```bash
python3 scripts/check_cache_artifacts.py \
  path/to/cache_artifacts.json \
  --repo-root . \
  --archive-dir path/to/local/cache_archives \
  --verify-local \
  --write-directory-sha256 outputs/cache_directory_sha256.json
```

The checker intentionally fails on `TODO` fields unless `--allow-placeholders`
is passed. Filled manifests must use HTTPS archive URLs, plain archive filenames
such as `.tar.gz`, `.tgz`, or `.tar.zst`, positive byte sizes, and exact
64-character lowercase SHA256 values. When `--archive-dir` is provided, the
checker validates each local archive file against `archive.file_name`,
`archive.size_bytes`, and `archive.sha256`. When `--verify-local` is enabled,
the checker also recomputes the unpacked cache directory SHA256 and rejects
manifests whose `unpacked.directory_sha256` does not match the local directory
contents.

To package local cache directories and create a filled manifest for local
validation:

```bash
python3 scripts/package_cache_artifacts.py \
  docs/cache_artifacts.example.json \
  --repo-root . \
  --out-dir outputs/cache_artifacts \
  --artifact-id exp04_iiwa_d23_aafk_support_hull \
  --force
```

This creates one `.tar.gz` per unique selected cache directory and writes
`outputs/cache_artifacts/cache_artifacts.json`. If two manifest entries share
the same `expected_unpack_path`, they intentionally reuse the same archive file
and checksum instead of duplicating a large cache bundle. The cache packager uses
deterministic gzip level 1 by default because these LECT bundles are large; pass
`--gzip-compresslevel` if a smaller archive is more important than packaging
time. Without `--url-base`, archive URLs remain `TODO-upload-url`; this is fine
for local-only archives when validating with `--allow-placeholders`.

For local archive bundles where size matters, prefer zstd:

```bash
python3 scripts/package_cache_artifacts.py \
  docs/cache_artifacts.example.json \
  --repo-root . \
  --out-dir outputs/cache_artifacts \
  --archive-format tar.zst \
  --zstd-level 10 \
  --force
```

The checker accepts `.tar.gz`, `.tgz`, and `.tar.zst` cache archives. The
packager keeps gzip as the default for portability; `.tar.zst` requires the
`zstd` command-line tool.

If you later decide to publish a separate cache bundle outside git, rewrite the
manifest URLs without re-running the expensive archive step:

```bash
python3 scripts/fill_cache_artifact_urls.py \
  outputs/cache_artifacts/cache_artifacts.json \
  --url-base https://example.org/RapidBoxForest/cache \
  --out outputs/cache_artifacts/cache_artifacts.release.json \
  --force
```

## Active Paper Assets

The current main generated assets are tracked in the private development
checkout and are not included in the public source export:

1. `paper/generated/tab_tro_endpoint_envelope.tex`
2. `paper/generated/tab_tro_link_envelope.tex`
3. `paper/generated/tab_tro_shelf_ablation.tex`
4. `paper/generated/tab_tro_shelf_cross_algorithm.tex`
5. `paper/generated/tab_tro_random_summary.tex`
6. `paper/generated/tab_tro_dynamic_update.tex`
7. `paper/generated/fig_tro_shelf_tradeoff.pdf`
8. `paper/generated/fig_tro_shelf_cross_tradeoff.pdf`
9. `paper/generated/fig_tro_random_tradeoff.pdf`
10. `paper/generated/tro_table_generation_manifest.json`

Paper-facing runs should use current runners under `experiments/`. Historical
module-local SBF experiment workflows have been removed from the active source
tree; archived scripts are not required by current tables or figures.
