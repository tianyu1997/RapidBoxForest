# Open-Source Release Checklist

This checklist tracks repository work required before making the code public.
The public source repository is source-only and intentionally does not include
the `paper/` directory. Manuscript sources and generated paper assets remain in
the private development checkout and can be validated there.

## Recommended Release Model

Do not publish the full development checkout and git history directly. Use a
new public repository populated from `scripts/export_public_release.py`. Keep
this development repository private for historical experiments, scratch notes,
local paths, and discarded implementation attempts.

## Completed In The Current Repository

- Root MIT `LICENSE` exists.
- Root `CITATION.cff` exists.
- Root `README.md` documents the workspace layout, build commands, experiment
  entry points, generated-file policy, license, and citation entry.
- Root `environment.yml` provides a conda-forge build-and-experiment
  environment for source users.
- `docs/REPRODUCIBILITY.md` documents smoke/full experiment workflows, cache
  policy, local manuscript-asset validation, and the public source export.
- `docs/cache_artifacts.example.json` records the optional local LECT cache
  artifact schema and expected unpack paths for paper-facing warm caches.
- `.gitignore` excludes build products, experiment outputs, local LECT caches,
  Python caches, and common LaTeX intermediates.
- Current paper asset generation no longer requires the removed
  `tab_tro_lect_performance.tex` asset.
- The registered RBF profile name in experiment principles matches the current
  defaults.
- The public smoke dispatcher default is limited to the fast core checks
  `exp01,exp02`; heavier/cache-dependent experiments must be requested
  explicitly with `--only`.
- `scripts/export_public_release.py` exports an allowlisted public source tree
  and excludes local outputs, caches, build trees, and historical archives by
  default. It fails if a forbidden prototype sidecar such as
  `improve_workspace`, `rbf_v2`, `sbf_v2`, or `sbf-standalone` exists, because
  optimized implementations must be migrated into the main modules rather than
  kept in a parallel workspace.
  The generated `PUBLIC_RELEASE_MANIFEST.json` records every exported file and
  its SHA256 hash.
- `scripts/check_public_release.py` validates the exported tree, checks for
  forbidden generated/history paths, scans text files for local absolute paths,
  checks for references to excluded historical entry points, checks local
  Markdown links, verifies the export manifest and per-file SHA256 hashes,
  validates the cache-artifact template, and can run dispatcher smoke dry-run,
  verify release helper scripts, or, after Python bindings are built, smoke
  execute. If a nonstandard tree intentionally includes `paper/`, it can also
  validate paper assets and compile the manuscript.
- `scripts/check_cache_artifacts.py` validates filled cache-artifact manifests
  and can verify local cache archives plus expected unpack directories.
- `scripts/package_cache_artifacts.py` packages selected local LECT cache
  directories and writes a filled cache artifact manifest with archive and
  unpacked-directory SHA256 values.
- `scripts/fill_cache_artifact_urls.py` can rewrite archive URLs in an already
  packaged cache-artifact manifest if a separate cache bundle is published
  outside git.
- `scripts/check_paper_result_sources.py` validates
  `paper/generated/tro_table_generation_manifest.json`, generated paper asset
  hashes, source-artifact references, and optional local `outputs/` artifacts.
- `scripts/check_release_readiness.py` runs the source-tree final audit:
  required release files, public-manifest files tracked by git in strict mode,
  forbidden source sidecars, tracked generated files, citation metadata, stale
  historical references, cache metadata, optional local paper provenance when
  `paper/` is present, optional public tree validation, and optional public
  source package validation.
- `scripts/check_public_package.py` validates the source archive against
  `RapidBoxForest-public.package.json`, including archive SHA256,
  `PUBLIC_RELEASE_MANIFEST.json`, file counts, duplicate tar entries, and
  optional cache-manifest SHA256.
- `scripts/package_public_release.py` exports, optionally writes final citation
  URL/DOI/version metadata, refreshes `PUBLIC_RELEASE_MANIFEST.json`, checks,
  runs strict citation validation when a repository URL is provided, and
  packages the public source tree into a deterministic `.tar.gz` with a
  companion SHA256 manifest, then validates the package manifest.
- `scripts/set_public_release_urls.py` writes the public repository URL and
  optional DOI into the root and module citation metadata before strict release.
- `scripts/init_public_repository.py` exports the clean tree, optionally writes
  the public repository URL into citation metadata, refreshes
  `PUBLIC_RELEASE_MANIFEST.json`, validates the final tree with strict citation
  checks, and initializes a fresh staged public git repository without carrying
  private development history.
- `scripts/self_test_release_tools.py` runs lightweight regression checks for
  cache artifact packaging and release-tool failure cases without requiring
  real LECT caches. `scripts/check_release_readiness.py --public-tree` runs
  this self-test inside the exported tree, so source-group and diagnostic API
  boundary checks are validated from the release artifact as well as the
  development checkout.
- `.github/workflows/ci.yml` defines the public CI path: C++ configure/build,
  CTest, public export self-check, Python-extension build, and smoke execute.

## Local Validation Evidence

Validated on 2026-06-18:

- Fresh configure completed after the public-export, cache-artifact, paper
  provenance, CI, and dependency-version changes:

  ```bash
  cmake -S . -B build-public-final-check \
    -DRBF_BUILD_ENVELOPE=ON \
    -DRBF_BUILD_LECT_DATABASE=ON \
    -DRBF_BUILD_SBF=ON \
    -DRBF_BUILD_TESTS=ON \
    -DRBF_WITH_PYTHON=OFF
  ```

- Fresh build completed:

  ```bash
  cmake --build build-public-final-check -j8
  ```

- CTest passed: 6/6 tests passed.

  ```bash
  ctest --test-dir build-public-final-check --output-on-failure
  ```

- Clean public export C++ configure, build, and CTest passed from an out-of-tree
  build directory. On this local machine, `nlohmann_json` was supplied from a
  previously fetched source directory because the system CMake package was not
  installed; the public CI installs `nlohmann-json3-dev` instead.

  ```bash
  python3 scripts/package_public_release.py \
    --out-dir /tmp/RapidBoxForest-release-cpp-verify \
    --force
  NLOHMANN_JSON_SOURCE_DIR=/path/to/nlohmann_json-src
  cmake -S /tmp/RapidBoxForest-release-cpp-verify/RapidBoxForest-public \
    -B /tmp/RapidBoxForest-release-cpp-verify/build-cpp-localdeps \
    -DRBF_BUILD_ENVELOPE=ON \
    -DRBF_BUILD_LECT_DATABASE=ON \
    -DRBF_BUILD_SBF=ON \
    -DRBF_BUILD_TESTS=ON \
    -DRBF_WITH_PYTHON=OFF \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${NLOHMANN_JSON_SOURCE_DIR}"
  cmake --build /tmp/RapidBoxForest-release-cpp-verify/build-cpp-localdeps -j8
  ctest --test-dir /tmp/RapidBoxForest-release-cpp-verify/build-cpp-localdeps \
    --output-on-failure
  ```

- Public smoke execute passed with the default fast-core selection:

  ```bash
  python3 experiments/run_tro2026.py \
    --phase smoke \
    --execute \
    --out-dir outputs/release_smoke_fast_final
  ```

  The generated dispatcher manifest is
  `outputs/release_smoke_fast_final/tro2026_smoke_manifest.json`. The run
  executed only `exp01,exp02` by design.

- Cache artifact template validation passed:

  ```bash
  python3 scripts/check_cache_artifacts.py \
    docs/cache_artifacts.example.json \
    --allow-placeholders
  ```

- Paper result source manifest validation passed:

  ```bash
  python3 scripts/check_paper_result_sources.py \
    --manifest paper/generated/tro_table_generation_manifest.json \
    --repo-root . \
    --verify-local
  ```

- Release readiness audit passed in template mode:

  ```bash
  python3 scripts/check_release_readiness.py \
    --repo-root . \
    --public-tree /tmp/RapidBoxForest-public
  ```

- Public source package generation passed in template mode:

  ```bash
  python3 scripts/package_public_release.py \
    --out-dir /tmp/RapidBoxForest-release \
    --force
  ```

  The archive SHA256 is recorded in `RapidBoxForest-public.package.json` next
  to the archive and printed by the package command. The package manifest also
  records the filled cache-artifact manifest SHA256 when `--cache-manifest` is
  provided. The source package manifest uses schema version 2, which requires
  explicit `release_tools_checked` and `cache_archives_checked` fields;
  `release_tools_checked=true` means the exported tree's release helper
  self-tests were run. `--cache-archive-dir` validates local cache archive files
  before packaging; the local directory path is not recorded in the public
  package metadata.

  The package script runs `scripts/check_public_package.py` automatically.
  Re-run it manually if the archive or package manifest is moved:

  ```bash
  python3 scripts/check_public_package.py \
    /tmp/RapidBoxForest-release/RapidBoxForest-public.package.json \
    --require-release-tools-checked
  ```

- Public git repository initialization passed in disposable directories,
  including the citation-URL stamping path:

  ```bash
  python3 scripts/init_public_repository.py \
    --out-dir /tmp/RapidBoxForest-public-git \
    --force

  # After the real public repository URL exists:
  python3 scripts/init_public_repository.py \
    --out-dir /tmp/RapidBoxForest-public-git-url-test \
    --repo-url https://github.com/<owner>/RapidBoxForest \
    --force
  ```

- Clean public export was generated with 271 source files and release self-check
  passed:

  ```bash
  python3 scripts/export_public_release.py \
    --out-dir /tmp/RapidBoxForest-public \
    --force
  python3 scripts/check_public_release.py \
    /tmp/RapidBoxForest-public \
    --check-release-tools \
    --run-smoke-dry-run
  ```

- The check passed over 272 files including `PUBLIC_RELEASE_MANIFEST.json`.
  It found no local absolute paths in text files and no `archive/`,
  `sbf_old/`, `outputs/`, `.sbf_lect_database/`, `__pycache__/`, or `tmp/`
  files in the clean exported tree. It also found no references to excluded
  historical entry points outside the release-script allowlist and no broken
  local Markdown links. It verified the export file list, per-file SHA256
  hashes, cache-artifact template validity, and release-tool self-tests. The
  source-only public export intentionally contains no `paper/` directory.

- Manuscript compilation is checked from the private development checkout,
  where `paper/` is present:

  ```bash
  cd paper
  latexmk -xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
  ```

- The public CI workflow is included in the exported tree and required by
  `scripts/check_public_release.py`. It installs Eigen, nlohmann_json,
  pybind11, builds C++ tests, exports the public tree, validates the public
  source package, and runs Python smoke execute from the clean export.

- Clean public export Python-extension configure now fails fast in offline mode
  when required C++ packages are missing:

  ```bash
  cmake -S /tmp/RapidBoxForest-public \
    -B /tmp/RapidBoxForest-python-smoke-offline-check \
    -DRBF_BUILD_ENVELOPE=ON \
    -DRBF_BUILD_LECT_DATABASE=OFF \
    -DRBF_BUILD_SBF=OFF \
    -DRBF_BUILD_TESTS=OFF \
    -DRBF_WITH_PYTHON=ON \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON
  ```

  Expected failure without installed packages:
  `nlohmann_json was not found and FetchContent is fully disconnected`.

- Clean public export Python-extension smoke build passed with an out-of-tree
  build directory. On this local machine, `nlohmann_json` was supplied from a
  previously fetched source directory because the system CMake package was not
  installed; the public CI installs `nlohmann-json3-dev` instead.

  ```bash
  NLOHMANN_JSON_SOURCE_DIR=/path/to/nlohmann_json-src
  PYBIND11_CMAKE_DIR="$(python3 -c 'import pybind11; print(pybind11.get_cmake_dir())')"
  cmake -S /tmp/RapidBoxForest-public \
    -B /tmp/RapidBoxForest-python-smoke-build \
    -DRBF_BUILD_ENVELOPE=ON \
    -DRBF_BUILD_LECT_DATABASE=OFF \
    -DRBF_BUILD_SBF=OFF \
    -DRBF_BUILD_TESTS=OFF \
    -DRBF_WITH_PYTHON=ON \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    "-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${NLOHMANN_JSON_SOURCE_DIR}" \
    "-DCMAKE_PREFIX_PATH=${PYBIND11_CMAKE_DIR}"
  cmake --build /tmp/RapidBoxForest-python-smoke-build -j8
  python3 /tmp/RapidBoxForest-public/scripts/check_public_release.py \
    /tmp/RapidBoxForest-public \
    --pythonpath /tmp/RapidBoxForest-python-smoke-build/python \
    --check-python-extension \
    --run-smoke-execute
  ```

## Required Before Public Release

1. Re-run a clean configure/build/test cycle from a fresh build directory after
   the final release patch:

   ```bash
   cmake -S . -B build-release-check \
     -DRBF_BUILD_ENVELOPE=ON \
     -DRBF_BUILD_LECT_DATABASE=ON \
     -DRBF_BUILD_SBF=ON \
     -DRBF_BUILD_TESTS=ON \
     -DRBF_WITH_PYTHON=OFF
   cmake --build build-release-check -j"$(nproc)"
   ctest --test-dir build-release-check --output-on-failure
   ```

2. Re-run the public smoke dispatcher after the final release patch. Dry-run
   mode should work from a source-only checkout:

   ```bash
   python3 experiments/run_tro2026.py \
     --phase smoke \
     --dry-run \
     --out-dir outputs/release_smoke
   ```

   Execute mode additionally requires the `link_interval_envelope` Python
   module to be built or installed:

   ```bash
   python3 experiments/run_tro2026.py \
     --phase smoke \
     --execute \
     --out-dir outputs/release_smoke
   ```

   The public smoke default is intentionally lightweight and runs only
   `exp01,exp02`. Run additional experiments with `--only` after preparing the
   required caches, catalogs, or optional external planners.

3. Regenerate paper assets in the private development checkout and compile the
   manuscript there:

   ```bash
   python3 experiments/generate_tro2026_paper_assets.py \
     --out-dir outputs/new_experiments/tro2026 \
     --paper-dir paper
   cd paper
   latexmk -xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
   ```

4. Verify that no generated caches, raw `outputs/`, Python bytecode, CMake
   build trees, or LaTeX intermediates are tracked, and that release-critical
   files have been added to git:

   ```bash
   git ls-files outputs .sbf_lect_database '**/__pycache__' '*.pyc' \
     '*.aux' '*.log' '*.xdv' '*.fdb_latexmk' '*.fls'

   python3 scripts/check_release_readiness.py \
     --repo-root . \
     --tracking-only
   ```

5. Decide which paper artifacts remain tracked in the private development
   repository. They are not part of the public source export:

   - manuscript source under `paper/`;
   - generated TeX tables and PDF/PNG figures under `paper/generated/`;
   - compiled `paper/sbf_tro_2026.pdf`, if the development repository should
     keep a rendered manuscript preview.

6. Large LECT caches are generated locally and must remain outside git. If a
   local exact-timing cache bundle is needed, use the cache packager to create
   archives and a filled manifest:

   ```bash
   python3 scripts/package_cache_artifacts.py \
     docs/cache_artifacts.example.json \
     --repo-root . \
     --out-dir outputs/cache_artifacts \
     --force
   ```

   The packager creates one archive per unique cache directory. Manifest entries
   that share `expected_unpack_path` reuse the same archive file and checksum,
   which avoids uploading duplicate IIWA cache bundles for Exp.4 and Exp.6.
   Use `--archive-format tar.zst --zstd-level 10` for large local cache bundles
   when smaller archive size is more important than gzip portability.

   If a separate cache bundle is later published outside git, rewrite URLs
   without rebuilding the archives:

   ```bash
   python3 scripts/fill_cache_artifact_urls.py \
     outputs/cache_artifacts/cache_artifacts.json \
     --url-base https://example.org/RapidBoxForest/cache \
     --out outputs/cache_artifacts/cache_artifacts.release.json \
     --force
   ```

   With `--verify-local`, the checker recomputes each unpacked cache directory
   hash and rejects stale or mismatched `unpacked.directory_sha256` entries:

   ```bash
   python3 scripts/check_cache_artifacts.py \
     path/to/cache_artifacts.json \
     --repo-root . \
     --archive-dir path/to/local/cache_archives \
     --verify-local
   ```

   Keep the filled manifest and cache archives outside git. The checked-in
   example manifest intentionally keeps TODO fields and is accepted by release
   readiness checks as a template when no `--cache-manifest` is supplied.

7. If a full paper artifact bundle is published with `outputs/`, validate the
   generated result-source manifest against the unpacked artifacts:

   ```bash
   python3 scripts/check_paper_result_sources.py \
     --manifest paper/generated/tro_table_generation_manifest.json \
     --repo-root . \
     --verify-local \
     --require-local \
     --require-source-hashes
   ```

8. Update repository URLs in `CITATION.cff` and module-level citation files
   after the public remote is created:

   ```bash
   python3 scripts/set_public_release_urls.py \
     --repo-url https://github.com/<owner>/RapidBoxForest
   ```

   Add `--doi ...` if a software archive DOI has already been minted.

9. Confirm that archived scripts are clearly non-paper-facing and cannot be
   confused with current experiment entry points.

10. Review the worktree manually before release:

   ```bash
   git status --short
   git diff --stat
   ```

11. Create the public release tree and run the same smoke checks from that
    tree before pushing it to a new public remote:

    ```bash
    python3 scripts/export_public_release.py \
      --out-dir /tmp/RapidBoxForest-public \
      --force
    python3 scripts/check_public_release.py \
      /tmp/RapidBoxForest-public \
      --check-release-tools \
      --run-smoke-dry-run
    ```

12. Create a checked source archive for upload or public-repo initialization:

    ```bash
    python3 scripts/package_public_release.py \
      --out-dir /tmp/RapidBoxForest-release \
      --repo-url https://github.com/<owner>/RapidBoxForest \
      --version v0.1.0 \
      --release-date YYYY-MM-DD \
      --strict-metadata \
      --force
    ```

13. Initialize a fresh public git repository directory from the clean export:

    ```bash
    python3 scripts/init_public_repository.py \
      --out-dir /tmp/RapidBoxForest-public-git \
      --repo-url https://github.com/<owner>/RapidBoxForest \
      --version v0.1.0 \
      --release-date YYYY-MM-DD \
      --strict-metadata \
      --force
    ```

    With `--repo-url`, this command updates citation metadata, refreshes the
    public manifest, and runs strict citation validation before staging.
    `--strict-metadata` requires a repository URL, version, and release date.
    Add `--doi` when that identifier is available. Inspect the staged tree, then
    commit and push from that directory after the public remote has been
    created.

14. Validate Python experiment smoke from the clean export using an out-of-tree
    Python-extension build:

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

15. After the public repository URL is available, run strict release readiness
    from the initialized public git tree. This assumes citation metadata was
    written by `init_public_repository.py`, and strict mode verifies that all
    files listed in `PUBLIC_RELEASE_MANIFEST.json` are git-tracked:

    ```bash
    python3 /tmp/RapidBoxForest-public-git/scripts/check_release_readiness.py \
      --repo-root /tmp/RapidBoxForest-public-git \
      --public-tree /tmp/RapidBoxForest-public-git \
      --package-manifest /tmp/RapidBoxForest-release/RapidBoxForest-public.package.json \
      --strict
    ```

## Notes

The current public workflow intentionally treats `outputs/` and large LECT
evidence caches as reproducible external artifacts. They should not be added to
normal source-control history.
