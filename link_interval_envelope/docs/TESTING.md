# Testing

## Full Package Test

```bash
bash tests/run_all.sh
```

Environment overrides:

```bash
PYTHON_EXECUTABLE="$(command -v python3)" \
BUILD_DIR=build \
JOBS=8 \
bash tests/run_all.sh
```

## What the Tests Cover

- C++ robot JSON loading.
- IFK endpoint-iAABB computation.
- `LinkIAABB` envelope generation.
- `KDOP` and `SupportHull` envelope generation.
- Low-level FKState incremental endpoint equivalence against full recompute.
- IFK `IncrementalEnvelopeContext` equivalence against full recompute.
- CritSample candidate/DH-cache incremental equivalence against full recompute.
- Unchanged interval reuse for IFK FK extraction and CritSample endpoint cache.
- CritSample serial and parallel combo enumeration produce identical endpoint iAABBs.
- Endpoint diagnostics expose combo count, enumeration threads, changed dimension, auto threshold/chunk sizing, dirty candidates, `PreDH` rebuilds, and cache reuse.
- IFK and CritSample batch results match sequential one-shot results at 1 and 2 threads.
- Python one-shot API schema.
- Python `IncrementalEnvelopeComputer` IFK and CritSample incremental equivalence.
- Python `compute_envelope_batch` deterministic equivalence for IFK and CritSample.
- Endpoint-iAABB reuse path.
- CLI JSON and HTML output.

## Manual Build and Test

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIE_BUILD_TESTS=ON \
  -DLIE_WITH_PYTHON=ON \
  -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build -j
ctest --test-dir build --output-on-failure
```

When CMake reports the wrong Python version in an existing build directory, use a fresh build directory such as `build_py310` or remove the stale `CMakeCache.txt`.

## Python-only Test

After building:

```bash
export PYTHONPATH=build/python:python
python3 -m unittest discover \
  -s tests \
  -p 'test_python_api.py'
```

## CLI Smoke Test

```bash
export PYTHONPATH=build/python:python
python3 -m link_interval_envelope compute \
  --robot examples/data/2dof_planar.json \
  --intervals-json '[[-0.4, 0.4], [-0.2, 0.2]]' \
  --endpoint-source ifk \
  --env link_iaabb \
  --n-sub 4 \
  --out-json /tmp/lie.json \
  --out-html /tmp/lie.html
```

## IFK/CritSample Benchmark

```bash
export PYTHONPATH=build/python:python
python3 examples/benchmark_ifk_crit.py \
  --n-boxes 512 \
  --repeats 7 \
  --threads 4 \
  --envelope-type link_iaabb \
  --sources ifk critsample
```

The script reports median, p95, and speedup against a Python one-shot loop for IFK and CritSample paths.

## C++ Microbench

```bash
cmake --build build -j --target lie_microbench
build/lie_microbench \
  --robot examples/data/panda.json \
  --iterations 1000 \
  --repeats 7 \
  --threads 4
```

The default C++ microbench reports median/p95 timings for full FK, incremental-copy FK, in-place FK, scalar FK, an explicit scalar-FMA FK experiment, CritSample serial/parallel endpoints, `LinkIAABB`/`KDOP`/`SupportHull` envelope timings, combo count, thread count, auto threshold/chunk sizing, and changed-dimension distribution.

CritSample sweep mode scans interval scale, thread count, and requested parallel threshold (`0` means auto):

```bash
build/lie_microbench \
  --robot examples/data/panda.json \
  --iterations 400 \
  --repeats 5 \
  --threads 8 \
  --sweep
```

Sequence mode benchmarks one-shot recompute, stateful incremental contexts, and a CritSample context pool over random-walk-like interval sequences:

```bash
build/lie_microbench \
  --robot examples/data/panda.json \
  --repeats 5 \
  --threads 4 \
  --sequence-length 256 \
  --sequence
```
