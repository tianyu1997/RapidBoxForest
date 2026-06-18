# Link Interval Envelope

Self-contained link interval envelope package. The package does not include or
link against planner, LECT, SafeBoxForest, Drake, or OMPL code.

Given a robot model, a joint-interval box, endpoint-source settings, and an
envelope type, the package computes per-link envelopes plus normalized JSON and
optional HTML visualization artifacts. It also exposes the compatibility
`sbf::FKState` layout and a stateful context for repeated nearby interval
queries.

## Current Surface

- C++ static library target `link_interval_envelope::core` (backed by the
  concrete target `link_interval_envelope_core`).
- Public facade headers under `include/link_interval_envelope/`, with
  `api.h` as the one-stop include for new C++ callers.
- Compatibility `sbf`/`rbf` types: `Robot`, `Interval`, `FKState`,
  `JointLimits`, `EndpointSourceConfig`, `EnvelopeTypeConfig`, and `GcpcCache`.
- Endpoint sources `IFK`, `CritSample`, `Analytical`, `GCPC`, and `MC`.
- Envelope types `LinkIAABB`, `KDOP`, and `SupportHull`.
- Stateful C++ helper `link_interval_envelope::IncrementalEnvelopeContext` for
  neighboring boxes.
- C++/Python batch API `compute_envelope_batch(...)` for independent boxes.
- Python package `link_interval_envelope` with one-shot, batch, endpoint-reuse,
  incremental, JSON-write, CLI, and visualization helpers.
- Normalized result schema `link_interval_envelope.v1`.

## Layout

```text
include/link_interval_envelope/      public facade and package-level API
include/sbf/                         compatibility and implementation headers
src/                                 standalone implementation
python/link_interval_envelope/       Python facade, CLI, visualization helpers
examples/data/                       example robot JSON files
tests/                               C++ tests, Python tests, wrapper script
docs/                                API, incremental FK, schema, and testing notes
```

## Build

Configure from the `link-interval-envelope/` directory:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIE_BUILD_TESTS=ON \
  -DLIE_WITH_PYTHON=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

The Python package is copied to `${build}/python/link_interval_envelope`.

If your machine has multiple Python installations, pass the interpreter
explicitly so the extension ABI matches the Python used at runtime:

```bash
cmake -S . -B build \
  -DLIE_WITH_PYTHON=ON \
  -DPython3_EXECUTABLE=/path/to/python
```

If an existing build directory still resolves the wrong Python, configure into a
fresh build directory or delete the stale CMake cache before rebuilding.

## Test Everything

```bash
tests/run_all.sh
```

The wrapper script configures the standalone project, builds the C++ library and
Python extension, runs CTest, runs the Python unittest suite, and performs a
CLI JSON/HTML smoke test.

## Python Usage

The high-level Python facade accepts a robot path, a `pathlib.Path`, a JSON-like
mapping, or an existing bound `Robot` object.

```python
from link_interval_envelope import compute_envelope, write_json
from link_interval_envelope.visualize import save_html

result = compute_envelope(
    "examples/data/2dof_planar.json",
    [[-0.4, 0.4], [-0.2, 0.2]],
    endpoint_source="ifk",
    envelope_type="link_iaabb",
    n_subdivisions=4,
)

write_json(result, "lie_2dof.json")
save_html(result, "lie_2dof.html")
```

When the endpoint evidence is already available, skip endpoint generation and
reuse it directly:

```python
from link_interval_envelope import compute_from_endpoint_iaabbs

envelope_only = compute_from_endpoint_iaabbs(
    "examples/data/2dof_planar.json",
    result["endpoint"]["endpoint_iaabbs_flat"],
  envelope_type="support_hull",
)
```

For `endpoint_source="gcpc"`, load a `GcpcCache` and pass it through
`gcpc_cache=...`.

## Incremental Usage

Use `IncrementalEnvelopeComputer` when repeatedly evaluating neighboring
interval boxes. The first call builds the source state. Later calls can pass
`changed_dim`, or omit it when exactly one interval dimension changed and the
class can infer it.

```python
from link_interval_envelope import IncrementalEnvelopeComputer

computer = IncrementalEnvelopeComputer(
    "examples/data/2dof_planar.json",
    endpoint_source="ifk",
    envelope_type="link_iaabb",
    n_subdivisions=4,
)

computer.compute([[-0.4, 0.4], [-0.2, 0.2]])
result = computer.compute([[-0.4, 0.4], [-0.1, 0.3]], changed_dim=1)
print(result["incremental"])
```

The optimized stateful endpoint paths are currently `IFK` and `CritSample`.
Analytical, GCPC, and MC still work in the stateful context, but keep their
one-shot endpoint behavior.

## Parallel Batch Usage

For independent interval boxes, use the batch API. The current optimized batch
path supports `ifk` and `critsample` endpoint sources.

```python
from link_interval_envelope import compute_envelope_batch

results = compute_envelope_batch(
    "examples/data/2dof_planar.json",
    [
        [[-0.4, 0.4], [-0.2, 0.2]],
        [[-0.4, 0.4], [-0.1, 0.3]],
    ],
    endpoint_source="critsample",
    envelope_type="link_iaabb",
    n_subdivisions=4,
    n_threads=4,
)
```

The native binding releases the Python GIL around long C++ compute sections so
multiple Python threads can continue making progress while C++ work is running.

## C++ Usage

```cpp
#include <link_interval_envelope/api.h>

namespace lie = link_interval_envelope;

lie::Robot robot = lie::Robot::from_json("2dof_planar.json");

lie::EndpointSourceConfig endpoint_config;
endpoint_config.source = lie::EndpointSource::IFK;

lie::EnvelopeTypeConfig envelope_config;
envelope_config.type = lie::EnvelopeType::LinkIAABB;
envelope_config.n_subdivisions = 4;

lie::IncrementalEnvelopeContext context(
    robot,
    endpoint_config,
    envelope_config);

context.compute({{-0.4, 0.4}, {-0.2, 0.2}});
auto next = context.compute({{-0.4, 0.4}, {-0.1, 0.3}}, 1);

auto batch = lie::compute_envelope_batch(
    robot,
    {{{-0.4, 0.4}, {-0.2, 0.2}}, {{-0.4, 0.4}, {-0.1, 0.3}}},
    endpoint_config,
    envelope_config,
    4);
```

The compatibility `rbf::FKState` struct keeps the workspace field layout, so
code that already targets those `FKState` fields remains source-compatible when
compiled against this standalone package. Do not link this package with another
independent library that provides the same `rbf` symbols in one target.

New C++ integrations should include `link_interval_envelope/api.h` and avoid
direct `sbf/*` includes unless they intentionally need a source-specific kernel,
FK internals, or an existing LECT/SBF compatibility wrapper.

## CLI

```bash
PYTHONPATH=build/python:python \
python -m link_interval_envelope compute \
  --robot examples/data/2dof_planar.json \
  --intervals-json '[[-0.4, 0.4], [-0.2, 0.2]]' \
  --endpoint-source ifk \
  --env link_iaabb \
  --n-sub 4 \
  --out-json lie_2dof.json \
  --out-html lie_2dof.html
```

The `compute` command accepts either `--intervals-json` / `--intervals-file` or
`--endpoint-iaabbs-file`. Additional public switches include:

- `--endpoint-threads` and `--parallel-min-combos` for CritSample control.
- `--gcpc-cache` for the GCPC endpoint source.
- `--include-voxels` with `none`, `centres`, or `bricks`.
- `--view` with `raw` or `inflated` for the HTML visualizer.

## Robot JSON

Robot files follow the workspace DH schema:

```json
{
  "name": "2dof_planar",
  "dh_params": [
    {"alpha": 0.0, "a": 0.0, "d": 0.0, "theta": 0.0, "type": "revolute"}
  ],
  "joint_limits": [[-3.14, 3.14]],
  "tool_frame": {"alpha": 0.0, "a": 1.0, "d": 0.0, "theta": 0.0},
  "link_radii": [0.05, 0.05]
}
```

Example robot JSON files live under `examples/data/`.

## Output Schema

The normalized Python and CLI result schema is `link_interval_envelope.v1` and
includes:

- `robot`: robot name, active link map, active radii, and midpoint link
  geometry.
- `intervals`: requested joint intervals when the caller provided them.
- `endpoint`: source, safety flag, shape metadata, and optional flat endpoint
  iAABBs.
- `diagnostics`: source-specific timing and reuse counters.
- `envelope`: raw and inflated link AABBs, per-link records, and optional grid
  metadata or voxel payloads.
- `timing_us`: endpoint, envelope, and total wall-clock microseconds.

Incremental calls add an `incremental` block with `changed_dim`,
`used_incremental_fk`, `used_source_incremental_state`, `reused_fk`, and
`reused_endpoint_cache`.

## More Documentation

- `docs/API.md`: C++ and Python API reference.
- `docs/INCREMENTAL_FK.md`: FKState compatibility and incremental workflow.
- `docs/OUTPUT_SCHEMA.md`: JSON result schema details.
- `docs/TESTING.md`: test matrix and commands.
- `DEVELOPMENT.md`: source boundary, Python ABI notes, and release checklist.
