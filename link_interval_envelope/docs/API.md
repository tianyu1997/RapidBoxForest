# API Reference

This package has two public layers:

- low-level C++ types and compute kernels in namespace `sbf`
- package-level helpers in namespace `link_interval_envelope`

The Python package `link_interval_envelope` mirrors the common one-shot,
incremental, batch, and endpoint-reuse workflows.

## Public Entry Points

- CMake target: `link_interval_envelope::core`
- Low-level headers under `include/sbf/`
- Package-level headers:
  `include/link_interval_envelope/incremental_context.h` and
  `include/link_interval_envelope/batch.h`
- Python package: `link_interval_envelope`

## Low-Level C++ Types (`sbf`)

### `sbf::Interval`

Represents one joint interval with fields `lo` and `hi` plus helpers such as
`width()`, `center()`, and `empty()`.

### `sbf::Robot`

Loaded from JSON with `sbf::Robot::from_json(path)`. Important methods include:

- `name()`
- `n_joints()`
- `n_active_links()`
- `joint_limits()`
- `active_link_map()`
- `active_link_radii()`
- `has_tool()`
- `fingerprint()`

### `sbf::FKState`

Stores interval-FK prefix transforms and per-joint matrices. The public layout
matches the current v6 field layout:

- `prefix_lo[MAX_TF][16]`, `prefix_hi[MAX_TF][16]`
- `joints_lo[MAX_JOINTS][16]`, `joints_hi[MAX_JOINTS][16]`
- `n_tf`, `n_jm`, `valid`

Core FK helpers:

- `compute_fk_full(robot, intervals)`
- `compute_fk_incremental(parent_fk, robot, intervals, changed_dim)`
- `update_fk_inplace(fk, robot, intervals, changed_dim)`
- `extract_endpoint_iaabbs(...)`
- `extract_link_aabbs(...)`

### Endpoint Sources

Configure with `sbf::EndpointSourceConfig`:

- `source`: `IFK`, `CritSample`, `Analytical`, `GCPC`, `MC`, or `HIFK`
- `n_samples_crit`
- `n_threads`: internal CritSample enumeration threads (`1` keeps serial
  behavior, `0` uses hardware auto-detect)
- `parallel_min_combos`: minimum CritSample combo count before internal
  parallelism (`<=0` selects the built-in automatic threshold)
- `max_phase_analytical`
- `bypass_narrow_skip`
- `gcpc_match_analytical`
- `gcpc_cache`
- `hifk_max_depth` (`<0` selects the interval-aware auto-depth schedule; the
  Python facade and CLI also accept `"auto"`)
- `hifk_n_threads`
- `hifk_vol_ratio_thresh`

The standalone package also exposes `sbf::recommend_hifk_depth(intervals,
max_depth_cap=5)` as a width-only fallback and
`sbf::recommend_hifk_depth(robot, intervals, max_depth_cap=5)` as the
robot-aware schedule used by HIFK auto-depth. The Python facade mirrors this as
`link_interval_envelope.recommend_hifk_depth(...)`: pass only intervals for the
fallback rule, or pass `(robot, intervals)` to score the first split using the
AA-FK volume-min policy. The current heuristic keeps IFK for narrow boxes,
defaults to HIFK-3 for most nontrivial boxes, and escalates to HIFK-5 only
when the box is globally wide and the split-policy-aware first split still
shows large absolute slack.

Python bindings also expose `EndpointSourceConfig.set_gcpc_cache(cache)`.

Compute entry point:

```cpp
sbf::EndpointIAABBResult compute_endpoint_iaabb(
    const sbf::Robot& robot,
    const std::vector<sbf::Interval>& intervals,
    const sbf::EndpointSourceConfig& config,
    sbf::FKState* fk = nullptr,
    int changed_dim = -1);
```

`IFK` is the unsplit AA-FK baseline and `HIFK` is the hierarchical AA-FK
variant. HIFK now uses a shared depth-synchronous split schedule: each depth
level commits one split dimension on first visit via the AA-FK volume-min rule,
and all later nodes at that depth reuse the same dimension. Both certified AA
sources currently use full endpoint recomputation inside the stateful wrappers.
For `CritSample`, the result reports the effective thread count, combo count,
chunk sizing, dirty candidate count, `PreDH` rebuild count, and whether the
endpoint cache was reused.

### Envelope Types

Configure with `sbf::EnvelopeTypeConfig`:

- `type`: `LinkIAABB`, `KDOP`, or `SupportHull`
- `n_subdivisions`

Compute entry point:

```cpp
sbf::LinkEnvelope compute_link_envelope(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    const sbf::EnvelopeTypeConfig& config);
```

`link_iaabbs` are the raw non-radius-inflated boxes. Grid modes use the active
link radii during envelope construction. `KDOP` and `SupportHull` select tighter
no-grid predicates for downstream collision checks while preserving the same
high-level entry point.

### `sbf::GcpcCache`

`GcpcCache` is the public container for the GCPC endpoint source. Python binds:

- `GcpcCache.load(path)`
- `cache.save(path)`
- `cache.n_points()`
- `cache.n_dims()`
- `cache.empty()`

## Package-Level C++ API (`link_interval_envelope`)

### Stateful incremental context

`link_interval_envelope::IncrementalEnvelopeContext` stores a `Robot`, endpoint
config, envelope config, last intervals, and current `sbf::FKState`.

```cpp
link_interval_envelope::IncrementalEnvelopeContext context(
    robot,
    endpoint_config,
    envelope_config);

auto first = context.compute(parent_intervals);
auto second = context.compute(child_intervals, changed_dim);
```

The returned `IncrementalEnvelopeResult` contains:

- `endpoint`
- `envelope`
- `endpoint_time_us`
- `envelope_time_us`
- `changed_dim`
- `used_incremental_fk`
- `used_source_incremental_state`
- `reused_fk`
- `reused_endpoint_cache`

The optimized stateful endpoint path is `CritSample`. `IFK`, `HIFK`,
Analytical, GCPC, and MC keep their one-shot endpoint behavior inside the
stateful context.

### Batch API

`link_interval_envelope/batch.h` provides parallel batch computation for
independent boxes:

```cpp
std::vector<link_interval_envelope::EnvelopeBatchResult> results =
    link_interval_envelope::compute_envelope_batch(
        robot,
        interval_boxes,
        endpoint_config,
        envelope_config,
        n_threads);
```

Important batch helpers:

- `resolve_thread_count(requested_threads, n_items)`
- `EnvelopeBatchResult`
- `compute_envelope_batch(...)`

The current batch entry point supports `EndpointSource::IFK`,
`EndpointSource::CritSample`, and `EndpointSource::HIFK`. Only `CritSample`
has a specialized reuse path; `IFK` and `HIFK` use the generic one-shot
endpoint path.

## Python Package: `link_interval_envelope`

Top-level exports include:

- bound low-level types: `Interval`, `JointLimits`, `Robot`, `EndpointSource`,
  `EnvelopeType`, `EndpointSourceConfig`, `EnvelopeTypeConfig`,
  `GcpcCache`, `FKState`
- facade helpers: `load_robot`, `make_intervals`, `make_endpoint_config`,
  `make_envelope_config`, `write_json`
- compute helpers: `compute_envelope`, `compute_envelope_batch`,
  `compute_from_endpoint_iaabbs`
- stateful helper: `IncrementalEnvelopeComputer`

### One-shot compute

```python
from link_interval_envelope import compute_envelope

result = compute_envelope(
    "examples/data/2dof_planar.json",
    [[-0.4, 0.4], [-0.2, 0.2]],
    endpoint_source="critsample",
    endpoint_threads=4,
    parallel_min_combos=0,
    envelope_type="link_iaabb",
    n_subdivisions=4,
    include_endpoint_iaabbs=True,
)
```

Public keyword arguments cover:

- endpoint source selection and tuning (`endpoint_source`, `n_samples_crit`,
  `endpoint_threads`, `parallel_min_combos`, `max_phase_analytical`,
  `bypass_narrow_skip`, `gcpc_match_analytical`, `hifk_max_depth`,
  `hifk_n_threads`, `hifk_vol_ratio_thresh`, `gcpc_cache`)
- envelope selection (`envelope_type`, `n_subdivisions`)
- output shaping (`include_endpoint_iaabbs`)

### Incremental compute

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
assert result["incremental"]["changed_dim"] == 1
```

`IncrementalEnvelopeComputer.compute(...)` accepts `changed_dim` and
`include_endpoint_iaabbs`.

### Batch compute

```python
from link_interval_envelope import compute_envelope_batch

results = compute_envelope_batch(
    "examples/data/2dof_planar.json",
    [[[-0.4, 0.4], [-0.2, 0.2]], [[-0.4, 0.4], [-0.1, 0.3]]],
    endpoint_source="ifk",
    envelope_type="link_iaabb",
    n_subdivisions=4,
    n_threads=2,
)
```

Batch output is a list of normalized `link_interval_envelope.v1` dictionaries.

### Endpoint reuse

```python
from link_interval_envelope import compute_from_endpoint_iaabbs

envelope_only = compute_from_endpoint_iaabbs(
    "examples/data/2dof_planar.json",
    result["endpoint"]["endpoint_iaabbs_flat"],
  envelope_type="support_hull",
)
```

### Result schema

The normalized Python result layout is:

- `schema`: always `link_interval_envelope.v1`
- `robot`: robot metadata, active link map/radii, midpoint link geometry
- `intervals`: requested intervals when available
- `endpoint`: endpoint source, safety flag, shape, and optional
  `endpoint_iaabbs_flat`
- `diagnostics`: source timing and reuse counters
- `envelope`: type, subdivisions, raw/inflated link AABBs, and per-link records
- `timing_us`: endpoint, envelope, and total wall-clock microseconds

Incremental results also include `incremental` with `changed_dim`,
`used_incremental_fk`, `used_source_incremental_state`, `reused_fk`,
`reused_endpoint_cache`, and `fk_valid`.

### Raw pybind layer

The extension module `_link_interval_envelope_cpp` also exposes lower-level
helpers such as `compute_endpoint_iaabb_info`,
`compute_link_envelope_from_endpoints`, `compute_envelope_info`,
`compute_envelope_batch_info`, and the bound
`IncrementalEnvelopeContext`. The package-level Python facade is the recommended
stable entry point.

## CLI

```bash
python -m link_interval_envelope compute \
  --robot examples/data/2dof_planar.json \
  --intervals-json '[[-0.4, 0.4], [-0.2, 0.2]]' \
  --endpoint-source ifk \
  --endpoint-threads 1 \
  --parallel-min-combos 0 \
  --env link_iaabb \
  --n-sub 4 \
  --out-json lie.json \
  --out-html lie.html
```

For hierarchical AA-FK, `--hifk-max-depth auto` selects the robot-aware
schedule above rather than a pure width threshold.

The public CLI options are defined in `python/link_interval_envelope/cli.py`.
Important workflow switches include:

- `--intervals-json` or `--intervals-file`
- `--endpoint-iaabbs-file`
- `--endpoint-source`
- `--gcpc-cache`
- `--include-voxels`
- `--view`
