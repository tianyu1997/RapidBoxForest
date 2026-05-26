# Incremental FK and v6 Compatibility

## Compatibility Goal

The standalone package keeps the current v6 `sbf::FKState` structure and the same incremental FK functions:

- `sbf::compute_fk_full`
- `sbf::compute_fk_incremental`
- `sbf::update_fk_inplace`
- `sbf::extract_endpoint_iaabbs`
- `sbf::extract_link_aabbs`

This makes code written for the v6 FK state source-compatible when it is compiled against this standalone package. The standalone package is self-contained and should be treated as the provider of `sbf::*` symbols in a downstream target. Avoid linking both `cpp/v6` and this package into the same binary because they intentionally define overlapping `sbf` symbols.

## How Incremental FK Is Used

For IFK endpoint computation, the low-level entry point accepts an optional mutable FK state:

```cpp
sbf::FKState fk;

auto parent = sbf::compute_endpoint_iaabb(
    robot, parent_intervals, endpoint_config, &fk, -1);

auto child = sbf::compute_endpoint_iaabb(
    robot, child_intervals, endpoint_config, &fk, changed_dim);
```

The first call computes the full chain and writes `fk`. The second call reuses the valid state and recomputes only from `changed_dim` onward. This matches the v6 incremental workflow used by tree expansion code.

## Stateful Context

`link_interval_envelope::IncrementalEnvelopeContext` wraps this pattern and also computes the requested link envelope:

```cpp
link_interval_envelope::IncrementalEnvelopeContext context(
    robot, endpoint_config, envelope_config);

context.compute(parent_intervals);
auto result = context.compute(child_intervals, changed_dim);
```

If `changed_dim` is omitted, the context compares the new intervals with `last_intervals()`:

- exactly one changed dimension: use that dimension incrementally;
- no changed dimensions: reuse the current FK state and only extract endpoints;
- multiple changed dimensions or no valid state: full recompute.

The result flags are:

- `used_incremental_fk`: the IFK incremental path was used;
- `used_source_incremental_state`: a non-IFK source reused its own incremental state;
- `reused_fk`: intervals were unchanged and FK extraction reused the existing state;
- `reused_endpoint_cache`: intervals were unchanged and cached endpoint iAABBs were reused;
- `changed_dim`: the supplied or inferred changed dimension, or `-1` when no single changed dimension applies.

## Python Workflow

```python
from link_interval_envelope import IncrementalEnvelopeComputer

computer = IncrementalEnvelopeComputer(
    "examples/data/2dof_planar.json",
    endpoint_source="ifk",
    envelope_type="link_iaabb",
    n_subdivisions=4,
)

computer.compute([[-0.4, 0.4], [-0.2, 0.2]])
result = computer.compute([[-0.4, 0.4], [-0.1, 0.3]])
print(result["incremental"])
```

The Python object exposes `fk_state`, `reset()`, and `has_valid_fk()` for diagnostics.

## Endpoint Source Notes

- `EndpointSource.IFK` reuses `FKState` and recomputes the FK chain from `changed_dim` onward.
- `EndpointSource.CritSample` reuses per-joint critical candidates and precomputed DH matrices. A single changed dimension only rebuilds that dimension's raw candidates, then refreshes any dimensions affected by the global combo cap. Unchanged intervals reuse cached endpoint iAABBs.
- `EndpointSource.Analytical`, `EndpointSource.GCPC`, and `EndpointSource.MC` currently keep their original one-shot behavior in this standalone package.

## Validation

The test suite verifies that incremental IFK and CritSample endpoint boxes and final link envelopes match a full recompute for the same child interval box.
