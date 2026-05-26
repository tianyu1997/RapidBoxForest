# Output Schema

Normalized Python and CLI results use schema `link_interval_envelope.v1`.

## Top-level Fields

- `schema`: fixed schema string.
- `robot`: robot metadata and midpoint link endpoints.
- `intervals`: input joint intervals.
- `endpoint`: endpoint source result.
- `envelope`: link envelope result.
- `timing_us`: endpoint, envelope, and total runtime.
- `incremental`: present when produced by `IncrementalEnvelopeComputer`.

`compute_envelope_batch()` returns a list of result objects with this same schema.

## `robot`

- `name`
- `source_path`
- `n_joints`
- `n_active_links`
- `active_link_map`
- `active_link_radii`
- `midpoint_links`: proximal/distal midpoint FK positions for visualization.

## `endpoint`

- `source`: `IFK`, `CritSample`, `Analytical`, `GCPC`, `MC`, or `HIFK`.
- `is_safe`: conservative safety flag reported by the endpoint source.
- `n_pruned_links`: analytical-source pruning count.
- `shape`: `[n_active_links, 2, 6]`.
- `endpoint_iaabbs_flat`: optional flat endpoint boxes.

Safe sources currently include `IFK`, `Analytical`, `GCPC`, and `HIFK`.
`IFK` is the unsplit AA-FK baseline and `HIFK` is the hierarchical AA-FK
variant; both are certified outer-bound sources.

Endpoint box layout is per active link, two endpoints per link, six numbers per endpoint:

```text
[lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
```

## `envelope`

- `type`: `LinkIAABB`, `LinkIAABB_Grid`, or `Hull16_Grid`.
- `n_subdivisions`
- `shape`: `[n_active_links, n_subdivisions, 6]`.
- `active_link_map`
- `active_link_radii`
- `link_iaabbs_flat`: raw core link boxes, not radius-inflated.
- `inflated_link_iaabbs_flat`: visualization boxes inflated by active link radii.
- `links`: per-link/per-subdivision records with raw and inflated AABBs.
- `grid`: grid metadata for grid envelope types.

## `grid`

- `has_grid`
- `delta`
- `safety_pad`
- `n_bricks`
- `n_occupied`
- `occupied_volume`
- `centres`: optional occupied voxel centers when `include_voxels="centres"`.
- `bricks`: optional sparse brick payload when `include_voxels="bricks"`.

## `incremental`

- `changed_dim`: supplied or inferred changed joint dimension, or `-1`.
- `used_incremental_fk`: currently false for the public AA-only endpoint sources.
- `used_source_incremental_state`: true when a non-IFK source reused its own incremental source state; currently this applies to CritSample.
- `reused_fk`: true when intervals were unchanged and endpoints were extracted from the current state.
- `reused_endpoint_cache`: true when intervals were unchanged and cached endpoint iAABBs were reused.
- `fk_valid`: whether the context owns a valid FK state after the call.

`IFK` and `HIFK` are available through the same stateful and batch APIs, but
they currently use full endpoint recomputation rather than specialized
incremental reuse.
