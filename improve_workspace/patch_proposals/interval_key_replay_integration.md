# Interval-Key Replay Production Integration Proposal

This proposal closes the production design gap for `docs/improve.md` Sections
3.3--3.6.  It is intentionally not applied to production source in this
workspace.

## Current Production State

Relevant source anchors:

- `lect_database/include/LECTDatabase/sbf/oracle.h`
  - exposes `endpoint_evidence_key(...)`, `root_intervals()`,
    `node_intervals(...)`, and `query_intervals_for_node(...)`.
  - exposes worker/shared endpoint cache comments as interval-keyed.
- `lect_database/src/sbf/oracle.cpp`
  - already has interval hashing helpers and canonical interval mapping:
    `hash_intervals`, `canonical_evidence_frame_for_intervals`,
    `endpoint_payload_for_node`.
  - external exact evidence lookup uses `endpoint_for_box_exact(...)`.
- `lect_database/src/lect_database/database.cpp`
  - owns database manifest, identity, fingerprint, split policy, and checkpoint
    logic.

The remaining gap is not the absence of interval hashing; it is the lack of one
explicit production compatibility contract that says evidence replay is keyed by
robot/envelope/interval semantics rather than by heap traversal identity.

## Required Compatibility Contract

Define a production evidence compatibility policy:

```cpp
struct IntervalEvidenceCompatibility {
    RobotFingerprint robot;
    EndpointSource endpoint_source;
    EnvelopeTypeConfig envelope_config;
    std::vector<int> active_links;
    std::vector<double> active_link_radii;
    std::vector<Interval> joint_intervals;
    std::optional<DyadicAddress> dyadic_address;
};
```

Replay is valid if and only if all identity fields match and the query
intervals are exactly represented in the evidence frame used for lookup.

Heap node id may remain an acceleration key, but it must not be the semantic
validity condition.

## Required Source Changes

1. Add an explicit `IntervalEvidenceKey` wrapper next to
   `lect_database::EvidenceKey` usage in the oracle adapter.
2. Route exact external/shared endpoint-cache lookup through:

```cpp
endpoint_for_box_exact(canonical_or_native_lookup_intervals, endpoint_key)
```

where the canonical/native frame is validated before lookup.
3. When the canonical evidence frame is invalid, fail closed.  Do not reuse
   key-only evidence.
4. Record provenance for child-hull parent evidence separately from direct
   endpoint evidence.
5. Add manifest fields that state:
   - root interval fingerprint;
   - coverage interval fingerprint;
   - split policy descriptor/fingerprint;
   - canonical coverage domain;
   - endpoint/envelope/radius identity.

## Sparse / Patricia Staging

Stage 1 keeps existing LECTDB persistence:

- Use `DyadicAddress` / `VirtualCell` only in runtime adaptive sweep.
- Materialize evidence by exact joint intervals through the existing oracle.
- Store scene labels outside LECT.

Stage 2 adds sparse persistent keys:

```cpp
struct SparseEvidenceNode {
    DyadicAddress address;
    EvidenceRef evidence_ref;
    bool terminal = false;
};
```

Only terminal/branching nodes need to be persisted.

## Canonical Safety Rules

- Canonicalization stays inside LECT.
- Native planner/query inputs and outputs remain native joint space.
- A native interval can reuse canonical evidence only if it maps to one valid
  canonical sector interval.
- If mapping is invalid or crosses sectors, do not reuse external canonical
  evidence.

## Tests

Add C++/Python regression tests:

- Same native interval and same evidence identity hits exact external evidence.
- Same heap id but different interval identity does not replay evidence.
- Cross-sector interval cannot reuse canonical evidence.
- Four native sectors map to the same canonical evidence only when the reflected
  interval is valid and contains the native seed.
- Child-hull evidence provenance is not confused with direct endpoint evidence.

## Experiment Use

When enabled in paper experiments, manifest rows should report:

- external exact hits/misses;
- canonical frame invalid count;
- interval replay hit rate;
- live materialization fallbacks.
