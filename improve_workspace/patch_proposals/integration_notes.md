# Production Integration Notes

These notes describe the source edits that would be needed to integrate the
sidecar implementation into RapidBoxForest.  They are intentionally not applied.

## Early-stop Adaptive Sweep

Target integration points:

- `safe_box_forest` leaf sweep / refine implementation.
- `RBFPlanningConfig` or equivalent leaf sweep config structures.
- Python bindings exposing the new configuration and diagnostics.

Required production changes:

1. Add sweep-specific accept depth separate from seed/FFB accept depth.
2. Replace fixed virtual leaf enumeration with a priority queue of terminal
   adaptive cells.
3. Preserve existing conservative scene validation before any box enters the
   forest.
4. Add diagnostics for evaluated cells, terminal cell states, depth histogram,
   blocker signatures, and deferral reasons.

## Rich Validation Reports

Target integration points:

- scene validation pipeline;
- AABB/support-hull/GJK/SDF failure reporting;
- dynamic update blocker metadata.

Required production changes:

1. Extend failure reports from obstacle IDs to
   `(link, obstacle, stage, margin, overlap, affected_joints)`.
2. Reuse blocker signatures for no-good cooldown and split-dimension scoring.
3. Keep `CERT_OCCUPIED` optional and disabled unless a sound signed-distance
   material witness is available.

## Sparse Dyadic Overlay

Target integration points:

- LECT oracle cell traversal;
- evidence key construction;
- cache replay compatibility checks.

Required production changes:

1. Introduce `DyadicAddress` or equivalent per-dimension level/index keys.
2. Permit virtual runtime cells that materialize evidence by exact joint
   intervals without creating all intermediate heap nodes.
3. Define an interval-compatible `EvidenceKey` policy:
   `EvidenceKey(robot, endpoint_source, envelope_rep, active_links, radii,
   joint_intervals_or_dyadic_address)`.
4. Keep scene labels outside LECT to preserve evidence/label separation.

## Portal Edges

Target integration points:

- typed graph edge definitions;
- graph search cost model;
- query path extraction and audit diagnostics.

Required production changes:

1. Add `E_portal` edge type carrying an internal conservative box chain.
2. Store internal boxes outside global vertices unless the query path uses the
   edge.
3. Expand each portal edge before applying the conservative corridor theorem.
4. Keep performance-mode portal chains separate from conservative chains.

Membership policy:

- Low-risk first pass: global membership lookup ignores portal internals; if a
  query endpoint is not in a global free box, online repair handles it.
- Full pass: add a `PortalInteriorIndex` membership layer. Query membership first
  checks global boxes, then checks portal internal boxes, and temporarily expands
  the owning portal corridor when an endpoint lies inside it.
