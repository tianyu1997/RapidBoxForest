# LECT 10us Query Fast-Path Plan

## Goal

Bring persisted-cache query latency for the practical online path down to the 10 us class for:

- `node_box`
- `box_to_node_exact`
- `endpoint_for_box_exact`
- exact or very small local range queries

Constraints agreed for this work:

- No new slow global prewarm/open phase before planning begins.
- Query acceleration must work in a lazy or already-open process.
- Memory may increase moderately, but not by keeping the full text node storage hot as the primary mechanism.
- Existing persisted cache format should remain readable; optional acceleration artifacts are acceptable.

## Diagnosis

The current evidence path is already fast because it uses a binary index plus mmap-backed reads.

The node query path is slow because node metadata is still resolved through page-text replay:

- `load_nodes()` scans every node page at open time only to populate `node_ids_` and `node_path_index_`.
- `node_box()`, `box_to_node_exact()`, and `range_query()` still fall back to `read_node()`.
- `read_node()` calls `touch_node_page()`.
- `touch_node_page()` opens the page file and reparses text rows on cache miss.

That means the database already pays most of the metadata scan cost at open, but does not retain the query-critical topology needed to make reads CPU-bound.

## Execution Plan

### Phase 1: Baseline and Acceptance Harness

1. Update the benchmark to distinguish the following query modes:
   - stateless direct API
   - query session API
   - fast-path API after the node metadata refactor
2. Keep the real Shelf+IIWA persisted cache as the primary validation target.
3. Add acceptance checks that log:
   - `node_page_reads`
   - `node_page_cache_hits/misses`
   - `query_path_cache_hits/misses`
   - `range_nodes_visited`
4. Keep a separate small-local range workload instead of using broad overlap queries as the only range benchmark.

Acceptance for this phase:

- We can compare old and new read paths on the same cache and workload.

### Phase 2: In-Memory Node Metadata Fast Path

1. Add a compact in-memory node metadata store that is retained after `load_nodes()`.
2. The metadata must cover at least:
   - `parent`
   - `left`
   - `right`
   - `depth`
   - `split_dim`
   - `split_value`
3. Do not require a separate prewarm step; piggyback on metadata that is already parsed during open.
4. Rebuild node-path-on-demand from parent links in memory instead of rereading text node pages.

Acceptance for this phase:

- Query code can resolve node topology without calling `touch_node_page()`.

### Phase 3: Query Path Rewrite

1. Rewrite `node_box()` to use the in-memory metadata path first.
2. Rewrite `box_to_node_exact()` to walk the tree entirely from in-memory metadata.
3. Rewrite `normalize_evidence_key()` for `node_id -> node_path` using in-memory topology reconstruction.
4. Route `endpoint_for_box_exact()` through the new exact lookup path.
5. Extend `LectDbQuerySession` so it benefits from the fast node metadata path.

Acceptance for this phase:

- `node_box` and `box_to_node_exact` show near-zero `node_page_reads` in benchmark runs.

### Phase 4: Small-Local Range Query Fast Path

1. Keep the existing broad `range_query()` semantics intact.
2. Make the traversal prefer the in-memory metadata path for exact or small-local query boxes.
3. Report broad-range queries separately if their traversal/output size makes 10 us unrealistic.

Acceptance for this phase:

- small-local range queries improve materially and are measured separately from broad overlap queries.

### Phase 5: Optional Sidecar Acceleration

1. Add an optional compact node-query sidecar if the in-memory path alone is still not enough.
2. The sidecar should be an acceleration artifact, not the only source of truth.
3. If the sidecar is missing or stale, the database must fall back to the compatible path.
4. Any manifest update should be optional and backward compatible for old readable caches.

Acceptance for this phase:

- existing caches remain readable without regeneration
- sidecar-enabled caches can skip text replay for query-critical metadata where appropriate

## Immediate Implementation Order

The first implementation pass in this repository will execute in this order:

1. Add this plan document.
2. Implement the retained node metadata store.
3. Rewrite `node_box`, `box_to_node_exact`, and `normalize_evidence_key` to use it.
4. Rebuild the benchmark target.
5. Re-run the real Shelf+IIWA persisted-cache microbenchmark.
6. Compare latency and `node_page_reads` against the previous run.
7. If exact lookup remains above target, continue with session caching and then sidecar work.

## Validation Criteria

For the real Shelf+IIWA persisted cache, the primary criteria are:

- `node_box` average latency approaches the 10 us class.
- `box_to_node_exact` average latency approaches the 10 us class.
- `endpoint_for_box_exact` follows once exact lookup no longer rereads node pages.
- `node_page_reads` for these query stages drop to approximately zero.

Secondary criteria:

- no new slow open phase is introduced
- compatibility with existing persisted caches is preserved
- evidence lookup remains at the current microsecond scale

## Notes

- If the retained metadata pass alone is sufficient, it is preferable to shipping a new sidecar immediately.
- If the retained metadata pass still leaves exact lookup above target, the next escalation is a dedicated node-query sidecar or more compact node metadata layout.