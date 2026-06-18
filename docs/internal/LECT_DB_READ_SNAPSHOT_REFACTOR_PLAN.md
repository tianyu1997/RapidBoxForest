# LECT DB Read-Snapshot Refactor Plan

## Goal

Refactor LECT DB around an immutable read snapshot for planning. The planning process opens a read-only snapshot and serves node/evidence queries with mmap-backed data structures. Runtime writes go to an online overlay, and a separate writer process publishes the next snapshot after planning. Write performance is intentionally secondary.

The first target workload is planning read-only open plus exact/evidence queries. The current validated baseline on the real Shelf+IIWA persisted cache is approximately:

- `load.open_read_only`: 93.216 ms
- `page_reads`: 0
- `read.evidence_disk`: 0.212 us/op
- `read.endpoint_for_box_exact_disk`: 1.163 us/op

The snapshot design should keep query latency within the existing microsecond envelope while removing open-time heap materialization from nodes and evidence.

## Format Decisions

- Runtime snapshot supports only the new snapshot format.
- Legacy persisted caches are converted offline into a snapshot; legacy compatibility must not live in the planner open path.
- The active snapshot is immutable. A writer builds a staging generation and publishes it atomically.
- Readers never observe partial writer output and can keep reading an older generation while a writer publishes a newer one.

## Snapshot Layout

A snapshot directory contains:

- `manifest.bin`: fixed binary header plus root intervals and split-policy identity fields.
- `nodes.bin`: dense node-id-indexed struct-of-records topology store.
- `direct_evidence.bin`: dense per-node descriptor table for the common first evidence record on each node.
- `evidence_table.bin`: open-addressed static lookup table keyed by node-id-only evidence keys.
- `payload.bin`: evidence payload byte store copied or packed by the writer.

All files include fixed magic/version/header-size fields, count validation, and bounds checks. Open maps files and validates headers; it does not scan entries into STL containers.

## Runtime Components

1. `LectReadSnapshot`
   - Opens snapshot files read-only.
   - Provides `node_box`, `box_to_node_exact`, `range_query`, `evidence`, and `endpoint_for_box_exact`.
   - Uses direct node-id indexing for topology.
   - Uses mmap-backed static evidence lookup and zero-copy payload views.

2. `LectOnlineOverlay`
   - Future runtime write surface.
   - Stores newly produced evidence during planning.
   - Query order is overlay first, snapshot second.
   - Persistent flush is not part of planning latency.

3. `LectSnapshotWriter`
   - Future independent process/tool.
   - Reads an old snapshot plus overlay/delta batch.
   - Builds a full next-generation snapshot under staging.
   - Verifies and atomically publishes the generation.

## Implementation Sequence

1. Add this plan document and keep benchmark outputs in `outputs/logs/`.
2. Add mmap/read-only snapshot utilities and snapshot on-disk structs.
3. Implement `LectReadSnapshot::build_from_legacy` as the initial offline converter from current sidecar-backed caches.
4. Implement `LectReadSnapshot::open` with mmap-only open and no heap materialization of node/evidence tables.
5. Implement snapshot node queries: `node_box`, `box_to_node_exact`, and `range_query` over mmapped node rows.
6. Implement snapshot evidence queries: static open-address table lookup and mmap-backed `EvidenceRecordView` payload spans.
7. Add benchmark support for building/opening/probing the snapshot from the existing real cache.
8. Validate compilation and run the same real persisted-cache probe with snapshot stages enabled.
9. Compare the previous persisted database baseline and read snapshot-only open/query numbers, then record final results.

## First Landing Scope

The first landing implements the read-snapshot lane and benchmark coverage. It does not yet replace planner Oracle calls or add the final independent writer process. Those become the next integration phase once the snapshot format and read performance are verified.

## Acceptance Criteria

- `lect_database_core` builds successfully.
- `lect_database_benchmark` builds successfully.
- A snapshot can be generated from the real Shelf+IIWA cache.
- snapshot open/query benchmark succeeds on that snapshot.
- snapshot open performs mmap/header validation only and reports no legacy node page reads.
- snapshot evidence and endpoint queries remain comfortably below 10 us/op.

## Landed Read-Snapshot Result

The first landing scope is implemented in `LectReadSnapshot` and benchmarked on the real Shelf+IIWA cache. The snapshot converter currently builds from the existing sidecar-backed legacy cache and writes the new read snapshot files. Runtime snapshot open maps and validates the binary files without scanning entries into STL containers.

Final measured rows from `outputs/logs/exp03_persisted_cache_snapshot_stages.csv`:

- `snapshot.build`: 243.606 ms
- `snapshot.load.open_read_only`: 0.025 ms
- `snapshot.read.node_box`: 1.310 us/op
- `snapshot.read.exact_box_lookup`: 0.846 us/op
- `snapshot.read.endpoint_for_box_exact`: 1.368 us/op
- `snapshot.read.range_query`: 50.406 us/op
- `snapshot.read.evidence`: 0.901 us/op

## Follow-Up Integration Phase

After the read snapshot is verified:

1. Introduce `LectOnlineOverlay` and route planner-produced evidence into it.
2. Change Oracle write sites to overlay enqueue instead of persistent DB writes.
3. Move `checkpoint_after_build` out of the planning critical path.
4. Add an independent writer tool/process that consumes overlay batches and publishes new snapshots.
5. Keep legacy conversion as an offline utility, not a planner open path.

## Landed Warm-Reuse Integration

The first downstream integration keeps the active SBF planning cache on the
existing writable `LectDatabase` path and routes only the read-only external
warm evidence source through `LectReadSnapshot`.

The public/runtime surface for this lane is:

- `LectExternalEvidenceSource`: small read-only abstraction shared by legacy DB and snapshot readers
- `DatabaseBoxOracle`: external evidence now probes `endpoint_for_box_exact(intervals, key)` first so snapshot direct-evidence lookup stays on the fast path
- `LectDatabaseRuntimeConfig.external_evidence_use_snapshot`
- `LectDatabaseRuntimeConfig.external_evidence_snapshot_path`
- `LectDatabaseRuntimeConfig.external_evidence_auto_build_snapshot`

This keeps runtime writes and online backfill behavior unchanged while moving
the warm external-read lane to the mmapped snapshot path that benchmarked best
on the real Shelf+IIWA cache.
