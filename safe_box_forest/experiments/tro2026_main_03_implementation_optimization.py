#!/usr/bin/env python3
"""Implementation-optimization experiment matrix for the TRO 2026 SBF stack.

This script does not run expensive benchmarks.  It writes a reproducible JSON/CSV
registry that maps implementation-level optimization targets to the artifacts and
validation gates that should be produced before a paper-facing claim is made.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PAPER_OUT = ROOT / "outputs" / "paper"
DEFAULT_OUT = PAPER_OUT / "tro2026_exp17_implementation_optimization_plan.json"
DEFAULT_CSV = PAPER_OUT / "tro2026_exp17_implementation_optimization_plan.csv"


OPTIMIZATION_ROWS: list[dict[str, Any]] = [
    {
        "package": "LECT",
        "id": "LECT-01",
        "priority": "P0",
        "target": "Incremental interval reconstruction in FFB descent",
        "source_files": ["LECT/src/lect_tree.cpp", "LECT/src/lect.cpp"],
        "artifact": "tro2026_lect_interval_cache.json",
        "primary_metric": "interval reconstruction time and FFB time",
        "validation_gate": "same leaf intervals and unchanged audited query success",
        "paper_mapping": "tab:tro_lect_reuse",
        "status": "planned",
    },
    {
        "package": "LECT",
        "id": "LECT-02",
        "priority": "P1",
        "target": "Hash index for evidence/grid lookup and remap",
        "source_files": ["LECT/src/grid_store.cpp", "LECT/src/evidence_store.cpp"],
        "artifact": "tro2026_lect_evidence_index.json",
        "primary_metric": "evidence lookup time, grid lookup time, cache hit rate",
        "validation_gate": "indexed lookup returns the same payloads as linear lookup",
        "paper_mapping": "tab:tro_lect_reuse",
        "status": "planned",
    },
    {
        "package": "LECT",
        "id": "LECT-03",
        "priority": "P2",
        "target": "Lazy/mmap storage load and streaming save",
        "source_files": ["LECT/src/storage.cpp", "LECT/src/mmap_lect_file.cpp"],
        "artifact": "tro2026_lect_storage_lazy.json",
        "primary_metric": "load time, peak memory, first-query latency",
        "validation_gate": "delta replay and section checksums remain valid",
        "paper_mapping": "tab:tro_lect_reuse",
        "status": "planned",
    },
    {
        "package": "LECT",
        "id": "LECT-04",
        "priority": "P1",
        "target": "Per-worker LECT session reuse",
        "source_files": ["LECT/src/worker_session.cpp", "sbf-standalone/src/grower.cpp"],
        "artifact": "tro2026_lect_worker_session.json",
        "primary_metric": "session count, merge time, grow time",
        "validation_gate": "deterministic reduction mode and unchanged audit success",
        "paper_mapping": "tab:tro_main_shelf_benchmark",
        "status": "planned",
    },
    {
        "package": "link-interval-envelope",
        "id": "LIE-01",
        "priority": "P0",
        "target": "Unified thread budget for batch and CritSample",
        "source_files": ["link-interval-envelope/src/batch.cpp", "link-interval-envelope/src/envelope/dh_enumerate.cpp"],
        "artifact": "tro2026_lie_thread_budget.json",
        "primary_metric": "wall time, CPU efficiency, variance under nested parallelism",
        "validation_gate": "same envelope outputs under fixed seeds and no oversubscription regression",
        "paper_mapping": "tab:tro_main_evidence_validation",
        "status": "planned",
    },
    {
        "package": "link-interval-envelope",
        "id": "LIE-02",
        "priority": "P1",
        "target": "CritSample candidate cache and dirty-joint tracking",
        "source_files": ["link-interval-envelope/src/envelope/crit_source.cpp"],
        "artifact": "tro2026_lie_critsample_cache.json",
        "primary_metric": "endpoint time, candidate count, dirty-joint count",
        "validation_gate": "provisional outputs are re-audited by SBF strict path audit",
        "paper_mapping": "tab:tro_main_evidence_validation",
        "status": "planned",
    },
    {
        "package": "link-interval-envelope",
        "id": "LIE-03",
        "priority": "P1",
        "target": "Sparse voxel grid reserve/mask/SIMD paths",
        "source_files": ["link-interval-envelope/include/sbf/voxel/voxel_grid.h", "link-interval-envelope/include/sbf/voxel/bit_brick.h"],
        "artifact": "tro2026_lie_voxel_grid.json",
        "primary_metric": "grid fill time, occupied bricks, intersection time",
        "validation_gate": "conservative voxel occupancy is not reduced incorrectly",
        "paper_mapping": "tab:tro_main_evidence_validation",
        "status": "planned",
    },
    {
        "package": "link-interval-envelope",
        "id": "LIE-04",
        "priority": "P0",
        "target": "Lightweight Python binding output modes",
        "source_files": ["link-interval-envelope/python/bindings.cpp"],
        "artifact": "tro2026_lie_python_payload.json",
        "primary_metric": "return time, payload size, optional-field overhead",
        "validation_gate": "default schema remains backwards compatible or changes are opt-in",
        "paper_mapping": "appendix payload diagnostics",
        "status": "planned",
    },
    {
        "package": "sbf-standalone",
        "id": "SBF-01",
        "priority": "P0",
        "target": "Indexed adjacency, merger, and connector candidates",
        "source_files": ["sbf-standalone/src/box_graph.cpp", "sbf-standalone/src/merger.cpp", "sbf-standalone/src/connector.cpp"],
        "artifact": "tro2026_sbf_graph_index.json",
        "primary_metric": "candidate pairs, adjacency time, edge count",
        "validation_gate": "indexed graph is equivalent to all-pairs graph",
        "paper_mapping": "tab:tro_main_shelf_benchmark",
        "status": "planned",
    },
    {
        "package": "sbf-standalone",
        "id": "SBF-02",
        "priority": "P0",
        "target": "Persistent runtime executor and per-thread diagnostics",
        "source_files": ["sbf-standalone/src/runtime.cpp"],
        "artifact": "tro2026_sbf_runtime_pool.json",
        "primary_metric": "stage time, speedup, efficiency, variance",
        "validation_gate": "exceptions propagate and deterministic mode remains available",
        "paper_mapping": "tab:tro_main_systems_summary",
        "status": "planned",
    },
    {
        "package": "sbf-standalone",
        "id": "SBF-03",
        "priority": "P1",
        "target": "Query point-location and graph-search caching",
        "source_files": ["sbf-standalone/src/query.cpp", "sbf-standalone/src/box_graph.cpp"],
        "artifact": "tro2026_sbf_query_index.json",
        "primary_metric": "locate time, Dijkstra time, query time",
        "validation_gate": "cache invalidates after forest mutation and dynamic update",
        "paper_mapping": "fig:tro_query_amortization",
        "status": "planned",
    },
    {
        "package": "sbf-standalone",
        "id": "SBF-04",
        "priority": "P1",
        "target": "Dynamic obstacle update compaction and evidence broadphase",
        "source_files": ["sbf-standalone/src/safe_box_forest.cpp", "sbf-standalone/src/scene.cpp"],
        "artifact": "tro2026_sbf_dynamic_broadphase.json",
        "primary_metric": "collision-check time, removed boxes, rebuild time",
        "validation_gate": "no deleted box or segment edge remains reachable",
        "paper_mapping": "tab:tro_main_systems_summary",
        "status": "planned",
    },
    {
        "package": "sbf-standalone",
        "id": "SBF-05",
        "priority": "P0",
        "target": "Release GIL for long Python binding calls",
        "source_files": ["sbf-standalone/python/bindings.cpp"],
        "artifact": "tro2026_sbf_python_gil.json",
        "primary_metric": "Python parallel throughput and C++ call wall time",
        "validation_gate": "exceptions and object lifetimes remain pybind11-safe",
        "paper_mapping": "appendix systems diagnostics",
        "status": "planned",
    },
]


def artifact_state(outputs: Path, artifact_name: str) -> dict[str, Any]:
    path = outputs / artifact_name
    if not path.exists():
        return {"artifact_path": str(path), "artifact_exists": False, "artifact_bytes": 0}
    return {"artifact_path": str(path), "artifact_exists": True, "artifact_bytes": path.stat().st_size}


def rows_with_state(outputs: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for item in OPTIMIZATION_ROWS:
        row = dict(item)
        row.update(artifact_state(outputs, str(item["artifact"])))
        row["source_files_present"] = all((ROOT.parent / source).exists() for source in item.get("source_files", []))
        rows.append(row)
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "package",
        "id",
        "priority",
        "target",
        "artifact",
        "artifact_exists",
        "primary_metric",
        "validation_gate",
        "paper_mapping",
        "status",
        "source_files_present",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Emit the TRO implementation-optimization experiment matrix.")
    parser.add_argument("--outputs", type=Path, default=PAPER_OUT)
    parser.add_argument("--out-json", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--out-csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--strict-sources", action="store_true", help="Fail if any listed source file is missing.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    rows = rows_with_state(args.outputs)
    missing_sources = [row["id"] for row in rows if not row.get("source_files_present")]
    if args.strict_sources and missing_sources:
        raise SystemExit("Missing source files for: " + ", ".join(missing_sources))
    payload = {
        "schema_version": 1,
        "experiment": "implementation_optimization_plan",
        "source_script": str(Path(__file__).resolve()),
        "outputs": str(args.outputs),
        "summary": {
            "row_count": len(rows),
            "ready_artifact_count": sum(1 for row in rows if row.get("artifact_exists")),
            "missing_source_ids": missing_sources,
        },
        "rows": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_csv(args.out_csv, rows)
    print(json.dumps({"out_json": str(args.out_json), "out_csv": str(args.out_csv), "summary": payload["summary"]}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())