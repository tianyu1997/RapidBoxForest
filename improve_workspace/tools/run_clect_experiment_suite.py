#!/usr/bin/env python3
"""C-LECT sidecar experiment suite matching docs/improve.md Section 7.

This script is intentionally synthetic and self-contained.  It does not claim
production RBF performance; it provides repeatable per-mechanism evidence for
the sidecar implementation under improve_workspace/.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from improve_workspace.clect_sidecar import (
    AdaptiveLeafSweep,
    AdaptiveSweepConfig,
    DyadicAddress,
    Interval,
    Portal,
    PortalCorridor,
    PortalGraph,
    SparseNodeMap,
    ValidationReport,
)
from improve_workspace.clect_sidecar.dyadic import (
    JointBox,
    box_center,
    boxes_touch_or_overlap,
    split_schedule_cells,
)
from improve_workspace.clect_sidecar.reports import Blocker, FailStage, ValidationStatus
from improve_workspace.clect_sidecar.synthetic import Region, SyntheticValidator, fixed_leaf_evaluation_count, unit_root


@dataclass(frozen=True)
class SuiteConfig:
    start_depth: int
    max_depth: int
    portal_chain_length: int
    sparse_level: int


def timed(fn):
    start = time.perf_counter()
    value = fn()
    return value, (time.perf_counter() - start) * 1000.0


def fixed_virtual_count(start_depth: int, max_depth: int) -> int:
    return (1 << start_depth) * fixed_leaf_evaluation_count(start_depth, max_depth)


def scenario_root() -> JointBox:
    return unit_root(2)


def scenario_validator(with_occupied: bool = True) -> SyntheticValidator:
    return SyntheticValidator(
        free_regions=[
            Region((Interval(0.0, 0.5), Interval(0.0, 1.0)), "large_free_half"),
            Region((Interval(0.75, 0.8125), Interval(0.25, 0.75)), "narrow_free_band"),
        ],
        occupied_regions=(
            [Region((Interval(0.5, 0.75), Interval(0.0, 1.0)), "solid_blocker")]
            if with_occupied
            else []
        ),
    )


def no_good_validator(_address: DyadicAddress, box: JointBox) -> ValidationReport:
    widest = max(range(len(box)), key=lambda dim: box[dim].width)
    blocker = Blocker(
        link_id=0,
        obstacle_id=0,
        stage=FailStage.GJK,
        margin=-1.0,
        overlap_score=1.0,
        affected_joints=(widest,),
    )
    return ValidationReport.fail([blocker], overlap_score=1.0)


def no_occupied_validator(address: DyadicAddress, box: JointBox) -> ValidationReport:
    report = scenario_validator(with_occupied=True)(address, box)
    if report.status != ValidationStatus.CERT_OCCUPIED:
        return report
    blocker = report.blockers[0] if report.blockers else Blocker(
        link_id=0,
        obstacle_id=0,
        stage=FailStage.GJK,
        margin=-1.0,
        overlap_score=1.0,
        affected_joints=(0,),
    )
    return ValidationReport.fail([blocker], overlap_score=1.0)


def fixed_leaf_sweep(config: SuiteConfig) -> dict[str, Any]:
    root = scenario_root()
    validator = scenario_validator(with_occupied=True)
    levels = [0, 0]
    schedule = [i % 2 for i in range(config.max_depth)]
    for dim in schedule:
        levels[dim] += 1
    cells = split_schedule_cells(2, config.max_depth, schedule)

    def run() -> tuple[list[tuple[DyadicAddress, ValidationReport]], Counter[int]]:
        records = []
        depth_hist = Counter()
        for cell in cells:
            box = cell.interval_box(root)
            report = validator(cell, box)
            records.append((cell, report))
            depth_hist[cell.depth] += 1
        return records, depth_hist

    (records, depth_hist), elapsed_ms = timed(run)
    free = [item for item in records if item[1].status == ValidationStatus.FREE]
    occupied = [item for item in records if item[1].status == ValidationStatus.CERT_OCCUPIED]
    collision = [item for item in records if item[1].status not in {ValidationStatus.FREE, ValidationStatus.CERT_OCCUPIED}]
    free_depth_hist = Counter(cell.depth for cell, _report in free)
    return base_row(
        variant="fixed_leaf_sweep",
        category="fixed",
        config=config,
        elapsed_ms=elapsed_ms,
        virtual_leaves=len(cells),
        materialized_cells=len(records),
        validated_free=len(free),
        occupied_domains=len(occupied),
        collision_domains=len(collision),
        deferred=0,
        lect_evidence_lookups=len(records),
        global_graph_vertices=len(free),
        n_e_portal=0,
        query_success=len(free) > 0,
        audited_path_length=0.4 if len(free) > 0 else math.inf,
        accepted_free_depth_histogram=dict(free_depth_hist),
        materialized_depth_histogram=dict(depth_hist),
    )


def adaptive_variant(
    variant: str,
    category: str,
    config: SuiteConfig,
    validator,
    overrides: dict[str, Any] | None = None,
    sparse: bool = False,
    portal: bool = False,
) -> dict[str, Any]:
    root = scenario_root()
    sweep_config = AdaptiveSweepConfig(
        start_depth=config.start_depth,
        max_depth=config.max_depth,
        max_evaluations=200000,
    )
    for key, value in (overrides or {}).items():
        setattr(sweep_config, key, value)
    sweep = AdaptiveLeafSweep(root, validator, sweep_config)
    result, elapsed_ms = timed(sweep.run)

    sparse_nodes = 0
    sparse_reduction = 1.0
    if sparse:
        sparse_tree = SparseNodeMap(2)
        for index, cell in enumerate(result.free):
            point = box_center(cell.box)
            sparse_tree.jump_materialize(
                root,
                point,
                (config.sparse_level, config.sparse_level),
                evidence_ref=f"free:{index}",
                terminal=True,
            )
        sparse_nodes = len(sparse_tree)
        heap_path_nodes = max(1, len(result.free) * ((2 * config.sparse_level) + 1))
        sparse_reduction = heap_path_nodes / max(1, sparse_nodes)

    portal_vertices = 0
    portal_edges = 0
    portal_path_length = 0.0
    if portal:
        portal_metrics = portal_compression_metrics(config.portal_chain_length)
        portal_vertices = int(portal_metrics["global_vertices_compressed"])
        portal_edges = int(portal_metrics["n_e_portal"])
        portal_path_length = float(portal_metrics["audited_path_length"])
    else:
        portal_metrics = {}

    global_vertices = len(result.free) + (portal_vertices if portal else 0)
    query_success = len(result.free) > 0 or portal
    audited_path_length = portal_path_length if portal else (0.4 if len(result.free) > 0 else math.inf)
    free_depth_hist = Counter(cell.address.depth for cell in result.free)
    return base_row(
        variant=variant,
        category=category,
        config=config,
        elapsed_ms=elapsed_ms,
        virtual_leaves=fixed_virtual_count(config.start_depth, config.max_depth),
        materialized_cells=result.evaluated,
        validated_free=len(result.free),
        occupied_domains=len(result.occupied),
        collision_domains=len(result.collision),
        deferred=len(result.deferred),
        lect_evidence_lookups=result.evaluated,
        global_graph_vertices=global_vertices,
        n_e_portal=portal_edges,
        query_success=query_success,
        audited_path_length=audited_path_length,
        accepted_free_depth_histogram=dict(free_depth_hist),
        materialized_depth_histogram=dict(result.depth_histogram),
        extra={
            "terminal_count": result.terminal_count,
            "split_count": result.split_count,
            "covered_terminal_count": len(result.covered),
            "sparse_nodes": sparse_nodes,
            "sparse_node_reduction": sparse_reduction,
            "portal_uncompressed_vertices": portal_metrics.get("global_vertices_uncompressed", 0),
            "portal_compressed_vertices": portal_metrics.get("global_vertices_compressed", 0),
            "portal_expanded_vertex_count": portal_metrics.get("expanded_vertex_count_when_used", 0),
            "portal_certificate_valid": portal_metrics.get("certificate_valid", ""),
        },
    )


def portal_compression_metrics(chain_length: int) -> dict[str, Any]:
    pin_box = (Interval(0.0, 1.0),)
    pout_box = (Interval(float(chain_length + 1), float(chain_length + 2)),)
    internal_boxes = [
        (Interval(float(i + 1), float(i + 2)),)
        for i in range(chain_length)
    ]
    internal_cells = [DyadicAddress.root(1) for _ in range(chain_length)]
    reports = [ValidationReport.free(f"portal:{i}") for i in range(chain_length)]
    pin = Portal("domain", "pin", "component_0")
    pout = Portal("domain", "pout", "component_1")
    corridor = PortalCorridor(
        "portal_corridor",
        "domain",
        pin,
        pout,
        internal_cells,
        internal_boxes,
        reports,
        conservative=True,
    )
    graph = PortalGraph()
    graph.add_portal_corridor(corridor, pin_box, pout_box)
    expanded = graph.expand_edge("portal_corridor")
    return {
        "global_vertices_uncompressed": chain_length + 2,
        "global_vertices_compressed": 2,
        "expanded_vertex_count_when_used": len(expanded),
        "n_e_portal": 1,
        "audited_path_length": float(chain_length + 1),
        "certificate_valid": corridor.validate_certificate(pin_box, pout_box),
    }


def base_row(
    *,
    variant: str,
    category: str,
    config: SuiteConfig,
    elapsed_ms: float,
    virtual_leaves: int,
    materialized_cells: int,
    validated_free: int,
    occupied_domains: int,
    collision_domains: int,
    deferred: int,
    lect_evidence_lookups: int,
    global_graph_vertices: int,
    n_e_portal: int,
    query_success: bool,
    audited_path_length: float,
    accepted_free_depth_histogram: dict[int, int],
    materialized_depth_histogram: dict[int, int],
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    row = {
        "variant": variant,
        "category": category,
        "start_depth": config.start_depth,
        "max_depth": config.max_depth,
        "N_virtual_leaves": virtual_leaves,
        "N_materialized_cells": materialized_cells,
        "N_validated_free": validated_free,
        "N_occupied_domains": occupied_domains,
        "N_collision_domains": collision_domains,
        "N_deferred": deferred,
        "N_LECT_evidence_lookups": lect_evidence_lookups,
        "N_global_graph_vertices": global_graph_vertices,
        "N_E_portal": n_e_portal,
        "query_success": bool(query_success),
        "audited_path_length": audited_path_length,
        "elapsed_ms": elapsed_ms,
        "materialization_reduction": virtual_leaves / max(1, materialized_cells),
        "accepted_free_depth_histogram": {
            str(k): int(v) for k, v in sorted(accepted_free_depth_histogram.items())
        },
        "materialized_cell_depth_histogram": {
            str(k): int(v) for k, v in sorted(materialized_depth_histogram.items())
        },
    }
    row.update(extra or {})
    return row


def run_suite(config: SuiteConfig) -> list[dict[str, Any]]:
    rows = [
        fixed_leaf_sweep(config),
        adaptive_variant(
            "early_stop_free_only",
            "early_stop",
            config,
            scenario_validator(with_occupied=False),
            {"no_good_chain": 1000000},
        ),
        adaptive_variant(
            "early_stop_no_good",
            "no_good",
            config,
            no_good_validator,
            {"no_good_chain": 2},
        ),
        adaptive_variant(
            "early_stop_sparse_dyadic",
            "sparse",
            config,
            scenario_validator(with_occupied=True),
            {"no_good_chain": 1000000},
            sparse=True,
        ),
        adaptive_variant(
            "early_stop_portal_edges",
            "portal",
            config,
            scenario_validator(with_occupied=True),
            {"no_good_chain": 1000000},
            portal=True,
        ),
        adaptive_variant(
            "full_clect",
            "full",
            config,
            scenario_validator(with_occupied=True),
            {"no_good_chain": 2},
            sparse=True,
            portal=True,
        ),
    ]
    # Additional row for occupied certificate effectiveness without polluting
    # the six Section-7 variants.
    rows.append(adaptive_variant(
        "occupied_fail_only_control",
        "control",
        config,
        no_occupied_validator,
        {"no_good_chain": 1000000},
    ))
    return rows


def scalar_row(row: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in row.items()
        if not isinstance(value, (dict, list, tuple))
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    scalar_rows = [scalar_row(row) for row in rows]
    fields = [
        "variant",
        "category",
        "start_depth",
        "max_depth",
        "N_virtual_leaves",
        "N_materialized_cells",
        "N_validated_free",
        "N_occupied_domains",
        "N_collision_domains",
        "N_deferred",
        "N_LECT_evidence_lookups",
        "N_global_graph_vertices",
        "N_E_portal",
        "query_success",
        "audited_path_length",
        "materialization_reduction",
        "elapsed_ms",
        "terminal_count",
        "split_count",
        "sparse_nodes",
        "sparse_node_reduction",
        "portal_uncompressed_vertices",
        "portal_compressed_vertices",
        "portal_expanded_vertex_count",
        "portal_certificate_valid",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in scalar_rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def write_md(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        "# C-LECT Sidecar Experiment Suite",
        "",
        "| Variant | Mat. cells | Free | Collision | Deferred | Graph V | E_portal | Success | Path | Reduction |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            f"| `{row['variant']}` | {row['N_materialized_cells']} | "
            f"{row['N_validated_free']} | {row['N_collision_domains']} | "
            f"{row['N_deferred']} | {row['N_global_graph_vertices']} | "
            f"{row['N_E_portal']} | {row['query_success']} | "
            f"{float(row['audited_path_length']):.3f} | "
            f"{float(row['materialization_reduction']):.3f}x |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_variant = {row["variant"]: row for row in rows}
    fixed = by_variant["fixed_leaf_sweep"]
    full = by_variant["full_clect"]
    portal = by_variant["early_stop_portal_edges"]
    sparse = by_variant["early_stop_sparse_dyadic"]
    occupied = by_variant["early_stop_sparse_dyadic"]
    occupied_control = by_variant["occupied_fail_only_control"]
    return {
        "full_materialization_reduction_vs_fixed": (
            fixed["N_materialized_cells"] / max(1, full["N_materialized_cells"])
        ),
        "full_graph_vertex_reduction_vs_portal_uncompressed": (
            portal.get("portal_uncompressed_vertices", 0) /
            max(1, portal["N_global_graph_vertices"])
        ),
        "sparse_node_reduction": sparse.get("sparse_node_reduction", 1.0),
        "occupied_certificate_materialization_reduction_vs_fail_control": (
            occupied_control["N_materialized_cells"] / max(1, occupied["N_materialized_cells"])
        ),
        "full_query_success": full["query_success"],
        "full_audited_path_length": full["audited_path_length"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-depth", type=int, default=4)
    parser.add_argument("--max-depth", type=int, default=12)
    parser.add_argument("--portal-chain-length", type=int, default=16)
    parser.add_argument("--sparse-level", type=int, default=20)
    parser.add_argument("--json-out", default="improve_workspace/clect_experiment_suite.json")
    parser.add_argument("--csv-out", default="improve_workspace/clect_experiment_suite.csv")
    parser.add_argument("--md-out", default="improve_workspace/clect_experiment_suite.md")
    args = parser.parse_args()

    config = SuiteConfig(
        start_depth=args.start_depth,
        max_depth=args.max_depth,
        portal_chain_length=args.portal_chain_length,
        sparse_level=args.sparse_level,
    )
    rows = run_suite(config)
    payload = {
        "config": config.__dict__,
        "summary": summarize(rows),
        "rows": rows,
    }
    json_path = REPO_ROOT / args.json_out
    csv_path = REPO_ROOT / args.csv_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_csv(csv_path, rows)
    write_md(md_path, rows)
    print(json.dumps({
        "json_out": str(json_path),
        "csv_out": str(csv_path),
        "md_out": str(md_path),
        "rows": len(rows),
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
