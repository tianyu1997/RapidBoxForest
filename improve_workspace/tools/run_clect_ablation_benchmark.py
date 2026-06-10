#!/usr/bin/env python3
"""Ablation benchmark for the C-LECT sidecar implementation.

The benchmark is synthetic by design: it measures the algorithmic effects
requested by docs/improve.md without relying on production RBF planner state.
It emits machine-readable JSON/CSV plus a short Markdown summary under
improve_workspace/.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
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
    MaterialWitness,
    Portal,
    PortalCorridor,
    PortalGraph,
    SparseNodeMap,
    ValidationReport,
    occupied_report_from_witness,
)
from improve_workspace.clect_sidecar.reports import Blocker, FailStage
from improve_workspace.clect_sidecar.synthetic import (
    Region,
    SyntheticValidator,
    fixed_leaf_evaluation_count,
    unit_root,
)


def timed(callable_obj):
    start = time.perf_counter()
    value = callable_obj()
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return value, elapsed_ms


def fixed_virtual_count(start_depth: int, max_depth: int) -> int:
    return (1 << start_depth) * fixed_leaf_evaluation_count(start_depth, max_depth)


def sweep_validator(with_occupied: bool = True):
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


def run_adaptive_sweep_case(
    name: str,
    start_depth: int,
    max_depth: int,
    validator,
    config_overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    root = unit_root(2)
    config = AdaptiveSweepConfig(
        start_depth=start_depth,
        max_depth=max_depth,
        max_evaluations=200000,
    )
    for key, value in (config_overrides or {}).items():
        setattr(config, key, value)
    sweep = AdaptiveLeafSweep(root, validator, config)
    result, elapsed_ms = timed(sweep.run)
    fixed = fixed_virtual_count(start_depth, max_depth)
    return {
        "case": name,
        "category": "adaptive_sweep",
        "start_depth": start_depth,
        "max_depth": max_depth,
        "fixed_virtual_leaf_evaluations": fixed,
        "evaluations": result.evaluated,
        "terminal_count": result.terminal_count,
        "free_terminal_count": len(result.free),
        "occupied_terminal_count": len(result.occupied),
        "collision_terminal_count": len(result.collision),
        "deferred_terminal_count": len(result.deferred),
        "covered_terminal_count": len(result.covered),
        "split_count": result.split_count,
        "eval_reduction": fixed / max(1, result.evaluated),
        "elapsed_ms": elapsed_ms,
        "depth_histogram": dict(sorted(result.depth_histogram.items())),
        "validation_counts": dict(result.validation_counts),
    }


def no_good_validator(_address: DyadicAddress, box):
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


def no_occupied_validator(address: DyadicAddress, box):
    report = sweep_validator(with_occupied=True)(address, box)
    if report.occupied_certificate is None:
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


def run_sparse_case(deep_level: int, count: int) -> dict[str, Any]:
    root = unit_root(2)
    tree = SparseNodeMap(2)
    points = []
    for index in range(count):
        x = (index + 0.5) / float(count)
        y = ((index * 37) % count + 0.5) / float(count)
        points.append((x, y))

    def materialize_sparse() -> int:
        for index, point in enumerate(points):
            tree.jump_materialize(
                root,
                point,
                (deep_level, deep_level),
                evidence_ref=f"e{index}",
                terminal=True,
            )
        return len(tree)

    sparse_nodes, elapsed_ms = timed(materialize_sparse)
    heap_path_nodes = sum((2 * deep_level) + 1 for _ in points)
    return {
        "case": "sparse_jump_materialization",
        "category": "sparse_tree",
        "deep_level": deep_level,
        "points": count,
        "heap_path_nodes_if_materialized": heap_path_nodes,
        "sparse_nodes": sparse_nodes,
        "node_reduction": heap_path_nodes / max(1, sparse_nodes),
        "elapsed_ms": elapsed_ms,
    }


def run_portal_case(chain_length: int) -> dict[str, Any]:
    root = (Interval(0.0, float(chain_length + 2)),)
    pin_box = (Interval(0.0, 1.0),)
    pout_box = (Interval(float(chain_length + 1), float(chain_length + 2)),)
    internal_boxes = [
        (Interval(float(i + 1), float(i + 2)),)
        for i in range(chain_length)
    ]
    internal_cells = [
        DyadicAddress.root(1)
        for _ in range(chain_length)
    ]
    reports = [ValidationReport.free(f"cert:{i}") for i in range(chain_length)]
    pin = Portal("domain", "pin", "c0")
    pout = Portal("domain", "pout", "c1")
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

    def add_and_expand() -> list[str]:
        graph.add_portal_corridor(corridor, pin_box, pout_box)
        return graph.expand_edge("portal_corridor")

    expanded, elapsed_ms = timed(add_and_expand)
    return {
        "case": "portal_edge_compression",
        "category": "portal",
        "internal_chain_length": chain_length,
        "global_vertices_uncompressed": chain_length + 2,
        "global_vertices_compressed": 2,
        "global_edges_compressed": 1,
        "expanded_vertex_count_when_used": len(expanded),
        "vertex_compression": (chain_length + 2) / 2.0,
        "certificate_valid": corridor.validate_certificate(pin_box, pout_box),
        "elapsed_ms": elapsed_ms,
    }


def run_occupied_witness_case() -> dict[str, Any]:
    blocker = Blocker(0, 1, FailStage.SDF, margin=-0.1, overlap_score=1.0, affected_joints=(0,))
    strong = MaterialWitness(0, 1, center_signed_distance=-0.1, motion_bound=0.02, epsilon_num=1e-6)
    weak = MaterialWitness(0, 1, center_signed_distance=-0.01, motion_bound=0.02, epsilon_num=1e-6)
    strong_report = occupied_report_from_witness(strong, blocker)
    weak_report = occupied_report_from_witness(weak, blocker)
    return {
        "case": "occupied_witness_predicate",
        "category": "occupied_certificate",
        "strong_certifies": strong_report is not None,
        "weak_certifies": weak_report is not None,
        "strong_margin_after_motion": strong.center_signed_distance + strong.motion_bound + strong.epsilon_num,
        "weak_margin_after_motion": weak.center_signed_distance + weak.motion_bound + weak.epsilon_num,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    scalar_rows = []
    for row in rows:
        scalar_rows.append({
            key: value
            for key, value in row.items()
            if not isinstance(value, (dict, list, tuple))
        })
    fields = sorted({key for row in scalar_rows for key in row})
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in scalar_rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def write_md(path: Path, rows: list[dict[str, Any]]) -> None:
    by_case = {str(row["case"]): row for row in rows}
    early = by_case.get("early_stop_free", {})
    occupied_cert = by_case.get("occupied_as_certificate", {})
    occupied_fail = by_case.get("occupied_as_fail", {})
    no_good_on = by_case.get("no_good_enabled", {})
    no_good_off = by_case.get("no_good_disabled", {})
    sparse = by_case.get("sparse_jump_materialization", {})
    portal = by_case.get("portal_edge_compression", {})
    witness = by_case.get("occupied_witness_predicate", {})

    def ratio(num: Any, den: Any) -> float:
        try:
            return float(num) / max(1.0, float(den))
        except (TypeError, ValueError):
            return 0.0

    lines = [
        "# C-LECT Sidecar Ablation Benchmark",
        "",
        "| Mechanism | Key metric | Value |",
        "| --- | ---: | ---: |",
    ]
    if early:
        lines.append(
            f"| `early_stop_free` | fixed leaf evals / adaptive evals | "
            f"{float(early['eval_reduction']):.3f}x |"
        )
    if occupied_cert and occupied_fail:
        lines.append(
            f"| `occupied_as_certificate` | fail-only evals / cert evals | "
            f"{ratio(occupied_fail.get('evaluations'), occupied_cert.get('evaluations')):.3f}x |"
        )
    if no_good_on and no_good_off:
        lines.append(
            f"| `no_good_enabled` | disabled evals / enabled evals | "
            f"{ratio(no_good_off.get('evaluations'), no_good_on.get('evaluations')):.3f}x |"
        )
    if sparse:
        lines.append(
            f"| `sparse_jump_materialization` | heap path nodes / sparse nodes | "
            f"{float(sparse['node_reduction']):.3f}x |"
        )
    if portal:
        lines.append(
            f"| `portal_edge_compression` | uncompressed vertices / global vertices | "
            f"{float(portal['vertex_compression']):.3f}x |"
        )
    if witness:
        lines.append(
            f"| `occupied_witness_predicate` | strong/weak witness certifies | "
            f"{witness['strong_certifies']}/{witness['weak_certifies']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize(rows: list[dict[str, Any]]) -> dict[str, float | bool]:
    by_case = {str(row["case"]): row for row in rows}

    def ratio(num: Any, den: Any) -> float:
        try:
            return float(num) / max(1.0, float(den))
        except (TypeError, ValueError):
            return 0.0

    early = by_case.get("early_stop_free", {})
    occupied_cert = by_case.get("occupied_as_certificate", {})
    occupied_fail = by_case.get("occupied_as_fail", {})
    no_good_on = by_case.get("no_good_enabled", {})
    no_good_off = by_case.get("no_good_disabled", {})
    sparse = by_case.get("sparse_jump_materialization", {})
    portal = by_case.get("portal_edge_compression", {})
    witness = by_case.get("occupied_witness_predicate", {})
    return {
        "early_stop_eval_reduction": float(early.get("eval_reduction", 0.0)),
        "occupied_certificate_eval_reduction_vs_fail": ratio(
            occupied_fail.get("evaluations"),
            occupied_cert.get("evaluations"),
        ),
        "no_good_eval_reduction_vs_disabled": ratio(
            no_good_off.get("evaluations"),
            no_good_on.get("evaluations"),
        ),
        "sparse_node_reduction": float(sparse.get("node_reduction", 0.0)),
        "portal_vertex_compression": float(portal.get("vertex_compression", 0.0)),
        "occupied_witness_strong_certifies": bool(witness.get("strong_certifies", False)),
        "occupied_witness_weak_certifies": bool(witness.get("weak_certifies", False)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-depth", type=int, default=4)
    parser.add_argument("--max-depth", type=int, default=12)
    parser.add_argument("--sparse-level", type=int, default=20)
    parser.add_argument("--sparse-points", type=int, default=64)
    parser.add_argument("--portal-chain-length", type=int, default=16)
    parser.add_argument("--json-out", default="improve_workspace/clect_ablation_benchmark.json")
    parser.add_argument("--csv-out", default="improve_workspace/clect_ablation_benchmark.csv")
    parser.add_argument("--md-out", default="improve_workspace/clect_ablation_benchmark.md")
    args = parser.parse_args()

    rows: list[dict[str, Any]] = []
    rows.append(run_adaptive_sweep_case(
        "early_stop_free",
        args.start_depth,
        args.max_depth,
        sweep_validator(with_occupied=False),
        {"no_good_chain": 1000000},
    ))
    rows.append(run_adaptive_sweep_case(
        "occupied_as_certificate",
        args.start_depth,
        args.max_depth,
        sweep_validator(with_occupied=True),
        {"no_good_chain": 1000000},
    ))
    rows.append(run_adaptive_sweep_case(
        "occupied_as_fail",
        args.start_depth,
        args.max_depth,
        no_occupied_validator,
        {"no_good_chain": 1000000},
    ))
    rows.append(run_adaptive_sweep_case(
        "no_good_enabled",
        0,
        args.max_depth,
        no_good_validator,
        {"no_good_chain": 2},
    ))
    rows.append(run_adaptive_sweep_case(
        "no_good_disabled",
        0,
        args.max_depth,
        no_good_validator,
        {"no_good_chain": 1000000},
    ))
    rows.append(run_sparse_case(args.sparse_level, args.sparse_points))
    rows.append(run_portal_case(args.portal_chain_length))
    rows.append(run_occupied_witness_case())

    payload = {
        "config": {
            "start_depth": args.start_depth,
            "max_depth": args.max_depth,
            "sparse_level": args.sparse_level,
            "sparse_points": args.sparse_points,
            "portal_chain_length": args.portal_chain_length,
        },
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
