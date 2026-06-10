#!/usr/bin/env python3
"""Four-way HiPaC comparison from docs/分级partition连通.md section 18."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import deque
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from improve_workspace.clect_sidecar import (  # noqa: E402
    AdaptiveLeafSweep,
    AdaptiveSweepConfig,
    Blocker,
    DyadicAddress,
    FailStage,
    HiPaCConfig,
    HierarchicalPartitionConnectivity,
    ValidationReport,
)
from improve_workspace.clect_sidecar.dyadic import boxes_touch_or_overlap, split_schedule_cells  # noqa: E402
from improve_workspace.clect_sidecar.synthetic import unit_root  # noqa: E402


def corridor_validator(address: DyadicAddress, box):
    lo = box[0].lo
    hi = box[0].hi
    if hi <= 0.25 + 1e-12:
        return ValidationReport.free("left")
    if lo >= 0.5 - 1e-12:
        return ValidationReport.free("right")
    if lo >= 0.25 - 1e-12 and hi <= 0.5 + 1e-12 and address.depth >= 3:
        return ValidationReport.free("hidden_mid")
    blocker = Blocker(0, 0, FailStage.GJK, margin=-0.1, overlap_score=0.5, affected_joints=(0,))
    return ValidationReport.fail([blocker], overlap_score=0.5)


def components_for_boxes(boxes) -> int:
    if not boxes:
        return 0
    adj = {i: [] for i in range(len(boxes))}
    for i, lhs in enumerate(boxes):
        for j, rhs in enumerate(boxes[i + 1 :], start=i + 1):
            if boxes_touch_or_overlap(lhs, rhs, tol=1e-12):
                adj[i].append(j)
                adj[j].append(i)
    seen: set[int] = set()
    count = 0
    for node in adj:
        if node in seen:
            continue
        count += 1
        queue = deque([node])
        seen.add(node)
        while queue:
            current = queue.popleft()
            for nxt in adj[current]:
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
    return count


def fixed_leaf_variant(max_depth: int) -> dict[str, object]:
    root = unit_root(1)
    leaves = split_schedule_cells(1, max_depth)
    free_boxes = []
    validation_counts: dict[str, int] = {}
    for address in leaves:
        report = corridor_validator(address, address.interval_box(root))
        validation_counts[report.status.value] = validation_counts.get(report.status.value, 0) + 1
        if report.status.value == "FREE":
            free_boxes.append(address.interval_box(root))
    return {
        "variant": "fixed_leaf_sweep",
        "materialized_cells": len(leaves),
        "global_graph_vertices": len(free_boxes),
        "certified_components": components_for_boxes(free_boxes),
        "resolved_portal_pairs": 0,
        "query_success": components_for_boxes(free_boxes) == 1,
        "online_mixed_cells_refined": 0,
        "validation_counts": validation_counts,
    }


def early_stop_variant(max_depth: int) -> dict[str, object]:
    root = unit_root(1)
    result = AdaptiveLeafSweep(
        root,
        corridor_validator,
        AdaptiveSweepConfig(start_depth=2, max_depth=max_depth, max_evaluations=128),
    ).run()
    free_boxes = [cell.box for cell in result.free]
    return {
        "variant": "early_stop_adaptive_sweep",
        "materialized_cells": result.terminal_count,
        "global_graph_vertices": len(free_boxes),
        "certified_components": components_for_boxes(free_boxes),
        "resolved_portal_pairs": 0,
        "query_success": components_for_boxes(free_boxes) == 1,
        "online_mixed_cells_refined": 0,
        "validation_counts": dict(result.validation_counts),
    }


def hipac_variant(max_depth: int, *, compressed: bool) -> dict[str, object]:
    planner = HierarchicalPartitionConnectivity(
        unit_root(1),
        corridor_validator,
        HiPaCConfig(coarse_depth=2, max_depth=max_depth, max_refinement_iterations=4, local_refine_budget=8),
    )
    build = planner.build()
    query = planner.query((0.1,), (0.8,), online_budget=2)
    hidden_cells = sum(len(corridor.internal_cells) for corridor in planner.portal_corridors.values())
    return {
        "variant": "hierarchical_partition_connectivity_portal_compressed" if compressed else "hierarchical_partition_connectivity_uncompressed",
        "materialized_cells": len(planner.cells) + hidden_cells,
        "global_graph_vertices": build.certified_graph_nodes if compressed else build.certified_graph_nodes + hidden_cells,
        "certified_components": build.certified_components,
        "resolved_portal_pairs": build.resolved_portal_pairs,
        "query_success": query.success,
        "online_mixed_cells_refined": query.refined_mixed_cells,
        "used_portal_edges": query.used_portal_edges,
        "expanded_path_len": len(query.expanded_path),
        "validation_counts": dict(build.validation_counts),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    keys = sorted({key for row in rows for key in row if key != "validation_counts"})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in keys})


def write_md(path: Path, rows: list[dict[str, object]]) -> None:
    lines = [
        "# HiPaC Experiment Suite",
        "",
        "Four-way sidecar comparison requested by `docs/分级partition连通.md` section 18.",
        "",
        "| Variant | Materialized | Global vertices | Components | Portals | Query |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            f"| {row['variant']} | {row['materialized_cells']} | {row['global_graph_vertices']} | "
            f"{row['certified_components']} | {row['resolved_portal_pairs']} | {row['query_success']} |"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-depth", type=int, default=4)
    parser.add_argument("--json-out", default="improve_workspace/hipac_experiment_suite.json")
    parser.add_argument("--csv-out", default="improve_workspace/hipac_experiment_suite.csv")
    parser.add_argument("--md-out", default="improve_workspace/hipac_experiment_suite.md")
    args = parser.parse_args()

    rows = [
        fixed_leaf_variant(int(args.max_depth)),
        early_stop_variant(int(args.max_depth)),
        hipac_variant(int(args.max_depth), compressed=False),
        hipac_variant(int(args.max_depth), compressed=True),
    ]
    ok = (
        len(rows) == 4
        and rows[-1]["query_success"]
        and rows[-1]["resolved_portal_pairs"] >= 1
        and rows[-1]["global_graph_vertices"] < rows[-2]["global_graph_vertices"]
    )
    payload = {
        "ok": ok,
        "source_plan": "docs/分级partition连通.md",
        "section": "18",
        "rows": rows,
    }
    json_path = REPO_ROOT / args.json_out
    csv_path = REPO_ROOT / args.csv_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_csv(csv_path, rows)
    write_md(md_path, rows)
    print(json.dumps({"ok": ok, "json_out": str(json_path), "csv_out": str(csv_path), "md_out": str(md_path)}, indent=2, ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
