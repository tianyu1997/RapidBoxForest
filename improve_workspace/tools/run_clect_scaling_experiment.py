#!/usr/bin/env python3
"""Scaling experiment for C-LECT sidecar mechanisms.

This complements the single-point ablation benchmark by sweeping depth and
reporting how each mechanism changes materialization, graph size, and runtime.
It is synthetic and sidecar-only; it does not claim production RBF performance.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from run_clect_ablation_benchmark import (  # noqa: E402
    no_good_validator,
    no_occupied_validator,
    run_adaptive_sweep_case,
    sweep_validator,
)
from run_clect_experiment_suite import SuiteConfig, run_suite  # noqa: E402


def parse_depths(text: str) -> list[int]:
    depths = [int(item.strip()) for item in text.split(",") if item.strip()]
    if not depths:
        raise ValueError("depth list is empty")
    if any(depth < 0 for depth in depths):
        raise ValueError("negative depth")
    return sorted(dict.fromkeys(depths))


def scalarize(row: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in row.items()
        if not isinstance(value, (dict, list, tuple, set))
    }


def normalize_suite_row(depth: int, row: dict[str, Any]) -> dict[str, Any]:
    return {
        "experiment": "section7_suite",
        "depth": depth,
        "variant": row["variant"],
        "N_virtual_leaves": row["N_virtual_leaves"],
        "N_materialized_cells": row["N_materialized_cells"],
        "N_validated_free": row["N_validated_free"],
        "N_collision_domains": row["N_collision_domains"],
        "N_deferred": row["N_deferred"],
        "N_global_graph_vertices": row["N_global_graph_vertices"],
        "N_E_portal": row["N_E_portal"],
        "query_success": row["query_success"],
        "audited_path_length": row["audited_path_length"],
        "materialization_reduction": row["materialization_reduction"],
        "elapsed_ms": row["elapsed_ms"],
    }


def normalize_isolated_row(depth: int, row: dict[str, Any]) -> dict[str, Any]:
    return {
        "experiment": "isolated_mechanism",
        "depth": depth,
        "variant": row["case"],
        "N_virtual_leaves": row["fixed_virtual_leaf_evaluations"],
        "N_materialized_cells": row["evaluations"],
        "N_validated_free": row["free_terminal_count"],
        "N_collision_domains": row["collision_terminal_count"],
        "N_deferred": row["deferred_terminal_count"],
        "N_global_graph_vertices": row["free_terminal_count"],
        "N_E_portal": 0,
        "query_success": row["free_terminal_count"] > 0,
        "audited_path_length": "",
        "materialization_reduction": row["eval_reduction"],
        "elapsed_ms": row["elapsed_ms"],
    }


def isolated_mechanism_rows(start_depth: int, max_depth: int) -> list[dict[str, Any]]:
    common = {"no_good_chain": 1000000}
    return [
        run_adaptive_sweep_case(
            "isolated_early_stop_free",
            start_depth,
            max_depth,
            sweep_validator(with_occupied=False),
            common,
        ),
        run_adaptive_sweep_case(
            "isolated_occupied_certificate",
            start_depth,
            max_depth,
            sweep_validator(with_occupied=True),
            common,
        ),
        run_adaptive_sweep_case(
            "isolated_occupied_fail_only",
            start_depth,
            max_depth,
            no_occupied_validator,
            common,
        ),
        run_adaptive_sweep_case(
            "isolated_no_good_enabled",
            start_depth,
            max_depth,
            no_good_validator,
            {"no_good_chain": 2},
        ),
        run_adaptive_sweep_case(
            "isolated_no_good_disabled",
            start_depth,
            max_depth,
            no_good_validator,
            common,
        ),
    ]


def run_scaling(
    depths: list[int],
    start_depth: int,
    portal_chain_length: int,
    sparse_level: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    for depth in depths:
        config = SuiteConfig(
            start_depth=start_depth,
            max_depth=depth,
            portal_chain_length=portal_chain_length,
            sparse_level=sparse_level,
        )
        suite_rows = run_suite(config)
        normalized_suite = [normalize_suite_row(depth, row) for row in suite_rows]
        rows.extend(normalized_suite)
        rows.extend(normalize_isolated_row(depth, row) for row in isolated_mechanism_rows(start_depth, depth))

        by_variant = {row["variant"]: row for row in normalized_suite}
        isolated = {
            row["variant"]: row
            for row in rows
            if row["experiment"] == "isolated_mechanism" and row["depth"] == depth
        }
        fixed = by_variant["fixed_leaf_sweep"]
        early = by_variant["early_stop_free_only"]
        sparse = by_variant["early_stop_sparse_dyadic"]
        portal = by_variant["early_stop_portal_edges"]
        full = by_variant["full_clect"]
        occupied_cert = isolated["isolated_occupied_certificate"]
        occupied_fail = isolated["isolated_occupied_fail_only"]
        no_good_enabled = isolated["isolated_no_good_enabled"]
        no_good_disabled = isolated["isolated_no_good_disabled"]
        summaries.append({
            "depth": depth,
            "fixed_materialized_cells": fixed["N_materialized_cells"],
            "early_stop_materialized_cells": early["N_materialized_cells"],
            "early_stop_reduction_vs_fixed": (
                fixed["N_materialized_cells"] / max(1, early["N_materialized_cells"])
            ),
            "occupied_certificate_reduction_vs_fail_only": (
                occupied_fail["N_materialized_cells"] / max(1, occupied_cert["N_materialized_cells"])
            ),
            "no_good_reduction_vs_disabled": (
                no_good_disabled["N_materialized_cells"] / max(1, no_good_enabled["N_materialized_cells"])
            ),
            "sparse_graph_vertices": sparse["N_global_graph_vertices"],
            "portal_graph_vertices": portal["N_global_graph_vertices"],
            "full_materialized_cells": full["N_materialized_cells"],
            "full_graph_vertices": full["N_global_graph_vertices"],
            "full_reduction_vs_fixed": (
                fixed["N_materialized_cells"] / max(1, full["N_materialized_cells"])
            ),
            "full_elapsed_ms": full["elapsed_ms"],
            "full_query_success": full["query_success"],
            "full_path_length": full["audited_path_length"],
        })
    return rows, summaries


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    scalar_rows = [scalarize(row) for row in rows]
    fields = sorted({key for row in scalar_rows for key in row})
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in scalar_rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def write_md(path: Path, summaries: list[dict[str, Any]]) -> None:
    lines = [
        "# C-LECT Sidecar Scaling Experiment",
        "",
        "| Depth | Fixed cells | Early cells | Early red. | Occ red. | No-good red. | Full cells | Full graph V | Full red. | Full ms |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summaries:
        lines.append(
            f"| {row['depth']} | {row['fixed_materialized_cells']} | "
            f"{row['early_stop_materialized_cells']} | "
            f"{float(row['early_stop_reduction_vs_fixed']):.3f}x | "
            f"{float(row['occupied_certificate_reduction_vs_fail_only']):.3f}x | "
            f"{float(row['no_good_reduction_vs_disabled']):.3f}x | "
            f"{row['full_materialized_cells']} | {row['full_graph_vertices']} | "
            f"{float(row['full_reduction_vs_fixed']):.3f}x | "
            f"{float(row['full_elapsed_ms']):.3f} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-depth", type=int, default=4)
    parser.add_argument("--depths", default="8,10,12,14,16")
    parser.add_argument("--portal-chain-length", type=int, default=16)
    parser.add_argument("--sparse-level", type=int, default=20)
    parser.add_argument("--json-out", default="improve_workspace/clect_scaling_experiment.json")
    parser.add_argument("--csv-out", default="improve_workspace/clect_scaling_experiment.csv")
    parser.add_argument("--md-out", default="improve_workspace/clect_scaling_experiment.md")
    args = parser.parse_args()

    depths = parse_depths(args.depths)
    rows, summaries = run_scaling(
        depths,
        args.start_depth,
        args.portal_chain_length,
        args.sparse_level,
    )
    payload = {
        "config": {
            "start_depth": args.start_depth,
            "depths": depths,
            "portal_chain_length": args.portal_chain_length,
            "sparse_level": args.sparse_level,
        },
        "summary": {
            "depth_count": len(depths),
            "max_depth": max(depths),
            "best_full_reduction_vs_fixed": max(row["full_reduction_vs_fixed"] for row in summaries),
            "best_no_good_reduction_vs_disabled": max(row["no_good_reduction_vs_disabled"] for row in summaries),
            "best_occupied_certificate_reduction_vs_fail_only": max(
                row["occupied_certificate_reduction_vs_fail_only"] for row in summaries
            ),
        },
        "depth_summaries": summaries,
        "rows": rows,
    }
    json_path = REPO_ROOT / args.json_out
    csv_path = REPO_ROOT / args.csv_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_csv(csv_path, rows)
    write_md(md_path, summaries)
    print(json.dumps({
        "json_out": str(json_path),
        "csv_out": str(csv_path),
        "md_out": str(md_path),
        "depths": depths,
        "rows": len(rows),
        "best_full_reduction_vs_fixed": payload["summary"]["best_full_reduction_vs_fixed"],
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
