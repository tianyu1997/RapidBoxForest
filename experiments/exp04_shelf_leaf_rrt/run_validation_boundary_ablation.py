#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.rbf_defaults import DEFAULT_RBF_SHELF_BOX_BUDGET, DEFAULT_RBF_THREADS


RUNNER = REPO_ROOT / "experiments" / "exp04_shelf_leaf_rrt" / "run_shelf_leaf_rrt.py"
BASELINE_CASE = "baseline_d23_aafk_support_hull_8t"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the Exp.4 Shelf+IIWA validation-boundary ablation. The protocol "
            "reuses the registered d23 baseline and progressively disables query "
            "bridge repair, final smoothing, endpoint anchoring, and connector "
            "assist where the current runner exposes switches."
        )
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp04_validation_boundary_ablation",
    )
    parser.add_argument("--seeds", default="0")
    parser.add_argument("--box-budget", type=int, default=DEFAULT_RBF_SHELF_BOX_BUDGET)
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full"], default="smoke")
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--leaf-start-depth", type=int, default=8)
    parser.add_argument("--leaf-max-depth", type=int, default=13)
    parser.add_argument("--adaptive-target-depth", type=int, default=13)
    parser.add_argument("--adaptive-grid-target-depth", type=int, default=13)
    parser.add_argument("--adaptive-time-budget-ms", type=float, default=60000.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def base_command(args: argparse.Namespace, out_dir: Path) -> list[str]:
    return [
        sys.executable,
        str(RUNNER),
        "--out-dir",
        str(out_dir),
        "--phase",
        str(args.phase),
        "--only",
        BASELINE_CASE,
        "--seeds",
        str(args.seeds),
        "--box-budgets",
        str(int(args.box_budget)),
        "--threads",
        str(int(args.threads)),
        "--timeout-ms",
        str(float(args.timeout_ms)),
        "--leaf-start-depth",
        str(int(args.leaf_start_depth)),
        "--leaf-max-depth",
        str(int(args.leaf_max_depth)),
        "--adaptive-target-depth",
        str(int(args.adaptive_target_depth)),
        "--adaptive-grid-target-depth",
        str(int(args.adaptive_grid_target_depth)),
        "--adaptive-time-budget-ms",
        str(float(args.adaptive_time_budget_ms)),
    ]


NO_QUERY_BRIDGE_ARGS = [
    "--no-query-bridge-all",
    "--no-query-bridge-adaptive-all",
    "--query-bridge-labels",
    "",
    "--query-bridge-force-indices",
    "",
    "--no-query-bridge-force-selected",
    "--query-bridge-forced-attempts",
    "0",
    "--query-bridge-no-path-retry-attempts",
    "0",
    "--query-bridge-adaptive-max-repair-subdivisions",
    "0",
    "--query-bridge-adaptive-max-repair-calls",
    "0",
    "--no-query-bridge-adaptive-step-repair",
]

NO_FINAL_SMOOTHING_ARGS = [
    "--no-final-collision-shortcut",
    "--no-final-rrt-simplify",
]

SEGMENT_SUPPRESSION_ARGS = [
    "--no-query-endpoint-anchor-before-bridge",
    "--no-connector-birrt",
    "--connector-bridge-boxes",
    "0",
    "--connector-max-pairs-per-gap",
    "0",
    "--connector-pair-timeout-ms",
    "0",
    "--connector-rrt-iters",
    "0",
    "--connector-rrt-timeout-ms",
    "0",
    "--connector-pave-steps",
    "0",
    "--connector-pave-max-chain",
    "0",
    "--connector-adaptive-min-segment-fraction",
    "1.0",
    "--segment-edges-fallback-only",
    "--offline-shortcut-edges",
    "0",
]


def variants() -> list[dict[str, Any]]:
    return [
        {
            "variant": "registered_full_profile",
            "category": "reported_profile",
            "description": (
                "Registered Shelf+IIWA d23 baseline, including endpoint anchoring, "
                "query bridge repair, segment witnesses, and final simplification."
            ),
            "extra_args": [],
        },
        {
            "variant": "no_query_bridge_no_smoothing",
            "category": "no_endpoint_to_endpoint_repair",
            "description": (
                "Registered d23 baseline with endpoint-to-endpoint query bridge, "
                "adaptive repair, collision shortcut, and final RRT simplification disabled. "
                "Endpoint anchoring and build-time connector behavior remain enabled."
            ),
            "extra_args": NO_QUERY_BRIDGE_ARGS + NO_FINAL_SMOOTHING_ARGS,
        },
        {
            "variant": "segment_suppressed_no_query_assist",
            "category": "runner_exposed_segment_suppression",
            "description": (
                "Adds the runner-exposed connector/segment suppression switches and disables "
                "query endpoint anchoring. This is a boundary probe, not a theorem-grade "
                "conservative-only mode, because the current core runner has no global "
                "segment-edge-disable contract."
            ),
            "extra_args": NO_QUERY_BRIDGE_ARGS + NO_FINAL_SMOOTHING_ARGS + SEGMENT_SUPPRESSION_ARGS,
        },
    ]


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def numeric(row: dict[str, Any], key: str) -> float:
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def run_variant(args: argparse.Namespace, variant: dict[str, Any]) -> dict[str, Any]:
    out_dir = args.out_dir / str(variant["variant"])
    command = base_command(args, out_dir) + list(variant["extra_args"])
    if args.dry_run:
        return {
            "variant": variant["variant"],
            "category": variant["category"],
            "description": variant["description"],
            "out_dir": str(out_dir),
            "command": command,
            "returncode": 0,
            "elapsed_s": 0.0,
            "dry_run": True,
        }
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    t0 = time.perf_counter()
    proc = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed = time.perf_counter() - t0
    summary_path = out_dir / "shelf_leaf_rrt_summary.csv"
    manifest_path = out_dir / "shelf_leaf_rrt_manifest.json"
    summary_rows = read_csv_rows(summary_path) if summary_path.exists() else []
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    return {
        "variant": variant["variant"],
        "category": variant["category"],
        "description": variant["description"],
        "out_dir": str(out_dir),
        "command": command,
        "returncode": int(proc.returncode),
        "elapsed_s": elapsed,
        "stdout_tail": proc.stdout[-4000:],
        "stderr_tail": proc.stderr[-4000:],
        "summary_path": str(summary_path),
        "manifest_path": str(manifest_path),
        "summary": summary_rows,
        "manifest_status": manifest.get("status"),
    }


def compact_row(result: dict[str, Any]) -> dict[str, Any]:
    summary = result.get("summary") or []
    row = summary[0] if summary else {}
    return {
        "variant": result["variant"],
        "category": result["category"],
        "returncode": int(result["returncode"]),
        "success_queries": row.get("success_queries", ""),
        "total_queries": row.get("total_queries", ""),
        "offline_build_s": numeric(row, "offline_build_s_median"),
        "online_per_query_s": numeric(row, "online_per_query_s_median"),
        "online_total_per_query_s": numeric(row, "online_total_per_query_s_median"),
        "audit_s": numeric(row, "audit_s_median"),
        "path_length_mean": numeric(row, "path_length_mean"),
        "raw_segment_fraction": numeric(row, "raw_segment_fraction_median"),
        "final_boxes": numeric(row, "final_boxes_median"),
        "final_segment_edges": numeric(row, "final_segment_edges_median"),
        "offline_segment_edges_added": numeric(row, "offline_segment_edges_added_median"),
        "query_bridge_per_query_s": numeric(row, "query_bridge_per_query_s_median"),
        "query_bridge_attempts": numeric(row, "query_bridge_attempts_median"),
        "query_bridge_added": numeric(row, "query_bridge_added_reported_median"),
        "query_bridge_boxes_added_observed": numeric(row, "query_bridge_boxes_added_observed_median"),
        "query_bridge_segment_edges_added_observed": numeric(row, "query_bridge_segment_edges_added_observed_median"),
        "direct_corridor_repair_calls": numeric(row, "diag_query_bridge_direct_corridor_repair_calls_total_median"),
        "adaptive_repair_calls": numeric(row, "diag_query_bridge_direct_corridor_adaptive_repair_calls_total_median"),
        "external_reused_hits": numeric(row, "external_reused_hits_median"),
        "elapsed_s": float(result["elapsed_s"]),
        "out_dir": result["out_dir"],
    }


def write_compact_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def render_float(value: Any) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return ""
    if number != number:
        return ""
    return f"{number:.6g}"


def write_summary_md(
    path: Path,
    rows: list[dict[str, Any]],
    results: list[dict[str, Any]],
    dry_run: bool,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Exp.4 Validation-Boundary Ablation",
        "",
        "This protocol inherits the registered Shelf+IIWA d23 baseline and varies only runner-exposed validation-boundary switches.",
        "It is designed to answer reviewer questions about how much success depends on query bridge repair, segment witnesses, endpoint anchoring, and final smoothing.",
        "",
        "Important limitation: the current core runner does not expose a theorem-grade global conservative-only switch. The `segment_suppressed_no_query_assist` row is therefore a boundary probe, not a formal conservative-box-only certificate experiment.",
        "",
    ]
    if dry_run:
        lines.append("Status: dry run only; commands were written but not executed.")
    else:
        lines.append("Status: executed; compact summary below.")
    lines.extend([
        "",
        "| Variant | SR | Build s | Online/q s | Total/q s | Audit s | Path | Seg. frac. | Seg. edges | Bridge added | Repair calls | Adaptive repair |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in rows:
        sr = f"{row.get('success_queries', '')}/{row.get('total_queries', '')}".strip("/")
        lines.append(
            f"| {row['variant']} | {sr} | {render_float(row['offline_build_s'])} | "
            f"{render_float(row['online_per_query_s'])} | {render_float(row['online_total_per_query_s'])} | "
            f"{render_float(row['audit_s'])} | {render_float(row['path_length_mean'])} | "
            f"{render_float(row['raw_segment_fraction'])} | {render_float(row['final_segment_edges'])} | "
            f"{render_float(row['query_bridge_added'])} | {render_float(row['direct_corridor_repair_calls'])} | "
            f"{render_float(row['adaptive_repair_calls'])} |"
        )
    lines.extend(["", "## Commands", ""])
    for result in results:
        command = result.get("command", [])
        if not isinstance(command, list):
            continue
        lines.extend([
            f"### {result['variant']}",
            "",
            "```bash",
            shlex.join(str(item) for item in command),
            "```",
            "",
        ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    selected_variants = variants()
    results = [run_variant(args, variant) for variant in selected_variants]
    compact = [compact_row(result) for result in results]
    payload = {
        "experiment": "exp04_validation_boundary_ablation",
        "run_id": run_id("exp04_validation_boundary"),
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "config": {
            "inherited_stage": "Exp.4 Shelf+IIWA registered d23 baseline",
            "baseline_case": BASELINE_CASE,
            "seeds": str(args.seeds),
            "box_budget": int(args.box_budget),
            "threads": int(args.threads),
            "phase": str(args.phase),
            "timeout_ms": float(args.timeout_ms),
            "leaf_start_depth": int(args.leaf_start_depth),
            "leaf_max_depth": int(args.leaf_max_depth),
            "adaptive_target_depth": int(args.adaptive_target_depth),
            "adaptive_grid_target_depth": int(args.adaptive_grid_target_depth),
            "protocol_limitation": (
                "Current runner exposes suppression switches for query bridge, smoothing, "
                "endpoint anchoring, and connector assist, but not a formal global "
                "conservative-only mode."
            ),
        },
        "variants": selected_variants,
        "results": results,
        "compact": compact,
    }
    write_json(args.out_dir / "validation_boundary_ablation.json", payload)
    write_compact_csv(args.out_dir / "validation_boundary_ablation.csv", compact)
    write_summary_md(args.out_dir / "validation_boundary_ablation.md", compact, results, bool(args.dry_run))
    print(f"wrote {args.out_dir / 'validation_boundary_ablation.json'}")
    return 0 if all(int(result["returncode"]) == 0 for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
