#!/usr/bin/env python3
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    csv_list,
    default_sbf_subprocess_env,
    environment_metadata,
    namespace_dict,
    run_command,
    run_id,
    write_json,
)
from experiments.common.anytime_defaults import (  # noqa: E402
    UNIFIED_SBF_SAMPLING_ANCHOR_TARGET_PROB,
    UNIFIED_SBF_SAMPLING_CATEGORICAL_ALLOCATION,
    UNIFIED_SBF_SAMPLING_INTERTREE_GOAL_BIAS,
    UNIFIED_SBF_SAMPLING_RRT_GOAL_BIAS,
    UNIFIED_SBF_SAMPLING_UNEXPLORED_PROB,
    UNIFIED_SBF_SAMPLING_UNIFORM_PROB,
)


SHELF_CASE = REPO_ROOT / "experiments" / "common" / "run_shelf_sbf_case.py"
D23_CACHE_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_d23" / "cache"
D23_CACHE_LABEL = "iiwa_shelf_endpoint_only_p23_canonical_dim0q4_fixed_root"
D23_ROOT_INTERVALS = ";".join([
    "0.0:1.5707963267948966",
    "0.3194:0.8645",
    "-0.5077:0.5073",
    "-1.98947519:-0.33002121",
    "-0.447:0.4473",
    "-1.34734773:1.51007653",
    "1.262:1.8794",
])


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def case_json_path(args: argparse.Namespace, name: str) -> Path:
    return args.out_dir / f"{name}.json"


def base_command(args: argparse.Namespace,
                 name: str,
                 *,
                 coverage_anchor_preset: str = "iris8",
                 use_external_evidence: bool = True,
                 endpoint_evidence_cache: bool = True,
                 segment_edge_policy: str = "normal",
                 final_simplify_s: float = 0.05,
                 max_boxes: int = 280,
                 ffb_depth: int = 28,
                 bootstrap_depth: int = 15,
                 bootstrap_boxes: int = 210,
                 bootstrap_samples: int = 8,
                 component_connect_prob: float = 0.0,
                 component_candidate_limit: int = 1,
                 component_chain_steps: int = 0,
                 component_chain_max_boxes: int = 0,
                 connector_step: float = 0.25,
                 connector_pair_timeout_ms: float = 80.0,
                 connector_max_pairs_per_gap: int = 3) -> list[str]:
    command = [
        sys.executable,
        str(SHELF_CASE),
        "--case-name", name,
        "--out-json", str(case_json_path(args, name)),
        "--database-path", str(args.out_dir / "active_cache" / name),
        "--seeds-list", ",".join(str(index) for index in range(max(1, int(args.seeds)))),
        "--coverage-anchor-preset", coverage_anchor_preset,
        "--lect-root-intervals", D23_ROOT_INTERVALS,
        "--endpoint-source", "aafk",
        "--lect-split-policy", "aafk_volume_min",
        "--rbf-envelope", "support_hull",
        "--rbf-canonical-cache",
        "--rbf-cache-root", str(args.rbf_cache_root),
        "--warm-cache-label", str(args.warm_cache_label),
        "--external-evidence-mode", "snapshot",
        "--rbf-max-depth", "40",
        "--ffb-depth", str(int(ffb_depth)),
        "--rbf-ffb-start-depth", "15",
        "--threads", str(max(1, int(args.threads))),
        "--task-batch-size", str(max(1, int(args.threads))),
        "--max-boxes", str(int(max_boxes)),
        "--timeout-ms", str(float(args.timeout_ms)),
        "--fixed-anchor-target-preset", "iris8",
        "--random-anchor-targets", "0",
        "--anchor-target-prob", str(UNIFIED_SBF_SAMPLING_ANCHOR_TARGET_PROB),
        "--anchor-wave-targets-per-batch", "4",
        "--rrt-goal-bias", str(UNIFIED_SBF_SAMPLING_RRT_GOAL_BIAS),
        "--intertree-goal-bias", str(UNIFIED_SBF_SAMPLING_INTERTREE_GOAL_BIAS),
        "--unexplored-prob", str(UNIFIED_SBF_SAMPLING_UNEXPLORED_PROB),
        "--sample-uniform-prob", str(UNIFIED_SBF_SAMPLING_UNIFORM_PROB),
        "--frontier-face-memory",
        "--frontier-face-bins-per-dim", "4",
        "--frontier-face-min-attempts", "1",
        "--frontier-face-max-attempts", "12",
        "--frontier-face-area-attempt-scale", "24",
        "--frontwave-bootstrap-boxes", str(int(bootstrap_boxes)),
        "--frontwave-bootstrap-depth", str(int(bootstrap_depth)),
        "--frontwave-bootstrap-boundary-samples", str(int(bootstrap_samples)),
        "--component-connect-prob", str(float(component_connect_prob)),
        "--component-connect-candidate-limit", str(int(component_candidate_limit)),
        "--component-connect-ffb-depth-increment", "14",
        "--component-connect-ffb-max-depth", str(int(ffb_depth)),
        "--component-connect-stage-normalized-linf", "0.16",
        "--component-connect-neighbor-root-bias", "1.0",
        "--component-connect-neighbor-root-window", "1",
        "--no-component-connect-require-target-direction",
        "--component-connect-chain-steps", str(int(component_chain_steps)),
        "--component-connect-chain-max-boxes", str(int(component_chain_max_boxes)),
        "--enable-connector",
        "--connector-birrt",
        "--connector-rrt-iters", "10000",
        "--connector-rrt-timeout-ms", "300",
        "--connector-rrt-step-size", str(float(connector_step)),
        "--connector-rrt-goal-bias", "0.45",
        "--connector-pair-timeout-ms", str(float(connector_pair_timeout_ms)),
        "--connector-max-pairs-per-gap", str(int(connector_max_pairs_per_gap)),
        "--connector-pave-max-chain", "0",
        "--connector-pave-depth", str(int(ffb_depth)),
        "--segment-edge-policy", segment_edge_policy,
        "--collision-shortcut",
        "--collision-shortcut-resolution", "24",
        "--no-repair-on-audit-failure",
        "--require-no-repair",
        "--audit-collision-tolerance", "0.002",
        "--post-audit-segment-step", "0.01",
        "--bridge-failed-queries",
        "--no-corridor-refine",
        "--final-ompl-simplify-time-s", str(float(final_simplify_s)),
        "--clean-active-cache",
    ]
    command.append("--use-external-evidence" if use_external_evidence else "--no-use-external-evidence")
    command.append("--endpoint-evidence-cache" if endpoint_evidence_cache else "--no-endpoint-evidence-cache")
    command.append("--external-evidence-materialization" if use_external_evidence else "--no-external-evidence-materialization")
    command.append("--external-evidence-scoring" if use_external_evidence else "--no-external-evidence-scoring")
    command.append("--external-evidence-auto-build-snapshot")
    command.append(
        "--sample-categorical-allocation"
        if bool(UNIFIED_SBF_SAMPLING_CATEGORICAL_ALLOCATION)
        else "--no-sample-categorical-allocation"
    )
    return command


def experiment_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows = [
        {
            "name": "tiny_d23_iris8_segment_simplify005",
            "factor": "recommended",
            "description": "Recommended shallow 8-anchor coverage with d23 cache, BiRRT segment edges, and 0.05 s final simplification.",
            "command": base_command(args, "tiny_d23_iris8_segment_simplify005"),
        },
        {
            "name": "tiny_d23_iris8_segment_no_simplify",
            "factor": "final_simplify",
            "description": "Same as recommended but disables final OMPL simplification.",
            "command": base_command(args, "tiny_d23_iris8_segment_no_simplify", final_simplify_s=0.0),
        },
        {
            "name": "tiny_no_cache_iris8_segment_simplify005",
            "factor": "d23_cache",
            "description": "Same as recommended but disables external d23 evidence and active endpoint evidence cache.",
            "command": base_command(
                args,
                "tiny_no_cache_iris8_segment_simplify005",
                use_external_evidence=False,
                endpoint_evidence_cache=False,
            ),
        },
        {
            "name": "tiny_d23_default5_segment_simplify005",
            "factor": "coverage_anchors",
            "description": "Same as recommended but uses the legacy five query endpoints as coverage roots.",
            "command": base_command(
                args,
                "tiny_d23_default5_segment_simplify005",
                coverage_anchor_preset="default",
            ),
        },
        {
            "name": "tiny_d23_iris8_no_segment",
            "factor": "segment_connector",
            "description": "Same shallow 8-anchor coverage but disables segment edges to test whether boxes alone connect.",
            "expected_success": False,
            "command": base_command(
                args,
                "tiny_d23_iris8_no_segment",
                segment_edge_policy="off",
                final_simplify_s=0.0,
            ),
        },
        {
            "name": "balanced_d23_iris8_segment_simplify005",
            "factor": "coverage_budget",
            "description": "Larger d18/b900 coverage budget with light component-connect growth.",
            "command": base_command(
                args,
                "balanced_d23_iris8_segment_simplify005",
                max_boxes=900,
                ffb_depth=40,
                bootstrap_depth=18,
                bootstrap_boxes=600,
                bootstrap_samples=14,
                component_connect_prob=0.10,
                component_candidate_limit=4,
                component_chain_steps=2,
                component_chain_max_boxes=2,
                connector_step=0.22,
                connector_pair_timeout_ms=180.0,
                connector_max_pairs_per_gap=4,
            ),
        },
    ]
    wanted = set(csv_list(args.only))
    if wanted and "all" not in wanted:
        rows = [row for row in rows if row["name"] in wanted]
    return rows


def load_case(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    import json
    return json.loads(path.read_text(encoding="utf-8"))


def summarize_case(path: Path) -> dict[str, Any]:
    payload = load_case(path)
    if payload is None:
        return {"artifact": str(path), "exists": False}
    rows = list(payload.get("rows", []))
    path_totals: list[float] = []
    segment_used: list[float] = []
    waypoint_totals: list[float] = []
    wall_s: list[float] = []
    planning_s: list[float] = []
    boxes: list[float] = []
    external_hits: list[float] = []
    connector_ms: list[float] = []
    grow_islands: list[float] = []
    adjacency_islands: list[float] = []
    query_successes: list[int] = []
    repair_counts: list[int] = []
    for row in rows:
        build = row.get("build", {})
        queries = list(row.get("queries", []))
        diagnostics = build.get("diagnostics", {})
        path_totals.append(float(sum(float(query.get("length", 0.0)) for query in queries)))
        segment_used.append(float(sum(int(query.get("segment_edges_used", 0)) for query in queries)))
        waypoint_totals.append(float(sum(int(query.get("waypoint_count", 0)) for query in queries)))
        wall_s.append(float(build.get("wall_s", 0.0)))
        planning_s.append(float(build.get("planning_s", 0.0)))
        boxes.append(float(build.get("unique_box_count", 0.0)))
        external_hits.append(float(diagnostics.get("oracle.materialization_reused_external_evidence", 0.0)))
        connector_ms.append(float(build.get("connector_ms", 0.0)))
        grow_islands.append(float(build.get("grow_adjacency_islands", 0.0)))
        adjacency_islands.append(float(build.get("adjacency_islands", 0.0)))
        query_successes.append(sum(1 for query in queries if bool(query.get("audit_passed")) and bool(query.get("post_audit_passed"))))
        repair_counts.append(sum(int(query.get("repair_count", 0)) for query in queries))
    return {
        "artifact": str(path),
        "exists": True,
        "ok": bool(payload.get("ok")),
        "audit_ok": bool(payload.get("audit_ok")),
        "n": len(rows),
        "query_successes": query_successes,
        "repair_counts": repair_counts,
        "wall_s_median": median(wall_s),
        "wall_s_range": [min(wall_s), max(wall_s)] if wall_s else [],
        "planning_s_median": median(planning_s),
        "boxes_median": median(boxes),
        "path_total_median": median(path_totals),
        "path_total_range": [min(path_totals), max(path_totals)] if path_totals else [],
        "segment_edges_used_median": median(segment_used),
        "waypoint_total_median": median(waypoint_totals),
        "external_hits_median": median(external_hits),
        "connector_ms_median": median(connector_ms),
        "grow_islands_median": median(grow_islands),
        "adjacency_islands_median": median(adjacency_islands),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run redesigned Exp.4: d23 cache + 8-anchor shallow coverage + BiRRT segment trade-off.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "exp04_anchor_segment_tradeoff")
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--rbf-cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=D23_CACHE_LABEL)
    parser.add_argument("--only", default="all")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = experiment_rows(args)
    out_json = args.out_json or (args.out_dir / "anchor_segment_tradeoff_manifest.json")
    args.out_json = out_json
    cache_path = args.rbf_cache_root / str(args.warm_cache_label)
    snapshot_path = cache_path / "lect_snapshot"
    run_records = []
    env = default_sbf_subprocess_env()
    if args.execute:
        for row in rows:
            measurement = run_command(row["command"], dry_run=bool(args.dry_run), extra_env=env)
            artifact = case_json_path(args, row["name"])
            if bool(args.dry_run):
                status = "dry_run"
            elif measurement.get("returncode") == 0 and artifact.exists():
                status = "completed"
            elif (
                not bool(row.get("expected_success", True))
                and measurement.get("returncode") != 0
                and artifact.exists()
            ):
                status = "completed_expected_failure"
            else:
                status = "failed"
            run_records.append({
                "name": row["name"],
                "status": status,
                "artifact": str(artifact) if artifact.exists() else None,
                "measurement": measurement,
            })
    summaries = {
        row["name"]: summarize_case(case_json_path(args, row["name"]))
        for row in rows
    }
    payload = {
        "experiment": "exp04_anchor_segment_tradeoff",
        "run_id": run_id("exp04_anchor_segment"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "d23_cache": {
            "path": str(cache_path),
            "exists": cache_path.exists(),
            "snapshot_path": str(snapshot_path),
            "snapshot_exists": snapshot_path.exists(),
        },
        "rows": [
            {key: value for key, value in row.items() if key != "command"} | {"command": row["command"]}
            for row in rows
        ],
        "runs": run_records,
        "summaries": summaries,
        "notes": [
            "Redesigned Exp.4 focuses on the current recommended shelf mechanism: shallow 8-anchor coverage followed by BiRRT segment-edge connection.",
            "The main row reuses the d23 SupportHull/AAFKVolumeMin LECT snapshot and keeps the tree shallow (d15, 320 boxes).",
            "Negative/control rows isolate final simplification, d23 cache reuse, 8-anchor coverage, segment edges, and a larger coverage budget.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    successful_statuses = {"completed", "completed_expected_failure"}
    return 0 if all(record.get("status") in successful_statuses for record in run_records) or not args.execute else 1


if __name__ == "__main__":
    raise SystemExit(main())
