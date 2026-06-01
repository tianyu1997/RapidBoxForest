#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.anytime_defaults import (  # noqa: E402
    UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP,
    UNIFIED_SBF_ANYTIME_FFB_START_DEPTH,
    UNIFIED_SBF_ANYTIME_MAX_BOXES,
    UNIFIED_SBF_ANYTIME_POST_AUDIT_SEGMENT_STEP,
    UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES,
    UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS,
    UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES,
    UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
    UNIFIED_SBF_ANYTIME_REFERENCE_ARTIFACT,
    UNIFIED_SBF_ANYTIME_STAGE_IDS,
    UNIFIED_SBF_ANYTIME_TASK_BATCH_SIZE,
    UNIFIED_SBF_ANYTIME_THREADS,
    csv_floats,
    csv_ints,
    csv_text,
)
from experiments.common.experiment_io import environment_metadata, namespace_dict, run_id, write_json  # noqa: E402
from experiments.common.marcucci_anchor_guard import validate_marcucci_query_artifact  # noqa: E402
from experiments.common import run_shelf_sbf_case as shelf  # noqa: E402
from safe_box_forest.experiments.sbf_old.common_anytime_tradeoff import (  # noqa: E402
    aggregate_stage_records,
    incumbent_stage_record,
    task_result,
    update_incumbents,
)


def parse_csv_text(raw: str) -> list[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Current shelf SBF anytime trade-off runner using experiments/common/run_shelf_sbf_case.py.")
    shelf.add_common_sbf_args(parser)
    parser.set_defaults(
        preset="support_hull_coverage",
        rbf_envelope="support_hull",
        threads=UNIFIED_SBF_ANYTIME_THREADS,
        task_batch_size=UNIFIED_SBF_ANYTIME_TASK_BATCH_SIZE,
        max_boxes=5000,
        timeout_ms=60000.0,
        ffb_depth=UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
        rbf_max_depth=UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
        connector_pave_depth=UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
        component_connect_ffb_max_depth=UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
        rrt_goal_bias=0.20,
        intertree_goal_bias=0.25,
        component_connect_prob=0.35,
        quality_min_connected_boxes=64,
        post_connect_extra_boxes=0,
        post_connect_time_budget_ms=450.0,
        repair_timeout_ms=750.0,
        latency_profile=shelf.LATENCY_PROFILE_BALANCED_LOW_LATENCY,
        latency_stage_selection_policy=shelf.LATENCY_STAGE_SELECTION_ACCEPT_REPAIR,
        latency_stage_early_stop=False,
        bridge_failed_queries=True,
        bridge_repaired_queries=True,
        corridor_refine=True,
        corridor_refine_budget_ms=250.0,
        corridor_refine_max_boxes=48,
        corridor_refine_boxes_per_query=12,
        corridor_refine_passes=2,
        corridor_refine_start_margin_ms=120.0,
        corridor_refine_defer_labels="CS->LB",
        endpoint_source=shelf.ENDPOINT_AAFK,
        lect_split_policy=shelf.LECT_SPLIT_AAFK_VOLUME_MIN,
        use_external_evidence=True,
        external_evidence_materialization=True,
        external_evidence_scoring=True,
        external_evidence_mode="snapshot",
        external_evidence_auto_build_snapshot=True,
        clean_active_cache=True,
        collision_shortcut=True,
        audit_segment_step=UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP,
        post_audit_segment_step=UNIFIED_SBF_ANYTIME_POST_AUDIT_SEGMENT_STEP,
        rbf_ffb_start_depth=UNIFIED_SBF_ANYTIME_FFB_START_DEPTH,
    )
    parser.add_argument("--case-name", default="shelf_sbf_anytime")
    parser.add_argument("--out-json", type=Path, required=True)
    parser.add_argument("--database-path", type=Path, required=True)
    parser.add_argument("--endpoint-source", choices=list(shelf.SUPPORTED_ENDPOINT_SOURCES), default=shelf.ENDPOINT_AAFK)
    parser.add_argument(
        "--lect-split-policy",
        choices=[
            shelf.LECT_SPLIT_AAFK_VOLUME_MIN,
            shelf.LECT_SPLIT_AAFK_VOLUME_MIN_DIM6,
            shelf.LECT_SPLIT_SUPPORT_HULL_VOLUME_MIN,
            shelf.LECT_SPLIT_ROUND_ROBIN,
        ],
        default=shelf.LECT_SPLIT_AAFK_VOLUME_MIN,
    )
    parser.add_argument("--use-external-evidence", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-materialization", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-scoring", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--warm-cache-label", default=shelf.DEFAULT_P18_CACHE_LABEL)
    parser.add_argument(
        "--lect-root-intervals",
        default="",
        help="Optional restricted LECT root intervals in 'lo:hi;lo:hi;...' format.",
    )
    parser.add_argument("--clean-active-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine-budget-ms", type=float, default=250.0)
    parser.add_argument("--corridor-refine-max-boxes", type=int, default=48)
    parser.add_argument("--corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--corridor-refine-passes", type=int, default=2)
    parser.add_argument("--corridor-refine-start-margin-ms", type=float, default=120.0)
    parser.add_argument("--corridor-refine-defer-labels", default="CS->LB")
    parser.add_argument("--post-audit-segment-step", type=float, default=UNIFIED_SBF_ANYTIME_POST_AUDIT_SEGMENT_STEP)
    parser.add_argument("--final-ompl-simplify-time-s", type=float, default=0.0)
    parser.add_argument("--require-no-repair", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--latency-profile",
        choices=[shelf.LATENCY_PROFILE_STABLE, shelf.LATENCY_PROFILE_BALANCED_LOW_LATENCY],
        default=shelf.LATENCY_PROFILE_BALANCED_LOW_LATENCY,
    )
    parser.add_argument(
        "--latency-stage-selection-policy",
        choices=[
            shelf.LATENCY_STAGE_SELECTION_AUTO,
            shelf.LATENCY_STAGE_SELECTION_ZERO_REPAIR,
            shelf.LATENCY_STAGE_SELECTION_ACCEPT_REPAIR,
        ],
        default=shelf.LATENCY_STAGE_SELECTION_ACCEPT_REPAIR,
    )
    parser.add_argument("--latency-stage-early-stop", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--stage-ids", default=csv_text(UNIFIED_SBF_ANYTIME_STAGE_IDS))
    parser.add_argument(
        "--latency-stage-quality-min-connected-boxes",
        type=shelf.parse_csv_ints,
        default=list(UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES),
    )
    parser.add_argument(
        "--latency-stage-post-connect-extra-boxes",
        type=shelf.parse_csv_ints,
        default=list(UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES),
    )
    parser.add_argument(
        "--latency-stage-post-connect-time-budget-ms",
        type=shelf.parse_csv_floats,
        default=list(UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS),
    )
    parser.add_argument(
        "--latency-stage-max-boxes",
        type=shelf.parse_csv_ints,
        default=list(UNIFIED_SBF_ANYTIME_MAX_BOXES),
    )
    parser.add_argument("--epsilon-path", type=float, default=1e-6)
    return parser.parse_args()


def stage_sequence(args: argparse.Namespace) -> list[dict[str, Any]]:
    stage_ids = parse_csv_text(args.stage_ids)
    max_boxes = [int(value) for value in list(args.latency_stage_max_boxes)]
    quality = [int(value) for value in list(args.latency_stage_quality_min_connected_boxes)]
    extra = [int(value) for value in list(args.latency_stage_post_connect_extra_boxes)]
    budgets = [float(value) for value in list(args.latency_stage_post_connect_time_budget_ms)]
    lengths = {len(stage_ids), len(max_boxes), len(quality), len(extra), len(budgets)}
    if len(lengths) != 1:
        raise ValueError(f"anytime stage overrides must have the same length, got {sorted(lengths)}")
    stages: list[dict[str, Any]] = []
    for index, stage_id in enumerate(stage_ids):
        stages.append({
            "stage_id": str(stage_id),
            "stage_index": index,
            "quality_min_connected_boxes": int(quality[index]),
            "post_connect_extra_boxes": int(extra[index]),
            "post_connect_time_budget_ms": float(budgets[index]),
            "max_boxes": int(max_boxes[index]),
        })
    return stages


def query_tasks(row: dict[str, Any], args: argparse.Namespace) -> list[dict[str, Any]]:
    tasks: list[dict[str, Any]] = []
    for query in row.get("queries", []):
        audit_ok = bool(query.get("audit_passed")) and bool(query.get("post_audit_passed", False))
        repair_count = int(query.get("repair_count", 0))
        segment_edges_used = int(query.get("segment_edges_used", 0))
        box_sequence_len = len(query.get("box_sequence", []) or [])
        native_ok = repair_count == 0 and segment_edges_used > 0 and box_sequence_len > 0
        task_ok = bool(query.get("ok")) and audit_ok
        if bool(getattr(args, "require_no_repair", False)):
            task_ok = task_ok and native_ok
        tasks.append(task_result(
            name=str(query.get("name")),
            ok=task_ok,
            audit_passed=audit_ok,
            path_length=float(query.get("length", 0.0)) if task_ok else None,
            query_s=float(query.get("t_s", 0.0)),
            reason=str(query.get("audit_status", "")),
            extra={
                "raw": dict(query),
                "repair_count": repair_count,
                "used_repair_fallback": repair_count > 0,
                "native_query_ok": native_ok,
                "segment_edges_used": segment_edges_used,
                "box_sequence_len": box_sequence_len,
            },
        ))
    return tasks


def main() -> int:
    args = parse_args()
    stages = stage_sequence(args)
    robot = shelf.sbf.load_iiwa14_robot()
    obstacles = shelf.sbf.make_combined_obstacles()
    coverage_seeds = [list(seed) for seed in shelf.sbf.make_coverage_seeds()]
    queries = list(shelf.sbf.make_combined_queries())

    records: list[dict[str, Any]] = []
    raw_stage_rows: list[dict[str, Any]] = []
    for seed_index in range(max(1, int(args.seeds))):
        seed = int(args.seed_base) + seed_index
        incumbents: dict[str, dict[str, Any]] = {}
        cumulative_build_s = 0.0
        cumulative_query_s = 0.0
        for stage in stages:
            local_args = shelf.stage_args(args, stage)
            row = shelf.run_single_seed_attempt(local_args, robot, obstacles, coverage_seeds, queries, seed)
            tasks = query_tasks(row, args)
            stage_build_s = float(row["build"].get("planning_s", 0.0))
            stage_query_s = sum(float(task.get("query_s", 0.0)) for task in tasks)
            cumulative_build_s += stage_build_s
            cumulative_query_s += stage_query_s
            incumbents, improved = update_incumbents(incumbents, tasks, epsilon_path=float(args.epsilon_path))
            records.append(incumbent_stage_record(
                method="sbf_current_anytime",
                stage_id=str(stage["stage_id"]),
                stage_index=int(stage["stage_index"]),
                seed_index=seed_index,
                task_count=len(tasks),
                cumulative_build_s=cumulative_build_s,
                cumulative_query_s=cumulative_query_s,
                stage_build_s=stage_build_s,
                stage_query_s=stage_query_s,
                raw_tasks=tasks,
                incumbents=incumbents,
                improved_tasks=improved,
                params={
                    **dict(stage),
                    "seed": int(seed),
                    "case_name": str(local_args.case_name),
                    "box_count": int(row["build"].get("unique_box_count", 0)),
                    "path_total": float(sum(float(query.get("length", 0.0)) for query in row.get("queries", []))),
                    "repair_total": int(sum(int(query.get("repair_count", 0)) for query in row.get("queries", []))),
                    "stage_ok": bool(row.get("ok")),
                },
                protocol="current_warm_staged_anytime_incumbent",
            ))
            raw_stage_rows.append({
                "seed_index": seed_index,
                "seed": int(seed),
                "stage_id": str(stage["stage_id"]),
                "stage_index": int(stage["stage_index"]),
                "row": row,
            })

    summary = aggregate_stage_records(records, epsilon_path=float(args.epsilon_path))
    payload = {
        "experiment": "shelf_sbf_anytime_current",
        "source_script": str(Path(__file__).resolve()),
        "source_protocol": "current_run_shelf_sbf_case_staged_anytime_incumbent",
        "reference_artifact": UNIFIED_SBF_ANYTIME_REFERENCE_ARTIFACT,
        "scene": "shelf_iiwa_marcucci_combined",
        "task_names": [query.label for query in queries],
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "stage_schedule": stages,
        "summary": summary,
        "records": records,
        "raw_stage_rows": raw_stage_rows,
        "notes": [
            "This runner reuses experiments/common/run_shelf_sbf_case.py so the SBF path matches the current warm native-joint-symmetry shelf backend instead of the legacy paper_14 runner.",
            "The default stage schedule is the refined d40 anytime operating configuration extracted from d40_r4_32_128_128_168_128.json.",
            "Trade-off charging uses build.planning_s plus summed per-query wall time from each stage row.",
        ],
    }
    payload["anchor_validation"] = validate_marcucci_query_artifact(payload, artifact_path=args.out_json)
    write_json(args.out_json, payload)
    print({
        "out_json": str(args.out_json),
        "records": len(records),
        "points": len(summary.get("points", [])),
        "promoted_points": len(summary.get("promoted_points", [])),
        "run_id": run_id("shelf_anytime_current"),
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())