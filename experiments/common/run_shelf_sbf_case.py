#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata, namespace_dict, proc_status, run_id, write_json  # noqa: E402
from safe_box_forest.experiments.sbf_old import common_sbf_config as sbf_config  # noqa: E402
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    add_common_sbf_args,
    configure_external_evidence_reuse,
    configure_standalone_sbf,
    make_aafk_volume_min_split_policy,
    mean,
    median,
    set_online_cache_backfill,
    set_rbf_envelope,
)

sbf = sbf_config.sbf


ENDPOINT_AAFK = "aafk"
ENDPOINT_CRITSAMPLE = "critsample"
LECT_SPLIT_AAFK_VOLUME_MIN = "aafk_volume_min"
LECT_SPLIT_ROUND_ROBIN = "round_robin"
DEFAULT_AAFK_SCHEDULE_DEPTH = 50


def parse_csv_ints(text: str) -> list[int]:
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one Shelf+IIWA SBF case with explicit LECT DB wiring.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset="support_hull_coverage",
        rbf_envelope="support_hull",
        threads=8,
        task_batch_size=8,
        max_boxes=5000,
        timeout_ms=60000.0,
        ffb_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        rbf_max_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        connector_pave_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        component_connect_ffb_max_depth=DEFAULT_AAFK_SCHEDULE_DEPTH,
        quality_min_connected_boxes=64,
        post_connect_extra_boxes=0,
        post_connect_time_budget_ms=450.0,
        repair_timeout_ms=750.0,
    )
    parser.add_argument("--case-name", default="shelf_sbf_case")
    parser.add_argument("--out-json", type=Path, required=True)
    parser.add_argument("--database-path", type=Path, required=True)
    parser.add_argument("--seeds-list", default="0")
    parser.add_argument("--endpoint-source", choices=[ENDPOINT_AAFK, ENDPOINT_CRITSAMPLE], default=ENDPOINT_AAFK)
    parser.add_argument("--lect-split-policy", choices=[LECT_SPLIT_AAFK_VOLUME_MIN, LECT_SPLIT_ROUND_ROBIN], default=LECT_SPLIT_AAFK_VOLUME_MIN)
    parser.add_argument("--use-external-evidence", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-materialization", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--external-evidence-scoring", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--warm-cache-label", default="iiwa_shelf_endpoint_only_p18")
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
    parser.add_argument("--require-no-repair", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def directory_size(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def configure_endpoint(cfg: Any, endpoint_source: str) -> None:
    if endpoint_source == ENDPOINT_AAFK:
        cfg.endpoint_source.source = sbf.EndpointSource.IFK
    elif endpoint_source == ENDPOINT_CRITSAMPLE:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
    else:
        raise ValueError(f"unsupported endpoint source {endpoint_source!r}")


def configure_lect_split(cfg: Any, robot: Any, policy: str, max_depth: int) -> None:
    if policy == LECT_SPLIT_AAFK_VOLUME_MIN:
        cfg.database.split_policy = make_aafk_volume_min_split_policy(robot, int(max_depth))
    elif policy == LECT_SPLIT_ROUND_ROBIN:
        descriptor = sbf.SplitPolicyDescriptor()
        descriptor.strategy = sbf.SplitStrategy.RoundRobin
        descriptor.min_width = 0.0
        descriptor.midpoint = True
        descriptor.deterministic_tie_break = True
        cfg.database.split_policy = descriptor
    else:
        raise ValueError(f"unsupported LECT split policy {policy!r}")


def effective_lect_schedule_depth(args: argparse.Namespace) -> int:
    return max(
        int(args.rbf_max_depth),
        int(args.ffb_depth),
        int(args.connector_pave_depth),
    )


def case_config(args: argparse.Namespace, robot: Any, seed: int) -> Any:
    local_args = argparse.Namespace(**vars(args))
    local_args.rbf_cache_root = Path(args.database_path).parent
    local_args.rbf_cache_label = Path(args.database_path).name
    local_args.task_batch_size = max(1, int(args.task_batch_size))
    cfg = configure_standalone_sbf(local_args, seed=seed, preset=str(args.preset), robot=robot)
    set_rbf_envelope(cfg, str(args.rbf_envelope), local_args)
    configure_endpoint(cfg, str(args.endpoint_source))
    configure_lect_split(cfg, robot, str(args.lect_split_policy), effective_lect_schedule_depth(args))
    cfg.database.path = str(args.database_path)
    cfg.database.create_if_missing = True
    cfg.database.read_only = False
    cfg.database.verify_identity = True
    cfg.database.replay_journal = True
    cfg.database.checkpoint_after_build = True
    cfg.database.max_tree_depth = int(args.rbf_max_tree_depth)
    cfg.database.online_cache.max_nodes = int(args.rbf_online_cache_max_nodes)
    cfg.database.online_cache.max_payload_bytes = int(args.rbf_online_cache_max_payload_bytes)
    set_online_cache_backfill(cfg, allow_database_backfill=False)
    return cfg


def box_count_with_status(boxes: Iterable[Any], status: Any) -> int:
    return sum(1 for box in boxes if box.safety_status == status)


def to_float_list(values: Iterable[Any]) -> list[float]:
    return [float(value) for value in values]


def query_payload(query: Any, result: Any, wall_s: float) -> dict[str, Any]:
    waypoints = [to_float_list(waypoint) for waypoint in result.path]
    return {
        "name": query.label,
        "from": query.start_name,
        "to": query.goal_name,
        "ok": bool(result.success),
        "audit_passed": bool(result.audit_passed),
        "audit_status": str(result.audit_status).split(".")[-1],
        "t_s": float(wall_s),
        "planning_time_ms": float(result.query_time_ms),
        "audit_time_ms": float(result.audit_time_ms),
        "repair_time_ms": float(result.repair_time_ms),
        "repair_count": int(result.repair_count),
        "remaining_unsafe_assumptions": int(result.remaining_unsafe_assumptions),
        "failed_segment_index": int(result.failed_segment_index),
        "start_box_id": int(result.start_box_id),
        "goal_box_id": int(result.goal_box_id),
        "length": float(result.path_length) if result.success else 0.0,
        "certified_box_length": float(result.certified_box_length),
        "provisional_audited_length": float(result.provisional_audited_length),
        "segment_edge_length": float(result.segment_edge_length),
        "segment_edges_used": int(result.segment_edges_used),
        "box_sequence": [int(value) for value in result.box_sequence],
        "segment_edge_sequence": [int(value) for value in result.segment_edge_sequence],
        "waypoints": waypoints,
        "waypoint_count": len(waypoints),
    }


def refine_corridors(forest: Any, queries: list[Any], args: argparse.Namespace) -> tuple[float, int, int]:
    if not bool(args.corridor_refine):
        return 0.0, 0, 0
    budget_s = max(0.0, float(args.corridor_refine_budget_ms)) / 1000.0
    max_total = max(0, int(args.corridor_refine_max_boxes))
    per_query = max(1, int(args.corridor_refine_boxes_per_query))
    if budget_s <= 0.0 or max_total <= 0:
        return 0.0, 0, 0
    t0 = time.perf_counter()
    added_total = 0
    attempts = 0
    start_margin_s = max(0.0, float(args.corridor_refine_start_margin_ms)) / 1000.0
    defer_labels = {item.strip() for item in str(args.corridor_refine_defer_labels).split(",") if item.strip()}
    ordered_queries = sorted(queries, key=lambda item: (item.label in defer_labels, item.label))
    for _ in range(max(1, int(args.corridor_refine_passes))):
        pass_added = 0
        for query in ordered_queries:
            elapsed_s = time.perf_counter() - t0
            if added_total >= max_total or elapsed_s >= budget_s:
                break
            if attempts > 0 and budget_s - elapsed_s < start_margin_s:
                break
            quota = min(per_query, max_total - added_total)
            added = int(forest.refine_query_corridor(list(query.start), list(query.goal), quota))
            attempts += 1
            added_total += added
            pass_added += added
        if pass_added == 0 or added_total >= max_total or time.perf_counter() - t0 >= budget_s:
            break
    return time.perf_counter() - t0, added_total, attempts


def run_query(forest: Any, query: Any, args: argparse.Namespace) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    query_s = time.perf_counter() - query_t0
    row = query_payload(query, result, query_s)
    row["pre_bridge_ok"] = bool(result.success)
    row["bridge_progress"] = 0
    row["bridge_time_s"] = 0.0
    should_bridge = (not result.success and args.bridge_failed_queries) or (
        bool(args.bridge_repaired_queries)
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        if hasattr(forest, "bridge_query_known_needed"):
            bridge_progress = forest.bridge_query_known_needed(list(query.start), list(query.goal))
        else:
            bridge_progress = forest.bridge_query(list(query.start), list(query.goal))
        bridge_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        retry = forest.query(list(query.start), list(query.goal))
        retry_s = time.perf_counter() - retry_t0
        row = query_payload(query, retry, query_s + bridge_s + retry_s)
        row["pre_bridge_ok"] = bool(result.success)
        row["bridge_progress"] = int(bridge_progress)
        row["bridge_time_s"] = float(bridge_s)
    return row


def metadata_payload(cfg: Any, args: argparse.Namespace) -> dict[str, Any]:
    split_policy = cfg.database.split_policy
    return {
        "case_name": str(args.case_name),
        "preset": str(args.preset),
        "endpoint_source": str(args.endpoint_source),
        "endpoint_source_raw": str(cfg.endpoint_source.source).split(".")[-1],
        "envelope": str(args.rbf_envelope),
        "envelope_type_raw": str(cfg.envelope_type.type).split(".")[-1],
        "lect_split_policy": str(args.lect_split_policy),
        "split_policy_descriptor": sbf.split_policy_descriptor(split_policy),
        "split_policy_hash": int(sbf.split_policy_hash(split_policy)),
        "depth_dimensions": [int(value) for value in list(getattr(split_policy, "depth_dimensions", []))],
        "dimension_schedule_hash": str(getattr(split_policy, "dimension_schedule_hash", "")),
        "database_path": str(cfg.database.path),
        "external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "external_evidence_snapshot_path": str(getattr(cfg.database, "external_evidence_snapshot_path", "")),
        "external_evidence_use_snapshot": bool(getattr(cfg.database, "external_evidence_use_snapshot", False)),
        "external_evidence_auto_build_snapshot": bool(getattr(cfg.database, "external_evidence_auto_build_snapshot", True)),
        "external_evidence_materialization": bool(getattr(cfg.validation, "external_evidence_materialization", True)),
        "external_evidence_scoring": bool(getattr(cfg.validation, "external_evidence_scoring", True)),
        "external_evidence_backfill_active": bool(getattr(cfg.validation, "external_evidence_backfill_active", False)),
        "canonical_mode": bool(getattr(cfg.database, "canonical_mode", False)),
        "checkpoint_after_build": bool(getattr(cfg.database, "checkpoint_after_build", False)),
        "online_cache_allow_database_backfill": bool(getattr(cfg.database.online_cache, "allow_database_backfill", True)),
        "max_depth": int(args.rbf_max_depth),
        "lect_schedule_depth": effective_lect_schedule_depth(args),
        "ffb_depth": int(args.ffb_depth),
        "threads": int(args.threads),
        "task_batch_size": int(args.task_batch_size),
    }


def run_seed(args: argparse.Namespace, robot: Any, obstacles: list[Any], coverage_seeds: list[list[float]], queries: list[Any], seed: int) -> dict[str, Any]:
    cfg = case_config(args, robot, seed)
    database_path = Path(cfg.database.path)
    if bool(args.clean_active_cache) and database_path.exists():
        shutil.rmtree(database_path)
    if bool(args.use_external_evidence):
        warm_path = Path(args.rbf_cache_root) / str(args.warm_cache_label)
        if not warm_path.exists():
            raise FileNotFoundError(f"warm LECT cache does not exist: {warm_path}")
        configure_external_evidence_reuse(
            cfg,
            warm_path,
            args,
            materialization=bool(args.external_evidence_materialization),
            scoring=bool(args.external_evidence_scoring),
            backfill_active=False,
        )
    cfg.database.create_if_missing = True

    print(
        f"[shelf-sbf-case] start case={args.case_name} seed={seed} endpoint={args.endpoint_source} "
        f"split={args.lect_split_policy} external={bool(args.use_external_evidence)}",
        flush=True,
    )
    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, coverage_seeds)
    refine_s, refine_added, refine_attempts = refine_corridors(forest, queries, args)
    build_wall_s = time.perf_counter() - build_t0
    boxes = list(forest.boxes())
    query_rows = [run_query(forest, query, args) for query in queries]
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    audit_ok = all(bool(query["audit_passed"]) for query in query_rows)
    no_repair = all(int(query.get("repair_count", 0)) == 0 for query in query_rows)
    unique_box_count = int(len(boxes))
    has_boxes = unique_box_count > 0
    repair_only_zero_box = (not has_boxes) and any(int(query.get("repair_count", 0)) > 0 for query in query_rows)
    effective_ok = bool(audit_ok and has_boxes and (no_repair or not bool(args.require_no_repair)))
    row = {
        "ok": effective_ok,
        "audit_ok": bool(audit_ok),
        "no_repair": bool(no_repair),
        "has_boxes": bool(has_boxes),
        "repair_only_zero_box": bool(repair_only_zero_box),
        "seed": int(seed),
        "scene": "shelf_iiwa_marcucci_combined",
        "metadata": metadata_payload(cfg, args),
        "cache_path": str(database_path),
        "cache_bytes": directory_size(database_path),
        "build": {
            "wall_s": float(build_wall_s),
            "planning_s": float(profile.total_ms) / 1000.0 + float(refine_s),
            "maintenance_s": max(0.0, float(build_wall_s) - (float(profile.total_ms) / 1000.0 + float(refine_s))),
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "merge_ms": float(profile.merge_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "unique_box_count": unique_box_count,
            "certified_box_count": box_count_with_status(boxes, sbf.BoxSafetyStatus.CertifiedFree),
            "provisional_box_count": box_count_with_status(boxes, sbf.BoxSafetyStatus.ProvisionalFree),
            "strict_audit_required_box_count": sum(1 for box in boxes if bool(box.strict_audit_required)),
            "segment_edges": int(profile.segment_edges),
            "adjacency_islands": int(profile.adjacency_islands),
            "prebridge_time_s": float(refine_s),
            "prebridge_added_boxes": int(refine_added),
            "prebridge_attempts": int(refine_attempts),
            "diagnostics": diagnostics,
        },
        "queries": query_rows,
    }
    print(
        f"[shelf-sbf-case] done case={args.case_name} seed={seed} ok={row['ok']} "
        f"boxes={row['build']['unique_box_count']} passed={sum(1 for query in query_rows if query['audit_passed'])}/{len(query_rows)} "
        f"repairs={sum(int(query.get('repair_count', 0)) for query in query_rows)} "
        f"external_hits={diagnostics.get('oracle.materialization_reused_external_evidence', 0.0):.0f}",
        flush=True,
    )
    del forest
    return row


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    query_names = [query["name"] for row in rows for query in row.get("queries", [])]
    query_names = list(dict.fromkeys(query_names))
    query_summary = []
    for name in query_names:
        items = [query for row in rows for query in row.get("queries", []) if query.get("name") == name]
        successes = [query for query in items if bool(query.get("ok"))]
        query_summary.append({
            "name": name,
            "sr": mean(1.0 if query.get("ok") else 0.0 for query in items),
            "audit_sr": mean(1.0 if query.get("audit_passed") else 0.0 for query in items),
            "query_time_median_s": median(float(query.get("t_s", 0.0)) for query in successes),
            "path_length_median": median(float(query.get("length", 0.0)) for query in successes),
            "repair_count_median": median(float(query.get("repair_count", 0.0)) for query in items),
        })
    return {
        "ok": all(bool(row.get("ok")) for row in rows),
        "audit_ok": all(bool(row.get("audit_ok")) for row in rows),
        "n": len(rows),
        "build_wall_median_s": median(float(row["build"]["wall_s"]) for row in rows),
        "planning_median_s": median(float(row["build"]["planning_s"]) for row in rows),
        "unique_box_count_median": median(float(row["build"]["unique_box_count"]) for row in rows),
        "external_hits_mean": mean(float(row["build"]["diagnostics"].get("oracle.materialization_reused_external_evidence", 0.0)) for row in rows),
        "stored_endpoint_mean": mean(float(row["build"]["diagnostics"].get("oracle.materialization_stored_endpoint", 0.0)) for row in rows),
        "queries": query_summary,
    }


def main() -> int:
    args = parse_args()
    robot = sbf.load_iiwa14_robot()
    obstacles = sbf.make_combined_obstacles()
    coverage_seeds = [list(seed) for seed in sbf.make_coverage_seeds(include_extra_anchors=False)]
    queries = sbf.make_combined_queries()
    before = proc_status()
    rows = [run_seed(args, robot, obstacles, coverage_seeds, queries, seed) for seed in parse_csv_ints(args.seeds_list)]
    payload = {
        "experiment": "shelf_sbf_case",
        "run_id": run_id("shelf_sbf_case"),
        "case_name": str(args.case_name),
        "ok": all(bool(row.get("ok")) for row in rows),
        "audit_ok": all(bool(row.get("audit_ok")) for row in rows),
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "proc_status_before": before,
        "proc_status_after": proc_status(),
        "summary": summarize(rows),
        "rows": rows,
        "notes": [
            "This backend is intentionally independent of safe_box_forest/experiments/rbf_only_shelf_iiwa_main.py.",
            "It consumes the Exp.3-style p18 LECT DB cache through external evidence when --use-external-evidence is enabled.",
            "AAFK endpoint source is represented by EndpointSource.IFK, matching the existing IFK_AA-backed helper metadata.",
            "A zero-box build is never considered ok, even if query repair later returns audit-passed paths.",
        ],
    }
    write_json(args.out_json, payload)
    print(f"wrote {args.out_json} ok={payload['ok']}")
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
