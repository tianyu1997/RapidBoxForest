from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path
from typing import Any

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import (
    RBF_LIFELONG_PRESET,
    RBF_ONLY_OUTPUT_ROOT,
    add_common_sbf_args,
    configure_external_evidence_reuse,
    configure_standalone_sbf,
    query_result_payload,
    set_if_available,
    set_online_cache_backfill,
    write_json,
)

import sbf


E3_PILOT_PROFILE = "e3_pilot"
EXP04_BASELINE_PROFILE = "exp04_baseline"
EXP04_SPLIT_STRATEGY_ROUND_ROBIN = "round_robin"
EXP04_SPLIT_STRATEGY_AAFK_VOLUME_MIN = "aafk_volume_min"


def directory_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the current Shelf+IIWA RBF-only experiment driver.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset=RBF_LIFELONG_PRESET,
        rbf_envelope="link",
        rbf_prewarm_depth=18,
        max_boxes=256,
        timeout_ms=10000.0,
        threads=1,
        task_batch_size=1,
        worker_local_ffb=False,
        enable_connector=True,
        connector_bridge_boxes=0,
        post_connect_extra_boxes=0,
    )
    parser.add_argument("--out-json", type=Path, default=RBF_ONLY_OUTPUT_ROOT / "e3_shelf_iiwa_main.json")
    parser.add_argument("--warm-cache-label", default="e5_lifelong_cache_link_d18_canonical_dim0q4")
    parser.add_argument("--seeds-list", default="0")
    parser.add_argument("--modes", default="cold,warm_d18")
    parser.add_argument("--run-profile", choices=[E3_PILOT_PROFILE, EXP04_BASELINE_PROFILE], default=E3_PILOT_PROFILE)
    parser.add_argument(
        "--exp04-split-strategy",
        choices=[EXP04_SPLIT_STRATEGY_ROUND_ROBIN, EXP04_SPLIT_STRATEGY_AAFK_VOLUME_MIN],
        default=EXP04_SPLIT_STRATEGY_ROUND_ROBIN,
    )
    parser.add_argument("--clean-cold-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clean-warm-active-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--corridor-refine-budget-ms", type=float, default=250.0)
    parser.add_argument("--corridor-refine-max-boxes", type=int, default=48)
    parser.add_argument("--corridor-refine-boxes-per-query", type=int, default=12)
    parser.add_argument("--corridor-refine-passes", type=int, default=2)
    parser.add_argument("--corridor-refine-start-margin-ms", type=float, default=120.0)
    parser.add_argument("--corridor-refine-defer-labels", type=str, default="CS->LB")
    parser.add_argument("--bridge-failed-queries", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--bridge-repaired-queries", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def parse_csv_ints(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def parse_csv(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def log_prefix(args: argparse.Namespace) -> str:
    return "[exp04-baseline]" if args.run_profile == EXP04_BASELINE_PROFILE else "[e3]"


def exp04_uses_lifelong_aafk(args: argparse.Namespace) -> bool:
    return (
        args.run_profile == EXP04_BASELINE_PROFILE
        and str(getattr(args, "exp04_split_strategy", "")) == EXP04_SPLIT_STRATEGY_AAFK_VOLUME_MIN
    )


def apply_profile_defaults(args: argparse.Namespace) -> None:
    if args.run_profile != EXP04_BASELINE_PROFILE:
        return
    if exp04_uses_lifelong_aafk(args):
        args.preset = RBF_LIFELONG_PRESET
    elif str(args.preset) == RBF_LIFELONG_PRESET:
        args.preset = "support_hull_coverage"
    if int(args.max_boxes) == 256:
        args.max_boxes = 5000
    if float(args.timeout_ms) == 10000.0:
        args.timeout_ms = 60000.0
    if int(args.task_batch_size) == 1 and int(args.threads) > 1:
        args.task_batch_size = int(args.threads)
    if not bool(args.worker_local_ffb) and int(args.threads) > 1:
        args.worker_local_ffb = True
    if str(args.rbf_envelope) == "link":
        args.rbf_envelope = "support_hull"


def cache_label_for_run(args: argparse.Namespace, mode: str, seed: int) -> str:
    prefix = "exp04" if args.run_profile == EXP04_BASELINE_PROFILE else "e3"
    return f"{prefix}_{mode}_shelf_iiwa_seed{seed}"


def apply_database_runtime_defaults(cfg: Any, args: argparse.Namespace, cache_label: str) -> None:
    cfg.database.canonical_mode = bool(args.rbf_canonical_cache)
    if bool(cfg.database.canonical_mode):
        set_if_available(cfg.database, "symmetry_descriptor", "joint_symmetry_native_v1")
    cfg.database.create_if_missing = True
    cfg.database.read_only = False
    cfg.database.verify_identity = True
    cfg.database.replay_journal = True
    cfg.database.max_tree_depth = int(args.rbf_max_tree_depth)
    set_if_available(cfg.database, "propagate_parent_hulls", True)
    set_if_available(cfg.database, "defer_parent_hull_writes", True)
    cfg.database.checkpoint_after_build = True
    cfg.database.online_cache.max_nodes = int(args.rbf_online_cache_max_nodes)
    cfg.database.online_cache.max_payload_bytes = int(args.rbf_online_cache_max_payload_bytes)
    cfg.database.path = str(Path(args.rbf_cache_root) / cache_label)
    set_if_available(
        cfg.database,
        "auto_publish_snapshot_after_checkpoint",
        bool(getattr(args, "rbf_auto_publish_snapshot", True)),
    )
    set_if_available(
        cfg.database,
        "auto_publish_snapshot_async",
        bool(getattr(args, "rbf_auto_publish_snapshot_async", True)),
    )
    if getattr(args, "rbf_snapshot_path", None):
        set_if_available(cfg.database, "auto_publish_snapshot_path", str(Path(args.rbf_snapshot_path)))


def build_row_metadata(cfg: Any, args: argparse.Namespace) -> dict[str, Any]:
    split_policy = cfg.database.split_policy
    depth_dimensions = list(getattr(split_policy, "depth_dimensions", []))
    return {
        "run_profile": str(args.run_profile),
        "exp04_split_strategy": str(getattr(args, "exp04_split_strategy", "")),
        "configured_preset": str(getattr(args, "preset", "")),
        "endpoint_source": str(cfg.endpoint_source.source).split(".")[-1],
        "envelope": str(getattr(args, "rbf_envelope", "")),
        "envelope_type_raw": str(cfg.envelope_type.type).split(".")[-1],
        "database_path": str(getattr(cfg.database, "path", "")),
        "database_snapshot_path": str(
            getattr(cfg.database, "auto_publish_snapshot_path", "") or (Path(str(cfg.database.path)) / "lect_snapshot")
        ) if str(getattr(cfg.database, "path", "")) else "",
        "auto_publish_snapshot_after_checkpoint": bool(
            getattr(cfg.database, "auto_publish_snapshot_after_checkpoint", False)
        ),
        "auto_publish_snapshot_async": bool(getattr(cfg.database, "auto_publish_snapshot_async", True)),
        "external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "external_evidence_snapshot_path": str(getattr(cfg.database, "external_evidence_snapshot_path", "")),
        "external_evidence_use_snapshot": bool(getattr(cfg.database, "external_evidence_use_snapshot", False)),
        "external_evidence_auto_build_snapshot": bool(
            getattr(cfg.database, "external_evidence_auto_build_snapshot", True)
        ),
        "external_evidence_materialization": bool(getattr(cfg.validation, "external_evidence_materialization", True)),
        "external_evidence_backfill_active": bool(getattr(cfg.validation, "external_evidence_backfill_active", False)),
        "canonical_mode": bool(getattr(cfg.database, "canonical_mode", False)),
        "checkpoint_after_build": bool(getattr(cfg.database, "checkpoint_after_build", False)),
        "propagate_parent_hulls": bool(getattr(cfg.database, "propagate_parent_hulls", True)),
        "defer_parent_hull_writes": bool(getattr(cfg.database, "defer_parent_hull_writes", False)),
        "online_cache_allow_database_backfill": bool(getattr(cfg.database.online_cache, "allow_database_backfill", True)),
        "split_policy_descriptor": sbf.split_policy_descriptor(split_policy),
        "split_policy_hash": int(sbf.split_policy_hash(split_policy)),
        "depth_dimensions": depth_dimensions,
        "schedule_depth": len(depth_dimensions),
        "dimension_schedule_hash": str(getattr(split_policy, "dimension_schedule_hash", "")),
        "max_depth": int(getattr(args, "rbf_max_depth", 40)),
        "ffb_start_depth": int(getattr(args, "rbf_ffb_start_depth", 15)),
        "prewarm_depth": int(getattr(args, "rbf_prewarm_depth", 18)),
    }


def refine_corridors(forest: sbf.SafeBoxForest, queries: list[Any], args: argparse.Namespace) -> tuple[float, int, int]:
    if args.run_profile != EXP04_BASELINE_PROFILE or not args.corridor_refine:
        return 0.0, 0, 0
    budget_s = max(0.0, float(args.corridor_refine_budget_ms)) / 1000.0
    max_total = max(0, int(args.corridor_refine_max_boxes))
    per_query = max(1, int(args.corridor_refine_boxes_per_query))
    if budget_s <= 0.0 or max_total <= 0:
        return 0.0, 0, 0
    t0 = time.perf_counter()
    added_total = 0
    attempted = 0
    start_margin_s = max(0.0, float(args.corridor_refine_start_margin_ms)) / 1000.0
    defer_labels = {item.strip() for item in str(args.corridor_refine_defer_labels).split(",") if item.strip()}
    ordered_queries = sorted(queries, key=lambda query: (query.label in defer_labels, query.label))
    for _ in range(max(1, int(args.corridor_refine_passes))):
        pass_added = 0
        for query in ordered_queries:
            elapsed_s = time.perf_counter() - t0
            if added_total >= max_total or elapsed_s >= budget_s:
                break
            if attempted > 0 and budget_s - elapsed_s < start_margin_s:
                break
            quota = min(per_query, max_total - added_total)
            added = int(forest.refine_query_corridor(list(query.start), list(query.goal), quota))
            attempted += 1
            added_total += added
            pass_added += added
        if pass_added == 0 or added_total >= max_total or time.perf_counter() - t0 >= budget_s:
            break
    return time.perf_counter() - t0, added_total, attempted


def make_config(args: argparse.Namespace, robot: Any, cache_label: str, seed: int) -> Any:
    local_args = argparse.Namespace(**vars(args))
    local_args.rbf_cache_label = cache_label
    chosen_preset = str(local_args.preset)
    if args.run_profile == EXP04_BASELINE_PROFILE and not exp04_uses_lifelong_aafk(args) and chosen_preset == RBF_LIFELONG_PRESET:
        chosen_preset = "support_hull_coverage"
    cfg = configure_standalone_sbf(local_args, seed=seed, preset=chosen_preset, robot=robot)
    if args.run_profile == EXP04_BASELINE_PROFILE:
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        apply_database_runtime_defaults(cfg, args, cache_label)
    return cfg


def run_row(args: argparse.Namespace, robot: Any, mode: str, seed: int) -> dict[str, Any]:
    cache_label = cache_label_for_run(args, mode, seed)
    cfg = make_config(args, robot, cache_label, seed)
    set_online_cache_backfill(cfg, allow_database_backfill=False)
    cache_path = Path(cfg.database.path)
    if mode == "cold" and args.clean_cold_cache and cache_path.exists():
        shutil.rmtree(cache_path)
    if mode == "warm_d18":
        warm_source_path = Path(args.rbf_cache_root) / str(args.warm_cache_label)
        if not warm_source_path.exists():
            raise FileNotFoundError(f"warm d18 cache does not exist: {warm_source_path}")
        if args.clean_warm_active_cache and cache_path.exists():
            shutil.rmtree(cache_path)
        configure_external_evidence_reuse(cfg, warm_source_path, args, backfill_active=False)
    cfg.database.create_if_missing = True

    obstacles = sbf.make_combined_obstacles()
    coverage_seeds = [list(seed_value) for seed_value in sbf.make_coverage_seeds(include_extra_anchors=False)]
    queries = sbf.make_combined_queries()
    print(f"{log_prefix(args)} start mode={mode} seed={seed} cache={cache_path.name}", flush=True)
    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, coverage_seeds)
    prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors(forest, queries, args)
    build_wall_s = time.perf_counter() - build_t0
    planning_s = float(profile.total_ms) / 1000.0 + float(prebridge_time_s)
    boxes = list(forest.boxes())
    query_rows: list[dict[str, Any]] = []
    for query in queries:
        query_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        query_wall_s = time.perf_counter() - query_t0
        row = query_result_payload(query.label, result, query_wall_s)
        row["pre_bridge_ok"] = bool(result.success)
        row["bridge_progress"] = 0
        row["bridge_time_s"] = 0.0
        should_bridge = args.run_profile == EXP04_BASELINE_PROFILE and (
            (not result.success and args.bridge_failed_queries)
            or (bool(args.bridge_repaired_queries) and bool(result.success) and int(result.repair_count) > 0)
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
            row = query_result_payload(query.label, retry, query_wall_s + bridge_s + retry_s)
            row["pre_bridge_ok"] = bool(result.success)
            row["bridge_progress"] = int(bridge_progress)
            row["bridge_time_s"] = float(bridge_s)
        query_rows.append(row)
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    row = {
        "ok": all(bool(query["audit_passed"]) for query in query_rows),
        "run_profile": str(args.run_profile),
        "mode": mode,
        "seed": int(seed),
        "scene": "shelf_iiwa_combined",
        "metadata": build_row_metadata(cfg, args),
        "cache_path": str(cache_path),
        "external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "cache_bytes": directory_size_bytes(cache_path),
        "build": {
            "wall_s": float(build_wall_s),
            "planning_s": planning_s,
            "maintenance_s": max(0.0, float(build_wall_s) - planning_s),
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "connector_ms": float(profile.connector_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "unique_box_count": int(len(boxes)),
            "segment_edges": int(profile.segment_edges),
            "adjacency_islands": int(profile.adjacency_islands),
            "prebridge_time_s": float(prebridge_time_s),
            "prebridge_added_boxes": int(prebridge_added_boxes),
            "prebridge_attempts": int(prebridge_attempts),
            "diagnostics": diagnostics,
        },
        "queries": query_rows,
    }
    print(
        f"{log_prefix(args)} done mode={mode} seed={seed} ok={row['ok']} planning_s={planning_s:.3f} "
        f"maintenance_s={row['build']['maintenance_s']:.3f} "
        f"boxes={row['build']['unique_box_count']} "
        f"passed={sum(1 for query in query_rows if query['audit_passed'])}/{len(query_rows)} "
        f"cache_hits={diagnostics.get('oracle.materialization_reused_endpoint_cache', 0.0):.0f} "
        f"external_hits={diagnostics.get('oracle.materialization_reused_external_evidence', 0.0):.0f}",
        flush=True,
    )
    del forest
    return row


def main() -> int:
    args = parse_args()
    apply_profile_defaults(args)
    robot = sbf.load_iiwa14_robot()
    rows: list[dict[str, Any]] = []
    for seed in parse_csv_ints(args.seeds_list):
        for mode in parse_csv(args.modes):
            rows.append(run_row(args, robot, mode, seed))
    payload = {
        "ok": all(bool(row["ok"]) for row in rows),
        "selected_configuration": {
            "run_profile": str(args.run_profile),
            "exp04_split_strategy": str(getattr(args, "exp04_split_strategy", "")),
            "preset": str(args.preset),
            "envelope": str(args.rbf_envelope),
            "prewarm_depth": int(args.rbf_prewarm_depth),
            "warm_cache_label": args.warm_cache_label,
        },
        "args": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "rows": rows,
    }
    write_json(args.out_json, payload)
    print(f"{log_prefix(args)} wrote {args.out_json} ok={payload['ok']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())