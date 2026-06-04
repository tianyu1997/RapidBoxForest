#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common import run_shelf_sbf_case as shelf_case  # noqa: E402
from experiments.exp04_shelf_ablation.run_shelf_ablation import (  # noqa: E402
    DEFAULT_BASELINE_WARM_PREWARM_DEPTH,
    FIXED_SHELF_ROOT_INTERVALS,
    warm_cache_paths,
)
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    RBF_LIFELONG_PRESET,
    add_common_sbf_args,
    configure_external_evidence_reuse,
)


sbf = shelf_case.sbf
_FOREST_KEEPALIVE: list[Any] = []


def make_case_defaults() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset=RBF_LIFELONG_PRESET,
        rbf_envelope="support_hull",
        threads=8,
        task_batch_size=8,
        max_boxes=5000,
        timeout_ms=60000.0,
        ffb_depth=64,
        rbf_max_depth=64,
        connector_pave_depth=64,
        component_connect_ffb_max_depth=64,
        endpoint_source="aafk",
        lect_split_policy="aafk_volume_min",
        use_external_evidence=False,
        external_evidence_materialization=True,
        external_evidence_scoring=True,
        external_evidence_mode="snapshot",
        external_evidence_auto_build_snapshot=True,
        clean_active_cache=True,
        lect_root_intervals=FIXED_SHELF_ROOT_INTERVALS,
        segment_edge_policy="fallback_only",
        coverage_anchor_preset="default",
        latency_profile="stable",
        latency_stage_id="",
        latency_stage_quality_min_connected_boxes=[],
        latency_stage_post_connect_extra_boxes=[],
        latency_stage_post_connect_time_budget_ms=[],
        latency_stage_max_boxes=[],
    )
    return parser.parse_args([])


def node_depth(node: Any) -> int:
    node_id = int(getattr(node, "tree_id", 0))
    if node_id <= 0:
        return 0
    return node_id.bit_length() - 1


def read_key_value_manifest(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    out: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, sep, value = line.partition("=")
        if sep:
            out[key.strip()] = value.strip()
    return out


def config_summary(cfg: Any) -> dict[str, Any]:
    split_descriptor = ""
    try:
        split_descriptor = str(sbf.split_policy_descriptor(cfg.database.split_policy))
    except Exception:
        split_descriptor = ""
    return {
        "canonical_mode": bool(cfg.database.canonical_mode),
        "root_intervals_override": [
            [float(interval.lo), float(interval.hi)]
            for interval in list(getattr(cfg.database, "root_intervals_override", []) or [])
        ],
        "envelope_type": str(cfg.envelope_type.type),
        "envelope_subdivisions": int(cfg.envelope_type.n_subdivisions),
        "endpoint_source": str(cfg.endpoint_source.source),
        "split_policy": split_descriptor,
        "validation_mode": str(cfg.validation.mode),
        "external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "external_evidence_snapshot_path": str(getattr(cfg.database, "external_evidence_snapshot_path", "")),
        "external_evidence_materialization": bool(cfg.validation.external_evidence_materialization),
        "external_evidence_scoring": bool(cfg.validation.external_evidence_scoring),
        "endpoint_evidence_cache": bool(getattr(cfg.validation, "enable_endpoint_evidence_cache", False)),
        "store_endpoint_evidence_cache": bool(getattr(cfg.validation, "store_endpoint_evidence_cache", False)),
        "auto_publish_snapshot_after_checkpoint": bool(
            getattr(cfg.database, "auto_publish_snapshot_after_checkpoint", False)
        ),
    }


def depth_histogram(boxes: list[Any]) -> dict[str, int]:
    counts = Counter(node_depth(box) for box in boxes)
    return {str(depth): int(counts[depth]) for depth in sorted(counts)}


def average_us(total_us: float, count: float) -> float | None:
    return float(total_us) / float(count) if float(count) > 0.0 else None


def timing_rows(diagnostics: dict[str, float], counters: dict[str, float]) -> list[dict[str, Any]]:
    node_validations = float(counters.get("node_validations", 0.0))
    materializations = float(counters.get("materializations", 0.0))
    envelope_queries = float(counters.get("envelope_collision_queries", 0.0))
    split_count = float(diagnostics.get("leaf_sweep.splits", 0.0) + diagnostics.get("leaf_sweep.initialize_splits", 0.0))
    split_ms = float(diagnostics.get("profile.oracle.split_node.total_ms", diagnostics.get("oracle.split_node.total_ms", 0.0)))
    rows = [
        {
            "name": "oracle.validate_node",
            "count": int(node_validations),
            "total_ms": float(counters.get("validate_node_total_time_us", 0.0)) / 1000.0,
            "avg_us": average_us(float(counters.get("validate_node_total_time_us", 0.0)), node_validations),
        },
        {
            "name": "oracle.validate_node.endpoint_path",
            "count": int(node_validations),
            "total_ms": float(counters.get("validate_node_endpoint_path_time_us", 0.0)) / 1000.0,
            "avg_us": average_us(float(counters.get("validate_node_endpoint_path_time_us", 0.0)), node_validations),
        },
        {
            "name": "oracle.validate_node.classify",
            "count": int(node_validations),
            "total_ms": float(counters.get("validate_node_classify_time_us", 0.0)) / 1000.0,
            "avg_us": average_us(float(counters.get("validate_node_classify_time_us", 0.0)), node_validations),
        },
        {
            "name": "oracle.materialize.endpoint",
            "count": int(materializations),
            "total_ms": float(counters.get("materialization_endpoint_time_us", 0.0)) / 1000.0,
            "avg_us": average_us(float(counters.get("materialization_endpoint_time_us", 0.0)), materializations),
        },
        {
            "name": "oracle.materialize.envelope",
            "count": int(materializations),
            "total_ms": float(counters.get("materialization_envelope_time_us", 0.0)) / 1000.0,
            "avg_us": average_us(float(counters.get("materialization_envelope_time_us", 0.0)), materializations),
        },
        {
            "name": "oracle.envelope_collision",
            "count": int(envelope_queries),
            "total_ms": None,
            "avg_us": None,
        },
        {
            "name": "oracle.split_node",
            "count": int(split_count),
            "total_ms": split_ms if split_ms > 0.0 else None,
            "avg_us": (1000.0 * split_ms / split_count) if split_ms > 0.0 and split_count > 0.0 else None,
        },
    ]
    return rows


def run_one(args: argparse.Namespace, robot: Any, obstacles: list[Any], name: str, use_d23: bool) -> dict[str, Any]:
    print(f"[leaf-sweep-probe] start case={name} use_d23={int(use_d23)}", file=sys.stderr, flush=True)
    case_args = make_case_defaults()
    case_args.database_path = args.out_dir / "active_cache" / name
    case_args.case_name = name
    case_args.use_external_evidence = bool(use_d23)
    case_args.endpoint_evidence_cache = bool(args.endpoint_evidence_cache)
    case_args.threads = int(args.threads)
    case_args.task_batch_size = int(args.validation_batch_size)
    case_args.rbf_cache_root = args.warm_cache_root
    case_args.warm_cache_label = args.warm_cache_label
    case_args.external_evidence_mode = str(args.external_evidence_mode)
    case_args.rbf_max_tree_depth = int(args.rbf_max_tree_depth)
    case_args.rbf_online_cache_max_nodes = int(args.rbf_online_cache_max_nodes)
    case_args.rbf_online_cache_max_payload_bytes = int(args.rbf_online_cache_max_payload_bytes)
    case_args.rbf_auto_publish_snapshot = False
    case_args.rbf_auto_publish_snapshot_async = False

    cfg = shelf_case.case_config(case_args, robot, seed=0)
    cfg.validation.enable_endpoint_evidence_cache = bool(args.endpoint_evidence_cache)
    cfg.validation.store_endpoint_evidence_cache = bool(args.endpoint_evidence_cache)
    warm_path = Path(case_args.rbf_cache_root) / str(case_args.warm_cache_label)
    database_path = Path(cfg.database.path)
    if database_path.exists():
        shutil.rmtree(database_path)
    if use_d23:
        if not warm_path.exists():
            raise FileNotFoundError(f"d23 warm cache not found: {warm_path}")
        configure_external_evidence_reuse(
            cfg,
            warm_path,
            case_args,
            materialization=True,
            scoring=True,
            backfill_active=False,
        )

    sweep_cfg = sbf.LeafSweepConfig()
    sweep_cfg.obstacle_cluster_gap = float(args.obstacle_cluster_gap)
    sweep_cfg.n_threads = int(args.threads)
    sweep_cfg.validation_batch_size = int(args.validation_batch_size)
    sweep_cfg.timeout_ms = float(args.timeout_ms)
    sweep_cfg.store_group_results = bool(args.store_group_results)
    sweep_cfg.pre_split_to_max_depth = bool(args.pre_split_to_max_depth)
    sweep_cfg.use_virtual_topology = bool(args.use_virtual_topology)
    sweep_cfg.parallel_virtual_validation = bool(args.parallel_virtual_validation)

    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    result = forest.build_leaf_sweep(obstacles, int(args.start_depth), int(args.max_depth), sweep_cfg)
    wall_s = time.perf_counter() - t0
    print(
        f"[leaf-sweep-probe] build done case={name} wall_s={wall_s:.3f} "
        f"free={len(result.free_boxes)} collision={len(result.collision_boxes)}",
        file=sys.stderr,
        flush=True,
    )

    diagnostics = {str(key): float(value) for key, value in dict(result.diagnostics).items()}
    counters = {str(key): float(value) for key, value in dict(forest.oracle_counters()).items()}
    free_boxes = list(result.free_boxes)
    collision_boxes = list(result.collision_boxes)
    _FOREST_KEEPALIVE.append(forest)
    print(f"[leaf-sweep-probe] collected counters case={name}", file=sys.stderr, flush=True)

    return {
        "name": name,
        "use_d23_cache": bool(use_d23),
        "database_path": str(database_path),
        "wall_s": float(wall_s),
        "leaf_sweep_total_ms": float(result.total_ms),
        "initialize_ms": float(result.initialize_ms),
        "group_sweep_ms": float(result.group_sweep_ms),
        "compose_ms": float(result.compose_ms),
        "deadline_reached": bool(result.deadline_reached),
        "config": config_summary(cfg),
        "warm_manifest": read_key_value_manifest(warm_path / "manifest.json") if use_d23 else {},
        "free_count": int(len(free_boxes)),
        "collision_count": int(len(collision_boxes)),
        "free_depth_histogram": depth_histogram(free_boxes),
        "collision_depth_histogram": depth_histogram(collision_boxes),
        "diagnostics": diagnostics,
        "oracle_counters": counters,
        "function_timings": timing_rows(diagnostics, counters),
    }


def parse_args() -> argparse.Namespace:
    default_label, default_root, _ = warm_cache_paths(DEFAULT_BASELINE_WARM_PREWARM_DEPTH)
    parser = argparse.ArgumentParser(description="Run exp04 shelf LeafSweep d10-d18 with/without d23 reuse.")
    parser.add_argument("--start-depth", type=int, default=10)
    parser.add_argument("--max-depth", type=int, default=18)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--validation-batch-size", type=int, default=256)
    parser.add_argument("--timeout-ms", type=float, default=0.0)
    parser.add_argument("--obstacle-cluster-gap", type=float, default=0.0)
    parser.add_argument("--endpoint-evidence-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--no-store-group-results", dest="store_group_results", action="store_false", default=True)
    parser.add_argument("--pre-split-to-max-depth", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--use-virtual-topology", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rbf-max-tree-depth", type=int, default=64)
    parser.add_argument("--rbf-online-cache-max-nodes", type=int, default=200000)
    parser.add_argument("--rbf-online-cache-max-payload-bytes", type=int, default=512 * 1024 * 1024)
    parser.add_argument("--warm-cache-root", type=Path, default=default_root)
    parser.add_argument("--warm-cache-label", default=default_label)
    parser.add_argument("--external-evidence-mode", choices=["snapshot", "legacy"], default="snapshot")
    parser.add_argument("--cases", default="cold,warm", help="Comma-separated subset of: cold,warm.")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=REPO_ROOT / "outputs" / "new_experiments" / "exp04_leaf_sweep_d10_d18",
    )
    parser.add_argument("--out-json", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if int(args.start_depth) > int(args.max_depth):
        raise ValueError("start_depth must be <= max_depth")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_json = Path(args.out_json) if args.out_json is not None else args.out_dir / "leaf_sweep_d23_compare.json"

    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    requested_cases = [item.strip() for item in str(args.cases).split(",") if item.strip()]
    rows = []
    for case in requested_cases:
        if case == "cold":
            rows.append(run_one(args, robot, obstacles, "leaf_sweep_cold_no_d23", False))
        elif case == "warm":
            rows.append(run_one(args, robot, obstacles, "leaf_sweep_warm_d23", True))
        else:
            raise ValueError(f"unknown case {case!r}; expected cold,warm")
    payload = {
        "experiment": "exp04_leaf_sweep_d23_probe",
        "scene": "iiwa14 + make_combined_obstacles",
        "start_depth": int(args.start_depth),
        "max_depth": int(args.max_depth),
        "root_intervals": FIXED_SHELF_ROOT_INTERVALS,
        "warm_cache_root": str(args.warm_cache_root),
        "warm_cache_label": str(args.warm_cache_label),
        "rows": rows,
    }
    out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "out_json": str(out_json),
        "rows": [
            {
                "name": row["name"],
                "wall_s": row["wall_s"],
                "free_count": row["free_count"],
                "collision_count": row["collision_count"],
                "node_validations": row["oracle_counters"].get("node_validations", 0),
                "materializations": row["oracle_counters"].get("materializations", 0),
            }
            for row in rows
        ],
    }, indent=2, sort_keys=True))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
