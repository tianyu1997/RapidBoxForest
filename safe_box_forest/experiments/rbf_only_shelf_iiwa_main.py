from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path
from typing import Any

from common_sbf_config import (
    RBF_LIFELONG_PRESET,
    RBF_ONLY_OUTPUT_ROOT,
    add_common_sbf_args,
    configure_standalone_sbf,
    query_result_payload,
    rbf_lifelong_config_metadata,
    set_online_cache_backfill,
    write_json,
)

import sbf


def directory_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the RBF-only E3 Shelf+IIWA pilot/main experiment.")
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
    parser.add_argument("--warm-cache-label", default="e5_lifelong_cache_link_d18_smoke")
    parser.add_argument("--seeds-list", default="0")
    parser.add_argument("--modes", default="cold,warm_d18")
    parser.add_argument("--clean-cold-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clean-warm-active-cache", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def parse_csv_ints(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def parse_csv(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def make_config(args: argparse.Namespace, robot: Any, cache_label: str, seed: int) -> Any:
    local_args = argparse.Namespace(**vars(args))
    local_args.rbf_envelope = "link"
    local_args.rbf_cache_label = cache_label
    return configure_standalone_sbf(local_args, seed=seed, preset=RBF_LIFELONG_PRESET, robot=robot)


def run_row(args: argparse.Namespace, robot: Any, mode: str, seed: int) -> dict[str, Any]:
    cache_label = f"e3_{mode}_shelf_iiwa_seed{seed}"
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
        cfg.database.external_evidence_path = str(warm_source_path)
        cfg.validation.external_evidence_materialization = True
        cfg.validation.external_evidence_scoring = True
        cfg.validation.external_evidence_backfill_active = False
    cfg.database.create_if_missing = True

    obstacles = sbf.make_combined_obstacles()
    coverage_seeds = [list(seed_value) for seed_value in sbf.make_coverage_seeds(include_extra_anchors=False)]
    queries = sbf.make_combined_queries()
    print(f"[e3] start mode={mode} seed={seed} cache={cache_path.name}", flush=True)
    forest = sbf.SafeBoxForest(robot, cfg)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, coverage_seeds)
    build_wall_s = time.perf_counter() - build_t0
    planning_s = float(profile.total_ms) / 1000.0
    query_rows: list[dict[str, Any]] = []
    for query in queries:
        query_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        query_rows.append(query_result_payload(query.label, result, time.perf_counter() - query_t0))
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    row = {
        "ok": all(bool(query["audit_passed"]) for query in query_rows),
        "mode": mode,
        "seed": int(seed),
        "scene": "shelf_iiwa_combined",
        "metadata": rbf_lifelong_config_metadata(cfg, args),
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
            "segment_edges": int(profile.segment_edges),
            "adjacency_islands": int(profile.adjacency_islands),
            "diagnostics": diagnostics,
        },
        "queries": query_rows,
    }
    print(
        f"[e3] done mode={mode} seed={seed} ok={row['ok']} planning_s={planning_s:.3f} "
        f"maintenance_s={row['build']['maintenance_s']:.3f} "
        f"passed={sum(1 for query in query_rows if query['audit_passed'])}/{len(query_rows)} "
        f"cache_hits={diagnostics.get('oracle.materialization_reused_endpoint_cache', 0.0):.0f} "
        f"external_hits={diagnostics.get('oracle.materialization_reused_external_evidence', 0.0):.0f}",
        flush=True,
    )
    del forest
    return row


def main() -> int:
    args = parse_args()
    robot = sbf.load_iiwa14_robot()
    rows: list[dict[str, Any]] = []
    for seed in parse_csv_ints(args.seeds_list):
        for mode in parse_csv(args.modes):
            rows.append(run_row(args, robot, mode, seed))
    payload = {
        "ok": all(bool(row["ok"]) for row in rows),
        "selected_configuration": {
            "preset": RBF_LIFELONG_PRESET,
            "envelope": "link",
            "prewarm_depth": int(args.rbf_prewarm_depth),
            "warm_cache_label": args.warm_cache_label,
        },
        "args": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "rows": rows,
    }
    write_json(args.out_json, payload)
    print(f"[e3] wrote {args.out_json} ok={payload['ok']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())