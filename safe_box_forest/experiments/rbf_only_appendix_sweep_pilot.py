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
    configure_standalone_sbf,
    median,
    query_result_payload,
    rbf_lifelong_config_metadata,
    set_online_cache_backfill,
    write_json,
)

import sbf


def parse_csv(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def directory_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def far_obstacle() -> Any:
    return sbf.Obstacle(100.0, 100.0, 100.0, 101.0, 101.0, 101.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the RBF-only E1 appendix configuration sweep pilot.")
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset=RBF_LIFELONG_PRESET,
        max_boxes=512,
        timeout_ms=10000.0,
        threads=1,
        task_batch_size=1,
        worker_local_ffb=False,
        enable_connector=True,
        connector_bridge_boxes=0,
        post_connect_extra_boxes=0,
    )
    parser.add_argument("--envelopes", default="link,kdop26,support_hull")
    parser.add_argument("--box-budgets", default="256,512")
    parser.add_argument("--query-labels", default="AS->TS")
    parser.add_argument("--scene", choices=["combined", "shelves", "far"], default="combined")
    parser.add_argument("--seeds-list", default="0")
    parser.add_argument("--cache-label-prefix", default="e1_appendix_pilot")
    parser.add_argument("--skip-prewarm", action="store_true", default=False)
    parser.add_argument("--reuse-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--warm-cache-label", default="e5_lifelong_cache_link_d18_smoke")
    parser.add_argument("--clean-warm-active-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-external-evidence", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--out-json", type=Path, default=RBF_ONLY_OUTPUT_ROOT / "e1_appendix_config_sweep.json")
    return parser.parse_args()


def make_obstacles(scene: str) -> list[Any]:
    if scene == "combined":
        return sbf.make_combined_obstacles()
    if scene == "shelves":
        return sbf.make_shelves_obstacles()
    if scene == "far":
        return [far_obstacle()]
    raise ValueError(f"unknown scene {scene!r}")


def select_queries(labels: list[str]) -> list[Any]:
    queries = sbf.make_combined_queries()
    by_label = {query.label: query for query in queries}
    unknown = [label for label in labels if label not in by_label]
    if unknown:
        raise ValueError(f"unknown query labels: {unknown}")
    return [by_label[label] for label in labels]


def make_config(args: argparse.Namespace, robot: Any, envelope: str, budget: int, seed: int) -> Any:
    local_args = argparse.Namespace(**vars(args))
    local_args.rbf_envelope = envelope
    local_args.max_boxes = int(budget)
    local_args.rbf_cache_label = f"{args.cache_label_prefix}_{envelope}_d{int(args.rbf_prewarm_depth)}_b{int(budget)}_seed{int(seed)}"
    return configure_standalone_sbf(local_args, seed=seed, preset=RBF_LIFELONG_PRESET, robot=robot)


def run_case(args: argparse.Namespace,
             robot: Any,
             envelope: str,
             budget: int,
             seed: int,
             obstacles: list[Any],
             coverage_seeds: list[list[float]],
             queries: list[Any]) -> dict[str, Any]:
    print(f"[e1-pilot] start envelope={envelope} budget={budget} seed={seed}", flush=True)
    cfg = make_config(args, robot, envelope, budget, seed)
    set_online_cache_backfill(cfg, allow_database_backfill=(args.scene == "far"))
    cache_path = Path(cfg.database.path)
    if (not args.reuse_cache or args.clean_warm_active_cache) and cache_path.exists():
        shutil.rmtree(cache_path)

    prewarm: dict[str, Any] | None = None
    if args.use_external_evidence:
        warm_source_path = Path(args.rbf_cache_root) / str(args.warm_cache_label)
        if not warm_source_path.exists():
            raise FileNotFoundError(f"warm d18 cache does not exist: {warm_source_path}")
        cfg.database.external_evidence_path = str(warm_source_path)
        cfg.validation.external_evidence_materialization = True
        cfg.validation.external_evidence_scoring = True
        cfg.validation.external_evidence_backfill_active = False
        prewarm = {
            "ok": True,
            "mode": "external_evidence_d18",
            "target_depth": int(args.rbf_prewarm_depth),
            "cache_path": str(warm_source_path),
            "create_if_missing": True,
        }

    cfg.database.create_if_missing = True
    forest = sbf.SafeBoxForest(robot, cfg)

    if not args.use_external_evidence and not args.skip_prewarm:
        prewarm = dict(forest.prewarm_lifelong_cache(int(args.rbf_prewarm_depth), [far_obstacle()]))

    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, coverage_seeds)
    build_wall_s = time.perf_counter() - build_t0
    query_rows: list[dict[str, Any]] = []
    for query in queries:
        query_t0 = time.perf_counter()
        result = forest.query(list(query.start), list(query.goal))
        query_rows.append(query_result_payload(query.label, result, time.perf_counter() - query_t0))

    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    row = {
        "ok": all(bool(item["audit_passed"]) for item in query_rows),
        "preset": RBF_LIFELONG_PRESET,
        "envelope": envelope,
        "budget": int(budget),
        "seed": int(seed),
        "scene": args.scene,
        "query_labels": [query.label for query in queries],
        "metadata": rbf_lifelong_config_metadata(cfg, args),
        "prewarm": prewarm,
        "cache_path": str(cache_path),
        "cache_bytes": directory_size_bytes(cache_path),
        "build": {
            "wall_s": float(build_wall_s),
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "merge_ms": float(profile.merge_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "segment_edges": int(profile.segment_edges),
            "adjacency_islands": int(profile.adjacency_islands),
            "diagnostics": diagnostics,
        },
        "queries": query_rows,
    }
    print(
        f"[e1-pilot] done envelope={envelope} budget={budget} seed={seed} "
        f"ok={row['ok']} build_s={build_wall_s:.3f} "
        f"cache_hits={diagnostics.get('oracle.materialization_reused_endpoint_cache', 0.0):.0f} "
        f"external_hits={diagnostics.get('oracle.materialization_reused_external_evidence', 0.0):.0f}",
        flush=True,
    )
    return row


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault((str(row["envelope"]), int(row["budget"])), []).append(row)
    summaries: list[dict[str, Any]] = []
    for (envelope, budget), items in sorted(groups.items()):
        query_success = [all(bool(query["audit_passed"]) for query in item["queries"]) for item in items]
        query_times = [sum(float(query["t_s"]) for query in item["queries"]) for item in items]
        build_times = [float(item["build"]["wall_s"]) for item in items]
        reuse_hits = [
            float(item["build"]["diagnostics"].get("oracle.materialization_reused_endpoint_cache", 0.0)) +
            float(item["build"]["diagnostics"].get("oracle.materialization_reused_external_evidence", 0.0))
            for item in items
        ]
        summaries.append({
            "envelope": envelope,
            "budget": budget,
            "n": len(items),
            "strict_audit_success_rate": sum(1 for ok in query_success if ok) / max(1, len(query_success)),
            "median_build_s": median(build_times),
            "median_query_s": median(query_times),
            "median_cache_hits": median(reuse_hits),
            "median_cache_bytes": median(float(item["cache_bytes"]) for item in items),
        })
    return summaries


def jsonable_args(args: argparse.Namespace) -> dict[str, Any]:
    return {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}


def main() -> int:
    args = parse_args()
    robot = sbf.load_iiwa14_robot()
    obstacles = make_obstacles(args.scene)
    coverage_seeds = [list(seed) for seed in sbf.make_coverage_seeds(include_extra_anchors=False)]
    queries = select_queries(parse_csv(args.query_labels))
    envelopes = parse_csv(args.envelopes)
    budgets = [int(item) for item in parse_csv(args.box_budgets)]
    seeds = [int(item) for item in parse_csv(args.seeds_list)]

    rows: list[dict[str, Any]] = []
    for envelope in envelopes:
        for budget in budgets:
            for seed in seeds:
                rows.append(run_case(args, robot, envelope, budget, seed, obstacles, coverage_seeds, queries))

    payload = {
        "ok": all(bool(row["ok"]) for row in rows),
        "note": "RBF-only appendix sweep pilot. By default this now reuses the E5 d18 cache through a fresh active DB plus read-only external evidence.",
        "args": jsonable_args(args),
        "rows": rows,
        "summary": summarize(rows),
    }
    write_json(args.out_json, payload)
    print(f"[e1-pilot] wrote {args.out_json} ok={payload['ok']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())