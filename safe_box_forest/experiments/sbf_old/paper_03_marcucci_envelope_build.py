#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import random
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from RapidBoxForest.safe_box_forest.experiments.sbf_old.common_sbf_config import (
    REPO_ROOT,
    ROOT,
    add_common_sbf_args,
    box_volume_sum,
    configure_standalone_sbf,
    count_status,
    mean,
    median,
    query_result_payload,
    sbf,
    write_json,
)
from sbf.marcucci import (
    make_bins_obstacles,
    make_combined_obstacles,
    make_combined_queries,
    make_coverage_seeds,
    make_shelves_obstacles,
    make_table_obstacles,
    iiwa14_robot_json,
    load_iiwa14_robot,
)


SCENE_BUILDERS = {
    "shelves": make_shelves_obstacles,
    "bins": make_bins_obstacles,
    "table": make_table_obstacles,
    "combined": make_combined_obstacles,
    "marcucci": make_combined_obstacles,
    "marcucci_combined": make_combined_obstacles,
}

CACHE_PROTOCOLS = {"no_cache", "cold", "warm", "cross_scene"}
SCENE_CHOICES = sorted([*SCENE_BUILDERS, "random"])
_FIRST_LINK_SPEC: tuple[list[dict[str, Any]], int, float] | None = None


def obstacle_bounds(obstacle: Any) -> list[float]:
    return [float(value) for value in obstacle.bounds]


def make_aabb(cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> Any:
    return sbf.Obstacle(cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz)


def inflate_obstacle(obstacle: Any, margin: float) -> Any:
    bounds = obstacle_bounds(obstacle)
    return sbf.Obstacle(
        bounds[0] - margin,
        bounds[1] - margin,
        bounds[2] - margin,
        bounds[3] + margin,
        bounds[4] + margin,
        bounds[5] + margin,
    )


def unique_query_endpoints(queries: list[Any]) -> list[list[float]]:
    endpoints: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for query in queries:
        for point in (list(query.start), list(query.goal)):
            key = tuple(round(float(value), 12) for value in point)
            if key not in seen:
                seen.add(key)
                endpoints.append([float(value) for value in point])
    return endpoints


def endpoints_clear(robot: Any, obstacles: list[Any], endpoints: list[list[float]], clearance: float) -> bool:
    inflated = [inflate_obstacle(obstacle, clearance) for obstacle in obstacles]
    return all(not sbf.check_config_collision(robot, inflated, endpoint) for endpoint in endpoints)


def dh_transform(alpha: float, a: float, d: float, theta: float) -> list[float]:
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)
    return [
        ct, -st, 0.0, a,
        st * ca, ct * ca, -sa, -d * sa,
        st * sa, ct * sa, ca, d * ca,
        0.0, 0.0, 0.0, 1.0,
    ]


def mat4_mul(lhs: list[float], rhs: list[float]) -> list[float]:
    out = [0.0] * 16
    for row in range(3):
        base = row * 4
        for col in range(3):
            out[base + col] = lhs[base + 0] * rhs[col] + lhs[base + 1] * rhs[4 + col] + lhs[base + 2] * rhs[8 + col]
        out[base + 3] = lhs[base + 0] * rhs[3] + lhs[base + 1] * rhs[7] + lhs[base + 2] * rhs[11] + lhs[base + 3]
    out[15] = 1.0
    return out


def segment_hits_inflated_aabb(origin: list[float], target: list[float], obstacle: Any, radius: float) -> bool:
    bounds = obstacle_bounds(obstacle)
    direction = [target[axis] - origin[axis] for axis in range(3)]
    enter = 0.0
    exit = 1.0
    for axis in range(3):
        lo = bounds[axis] - radius
        hi = bounds[axis + 3] + radius
        if abs(direction[axis]) < 1e-15:
            if origin[axis] < lo or origin[axis] > hi:
                return False
            continue
        inv = 1.0 / direction[axis]
        t0 = (lo - origin[axis]) * inv
        t1 = (hi - origin[axis]) * inv
        if t0 > t1:
            t0, t1 = t1, t0
        enter = max(enter, t0)
        exit = min(exit, t1)
        if enter > exit:
            return False
    return True


def first_link_spec() -> tuple[list[dict[str, Any]], int, float] | None:
    global _FIRST_LINK_SPEC
    if _FIRST_LINK_SPEC is not None:
        return _FIRST_LINK_SPEC
    model = json.loads(iiwa14_robot_json().read_text(encoding="utf-8"))
    dh_params = model.get("dh_params", [])
    link_radii = [float(value) for value in model.get("link_radii", [])]
    first_link = next((index for index, radius in enumerate(link_radii) if radius > 0.0), None)
    if first_link is None or first_link >= len(dh_params):
        return None
    _FIRST_LINK_SPEC = (dh_params, int(first_link), float(link_radii[first_link]))
    return _FIRST_LINK_SPEC


def first_link_clear(obstacles: list[Any], endpoints: list[list[float]], clearance: float) -> bool:
    spec = first_link_spec()
    if spec is None:
        return True
    dh_params, first_link, link_radius = spec
    radius = link_radius + max(0.0, float(clearance))
    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    for q in endpoints:
        prefix = [identity]
        for index, item in enumerate(dh_params):
            theta = float(item.get("theta", 0.0))
            d_value = float(item.get("d", 0.0))
            if item.get("type", "revolute") == "prismatic":
                d_value += float(q[index])
            else:
                theta += float(q[index])
            prefix.append(mat4_mul(prefix[-1], dh_transform(float(item.get("alpha", 0.0)), float(item.get("a", 0.0)), d_value, theta)))
        origin_tf = prefix[first_link]
        target_tf = prefix[first_link + 1]
        origin = [origin_tf[3], origin_tf[7], origin_tf[11]]
        target = [target_tf[3], target_tf[7], target_tf[11]]
        if any(segment_hits_inflated_aabb(origin, target, obstacle, radius) for obstacle in obstacles):
            return False
    return True


def random_obstacle(rng: random.Random, difficulty: str) -> Any:
    size_scale = {"easy": 0.10, "medium": 0.14, "hard": 0.18}.get(difficulty, 0.14)
    cx = rng.uniform(-0.85, 0.85)
    cy = rng.uniform(-0.85, 0.85)
    cz = rng.uniform(-0.10, 0.95)
    hx = rng.uniform(size_scale * 0.45, size_scale * 1.10)
    hy = rng.uniform(size_scale * 0.45, size_scale * 1.10)
    hz = rng.uniform(size_scale * 0.45, size_scale * 1.35)
    return make_aabb(cx, cy, cz, hx, hy, hz)


def make_random_marcucci_obstacles(args: argparse.Namespace,
                                   robot: Any,
                                   queries: list[Any],
                                   scene_seed: int) -> tuple[list[Any], dict[str, Any]]:
    base_seed = int(args.cross_random_seed_base) + 1009 * int(scene_seed)
    blocked = {int(item) for item in parse_csv(args.cross_random_blocked_seeds)}
    actual_seed = base_seed
    reroll_index = 0
    while actual_seed in blocked:
        reroll_index += 1
        actual_seed = base_seed + 1000003 * reroll_index
    rng = random.Random(actual_seed)
    endpoints = unique_query_endpoints(queries)
    obstacles: list[Any] = []
    attempts = 0
    first_link_rejections = 0
    target = int(args.cross_random_obstacles)
    while len(obstacles) < target and attempts < int(args.cross_random_max_tries):
        attempts += 1
        candidate = random_obstacle(rng, args.cross_random_difficulty)
        proposed = [*obstacles, candidate]
        if not endpoints_clear(robot, proposed, endpoints, float(args.cross_clearance)):
            continue
        if not first_link_clear(proposed, endpoints, float(args.cross_first_link_clearance)):
            first_link_rejections += 1
            continue
        if endpoints_clear(robot, proposed, endpoints, float(args.cross_clearance)):
            obstacles.append(candidate)
    if len(obstacles) < target:
        raise RuntimeError(
            f"could place only {len(obstacles)}/{target} random obstacles while preserving "
            f"endpoint clearance {args.cross_clearance}"
        )
    return obstacles, {
        "scene_kind": "random",
        "difficulty": args.cross_random_difficulty,
        "seed": actual_seed,
        "base_seed": base_seed,
        "reroll_index": reroll_index,
        "blocked_seed": base_seed if reroll_index else None,
        "requested_obstacles": target,
        "obstacle_count": len(obstacles),
        "placement_attempts": attempts,
        "endpoint_count": len(endpoints),
        "clearance": float(args.cross_clearance),
        "first_link_clearance": float(args.cross_first_link_clearance),
        "endpoint_clearance_ok": endpoints_clear(robot, obstacles, endpoints, float(args.cross_clearance)),
        "first_link_clearance_ok": first_link_clear(obstacles, endpoints, float(args.cross_first_link_clearance)),
        "first_link_rejections": first_link_rejections,
        "placement_version": "first_link_clearance_v1",
        "obstacle_bounds": [obstacle_bounds(obstacle) for obstacle in obstacles],
    }


def parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def make_scene_obstacles(name: str) -> list[Any]:
    try:
        return SCENE_BUILDERS[name]()
    except KeyError as exc:
        raise ValueError(f"unknown scene {name!r}; choices={sorted(SCENE_BUILDERS)}") from exc


def make_experiment_scene(args: argparse.Namespace,
                          name: str,
                          robot: Any,
                          queries: list[Any],
                          seed: int) -> tuple[list[Any], dict[str, Any]]:
    if name == "random":
        return make_random_marcucci_obstacles(args, robot, queries, seed)
    obstacles = make_scene_obstacles(name)
    endpoints = unique_query_endpoints(queries)
    return obstacles, {
        "scene_kind": "marcucci_combined" if name in {"combined", "marcucci", "marcucci_combined"} else name,
        "obstacle_count": len(obstacles),
        "endpoint_count": len(endpoints),
        "clearance": float(args.cross_clearance),
        "endpoint_clearance_ok": endpoints_clear(robot, obstacles, endpoints, float(args.cross_clearance)),
    }


def scene_cache_label(name: str, metadata: dict[str, Any]) -> str:
    if name == "random":
        return safe_namespace(
            "random",
            metadata.get("difficulty", "scene"),
            metadata.get("seed", "seed"),
            metadata.get("placement_version", "legacy"),
        )
    if name in {"combined", "marcucci", "marcucci_combined"}:
        return "marcucci_combined"
    return name


def apply_database_profile(cfg: Any, name: str) -> None:
    key = name.strip().lower().replace("-", "_")
    if key == "compact":
        cfg.database.page_size_bytes = 4096
        cfg.database.max_resident_pages = 2048
    elif key == "balanced":
        cfg.database.page_size_bytes = 16384
        cfg.database.max_resident_pages = 8192
    elif key in {"fast", "fast_query", "fastquery"}:
        cfg.database.page_size_bytes = 65536
        cfg.database.max_resident_pages = 32768
    else:
        raise ValueError(f"unknown database profile {name!r}")


def safe_namespace(*parts: Any) -> str:
    text = "_".join(str(part) for part in parts if str(part))
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in text)


def cache_metrics(cache_root: Path, namespace: str) -> dict[str, Any]:
    directory = cache_root / namespace
    files = sorted(path for path in directory.rglob("*") if path.is_file()) if directory.exists() else []
    return {
        "cache_file_count": len(files),
        "cache_file_bytes": sum(path.stat().st_size for path in files),
        "cache_files": [str(path) for path in files],
    }


def apply_cache_protocol(cfg: Any, args: argparse.Namespace, namespace: str | None) -> None:
    apply_database_profile(cfg, args.storage_profile)
    cfg.grower.root_seed_max_lca_depth = -1
    if namespace is None:
        return
    cfg.database.path = str(args.cache_root / namespace)
    cfg.database.create_if_missing = True
    cfg.database.verify_identity = True


def summarize_variant(trials: list[dict[str, Any]]) -> dict[str, Any]:
    def diag_value(row: dict[str, Any], key: str) -> float:
        return float(row["diagnostics"].get(key, 0.0))

    def grid_read_us_per_materialization(row: dict[str, Any]) -> float | None:
        materializations = diag_value(row, "oracle.lect.lazy_grid_materializations")
        if materializations <= 0.0:
            return None
        return diag_value(row, "oracle.lect.grid_read_time_us") / materializations

    grid_read_samples = [value for row in trials if (value := grid_read_us_per_materialization(row)) is not None]
    return {
        "seeds": len(trials),
        "cache_file_bytes_mean": mean(float(row.get("cache_file_bytes", 0.0)) for row in trials),
        "prewarm_build_mean_s": mean(
            float((row.get("prewarm") or {}).get("build_s"))
            for row in trials
            if (row.get("prewarm") or {}).get("build_s") is not None
        ),
        "build_mean_s": mean(float(row["build_s"]) for row in trials),
        "build_median_s": median(float(row["build_s"]) for row in trials),
        "box_count_mean": mean(float(row["n_boxes"]) for row in trials),
        "certified_box_count_mean": mean(float(row["certified_box_count"]) for row in trials),
        "provisional_box_count_mean": mean(float(row["provisional_box_count"]) for row in trials),
        "volume_sum_mean": mean(float(row["box_volume_sum"]) for row in trials),
        "segment_edge_count_mean": mean(float(row["segment_edge_count"]) for row in trials),
        "audited_query_sr": mean(1.0 if query["audit_passed"] else 0.0 for row in trials for query in row["queries"]),
        "raw_query_sr": mean(1.0 if query["ok"] else 0.0 for row in trials for query in row["queries"]),
        "repair_count_mean": mean(float(query["repair_count"]) for row in trials for query in row["queries"]),
        "grid_refinements_mean": mean(diag_value(row, "oracle.grid_refinements") for row in trials),
        "validation_cache_hits_mean": mean(diag_value(row, "oracle.validation_cache_hits") for row in trials),
        "validation_cache_misses_mean": mean(diag_value(row, "oracle.validation_cache_misses") for row in trials),
        "t_read_us_mean": mean(grid_read_samples),
        "grid_read_time_us_mean": mean(diag_value(row, "oracle.lect.grid_read_time_us") for row in trials),
        "grid_collision_time_us_mean": mean(diag_value(row, "oracle.lect.grid_collision_time_us") for row in trials),
        "lazy_grid_materializations_mean": mean(diag_value(row, "oracle.lect.lazy_grid_materializations") for row in trials),
        "materialization_source_incremental_state_mean": mean(diag_value(row, "oracle.materialization_source_incremental_state") for row in trials),
        "materialization_reused_endpoint_cache_mean": mean(diag_value(row, "oracle.materialization_reused_endpoint_cache") for row in trials),
        "materialization_reused_external_evidence_mean": mean(diag_value(row, "oracle.materialization_reused_external_evidence") for row in trials),
        "materialization_stored_endpoint_mean": mean(diag_value(row, "oracle.materialization_stored_endpoint") for row in trials),
        "materialization_skipped_endpoint_cache_mean": mean(diag_value(row, "oracle.materialization_skipped_endpoint_cache") for row in trials),
        "materialization_candidate_dirty_count_mean": mean(diag_value(row, "oracle.materialization_candidate_dirty_count") for row in trials),
        "scoring_source_incremental_state_mean": mean(diag_value(row, "oracle.scoring_source_incremental_state") for row in trials),
        "scoring_reused_endpoint_cache_mean": mean(diag_value(row, "oracle.scoring_reused_endpoint_cache") for row in trials),
        "scoring_reused_external_evidence_mean": mean(diag_value(row, "oracle.scoring_reused_external_evidence") for row in trials),
        "scoring_candidate_dirty_count_mean": mean(diag_value(row, "oracle.scoring_candidate_dirty_count") for row in trials),
    }


def compare_to_v6(v6_path: Path | None) -> dict[str, Any] | None:
    if v6_path is None or not v6_path.exists():
        return None
    data = json.loads(v6_path.read_text(encoding="utf-8"))
    return {
        "v6_path": str(v6_path),
        "note": "Reference only: this standalone Exp.3 runner does not import or execute v6 code.",
        "v6_build_mean_s": data.get("build", {}).get("mean_s"),
        "v6_query_count": len(data.get("queries", [])),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone SBF Exp.3 Marcucci envelope/build runner.")
    add_common_sbf_args(parser)
    parser.add_argument("--variants", default="crit_link_coverage,kdop26_coverage,support_hull_coverage")
    parser.add_argument("--protocols", default="cold,warm,cross_scene")
    parser.add_argument("--target-scene", choices=SCENE_CHOICES, default="marcucci")
    parser.add_argument("--cross-source-scene", choices=SCENE_CHOICES, default="random")
    parser.add_argument("--cross-random-difficulty", choices=["easy", "medium", "hard"], default="medium")
    parser.add_argument("--cross-random-obstacles", type=int, default=10)
    parser.add_argument("--cross-random-seed-base", type=int, default=20260505)
    parser.add_argument("--cross-random-blocked-seeds", default="20262523")
    parser.add_argument("--cross-random-max-tries", type=int, default=20000)
    parser.add_argument("--cross-clearance", type=float, default=0.12)
    parser.add_argument("--cross-first-link-clearance", type=float, default=0.02)
    parser.add_argument("--grid-pad-policy", choices=["strict_half_diagonal", "no_extra_pad"], default="strict_half_diagonal")
    parser.add_argument("--storage-profile", choices=["compact", "balanced", "fast_query"], default="fast_query")
    parser.add_argument("--cache-root", type=Path, default=ROOT / "outputs" / "paper" / "lect_database")
    parser.add_argument("--cache-run-id", default=None)
    parser.add_argument("--cached-replay-worker-local-ffb", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--cached-replay-best-tighten", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--record-prewarm-trials", action="store_true", default=False)
    parser.add_argument("--out-json", type=Path, default=ROOT / "outputs" / "paper" / "marcucci_envelope_build_standalone.json")
    parser.add_argument("--checkpoint-json", type=Path, default=None, help="Write partial results after each build trial.")
    parser.add_argument("--v6-json", type=Path, default=REPO_ROOT / "cpp" / "v6" / "experiments" / "results_paper" / "marcucci_combined.json")
    return parser.parse_args()


def progress(message: str) -> None:
    print(f"[exp3] {message}", flush=True)


def write_checkpoint(path: Path | None, payload: dict[str, Any]) -> None:
    if path is None:
        return
    write_json(path, payload)


def refine_corridors_if_requested(forest: Any, queries: list[Any], args: argparse.Namespace) -> tuple[float, int, int]:
    if not bool(getattr(args, "corridor_refine", False)):
        return 0.0, 0, 0
    deterministic = bool(getattr(args, "corridor_refine_deterministic", False))
    budget_s = max(0.0, float(getattr(args, "corridor_refine_budget_ms", 0.0))) / 1000.0
    max_total = max(0, int(getattr(args, "corridor_refine_max_boxes", 0)))
    per_query = max(1, int(getattr(args, "corridor_refine_boxes_per_query", 1)))
    if max_total <= 0 or (budget_s <= 0.0 and not deterministic):
        return 0.0, 0, 0
    t0 = time.perf_counter()
    added_total = 0
    attempted = 0
    start_margin_s = max(0.0, float(getattr(args, "corridor_refine_start_margin_ms", 0.0))) / 1000.0
    defer_labels = {item.strip() for item in str(getattr(args, "corridor_refine_defer_labels", "")).split(",") if item.strip()}
    ordered_queries = sorted(queries, key=lambda query: (getattr(query, "label", "") in defer_labels, getattr(query, "label", "")))
    for _ in range(max(1, int(getattr(args, "corridor_refine_passes", 1)))):
        pass_added = 0
        for query in ordered_queries:
            if added_total >= max_total:
                break
            if not deterministic:
                elapsed_s = time.perf_counter() - t0
                if elapsed_s >= budget_s:
                    break
                if attempted > 0 and budget_s - elapsed_s < start_margin_s:
                    break
            quota = min(per_query, max_total - added_total)
            added = int(forest.refine_query_corridor(list(query.start), list(query.goal), quota))
            attempted += 1
            added_total += added
            pass_added += added
        if pass_added == 0 or added_total >= max_total or (not deterministic and time.perf_counter() - t0 >= budget_s):
            break
    return time.perf_counter() - t0, added_total, attempted


def query_with_optional_bridge(forest: Any, query: Any, args: argparse.Namespace) -> dict[str, Any]:
    query_t0 = time.perf_counter()
    result = forest.query(list(query.start), list(query.goal))
    query_s = time.perf_counter() - query_t0
    should_bridge = (not result.success and bool(getattr(args, "bridge_failed_queries", False))) or (
        bool(getattr(args, "bridge_repaired_queries", False))
        and result.success
        and int(result.repair_count) > 0
        and int(result.start_box_id) != int(result.goal_box_id)
    )
    if should_bridge:
        bridge_t0 = time.perf_counter()
        if hasattr(forest, "bridge_query_known_needed"):
            bridge_progress = int(forest.bridge_query_known_needed(list(query.start), list(query.goal)))
        else:
            bridge_progress = int(forest.bridge_query(list(query.start), list(query.goal)))
        bridge_s = time.perf_counter() - bridge_t0
        retry_t0 = time.perf_counter()
        retry = forest.query(list(query.start), list(query.goal))
        retry_s = time.perf_counter() - retry_t0
        row = query_result_payload(query.label, retry, query_s + bridge_s + retry_s)
        row["bridge_progress"] = int(bridge_progress)
        row["bridge_time_s"] = float(bridge_s)
        row["pre_bridge_ok"] = bool(result.success)
        row["pre_bridge_audit_passed"] = bool(result.audit_passed)
        row["pre_bridge_remaining_unsafe_assumptions"] = int(result.remaining_unsafe_assumptions)
    else:
        row = query_result_payload(query.label, result, query_s)
        row["bridge_progress"] = 0
        row["bridge_time_s"] = 0.0
        row["pre_bridge_ok"] = bool(result.success)
        row["pre_bridge_audit_passed"] = bool(result.audit_passed)
        row["pre_bridge_remaining_unsafe_assumptions"] = int(result.remaining_unsafe_assumptions)
    return row


def run_build_trial(
    args: argparse.Namespace,
    *,
    robot: Any,
    variant: str,
    seed: int,
    protocol: str,
    scene_name: str,
    obstacles: list[Any],
    scene_metadata: dict[str, Any],
    seeds: list[list[float]],
    queries: list[Any],
    cache_namespace: str | None,
    prewarm: dict[str, Any] | None = None,
    locked_split_events: list[dict[str, Any]] | None = None,
    locked_validation_events: list[dict[str, Any]] | None = None,
    return_split_events: bool = False,
    return_validation_events: bool = False,
) -> dict[str, Any]:
    progress(f"start variant={variant} protocol={protocol} seed={seed} scene={scene_name} cache={cache_namespace or 'off'}")
    cfg = configure_standalone_sbf(args, seed, preset=variant, robot=robot)
    apply_cache_protocol(cfg, args, cache_namespace)
    if hasattr(args, "stateless_materialization_context"):
        cfg.validation.stateless_materialization_context = bool(args.stateless_materialization_context)
    external_cache_protocol = protocol in {"cross_scene", "warm_budget", "warm_guided_external"}
    guided_external_protocol = protocol == "warm_guided_external"
    guided_active_protocol = protocol == "warm_guided_active"
    if cache_namespace is not None and external_cache_protocol and bool(getattr(args, "cross_detach_cache_tree", False)):
        cfg.database.replay_journal = False
    if cache_namespace is not None and external_cache_protocol and bool(getattr(args, "matched_trajectory_reuse", False)):
        cfg.database.replay_journal = False
    if guided_external_protocol:
        cfg.database.replay_journal = False
        cfg.validation.external_evidence_materialization = True
        cfg.validation.external_evidence_scoring = True
        cfg.validation.external_evidence_backfill_active = True
    if guided_active_protocol:
        cfg.database.replay_journal = True
        cfg.validation.external_evidence_materialization = True
        cfg.validation.external_evidence_scoring = True
        cfg.validation.external_evidence_backfill_active = True
    if protocol in {"warm", "cross_scene", "warm_budget", "warm_guided_external", "warm_guided_active"}:
        if external_cache_protocol and hasattr(args, "cross_external_evidence_materialization"):
            cfg.validation.external_evidence_materialization = bool(args.cross_external_evidence_materialization)
        if protocol == "warm" and hasattr(args, "replay_external_evidence_materialization"):
            cfg.validation.external_evidence_materialization = bool(args.replay_external_evidence_materialization)
        if external_cache_protocol and hasattr(args, "cross_external_evidence_scoring"):
            cfg.validation.external_evidence_scoring = bool(args.cross_external_evidence_scoring)
        if external_cache_protocol and hasattr(args, "cross_external_evidence_backfill_active"):
            cfg.validation.external_evidence_backfill_active = bool(args.cross_external_evidence_backfill_active)
        if guided_external_protocol:
            cfg.database.replay_journal = False
            cfg.validation.external_evidence_materialization = True
            cfg.validation.external_evidence_scoring = True
            cfg.validation.external_evidence_backfill_active = True
        if guided_active_protocol:
            cfg.database.replay_journal = True
            cfg.validation.external_evidence_materialization = True
            cfg.validation.external_evidence_scoring = True
            cfg.validation.external_evidence_backfill_active = True
        if protocol == "warm" and hasattr(args, "replay_external_evidence_scoring"):
            cfg.validation.external_evidence_scoring = bool(args.replay_external_evidence_scoring)
        worker_local_ffb = getattr(args, "cached_replay_worker_local_ffb", None)
        if worker_local_ffb is not None:
            cfg.grower.worker_local_ffb = bool(worker_local_ffb) and int(args.threads) > 1
        replay_best_tighten = getattr(args, "cached_replay_best_tighten", None)
        if replay_best_tighten is not None:
            cfg.grower.find_free_box.split.use_best_tighten = bool(replay_best_tighten)
            cfg.connector.pave.find_free_box.split.use_best_tighten = bool(replay_best_tighten)
    prebuild_metrics = cache_metrics(args.cache_root, cache_namespace) if cache_namespace else {
        "cache_file_count": 0,
        "cache_file_bytes": 0,
        "cache_files": [],
    }
    forest = sbf.SafeBoxForest(robot, cfg)
    if locked_split_events is not None:
        forest.set_locked_split_events(locked_split_events)
    if locked_validation_events is not None:
        forest.set_locked_validation_events(locked_validation_events)
    build_t0 = time.perf_counter()
    profile = forest.build_coverage(obstacles, seeds)
    prebridge_time_s, prebridge_added_boxes, prebridge_attempts = refine_corridors_if_requested(forest, queries, args)
    build_s = time.perf_counter() - build_t0
    boxes = forest.boxes()
    query_rows: list[dict[str, Any]] = []
    for query in queries:
        query_rows.append(query_with_optional_bridge(forest, query, args))
    boxes = forest.boxes()
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    metrics = cache_metrics(args.cache_root, cache_namespace) if cache_namespace else {
        "cache_file_count": 0,
        "cache_file_bytes": 0,
        "cache_files": [],
    }
    trial = {
        "variant": variant,
        "seed": seed,
        "protocol": protocol,
        "scene": scene_name,
        "scene_metadata": scene_metadata,
        "cache_enabled": cache_namespace is not None,
        "database_replay_journal": bool(cfg.database.replay_journal),
        "database_checkpoint_after_build": bool(cfg.database.checkpoint_after_build),
        "external_evidence_materialization": bool(getattr(cfg.validation, "external_evidence_materialization", True)),
        "external_evidence_scoring": bool(getattr(cfg.validation, "external_evidence_scoring", True)),
        "external_evidence_backfill_active": bool(getattr(cfg.validation, "external_evidence_backfill_active", True)),
        "stateless_materialization_context": bool(getattr(cfg.validation, "stateless_materialization_context", False)),
        "cache_namespace": cache_namespace,
        "cache_root": str(args.cache_root) if cache_namespace else None,
        "worker_local_ffb": bool(cfg.grower.worker_local_ffb),
        "prebuild_cache_file_count": int(prebuild_metrics["cache_file_count"]),
        "prebuild_cache_file_bytes": int(prebuild_metrics["cache_file_bytes"]),
        "prebuild_cache_files": prebuild_metrics["cache_files"],
        "cache_file_count": int(metrics["cache_file_count"]),
        "cache_file_bytes": int(metrics["cache_file_bytes"]),
        "cache_files": metrics["cache_files"],
        "prewarm": prewarm,
        "build_s": float(build_s),
        "prebridge_time_s": float(prebridge_time_s),
        "prebridge_added_boxes": int(prebridge_added_boxes),
        "prebridge_attempts": int(prebridge_attempts),
        "n_boxes": int(len(boxes)),
        "certified_box_count": count_status(boxes, sbf.BoxSafetyStatus.CertifiedFree),
        "provisional_box_count": count_status(boxes, sbf.BoxSafetyStatus.ProvisionalFree),
        "strict_audit_required_box_count": sum(1 for box in boxes if bool(box.strict_audit_required)),
        "box_volume_sum": box_volume_sum(boxes),
        "segment_edge_count": len(forest.segment_edges()),
        "profile": {
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "merge_ms": float(profile.merge_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "segment_edges": int(profile.segment_edges),
            "adjacency_islands": int(profile.adjacency_islands),
        },
        "diagnostics": diagnostics,
        "queries": query_rows,
    }
    if return_split_events:
        trial["split_events"] = list(forest.split_events())
    if return_validation_events:
        trial["validation_events"] = list(forest.validation_events())
    audit_sr = mean(1.0 if query["audit_passed"] else 0.0 for query in query_rows)
    progress(
        f"done variant={variant} protocol={protocol} seed={seed} "
        f"build_s={build_s:.3f} boxes={len(boxes)} audit_sr={audit_sr}"
    )
    return trial


def run_prewarm(
    args: argparse.Namespace,
    *,
    robot: Any,
    variant: str,
    seed: int,
    protocol: str,
    scene_name: str,
    obstacles: list[Any],
    scene_metadata: dict[str, Any],
    cache_namespace: str,
    seeds: list[list[float]],
    queries: list[Any],
) -> dict[str, Any]:
    trial = run_build_trial(
        args,
        robot=robot,
        variant=variant,
        seed=seed,
        protocol=f"{protocol}_prewarm",
        scene_name=scene_name,
        obstacles=obstacles,
        scene_metadata=scene_metadata,
        seeds=seeds,
        queries=queries if args.record_prewarm_trials else [],
        cache_namespace=cache_namespace,
    )
    return {
        "protocol": trial["protocol"],
        "scene": scene_name,
        "scene_metadata": scene_metadata,
        "build_s": trial["build_s"],
        "n_boxes": trial["n_boxes"],
        "cache_file_bytes": trial["cache_file_bytes"],
    }


def main() -> int:
    args = parse_args()
    robot = load_iiwa14_robot()
    queries = make_combined_queries()
    target_obstacles, target_scene_metadata = make_experiment_scene(args, args.target_scene, robot, queries, 0)
    seeds = [list(seed) for seed in make_coverage_seeds(include_extra_anchors=False)]
    variants = parse_csv(args.variants)
    protocols = parse_csv(args.protocols)
    unknown_protocols = [item for item in protocols if item not in CACHE_PROTOCOLS]
    if unknown_protocols:
        raise ValueError(f"unknown protocols {unknown_protocols}; choices={sorted(CACHE_PROTOCOLS)}")
    if args.cache_run_id is None:
        args.cache_run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    all_trials: list[dict[str, Any]] = []
    summaries: dict[str, Any] = {}
    checkpoint_base = {
        "experiment": "exp3_marcucci_envelope_build",
        "status": "running",
        "source_script": str(Path(__file__).resolve()),
        "params": {
            "variants": variants,
            "protocols": protocols,
            "target_scene": args.target_scene,
            "cross_source_scene": args.cross_source_scene,
            "cache_root": str(args.cache_root),
            "cache_run_id": args.cache_run_id,
            "seeds": max(1, int(args.seeds)),
            "max_boxes": int(args.max_boxes),
            "timeout_ms": float(args.timeout_ms),
        },
    }
    write_checkpoint(args.checkpoint_json, {**checkpoint_base, "trials": all_trials})

    for variant in variants:
        summaries[variant] = {}
        for seed in range(max(1, int(args.seeds))):
            same_scene_namespace = safe_namespace(
                "exp3", args.cache_run_id, variant, f"seed{seed}",
                scene_cache_label(args.target_scene, target_scene_metadata), "same_scene"
            )
            same_scene_warmed = False
            for protocol in protocols:
                cache_namespace: str | None
                prewarm: dict[str, Any] | None = None
                if protocol == "no_cache":
                    cache_namespace = None
                elif protocol == "cold":
                    cache_namespace = same_scene_namespace
                    same_scene_warmed = True
                elif protocol == "warm":
                    cache_namespace = same_scene_namespace
                    if not same_scene_warmed:
                        prewarm = run_prewarm(
                            args,
                            robot=robot,
                            variant=variant,
                            seed=seed,
                            protocol=protocol,
                            scene_name=args.target_scene,
                            obstacles=target_obstacles,
                            scene_metadata=target_scene_metadata,
                            cache_namespace=cache_namespace,
                            seeds=seeds,
                            queries=queries,
                        )
                        same_scene_warmed = True
                else:
                    source_obstacles, source_scene_metadata = make_experiment_scene(
                        args, args.cross_source_scene, robot, queries, seed
                    )
                    cache_namespace = safe_namespace(
                        "exp3", args.cache_run_id, variant, f"seed{seed}",
                        scene_cache_label(args.cross_source_scene, source_scene_metadata),
                        "to", scene_cache_label(args.target_scene, target_scene_metadata), "cross_scene"
                    )
                    prewarm = run_prewarm(
                        args,
                        robot=robot,
                        variant=variant,
                        seed=seed,
                        protocol=protocol,
                        scene_name=args.cross_source_scene,
                        obstacles=source_obstacles,
                        scene_metadata=source_scene_metadata,
                        cache_namespace=cache_namespace,
                        seeds=seeds,
                        queries=queries,
                    )

                trial = run_build_trial(
                    args,
                    robot=robot,
                    variant=variant,
                    seed=seed,
                    protocol=protocol,
                    scene_name=args.target_scene,
                    obstacles=target_obstacles,
                    scene_metadata=target_scene_metadata,
                    seeds=seeds,
                    queries=queries,
                    cache_namespace=cache_namespace,
                    prewarm=prewarm,
                )
                all_trials.append(trial)
                write_checkpoint(args.checkpoint_json, {**checkpoint_base, "trials": all_trials})

        for protocol in protocols:
            summaries[variant][protocol] = summarize_variant(
                [row for row in all_trials if row["variant"] == variant and row["protocol"] == protocol]
            )

    payload = {
        "experiment": "exp3_marcucci_envelope_build",
        "source_protocol": "standalone_sbf_no_v6_runtime_dependency",
        "source_script": str(Path(__file__).resolve()),
        "params": {
            "variants": variants,
            "protocols": protocols,
            "target_scene": args.target_scene,
            "cross_source_scene": args.cross_source_scene,
            "target_scene_metadata": target_scene_metadata,
            "cross_random_difficulty": args.cross_random_difficulty,
            "cross_random_obstacles": int(args.cross_random_obstacles),
            "cross_random_seed_base": int(args.cross_random_seed_base),
            "cross_random_blocked_seeds": parse_csv(args.cross_random_blocked_seeds),
            "cross_clearance": float(args.cross_clearance),
            "cross_first_link_clearance": float(args.cross_first_link_clearance),
            "grid_pad_policy": args.grid_pad_policy,
            "cache_root": str(args.cache_root),
            "cache_run_id": args.cache_run_id,
            "storage_profile": args.storage_profile,
            "cached_replay_worker_local_ffb": bool(args.cached_replay_worker_local_ffb),
            "cached_replay_best_tighten": bool(args.cached_replay_best_tighten),
            "worker_local_ffb": bool(args.worker_local_ffb),
            "seeds": max(1, int(args.seeds)),
            "max_boxes": int(args.max_boxes),
            "timeout_ms": float(args.timeout_ms),
            "ffb_depth": int(args.ffb_depth),
            "strict_path_audit": bool(args.strict_path_audit),
            "audit_resolution": int(args.audit_resolution),
            "validation_cache": bool(args.validation_cache),
        },
        "summaries": summaries,
        "trials": all_trials,
        "comparison_to_v6": compare_to_v6(args.v6_json),
    }
    write_json(args.out_json, payload)
    write_checkpoint(args.checkpoint_json, {**payload, "status": "complete"})
    print(json.dumps({"out_json": str(args.out_json), "summaries": summaries}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())