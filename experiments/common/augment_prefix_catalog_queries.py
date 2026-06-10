#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata
from experiments.common.generate_prefix_mapped_workspace_catalog import (
    DIFFICULTY_ORDER,
    augment_prefix_matching_queries,
    find_prefixes_by_mode,
    path_blocking_obstacle_bounds,
    parse_window_map,
)
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import CATALOG_SCHEMA, make_robot, obstacle_from_bounds
from experiments.common.rbf_defaults import robot_joint_limit_tuples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Augment a saved prefix catalog with matching query records.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--queries-per-scene", type=int, default=10)
    parser.add_argument("--query-min-l2", type=float, default=0.8)
    parser.add_argument("--query-max-l2", type=float, default=math.inf)
    parser.add_argument("--query-local-radius", type=float, default=0.06)
    parser.add_argument("--query-clearance-margin-m", type=float, default=0.0)
    parser.add_argument("--post-query-max-tries", type=int, default=4096)
    parser.add_argument("--post-query-planner-seeds", type=int, default=1)
    parser.add_argument(
        "--post-query-check-mode",
        choices=("all", "hard", "group", "none"),
        default="all",
        help="Whether each augmented query is gated individually, by hard only, by the growing query-set median, or only by final prefix search.",
    )
    parser.add_argument("--post-query-local-prob", type=float, default=0.85)
    parser.add_argument("--post-query-rrt-median-windows", default="")
    parser.add_argument("--post-query-bitstar-median-windows", default="")
    parser.add_argument("--planner-seeds", type=int, default=1)
    parser.add_argument("--rrt-median-windows", default="easy:0.0-0.050,medium:0.050-0.200,hard:0.050-1.000")
    parser.add_argument("--bitstar-median-windows", default="easy:0.0-0.050,medium:0.050-0.100,hard:0.100-0.750")
    parser.add_argument("--rrt-probe-timeout-s", type=float, default=0.5)
    parser.add_argument("--bitstar-probe-timeout-s", type=float, default=0.75)
    parser.add_argument("--bitstar-probe-checkpoint-interval-s", type=float, default=0.005)
    parser.add_argument("--bitstar-probe-mode", choices=("path", "trace"), default="trace")
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=5.0)
    parser.add_argument("--strict-audit-step", type=float, default=0.01)
    parser.add_argument("--min-probe-success-fraction", type=float, default=0.5)
    parser.add_argument("--direct-obstruction-samples", type=int, default=96)
    parser.add_argument("--prefix-fine-until", type=int, default=40)
    parser.add_argument("--prefix-mid-step", type=int, default=1)
    parser.add_argument("--prefix-coarse-step", type=int, default=50)
    parser.add_argument("--prefix-selection-mode", choices=("distribution", "window"), default="distribution")
    parser.add_argument("--prefix-confirm-mode", choices=("single_stage", "two_stage"), default="single_stage")
    parser.add_argument("--prefix-stage-a-planner-seeds", type=int, default=1)
    parser.add_argument("--prefix-stage-b-planner-seeds", type=int, default=3)
    parser.add_argument("--prefix-stage-b-neighbor-radius", type=int, default=1)
    parser.add_argument("--distribution-medium-ratio", type=float, default=5.0)
    parser.add_argument("--distribution-hard-ratio", type=float, default=2.0)
    parser.add_argument("--distribution-hard-not-faster-factor", type=float, default=1.0)
    parser.add_argument("--distribution-min-medium-count", type=int, default=0)
    parser.add_argument("--distribution-min-hard-count", type=int, default=0)
    parser.add_argument("--distribution-require-strong-planner", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--seed", type=int, default=20260609)
    parser.add_argument("--extend-path-blocking", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extend-max-rounds", type=int, default=2)
    parser.add_argument("--extend-candidate-pool", type=int, default=1)
    parser.add_argument("--extend-max-prefix", type=int, default=8)
    parser.add_argument("--extend-samples-per-path", type=int, default=3)
    parser.add_argument("--extend-box-half-width", type=float, default=0.07)
    parser.add_argument("--extend-query-count", type=int, default=10)
    parser.add_argument("--endpoint-source", default="critsample")
    parser.add_argument("--n-samples-crit", type=int, default=220)
    parser.add_argument("--endpoint-threads", type=int, default=1)
    parser.add_argument("--envelope-subdivisions", type=int, default=1)
    parser.add_argument("--workspace-aabb-shrink", type=float, default=0.5)
    parser.add_argument("--workspace-margin", type=float, default=0.0)
    parser.add_argument("--min-active-link-idx", type=int, default=0)
    parser.add_argument("--max-active-link-idx", type=int, default=1000000)
    parser.add_argument("--allowed-link-idxs", default="auto")
    parser.add_argument("--summary-json", type=Path, default=None)
    return parser.parse_args()


def query_max_l2_limit(value: float) -> float:
    raw = float(value)
    if raw <= 0.0 or not math.isfinite(raw):
        return math.inf
    return raw


def csv_ints(raw: str) -> list[int]:
    text = str(raw).strip()
    if not text:
        return []
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def resolved_allowed_link_idxs(raw: str, robot_name: str) -> list[int]:
    key = str(raw).strip().lower()
    if key and key != "auto":
        return csv_ints(raw)
    robot_key = str(robot_name).lower()
    if robot_key == "iiwa":
        return [4, 6, 7]
    if robot_key == "ur5":
        return [2, 3, 4]
    if robot_key == "panda":
        return [4]
    return [4, 6, 7]


def record_ordered_obstacles(record: dict[str, Any]) -> list[list[float]]:
    workspace_mapping = dict(record.get("workspace_mapping", {}))
    ordered = workspace_mapping.get("ordered_obstacles", [])
    if ordered:
        return [[float(value) for value in item] for item in ordered]
    return [[float(value) for value in item] for item in record.get("obstacles", [])]


def main() -> int:
    args = parse_args()
    t0 = time.perf_counter()
    payload = json.loads(args.input.read_text())
    records = [dict(row) for row in payload.get("records", [])]
    if not records:
        raise RuntimeError(f"input catalog has no records: {args.input}")
    grouped: dict[tuple[str, int], dict[str, dict[str, Any]]] = {}
    for row in records:
        key = (str(row["robot"]), int(row["scene_seed"]))
        grouped.setdefault(key, {})[str(row["difficulty"])] = row
    out_records: list[dict[str, Any]] = []
    rrt_windows = parse_window_map(str(args.rrt_median_windows))
    bitstar_windows = parse_window_map(str(args.bitstar_median_windows))
    summaries: list[dict[str, Any]] = []
    for (robot_name, scene_seed), by_difficulty in grouped.items():
        missing = [name for name in DIFFICULTY_ORDER if name not in by_difficulty]
        if missing:
            raise RuntimeError(f"{robot_name}/{scene_seed} missing difficulties: {missing}")
        robot = make_robot(robot_name)
        hard_record = by_difficulty["hard"]
        bounds_ordered = record_ordered_obstacles(hard_record)
        hard_prefix_obstacles = [[float(value) for value in item] for item in hard_record.get("obstacles", [])]
        if len(bounds_ordered) < len(hard_prefix_obstacles):
            bounds_ordered = hard_prefix_obstacles
        print(
            f"[augment] {robot_name}/{scene_seed}: ordered_obstacles={len(bounds_ordered)} "
            f"existing_queries={len(by_difficulty['easy'].get('queries', []))} target={int(args.queries_per_scene)}",
            flush=True,
        )
        prefixes = {
            difficulty: (
                len(by_difficulty[difficulty].get("obstacles", [])),
                dict(by_difficulty[difficulty].get("difficulty_probe", {})),
                float(by_difficulty[difficulty].get("direct_obstruction_fraction_mean", math.nan)),
            )
            for difficulty in DIFFICULTY_ORDER
        }
        queries = [dict(item) for item in by_difficulty["easy"].get("queries", [])]
        queries, rejects = augment_prefix_matching_queries(
            robot=robot,
            bounds_ordered=bounds_ordered,
            prefixes=prefixes,
            queries=queries,
            target_count=int(args.queries_per_scene),
            seed=int(args.seed) + 1009 * int(scene_seed),
            min_l2=float(args.query_min_l2),
            max_l2=query_max_l2_limit(float(args.query_max_l2)),
            local_radius=float(args.query_local_radius),
            clearance_margin_m=float(args.query_clearance_margin_m),
            max_tries=int(args.post_query_max_tries),
            planner_seeds=int(args.post_query_planner_seeds),
            rrt_windows=rrt_windows,
            bitstar_windows=bitstar_windows,
            post_rrt_windows=(
                parse_window_map(str(args.post_query_rrt_median_windows))
                if str(args.post_query_rrt_median_windows).strip()
                else None
            ),
            post_bitstar_windows=(
                parse_window_map(str(args.post_query_bitstar_median_windows))
                if str(args.post_query_bitstar_median_windows).strip()
                else None
            ),
            rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
            bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
            bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
            audit_step=float(args.strict_audit_step),
            bitstar_probe_mode=str(args.bitstar_probe_mode),
            bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
            bitstar_rewire_factor=float(args.bitstar_rewire_factor),
            check_mode=str(args.post_query_check_mode),
            local_sample_prob=float(args.post_query_local_prob),
            group_min_success_fraction=float(args.min_probe_success_fraction),
        )
        print(f"[augment] {robot_name}/{scene_seed}: post-query accepted={len(queries)} rejects={rejects}", flush=True)
        extend_traces: list[dict[str, Any]] = []

        def run_find_prefixes(seed_offset: int) -> dict[str, tuple[int, dict[str, Any], float]]:
            return find_prefixes_by_mode(
                selection_mode=str(args.prefix_selection_mode),
                robot_name=robot_name,
                robot=robot,
                bounds_ordered=bounds_ordered,
                queries=queries,
                seed=int(args.seed) + 65_537 * int(scene_seed) + int(seed_offset),
                planner_seeds=int(args.planner_seeds),
                direct_obstruction_samples=int(args.direct_obstruction_samples),
                rrt_windows=rrt_windows,
                bitstar_windows=bitstar_windows,
                rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
                bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                audit_step=float(args.strict_audit_step),
                min_success_fraction=float(args.min_probe_success_fraction),
                bitstar_probe_mode=str(args.bitstar_probe_mode),
                bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                prefix_fine_until=int(args.prefix_fine_until),
                prefix_mid_step=int(args.prefix_mid_step),
                prefix_coarse_step=int(args.prefix_coarse_step),
                distribution_medium_ratio=float(args.distribution_medium_ratio),
                distribution_hard_ratio=float(args.distribution_hard_ratio),
                distribution_hard_not_faster_factor=float(args.distribution_hard_not_faster_factor),
                distribution_require_strong_planner=bool(args.distribution_require_strong_planner),
                distribution_min_medium_count=int(args.distribution_min_medium_count),
                distribution_min_hard_count=int(args.distribution_min_hard_count),
                prefix_confirm_mode=str(args.prefix_confirm_mode),
                prefix_stage_a_planner_seeds=int(args.prefix_stage_a_planner_seeds),
                prefix_stage_b_planner_seeds=int(args.prefix_stage_b_planner_seeds),
                prefix_stage_b_neighbor_radius=int(args.prefix_stage_b_neighbor_radius),
            )

        final_prefixes: dict[str, tuple[int, dict[str, Any], float]] | None = None
        prefix_error: str | None = None
        print(f"[augment] {robot_name}/{scene_seed}: finding strict prefixes on {len(bounds_ordered)} obstacles", flush=True)
        try:
            final_prefixes = run_find_prefixes(0)
            print(
                f"[augment] {robot_name}/{scene_seed}: strict prefixes "
                f"{ {name: int(final_prefixes[name][0]) for name in DIFFICULTY_ORDER} }",
                flush=True,
            )
        except RuntimeError as exc:
            prefix_error = str(exc)
            print(f"[augment] {robot_name}/{scene_seed}: prefix search failed; {prefix_error[:500]}", flush=True)

        if final_prefixes is None and bool(args.extend_path_blocking):
            limits = robot_joint_limit_tuples(robot)
            allowed_link_idxs = resolved_allowed_link_idxs(str(args.allowed_link_idxs), robot_name)
            for round_index in progress(
                range(max(0, int(args.extend_max_rounds))),
                desc=f"{robot_name} augment-extend",
            ):
                before_count = len(bounds_ordered)
                print(
                    f"[augment] {robot_name}/{scene_seed}: extension round {round_index} "
                    f"starting from {before_count} obstacles",
                    flush=True,
                )
                extended_order, trace = path_blocking_obstacle_bounds(
                    robot_name=robot_name,
                    robot=robot,
                    limits=limits,
                    queries=queries,
                    seed=int(args.seed) + 8_388_593 * int(scene_seed) + 1_009 * int(round_index),
                    planner_seeds=int(args.planner_seeds),
                    rrt_probe_timeout_s=float(args.rrt_probe_timeout_s),
                    bitstar_timeout_s=float(args.bitstar_probe_timeout_s),
                    bitstar_checkpoint_interval_s=float(args.bitstar_probe_checkpoint_interval_s),
                    audit_step=float(args.strict_audit_step),
                    min_success_fraction=float(args.min_probe_success_fraction),
                    bitstar_probe_mode=str(args.bitstar_probe_mode),
                    bitstar_samples_per_batch=int(args.bitstar_samples_per_batch),
                    bitstar_rewire_factor=float(args.bitstar_rewire_factor),
                    direct_obstruction_samples=int(args.direct_obstruction_samples),
                    max_obstacles=int(args.extend_max_prefix),
                    max_prefix=int(args.extend_max_prefix),
                    candidate_pool=int(args.extend_candidate_pool),
                    samples_per_path=int(args.extend_samples_per_path),
                    box_half_width=float(args.extend_box_half_width),
                    use_inflated=False,
                    workspace_margin=float(args.workspace_margin),
                    endpoint_source=str(args.endpoint_source),
                    n_samples_crit=int(args.n_samples_crit),
                    endpoint_threads=int(args.endpoint_threads),
                    n_subdivisions=int(args.envelope_subdivisions),
                    workspace_aabb_shrink=float(args.workspace_aabb_shrink),
                    path_blocking_workspace_aabb_shrink=None,
                    min_active_link_idx=int(args.min_active_link_idx),
                    max_active_link_idx=int(args.max_active_link_idx),
                    allowed_link_idxs=allowed_link_idxs,
                    query_count=int(args.extend_query_count),
                    initial_obstacles=bounds_ordered,
                )
                bounds_ordered = [[float(value) for value in item] for item in extended_order]
                extend_traces.append(
                    {
                        "round": int(round_index),
                        "before_count": int(before_count),
                        "after_count": int(len(bounds_ordered)),
                        "added_count": int(max(0, len(bounds_ordered) - before_count)),
                        "trace": trace,
                    }
                )
                print(
                    f"[augment] {robot_name}/{scene_seed}: extension round {round_index} "
                    f"added {max(0, len(bounds_ordered) - before_count)} obstacles",
                    flush=True,
                )
                if len(bounds_ordered) <= before_count:
                    break
                try:
                    final_prefixes = run_find_prefixes(1_000_003 * (round_index + 1))
                    prefix_error = None
                    print(
                        f"[augment] {robot_name}/{scene_seed}: strict prefixes after extension "
                        f"{ {name: int(final_prefixes[name][0]) for name in DIFFICULTY_ORDER} }",
                        flush=True,
                    )
                    break
                except RuntimeError as exc:
                    prefix_error = str(exc)
                    print(
                        f"[augment] {robot_name}/{scene_seed}: prefix search still failed; {prefix_error[:500]}",
                        flush=True,
                    )

        if final_prefixes is None:
            raise RuntimeError(
                f"{robot_name}/{scene_seed} no strict prefix triple after query augmentation; "
                f"last_error={prefix_error}"
            )

        for difficulty in DIFFICULTY_ORDER:
            source = copy.deepcopy(by_difficulty[difficulty])
            count, probe, direct_mean = final_prefixes[difficulty]
            first = queries[0]
            source["queries"] = [dict(item) for item in queries]
            source["queries_per_scene"] = int(len(queries))
            source["start"] = [float(value) for value in first["start"]]
            source["goal"] = [float(value) for value in first["goal"]]
            source["canonical_start"] = [float(value) for value in first["canonical_start"]]
            source["canonical_goal"] = [float(value) for value in first["canonical_goal"]]
            source["obstacles"] = [[float(value) for value in item] for item in bounds_ordered[: int(count)]]
            source["difficulty_probe"] = probe
            source["direct_obstruction_fraction_mean"] = float(direct_mean)
            source.setdefault("workspace_mapping", {})["query_augmentation"] = {
                "tool": "augment_prefix_catalog_queries",
                "target_queries": int(args.queries_per_scene),
                "check_mode": str(args.post_query_check_mode),
                "local_sample_probability": float(args.post_query_local_prob),
                "post_query_rrt_median_windows": str(args.post_query_rrt_median_windows),
                "post_query_bitstar_median_windows": str(args.post_query_bitstar_median_windows),
                "post_query_rejections": rejects,
                "extend_path_blocking": bool(args.extend_path_blocking),
                "extend_traces": extend_traces,
            }
            source.setdefault("workspace_mapping", {})["prefix_selection_mode"] = str(args.prefix_selection_mode)
            source.setdefault("workspace_mapping", {})["obstacle_prefix_difficulty"] = str(difficulty)
            source.setdefault("workspace_mapping", {})["obstacle_prefix_count"] = int(count)
            source.setdefault("workspace_mapping", {})["distribution_separation"] = {
                "medium_over_easy_min": float(args.distribution_medium_ratio),
                "hard_over_medium_min": float(args.distribution_hard_ratio),
                "hard_not_faster_factor": float(args.distribution_hard_not_faster_factor),
                "min_medium_count": int(args.distribution_min_medium_count),
                "min_hard_count": int(args.distribution_min_hard_count),
                "require_strong_reference_planner": bool(args.distribution_require_strong_planner),
            }
            source.setdefault("workspace_mapping", {})["ordered_obstacles"] = [
                [float(value) for value in item]
                for item in bounds_ordered
            ]
            source["incremental_scene"] = {
                **dict(source.get("incremental_scene", {})),
                "shared_query_set": True,
                "obstacle_prefix_count": int(count),
            }
            out_records.append(source)
        summaries.append(
            {
                "robot": robot_name,
                "scene_seed": int(scene_seed),
                "queries": int(len(queries)),
                "prefix_counts": {name: int(final_prefixes[name][0]) for name in DIFFICULTY_ORDER},
                "post_query_rejections": rejects,
                "ordered_obstacle_count": int(len(bounds_ordered)),
                "extend_rounds": int(len(extend_traces)),
            }
        )
    out_payload = {
        **{key: value for key, value in payload.items() if key != "records"},
        "schema": CATALOG_SCHEMA,
        "queries_per_scene": int(args.queries_per_scene),
        "records": out_records,
        "generation_policy": {
            **dict(payload.get("generation_policy", {})),
            "query_augmentation": "prefix_matching_post_sampling",
            "query_augmentation_extend_path_blocking": bool(args.extend_path_blocking),
            "query_augmentation_check_mode": str(args.post_query_check_mode),
            "query_augmentation_local_sample_probability": float(args.post_query_local_prob),
            "query_augmentation_rrt_median_windows": str(args.post_query_rrt_median_windows),
            "query_augmentation_bitstar_median_windows": str(args.post_query_bitstar_median_windows),
            "prefix_selection_mode": str(args.prefix_selection_mode),
            "prefix_confirm_mode": str(args.prefix_confirm_mode),
            "prefix_stage_a_planner_seeds": int(args.prefix_stage_a_planner_seeds),
            "prefix_stage_b_planner_seeds": int(args.prefix_stage_b_planner_seeds),
            "prefix_stage_b_neighbor_radius": int(args.prefix_stage_b_neighbor_radius),
            "distribution_separation": {
                "medium_over_easy_min": float(args.distribution_medium_ratio),
                "hard_over_medium_min": float(args.distribution_hard_ratio),
                "hard_not_faster_factor": float(args.distribution_hard_not_faster_factor),
                "min_medium_count": int(args.distribution_min_medium_count),
                "min_hard_count": int(args.distribution_min_hard_count),
                "require_strong_reference_planner": bool(args.distribution_require_strong_planner),
            },
            "query_max_l2": None if math.isinf(query_max_l2_limit(float(args.query_max_l2))) else float(args.query_max_l2),
            "query_max_l2_unbounded": math.isinf(query_max_l2_limit(float(args.query_max_l2))),
        },
        "environment": environment_metadata(),
        "generation_s": time.perf_counter() - t0,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out_payload, indent=2, sort_keys=True))
    summary = {"catalog": str(args.out), "records": len(out_records), "groups": summaries, "generation_s": time.perf_counter() - t0}
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
