#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median, tex_num
from experiments.common.progress import progress
from experiments.common.random_scene_catalog import generate_catalog, make_robot, scene_for_key
from experiments.common.rbf_defaults import (
    DEFAULT_RBF_DEEP_MAX_BOXES,
    ROBOT_LECTDB_CACHE_ROOT,
    default_rbf_profile,
    rbf_budget_grid,
    robot_lectdb_profile,
    robot_sector_expanded_root_tuples,
)
from experiments.common.rbf_leaf_rrt import (
    QuerySpec,
    RBFLeafRRTOptions,
    canonical_priority_points,
    configure_leaf_rrt,
    make_refine_config,
    query_rows,
    run_leaf_rrt,
)
from experiments.common.robot_lectdb_cache import ensure_robot_lectdb_cache, robot_external_evidence_path
from experiments.common.sbf_import import import_sbf


TRANSITIONS = ["easy->medium", "medium->hard", "hard->medium", "medium->easy"]
sbf = import_sbf()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.7 current dynamic-update study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp07")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scene-catalog", type=Path, default=None)
    parser.add_argument("--scene-catalog-mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--robot", default="iiwa")
    parser.add_argument("--scene-profile", choices=["balanced", "balanced_probe", "legacy"], default="balanced")
    parser.add_argument("--max-scene-tries", type=int, default=64)
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--deep-max-boxes", type=int, default=DEFAULT_RBF_DEEP_MAX_BOXES)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--lect-cache-root", type=Path, default=ROBOT_LECTDB_CACHE_ROOT)
    parser.add_argument("--skip-lect-cache-ensure", action="store_true")
    return parser.parse_args()


def profile_row(profile: Any) -> dict[str, Any]:
    return {
        "boxes_before": int(getattr(profile, "boxes_before", 0)),
        "boxes_after": int(getattr(profile, "boxes_after", 0)),
        "boxes_added": int(getattr(profile, "boxes_added", 0)),
        "boxes_removed": int(getattr(profile, "boxes_removed", 0)),
        "dirty_boxes": int(getattr(profile, "dirty_boxes", 0)),
        "dirty_seed_count": int(getattr(profile, "dirty_seed_count", 0)),
        "regrow_attempts": int(getattr(profile, "regrow_attempts", 0)),
        "segment_edges_added": int(getattr(profile, "segment_edges_added", 0)),
        "collision_cache_candidates": int(getattr(profile, "collision_cache_candidates", 0)),
        "collision_cache_promoted": int(getattr(profile, "collision_cache_promoted", 0)),
        "used_warm_rebuild": bool(getattr(profile, "used_warm_rebuild", False)),
        "fallback_reason": str(getattr(profile, "fallback_reason", "")),
        "dirty_region_ms": float(getattr(profile, "dirty_region_ms", 0.0)),
        "regrow_ms": float(getattr(profile, "regrow_ms", 0.0)),
        "warm_rebuild_ms": float(getattr(profile, "warm_rebuild_ms", 0.0)),
        "total_ms": float(getattr(profile, "total_ms", 0.0)),
    }


def merge_profile_rows(items: list[dict[str, Any]]) -> dict[str, Any]:
    if not items:
        return {
            "boxes_before": 0,
            "boxes_after": 0,
            "boxes_added": 0,
            "boxes_removed": 0,
            "dirty_boxes": 0,
            "dirty_seed_count": 0,
            "regrow_attempts": 0,
            "segment_edges_added": 0,
            "collision_cache_candidates": 0,
            "collision_cache_promoted": 0,
            "used_warm_rebuild": False,
            "fallback_reason": "",
            "dirty_region_ms": 0.0,
            "regrow_ms": 0.0,
            "warm_rebuild_ms": 0.0,
            "total_ms": 0.0,
        }
    first = items[0]
    last = items[-1]
    summed = dict(last)
    summed["boxes_before"] = first["boxes_before"]
    summed["boxes_added"] = sum(int(row["boxes_added"]) for row in items)
    summed["boxes_removed"] = sum(int(row["boxes_removed"]) for row in items)
    summed["dirty_boxes"] = sum(int(row["dirty_boxes"]) for row in items)
    summed["dirty_seed_count"] = sum(int(row["dirty_seed_count"]) for row in items)
    summed["regrow_attempts"] = sum(int(row["regrow_attempts"]) for row in items)
    summed["segment_edges_added"] = sum(int(row["segment_edges_added"]) for row in items)
    summed["collision_cache_candidates"] = sum(int(row["collision_cache_candidates"]) for row in items)
    summed["collision_cache_promoted"] = sum(int(row["collision_cache_promoted"]) for row in items)
    summed["used_warm_rebuild"] = any(bool(row["used_warm_rebuild"]) for row in items)
    summed["fallback_reason"] = ";".join(str(row["fallback_reason"]) for row in items if str(row["fallback_reason"]))
    for key in ["dirty_region_ms", "regrow_ms", "warm_rebuild_ms", "total_ms"]:
        summed[key] = sum(float(row[key]) for row in items)
    return summed


def add_profile_rows(lhs: dict[str, Any], rhs: dict[str, Any]) -> dict[str, Any]:
    return merge_profile_rows([lhs, rhs])


def options(args: argparse.Namespace, seed: int, label: str) -> RBFLeafRRTOptions:
    root_override = robot_sector_expanded_root_tuples(str(args.robot), make_robot(str(args.robot)))
    return RBFLeafRRTOptions(
        seed=int(seed),
        deep_max_boxes=int(args.deep_max_boxes),
        threads=int(args.threads),
        use_external_evidence=True,
        external_evidence_path=robot_external_evidence_path(str(args.robot), cache_root=Path(args.lect_cache_root)),
        external_evidence_verify_identity=root_override is None,
        root_override_tuples=root_override,
        coverage_override_tuples=root_override,
        database_canonical_mode=True,
        case_label=label,
        parallel_virtual_validation=False,
        leaf_threads=1,
        canonicalize_queries=False,
    )


def run_transition(args: argparse.Namespace, catalog: dict[str, Any], transition: str, seed: int) -> dict[str, Any]:
    source_diff, target_diff = transition.split("->")
    robot_name = str(args.robot)
    source = scene_for_key(catalog, robot_name, source_diff, seed)
    target = scene_for_key(catalog, robot_name, target_diff, seed)
    robot = make_robot(robot_name)
    query = QuerySpec(
        label=f"{robot_name}_{transition}_{seed}",
        start=list(source.start),
        goal=list(source.goal),
        actual_start=list(source.start),
        actual_goal=list(source.goal),
    )
    opt = options(args, seed, f"dynamic_{robot_name}_{transition}")
    cfg = configure_leaf_rrt(robot, args.out_dir / "active_cache" / f"dynamic_{transition}_{seed}", opt)
    cfg.dynamic_update.dirty_region_padding = 100.0
    cfg.dynamic_update.local_regrow_box_limit = int(args.deep_max_boxes)
    cfg.dynamic_update.local_regrow_timeout_ms = 1000.0
    cfg.dynamic_update.enable_warm_rebuild_fallback = False
    cfg.dynamic_update.warm_rebuild_dirty_box_fraction = 0.0
    cfg.dynamic_update.warm_rebuild_min_local_boxes_added = int(args.deep_max_boxes) + 1
    forest = sbf.SafeBoxForest(robot, cfg)
    forest.build_leaf_sweep_refined(
        list(source.obstacles),
        make_refine_config(opt),
        canonical_priority_points(robot, [query], canonicalize=False),
    )
    source_query = query_rows(
        forest,
        robot,
        [query],
        obstacles=list(source.obstacles),
        audit_step=opt.audit_segment_step,
        canonicalize_queries=False,
    )[0]
    source_count = len(source.obstacles)
    target_count = len(target.obstacles)
    if target_count > source_count:
        update = merge_profile_rows([profile_row(forest.add_obstacle_and_rebuild(obstacle)) for obstacle in target.obstacles[source_count:target_count]])
    else:
        update = profile_row(forest.remove_obstacle_suffix_and_regrow(target_count))
    target_query_spec = QuerySpec(
        label=query.label,
        start=list(target.start),
        goal=list(target.goal),
        actual_start=list(target.start),
        actual_goal=list(target.goal),
    )
    target_query = query_rows(
        forest,
        robot,
        [target_query_spec],
        obstacles=list(target.obstacles),
        audit_step=opt.audit_segment_step,
        canonicalize_queries=False,
    )[0]
    target_source = "incremental"
    if (
        target_count > source_count
        and not bool(target_query["audit_passed"])
        and hasattr(forest, "connect_update_endpoint_segment_fallback")
    ):
        fallback = profile_row(forest.connect_update_endpoint_segment_fallback(list(target_query_spec.start), list(target_query_spec.goal)))
        fallback["fallback_reason"] = fallback["fallback_reason"] or "endpoint_segment_fallback_after_failed_audit"
        update = add_profile_rows(update, fallback)
        target_query = query_rows(
            forest,
            robot,
            [target_query_spec],
            obstacles=list(target.obstacles),
            audit_step=opt.audit_segment_step,
            canonicalize_queries=False,
        )[0]
        target_source = "endpoint_segment_fallback"
    warm = run_leaf_rrt(
        robot=robot,
        obstacles=list(target.obstacles),
        queries=[target_query_spec],
        database_path=args.out_dir / "active_cache" / f"warm_{transition}_{seed}",
        options=options(args, seed, f"warm_{robot_name}_{transition}"),
    )
    return {
        "transition": transition,
        "robot": robot_name,
        "seed": int(seed),
        "source_obstacles": source_count,
        "target_obstacles": target_count,
        "source_audit_passed": bool(source_query["audit_passed"]),
        "target_audit_passed": bool(target_query["audit_passed"]),
        "target_source": target_source,
        "update_s": update["total_ms"] / 1000.0,
        "warm_rebuild_s": float(warm["planning_s"]),
        "speedup_vs_warm": (float(warm["planning_s"]) / (update["total_ms"] / 1000.0)) if update["total_ms"] > 1e-9 else math.nan,
        "target_path_length": float(target_query["path_length"]) if bool(target_query["audit_passed"]) else math.nan,
        "target_segment_fraction": float(target_query["segment_fraction"]) if bool(target_query["audit_passed"]) else math.nan,
        "warm_path_length": float(warm["path_length_mean"]),
        "warm_segment_fraction": float(warm["raw_segment_fraction"]),
        **update,
        "status": "executed",
        "lectdb": robot_lectdb_profile(robot_name),
        "external_evidence_path": str(robot_external_evidence_path(robot_name, cache_root=Path(args.lect_cache_root))),
    }


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for transition in sorted({str(row["transition"]) for row in rows}):
        items = [row for row in rows if str(row["transition"]) == transition]
        out.append(
            {
                "transition": transition,
                "runs": len(items),
                "success_runs": sum(1 for row in items if bool(row["target_audit_passed"])),
                "update_s_median": median(row["update_s"] for row in items),
                "warm_rebuild_s_median": median(row["warm_rebuild_s"] for row in items),
                "speedup_median": median(row["speedup_vs_warm"] for row in items),
                "dirty_boxes_median": median(row["dirty_boxes"] for row in items),
                "boxes_added_median": median(row["boxes_added"] for row in items),
                "boxes_removed_median": median(row["boxes_removed"] for row in items),
                "segment_fraction_median": median(row["target_segment_fraction"] for row in items if bool(row["target_audit_passed"])),
                "path_length_mean": mean(row["target_path_length"] for row in items if bool(row["target_audit_passed"])),
                "segment_fallback_runs": sum(1 for row in items if str(row.get("target_source")) == "endpoint_segment_fallback"),
                "status": "executed",
            }
        )
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "transition",
        "runs",
        "success_runs",
        "update_s_median",
        "warm_rebuild_s_median",
        "speedup_median",
        "dirty_boxes_median",
        "boxes_added_median",
        "boxes_removed_median",
        "path_length_mean",
        "segment_fraction_median",
        "segment_fallback_runs",
        "status",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)


def write_tex(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Saved-catalog dynamic-update results. Update time is compared with a fresh warm rebuild on the target scene; final audit time is excluded.}",
        r"\label{tab:tro-dynamic-update}",
        r"\footnotesize",
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Transition & SR & Update (s) & Warm (s) & Speedup \\",
        r"\midrule",
    ]
    for row in rows:
        sr = f"{int(row.get('success_runs', 0))}/{int(row.get('runs', 0))}"
        lines.append(
            f"{row.get('transition')} & {sr} & {tex_num(row.get('update_s_median'))} & "
            f"{tex_num(row.get('warm_rebuild_s_median'))} & {tex_num(row.get('speedup_median'))} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    seeds = [int(item) for item in str(args.seeds).split(",") if item.strip()]
    if args.phase == "smoke":
        seeds = seeds[:1]
        transitions = ["medium->easy"]
    else:
        transitions = TRANSITIONS
    cache_rows: list[dict[str, Any]] = []
    for robot_name in progress([str(args.robot)], desc="exp07 lect cache", total=1, disable=bool(args.dry_run or args.skip_lect_cache_ensure)):
        cache_rows.append(
            ensure_robot_lectdb_cache(
                robot_name,
                cache_root=Path(args.lect_cache_root),
                threads=int(args.threads),
                dry_run=bool(args.dry_run or args.skip_lect_cache_ensure),
            )
        )
    catalog = args.scene_catalog or (args.out_dir / "dynamic_scene_catalog.json")
    catalog_payload: dict[str, Any] | None = None
    if not args.dry_run:
        catalog_payload = generate_catalog(
            path=catalog,
            robots=[str(args.robot)],
            difficulties=["easy", "medium", "hard"],
            scene_seeds=max(seeds) + 1 if seeds else 1,
            scene_profile=str(args.scene_profile),
            seed_base=int(args.seed_base),
            max_scene_tries=int(args.max_scene_tries),
            mode=str(args.scene_catalog_mode),
        )
    rows = [
        {
            "transition": transition,
            "seed": seed,
            "scene_catalog": str(catalog),
            "scene_catalog_mode": str(args.scene_catalog_mode),
            "backend": "leaf_sweep_update_current_exp07",
            "warm_rebuild_backend": "build_leaf_sweep_refined",
            "rbf_default_profile": default_rbf_profile(),
            "rbf_robot_lectdb": robot_lectdb_profile(str(args.robot)),
            "rbf_box_budgets": rbf_budget_grid(args.phase),
            "status": "planned" if args.dry_run else "planned_for_execution",
            "metrics": ["update_s", "warm_rebuild_s", "invalidated_boxes", "promoted_boxes", "audit_success", "segment_fraction"],
        }
        for transition in transitions
        for seed in seeds
    ]
    run_rows: list[dict[str, Any]] = []
    if not args.dry_run and catalog_payload is not None:
        for row in progress(rows, desc="exp07 transitions", total=len(rows)):
            print(f"[exp07] transition={row['transition']} seed={row['seed']}", flush=True)
            run_rows.append(run_transition(args, catalog_payload, str(row["transition"]), int(row["seed"])))
    summary_rows = aggregate(run_rows) if run_rows else []
    payload: dict[str, Any] = {
        "experiment": "exp07_dynamic_update",
        "run_id": run_id("exp07"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "rbf_default_profile": default_rbf_profile(),
        "lectdb_caches": cache_rows,
        "scene_catalog": {
            "path": str(catalog),
            "mode": str(args.scene_catalog_mode),
            "schema": catalog_payload.get("schema") if catalog_payload is not None else None,
            "records": len(catalog_payload.get("records", [])) if catalog_payload is not None else None,
            "scene_profile": str(args.scene_profile),
            "seed_base": int(args.seed_base),
            "max_scene_tries": int(args.max_scene_tries),
        },
        "planned_rows": rows,
        "rows": run_rows,
        "summary": summary_rows,
    }
    write_json(args.out_dir / "dynamic_update_manifest.json", payload)
    if summary_rows:
        write_csv(args.out_dir / "dynamic_update_summary.csv", summary_rows)
        write_tex(REPO_ROOT / "paper" / "generated" / "tab_tro_dynamic_update.tex", summary_rows)
    print(f"wrote {args.out_dir / 'dynamic_update_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
