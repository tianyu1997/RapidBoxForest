#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import statistics
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata, write_json  # noqa: E402
from experiments.exp04_shelf_ablation import profile_anchor_segment_recommended as profile  # noqa: E402
from experiments.common import run_shelf_sbf_case as shelf  # noqa: E402


sbf = profile.sbf
DEFAULT_OUT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_leaf_refine_tradeoff"
TARGET3 = {"AS->TS", "TS->CS", "CS->LB"}


def median(values: list[float]) -> float | None:
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return float(statistics.median(finite)) if finite else None


def max_finite(values: list[float]) -> float | None:
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return max(finite) if finite else None


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "NA"
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(x):
        return "NA"
    return f"{x:.{digits}f}"


def point_list(point: Any) -> list[float]:
    return [float(value) for value in list(point)]


def make_profile_args(args: argparse.Namespace,
                      seed: int,
                      case_name: str,
                      deep_ffb_depth: int,
                      deep_max_boxes: int) -> argparse.Namespace:
    return argparse.Namespace(
        out_dir=args.out_dir / "runs" / case_name,
        case_name=case_name,
        rbf_cache_root=args.rbf_cache_root,
        warm_cache_label=args.warm_cache_label,
        rbf_max_depth=args.rbf_max_depth,
        ffb_depth=deep_ffb_depth,
        rbf_ffb_start_depth=15,
        threads=args.threads,
        max_boxes=max(1, deep_max_boxes),
        timeout_ms=args.timeout_ms,
        anchor_target_prob=0.10,
        anchor_wave_targets_per_batch=4,
        rrt_goal_bias=0.15,
        intertree_goal_bias=0.25,
        unexplored_prob=0.20,
        sample_uniform_prob=0.30,
        frontier_face_candidate_limit=128,
        bootstrap_boxes=210,
        bootstrap_depth=15,
        bootstrap_boundary_samples=8,
        component_connect_prob=0.0,
        component_connect_candidate_limit=1,
        component_connect_ffb_depth_increment=14,
        component_connect_neighbor_root_bias=1.0,
        component_connect_neighbor_root_window=1,
        component_connect_chain_steps=0,
        component_connect_chain_max_boxes=0,
        connector_rrt_iters=10000,
        connector_rrt_timeout_ms=300.0,
        connector_rrt_step_size=0.25,
        connector_rrt_goal_bias=0.45,
        connector_pair_timeout_ms=80.0,
        connector_max_pairs_per_gap=3,
        connector_bridge_boxes=0,
        connector_pave_max_chain=0,
        connector_pave_steps=12,
        connector_pave_depth=args.connector_pave_depth,
        connector_pave_fill_gaps=False,
        connector_pave_require_connected_chain=False,
        connector_pave_gap_fill_time_budget_ms=10.0,
        connector_pave_gap_fill_max_ffb_calls=32,
        connector_pave_gap_fill_sample_step=0.05,
        connector_pave_gap_fill_min_arc_gain=0.01,
        segment_edge_policy="normal",
        collision_shortcut=False,
        final_ompl_simplify_time_s=0.0,
        enable_merger=False,
        sample_categorical_allocation=True,
        query_shortcut_boxes=False,
    )


def configure_forest(args: argparse.Namespace,
                     seed: int,
                     case_name: str,
                     deep_ffb_depth: int,
                     deep_max_boxes: int) -> tuple[Any, Any, list[Any], list[Any]]:
    profile_args = make_profile_args(args, seed, case_name, deep_ffb_depth, deep_max_boxes)
    case_args = profile.recommended_case_args(profile_args, seed)
    effective_args = shelf.effective_case_args(case_args)
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    cfg = shelf.case_config(effective_args, robot, seed)
    cfg.query.shortcut_boxes = False
    cfg.query.collision_shortcut = False
    cfg.database.create_if_missing = True
    cfg.validation.enable_endpoint_evidence_cache = False
    cfg.validation.store_endpoint_evidence_cache = False
    database_path = Path(cfg.database.path)
    if bool(effective_args.clean_active_cache) and database_path.exists():
        shutil.rmtree(database_path)
    if bool(effective_args.use_external_evidence):
        warm_path = Path(effective_args.rbf_cache_root) / str(effective_args.warm_cache_label)
        shelf.configure_external_evidence_reuse(
            cfg,
            warm_path,
            effective_args,
            materialization=bool(effective_args.external_evidence_materialization),
            scoring=bool(effective_args.external_evidence_scoring),
            backfill_active=False,
        )
    return robot, obstacles, queries, cfg


def query_priority_points(robot: Any, queries: list[Any], labels: set[str]) -> list[list[float]]:
    points: list[list[float]] = []
    for query in queries:
        label = str(getattr(query, "label", ""))
        start = point_list(query.start)
        goal = point_list(query.goal)
        fractions = (0.0, 1.0)
        if not labels or label in labels:
            fractions = (0.0, 0.25, 0.5, 0.75, 1.0)
        for fraction in fractions:
            point = [(1.0 - fraction) * a + fraction * b for a, b in zip(start, goal)]
            points.append(list(sbf.canonicalize_configuration_for_robot(robot, point)))
    return points


def run_query_summary(robot: Any, forest: Any, queries: list[Any]) -> dict[str, Any]:
    rows = []
    total_length = 0.0
    total_segment = 0.0
    target_length = 0.0
    target_segment = 0.0
    for query in queries:
        start = list(sbf.canonicalize_configuration_for_robot(robot, list(query.start)))
        goal = list(sbf.canonicalize_configuration_for_robot(robot, list(query.goal)))
        result = forest.query(start, goal)
        length = float(result.path_length)
        segment = float(result.segment_edge_length)
        fraction = segment / length if length > 1e-12 else 0.0
        label = str(getattr(query, "label", ""))
        rows.append({
            "name": label,
            "success": bool(result.success),
            "audit_passed": bool(result.audit_passed),
            "path_length": length,
            "segment_edge_length": segment,
            "segment_fraction": fraction,
            "segment_edges_used": int(result.segment_edges_used),
        })
        if result.success:
            total_length += length
            total_segment += segment
            if label in TARGET3:
                target_length += length
                target_segment += segment
    return {
        "queries": rows,
        "route_length": total_length,
        "segment_length": total_segment,
        "segment_fraction": total_segment / total_length if total_length > 1e-12 else float("nan"),
        "target3_length": target_length,
        "target3_segment_length": target_segment,
        "target3_segment_fraction": target_segment / target_length if target_length > 1e-12 else float("nan"),
        "ok": all(row["success"] and row["audit_passed"] for row in rows),
    }


def run_one(args: argparse.Namespace, seed: int, leaf_start: int, leaf_max: int,
            deep_boxes: int, deep_depth: int, refine_timeout: float) -> dict[str, Any]:
    case_name = f"leaf{leaf_start}_{leaf_max}_box{deep_boxes}_d{deep_depth}_t{int(refine_timeout)}_seed{seed}"
    robot, obstacles, queries, cfg = configure_forest(args, seed, case_name, deep_depth, deep_boxes)
    forest = sbf.SafeBoxForest(robot, cfg)
    refine_cfg = sbf.LeafSweepRefineConfig()
    refine_cfg.leaf_start_depth = int(leaf_start)
    refine_cfg.leaf_max_depth = int(leaf_max)
    refine_cfg.obstacle_cluster_gap = 1000.0
    refine_cfg.use_virtual_topology = True
    refine_cfg.parallel_virtual_validation = bool(args.parallel_virtual_validation)
    refine_cfg.store_group_results = False
    refine_cfg.validation_batch_size = 512
    refine_cfg.leaf_threads = int(args.leaf_threads)
    refine_cfg.deep_max_boxes = int(deep_boxes)
    refine_cfg.deep_ffb_depth = int(deep_depth)
    refine_cfg.domain_seed_cap = int(args.domain_seed_cap)
    refine_cfg.domain_success_cap = int(args.domain_success_cap)
    refine_cfg.domain_attempt_cap = int(args.domain_attempt_cap)
    refine_cfg.allow_anchor_roots = bool(args.allow_anchor_roots)
    refine_cfg.refine_timeout_ms = float(refine_timeout)
    priority = query_priority_points(robot, queries, set())
    start = time.perf_counter()
    build = forest.build_leaf_sweep_refined(obstacles, refine_cfg, priority)
    wall_ms = 1000.0 * (time.perf_counter() - start)
    query_summary = run_query_summary(robot, forest, queries)
    ok = bool(query_summary["ok"])
    return {
        "seed": int(seed),
        "case_name": case_name,
        "leaf_start_depth": int(leaf_start),
        "leaf_max_depth": int(leaf_max),
        "deep_max_boxes": int(deep_boxes),
        "deep_ffb_depth": int(deep_depth),
        "refine_timeout_ms": float(refine_timeout),
        "wall_ms": wall_ms,
        "leaf_sweep_ms": float(build.leaf_sweep_ms),
        "deep_refine_ms": float(build.deep_refine_ms),
        "connector_ms": float(build.connector_ms),
        "total_ms": float(build.total_ms),
        "leaf_free_count": int(build.leaf_free_count),
        "leaf_collision_count": int(build.leaf_collision_count),
        "deep_boxes_added": int(build.deep_boxes_added),
        "deep_domain_attempts": int(build.deep_domain_attempts),
        "deep_ffb_success": int(build.deep_ffb_success),
        "deep_ffb_fail": int(build.deep_ffb_fail),
        "deep_commit_rejects": int(build.deep_commit_rejects),
        "deep_domain_rejects": int(build.deep_domain_rejects),
        "deep_contained_rejects": int(build.deep_contained_rejects),
        "deep_adjacency_rejects": int(build.deep_adjacency_rejects),
        "deep_anchor_roots_added": int(build.deep_anchor_roots_added),
        "final_boxes": int(build.profile.final_boxes),
        "segment_edges": int(build.profile.segment_edges),
        "adjacency_islands": int(build.profile.adjacency_islands),
        "ok": ok,
        "route_length": float(query_summary["route_length"]) if ok else float("nan"),
        "segment_fraction": float(query_summary["segment_fraction"]) if ok else float("nan"),
        "target3_segment_fraction": float(query_summary["target3_segment_fraction"]) if ok else float("nan"),
        "queries": query_summary["queries"],
    }


def aggregate_case(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "n": len(rows),
        "ok_count": sum(1 for row in rows if row["ok"]),
        "planning_ms_median": median([row["total_ms"] for row in rows]),
        "wall_ms_median": median([row["wall_ms"] for row in rows]),
        "leaf_sweep_ms_median": median([row["leaf_sweep_ms"] for row in rows]),
        "deep_refine_ms_median": median([row["deep_refine_ms"] for row in rows]),
        "connector_ms_median": median([row["connector_ms"] for row in rows]),
        "route_length_median": median([row["route_length"] for row in rows]),
        "segment_fraction_median": median([row["segment_fraction"] for row in rows]),
        "segment_fraction_max": max_finite([row["segment_fraction"] for row in rows]),
        "target3_segment_fraction_median": median([row["target3_segment_fraction"] for row in rows]),
        "target3_segment_fraction_max": max_finite([row["target3_segment_fraction"] for row in rows]),
        "final_boxes_median": median([row["final_boxes"] for row in rows]),
        "deep_boxes_added_median": median([row["deep_boxes_added"] for row in rows]),
        "deep_anchor_roots_median": median([row["deep_anchor_roots_added"] for row in rows]),
    }


def write_summary(args: argparse.Namespace, payload: dict[str, Any]) -> None:
    rows = payload["cases"]
    csv_path = args.out_dir / "leaf_refine_tradeoff_summary.csv"
    md_path = args.out_dir / "leaf_refine_tradeoff_summary.md"
    fields = [
        "case_id", "ok_count", "n", "planning_ms_median", "leaf_sweep_ms_median",
        "deep_refine_ms_median", "connector_ms_median", "route_length_median",
        "segment_fraction_median", "segment_fraction_max",
        "target3_segment_fraction_median", "target3_segment_fraction_max",
        "final_boxes_median", "deep_boxes_added_median", "deep_anchor_roots_median",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            agg = row["aggregate"]
            writer.writerow({"case_id": row["case_id"], **{key: agg.get(key) for key in fields if key != "case_id"}})
    lines = [
        "# Exp04 Leaf Sweep Refine Trade-off",
        "",
        "| case | ok | time ms | leaf ms | refine ms | conn ms | length | seg med | seg max | target3 med | target3 max | boxes | added | roots |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        agg = row["aggregate"]
        lines.append(
            f"| {row['case_id']} | {agg['ok_count']}/{agg['n']} | {fmt(agg['planning_ms_median'], 1)} | "
            f"{fmt(agg['leaf_sweep_ms_median'], 1)} | {fmt(agg['deep_refine_ms_median'], 1)} | "
            f"{fmt(agg['connector_ms_median'], 1)} | {fmt(agg['route_length_median'], 3)} | "
            f"{fmt(agg['segment_fraction_median'], 3)} | {fmt(agg['segment_fraction_max'], 3)} | "
            f"{fmt(agg['target3_segment_fraction_median'], 3)} | {fmt(agg['target3_segment_fraction_max'], 3)} | "
            f"{fmt(agg['final_boxes_median'], 0)} | {fmt(agg['deep_boxes_added_median'], 0)} | "
            f"{fmt(agg['deep_anchor_roots_median'], 0)} |"
        )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    write_plots(args, rows)


def write_plots(args: argparse.Namespace, rows: list[dict[str, Any]]) -> None:
    try:
        os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "matplotlib-codex"))
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    selected = sorted(rows, key=lambda row: (
        row["leaf_start_depth"], row["leaf_max_depth"], row["deep_ffb_depth"],
        row["refine_timeout_ms"], row["deep_max_boxes"],
    ))
    x = list(range(len(selected)))
    labels = [row["case_id"] for row in selected]
    fig, axes = plt.subplots(3, 1, figsize=(max(8, len(selected) * 0.35), 9), sharex=True)
    def plot_value(row: dict[str, Any], key: str) -> float:
        value = row["aggregate"].get(key)
        return float(value) if value is not None else float("nan")

    axes[0].plot(x, [plot_value(row, "planning_ms_median") for row in selected], marker="o")
    axes[0].set_ylabel("time ms")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(x, [plot_value(row, "segment_fraction_median") for row in selected], marker="o", label="overall")
    axes[1].plot(x, [plot_value(row, "target3_segment_fraction_median") for row in selected], marker="s", label="target3")
    axes[1].axhline(0.4, color="tab:red", ls="--", lw=1)
    axes[1].set_ylabel("segment fraction")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(x, [plot_value(row, "route_length_median") for row in selected], marker="o")
    axes[2].set_ylabel("route length")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=80, ha="right", fontsize=7)
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out_dir / "leaf_refine_tradeoff_curves.png", dpi=180)


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def parse_depth_pairs(text: str) -> list[tuple[int, int]]:
    pairs = []
    for item in str(text).split(","):
        item = item.strip()
        if not item:
            continue
        lhs, rhs = item.split(":")
        pairs.append((int(lhs), int(rhs)))
    return pairs


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in str(text).split(",") if item.strip()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Exp04 leaf-sweep refined trade-off sweep.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--seeds-list", default="0,1,2")
    parser.add_argument("--leaf-depths", default="8:14,10:16,10:18,12:18")
    parser.add_argument("--deep-max-boxes-list", default="200,400,600,800,1000")
    parser.add_argument("--deep-ffb-depth-list", default="28,34")
    parser.add_argument("--refine-timeout-ms-list", default="400,800,1200")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--leaf-threads", type=int, default=8)
    parser.add_argument("--parallel-virtual-validation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--connector-pave-depth", type=int, default=28)
    parser.add_argument("--domain-seed-cap", type=int, default=24)
    parser.add_argument("--domain-success-cap", type=int, default=8)
    parser.add_argument("--domain-attempt-cap", type=int, default=24)
    parser.add_argument("--allow-anchor-roots", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--rbf-cache-root", type=Path, default=profile.D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=profile.D23_CACHE_LABEL)
    parser.add_argument("--confirm-top", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    seeds = parse_int_list(args.seeds_list)
    cases = []
    for leaf_start, leaf_max in parse_depth_pairs(args.leaf_depths):
        for deep_boxes in parse_int_list(args.deep_max_boxes_list):
            for deep_depth in parse_int_list(args.deep_ffb_depth_list):
                for refine_timeout in parse_float_list(args.refine_timeout_ms_list):
                    case_id = f"leaf{leaf_start}_{leaf_max}_box{deep_boxes}_d{deep_depth}_t{int(refine_timeout)}"
                    print(f"[leaf-refine] case={case_id}", flush=True)
                    seed_rows = [
                        run_one(args, seed, leaf_start, leaf_max, deep_boxes, deep_depth, refine_timeout)
                        for seed in seeds
                    ]
                    cases.append({
                        "case_id": case_id,
                        "leaf_start_depth": leaf_start,
                        "leaf_max_depth": leaf_max,
                        "deep_max_boxes": deep_boxes,
                        "deep_ffb_depth": deep_depth,
                        "refine_timeout_ms": refine_timeout,
                        "seeds": seed_rows,
                        "aggregate": aggregate_case(seed_rows),
                    })
    payload = {
        "experiment": "exp04_leaf_sweep_refine_tradeoff",
        "environment": environment_metadata(),
        "seeds": seeds,
        "cases": cases,
    }
    write_json(args.out_dir / "leaf_refine_tradeoff_summary.json", payload)
    write_summary(args, payload)
    print(json.dumps({
        "out_dir": str(args.out_dir),
        "cases": len(cases),
        "summary": str(args.out_dir / "leaf_refine_tradeoff_summary.md"),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
