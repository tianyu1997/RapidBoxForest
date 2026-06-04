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
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import environment_metadata, write_json  # noqa: E402
from experiments.common import run_shelf_sbf_case as shelf  # noqa: E402
from experiments.exp04_shelf_ablation import profile_anchor_segment_recommended as profile  # noqa: E402
from experiments.exp04_shelf_ablation import run_leaf_refine_tradeoff as leaf  # noqa: E402


sbf = profile.sbf
DEFAULT_OUT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_leaf_refine_ablation"
TARGET3 = {"AS->TS", "TS->CS", "CS->LB"}


@dataclass(frozen=True)
class AblationRow:
    name: str
    factor: str
    description: str
    deep_max_boxes: int = 400
    deep_ffb_depth: int = 28
    refine_timeout_ms: float = 800.0
    leaf_start_depth: int = 8
    leaf_max_depth: int = 14
    envelope: str = "support_hull"
    threads: int = 8
    leaf_threads: int = 8
    use_external_evidence: bool = False
    endpoint_evidence_cache: bool = False
    parallel_virtual_validation: bool = True
    allow_anchor_roots: bool = True


def finite(values: list[float]) -> list[float]:
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


def median(values: list[float]) -> float | None:
    vals = finite(values)
    return float(statistics.median(vals)) if vals else None


def max_finite(values: list[float]) -> float | None:
    vals = finite(values)
    return max(vals) if vals else None


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


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def make_profile_args(args: argparse.Namespace, row: AblationRow, seed: int, case_name: str) -> argparse.Namespace:
    base = argparse.Namespace(
        out_dir=args.out_dir / "runs" / case_name,
        case_name=case_name,
        rbf_cache_root=args.rbf_cache_root,
        warm_cache_label=args.warm_cache_label,
        rbf_max_depth=args.rbf_max_depth,
        ffb_depth=row.deep_ffb_depth,
        rbf_ffb_start_depth=args.rbf_ffb_start_depth,
        threads=row.threads,
        max_boxes=max(1, row.deep_max_boxes),
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
    return base


def configure_forest(args: argparse.Namespace,
                     row: AblationRow,
                     seed: int,
                     case_name: str) -> tuple[Any, list[Any], list[Any], Any]:
    profile_args = make_profile_args(args, row, seed, case_name)
    case_args = profile.recommended_case_args(profile_args, seed)
    case_args.use_external_evidence = bool(row.use_external_evidence)
    case_args.endpoint_evidence_cache = bool(row.endpoint_evidence_cache)
    case_args.worker_shared_endpoint_cache = bool(row.endpoint_evidence_cache)
    case_args.incremental_materialization_bind_online_cache = bool(row.endpoint_evidence_cache)
    case_args.rbf_envelope = str(row.envelope)
    case_args.threads = int(row.threads)
    case_args.task_batch_size = max(1, int(row.threads))
    case_args.clean_active_cache = True
    effective_args = shelf.effective_case_args(case_args)

    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    cfg = shelf.case_config(effective_args, robot, seed)
    cfg.query.shortcut_boxes = False
    cfg.query.collision_shortcut = False
    cfg.database.create_if_missing = True
    cfg.validation.enable_endpoint_evidence_cache = bool(row.endpoint_evidence_cache)
    cfg.validation.store_endpoint_evidence_cache = bool(row.endpoint_evidence_cache)

    database_path = Path(cfg.database.path)
    if bool(effective_args.clean_active_cache) and database_path.exists():
        shutil.rmtree(database_path)
    if bool(row.use_external_evidence):
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


def run_one(args: argparse.Namespace, row: AblationRow, seed: int) -> dict[str, Any]:
    case_name = f"{row.name}_seed{seed}"
    robot, obstacles, queries, cfg = configure_forest(args, row, seed, case_name)
    forest = sbf.SafeBoxForest(robot, cfg)
    refine_cfg = sbf.LeafSweepRefineConfig()
    refine_cfg.leaf_start_depth = int(row.leaf_start_depth)
    refine_cfg.leaf_max_depth = int(row.leaf_max_depth)
    refine_cfg.obstacle_cluster_gap = 1000.0
    refine_cfg.use_virtual_topology = True
    refine_cfg.parallel_virtual_validation = bool(row.parallel_virtual_validation)
    refine_cfg.store_group_results = False
    refine_cfg.validation_batch_size = 512
    refine_cfg.leaf_threads = int(row.leaf_threads)
    refine_cfg.deep_max_boxes = int(row.deep_max_boxes)
    refine_cfg.deep_ffb_depth = int(row.deep_ffb_depth)
    refine_cfg.domain_seed_cap = int(args.domain_seed_cap)
    refine_cfg.domain_success_cap = int(args.domain_success_cap)
    refine_cfg.domain_attempt_cap = int(args.domain_attempt_cap)
    refine_cfg.allow_anchor_roots = bool(row.allow_anchor_roots)
    refine_cfg.refine_timeout_ms = float(row.refine_timeout_ms)

    priority = leaf.query_priority_points(robot, queries, set())
    start = time.perf_counter()
    build = forest.build_leaf_sweep_refined(obstacles, refine_cfg, priority)
    wall_ms = 1000.0 * (time.perf_counter() - start)
    query_summary = leaf.run_query_summary(robot, forest, queries)
    ok = bool(query_summary["ok"])
    diagnostics = {str(key): float(value) for key, value in dict(build.diagnostics).items()}
    del forest
    return {
        "seed": int(seed),
        "case_name": case_name,
        "ok": ok,
        "wall_ms": wall_ms,
        "planning_ms": float(build.total_ms),
        "leaf_sweep_ms": float(build.leaf_sweep_ms),
        "deep_refine_ms": float(build.deep_refine_ms),
        "connector_ms": float(build.connector_ms),
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
        "route_length": float(query_summary["route_length"]) if ok else float("nan"),
        "segment_fraction": float(query_summary["segment_fraction"]) if ok else float("nan"),
        "target3_segment_fraction": float(query_summary["target3_segment_fraction"]) if ok else float("nan"),
        "queries": query_summary["queries"],
        "diagnostics": diagnostics,
    }


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "n": len(rows),
        "ok_count": sum(1 for row in rows if row["ok"]),
        "planning_ms_median": median([row["planning_ms"] for row in rows]),
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
        "leaf_free_count_median": median([row["leaf_free_count"] for row in rows]),
        "leaf_collision_count_median": median([row["leaf_collision_count"] for row in rows]),
    }


def default_rows() -> list[AblationRow]:
    return [
        AblationRow(
            name="baseline_d23_sh_8t_leaf8_14_box200_d28",
            factor="baseline_tradeoff",
            description="Fast baseline: d23 external LECT replay, SupportHull, 8 threads, leaf8->14, 200 refine boxes.",
            deep_max_boxes=200,
            deep_ffb_depth=28,
            use_external_evidence=True,
        ),
        AblationRow(
            name="baseline_d23_sh_8t_leaf8_14_box400_d28",
            factor="baseline_tradeoff",
            description="Previous baseline trade-off point with 400 refine boxes.",
            deep_max_boxes=400,
            deep_ffb_depth=28,
            use_external_evidence=True,
        ),
        AblationRow(
            name="baseline_d23_sh_8t_leaf8_14_box500_d24",
            factor="baseline_tradeoff",
            description="Zero-segment quality point with 500 refine boxes and d24 refinement.",
            deep_max_boxes=500,
            deep_ffb_depth=24,
            use_external_evidence=True,
        ),
        AblationRow(
            name="nocache_sh_8t_leaf8_14_box200_d28",
            factor="cache",
            description="Disable d23 external replay and active endpoint evidence cache.",
            deep_max_boxes=200,
            deep_ffb_depth=28,
        ),
        AblationRow(
            name="nocache_aabb_8t_leaf8_14_box200_d28",
            factor="envelope",
            description="No-cache AABB/LinkIAABB envelope comparison.",
            deep_max_boxes=200,
            deep_ffb_depth=28,
            envelope="link",
        ),
        AblationRow(
            name="nocache_sh_1t_leaf8_14_box200_d28",
            factor="threads",
            description="No-cache single-thread runtime and serial leaf validation.",
            deep_max_boxes=200,
            deep_ffb_depth=28,
            threads=1,
            leaf_threads=1,
        ),
        AblationRow(
            name="nocache_sh_8t_no_anchor_roots_box200_d28",
            factor="anchor_roots",
            description="No-cache row disabling unrestricted priority-anchor roots.",
            deep_max_boxes=200,
            deep_ffb_depth=28,
            allow_anchor_roots=False,
        ),
        AblationRow(
            name="nocache_sh_8t_serial_leaf_box200_d28",
            factor="leaf_parallelism",
            description="No-cache row with serial virtual leaf validation.",
            deep_max_boxes=200,
            deep_ffb_depth=28,
            leaf_threads=1,
            parallel_virtual_validation=False,
        ),
    ]


def selected_rows(args: argparse.Namespace) -> list[AblationRow]:
    rows = default_rows()
    wanted = {item.strip() for item in str(args.only).split(",") if item.strip()}
    if wanted and "all" not in wanted:
        rows = [row for row in rows if row.name in wanted or row.factor in wanted]
    return rows


def write_outputs(args: argparse.Namespace, payload: dict[str, Any]) -> None:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.out_dir / "leaf_refine_ablation_summary.json", payload)
    csv_fields = [
        "name", "factor", "ok_count", "n", "planning_ms_median", "leaf_sweep_ms_median",
        "deep_refine_ms_median", "connector_ms_median", "route_length_median",
        "segment_fraction_median", "segment_fraction_max",
        "target3_segment_fraction_median", "target3_segment_fraction_max",
        "final_boxes_median", "deep_boxes_added_median", "deep_anchor_roots_median",
        "leaf_free_count_median", "leaf_collision_count_median",
    ]
    with (args.out_dir / "leaf_refine_ablation_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_fields)
        writer.writeheader()
        for case in payload["cases"]:
            agg = case["aggregate"]
            writer.writerow({
                "name": case["name"],
                "factor": case["factor"],
                **{key: agg.get(key) for key in csv_fields if key not in {"name", "factor"}},
            })

    lines = [
        "# Exp04 Leaf-Refine Shelf Ablation",
        "",
        "| row | factor | ok | time ms | leaf ms | refine ms | conn ms | length | seg med | seg max | target3 max | boxes | roots |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for case in payload["cases"]:
        agg = case["aggregate"]
        lines.append(
            f"| {case['name']} | {case['factor']} | {agg.get('ok_count', 0)}/{agg.get('n', 0)} | "
            f"{fmt(agg.get('planning_ms_median'), 1)} | {fmt(agg.get('leaf_sweep_ms_median'), 1)} | "
            f"{fmt(agg.get('deep_refine_ms_median'), 1)} | {fmt(agg.get('connector_ms_median'), 1)} | "
            f"{fmt(agg.get('route_length_median'), 3)} | {fmt(agg.get('segment_fraction_median'), 3)} | "
            f"{fmt(agg.get('segment_fraction_max'), 3)} | {fmt(agg.get('target3_segment_fraction_max'), 3)} | "
            f"{fmt(agg.get('final_boxes_median'), 0)} | {fmt(agg.get('deep_anchor_roots_median'), 0)} |"
        )
    (args.out_dir / "leaf_refine_ablation_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    write_plots(args, payload["cases"])


def write_plots(args: argparse.Namespace, cases: list[dict[str, Any]]) -> None:
    try:
        os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "matplotlib-codex"))
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        write_svg_plot(args, cases)
        return
    labels = [case["name"].replace("_", "\n") for case in cases]
    x = list(range(len(cases)))

    def value(case: dict[str, Any], key: str) -> float:
        raw = case["aggregate"].get(key)
        return float(raw) if raw is not None else float("nan")

    fig, axes = plt.subplots(3, 1, figsize=(max(9, len(cases) * 1.0), 10), sharex=True)
    axes[0].bar(x, [value(case, "planning_ms_median") for case in cases])
    axes[0].set_ylabel("planning ms")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(x, [value(case, "segment_fraction_median") for case in cases], label="overall")
    axes[1].plot(x, [value(case, "target3_segment_fraction_median") for case in cases], marker="o", color="tab:red", label="target3")
    axes[1].set_ylabel("segment fraction")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, axis="y", alpha=0.3)
    axes[2].bar(x, [value(case, "route_length_median") for case in cases])
    axes[2].set_ylabel("route length")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=70, ha="right", fontsize=7)
    axes[2].grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out_dir / "leaf_refine_ablation_curves.png", dpi=180)


def write_svg_plot(args: argparse.Namespace, cases: list[dict[str, Any]]) -> None:
    width = max(960, 120 * len(cases))
    height = 780
    margin_left = 90
    margin_right = 30
    panel_height = 190
    gap = 45
    labels = [case["name"] for case in cases]

    def value(case: dict[str, Any], key: str) -> float:
        raw = case["aggregate"].get(key)
        return float(raw) if raw is not None and math.isfinite(float(raw)) else float("nan")

    def series(key: str) -> list[float]:
        return [value(case, key) for case in cases]

    def max_series(keys: list[str], floor: float = 1.0) -> float:
        vals: list[float] = []
        for key in keys:
            vals.extend(v for v in series(key) if math.isfinite(v))
        return max(vals + [floor])

    inner_width = width - margin_left - margin_right
    slot = inner_width / max(1, len(cases))
    bar_width = max(12.0, slot * 0.58)
    svg: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Arial,sans-serif;font-size:12px;fill:#222}.axis{stroke:#555;stroke-width:1}.grid{stroke:#ddd;stroke-width:1}.bar{fill:#477db3}.line{fill:none;stroke:#c43;stroke-width:2}.point{fill:#c43}</style>',
        '<text x="20" y="28" font-size="18">Exp04 leaf-refine ablation trade-off</text>',
    ]

    panels = [
        ("planning_ms_median", [], "planning ms", 50),
        ("segment_fraction_median", ["target3_segment_fraction_median"], "segment fraction", 50 + panel_height + gap),
        ("route_length_median", [], "route length", 50 + 2 * (panel_height + gap)),
    ]
    for bar_key, line_keys, title, top in panels:
        scale_max = max_series([bar_key] + line_keys, 1.0)
        svg.append(f'<text x="20" y="{top + 15}" font-size="14">{title}</text>')
        for tick in range(0, 5):
            y = top + panel_height - tick * panel_height / 4.0
            value_tick = scale_max * tick / 4.0
            svg.append(f'<line class="grid" x1="{margin_left}" y1="{y:.1f}" x2="{width - margin_right}" y2="{y:.1f}"/>')
            svg.append(f'<text x="12" y="{y + 4:.1f}">{value_tick:.3g}</text>')
        svg.append(f'<line class="axis" x1="{margin_left}" y1="{top + panel_height}" x2="{width - margin_right}" y2="{top + panel_height}"/>')
        for index, case in enumerate(cases):
            raw = value(case, bar_key)
            if not math.isfinite(raw):
                continue
            x_center = margin_left + slot * (index + 0.5)
            bar_h = panel_height * raw / scale_max if scale_max > 0 else 0.0
            y = top + panel_height - bar_h
            svg.append(f'<rect class="bar" x="{x_center - bar_width / 2:.1f}" y="{y:.1f}" width="{bar_width:.1f}" height="{bar_h:.1f}"/>')
        for line_key in line_keys:
            points = []
            for index, case in enumerate(cases):
                raw = value(case, line_key)
                if not math.isfinite(raw):
                    continue
                x_center = margin_left + slot * (index + 0.5)
                y = top + panel_height - panel_height * raw / scale_max if scale_max > 0 else top + panel_height
                points.append((x_center, y))
            if points:
                path = " ".join(("M" if i == 0 else "L") + f"{x:.1f},{y:.1f}" for i, (x, y) in enumerate(points))
                svg.append(f'<path class="line" d="{path}"/>')
                for x, y in points:
                    svg.append(f'<circle class="point" cx="{x:.1f}" cy="{y:.1f}" r="3"/>')

    label_top = 50 + 3 * (panel_height + gap) - 15
    for index, label in enumerate(labels):
        x_center = margin_left + slot * (index + 0.5)
        short = label.replace("baseline_d23_", "base_").replace("nocache_", "nc_")
        svg.append(f'<text transform="translate({x_center:.1f},{label_top}) rotate(65)" text-anchor="start">{short}</text>')
    svg.append("</svg>")
    (args.out_dir / "leaf_refine_ablation_curves.svg").write_text("\n".join(svg), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Exp04 leaf-refine shelf ablation matrix.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--seeds-list", default="0,1,2,3,4,5,6,7")
    parser.add_argument("--only", default="all", help="Comma-separated row names/factors or all.")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--connector-pave-depth", type=int, default=28)
    parser.add_argument("--domain-seed-cap", type=int, default=24)
    parser.add_argument("--domain-success-cap", type=int, default=8)
    parser.add_argument("--domain-attempt-cap", type=int, default=24)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--rbf-ffb-start-depth", type=int, default=15)
    parser.add_argument("--rbf-cache-root", type=Path, default=profile.D23_CACHE_ROOT)
    parser.add_argument("--warm-cache-label", default=profile.D23_CACHE_LABEL)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    seeds = parse_int_list(args.seeds_list)
    rows = selected_rows(args)
    cases: list[dict[str, Any]] = []
    for row in rows:
        print(f"[leaf-ablation] row={row.name}", flush=True)
        seed_rows: list[dict[str, Any]] = []
        if not args.dry_run:
            seed_rows = [run_one(args, row, seed) for seed in seeds]
        cases.append({
            "name": row.name,
            "factor": row.factor,
            "description": row.description,
            "config": row.__dict__,
            "seeds": seed_rows,
            "aggregate": aggregate(seed_rows) if seed_rows else {"n": 0, "ok_count": 0},
        })
    payload = {
        "experiment": "exp04_leaf_refine_shelf_ablation",
        "environment": environment_metadata(),
        "seeds": seeds,
        "cache_policy": {
            "baseline": "d23 external LECT snapshot replay",
            "non_baseline": "no external evidence and no active endpoint evidence cache",
            "cache_root": str(args.rbf_cache_root),
            "warm_cache_label": str(args.warm_cache_label),
        },
        "cases": cases,
    }
    write_outputs(args, payload)
    print(json.dumps({
        "out_dir": str(args.out_dir),
        "cases": len(cases),
        "summary": str(args.out_dir / "leaf_refine_ablation_summary.md"),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
