#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.metrics import mean, median
from experiments.common.progress import progress
from experiments.exp04_shelf_leaf_rrt import run_shelf_leaf_rrt as exp04


def csv_ints(raw: str) -> list[int]:
    return [int(item.strip()) for item in str(raw).split(",") if item.strip()]


def csv_floats(raw: str) -> list[float]:
    return [float(item.strip()) for item in str(raw).split(",") if item.strip()]


def fmt_float(value: float) -> str:
    text = f"{float(value):g}"
    return text.replace(".", "p").replace("-", "m")


def base_exp04_args(out_dir: Path, threads: int) -> argparse.Namespace:
    old_argv = sys.argv[:]
    try:
        sys.argv = [old_argv[0]]
        args = exp04.parse_args()
    finally:
        sys.argv = old_argv
    args.out_dir = out_dir
    args.threads = int(threads)
    args.only = "baseline_d23_aafk_support_hull_8t"
    args.active_cache_tag = ""
    return args


def explicit_configs(args: argparse.Namespace) -> list[dict[str, Any]]:
    raw = str(args.configs).strip()
    if not raw:
        return []
    configs: list[dict[str, Any]] = []
    for item in raw.split(";"):
        item = item.strip()
        if not item:
            continue
        cfg: dict[str, Any] = {}
        for token in item.split(","):
            if not token.strip():
                continue
            key, value = token.split("=", 1)
            key = key.strip().replace("-", "_")
            value = value.strip()
            aliases = {
                "target_k": "endpoint_main_target_k",
                "coarse_step": "endpoint_main_coarse_step",
                "fine_step": "endpoint_main_fine_step",
                "max_ffb_calls": "endpoint_main_max_ffb_calls",
                "max_boxes": "endpoint_main_max_boxes",
                "residual": "endpoint_main_residual_segment_max_length",
                "lateral_offset": "endpoint_main_lateral_offset",
                "lateral_rounds": "endpoint_main_lateral_rounds",
                "face_epsilon": "endpoint_main_face_epsilon",
                "depths": "endpoint_main_adaptive_ffb_depths",
            }
            key = aliases.get(key, key)
            if key in {
                "leaf_max",
                "anchors",
                "candidates",
                "boxes",
                "bridge_boxes",
                "max_pairs",
                "leaf_start",
                "ffb_depth",
                "offline_shortcut_edges",
                "shortcut_edges",
                "shortcut_candidate_limit",
            }:
                cfg[key] = int(value)
            elif key in {"pair_timeout_ms", "refine_timeout_ms"}:
                cfg[key] = float(value)
            elif key in {
                "accept_segment_fraction",
                "accept_path_ratio",
                "accept_path_additive",
                "adaptive_max_path_length",
                "shortcut_min_gain_ratio",
                "shortcut_max_segment_length",
                "to_main_direct",
                "query_bridge_to_main_direct_segment_max_length",
                "endpoint_main_coarse_step",
                "endpoint_main_fine_step",
                "endpoint_main_residual_segment_max_length",
                "endpoint_main_lateral_offset",
                "endpoint_main_face_epsilon",
            }:
                if key == "to_main_direct":
                    cfg["query_bridge_to_main_direct_segment_max_length"] = float(value)
                else:
                    cfg[key] = float(value)
            elif key in {
                "endpoint_main_target_k",
                "endpoint_main_max_ffb_calls",
                "endpoint_main_max_boxes",
                "endpoint_main_lateral_rounds",
            }:
                cfg[key] = int(value)
            elif key in {"endpoint_anchor", "to_main_box", "query_bridge_to_main_box_corridor"}:
                if key == "to_main_box":
                    cfg["query_bridge_to_main_box_corridor"] = value.lower() not in {"0", "false", "no", "off"}
                else:
                    cfg[key] = value.lower() not in {"0", "false", "no", "off"}
            elif key in {"endpoint_main_adaptive_ffb_depths", "endpoint_main_depths"}:
                cfg["endpoint_main_adaptive_ffb_depths"] = value
            elif key in {"to_main", "query_bridge_to_main_island"}:
                cfg["query_bridge_to_main_island"] = value.lower() not in {"0", "false", "no", "off"}
            else:
                cfg[key] = value
        configs.append(cfg)
    return configs


def grid_configs(args: argparse.Namespace) -> list[dict[str, Any]]:
    configs = explicit_configs(args)
    if configs:
        return configs
    out: list[dict[str, Any]] = []
    for leaf_max in csv_ints(args.leaf_max_depths):
        for anchors in csv_ints(args.offline_anchor_counts):
            for candidates in csv_ints(args.offline_anchor_candidate_counts):
                for boxes in csv_ints(args.box_budgets):
                    for bridge_boxes in csv_ints(args.connector_bridge_boxes):
                        for pair_timeout_ms in csv_floats(args.connector_pair_timeouts_ms):
                            out.append({
                                "leaf_start": int(args.leaf_start_depth),
                                "leaf_max": leaf_max,
                                "anchors": anchors,
                                "candidates": candidates,
                                "boxes": boxes,
                                "bridge_boxes": bridge_boxes,
                                "pair_timeout_ms": pair_timeout_ms,
                                "max_pairs": int(args.connector_max_pairs_per_gap),
                                "ffb_depth": int(args.deep_ffb_depth),
                                "refine_timeout_ms": float(args.refine_timeout_ms),
                            })
    return out


def config_id(cfg: dict[str, Any]) -> str:
    base = (
        f"l{int(cfg.get('leaf_start', 8))}_{int(cfg['leaf_max'])}"
        f"_a{int(cfg['anchors'])}_c{int(cfg['candidates'])}"
        f"_b{int(cfg['boxes'])}_bb{int(cfg['bridge_boxes'])}"
        f"_p{fmt_float(float(cfg['pair_timeout_ms']))}"
    )
    if "accept_segment_fraction" in cfg:
        base += f"_sf{fmt_float(float(cfg['accept_segment_fraction']))}"
    sampling = str(cfg.get("sampling", cfg.get("anchor_sampling", ""))).strip().lower()
    if sampling:
        base += f"_{sampling}"
    shortcut_edges = int(cfg.get("offline_shortcut_edges", cfg.get("shortcut_edges", 0)))
    if shortcut_edges > 0:
        base += f"_os{shortcut_edges}"
        if "shortcut_candidate_limit" in cfg:
            base += f"_ocl{int(cfg['shortcut_candidate_limit'])}"
        if "shortcut_min_gain_ratio" in cfg:
            base += f"_og{fmt_float(float(cfg['shortcut_min_gain_ratio']))}"
    if bool(cfg.get("query_bridge_to_main_island", False)):
        base += "_main"
        if bool(cfg.get("query_bridge_to_main_box_corridor", True)):
            base += "_box"
        direct = float(cfg.get("query_bridge_to_main_direct_segment_max_length", 0.0))
        if direct > 0.0:
            base += f"_md{fmt_float(direct)}"
        if "endpoint_main_max_ffb_calls" in cfg:
            base += f"_emc{int(cfg['endpoint_main_max_ffb_calls'])}"
        if "endpoint_main_max_boxes" in cfg:
            base += f"_emb{int(cfg['endpoint_main_max_boxes'])}"
        if "endpoint_main_adaptive_ffb_depths" in cfg:
            depths = str(cfg["endpoint_main_adaptive_ffb_depths"]).replace(",", "d")
            base += f"_emd{depths}"
    return base


def apply_config(base: argparse.Namespace, cfg: dict[str, Any], out_dir: Path, seed: int) -> argparse.Namespace:
    args = argparse.Namespace(**vars(base))
    args.out_dir = out_dir
    args.leaf_start_depth = int(cfg.get("leaf_start", args.leaf_start_depth))
    args.leaf_max_depth = int(cfg["leaf_max"])
    args.offline_anchor_count = int(cfg["anchors"])
    args.offline_anchor_candidate_count = int(cfg["candidates"])
    args.connector_bridge_boxes = int(cfg["bridge_boxes"])
    args.connector_pair_timeout_ms = float(cfg["pair_timeout_ms"])
    args.connector_max_pairs_per_gap = int(cfg.get("max_pairs", args.connector_max_pairs_per_gap))
    args.deep_ffb_depth = int(cfg.get("ffb_depth", args.deep_ffb_depth))
    args.connector_pave_depth = int(cfg.get("ffb_depth", args.connector_pave_depth))
    args.query_bridge_pave_depth = int(cfg.get("ffb_depth", args.query_bridge_pave_depth))
    args.refine_timeout_ms = float(cfg.get("refine_timeout_ms", args.refine_timeout_ms))
    if "accept_segment_fraction" in cfg:
        args.query_bridge_accept_segment_fraction = float(cfg["accept_segment_fraction"])
    if "accept_path_ratio" in cfg:
        args.query_bridge_accept_path_ratio = float(cfg["accept_path_ratio"])
    if "accept_path_additive" in cfg:
        args.query_bridge_accept_path_additive = float(cfg["accept_path_additive"])
    if "adaptive_max_path_length" in cfg:
        args.query_bridge_adaptive_max_path_length = float(cfg["adaptive_max_path_length"])
    if "endpoint_anchor" in cfg:
        args.query_endpoint_anchor_before_bridge = bool(cfg["endpoint_anchor"])
    if "query_bridge_to_main_island" in cfg:
        args.query_bridge_to_main_island = bool(cfg["query_bridge_to_main_island"])
    if "query_bridge_to_main_direct_segment_max_length" in cfg:
        args.query_bridge_to_main_direct_segment_max_length = float(
            cfg["query_bridge_to_main_direct_segment_max_length"]
        )
    if "query_bridge_to_main_box_corridor" in cfg:
        args.query_bridge_to_main_box_corridor = bool(cfg["query_bridge_to_main_box_corridor"])
    if "endpoint_main_target_k" in cfg:
        args.endpoint_main_target_k = int(cfg["endpoint_main_target_k"])
    if "endpoint_main_coarse_step" in cfg:
        args.endpoint_main_coarse_step = float(cfg["endpoint_main_coarse_step"])
    if "endpoint_main_fine_step" in cfg:
        args.endpoint_main_fine_step = float(cfg["endpoint_main_fine_step"])
    if "endpoint_main_max_ffb_calls" in cfg:
        args.endpoint_main_max_ffb_calls = int(cfg["endpoint_main_max_ffb_calls"])
    if "endpoint_main_max_boxes" in cfg:
        args.endpoint_main_max_boxes = int(cfg["endpoint_main_max_boxes"])
    if "endpoint_main_adaptive_ffb_depths" in cfg:
        args.endpoint_main_adaptive_ffb_depths = str(cfg["endpoint_main_adaptive_ffb_depths"])
    if "endpoint_main_residual_segment_max_length" in cfg:
        args.endpoint_main_residual_segment_max_length = float(
            cfg["endpoint_main_residual_segment_max_length"]
        )
    if "endpoint_main_lateral_offset" in cfg:
        args.endpoint_main_lateral_offset = float(cfg["endpoint_main_lateral_offset"])
    if "endpoint_main_lateral_rounds" in cfg:
        args.endpoint_main_lateral_rounds = int(cfg["endpoint_main_lateral_rounds"])
    if "endpoint_main_face_epsilon" in cfg:
        args.endpoint_main_face_epsilon = float(cfg["endpoint_main_face_epsilon"])
    if "offline_shortcut_edges" in cfg or "shortcut_edges" in cfg:
        args.offline_shortcut_edges = int(cfg.get("offline_shortcut_edges", cfg.get("shortcut_edges", 0)))
    if "sampling" in cfg or "anchor_sampling" in cfg:
        args.offline_anchor_sampling = str(cfg.get("sampling", cfg.get("anchor_sampling", args.offline_anchor_sampling)))
    if "shortcut_candidate_limit" in cfg:
        args.offline_shortcut_candidate_limit = int(cfg["shortcut_candidate_limit"])
    if "shortcut_min_gain_ratio" in cfg:
        args.offline_shortcut_min_gain_ratio = float(cfg["shortcut_min_gain_ratio"])
    if "shortcut_max_segment_length" in cfg:
        args.offline_shortcut_max_segment_length = float(cfg["shortcut_max_segment_length"])
    args.active_cache_tag = f"{config_id(cfg)}_s{seed}"
    return args


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    keys = sorted({str(row["config_id"]) for row in rows})
    for key in keys:
        items = [row for row in rows if str(row["config_id"]) == key]
        success_items = [row for row in items if int(row.get("success_count", 0)) == int(row.get("query_count", 1))]
        total_queries = sum(int(row.get("query_count", 0)) for row in items)
        success_queries = sum(int(row.get("success_count", 0)) for row in items)
        out.append({
            "config_id": key,
            "runs": len(items),
            "success_runs": len(success_items),
            "success_queries": success_queries,
            "total_queries": total_queries,
            "leaf_start_depth": median(row.get("leaf_start_depth", math.nan) for row in items),
            "leaf_max_depth": median(row.get("leaf_max_depth", math.nan) for row in items),
            "offline_anchor_count": median(row.get("offline_anchor_count", math.nan) for row in items),
            "offline_anchor_candidate_count": median(row.get("offline_anchor_candidate_count", math.nan) for row in items),
            "offline_anchor_sampling": str(items[0].get("offline_anchor_sampling", "")),
            "deep_max_boxes": median(row.get("deep_max_boxes", math.nan) for row in items),
            "connector_bridge_boxes": median(row.get("connector_bridge_boxes", math.nan) for row in items),
            "connector_pair_timeout_ms": median(row.get("connector_pair_timeout_ms", math.nan) for row in items),
            "query_bridge_accept_segment_fraction": median(row.get("query_bridge_accept_segment_fraction", math.nan) for row in items),
            "query_bridge_accept_path_ratio": median(row.get("query_bridge_accept_path_ratio", math.nan) for row in items),
            "query_bridge_accept_path_additive": median(row.get("query_bridge_accept_path_additive", math.nan) for row in items),
            "query_bridge_to_main_island": bool(items[0].get("query_bridge_to_main_island", False)),
            "query_bridge_to_main_box_corridor": bool(items[0].get("query_bridge_to_main_box_corridor", True)),
            "query_bridge_to_main_direct_segment_max_length": median(row.get("query_bridge_to_main_direct_segment_max_length", math.nan) for row in items),
            "endpoint_main_target_k": median(row.get("endpoint_main_target_k", math.nan) for row in items),
            "endpoint_main_coarse_step": median(row.get("endpoint_main_coarse_step", math.nan) for row in items),
            "endpoint_main_fine_step": median(row.get("endpoint_main_fine_step", math.nan) for row in items),
            "endpoint_main_max_ffb_calls": median(row.get("endpoint_main_max_ffb_calls", math.nan) for row in items),
            "endpoint_main_max_boxes": median(row.get("endpoint_main_max_boxes", math.nan) for row in items),
            "endpoint_main_adaptive_ffb_depths": str(items[0].get("endpoint_main_adaptive_ffb_depths", "")),
            "endpoint_main_residual_segment_max_length": median(row.get("endpoint_main_residual_segment_max_length", math.nan) for row in items),
            "offline_build_s_median": median(row.get("offline_build_s", math.nan) for row in items),
            "leaf_sweep_s_median": median(row.get("leaf_sweep_s", math.nan) for row in items),
            "deep_refine_s_median": median(row.get("deep_refine_s", math.nan) for row in items),
            "connector_s_median": median(row.get("connector_s", math.nan) for row in items),
            "query_bridge_s_median": median(row.get("query_bridge_s", math.nan) for row in items),
            "query_bridge_per_query_s_median": median(row.get("query_bridge_per_query_s", row.get("query_bridge_s", math.nan) / max(1, row.get("query_count", 1))) for row in items),
            "endpoint_main_s_median": median(row.get("endpoint_main_s", 0.0) for row in items),
            "endpoint_main_per_query_s_median": median(row.get("endpoint_main_per_query_s", row.get("endpoint_main_s", 0.0) / max(1, row.get("query_count", 1))) for row in items),
            "endpoint_main_success_count_median": median(row.get("endpoint_main_success_count", 0) for row in items),
            "endpoint_main_fallback_to_e2e_median": median(row.get("endpoint_main_fallback_to_e2e", 0) for row in items),
            "online_solve_per_query_s_median": median(row.get("online_solve_per_query_s", math.nan) for row in items),
            "online_simplify_per_query_s_median": median(row.get("online_simplify_per_query_s", math.nan) for row in items),
            "online_per_query_s_median": median(row.get("online_per_query_s", row.get("online_solve_per_query_s", math.nan)) for row in items),
            "online_total_per_query_s_median": median(row.get("online_total_per_query_s", row.get("online_per_query_s", math.nan)) for row in items),
            "amortized_s_k5": median(row.get("amortized_s_k5", math.nan) for row in items),
            "amortized_s_k10": median(row.get("amortized_s_k10", math.nan) for row in items),
            "amortized_s_k20": median(row.get("amortized_s_k20", math.nan) for row in items),
            "path_length_mean": mean(row.get("path_length_mean", math.nan) for row in success_items),
            "raw_segment_fraction_median": median(row.get("raw_segment_fraction", math.nan) for row in success_items),
            "leaf_free_count_median": median(row.get("leaf_free_count", math.nan) for row in items),
            "leaf_collision_count_median": median(row.get("leaf_collision_count", math.nan) for row in items),
            "offline_anchor_roots_added_median": median(row.get("offline_anchor_roots_added", math.nan) for row in items),
            "offline_segment_edges_added_median": median(row.get("offline_segment_edges_added", math.nan) for row in items),
            "offline_shortcut_edges_requested_median": median(row.get("offline_shortcut_edges_requested", math.nan) for row in items),
            "offline_shortcut_edges_added_median": median(row.get("offline_shortcut_edges_added", math.nan) for row in items),
            "offline_shortcut_box_corridor_edges_added_median": median(row.get("offline_shortcut_box_corridor_edges_added", math.nan) for row in items),
            "offline_shortcut_segment_edges_added_median": median(row.get("offline_shortcut_segment_edges_added", math.nan) for row in items),
            "offline_shortcut_pave_boxes_added_median": median(row.get("offline_shortcut_pave_boxes_added", math.nan) for row in items),
            "offline_shortcut_pave_fail_median": median(row.get("offline_shortcut_pave_fail", math.nan) for row in items),
            "final_boxes_median": median(row.get("final_boxes", math.nan) for row in items),
            "final_segment_edges_median": median(row.get("final_segment_edges", math.nan) for row in items),
            "final_adjacency_islands_median": median(row.get("final_adjacency_islands", math.nan) for row in items),
        })
    return out


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_per_query(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "config_id",
        "seed",
        "label",
        "success",
        "audit_passed",
        "query_ms",
        "solve_ms",
        "simplify_ms",
        "path_length",
        "raw_path_length",
        "segment_fraction",
        "segment_edges_used",
        "box_sequence_len",
        "audit_status",
    ]
    out: list[dict[str, Any]] = []
    for row in rows:
        for query in row.get("queries", []):
            item = {field: query.get(field) for field in fields}
            item["config_id"] = row.get("config_id")
            item["seed"] = row.get("seed")
            out.append(item)
    write_csv(path, out, fields)


def write_summary_md(path: Path, rows: list[dict[str, Any]]) -> None:
    ordered = sorted(
        rows,
        key=lambda row: (
            int(row.get("success_queries", 0)) < int(row.get("total_queries", 1)),
            float(row.get("online_per_query_s_median", math.inf)),
            float(row.get("offline_build_s_median", math.inf)),
        ),
    )
    lines = [
        "# Exp4 Offline/Online RBF Trade-off",
        "",
        "Main columns use the two-stage口径: offline build is query-agnostic; online/q includes online repair, graph search, and 10 ms fixed OMPL simplify, excluding final audit.",
        "",
        "| config | SR | build | online/q | qbridge/q | emain/q | solve/q | simplify/q | amort@10 | path | seg | boxes |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in ordered:
        sr = f"{int(row.get('success_queries', 0))}/{int(row.get('total_queries', 0))}"
        lines.append(
            f"| {row.get('config_id')} | {sr} | "
            f"{float(row.get('offline_build_s_median', math.nan)):.3f} | "
            f"{float(row.get('online_per_query_s_median', math.nan)):.4f} | "
            f"{float(row.get('query_bridge_per_query_s_median', math.nan)):.4f} | "
            f"{float(row.get('endpoint_main_per_query_s_median', math.nan)):.4f} | "
            f"{float(row.get('online_solve_per_query_s_median', math.nan)):.4f} | "
            f"{float(row.get('online_simplify_per_query_s_median', math.nan)):.4f} | "
            f"{float(row.get('amortized_s_k10', math.nan)):.3f} | "
            f"{float(row.get('path_length_mean', math.nan)):.3f} | "
            f"{float(row.get('raw_segment_fraction_median', math.nan)):.3f} | "
            f"{float(row.get('final_boxes_median', math.nan)):.0f} |"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_plot(path: Path, rows: list[dict[str, Any]]) -> None:
    try:
        import os

        os.environ.setdefault("MPLCONFIGDIR", "/tmp/rbf_matplotlib")
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    full = [
        row for row in rows
        if int(row.get("success_queries", 0)) == int(row.get("total_queries", 1))
    ]
    if not full:
        return
    x = [float(row.get("offline_build_s_median", math.nan)) for row in full]
    online = [float(row.get("online_per_query_s_median", math.nan)) for row in full]
    path_len = [float(row.get("path_length_mean", math.nan)) for row in full]
    labels = [str(row.get("config_id")) for row in full]
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.2))
    axes[0].scatter(x, online, s=28)
    axes[0].set_xlabel("Offline build (s)")
    axes[0].set_ylabel("Online/query (s)")
    axes[0].grid(True, alpha=0.25)
    axes[1].scatter(x, path_len, s=28)
    axes[1].set_xlabel("Offline build (s)")
    axes[1].set_ylabel("Mean path length")
    axes[1].grid(True, alpha=0.25)
    for axis, y_values in zip(axes, [online, path_len]):
        for bx, by, label in zip(x, y_values, labels):
            axis.annotate(label, (bx, by), fontsize=5, alpha=0.75)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan query-agnostic offline RBF coverage against online shelf query performance.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp04_offline_online_tradeoff")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--leaf-start-depth", type=int, default=8)
    parser.add_argument("--leaf-max-depths", default="14,15,16")
    parser.add_argument("--offline-anchor-counts", default="16,32")
    parser.add_argument("--offline-anchor-candidate-counts", default="512,1024")
    parser.add_argument("--box-budgets", default="400,800")
    parser.add_argument("--connector-bridge-boxes", default="200,400")
    parser.add_argument("--connector-pair-timeouts-ms", default="30")
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=2)
    parser.add_argument("--deep-ffb-depth", type=int, default=62)
    parser.add_argument("--refine-timeout-ms", type=float, default=1200.0)
    parser.add_argument(
        "--configs",
        default="",
        help=(
            "Optional semicolon-separated explicit configs, e.g. "
            "'leaf_max=14,anchors=16,candidates=512,boxes=400,bridge_boxes=200,pair_timeout_ms=30;...'."
        ),
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    exp04.configure_thread_environment(int(args.threads))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    seeds = csv_ints(args.seeds)
    configs = grid_configs(args)
    base = base_exp04_args(args.out_dir, int(args.threads))
    planned = [
        {"config_id": config_id(cfg), "seed": seed, **cfg}
        for cfg in configs
        for seed in seeds
    ]
    if args.dry_run:
        write_json(args.out_dir / "offline_online_tradeoff_manifest.json", {
            "run_id": run_id("exp04_offline_online_tradeoff"),
            "status": "dry_run",
            "planned": planned,
            "environment": environment_metadata(),
        })
        print(f"planned {len(planned)} runs")
        return 0

    rows: list[dict[str, Any]] = []
    for item in progress(planned, desc="exp04 offline/online", total=len(planned)):
        cfg = {key: item[key] for key in item if key not in {"config_id", "seed"}}
        seed = int(item["seed"])
        run_args = apply_config(base, cfg, args.out_dir, seed)
        row = exp04.run_case("baseline_d23_aafk_support_hull_8t", seed, int(cfg["boxes"]), run_args)
        row.update({
            "config_id": item["config_id"],
            "leaf_start_depth": int(cfg.get("leaf_start", run_args.leaf_start_depth)),
            "leaf_max_depth": int(cfg["leaf_max"]),
            "offline_anchor_count": int(cfg["anchors"]),
            "offline_anchor_candidate_count": int(cfg["candidates"]),
            "offline_anchor_sampling": str(run_args.offline_anchor_sampling),
            "connector_bridge_boxes": int(cfg["bridge_boxes"]),
            "connector_pair_timeout_ms": float(cfg["pair_timeout_ms"]),
            "query_bridge_accept_segment_fraction": float(run_args.query_bridge_accept_segment_fraction),
            "query_bridge_accept_path_ratio": float(run_args.query_bridge_accept_path_ratio),
            "query_bridge_accept_path_additive": float(run_args.query_bridge_accept_path_additive),
            "query_bridge_to_main_island": bool(run_args.query_bridge_to_main_island),
            "query_bridge_to_main_box_corridor": bool(run_args.query_bridge_to_main_box_corridor),
            "query_bridge_to_main_direct_segment_max_length": float(run_args.query_bridge_to_main_direct_segment_max_length),
            "endpoint_main_target_k": int(run_args.endpoint_main_target_k),
            "endpoint_main_coarse_step": float(run_args.endpoint_main_coarse_step),
            "endpoint_main_fine_step": float(run_args.endpoint_main_fine_step),
            "endpoint_main_max_ffb_calls": int(run_args.endpoint_main_max_ffb_calls),
            "endpoint_main_max_boxes": int(run_args.endpoint_main_max_boxes),
            "endpoint_main_adaptive_ffb_depths": str(run_args.endpoint_main_adaptive_ffb_depths),
            "endpoint_main_residual_segment_max_length": float(run_args.endpoint_main_residual_segment_max_length),
            "offline_shortcut_edges_requested": int(run_args.offline_shortcut_edges),
            "offline_shortcut_edges_added": int(row.get("offline_shortcut_edges_added", 0)),
            "offline_shortcut_box_corridor_edges_added": int(row.get("offline_shortcut_box_corridor_edges_added", 0)),
            "offline_shortcut_segment_edges_added": int(row.get("offline_shortcut_segment_edges_added", 0)),
            "offline_shortcut_pave_boxes_added": int(row.get("offline_shortcut_pave_boxes_added", 0)),
            "offline_shortcut_pave_fail": int(row.get("offline_shortcut_pave_fail", 0)),
        })
        row["query_bridge_per_query_s"] = float(row.get("query_bridge_s", 0.0)) / max(1, int(row.get("query_count", 1)))
        rows.append(row)

    summary = summarize(rows)
    run_fields = [
        "config_id", "case", "seed", "deep_max_boxes", "success_count", "query_count",
        "leaf_start_depth", "leaf_max_depth", "offline_anchor_count", "offline_anchor_candidate_count",
        "connector_bridge_boxes", "connector_pair_timeout_ms",
        "offline_anchor_sampling",
        "query_bridge_accept_segment_fraction", "query_bridge_accept_path_ratio",
        "query_bridge_accept_path_additive",
        "query_bridge_to_main_island",
        "query_bridge_to_main_box_corridor",
        "query_bridge_to_main_direct_segment_max_length",
        "endpoint_main_target_k",
        "endpoint_main_coarse_step",
        "endpoint_main_fine_step",
        "endpoint_main_max_ffb_calls",
        "endpoint_main_max_boxes",
        "endpoint_main_adaptive_ffb_depths",
        "endpoint_main_residual_segment_max_length",
        "offline_build_s", "leaf_sweep_s", "deep_refine_s", "connector_s",
        "query_bridge_s", "query_bridge_per_query_s",
        "endpoint_main_s", "endpoint_main_per_query_s",
        "endpoint_main_success_count", "endpoint_main_fallback_to_e2e",
        "online_solve_per_query_s", "online_simplify_per_query_s",
        "online_per_query_s", "online_total_per_query_s", "amortized_s_k5", "amortized_s_k10", "path_length_mean",
        "raw_segment_fraction", "leaf_free_count", "leaf_collision_count",
        "offline_anchor_roots_added", "offline_segment_edges_added",
        "offline_shortcut_edges_requested", "offline_shortcut_edges_added",
        "offline_shortcut_box_corridor_edges_added", "offline_shortcut_segment_edges_added",
        "offline_shortcut_pave_boxes_added", "offline_shortcut_pave_fail",
        "final_boxes", "final_segment_edges", "final_adjacency_islands",
    ]
    summary_fields = [
        "config_id", "runs", "success_runs", "success_queries", "total_queries",
        "leaf_start_depth", "leaf_max_depth", "offline_anchor_count",
        "offline_anchor_candidate_count", "offline_anchor_sampling", "deep_max_boxes", "connector_bridge_boxes",
        "connector_pair_timeout_ms", "query_bridge_accept_segment_fraction",
        "query_bridge_accept_path_ratio", "query_bridge_accept_path_additive",
        "query_bridge_to_main_island",
        "query_bridge_to_main_box_corridor",
        "query_bridge_to_main_direct_segment_max_length",
        "endpoint_main_target_k",
        "endpoint_main_coarse_step",
        "endpoint_main_fine_step",
        "endpoint_main_max_ffb_calls",
        "endpoint_main_max_boxes",
        "endpoint_main_adaptive_ffb_depths",
        "endpoint_main_residual_segment_max_length",
        "offline_build_s_median", "leaf_sweep_s_median",
        "deep_refine_s_median", "connector_s_median",
        "query_bridge_s_median", "query_bridge_per_query_s_median",
        "endpoint_main_s_median", "endpoint_main_per_query_s_median",
        "endpoint_main_success_count_median", "endpoint_main_fallback_to_e2e_median",
        "online_solve_per_query_s_median", "online_simplify_per_query_s_median",
        "online_per_query_s_median", "online_total_per_query_s_median", "amortized_s_k5", "amortized_s_k10", "amortized_s_k20",
        "path_length_mean", "raw_segment_fraction_median", "leaf_free_count_median",
        "leaf_collision_count_median", "offline_anchor_roots_added_median",
        "offline_segment_edges_added_median", "offline_shortcut_edges_requested_median",
        "offline_shortcut_edges_added_median", "offline_shortcut_box_corridor_edges_added_median",
        "offline_shortcut_segment_edges_added_median", "offline_shortcut_pave_boxes_added_median",
        "offline_shortcut_pave_fail_median",
        "final_boxes_median", "final_segment_edges_median", "final_adjacency_islands_median",
    ]
    write_csv(args.out_dir / "offline_online_tradeoff_runs.csv", rows, run_fields)
    write_csv(args.out_dir / "offline_online_tradeoff_summary.csv", summary, summary_fields)
    write_per_query(args.out_dir / "offline_online_tradeoff_per_query.csv", rows)
    write_summary_md(args.out_dir / "offline_online_tradeoff_summary.md", summary)
    write_plot(args.out_dir / "offline_online_tradeoff_curve.png", summary)
    write_json(args.out_dir / "offline_online_tradeoff_manifest.json", {
        "run_id": run_id("exp04_offline_online_tradeoff"),
        "status": "ok",
        "planned": planned,
        "rows": rows,
        "summary": summary,
        "environment": environment_metadata(),
    })
    print(f"wrote {args.out_dir / 'offline_online_tradeoff_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
