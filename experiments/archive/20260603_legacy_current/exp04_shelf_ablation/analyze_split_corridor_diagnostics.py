#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "new_experiments" / "exp04_split_corridor_diagnostics_20260528"
DEFAULT_BASELINE_D40 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_full_ablation_20260528" / "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json"
DEFAULT_BASELINE_D48 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_split_compare_depth48_20260528" / "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json"
DEFAULT_ROUND_ROBIN_D48 = REPO_ROOT / "outputs" / "new_experiments" / "exp04_split_compare_depth48_20260528" / "round_robin_split_policy.json"
DEFAULT_BASELINE_D48_BOOSTED = REPO_ROOT / "outputs" / "new_experiments" / "exp04_depth48_boosted_schedule_20260528" / "baseline_depth48_boosted_schedule.json"

ARTIFACT_ORDER = [
    "baseline_d40",
    "baseline_d48",
    "round_robin_d48",
    "baseline_d48_boosted",
]
STAGE_ORDER = ["seed", "fast", "balanced", "quality", "high"]
QUERY_ORDER = ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze Exp.4 split/frontier/corridor diagnostics across depth variants.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--baseline-d40", type=Path, default=DEFAULT_BASELINE_D40)
    parser.add_argument("--baseline-d48", type=Path, default=DEFAULT_BASELINE_D48)
    parser.add_argument("--round-robin-d48", type=Path, default=DEFAULT_ROUND_ROBIN_D48)
    parser.add_argument("--baseline-d48-boosted", type=Path, default=DEFAULT_BASELINE_D48_BOOSTED)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def artifact_paths(args: argparse.Namespace) -> dict[str, Path]:
    return {
        "baseline_d40": args.baseline_d40,
        "baseline_d48": args.baseline_d48,
        "round_robin_d48": args.round_robin_d48,
        "baseline_d48_boosted": args.baseline_d48_boosted,
    }


def ratio(numerator: float, denominator: float) -> float | None:
    if abs(float(denominator)) <= 1e-12:
        return None
    return float(numerator) / float(denominator)


def stage_row(payload: dict[str, Any], stage_id: str) -> dict[str, Any]:
    for item in list(payload.get("raw_stage_rows") or []):
        if str(item.get("stage_id")) == str(stage_id):
            return dict(item.get("row") or {})
    raise KeyError(stage_id)


def record_row(payload: dict[str, Any], stage_id: str) -> dict[str, Any]:
    for row in list(payload.get("records") or []):
        if str(row.get("stage_id")) == str(stage_id):
            return dict(row)
    raise KeyError(stage_id)


def split_dim_counts(diagnostics: dict[str, Any]) -> dict[str, float]:
    return {
        f"dim_{index}": float(diagnostics.get(f"oracle.split_dim.{index}", 0.0))
        for index in range(7)
    }


def split_dim_fractions(counts: dict[str, float]) -> dict[str, float]:
    total = sum(float(value) for value in counts.values())
    if total <= 1e-12:
        return {key: 0.0 for key in counts}
    return {key: float(value) / total for key, value in counts.items()}


def final_query_lengths(payload: dict[str, Any]) -> dict[str, float]:
    high = stage_row(payload, "high")
    out: dict[str, float] = {}
    for query in list(high.get("queries") or []):
        name = str(query.get("name"))
        out[name] = float(query.get("length", 0.0))
    return out


def summarize_stage(payload: dict[str, Any], stage_id: str) -> dict[str, Any]:
    row = stage_row(payload, stage_id)
    record = record_row(payload, stage_id)
    build = dict(row.get("build") or {})
    diagnostics = dict(build.get("diagnostics") or {})
    split_counts = split_dim_counts(diagnostics)
    split_fractions = split_dim_fractions(split_counts)
    component_attempts = float(diagnostics.get("grower.component_connect_attempts", 0.0))
    component_successes = float(diagnostics.get("grower.component_connect_successes", 0.0))
    covered_inserts = float(diagnostics.get("grower.frontier_covered_cache_inserts", 0.0))
    uncovered_inserts = float(diagnostics.get("grower.frontier_uncovered_cache_inserts", 0.0))
    return {
        "stage_id": stage_id,
        "cumulative_total_s": float(record.get("cumulative_total_s", 0.0)),
        "incumbent_total_length": float(record.get("incumbent_total_length", 0.0)),
        "improved_tasks": list(record.get("improved_tasks") or []),
        "planning_s": float(build.get("planning_s", 0.0)),
        "unique_box_count": int(build.get("unique_box_count", 0)),
        "prebridge_time_s": float(build.get("prebridge_time_s", 0.0)),
        "prebridge_added_boxes": int(build.get("prebridge_added_boxes", 0)),
        "prebridge_attempts": int(build.get("prebridge_attempts", 0)),
        "split_node_calls": float(diagnostics.get("profile.oracle.split_node.calls", 0.0)),
        "split_node_total_ms": float(diagnostics.get("profile.oracle.split_node.total_ms", 0.0)),
        "split_parent_aspect_ratio_max": float(diagnostics.get("oracle.split_parent_aspect_ratio_max", 0.0)),
        "split_dim_counts": split_counts,
        "split_dim_fractions": split_fractions,
        "split_dim_zero_axes": [key for key, value in split_counts.items() if abs(float(value)) <= 1e-12],
        "frontier": {
            "covered_cache_hits": float(diagnostics.get("grower.frontier_covered_cache_hits", 0.0)),
            "covered_cache_inserts": covered_inserts,
            "covered_reserved_leaf_hits": float(diagnostics.get("grower.frontier_covered_reserved_leaf_hits", 0.0)),
            "covered_hit_per_insert": ratio(float(diagnostics.get("grower.frontier_covered_cache_hits", 0.0)), covered_inserts),
            "reserved_path_misses": float(diagnostics.get("grower.frontier_reserved_path_misses", 0.0)),
            "uncovered_cache_hits": float(diagnostics.get("grower.frontier_uncovered_cache_hits", 0.0)),
            "uncovered_cache_inserts": uncovered_inserts,
            "uncovered_cache_resets": float(diagnostics.get("grower.frontier_uncovered_cache_resets", 0.0)),
            "uncovered_hit_per_insert": ratio(float(diagnostics.get("grower.frontier_uncovered_cache_hits", 0.0)), uncovered_inserts),
        },
        "component_connect": {
            "attempts": component_attempts,
            "successes": component_successes,
            "failures": float(diagnostics.get("grower.component_connect_failures", 0.0)),
            "success_rate": ratio(component_successes, component_attempts),
            "face_score_calls": float(diagnostics.get("grower.component_connect_face_score_calls", 0.0)),
            "face_score_calls_per_attempt": ratio(float(diagnostics.get("grower.component_connect_face_score_calls", 0.0)), component_attempts),
            "connected_root_pairs_max": float(diagnostics.get("grower.component_connect_connected_root_pairs_max", 0.0)),
            "staged_targets": float(diagnostics.get("grower.component_connect_staged_targets", 0.0)),
            "staged_coarse_scan_limit_sum": float(diagnostics.get("grower.component_connect_staged_coarse_scan_limit_sum", 0.0)),
        },
    }


def summarize_artifact(name: str, path: Path, payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": name,
        "path": str(path),
        "final_query_lengths": final_query_lengths(payload),
        "stages": {stage_id: summarize_stage(payload, stage_id) for stage_id in STAGE_ORDER},
    }


def comparison(summary: dict[str, Any]) -> dict[str, Any]:
    base40 = summary["artifacts"]["baseline_d40"]
    base48 = summary["artifacts"]["baseline_d48"]
    rr48 = summary["artifacts"]["round_robin_d48"]
    boosted48 = summary["artifacts"]["baseline_d48_boosted"]
    query_delta = {}
    for query_name in QUERY_ORDER:
        query_delta[query_name] = {
            "baseline48_minus_baseline40": float(base48["final_query_lengths"].get(query_name, 0.0)) - float(base40["final_query_lengths"].get(query_name, 0.0)),
            "round48_minus_baseline48": float(rr48["final_query_lengths"].get(query_name, 0.0)) - float(base48["final_query_lengths"].get(query_name, 0.0)),
            "boosted48_minus_baseline48": float(boosted48["final_query_lengths"].get(query_name, 0.0)) - float(base48["final_query_lengths"].get(query_name, 0.0)),
        }

    stage_focus: dict[str, Any] = {}
    for stage_id in ["fast", "quality", "high"]:
        b40 = base40["stages"][stage_id]
        b48 = base48["stages"][stage_id]
        rr = rr48["stages"][stage_id]
        stage_focus[stage_id] = {
            "baseline48_vs_baseline40": {
                "split_node_calls_delta": b48["split_node_calls"] - b40["split_node_calls"],
                "prebridge_added_boxes_delta": b48["prebridge_added_boxes"] - b40["prebridge_added_boxes"],
                "component_connect_success_rate_delta": (b48["component_connect"]["success_rate"] or 0.0) - (b40["component_connect"]["success_rate"] or 0.0),
                "connected_root_pairs_max_delta": b48["component_connect"]["connected_root_pairs_max"] - b40["component_connect"]["connected_root_pairs_max"],
                "staged_targets_delta": b48["component_connect"]["staged_targets"] - b40["component_connect"]["staged_targets"],
                "frontier_reserved_path_misses_delta": b48["frontier"]["reserved_path_misses"] - b40["frontier"]["reserved_path_misses"],
            },
            "round48_vs_baseline48": {
                "dim6_fraction_delta": rr["split_dim_fractions"]["dim_6"] - b48["split_dim_fractions"]["dim_6"],
                "split_parent_aspect_ratio_max_delta": rr["split_parent_aspect_ratio_max"] - b48["split_parent_aspect_ratio_max"],
                "component_connect_success_rate_delta": (rr["component_connect"]["success_rate"] or 0.0) - (b48["component_connect"]["success_rate"] or 0.0),
                "connected_root_pairs_max_delta": rr["component_connect"]["connected_root_pairs_max"] - b48["component_connect"]["connected_root_pairs_max"],
                "staged_targets_delta": rr["component_connect"]["staged_targets"] - b48["component_connect"]["staged_targets"],
                "prebridge_added_boxes_delta": rr["prebridge_added_boxes"] - b48["prebridge_added_boxes"],
            },
        }

    return {
        "final_query_length_deltas": query_delta,
        "stage_focus": stage_focus,
        "interpretation": [
            "AAFKVolumeMin at depth 48 does not lose quality because corridor refinement is missing; its prebridge corridor step is active and adds more boxes than the depth-40 baseline.",
            "The large behavioral change is in component-connect staging: baseline depth 48 loses connected root pairs and staged targets, especially relative to round-robin at the same depth.",
            "Round-robin depth 48 is the only depth-48 curve that keeps all seven split axes active; AAFKVolumeMin keeps joint axis dim_6 at exactly zero splits across all observed stages.",
        ],
    }


def main() -> int:
    args = parse_args()
    paths = artifact_paths(args)
    payloads = {name: load_json(path) for name, path in paths.items()}
    summary = {
        "artifacts": {
            name: summarize_artifact(name, path, payloads[name])
            for name, path in paths.items()
        },
    }
    summary["comparison"] = comparison(summary)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / "exp04_split_corridor_diagnostics.json"
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({"out_json": str(out_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())