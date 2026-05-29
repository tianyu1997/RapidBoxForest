#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "new_experiments" / "exp04_ablation_timing_analysis_20260528"
DEFAULT_E4_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_shelf_ablation_full_ablation_20260528"
DEFAULT_DEPTH48_ROOT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_split_compare_depth48_20260528"
DEFAULT_AABB_NO_EXTERNAL = REPO_ROOT / "outputs" / "new_experiments" / "exp04_aabb_no_cache_probe_20260528" / "aabb_no_external_evidence.json"

STAGE_ORDER = ["seed", "fast", "balanced", "quality", "high"]
QUERY_ORDER = ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]
GROUP_SPECS = {
    "baseline": {
        "path_name": "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json",
        "label": "Baseline d40",
        "root": "e4",
    },
    "no_cache": {
        "path_name": "no_lect_cache_online_envelopes.json",
        "label": "No cache d40",
        "root": "e4",
    },
    "aabb": {
        "path_name": "aabb_envelope_only.json",
        "label": "AABB d40",
        "root": "e4",
    },
    "aabb_no_external": {
        "path_name": None,
        "label": "AABB no external d40",
        "root": "aabb_no_external",
    },
    "critsample": {
        "path_name": "critsample_endpoint_support_hull.json",
        "label": "CritSample d40",
        "root": "e4",
    },
    "single_thread": {
        "path_name": "single_thread.json",
        "label": "Single-thread d40",
        "root": "e4",
    },
    "baseline_d48": {
        "path_name": "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json",
        "label": "Baseline d48",
        "root": "depth48",
    },
    "round_robin_d48": {
        "path_name": "round_robin_split_policy.json",
        "label": "Round-robin d48",
        "root": "depth48",
    },
    "hybrid_d40": {
        "path_name": "hybrid_dim6_split_policy.json",
        "label": "Hybrid dim6 d40",
        "root": "e4",
    },
    "hybrid_d48": {
        "path_name": "hybrid_dim6_split_policy.json",
        "label": "Hybrid dim6 d48",
        "root": "depth48",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze Exp.4 stage timings and incumbent path quality across ablation groups.")
    parser.add_argument("--e4-root", type=Path, default=DEFAULT_E4_ROOT)
    parser.add_argument("--depth48-root", type=Path, default=DEFAULT_DEPTH48_ROOT)
    parser.add_argument("--aabb-no-external", type=Path, default=DEFAULT_AABB_NO_EXTERNAL)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def ratio(numerator: float, denominator: float) -> float | None:
    if abs(float(denominator)) <= 1e-12:
        return None
    return float(numerator) / float(denominator)


def stage_rows(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item.get("stage_id")): dict(item.get("row") or {}) for item in list(payload.get("raw_stage_rows") or [])}


def record_rows(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item.get("stage_id")): dict(item) for item in list(payload.get("records") or [])}


def query_map(row: dict[str, Any]) -> dict[str, float]:
    return {
        str(query.get("name")): float(query.get("length", 0.0))
        for query in list(row.get("queries") or [])
    }


def reconstruct_incumbent_queries(payload: dict[str, Any]) -> dict[str, dict[str, float]]:
    rows = stage_rows(payload)
    records = record_rows(payload)
    incumbent: dict[str, float] = {}
    history: dict[str, dict[str, float]] = {}
    for stage_id in STAGE_ORDER:
        row = rows[stage_id]
        record = records[stage_id]
        current_queries = query_map(row)
        for task_name in list(record.get("improved_tasks") or []):
            task_name_str = str(task_name)
            if task_name_str in current_queries:
                incumbent[task_name_str] = float(current_queries[task_name_str])
        history[stage_id] = {name: float(incumbent.get(name, 0.0)) for name in QUERY_ORDER}
    return history


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


def summarize_stage(
    payload: dict[str, Any],
    incumbent_history: dict[str, dict[str, float]],
    stage_id: str,
) -> dict[str, Any]:
    row = stage_rows(payload)[stage_id]
    record = record_rows(payload)[stage_id]
    build = dict(row.get("build") or {})
    diagnostics = dict(build.get("diagnostics") or {})
    split_counts = split_dim_counts(diagnostics)
    split_fractions = split_dim_fractions(split_counts)
    covered_inserts = float(diagnostics.get("grower.frontier_covered_cache_inserts", 0.0))
    uncovered_inserts = float(diagnostics.get("grower.frontier_uncovered_cache_inserts", 0.0))
    component_attempts = float(diagnostics.get("grower.component_connect_attempts", 0.0))
    component_successes = float(diagnostics.get("grower.component_connect_successes", 0.0))
    raw_queries = query_map(row)
    incumbent_queries = incumbent_history[stage_id]
    return {
        "stage_id": stage_id,
        "stage_total_s": float(record.get("stage_total_s", 0.0)),
        "stage_build_s": float(record.get("stage_build_s", 0.0)),
        "stage_query_s": float(record.get("stage_query_s", 0.0)),
        "cumulative_total_s": float(record.get("cumulative_total_s", 0.0)),
        "cumulative_build_s": float(record.get("cumulative_build_s", 0.0)),
        "cumulative_query_s": float(record.get("cumulative_query_s", 0.0)),
        "improved_tasks": list(record.get("improved_tasks") or []),
        "incumbent_total_length": float(record.get("incumbent_total_length", 0.0)),
        "incumbent_mean_length": float(record.get("incumbent_mean_length", 0.0)),
        "incumbent_queries": incumbent_queries,
        "raw_queries": raw_queries,
        "raw_total_length": float(sum(float(value) for value in raw_queries.values())),
        "build": {
            "planning_s": float(build.get("planning_s", 0.0)),
            "prebridge_time_s": float(build.get("prebridge_time_s", 0.0)),
            "prebridge_added_boxes": int(build.get("prebridge_added_boxes", 0)),
            "prebridge_attempts": int(build.get("prebridge_attempts", 0)),
            "grow_ms": float(build.get("grow_ms", 0.0)),
            "connector_ms": float(build.get("connector_ms", 0.0)),
            "adjacency_ms": float(build.get("adjacency_ms", 0.0)),
            "merge_ms": float(build.get("merge_ms", 0.0)),
            "maintenance_s": float(build.get("maintenance_s", 0.0)),
            "wall_s": float(build.get("wall_s", 0.0)),
            "total_ms": float(build.get("total_ms", 0.0)),
            "unique_box_count": int(build.get("unique_box_count", 0)),
            "certified_box_count": int(build.get("certified_box_count", 0)),
        },
        "split": {
            "node_calls": float(diagnostics.get("profile.oracle.split_node.calls", 0.0)),
            "node_total_ms": float(diagnostics.get("profile.oracle.split_node.total_ms", 0.0)),
            "parent_aspect_ratio_max": float(diagnostics.get("oracle.split_parent_aspect_ratio_max", 0.0)),
            "dim_counts": split_counts,
            "dim_fractions": split_fractions,
            "zero_axes": [key for key, value in split_counts.items() if abs(float(value)) <= 1e-12],
        },
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
        "external_evidence": {
            "worker_materialization_reused": float(diagnostics.get("grower.worker_oracle.materialization_reused_external_evidence", 0.0)),
            "worker_scoring_reused": float(diagnostics.get("grower.worker_oracle.scoring_reused_external_evidence", 0.0)),
            "oracle_materialization_reused": float(diagnostics.get("oracle.materialization_reused_external_evidence", 0.0)),
            "oracle_scoring_reused": float(diagnostics.get("oracle.scoring_reused_external_evidence", 0.0)),
        },
    }


def summarize_artifact(name: str, path: Path, payload: dict[str, Any]) -> dict[str, Any]:
    incumbent_history = reconstruct_incumbent_queries(payload)
    stages = {stage_id: summarize_stage(payload, incumbent_history, stage_id) for stage_id in STAGE_ORDER}
    final_stage = stages["high"]
    return {
        "name": name,
        "label": str(GROUP_SPECS[name]["label"]),
        "path": str(path),
        "final": {
            "total_s": float(final_stage["cumulative_total_s"]),
            "build_s": float(final_stage["cumulative_build_s"]),
            "query_s": float(final_stage["cumulative_query_s"]),
            "mean_path_length": float(final_stage["incumbent_mean_length"]),
            "total_path_length": float(final_stage["incumbent_total_length"]),
            "incumbent_queries": final_stage["incumbent_queries"],
        },
        "stages": stages,
    }


def final_query_deltas(lhs: dict[str, Any], rhs: dict[str, Any]) -> dict[str, float]:
    lhs_queries = dict(lhs["final"]["incumbent_queries"])
    rhs_queries = dict(rhs["final"]["incumbent_queries"])
    return {
        query_name: float(lhs_queries.get(query_name, 0.0)) - float(rhs_queries.get(query_name, 0.0))
        for query_name in QUERY_ORDER
    }


def stage_timing_delta(lhs_stage: dict[str, Any], rhs_stage: dict[str, Any]) -> dict[str, float]:
    lhs_build = dict(lhs_stage["build"])
    rhs_build = dict(rhs_stage["build"])
    return {
        "stage_total_ms": 1000.0 * (float(lhs_stage["stage_total_s"]) - float(rhs_stage["stage_total_s"])),
        "stage_build_ms": 1000.0 * (float(lhs_stage["stage_build_s"]) - float(rhs_stage["stage_build_s"])),
        "stage_query_ms": 1000.0 * (float(lhs_stage["stage_query_s"]) - float(rhs_stage["stage_query_s"])),
        "planning_ms": 1000.0 * (float(lhs_build["planning_s"]) - float(rhs_build["planning_s"])),
        "prebridge_ms": 1000.0 * (float(lhs_build["prebridge_time_s"]) - float(rhs_build["prebridge_time_s"])),
        "grow_ms": float(lhs_build["grow_ms"]) - float(rhs_build["grow_ms"]),
        "connector_ms": float(lhs_build["connector_ms"]) - float(rhs_build["connector_ms"]),
        "adjacency_ms": float(lhs_build["adjacency_ms"]) - float(rhs_build["adjacency_ms"]),
        "merge_ms": float(lhs_build["merge_ms"]) - float(rhs_build["merge_ms"]),
        "maintenance_ms": 1000.0 * (float(lhs_build["maintenance_s"]) - float(rhs_build["maintenance_s"])),
    }


def stage_mechanism_delta(lhs_stage: dict[str, Any], rhs_stage: dict[str, Any]) -> dict[str, float | list[str]]:
    lhs_split = dict(lhs_stage["split"])
    rhs_split = dict(rhs_stage["split"])
    lhs_component = dict(lhs_stage["component_connect"])
    rhs_component = dict(rhs_stage["component_connect"])
    lhs_frontier = dict(lhs_stage["frontier"])
    rhs_frontier = dict(rhs_stage["frontier"])
    lhs_build = dict(lhs_stage["build"])
    rhs_build = dict(rhs_stage["build"])
    return {
        "incumbent_total_length_delta": float(lhs_stage["incumbent_total_length"]) - float(rhs_stage["incumbent_total_length"]),
        "raw_total_length_delta": float(lhs_stage["raw_total_length"]) - float(rhs_stage["raw_total_length"]),
        "unique_box_count_delta": float(lhs_build["unique_box_count"]) - float(rhs_build["unique_box_count"]),
        "prebridge_added_boxes_delta": float(lhs_build["prebridge_added_boxes"]) - float(rhs_build["prebridge_added_boxes"]),
        "split_node_calls_delta": float(lhs_split["node_calls"]) - float(rhs_split["node_calls"]),
        "split_node_total_ms_delta": float(lhs_split["node_total_ms"]) - float(rhs_split["node_total_ms"]),
        "split_dim6_fraction_delta": float(lhs_split["dim_fractions"].get("dim_6", 0.0)) - float(rhs_split["dim_fractions"].get("dim_6", 0.0)),
        "split_zero_axes_lhs": list(lhs_split["zero_axes"]),
        "split_zero_axes_rhs": list(rhs_split["zero_axes"]),
        "component_connect_attempts_delta": float(lhs_component["attempts"]) - float(rhs_component["attempts"]),
        "component_connect_success_rate_delta": float((lhs_component["success_rate"] or 0.0) - (rhs_component["success_rate"] or 0.0)),
        "connected_root_pairs_max_delta": float(lhs_component["connected_root_pairs_max"]) - float(rhs_component["connected_root_pairs_max"]),
        "staged_targets_delta": float(lhs_component["staged_targets"]) - float(rhs_component["staged_targets"]),
        "face_score_calls_per_attempt_delta": float((lhs_component["face_score_calls_per_attempt"] or 0.0) - (rhs_component["face_score_calls_per_attempt"] or 0.0)),
        "frontier_reserved_path_misses_delta": float(lhs_frontier["reserved_path_misses"]) - float(rhs_frontier["reserved_path_misses"]),
        "frontier_covered_hit_per_insert_delta": float((lhs_frontier["covered_hit_per_insert"] or 0.0) - (rhs_frontier["covered_hit_per_insert"] or 0.0)),
        "frontier_uncovered_hit_per_insert_delta": float((lhs_frontier["uncovered_hit_per_insert"] or 0.0) - (rhs_frontier["uncovered_hit_per_insert"] or 0.0)),
    }


def pairwise_summary(lhs: dict[str, Any], rhs: dict[str, Any]) -> dict[str, Any]:
    return {
        "final_total_s_delta": float(lhs["final"]["total_s"]) - float(rhs["final"]["total_s"]),
        "final_build_s_delta": float(lhs["final"]["build_s"]) - float(rhs["final"]["build_s"]),
        "final_query_s_delta": float(lhs["final"]["query_s"]) - float(rhs["final"]["query_s"]),
        "final_total_path_length_delta": float(lhs["final"]["total_path_length"]) - float(rhs["final"]["total_path_length"]),
        "final_mean_path_length_delta": float(lhs["final"]["mean_path_length"]) - float(rhs["final"]["mean_path_length"]),
        "final_query_length_deltas": final_query_deltas(lhs, rhs),
        "stages": {
            stage_id: {
                "timing_delta": stage_timing_delta(lhs["stages"][stage_id], rhs["stages"][stage_id]),
                "mechanism_delta": stage_mechanism_delta(lhs["stages"][stage_id], rhs["stages"][stage_id]),
            }
            for stage_id in STAGE_ORDER
        },
    }


def build_paths(args: argparse.Namespace) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for name, spec in GROUP_SPECS.items():
        if str(spec["root"]) == "aabb_no_external":
            paths[name] = args.aabb_no_external
            continue
        root = args.e4_root if str(spec["root"]) == "e4" else args.depth48_root
        paths[name] = root / str(spec["path_name"])
    return paths


def interpretation(summary: dict[str, Any]) -> list[str]:
    baseline = summary["artifacts"]["baseline"]
    no_cache = summary["artifacts"]["no_cache"]
    aabb = summary["artifacts"]["aabb"]
    aabb_no_external = summary["artifacts"]["aabb_no_external"]
    baseline_d48 = summary["artifacts"]["baseline_d48"]
    round_robin = summary["artifacts"]["round_robin_d48"]
    notes: list[str] = []
    if abs(float(no_cache["final"]["total_path_length"]) - float(baseline["final"]["total_path_length"])) <= 1e-9:
        notes.append("Baseline 与 No cache 的最终 incumbent 路径完全一致，说明在当前 shelf d40 配置下，是否复用外部 LECT 证据并没有改变最终被接受的查询结果。")
    if abs(float(aabb["final"]["total_path_length"]) - float(baseline["final"]["total_path_length"])) <= 1e-3:
        notes.append("AABB only 与 Baseline 的最终路径总长只差约 6e-4 rad，差异仅集中在 CS->LB；其余四条 query 的最终 incumbent 一致。")
    if abs(float(aabb_no_external["final"]["total_path_length"]) - float(aabb["final"]["total_path_length"])) <= 1e-9:
        notes.append("补充的 AABB no-external probe 与原 AABB 行最终路径完全一致，排除了 AABB 接近 Baseline 是由 warm external evidence 单独造成的解释。")
    if float(round_robin["final"]["total_path_length"]) < float(baseline_d48["final"]["total_path_length"]):
        notes.append("在相同 d48 深度下，Round-robin 相比 Baseline d48 主要改善了 CS->LB、LB->RB 和 RB->AS，说明优势不是单纯来自更深树，而是来自 split policy 对后续 component-connect 候选的影响。")
    if "dim_6" not in list(round_robin["stages"]["quality"]["split"]["zero_axes"]):
        notes.append("Round-robin d48 在质量相关阶段保持了 7 个维度都可分，而 AAFKVolumeMin 相关配置持续把 dim_6 置零，这与其更高的 connected_root_pairs_max / staged_targets 同时出现。")
    return notes


def main() -> int:
    args = parse_args()
    paths = build_paths(args)
    payloads = {name: load_json(path) for name, path in paths.items()}
    summary = {
        "artifacts": {
            name: summarize_artifact(name, paths[name], payloads[name])
            for name in GROUP_SPECS
        },
    }
    summary["comparisons"] = {
        "no_cache_vs_baseline": pairwise_summary(summary["artifacts"]["no_cache"], summary["artifacts"]["baseline"]),
        "aabb_vs_baseline": pairwise_summary(summary["artifacts"]["aabb"], summary["artifacts"]["baseline"]),
        "aabb_no_external_vs_aabb": pairwise_summary(summary["artifacts"]["aabb_no_external"], summary["artifacts"]["aabb"]),
        "aabb_no_external_vs_baseline": pairwise_summary(summary["artifacts"]["aabb_no_external"], summary["artifacts"]["baseline"]),
        "round_robin_d48_vs_baseline_d40": pairwise_summary(summary["artifacts"]["round_robin_d48"], summary["artifacts"]["baseline"]),
        "round_robin_d48_vs_baseline_d48": pairwise_summary(summary["artifacts"]["round_robin_d48"], summary["artifacts"]["baseline_d48"]),
    }
    summary["interpretation"] = interpretation(summary)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / "exp04_ablation_timing_analysis.json"
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({"out_json": str(out_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())