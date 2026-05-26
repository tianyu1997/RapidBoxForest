#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
OUTPUTS = ROOT / "outputs" / "paper"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reviewer-facing mechanism diagnostics for the TRO rewrite.")
    parser.add_argument("--outputs", type=Path, default=OUTPUTS)
    parser.add_argument("--out-json", type=Path, default=OUTPUTS / "tro_mechanism_diagnostics.json")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def mean(values: Iterable[float]) -> float | None:
    rows = [float(value) for value in values if value is not None]
    return statistics.fmean(rows) if rows else None


def median(values: Iterable[float]) -> float | None:
    rows = [float(value) for value in values if value is not None]
    return statistics.median(rows) if rows else None


def add_parallel(rows: list[dict[str, Any]], outputs: Path) -> None:
    payload = load_json(outputs / "tro2026_exp08_parallel_full.json") or load_json(outputs / "parallel_scaling_standalone.json")
    if not payload:
        return
    summary = payload.get("summary", [])
    best = max(summary, key=lambda item: float(item.get("speedup") or 0.0), default={})
    worst_eff = min(summary, key=lambda item: float(item.get("efficiency") or 1.0), default={})
    grow_ms = mean(row.get("grow_ms") for row in payload.get("trials", []))
    prebridge_s = mean(row.get("prebridge_time_s") for row in payload.get("trials", []))
    rows.append({
        "scope": "parallel scaling",
        "metric": "best speedup / lowest efficiency",
        "value": f"{float(best.get('speedup', 0.0)):.2f}x at {int(best.get('threads', 0))} threads; {float(worst_eff.get('efficiency', 0.0)):.2f} efficiency at {int(worst_eff.get('threads', 0))} threads",
        "interpretation": "small Marcucci builds are dominated by serial scheduling, bridge/refine, and merge overhead; parallelism is not claimed as strong scaling",
        "evidence": {"mean_grow_ms": grow_ms, "mean_prebridge_s": prebridge_s},
    })


def add_grow_stop(rows: list[dict[str, Any]], outputs: Path) -> None:
    payload = load_json(outputs / "tro2026_exp03_grower_full.json") or load_json(outputs / "marcucci_grower_tradeoff.json")
    if not payload:
        return
    settings = payload.get("settings", [])
    floor0 = next((row for row in settings if int(row.get("quality_min_connected_boxes", -1)) == 0), {})
    balance = next((row for row in settings if int(row.get("quality_min_connected_boxes", -1)) == 64), payload.get("balance_point", {}))
    rows.append({
        "scope": "grow-stop policy",
        "metric": "quality floor 0 vs selected knee",
        "value": f"boxes {float(floor0.get('box_mean', 0.0)):.1f} to {float(balance.get('box_mean', 0.0)):.1f}; total path {float(floor0.get('total_length_median', 0.0)):.3f} to {float(balance.get('total_length_median', 0.0)):.3f}",
        "interpretation": "larger connected-box floor improves reusable corridor quality but does not remove all local repairs on this topology",
        "evidence": {"repair_floor0": floor0.get("repair_total_median"), "repair_balance": balance.get("repair_total_median")},
    })


def add_gcs(rows: list[dict[str, Any]], outputs: Path) -> None:
    payload = load_json(outputs / "marcucci_protected_merger_study.json")
    if not payload:
        return
    by_mode = {row.get("mode"): row for row in payload.get("rows", [])}
    direct = by_mode.get("direct_merger_gcs", {})
    expanded = by_mode.get("overlap_expanded_sbf_corridor", {})
    protected = by_mode.get("protected_final_policy", {})
    rows.append({
        "scope": "GCS composition",
        "metric": "direct / expanded / protected audit",
        "value": f"{int(direct.get('audit_pass_count', 0))}/{int(direct.get('solve_count', 0))}; {int(expanded.get('audit_pass_count', 0))}/{int(expanded.get('solve_count', 0))}; {int(protected.get('audit_pass_count', 0))}/{int(protected.get('query_count', 0))}",
        "interpretation": "box expansion repairs overlap feasibility but safety is recovered only by the protected audited interface",
        "evidence": {"expanded_max_gap_before_expansion": expanded.get("max_gap_before_expansion")},
    })


def add_random_baseline(rows: list[dict[str, Any]], outputs: Path) -> None:
    payload = load_json(outputs / "tro2026_exp05_random_rrt_full.json") or load_json(outputs / "random_scene_rrt_connect_baseline.json")
    if not payload:
        return
    summary = payload.get("summary", [])
    rows.append({
        "scope": "random-scene baseline",
        "metric": "OMPL RRTConnect audit SR range",
        "value": f"{min(float(row.get('audit_sr') or 0.0) for row in summary):.2f}-{max(float(row.get('audit_sr') or 0.0) for row in summary):.2f}",
        "interpretation": "random-scene SBF rows are now contextualized against an OMPL single-query planner using the same SBF collision checker",
        "evidence": {"mean_success_time_s": mean(row.get("success_time_mean_s") for row in summary)},
    })


def add_shelf_rrt_baseline(rows: list[dict[str, Any]], outputs: Path) -> None:
    payload = load_json(outputs / "tro2026_exp04_rrt_connect_full.json") or load_json(outputs / "marcucci_rrt_connect_baseline.json")
    if not payload:
        return
    summary = payload.get("summary", {})
    query_rows = payload.get("queries", [])
    median_time = median(row.get("t_med_s") for row in query_rows)
    rows.append({
        "scope": "shelf baseline",
        "metric": "OMPL RRTConnect audit SR / median time",
        "value": f"{float(summary.get('audit_sr') or 0.0):.2f}; {float(median_time):.3f} s" if median_time is not None else f"{float(summary.get('audit_sr') or 0.0):.2f}; --",
        "interpretation": "OMPL RRTConnect is a single-query shelf reference with zero reusable build cost",
        "evidence": {"path_count": summary.get("path_count"), "sr": summary.get("sr")},
    })


def add_soundness(rows: list[dict[str, Any]], outputs: Path) -> None:
    payload = load_json(outputs / "tro2026_safety_accounting_full.json") or load_json(outputs / "paper_soundness_audit_suite.json")
    if not payload:
        return
    summary = payload.get("summary", {})
    rows.append({
        "scope": "paper-wide audit",
        "metric": "strict audit pass",
        "value": f"{int(summary.get('audit_pass_count', 0))}/{int(summary.get('path_count', 0))}",
        "interpretation": "failed random-scene queries remain failures; solved-but-unsafe paths are separated as negative controls",
        "evidence": {"solved_unsafe_count": summary.get("solved_unsafe_count")},
    })


def main() -> int:
    args = parse_args()
    rows: list[dict[str, Any]] = []
    add_shelf_rrt_baseline(rows, args.outputs)
    add_random_baseline(rows, args.outputs)
    add_grow_stop(rows, args.outputs)
    add_gcs(rows, args.outputs)
    add_parallel(rows, args.outputs)
    add_soundness(rows, args.outputs)
    payload = {
        "experiment": "paper_13_mechanism_diagnostics",
        "source_script": str(Path(__file__).resolve()),
        "note": "Aggregates claim-boundary mechanism checks without changing the primary success metric.",
        "rows": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "rows": rows}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())