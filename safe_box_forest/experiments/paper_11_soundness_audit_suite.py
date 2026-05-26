#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PAPER_OUTPUTS = ROOT / "outputs" / "paper"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Aggregate strict collision-audit accounting across paper path artifacts.")
    parser.add_argument("--outputs", type=Path, default=PAPER_OUTPUTS)
    parser.add_argument("--out-json", type=Path, default=PAPER_OUTPUTS / "paper_soundness_audit_suite.json")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def load_first(outputs: Path, names: list[str]) -> tuple[str, dict[str, Any]] | tuple[None, None]:
    for name in names:
        payload = load_json(outputs / name)
        if payload:
            return name, payload
    return None, None


def truthy(value: Any) -> bool:
    return bool(value)


def add_row(
    rows: list[dict[str, Any]],
    scope: str,
    artifact: str,
    path_count: int,
    audit_pass_count: int,
    repair_or_fallback_events: float,
    solved_unsafe_count: int,
    remaining_unsafe_assumptions: str,
    count_basis: str,
) -> None:
    audit_sr = float(audit_pass_count) / float(path_count) if path_count else None
    rows.append({
        "scope": scope,
        "artifact": artifact,
        "path_count": int(path_count),
        "audit_pass_count": int(audit_pass_count),
        "audit_sr": audit_sr,
        "repair_or_fallback_events": float(repair_or_fallback_events),
        "solved_unsafe_count": int(solved_unsafe_count),
        "remaining_unsafe_assumptions": remaining_unsafe_assumptions,
        "count_basis": count_basis,
    })


def aggregate_marcucci(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp04_marcucci_full.json", "marcucci_corridor_refine_selfedge_s10.json"])
    if not payload:
        return
    trial_queries = [query for trial in payload.get("trials", []) for query in trial.get("queries", [])]
    if trial_queries:
        path_count = len(trial_queries)
        audit_pass = sum(1 for query in trial_queries if truthy(query.get("audit_passed")))
        solved = sum(1 for query in trial_queries if truthy(query.get("ok")))
        repair_events = sum(float(query.get("repair_count", 0.0)) for query in trial_queries)
        add_row(rows, "Exp.4 Marcucci SBF", name, path_count, audit_pass, repair_events, max(0, solved - audit_pass), "none after strict audit", "exact per-trial query rows")
        return
    seeds = int(payload.get("seeds", 1))
    queries = payload.get("queries", [])
    path_count = seeds * len(queries)
    audit_pass = sum(int(round(float(query.get("audit_sr", 0.0)) * seeds)) for query in queries)
    solved = sum(int(round(float(query.get("sr", 0.0)) * seeds)) for query in queries)
    repair_events = sum(float(query.get("repair_count_med", 0.0)) * seeds for query in queries)
    add_row(rows, "Exp.4 Marcucci SBF", name, path_count, audit_pass, repair_events, max(0, solved - audit_pass), "none after strict audit", "per-query SR medians scaled by seed count")


def aggregate_grower(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp03_grower_full.json", "marcucci_grower_tradeoff.json"])
    if not payload:
        return
    trial_queries = [query for trials in payload.get("trials_by_quality", {}).values() for trial in trials for query in trial.get("queries", [])]
    if trial_queries:
        path_count = len(trial_queries)
        audit_pass = sum(1 for query in trial_queries if truthy(query.get("audit_passed")))
        solved = sum(1 for query in trial_queries if truthy(query.get("ok")))
        repair_events = sum(float(query.get("repair_count", 0.0)) for query in trial_queries)
        add_row(rows, "Exp.3 grow-stop sweep", name, path_count, audit_pass, repair_events, max(0, solved - audit_pass), "none after strict audit", "exact per-trial query rows across quality settings")
        return
    path_count = 0
    audit_pass = 0
    solved = 0
    repair_events = 0.0
    for setting in payload.get("settings", []):
        trials = int(setting.get("trial_count", 1))
        queries = setting.get("queries", [])
        setting_paths = trials * len(queries)
        path_count += setting_paths
        audit_pass += int(round(float(setting.get("audit_sr", 0.0)) * setting_paths))
        solved += int(round(float(setting.get("sr", 0.0)) * setting_paths))
        repair_events += float(setting.get("repair_total_median", 0.0)) * trials
    add_row(rows, "Exp.3 grow-stop sweep", name, path_count, audit_pass, repair_events, max(0, solved - audit_pass), "none after strict audit", "setting-level SR and median repairs scaled by trial count")


def aggregate_direct_gcs(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp07_merger_gcs_full.json", "marcucci_merger_gcs.json"])
    if not payload:
        return
    gcs_rows = payload.get("gcs_queries", [])
    path_count = len(gcs_rows)
    audit_pass = sum(1 for row in gcs_rows if row.get("audit", {}).get("passed"))
    solved_unsafe = sum(1 for row in gcs_rows if row.get("ok") and not row.get("audit", {}).get("passed"))
    add_row(rows, "Exp.7 direct merger+GCS control", name, path_count, audit_pass, 0.0, solved_unsafe, "provisional/merged region interiors are not convex-free certificates", "direct GCS rows")


def aggregate_audited_gcs(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp07_gcs_full.json", "marcucci_audited_corridor_gcs.json"])
    if not payload:
        return
    query_rows = payload.get("queries", [])
    path_count = len(query_rows)
    audit_pass = sum(1 for row in query_rows if row.get("final_strict_audit_passed"))
    solved_unsafe_attempts = 0
    for row in query_rows:
        for attempt in row.get("gcs_attempts", []):
            if attempt.get("ok") and not attempt.get("strict_audit_passed"):
                solved_unsafe_attempts += 1
    fallback_count = float(payload.get("summary", {}).get("fallback_count", 0.0))
    residual = f"none for final counted paths; {solved_unsafe_attempts} unsafe solved attempts rejected"
    add_row(rows, "Exp.7 audited corridor GCS", name, path_count, audit_pass, fallback_count, solved_unsafe_attempts, residual, "final strict audit plus rejected-attempt accounting")


def aggregate_random(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp05_random_sbf_full.json", "exp5_random_robot_scenes_standalone.json"])
    if not payload:
        return
    result_rows = payload.get("rows", [])
    path_count = len(result_rows)
    audit_pass = sum(1 for row in result_rows if truthy(row.get("query", {}).get("audit_passed")))
    solved_unsafe = sum(1 for row in result_rows if row.get("query", {}).get("ok") and not truthy(row.get("query", {}).get("audit_passed")))
    repair_events = sum(float(row.get("query", {}).get("repair_count", 0.0)) for row in result_rows)
    add_row(rows, "Exp.5 random robot scenes", name, path_count, audit_pass, repair_events, solved_unsafe, "none for audited successes; failures remain failures", "per-scene query rows")


def aggregate_rrt_baseline(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp05_random_rrt_full.json", "random_scene_rrt_connect_baseline.json"])
    if not payload:
        return
    result_rows = payload.get("rows", [])
    path_count = len(result_rows)
    audit_pass = sum(1 for row in result_rows if truthy(row.get("audit_passed")))
    solved_unsafe = sum(1 for row in result_rows if row.get("ok") and not truthy(row.get("audit_passed")))
    add_row(rows, "Exp.5 OMPL RRTConnect random-scene baseline", name, path_count, audit_pass, 0.0, solved_unsafe, "failed baseline trials remain failures", "OMPL RRTConnect trial rows")


def aggregate_marcucci_rrt_baseline(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp04_rrt_connect_full.json", "marcucci_rrt_connect_baseline.json"])
    if not payload:
        return
    result_rows = payload.get("rows", [])
    path_count = len(result_rows)
    audit_pass = sum(1 for row in result_rows if truthy(row.get("audit_passed")))
    solved_unsafe = sum(1 for row in result_rows if row.get("ok") and not truthy(row.get("audit_passed")))
    add_row(rows, "Exp.4 OMPL RRTConnect shelf baseline", name, path_count, audit_pass, 0.0, solved_unsafe, "failed baseline trials remain failures", "OMPL RRTConnect shelf trial rows")


def aggregate_parallel(outputs: Path, rows: list[dict[str, Any]]) -> None:
    name, payload = load_first(outputs, ["tro2026_exp08_parallel_full.json", "parallel_scaling_standalone.json"])
    if not payload:
        return
    query_rows = [query for trial in payload.get("trials", []) for query in trial.get("query_rows", [])]
    path_count = len(query_rows)
    audit_pass = sum(1 for row in query_rows if truthy(row.get("audit_passed")))
    solved_unsafe = sum(1 for row in query_rows if row.get("ok") and not truthy(row.get("audit_passed")))
    repair_events = sum(float(row.get("repair_count", 0.0)) for row in query_rows)
    add_row(rows, "Exp.8 parallel sweep", name, path_count, audit_pass, repair_events, solved_unsafe, "none for audited successes", "thread-sweep query rows")


def main() -> int:
    args = parse_args()
    rows: list[dict[str, Any]] = []
    aggregate_marcucci(args.outputs, rows)
    aggregate_grower(args.outputs, rows)
    aggregate_audited_gcs(args.outputs, rows)
    aggregate_direct_gcs(args.outputs, rows)
    aggregate_marcucci_rrt_baseline(args.outputs, rows)
    aggregate_random(args.outputs, rows)
    aggregate_rrt_baseline(args.outputs, rows)
    aggregate_parallel(args.outputs, rows)
    total_paths = sum(int(row["path_count"]) for row in rows)
    total_audit = sum(int(row["audit_pass_count"]) for row in rows)
    payload = {
        "experiment": "paper_11_soundness_audit_suite",
        "rows": rows,
        "summary": {
            "path_count": total_paths,
            "audit_pass_count": total_audit,
            "audit_sr": float(total_audit) / float(total_paths) if total_paths else None,
            "solved_unsafe_count": sum(int(row["solved_unsafe_count"]) for row in rows),
            "repair_or_fallback_events": sum(float(row["repair_or_fallback_events"]) for row in rows),
        },
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "summary": payload["summary"]}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())