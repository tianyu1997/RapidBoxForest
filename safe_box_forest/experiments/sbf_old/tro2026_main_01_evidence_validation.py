#!/usr/bin/env python3
"""Planner-level validation-profile ablation for the TRO 2026 SBF experiments.

This wrapper runs the existing Marcucci combined-scene SBF experiment with several
strict/provisional evidence profiles, then writes a compact artifact for table and
figure generation.
"""
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / "experiments" / "paper_04_marcucci_combined.py"
DEFAULT_OUT = ROOT / "outputs" / "paper" / "tro2026_exp14_validation_profiles.json"
DEFAULT_WORK = ROOT / "outputs" / "paper" / "exp14_validation_profiles"

PROFILE_SPECS: dict[str, dict[str, Any]] = {
    "ifk_linkiaabb": {
        "label": "IFK+LinkIAABB",
        "source": "IFK",
        "envelope": "LinkIAABB",
        "preset": "ifk_strict",
        "cli_envelope": "link",
        "evidence": "strict",
        "extra_args": [],
    },
    "ifk_kdop26": {
        "label": "IFK+KDOP26",
        "source": "IFK",
        "envelope": "KDOP26",
        "preset": "ifk_strict",
        "cli_envelope": "kdop26",
        "evidence": "strict",
        "extra_args": [],
    },
    "ifk_support_hull": {
        "label": "IFK+SupportHull",
        "source": "IFK",
        "envelope": "SupportHull",
        "preset": "ifk_strict",
        "cli_envelope": "support_hull",
        "evidence": "strict",
        "extra_args": [],
    },
    "crit_linkiaabb": {
        "label": "Crit+LinkIAABB",
        "source": "Crit",
        "envelope": "LinkIAABB",
        "preset": "crit_link_coverage",
        "cli_envelope": "link",
        "evidence": "provisional",
        "extra_args": [],
    },
    "crit_kdop26": {
        "label": "Crit+KDOP26",
        "source": "Crit",
        "envelope": "KDOP26",
        "preset": "crit_link_coverage",
        "cli_envelope": "kdop26",
        "evidence": "provisional",
        "extra_args": [],
    },
    "crit_support_hull": {
        "label": "Crit+SupportHull",
        "source": "Crit",
        "envelope": "SupportHull",
        "preset": "crit_link_coverage",
        "cli_envelope": "support_hull",
        "evidence": "provisional",
        "extra_args": [],
    },
}

PROFILE_ALIASES = {
    "ifk_strict": "ifk_linkiaabb",
    "ifk_sh": "ifk_support_hull",
    "crit_link_coverage": "crit_linkiaabb",
    "crit_sh": "crit_support_hull",
    "kdop26_coverage": "crit_kdop26",
    "support_hull_no_kdop": "crit_support_hull",
}


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def mean(values: list[float]) -> float | None:
    return float(sum(values) / len(values)) if values else None


def pct(num: int, den: int) -> float | None:
    return float(num / den) if den else None


def wilson_ci(success: int, total: int, z: float = 1.959963984540054) -> list[float | None]:
    if total <= 0:
        return [None, None]
    phat = success / total
    denom = 1.0 + z * z / total
    center = (phat + z * z / (2.0 * total)) / denom
    radius = z * ((phat * (1.0 - phat) + z * z / (4.0 * total)) / total) ** 0.5 / denom
    return [max(0.0, center - radius), min(1.0, center + radius)]


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def query_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for trial in payload.get("trials", []):
        seed = trial.get("seed")
        for query in trial.get("queries", []):
            row = dict(query)
            row["seed"] = seed
            rows.append(row)
    if rows:
        return rows
    for query in payload.get("queries", []):
        row = dict(query)
        row["seed"] = None
        rows.append(row)
    return rows


def safe_float(row: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = row.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def query_name(row: dict[str, Any]) -> str:
    if "query" in row:
        return str(row["query"])
    if "from" in row and "to" in row:
        return f"{row['from']}->{row['to']}"
    return "unknown"


def summarize_per_query(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(query_name(row), []).append(row)
    summary: list[dict[str, Any]] = []
    for name, group in sorted(grouped.items()):
        total = len(group)
        candidates = [row for row in group if bool(row.get("ok", row.get("success", False)))]
        audited = [row for row in group if bool(row.get("audit_passed", False))]
        repair_count_sum = sum(int(row.get("repair_count", 0) or 0) for row in group)
        lengths = [safe_float(row, "length", safe_float(row, "path_length")) for row in audited]
        summary.append({
            "query": name,
            "count": total,
            "candidate_success_count": len(candidates),
            "audit_success_count": len(audited),
            "candidate_sr": pct(len(candidates), total),
            "audit_sr": pct(len(audited), total),
            "repair_count_sum": repair_count_sum,
            "median_audited_path_length": median(lengths),
        })
    return summary


def aggregate_profile(profile: str, spec: dict[str, Any], artifact: Path) -> dict[str, Any]:
    payload = load_json(artifact)
    trials = payload.get("trials", [])
    rows = query_rows(payload)
    total = len(rows)
    ok_rows = [row for row in rows if bool(row.get("ok", row.get("success", False)))]
    audited_rows = [row for row in rows if bool(row.get("audit_passed", False))]
    repaired_rows = [row for row in rows if int(row.get("repair_count", 0) or 0) > 0]
    failed_candidate_rows = [row for row in rows if not bool(row.get("ok", row.get("success", False)))]
    missing_audit_rows = [row for row in ok_rows if "audit_passed" not in row or row.get("audit_passed") is None]
    failed_audit_rows = [row for row in ok_rows if row.get("audit_passed") is False]
    repair_count_sum = sum(int(row.get("repair_count", 0) or 0) for row in rows)

    certified_len = sum(safe_float(row, "certified_box_length") for row in rows)
    provisional_len = sum(safe_float(row, "provisional_audited_length") for row in rows)
    segment_len = sum(safe_float(row, "segment_edge_length") for row in rows)
    total_accounted_len = certified_len + provisional_len + segment_len

    build_s = [float(trial.get("build_s", 0.0)) for trial in trials if "build_s" in trial]
    box_counts = [float(trial.get("unique_box_count", 0.0)) for trial in trials if "unique_box_count" in trial]
    certified_boxes = [float(trial.get("certified_box_count", 0.0)) for trial in trials if "certified_box_count" in trial]
    provisional_boxes = [float(trial.get("provisional_box_count", 0.0)) for trial in trials if "provisional_box_count" in trial]
    segment_edges = [float(trial.get("segment_edge_count", 0.0)) for trial in trials if "segment_edge_count" in trial]

    query_ms = [safe_float(row, "planning_time_ms") for row in rows if bool(row.get("ok", row.get("success", False)))]
    audit_ms = [safe_float(row, "audit_time_ms") for row in rows if bool(row.get("ok", row.get("success", False)))]
    path_len = [safe_float(row, "length", safe_float(row, "path_length")) for row in rows if bool(row.get("audit_passed", False))]

    return {
        "profile": profile,
        "label": spec["label"],
        "source": spec.get("source"),
        "envelope": spec.get("envelope"),
        "preset": spec["preset"],
        "evidence": spec["evidence"],
        "artifact": str(artifact),
        "seeds": payload.get("seeds", len(trials)),
        "query_count": total,
        "candidate_success_count": len(ok_rows),
        "audit_success_count": len(audited_rows),
        "candidate_failed_count": len(failed_candidate_rows),
        "no_candidate_count": len(failed_candidate_rows),
        "failed_audit_count": len(failed_audit_rows),
        "candidate_failed_audit_count": len(failed_audit_rows),
        "missing_audit_count": len(missing_audit_rows),
        "repair_event_count": len(repaired_rows),
        "repair_query_count": len(repaired_rows),
        "repair_count_sum": repair_count_sum,
        "query_sr": pct(len(ok_rows), total),
        "audit_sr": pct(len(audited_rows), total),
        "query_sr_ci95": wilson_ci(len(ok_rows), total),
        "audit_sr_ci95": wilson_ci(len(audited_rows), total),
        "repair_per_query": pct(len(repaired_rows), total),
        "median_build_s": median(build_s),
        "mean_build_s": mean(build_s),
        "median_box_count": median(box_counts),
        "mean_box_count": mean(box_counts),
        "median_certified_box_count": median(certified_boxes),
        "mean_certified_box_count": mean(certified_boxes),
        "median_provisional_box_count": median(provisional_boxes),
        "mean_provisional_box_count": mean(provisional_boxes),
        "median_segment_edge_count": median(segment_edges),
        "median_query_ms_success": median(query_ms),
        "median_audit_ms_success": median(audit_ms),
        "median_audited_path_length": median(path_len),
        "length_accounting": {
            "certified_box_length": certified_len,
            "provisional_audited_length": provisional_len,
            "segment_edge_length": segment_len,
            "certified_fraction": certified_len / total_accounted_len if total_accounted_len else None,
            "provisional_fraction": provisional_len / total_accounted_len if total_accounted_len else None,
            "segment_edge_fraction": segment_len / total_accounted_len if total_accounted_len else None,
        },
        "per_query_summary": summarize_per_query(rows),
        "params": payload.get("params", {}),
    }


def run_profile(profile: str, spec: dict[str, Any], artifact: Path, args: argparse.Namespace) -> list[str]:
    cmd = [
        sys.executable,
        str(EXPERIMENT),
        "--preset",
        spec["preset"],
        "--envelope",
        spec.get("cli_envelope", "preset"),
        "--out-json",
        str(artifact),
        "--seeds",
        str(args.seeds),
        "--seed-base",
        str(args.seed_base),
        "--threads",
        str(args.threads),
        "--max-boxes",
        str(args.max_boxes),
        "--timeout-ms",
        str(args.timeout_ms),
        "--audit-resolution",
        str(args.audit_resolution),
    ]
    cmd.extend(str(value) for value in spec.get("extra_args", []))
    if args.full_growth:
        cmd.extend(["--no-stop-after-connect", "--post-connect-extra-boxes", str(args.post_connect_extra_boxes)])
    if args.no_repair:
        cmd.append("--no-repair-on-audit-failure")
    if args.no_segment_edges:
        cmd.append("--no-segment-edges")
    subprocess.run(cmd, cwd=ROOT, check=True)
    return [str(part) for part in cmd]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run/aggregate TRO validation-profile ablation.")
    parser.add_argument("--out-json", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--work-dir", type=Path, default=DEFAULT_WORK)
    parser.add_argument("--profiles", default=",".join(PROFILE_SPECS.keys()))
    parser.add_argument("--reuse-existing", action="store_true", help="Do not rerun profiles whose per-profile JSON already exists.")
    parser.add_argument("--aggregate-only", action="store_true", help="Require existing per-profile JSON artifacts and only aggregate them.")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--max-boxes", type=int, default=5000)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--audit-resolution", type=int, default=32)
    parser.add_argument("--full-growth", action="store_true", help="Continue growth after first connection for coverage-quality rows.")
    parser.add_argument("--post-connect-extra-boxes", type=int, default=500)
    parser.add_argument("--no-repair", action="store_true", help="Disable local repair to expose raw audit failures.")
    parser.add_argument("--no-segment-edges", action="store_true", help="Disable segment edges for a box-only comparison.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    requested = [PROFILE_ALIASES.get(name.strip(), name.strip()) for name in args.profiles.split(",") if name.strip()]
    unknown = [name for name in requested if name not in PROFILE_SPECS]
    if unknown:
        raise SystemExit(f"Unknown profiles: {', '.join(unknown)}")

    args.work_dir.mkdir(parents=True, exist_ok=True)
    commands: dict[str, list[str]] = {}
    rows: list[dict[str, Any]] = []
    for profile in requested:
        spec = PROFILE_SPECS[profile]
        artifact = args.work_dir / f"{profile}.json"
        if not artifact.exists():
            if args.aggregate_only:
                raise FileNotFoundError(f"Missing profile artifact: {artifact}")
            commands[profile] = run_profile(profile, spec, artifact, args)
        elif not args.reuse_existing and not args.aggregate_only:
            commands[profile] = run_profile(profile, spec, artifact, args)
        rows.append(aggregate_profile(profile, spec, artifact))

    payload = {
        "experiment": "validation_profile_ablation",
        "source_script": str(Path(__file__).resolve()),
        "profiles": requested,
        "params": {
            "seeds": args.seeds,
            "seed_base": args.seed_base,
            "threads": args.threads,
            "max_boxes": args.max_boxes,
            "timeout_ms": args.timeout_ms,
            "audit_resolution": args.audit_resolution,
            "full_growth": args.full_growth,
            "post_connect_extra_boxes": args.post_connect_extra_boxes if args.full_growth else 0,
            "repair_enabled": not args.no_repair,
            "segment_edges_enabled": not args.no_segment_edges,
            "fair_design": "All profile rows use the same fixed box/time budget, 8 SBF worker threads, connector settings, segment-edge policy, and query set; the run does not require all islands to become connected before reporting outcomes.",
        },
        "commands": commands,
        "rows": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"out_json": str(args.out_json), "rows": rows}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
