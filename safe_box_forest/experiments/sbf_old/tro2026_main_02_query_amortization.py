#!/usr/bin/env python3
"""Build/query amortization postprocessor for TRO 2026 experiments.

The script consumes existing SBF and baseline artifacts and writes a compact JSON,
CSV, and optional matplotlib figure for the multi-query amortization plot.
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
PAPER_OUT = ROOT / "outputs" / "paper"
DEFAULT_OUT = PAPER_OUT / "tro2026_exp15_query_amortization.json"
DEFAULT_CSV = PAPER_OUT / "tro2026_exp15_query_amortization.csv"
DEFAULT_FIG = ROOT / "doc" / "paper" / "tro_rewrite_2026" / "generated" / "fig_tro_query_amortization.pdf"
DEFAULT_BASELINES = [
    PAPER_OUT / "marcucci_iris_np_gcs.json",
    PAPER_OUT / "marcucci_ompl_prm.json",
    PAPER_OUT / "marcucci_ompl_bitstar_budget.json",
    PAPER_OUT / "tro2026_exp04_rrt_connect_full.json",
    PAPER_OUT / "marcucci_rrt_connect_baseline.json",
]


def load_json(path: Path | None) -> dict[str, Any] | None:
    if path is None or not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def first_existing(paths: list[Path]) -> Path | None:
    for path in paths:
        if path.exists():
            return path
    return None


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def method_label(name: str) -> str:
    labels = {
        "iris_np_gcs": "IRIS-NP+GCS",
        "ompl_prm": "PRM",
        "ompl_bitstar_budget": "BIT*",
        "rrt_connect": "OMPL RRTConnect",
        "marcucci_rrt_connect_baseline": "OMPL RRTConnect",
        "tro2026_exp04_rrt_connect_full": "OMPL RRTConnect",
    }
    return labels.get(name, name)


def iter_sbf_query_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for trial in payload.get("trials", []):
        for query in trial.get("queries", []):
            rows.append(dict(query))
    if rows:
        return rows
    return [dict(query) for query in payload.get("queries", [])]


def sbf_method_from_payload(label: str, payload: dict[str, Any]) -> dict[str, Any]:
    build = payload.get("build", {})
    build_s = safe_float(build.get("median_s", build.get("mean_s", 0.0)))
    if not build_s:
        trial_builds = [safe_float(trial.get("build_s")) for trial in payload.get("trials", []) if "build_s" in trial]
        build_s = median(trial_builds) or 0.0
    rows = iter_sbf_query_rows(payload)
    audited = [row for row in rows if bool(row.get("audit_passed", row.get("ok", False)))]
    candidates = [row for row in rows if bool(row.get("ok", row.get("success", False)))]
    query_ms = [safe_float(row.get("planning_time_ms", row.get("query_time_ms"))) for row in audited or rows]
    audit_ms = [safe_float(row.get("audit_time_ms")) for row in audited or rows]
    return {
        "method": label,
        "class": "sbf_reusable",
        "source": "sbf_artifact",
        "success_semantics": "strict_audited_after_repair",
        "build_s": build_s,
        "query_s": (median(query_ms) or 0.0) / 1000.0,
        "audit_s": (median(audit_ms) or 0.0) / 1000.0,
        "candidate_success_rate": len(candidates) / len(rows) if rows else None,
        "audit_success_rate": len(audited) / len(rows) if rows else None,
        "success_rate": len(audited) / len(rows) if rows else None,
        "query_count_observed": len(rows),
    }


def seed_trial_methods_from_payload(path: Path, payload: dict[str, Any]) -> list[dict[str, Any]]:
    seed_trials = payload.get("seed_trials", [])
    if not seed_trials:
        return []
    name = method_label(str(payload.get("method", path.stem)))
    build_values = [safe_float(trial.get("build_s")) for trial in seed_trials if trial.get("build_s") is not None]
    query_rows: list[dict[str, Any]] = []
    for trial in seed_trials:
        query_rows.extend(trial.get("queries", []))
    successful = [row for row in query_rows if bool(row.get("success", row.get("ok", False)))]
    time_s = [safe_float(row.get("time_s", row.get("planning_time_s"))) for row in successful if row.get("time_s", row.get("planning_time_s")) is not None]
    return [{
        "method": name,
        "class": "baseline",
        "source": str(path),
        "success_semantics": "candidate_path; external audit unavailable in archived artifact",
        "build_s": median(build_values) or 0.0,
        "query_s": median(time_s) or 0.0,
        "audit_s": 0.0,
        "candidate_success_rate": len(successful) / len(query_rows) if query_rows else None,
        "audit_success_rate": None,
        "success_rate": len(successful) / len(query_rows) if query_rows else None,
        "query_count_observed": len(query_rows),
    }]


def baseline_methods_from_payload(path: Path, payload: dict[str, Any]) -> list[dict[str, Any]]:
    methods: list[dict[str, Any]] = []

    methods.extend(seed_trial_methods_from_payload(path, payload))
    if methods:
        return [method for method in methods if method["query_s"] > 0.0 or method["build_s"] > 0.0]

    # Common layout: {"methods": [{"method": ..., "build_s": ..., "query_ms": ...}, ...]}
    for row in payload.get("methods", []) if isinstance(payload.get("methods"), list) else []:
        name = method_label(str(row.get("method", row.get("name", path.stem))))
        build_s = safe_float(row.get("build_s", row.get("build_time_s", 0.0)))
        query_s = safe_float(row.get("median_query_ms", row.get("query_ms", row.get("planning_time_ms", 0.0)))) / 1000.0
        if not query_s:
            query_s = safe_float(row.get("median_query_s", row.get("query_s", 0.0)))
        success_rate = row.get("audit_success_rate", row.get("success_rate", row.get("sr")))
        methods.append({
            "method": name,
            "class": "baseline",
            "source": str(path),
            "success_semantics": "as_reported_by_source_artifact",
            "build_s": build_s,
            "query_s": query_s,
            "audit_s": safe_float(row.get("audit_ms", 0.0)) / 1000.0,
            "candidate_success_rate": row.get("candidate_success_rate", success_rate),
            "audit_success_rate": row.get("audit_success_rate"),
            "success_rate": success_rate,
        })

    # Trial/query layout used by some baseline scripts.
    if not methods:
        rows = []
        for trial in payload.get("trials", []):
            rows.extend(trial.get("queries", []))
        if rows:
            by_method: dict[str, list[dict[str, Any]]] = {}
            for row in rows:
                by_method.setdefault(str(row.get("method", path.stem)), []).append(row)
            for name, group in by_method.items():
                successful = [row for row in group if bool(row.get("success", row.get("ok", False)))]
                times_ms = [safe_float(row.get("planning_time_ms", row.get("time_ms"))) for row in successful or group]
                methods.append({
                    "method": method_label(name),
                    "class": "baseline",
                    "source": str(path),
                    "success_semantics": "candidate_path; audit unavailable unless source artifact says otherwise",
                    "build_s": 0.0,
                    "query_s": (median(times_ms) or 0.0) / 1000.0,
                    "audit_s": 0.0,
                    "candidate_success_rate": len(successful) / len(group) if group else None,
                    "audit_success_rate": None,
                    "success_rate": len(successful) / len(group) if group else None,
                })

    return [method for method in methods if method["query_s"] > 0.0 or method["build_s"] > 0.0]


def amortization_rows(methods: list[dict[str, Any]], counts: list[int]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for method in methods:
        for count in counts:
            total_s = method["build_s"] + count * (method["query_s"] + method.get("audit_s", 0.0))
            rows.append({
                "method": method["method"],
                "class": method["class"],
                "queries": count,
                "build_s": method["build_s"],
                "per_query_s": method["query_s"],
                "audit_s": method.get("audit_s", 0.0),
                "total_s": total_s,
                "amortized_s_per_query": total_s / count,
                "candidate_success_rate": method.get("candidate_success_rate"),
                "audit_success_rate": method.get("audit_success_rate"),
                "success_rate": method.get("success_rate"),
                "success_semantics": method.get("success_semantics"),
                "source": method.get("source"),
            })
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "method",
        "class",
        "queries",
        "build_s",
        "per_query_s",
        "audit_s",
        "total_s",
        "amortized_s_per_query",
        "candidate_success_rate",
        "audit_success_rate",
        "success_rate",
        "success_semantics",
        "source",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def maybe_plot(path: Path | None, rows: list[dict[str, Any]], *, title: str) -> str | None:
    if path is None:
        return None
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return None
    path.parent.mkdir(parents=True, exist_ok=True)
    by_method: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        by_method.setdefault(row["method"], []).append(row)
    fig, ax = plt.subplots(figsize=(5.2, 3.2))
    for method, group in sorted(by_method.items()):
        group = sorted(group, key=lambda row: row["queries"])
        ax.plot(
            [row["queries"] for row in group],
            [row["amortized_s_per_query"] for row in group],
            marker="o",
            linewidth=1.8,
            label=method,
        )
    ax.set_xlabel("number of queries")
    ax.set_ylabel("amortized audited time / query (s)")
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    if any(row["amortized_s_per_query"] > 0 for row in rows):
        ax.set_yscale("log")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return str(path)


def parse_counts(text: str) -> list[int]:
    counts = sorted({int(value.strip()) for value in text.split(",") if value.strip()})
    if not counts or counts[0] <= 0:
        raise argparse.ArgumentTypeError("query counts must be positive integers")
    return counts


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate query-amortization JSON/CSV/figure from paper artifacts.")
    parser.add_argument("--sbf-json", type=Path, default=None, help="SBF artifact. Defaults to first known Exp.4 artifact.")
    parser.add_argument("--baseline-json", type=Path, action="append", default=[], help="Optional baseline artifact; may be repeated.")
    parser.add_argument("--no-default-baselines", action="store_true", help="Only use baselines passed with --baseline-json.")
    parser.add_argument("--counts", type=parse_counts, default=parse_counts("1,5,10,20,50"))
    parser.add_argument("--out-json", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--out-csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--fig", type=Path, default=DEFAULT_FIG)
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    sbf_path = args.sbf_json or first_existing([
        PAPER_OUT / "tro2026_exp04_marcucci_full.json",
        PAPER_OUT / "marcucci_combined_standalone.json",
        PAPER_OUT / "exp14_validation_profiles" / "support_hull_keep_kdop.json",
        PAPER_OUT / "exp14_validation_profiles" / "support_hull_coverage.json",
    ])
    methods: list[dict[str, Any]] = []
    if sbf_path is not None:
        payload = load_json(sbf_path)
        if payload is not None:
            methods.append(sbf_method_from_payload("SBF-SH", payload))

    baseline_paths = list(args.baseline_json)
    if not args.no_default_baselines:
        for path in DEFAULT_BASELINES:
            if path.exists() and path not in baseline_paths:
                baseline_paths.append(path)

    for baseline_path in baseline_paths:
        payload = load_json(baseline_path)
        if payload is not None:
            methods.extend(baseline_methods_from_payload(baseline_path, payload))

    if not methods:
        raise SystemExit("No usable SBF or baseline artifacts found. Pass --sbf-json or --baseline-json.")

    rows = amortization_rows(methods, args.counts)
    payload = {
        "experiment": "query_amortization",
        "source_script": str(Path(__file__).resolve()),
        "query_counts": args.counts,
        "baseline_artifacts": [str(path) for path in baseline_paths if path.exists()],
        "methods": methods,
        "rows": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_csv(args.out_csv, rows)
    figure = maybe_plot(None if args.no_plot else args.fig, rows, title="Query amortization")
    print(json.dumps({"out_json": str(args.out_json), "out_csv": str(args.out_csv), "figure": figure, "methods": methods}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
