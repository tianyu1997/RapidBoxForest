#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, write_json
from experiments.common.metrics import median


OLD_PAPER_ROOT_ENV = "RBF_OLD_TRO_PAPER_ROOT"
OLD_OUTPUT_ROOT_ENV = "RBF_OLD_TRO_OUTPUT_ROOT"
OLD_PAPER_ROOT = Path(os.environ[OLD_PAPER_ROOT_ENV]) if OLD_PAPER_ROOT_ENV in os.environ else REPO_ROOT / "external" / "old_tro2026" / "paper"
OLD_OUTPUT_ROOT = Path(os.environ[OLD_OUTPUT_ROOT_ENV]) if OLD_OUTPUT_ROOT_ENV in os.environ else REPO_ROOT / "external" / "old_tro2026" / "outputs"
OLD_TABLE = OLD_PAPER_ROOT / "generated" / "tab_tro_main_shelf_best_tradeoff.tex"
OLD_ANYTIME_JSON = OLD_OUTPUT_ROOT / "tro2026_shelf_anytime_tradeoff_full.json"
OLD_IRIS_JSON = OLD_OUTPUT_ROOT / "tro2026_shelf_iris_np_gcs_anytime.json"
OLD_IRIS_QUICK_JSON = OLD_OUTPUT_ROOT / "marcucci_iris_np_gcs.json"

QUERY_NAMES = ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]
METHOD_COLUMNS = [
    ("old_sbf_sh", "SBF-SH"),
    ("iris_np_gcs", "IRIS-NP+GCS"),
    ("prm", "PRM"),
    ("rrtconnect", "RRTConnect"),
    ("bitstar", "BIT*"),
]
REUSED_METHODS = {"iris_np_gcs", "prm", "rrtconnect", "bitstar"}
BUILD_RE = re.compile(r"(?P<method>SBF-SH|IRIS-NP\+GCS|PRM).*?Build (?P<build>[0-9.]+)\\,s")


def file_sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_float(raw: str) -> float:
    raw = raw.strip()
    if raw in {"", "--"}:
        return math.nan
    return float(raw)


def normalize_query(raw: str) -> str:
    return (
        raw.replace("$", "")
        .replace(r"\rightarrow", "->")
        .replace(" ", "")
        .strip()
    )


def parse_build_headers(table_text: str) -> dict[str, float]:
    builds: dict[str, float] = {"rrtconnect": 0.0, "bitstar": 0.0}
    name_map = {"SBF-SH": "old_sbf_sh", "IRIS-NP+GCS": "iris_np_gcs", "PRM": "prm"}
    for match in BUILD_RE.finditer(table_text):
        builds[name_map[match.group("method")]] = float(match.group("build"))
    return builds


def parse_old_table(table_path: Path) -> tuple[list[dict[str, Any]], dict[str, float]]:
    text = table_path.read_text(encoding="utf-8")
    builds = parse_build_headers(text)
    rows: list[dict[str, Any]] = []
    for line in text.splitlines():
        if r"\rightarrow" not in line or "&" not in line:
            continue
        cells = [cell.strip() for cell in line.rstrip("\\").split("&")]
        if len(cells) < 11:
            continue
        query = normalize_query(cells[0])
        for method_index, (method, label) in enumerate(METHOD_COLUMNS):
            offset = 1 + 2 * method_index
            query_s = parse_float(cells[offset])
            path_length = parse_float(cells[offset + 1])
            rows.append(
                {
                    "method": method,
                    "method_label": label,
                    "query": query,
                    "query_s": query_s,
                    "path_length": path_length,
                    "build_s": builds.get(method, 0.0),
                    "audit_segment_step": 0.01,
                    "source": "old_tro_table_common_rule",
                }
            )
    return rows, builds


def audit_reuse(
    table_path: Path,
    anytime_path: Path,
    iris_path: Path,
    iris_quick_path: Path,
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    for label, path in [
        ("old_table", table_path),
        ("old_anytime_json", anytime_path),
        ("old_iris_json", iris_path),
        ("old_iris_quick_json", iris_quick_path),
    ]:
        checks.append(
            {
                "name": f"{label}_exists",
                "status": "pass" if path.exists() else "fail",
                "path": str(path),
                "sha256": file_sha256(path),
            }
        )
    anytime = load_json(anytime_path) if anytime_path.exists() else {}
    iris = load_json(iris_path) if iris_path.exists() else {}
    iris_quick = load_json(iris_quick_path) if iris_quick_path.exists() else {}
    audit_protocol = anytime.get("audit_protocol", {})
    checks.extend(
        [
            {
                "name": "scene_matches_shelf_iiwa_marcucci",
                "status": "pass" if str(anytime.get("scene")) == "shelf_iiwa_marcucci_combined" else "fail",
                "value": anytime.get("scene"),
            },
            {
                "name": "query_set_matches_current_shelf",
                "status": "pass" if list(anytime.get("task_names", [])) == QUERY_NAMES else "fail",
                "value": anytime.get("task_names"),
            },
            {
                "name": "final_audit_step_is_0p01",
                "status": "pass" if float(audit_protocol.get("path_audit_segment_step", -1.0)) == 0.01 else "fail",
                "value": audit_protocol.get("path_audit_segment_step"),
            },
            {
                "name": "ompl_planner_step_is_0p01",
                "status": "pass" if float(audit_protocol.get("planner_segment_step", -1.0)) == 0.01 else "fail",
                "value": audit_protocol.get("planner_segment_step"),
            },
            {
                "name": "old_table_contains_all_queries",
                "status": "pass" if all(q in table_path.read_text(encoding="utf-8") for q in [r"AS$\rightarrow$TS", r"TS$\rightarrow$CS", r"CS$\rightarrow$LB", r"LB$\rightarrow$RB", r"RB$\rightarrow$AS"]) else "fail",
            },
            {
                "name": "iris_artifact_uses_segment_step_0p01",
                "status": "pass"
                if float(iris.get("params", {}).get("segment_step", iris_quick.get("params", {}).get("segment_step", -1.0))) == 0.01
                else "warn",
                "value": iris.get("params", {}).get("segment_step", iris_quick.get("params", {}).get("segment_step")),
            },
        ]
    )
    records = list(anytime.get("records", []))
    point_methods = {str(point.get("method")) for point in anytime.get("summary", {}).get("points", [])}
    checks.append(
        {
            "name": "anytime_artifact_has_required_non_rbf_methods",
            "status": "pass"
            if {"ompl_prm", "ompl_rrtconnect", "ompl_bitstar"}.issubset(point_methods)
            else "fail",
            "value": sorted(point_methods),
        }
    )
    checks.append(
        {
            "name": "raw_records_include_final_audited_tasks",
            "status": "pass"
            if any(task.get("audit_passed") for record in records for task in record.get("raw_tasks", []))
            else "fail",
        }
    )
    reusable = all(check["status"] == "pass" for check in checks if check["name"] != "iris_artifact_uses_segment_step_0p01")
    return {
        "status": "reusable" if reusable else "not_reusable",
        "checks": checks,
        "old_protocol_summary": {
            "scene": anytime.get("scene"),
            "task_names": anytime.get("task_names"),
            "audit_protocol": audit_protocol,
            "method_build_query_semantics": anytime.get("method_build_query_semantics"),
            "iris_params": iris.get("params", iris_quick.get("params")),
        },
        "reuse_policy": {
            "reuse": sorted(REUSED_METHODS),
            "excluded": ["old_sbf_sh"],
            "reason": "Current RBF rows must be regenerated with the current leaf-sweep + partition-native profile; old SBF-SH is only used as historical context.",
        },
    }


def aggregate_method(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for method in ["iris_np_gcs", "prm", "rrtconnect", "bitstar"]:
        items = [row for row in rows if row["method"] == method]
        if not items:
            continue
        out.append(
            {
                "method": method,
                "method_label": items[0]["method_label"],
                "source": "imported_old_tro2026_common_rule",
                "runs": 1,
                "success_runs": 1 if len(items) == len(QUERY_NAMES) else 0,
                "query_count": len(items),
                "build_s": median(row["build_s"] for row in items),
                "query_s_median": median(row["query_s"] for row in items),
                "planning_s_median": median(row["query_s"] for row in items),
                "audit_s_median": 0.0,
                "path_length_median": median(row["path_length"] for row in items),
                "path_length_mean": sum(row["path_length"] for row in items) / len(items),
                "raw_segment_fraction_median": 0.0,
                "status": "imported_reusable",
            }
        )
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row.keys()})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(path: Path, payload: dict[str, Any], summary_rows: list[dict[str, Any]]) -> None:
    lines = [
        "# Old Shelf Baseline Reuse Audit",
        "",
        f"Status: **{payload['audit']['status']}**",
        "",
        "## Checks",
        "",
        "| Check | Status | Value |",
        "|---|---:|---|",
    ]
    for check in payload["audit"]["checks"]:
        value = check.get("value", check.get("path", ""))
        lines.append(f"| {check['name']} | {check['status']} | `{value}` |")
    lines.extend(["", "## Imported Rows", "", "| Method | Build (s) | Query median (s) | Path median |", "|---|---:|---:|---:|"])
    for row in summary_rows:
        lines.append(
            f"| {row['method_label']} | {row['build_s']:.3g} | {row['query_s_median']:.3g} | {row['path_length_mean']:.3g} |"
        )
    lines.extend(
        [
            "",
            "Old SBF-SH is intentionally excluded. Current RBF is regenerated with the current leaf-sweep + partition-native implementation.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def import_old_baselines(
    out_dir: Path,
    old_paper_root: Path = OLD_PAPER_ROOT,
    old_output_root: Path = OLD_OUTPUT_ROOT,
) -> dict[str, Any]:
    table_path = old_paper_root / "generated" / "tab_tro_main_shelf_best_tradeoff.tex"
    anytime_path = old_output_root / "tro2026_shelf_anytime_tradeoff_full.json"
    iris_path = old_output_root / "tro2026_shelf_iris_np_gcs_anytime.json"
    iris_quick_path = old_output_root / "marcucci_iris_np_gcs.json"
    per_query_rows, builds = parse_old_table(table_path)
    reused_per_query = [row for row in per_query_rows if row["method"] in REUSED_METHODS]
    summary_rows = aggregate_method(reused_per_query)
    audit = audit_reuse(table_path, anytime_path, iris_path, iris_quick_path)
    payload = {
        "experiment": "exp05_old_shelf_baseline_reuse",
        "environment": environment_metadata(),
        "old_paper_root": str(old_paper_root),
        "old_output_root": str(old_output_root),
        "build_headers": builds,
        "audit": audit,
        "per_query_rows": reused_per_query,
        "summary": summary_rows,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    write_json(out_dir / "old_shelf_baseline_reuse_audit.json", payload)
    write_csv(out_dir / "old_shelf_baselines_per_query.csv", reused_per_query)
    write_csv(out_dir / "old_shelf_baselines_summary.csv", summary_rows)
    write_markdown(out_dir / "old_shelf_baseline_reuse_summary.md", payload, summary_rows)
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit and import old TRO Shelf+IIWA non-RBF baselines.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp05")
    parser.add_argument("--old-paper-root", type=Path, default=OLD_PAPER_ROOT)
    parser.add_argument("--old-output-root", type=Path, default=OLD_OUTPUT_ROOT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = import_old_baselines(args.out_dir, args.old_paper_root, args.old_output_root)
    print(f"reuse status: {payload['audit']['status']}")
    print(f"wrote {args.out_dir / 'old_shelf_baseline_reuse_audit.json'}")
    return 0 if payload["audit"]["status"] == "reusable" else 2


if __name__ == "__main__":
    raise SystemExit(main())
