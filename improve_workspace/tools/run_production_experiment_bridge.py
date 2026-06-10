#!/usr/bin/env python3
"""Bridge from improve_workspace to production Exp.4/Exp.6 runners.

The C-LECT implementation in this workspace is a sidecar/prototype. This tool
does not claim full production C-LECT performance. In default dry-run mode it
verifies that the production experiment runners can generate the shelf/random
manifests needed by the improve.md Section 7 experiment plan. In executed-smoke
mode it runs one minimal production smoke point and records measured runner
metrics separately from the sidecar C-LECT benchmarks.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]


def run(cmd: list[str], cwd: Path) -> dict[str, Any]:
    proc = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, check=False)
    return {
        "cmd": cmd,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }


def read_json_if_present(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - diagnostic path
        return {"_read_error": str(exc)}


def _finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or number in (float("inf"), float("-inf")):
        return None
    return number


def _row_metrics(row: dict[str, Any]) -> dict[str, Any]:
    keys = [
        "success_count",
        "query_count",
        "planning_s",
        "planning_s_median",
        "planning_total_s",
        "offline_build_s",
        "offline_build_s_median",
        "online_batch_s",
        "online_batch_s_median",
        "online_per_query_s",
        "online_per_query_s_median",
        "audit_s",
        "audit_s_median",
        "path_length_mean",
        "raw_segment_fraction",
        "raw_segment_fraction_median",
    ]
    out: dict[str, Any] = {}
    for key in keys:
        if key in row:
            value = _finite_number(row.get(key))
            out[key] = value if value is not None else row.get(key)
    return out


def compact_manifest(path: Path, manifest: dict[str, Any] | None) -> dict[str, Any]:
    if manifest is None:
        return {
            "path": str(path),
            "exists": False,
            "status": "missing",
        }
    planned = manifest.get("planned_rows", [])
    rows = manifest.get("rows", [])
    summary = manifest.get("summary", [])
    config = manifest.get("config") or {}
    offline_query_agnostic = (
        config.get("offline_query_agnostic_build")
        if isinstance(config, dict)
        else _first_row_value(planned, "offline_query_agnostic_build")
    )
    if offline_query_agnostic is None:
        offline_query_agnostic = _first_row_value(planned, "offline_query_agnostic_build")
    warnings: list[str] = []
    if offline_query_agnostic is None:
        warnings.append("offline_query_agnostic_build_not_declared")
    return {
        "path": str(path),
        "exists": True,
        "experiment": manifest.get("experiment"),
        "phase": manifest.get("phase"),
        "status": manifest.get("status"),
        "planned_rows": len(planned) if isinstance(planned, list) else None,
        "executed_rows": len(rows) if isinstance(rows, list) else None,
        "summary_rows": len(summary) if isinstance(summary, list) else None,
        "canonical_mapping_scope": (
            config.get("canonical_mapping_scope")
            or manifest.get("rbf_default_profile", {}).get("canonical_mapping_scope")
            or _first_row_value(planned, "canonical_mapping_scope")
        ),
        "offline_query_agnostic_build": offline_query_agnostic,
        "row_metrics": _row_metrics(rows[0]) if isinstance(rows, list) and rows and isinstance(rows[0], dict) else {},
        "summary_metrics": _row_metrics(summary[0]) if isinstance(summary, list) and summary and isinstance(summary[0], dict) else {},
        "warnings": warnings,
    }


def _first_row_value(rows: Any, key: str) -> Any:
    if isinstance(rows, list) and rows and isinstance(rows[0], dict):
        return rows[0].get(key)
    return None


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Production Experiment Bridge",
        "",
        "This artifact verifies production experiment runner readiness from",
        "`improve_workspace`. Dry-run mode checks manifests; executed-smoke mode",
        "records a minimal production runner measurement. It is not a full",
        "production C-LECT performance result.",
        "",
        f"- Mode: `{payload['mode']}`",
        f"- Production C-LECT integrated: `{payload['production_clect_integrated']}`",
        f"- Sidecar-only implementation: `{payload['sidecar_only']}`",
        f"- Overall bridge ok: `{payload['ok']}`",
        "",
        "| Experiment | Manifest | Status | Planned | Rows | Summary | Canonical scope | Query-agnostic build | Warnings |",
        "| --- | --- | --- | ---: | ---: | ---: | --- | --- | --- |",
    ]
    for item in payload["manifests"]:
        lines.append(
            f"| `{item.get('experiment')}` | `{item.get('path')}` | `{item.get('status')}` | "
            f"{item.get('planned_rows')} | {item.get('executed_rows')} | {item.get('summary_rows')} | "
            f"`{item.get('canonical_mapping_scope')}` | "
            f"`{item.get('offline_query_agnostic_build')}` | "
            f"`{','.join(item.get('warnings') or [])}` |"
        )
    lines.extend([
        "",
        "## Smoke Metrics",
        "",
        "| Experiment | Source | Metrics |",
        "| --- | --- | --- |",
    ])
    for item in payload["manifests"]:
        metrics = item.get("summary_metrics") or item.get("row_metrics") or {}
        lines.append(
            f"| `{item.get('experiment')}` | "
            f"`{'summary' if item.get('summary_metrics') else 'row' if item.get('row_metrics') else 'none'}` | "
            f"`{json.dumps(metrics, ensure_ascii=False, sort_keys=True)}` |"
        )
    lines.extend([
        "",
        "## Commands",
        "",
    ])
    for result in payload["commands"]:
        lines.extend([
            "```text",
            " ".join(result["cmd"]),
            f"returncode={result['returncode']}",
            "```",
            "",
        ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=Path("improve_workspace/production_bridge"))
    parser.add_argument("--json-out", type=Path, default=Path("improve_workspace/production_experiment_bridge.json"))
    parser.add_argument("--md-out", type=Path, default=Path("improve_workspace/production_experiment_bridge.md"))
    parser.add_argument("--mode", choices=["dry-run", "executed-smoke"], default="dry-run")
    parser.add_argument(
        "--exp06-catalog",
        type=Path,
        default=Path("improve_workspace/production_bridge/catalogs/exp06_iiwa_easy_smoke_catalog.json"),
        help="Saved Exp.6 catalog fixture used for dry-run/executed smoke reuse.",
    )
    args = parser.parse_args()

    out_dir = REPO_ROOT / args.out_dir
    mode_suffix = "executed" if args.mode == "executed-smoke" else "dry_run"
    exp04_dir = out_dir / f"exp04_shelf_smoke_{mode_suffix}"
    exp06_dir = out_dir / f"exp06_random_smoke_{mode_suffix}"
    exp04_dir.mkdir(parents=True, exist_ok=True)
    exp06_dir.mkdir(parents=True, exist_ok=True)

    exp04_cmd = [
        sys.executable,
        "experiments/exp04_shelf_leaf_rrt/run_shelf_leaf_rrt.py",
        "--phase",
        "smoke",
        "--out-dir",
        str(exp04_dir),
        "--only",
        "baseline_d23_aafk_support_hull_8t",
        "--seeds",
        "0",
        "--box-budgets",
        "400",
    ]
    exp06_cmd = [
        sys.executable,
        "experiments/exp06_random_robot/run_random_robot.py",
        "--phase",
        "smoke",
        "--out-dir",
        str(exp06_dir),
        "--methods",
        "sbf_leaf_rrt",
        "--robots",
        "iiwa",
        "--difficulties",
        "easy",
        "--scene-seeds",
        "1",
        "--queries-per-scene",
        "3",
        "--scene-catalog",
        str(REPO_ROOT / args.exp06_catalog),
        "--scene-catalog-mode",
        "reuse",
        "--skip-lect-cache-ensure",
    ]
    if args.mode == "dry-run":
        exp04_cmd.insert(4, "--dry-run")
        exp06_cmd.insert(4, "--dry-run")
    commands = [
        exp04_cmd,
        exp06_cmd,
    ]
    results = [run(cmd, REPO_ROOT) for cmd in commands]

    manifest_paths = [
        exp04_dir / "shelf_leaf_rrt_manifest.json",
        exp06_dir / "random_robot_manifest.json",
    ]
    manifests = [
        compact_manifest(path, read_json_if_present(path))
        for path in manifest_paths
    ]
    ok = all(result["returncode"] == 0 for result in results) and all(item["exists"] for item in manifests)
    payload = {
        "ok": ok,
        "mode": args.mode,
        "production_clect_integrated": False,
        "sidecar_only": True,
        "scope": (
            "Production Exp.4/Exp.6 smoke manifest bridge. "
            "Executed-smoke metrics are minimal runner checks, not full production C-LECT performance."
        ),
        "commands": results,
        "manifests": manifests,
    }
    json_path = REPO_ROOT / args.json_out
    md_path = REPO_ROOT / args.md_out
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    write_md(md_path, payload)
    print(json.dumps({
        "ok": ok,
        "json_out": str(json_path),
        "md_out": str(md_path),
        "manifests": [item["path"] for item in manifests],
    }, indent=2, ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
