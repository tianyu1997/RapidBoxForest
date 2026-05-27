#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    environment_metadata,
    namespace_dict,
    proc_status,
    run_command,
    run_id,
    write_json,
)


DEFAULT_OUT_DIR = DEFAULT_OUTPUT_ROOT / "exp03_lect_microbench_shelf_iiwa_persisted_cache"
DEFAULT_BENCHMARK_BIN = REPO_ROOT / "build-exp03-lect-bench" / "lect_database_benchmark"
DEFAULT_CACHE_PATH = (
    REPO_ROOT
    / "safe_box_forest"
    / "experiments"
    / "outputs"
    / "paper"
    / "rbf_only"
    / "cache"
    / "e5_lifelong_cache_link_d18_smoke"
)
DEFAULT_E5_ARTIFACT = (
    REPO_ROOT
    / "safe_box_forest"
    / "experiments"
    / "outputs"
    / "paper"
    / "rbf_only"
    / "e5_lifelong_cache_mechanism_smoke.json"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Exp.3 benchmark on the real Shelf+IIWA persisted cache.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--csv-path", type=Path, default=None)
    parser.add_argument("--benchmark-bin", type=Path, default=DEFAULT_BENCHMARK_BIN)
    parser.add_argument("--cache-path", type=Path, default=DEFAULT_CACHE_PATH)
    parser.add_argument("--e5-artifact", type=Path, default=DEFAULT_E5_ARTIFACT)
    parser.add_argument("--queries", type=int, default=20000)
    parser.add_argument("--evidence-records", type=int, default=20000)
    parser.add_argument("--compact-delete-ops", type=int, default=0)
    parser.add_argument("--no-verify", action="store_true", default=False)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def directory_size(path: Path | None) -> int:
    if path is None or not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def read_manifest(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, sep, value = line.partition("=")
        if sep:
            values[key] = value
    return values


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def coerce_cell(value: str) -> Any:
    text = value.strip()
    if text == "":
        return text
    try:
        if any(ch in text for ch in ".eE"):
            return float(text)
        return int(text)
    except ValueError:
        return text


def read_stage_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [{key: coerce_cell(value) for key, value in row.items()} for row in csv.DictReader(handle)]


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "lect_microbench.json")
    csv_path = args.csv_path or (args.out_dir / "stages.csv")
    command = [
        str(args.benchmark_bin),
        "--db",
        str(args.cache_path),
        "--existing-db",
        "--csv",
        str(csv_path),
        "--queries",
        str(max(1, int(args.queries))),
        "--evidence-records",
        str(max(1, int(args.evidence_records))),
    ]
    if int(args.compact_delete_ops) > 0:
        command.extend(["--compact-delete-ops", str(int(args.compact_delete_ops))])
    if args.no_verify:
        command.append("--no-verify")

    measurement = run_command(command, cwd=REPO_ROOT, dry_run=bool(args.dry_run))
    rows = read_stage_rows(csv_path)
    e5_payload = load_json(args.e5_artifact)
    manifest = read_manifest(args.cache_path / "manifest.json")
    payload = {
        "experiment": "exp03_lect_microbench_shelf_iiwa_persisted_cache",
        "run_id": run_id("exp03_shelf_cache"),
        "status": "dry_run" if args.dry_run else ("ok" if measurement.get("returncode") == 0 else "failed"),
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": {
            "path": str(args.cache_path),
            "bytes": directory_size(args.cache_path),
            "manifest": manifest,
            "proc_status_at_manifest": proc_status(),
            "e5_artifact_path": str(args.e5_artifact),
            "e5_summary": {
                "ok": bool(e5_payload.get("ok")),
                "cache_bytes": e5_payload.get("cache_bytes"),
                "first_prewarm_wall_s": ((e5_payload.get("first_prewarm") or {}).get("wall_s")),
                "reopen_prewarm_wall_s": ((e5_payload.get("reopen_prewarm") or {}).get("wall_s")),
                "reused_endpoint_cache": ((e5_payload.get("reopen_prewarm") or {}).get("reused_endpoint_cache")),
            },
        },
        "measurement": measurement,
        "rows": rows,
        "notes": [
            "Benchmarks the current-format Shelf+IIWA d18 persisted cache with fresh reopen per stage.",
            "The benchmark binary performs checkpoint/compact on copied sibling directories, not on the source cache namespace.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0 if args.dry_run or measurement.get("returncode") == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
