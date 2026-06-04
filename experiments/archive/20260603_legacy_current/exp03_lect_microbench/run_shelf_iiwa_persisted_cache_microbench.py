#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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
from experiments.common.shelf_iiwa_cache import (  # noqa: E402
    DEFAULT_P18_CACHE_LABEL,
    cache_file_sizes,
    directory_size,
    ensure_p18_prewarm_summary,
    read_manifest,
    snapshot_path_for_cache,
    snapshot_summary,
)


DEFAULT_OUT_DIR = DEFAULT_OUTPUT_ROOT / "exp03_lect_microbench_shelf_iiwa_persisted_cache"
DEFAULT_CACHE_PATH = DEFAULT_OUT_DIR / "cache" / DEFAULT_P18_CACHE_LABEL
DEFAULT_PREWARM_JSON = DEFAULT_OUT_DIR / "p18_prewarm.json"


def default_benchmark_bin() -> Path:
    candidates = [
        REPO_ROOT / "build-consolidated-python" / "lect_database" / "lect_database_benchmark",
        REPO_ROOT / "build-consolidated-sbf-tests" / "lect_database" / "lect_database_benchmark",
        REPO_ROOT / "build-rbf-only-exec" / "lect_database" / "lect_database_benchmark",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


DEFAULT_BENCHMARK_BIN = default_benchmark_bin()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Exp.3 benchmark on the real Shelf+IIWA persisted cache.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--prewarm-json", type=Path, default=DEFAULT_PREWARM_JSON)
    parser.add_argument("--csv-path", type=Path, default=None)
    parser.add_argument("--benchmark-bin", type=Path, default=DEFAULT_BENCHMARK_BIN)
    parser.add_argument("--cache-path", type=Path, default=DEFAULT_CACHE_PATH)
    parser.add_argument("--snapshot-path", type=Path, default=None)
    parser.add_argument("--read-path", choices=["snapshot", "legacy"], default="snapshot")
    parser.add_argument("--reuse-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--rbf-envelope", choices=["link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--queries", type=int, default=20000)
    parser.add_argument("--evidence-records", type=int, default=20000)
    parser.add_argument("--compact-delete-ops", type=int, default=0)
    parser.add_argument(
        "--verify-after-reopen",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Include strict verify in the final reopen stage. Disabled by default so the benchmark measures reopen cost rather than full consistency checking.",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()

def resolve_benchmark_bin(path: Path) -> Path:
    if path.exists():
        return path
    raise FileNotFoundError(
        f"lect_database_benchmark not found at {path}; build one under build-consolidated-python or build-consolidated-sbf-tests"
    )
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
    snapshot_path = args.snapshot_path or snapshot_path_for_cache(args.cache_path)
    prewarm_summary = ensure_p18_prewarm_summary(
        args.prewarm_json,
        cache_path=args.cache_path,
        prewarm_depth=int(args.prewarm_depth),
        envelope=str(args.rbf_envelope),
        prewarm_threads=int(args.prewarm_threads),
        clean_cache=bool(args.clean_cache),
        dry_run=bool(args.dry_run),
    )

    command = [
        str(resolve_benchmark_bin(args.benchmark_bin)),
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
    if args.read_path == "snapshot":
        command.extend(["--snapshot", "--snapshot-path", str(snapshot_path)])
        if args.reuse_snapshot:
            command.append("--reuse-snapshot")
    if args.read_path == "legacy" and int(args.compact_delete_ops) > 0:
        command.extend(["--compact-delete-ops", str(int(args.compact_delete_ops))])
    if args.read_path == "legacy" and not args.verify_after_reopen:
        command.append("--no-verify")

    measurement = run_command(command, cwd=REPO_ROOT, dry_run=bool(args.dry_run))
    rows = [] if args.dry_run else read_stage_rows(csv_path)
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
            "file_sizes": cache_file_sizes(args.cache_path),
            "manifest": manifest,
            "proc_status_at_manifest": proc_status(),
            "prewarm_json_path": str(args.prewarm_json),
            "prewarm_summary": prewarm_summary,
            "snapshot": snapshot_summary(snapshot_path),
        },
        "measurement": measurement,
        "rows": rows,
        "notes": [
            "Benchmarks a freshly rebuilt Shelf+IIWA p18 persisted cache with the selected read path.",
            "The default read path is the immutable LectReadSnapshot generation built from the endpoint-only cache.",
            "The benchmark binary performs checkpoint/compact on copied sibling directories, not on the source cache namespace.",
            "LECT DB stores endpoint-envelope payloads only; link envelopes are reconstructed online from endpoint envelopes.",
            "The final reopen stage defaults to read-only reopen without strict verify so the timing reflects performance rather than full database auditing.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0 if args.dry_run or measurement.get("returncode") == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
