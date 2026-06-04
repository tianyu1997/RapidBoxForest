#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    csv_list,
    environment_metadata,
    namespace_dict,
    proc_status,
    run_command,
    run_id,
    write_json,
)


DEFAULT_OPERATIONS = "load,query,split,save,materialize,spill"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp03_lect_microbench"
    parser = argparse.ArgumentParser(description="Run Experiment 3 LECT microbench harness.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--cache-path", type=Path, default=None)
    parser.add_argument("--operations", default=DEFAULT_OPERATIONS)
    parser.add_argument(
        "--case",
        action="append",
        default=[],
        help="Case as operation:name::command, for example load:cold::python3 tool.py --load cache. Repeatable.",
    )
    parser.add_argument("--iterations", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def parse_case(spec: str) -> dict[str, Any]:
    head, sep, command_text = spec.partition("::")
    if not sep:
        raise ValueError(f"case must be operation:name::command, got {spec!r}")
    operation, sep, name = head.partition(":")
    if not sep:
        raise ValueError(f"case head must be operation:name, got {head!r}")
    return {"operation": operation.strip(), "name": name.strip(), "command": shlex.split(command_text)}


def directory_size(path: Path | None) -> int:
    if path is None or not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "lect_microbench.json")
    operations = set(csv_list(args.operations))
    cases = [parse_case(item) for item in args.case]
    rows: list[dict[str, Any]] = []
    for case in cases:
        if case["operation"] not in operations:
            continue
        command = case["command"]
        for iteration in range(max(1, int(args.iterations))):
            rows.append({
                "operation": case["operation"],
                "name": case["name"],
                "iteration": int(iteration),
                "measurement": run_command(command, dry_run=bool(args.dry_run)),
            })
    payload = {
        "experiment": "exp03_lect_microbench",
        "run_id": run_id("exp03"),
        "status": "dry_run" if args.dry_run else ("ok" if rows else "manifest_only"),
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": {
            "path": str(args.cache_path) if args.cache_path else None,
            "bytes": directory_size(args.cache_path),
            "proc_status_at_manifest": proc_status(),
        },
        "operations": sorted(operations),
        "rows": rows,
        "notes": [
            "This harness records external commands until native LECT operation hooks are exposed.",
            "Use separate --case entries for load/query/split/save to keep timings phase-pure.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
