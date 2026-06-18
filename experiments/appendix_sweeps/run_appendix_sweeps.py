#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, run_id, write_json
from experiments.common.progress import progress


SWEEPS = ["box_budget", "leaf_depth", "deep_ffb_depth", "ffb_start_depth", "sampling_probability", "worker_count", "audit_resolution", "segment_policy", "anchor_policy"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Appendix parameter/sensitivity sweeps.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "appendix")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sweeps = SWEEPS[:1] if args.phase == "smoke" else SWEEPS
    rows = [
        {"sweep": sweep, "status": "planned" if args.dry_run else "backend_pending"}
        for sweep in progress(sweeps, desc="appendix sweeps", total=len(sweeps), disable=bool(args.dry_run))
    ]
    payload: dict[str, Any] = {
        "experiment": "appendix_sweeps",
        "run_id": run_id("appendix"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "backend_pending",
        "environment": environment_metadata(),
        "rows": rows,
    }
    write_json(args.out_dir / "appendix_sweeps_manifest.json", payload)
    print(f"wrote {args.out_dir / 'appendix_sweeps_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
