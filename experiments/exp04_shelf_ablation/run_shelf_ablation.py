#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
    run_command,
    run_id,
    write_json,
)


SBF_COMBINED = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_marcucci_combined.py"


def legacy_sbf_env() -> dict[str, str]:
    candidates = [
        REPO_ROOT / "build-rbf-only-exec",
        REPO_ROOT / "build-consolidated-python",
    ]
    build_dir = next((candidate for candidate in candidates if (candidate / "python" / "sbf").exists()), None)
    if build_dir is None:
        return {}
    return {"SBF_BUILD_DIR": str(build_dir)}


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp04_shelf_ablation"
    parser = argparse.ArgumentParser(description="Run Experiment 4 shelf planning ablation matrix.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--only", default="all", help="Comma-separated ablation names or all.")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def combined_command(args: argparse.Namespace, name: str, preset: str, envelope: str, threads: int, split_policy: str) -> list[str]:
    command = [
        sys.executable,
        str(SBF_COMBINED),
        "--out-json",
        str(args.out_dir / f"{name}.json"),
        "--database-path",
        str(args.out_dir / "cache" / name),
        "--preset",
        preset,
        "--envelope",
        envelope,
        "--threads",
        str(threads),
        "--seeds",
        str(args.seeds),
        "--timeout-ms",
        str(args.timeout_ms),
        "--split-policy",
        split_policy,
        "--strict-path-audit",
        "--audit-resolution",
        "32",
    ]
    if envelope == "support_hull":
        command.append("--no-support-hull-keep-kdop")
    return command


def matrix(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows = [
        {
            "name": "baseline_ifk_support_hull_8t_best_tighten",
            "factor": "baseline_proxy",
            "expected_baseline": "warm+AAFK+SupportHull+8threads+AAFKVolumeMin",
            "supported": True,
            "limitations": "legacy proxy uses IFK strict and best-tighten, not full warm LECT AAFKVolumeMin yet",
            "command": combined_command(args, "baseline_ifk_support_hull_8t_best_tighten", "ifk_strict", "support_hull", int(args.threads), "best-tighten"),
        },
        {
            "name": "no_lect_cache_online_envelopes",
            "factor": "cache",
            "supported": False,
            "requires_new_hook": "disable LectCacheSession and force online envelope materialization",
            "command": [],
        },
        {
            "name": "no_aafk_critsample_support_hull",
            "factor": "endpoint_source",
            "supported": True,
            "command": combined_command(args, "no_aafk_critsample_support_hull", "support_hull_coverage", "support_hull", int(args.threads), "best-tighten"),
        },
        {
            "name": "envelope_aabb_only",
            "factor": "envelope_collision",
            "supported": True,
            "command": combined_command(args, "envelope_aabb_only", "ifk_strict", "link", int(args.threads), "best-tighten"),
        },
        {
            "name": "envelope_aabb_to_support_hull_chain",
            "factor": "envelope_collision",
            "supported": True,
            "implementation": "legacy support_hull already runs shared AABB broadphase followed by SupportHull narrow phase when support_hull_keep_kdop=false",
            "command": combined_command(args, "envelope_aabb_to_support_hull_chain", "ifk_strict", "support_hull", int(args.threads), "best-tighten"),
        },
        {
            "name": "single_thread",
            "factor": "threads",
            "supported": True,
            "command": combined_command(args, "single_thread", "ifk_strict", "support_hull", 1, "best-tighten"),
        },
        {
            "name": "round_robin_split_policy",
            "factor": "split_policy",
            "supported": False,
            "requires_new_hook": "bind or expose true round-robin split policy; legacy closest option is widest-first",
            "command": [],
        },
    ]
    wanted = set(csv_list(args.only))
    if wanted and "all" not in wanted:
        rows = [row for row in rows if row["name"] in wanted]
    return rows


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "shelf_ablation_manifest.json")
    rows = matrix(args)
    run_records = []
    extra_env = legacy_sbf_env()
    if args.execute:
        for row in rows:
            if not row.get("supported"):
                run_records.append({"name": row["name"], "skipped": True, "reason": row.get("requires_new_hook")})
                continue
            run_records.append({
                "name": row["name"],
                "measurement": run_command(row["command"], dry_run=bool(args.dry_run), extra_env=extra_env),
            })
    payload = {
        "experiment": "exp04_shelf_ablation",
        "run_id": run_id("exp04"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "matrix": rows,
        "runs": run_records,
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
