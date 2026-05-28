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
    default_sbf_subprocess_env,
    environment_metadata,
    namespace_dict,
    run_command,
    run_id,
    write_json,
)
from experiments.common.shelf_iiwa_cache import (  # noqa: E402
    DEFAULT_P18_CACHE_LABEL,
    cache_file_sizes,
    directory_size,
    ensure_p18_prewarm_summary,
    load_json,
    read_manifest,
    snapshot_path_for_cache,
    snapshot_summary,
)


SBF_COMBINED = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_marcucci_combined.py"
SBF_SHELF_MAIN = REPO_ROOT / "safe_box_forest" / "experiments" / "rbf_only_shelf_iiwa_main.py"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp04_shelf_ablation"
    parser = argparse.ArgumentParser(description="Run Experiment 4 shelf planning ablation matrix.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--prewarm-json", type=Path, default=output_dir / "p18_prewarm.json")
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--prewarm-envelope", choices=["link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--only", default="all", help="Comma-separated ablation names or all.")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--rbf-cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument("--warm-cache-label", default=DEFAULT_P18_CACHE_LABEL)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--external-evidence-mode", choices=["legacy", "snapshot"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
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
        "--rbf-cache-root",
        str(args.rbf_cache_root),
        "--warm-cache-label",
        str(args.warm_cache_label),
        "--external-evidence-mode",
        str(args.external_evidence_mode),
        "--use-external-evidence",
        "--split-policy",
        split_policy,
        "--strict-path-audit",
        "--audit-resolution",
        "32",
    ]
    if args.external_evidence_auto_build_snapshot:
        command.append("--external-evidence-auto-build-snapshot")
    else:
        command.append("--no-external-evidence-auto-build-snapshot")
    if envelope == "support_hull":
        command.append("--no-support-hull-keep-kdop")
    return command


def seeds_list_arg(seed_count: int) -> str:
    return ",".join(str(index) for index in range(max(1, int(seed_count))))


def shelf_baseline_command(args: argparse.Namespace, name: str, threads: int) -> list[str]:
    command = [
        sys.executable,
        str(SBF_SHELF_MAIN),
        "--out-json",
        str(args.out_dir / f"{name}.json"),
        "--modes",
        "warm_d18",
        "--seeds-list",
        seeds_list_arg(args.seeds),
        "--threads",
        str(threads),
        "--timeout-ms",
        str(args.timeout_ms),
        "--rbf-prewarm-depth",
        "18",
        "--rbf-envelope",
        "support_hull",
        "--rbf-cache-root",
        str(args.rbf_cache_root),
        "--warm-cache-label",
        str(args.warm_cache_label),
        "--external-evidence-mode",
        str(args.external_evidence_mode),
        "--no-support-hull-keep-kdop",
    ]
    if args.external_evidence_auto_build_snapshot:
        command.append("--external-evidence-auto-build-snapshot")
    else:
        command.append("--no-external-evidence-auto-build-snapshot")
    return command


def matrix(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows = [
        {
            "name": "baseline_ifk_support_hull_8t_best_tighten",
            "factor": "baseline",
            "expected_baseline": "warm+AAFK+SupportHull+8threads+AAFKVolumeMin",
            "supported": True,
            "implementation": "row name retained for compatibility; command now runs the full warm+d18+A A F K VolumeMin shelf baseline via the lifelong runner",
            "command": shelf_baseline_command(args, "baseline_ifk_support_hull_8t_best_tighten", int(args.threads)),
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
    cache_path = args.rbf_cache_root / str(args.warm_cache_label)
    prewarm_summary = load_json(args.prewarm_json)
    extra_env = default_sbf_subprocess_env()
    needs_prewarm = any(row.get("supported") and row.get("command") for row in rows)
    if args.execute and needs_prewarm:
        prewarm_summary = ensure_p18_prewarm_summary(
            args.prewarm_json,
            cache_path=cache_path,
            prewarm_depth=int(args.prewarm_depth),
            envelope=str(args.prewarm_envelope),
            prewarm_threads=int(args.prewarm_threads),
            clean_cache=bool(args.clean_cache),
            dry_run=bool(args.dry_run),
        )
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
        "cache": {
            "path": str(cache_path),
            "bytes": directory_size(cache_path),
            "file_sizes": cache_file_sizes(cache_path),
            "manifest": read_manifest(cache_path / "manifest.json"),
            "snapshot": prewarm_summary.get("snapshot", snapshot_summary(snapshot_path_for_cache(cache_path))),
            "prewarm_json_path": str(args.prewarm_json),
            "prewarm_summary": prewarm_summary,
        },
        "matrix": rows,
        "runs": run_records,
        "notes": [
            "SBF rows reuse a shared Shelf+IIWA p18 endpoint-only cache under the experiment output root.",
            "LECT DB stores endpoint-envelope payloads only; link envelopes are reconstructed online from endpoint envelopes.",
            "Warm-cache snapshots are published by the writer after checkpoint; reused caches backfill missing snapshots before baseline execution.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
