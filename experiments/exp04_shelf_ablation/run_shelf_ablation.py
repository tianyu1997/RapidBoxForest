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
from experiments.common.lect_db_dispatch import (  # noqa: E402
    build_shelf_sbf_case_command,
    ensure_shelf_cache,
    shelf_cache_payload,
)
from experiments.common.shelf_iiwa_cache import DEFAULT_P18_CACHE_LABEL, load_json  # noqa: E402


BASELINE_NAME = "baseline_warm_aafk_support_hull_8t_aafk_volume_min"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp04_shelf_ablation"
    parser = argparse.ArgumentParser(description="Run Experiment 4 from the current ablation plan.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--prewarm-json", type=Path, default=output_dir / "p18_prewarm.json")
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--prewarm-envelope", choices=["link", "support_hull"], default="support_hull")
    parser.add_argument("--only", default="all", help="Comma-separated row names or all.")
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


def case_command(
    args: argparse.Namespace,
    *,
    name: str,
    endpoint_source: str,
    lect_split_policy: str,
    envelope: str,
    threads: int,
    use_external_evidence: bool,
    external_evidence_materialization: bool = True,
    external_evidence_scoring: bool = True,
    latency_profile: str = "stable",
) -> list[str]:
    return build_shelf_sbf_case_command(
        python_executable=sys.executable,
        out_json=args.out_dir / f"{name}.json",
        database_path=args.rbf_cache_root / name,
        case_name=name,
        endpoint_source=endpoint_source,
        lect_split_policy=lect_split_policy,
        envelope=envelope,
        threads=threads,
        seeds=int(args.seeds),
        timeout_ms=float(args.timeout_ms),
        use_external_evidence=use_external_evidence,
        rbf_cache_root=args.rbf_cache_root,
        warm_cache_label=str(args.warm_cache_label),
        external_evidence_mode=str(args.external_evidence_mode),
        external_evidence_auto_build_snapshot=bool(args.external_evidence_auto_build_snapshot),
        external_evidence_materialization=external_evidence_materialization,
        external_evidence_scoring=external_evidence_scoring,
        latency_profile=latency_profile,
        clean_active_cache=True,
    )


def full_matrix(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows = [
        {
            "name": BASELINE_NAME,
            "factor": "baseline",
            "description": "warm LECT cache + AAFK endpoint + SupportHull + 8 threads + AAFKVolumeMin split policy",
            "changes_from_baseline": [],
            "command": case_command(
                args,
                name=BASELINE_NAME,
                endpoint_source="aafk",
                lect_split_policy="aafk_volume_min",
                envelope="support_hull",
                threads=int(args.threads),
                use_external_evidence=True,
                external_evidence_materialization=True,
                external_evidence_scoring=True,
            ),
        },
        {
            "name": "balanced_low_latency",
            "factor": "latency_profile",
            "description": "warm LECT cache + AAFK + SupportHull with a staged low-box incumbent protocol adapted from the old shelf anytime settings",
            "changes_from_baseline": [
                "latency_profile=balanced_low_latency",
                "task_batch_size<=2",
                "component_connect_candidate_limit=1",
                "staged low-box schedule with incumbent retention",
                "quality/max_boxes staged as 2/16 -> 64/96 -> 128/160 -> 192/224 -> 256/320",
            ],
            "command": case_command(
                args,
                name="balanced_low_latency",
                endpoint_source="aafk",
                lect_split_policy="aafk_volume_min",
                envelope="support_hull",
                threads=int(args.threads),
                use_external_evidence=True,
                external_evidence_materialization=True,
                external_evidence_scoring=True,
                latency_profile="balanced_low_latency",
            ),
        },
        {
            "name": "no_lect_cache_online_envelopes",
            "factor": "cache",
            "description": "same as baseline, but do not reuse the Exp.3 LECT DB cache as external evidence",
            "changes_from_baseline": ["use_external_evidence=false"],
            "command": case_command(
                args,
                name="no_lect_cache_online_envelopes",
                endpoint_source="aafk",
                lect_split_policy="aafk_volume_min",
                envelope="support_hull",
                threads=int(args.threads),
                use_external_evidence=False,
            ),
        },
        {
            "name": "critsample_endpoint_support_hull",
            "factor": "endpoint_source",
            "description": "replace the AAFK endpoint channel with CritSample while keeping SupportHull and AAFKVolumeMin splitting",
            "changes_from_baseline": ["endpoint_source=critsample"],
            "command": case_command(
                args,
                name="critsample_endpoint_support_hull",
                endpoint_source="critsample",
                lect_split_policy="aafk_volume_min",
                envelope="support_hull",
                threads=int(args.threads),
                use_external_evidence=True,
                external_evidence_materialization=True,
                external_evidence_scoring=True,
            ),
        },
        {
            "name": "aabb_envelope_only",
            "factor": "envelope_collision",
            "description": "replace SupportHull envelope checks with the legacy link/AABB envelope mode",
            "changes_from_baseline": ["rbf_envelope=link"],
            "command": case_command(
                args,
                name="aabb_envelope_only",
                endpoint_source="aafk",
                lect_split_policy="aafk_volume_min",
                envelope="link",
                threads=int(args.threads),
                use_external_evidence=True,
                external_evidence_materialization=True,
                external_evidence_scoring=True,
            ),
        },
        {
            "name": "single_thread",
            "factor": "threads",
            "description": "same as baseline with one worker thread",
            "changes_from_baseline": ["threads=1"],
            "command": case_command(
                args,
                name="single_thread",
                endpoint_source="aafk",
                lect_split_policy="aafk_volume_min",
                envelope="support_hull",
                threads=1,
                use_external_evidence=True,
                external_evidence_materialization=True,
                external_evidence_scoring=True,
            ),
        },
        {
            "name": "round_robin_split_policy",
            "factor": "lect_split_policy",
            "description": "same as baseline, but replace AAFKVolumeMin LECT splitting with deterministic round-robin splitting",
            "changes_from_baseline": ["lect_split_policy=round_robin"],
            "command": case_command(
                args,
                name="round_robin_split_policy",
                endpoint_source="aafk",
                lect_split_policy="round_robin",
                envelope="support_hull",
                threads=int(args.threads),
                use_external_evidence=True,
                external_evidence_materialization=True,
                external_evidence_scoring=True,
            ),
        },
    ]
    wanted = set(csv_list(args.only))
    if wanted and "all" not in wanted:
        rows = [row for row in rows if row["name"] in wanted]
    return rows


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "shelf_ablation_manifest.json")
    rows = full_matrix(args)
    cache_path = args.rbf_cache_root / str(args.warm_cache_label)
    needs_prewarm = any("--use-external-evidence" in row["command"] for row in rows)
    prewarm_summary: dict[str, Any]
    if args.execute and needs_prewarm:
        prewarm_summary = ensure_shelf_cache(
            prewarm_json=args.prewarm_json,
            cache_path=cache_path,
            prewarm_depth=int(args.prewarm_depth),
            envelope=str(args.prewarm_envelope),
            prewarm_threads=int(args.prewarm_threads),
            clean_cache=bool(args.clean_cache),
            dry_run=bool(args.dry_run),
        )
    else:
        prewarm_summary = load_json(args.prewarm_json)

    run_records = []
    extra_env = default_sbf_subprocess_env()
    if args.execute:
        for row in rows:
            measurement = run_command(row["command"], dry_run=bool(args.dry_run), extra_env=extra_env)
            run_records.append({"name": row["name"], "measurement": measurement})

    payload = {
        "experiment": "exp04_shelf_ablation",
        "plan_file": "experiments/04_shelf_ablation_plan.md",
        "run_id": run_id("exp04"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": shelf_cache_payload(cache_path, args.prewarm_json, prewarm_summary),
        "matrix": rows,
        "runs": run_records,
        "notes": [
            "This wrapper is a fresh plan-driven Exp.4 entrypoint under /experiments.",
            "All SBF rows use experiments/common/run_shelf_sbf_case.py instead of the transitional shelf runner.",
            "The Exp.3 p18 LECT DB cache is prepared once and passed to warm rows as snapshot-backed external evidence.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
