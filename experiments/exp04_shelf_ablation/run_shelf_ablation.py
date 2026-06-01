#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
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
from experiments.common.cache_maintenance import prune_directory_children  # noqa: E402
from experiments.common.anytime_defaults import (  # noqa: E402
    UNIFIED_SBF_ANYTIME_FFB_START_DEPTH,
    UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
)
from experiments.common.lect_db_dispatch import (  # noqa: E402
    build_current_shelf_sbf_anytime_command,
    ensure_shelf_cache,
    shelf_cache_payload,
)
from experiments.common.shelf_iiwa_cache import (  # noqa: E402
    DEFAULT_P18_CACHE_LABEL,
    DEFAULT_P18_NATIVE_CACHE_LABEL,
    channel_cache_label,
    load_json,
)


BASELINE_NAME = "baseline_warm_aafk_support_hull_8t_aafk_volume_min"
FIXED_SHELF_ROOT_INTERVALS = ";".join([
    "0.0:1.5707963267948966",
    "0.3194:0.8645",
    "-0.5077:0.5073",
    "-1.98947519:-0.33002121",
    "-0.447:0.4473",
    "-1.34734773:1.51007653",
    "1.262:1.8794",
])


def row_artifact_path(args: argparse.Namespace, name: str) -> Path:
    return args.out_dir / f"{name}.json"


def build_run_record(row: dict[str, Any], measurement: dict[str, Any]) -> dict[str, Any]:
    artifact_path = Path(str(row["artifact_path"]))
    artifact_exists = artifact_path.exists()
    returncode = measurement.get("returncode")
    if returncode == 0 and artifact_exists:
        status = "completed"
    elif returncode == 0:
        status = "missing_artifact"
    elif returncode is None:
        status = "dry_run"
    else:
        status = "failed"
    return {
        "name": row["name"],
        "status": status,
        "artifact": str(artifact_path) if artifact_exists else None,
        "measurement": measurement,
    }


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp04_shelf_ablation"
    parser = argparse.ArgumentParser(description="Run Experiment 4 as the full current shelf ablation matrix on the anytime backend.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--prewarm-json", type=Path, default=None)
    parser.add_argument("--prewarm-depth", type=int, default=18)
    parser.add_argument("--rbf-max-depth", type=int, default=UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH)
    parser.add_argument("--prewarm-max-depth", type=int, default=None)
    parser.add_argument("--prewarm-threads", type=int, default=8)
    parser.add_argument("--prewarm-envelope", choices=["link", "support_hull"], default="support_hull")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--rbf-cache-root", type=Path, default=None)
    parser.add_argument("--warm-cache-label", default=None)
    parser.add_argument("--warm-cache-canonical", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-no-repair", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--only", default="all", help="Comma-separated row names or all.")
    parser.add_argument("--external-evidence-mode", choices=["snapshot", "legacy"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lect-root-intervals", default=FIXED_SHELF_ROOT_INTERVALS)
    parser.add_argument("--aafk-sample-nodes-per-depth", type=int, default=8)
    # Global recommended defaults for FFB depth compression.
    parser.add_argument("--ffb-auto-mask-inert", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-ffb-start-depth", type=int, default=UNIFIED_SBF_ANYTIME_FFB_START_DEPTH)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def ablation_command(
    args: argparse.Namespace,
    *,
    name: str,
    endpoint_source: str = "aafk",
    lect_split_policy: str = "aafk_volume_min",
    envelope: str = "support_hull",
    threads: int | None = None,
    use_external_evidence: bool = True,
    endpoint_evidence_cache: bool = True,
    rbf_max_depth: int | None = None,
    warm_cache_label: str | None = None,
) -> list[str]:
    return build_current_shelf_sbf_anytime_command(
        python_executable=sys.executable,
        out_json=row_artifact_path(args, name),
        database_path=args.out_dir / "active_cache" / name,
        case_name=name,
        endpoint_source=str(endpoint_source),
        lect_split_policy=str(lect_split_policy),
        envelope=str(envelope),
        threads=int(threads if threads is not None else args.threads),
        seeds=int(args.seeds),
        timeout_ms=float(args.timeout_ms),
        use_external_evidence=bool(use_external_evidence),
        endpoint_evidence_cache=bool(endpoint_evidence_cache),
        rbf_cache_root=args.rbf_cache_root,
        warm_cache_label=str(warm_cache_label or args.warm_cache_label),
        external_evidence_mode=str(args.external_evidence_mode),
        external_evidence_auto_build_snapshot=bool(args.external_evidence_auto_build_snapshot),
        external_evidence_materialization=True,
        external_evidence_scoring=True,
        clean_active_cache=True,
        rbf_max_depth=int(rbf_max_depth if rbf_max_depth is not None else args.rbf_max_depth),
        canonical_cache=bool(args.warm_cache_canonical),
        require_no_repair=bool(args.require_no_repair),
        lect_root_intervals=str(args.lect_root_intervals),
        aafk_sample_nodes_per_depth=int(args.aafk_sample_nodes_per_depth),
        ffb_auto_mask_inert=bool(getattr(args, "ffb_auto_mask_inert", True)),
        rbf_ffb_start_depth=int(getattr(args, "rbf_ffb_start_depth", UNIFIED_SBF_ANYTIME_FFB_START_DEPTH)),
    )


def command_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    baseline_cache_label = channel_cache_label(str(args.warm_cache_label), "aafk", "aafk_volume_min")
    critsample_cache_label = channel_cache_label(str(args.warm_cache_label), "critsample", "aafk_volume_min")
    round_robin_cache_label = channel_cache_label(str(args.warm_cache_label), "aafk", "round_robin")
    hybrid_dim6_cache_label = channel_cache_label(str(args.warm_cache_label), "aafk", "aafk_volume_min_dim6")
    support_hull_split_cache_label = channel_cache_label(str(args.warm_cache_label), "aafk", "support_hull_volume_min")
    rows = [
        {
            "name": BASELINE_NAME,
            "artifact_path": str(row_artifact_path(args, BASELINE_NAME)),
            "kind": "sbf_anytime_current",
            "factor": "baseline",
            "description": "No external LECT DB reuse; keep the active online endpoint cache path with AAFK + SupportHull + 8 threads + AAFKVolumeMin split policy.",
            "changes_from_baseline": [],
            "uses_external_evidence": False,
            "active_cache_path": str(args.out_dir / "active_cache" / BASELINE_NAME),
            "command": ablation_command(
                args,
                name=BASELINE_NAME,
                use_external_evidence=False,
                endpoint_evidence_cache=True,
            ),
        },
        {
            "name": "no_lect_cache_online_envelopes",
            "artifact_path": str(row_artifact_path(args, "no_lect_cache_online_envelopes")),
            "kind": "sbf_anytime_current",
            "factor": "cache",
            "description": "Disable external reuse and disable active endpoint evidence reuse so payloads stay fully online.",
            "changes_from_baseline": ["use_external_evidence=false", "endpoint_evidence_cache=false"],
            "uses_external_evidence": False,
            "active_cache_path": str(args.out_dir / "active_cache" / "no_lect_cache_online_envelopes"),
            "command": ablation_command(
                args,
                name="no_lect_cache_online_envelopes",
                use_external_evidence=False,
                endpoint_evidence_cache=False,
            ),
        },
        {
            "name": "critsample_endpoint_support_hull",
            "artifact_path": str(row_artifact_path(args, "critsample_endpoint_support_hull")),
            "kind": "sbf_anytime_current",
            "factor": "endpoint_source",
            "description": "Replace AAFK with CritSample and reuse a CritSample-specific warm LECT cache.",
            "changes_from_baseline": ["endpoint_source=critsample", "warm_cache=critsample"],
            "uses_external_evidence": True,
            "warm_cache_label": critsample_cache_label,
            "warm_cache_endpoint_source": "critsample",
            "warm_cache_lect_split_policy": "aafk_volume_min",
            "active_cache_path": str(args.out_dir / "active_cache" / "critsample_endpoint_support_hull"),
            "command": ablation_command(
                args,
                name="critsample_endpoint_support_hull",
                endpoint_source="critsample",
                warm_cache_label=critsample_cache_label,
            ),
        },
        {
            "name": "aabb_envelope_only",
            "artifact_path": str(row_artifact_path(args, "aabb_envelope_only")),
            "kind": "sbf_anytime_current",
            "factor": "envelope_collision",
            "description": "Replace SupportHull with link/AABB envelopes under the same staged protocol.",
            "changes_from_baseline": ["rbf_envelope=link"],
            "uses_external_evidence": True,
            "warm_cache_label": baseline_cache_label,
            "warm_cache_endpoint_source": "aafk",
            "warm_cache_lect_split_policy": "aafk_volume_min",
            "active_cache_path": str(args.out_dir / "active_cache" / "aabb_envelope_only"),
            "command": ablation_command(
                args,
                name="aabb_envelope_only",
                envelope="link",
                warm_cache_label=baseline_cache_label,
            ),
        },
        {
            "name": "single_thread",
            "artifact_path": str(row_artifact_path(args, "single_thread")),
            "kind": "sbf_anytime_current",
            "factor": "threads",
            "description": "Same as baseline with one worker thread.",
            "changes_from_baseline": ["threads=1"],
            "uses_external_evidence": True,
            "warm_cache_label": baseline_cache_label,
            "warm_cache_endpoint_source": "aafk",
            "warm_cache_lect_split_policy": "aafk_volume_min",
            "active_cache_path": str(args.out_dir / "active_cache" / "single_thread"),
            "command": ablation_command(
                args,
                name="single_thread",
                threads=1,
                warm_cache_label=baseline_cache_label,
            ),
        },
        {
            "name": "round_robin_split_policy",
            "artifact_path": str(row_artifact_path(args, "round_robin_split_policy")),
            "kind": "sbf_anytime_current",
            "factor": "lect_split_policy",
            "description": "Same as baseline but replace AAFKVolumeMin with deterministic round-robin splitting and reuse a round-robin warm LECT cache.",
            "changes_from_baseline": ["lect_split_policy=round_robin", "warm_cache=round_robin"],
            "uses_external_evidence": True,
            "warm_cache_label": round_robin_cache_label,
            "warm_cache_endpoint_source": "aafk",
            "warm_cache_lect_split_policy": "round_robin",
            "active_cache_path": str(args.out_dir / "active_cache" / "round_robin_split_policy"),
            "command": ablation_command(
                args,
                name="round_robin_split_policy",
                lect_split_policy="round_robin",
                warm_cache_label=round_robin_cache_label,
            ),
        },
        {
            "name": "hybrid_dim6_split_policy",
            "artifact_path": str(row_artifact_path(args, "hybrid_dim6_split_policy")),
            "kind": "sbf_anytime_current",
            "factor": "lect_split_policy",
            "description": "Same as baseline but use the AAFKVolumeMin+dim6 hybrid schedule that guarantees coverage of starved DOFs (e.g. the wrist roll) and reuse a hybrid-specific warm LECT cache.",
            "changes_from_baseline": ["lect_split_policy=aafk_volume_min_dim6", "warm_cache=aafk_volume_min_dim6"],
            "uses_external_evidence": True,
            "warm_cache_label": hybrid_dim6_cache_label,
            "warm_cache_endpoint_source": "aafk",
            "warm_cache_lect_split_policy": "aafk_volume_min_dim6",
            "active_cache_path": str(args.out_dir / "active_cache" / "hybrid_dim6_split_policy"),
            "command": ablation_command(
                args,
                name="hybrid_dim6_split_policy",
                lect_split_policy="aafk_volume_min_dim6",
                warm_cache_label=hybrid_dim6_cache_label,
            ),
        },
        {
            "name": "support_hull_split_schedule",
            "artifact_path": str(row_artifact_path(args, "support_hull_split_schedule")),
            "kind": "sbf_anytime_current",
            "factor": "lect_split_schedule_criterion",
            "description": "NEW: seed/scene-independent split schedule that minimises the SupportHull swept-envelope volume (the certification envelope) instead of the looser endpoint-AABB sum. Compares the schedule criterion (AABB vs support-hull) while keeping the canonical, seed-independent invariant.",
            "changes_from_baseline": ["lect_split_policy=support_hull_volume_min", "warm_cache=support_hull_volume_min"],
            "uses_external_evidence": True,
            "warm_cache_label": support_hull_split_cache_label,
            "warm_cache_endpoint_source": "aafk",
            "warm_cache_lect_split_policy": "support_hull_volume_min",
            "active_cache_path": str(args.out_dir / "active_cache" / "support_hull_split_schedule"),
            "command": ablation_command(
                args,
                name="support_hull_split_schedule",
                lect_split_policy="support_hull_volume_min",
                warm_cache_label=support_hull_split_cache_label,
            ),
        },
    ]
    wanted = set(csv_list(args.only))
    if wanted and "all" not in wanted:
        rows = [row for row in rows if row["name"] in wanted]
    return rows


def omitted_rows() -> list[dict[str, Any]]:
    return [{
        "name": "aabb_to_support_hull_chain",
        "kind": "merged_configuration",
        "executed": False,
        "reason": "SupportHull now uses the unified pure-GJK narrow phase; the historical AABB->SH chain path is no longer a distinct executable configuration.",
    }]


def main() -> int:
    args = parse_args()
    if args.warm_cache_label is None:
        args.warm_cache_label = DEFAULT_P18_CACHE_LABEL if bool(args.warm_cache_canonical) else DEFAULT_P18_NATIVE_CACHE_LABEL
    prewarm_max_depth = int(args.prewarm_max_depth if args.prewarm_max_depth is not None else args.rbf_max_depth)
    args.prewarm_json = args.prewarm_json or (args.out_dir / "p18_prewarm.json")
    args.rbf_cache_root = args.rbf_cache_root or (args.out_dir / "cache")
    out_json = args.out_json or (args.out_dir / "shelf_ablation_manifest.json")
    args.out_json = out_json
    rows = command_rows(args)
    cache_specs: dict[str, dict[str, str]] = {}
    for row in rows:
        if not bool(row.get("uses_external_evidence")):
            continue
        label = str(row.get("warm_cache_label") or args.warm_cache_label)
        cache_specs[label] = {
            "endpoint_source": str(row.get("warm_cache_endpoint_source", "aafk")),
            "lect_split_policy": str(row.get("warm_cache_lect_split_policy", "aafk_volume_min")),
            "envelope": str(args.prewarm_envelope),
            "canonical_mode": bool(args.warm_cache_canonical),
        }
    keep_cache_labels = sorted(cache_specs) or [str(args.warm_cache_label)]
    prune_summary: dict[str, Any] = {
        "path": str(args.rbf_cache_root),
        "kept": keep_cache_labels,
        "removed": [],
        "removed_count": 0,
        "dry_run": bool(args.dry_run or not args.execute),
    }
    if args.execute and not args.dry_run:
        prune_summary = prune_directory_children(args.rbf_cache_root, keep_cache_labels)
    prewarm_jsons: dict[str, Path] = {}
    prewarm_summaries: dict[str, dict[str, Any]] = {}
    for label, spec in cache_specs.items():
        prewarm_json = args.prewarm_json if label == str(args.warm_cache_label) else args.out_dir / f"p18_prewarm_{label}.json"
        prewarm_jsons[label] = prewarm_json
        if args.execute:
            prewarm_summaries[label] = ensure_shelf_cache(
                prewarm_json=prewarm_json,
                cache_path=args.rbf_cache_root / label,
                prewarm_depth=int(args.prewarm_depth),
                envelope=str(spec["envelope"]),
                prewarm_threads=int(args.prewarm_threads),
                max_depth=prewarm_max_depth,
                endpoint_source=str(spec["endpoint_source"]),
                lect_split_policy=str(spec["lect_split_policy"]),
                lect_root_intervals=str(args.lect_root_intervals),
                canonical_mode=bool(spec["canonical_mode"]),
                clean_cache=bool(args.clean_cache),
                dry_run=bool(args.dry_run),
            )
        else:
            prewarm_summaries[label] = load_json(prewarm_json)

    run_records = []
    extra_env = default_sbf_subprocess_env()
    active_cache_root = args.out_dir / "active_cache"
    if args.execute and not args.dry_run and active_cache_root.exists():
        shutil.rmtree(active_cache_root)
    if args.execute:
        for row in rows:
            measurement = run_command(row["command"], dry_run=bool(args.dry_run), extra_env=extra_env)
            run_records.append(build_run_record(row, measurement))
            if not args.dry_run:
                row_prefix = f"{row['name']}"
                for candidate in active_cache_root.glob(f"{row_prefix}*"):
                    if candidate.is_dir():
                        shutil.rmtree(candidate)
                    elif candidate.exists():
                        candidate.unlink()
    if active_cache_root.exists() and not any(active_cache_root.iterdir()):
        active_cache_root.rmdir()
    active_cache_cleanup = {
        "path": str(active_cache_root),
        "exists_after_run": active_cache_root.exists(),
    }
    primary_cache_label = next(iter(cache_specs), str(args.warm_cache_label))
    primary_cache_path = args.rbf_cache_root / primary_cache_label
    primary_prewarm_json = prewarm_jsons.get(primary_cache_label, args.prewarm_json)
    primary_prewarm_summary = prewarm_summaries.get(primary_cache_label, load_json(primary_prewarm_json))
    warm_cache_payloads = {
        label: shelf_cache_payload(args.rbf_cache_root / label, prewarm_jsons[label], prewarm_summaries[label])
        for label in sorted(prewarm_summaries)
    }

    payload = {
        "experiment": "exp04_shelf_ablation",
        "plan_file": "experiments/04_shelf_ablation_plan.md",
        "run_id": run_id("exp04"),
        "status": "executed" if args.execute and not args.dry_run else "dry_run",
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "cache": {
            **shelf_cache_payload(primary_cache_path, primary_prewarm_json, primary_prewarm_summary),
            "prune": prune_summary,
            "active_cache_cleanup": active_cache_cleanup,
            "warm_caches": warm_cache_payloads,
        },
        "commands": rows,
        "omitted_rows": omitted_rows(),
        "runs": run_records,
        "notes": [
            "Experiment 4 now emits the full current ablation matrix on top of the shared shelf anytime backend.",
            "All executable rows keep the same staged d40_r4 protocol unless the ablation factor itself changes endpoint source, envelope, thread count, split policy, or external cache reuse.",
            "Shelf runs preserve only the endpoint/split-compatible p18 warm caches required by selected rows; per-row active caches are created under the output directory and removed after execution, and the shelf backend does not checkpoint online warm-cache results back into LECT.",
            "The historical AABB->SH chain row is recorded as omitted because SupportHull already subsumes that path in the current pure-GJK implementation.",
        ],
    }
    write_json(out_json, payload)
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
