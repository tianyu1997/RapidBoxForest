from __future__ import annotations

import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.shelf_iiwa_cache import (  # noqa: E402
    cache_file_sizes,
    directory_size,
    ensure_p18_prewarm_summary,
    load_json,
    read_manifest,
    snapshot_path_for_cache,
    snapshot_summary,
)
from experiments.common.anytime_defaults import (  # noqa: E402
    UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP,
    UNIFIED_SBF_ANYTIME_FFB_START_DEPTH,
    UNIFIED_SBF_ANYTIME_MAX_BOXES,
    UNIFIED_SBF_ANYTIME_POST_AUDIT_SEGMENT_STEP,
    UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES,
    UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS,
    UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES,
    UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
    csv_floats,
    csv_ints,
    legacy_stage_spec,
)


SHELF_SBF_CASE = REPO_ROOT / "experiments" / "common" / "run_shelf_sbf_case.py"
SHELF_SBF_ANYTIME_CURRENT = REPO_ROOT / "experiments" / "common" / "run_shelf_sbf_anytime.py"
SHELF_ANYTIME = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_14_shelf_anytime_tradeoff.py"
SHELF_IRIS_ANYTIME = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_16_shelf_iris_np_gcs_anytime.py"
RANDOM_ANYTIME = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_15_random_anytime_tradeoff.py"
RANDOM_IRIS_ANYTIME = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_16_random_iris_np_gcs_anytime.py"


def ensure_shelf_cache(
    *,
    prewarm_json: Path,
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    max_depth: int,
    endpoint_source: str = "aafk",
    lect_split_policy: str = "aafk_volume_min",
    canonical_mode: bool = True,
    clean_cache: bool,
    dry_run: bool,
) -> dict[str, Any]:
    return ensure_p18_prewarm_summary(
        prewarm_json,
        cache_path=cache_path,
        prewarm_depth=int(prewarm_depth),
        envelope=str(envelope),
        prewarm_threads=int(prewarm_threads),
        max_depth=int(max_depth),
        endpoint_source=str(endpoint_source),
        lect_split_policy=str(lect_split_policy),
        canonical_mode=bool(canonical_mode),
        clean_cache=bool(clean_cache),
        dry_run=bool(dry_run),
    )


def shelf_cache_payload(cache_path: Path, prewarm_json: Path, prewarm_summary: dict[str, Any] | None = None) -> dict[str, Any]:
    summary = prewarm_summary or load_json(prewarm_json)
    return {
        "path": str(cache_path),
        "bytes": directory_size(cache_path),
        "file_sizes": cache_file_sizes(cache_path),
        "manifest": read_manifest(cache_path / "manifest.json"),
        "snapshot": summary.get("snapshot", snapshot_summary(snapshot_path_for_cache(cache_path))),
        "prewarm_json_path": str(prewarm_json),
        "prewarm_summary": summary,
    }


def build_shelf_sbf_case_command(
    *,
    python_executable: str,
    out_json: Path,
    database_path: Path,
    case_name: str,
    endpoint_source: str,
    lect_split_policy: str,
    envelope: str,
    threads: int,
    seeds: int,
    timeout_ms: float,
    use_external_evidence: bool,
    rbf_cache_root: Path,
    warm_cache_label: str,
    external_evidence_mode: str,
    external_evidence_auto_build_snapshot: bool,
    endpoint_evidence_cache: bool = True,
    rbf_max_depth: int = 50,
    external_evidence_materialization: bool = True,
    external_evidence_scoring: bool = True,
    clean_active_cache: bool = True,
    latency_profile: str = "stable",
) -> list[str]:
    command = [
        python_executable,
        str(SHELF_SBF_CASE),
        "--case-name",
        str(case_name),
        "--out-json",
        str(out_json),
        "--database-path",
        str(database_path),
        "--endpoint-source",
        str(endpoint_source),
        "--lect-split-policy",
        str(lect_split_policy),
        "--rbf-envelope",
        str(envelope),
        "--threads",
        str(max(1, int(threads))),
        "--task-batch-size",
        str(max(1, int(threads))),
        "--seeds-list",
        ",".join(str(index) for index in range(max(1, int(seeds)))),
        "--timeout-ms",
        str(float(timeout_ms)),
        "--rbf-max-depth",
        str(max(1, int(rbf_max_depth))),
        "--ffb-depth",
        str(max(1, int(rbf_max_depth))),
        "--connector-pave-depth",
        str(max(1, int(rbf_max_depth))),
        "--component-connect-ffb-max-depth",
        str(max(1, int(rbf_max_depth))),
        "--rbf-cache-root",
        str(rbf_cache_root),
        "--warm-cache-label",
        str(warm_cache_label),
        "--external-evidence-mode",
        str(external_evidence_mode),
        "--latency-profile",
        str(latency_profile),
    ]
    command.append("--clean-active-cache" if clean_active_cache else "--no-clean-active-cache")
    command.append("--endpoint-evidence-cache" if endpoint_evidence_cache else "--no-endpoint-evidence-cache")
    command.append("--use-external-evidence" if use_external_evidence else "--no-use-external-evidence")
    command.append("--external-evidence-materialization" if external_evidence_materialization else "--no-external-evidence-materialization")
    command.append("--external-evidence-scoring" if external_evidence_scoring else "--no-external-evidence-scoring")
    if external_evidence_auto_build_snapshot:
        command.append("--external-evidence-auto-build-snapshot")
    else:
        command.append("--no-external-evidence-auto-build-snapshot")
    return command


def build_current_shelf_sbf_anytime_command(
    *,
    python_executable: str,
    out_json: Path,
    database_path: Path,
    case_name: str,
    threads: int,
    seeds: int,
    timeout_ms: float,
    rbf_cache_root: Path,
    warm_cache_label: str,
    external_evidence_mode: str,
    external_evidence_auto_build_snapshot: bool,
    endpoint_source: str = "aafk",
    lect_split_policy: str = "aafk_volume_min",
    envelope: str = "support_hull",
    use_external_evidence: bool = True,
    endpoint_evidence_cache: bool = True,
    external_evidence_materialization: bool = True,
    external_evidence_scoring: bool = True,
    clean_active_cache: bool = True,
    rbf_max_depth: int = UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH,
    canonical_cache: bool = True,
    require_no_repair: bool = False,
) -> list[str]:
    effective_rbf_max_depth = max(1, int(rbf_max_depth))
    command = [
        python_executable,
        str(SHELF_SBF_ANYTIME_CURRENT),
        "--case-name",
        str(case_name),
        "--out-json",
        str(out_json),
        "--database-path",
        str(database_path),
        "--endpoint-source",
        str(endpoint_source),
        "--lect-split-policy",
        str(lect_split_policy),
        "--preset",
        "support_hull_coverage",
        "--rbf-envelope",
        str(envelope),
        "--threads",
        str(max(1, int(threads))),
        "--task-batch-size",
        str(max(1, int(threads))),
        "--seeds",
        str(max(1, int(seeds))),
        "--timeout-ms",
        str(float(timeout_ms)),
        "--rbf-max-depth",
        str(effective_rbf_max_depth),
        "--ffb-depth",
        str(effective_rbf_max_depth),
        "--connector-pave-depth",
        str(effective_rbf_max_depth),
        "--component-connect-ffb-max-depth",
        str(effective_rbf_max_depth),
        "--rbf-ffb-start-depth",
        str(min(int(UNIFIED_SBF_ANYTIME_FFB_START_DEPTH), effective_rbf_max_depth)),
        "--audit-segment-step",
        str(UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP),
        "--post-audit-segment-step",
        str(UNIFIED_SBF_ANYTIME_POST_AUDIT_SEGMENT_STEP),
        "--rbf-cache-root",
        str(rbf_cache_root),
        "--warm-cache-label",
        str(warm_cache_label),
        "--external-evidence-mode",
        str(external_evidence_mode),
        "--latency-profile",
        "balanced_low_latency",
        "--latency-stage-selection-policy",
        "zero_repair" if bool(require_no_repair) else "accept_repair",
        "--no-latency-stage-early-stop",
        "--latency-stage-max-boxes",
        csv_ints(UNIFIED_SBF_ANYTIME_MAX_BOXES),
        "--latency-stage-quality-min-connected-boxes",
        csv_ints(UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES),
        "--latency-stage-post-connect-extra-boxes",
        csv_ints(UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES),
        "--latency-stage-post-connect-time-budget-ms",
        csv_floats(UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS),
        "--connector-max-pairs-per-gap",
        "1",
    ]
    command.append("--clean-active-cache" if clean_active_cache else "--no-clean-active-cache")
    command.append("--endpoint-evidence-cache" if endpoint_evidence_cache else "--no-endpoint-evidence-cache")
    command.append("--use-external-evidence" if use_external_evidence else "--no-use-external-evidence")
    command.append("--external-evidence-materialization" if external_evidence_materialization else "--no-external-evidence-materialization")
    command.append("--external-evidence-scoring" if external_evidence_scoring else "--no-external-evidence-scoring")
    command.append("--external-evidence-auto-build-snapshot" if external_evidence_auto_build_snapshot else "--no-external-evidence-auto-build-snapshot")
    command.append("--rbf-canonical-cache" if bool(canonical_cache) else "--no-rbf-canonical-cache")
    command.append("--require-no-repair" if bool(require_no_repair) else "--no-require-no-repair")
    return command


def build_legacy_shelf_anytime_command(
    *,
    python_executable: str,
    out_json: Path,
    methods: str,
    threads: int,
    seeds: int,
) -> list[str]:
    return [
        python_executable,
        str(SHELF_ANYTIME),
        "--methods",
        str(methods),
        "--seeds",
        str(max(1, int(seeds))),
        "--threads",
        str(max(1, int(threads))),
        "--task-batch-size",
        str(max(1, int(threads))),
        "--segment-step",
        str(UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP),
        "--audit-segment-step",
        str(UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP),
        "--final-ompl-simplify-time-s",
        "0.01",
        "--out-json",
        str(out_json),
    ]


def build_shelf_iris_anytime_command(
    *,
    python_executable: str,
    out_json: Path,
    seeds: int,
    threads: int,
) -> list[str]:
    return [
        python_executable,
        str(SHELF_IRIS_ANYTIME),
        "--seeds",
        str(max(1, int(seeds))),
        "--logical-threads",
        str(max(1, int(threads))),
        "--segment-step",
        str(UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP),
        "--final-ompl-simplify-time-s",
        "0.01",
        "--out-json",
        str(out_json),
    ]


def build_random_anytime_command(
    *,
    python_executable: str,
    out_json: Path,
    robots: str,
    difficulties: str,
    scene_seeds: int,
    scene_profile: str,
    threads: int,
    trials: int,
    methods: str,
    baseline_methods: str,
    cache_root: Path,
    cache_run_id: str,
    clear_cache: bool,
) -> list[str]:
    command = [
        python_executable,
        str(RANDOM_ANYTIME),
        "--robots",
        str(robots),
        "--difficulties",
        str(difficulties),
        "--scene-seeds",
        str(max(1, int(scene_seeds))),
        "--scene-profile",
        str(scene_profile),
        "--methods",
        str(methods),
        "--baseline-methods",
        str(baseline_methods),
        "--baseline-trials",
        str(max(1, int(trials))),
        "--threads",
        str(max(1, int(threads))),
        "--task-batch-size",
        str(max(1, int(threads))),
        "--sbf-stages",
        legacy_stage_spec(),
        "--cache-run-id",
        str(cache_run_id),
        "--rbf-max-depth",
        str(UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH),
        "--ffb-depth",
        str(UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH),
        "--connector-pave-depth",
        str(UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH),
        "--component-connect-ffb-max-depth",
        str(UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH),
        "--rbf-ffb-start-depth",
        str(UNIFIED_SBF_ANYTIME_FFB_START_DEPTH),
        "--rbf-canonical-cache",
        "--segment-step",
        str(UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP),
        "--audit-segment-step",
        str(UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP),
        "--final-ompl-simplify-time-s",
        "0.01",
        "--rbf-envelope",
        "support_hull",
        "--cache-root",
        str(cache_root),
        "--out-json",
        str(out_json),
    ]
    command.append("--clear-cache" if clear_cache else "--no-clear-cache")
    return command


def build_random_iris_anytime_command(
    *,
    python_executable: str,
    out_json: Path,
    robots: str,
    difficulties: str,
    scene_seeds: int,
    scene_profile: str,
    threads: int,
    trials: int,
) -> list[str]:
    return [
        python_executable,
        str(RANDOM_IRIS_ANYTIME),
        "--robots",
        str(robots),
        "--difficulties",
        str(difficulties),
        "--scene-seeds",
        str(max(1, int(scene_seeds))),
        "--trials",
        str(max(1, int(trials))),
        "--scene-profile",
        str(scene_profile),
        "--logical-threads",
        str(max(1, int(threads))),
        "--segment-step",
        "0.01",
        "--audit-segment-step",
        "0.01",
        "--final-ompl-simplify-time-s",
        "0.01",
        "--out-json",
        str(out_json),
    ]
