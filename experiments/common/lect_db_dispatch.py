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


SHELF_SBF_COMBINED = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_marcucci_combined.py"
SHELF_SBF_CASE = REPO_ROOT / "experiments" / "common" / "run_shelf_sbf_case.py"
SHELF_BASELINES = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_baselines_marcucci.py"
SHELF_RRTCONNECT = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_04_rrt_connect_baseline.py"
RANDOM_SBF = REPO_ROOT / "safe_box_forest" / "experiments" / "rbf_only_random_robot_scenes.py"
RANDOM_RRTCONNECT = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_12_random_scene_rrt_baseline.py"
RANDOM_OMPL = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_12_random_scene_ompl_baselines.py"
RANDOM_IRIS = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_12_random_scene_iris_np_gcs_baseline.py"


def ensure_shelf_cache(
    *,
    prewarm_json: Path,
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    clean_cache: bool,
    dry_run: bool,
) -> dict[str, Any]:
    return ensure_p18_prewarm_summary(
        prewarm_json,
        cache_path=cache_path,
        prewarm_depth=int(prewarm_depth),
        envelope=str(envelope),
        prewarm_threads=int(prewarm_threads),
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
    command.append("--use-external-evidence" if use_external_evidence else "--no-use-external-evidence")
    command.append("--external-evidence-materialization" if external_evidence_materialization else "--no-external-evidence-materialization")
    command.append("--external-evidence-scoring" if external_evidence_scoring else "--no-external-evidence-scoring")
    if external_evidence_auto_build_snapshot:
        command.append("--external-evidence-auto-build-snapshot")
    else:
        command.append("--no-external-evidence-auto-build-snapshot")
    return command


def build_legacy_shelf_sbf_command(
    *,
    python_executable: str,
    out_json: Path,
    database_path: Path,
    preset: str,
    envelope: str,
    threads: int,
    seeds: int,
    timeout_ms: float,
    split_policy: str,
    use_external_evidence: bool,
    rbf_cache_root: Path,
    warm_cache_label: str,
    external_evidence_mode: str,
    external_evidence_auto_build_snapshot: bool,
    audit_resolution: int = 32,
) -> list[str]:
    command = [
        python_executable,
        str(SHELF_SBF_COMBINED),
        "--out-json",
        str(out_json),
        "--database-path",
        str(database_path),
        "--preset",
        str(preset),
        "--envelope",
        str(envelope),
        "--threads",
        str(int(threads)),
        "--seeds",
        str(max(1, int(seeds))),
        "--timeout-ms",
        str(float(timeout_ms)),
        "--split-policy",
        str(split_policy),
        "--audit-resolution",
        str(max(1, int(audit_resolution))),
        "--strict-path-audit",
    ]
    if use_external_evidence:
        command.extend([
            "--rbf-cache-root",
            str(rbf_cache_root),
            "--warm-cache-label",
            str(warm_cache_label),
            "--use-external-evidence",
            "--external-evidence-mode",
            str(external_evidence_mode),
        ])
        if external_evidence_auto_build_snapshot:
            command.append("--external-evidence-auto-build-snapshot")
        else:
            command.append("--no-external-evidence-auto-build-snapshot")
    return command


def build_random_sbf_command(
    *,
    python_executable: str,
    out_json: Path,
    robots: str,
    difficulties: str,
    scene_seeds: int,
    scene_profile: str,
    rbf_cache_root: Path,
    iiwa_warm_cache_label: str,
    external_evidence_mode: str,
    external_evidence_auto_build_snapshot: bool,
    threads: int,
    prewarm_depth: int,
    rbf_envelope: str,
    rbf_max_depth: int = 160,
) -> list[str]:
    command = [
        python_executable,
        str(RANDOM_SBF),
        "--robots",
        str(robots),
        "--difficulties",
        str(difficulties),
        "--scene-seeds",
        str(max(1, int(scene_seeds))),
        "--scene-profile",
        str(scene_profile),
        "--modes",
        "warm_d18",
        "--threads",
        str(max(1, int(threads))),
        "--task-batch-size",
        str(max(1, int(threads))),
        "--rbf-prewarm-depth",
        str(max(1, int(prewarm_depth))),
        "--rbf-max-depth",
        str(max(1, int(rbf_max_depth))),
        "--ffb-depth",
        str(max(1, int(rbf_max_depth))),
        "--connector-pave-depth",
        str(max(1, int(rbf_max_depth))),
        "--rbf-envelope",
        str(rbf_envelope),
        "--rbf-cache-root",
        str(rbf_cache_root),
        "--iiwa-warm-cache-label",
        str(iiwa_warm_cache_label),
        "--external-evidence-mode",
        str(external_evidence_mode),
        "--out-json",
        str(out_json),
    ]
    if external_evidence_auto_build_snapshot:
        command.append("--external-evidence-auto-build-snapshot")
    else:
        command.append("--no-external-evidence-auto-build-snapshot")
    return command
