from __future__ import annotations

import shutil
import time
from pathlib import Path
from typing import Any

from experiments.common.random_scene_catalog import make_robot
from experiments.common.rbf_defaults import (
    CANONICAL_SYMMETRY_DESCRIPTOR,
    ROBOT_LECTDB_CACHE_ROOT,
    ROBOT_LECTDB_MAX_DEPTH,
    robot_lectdb_depth,
    robot_lectdb_label,
    robot_lectdb_path,
)
from experiments.common.rbf_leaf_rrt import make_aafk_split_policy
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


def far_obstacle() -> Any:
    return sbf.Obstacle(100.0, 100.0, 100.0, 101.0, 101.0, 101.0)


def directory_size(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def robot_split_schedule_kind(robot_name: str) -> str:
    return "support_hull_volume_min" if str(robot_name) in {"ur5", "panda"} else "aafk_volume_min"


def make_prewarm_config(
    robot: Any,
    database_path: Path,
    *,
    max_depth: int = ROBOT_LECTDB_MAX_DEPTH,
    threads: int = 8,
    split_schedule_kind: str = "aafk_volume_min",
) -> Any:
    cfg = sbf.SBFConfig()
    cfg.enable_connector = False
    cfg.endpoint_source.source = sbf.EndpointSource.IFK
    cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
    cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
    cfg.validation.accept_unsafe_free = False
    cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly

    cfg.database.path = str(database_path)
    cfg.database.create_if_missing = True
    cfg.database.max_tree_depth = int(max_depth)
    cfg.database.canonical_mode = True
    cfg.database.symmetry_descriptor = CANONICAL_SYMMETRY_DESCRIPTOR
    cfg.database.online_cache.allow_database_backfill = True
    cfg.database.split_policy = make_aafk_split_policy(
        robot,
        int(max_depth),
        None,
        split_schedule_kind=split_schedule_kind,
    )

    n_threads = max(1, int(threads))
    cfg.runtime.mode = sbf.ExecutionMode.Parallel if n_threads > 1 else sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = n_threads
    cfg.runtime.batch_size = n_threads
    cfg.grower.n_threads = n_threads
    cfg.grower.task_batch_size = n_threads
    return cfg


def cache_has_manifest(path: Path) -> bool:
    return (path / "manifest.json").exists()


def ensure_robot_lectdb_cache(
    robot_name: str,
    *,
    cache_root: Path = ROBOT_LECTDB_CACHE_ROOT,
    depth: int | None = None,
    max_depth: int = ROBOT_LECTDB_MAX_DEPTH,
    threads: int = 8,
    clean: bool = False,
    verify: bool = False,
    publish_snapshot: bool = True,
    dry_run: bool = False,
) -> dict[str, Any]:
    if str(robot_name) == "iiwa":
        path = robot_lectdb_path(robot_name)
        payload: dict[str, Any] = {
            "robot": "iiwa",
            "depth": robot_lectdb_depth(robot_name),
            "cache_root": str(path.parent),
            "cache_label": path.name,
            "cache_path": str(path),
            "dry_run": bool(dry_run),
            "restricted_root": False,
            "coverage_domain": "full_robot_joint_limits",
            "canonical_mapping_scope": "LECT_internal_only",
        }
        if dry_run:
            payload["ok"] = True
            payload["would_reuse_existing"] = path.exists()
            return payload
        payload.update({
            "ok": path.exists() and cache_has_manifest(path),
            "reused_existing": True,
            "cache_bytes": directory_size(path),
            "snapshot_path": str(path / "lect_snapshot"),
            "snapshot_exists": (path / "lect_snapshot").exists(),
        })
        return payload
    actual_depth = robot_lectdb_depth(robot_name) if depth is None else int(depth)
    label = robot_lectdb_label(robot_name, depth=actual_depth)
    path = Path(cache_root) / label
    payload: dict[str, Any] = {
        "robot": str(robot_name),
        "depth": actual_depth,
        "max_depth": int(max_depth),
        "cache_root": str(cache_root),
        "cache_label": label,
        "cache_path": str(path),
        "dry_run": bool(dry_run),
    }
    if dry_run:
        payload["ok"] = True
        payload["would_reuse_existing"] = path.exists()
        return payload
    if path.exists() and cache_has_manifest(path) and not clean:
        payload.update({
            "ok": True,
            "reused_existing": True,
            "cache_bytes": directory_size(path),
            "snapshot_path": str(path / "lect_snapshot"),
            "snapshot_exists": (path / "lect_snapshot").exists(),
        })
        return payload
    if clean and path.exists():
        shutil.rmtree(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    robot = make_robot(robot_name)
    cfg = make_prewarm_config(
        robot,
        path,
        max_depth=max_depth,
        threads=threads,
        split_schedule_kind=robot_split_schedule_kind(str(robot_name)),
    )
    forest = sbf.SafeBoxForest(robot, cfg)
    start = time.perf_counter()
    result = dict(forest.prewarm_lifelong_cache(actual_depth, [far_obstacle()]))
    wall_s = time.perf_counter() - start
    verify_ok = bool(forest.database_verify(True)) if verify and hasattr(forest, "database_verify") else True
    snapshot_ok = bool(forest.database_wait_for_snapshot_publish()) if publish_snapshot and hasattr(forest, "database_wait_for_snapshot_publish") else False
    payload.update({
        "ok": bool(result.get("ok")) and verify_ok,
        "reused_existing": False,
        "wall_s": wall_s,
        "prewarm": result,
        "verify_ok": verify_ok,
        "snapshot_ok": snapshot_ok,
        "snapshot_path": str(path / "lect_snapshot"),
        "snapshot_exists": (path / "lect_snapshot").exists(),
        "cache_bytes": directory_size(path),
    })
    return payload


def robot_external_evidence_path(robot_name: str, *, cache_root: Path = ROBOT_LECTDB_CACHE_ROOT) -> Path:
    if str(robot_name) == "iiwa":
        return robot_lectdb_path(robot_name)
    return Path(cache_root) / robot_lectdb_path(robot_name).name
