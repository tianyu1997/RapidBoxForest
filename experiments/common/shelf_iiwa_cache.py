from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import write_json
from safe_box_forest.experiments.sbf_old.common_sbf_config import (
    RBF_LIFELONG_PRESET,
    add_common_sbf_args,
    configure_standalone_sbf,
    rbf_lifelong_config_metadata,
    set_online_cache_backfill,
)

import sbf


DEFAULT_P18_CACHE_LABEL = "iiwa_shelf_endpoint_only_p18_canonical_dim0q4"
DEFAULT_PREWARM_THREADS = 8
DEFAULT_SNAPSHOT_DIRNAME = "lect_snapshot"
DEFAULT_AAFK_SCHEDULE_DEPTH = 50


def directory_size(path: Path | None) -> int:
    if path is None or not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def cache_file_sizes(path: Path | None) -> dict[str, int]:
    if path is None or not path.exists() or not path.is_dir():
        return {}
    return {item.name: item.stat().st_size for item in sorted(path.iterdir()) if item.is_file()}


def snapshot_path_for_cache(cache_path: Path) -> Path:
    return cache_path / DEFAULT_SNAPSHOT_DIRNAME


def snapshot_summary(snapshot_path: Path) -> dict[str, Any]:
    return {
        "path": str(snapshot_path),
        "exists": snapshot_path.exists(),
        "bytes": directory_size(snapshot_path),
        "file_sizes": cache_file_sizes(snapshot_path),
    }


def ensure_cache_snapshot(cache_path: Path, prewarm_depth: int, envelope: str, prewarm_threads: int, dry_run: bool) -> dict[str, Any]:
    snapshot_path = snapshot_path_for_cache(cache_path)
    summary = snapshot_summary(snapshot_path)
    summary["ensured"] = False
    if dry_run or summary["exists"]:
        return summary

    prewarm_args = make_prewarm_config_args(cache_path, prewarm_depth, envelope, prewarm_threads)
    robot = sbf.load_iiwa14_robot()
    cfg = configure_standalone_sbf(prewarm_args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    set_online_cache_backfill(cfg, True)
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    publish_ok = bool(forest.database_publish_snapshot(True))
    actual_snapshot_path = Path(forest.database_snapshot_path() or str(snapshot_path))
    summary = snapshot_summary(actual_snapshot_path)
    summary["ensured"] = True
    summary["publish_ok"] = publish_ok
    summary["publish_wait_s"] = time.perf_counter() - t0
    del forest
    return summary


def read_manifest(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, sep, value = line.partition("=")
        if sep:
            values[key] = value
    return values


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def far_obstacle() -> Any:
    return sbf.Obstacle(100.0, 100.0, 100.0, 101.0, 101.0, 101.0)


def make_prewarm_config_args(cache_path: Path, prewarm_depth: int, envelope: str, prewarm_threads: int) -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    add_common_sbf_args(parser)
    args = parser.parse_args([])
    args.preset = RBF_LIFELONG_PRESET
    args.rbf_cache_root = cache_path.parent
    args.rbf_cache_label = cache_path.name
    args.rbf_prewarm_depth = int(prewarm_depth)
    args.rbf_max_depth = DEFAULT_AAFK_SCHEDULE_DEPTH
    args.ffb_depth = DEFAULT_AAFK_SCHEDULE_DEPTH
    args.connector_pave_depth = DEFAULT_AAFK_SCHEDULE_DEPTH
    args.rbf_envelope = str(envelope)
    args.threads = max(1, int(prewarm_threads))
    args.task_batch_size = max(1, int(prewarm_threads))
    return args


def run_p18_prewarm(cache_path: Path, prewarm_depth: int, envelope: str, prewarm_threads: int, clean_cache: bool, dry_run: bool) -> dict[str, Any]:
    prewarm_args = make_prewarm_config_args(cache_path, prewarm_depth, envelope, prewarm_threads)
    if dry_run:
        return {
            "ok": True,
            "dry_run": True,
            "cache_path": str(cache_path),
            "cache_bytes": 0,
            "cache_file_sizes": {},
            "snapshot": snapshot_summary(snapshot_path_for_cache(cache_path)),
            "verify_ok": None,
            "metadata": {"prewarm_depth": int(prewarm_depth), "envelope": str(envelope), "prewarm_threads": int(prewarm_threads)},
            "prewarm": {},
        }

    if clean_cache and cache_path.exists():
        shutil.rmtree(cache_path)

    robot = sbf.load_iiwa14_robot()
    cfg = configure_standalone_sbf(prewarm_args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    set_online_cache_backfill(cfg, True)
    metadata = rbf_lifelong_config_metadata(cfg, prewarm_args)
    metadata["prewarm_threads"] = int(prewarm_threads)
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    prewarm = dict(forest.prewarm_lifelong_cache(int(prewarm_depth), [far_obstacle()]))
    prewarm["wall_s_outer"] = time.perf_counter() - t0
    verify_ok = bool(forest.database_verify(True))
    snapshot_wait_t0 = time.perf_counter()
    snapshot_wait_ok = bool(forest.database_wait_for_snapshot_publish())
    actual_snapshot_path = Path(forest.database_snapshot_path() or str(snapshot_path_for_cache(cache_path)))
    snapshot = snapshot_summary(actual_snapshot_path)
    snapshot["wait_ok"] = snapshot_wait_ok
    snapshot["publish_wait_s"] = time.perf_counter() - snapshot_wait_t0
    del forest

    return {
        "ok": bool(prewarm.get("ok")) and verify_ok and snapshot_wait_ok,
        "dry_run": False,
        "cache_path": str(cache_path),
        "cache_bytes": directory_size(cache_path),
        "cache_file_sizes": cache_file_sizes(cache_path),
        "snapshot": snapshot,
        "verify_ok": verify_ok,
        "manifest": read_manifest(cache_path / "manifest.json"),
        "metadata": metadata,
        "prewarm": prewarm,
    }


def existing_p18_cache_summary(cache_path: Path, prewarm_depth: int, envelope: str, prewarm_threads: int) -> dict[str, Any]:
    manifest = read_manifest(cache_path / "manifest.json")
    evidence_after = int(manifest.get("generation", manifest.get("node_count", "0")) or 0) if manifest else 0
    snapshot = ensure_cache_snapshot(cache_path, prewarm_depth, envelope, prewarm_threads, dry_run=False)
    return {
        "ok": bool(manifest) and bool(snapshot.get("exists")),
        "dry_run": False,
        "reused_existing_cache": True,
        "cache_path": str(cache_path),
        "cache_bytes": directory_size(cache_path),
        "cache_file_sizes": cache_file_sizes(cache_path),
        "snapshot": snapshot,
        "verify_ok": None,
        "manifest": manifest,
        "metadata": {
            "prewarm_depth": int(prewarm_depth),
            "envelope": str(envelope),
            "prewarm_threads": int(prewarm_threads),
            "max_depth": manifest_schedule_depth(manifest),
            "lect_schedule_depth": manifest_schedule_depth(manifest),
        },
        "prewarm": {
            "skipped_existing_cache": True,
            "prewarm_depth": int(prewarm_depth),
            "threads_requested": int(prewarm_threads),
            "evidence_after": evidence_after,
        },
    }


def prewarm_summary_matches(summary: dict[str, Any], *, cache_path: Path, prewarm_depth: int, envelope: str, prewarm_threads: int) -> bool:
    if not summary:
        return False
    if bool(summary.get("dry_run")):
        return False
    if str(summary.get("cache_path", "")) != str(cache_path):
        return False
    metadata = summary.get("metadata", {})
    return (
        int(metadata.get("prewarm_depth", -1)) == int(prewarm_depth)
        and str(metadata.get("envelope", "")) == str(envelope)
        and int(metadata.get("prewarm_threads", -1)) == int(prewarm_threads)
        and int(metadata.get("max_depth", metadata.get("lect_schedule_depth", -1))) >= DEFAULT_AAFK_SCHEDULE_DEPTH
    )


def manifest_schedule_depth(manifest: dict[str, str]) -> int:
    raw = str(manifest.get("split_depth_dimensions", ""))
    if not raw:
        return -1
    return len([item for item in raw.split(",") if item.strip()])


def p18_cache_manifest_matches(cache_path: Path, *, envelope: str) -> bool:
    manifest = read_manifest(cache_path / "manifest.json")
    if not manifest:
        return False
    if manifest_schedule_depth(manifest) < DEFAULT_AAFK_SCHEDULE_DEPTH:
        return False
    envelope_descriptor = str(manifest.get("envelope_descriptor", ""))
    if str(envelope) == "support_hull" and "type=2" not in envelope_descriptor:
        return False
    if str(envelope) == "link" and "type=0" not in envelope_descriptor:
        return False
    return True


def ensure_p18_prewarm_summary(
    prewarm_json: Path,
    *,
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int = DEFAULT_PREWARM_THREADS,
    clean_cache: bool,
    dry_run: bool,
) -> dict[str, Any]:
    if dry_run:
        existing = load_json(prewarm_json)
        if prewarm_summary_matches(
            existing,
            cache_path=cache_path,
            prewarm_depth=int(prewarm_depth),
            envelope=str(envelope),
            prewarm_threads=int(prewarm_threads),
        ):
            return existing
        return run_p18_prewarm(
            cache_path=cache_path,
            prewarm_depth=int(prewarm_depth),
            envelope=str(envelope),
            prewarm_threads=int(prewarm_threads),
            clean_cache=False,
            dry_run=True,
        )

    reuse_existing = bool(not dry_run and not clean_cache and prewarm_json.exists())
    existing = load_json(prewarm_json) if prewarm_json.exists() else {}
    if reuse_existing:
        if prewarm_summary_matches(
            existing,
            cache_path=cache_path,
            prewarm_depth=int(prewarm_depth),
            envelope=str(envelope),
            prewarm_threads=int(prewarm_threads),
        ):
            existing["snapshot"] = ensure_cache_snapshot(
                cache_path,
                int(prewarm_depth),
                str(envelope),
                int(prewarm_threads),
                dry_run=False,
            )
            write_json(prewarm_json, existing)
            return existing
    if not clean_cache and (cache_path / "manifest.json").exists() and p18_cache_manifest_matches(cache_path, envelope=str(envelope)):
        summary = existing_p18_cache_summary(
            cache_path=cache_path,
            prewarm_depth=int(prewarm_depth),
            envelope=str(envelope),
            prewarm_threads=int(prewarm_threads),
        )
        write_json(prewarm_json, summary)
        return summary
    rebuild_clean = bool(clean_cache or cache_path.exists() or (prewarm_json.exists() and not bool(existing.get("dry_run"))))
    summary = run_p18_prewarm(
        cache_path=cache_path,
        prewarm_depth=int(prewarm_depth),
        envelope=str(envelope),
        prewarm_threads=int(prewarm_threads),
        clean_cache=bool(rebuild_clean),
        dry_run=bool(dry_run),
    )
    write_json(prewarm_json, summary)
    return summary