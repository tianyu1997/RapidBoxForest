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
    make_aafk_volume_min_split_policy,
    rbf_lifelong_config_metadata,
    set_online_cache_backfill,
)

import sbf


DEFAULT_P18_CACHE_LABEL = "iiwa_shelf_endpoint_only_p18_canonical_dim0q4"
DEFAULT_P18_NATIVE_CACHE_LABEL = "iiwa_shelf_endpoint_only_p18_native_dim0q4"
DEFAULT_PREWARM_THREADS = 8
DEFAULT_SNAPSHOT_DIRNAME = "lect_snapshot"
DEFAULT_AAFK_SCHEDULE_DEPTH = 50
ENDPOINT_AAFK = "aafk"
ENDPOINT_IFK = "ifk"
ENDPOINT_HIFK = "hifk"
ENDPOINT_CRITSAMPLE = "critsample"
ENDPOINT_ANALYTICAL = "analytical"
ENDPOINT_MC = "mc"
ENDPOINT_SOURCE_ALIASES = {
    "aafk": ENDPOINT_AAFK,
    "ifk": ENDPOINT_AAFK,
    "ifk_aa": ENDPOINT_AAFK,
    "hifk": ENDPOINT_HIFK,
    "hifk_aa": ENDPOINT_HIFK,
    "crit": ENDPOINT_CRITSAMPLE,
    "crit_sample": ENDPOINT_CRITSAMPLE,
    "critsample": ENDPOINT_CRITSAMPLE,
    "analytical": ENDPOINT_ANALYTICAL,
    "analytic": ENDPOINT_ANALYTICAL,
    "mc": ENDPOINT_MC,
    "monte_carlo": ENDPOINT_MC,
}
SUPPORTED_ENDPOINT_SOURCES = (
    ENDPOINT_AAFK,
    ENDPOINT_IFK,
    ENDPOINT_HIFK,
    ENDPOINT_CRITSAMPLE,
    ENDPOINT_ANALYTICAL,
    ENDPOINT_MC,
)
ENDPOINT_CACHE_CHANNELS = {
    ENDPOINT_AAFK: "safe",
    ENDPOINT_HIFK: "safe",
    ENDPOINT_CRITSAMPLE: "rapid",
    ENDPOINT_ANALYTICAL: "rapid",
    ENDPOINT_MC: "rapid",
}
ENDPOINT_CPP_NAMES = {
    ENDPOINT_AAFK: "IFK",
    ENDPOINT_HIFK: "HIFK",
    ENDPOINT_CRITSAMPLE: "CritSample",
    ENDPOINT_ANALYTICAL: "Analytical",
    ENDPOINT_MC: "MC",
}
LECT_SPLIT_AAFK_VOLUME_MIN = "aafk_volume_min"
LECT_SPLIT_ROUND_ROBIN = "round_robin"
CANONICAL_SYMMETRY_DESCRIPTOR = "joint_symmetry_native_v1"


def normalize_endpoint_source(endpoint_source: str) -> str:
    key = str(endpoint_source).strip().lower().replace("-", "_")
    if key not in ENDPOINT_SOURCE_ALIASES:
        raise ValueError(f"unsupported cache endpoint source {endpoint_source!r}")
    return ENDPOINT_SOURCE_ALIASES[key]


def endpoint_cache_channel(endpoint_source: str) -> str:
    return ENDPOINT_CACHE_CHANNELS[normalize_endpoint_source(endpoint_source)]


def endpoint_cpp_name(endpoint_source: str) -> str:
    return ENDPOINT_CPP_NAMES[normalize_endpoint_source(endpoint_source)]


def endpoint_enum(endpoint_source: str) -> Any:
    name = endpoint_cpp_name(endpoint_source)
    try:
        return getattr(sbf.EndpointSource, name)
    except AttributeError as exc:
        raise ValueError(f"EndpointSource.{name} is not exposed by the current sbf binding") from exc


def cache_identity_fingerprint(
    robot: Any,
    *,
    prewarm_depth: int,
    max_depth: int,
    envelope: str,
    endpoint_source: str,
    lect_split_policy: str,
    canonical_mode: bool = True,
) -> dict[str, Any]:
    source = normalize_endpoint_source(endpoint_source)
    return {
        "robot_name": str(robot.name()),
        "robot_fingerprint": int(robot.fingerprint()),
        "canonical_mode": bool(canonical_mode),
        "symmetry_descriptor": CANONICAL_SYMMETRY_DESCRIPTOR if bool(canonical_mode) else "",
        "prewarm_depth": int(prewarm_depth),
        "max_depth": int(max_depth),
        "envelope": str(envelope),
        "endpoint_source": source,
        "endpoint_cpp_source": endpoint_cpp_name(source),
        "endpoint_channel": endpoint_cache_channel(source),
        "lect_split_policy": str(lect_split_policy),
        "payload_layout": "endpoint_envelope_v1",
        "builder_version": "sbf_online_cache_v1",
    }


def channel_cache_label(base_label: str, endpoint_source: str, lect_split_policy: str) -> str:
    source = normalize_endpoint_source(endpoint_source)
    parts = [str(base_label)]
    if source != ENDPOINT_AAFK:
        parts.append(f"{endpoint_cache_channel(source)}_{source}")
    if str(lect_split_policy) != LECT_SPLIT_AAFK_VOLUME_MIN:
        parts.append(f"split_{lect_split_policy}")
    return "__".join(parts)


def configure_cache_endpoint(cfg: Any, endpoint_source: str) -> None:
    cfg.endpoint_source.source = endpoint_enum(endpoint_source)


def configure_cache_split_policy(cfg: Any, robot: Any, lect_split_policy: str, max_depth: int) -> None:
    if lect_split_policy == LECT_SPLIT_AAFK_VOLUME_MIN:
        cfg.database.split_policy = make_aafk_volume_min_split_policy(robot, int(max_depth))
    elif lect_split_policy == LECT_SPLIT_ROUND_ROBIN:
        descriptor = sbf.SplitPolicyDescriptor()
        descriptor.strategy = sbf.SplitStrategy.RoundRobin
        descriptor.min_width = 0.0
        descriptor.midpoint = True
        descriptor.deterministic_tie_break = True
        cfg.database.split_policy = descriptor
    else:
        raise ValueError(f"unsupported cache LECT split policy {lect_split_policy!r}")


def endpoint_manifest_matches(manifest: dict[str, str], endpoint_source: str) -> bool:
    descriptor = str(manifest.get("endpoint_descriptor", ""))
    source = normalize_endpoint_source(endpoint_source)
    return (
        f"channel={endpoint_cache_channel(source)}" in descriptor
        and f"source={endpoint_cpp_name(source)}" in descriptor
    )


def canonical_manifest_matches(manifest: dict[str, str], robot: Any, canonical_mode: bool = True) -> bool:
    expected_canonical = "1" if bool(canonical_mode) else "0"
    if str(manifest.get("canonical_mode", "0")) != expected_canonical:
        return False
    expected_symmetry = CANONICAL_SYMMETRY_DESCRIPTOR if bool(canonical_mode) else ""
    if str(manifest.get("symmetry_descriptor", "")) != expected_symmetry:
        return False
    try:
        if int(manifest.get("robot_fingerprint", "-1")) != int(robot.fingerprint()):
            return False
    except ValueError:
        return False
    return True


def split_manifest_matches(manifest: dict[str, str], lect_split_policy: str) -> bool:
    expected_strategy = {LECT_SPLIT_ROUND_ROBIN: 0, LECT_SPLIT_AAFK_VOLUME_MIN: 2}.get(str(lect_split_policy))
    if expected_strategy is None:
        return False
    try:
        return int(manifest.get("split_strategy", "-1")) == int(expected_strategy)
    except ValueError:
        return False


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


def ensure_cache_snapshot(
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    max_depth: int,
    endpoint_source: str,
    lect_split_policy: str,
    canonical_mode: bool,
    dry_run: bool,
) -> dict[str, Any]:
    snapshot_path = snapshot_path_for_cache(cache_path)
    summary = snapshot_summary(snapshot_path)
    summary["ensured"] = False
    if dry_run or summary["exists"]:
        return summary

    prewarm_args = make_prewarm_config_args(
        cache_path,
        prewarm_depth,
        envelope,
        prewarm_threads,
        max_depth,
        endpoint_source,
        lect_split_policy,
        canonical_mode,
    )
    robot = sbf.load_iiwa14_robot()
    cfg = configure_standalone_sbf(prewarm_args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    configure_cache_endpoint(cfg, str(endpoint_source))
    configure_cache_split_policy(cfg, robot, str(lect_split_policy), int(max_depth))
    set_online_cache_backfill(cfg, True)
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    publish_ok = bool(forest.database_wait_for_snapshot_publish())
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


def make_prewarm_config_args(
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    max_depth: int,
    endpoint_source: str = ENDPOINT_AAFK,
    lect_split_policy: str = LECT_SPLIT_AAFK_VOLUME_MIN,
    canonical_mode: bool = True,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    add_common_sbf_args(parser)
    args = parser.parse_args([])
    args.preset = RBF_LIFELONG_PRESET
    args.rbf_cache_root = cache_path.parent
    args.rbf_cache_label = cache_path.name
    args.rbf_prewarm_depth = int(prewarm_depth)
    args.rbf_max_depth = int(max_depth)
    args.ffb_depth = int(max_depth)
    args.connector_pave_depth = int(max_depth)
    args.rbf_ffb_start_depth = min(int(getattr(args, "rbf_ffb_start_depth", 0)), int(max_depth))
    args.rbf_envelope = str(envelope)
    args.endpoint_source = normalize_endpoint_source(endpoint_source)
    args.lect_split_policy = str(lect_split_policy)
    args.rbf_canonical_cache = bool(canonical_mode)
    args.threads = max(1, int(prewarm_threads))
    args.task_batch_size = max(1, int(prewarm_threads))
    return args


def run_p18_prewarm(
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    max_depth: int,
    clean_cache: bool,
    dry_run: bool,
    endpoint_source: str = ENDPOINT_AAFK,
    lect_split_policy: str = LECT_SPLIT_AAFK_VOLUME_MIN,
    canonical_mode: bool = True,
) -> dict[str, Any]:
    prewarm_args = make_prewarm_config_args(
        cache_path,
        prewarm_depth,
        envelope,
        prewarm_threads,
        max_depth,
        endpoint_source,
        lect_split_policy,
        canonical_mode,
    )
    if dry_run:
        return {
            "ok": True,
            "dry_run": True,
            "cache_path": str(cache_path),
            "cache_bytes": 0,
            "cache_file_sizes": {},
            "snapshot": snapshot_summary(snapshot_path_for_cache(cache_path)),
            "verify_ok": None,
            "metadata": {
                "prewarm_depth": int(prewarm_depth),
                "envelope": str(envelope),
                "prewarm_threads": int(prewarm_threads),
                "max_depth": int(max_depth),
                "endpoint_source": str(endpoint_source),
                "lect_split_policy": str(lect_split_policy),
                "canonical_mode": bool(canonical_mode),
            },
            "prewarm": {},
        }

    if clean_cache and cache_path.exists():
        shutil.rmtree(cache_path)

    robot = sbf.load_iiwa14_robot()
    cfg = configure_standalone_sbf(prewarm_args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    configure_cache_endpoint(cfg, str(endpoint_source))
    configure_cache_split_policy(cfg, robot, str(lect_split_policy), int(max_depth))
    set_online_cache_backfill(cfg, True)
    identity = cache_identity_fingerprint(
        robot,
        prewarm_depth=int(prewarm_depth),
        max_depth=int(max_depth),
        envelope=str(envelope),
        endpoint_source=str(endpoint_source),
        lect_split_policy=str(lect_split_policy),
        canonical_mode=bool(canonical_mode),
    )
    metadata = rbf_lifelong_config_metadata(cfg, prewarm_args)
    metadata["prewarm_threads"] = int(prewarm_threads)
    metadata["cache_identity"] = identity
    metadata["endpoint_source"] = identity["endpoint_source"]
    metadata["endpoint_channel"] = identity["endpoint_channel"]
    metadata["endpoint_source_raw"] = str(cfg.endpoint_source.source).split(".")[-1]
    metadata["lect_split_policy"] = str(lect_split_policy)
    metadata["split_policy"] = str(lect_split_policy)
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


def existing_p18_cache_summary(
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    max_depth: int,
    endpoint_source: str,
    lect_split_policy: str,
    canonical_mode: bool,
) -> dict[str, Any]:
    manifest = read_manifest(cache_path / "manifest.json")
    evidence_after = int(manifest.get("generation", manifest.get("node_count", "0")) or 0) if manifest else 0
    snapshot = ensure_cache_snapshot(
        cache_path,
        prewarm_depth,
        envelope,
        prewarm_threads,
        max_depth,
        endpoint_source,
        lect_split_policy,
        canonical_mode,
        dry_run=False,
    )
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
            "endpoint_source": str(endpoint_source),
            "lect_split_policy": str(lect_split_policy),
            "canonical_mode": bool(canonical_mode),
            "prewarm_threads": int(prewarm_threads),
            "max_depth": int(max_depth),
            "lect_schedule_depth": manifest_schedule_depth(manifest),
        },
        "prewarm": {
            "skipped_existing_cache": True,
            "prewarm_depth": int(prewarm_depth),
            "threads_requested": int(prewarm_threads),
            "evidence_after": evidence_after,
        },
    }


def prewarm_summary_matches(
    summary: dict[str, Any],
    *,
    cache_path: Path,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    max_depth: int,
    endpoint_source: str,
    lect_split_policy: str,
    canonical_mode: bool,
) -> bool:
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
        and str(metadata.get("endpoint_source", ENDPOINT_AAFK)) == str(endpoint_source)
        and str(metadata.get("lect_split_policy", LECT_SPLIT_AAFK_VOLUME_MIN)) == str(lect_split_policy)
        and bool(metadata.get("canonical_mode", True)) == bool(canonical_mode)
        and int(metadata.get("prewarm_threads", -1)) == int(prewarm_threads)
        and int(metadata.get("max_depth", metadata.get("lect_schedule_depth", -1))) == int(max_depth)
    )


def manifest_schedule_depth(manifest: dict[str, str]) -> int:
    raw = str(manifest.get("split_depth_dimensions", ""))
    if not raw:
        return -1
    return len([item for item in raw.split(",") if item.strip()])


def p18_cache_manifest_matches(
    cache_path: Path,
    *,
    envelope: str,
    max_depth: int,
    endpoint_source: str,
    lect_split_policy: str,
    canonical_mode: bool,
) -> bool:
    manifest = read_manifest(cache_path / "manifest.json")
    if not manifest:
        return False
    robot = sbf.load_iiwa14_robot()
    if not canonical_manifest_matches(manifest, robot, canonical_mode=bool(canonical_mode)):
        return False
    if manifest_schedule_depth(manifest) != int(max_depth):
        return False
    if not endpoint_manifest_matches(manifest, str(endpoint_source)):
        return False
    if not split_manifest_matches(manifest, str(lect_split_policy)):
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
    max_depth: int = DEFAULT_AAFK_SCHEDULE_DEPTH,
    endpoint_source: str = ENDPOINT_AAFK,
    lect_split_policy: str = LECT_SPLIT_AAFK_VOLUME_MIN,
    canonical_mode: bool = True,
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
            max_depth=int(max_depth),
            endpoint_source=str(endpoint_source),
            lect_split_policy=str(lect_split_policy),
            canonical_mode=bool(canonical_mode),
        ):
            return existing
        return run_p18_prewarm(
            cache_path=cache_path,
            prewarm_depth=int(prewarm_depth),
            envelope=str(envelope),
            prewarm_threads=int(prewarm_threads),
            max_depth=int(max_depth),
            endpoint_source=str(endpoint_source),
            lect_split_policy=str(lect_split_policy),
            canonical_mode=bool(canonical_mode),
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
            max_depth=int(max_depth),
            endpoint_source=str(endpoint_source),
            lect_split_policy=str(lect_split_policy),
            canonical_mode=bool(canonical_mode),
        ):
            existing["snapshot"] = ensure_cache_snapshot(
                cache_path,
                int(prewarm_depth),
                str(envelope),
                int(prewarm_threads),
                int(max_depth),
                str(endpoint_source),
                str(lect_split_policy),
                bool(canonical_mode),
                dry_run=False,
            )
            write_json(prewarm_json, existing)
            return existing
    if not clean_cache and (cache_path / "manifest.json").exists() and p18_cache_manifest_matches(
        cache_path,
        envelope=str(envelope),
        max_depth=int(max_depth),
        endpoint_source=str(endpoint_source),
        lect_split_policy=str(lect_split_policy),
        canonical_mode=bool(canonical_mode),
    ):
        summary = existing_p18_cache_summary(
            cache_path=cache_path,
            prewarm_depth=int(prewarm_depth),
            envelope=str(envelope),
            prewarm_threads=int(prewarm_threads),
            max_depth=int(max_depth),
            endpoint_source=str(endpoint_source),
            lect_split_policy=str(lect_split_policy),
            canonical_mode=bool(canonical_mode),
        )
        write_json(prewarm_json, summary)
        return summary
    rebuild_clean = bool(clean_cache or cache_path.exists() or (prewarm_json.exists() and not bool(existing.get("dry_run"))))
    summary = run_p18_prewarm(
        cache_path=cache_path,
        prewarm_depth=int(prewarm_depth),
        envelope=str(envelope),
        prewarm_threads=int(prewarm_threads),
        max_depth=int(max_depth),
        endpoint_source=str(endpoint_source),
        lect_split_policy=str(lect_split_policy),
        canonical_mode=bool(canonical_mode),
        clean_cache=bool(rebuild_clean),
        dry_run=bool(dry_run),
    )
    write_json(prewarm_json, summary)
    return summary