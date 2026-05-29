from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.cache_maintenance import copytree_fresh  # noqa: E402
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    RBF_LIFELONG_PRESET,
    add_common_sbf_args,
    configure_standalone_sbf,
    rbf_lifelong_config_metadata,
    set_online_cache_backfill,
)
from safe_box_forest.experiments.sbf_old.common_scene_sampling import make_robot  # noqa: E402

import sbf  # noqa: E402


DEFAULT_RANDOM_P18_PREWARM_DEPTH = 18
DEFAULT_RANDOM_P18_THREADS = 8
DEFAULT_RANDOM_P18_MAX_DEPTH = 40
DEFAULT_RANDOM_P18_ENVELOPE = "support_hull"
DEFAULT_RANDOM_CACHE_RUN_ID = "exp06_random_anytime_canonical_native"


def canonical_random_p18_cache_label(
    robot_name: str,
    *,
    prewarm_depth: int = DEFAULT_RANDOM_P18_PREWARM_DEPTH,
    max_depth: int = DEFAULT_RANDOM_P18_MAX_DEPTH,
    envelope: str = DEFAULT_RANDOM_P18_ENVELOPE,
) -> str:
    return f"exp06_{robot_name}_p{int(prewarm_depth)}_{envelope}_d{int(max_depth)}_canonical_native"


def far_obstacle() -> Any:
    return sbf.Obstacle(100.0, 100.0, 100.0, 101.0, 101.0, 101.0)


def scene_stage_namespace(
    cache_run_id: str,
    *,
    robot_name: str,
    method: str,
    stage_id: str,
    difficulty: str,
    scene_seed: int,
) -> str:
    base = f"{cache_run_id}_{robot_name}_{method}_{stage_id}"
    return f"{base}_{difficulty}_seed{int(scene_seed)}"


def _manifest_lines(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, sep, value = line.partition("=")
        if sep:
            values[key] = value
    return values


def _matches_random_p18_manifest(path: Path, *, envelope: str) -> bool:
    manifest = _manifest_lines(path / "manifest.json")
    if not manifest:
        return False
    if str(manifest.get("canonical_mode", "0")) != "1":
        return False
    if "joint_symmetry_native_v1" not in str(manifest.get("symmetry_descriptor", "joint_symmetry_native_v1")) and str(manifest.get("symmetry_descriptor", "")) not in {"", "joint_symmetry_native_v1"}:
        return False
    envelope_descriptor = str(manifest.get("envelope_descriptor", ""))
    if envelope == "support_hull" and "type=2" not in envelope_descriptor:
        return False
    return True


def _make_prewarm_args(
    cache_root: Path,
    cache_label: str,
    *,
    prewarm_depth: int,
    envelope: str,
    prewarm_threads: int,
    max_depth: int,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    add_common_sbf_args(parser)
    args = parser.parse_args([])
    args.preset = RBF_LIFELONG_PRESET
    args.rbf_cache_root = cache_root
    args.rbf_cache_label = cache_label
    args.rbf_prewarm_depth = int(prewarm_depth)
    args.rbf_max_depth = int(max_depth)
    args.ffb_depth = int(max_depth)
    args.connector_pave_depth = int(max_depth)
    args.component_connect_ffb_max_depth = int(max_depth)
    args.rbf_envelope = str(envelope)
    args.threads = max(1, int(prewarm_threads))
    args.task_batch_size = max(1, int(prewarm_threads))
    args.rbf_canonical_cache = True
    return args


def ensure_random_robot_p18_cache(
    *,
    cache_root: Path,
    robot_name: str,
    prewarm_depth: int = DEFAULT_RANDOM_P18_PREWARM_DEPTH,
    envelope: str = DEFAULT_RANDOM_P18_ENVELOPE,
    prewarm_threads: int = DEFAULT_RANDOM_P18_THREADS,
    max_depth: int = DEFAULT_RANDOM_P18_MAX_DEPTH,
    dry_run: bool = False,
) -> dict[str, Any]:
    cache_label = canonical_random_p18_cache_label(
        robot_name,
        prewarm_depth=int(prewarm_depth),
        max_depth=int(max_depth),
        envelope=str(envelope),
    )
    cache_path = Path(cache_root) / cache_label
    if dry_run:
        return {
            "ok": True,
            "dry_run": True,
            "robot": robot_name,
            "cache_label": cache_label,
            "cache_path": str(cache_path),
        }
    if _matches_random_p18_manifest(cache_path, envelope=str(envelope)):
        return {
            "ok": True,
            "dry_run": False,
            "robot": robot_name,
            "cache_label": cache_label,
            "cache_path": str(cache_path),
            "reused_existing": True,
            "metadata": {},
        }
    cache_root.mkdir(parents=True, exist_ok=True)
    args = _make_prewarm_args(
        cache_root,
        cache_label,
        prewarm_depth=int(prewarm_depth),
        envelope=str(envelope),
        prewarm_threads=int(prewarm_threads),
        max_depth=int(max_depth),
    )
    robot = make_robot(robot_name)
    cfg = configure_standalone_sbf(args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    set_online_cache_backfill(cfg, True)
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    result = dict(forest.prewarm_lifelong_cache(int(prewarm_depth), [far_obstacle()]))
    wall_s = time.perf_counter() - t0
    verify_ok = bool(forest.database_verify(True)) if hasattr(forest, "database_verify") else True
    payload = {
        "ok": bool(result.get("ok")) and bool(verify_ok),
        "dry_run": False,
        "robot": robot_name,
        "cache_label": cache_label,
        "cache_path": str(cache_path),
        "reused_existing": False,
        "wall_s": float(wall_s),
        "verify_ok": bool(verify_ok),
        "result": result,
        "metadata": rbf_lifelong_config_metadata(cfg, args),
    }
    del forest
    return payload


def seed_scene_stage_eval_caches_from_p18(
    *,
    cache_root: Path,
    cache_run_id: str,
    robot_names: Iterable[str],
    method_names: Iterable[str],
    stage_ids: Iterable[str],
    difficulties: Iterable[str],
    scene_seeds: int,
    p18_cache_labels: dict[str, str],
    dry_run: bool = False,
) -> dict[str, Any]:
    namespaces: list[str] = []
    sources: dict[str, str] = {}
    for robot_name in robot_names:
        cache_label = p18_cache_labels[str(robot_name)]
        source = cache_root / cache_label
        if not dry_run and not source.exists():
            raise FileNotFoundError(f"random p18 cache does not exist: {source}")
        for method_name in method_names:
            for stage_id in stage_ids:
                for difficulty in difficulties:
                    for scene_seed in range(max(1, int(scene_seeds))):
                        namespace = scene_stage_namespace(
                            str(cache_run_id),
                            robot_name=str(robot_name),
                            method=str(method_name),
                            stage_id=str(stage_id),
                            difficulty=str(difficulty),
                            scene_seed=int(scene_seed),
                        )
                        namespaces.append(namespace)
                        sources[namespace] = cache_label
                        if dry_run:
                            continue
                        copytree_fresh(source, cache_root / namespace)
    return {
        "dry_run": bool(dry_run),
        "seeded_namespace_count": len(namespaces),
        "seeded_namespaces": namespaces,
        "source_cache_labels": sources,
    }