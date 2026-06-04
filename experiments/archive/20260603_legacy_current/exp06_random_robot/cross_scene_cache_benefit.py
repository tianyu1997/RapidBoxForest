#!/usr/bin/env python3
"""Cross-scene cache benefit on random scenes.

This experiment builds the SBF coverage forest over a sequence of random
scenes and contrasts two cache regimes that are otherwise identical:

  * ``accumulate``: every scene reuses the *same* persistent canonical LECT
    database. Obstacle-independent envelope/endpoint geometry computed for
    earlier scenes is read back during later scenes, so the cache grows over
    the sequence.
  * ``fresh``: every scene starts from an empty database, so no geometry is
    ever shared across scenes.

Both regimes use the lifelong RBF preset (IFK endpoints, support-hull
envelope, canonical cache, checkpoint + database backfill) and run on the
exact same pre-sampled scene list, so per-scene build cost is directly
comparable. The headline question is whether accumulating the cache makes
later scenes (and the cumulative build) measurably faster.
"""
from __future__ import annotations

import argparse
import shutil
import statistics
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (REPO_ROOT, REPO_ROOT.parent):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    environment_metadata,
    namespace_dict,
    run_id,
    write_json,
)
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    RBF_LIFELONG_PRESET,
    add_common_sbf_args,
    configure_standalone_sbf,
)
from safe_box_forest.experiments.sbf_old.common_scene_sampling import (  # noqa: E402
    RANDOM_DIFFICULTY_ORDER,
    SceneSpec,
    make_nested_random_scene,
    make_robot,
)

import sbf  # noqa: E402


CACHE_DIAGNOSTIC_KEYS = (
    "grower.worker_oracle.materializations",
    "grower.worker_oracle.materialization_reused_cached_envelope",
    "grower.worker_oracle.materialization_reused_endpoint_cache",
    "grower.worker_oracle.materialization_reused_shared_endpoint_cache",
    "grower.worker_oracle.materialization_envelope_compute_time_us",
    "grower.worker_oracle.materialization_external_lookup_time_us",
    "oracle.materializations",
    "oracle.materialization_reused_cached_envelope",
)


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp06_cross_scene_cache"
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_sbf_args(parser)
    parser.set_defaults(
        preset=RBF_LIFELONG_PRESET,
        rbf_envelope="support_hull",
        rbf_canonical_cache=True,
        threads=8,
        task_batch_size=8,
        rbf_max_depth=40,
        rbf_ffb_start_depth=15,
        max_boxes=800,
        timeout_ms=8000.0,
    )
    parser.add_argument("--robot", default="iiwa", choices=["iiwa", "ur5", "panda"])
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--num-scenes", type=int, default=12)
    parser.add_argument("--scene-seed-base", type=int, default=70000)
    parser.add_argument("--max-scene-tries", type=int, default=400)
    parser.add_argument(
        "--balanced-probe",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Run the OMPL solvability probe while sampling scenes (slower, more realistic).",
    )
    parser.add_argument("--out-json", type=Path, default=output_dir / "cross_scene_cache_benefit.json")
    parser.add_argument("--cache-root", type=Path, default=output_dir / "cache")
    parser.add_argument(
        "--modes",
        default="accumulate,fresh",
        help="Comma list of regimes to run; subset of {accumulate,fresh}.",
    )
    return parser.parse_args()


def csv_items(text: str) -> list[str]:
    return [item.strip() for item in str(text).split(",") if item.strip()]


def sample_scene_list(args: argparse.Namespace) -> list[SceneSpec]:
    difficulties = csv_items(args.difficulties) or list(RANDOM_DIFFICULTY_ORDER)
    scenes: list[SceneSpec] = []
    for index in range(max(1, int(args.num_scenes))):
        difficulty = difficulties[index % len(difficulties)]
        seed = int(args.scene_seed_base) + 101 * index
        scene = make_nested_random_scene(
            robot_name=str(args.robot),
            difficulty=str(difficulty),
            seed=seed,
            max_scene_tries=int(args.max_scene_tries),
            balanced=bool(args.balanced_probe),
        )
        scenes.append(scene)
        print(
            f"[cross-scene] sampled scene {index} robot={args.robot} difficulty={difficulty} "
            f"obstacles={len(scene.obstacles)} seed={seed}",
            flush=True,
        )
    return scenes


def scene_config(args: argparse.Namespace, robot: Any, cache_label: str, seed: int) -> Any:
    local = argparse.Namespace(**vars(args))
    local.rbf_cache_root = Path(args.cache_root)
    local.rbf_cache_label = cache_label
    # Keep cross-scene reuse deterministic and in-process; avoid async snapshot noise.
    local.rbf_auto_publish_snapshot = False
    local.rbf_auto_publish_snapshot_async = False
    cfg = configure_standalone_sbf(local, seed=seed, preset=RBF_LIFELONG_PRESET, robot=robot)
    return cfg


def directory_size(path: Path) -> int:
    if not path.exists():
        return 0
    total = 0
    for child in path.rglob("*"):
        if child.is_file():
            try:
                total += child.stat().st_size
            except OSError:
                pass
    return total


def build_scene(args: argparse.Namespace,
                robot: Any,
                scene: SceneSpec,
                cache_label: str,
                scene_index: int,
                clean_cache: bool) -> dict[str, Any]:
    cfg = scene_config(args, robot, cache_label, seed=scene_index)
    database_path = Path(cfg.database.path)
    if clean_cache and database_path.exists():
        shutil.rmtree(database_path)
    database_path.parent.mkdir(parents=True, exist_ok=True)

    coverage_seeds = [list(scene.start), list(scene.goal)]
    forest = sbf.SafeBoxForest(robot, cfg)
    t0 = time.perf_counter()
    profile = forest.build_coverage(scene.obstacles, coverage_seeds)
    build_wall_s = time.perf_counter() - t0
    diagnostics = {str(key): float(value) for key, value in dict(profile.diagnostics).items()}
    final_boxes = int(profile.final_boxes)
    cache_diag = {key: diagnostics.get(key, 0.0) for key in CACHE_DIAGNOSTIC_KEYS}
    materializations = cache_diag.get("grower.worker_oracle.materializations", 0.0)
    reused_env = cache_diag.get("grower.worker_oracle.materialization_reused_cached_envelope", 0.0)
    reused_shared_ep = cache_diag.get("grower.worker_oracle.materialization_reused_shared_endpoint_cache", 0.0)
    envelope_reuse_rate = (reused_env / materializations) if materializations > 0 else 0.0
    shared_endpoint_reuse_rate = (reused_shared_ep / materializations) if materializations > 0 else 0.0
    del forest
    return {
        "scene_index": int(scene_index),
        "difficulty": str(scene.difficulty),
        "obstacle_count": int(len(scene.obstacles)),
        "cache_label": cache_label,
        "ok": final_boxes > 0,
        "build_wall_s": float(build_wall_s),
        "grow_ms": float(profile.grow_ms),
        "total_ms": float(profile.total_ms),
        "merge_ms": float(profile.merge_ms),
        "connector_ms": float(profile.connector_ms),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": final_boxes,
        "materializations": float(materializations),
        "reused_cached_envelope": float(reused_env),
        "reused_shared_endpoint_cache": float(reused_shared_ep),
        "envelope_reuse_rate": float(envelope_reuse_rate),
        "shared_endpoint_reuse_rate": float(shared_endpoint_reuse_rate),
        "cache_bytes_after": directory_size(database_path),
        "cache_diagnostics": cache_diag,
    }


def run_mode(args: argparse.Namespace, robot: Any, scenes: list[SceneSpec], mode: str) -> dict[str, Any]:
    accumulate = mode == "accumulate"
    shared_label = f"{args.robot}_cross_scene_{mode}"
    if accumulate:
        # One persistent DB reused across the whole sequence; clean once up front.
        shared_path = Path(args.cache_root) / shared_label
        if shared_path.exists():
            shutil.rmtree(shared_path)
    rows: list[dict[str, Any]] = []
    print(f"[cross-scene] === mode={mode} (accumulate={accumulate}) ===", flush=True)
    for scene_index, scene in enumerate(scenes):
        if accumulate:
            cache_label = shared_label
            clean_cache = False  # never wipe between scenes -> cache grows
        else:
            cache_label = f"{args.robot}_cross_scene_fresh_scene{scene_index}"
            clean_cache = True  # empty DB per scene -> no cross-scene reuse
        row = build_scene(args, robot, scene, cache_label, scene_index, clean_cache)
        rows.append(row)
        print(
            f"[cross-scene] mode={mode} scene={scene_index} difficulty={row['difficulty']} "
            f"ok={row['ok']} grow_ms={row['grow_ms']:.1f} build_wall_s={row['build_wall_s']:.2f} "
            f"boxes={row['final_boxes']} mat={row['materializations']:.0f} "
            f"shared_ep_reuse={row['shared_endpoint_reuse_rate']:.3f}",
            flush=True,
        )
    return {
        "mode": mode,
        "accumulate": accumulate,
        "rows": rows,
        "totals": {
            "build_wall_s": float(sum(r["build_wall_s"] for r in rows)),
            "grow_ms": float(sum(r["grow_ms"] for r in rows)),
            "materializations": float(sum(r["materializations"] for r in rows)),
            "reused_cached_envelope": float(sum(r["reused_cached_envelope"] for r in rows)),
            "reused_shared_endpoint_cache": float(sum(r["reused_shared_endpoint_cache"] for r in rows)),
            "final_boxes": int(sum(r["final_boxes"] for r in rows)),
        },
    }


def median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else 0.0


def summarize(modes: dict[str, dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    if "accumulate" in modes and "fresh" in modes:
        acc = modes["accumulate"]
        fresh = modes["fresh"]
        acc_total = acc["totals"]["build_wall_s"]
        fresh_total = fresh["totals"]["build_wall_s"]
        acc_grow = acc["totals"]["grow_ms"]
        fresh_grow = fresh["totals"]["grow_ms"]
        per_scene = []
        for acc_row, fresh_row in zip(acc["rows"], fresh["rows"]):
            grow_delta = acc_row["grow_ms"] - fresh_row["grow_ms"]
            wall_delta = acc_row["build_wall_s"] - fresh_row["build_wall_s"]
            per_scene.append({
                "scene_index": acc_row["scene_index"],
                "difficulty": acc_row["difficulty"],
                "accumulate_grow_ms": acc_row["grow_ms"],
                "fresh_grow_ms": fresh_row["grow_ms"],
                "grow_ms_delta": grow_delta,
                "grow_ms_delta_pct": (grow_delta / fresh_row["grow_ms"] * 100.0) if fresh_row["grow_ms"] > 0 else 0.0,
                "accumulate_build_wall_s": acc_row["build_wall_s"],
                "fresh_build_wall_s": fresh_row["build_wall_s"],
                "build_wall_s_delta": wall_delta,
                "accumulate_shared_endpoint_reuse_rate": acc_row["shared_endpoint_reuse_rate"],
                "fresh_shared_endpoint_reuse_rate": fresh_row["shared_endpoint_reuse_rate"],
            })
        summary = {
            "cumulative_build_wall_s": {
                "accumulate": acc_total,
                "fresh": fresh_total,
                "delta": acc_total - fresh_total,
                "speedup_pct": ((fresh_total - acc_total) / fresh_total * 100.0) if fresh_total > 0 else 0.0,
            },
            "cumulative_grow_ms": {
                "accumulate": acc_grow,
                "fresh": fresh_grow,
                "delta": acc_grow - fresh_grow,
                "speedup_pct": ((fresh_grow - acc_grow) / fresh_grow * 100.0) if fresh_grow > 0 else 0.0,
            },
            "median_grow_ms": {
                "accumulate": median([r["grow_ms"] for r in acc["rows"]]),
                "fresh": median([r["grow_ms"] for r in fresh["rows"]]),
            },
            "per_scene": per_scene,
        }
    return summary


def main() -> int:
    args = parse_args()
    requested_modes = [mode for mode in csv_items(args.modes) if mode in {"accumulate", "fresh"}]
    if not requested_modes:
        raise ValueError("no valid modes selected; choose from accumulate,fresh")
    robot = make_robot(str(args.robot))
    scenes = sample_scene_list(args)
    modes: dict[str, dict[str, Any]] = {}
    for mode in requested_modes:
        modes[mode] = run_mode(args, robot, scenes, mode)

    payload = {
        "experiment": "exp06_cross_scene_cache_benefit",
        "run_id": run_id("cross_scene_cache_benefit"),
        "robot": str(args.robot),
        "num_scenes": int(len(scenes)),
        "difficulties": csv_items(args.difficulties),
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "scene_specs": [
            {
                "scene_index": index,
                "difficulty": scene.difficulty,
                "obstacle_count": len(scene.obstacles),
            }
            for index, scene in enumerate(scenes)
        ],
        "modes": modes,
        "summary": summarize(modes),
    }
    write_json(args.out_json, payload)
    print(f"wrote {args.out_json}", flush=True)

    summary = payload["summary"]
    if summary:
        cw = summary["cumulative_build_wall_s"]
        cg = summary["cumulative_grow_ms"]
        print(
            f"[cross-scene] cumulative build_wall_s accumulate={cw['accumulate']:.2f} "
            f"fresh={cw['fresh']:.2f} speedup={cw['speedup_pct']:.2f}%",
            flush=True,
        )
        print(
            f"[cross-scene] cumulative grow_ms accumulate={cg['accumulate']:.1f} "
            f"fresh={cg['fresh']:.1f} speedup={cg['speedup_pct']:.2f}%",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
