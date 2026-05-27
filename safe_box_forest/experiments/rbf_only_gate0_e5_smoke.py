from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path
from typing import Any

from common_sbf_config import (
    RBF_LIFELONG_PRESET,
    RBF_ONLY_OUTPUT_ROOT,
    add_common_sbf_args,
    configure_standalone_sbf,
    rbf_lifelong_config_metadata,
    write_json,
)

import sbf


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run RBF-only Gate 0 and E5 lifelong cache smokes.")
    add_common_sbf_args(parser)
    parser.set_defaults(preset=RBF_LIFELONG_PRESET)
    parser.add_argument("--output-dir", type=Path, default=RBF_ONLY_OUTPUT_ROOT)
    parser.add_argument("--only-gate0", action="store_true", default=False)
    parser.add_argument("--only-e5", action="store_true", default=False)
    parser.add_argument("--clean-cache", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def far_obstacle() -> Any:
    return sbf.Obstacle(100.0, 100.0, 100.0, 101.0, 101.0, 101.0)


def make_config(args: argparse.Namespace, robot: Any, seed: int, cache_label: str) -> Any:
    local_args = argparse.Namespace(**vars(args))
    local_args.rbf_cache_label = cache_label
    return configure_standalone_sbf(local_args, seed=seed, preset=RBF_LIFELONG_PRESET, robot=robot)


def directory_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def cache_file_sizes(path: Path) -> dict[str, int]:
    if not path.exists():
        return {}
    return {item.name: item.stat().st_size for item in sorted(path.iterdir()) if item.is_file()}


def run_gate0(args: argparse.Namespace, robot: Any) -> dict[str, Any]:
    cfg = make_config(args, robot, seed=0, cache_label="gate0_api_smoke_cache")
    metadata = rbf_lifelong_config_metadata(cfg, args)
    required = {
        "EndpointSource.IFK": hasattr(sbf.EndpointSource, "IFK"),
        "SplitStrategy.AAFKVolumeMin": hasattr(sbf.SplitStrategy, "AAFKVolumeMin"),
        "SplitPolicyDescriptor.depth_dimensions": hasattr(sbf.SplitPolicyDescriptor(), "depth_dimensions"),
        "LectDatabaseRuntimeConfig.canonical_mode": hasattr(sbf.LectDatabaseRuntimeConfig(), "canonical_mode"),
        "FindFreeBoxOptions.skip_to_depth": hasattr(sbf.FindFreeBoxOptions(), "skip_to_depth"),
        "aafk_volume_min_depth_schedule": hasattr(sbf, "aafk_volume_min_depth_schedule"),
    }
    checks = {
        **required,
        "schedule_depth_at_least_40": int(metadata["schedule_depth"]) >= 40,
        "max_depth_is_40": int(metadata["max_depth"]) == 40,
        "ffb_start_depth_is_15": int(metadata["ffb_start_depth"]) == 15,
        "canonical_mode_enabled": bool(metadata["canonical_mode"]),
        "dimension_schedule_hash_present": bool(metadata["dimension_schedule_hash"]),
    }
    payload = {
        "ok": all(bool(value) for value in checks.values()),
        "checks": checks,
        "metadata": metadata,
    }
    write_json(args.output_dir / "gate0_api_smoke.json", payload)
    return payload


def run_e5(args: argparse.Namespace, robot: Any) -> dict[str, Any]:
    cache_label = f"e5_lifelong_cache_{args.rbf_envelope}_d{int(args.rbf_prewarm_depth)}_smoke"
    cfg = make_config(args, robot, seed=0, cache_label=cache_label)
    cache_path = Path(cfg.database.path)
    if args.clean_cache and cache_path.exists():
        shutil.rmtree(cache_path)

    obstacles = [far_obstacle()]
    first_forest = sbf.SafeBoxForest(robot, cfg)
    print(f"e5 first prewarm start depth={int(args.rbf_prewarm_depth)} envelope={args.rbf_envelope}", flush=True)
    first_t0 = time.perf_counter()
    first = dict(first_forest.prewarm_lifelong_cache(int(args.rbf_prewarm_depth), obstacles))
    first["wall_s_outer"] = time.perf_counter() - first_t0
    first_verify = bool(first_forest.database_verify(True))
    print(f"e5 first prewarm done ok={first.get('ok')} evidence_after={first.get('evidence_after')}", flush=True)
    del first_forest

    reopen_cfg = make_config(args, robot, seed=0, cache_label=cache_label)
    reopen_cfg.database.create_if_missing = False
    reopen_forest = sbf.SafeBoxForest(robot, reopen_cfg)
    print("e5 reopen prewarm start", flush=True)
    second_t0 = time.perf_counter()
    second = dict(reopen_forest.prewarm_lifelong_cache(int(args.rbf_prewarm_depth), obstacles))
    second["wall_s_outer"] = time.perf_counter() - second_t0
    second_verify = bool(reopen_forest.database_verify(True))
    print(f"e5 reopen prewarm done reused={second.get('reused_endpoint_cache')}", flush=True)

    metadata = rbf_lifelong_config_metadata(cfg, args)
    checks = {
        "first_prewarm_ok": bool(first.get("ok")),
        "first_verify_ok": first_verify,
        "first_materialized_evidence": int(first.get("evidence_after", 0)) > int(first.get("evidence_before", 0)),
        "reopen_verify_ok": second_verify,
        "reopen_reused_endpoint_cache": int(second.get("reused_endpoint_cache", 0)) > 0,
        "canonical_mode_enabled": bool(metadata["canonical_mode"]),
        "prewarm_depth_matches": int(first.get("target_depth", -1)) == int(args.rbf_prewarm_depth),
    }
    payload = {
        "ok": all(bool(value) for value in checks.values()),
        "checks": checks,
        "metadata": metadata,
        "cache_path": str(cache_path),
        "cache_bytes": directory_size_bytes(cache_path),
        "cache_file_sizes": cache_file_sizes(cache_path),
        "first_prewarm": first,
        "reopen_prewarm": second,
    }
    write_json(args.output_dir / "e5_lifelong_cache_mechanism_smoke.json", payload)
    return payload


def main() -> int:
    args = parse_args()
    robot = sbf.load_iiwa14_robot()
    if not args.only_e5:
        gate0 = run_gate0(args, robot)
        print(f"gate0 ok={gate0['ok']} output={args.output_dir / 'gate0_api_smoke.json'}", flush=True)
    if not args.only_gate0:
        e5 = run_e5(args, robot)
        print(f"e5 ok={e5['ok']} output={args.output_dir / 'e5_lifelong_cache_mechanism_smoke.json'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())