#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any


def _bootstrap_imports() -> tuple[Path, Path]:
    here = Path(__file__).resolve()
    root = here.parents[1]
    repo_root = root.parents[1]
    default_build_dir = repo_root / "build-rbf-only-exec"
    os.environ.setdefault("SBF_BUILD_DIR", str(default_build_dir))
    build_dir = os.environ.get("SBF_BUILD_DIR")
    candidates: list[Path] = []
    if build_dir:
        candidates.append(Path(build_dir) / "python")
    candidates.extend((
        repo_root / "build-rbf-only-exec" / "python",
        repo_root / "build-consolidated-python" / "python",
        root / "build_py310" / "python",
        root / "build" / "python",
        root / "python",
        repo_root,
        here.parent,
    ))
    for candidate in reversed(candidates):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root, repo_root


ROOT, REPO_ROOT = _bootstrap_imports()

import paper_04_marcucci_combined as p4  # noqa: E402
import sbf  # noqa: E402
from sbf.marcucci import make_combined_obstacles, make_coverage_seeds, load_iiwa14_robot  # noqa: E402


OLD_BUILD_DEFAULT = Path("/home/tian/桌面/box_aabb/cpp/SBF/build_py310")
CURRENT_BUILD_DEFAULT = Path(os.environ.get("SBF_BUILD_DIR", str(REPO_ROOT / "build-rbf-only-exec")))
OUTPUT_DEFAULT = ROOT / "outputs" / "debug" / "old_current_grower_compare.json"

OLD_GROWER_SNIPPET = r'''
from __future__ import annotations

import json
import os
from collections import Counter
from pathlib import Path

import paper_04_marcucci_combined as p4
import sbf


def set_if_available(obj, name, value):
    try:
        setattr(obj, name, value)
        return True
    except AttributeError:
        return False


def set_path_if_available(obj, path, value):
    current = obj
    parts = path.split('.')
    for part in parts[:-1]:
        try:
            current = getattr(current, part)
        except AttributeError:
            return False
    return set_if_available(current, parts[-1], value)


def make_args(payload):
    args = p4.parse_args([])
    args.out_json = Path(payload['out_json'])
    args.database_path = Path(payload['database_path'])
    args.preset = payload['preset']
    args.envelope = payload['envelope']
    args.split_policy = payload['split_policy']
    args.best_tighten_shape_balancing = bool(payload.get('best_tighten_shape_balancing', False))
    args.best_tighten_recent_dim_cooling = bool(payload.get('best_tighten_recent_dim_cooling', False))
    args.ffb_depth = int(payload['ffb_depth'])
    args.seed_base = int(payload['seed_base'])
    args.seeds = 1
    args.threads = 1
    args.task_batch_size = 1
    args.max_boxes = int(payload['max_boxes'])
    args.timeout_ms = float(payload['timeout_ms'])
    args.grow_only = True
    args.segment_edges = False
    args.enable_merger = False
    args.enable_connector = False
    args.collision_shortcut = False
    args.strict_path_audit = False
    args.repair_on_audit_failure = False
    args.bridge_failed_queries = False
    args.bridge_repaired_queries = False
    args.corridor_refine = False
    args.run_baselines = False
    args.use_external_evidence = False
    args.validation_cache = False
    if hasattr(args, 'endpoint_evidence_cache'):
        args.endpoint_evidence_cache = False
    if hasattr(args, 'worker_shared_endpoint_cache'):
        args.worker_shared_endpoint_cache = False
    return args


def make_old_config(args, trial_seed):
    cfg = sbf.SBFConfig()
    cfg.enable_merger = False
    cfg.enable_connector = False
    cfg.use_fingerprint_cache = False
    cfg.reuse_fingerprint_cache_tree = False
    set_if_available(cfg, 'save_fingerprint_cache', False)
    cfg.runtime.mode = sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = 1
    cfg.runtime.batch_size = 1
    cfg.runtime.parallel_threshold = 1
    cfg.cache.explicit_path = str(args.database_path)
    set_if_available(cfg.cache, 'create_directories', True)
    set_if_available(cfg.cache, 'require_stored_fingerprint', False)

    if args.preset in {'crit_link_coverage', 'kdop26_coverage', 'support_hull_coverage'}:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
        if args.preset == 'kdop26_coverage':
            cfg.envelope_type.type = sbf.EnvelopeType.KDOP
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        elif args.preset == 'support_hull_coverage':
            cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
            cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)
        else:
            cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
    else:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
        cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
        cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
        cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed

    if args.envelope != 'preset':
        if args.envelope == 'link':
            cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        elif args.envelope == 'kdop26':
            cfg.envelope_type.type = sbf.EnvelopeType.KDOP
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        elif args.envelope == 'support_hull':
            cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
            cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)

    cfg.envelope_type.n_subdivisions = int(args.envelope_subdivisions)
    use_best_tighten = args.split_policy == 'best-tighten'
    set_path_if_available(cfg, 'grower.find_free_box.split.use_best_tighten', use_best_tighten)
    set_path_if_available(cfg, 'connector.pave.find_free_box.split.use_best_tighten', use_best_tighten)
    for prefix in ('grower.find_free_box', 'connector.pave.find_free_box'):
        set_path_if_available(cfg, f'{prefix}.split.best_tighten.depth_synchronous', bool(args.best_tighten_depth_synchronous))
        set_path_if_available(cfg, f'{prefix}.split.best_tighten.prefer_sector_boundary', bool(args.best_tighten_prefer_sector_boundary))
        set_path_if_available(cfg, f'{prefix}.split.best_tighten.use_minimax', bool(args.best_tighten_use_minimax))
        set_path_if_available(cfg, f'{prefix}.split.best_tighten.shape_balancing', bool(args.best_tighten_shape_balancing))
        set_path_if_available(cfg, f'{prefix}.split.best_tighten.recent_dim_cooling', bool(args.best_tighten_recent_dim_cooling))

    cfg.grower.mode = sbf.GrowerMode.RRT
    cfg.grower.rng_seed = int(args.seed_base) + int(trial_seed)
    cfg.grower.max_boxes = int(args.max_boxes)
    cfg.grower.timeout_ms = float(args.timeout_ms)
    cfg.grower.max_consecutive_miss = int(args.max_consecutive_miss)
    cfg.grower.n_threads = 1
    cfg.grower.task_batch_size = 1
    cfg.grower.parallel_threshold = 1
    set_if_available(cfg.grower, 'worker_local_ffb', False)
    cfg.grower.find_free_box.max_depth = int(args.ffb_depth)
    cfg.grower.find_free_box.split_reserved_leaf = True
    cfg.grower.find_free_box.split_unknown_leaf = True
    cfg.grower.find_free_box.reject_seed_collision = False
    set_if_available(cfg.grower, 'rrt_goal_bias', float(args.rrt_goal_bias))
    set_if_available(cfg.grower, 'intertree_goal_bias', float(args.intertree_goal_bias))
    set_if_available(cfg.grower, 'sustained_goal_bias_cap', min(0.25, float(args.intertree_goal_bias)))
    set_if_available(cfg.grower, 'rrt_step_ratio', float(args.step_ratio))
    set_if_available(cfg.grower, 'unexplored_sample_prob', float(args.unexplored_prob))
    cfg.grower.connect_mode = True
    cfg.grower.expand_all_roots_per_sample = True
    set_if_available(cfg.grower, 'component_connect_prob', float(args.component_connect_prob))
    set_if_available(cfg.grower, 'component_connect_candidate_limit', int(args.component_connect_candidate_limit))
    cfg.grower.component_connect_island_aware = True
    cfg.grower.component_connect_frontier_cache = True
    cfg.grower.component_connect_staged_growth = True
    set_if_available(cfg.grower, 'component_connect_stage_normalized_linf', float(args.component_connect_stage_normalized_linf))
    set_if_available(cfg.grower, 'component_connect_adaptive_ffb', True)
    set_if_available(cfg.grower, 'component_connect_ffb_depth_increment', int(args.component_connect_ffb_depth_increment))
    set_if_available(cfg.grower, 'component_connect_ffb_max_depth', int(args.component_connect_ffb_max_depth))
    set_if_available(cfg.grower, 'stop_after_connect', bool(args.stop_after_connect))
    set_if_available(cfg.grower, 'post_connect_extra_boxes', int(args.post_connect_extra_boxes))
    set_if_available(cfg.grower, 'quality_min_connected_boxes', int(args.quality_min_connected_boxes))
    set_if_available(cfg.grower, 'post_connect_time_budget_ms', float(args.post_connect_time_budget_ms))
    set_if_available(cfg.grower, 'coverage_first_stop_loss', bool(args.coverage_first_stop_loss))
    set_if_available(cfg.grower, 'hard_frontier_failure_threshold', int(args.hard_frontier_failure_threshold))
    set_if_available(cfg.grower, 'hard_frontier_box_horizon', int(args.hard_frontier_box_horizon))

    cfg.query.nearest_if_outside = False
    cfg.query.shortcut_boxes = False
    cfg.query.collision_shortcut = False
    cfg.query.strict_path_audit = False
    cfg.query.repair_on_audit_failure = False
    cfg.validation.enable_validation_cache = False
    cfg.validation.validation_cache_max_entries = int(args.validation_cache_max_entries)
    set_if_available(cfg.validation, 'external_evidence_materialization', False)
    set_if_available(cfg.validation, 'external_evidence_scoring', False)
    set_if_available(cfg.validation, 'external_evidence_backfill_active', False)
    set_if_available(cfg.validation, 'stateless_materialization_context', True)
    return cfg


def obstacle_from_bounds(bounds):
    return sbf.Obstacle(*[float(v) for v in bounds])


def box_to_dict(box):
    return {
        'id': int(box.id),
        'root_id': int(box.root_id),
        'parent_box_id': int(box.parent_box_id),
        'tree_id': int(box.tree_id),
        'safety_status': int(box.safety_status),
        'strict_audit_required': bool(box.strict_audit_required),
        'volume': float(box.volume),
        'joint_intervals': [[float(iv.lo), float(iv.hi)] for iv in box.joint_intervals],
    }


def summarize_side(profile, boxes):
    root_counts = Counter(int(box.root_id) for box in boxes)
    safety_counts = Counter(int(box.safety_status) for box in boxes)
    return {
        'profile': {
            'raw_boxes': int(profile.raw_boxes),
            'final_boxes': int(profile.final_boxes),
            'adjacency_islands': int(profile.adjacency_islands),
            'total_ms': float(profile.total_ms),
            'grow_ms': float(profile.grow_ms),
            'merge_ms': float(profile.merge_ms),
            'connector_ms': float(profile.connector_ms),
            'adjacency_ms': float(profile.adjacency_ms),
        },
        'box_count': len(boxes),
        'root_counts': [[int(root_id), int(count)] for root_id, count in sorted(root_counts.items())],
        'safety_counts': [[int(status), int(count)] for status, count in sorted(safety_counts.items())],
        'strict_audit_required_count': sum(1 for box in boxes if bool(box.strict_audit_required)),
        'boxes': [box_to_dict(box) for box in boxes],
    }


payload = json.loads(os.environ['COMPARE_PAYLOAD_JSON'])
args = make_args(payload)
robot = p4.load_iiwa14_robot()
obstacles = [obstacle_from_bounds(bounds) for bounds in payload['obstacle_bounds']]
coverage_seeds = [[float(v) for v in seed] for seed in payload['coverage_seeds']]
forest = sbf.SafeBoxForest(robot, make_old_config(args, int(payload['trial_seed'])))
profile = forest.build_coverage(obstacles, coverage_seeds)
boxes = list(forest.boxes())
result = summarize_side(profile, boxes)
result['trial_seed'] = int(payload['trial_seed'])
print(json.dumps(result, ensure_ascii=False, sort_keys=True))
'''


def parse_trial_seeds(text: str) -> list[int]:
    values: list[int] = []
    for item in str(text).split(","):
        chunk = item.strip()
        if not chunk:
            continue
        values.append(int(chunk))
    if not values:
        raise ValueError("trial seed list is empty")
    return values


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare old/current grower-only build_coverage results under the same Marcucci CritSample settings.")
    parser.add_argument("--out-json", type=Path, default=OUTPUT_DEFAULT)
    parser.add_argument("--old-build-dir", type=Path, default=OLD_BUILD_DEFAULT)
    parser.add_argument("--current-build-dir", type=Path, default=CURRENT_BUILD_DEFAULT)
    parser.add_argument("--preset", default="support_hull_coverage")
    parser.add_argument("--envelope", choices=["preset", "link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--split-policy", choices=["widest-first", "best-tighten"], default="best-tighten")
    parser.add_argument("--trial-seeds", default="0")
    parser.add_argument("--include-extra-anchors", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--disable-current-canonical-symmetry", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--max-boxes", type=int, default=5000)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--max-consecutive-miss", type=int, default=2000)
    parser.add_argument("--best-tighten-shape-balancing", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--best-tighten-recent-dim-cooling", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--emit-boxes", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args(argv)


def canonical_intervals(intervals: list[list[float]], ndigits: int = 12) -> tuple[tuple[float, float], ...]:
    return tuple((round(float(lo), ndigits), round(float(hi), ndigits)) for lo, hi in intervals)


def box_to_dict(box: Any) -> dict[str, Any]:
    return {
        "id": int(box.id),
        "root_id": int(box.root_id),
        "parent_box_id": int(box.parent_box_id),
        "tree_id": int(box.tree_id),
        "safety_status": int(box.safety_status),
        "strict_audit_required": bool(box.strict_audit_required),
        "volume": float(box.volume),
        "joint_intervals": [[float(iv.lo), float(iv.hi)] for iv in box.joint_intervals],
    }


def summarize_boxes(boxes: list[dict[str, Any]]) -> dict[str, Any]:
    root_counts = Counter(int(box["root_id"]) for box in boxes)
    safety_counts = Counter(int(box["safety_status"]) for box in boxes)
    return {
        "box_count": len(boxes),
        "root_counts": [[int(root_id), int(count)] for root_id, count in sorted(root_counts.items())],
        "safety_counts": [[int(status), int(count)] for status, count in sorted(safety_counts.items())],
        "strict_audit_required_count": sum(1 for box in boxes if bool(box["strict_audit_required"])),
    }


def compare_boxes(old_boxes: list[dict[str, Any]], current_boxes: list[dict[str, Any]]) -> dict[str, Any]:
    old_map = {
        canonical_intervals(box["joint_intervals"]): {
            "safety_status": int(box["safety_status"]),
            "strict_audit_required": bool(box["strict_audit_required"]),
        }
        for box in old_boxes
    }
    current_map = {
        canonical_intervals(box["joint_intervals"]): {
            "safety_status": int(box["safety_status"]),
            "strict_audit_required": bool(box["strict_audit_required"]),
        }
        for box in current_boxes
    }
    old_keys = set(old_map)
    current_keys = set(current_map)
    common = old_keys & current_keys
    status_mismatches = [
        {
            "intervals": [[float(lo), float(hi)] for lo, hi in key],
            "old": old_map[key],
            "current": current_map[key],
        }
        for key in sorted(common)
        if old_map[key] != current_map[key]
    ]
    only_old = sorted(old_keys - current_keys)
    only_current = sorted(current_keys - old_keys)
    return {
        "old_box_count": len(old_boxes),
        "current_box_count": len(current_boxes),
        "common_box_count": len(common),
        "only_old_count": len(only_old),
        "only_current_count": len(only_current),
        "status_mismatch_count_in_common": len(status_mismatches),
        "box_set_match": not only_old and not only_current and not status_mismatches,
        "first_only_old": None if not only_old else [[float(lo), float(hi)] for lo, hi in only_old[0]],
        "first_only_current": None if not only_current else [[float(lo), float(hi)] for lo, hi in only_current[0]],
        "sample_status_mismatches": status_mismatches[:5],
    }


def make_compare_args(cli_args: argparse.Namespace, database_path: Path) -> argparse.Namespace:
    args = p4.parse_args([])
    args.out_json = database_path.with_suffix(".json")
    args.database_path = database_path
    args.preset = cli_args.preset
    args.envelope = cli_args.envelope
    args.split_policy = cli_args.split_policy
    args.best_tighten_shape_balancing = bool(cli_args.best_tighten_shape_balancing)
    args.best_tighten_recent_dim_cooling = bool(cli_args.best_tighten_recent_dim_cooling)
    args.canonical_symmetry = not bool(cli_args.disable_current_canonical_symmetry)
    args.ffb_depth = int(cli_args.ffb_depth)
    args.seed_base = int(cli_args.seed_base)
    args.seeds = 1
    args.threads = 1
    args.task_batch_size = 1
    args.max_boxes = int(cli_args.max_boxes)
    args.timeout_ms = float(cli_args.timeout_ms)
    args.max_consecutive_miss = int(cli_args.max_consecutive_miss)
    args.grow_only = True
    args.segment_edges = False
    args.enable_merger = False
    args.enable_connector = False
    args.collision_shortcut = False
    args.strict_path_audit = False
    args.repair_on_audit_failure = False
    args.bridge_failed_queries = False
    args.bridge_repaired_queries = False
    args.corridor_refine = False
    args.run_baselines = False
    args.use_external_evidence = False
    args.validation_cache = False
    if hasattr(args, "endpoint_evidence_cache"):
        args.endpoint_evidence_cache = False
    if hasattr(args, "worker_shared_endpoint_cache"):
        args.worker_shared_endpoint_cache = False
    return args


def run_current_trial(work_root: Path,
                      trial_seed: int,
                      coverage_seeds: list[list[float]],
                      obstacles: list[Any],
                      cli_args: argparse.Namespace) -> dict[str, Any]:
    work_dir = work_root / f"current_seed{trial_seed:03d}"
    work_dir.mkdir(parents=True, exist_ok=True)
    args = make_compare_args(cli_args, work_dir / "db")
    robot = load_iiwa14_robot()
    config = p4.configure(args, trial_seed)
    if bool(cli_args.disable_current_canonical_symmetry):
        config.database.canonical_mode = False
    forest = sbf.SafeBoxForest(robot, config)
    profile = forest.build_coverage(obstacles, coverage_seeds)
    boxes = [box_to_dict(box) for box in forest.boxes()]
    result = {
        "trial_seed": int(trial_seed),
        "profile": {
            "raw_boxes": int(profile.raw_boxes),
            "final_boxes": int(profile.final_boxes),
            "adjacency_islands": int(profile.adjacency_islands),
            "total_ms": float(profile.total_ms),
            "grow_ms": float(profile.grow_ms),
            "merge_ms": float(profile.merge_ms),
            "connector_ms": float(profile.connector_ms),
            "adjacency_ms": float(profile.adjacency_ms),
        },
        **summarize_boxes(boxes),
        "boxes": boxes,
    }
    return result


def run_old_trial(work_root: Path,
                  trial_seed: int,
                  coverage_seeds: list[list[float]],
                  obstacle_bounds_payload: list[list[float]],
                  cli_args: argparse.Namespace) -> dict[str, Any]:
    work_dir = work_root / f"old_seed{trial_seed:03d}"
    work_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "trial_seed": int(trial_seed),
        "preset": cli_args.preset,
        "envelope": cli_args.envelope,
        "split_policy": cli_args.split_policy,
        "best_tighten_shape_balancing": bool(cli_args.best_tighten_shape_balancing),
        "best_tighten_recent_dim_cooling": bool(cli_args.best_tighten_recent_dim_cooling),
        "ffb_depth": int(cli_args.ffb_depth),
        "max_boxes": int(cli_args.max_boxes),
        "seed_base": int(cli_args.seed_base),
        "timeout_ms": float(cli_args.timeout_ms),
        "obstacle_bounds": obstacle_bounds_payload,
        "coverage_seeds": coverage_seeds,
        "database_path": str(work_dir / "db"),
        "out_json": str(work_dir / "run.json"),
    }
    env = os.environ.copy()
    env["SBF_BUILD_DIR"] = str(cli_args.old_build_dir)
    env["COMPARE_PAYLOAD_JSON"] = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    extra_paths = [str(REPO_ROOT), str(Path(__file__).resolve().parent)]
    current_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join([*extra_paths, current_pythonpath] if current_pythonpath else extra_paths)
    completed = subprocess.run(
        [sys.executable, "-c", OLD_GROWER_SNIPPET],
        cwd=str(REPO_ROOT),
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "old grower subprocess failed\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    stdout = completed.stdout.strip()
    if not stdout:
        raise RuntimeError("old grower subprocess produced no JSON output")
    return json.loads(stdout)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    trial_seeds = parse_trial_seeds(args.trial_seeds)
    work_root = args.out_json.parent / "grower_compare_work" / args.out_json.stem
    if work_root.exists():
        shutil.rmtree(work_root)
    obstacles = make_combined_obstacles()
    obstacle_bounds_payload = [[float(value) for value in obstacle.bounds] for obstacle in obstacles]
    coverage_seeds = [
        [float(value) for value in seed]
        for seed in make_coverage_seeds(include_extra_anchors=bool(args.include_extra_anchors))
    ]

    rows: list[dict[str, Any]] = []
    for trial_seed in trial_seeds:
        old_result = run_old_trial(work_root, trial_seed, coverage_seeds, obstacle_bounds_payload, args)
        current_result = run_current_trial(work_root, trial_seed, coverage_seeds, obstacles, args)
        old_boxes = old_result.pop("boxes")
        current_boxes = current_result.pop("boxes")
        row = {
            "trial_seed": int(trial_seed),
            "old": old_result,
            "current": current_result,
            "box_compare": compare_boxes(old_boxes, current_boxes),
        }
        if bool(args.emit_boxes):
            row["old_boxes"] = old_boxes
            row["current_boxes"] = current_boxes
        rows.append(row)

    payload = {
        "experiment": "old_current_grower_compare",
        "old_build_dir": str(args.old_build_dir),
        "current_build_dir": str(args.current_build_dir),
        "preset": args.preset,
        "envelope": args.envelope,
        "split_policy": args.split_policy,
        "best_tighten_shape_balancing": bool(args.best_tighten_shape_balancing),
        "best_tighten_recent_dim_cooling": bool(args.best_tighten_recent_dim_cooling),
        "disable_current_canonical_symmetry": bool(args.disable_current_canonical_symmetry),
        "grow_only": True,
        "include_extra_anchors": bool(args.include_extra_anchors),
        "coverage_seed_count": len(coverage_seeds),
        "coverage_seeds": coverage_seeds,
        "trial_seeds": trial_seeds,
        "max_boxes": int(args.max_boxes),
        "timeout_ms": float(args.timeout_ms),
        "ffb_depth": int(args.ffb_depth),
        "rows": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "out_json": str(args.out_json),
        "trial_seeds": trial_seeds,
        "rows": [
            {
                "trial_seed": int(row["trial_seed"]),
                "old_final_boxes": int(row["old"]["profile"]["final_boxes"]),
                "current_final_boxes": int(row["current"]["profile"]["final_boxes"]),
                "old_islands": int(row["old"]["profile"]["adjacency_islands"]),
                "current_islands": int(row["current"]["profile"]["adjacency_islands"]),
                "box_set_match": bool(row["box_compare"]["box_set_match"]),
                "common_box_count": int(row["box_compare"]["common_box_count"]),
                "only_old_count": int(row["box_compare"]["only_old_count"]),
                "only_current_count": int(row["box_compare"]["only_current_count"]),
            }
            for row in rows
        ],
    }, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())