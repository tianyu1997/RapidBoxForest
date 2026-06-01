#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
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
from sbf.marcucci import load_iiwa14_robot, make_combined_obstacles, make_coverage_seeds  # noqa: E402


OLD_BUILD_DEFAULT = Path("/home/tian/桌面/box_aabb/cpp/SBF/build_py310")
CURRENT_BUILD_DEFAULT = Path(os.environ.get("SBF_BUILD_DIR", str(REPO_ROOT / "build-rbf-only-exec")))
OUTPUT_DEFAULT = ROOT / "outputs" / "debug" / "old_current_interval_ffb_compare.json"
BOX_TABLE_DEFAULT = ROOT / "outputs" / "debug" / "old_current_interval_ffb_compare_boxes.json"

OLD_TRACE_SNIPPET = r'''
from __future__ import annotations

import json
import os
from pathlib import Path

import paper_04_marcucci_combined as p4
import sbf


def make_args(payload: dict) -> object:
    args = p4.parse_args([])
    args.out_json = Path(payload["out_json"])
    args.database_path = Path(payload["database_path"])
    args.preset = payload["preset"]
    args.envelope = payload["envelope"]
    args.split_policy = payload["split_policy"]
    args.best_tighten_shape_balancing = bool(payload.get("best_tighten_shape_balancing", False))
    args.best_tighten_recent_dim_cooling = bool(payload.get("best_tighten_recent_dim_cooling", False))
    args.ffb_depth = int(payload["depth"])
    args.seed_base = int(payload["seed_base"])
    args.seeds = 1
    args.threads = 1
    args.task_batch_size = 1
    args.max_boxes = 1
    args.timeout_ms = float(payload["timeout_ms"])
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


def set_if_available(obj: object, name: str, value: object) -> bool:
    try:
        setattr(obj, name, value)
        return True
    except AttributeError:
        return False


def set_path_if_available(obj: object, path: str, value: object) -> bool:
    current = obj
    parts = path.split(".")
    for part in parts[:-1]:
        try:
            current = getattr(current, part)
        except AttributeError:
            return False
    return set_if_available(current, parts[-1], value)


def make_old_config(args: object) -> object:
    cfg = sbf.SBFConfig()
    cfg.enable_merger = False
    cfg.enable_connector = False
    cfg.use_fingerprint_cache = False
    cfg.reuse_fingerprint_cache_tree = False
    set_if_available(cfg, "save_fingerprint_cache", False)
    cfg.runtime.mode = sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = 1
    cfg.runtime.batch_size = 1
    cfg.runtime.parallel_threshold = 1
    cfg.cache.explicit_path = str(args.database_path)
    set_if_available(cfg.cache, "create_directories", True)
    set_if_available(cfg.cache, "require_stored_fingerprint", False)

    if args.preset == "ifk_strict":
        cfg.endpoint_source.source = sbf.EndpointSource.IFK
        cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    elif args.preset in {"crit_link_coverage", "kdop26_coverage", "support_hull_coverage"}:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
        if args.preset == "kdop26_coverage":
            cfg.envelope_type.type = sbf.EnvelopeType.KDOP
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        elif args.preset == "support_hull_coverage":
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

    if args.envelope != "preset":
        if args.envelope == "link":
            cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        elif args.envelope == "kdop26":
            cfg.envelope_type.type = sbf.EnvelopeType.KDOP
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        elif args.envelope == "support_hull":
            cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
            cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)

    cfg.envelope_type.n_subdivisions = int(args.envelope_subdivisions)
    use_best_tighten = args.split_policy == "best-tighten"
    set_path_if_available(cfg, "grower.find_free_box.split.use_best_tighten", use_best_tighten)
    set_path_if_available(cfg, "connector.pave.find_free_box.split.use_best_tighten", use_best_tighten)
    for prefix in ("grower.find_free_box", "connector.pave.find_free_box"):
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.depth_synchronous", bool(args.best_tighten_depth_synchronous))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.prefer_sector_boundary", bool(args.best_tighten_prefer_sector_boundary))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.use_minimax", bool(args.best_tighten_use_minimax))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.shape_balancing", bool(args.best_tighten_shape_balancing))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.recent_dim_cooling", bool(args.best_tighten_recent_dim_cooling))

    cfg.grower.mode = sbf.GrowerMode.RRT
    cfg.grower.rng_seed = int(args.seed_base)
    cfg.grower.max_boxes = int(args.max_boxes)
    cfg.grower.timeout_ms = float(args.timeout_ms)
    cfg.grower.max_consecutive_miss = int(args.max_consecutive_miss)
    cfg.grower.n_threads = 1
    cfg.grower.task_batch_size = 1
    cfg.grower.parallel_threshold = 1
    set_if_available(cfg.grower, "worker_local_ffb", False)
    cfg.grower.find_free_box.max_depth = int(args.ffb_depth)
    cfg.grower.find_free_box.split_reserved_leaf = True
    cfg.grower.find_free_box.split_unknown_leaf = True
    cfg.grower.find_free_box.reject_seed_collision = False
    set_if_available(cfg.grower, "rrt_goal_bias", float(args.rrt_goal_bias))
    set_if_available(cfg.grower, "intertree_goal_bias", float(args.intertree_goal_bias))
    set_if_available(cfg.grower, "sustained_goal_bias_cap", min(0.25, float(args.intertree_goal_bias)))
    set_if_available(cfg.grower, "rrt_step_ratio", float(args.step_ratio))
    set_if_available(cfg.grower, "unexplored_sample_prob", float(args.unexplored_prob))
    cfg.grower.connect_mode = True
    cfg.grower.expand_all_roots_per_sample = True
    set_if_available(cfg.grower, "component_connect_prob", float(args.component_connect_prob))
    set_if_available(cfg.grower, "component_connect_candidate_limit", int(args.component_connect_candidate_limit))
    cfg.grower.component_connect_island_aware = True
    cfg.grower.component_connect_frontier_cache = True
    cfg.grower.component_connect_staged_growth = True
    set_if_available(cfg.grower, "component_connect_stage_normalized_linf", float(args.component_connect_stage_normalized_linf))
    set_if_available(cfg.grower, "component_connect_adaptive_ffb", True)
    set_if_available(cfg.grower, "component_connect_ffb_depth_increment", int(args.component_connect_ffb_depth_increment))
    set_if_available(cfg.grower, "component_connect_ffb_max_depth", int(args.component_connect_ffb_max_depth))
    set_if_available(cfg.grower, "stop_after_connect", bool(args.stop_after_connect))
    set_if_available(cfg.grower, "post_connect_extra_boxes", int(args.post_connect_extra_boxes))
    set_if_available(cfg.grower, "quality_min_connected_boxes", int(args.quality_min_connected_boxes))
    set_if_available(cfg.grower, "post_connect_time_budget_ms", float(args.post_connect_time_budget_ms))
    set_if_available(cfg.grower, "coverage_first_stop_loss", bool(args.coverage_first_stop_loss))
    set_if_available(cfg.grower, "hard_frontier_failure_threshold", int(args.hard_frontier_failure_threshold))
    set_if_available(cfg.grower, "hard_frontier_box_horizon", int(args.hard_frontier_box_horizon))

    cfg.query.nearest_if_outside = False
    cfg.query.shortcut_boxes = False
    cfg.query.collision_shortcut = False
    cfg.query.strict_path_audit = False
    cfg.query.repair_on_audit_failure = False
    cfg.validation.enable_validation_cache = False
    cfg.validation.validation_cache_max_entries = int(args.validation_cache_max_entries)
    set_if_available(cfg.validation, "external_evidence_materialization", False)
    set_if_available(cfg.validation, "external_evidence_scoring", False)
    set_if_available(cfg.validation, "external_evidence_backfill_active", False)
    set_if_available(cfg.validation, "stateless_materialization_context", True)
    return cfg


def obstacle_from_bounds(bounds: list[float]) -> object:
    return sbf.Obstacle(*[float(value) for value in bounds])


def box_to_dict(box: object) -> dict[str, object]:
    return {
        "id": int(box.id),
        "root_id": int(box.root_id),
        "parent_box_id": int(box.parent_box_id),
        "tree_id": int(box.tree_id),
        "safety_status": int(box.safety_status),
        "strict_audit_required": bool(box.strict_audit_required),
        "volume": float(box.volume),
        "joint_intervals": [[float(interval.lo), float(interval.hi)] for interval in box.joint_intervals],
    }


payload = json.loads(os.environ["COMPARE_PAYLOAD_JSON"])
args = make_args(payload)
robot = p4.load_iiwa14_robot()
obstacles = [obstacle_from_bounds(bounds) for bounds in payload["obstacle_bounds"]]
seed = [float(value) for value in payload["seed"]]
forest = sbf.SafeBoxForest(robot, make_old_config(args))
profile = forest.build_coverage(obstacles, [seed])
result = {
    "depth": int(payload["depth"]),
    "profile": {
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "adjacency_islands": int(profile.adjacency_islands),
        "total_ms": float(profile.total_ms),
    },
    "validation_events": list(forest.validation_events()),
    "split_events": list(forest.split_events()),
    "boxes": [box_to_dict(box) for box in forest.boxes()],
}
print(json.dumps(result, ensure_ascii=False, sort_keys=True))
'''


def parse_depths(text: str) -> list[int]:
    result: list[int] = []
    for item in text.split(","):
        chunk = item.strip()
        if not chunk:
            continue
        result.append(int(chunk))
    if not result:
        raise ValueError("depth list is empty")
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare old/current same-seed FFB traces and replay old intervals on current.")
    parser.add_argument("--out-json", type=Path, default=OUTPUT_DEFAULT)
    parser.add_argument("--save-box-table-json", type=Path, default=BOX_TABLE_DEFAULT)
    parser.add_argument("--old-build-dir", type=Path, default=OLD_BUILD_DEFAULT)
    parser.add_argument("--current-build-dir", type=Path, default=CURRENT_BUILD_DEFAULT)
    parser.add_argument("--preset", default="support_hull_coverage")
    parser.add_argument("--envelope", choices=["preset", "link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--split-policy", choices=["widest-first", "best-tighten"], default="best-tighten")
    parser.add_argument("--depths", default="1,2,3,4,5,6,8,10,12,16,24,32,48,64,120")
    parser.add_argument("--interval-reference-depth", type=int, default=None)
    parser.add_argument("--coverage-seed-index", type=int, default=0)
    parser.add_argument("--include-extra-anchors", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--disable-current-canonical-symmetry", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--best-tighten-shape-balancing", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--best-tighten-recent-dim-cooling", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args(argv)


def obstacle_bounds(obstacles: list[Any]) -> list[list[float]]:
    return [[float(value) for value in obstacle.bounds] for obstacle in obstacles]


def make_compare_args(depth: int, database_path: Path, cli_args: argparse.Namespace) -> argparse.Namespace:
    args = p4.parse_args([])
    args.out_json = database_path.with_suffix(".json")
    args.database_path = database_path
    args.preset = cli_args.preset
    args.envelope = cli_args.envelope
    args.split_policy = cli_args.split_policy
    args.best_tighten_shape_balancing = bool(cli_args.best_tighten_shape_balancing)
    args.best_tighten_recent_dim_cooling = bool(cli_args.best_tighten_recent_dim_cooling)
    args.canonical_symmetry = not bool(cli_args.disable_current_canonical_symmetry)
    args.ffb_depth = int(depth)
    args.seed_base = int(cli_args.seed_base)
    args.seeds = 1
    args.threads = 1
    args.task_batch_size = 1
    args.max_boxes = 1
    args.timeout_ms = float(cli_args.timeout_ms)
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


def current_mode_tag(cli_args: argparse.Namespace) -> str:
    split_tag = "best_tighten" if str(cli_args.split_policy) == "best-tighten" else str(cli_args.split_policy).replace("-", "_")
    canonical_tag = "nocanon" if bool(cli_args.disable_current_canonical_symmetry) else "canon"
    suffixes: list[str] = []
    if bool(cli_args.best_tighten_shape_balancing):
        suffixes.append("shapebal")
    if bool(cli_args.best_tighten_recent_dim_cooling):
        suffixes.append("cooling")
    suffix = "" if not suffixes else "_" + "_".join(suffixes)
    return f"{canonical_tag}_{split_tag}{suffix}"


def canonical_interval_pairs(pairs: list[list[float]], ndigits: int = 12) -> tuple[tuple[float, float], ...]:
    return tuple((round(float(lo), ndigits), round(float(hi), ndigits)) for lo, hi in pairs)


def current_trace_events(trace_result: dict[str, Any]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    sequence = 0
    for step in trace_result.get("trace", []):
        if "validation" in step:
            detail = step.get("validation_detail", {})
            events.append({
                "sequence": sequence,
                "kind": "validation",
                "depth": int(step.get("depth", 0)),
                "node": int(step.get("node", -1)),
                "validation": int(step.get("validation", -1)),
                "safety_status": int(detail.get("safety_status", -1)),
                "intervals": canonical_interval_pairs(step.get("query_intervals", [])),
            })
            sequence += 1
        if step.get("split", False):
            events.append({
                "sequence": sequence,
                "kind": "split",
                "depth": int(step.get("depth", 0)),
                "node": int(step.get("node", -1)),
                "split_dim": int(step.get("split_dim", -1)),
                "split_val": round(float(step.get("split_value", 0.0)), 12),
            })
            sequence += 1
    return events


def old_trace_events(old_result: dict[str, Any]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for event in old_result.get("validation_events", []):
        events.append({
            "sequence": int(event.get("sequence", 0)),
            "kind": "validation",
            "depth": int(event.get("depth", 0)),
            "node": int(event.get("node", -1)),
            "validation": int(event.get("validation", -1)),
            "safety_status": int(event.get("safety_status", -1)),
            "intervals": canonical_interval_pairs(event.get("intervals", [])),
        })
    for event in old_result.get("split_events", []):
        events.append({
            "sequence": int(event.get("sequence", 0)),
            "kind": "split",
            "depth": int(event.get("depth", 0)),
            "node": int(event.get("node", -1)),
            "split_dim": int(event.get("split_dim", -1)),
            "split_val": round(float(event.get("split_val", 0.0)), 12),
        })
    ordered = sorted(
        events,
        key=lambda item: (
            int(item.get("sequence", 0)),
            0 if item.get("kind") == "validation" else 1,
        ),
    )
    for index, item in enumerate(ordered):
        item["sequence"] = index
    return ordered


def first_trace_divergence(old_events: list[dict[str, Any]], current_events: list[dict[str, Any]]) -> dict[str, Any] | None:
    prefix = min(len(old_events), len(current_events))
    for index in range(prefix):
        if old_events[index] != current_events[index]:
            return {"index": index, "old": old_events[index], "current": current_events[index]}
    if len(old_events) != len(current_events):
        return {
            "index": prefix,
            "old": old_events[prefix] if prefix < len(old_events) else None,
            "current": current_events[prefix] if prefix < len(current_events) else None,
        }
    return None


def geometric_mean_width(interval_pairs: list[list[float]]) -> float:
    widths = [max(0.0, float(hi) - float(lo)) for lo, hi in interval_pairs]
    positive = [width for width in widths if width > 0.0]
    if not positive:
        return 0.0
    return float(math.exp(sum(math.log(width) for width in positive) / len(positive)))


def write_box_table(path: Path, validation_events: list[dict[str, Any]]) -> None:
    boxes = []
    for index, event in enumerate(validation_events):
        interval_pairs = [[float(lo), float(hi)] for lo, hi in event.get("intervals", [])]
        boxes.append({
            "box_id": f"val_{index:04d}",
            "intervals": interval_pairs,
            "width_geo_mean": geometric_mean_width(interval_pairs),
        })
    payload = {
        "version": 1,
        "protocol": "old_ffb_validation_intervals",
        "generation": "single_seed_ffb_trace",
        "width_bins": [{
            "name": "trace_intervals",
            "width_bin": "trace_intervals",
            "fixed_width": 0.0,
            "width_lo": 0.0,
            "width_hi": 0.0,
            "boxes": boxes,
        }],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def run_old_trace(old_build_dir: Path,
                  depth: int,
                  seed_values: list[float],
                  obstacle_bounds_payload: list[list[float]],
                  cli_args: argparse.Namespace) -> dict[str, Any]:
    work_dir = cli_args.out_json.parent / "compare_work" / f"old_d{depth:03d}"
    work_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "depth": int(depth),
        "seed": [float(value) for value in seed_values],
        "obstacle_bounds": obstacle_bounds_payload,
        "seed_base": int(cli_args.seed_base),
        "timeout_ms": float(cli_args.timeout_ms),
        "preset": cli_args.preset,
        "envelope": cli_args.envelope,
        "split_policy": cli_args.split_policy,
        "best_tighten_shape_balancing": bool(cli_args.best_tighten_shape_balancing),
        "best_tighten_recent_dim_cooling": bool(cli_args.best_tighten_recent_dim_cooling),
        "database_path": str(work_dir / "db"),
        "out_json": str(work_dir / "trace.json"),
    }
    env = os.environ.copy()
    env["SBF_BUILD_DIR"] = str(old_build_dir)
    env["COMPARE_PAYLOAD_JSON"] = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    extra_paths = [str(REPO_ROOT), str(Path(__file__).resolve().parent)]
    current_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join([*extra_paths, current_pythonpath] if current_pythonpath else extra_paths)
    completed = subprocess.run(
        [sys.executable, "-c", OLD_TRACE_SNIPPET],
        cwd=str(REPO_ROOT),
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "old trace subprocess failed\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    stdout = completed.stdout.strip()
    if not stdout:
        raise RuntimeError("old trace subprocess produced no JSON output")
    return json.loads(stdout)


def run_current_trace(depth: int,
                      seed_values: list[float],
                      obstacles: list[Any],
                      cli_args: argparse.Namespace) -> dict[str, Any]:
    current_label = f"current_{current_mode_tag(cli_args)}"
    work_dir = cli_args.out_json.parent / "compare_work" / f"{current_label}_d{depth:03d}"
    work_dir.mkdir(parents=True, exist_ok=True)
    args = make_compare_args(depth, work_dir / "db", cli_args)
    robot = load_iiwa14_robot()
    config = p4.configure(args, 0)
    if bool(cli_args.disable_current_canonical_symmetry):
        config.database.canonical_mode = False
    forest = sbf.SafeBoxForest(robot, config)
    result = forest.debug_find_free_box(seed_values, obstacles, config.grower.find_free_box)
    result["current_canonical_mode"] = bool(config.database.canonical_mode)
    return dict(result)


def replay_old_intervals_on_current(depth: int,
                                    old_result: dict[str, Any],
                                    obstacles: list[Any],
                                    cli_args: argparse.Namespace) -> list[dict[str, Any]]:
    replay_label = f"current_replay_{current_mode_tag(cli_args)}"
    work_dir = cli_args.out_json.parent / "compare_work" / f"{replay_label}_d{depth:03d}"
    work_dir.mkdir(parents=True, exist_ok=True)
    args = make_compare_args(depth, work_dir / "db", cli_args)
    robot = load_iiwa14_robot()
    config = p4.configure(args, 0)
    if bool(cli_args.disable_current_canonical_symmetry):
        config.database.canonical_mode = False
    forest = sbf.SafeBoxForest(robot, config)
    replays: list[dict[str, Any]] = []
    for event in old_result.get("validation_events", []):
        current = dict(forest.debug_validate_intervals(obstacles, event.get("intervals", [])))
        replay = {
            "sequence": int(event.get("sequence", 0)),
            "depth": int(event.get("depth", 0)),
            "intervals": event.get("intervals", []),
            "old_validation": int(event.get("validation", -1)),
            "old_safety_status": int(event.get("safety_status", -1)),
            "current_validation": int(current.get("validation", -1)),
            "current_safety_status": int(current.get("validation_detail", {}).get("safety_status", -1)),
            "current_collision_possible": bool(current.get("validation_detail", {}).get("collision_possible", True)),
            "current_canonical_mode": bool(config.database.canonical_mode),
            "validation_match": int(event.get("validation", -1)) == int(current.get("validation", -1)),
            "safety_status_match": int(event.get("safety_status", -1)) == int(current.get("validation_detail", {}).get("safety_status", -1)),
            "current": current,
        }
        replay["status_only_mismatch"] = replay["validation_match"] and not replay["safety_status_match"]
        replay["match"] = replay["validation_match"]
        replays.append(replay)
    return replays


def summarize_depth(old_result: dict[str, Any], current_result: dict[str, Any]) -> dict[str, Any]:
    old_events = old_trace_events(old_result)
    current_events = current_trace_events(current_result)
    return {
        "depth": int(old_result.get("depth", -1)),
        "old_validation_events": len(old_result.get("validation_events", [])),
        "old_split_events": len(old_result.get("split_events", [])),
        "old_boxes": len(old_result.get("boxes", [])),
        "current_validation_events": len(current_result.get("validation_events", [])),
        "current_split_events": len(current_result.get("split_events", [])),
        "current_found": bool(current_result.get("found", False)),
        "current_fail_code": int(current_result.get("fail_code", -1)),
        "first_trace_divergence": first_trace_divergence(old_events, current_events),
        "old_events": old_events,
        "current_events": current_events,
    }


def first_interval_replay_mismatch(replays: list[dict[str, Any]]) -> dict[str, Any] | None:
    for replay in replays:
        if not replay.get("match", False):
            return replay
    return None


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    depths = parse_depths(args.depths)
    interval_reference_depth = int(args.interval_reference_depth) if args.interval_reference_depth is not None else max(depths)
    if interval_reference_depth not in depths:
        depths = sorted(set([*depths, interval_reference_depth]))

    robot = load_iiwa14_robot()
    seeds = make_coverage_seeds(include_extra_anchors=bool(args.include_extra_anchors))
    if args.coverage_seed_index < 0 or args.coverage_seed_index >= len(seeds):
        raise ValueError(f"coverage seed index {args.coverage_seed_index} out of range [0, {len(seeds) - 1}]")
    seed_values = [float(value) for value in seeds[args.coverage_seed_index]]
    obstacles = make_combined_obstacles()
    obstacle_bounds_payload = obstacle_bounds(obstacles)

    depth_results: list[dict[str, Any]] = []
    old_by_depth: dict[int, dict[str, Any]] = {}
    current_by_depth: dict[int, dict[str, Any]] = {}
    for depth in sorted(depths):
        old_result = run_old_trace(args.old_build_dir, depth, seed_values, obstacle_bounds_payload, args)
        current_result = run_current_trace(depth, seed_values, obstacles, args)
        old_by_depth[depth] = old_result
        current_by_depth[depth] = current_result
        depth_results.append(summarize_depth(old_result, current_result))

    reference_old = old_by_depth[interval_reference_depth]
    interval_replays = replay_old_intervals_on_current(interval_reference_depth, reference_old, obstacles, args)
    first_mismatch = first_interval_replay_mismatch(interval_replays)
    if args.save_box_table_json is not None:
        write_box_table(args.save_box_table_json, reference_old.get("validation_events", []))

    payload = {
        "experiment": "old_current_interval_ffb_compare",
        "old_build_dir": str(args.old_build_dir),
        "current_build_dir": str(args.current_build_dir),
        "disable_current_canonical_symmetry": bool(args.disable_current_canonical_symmetry),
        "preset": args.preset,
        "envelope": args.envelope,
        "split_policy": args.split_policy,
        "best_tighten_shape_balancing": bool(args.best_tighten_shape_balancing),
        "best_tighten_recent_dim_cooling": bool(args.best_tighten_recent_dim_cooling),
        "coverage_seed_index": int(args.coverage_seed_index),
        "coverage_seed": seed_values,
        "depths": sorted(depths),
        "interval_reference_depth": interval_reference_depth,
        "notes": [
            "old Python bindings do not expose raw envelope payloads; the interval replay stage compares current envelope/oracle classification on the exact intervals visited by old FFB.",
            "depth stage compares chronological split/validation trace prefixes for one seed in the same Marcucci combined environment.",
        ],
        "depth_results": depth_results,
        "interval_replays": interval_replays,
        "first_interval_replay_mismatch": first_mismatch,
        "reference_old_trace": reference_old,
        "reference_current_trace": current_by_depth[interval_reference_depth],
        "save_box_table_json": str(args.save_box_table_json) if args.save_box_table_json is not None else None,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "out_json": str(args.out_json),
        "depths": sorted(depths),
        "reference_depth": interval_reference_depth,
        "first_interval_replay_mismatch": None if first_mismatch is None else {
            "sequence": int(first_mismatch.get("sequence", -1)),
            "depth": int(first_mismatch.get("depth", -1)),
            "old_validation": int(first_mismatch.get("old_validation", -1)),
            "current_validation": int(first_mismatch.get("current_validation", -1)),
        },
    }, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())