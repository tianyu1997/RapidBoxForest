from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.marcucci_anchor_guard import CANONICAL_ANCHORS  # noqa: E402
from safe_box_forest.experiments.sbf_old.common_sbf_config import (  # noqa: E402
    RBF_LIFELONG_PRESET,
    add_common_sbf_args,
    configure_standalone_sbf,
    make_aafk_volume_min_split_policy,
    set_online_cache_backfill,
)

import sbf  # noqa: E402


DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "new_experiments" / "exp04_restricted_root_probe_20260531"
DEFAULT_MARGIN_SCALES = (0.20, 0.35, 0.50)
DEFAULT_DEPTHS = (24, 28, 32, 36, 40, 44, 48)
DEFAULT_ANCHORS = ("AS", "TS", "CS", "LB", "RB")


def parse_csv_floats(text: str) -> tuple[float, ...]:
    values = [item.strip() for item in str(text).split(",") if item.strip()]
    return tuple(float(item) for item in values)


def parse_csv_ints(text: str) -> tuple[int, ...]:
    values = [item.strip() for item in str(text).split(",") if item.strip()]
    return tuple(int(item) for item in values)


def interval_pairs(intervals: Iterable[Any]) -> list[list[float]]:
    return [[float(interval.lo), float(interval.hi)] for interval in intervals]


def make_restricted_root_intervals(
    robot: Any,
    anchor_names: Iterable[str],
    margin_scale: float,
    keep_full_dims: set[int],
) -> list[Any]:
    root = list(sbf.canonical_root_intervals_for_robot(robot, True, "joint_symmetry_native_v1"))
    anchors = [CANONICAL_ANCHORS[name] for name in anchor_names]
    mins = [min(float(q[dim]) for q in anchors) for dim in range(len(root))]
    maxs = [max(float(q[dim]) for q in anchors) for dim in range(len(root))]
    restricted: list[Any] = []
    for dim, (base, lo, hi) in enumerate(zip(root, mins, maxs)):
        if dim in keep_full_dims:
            restricted.append(sbf.Interval(float(base.lo), float(base.hi)))
            continue
        span = max(0.0, hi - lo)
        pad = float(margin_scale) * span
        out_lo = max(float(base.lo), lo - pad)
        out_hi = min(float(base.hi), hi + pad)
        restricted.append(sbf.Interval(out_lo, out_hi))
    return restricted


def root_stats(root_intervals: list[Any], baseline: list[Any]) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for dim, (root, base) in enumerate(zip(root_intervals, baseline)):
        width = float(root.hi - root.lo)
        base_width = float(base.hi - base.lo)
        rows.append({
            "dim": int(dim),
            "lo": float(root.lo),
            "hi": float(root.hi),
            "width": width,
            "baseline_width": base_width,
            "shrink_ratio": (width / base_width) if base_width > 0.0 else 0.0,
        })
    return rows


def make_base_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    add_common_sbf_args(parser)
    args = parser.parse_args([])
    args.preset = RBF_LIFELONG_PRESET
    args.rbf_envelope = "support_hull"
    args.threads = 1
    args.task_batch_size = 1
    args.seed_base = 0
    args.max_boxes = 1
    args.timeout_ms = 1.0
    args.rbf_ffb_start_depth = 0
    args.rbf_canonical_cache = True
    args.endpoint_source = "aafk"
    return args


def build_probe_forest(
    robot: Any,
    depth: int,
    database_path: Path,
    root_intervals: list[Any],
    sample_nodes_per_depth: int,
) -> tuple[Any, Any]:
    args = make_base_args()
    args.rbf_cache_root = database_path.parent
    args.rbf_cache_label = database_path.name
    args.rbf_max_depth = int(depth)
    args.ffb_depth = int(depth)
    args.connector_pave_depth = int(depth)
    args.component_connect_ffb_max_depth = int(depth)
    cfg = configure_standalone_sbf(args, seed=0, preset=RBF_LIFELONG_PRESET, robot=robot)
    cfg.database.path = str(database_path)
    cfg.database.root_intervals_override = list(root_intervals)
    cfg.database.split_policy = make_aafk_volume_min_split_policy(
        robot,
        int(depth),
        root_intervals=root_intervals,
        sample_nodes_per_depth=int(sample_nodes_per_depth),
    )
    set_online_cache_backfill(cfg, True)
    return sbf.SafeBoxForest(robot, cfg), cfg


def probe_depth(
    robot: Any,
    obstacles: list[Any],
    anchor_names: Iterable[str],
    depth: int,
    root_intervals: list[Any],
    database_path: Path,
    sample_nodes_per_depth: int,
) -> dict[str, Any]:
    if database_path.exists():
        shutil.rmtree(database_path)
    forest, cfg = build_probe_forest(robot, depth, database_path, root_intervals, sample_nodes_per_depth)
    opts = cfg.grower.find_free_box
    opts.max_depth = int(depth)
    anchor_results: dict[str, Any] = {}
    for name in anchor_names:
        result = dict(forest.debug_find_free_box(list(CANONICAL_ANCHORS[name]), obstacles, opts))
        anchor_results[name] = {
            "found": bool(result.get("found", False)),
            "splits": int(result.get("splits", 0)),
            "fail_code": int(result.get("fail_code", -1)),
            "hit_unknown_depth_cap": bool(result.get("hit_unknown_depth_cap", False)),
            "split_dim_hist": {
                str(dim): int(count)
                for dim, count in _split_dim_hist(result.get("split_events", []))
            },
        }
    schedule = list(
        sbf.aafk_volume_min_depth_schedule(
            robot,
            root_intervals,
            int(depth),
            int(sample_nodes_per_depth),
        )
    )
    ok = all(bool(item["found"]) for item in anchor_results.values())
    forest.database_wait_for_snapshot_publish()
    del forest
    return {
        "depth": int(depth),
        "ok": bool(ok),
        "database_path": str(database_path),
        "schedule": [int(dim) for dim in schedule],
        "anchor_results": anchor_results,
        "aafk_sample_nodes_per_depth": int(sample_nodes_per_depth),
    }


def _split_dim_hist(split_events: list[dict[str, Any]]) -> list[tuple[int, int]]:
    counts: dict[int, int] = {}
    for event in split_events:
        dim = int(event.get("split_dim", -1))
        counts[dim] = counts.get(dim, 0) + 1
    return sorted(counts.items())


def minimal_depths(depth_rows: list[dict[str, Any]], anchor_names: Iterable[str]) -> dict[str, int | None]:
    out: dict[str, int | None] = {str(name): None for name in anchor_names}
    for row in depth_rows:
        depth = int(row["depth"])
        for name in anchor_names:
            if out[str(name)] is None and bool(row["anchor_results"][str(name)]["found"]):
                out[str(name)] = depth
    return out


def pick_recommendation(candidates: list[dict[str, Any]]) -> dict[str, Any] | None:
    viable = [item for item in candidates if item.get("all_anchor_depth") is not None]
    if not viable:
        return None
    viable.sort(key=lambda item: (int(item["all_anchor_depth"]), -float(item["margin_scale"])))
    target_depth = int(viable[0]["all_anchor_depth"])
    same_depth = [item for item in viable if int(item["all_anchor_depth"]) == target_depth]
    same_depth.sort(key=lambda item: float(item["margin_scale"]))
    mid = len(same_depth) // 2
    return same_depth[mid]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Probe shelf restricted-root LECT depth compression.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--margin-scales", type=parse_csv_floats, default=DEFAULT_MARGIN_SCALES)
    parser.add_argument("--depths", type=parse_csv_ints, default=DEFAULT_DEPTHS)
    parser.add_argument("--anchors", default=",".join(DEFAULT_ANCHORS))
    parser.add_argument("--keep-full-dims", type=parse_csv_ints, default=(0,))
    parser.add_argument("--clean", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--aafk-sample-nodes-per-depth", type=int, default=8)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    anchor_names = tuple(item.strip() for item in str(args.anchors).split(",") if item.strip())
    keep_full_dims = {int(dim) for dim in tuple(args.keep_full_dims)}
    if args.clean and args.out_dir.exists():
        shutil.rmtree(args.out_dir)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    robot = sbf.load_iiwa14_robot()
    obstacles = sbf.make_combined_obstacles()
    canonical_root = list(sbf.canonical_root_intervals_for_robot(robot, True, "joint_symmetry_native_v1"))

    candidates: list[dict[str, Any]] = []
    for margin_scale in tuple(args.margin_scales):
        root_intervals = make_restricted_root_intervals(robot, anchor_names, float(margin_scale), keep_full_dims)
        depth_rows: list[dict[str, Any]] = []
        candidate_dir = args.out_dir / f"margin_{str(margin_scale).replace('.', 'p')}"
        candidate_dir.mkdir(parents=True, exist_ok=True)
        for depth in tuple(args.depths):
            probe = probe_depth(
                robot,
                obstacles,
                anchor_names,
                int(depth),
                root_intervals,
                candidate_dir / f"db_d{int(depth)}",
                int(args.aafk_sample_nodes_per_depth),
            )
            depth_rows.append(probe)
        anchor_min_depth = minimal_depths(depth_rows, anchor_names)
        all_anchor_depth = None
        if all(value is not None for value in anchor_min_depth.values()):
            all_anchor_depth = max(int(value) for value in anchor_min_depth.values() if value is not None)
        candidate = {
            "margin_scale": float(margin_scale),
            "anchor_names": list(anchor_names),
            "keep_full_dims": sorted(int(dim) for dim in keep_full_dims),
            "root_intervals": interval_pairs(root_intervals),
            "root_stats": root_stats(root_intervals, canonical_root),
            "depth_rows": depth_rows,
            "anchor_min_depth": anchor_min_depth,
            "all_anchor_depth": all_anchor_depth,
        }
        candidates.append(candidate)
        (candidate_dir / "summary.json").write_text(json.dumps(candidate, indent=2, sort_keys=True), encoding="utf-8")

    recommendation = pick_recommendation(candidates)
    summary = {
        "out_dir": str(args.out_dir),
        "anchor_names": list(anchor_names),
        "margin_scales": [float(value) for value in tuple(args.margin_scales)],
        "depths": [int(value) for value in tuple(args.depths)],
        "keep_full_dims": sorted(int(dim) for dim in keep_full_dims),
        "aafk_sample_nodes_per_depth": int(args.aafk_sample_nodes_per_depth),
        "canonical_root_intervals": interval_pairs(canonical_root),
        "candidates": candidates,
        "recommendation": recommendation,
    }
    out_json = args.out_dir / "summary.json"
    out_json.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "summary_json": str(out_json),
        "recommendation": recommendation,
    }, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())