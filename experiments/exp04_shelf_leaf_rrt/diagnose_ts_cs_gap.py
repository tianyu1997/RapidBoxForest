#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import write_json
from experiments.common.progress import progress
from experiments.common.sbf_import import import_sbf
from experiments.exp04_shelf_leaf_rrt.study_ts_cs_box_cover import (
    box_contains,
    build_forest,
    densify,
    dist,
)

sbf = import_sbf()


def interval_gap(box: list[list[float]], q: list[float]) -> float:
    gap_sq = 0.0
    for lo_hi, value in zip(box, q):
        lo, hi = float(lo_hi[0]), float(lo_hi[1])
        v = float(value)
        if v < lo:
            gap_sq += (lo - v) ** 2
        elif v > hi:
            gap_sq += (v - hi) ** 2
    return math.sqrt(gap_sq)


def boxes_touch(a: list[list[float]], b: list[list[float]], tol: float = 1e-9) -> bool:
    for lhs, rhs in zip(a, b):
        if float(lhs[1]) + tol < float(rhs[0]):
            return False
        if float(rhs[1]) + tol < float(lhs[0]):
            return False
    return True


def load_ts_cs_path(manifest_path: Path, seed: int) -> list[list[float]]:
    data = json.loads(manifest_path.read_text())
    for row in data.get("rows", []):
        if int(row.get("seed", -1)) != int(seed):
            continue
        for query in row.get("queries", []):
            if query.get("label") == "TS->CS":
                path = query.get("path")
                if path:
                    return [[float(v) for v in point] for point in path]
                # Older manifests do not store the path directly. Fall back to
                # canonical endpoints plus intermediate debug path if present.
                points = query.get("debug_path") or query.get("waypoints")
                if points:
                    return [[float(v) for v in point] for point in points]
                raise RuntimeError("TS->CS row found, but no path field is present")
    raise RuntimeError(f"TS->CS seed {seed} not found in {manifest_path}")


def load_ts_cs_path_or_replay(manifest_path: Path, seed: int) -> list[list[float]]:
    try:
        return load_ts_cs_path(manifest_path, seed)
    except Exception:
        return []


def path_arclengths(path: list[list[float]]) -> list[float]:
    acc = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        acc.append(acc[-1] + dist(a, b))
    return acc


def sample_arc(path: list[list[float]], q: list[float], arclens: list[float]) -> float:
    best_arc = 0.0
    best_dist = float("inf")
    for index, (a, b) in enumerate(zip(path[:-1], path[1:])):
        ab = [float(y) - float(x) for x, y in zip(a, b)]
        aq = [float(x) - float(y) for x, y in zip(q, a)]
        denom = sum(v * v for v in ab)
        u = 0.0 if denom <= 1e-18 else max(0.0, min(1.0, sum(x * y for x, y in zip(aq, ab)) / denom))
        proj = [float(x) + u * v for x, v in zip(a, ab)]
        d = dist(proj, q)
        if d < best_dist:
            best_dist = d
            best_arc = arclens[index] + u * math.sqrt(denom)
    return best_arc


def contiguous_gaps(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    gaps: list[dict[str, Any]] = []
    start = None
    last = None
    for index, row in enumerate(samples):
        if not row["covered_all"]:
            if start is None:
                start = index
            last = index
        elif start is not None and last is not None:
            gaps.append({"start_index": start, "end_index": last})
            start = None
            last = None
    if start is not None and last is not None:
        gaps.append({"start_index": start, "end_index": last})
    for gap in gaps:
        subset = samples[gap["start_index"] : gap["end_index"] + 1]
        gap["count"] = len(subset)
        gap["arc_start"] = subset[0]["arc"]
        gap["arc_end"] = subset[-1]["arc"]
        gap["arc_mid"] = subset[len(subset) // 2]["arc"]
        gap["nearest_gap_min"] = min(float(row["nearest_all_gap"]) for row in subset)
        gap["nearest_gap_max"] = max(float(row["nearest_all_gap"]) for row in subset)
        gap["representative_index"] = subset[len(subset) // 2]["sample_index"]
    return gaps


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--rbf-max-depth", type=int, default=128)
    parser.add_argument("--connector-pave-depth", type=int, default=128)
    parser.add_argument("--connector-pave-max-chain", type=int, default=256)
    parser.add_argument("--sample-step", type=float, default=0.0025)
    parser.add_argument("--coverage-step", type=float, default=0.01)
    parser.add_argument("--ffb-depth", type=int, default=128)
    parser.add_argument("--disable-caches", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--threads", type=int, default=8)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    # Reuse the study helper's argparse-shaped object for an identical build
    # profile, but keep connector segments disabled for pure coverage analysis.
    helper_args = argparse.Namespace(
        seed=args.seed,
        box_budget=200,
        rbf_max_depth=args.rbf_max_depth,
        timeout_ms=8000.0,
        threads=args.threads,
        leaf_start_depth=14,
        leaf_max_depth=18,
        deep_ffb_depth=40,
        ffb_start_depth=23,
        validation_batch_size=512,
        audit_segment_step=0.01,
        connector_pair_timeout_ms=10.0,
        connector_max_pairs_per_gap=4,
        connector_pave_max_chain=args.connector_pave_max_chain,
        connector_pave_depth=args.connector_pave_depth,
        collision_overlap_prune_min_depth=14,
        collision_overlap_prune_threshold=0.05,
        collision_overlap_prune_ratio_threshold=0.0,
        rbf_cache_root=Path("outputs/lect_cache"),
        warm_cache_label="unused",
        out_dir=args.out_dir,
    )
    path = load_ts_cs_path_or_replay(args.manifest, args.seed)
    if not path:
        reference_forest, _robot, _obstacles, reference_queries, _build = build_forest(
            helper_args,
            args.out_dir / "reference_cache",
            connector_bridge_boxes=0,
            query_bridge_all=False,
            segment_edges_enabled=True,
        )
        query = next(q for q in reference_queries if str(q.label) == "TS->CS")
        reference_forest.bridge_query([float(v) for v in query.start], [float(v) for v in query.goal])
        reference_result = reference_forest.query([float(v) for v in query.start], [float(v) for v in query.goal])
        if not bool(reference_result.success):
            raise RuntimeError("failed to replay TS->CS reference bridge")
        path = [[float(v) for v in point] for point in reference_result.path_as_lists()]

    forest, _robot, obstacles, _queries, build = build_forest(
        helper_args,
        args.out_dir / "diagnostic_cache",
        connector_bridge_boxes=0,
        query_bridge_all=False,
        segment_edges_enabled=False,
    )
    debug = dict(
        forest.debug_chain_pave_waypoints(
            path,
            max_chain=int(args.connector_pave_max_chain),
            max_depth=int(args.connector_pave_depth),
            max_gap_fill_steps=8,
            fill_segment_gaps=True,
            gap_fill_min_step=1e-4,
            adjacency_tolerance=1e-9,
            gap_fill_sample_step=float(args.sample_step),
            gap_fill_time_budget_ms=0.0,
            gap_fill_max_ffb_calls=-1,
            gap_fill_min_arc_gain=0.0,
            require_connected_chain=False,
            commit_certified_only=True,
        )
    )
    all_boxes = list(debug.get("all_boxes", []))
    committed_boxes = list(debug.get("committed_boxes", []))
    arclens = path_arclengths(path)
    dense_samples = densify(path, args.coverage_step)

    sample_rows: list[dict[str, Any]] = []
    for index, q in enumerate(dense_samples):
        nearest_gap, nearest_idx = min(
            ((interval_gap(box, q), i) for i, box in enumerate(all_boxes)),
            default=(float("inf"), -1),
        )
        nearest_committed_gap, nearest_committed_idx = min(
            ((interval_gap(box, q), i) for i, box in enumerate(committed_boxes)),
            default=(float("inf"), -1),
        )
        sample_rows.append(
            {
                "sample_index": index,
                "arc": sample_arc(path, q, arclens),
                "q": q,
                "covered_all": any(box_contains(box, q) for box in all_boxes),
                "covered_committed": any(box_contains(box, q) for box in committed_boxes),
                "nearest_all_box": nearest_idx,
                "nearest_all_gap": nearest_gap,
                "nearest_committed_box": nearest_committed_idx,
                "nearest_committed_gap": nearest_committed_gap,
            }
        )
    gaps = contiguous_gaps(sample_rows)

    ffb_options = sbf.FindFreeBoxOptions()
    ffb_options.max_depth = int(args.ffb_depth)
    ffb_options.reject_seed_collision = False
    representatives = []
    probe_jobs: list[tuple[dict[str, Any], int, str]] = []
    for gap in gaps:
        start = int(gap["start_index"])
        end = int(gap["end_index"])
        mid = int(gap["representative_index"])
        for sample_index, role in [(start, "start"), (mid, "mid"), (end, "end")]:
            if not any(
                existing_gap is gap and existing_index == sample_index and existing_role == role
                for existing_gap, existing_index, existing_role in probe_jobs
            ):
                probe_jobs.append((gap, sample_index, role))
    for gap, sample_index, role in progress(probe_jobs, desc="gap ffb diagnostics", total=len(probe_jobs)):
        q = sample_rows[sample_index]["q"]
        ffb = dict(
            forest.debug_find_free_box(
                q,
                obstacles,
                ffb_options,
                bool(args.disable_caches),
            )
        )
        intervals = list(ffb.get("intervals", []))
        adjacent_count = 0
        nearest_to_result = float("inf")
        if intervals:
            adjacent_count = sum(1 for box in all_boxes if boxes_touch(intervals, box))
            nearest_to_result = min((interval_gap(box, q) for box in all_boxes), default=float("inf"))
        representatives.append(
            {
                "gap": gap,
                "probe_role": role,
                "sample_index": sample_index,
                "sample": q,
                "ffb_found": bool(ffb.get("found", False)),
                "ffb_fail_code": int(ffb.get("fail_code", -1)),
                "seed_collision": bool(ffb.get("seed_collision", False)),
                "hit_reserved_depth_cap": bool(ffb.get("hit_reserved_depth_cap", False)),
                "hit_unknown_depth_cap": bool(ffb.get("hit_unknown_depth_cap", False)),
                "effective_max_depth": int(ffb.get("effective_max_depth", -1)),
                "decisions": int(ffb.get("decisions", 0)),
                "splits": int(ffb.get("splits", 0)),
                "total_ms": float(ffb.get("total_ms", 0.0)),
                "result_intervals": intervals,
                "result_adjacent_existing_count": adjacent_count,
                "nearest_existing_gap_at_sample": nearest_to_result,
                "validation_detail": ffb.get("validation_detail", {}),
                "last_validation": (ffb.get("validation_events", []) or [{}])[-1],
            }
        )

    with (args.out_dir / "ts_cs_seed_gap_samples.csv").open("w", newline="") as f:
        fieldnames = [
            "sample_index",
            "arc",
            "covered_all",
            "covered_committed",
            "nearest_all_box",
            "nearest_all_gap",
            "nearest_committed_box",
            "nearest_committed_gap",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in sample_rows:
            writer.writerow({key: row[key] for key in fieldnames})

    write_json(
        args.out_dir / "ts_cs_seed_gap_diagnostics.json",
        {
            "seed": int(args.seed),
            "manifest": str(args.manifest),
            "path": path,
            "path_length": arclens[-1] if arclens else 0.0,
            "build_ms": float(build.total_ms),
            "debug": {
                key: value
                for key, value in debug.items()
                if key not in {"all_boxes", "committed_boxes", "waypoints"}
            },
            "all_box_count": len(all_boxes),
            "committed_box_count": len(committed_boxes),
            "sample_count": len(sample_rows),
            "uncovered_all_count": sum(1 for row in sample_rows if not row["covered_all"]),
            "uncovered_committed_count": sum(1 for row in sample_rows if not row["covered_committed"]),
            "gaps": gaps,
            "representatives": representatives,
        },
    )
    print(
        json.dumps(
            {
                "seed": args.seed,
                "path_length": arclens[-1] if arclens else 0.0,
                "added": debug.get("added"),
                "all_box_count": len(all_boxes),
                "committed_box_count": len(committed_boxes),
                "sample_count": len(sample_rows),
                "uncovered_all_count": sum(1 for row in sample_rows if not row["covered_all"]),
                "gap_count": len(gaps),
                "representatives": representatives,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
