#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import shutil
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.rbf_defaults import (
    D23_CACHE_LABEL,
    D23_CACHE_ROOT,
    DEFAULT_RBF_DEEP_FFB_DEPTH,
    DEFAULT_RBF_FFB_SEARCH_MODE,
    DEFAULT_RBF_FFB_START_DEPTH,
    DEFAULT_RBF_THREADS,
    robot_joint_limit_tuples,
)
from experiments.common.rbf_leaf_rrt import (
    RBFLeafRRTOptions,
    configure_leaf_rrt,
)
from experiments.common.sbf_import import import_sbf


def parse_csv_floats(text: str) -> list[float]:
    return [float(item.strip()) for item in str(text).split(",") if item.strip()]


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def interval_pairs(value: Any) -> list[list[float]]:
    pairs: list[list[float]] = []
    for interval in list(value or []):
        if isinstance(interval, (list, tuple)):
            pairs.append([float(interval[0]), float(interval[1])])
        else:
            pairs.append([float(interval.lo), float(interval.hi)])
    return pairs


def max_interval_abs_diff(lhs: list[list[float]], rhs: list[list[float]]) -> float:
    if len(lhs) != len(rhs):
        return math.inf
    diff = 0.0
    for a, b in zip(lhs, rhs):
        diff = max(diff, abs(float(a[0]) - float(b[0])), abs(float(a[1]) - float(b[1])))
    return diff


def seed_set(
    robot: Any,
    *,
    random_count: int,
    random_seed: int,
    boundary_eps: float,
    random_domain: list[tuple[float, float]],
) -> list[dict[str, Any]]:
    sbf = import_sbf()
    seeds: list[dict[str, Any]] = []
    critical_dim0 = [
        ("m1e-4", -1e-4),
        ("m1e-6", -1e-6),
        ("zero", 0.0),
        ("p1e-6", 1e-6),
        ("pi2_m_eps", 0.5 * math.pi - float(boundary_eps)),
        ("pi2_p_eps", 0.5 * math.pi + float(boundary_eps)),
        ("pi", math.pi),
        ("m_pi2", -0.5 * math.pi),
    ]
    anchor_map = {str(item): tuple(float(v) for v in values) for item, values in sbf.ANCHORS.items()}
    for anchor_name in ("TS", "CS"):
        base = list(anchor_map[anchor_name])
        for value_name, dim0 in critical_dim0:
            q = list(base)
            q[0] = float(dim0)
            seeds.append({
                "label": f"critical_dim0_{value_name}_{anchor_name}",
                "source": "critical_dim0",
                "q": q,
            })
    for query in sbf.make_combined_queries():
        start = [float(v) for v in query.start]
        goal = [float(v) for v in query.goal]
        for alpha in (0.0, 0.25, 0.5, 0.75, 1.0):
            q = [(1.0 - alpha) * a + alpha * b for a, b in zip(start, goal)]
            seeds.append({
                "label": f"{query.label}@{alpha:.2f}",
                "source": "query_interpolation",
                "q": q,
            })
    rng = random.Random(int(random_seed))
    for index in range(max(0, int(random_count))):
        q = [rng.uniform(float(lo), float(hi)) for lo, hi in random_domain]
        seeds.append({
            "label": f"random{index:02d}",
            "source": "uniform_shared_domain",
            "q": q,
        })
    return seeds


def make_options(mode: str, args: argparse.Namespace, robot: Any) -> RBFLeafRRTOptions:
    if mode == "canonical_d23_cache":
        root = robot_joint_limit_tuples(robot)
        return RBFLeafRRTOptions(
            seed=int(args.seed),
            rbf_max_depth=int(args.max_depth),
            threads=int(args.threads),
            ffb_start_depth=int(args.start_depth),
            ffb_search_mode=str(args.search_mode),
            envelope="support_hull",
            endpoint_source="ifk",
            use_external_evidence=True,
            external_evidence_path=Path(args.cache_root) / str(args.cache_label),
            external_evidence_verify_identity=False,
            root_override_tuples=root,
            coverage_override_tuples=root,
            database_canonical_mode=True,
            case_label=mode,
            canonicalize_queries=False,
        )
    if mode == "native_no_cache":
        root = robot_joint_limit_tuples(robot)
        return RBFLeafRRTOptions(
            seed=int(args.seed),
            rbf_max_depth=int(args.max_depth),
            threads=int(args.threads),
            ffb_start_depth=int(args.start_depth),
            ffb_search_mode=str(args.search_mode),
            envelope="support_hull",
            endpoint_source="ifk",
            use_external_evidence=False,
            external_evidence_verify_identity=False,
            root_override_tuples=root,
            coverage_override_tuples=root,
            database_canonical_mode=False,
            case_label=mode,
            canonicalize_queries=False,
        )
    raise ValueError(f"unknown mode: {mode}")


def make_find_free_box_options(args: argparse.Namespace) -> Any:
    sbf = import_sbf()
    options = sbf.FindFreeBoxOptions()
    options.max_depth = int(args.max_depth)
    options.start_depth = int(args.start_depth)
    options.skip_to_depth = int(args.start_depth)
    mode = str(args.search_mode).lower().replace("_", "-")
    if hasattr(sbf, "FindFreeBoxSearchMode"):
        options.search_mode = (
            sbf.FindFreeBoxSearchMode.BinaryDepth
            if mode in {"binary", "binary-depth", "binarydepth"}
            else sbf.FindFreeBoxSearchMode.Linear
        )
    options.split_reserved_leaf = True
    options.split_unknown_leaf = True
    options.reject_seed_collision = bool(args.reject_seed_collision)
    return options


def forest_for_mode(mode: str, args: argparse.Namespace, robot: Any) -> Any:
    sbf = import_sbf()
    path = Path(args.out_dir) / "tmp_db" / mode
    if path.exists():
        shutil.rmtree(path)
    cfg = configure_leaf_rrt(robot, path, make_options(mode, args, robot))
    cfg.enable_connector = False
    return sbf.SafeBoxForest(robot, cfg)


def compact_result(result: dict[str, Any]) -> dict[str, Any]:
    counters = dict(result.get("counters", {}) or {})
    validation_detail = dict(result.get("validation_detail", {}) or {})
    return {
        "exception": "",
        "found": bool(result.get("found", False)),
        "fail_code": int(result.get("fail_code", -1)),
        "node": int(result.get("node", -1)),
        "effective_max_depth": int(result.get("effective_max_depth", -1)),
        "decisions": int(result.get("decisions", 0)),
        "splits": int(result.get("splits", 0)),
        "seed_in_domain": bool(result.get("seed_in_domain", False)),
        "seed_collision": bool(result.get("seed_collision", False)),
        "hit_unknown_depth_cap": bool(result.get("hit_unknown_depth_cap", False)),
        "hit_reserved_depth_cap": bool(result.get("hit_reserved_depth_cap", False)),
        "deadline_reached": bool(result.get("deadline_reached", False)),
        "tree_seed": [float(v) for v in list(result.get("tree_seed", []) or [])],
        "intervals": interval_pairs(result.get("intervals", [])),
        "validation_safety_status": validation_detail.get("safety_status"),
        "validation_collision_possible": validation_detail.get("collision_possible"),
        "total_ms": float(result.get("total_ms", 0.0)),
        "external_exact_hits": int(counters.get("materialization_external_exact_hits", 0)),
        "external_exact_misses": int(counters.get("materialization_external_exact_misses", 0)),
        "external_reused": int(counters.get("materialization_reused_external_evidence", 0)),
        "canonical_frame_invalid": int(counters.get("canonical_frame_invalid", 0)),
        "canonical_reflected_seed_misses": int(counters.get("canonical_reflected_seed_misses", 0)),
        "materializations": int(counters.get("materializations", 0)),
    }


def compact_exception(exc: BaseException) -> dict[str, Any]:
    return {
        "exception": f"{type(exc).__name__}: {exc}",
        "found": False,
        "fail_code": -999,
        "node": -1,
        "effective_max_depth": -1,
        "decisions": 0,
        "splits": 0,
        "seed_in_domain": False,
        "seed_collision": False,
        "hit_unknown_depth_cap": False,
        "hit_reserved_depth_cap": False,
        "deadline_reached": False,
        "tree_seed": [],
        "intervals": [],
        "validation_safety_status": None,
        "validation_collision_possible": None,
        "total_ms": 0.0,
        "external_exact_hits": 0,
        "external_exact_misses": 0,
        "external_reused": 0,
        "canonical_frame_invalid": 0,
        "canonical_reflected_seed_misses": 0,
        "materializations": 0,
    }


def compare_row(seed_row: dict[str, Any], native: dict[str, Any], canonical: dict[str, Any]) -> dict[str, Any]:
    interval_diff = max_interval_abs_diff(native["intervals"], canonical["intervals"])
    tree_seed_dist = point_distance(native["tree_seed"], canonical["tree_seed"]) if native["tree_seed"] and canonical["tree_seed"] else math.inf
    same_class = (
        not native["exception"] and
        not canonical["exception"] and
        native["found"] == canonical["found"] and
        native["fail_code"] == canonical["fail_code"] and
        native["validation_collision_possible"] == canonical["validation_collision_possible"]
    )
    return {
        "label": seed_row["label"],
        "source": seed_row["source"],
        "seed": seed_row["q"],
        "same_classification": same_class,
        "same_found": native["found"] == canonical["found"],
        "same_fail_code": native["fail_code"] == canonical["fail_code"],
        "native_exception": native["exception"],
        "canonical_exception": canonical["exception"],
        "interval_max_abs_diff": interval_diff,
        "tree_seed_distance": tree_seed_dist,
        "native": native,
        "canonical": canonical,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "label",
        "source",
        "same_classification",
        "same_found",
        "same_fail_code",
        "interval_max_abs_diff",
        "native_found",
        "canonical_found",
        "native_fail_code",
        "canonical_fail_code",
        "native_splits",
        "canonical_splits",
        "native_total_ms",
        "canonical_total_ms",
        "canonical_external_exact_hits",
        "canonical_frame_invalid",
        "canonical_reflected_seed_misses",
        "native_exception",
        "canonical_exception",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                "label": row["label"],
                "source": row["source"],
                "same_classification": row["same_classification"],
                "same_found": row["same_found"],
                "same_fail_code": row["same_fail_code"],
                "interval_max_abs_diff": row["interval_max_abs_diff"],
                "native_found": row["native"]["found"],
                "canonical_found": row["canonical"]["found"],
                "native_fail_code": row["native"]["fail_code"],
                "canonical_fail_code": row["canonical"]["fail_code"],
                "native_splits": row["native"]["splits"],
                "canonical_splits": row["canonical"]["splits"],
                "native_total_ms": row["native"]["total_ms"],
                "canonical_total_ms": row["canonical"]["total_ms"],
                "canonical_external_exact_hits": row["canonical"]["external_exact_hits"],
                "canonical_frame_invalid": row["canonical"]["canonical_frame_invalid"],
                "canonical_reflected_seed_misses": row["canonical"]["canonical_reflected_seed_misses"],
                "native_exception": row["native_exception"],
                "canonical_exception": row["canonical_exception"],
            })


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare shelf FFB behavior between native no-cache and canonical d23-cache oracles."
    )
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "outputs" / "ffb_native_vs_canonical")
    parser.add_argument("--cache-root", type=Path, default=D23_CACHE_ROOT)
    parser.add_argument("--cache-label", default=D23_CACHE_LABEL)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--random-count", type=int, default=16)
    parser.add_argument("--random-seed", type=int, default=20260605)
    parser.add_argument("--boundary-eps", type=float, default=1e-6)
    parser.add_argument("--max-depth", type=int, default=DEFAULT_RBF_DEEP_FFB_DEPTH)
    parser.add_argument("--start-depth", type=int, default=DEFAULT_RBF_FFB_START_DEPTH)
    parser.add_argument("--search-mode", default=DEFAULT_RBF_FFB_SEARCH_MODE, choices=["linear", "binary", "binary-depth"])
    parser.add_argument("--threads", type=int, default=DEFAULT_RBF_THREADS)
    parser.add_argument("--random-scope", choices=["full-joint-limits"], default="full-joint-limits")
    parser.add_argument("--reject-seed-collision", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--extra-seed",
        action="append",
        default=[],
        help="Comma-separated native joint seed. Can be supplied multiple times.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    sbf = import_sbf()
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    random_domain = robot_joint_limit_tuples(robot)
    seeds = seed_set(
        robot,
        random_count=int(args.random_count),
        random_seed=int(args.random_seed),
        boundary_eps=float(args.boundary_eps),
        random_domain=random_domain,
    )
    for index, text in enumerate(args.extra_seed or []):
        q = parse_csv_floats(text)
        if len(q) != int(robot.n_joints()):
            raise ValueError(f"--extra-seed #{index} has {len(q)} dims; expected {robot.n_joints()}")
        seeds.append({"label": f"extra{index:02d}", "source": "user_extra", "q": q})

    options = make_find_free_box_options(args)
    native_forest = forest_for_mode("native_no_cache", args, robot)
    canonical_forest = forest_for_mode("canonical_d23_cache", args, robot)

    rows: list[dict[str, Any]] = []
    for seed_row in seeds:
        q = [float(v) for v in seed_row["q"]]
        try:
            native_result = compact_result(native_forest.debug_find_free_box(q, obstacles, options, True))
        except Exception as exc:  # noqa: BLE001 - diagnostics must keep scanning after hard errors.
            native_result = compact_exception(exc)
        try:
            canonical_result = compact_result(canonical_forest.debug_find_free_box(q, obstacles, options, False))
        except Exception as exc:  # noqa: BLE001 - diagnostics must keep scanning after hard errors.
            canonical_result = compact_exception(exc)
        rows.append(compare_row(seed_row, native_result, canonical_result))

    summary = {
        "seed_count": len(rows),
        "same_classification_count": sum(1 for row in rows if row["same_classification"]),
        "same_found_count": sum(1 for row in rows if row["same_found"]),
        "same_fail_code_count": sum(1 for row in rows if row["same_fail_code"]),
        "canonical_external_exact_hits_total": sum(row["canonical"]["external_exact_hits"] for row in rows),
        "canonical_frame_invalid_total": sum(row["canonical"]["canonical_frame_invalid"] for row in rows),
        "canonical_reflected_seed_misses_total": sum(row["canonical"]["canonical_reflected_seed_misses"] for row in rows),
        "native_exception_count": sum(1 for row in rows if row["native_exception"]),
        "canonical_exception_count": sum(1 for row in rows if row["canonical_exception"]),
        "max_interval_abs_diff": max((row["interval_max_abs_diff"] for row in rows), default=0.0),
        "native_root_scope": "full",
        "max_depth": int(args.max_depth),
        "start_depth": int(args.start_depth),
        "search_mode": str(args.search_mode),
    }
    payload = {
        "experiment": "exp04_ffb_native_vs_canonical",
        "description": "FFB-only comparison on shelf native seeds. Planning/query/connector are not run.",
        "config": {
            "native_mode": "database_canonical_mode=false, no external evidence",
            "canonical_mode": "database_canonical_mode=true, d23 external evidence",
            "cache_path": str(Path(args.cache_root) / str(args.cache_label)),
            "seed_space": "native_joint_space",
            "canonical_mapping_scope": "LECT_internal_only",
            "native_root_scope": "full",
            "random_scope": str(args.random_scope),
        },
        "summary": summary,
        "rows": rows,
    }
    json_path = args.out_dir / "ffb_native_vs_canonical.json"
    csv_path = args.out_dir / "ffb_native_vs_canonical.csv"
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_csv(csv_path, rows)
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"wrote {json_path}")
    print(f"wrote {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
