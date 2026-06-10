#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import csv_list, environment_metadata
from experiments.common.generate_prefix_mapped_workspace_catalog import DIFFICULTY_ORDER
from experiments.common.random_scene_catalog import (
    CATALOG_SCHEMA,
    catalog_record_map,
    load_catalog,
    query_records_from_record,
    record_has_shared_query_median_gate,
    records_have_shared_queries,
    records_have_strict_nested_prefixes,
    scene_cache_key,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Assemble and validate the Exp.6 q10x10 strict prefix catalog.")
    parser.add_argument("--parts-dir", type=Path, default=None)
    parser.add_argument("--catalog", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--report-json", type=Path, default=None)
    parser.add_argument("--report-md", type=Path, default=None)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=10)
    parser.add_argument("--queries-per-scene", type=int, default=10)
    parser.add_argument("--allow-extra-records", action="store_true")
    return parser.parse_args()


def probe_success_fraction(record: dict[str, Any], planner_key: str) -> float:
    probe = record.get("difficulty_probe", {})
    planner = probe.get(planner_key, {}) if isinstance(probe, dict) else {}
    return float(planner.get("success_fraction", 0.0) or 0.0)


def probe_median(record: dict[str, Any], planner_key: str) -> float:
    probe = record.get("difficulty_probe", {})
    planner = probe.get(planner_key, {}) if isinstance(probe, dict) else {}
    return float(planner.get("median_first_success_s", math.nan) or math.nan)


def load_part_records(parts_dir: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    inputs: list[dict[str, Any]] = []
    for path in sorted(parts_dir.glob("*_confirm.json")):
        payload = load_catalog(path)
        part_records = [dict(row) for row in payload.get("records", [])]
        records.extend(part_records)
        inputs.append({"path": str(path), "records": len(part_records)})
    return records, inputs


def comparable_queries(record: dict[str, Any]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for query in query_records_from_record(record):
        out.append(
            {
                "start": [float(value) for value in query["start"]],
                "goal": [float(value) for value in query["goal"]],
                "canonical_start": [float(value) for value in query.get("canonical_start", query["start"])],
                "canonical_goal": [float(value) for value in query.get("canonical_goal", query["goal"])],
            }
        )
    return out


def validate_records(
    records: list[dict[str, Any]],
    *,
    robots: list[str],
    difficulties: list[str],
    scene_seeds: int,
    queries_per_scene: int,
    allow_extra_records: bool,
) -> dict[str, Any]:
    errors: list[str] = []
    record_map = catalog_record_map({"records": records})
    expected_keys = [
        scene_cache_key(robot, difficulty, seed)
        for robot in robots
        for difficulty in difficulties
        for seed in range(int(scene_seeds))
    ]
    if not allow_extra_records and len(records) != len(expected_keys):
        errors.append(f"expected exactly {len(expected_keys)} records, got {len(records)}")
    missing = [key for key in expected_keys if key not in record_map]
    if missing:
        errors.append(f"missing {len(missing)} records; first={missing[0]}")
    extra = [key for key in record_map if key not in set(expected_keys)]
    if extra and not allow_extra_records:
        errors.append(f"found {len(extra)} unexpected records; first={extra[0]}")

    group_rows: list[dict[str, Any]] = []
    for robot in robots:
        for scene_seed in range(int(scene_seeds)):
            group_keys = [scene_cache_key(robot, difficulty, scene_seed) for difficulty in difficulties]
            if any(key not in record_map for key in group_keys):
                continue
            ok, reason = records_have_strict_nested_prefixes(record_map, robot, difficulties, scene_seed)
            if not ok:
                errors.append(reason)
            ok, reason = records_have_shared_queries(record_map, robot, difficulties, scene_seed)
            if not ok:
                errors.append(reason)
            reference_queries = comparable_queries(record_map[group_keys[0]])
            if len(reference_queries) != int(queries_per_scene):
                errors.append(f"{group_keys[0]} has {len(reference_queries)} queries, expected {queries_per_scene}")
            prefix_counts = {
                difficulty: len(record_map[scene_cache_key(robot, difficulty, scene_seed)].get("obstacles", []))
                for difficulty in difficulties
            }
            for difficulty in difficulties:
                key = scene_cache_key(robot, difficulty, scene_seed)
                record = record_map[key]
                if len(query_records_from_record(record)) < int(queries_per_scene):
                    errors.append(f"{key} has too few queries")
                if not record_has_shared_query_median_gate(record):
                    errors.append(f"{key} does not have a strict confirmed difficulty probe")
                for planner_key in ("rrtconnect", "bitstar"):
                    if probe_success_fraction(record, planner_key) < 1.0 - 1e-12:
                        errors.append(f"{key} {planner_key} success_fraction is not 1.0")
            group_rows.append(
                {
                    "robot": robot,
                    "scene_seed": int(scene_seed),
                    "prefix_counts": prefix_counts,
                    "query_count": int(len(reference_queries)),
                }
            )
    summary_rows: list[dict[str, Any]] = []
    for robot in robots:
        for difficulty in difficulties:
            selected = [
                record_map[scene_cache_key(robot, difficulty, seed)]
                for seed in range(int(scene_seeds))
                if scene_cache_key(robot, difficulty, seed) in record_map
            ]
            rrt_values = [probe_median(record, "rrtconnect") for record in selected]
            bit_values = [probe_median(record, "bitstar") for record in selected]
            summary_rows.append(
                {
                    "robot": robot,
                    "difficulty": difficulty,
                    "scene_count": int(len(selected)),
                    "query_count_total": int(sum(len(query_records_from_record(record)) for record in selected)),
                    "prefix_count_mean": (
                        float(statistics.mean(len(record.get("obstacles", [])) for record in selected))
                        if selected
                        else math.nan
                    ),
                    "rrtconnect_median_first_s_mean": (
                        float(statistics.mean(value for value in rrt_values if math.isfinite(value)))
                        if any(math.isfinite(value) for value in rrt_values)
                        else math.nan
                    ),
                    "bitstar_median_first_s_mean": (
                        float(statistics.mean(value for value in bit_values if math.isfinite(value)))
                        if any(math.isfinite(value) for value in bit_values)
                        else math.nan
                    ),
                }
            )
    return {
        "ok": not errors,
        "errors": errors,
        "records": int(len(records)),
        "expected_records": int(len(expected_keys)),
        "robots": robots,
        "difficulties": difficulties,
        "scene_seeds": int(scene_seeds),
        "queries_per_scene": int(queries_per_scene),
        "groups": group_rows,
        "summary": summary_rows,
    }


def write_markdown(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Exp.6 q10x10 Prefix Catalog Validation",
        "",
        f"- ok: `{bool(report['ok'])}`",
        f"- records: `{report['records']}/{report['expected_records']}`",
        f"- scene seeds per robot/difficulty: `{report['scene_seeds']}`",
        f"- queries per scene: `{report['queries_per_scene']}`",
        "",
        "## Summary",
        "",
        "| Robot | Difficulty | Scenes | Queries | Prefix count mean | RRTConnect first-s mean | BIT* first-s mean |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in report["summary"]:
        lines.append(
            f"| {row['robot']} | {row['difficulty']} | {row['scene_count']} | "
            f"{row['query_count_total']} | {row['prefix_count_mean']:.2f} | "
            f"{row['rrtconnect_median_first_s_mean']:.4f} | {row['bitstar_median_first_s_mean']:.4f} |"
        )
    if report["errors"]:
        lines.extend(["", "## Errors", ""])
        lines.extend(f"- {error}" for error in report["errors"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    t0 = time.perf_counter()
    if args.parts_dir is None and args.catalog is None:
        raise SystemExit("provide --parts-dir or --catalog")
    robots = csv_list(args.robots)
    difficulties = csv_list(args.difficulties)
    inputs: list[dict[str, Any]] = []
    if args.parts_dir is not None:
        records, inputs = load_part_records(args.parts_dir)
    else:
        payload = load_catalog(args.catalog)
        records = [dict(row) for row in payload.get("records", [])]
        inputs = [{"path": str(args.catalog), "records": len(records)}]
    report = validate_records(
        records,
        robots=robots,
        difficulties=difficulties,
        scene_seeds=int(args.scene_seeds),
        queries_per_scene=int(args.queries_per_scene),
        allow_extra_records=bool(args.allow_extra_records),
    )
    report["inputs"] = inputs
    report["generation_s"] = time.perf_counter() - t0
    report["environment"] = environment_metadata()
    if args.out is not None:
        if not bool(report["ok"]):
            raise SystemExit("catalog validation failed; refusing to write --out")
        payload = {
            "schema": CATALOG_SCHEMA,
            "scene_profile": "prefix_cspace_mapped_workspace_median_gated_q10x10",
            "robots": robots,
            "difficulties": difficulties,
            "scene_seeds": int(args.scene_seeds),
            "queries_per_scene": int(args.queries_per_scene),
            "records": sorted(
                records,
                key=lambda row: (
                    str(row.get("robot")),
                    int(row.get("scene_seed", -1)),
                    DIFFICULTY_ORDER.index(str(row.get("difficulty")))
                    if str(row.get("difficulty")) in DIFFICULTY_ORDER
                    else 99,
                ),
            ),
            "partial": False,
            "generation_policy": {
                "source": "assembled_from_per_scene_confirm_parts",
                "strict_prefix_nesting": True,
                "shared_queries_across_difficulties": True,
                "queries_per_scene": int(args.queries_per_scene),
                "scene_seeds_per_robot_difficulty": int(args.scene_seeds),
            },
            "validation_report": {
                "ok": bool(report["ok"]),
                "records": int(report["records"]),
                "expected_records": int(report["expected_records"]),
            },
            "environment": environment_metadata(),
            "generation_s": time.perf_counter() - t0,
        }
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    if args.report_json is not None:
        args.report_json.parent.mkdir(parents=True, exist_ok=True)
        args.report_json.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    if args.report_md is not None:
        write_markdown(args.report_md, report)
    print(json.dumps({"ok": report["ok"], "records": report["records"], "errors": report["errors"][:3]}, indent=2))
    return 0 if bool(report["ok"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
