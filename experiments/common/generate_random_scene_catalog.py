#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import csv_list, write_json
from experiments.common.random_scene_catalog import generate_catalog


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate or verify a TRO2026 random scene catalog.")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--scene-seeds", type=int, default=50)
    parser.add_argument(
        "--scene-profile",
        choices=[
            "bitstar_gated", "bitstar_gated_independent",
            "balanced", "balanced_independent", "balanced_probe",
            "timed_probe", "timed_probe_independent",
            "narrow_passage", "narrow_passage_independent",
            "direct_blocker",
        ],
        default="timed_probe_independent",
    )
    parser.add_argument("--max-scene-tries", type=int, default=64)
    parser.add_argument("--queries-per-scene", type=int, default=10)
    parser.add_argument("--seed-base", type=int, default=9176)
    parser.add_argument("--mode", choices=["auto", "generate", "reuse", "verify"], default="auto")
    parser.add_argument("--summary-json", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = generate_catalog(
        path=args.out,
        robots=csv_list(args.robots),
        difficulties=csv_list(args.difficulties),
        scene_seeds=int(args.scene_seeds),
        scene_profile=str(args.scene_profile),
        seed_base=int(args.seed_base),
        queries_per_scene=int(args.queries_per_scene),
        max_scene_tries=int(args.max_scene_tries),
        mode="generate" if str(args.mode) == "generate" else str(args.mode),
    )
    summary = {
        "catalog": str(args.out),
        "schema": payload.get("schema"),
        "records": len(payload.get("records", [])),
        "robots": payload.get("robots"),
        "difficulties": payload.get("difficulties"),
        "scene_seeds": payload.get("scene_seeds"),
        "scene_profile": payload.get("scene_profile"),
        "queries_per_scene": payload.get("queries_per_scene"),
    }
    if args.summary_json is not None:
        write_json(args.summary_json, summary)
    print(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
