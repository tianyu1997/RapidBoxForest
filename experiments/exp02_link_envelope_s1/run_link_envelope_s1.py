#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    environment_metadata,
    load_module_from_path,
    namespace_dict,
    run_id,
    write_json,
)


LEGACY_SCRIPT = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_02_link_envelope_pipeline.py"
DEFAULT_VARIANTS = "link_s1,kdop26_s1,support_hull_nokdop_s1"
DEFAULT_CHAINS = "chain_aabb_kdop26_s1,chain_aabb_support_hull_s1,chain_aabb_kdop26_support_hull_s1"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp02_link_envelope_s1"
    default_boxes = DEFAULT_OUTPUT_ROOT / "exp01_endpoint_envelope" / "endpoint_envelope_fixed_boxes.json"
    parser = argparse.ArgumentParser(description="Run Experiment 2 link-envelope S=1 microbenchmark.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--boxes-json", type=Path, default=default_boxes)
    parser.add_argument("--variants", default=DEFAULT_VARIANTS)
    parser.add_argument("--chain-variants", default=DEFAULT_CHAINS)
    parser.add_argument("--endpoint-threads", type=int, default=0)
    parser.add_argument("--batch-threads", type=int, default=1)
    parser.add_argument("--parallel-min-combos", type=int, default=0)
    parser.add_argument("--max-boxes-per-width", type=int, default=None)
    parser.add_argument("--include-collision-benchmark", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def patch_s1_chain_variants(module: Any) -> None:
    original = module.parse_chain_variant

    def parse_chain_variant(text: str) -> dict[str, Any]:
        key = text.strip()
        specs: dict[str, tuple[str, list[str]]] = {}
        for split in (1, 4):
            specs[f"chain_aabb_kdop26_s{split}"] = (f"AABB->KDOP26 S={split}", [f"link_s{split}", f"kdop26_s{split}"])
            specs[f"chain_aabb_support_hull_s{split}"] = (
                f"AABB->SupportHull S={split}",
                [f"link_s{split}", f"support_hull_nokdop_s{split}"],
            )
            specs[f"chain_aabb_kdop26_support_hull_s{split}"] = (
                f"AABB->KDOP26->SupportHull S={split}",
                [f"link_s{split}", f"kdop26_s{split}", f"support_hull_nokdop_s{split}"],
            )
        if key in specs:
            label, stages = specs[key]
            return {"key": key, "label": label, "stages": stages}
        return original(text)

    module.parse_chain_variant = parse_chain_variant


def legacy_argv(args: argparse.Namespace, out_json: Path) -> list[str]:
    command = [
        str(LEGACY_SCRIPT),
        "--boxes-json",
        str(args.boxes_json),
        "--out-json",
        str(out_json),
        "--variants",
        str(args.variants),
        "--chain-variants",
        str(args.chain_variants),
        "--endpoint-threads",
        str(args.endpoint_threads),
        "--batch-threads",
        str(args.batch_threads),
        "--parallel-min-combos",
        str(args.parallel_min_combos),
    ]
    if args.max_boxes_per_width is not None:
        command += ["--max-boxes-per-width", str(args.max_boxes_per_width)]
    command.append("--include-collision-benchmark" if args.include_collision_benchmark else "--no-include-collision-benchmark")
    return command


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "link_envelope_s1.json")
    if args.smoke:
        args.max_boxes_per_width = 1
        args.variants = "link_s1,kdop26_s1,support_hull_nokdop_s1"
        args.chain_variants = "chain_aabb_kdop26_s1"
    manifest = {
        "experiment": "exp02_link_envelope_s1",
        "run_id": run_id("exp02"),
        "status": "dry_run" if args.dry_run else "running",
        "params": namespace_dict(args),
        "artifacts": {"out_json": str(out_json), "box_table_json": str(args.boxes_json)},
        "legacy_script": str(LEGACY_SCRIPT),
        "legacy_argv": legacy_argv(args, out_json),
        "s_fixed": 1,
        "environment": environment_metadata(),
    }
    write_json(args.out_dir / "run_manifest.json", manifest)
    if args.dry_run:
        print(f"wrote dry-run manifest: {args.out_dir / 'run_manifest.json'}")
        return 0
    if not args.boxes_json.exists():
        raise FileNotFoundError(f"box table not found: {args.boxes_json}; run exp01 first or pass --boxes-json")
    module = load_module_from_path("exp02_legacy_link", LEGACY_SCRIPT)
    patch_s1_chain_variants(module)
    old_argv = sys.argv
    sys.argv = legacy_argv(args, out_json)
    try:
        return int(module.main())
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
