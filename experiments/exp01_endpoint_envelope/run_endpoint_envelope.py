#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import os
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


LEGACY_SCRIPT = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "paper_01_epiaabb_pipeline.py"
DEFAULT_SOURCES = "IFK,AAFK,HIFK_3,HIFK_5,CritSample,Analytical,MC"
DEFAULT_WIDTHS = "0.02,0.05,0.10,0.20,0.30,0.50"


def bootstrap_lie_import() -> None:
    os.environ.setdefault("RBF_ENVELOPE_MODULE_DIR", str(REPO_ROOT / "link_interval_envelope"))
    candidates = [
        REPO_ROOT / "build-rbf-only-exec" / "python",
        REPO_ROOT / "build-consolidated-python" / "python",
        REPO_ROOT / "link_interval_envelope" / "build_py310" / "python",
    ]
    for candidate in candidates:
        package_dir = candidate / "link_interval_envelope"
        if not package_dir.exists() or not any(package_dir.glob("_link_interval_envelope_cpp*.so")):
            continue
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)
        importlib.import_module("link_interval_envelope")
        return
    raise ImportError(
        "link_interval_envelope Python extension was not found; "
        "build target link_interval_envelope_python_package first"
    )


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp01_endpoint_envelope"
    parser = argparse.ArgumentParser(description="Run Experiment 1 endpoint-envelope microbenchmark.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--fixed-widths", default=DEFAULT_WIDTHS)
    parser.add_argument("--sources", default=DEFAULT_SOURCES)
    parser.add_argument("--n-boxes", type=int, default=10000)
    parser.add_argument("--base-seed", type=int, default=6100)
    parser.add_argument("--endpoint-threads", type=int, default=0)
    parser.add_argument("--ref-samples", type=int, default=50000)
    parser.add_argument("--min-samples", type=int, default=1000)
    parser.add_argument("--max-samples", type=int, default=10000000)
    parser.add_argument("--analytical-max-phase", type=int, default=3)
    parser.add_argument("--width-sampling", choices=["per-joint", "per-box"], default="per-joint")
    parser.add_argument("--fixed-boxes-json", type=Path, default=None)
    parser.add_argument("--save-fixed-boxes-json", type=Path, default=None)
    parser.add_argument("--smoke", action="store_true", help="Use one width, one box, and cheap sources.")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def patch_aafk_source(module: Any, sources: list[str]) -> None:
    if "AAFK" not in sources:
        return
    module.SOURCE_SPECS["AAFK"] = {"source": "ifk_aa"}
    module.SOURCES = list(dict.fromkeys([*list(module.SOURCES), "AAFK"]))
    original_certified_source = module.certified_source

    def certified_source(source: str) -> bool:
        if source == "AAFK":
            return True
        return bool(original_certified_source(source))

    module.certified_source = certified_source


def legacy_argv(args: argparse.Namespace, out_json: Path, boxes_json: Path, sources: list[str]) -> list[str]:
    command = [
        str(LEGACY_SCRIPT),
        "--out-json",
        str(out_json),
        "--fixed-widths",
        str(args.fixed_widths),
        "--sources",
        ",".join(sources),
        "--n-boxes",
        str(args.n_boxes),
        "--base-seed",
        str(args.base_seed),
        "--endpoint-threads",
        str(args.endpoint_threads),
        "--ref-samples",
        str(args.ref_samples),
        "--min-samples",
        str(args.min_samples),
        "--max-samples",
        str(args.max_samples),
        "--analytical-max-phase",
        str(args.analytical_max_phase),
        "--width-sampling",
        str(args.width_sampling),
        "--save-fixed-boxes-json",
        str(boxes_json),
    ]
    if args.fixed_boxes_json is not None:
        command += ["--fixed-boxes-json", str(args.fixed_boxes_json)]
    return command


def main() -> int:
    args = parse_args()
    out_json = args.out_json or (args.out_dir / "endpoint_envelope.json")
    boxes_json = args.save_fixed_boxes_json or (args.out_dir / "endpoint_envelope_fixed_boxes.json")
    sources = [item.strip() for item in str(args.sources).split(",") if item.strip()]
    if args.smoke:
        args.n_boxes = 1
        args.fixed_widths = "0.02"
        args.ref_samples = min(int(args.ref_samples), 1000)
        args.max_samples = min(int(args.max_samples), 1000)
        sources = [source for source in ["IFK", "AAFK", "CritSample", "MC"] if source in set(sources)]
    manifest = {
        "experiment": "exp01_endpoint_envelope",
        "run_id": run_id("exp01"),
        "status": "dry_run" if args.dry_run else "running",
        "params": namespace_dict(args),
        "sources": sources,
        "artifacts": {"out_json": str(out_json), "box_table_json": str(boxes_json)},
        "legacy_script": str(LEGACY_SCRIPT),
        "legacy_argv": legacy_argv(args, out_json, boxes_json, sources),
        "environment": environment_metadata(),
    }
    write_json(args.out_dir / "run_manifest.json", manifest)
    if args.dry_run:
        print(f"wrote dry-run manifest: {args.out_dir / 'run_manifest.json'}")
        return 0
    bootstrap_lie_import()
    module = load_module_from_path("exp01_legacy_epiaabb", LEGACY_SCRIPT)
    patch_aafk_source(module, sources)
    old_argv = sys.argv
    sys.argv = legacy_argv(args, out_json, boxes_json, sources)
    try:
        return int(module.main())
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    raise SystemExit(main())
