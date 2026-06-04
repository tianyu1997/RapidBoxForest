#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
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
LEGACY_TABLE_SCRIPT = REPO_ROOT / "safe_box_forest" / "experiments" / "sbf_old" / "tro2026_generate_tables.py"
FIXED_SPLIT = 1
DEFAULT_VARIANTS = "link_s1,support_hull_plain_s1"
DEFAULT_CHAINS = "chain_aabb_support_hull_s1"
FIXED_ENDPOINT_SOURCE = "ifk_aa"
FIXED_ENDPOINT_LABEL = "IFK_AA"
DEFAULT_OUT_BASENAME = "link_envelope_table_iv_ifk_aa_s1.json"
DEFAULT_TABLE_BASENAME = "tab_tro_link_envelope_widthwise_ifk_s1.tex"
TABLE_VARIANT_ORDER = [
    "link_s1",
    "support_hull_plain_s1",
    "chain_aabb_support_hull_s1",
]
PAPER_TABLE_PATHS = [
    REPO_ROOT / "paper" / "sbf_old" / "generated" / "tab_tro_link_envelope_widthwise_ifk_s1.tex",
]


def default_boxes_json() -> Path:
    candidates = [
        DEFAULT_OUTPUT_ROOT / "exp01_endpoint_envelope" / "endpoint_envelope_fixed_boxes.json",
        DEFAULT_OUTPUT_ROOT / "exp01_endpoint_envelope_n400" / "endpoint_envelope_fixed_boxes.json",
        DEFAULT_OUTPUT_ROOT / "exp01_endpoint_envelope_n1000" / "endpoint_envelope_fixed_boxes.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def variant_display_label(variant_key: str) -> str:
    key = str(variant_key).strip()
    if key == "ifk_link_s4" or key.startswith("link_s"):
        return "AABB"
    if key.startswith("support_hull_plain_s"):
        return "SH"
    if key.startswith("chain_aabb_support_hull_s"):
        return "AABB->SH chain"
    return key


def variant_summary_label(variant: dict[str, Any]) -> str:
    return f"{FIXED_ENDPOINT_LABEL}+{variant_display_label(str(variant.get('key', '')))} S={int(variant.get('n_subdivisions', 1))}"


def parse_args() -> argparse.Namespace:
    output_dir = DEFAULT_OUTPUT_ROOT / "exp02_link_envelope_s1"
    default_boxes = default_boxes_json()
    parser = argparse.ArgumentParser(description="Run Experiment 2 link-envelope widthwise microbenchmark with fixed IFK_AA endpoints, S=1, and AABB/SH/AABB->SH chain collision options.")
    parser.add_argument("--out-dir", type=Path, default=output_dir)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--boxes-json", type=Path, default=default_boxes)
    parser.add_argument("--variants", default=DEFAULT_VARIANTS)
    parser.add_argument("--chain-variants", default=DEFAULT_CHAINS)
    parser.add_argument("--endpoint-threads", type=int, default=0)
    parser.add_argument("--batch-threads", type=int, default=1)
    parser.add_argument("--parallel-min-combos", type=int, default=0)
    parser.add_argument("--max-boxes-per-width", type=int, default=None)
    parser.add_argument("--include-collision-benchmark", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def patch_s1_chain_variants(module: Any) -> None:
    original = module.parse_chain_variant

    def parse_chain_variant(text: str) -> dict[str, Any]:
        key = text.strip()
        specs: dict[str, tuple[str, list[str]]] = {}
        for split in (1, 4):
            specs[f"chain_aabb_support_hull_s{split}"] = (
                f"AABB->SH chain S={split}",
                [f"link_s{split}", f"support_hull_plain_s{split}"],
            )
        if key in specs:
            label, stages = specs[key]
            return {"key": key, "label": label, "stages": stages}
        return original(text)

    module.parse_chain_variant = parse_chain_variant


def patch_fixed_endpoint_variants(module: Any) -> None:
    original = module.parse_variant

    def parse_variant(text: str) -> dict[str, Any]:
        variant = dict(original(text))
        variant["endpoint_source"] = FIXED_ENDPOINT_SOURCE
        variant["label"] = variant_summary_label(variant)
        return variant

    module.parse_variant = parse_variant


def patch_chain_summary_metadata(module: Any) -> None:
    original = module.summarize_chain_results

    def summarize_chain_results(*args: Any, **kwargs: Any) -> dict[str, Any]:
        row = dict(original(*args, **kwargs))
        row["endpoint_source"] = FIXED_ENDPOINT_SOURCE
        row["n_subdivisions"] = FIXED_SPLIT
        return row

    module.summarize_chain_results = summarize_chain_results


def patch_robot_json_path(module: Any) -> None:
    candidates = [
        REPO_ROOT / "link_interval_envelope" / "examples" / "data" / "iiwa14.json",
        REPO_ROOT / "safe_box_forest" / "python" / "sbf" / "data" / "iiwa14.json",
    ]
    robot_json = next((candidate for candidate in candidates if candidate.exists()), None)
    if robot_json is None:
        raise FileNotFoundError("iiwa14.json was not found in workspace data paths")
    module.robot_json_path = lambda: robot_json


def prepare_python_imports() -> None:
    paths = [
        REPO_ROOT / "build-rbf-only-exec" / "python",
        REPO_ROOT / "link_interval_envelope" / "python",
        REPO_ROOT / "safe_box_forest" / "python",
    ]
    for path in reversed(paths):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        if path.exists():
            sys.path.insert(0, text)
    sys.modules.pop("link_interval_envelope", None)
    sys.modules.pop("sbf", None)


def write_widthwise_table(out_json: Path, table_tex: Path) -> str:
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    table_module = load_module_from_path("exp02_legacy_tables_table_iv", LEGACY_TABLE_SCRIPT)
    grouped: dict[tuple[float, str], dict[str, Any]] = {}
    for row in payload.get("rows_by_width", []):
        variant = str(row.get("variant", "")).strip()
        if variant not in TABLE_VARIANT_ORDER:
            continue
        if str(row.get("endpoint_source", "")).strip().lower() != FIXED_ENDPOINT_SOURCE:
            continue
        try:
            width = float(row.get("fixed_width"))
        except (TypeError, ValueError):
            continue
        grouped[(width, variant)] = row

    width_order = sorted({width for width, _ in grouped})
    body: list[str] = []
    for width in width_order:
        for variant in TABLE_VARIANT_ORDER:
            row = grouped.get((width, variant))
            if row is None:
                continue
            body.append(" & ".join([
                table_module.tex_escape(table_module.fmt_fixed(width, 2)),
                table_module.tex_escape(variant_display_label(variant)),
                table_module.tex_escape(table_module.fmt(row.get("volume_mean"), 3)),
                table_module.tex_escape(table_module.fmt_fixed(row.get("envelope_us_mean"), 1)),
                table_module.tex_escape(table_module.fmt_fixed(row.get("collision_us_mean"), 1)),
            ]) + r" \\")
        if width != width_order[-1]:
            body.append(r"\addlinespace")

    if not body:
        body.append(r"\multicolumn{5}{c}{No result artifact available.} \\")

    content = "\n".join([
        "% Auto-generated from the width-wise IFK_AA link-envelope representation artifact.",
        r"\begin{table}[!t]",
        r"\centering",
        r"\caption{Width-wise IFK\_AA link-envelope comparison.}",
        r"\label{tab:tro_link_envelopes_ifk_s1}",
        r"\label{tab:tro_link_envelope_widthwise_ifk_s1}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{2.2pt}",
        r"\renewcommand{\arraystretch}{0.82}",
        r"\begin{tabular}{@{}llrrr@{}}",
        r"\toprule",
        r"Width & Envelope & $V$ (m$^3$) & Build ($\mu$s) & Collision ($\mu$s) \\",
        r"\midrule",
        *body,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        "",
    ])
    table_tex.parent.mkdir(parents=True, exist_ok=True)
    table_tex.write_text(content, encoding="utf-8")
    return content


def normalize_official_artifact(out_json: Path) -> None:
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    active_variants = set(TABLE_VARIANT_ORDER)
    payload["rows"] = [
        row for row in payload.get("rows", [])
        if str(row.get("variant", "")).strip() in active_variants
    ]
    payload["rows_by_width"] = [
        row for row in payload.get("rows_by_width", [])
        if str(row.get("variant", "")).strip() in active_variants
    ]
    payload["collision_mode_policy"] = "AABB=aabb_only, SH=support_hull_only, AABB->SH chain=stagewise early-out over AABB then SH"
    payload["d32_disk_model"] = (
        "compact payload estimate after short-link pruning: "
        "AABB=6 floats per retained sub-box; SH=13 support floats per retained sub-box; "
        "AABB->SH chain uses the cumulative staged payload bytes actually touched"
    )
    payload["envelope_collision_options"] = ["AABB", "SH", "AABB->SH chain"]
    payload["support_hull_volume_policy"] = (
        "Pure SH volume is computed directly from each uninflated "
        "short-link endpoint-AABB convex hull, and link-radius expansion is "
        "applied only by the collision test."
    )
    payload["volume_model"] = (
        "All mainline representation volumes are uninflated record-wise envelope "
        "volumes under the same short-link split. AABB sums each retained sub-AABB "
        "volume; SH sums exact Conv(proximal endpoint AABB union distal endpoint AABB) "
        "volumes per short-link record and ignores the stored link radius; AABB->SH "
        "chain reports the terminal stage volume reached by the staged early-out policy."
    )
    payload["mainline_representation_variants"] = list(TABLE_VARIANT_ORDER)
    out_json.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def update_paper_tables(content: str) -> None:
    for path in PAPER_TABLE_PATHS:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


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
    out_json = args.out_json or (args.out_dir / DEFAULT_OUT_BASENAME)
    table_tex = args.out_dir / DEFAULT_TABLE_BASENAME
    update_paper = True
    if args.smoke:
        args.max_boxes_per_width = 1
        args.variants = DEFAULT_VARIANTS
        args.chain_variants = DEFAULT_CHAINS
        out_json = args.out_dir / "link_envelope_table_iv_ifk_aa_s1_smoke.json"
        table_tex = args.out_dir / "tab_tro_link_envelope_widthwise_ifk_s1_smoke.tex"
        update_paper = False
    manifest = {
        "experiment": "exp02_link_envelope_s1",
        "run_id": run_id("exp02"),
        "status": "dry_run" if args.dry_run else "running",
        "params": namespace_dict(args),
        "artifacts": {
            "out_json": str(out_json),
            "box_table_json": str(args.boxes_json),
            "widthwise_table_tex": str(table_tex),
        },
        "legacy_script": str(LEGACY_SCRIPT),
        "legacy_argv": legacy_argv(args, out_json),
        "endpoint_source_fixed": FIXED_ENDPOINT_SOURCE,
        "s_fixed": FIXED_SPLIT,
        "environment": environment_metadata(),
    }
    write_json(args.out_dir / "run_manifest.json", manifest)
    if args.dry_run:
        print(f"wrote dry-run manifest: {args.out_dir / 'run_manifest.json'}")
        return 0
    if not args.boxes_json.exists():
        raise FileNotFoundError(f"box table not found: {args.boxes_json}; run exp01 first or pass --boxes-json")
    prepare_python_imports()
    module = load_module_from_path("exp02_legacy_link", LEGACY_SCRIPT)
    patch_fixed_endpoint_variants(module)
    patch_s1_chain_variants(module)
    patch_chain_summary_metadata(module)
    patch_robot_json_path(module)
    old_argv = sys.argv
    sys.argv = legacy_argv(args, out_json)
    try:
        status = int(module.main())
    finally:
        sys.argv = old_argv
    if status == 0:
        if not out_json.exists():
            raise FileNotFoundError(f"expected experiment artifact was not written: {out_json}")
        normalize_official_artifact(out_json)
        table_content = write_widthwise_table(out_json, table_tex)
        if update_paper:
            update_paper_tables(table_content)
        print(f"wrote table: {table_tex}")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
