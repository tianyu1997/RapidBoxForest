#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PAPER_OUTPUTS = ROOT / "outputs" / "paper"
DEFAULT_ARTIFACT = PAPER_OUTPUTS / "tro2026_random_anytime_tradeoff_full_unbiased_strictaudit_rrtgrid_sbfopt_20260512.json"
DEFAULT_STEPS = [0.02, 0.01, 0.005, 0.002]

from RapidBoxForest.safe_box_forest.experiments.sbf_old.audit_random_anytime_artifact import audit_artifact  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep strict audit segment steps on stored random anytime paths.")
    parser.add_argument("--artifact", type=Path, default=DEFAULT_ARTIFACT)
    parser.add_argument("--steps", nargs="*", type=float, default=DEFAULT_STEPS)
    parser.add_argument("--out-json", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_audit_resolution_sweep.json")
    parser.add_argument("--out-md", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_audit_resolution_sweep.md")
    return parser.parse_args()


def write_markdown(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Mandatory Audit-Resolution Sweep",
        "",
        f"Artifact: `{payload['artifact']}`",
        "",
        "This report re-audits stored counted random-scene anytime paths at multiple joint-space segment steps.",
        "",
        "| Segment step | Counted success paths | Invalid records | Unique invalid paths | Unique scene/method/tasks | Invalid record rate |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for report in payload["reports"]:
        counted = int(report.get("counted_success_paths", 0))
        invalid = int(report.get("invalid_count", 0))
        unique_paths = int(report.get("unique_invalid_path_count", 0))
        unique_tasks = int(report.get("unique_invalid_scene_method_task_count", 0))
        rate = float(invalid) / float(counted) if counted else 0.0
        lines.append(f"| {float(report['audit_segment_step']):.4f} | {counted} | {invalid} | {unique_paths} | {unique_tasks} | {rate:.4f} |")
    examples = [
        example
        for report in payload["reports"]
        for example in report.get("invalid_examples", [])
    ]
    lines.extend(["", "## Invalid Examples", ""])
    if not examples:
        lines.append("No invalid examples were found at the tested segment steps.")
    else:
        lines.extend([
            "| Step | Method | Stage | Robot | Difficulty | Scene seed | Path/direct |",
            "|---:|---|---|---|---|---:|---:|",
        ])
        for report in payload["reports"]:
            step = float(report["audit_segment_step"])
            for example in report.get("invalid_examples", [])[:20]:
                lines.append(
                    f"| {step:.4f} | {example.get('method')} | {example.get('stage_id')} | {example.get('robot')} | {example.get('difficulty')} | {example.get('scene_seed')} | {float(example.get('path_over_direct', 0.0)):.3f} |"
                )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    reports = [audit_artifact(args.artifact, float(step)) for step in args.steps]
    payload = {
        "experiment": "tro2026_followup_04_audit_resolution_sweep",
        "artifact": str(args.artifact),
        "reports": reports,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_markdown(args.out_md, payload)
    print(json.dumps({
        "out_json": str(args.out_json),
        "out_md": str(args.out_md),
        "summary": [
            {
                "step": report["audit_segment_step"],
                "counted_success_paths": report["counted_success_paths"],
                "invalid_count": report["invalid_count"],
                "unique_invalid_path_count": report.get("unique_invalid_path_count", 0),
                "unique_invalid_scene_method_task_count": report.get("unique_invalid_scene_method_task_count", 0),
            }
            for report in reports
        ],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())