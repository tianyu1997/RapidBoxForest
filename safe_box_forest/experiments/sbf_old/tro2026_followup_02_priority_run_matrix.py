#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PAPER_OUTPUTS = ROOT / "outputs" / "paper"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write a prioritized minimal run matrix for TRO follow-up experiments.")
    parser.add_argument("--out-json", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_priority_run_matrix.json")
    parser.add_argument("--out-md", type=Path, default=PAPER_OUTPUTS / "tro2026_followup_priority_run_matrix.md")
    parser.add_argument("--seed-count", type=int, default=10)
    parser.add_argument("--threads", type=int, default=8)
    return parser.parse_args()


def command(parts: list[str]) -> str:
    return " ".join(parts)


def build_matrix(seed_count: int, threads: int) -> list[dict[str, Any]]:
    return [
        {
            "id": "P0-VALIDATION-CLOSURE",
            "priority": "P0",
            "target": "final-audit accounting, corridor-field availability, failure taxonomy",
            "artifact": "outputs/paper/tro2026_followup_validation_closure.json",
            "command": command([
                "python", "experiments/tro2026_followup_01_validation_closure.py",
                "--outputs", "outputs/paper",
                "--out-json", "outputs/paper/tro2026_followup_validation_closure.json",
                "--out-csv", "outputs/paper/tro2026_followup_validation_closure.csv",
                "--out-md", "outputs/paper/tro2026_followup_validation_closure.md",
            ]),
            "paper_gate": "Do not claim measured corridor-certified coverage unless corridor_known_count is nonzero.",
        },
        {
            "id": "P0-RANDOM-SEED-ROBUSTNESS",
            "priority": "P0",
            "target": "random-scene seed-count robustness under shared workload seeds",
            "artifact": f"outputs/paper/tro2026_followup_random_seed{seed_count}_anytime.json",
            "command": command([
                "python", "experiments/paper_15_random_anytime_tradeoff.py",
                "--scene-seeds", str(seed_count),
                "--threads", str(threads),
                "--methods", "support_hull_coverage",
                "--baseline-methods", "rrt,prm,bitstar",
                "--out-json", f"outputs/paper/tro2026_followup_random_seed{seed_count}_anytime.json",
            ]),
            "paper_gate": "Keep broad random-scene claims narrow unless 10-seed medians and audit SR preserve the reported design point.",
        },
        {
            "id": "P0-RANDOM-IRIS-SEED-ROBUSTNESS",
            "priority": "P0",
            "target": "IRIS-NP+GCS on the same expanded random-scene seed set",
            "artifact": f"outputs/paper/tro2026_followup_random_seed{seed_count}_iris_np_gcs.json",
            "command": command([
                "python", "experiments/paper_16_random_iris_np_gcs_anytime.py",
                "--scene-seeds", str(seed_count),
                "--threads", str(threads),
                "--out-json", f"outputs/paper/tro2026_followup_random_seed{seed_count}_iris_np_gcs.json",
            ]),
            "paper_gate": "IRIS comparison remains pipeline-level; use this only to harden shared-seed statistics.",
        },
        {
            "id": "P1-GROWER-ABLATION-FULL",
            "priority": "P1",
            "target": "reported RBF-SH shelf configuration",
            "artifact": "outputs/paper/tro2026_followup_grower_ablation_full.json",
            "command": command([
                "python", "experiments/paper_04_marcucci_combined.py",
                "--seeds", "5",
                "--threads", str(threads),
                "--preset", "support_hull_coverage",
                "--out-json", "outputs/paper/tro2026_followup_grower_ablation_full.json",
            ]),
            "paper_gate": "Reference row for all ablations.",
        },
        {
            "id": "P1-GROWER-ABLATION-NO-UNEXPLORED",
            "priority": "P1",
            "target": "remove unexplored-volume sampling",
            "artifact": "outputs/paper/tro2026_followup_grower_ablation_no_unexplored.json",
            "command": command([
                "python", "experiments/paper_04_marcucci_combined.py",
                "--seeds", "5",
                "--threads", str(threads),
                "--preset", "support_hull_coverage",
                "--unexplored-prob", "0.0",
                "--out-json", "outputs/paper/tro2026_followup_grower_ablation_no_unexplored.json",
            ]),
            "paper_gate": "Quantifies contribution of unexplored-volume target source.",
        },
        {
            "id": "P1-GROWER-ABLATION-NO-CONNECTOR-TARGETS",
            "priority": "P1",
            "target": "remove component-connector targets",
            "artifact": "outputs/paper/tro2026_followup_grower_ablation_no_connector_targets.json",
            "command": command([
                "python", "experiments/paper_04_marcucci_combined.py",
                "--seeds", "5",
                "--threads", str(threads),
                "--preset", "support_hull_coverage",
                "--component-connect-prob", "0.0",
                "--out-json", "outputs/paper/tro2026_followup_grower_ablation_no_connector_targets.json",
            ]),
            "paper_gate": "Quantifies contribution of explicit component-connection bias.",
        },
        {
            "id": "P1-GROWER-ABLATION-NO-REPAIR",
            "priority": "P1",
            "target": "remove post-audit repair",
            "artifact": "outputs/paper/tro2026_followup_grower_ablation_no_repair.json",
            "command": command([
                "python", "experiments/paper_04_marcucci_combined.py",
                "--seeds", "5",
                "--threads", str(threads),
                "--preset", "support_hull_coverage",
                "--no-repair-on-audit-failure",
                "--out-json", "outputs/paper/tro2026_followup_grower_ablation_no_repair.json",
            ]),
            "paper_gate": "Separates box-forest/corridor behavior from local repair contribution.",
        },
    ]


def write_markdown(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# TRO 2026 Follow-Up Priority Run Matrix",
        "",
        "| ID | Priority | Target | Artifact | Gate |",
        "|---|---|---|---|---|",
    ]
    for row in rows:
        lines.append(f"| {row['id']} | {row['priority']} | {row['target']} | `{row['artifact']}` | {row['paper_gate']} |")
    lines.extend(["", "## Commands", ""])
    for row in rows:
        lines.append(f"### {row['id']}")
        lines.append("")
        lines.append("```powershell")
        lines.append(row["command"])
        lines.append("```")
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    rows = build_matrix(int(args.seed_count), int(args.threads))
    payload = {
        "experiment": "tro2026_followup_02_priority_run_matrix",
        "seed_count": int(args.seed_count),
        "threads": int(args.threads),
        "rows": rows,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    write_markdown(args.out_md, rows)
    print(json.dumps({"out_json": str(args.out_json), "out_md": str(args.out_md), "row_count": len(rows)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())