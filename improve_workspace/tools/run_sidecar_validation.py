#!/usr/bin/env python3
"""Run sidecar validation for the C-LECT improve.md implementation."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def run(cmd: list[str], cwd: Path) -> dict[str, object]:
    proc = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, check=False)
    return {
        "cmd": cmd,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", default="improve_workspace/sidecar_validation.json")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[2]
    commands = [
        [sys.executable, "improve_workspace/tools/check_improve_plan.py", "--json"],
        [sys.executable, "-m", "unittest", "discover", "-s", "improve_workspace/tests", "-v"],
        [sys.executable, "improve_workspace/tools/synthetic_clect_benchmark.py", "--json"],
        [sys.executable, "improve_workspace/tools/run_clect_ablation_benchmark.py"],
        [sys.executable, "improve_workspace/tools/run_clect_experiment_suite.py"],
        [sys.executable, "improve_workspace/tools/run_clect_scaling_experiment.py"],
        [sys.executable, "improve_workspace/tools/plot_clect_experiments.py"],
        [sys.executable, "improve_workspace/tools/run_hipac_validation.py"],
        [sys.executable, "improve_workspace/tools/run_production_experiment_bridge.py"],
        [
            sys.executable,
            "improve_workspace/tools/run_production_experiment_bridge.py",
            "--mode",
            "executed-smoke",
            "--json-out",
            "improve_workspace/production_experiment_bridge_executed.json",
            "--md-out",
            "improve_workspace/production_experiment_bridge_executed.md",
        ],
        [sys.executable, "improve_workspace/tools/run_production_clect_ablation.py"],
        [sys.executable, "improve_workspace/tools/summarize_improve_performance.py"],
        [sys.executable, "improve_workspace/tools/audit_production_integration_readiness.py"],
        [sys.executable, "improve_workspace/tools/audit_production_completion_status.py"],
        [sys.executable, "improve_workspace/tools/audit_improve_plan_sections.py"],
        [sys.executable, "improve_workspace/tools/audit_improve_requirements.py"],
    ]
    results = [run(cmd, repo) for cmd in commands]
    ok = all(item["returncode"] == 0 for item in results)
    payload = {"ok": ok, "results": results}
    out_path = repo / args.json_out
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"ok": ok, "json_out": str(out_path)}, indent=2, ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
