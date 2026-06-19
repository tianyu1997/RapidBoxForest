from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from experiments.common.metrics import mean, median


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_IRIS_PYTHON_ENV = "RBF_IRIS_PYTHON"
DEFAULT_GCS_REPO_ENV = "RBF_GCS_REPO"
SHELF_IRIS_SCRIPT_ENV = "RBF_SHELF_IRIS_SCRIPT"


def shelf_iris_script() -> Path:
    configured = os.environ.get(SHELF_IRIS_SCRIPT_ENV)
    if configured:
        return Path(configured)
    return REPO_ROOT / "external" / "old_tro2026" / "paper_16_shelf_iris_np_gcs_anytime.py"


def default_iris_python() -> Path:
    configured = os.environ.get(DEFAULT_IRIS_PYTHON_ENV)
    if configured:
        path = Path(configured)
        if path.exists():
            return path
    return Path(sys.executable)


def default_gcs_repo() -> Path:
    configured = os.environ.get(DEFAULT_GCS_REPO_ENV)
    if configured:
        return Path(configured)
    return REPO_ROOT / "gcs-science-robotics"


def iris_env() -> dict[str, str]:
    env = dict(os.environ)
    candidates = [
        REPO_ROOT / "build" / "python",
        REPO_ROOT / "build-leaf-sweep" / "python",
        REPO_ROOT / "build-exp04" / "python",
        REPO_ROOT.parent,
        REPO_ROOT,
    ]
    existing = env.get("PYTHONPATH", "")
    entries = [str(path) for path in candidates if path.exists()]
    if existing:
        entries.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(entries)
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[name] = env.get(name, "8")
    return env


def check_iris_dependencies(python_executable: Path, gcs_repo: Path) -> dict[str, Any]:
    missing: list[str] = []
    if not Path(python_executable).exists():
        missing.append(str(python_executable))
    if not Path(gcs_repo).is_dir():
        missing.append(str(gcs_repo))
    if missing:
        return {"ok": False, "missing": missing}
    probe = (
        "import sys; "
        f"sys.path.insert(0, {str(gcs_repo)!r}); "
        "import numpy, pydrake, sbf; from gcs.bezier import BezierGCS; "
        "print('ok')"
    )
    result = subprocess.run(
        [str(python_executable), "-c", probe],
        cwd=str(REPO_ROOT),
        env=iris_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        return {
            "ok": False,
            "missing": ["iris_import_probe_failed"],
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    return {"ok": True, "python": str(python_executable), "gcs_repo": str(gcs_repo)}


def run_shelf_iris_anytime(
    *,
    out_json: Path,
    python_executable: Path,
    gcs_repo: Path,
    seeds: int,
    threads: int,
    budget_s: float,
    stage_region_counts: str,
    iteration_limit: int,
    query_time_limit_s: float,
    rounding_max_paths: int,
    rounding_max_trials: int,
    segment_step: float,
    final_ompl_simplify_time_s: float,
) -> dict[str, Any]:
    deps = check_iris_dependencies(python_executable, gcs_repo)
    if not deps.get("ok"):
        raise RuntimeError(f"IRIS/GCS dependency check failed: {deps}")
    script = shelf_iris_script()
    if not script.exists():
        raise RuntimeError(
            "IRIS/GCS shelf runner is not bundled in the clean public release. "
            f"Set {SHELF_IRIS_SCRIPT_ENV} to an external runner path or import the "
            "published baseline artifact instead."
        )
    out_json.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(python_executable),
        str(script),
        "--seeds",
        str(max(1, int(seeds))),
        "--logical-threads",
        str(max(1, int(threads))),
        "--budget-s",
        str(float(budget_s)),
        "--stage-region-counts",
        str(stage_region_counts),
        "--iteration-limit",
        str(int(iteration_limit)),
        "--query-time-limit-s",
        str(float(query_time_limit_s)),
        "--rounding-max-paths",
        str(int(rounding_max_paths)),
        "--rounding-max-trials",
        str(int(rounding_max_trials)),
        "--gcs-repo",
        str(gcs_repo),
        "--segment-step",
        str(float(segment_step)),
        "--final-ompl-simplify-time-s",
        str(float(final_ompl_simplify_time_s)),
        "--out-json",
        str(out_json),
    ]
    result = subprocess.run(
        command,
        cwd=str(REPO_ROOT),
        env=iris_env(),
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"IRIS/GCS command failed with code {result.returncode}: {' '.join(command)}")
    return json.loads(out_json.read_text(encoding="utf-8"))


def shelf_iris_json_to_run_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for record in payload.get("records", []):
        tasks = list(record.get("raw_tasks", []))
        queries = []
        success_count = 0
        for task in tasks:
            raw = task.get("raw", {}) if isinstance(task.get("raw"), dict) else {}
            path_length = task.get("path_length")
            if path_length is None and bool(raw.get("success")):
                path_length = raw.get("path_length")
            ok = bool(task.get("audit_passed")) and path_length is not None
            if ok:
                success_count += 1
            queries.append({
                "label": str(task.get("name", "")),
                "success": ok,
                "audit_passed": ok,
                "audit_status": "passed" if ok else str(task.get("reason", "failed")),
                "path_length": path_length,
                "query_ms": 1000.0 * float(task.get("query_s", 0.0) or 0.0),
                "audit_ms": 0.0,
                "waypoint_count": int(task.get("waypoint_count", 0) or 0),
                "planner_status": "ok" if ok else str(task.get("reason", "failed")),
                "segment_fraction": 0.0,
            })
        success_lengths = [float(q["path_length"]) for q in queries if q.get("path_length") is not None and bool(q.get("success"))]
        query_count = int(record.get("task_count", len(tasks)) or len(tasks))
        rows.append({
            "method": "iris_np_gcs",
            "seed": int(record.get("seed_index", 0) or 0),
            "stage_id": str(record.get("stage_id", "iris_np_gcs")),
            "budget_s": float(record.get("cumulative_total_s", math.nan) or math.nan),
            "status": "executed",
            "success_count": success_count,
            "query_count": query_count,
            "build_s": float(record.get("cumulative_build_s", math.nan) or math.nan),
            "planning_s": float(record.get("cumulative_total_s", math.nan) or math.nan),
            "audit_s": 0.0,
            "path_length_mean": mean(success_lengths),
            "raw_segment_fraction": 0.0,
            "queries": queries,
            "diagnostics": {
                "protocol": record.get("protocol"),
                "params": record.get("params", {}),
                "stage_query_s": record.get("cumulative_query_s"),
                "source": "current_iris_gcs_dispatch",
            },
        })
    return rows


def shelf_iris_summary_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    stage_ids = sorted({str(row.get("stage_id", "")) for row in rows})
    for stage_id in stage_ids:
        items = [row for row in rows if str(row.get("stage_id", "")) == stage_id]
        success_items = [row for row in items if int(row.get("success_count", 0)) == int(row.get("query_count", 0))]
        out.append({
            "method": "iris_np_gcs",
            "method_label": "IRIS-NP+GCS",
            "stage_id": stage_id,
            "budget_s": median(row.get("budget_s", math.nan) for row in items),
            "deep_max_boxes": 0,
            "runs": len(items),
            "success_runs": len(success_items),
            "source": "current_execution",
            "build_s": median(row.get("build_s", math.nan) for row in items),
            "query_s_median": median(
                median(q.get("query_ms", math.nan) / 1000.0 for q in row.get("queries", []))
                for row in items
            ),
            "planning_s_median": median(row.get("planning_s", math.nan) for row in items),
            "audit_s_median": median(row.get("audit_s", math.nan) for row in items),
            "path_length_mean": mean(row.get("path_length_mean") for row in success_items),
            "raw_segment_fraction_median": median(row.get("raw_segment_fraction", math.nan) for row in success_items),
            "status": "executed",
        })
    return out
