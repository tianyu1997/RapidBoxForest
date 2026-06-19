from __future__ import annotations

import math
from typing import Any

from experiments.common.query_summary import query_success_summary
from experiments.common.query_timing import online_timing_from_query_rows


def fully_successful_run_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Return run rows whose all registered queries succeeded."""

    return [
        row for row in rows
        if int(row.get("success_count", 0)) == int(row.get("query_count", 1))
    ]


def run_success_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize run-level and query-level success counts."""

    success_rows = fully_successful_run_rows(rows)
    return {
        "runs": len(rows),
        "success_runs": len(success_rows),
        "success_queries": sum(int(row.get("success_count", 0)) for row in rows),
        "total_queries": sum(int(row.get("query_count", 0)) for row in rows),
        "success_rows": success_rows,
    }


def diagnostics_timeout_s(row: dict[str, Any]) -> float:
    """Read a planner timeout cap from a row diagnostics payload."""

    diagnostics = row.get("diagnostics", {})
    if isinstance(diagnostics, dict):
        try:
            value = float(diagnostics.get("timeout_s", math.nan))
        except (TypeError, ValueError):
            value = math.nan
        if math.isfinite(value):
            return value
    return math.nan


def summarize_query_batch_run(
    method: str,
    qrows: list[dict[str, Any]],
    *,
    build_s: float = 0.0,
    online_batch_s: float | None = None,
    planning_s: float | None = None,
    audit_s: float = 0.0,
    diagnostics: dict[str, Any] | None = None,
    stage_id: str | None = None,
    budget_s: float | None = None,
    raw_segment_fraction: float = 0.0,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the common run-row payload for independently timed query rows."""

    build_value = float(build_s)
    if online_batch_s is None:
        if planning_s is not None:
            online_value = max(0.0, float(planning_s) - build_value)
        else:
            online_value = sum(float(row.get("query_ms", 0.0)) for row in qrows) / 1000.0
    else:
        online_value = float(online_batch_s)
    timing = online_timing_from_query_rows(
        qrows,
        online_total_s=online_value,
        build_s=build_value,
    )
    row = {
        "method": method,
        "stage_id": stage_id or method,
        "budget_s": float(budget_s) if budget_s is not None else math.nan,
        **query_success_summary(qrows),
        "planning_s": build_value + timing["online_batch_s"],
        "planning_total_s": build_value + timing["online_total_s"],
        "build_s": build_value,
        "offline_build_s": build_value,
        **timing,
        "audit_s": float(audit_s),
        "raw_segment_fraction": float(raw_segment_fraction),
        "queries": qrows,
        "diagnostics": dict(diagnostics or {}),
    }
    row.update(dict(extra or {}))
    return row


def external_pending_run_row(
    method: str,
    seed: int,
    reason: str,
    *,
    stage_id: str | None = None,
    query_count: int = 0,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Create a run-row placeholder for externally produced baselines."""

    row = {
        "method": method,
        "seed": int(seed),
        "stage_id": stage_id or method,
        "budget_s": math.nan,
        "status": "external_pending",
        "success_count": 0,
        "query_count": int(query_count),
        "planning_s": math.nan,
        "audit_s": math.nan,
        "path_length_mean": math.nan,
        "raw_segment_fraction": math.nan,
        "queries": [],
        "diagnostics": {"reason": reason},
    }
    row.update(dict(extra or {}))
    return row


def empty_pending_summary_row(
    method: str,
    stage_id: str,
    *,
    pending_count: int = 0,
    deep_max_boxes: Any = "",
    method_label: str | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Create a summary placeholder when no executable rows are available."""

    row = {
        "method": method,
        "stage_id": stage_id,
        "budget_s": None,
        "timeout_cap_s": None,
        "deep_max_boxes": deep_max_boxes,
        "runs": int(pending_count),
        "success_runs": 0,
        "success_queries": 0,
        "total_queries": 0,
        "offline_build_s_median": None,
        "online_batch_s_median": None,
        "online_solve_s_median": None,
        "online_simplify_s_median": None,
        "online_solve_per_query_s_median": None,
        "online_simplify_per_query_s_median": None,
        "online_per_query_s_median": None,
        "amortized_s_k1": None,
        "amortized_s_k5": None,
        "amortized_s_k10": None,
        "amortized_s_k20": None,
        "amortized_s_k50": None,
        "measured_time_s_median": None,
        "planning_s_median": None,
        "audit_s_median": None,
        "path_length_mean": None,
        "raw_segment_fraction_median": None,
        "status": "external_pending" if pending_count else "missing",
    }
    if method_label is not None:
        row["method_label"] = method_label
    row.update(dict(extra or {}))
    return row
