from __future__ import annotations

from typing import Any


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
