from __future__ import annotations

from typing import Any

from experiments.common.metrics import mean


def successful_query_rows(qrows: list[dict[str, Any]], success_key: str = "audit_passed") -> list[dict[str, Any]]:
    """Return query rows that satisfy the experiment success predicate."""

    return [row for row in qrows if bool(row.get(success_key))]


def query_success_summary(
    qrows: list[dict[str, Any]],
    *,
    success_key: str = "audit_passed",
    path_length_key: str = "path_length",
) -> dict[str, Any]:
    """Summarize per-query success and success-only path length."""

    successes = successful_query_rows(qrows, success_key=success_key)
    return {
        "status": "ok" if len(successes) == len(qrows) else "partial",
        "success_count": len(successes),
        "query_count": len(qrows),
        "path_length_mean": mean(row.get(path_length_key) for row in successes),
    }
