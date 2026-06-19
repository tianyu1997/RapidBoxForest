from __future__ import annotations

import math
from typing import Any

from experiments.common.metrics import median


def online_timing_from_query_rows(
    qrows: list[dict[str, Any]],
    *,
    online_total_s: float,
    build_s: float,
) -> dict[str, float]:
    query_count = max(1, len(qrows))
    total_s = float(online_total_s)
    online_solve_s = 0.0
    online_simplify_s = 0.0
    split_available = False
    for row in qrows:
        try:
            total_ms = float(row.get("query_ms", math.nan))
            solve_ms = float(row.get("solve_ms", math.nan))
            simplify_ms = float(row.get("simplify_ms", math.nan))
        except (TypeError, ValueError):
            continue
        if math.isfinite(solve_ms) or math.isfinite(simplify_ms):
            split_available = True
            if math.isfinite(solve_ms):
                online_solve_s += solve_ms / 1000.0
            elif math.isfinite(total_ms) and math.isfinite(simplify_ms):
                online_solve_s += max(0.0, total_ms - simplify_ms) / 1000.0
            if math.isfinite(simplify_ms):
                online_simplify_s += simplify_ms / 1000.0
    if not split_available:
        online_solve_s = total_s
        online_simplify_s = 0.0
    else:
        residual_s = total_s - online_solve_s - online_simplify_s
        if residual_s > 1e-9:
            online_solve_s += residual_s
    online_per_query_s = online_solve_s / query_count
    online_total_per_query_s = total_s / query_count
    out = {
        "online_batch_s": online_solve_s,
        "online_total_s": total_s,
        "online_total_batch_s": total_s,
        "online_solve_s": online_solve_s,
        "online_simplify_s": online_simplify_s,
        "online_per_query_s": online_per_query_s,
        "online_total_per_query_s": online_total_per_query_s,
        "online_solve_per_query_s": online_per_query_s,
        "online_simplify_per_query_s": online_simplify_s / query_count,
    }
    out.update({
        f"amortized_s_k{k}": float(build_s) / float(k) + online_per_query_s
        for k in (1, 5, 10, 20, 50)
    })
    return out


def row_online_batch_s(row: dict[str, Any]) -> float:
    return max(0.0, float(row.get("planning_s", 0.0)) - float(row.get("build_s", 0.0)))


def online_timing_medians(rows: list[dict[str, Any]]) -> dict[str, float | None]:
    """Return the common median online-timing summary fields for run rows."""

    return {
        "online_batch_s_median": median(row.get("online_batch_s", row_online_batch_s(row)) for row in rows),
        "online_total_s_median": median(
            row.get("online_total_s", row.get("online_batch_s", row_online_batch_s(row)))
            for row in rows
        ),
        "online_solve_s_median": median(
            row.get("online_solve_s", row.get("online_batch_s", row_online_batch_s(row)))
            for row in rows
        ),
        "online_simplify_s_median": median(row.get("online_simplify_s", 0.0) for row in rows),
        "online_solve_per_query_s_median": median(
            row.get(
                "online_solve_per_query_s",
                row.get("online_solve_s", row.get("online_batch_s", row_online_batch_s(row)))
                / max(1, int(row.get("query_count", 1))),
            )
            for row in rows
        ),
        "online_simplify_per_query_s_median": median(
            row.get(
                "online_simplify_per_query_s",
                row.get("online_simplify_s", 0.0) / max(1, int(row.get("query_count", 1))),
            )
            for row in rows
        ),
        "online_per_query_s_median": median(
            row.get(
                "online_per_query_s",
                row.get("online_solve_s", row_online_batch_s(row)) / max(1, int(row.get("query_count", 1))),
            )
            for row in rows
        ),
        "online_total_per_query_s_median": median(
            row.get(
                "online_total_per_query_s",
                row.get("online_total_s", row.get("online_batch_s", row_online_batch_s(row)))
                / max(1, int(row.get("query_count", 1))),
            )
            for row in rows
        ),
        "amortized_s_k1": median(row.get("amortized_s_k1", row.get("planning_s", math.nan)) for row in rows),
        "amortized_s_k5": median(row.get("amortized_s_k5", row.get("planning_s", math.nan) / 5.0) for row in rows),
        "amortized_s_k10": median(row.get("amortized_s_k10", row.get("planning_s", math.nan) / 10.0) for row in rows),
        "amortized_s_k20": median(row.get("amortized_s_k20", row.get("planning_s", math.nan) / 20.0) for row in rows),
        "amortized_s_k50": median(row.get("amortized_s_k50", row.get("planning_s", math.nan) / 50.0) for row in rows),
    }
