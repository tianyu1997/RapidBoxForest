from __future__ import annotations

import math
from typing import Any


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
