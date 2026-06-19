from __future__ import annotations

import math
from typing import Any, Sequence


def finite_float(value: Any, fallback: float = 1e9) -> float:
    """Return a finite float for sorting; use fallback for missing/NaN values."""

    try:
        out = float(value)
    except (TypeError, ValueError):
        return fallback
    return out if math.isfinite(out) else fallback


def first_row_value(row: dict[str, Any], keys: Sequence[str], default: Any = None) -> Any:
    """Return the first present, non-None row value from a list of keys."""

    for key in keys:
        if key in row and row.get(key) is not None:
            return row.get(key)
    return default


def count_ratio_text(
    row: dict[str, Any],
    success_keys: Sequence[str],
    total_keys: Sequence[str],
) -> str:
    """Format a success/total ratio from preferred row keys."""

    success = int(finite_float(first_row_value(row, success_keys, 0), fallback=0.0))
    total = int(finite_float(first_row_value(row, total_keys, 0), fallback=0.0))
    return f"{success}/{total}"


def amortized_query_time(
    row: dict[str, Any],
    queries_per_build: int | float,
    *,
    online_keys: Sequence[str] = ("online_per_query_s_median",),
    build_keys: Sequence[str] = ("offline_build_s_median", "build_s"),
) -> float:
    """Return build/K plus online-query time for reusable-planner tables."""

    queries = max(1.0, float(queries_per_build))
    build_s = finite_float(first_row_value(row, build_keys, 0.0), fallback=0.0)
    online_s = finite_float(first_row_value(row, online_keys, None))
    return build_s / queries + online_s


def path_length_stat(
    row: dict[str, Any],
    keys: Sequence[str] = ("path_length_mean", "path_length_median"),
) -> float:
    """Return the first finite path-length summary from an experiment row."""

    for key in keys:
        value = finite_float(row.get(key), fallback=math.nan)
        if math.isfinite(value):
            return value
    return math.nan


def filter_within_best_path_factor(
    rows: Sequence[dict[str, Any]],
    factor: float = 1.08,
) -> list[dict[str, Any]]:
    """Keep rows whose path length is within a relative factor of the best row."""

    candidates = list(rows)
    finite_paths = [
        path_length_stat(row)
        for row in candidates
        if math.isfinite(path_length_stat(row))
    ]
    if not finite_paths:
        return candidates
    best_path = min(finite_paths)
    filtered = [
        row for row in candidates
        if math.isfinite(path_length_stat(row))
        and path_length_stat(row) <= float(factor) * best_path
    ]
    return filtered or candidates
