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
