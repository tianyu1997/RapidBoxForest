from __future__ import annotations

import math
import statistics
from typing import Any, Iterable


def finite(values: Iterable[Any]) -> list[float]:
    out: list[float] = []
    for value in values:
        try:
            x = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(x):
            out.append(x)
    return out


def mean(values: Iterable[Any]) -> float | None:
    vals = finite(values)
    return float(statistics.mean(vals)) if vals else None


def median(values: Iterable[Any]) -> float | None:
    vals = finite(values)
    return float(statistics.median(vals)) if vals else None


def percentile(values: Iterable[Any], q: float) -> float | None:
    vals = sorted(finite(values))
    if not vals:
        return None
    if len(vals) == 1:
        return vals[0]
    pos = max(0.0, min(1.0, float(q))) * (len(vals) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    alpha = pos - lo
    return vals[lo] * (1.0 - alpha) + vals[hi] * alpha


def success_rate(rows: Iterable[dict[str, Any]], key: str = "audit_passed") -> float:
    items = list(rows)
    if not items:
        return 0.0
    return sum(1 for row in items if bool(row.get(key))) / len(items)


def tex_num(value: Any, digits: int = 3) -> str:
    if value is None:
        return "--"
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(x):
        return "--"
    if abs(x) >= 100:
        return f"{x:.1f}"
    return f"{x:.{digits}f}"

