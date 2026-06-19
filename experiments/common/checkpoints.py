from __future__ import annotations

import argparse
from typing import Any


def _csv_floats(raw: str) -> list[float]:
    return [float(item.strip()) for item in str(raw).split(",") if item.strip()]


def progressive_checkpoint_grid(timeout_s: float, *, max_step_s: float = 0.1) -> list[float]:
    timeout = max(0.0, float(timeout_s))
    if timeout <= 0.0:
        return []
    # Dense first-solution region, then relaxed checkpoints. The final segment
    # is capped by max_step_s so per-query trade-off selection is not forced to
    # wait for a coarse one-second checkpoint.
    segments = [
        (0.10, 0.005),
        (0.50, 0.010),
        (1.00, 0.020),
        (2.00, 0.050),
        (timeout, max(1e-9, min(float(max_step_s), 0.100))),
    ]
    values: list[float] = []
    current = 0.0
    for end_s, step_s in segments:
        end = min(timeout, float(end_s))
        step = max(1e-9, float(step_s))
        while current + step < end - 1e-9:
            current += step
            values.append(round(current, 9))
        if end > current + 1e-9:
            current = end
            values.append(round(current, 9))
        if current >= timeout - 1e-9:
            break
    if not values or abs(values[-1] - timeout) > 1e-9:
        values.append(round(timeout, 9))
    return sorted({float(value) for value in values if value > 0.0 and value <= timeout + 1e-9})


def _explicit_or_uniform_grid(raw_grid: str, timeout_s: float, interval_s: float) -> list[float]:
    timeout = float(timeout_s)
    if raw_grid:
        values = sorted({float(value) for value in _csv_floats(raw_grid) if float(value) > 0.0})
        values = [min(timeout, value) for value in values if value <= timeout + 1e-9]
        if not values or abs(values[-1] - timeout) > 1e-9:
            values.append(timeout)
        return values
    interval = max(float(interval_s), 1e-9)
    values: list[float] = []
    target_s = interval
    while target_s < timeout - 1e-9:
        values.append(float(target_s))
        target_s += interval
    values.append(timeout)
    return values


def bitstar_checkpoint_grid_from_args(args: argparse.Namespace, timeout_s: float) -> list[float]:
    raw_grid = str(getattr(args, "bitstar_checkpoint_grid_s", "")).strip()
    if not raw_grid and str(getattr(args, "bitstar_timeout_grid_s", "")).strip():
        raw_grid = str(getattr(args, "bitstar_timeout_grid_s"))
    schedule = str(getattr(args, "bitstar_checkpoint_schedule", "uniform"))
    if not raw_grid and schedule == "explicit":
        raise ValueError("--bitstar-checkpoint-schedule=explicit requires --bitstar-checkpoint-grid-s")
    if raw_grid:
        return _explicit_or_uniform_grid(
            raw_grid,
            float(timeout_s),
            float(getattr(args, "bitstar_checkpoint_interval_s", 1e-9)),
        )
    if schedule == "progressive":
        return progressive_checkpoint_grid(
            float(timeout_s),
            max_step_s=float(getattr(args, "bitstar_checkpoint_max_step_s", 0.1)),
        )
    return _explicit_or_uniform_grid(
        "",
        float(timeout_s),
        float(getattr(args, "bitstar_checkpoint_interval_s", 1e-9)),
    )


def bitstar_trace_interval_for_grid(
    args: argparse.Namespace,
    checkpoint_grid_s: list[float],
    timeout_s: float,
) -> float:
    deltas = [
        float(b) - float(a)
        for a, b in zip([0.0] + checkpoint_grid_s[:-1], checkpoint_grid_s)
        if float(b) - float(a) > 1e-9
    ]
    if deltas:
        return max(1e-9, min(deltas))
    return max(1e-9, min(float(getattr(args, "bitstar_checkpoint_interval_s", 1e-9)), float(timeout_s)))


def checkpoint_at_or_after(checkpoints: list[dict[str, Any]], target_s: float) -> dict[str, Any]:
    if not checkpoints:
        return {}
    for checkpoint in checkpoints:
        if float(checkpoint.get("checkpoint_s", 0.0) or 0.0) >= float(target_s) - 1e-9:
            return checkpoint
    return checkpoints[-1]
