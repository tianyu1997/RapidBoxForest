from __future__ import annotations


def fmt_float(value: float) -> str:
    """Format numeric CLI parameters for stable filename/stage slugs."""

    return f"{float(value):g}".replace("-", "m").replace(".", "p")
