#!/usr/bin/env python3
"""Compatibility wrapper for the renamed TRO 2026 table generator.

Prefer experiments/tro2026_generate_tables.py for new commands.
"""
from __future__ import annotations

from RapidBoxForest.safe_box_forest.experiments.sbf_old.tro2026_generate_tables import main


if __name__ == "__main__":
    raise SystemExit(main())
