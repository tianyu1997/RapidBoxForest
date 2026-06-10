#!/usr/bin/env python3
"""Inspect the improve.md source plan without modifying the repository."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def markdown_headings(text: str) -> list[dict[str, object]]:
    headings: list[dict[str, object]] = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        stripped = line.lstrip()
        if not stripped.startswith("#"):
            continue
        marks = len(stripped) - len(stripped.lstrip("#"))
        if marks <= 0 or marks > 6:
            continue
        title = stripped[marks:].strip()
        if not title:
            continue
        headings.append({"line": lineno, "level": marks, "title": title})
    return headings


def inspect_plan(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    text = data.decode("utf-8", errors="replace")
    return {
        "path": str(path),
        "exists": path.exists(),
        "bytes": len(data),
        "lines": len(text.splitlines()),
        "sha256": hashlib.sha256(data).hexdigest(),
        "is_empty": len(data) == 0,
        "headings": markdown_headings(text),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan",
        default="docs/improve.md",
        help="Path to the improve.md plan file.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit machine-readable JSON.",
    )
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="Return success even when the plan file is empty.",
    )
    args = parser.parse_args()

    path = Path(args.plan)
    if not path.exists():
        payload = {
            "path": str(path),
            "exists": False,
            "error": "plan file does not exist",
        }
        print(json.dumps(payload, indent=2, ensure_ascii=False) if args.json else payload["error"])
        return 2

    payload = inspect_plan(path)
    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        print(f"path: {payload['path']}")
        print(f"bytes: {payload['bytes']}")
        print(f"lines: {payload['lines']}")
        print(f"sha256: {payload['sha256']}")
        print(f"headings: {len(payload['headings'])}")
        if payload["is_empty"]:
            print("status: empty plan; implementation cannot proceed")
    if payload["is_empty"] and not args.allow_empty:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
