#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from experiments.common.marcucci_anchor_guard import anchor_validation_report


KNOWN_STALE_KINDS = {"legacy_joint0_pi_over_2", "legacy_lb_joint0_shift", "legacy_rb_joint0_shift"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Delete stale Marcucci artifacts that match known historical anchor conventions.")
    parser.add_argument("--root", type=Path, default=Path("outputs/new_experiments"))
    parser.add_argument("--delete", action="store_true")
    return parser.parse_args()


def prune_empty_parents(path: Path, stop: Path) -> None:
    current = path
    while current != stop and current.exists():
        try:
            next(current.iterdir())
            return
        except StopIteration:
            current.rmdir()
            current = current.parent


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    stale_files: list[tuple[Path, dict[str, object]]] = []
    mismatch_files: list[tuple[Path, dict[str, object]]] = []
    for path in sorted(root.rglob("*.json")):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        report = anchor_validation_report(payload)
        if not report["findings"]:
            continue
        only_known_stale = all(item["kind"] in KNOWN_STALE_KINDS for item in report["findings"])
        if only_known_stale:
            stale_files.append((path, report))
        else:
            mismatch_files.append((path, report))

    print(f"known-stale files: {len(stale_files)}")
    for path, report in stale_files:
        print(f"  {path}")
        if report["findings"]:
            item = report["findings"][0]
            print(f"    {item['query']} {item['endpoint']} {item['kind']} delta_joint0={item['delta_joint0']:.6f}")

    if mismatch_files:
        print(f"unexpected-mismatch files: {len(mismatch_files)}")
        for path, report in mismatch_files[:20]:
            item = report["findings"][0]
            print(f"  {path}  {item['query']} {item['endpoint']} {item['kind']}")

    if args.delete:
        for path, _ in stale_files:
            path.unlink(missing_ok=True)
            prune_empty_parents(path.parent, root)
        print(f"deleted known-stale files: {len(stale_files)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())