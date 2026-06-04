#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, write_json


OLD_GENERATED = Path("/home/tian/桌面/box_aabb/cpp/SBF/doc/paper/tro_rewrite_2026/generated")

TABLES = {}


def file_sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Import old TRO supporting mechanism tables with source hashes.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "supporting_imports")
    parser.add_argument("--paper-dir", type=Path, default=REPO_ROOT / "paper")
    parser.add_argument("--old-generated", type=Path, default=OLD_GENERATED)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    generated = args.paper_dir / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    ok = True
    for target_name, spec in TABLES.items():
        source = args.old_generated / str(spec["source"])
        target = generated / target_name
        status = "pass"
        reason = ""
        text = ""
        if not source.exists():
            status = "fail"
            reason = "source_missing"
            ok = False
        else:
            text = source.read_text(encoding="utf-8")
            missing = [term for term in spec["required_terms"] if term not in text]
            if missing:
                status = "fail"
                reason = "missing_required_terms:" + ",".join(missing)
                ok = False
            else:
                header = "\n".join(
                    [
                        "% Imported by experiments/import_old_tro2026_supporting_tables.py.",
                        f"% Source: {source}",
                        f"% Source SHA256: {file_sha256(source)}",
                    ]
                )
                target.write_text(header + "\n" + text, encoding="utf-8")
        rows.append(
            {
                "target": str(target),
                "source": str(source),
                "experiment": spec["experiment"],
                "status": status,
                "reason": reason,
                "source_sha256": file_sha256(source),
                "target_sha256": file_sha256(target),
            }
        )
    payload = {
        "experiment": "old_tro2026_supporting_table_import",
        "status": "imported" if ok else "failed",
        "environment": environment_metadata(),
        "old_generated": str(args.old_generated),
        "paper_generated": str(generated),
        "rows": rows,
        "policy": "Tables are copied as immutable old TRO artifacts with hashes. No old experiment scripts are imported or executed.",
    }
    write_json(args.out_dir / "supporting_table_import_manifest.json", payload)
    (args.out_dir / "supporting_table_import_summary.md").write_text(
        "\n".join(
            [
                "# Supporting Table Import",
                "",
                f"Status: **{payload['status']}**",
                "",
                "| Target | Status | Source SHA256 |",
                "|---|---:|---|",
                *[f"| `{Path(row['target']).name}` | {row['status']} | `{row['source_sha256']}` |" for row in rows],
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(f"wrote {args.out_dir / 'supporting_table_import_manifest.json'}")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
