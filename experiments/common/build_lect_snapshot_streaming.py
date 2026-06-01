#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

try:
    import sbf
except Exception:  # pragma: no cover - fallback for environments without the extension
    sbf = None


REQUIRED_FILES = (
    "manifest.json",
    "nodes.pages",
    "evidence.pages",
    "evidence.index",
    "journal.log",
)


def file_size(path: Path) -> int:
    return path.stat().st_size if path.exists() and path.is_file() else 0


def replace_path(tmp: Path, dst: Path) -> None:
    if dst.exists() or dst.is_symlink():
        if dst.is_dir() and not dst.is_symlink():
            shutil.rmtree(dst)
        else:
            dst.unlink()
    tmp.rename(dst)


def link_or_copy_file(src: Path, dst: Path, *, copy: bool) -> str:
    if copy:
        shutil.copy2(src, dst)
        return "copy"
    try:
        os.link(src, dst)
        return "hardlink"
    except OSError:
        shutil.copy2(src, dst)
        return "copy"


def link_node_pages(src: Path, dst: Path, *, copy: bool) -> str:
    if not src.exists():
        return "missing"
    if copy:
        shutil.copytree(src, dst)
        return "copy"
    try:
        dst.symlink_to(src, target_is_directory=True)
        return "symlink"
    except OSError:
        shutil.copytree(src, dst)
        return "copy"


def build_snapshot(legacy_root: Path, snapshot_path: Path, *, copy: bool = False) -> dict[str, object]:
    legacy_root = legacy_root.resolve()
    missing = [name for name in REQUIRED_FILES if not (legacy_root / name).exists()]
    if missing:
        raise RuntimeError(f"legacy cache is missing required files: {missing}")
    journal_size = file_size(legacy_root / "journal.log")
    if journal_size != 0:
        raise RuntimeError(f"legacy cache journal is not clean: {journal_size} bytes")

    if sbf is None or not hasattr(sbf, "build_lect_snapshot_from_legacy"):
        raise RuntimeError("sbf.build_lect_snapshot_from_legacy is unavailable; rebuild _sbf_cpp first")

    if snapshot_path.exists() or snapshot_path.is_symlink():
        if snapshot_path.is_dir() and not snapshot_path.is_symlink():
            shutil.rmtree(snapshot_path)
        else:
            snapshot_path.unlink()

    snapshot_path.parent.mkdir(parents=True, exist_ok=True)
    built_path = Path(sbf.build_lect_snapshot_from_legacy(str(legacy_root), str(snapshot_path)))
    modes: dict[str, str] = {
        "manifest.bin": "cpp",
        "nodes.bin": "cpp",
        "evidence_table.bin": "cpp",
        "direct_evidence.bin": "cpp",
        "payload.bin": "cpp",
    }
    return {
        "snapshot_path": str(built_path),
        "legacy_root": str(legacy_root),
        "copy": bool(copy),
        "modes": modes,
        "bytes": sum(file_size(built_path / name) for name in modes),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a low-memory LECT read snapshot from an existing checkpoint cache.")
    parser.add_argument("legacy_root", type=Path)
    parser.add_argument("snapshot_path", type=Path, nargs="?")
    parser.add_argument("--copy", action="store_true", help="Copy files instead of hardlinking/symlinking them.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    snapshot_path = args.snapshot_path or (args.legacy_root / "lect_snapshot")
    summary = build_snapshot(args.legacy_root, snapshot_path, copy=bool(args.copy))
    for key, value in summary.items():
        print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())