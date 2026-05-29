from __future__ import annotations

import shutil
from pathlib import Path
from typing import Iterable, Any


def prune_directory_children(path: Path, keep_names: Iterable[str]) -> dict[str, Any]:
    keep = {str(name) for name in keep_names if str(name)}
    removed: list[str] = []
    path.mkdir(parents=True, exist_ok=True)
    for child in sorted(path.iterdir(), key=lambda item: item.name):
        if child.name in keep:
            continue
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()
        removed.append(child.name)
    return {
        "path": str(path),
        "kept": sorted(keep),
        "removed": removed,
        "removed_count": len(removed),
    }


def copytree_fresh(source: Path, target: Path) -> None:
    if target.exists():
        if target.is_dir():
            shutil.rmtree(target)
        else:
            target.unlink()
    shutil.copytree(source, target)