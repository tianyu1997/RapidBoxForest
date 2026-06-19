from __future__ import annotations

import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]


def configure_sbf_python_path() -> None:
    """Expose the local SBF Python binding without importing legacy experiments."""
    candidates = [
        REPO_ROOT / "build" / "python",
        REPO_ROOT / "build-leaf-sweep" / "python",
        REPO_ROOT / "build-exp04" / "python",
        REPO_ROOT / "build-rbf-python-current" / "python",
        REPO_ROOT / "build-consolidated-python" / "python",
        REPO_ROOT.parent,
        REPO_ROOT,
    ]
    candidate_resolved = {candidate.resolve() for candidate in candidates}
    for entry in list(sys.path):
        if not entry:
            continue
        candidate = Path(entry)
        try:
            resolved = candidate.resolve()
        except OSError:
            continue
        if resolved not in candidate_resolved:
            continue
        if not (candidate / "sbf" / "__init__.py").exists():
            continue
        sys.path.remove(entry)
        sys.path.insert(0, entry)
        return
    for candidate in reversed(candidates):
        if not candidate.exists():
            continue
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)


def import_sbf() -> Any:
    configure_sbf_python_path()
    import sbf  # type: ignore

    return sbf
