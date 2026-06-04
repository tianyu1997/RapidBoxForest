from __future__ import annotations

import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]


def configure_sbf_python_path() -> None:
    """Expose the local SBF Python binding without importing legacy experiments."""
    candidates = [
        REPO_ROOT / "build-leaf-sweep" / "python",
        REPO_ROOT / "build-exp04" / "python",
        REPO_ROOT / "build" / "python",
        REPO_ROOT / "build-rbf-python-current" / "python",
        REPO_ROOT / "build-consolidated-python" / "python",
        REPO_ROOT.parent,
        REPO_ROOT,
    ]
    for candidate in candidates:
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))


def import_sbf() -> Any:
    configure_sbf_python_path()
    import sbf  # type: ignore

    return sbf

