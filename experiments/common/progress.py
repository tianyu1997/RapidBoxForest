"""Progress-bar helper for experiment runners."""

from __future__ import annotations

import os
import sys
from collections.abc import Iterable, Iterator
from typing import TypeVar

T = TypeVar("T")


def progress_enabled() -> bool:
    value = os.environ.get("RBF_PROGRESS", "1").strip().lower()
    return value not in {"0", "false", "no", "off"}


def progress(
    iterable: Iterable[T],
    *,
    total: int | None = None,
    desc: str = "",
    disable: bool = False,
) -> Iterator[T]:
    """Wrap an iterable with tqdm when available, with a stderr fallback."""

    if disable or not progress_enabled():
        yield from iterable
        return

    try:
        from tqdm.auto import tqdm  # type: ignore
    except Exception:
        yield from _fallback_progress(iterable, total=total, desc=desc)
        return

    yield from tqdm(iterable, total=total, desc=desc or None, dynamic_ncols=True)


def _fallback_progress(iterable: Iterable[T], *, total: int | None, desc: str) -> Iterator[T]:
    label = desc or "progress"
    count = 0
    if total is not None:
        print(f"[{label}] 0/{total}", file=sys.stderr, flush=True)
    for item in iterable:
        yield item
        count += 1
        if total is None:
            if count == 1 or count % 10 == 0:
                print(f"[{label}] {count}", file=sys.stderr, flush=True)
        else:
            step = max(1, total // 20)
            if count == total or count % step == 0:
                print(f"[{label}] {count}/{total}", file=sys.stderr, flush=True)
