from __future__ import annotations

from typing import Iterable, Sequence


UNIFIED_SBF_ANYTIME_REFERENCE_ARTIFACT = "d40_r4_32_128_128_168_128.json"

UNIFIED_SBF_ANYTIME_STAGE_IDS: tuple[str, ...] = (
    "seed",
    "fast",
    "balanced",
    "quality",
    "high",
)
UNIFIED_SBF_ANYTIME_MAX_BOXES: tuple[int, ...] = (32, 128, 128, 168, 128)
UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES: tuple[int, ...] = (16, 80, 80, 132, 80)
UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES: tuple[int, ...] = (0, 8, 0, 12, 0)
UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS: tuple[float, ...] = (0.0, 40.0, 0.0, 80.0, 0.0)

UNIFIED_SBF_ANYTIME_THREADS = 8
UNIFIED_SBF_ANYTIME_TASK_BATCH_SIZE = 8
UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH = 40
UNIFIED_SBF_ANYTIME_FFB_START_DEPTH = 8
UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP = 0.01
UNIFIED_SBF_ANYTIME_POST_AUDIT_SEGMENT_STEP = 0.01


def _validate_lengths(items: Sequence[Sequence[object]]) -> None:
    lengths = {len(item) for item in items}
    if len(lengths) != 1:
        raise ValueError(f"unified anytime schedule has mismatched lengths: {sorted(lengths)}")


def csv_text(values: Iterable[object]) -> str:
    return ",".join(str(value) for value in values)


def csv_ints(values: Iterable[int]) -> str:
    return ",".join(str(int(value)) for value in values)


def csv_floats(values: Iterable[float]) -> str:
    return ",".join(f"{float(value):g}" for value in values)


def legacy_stage_spec(*, ffb_depth: int = UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH) -> str:
    _validate_lengths(
        (
            UNIFIED_SBF_ANYTIME_STAGE_IDS,
            UNIFIED_SBF_ANYTIME_MAX_BOXES,
            UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES,
            UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES,
            UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS,
        )
    )
    parts: list[str] = []
    for stage_id, quality, extra_boxes, budget_ms, max_boxes in zip(
        UNIFIED_SBF_ANYTIME_STAGE_IDS,
        UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES,
        UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES,
        UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS,
        UNIFIED_SBF_ANYTIME_MAX_BOXES,
    ):
        parts.append(
            f"{stage_id}:{int(quality)}:{int(extra_boxes)}:{float(budget_ms):g}:{int(max_boxes)}:{int(ffb_depth)}"
        )
    return ",".join(parts)