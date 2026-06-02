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
UNIFIED_SBF_ANYTIME_MAX_BOXES: tuple[int, ...] = (32, 96, 160, 224, 320)
UNIFIED_SBF_ANYTIME_QUALITY_MIN_CONNECTED_BOXES: tuple[int, ...] = (16, 64, 128, 192, 256)
UNIFIED_SBF_ANYTIME_POST_CONNECT_EXTRA_BOXES: tuple[int, ...] = (0, 0, 16, 32, 64)
UNIFIED_SBF_ANYTIME_POST_CONNECT_TIME_BUDGET_MS: tuple[float, ...] = (0.0, 0.0, 150.0, 300.0, 450.0)

UNIFIED_SBF_ANYTIME_THREADS = 8
UNIFIED_SBF_ANYTIME_TASK_BATCH_SIZE = 8
UNIFIED_SBF_ANYTIME_RBF_MAX_DEPTH = 40
UNIFIED_SBF_ANYTIME_FFB_START_DEPTH = 15
UNIFIED_SBF_ANYTIME_AUDIT_SEGMENT_STEP = 0.01
UNIFIED_SBF_ANYTIME_POST_AUDIT_SEGMENT_STEP = 0.01

# Exp.4 default grower sampling baseline:
# cat_i25_u20_r15_a025_localbest.
UNIFIED_SBF_SAMPLING_INTERTREE_GOAL_BIAS = 0.25
UNIFIED_SBF_SAMPLING_UNEXPLORED_PROB = 0.20
UNIFIED_SBF_SAMPLING_RRT_GOAL_BIAS = 0.15
UNIFIED_SBF_SAMPLING_ANCHOR_TARGET_PROB = 0.025
UNIFIED_SBF_SAMPLING_UNIFORM_PROB = 0.375
UNIFIED_SBF_SAMPLING_COMPONENT_CONNECT_PROB = 0.0
UNIFIED_SBF_SAMPLING_CATEGORICAL_ALLOCATION = True


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
