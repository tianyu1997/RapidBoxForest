"""Adaptive early-stop leaf sweep sidecar implementation."""

from __future__ import annotations

import heapq
import itertools
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from enum import Enum
from math import log
from typing import Callable, Sequence

from .dyadic import (
    DyadicAddress,
    Interval,
    JointBox,
    box_center,
    box_contains_point,
    box_is_subset,
    box_volume,
    boxes_touch_or_overlap,
    split_schedule_cells,
)
from .reports import Blocker, ValidationReport, ValidationStatus


class SweepMode(str, Enum):
    OFFLINE_COVERAGE = "offline_coverage"
    RESTRICTED_DOMAIN_REFINE = "restricted_domain_refine"
    ONLINE_QUERY_REPAIR = "online_query_repair"


@dataclass
class SweepContext:
    mode: SweepMode = SweepMode.OFFLINE_COVERAGE
    anchors: tuple[tuple[float, ...], ...] = ()
    target_points: tuple[tuple[float, ...], ...] = ()
    target_domain: JointBox | None = None
    targeted_child_selection: bool = False


@dataclass
class AdaptiveSweepConfig:
    start_depth: int = 8
    max_depth: int = 13
    max_evaluations: int = 10000
    no_good_chain: int = 3
    no_good_progress_threshold: float = 0.05
    no_good_cooldown: int = 100
    top_k_blockers: int = 3
    frontier_weight: float = 1.0
    component_weight: float = 1.5
    anchor_weight: float = 2.0
    portal_weight: float = 1.0
    blocker_weight: float = 0.75
    history_weight: float = 1.0
    volume_weight: float = 0.05
    depth_weight: float = 0.02
    split_schedule: tuple[int, ...] | None = None


@dataclass
class TerminalCell:
    address: DyadicAddress
    box: JointBox
    status: ValidationStatus
    report: ValidationReport
    reason: str = ""


@dataclass
class AdaptiveSweepResult:
    free: list[TerminalCell] = field(default_factory=list)
    occupied: list[TerminalCell] = field(default_factory=list)
    collision: list[TerminalCell] = field(default_factory=list)
    deferred: list[TerminalCell] = field(default_factory=list)
    covered: list[TerminalCell] = field(default_factory=list)
    evaluated: int = 0
    split_count: int = 0
    validation_counts: Counter[str] = field(default_factory=Counter)
    depth_histogram: Counter[int] = field(default_factory=Counter)
    blocker_histogram: Counter[str] = field(default_factory=Counter)

    @property
    def terminal_count(self) -> int:
        return (
            len(self.free)
            + len(self.occupied)
            + len(self.collision)
            + len(self.deferred)
            + len(self.covered)
        )


ValidationFn = Callable[[DyadicAddress, JointBox], ValidationReport]


class AdaptiveLeafSweep:
    """Priority-driven adaptive classifier.

    This implements the plan's `AdaptiveLeafSweep + AdaptiveClassify` semantics
    with early free acceptance, optional no-good deferral, local split scoring,
    and targeted child selection.  It is a sidecar prototype and does not touch
    existing RBF data structures.
    """

    def __init__(
        self,
        root: JointBox,
        validate: ValidationFn,
        config: AdaptiveSweepConfig | None = None,
        context: SweepContext | None = None,
    ) -> None:
        self.root = root
        self.validate = validate
        self.config = config or AdaptiveSweepConfig()
        self.context = context or SweepContext()
        self._counter = itertools.count()
        self._accepted_free: list[TerminalCell] = []
        self._history_count: defaultdict[tuple[tuple[int, int, str], ...], int] = defaultdict(int)
        self._history_best_overlap: dict[tuple[tuple[int, int, str], ...], float] = {}

    def run(self) -> AdaptiveSweepResult:
        result = AdaptiveSweepResult()
        queue: list[tuple[float, int, DyadicAddress]] = []
        for cell in split_schedule_cells(
            len(self.root),
            self.config.start_depth,
            self.config.split_schedule,
        ):
            self._push(queue, cell)

        while queue and result.evaluated < self.config.max_evaluations:
            _, _, address = heapq.heappop(queue)
            box = address.interval_box(self.root)

            if self._covered_by_accepted(box):
                report = ValidationReport(status=ValidationStatus.COVERED)
                result.covered.append(TerminalCell(address, box, ValidationStatus.COVERED, report, "covered"))
                continue

            report = self.validate(address, box)
            result.evaluated += 1
            result.validation_counts[report.status.value] += 1
            result.depth_histogram[address.depth] += 1
            if report.blockers:
                result.blocker_histogram[str(report.blocker_signature(self.config.top_k_blockers))] += 1

            if report.status == ValidationStatus.FREE:
                terminal = TerminalCell(address, box, ValidationStatus.FREE, report, "early_free_accept")
                result.free.append(terminal)
                self._accepted_free.append(terminal)
                continue

            if report.status == ValidationStatus.CERT_OCCUPIED:
                result.occupied.append(TerminalCell(address, box, report.status, report, "sound_occupied"))
                continue

            if address.depth >= self.config.max_depth:
                result.collision.append(TerminalCell(address, box, ValidationStatus.INCONCLUSIVE, report, "depth_cap"))
                continue

            if self._dominated_by_no_good(report):
                result.deferred.append(TerminalCell(address, box, ValidationStatus.DEFERRED, report, "no_good"))
                continue

            if not self._relevant(box, report):
                result.deferred.append(TerminalCell(address, box, ValidationStatus.DEFERRED, report, "low_relevance"))
                continue

            split_dim = self._choose_split_dimension(box, report)
            children = list(address.split(split_dim))
            selected = self._select_children(children)
            result.split_count += 1
            for child in selected:
                self._push(queue, child)

        return result

    def _push(self, queue: list[tuple[float, int, DyadicAddress]], address: DyadicAddress) -> None:
        heapq.heappush(queue, (-self._priority(address), next(self._counter), address))

    def _priority(self, address: DyadicAddress) -> float:
        box = address.interval_box(self.root)
        score = self.config.volume_weight * log(max(box_volume(box), 1e-300))
        score -= self.config.depth_weight * address.depth
        if any(box_contains_point(box, anchor) for anchor in self.context.anchors):
            score += self.config.anchor_weight
        if self._touches_free_frontier(box):
            score += self.config.frontier_weight
        if self.context.target_domain is not None and boxes_touch_or_overlap(box, self.context.target_domain):
            score += self.config.portal_weight
        return score

    def _covered_by_accepted(self, box: JointBox) -> bool:
        return any(box_is_subset(box, accepted.box, tol=1e-12) for accepted in self._accepted_free)

    def _touches_free_frontier(self, box: JointBox) -> bool:
        return any(boxes_touch_or_overlap(box, accepted.box, tol=1e-12) for accepted in self._accepted_free)

    def _relevant(self, box: JointBox, report: ValidationReport) -> bool:
        if self.context.mode == SweepMode.OFFLINE_COVERAGE:
            return True
        if self._touches_free_frontier(box):
            return True
        if any(box_contains_point(box, point) for point in self.context.anchors):
            return True
        if self.context.target_domain is not None and boxes_touch_or_overlap(box, self.context.target_domain):
            return True
        if report.blockers and report.overlap_score > 0.0:
            return True
        return False

    def _dominated_by_no_good(self, report: ValidationReport) -> bool:
        signature = report.blocker_signature(self.config.top_k_blockers)
        if not signature:
            return False
        previous = self._history_best_overlap.get(signature)
        self._history_count[signature] += 1
        if previous is None:
            self._history_best_overlap[signature] = report.overlap_score
            return False
        best = max(previous, report.overlap_score)
        self._history_best_overlap[signature] = best
        progress = 0.0 if best <= 0.0 else (best - report.overlap_score) / (best + 1e-12)
        return (
            self._history_count[signature] >= self.config.no_good_chain
            and progress < self.config.no_good_progress_threshold
        )

    def _choose_split_dimension(self, box: JointBox, report: ValidationReport) -> int:
        scores = [interval.width for interval in box]
        for blocker in report.blockers:
            for joint in blocker.affected_joints:
                if 0 <= joint < len(scores):
                    scores[joint] += self.config.blocker_weight * (1.0 + blocker.overlap_score)
        return max(range(len(scores)), key=lambda dim: (scores[dim], -dim))

    def _select_children(self, children: Sequence[DyadicAddress]) -> list[DyadicAddress]:
        if not self.context.targeted_child_selection or not self.context.target_points:
            return list(children)
        ranked = sorted(children, key=self._distance_to_targets)
        if self.context.mode == SweepMode.ONLINE_QUERY_REPAIR:
            return ranked[:1]
        return ranked

    def _distance_to_targets(self, address: DyadicAddress) -> float:
        center = box_center(address.interval_box(self.root))
        best = float("inf")
        for target in self.context.target_points:
            dist = sum((a - b) ** 2 for a, b in zip(center, target))
            best = min(best, dist)
        return best

