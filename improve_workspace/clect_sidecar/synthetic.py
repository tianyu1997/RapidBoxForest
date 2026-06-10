"""Synthetic validators and benchmark helpers for sidecar validation."""

from __future__ import annotations

from dataclasses import dataclass

from .dyadic import DyadicAddress, Interval, JointBox, box_is_subset, boxes_touch_or_overlap
from .reports import Blocker, FailStage, ValidationReport


@dataclass(frozen=True)
class Region:
    box: JointBox
    label: str


class SyntheticValidator:
    """A deterministic stand-in for the SBF scene validator.

    A cell is FREE when it is fully contained in one configured free region.
    It is CERT_OCCUPIED when fully contained in an occupied region.  Otherwise
    it fails with a blocker signature whose affected joint is the widest
    dimension of the cell.
    """

    def __init__(
        self,
        free_regions: list[Region],
        occupied_regions: list[Region] | None = None,
    ) -> None:
        self.free_regions = free_regions
        self.occupied_regions = occupied_regions or []

    def __call__(self, address: DyadicAddress, box: JointBox) -> ValidationReport:
        for region in self.free_regions:
            if box_is_subset(box, region.box, tol=1e-12):
                return ValidationReport.free(certificate=f"inside:{region.label}")
        for region in self.occupied_regions:
            if box_is_subset(box, region.box, tol=1e-12):
                blocker = self._blocker(box, overlap=1.0)
                return ValidationReport.cert_occupied(f"occupied:{region.label}", [blocker])
        overlap = 1.0 if any(boxes_touch_or_overlap(box, region.box) for region in self.free_regions) else 0.25
        return ValidationReport.fail([self._blocker(box, overlap=overlap)], overlap_score=overlap)

    @staticmethod
    def _blocker(box: JointBox, overlap: float) -> Blocker:
        widest = max(range(len(box)), key=lambda dim: box[dim].width)
        return Blocker(
            link_id=0,
            obstacle_id=0,
            stage=FailStage.GJK,
            margin=-overlap,
            overlap_score=overlap,
            affected_joints=(widest,),
        )


def unit_root(dims: int) -> JointBox:
    return tuple(Interval(0.0, 1.0) for _ in range(dims))


def fixed_leaf_evaluation_count(start_depth: int, max_depth: int) -> int:
    if max_depth < start_depth:
        raise ValueError("max_depth < start_depth")
    return 1 << (max_depth - start_depth)

