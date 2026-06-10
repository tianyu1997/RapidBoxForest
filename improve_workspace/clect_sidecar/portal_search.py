"""Domain-internal portal search prototype."""

from __future__ import annotations

from dataclasses import dataclass

from .dyadic import DyadicAddress, JointBox, box_center, box_is_subset, boxes_touch_or_overlap
from .portal import Portal, PortalCorridor
from .reports import ValidationReport, ValidationStatus


@dataclass(frozen=True)
class BoundaryBox:
    box_id: str
    component_id: str
    box: JointBox
    face_id: str = ""


def detect_portals(domain_id: str, domain_box: JointBox, boundary_boxes: list[BoundaryBox]) -> list[Portal]:
    """Return forest boxes touching a retained collision/refinement domain."""

    portals: list[Portal] = []
    for item in boundary_boxes:
        if boxes_touch_or_overlap(domain_box, item.box, tol=1e-12):
            portals.append(
                Portal(
                    domain_id=domain_id,
                    boundary_box_id=item.box_id,
                    component_id=item.component_id,
                    face_id=item.face_id,
                )
            )
    return portals


def greedy_portal_search(
    corridor_id: str,
    domain_id: str,
    domain_box: JointBox,
    pin: Portal,
    pout: Portal,
    pin_box: JointBox,
    pout_box: JointBox,
    candidates: list[tuple[DyadicAddress, JointBox]],
    reports: dict[DyadicAddress, ValidationReport],
) -> PortalCorridor | None:
    """Build a conservative internal chain from candidate cells.

    This prototype is deliberately simple: it filters validated free candidates
    inside the domain, then greedily advances through adjacent cells toward the
    output portal.  Production code should replace the greedy step with the
    priority-driven local adaptive search described in `docs/improve.md`.
    """

    free_candidates: list[tuple[DyadicAddress, JointBox, ValidationReport]] = []
    for address, box in candidates:
        report = reports.get(address)
        if report is None or report.status != ValidationStatus.FREE:
            continue
        if not box_is_subset(box, domain_box, tol=1e-12):
            continue
        free_candidates.append((address, box, report))

    chain_cells: list[DyadicAddress] = []
    chain_boxes: list[JointBox] = []
    chain_reports: list[ValidationReport] = []
    current = pin_box
    target_center = box_center(pout_box)
    used: set[DyadicAddress] = set()

    while True:
        if boxes_touch_or_overlap(current, pout_box, tol=1e-12):
            return PortalCorridor(
                corridor_id,
                domain_id,
                pin,
                pout,
                chain_cells,
                chain_boxes,
                chain_reports,
                conservative=True,
            )
        adjacent = [
            item
            for item in free_candidates
            if item[0] not in used and boxes_touch_or_overlap(current, item[1], tol=1e-12)
        ]
        if not adjacent:
            return None
        address, box, report = min(
            adjacent,
            key=lambda item: sum((a - b) ** 2 for a, b in zip(box_center(item[1]), target_center)),
        )
        used.add(address)
        chain_cells.append(address)
        chain_boxes.append(box)
        chain_reports.append(report)
        current = box

