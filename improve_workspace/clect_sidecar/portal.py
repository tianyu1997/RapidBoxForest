"""Compressed portal edge model for sidecar C-LECT experiments."""

from __future__ import annotations

from dataclasses import dataclass, field

from .dyadic import DyadicAddress, JointBox, boxes_touch_or_overlap
from .reports import ValidationReport, ValidationStatus


@dataclass(frozen=True)
class Portal:
    domain_id: str
    boundary_box_id: str
    component_id: str
    face_id: str = ""
    seed_points: tuple[tuple[float, ...], ...] = ()


@dataclass
class PortalCorridor:
    corridor_id: str
    domain_id: str
    pin: Portal
    pout: Portal
    internal_cells: list[DyadicAddress]
    internal_boxes: list[JointBox]
    reports: list[ValidationReport]
    conservative: bool = True

    def validate_certificate(self, pin_box: JointBox, pout_box: JointBox, tol: float = 1e-12) -> bool:
        if not self.internal_boxes:
            return boxes_touch_or_overlap(pin_box, pout_box, tol)
        if len(self.internal_boxes) != len(self.reports):
            return False
        if any(report.status != ValidationStatus.FREE for report in self.reports):
            return False
        if not boxes_touch_or_overlap(pin_box, self.internal_boxes[0], tol):
            return False
        for lhs, rhs in zip(self.internal_boxes, self.internal_boxes[1:]):
            if not boxes_touch_or_overlap(lhs, rhs, tol):
                return False
        return boxes_touch_or_overlap(self.internal_boxes[-1], pout_box, tol)

    def expanded_box_ids(self) -> list[str]:
        internal = [f"{self.corridor_id}:cell:{i}" for i, _ in enumerate(self.internal_cells)]
        return [self.pin.boundary_box_id, *internal, self.pout.boundary_box_id]


@dataclass
class PortalGraph:
    """Global graph view that stores deep local chains as compressed edges."""

    adjacency: dict[str, list[tuple[str, str]]] = field(default_factory=dict)
    corridors: dict[str, PortalCorridor] = field(default_factory=dict)

    def add_portal_corridor(self, corridor: PortalCorridor, pin_box: JointBox, pout_box: JointBox) -> None:
        if corridor.conservative and not corridor.validate_certificate(pin_box, pout_box):
            raise ValueError(f"invalid conservative portal corridor {corridor.corridor_id}")
        self.corridors[corridor.corridor_id] = corridor
        self.adjacency.setdefault(corridor.pin.boundary_box_id, []).append(
            (corridor.pout.boundary_box_id, corridor.corridor_id)
        )
        self.adjacency.setdefault(corridor.pout.boundary_box_id, []).append(
            (corridor.pin.boundary_box_id, corridor.corridor_id)
        )

    def expand_edge(self, corridor_id: str, reverse: bool = False) -> list[str]:
        corridor = self.corridors[corridor_id]
        expanded = corridor.expanded_box_ids()
        return list(reversed(expanded)) if reverse else expanded

