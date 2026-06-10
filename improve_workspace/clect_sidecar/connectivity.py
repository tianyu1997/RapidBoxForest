"""Connectivity-dominance sidecar helpers.

These helpers implement the budget-allocation logic from `docs/improve.md`
Section 2.4.  They never certify a cell as free or occupied; they only classify
whether an already inconclusive cell is worth refining soon.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .dyadic import JointBox, box_is_subset, boxes_touch_or_overlap


class ConnectivityAction(str, Enum):
    COVERED = "covered"
    PRIORITIZE_CONNECTOR = "prioritize_connector"
    RELEVANT = "relevant"
    DEFER_CONNECTIVITY = "defer_connectivity"
    LOW_PRIORITY_DEFER = "low_priority_defer"


@dataclass(frozen=True)
class ComponentBox:
    box_id: str
    component_id: str
    box: JointBox


@dataclass(frozen=True)
class ConnectivityContext:
    contains_anchor: bool = False
    on_portal_boundary: bool = False
    near_query_repair_target: bool = False
    opens_new_frontier: bool = False
    offline_anchor_cell: bool = False

    @property
    def is_useful_single_component_refine(self) -> bool:
        return (
            self.contains_anchor
            or self.on_portal_boundary
            or self.near_query_repair_target
            or self.opens_new_frontier
        )


@dataclass(frozen=True)
class ConnectivityDecision:
    action: ConnectivityAction
    covered_by: str | None
    adjacent_components: frozenset[str]
    adjacent_box_ids: tuple[str, ...]
    reason: str


def component_neighbors(
    cell: JointBox,
    accepted: list[ComponentBox],
    tol: float = 1e-12,
) -> tuple[frozenset[str], tuple[str, ...]]:
    components: set[str] = set()
    box_ids: list[str] = []
    for item in accepted:
        if boxes_touch_or_overlap(cell, item.box, tol):
            components.add(item.component_id)
            box_ids.append(item.box_id)
    return frozenset(components), tuple(box_ids)


def covered_by_accepted(
    cell: JointBox,
    accepted: list[ComponentBox],
    tol: float = 1e-12,
) -> str | None:
    for item in accepted:
        if box_is_subset(cell, item.box, tol):
            return item.box_id
    return None


def classify_connectivity_dominance(
    cell: JointBox,
    accepted: list[ComponentBox],
    context: ConnectivityContext | None = None,
    tol: float = 1e-12,
) -> ConnectivityDecision:
    context = context or ConnectivityContext()
    covered = covered_by_accepted(cell, accepted, tol)
    if covered is not None:
        return ConnectivityDecision(
            action=ConnectivityAction.COVERED,
            covered_by=covered,
            adjacent_components=frozenset(),
            adjacent_box_ids=(),
            reason="cell subset of an accepted free box",
        )

    components, box_ids = component_neighbors(cell, accepted, tol)
    if len(components) >= 2:
        return ConnectivityDecision(
            action=ConnectivityAction.PRIORITIZE_CONNECTOR,
            covered_by=None,
            adjacent_components=components,
            adjacent_box_ids=box_ids,
            reason="cell can potentially connect multiple components",
        )

    if len(components) == 1:
        if context.is_useful_single_component_refine:
            return ConnectivityDecision(
                action=ConnectivityAction.RELEVANT,
                covered_by=None,
                adjacent_components=components,
                adjacent_box_ids=box_ids,
                reason="single-component cell has anchor, portal, query, or frontier value",
            )
        return ConnectivityDecision(
            action=ConnectivityAction.DEFER_CONNECTIVITY,
            covered_by=None,
            adjacent_components=components,
            adjacent_box_ids=box_ids,
            reason="single-component interior refinement has low connectivity value",
        )

    if context.offline_anchor_cell or context.is_useful_single_component_refine:
        return ConnectivityDecision(
            action=ConnectivityAction.RELEVANT,
            covered_by=None,
            adjacent_components=frozenset(),
            adjacent_box_ids=(),
            reason="isolated cell is explicitly protected by context",
        )

    return ConnectivityDecision(
        action=ConnectivityAction.LOW_PRIORITY_DEFER,
        covered_by=None,
        adjacent_components=frozenset(),
        adjacent_box_ids=(),
        reason="isolated cell has no anchor, portal, query, or frontier value",
    )
