"""Hierarchical Partition Connectivity sidecar implementation.

This module implements the strategy described in
``docs/分级partition连通.md`` as a standalone prototype.  It deliberately
stays inside ``improve_workspace`` and reuses the existing sidecar dyadic
address, validation-report, and portal-corridor primitives.
"""

from __future__ import annotations

import heapq
import itertools
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from enum import Enum
from math import log
from typing import Callable, Iterable, Sequence

from .dyadic import (
    DyadicAddress,
    JointBox,
    box_center,
    box_contains_point,
    box_volume,
    boxes_touch_or_overlap,
    jump_cell_containing,
    split_schedule_cells,
)
from .portal import Portal, PortalCorridor
from .reports import ValidationReport, ValidationStatus


class HiPaCCellState(str, Enum):
    """Terminal and parent states from the hierarchical-connectivity plan."""

    FREE = "FREE"
    CERT_OCCUPIED = "CERT_OCCUPIED"
    MIXED = "MIXED"
    DEFERRED = "DEFERRED"
    REFINED = "REFINED"


class HiPaCOfflineMode(str, Enum):
    """Offline build modes from the plan."""

    QUERY_AGNOSTIC_SCENE_SKELETON = "query_agnostic_scene_skeleton"
    WORKLOAD_AWARE_COMPONENT_CONNECTIVITY = "workload_aware_component_connectivity"
    LIFELONG_ONLINE_REFINEMENT = "lifelong_online_refinement"


@dataclass(frozen=True)
class HiPaCPortalPair:
    pin: Portal
    pout: Portal

    def key(self) -> tuple[str, str]:
        return (self.pin.boundary_box_id, self.pout.boundary_box_id)


@dataclass
class HiPaCCellSummary:
    """Parent-cell connectivity summary.

    A refined mixed domain keeps portal-pair certificates and unresolved
    pairs here instead of exposing every deep child to the global graph.
    """

    cell_id: str
    state: HiPaCCellState
    boundary_ports: list[Portal] = field(default_factory=list)
    certified_portal_pairs: dict[tuple[str, str], str] = field(default_factory=dict)
    unresolved_portal_pairs: set[tuple[str, str]] = field(default_factory=set)
    blockers: tuple[tuple[int, int, str], ...] = ()
    children: list[str] = field(default_factory=list)


@dataclass
class HiPaCCell:
    cell_id: str
    address: DyadicAddress
    box: JointBox
    state: HiPaCCellState
    report: ValidationReport
    parent_id: str | None = None
    depth: int = 0
    refined: bool = False


@dataclass(frozen=True)
class HiPaCEdge:
    source: str
    target: str
    kind: str
    certified: bool
    corridor_id: str | None = None
    cost: float = 1.0


@dataclass
class HiPaCConfig:
    coarse_depth: int = 3
    max_depth: int = 12
    max_refinement_iterations: int = 64
    local_refine_budget: int = 64
    split_schedule: tuple[int, ...] | None = None
    offline_mode: HiPaCOfflineMode = HiPaCOfflineMode.QUERY_AGNOSTIC_SCENE_SKELETON
    component_weight: float = 2.0
    portal_weight: float = 2.0
    query_weight: float = 1.0
    blocker_weight: float = 1.5
    volume_weight: float = 0.10
    depth_weight: float = 0.05
    kin_weight: float = 1.0
    max_optimistic_path_len: int = 64


@dataclass
class HiPaCBuildResult:
    certified_graph_nodes: int
    optimistic_graph_nodes: int
    certified_components: int
    resolved_portal_pairs: int
    unresolved_portal_pairs: int
    refined_mixed_cells: int
    deferred_mixed_cells: int
    validation_counts: Counter[str]
    cell_state_counts: Counter[str]


@dataclass
class HiPaCQueryResult:
    success: bool
    status: str
    certified_path: list[str]
    expanded_path: list[str]
    refined_mixed_cells: int
    unresolved_cells_seen: int
    used_portal_edges: int
    online_repair_s: float


@dataclass
class HiPaCMetrics:
    certified_components: int
    resolved_portal_pairs: int
    unresolved_portal_pairs: int
    connected_anchor_pairs: int
    anchor_pairs: int
    p_attach: float
    p_samecomp: float
    online_mixed_cells_refined: int
    online_repair_s: float


ValidationFn = Callable[[DyadicAddress, JointBox], ValidationReport]


def _cell_id(address: DyadicAddress) -> str:
    return "L" + ",".join(str(x) for x in address.levels) + ":I" + ",".join(str(x) for x in address.indices)


def _state_from_report(report: ValidationReport) -> HiPaCCellState:
    if report.status == ValidationStatus.FREE:
        return HiPaCCellState.FREE
    if report.status == ValidationStatus.CERT_OCCUPIED:
        return HiPaCCellState.CERT_OCCUPIED
    if report.status == ValidationStatus.DEFERRED:
        return HiPaCCellState.DEFERRED
    return HiPaCCellState.MIXED


def _distance_between_boxes(a: JointBox, b: JointBox) -> float:
    ca = box_center(a)
    cb = box_center(b)
    return sum((x - y) ** 2 for x, y in zip(ca, cb)) ** 0.5


def _path_cost(edge_path: list[HiPaCEdge]) -> float:
    return sum(edge.cost for edge in edge_path)


class _DisjointSet:
    def __init__(self) -> None:
        self.parent: dict[str, str] = {}

    def add(self, item: str) -> None:
        self.parent.setdefault(item, item)

    def find(self, item: str) -> str:
        self.add(item)
        parent = self.parent[item]
        if parent != item:
            self.parent[item] = self.find(parent)
        return self.parent[item]

    def union(self, a: str, b: str) -> None:
        ra = self.find(a)
        rb = self.find(b)
        if ra != rb:
            self.parent[rb] = ra


class HierarchicalPartitionConnectivity:
    """Connectivity-driven hierarchical partition refinement.

    The public methods map directly to the plan:

    * ``build``: coarse classification plus component-pair refinement.
    * ``refine_mixed_cell``: domain-local certified child-chain search.
    * ``query``: lazy hierarchical query using certified and optimistic graphs.
    * ``metrics``: connectivity-oriented reporting.
    """

    def __init__(
        self,
        root: JointBox,
        validate: ValidationFn,
        config: HiPaCConfig | None = None,
        workload_anchors: Sequence[Sequence[float]] = (),
        workload_anchor_pairs: Sequence[tuple[int, int]] = (),
    ) -> None:
        self.root = root
        self.validate = validate
        self.config = config or HiPaCConfig()
        self.workload_anchors = tuple(tuple(float(x) for x in point) for point in workload_anchors)
        self.workload_anchor_pairs = tuple(workload_anchor_pairs)
        self.cells: dict[str, HiPaCCell] = {}
        self.summaries: dict[str, HiPaCCellSummary] = {}
        self.cert_adj: dict[str, list[HiPaCEdge]] = {}
        self.opt_adj: dict[str, list[HiPaCEdge]] = {}
        self.portal_corridors: dict[str, PortalCorridor] = {}
        self.validation_counts: Counter[str] = Counter()
        self.refined_mixed_cells = 0
        self.deferred_mixed_cells = 0
        self.online_mixed_cells_refined = 0
        self.online_repair_s = 0.0
        self._corridor_counter = itertools.count()

    def build(self) -> HiPaCBuildResult:
        self._classify_coarse_partition()
        self._rebuild_graphs()
        for _ in range(self.config.max_refinement_iterations):
            pair = self._select_disconnected_component_pair()
            if pair is None:
                break
            source, target = pair
            optimistic = self._shortest_path(self.opt_adj, source, target)
            if not optimistic:
                self._mark_pair_unresolved(source, target)
                continue
            mixed_cell = self._select_mixed_cell_on_path([edge.source for edge in optimistic] + [optimistic[-1].target])
            if mixed_cell is None:
                continue
            result = self.refine_mixed_cell(mixed_cell.cell_id, source, target)
            if result is None:
                self._defer_cell(mixed_cell.cell_id)
            self._rebuild_graphs()
        return self._build_result()

    def query(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        *,
        online_budget: int = 8,
        lifelong_writeback: bool = True,
    ) -> HiPaCQueryResult:
        t0 = time.perf_counter()
        start_id = self.attach_or_refine(start)
        goal_id = self.attach_or_refine(goal)
        if start_id is None or goal_id is None:
            return HiPaCQueryResult(False, "attach_failed", [], [], 0, 0, 0, time.perf_counter() - t0)
        refined = 0
        unresolved_seen = 0
        for _ in range(max(1, online_budget)):
            certified_edges = self._shortest_path(self.cert_adj, start_id, goal_id)
            if certified_edges is not None:
                expanded = self._expand_certified_edge_path(certified_edges)
                elapsed = time.perf_counter() - t0
                if lifelong_writeback or self.config.offline_mode == HiPaCOfflineMode.LIFELONG_ONLINE_REFINEMENT:
                    self.online_mixed_cells_refined += refined
                    self.online_repair_s += elapsed
                return HiPaCQueryResult(
                    True,
                    "certified",
                    [start_id, *[edge.target for edge in certified_edges]],
                    expanded,
                    refined,
                    unresolved_seen,
                    sum(1 for edge in certified_edges if edge.kind == "portal"),
                    elapsed,
                )
            optimistic_edges = self._shortest_path(self.opt_adj, start_id, goal_id)
            if not optimistic_edges:
                break
            optimistic_nodes = [start_id, *[edge.target for edge in optimistic_edges]]
            mixed = self._select_mixed_cell_on_path(optimistic_nodes, query_points=(tuple(start), tuple(goal)))
            if mixed is None:
                break
            unresolved_seen += 1
            entry, exit_ = self._portal_boundary_candidates(mixed.cell_id, preferred_nodes=optimistic_nodes)
            if entry is None or exit_ is None:
                self._defer_cell(mixed.cell_id)
                self._rebuild_graphs()
                continue
            if self.refine_mixed_cell(mixed.cell_id, entry, exit_) is not None:
                refined += 1
            else:
                self._defer_cell(mixed.cell_id)
            self._rebuild_graphs()
        elapsed = time.perf_counter() - t0
        if lifelong_writeback or self.config.offline_mode == HiPaCOfflineMode.LIFELONG_ONLINE_REFINEMENT:
            self.online_mixed_cells_refined += refined
            self.online_repair_s += elapsed
        return HiPaCQueryResult(False, "unresolved", [], [], refined, unresolved_seen, 0, elapsed)

    def attach_or_refine(self, point: Sequence[float]) -> str | None:
        point_tuple = tuple(float(x) for x in point)
        for cell in self.cells.values():
            if cell.state == HiPaCCellState.FREE and box_contains_point(cell.box, point_tuple, tol=1e-12):
                return cell.cell_id
        for cell in self.cells.values():
            if cell.state in {HiPaCCellState.MIXED, HiPaCCellState.DEFERRED} and box_contains_point(cell.box, point_tuple, tol=1e-12):
                levels = tuple(max(level, self.config.max_depth // len(self.root)) for level in cell.address.levels)
                address = jump_cell_containing(self.root, point_tuple, levels)
                report = self.validate(address, address.interval_box(self.root))
                self.validation_counts[report.status.value] += 1
                if report.status == ValidationStatus.FREE:
                    added = self._insert_cell(address, report, parent_id=cell.cell_id)
                    self._rebuild_graphs()
                    return added.cell_id
                return None
        return None

    def refine_mixed_cell(self, cell_id: str, entry_id: str, exit_id: str) -> PortalCorridor | None:
        domain = self.cells.get(cell_id)
        entry = self.cells.get(entry_id)
        exit_ = self.cells.get(exit_id)
        if domain is None or entry is None or exit_ is None:
            return None
        if domain.state not in {HiPaCCellState.MIXED, HiPaCCellState.DEFERRED}:
            return None

        pin = Portal(cell_id, entry_id, self._component_id(entry_id), face_id="entry")
        pout = Portal(cell_id, exit_id, self._component_id(exit_id), face_id="exit")
        summary = self.summaries.setdefault(cell_id, HiPaCCellSummary(cell_id, domain.state))
        summary.boundary_ports = [pin, pout]

        local_cells, local_reports = self._local_refine_for_portals(domain, entry.box, exit_.box)
        chain = self._local_free_chain(local_cells, entry.box, exit_.box)
        if not chain:
            for child in local_cells:
                self._register_local_cell(child)
            summary.unresolved_portal_pairs.add((entry_id, exit_id))
            summary.children.extend(cell.cell_id for cell in local_cells)
            domain.state = HiPaCCellState.REFINED
            summary.state = HiPaCCellState.REFINED
            return None

        corridor_id = f"hipac_portal_{next(self._corridor_counter)}"
        corridor = PortalCorridor(
            corridor_id,
            cell_id,
            pin,
            pout,
            [cell.address for cell in chain],
            [cell.box for cell in chain],
            [local_reports[cell.cell_id] for cell in chain],
            conservative=True,
        )
        if not corridor.validate_certificate(entry.box, exit_.box):
            summary.unresolved_portal_pairs.add((entry_id, exit_id))
            return None
        self.portal_corridors[corridor_id] = corridor
        summary.certified_portal_pairs[(entry_id, exit_id)] = corridor_id
        summary.state = HiPaCCellState.REFINED
        summary.children.extend(cell.cell_id for cell in local_cells)
        domain.state = HiPaCCellState.REFINED
        self.refined_mixed_cells += 1
        self._add_edge(self.cert_adj, entry_id, exit_id, "portal", True, corridor_id, _distance_between_boxes(entry.box, exit_.box))
        self._add_edge(self.opt_adj, entry_id, exit_id, "portal", True, corridor_id, _distance_between_boxes(entry.box, exit_.box))
        return corridor

    def metrics(
        self,
        *,
        probe_points: Sequence[Sequence[float]] = (),
        query_pairs: Sequence[tuple[Sequence[float], Sequence[float]]] = (),
    ) -> HiPaCMetrics:
        components = self._cert_components()
        attach_count = sum(1 for point in probe_points if self._owner_free_cell(point) is not None)
        same_count = 0
        for start, goal in query_pairs:
            a = self._owner_free_cell(start)
            b = self._owner_free_cell(goal)
            if a is not None and b is not None and components.get(a) == components.get(b):
                same_count += 1
        connected_anchor_pairs = 0
        for a, b in self.workload_anchor_pairs:
            if a >= len(self.workload_anchors) or b >= len(self.workload_anchors):
                continue
            ca = self._owner_free_cell(self.workload_anchors[a])
            cb = self._owner_free_cell(self.workload_anchors[b])
            if ca is not None and cb is not None and components.get(ca) == components.get(cb):
                connected_anchor_pairs += 1
        return HiPaCMetrics(
            certified_components=len(set(components.values())),
            resolved_portal_pairs=sum(len(s.certified_portal_pairs) for s in self.summaries.values()),
            unresolved_portal_pairs=sum(len(s.unresolved_portal_pairs) for s in self.summaries.values()),
            connected_anchor_pairs=connected_anchor_pairs,
            anchor_pairs=len(self.workload_anchor_pairs),
            p_attach=attach_count / len(probe_points) if probe_points else 0.0,
            p_samecomp=same_count / len(query_pairs) if query_pairs else 0.0,
            online_mixed_cells_refined=self.online_mixed_cells_refined,
            online_repair_s=self.online_repair_s,
        )

    def _classify_coarse_partition(self) -> None:
        for address in split_schedule_cells(len(self.root), self.config.coarse_depth, self.config.split_schedule):
            box = address.interval_box(self.root)
            report = self.validate(address, box)
            self.validation_counts[report.status.value] += 1
            self._insert_cell(address, report)

    def _insert_cell(
        self,
        address: DyadicAddress,
        report: ValidationReport,
        *,
        parent_id: str | None = None,
    ) -> HiPaCCell:
        cell_id = _cell_id(address)
        state = _state_from_report(report)
        cell = HiPaCCell(cell_id, address, address.interval_box(self.root), state, report, parent_id, address.depth)
        self.cells[cell_id] = cell
        self.summaries[cell_id] = HiPaCCellSummary(
            cell_id,
            state,
            blockers=report.blocker_signature(),
        )
        return cell

    def _register_local_cell(self, cell: HiPaCCell) -> None:
        self.cells[cell.cell_id] = cell
        self.summaries[cell.cell_id] = HiPaCCellSummary(
            cell.cell_id,
            cell.state,
            blockers=cell.report.blocker_signature(),
        )

    def _rebuild_graphs(self) -> None:
        self.cert_adj = {}
        self.opt_adj = {}
        active = [cell for cell in self.cells.values() if cell.state != HiPaCCellState.REFINED]
        for cell in active:
            if cell.state == HiPaCCellState.FREE:
                self.cert_adj.setdefault(cell.cell_id, [])
                self.opt_adj.setdefault(cell.cell_id, [])
            elif cell.state in {HiPaCCellState.MIXED, HiPaCCellState.DEFERRED}:
                self.opt_adj.setdefault(cell.cell_id, [])
        for i, lhs in enumerate(active):
            for rhs in active[i + 1 :]:
                if not boxes_touch_or_overlap(lhs.box, rhs.box, tol=1e-12):
                    continue
                cost = _distance_between_boxes(lhs.box, rhs.box)
                if lhs.state == rhs.state == HiPaCCellState.FREE:
                    self._add_edge(self.cert_adj, lhs.cell_id, rhs.cell_id, "free_adjacency", True, None, cost)
                    self._add_edge(self.opt_adj, lhs.cell_id, rhs.cell_id, "free_adjacency", True, None, cost)
                elif lhs.state in {HiPaCCellState.FREE, HiPaCCellState.MIXED, HiPaCCellState.DEFERRED} and rhs.state in {
                    HiPaCCellState.FREE,
                    HiPaCCellState.MIXED,
                    HiPaCCellState.DEFERRED,
                }:
                    self._add_edge(self.opt_adj, lhs.cell_id, rhs.cell_id, "optimistic_adjacency", False, None, cost + 1.0)
        for summary in self.summaries.values():
            for (source, target), corridor_id in summary.certified_portal_pairs.items():
                source_cell = self.cells.get(source)
                target_cell = self.cells.get(target)
                if source_cell is None or target_cell is None:
                    continue
                cost = _distance_between_boxes(source_cell.box, target_cell.box)
                self._add_edge(self.cert_adj, source, target, "portal", True, corridor_id, cost)
                self._add_edge(self.opt_adj, source, target, "portal", True, corridor_id, cost)

    @staticmethod
    def _add_edge(
        graph: dict[str, list[HiPaCEdge]],
        source: str,
        target: str,
        kind: str,
        certified: bool,
        corridor_id: str | None,
        cost: float,
    ) -> None:
        graph.setdefault(source, []).append(HiPaCEdge(source, target, kind, certified, corridor_id, cost))
        graph.setdefault(target, []).append(HiPaCEdge(target, source, kind, certified, corridor_id, cost))

    def _cert_components(self) -> dict[str, str]:
        dsu = _DisjointSet()
        for node in self.cert_adj:
            dsu.add(node)
        for source, edges in self.cert_adj.items():
            for edge in edges:
                dsu.union(source, edge.target)
        return {node: dsu.find(node) for node in self.cert_adj}

    def _select_disconnected_component_pair(self) -> tuple[str, str] | None:
        free_nodes = sorted(self.cert_adj)
        if len(free_nodes) < 2:
            return None
        components = self._cert_components()
        candidate_pairs: list[tuple[float, str, str]] = []
        anchor_pairs = self._anchor_cell_pairs()
        if self.config.offline_mode == HiPaCOfflineMode.WORKLOAD_AWARE_COMPONENT_CONNECTIVITY and anchor_pairs:
            for source, target in anchor_pairs:
                if components.get(source) != components.get(target):
                    candidate_pairs.append((-10.0, source, target))
        for i, source in enumerate(free_nodes):
            for target in free_nodes[i + 1 :]:
                if components.get(source) == components.get(target):
                    continue
                optimistic = self._shortest_path(self.opt_adj, source, target)
                if optimistic is None:
                    continue
                candidate_pairs.append((_path_cost(optimistic), source, target))
        if not candidate_pairs:
            return None
        _, source, target = min(candidate_pairs, key=lambda item: item[0])
        return source, target

    def _anchor_cell_pairs(self) -> list[tuple[str, str]]:
        pairs: list[tuple[str, str]] = []
        owners = [self._owner_free_cell(point) for point in self.workload_anchors]
        for a, b in self.workload_anchor_pairs:
            if a < len(owners) and b < len(owners) and owners[a] is not None and owners[b] is not None:
                pairs.append((owners[a], owners[b]))  # type: ignore[arg-type]
        return pairs

    def _owner_free_cell(self, point: Sequence[float]) -> str | None:
        for cell in self.cells.values():
            if cell.state == HiPaCCellState.FREE and box_contains_point(cell.box, point, tol=1e-12):
                return cell.cell_id
        return None

    def _shortest_path(self, graph: dict[str, list[HiPaCEdge]], source: str, target: str) -> list[HiPaCEdge] | None:
        if source == target:
            return []
        if source not in graph or target not in graph:
            return None
        queue: list[tuple[float, str]] = [(0.0, source)]
        previous: dict[str, tuple[str, HiPaCEdge]] = {}
        best: dict[str, float] = {source: 0.0}
        while queue:
            cost, node = heapq.heappop(queue)
            if node == target:
                break
            if cost > best.get(node, float("inf")) + 1e-12:
                continue
            for edge in graph.get(node, []):
                next_cost = cost + edge.cost
                if next_cost < best.get(edge.target, float("inf")):
                    best[edge.target] = next_cost
                    previous[edge.target] = (node, edge)
                    heapq.heappush(queue, (next_cost, edge.target))
        if target not in previous:
            return None
        out: list[HiPaCEdge] = []
        node = target
        while node != source:
            prev, edge = previous[node]
            out.append(edge)
            node = prev
        out.reverse()
        return out

    def _select_mixed_cell_on_path(
        self,
        nodes: list[str],
        *,
        query_points: tuple[tuple[float, ...], tuple[float, ...]] | None = None,
    ) -> HiPaCCell | None:
        candidates = [
            self.cells[node]
            for node in nodes
            if node in self.cells and self.cells[node].state in {HiPaCCellState.MIXED, HiPaCCellState.DEFERRED}
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda cell: self._mixed_score(cell, query_points=query_points))

    def _mixed_score(
        self,
        cell: HiPaCCell,
        *,
        query_points: tuple[tuple[float, ...], tuple[float, ...]] | None = None,
    ) -> float:
        adjacent_free_components = {
            self._component_id(other.cell_id)
            for other in self.cells.values()
            if other.state == HiPaCCellState.FREE and boxes_touch_or_overlap(cell.box, other.box, tol=1e-12)
        }
        component_score = len(adjacent_free_components)
        portal_score = 1.0 if component_score >= 2 else 0.0
        query_score = 0.0
        if query_points:
            center = box_center(cell.box)
            query_score = -min(sum((a - b) ** 2 for a, b in zip(center, point)) for point in query_points)
        blocker_score = cell.report.overlap_score + len(cell.report.blockers)
        return (
            self.config.component_weight * component_score
            + self.config.portal_weight * portal_score
            + self.config.query_weight * query_score
            - self.config.blocker_weight * blocker_score
            + self.config.volume_weight * log(max(box_volume(cell.box), 1e-300))
            - self.config.depth_weight * cell.depth
        )

    def _component_id(self, cell_id: str) -> str:
        return self._cert_components().get(cell_id, cell_id)

    def _portal_boundary_candidates(
        self,
        mixed_id: str,
        *,
        preferred_nodes: Iterable[str] = (),
    ) -> tuple[str | None, str | None]:
        mixed = self.cells[mixed_id]
        preferred = [
            node
            for node in preferred_nodes
            if node in self.cells
            and self.cells[node].state == HiPaCCellState.FREE
            and boxes_touch_or_overlap(mixed.box, self.cells[node].box, tol=1e-12)
        ]
        candidates = preferred or [
            cell.cell_id
            for cell in self.cells.values()
            if cell.state == HiPaCCellState.FREE and boxes_touch_or_overlap(mixed.box, cell.box, tol=1e-12)
        ]
        if len(candidates) < 2:
            return (candidates[0], None) if candidates else (None, None)
        components = self._cert_components()
        for i, source in enumerate(candidates):
            for target in candidates[i + 1 :]:
                if components.get(source) != components.get(target):
                    return source, target
        return candidates[0], candidates[-1]

    def _local_refine_for_portals(
        self,
        domain: HiPaCCell,
        entry_box: JointBox,
        exit_box: JointBox,
    ) -> tuple[list[HiPaCCell], dict[str, ValidationReport]]:
        queue: list[tuple[float, int, DyadicAddress]] = []
        counter = itertools.count()
        for child in domain.address.split(self._choose_split_dimension(domain)):
            heapq.heappush(queue, (self._local_priority(child, entry_box, exit_box), next(counter), child))
        created: list[HiPaCCell] = []
        reports: dict[str, ValidationReport] = {}
        budget = max(1, self.config.local_refine_budget)
        while queue and budget > 0:
            _, _, address = heapq.heappop(queue)
            budget -= 1
            report = self.validate(address, address.interval_box(self.root))
            self.validation_counts[report.status.value] += 1
            state = _state_from_report(report)
            cell = HiPaCCell(
                _cell_id(address),
                address,
                address.interval_box(self.root),
                state,
                report,
                parent_id=domain.cell_id,
                depth=address.depth,
            )
            created.append(cell)
            reports[cell.cell_id] = report
            if report.status == ValidationStatus.FREE:
                continue
            if report.status == ValidationStatus.CERT_OCCUPIED:
                continue
            if address.depth >= self.config.max_depth:
                continue
            split_dim = self._choose_split_dimension(cell)
            for child in address.split(split_dim):
                heapq.heappush(queue, (self._local_priority(child, entry_box, exit_box), next(counter), child))
        return created, reports

    def _choose_split_dimension(self, cell: HiPaCCell) -> int:
        scores = [self.config.kin_weight * interval.width for interval in cell.box]
        for blocker in cell.report.blockers:
            for joint in blocker.affected_joints:
                if 0 <= joint < len(scores):
                    scores[joint] += self.config.blocker_weight * (1.0 + blocker.overlap_score)
        return max(range(len(scores)), key=lambda dim: (scores[dim], -dim))

    def _local_priority(self, address: DyadicAddress, entry_box: JointBox, exit_box: JointBox) -> float:
        box = address.interval_box(self.root)
        d_entry = _distance_between_boxes(box, entry_box)
        d_exit = _distance_between_boxes(box, exit_box)
        return d_entry + d_exit - self.config.volume_weight * log(max(box_volume(box), 1e-300))

    def _local_free_chain(
        self,
        local_cells: list[HiPaCCell],
        entry_box: JointBox,
        exit_box: JointBox,
    ) -> list[HiPaCCell] | None:
        free = [cell for cell in local_cells if cell.state == HiPaCCellState.FREE]
        starts = [cell.cell_id for cell in free if boxes_touch_or_overlap(entry_box, cell.box, tol=1e-12)]
        goals = {cell.cell_id for cell in free if boxes_touch_or_overlap(exit_box, cell.box, tol=1e-12)}
        if boxes_touch_or_overlap(entry_box, exit_box, tol=1e-12):
            return []
        if not starts or not goals:
            return None
        by_id = {cell.cell_id: cell for cell in free}
        adj: dict[str, list[str]] = {cell.cell_id: [] for cell in free}
        for i, lhs in enumerate(free):
            for rhs in free[i + 1 :]:
                if boxes_touch_or_overlap(lhs.box, rhs.box, tol=1e-12):
                    adj[lhs.cell_id].append(rhs.cell_id)
                    adj[rhs.cell_id].append(lhs.cell_id)
        queue = deque(starts)
        previous: dict[str, str | None] = {start: None for start in starts}
        target: str | None = None
        while queue:
            node = queue.popleft()
            if node in goals:
                target = node
                break
            for neighbor in adj[node]:
                if neighbor not in previous:
                    previous[neighbor] = node
                    queue.append(neighbor)
        if target is None:
            return None
        path_ids: list[str] = []
        node: str | None = target
        while node is not None:
            path_ids.append(node)
            node = previous[node]
        path_ids.reverse()
        return [by_id[node] for node in path_ids]

    def _defer_cell(self, cell_id: str) -> None:
        cell = self.cells.get(cell_id)
        if cell is None or cell.state not in {HiPaCCellState.MIXED, HiPaCCellState.DEFERRED}:
            return
        cell.state = HiPaCCellState.DEFERRED
        self.summaries[cell_id].state = HiPaCCellState.DEFERRED
        self.deferred_mixed_cells += 1

    def _mark_pair_unresolved(self, source: str, target: str) -> None:
        for cell in self.cells.values():
            if cell.state in {HiPaCCellState.MIXED, HiPaCCellState.DEFERRED}:
                summary = self.summaries[cell.cell_id]
                summary.unresolved_portal_pairs.add((source, target))
                return

    def _expand_certified_edge_path(self, edges: list[HiPaCEdge]) -> list[str]:
        if not edges:
            return []
        expanded = [edges[0].source]
        for edge in edges:
            if edge.kind == "portal" and edge.corridor_id in self.portal_corridors:
                corridor_nodes = self.portal_corridors[edge.corridor_id].expanded_box_ids()
                if corridor_nodes[0] != expanded[-1]:
                    corridor_nodes.reverse()
                expanded.extend(corridor_nodes[1:])
            else:
                expanded.append(edge.target)
        return expanded

    def _build_result(self) -> HiPaCBuildResult:
        components = self._cert_components()
        return HiPaCBuildResult(
            certified_graph_nodes=len(self.cert_adj),
            optimistic_graph_nodes=len(self.opt_adj),
            certified_components=len(set(components.values())),
            resolved_portal_pairs=sum(len(summary.certified_portal_pairs) for summary in self.summaries.values()),
            unresolved_portal_pairs=sum(len(summary.unresolved_portal_pairs) for summary in self.summaries.values()),
            refined_mixed_cells=self.refined_mixed_cells,
            deferred_mixed_cells=self.deferred_mixed_cells,
            validation_counts=Counter(self.validation_counts),
            cell_state_counts=Counter(cell.state.value for cell in self.cells.values()),
        )
