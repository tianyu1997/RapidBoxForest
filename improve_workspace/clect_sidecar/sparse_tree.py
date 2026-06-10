"""Sparse Patricia-style runtime overlay for dyadic cells."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable

from .dyadic import DyadicAddress, JointBox, jump_cell_containing


class SparseNodeStatus(str, Enum):
    VIRTUAL = "virtual"
    MATERIALIZED = "materialized"
    TERMINAL = "terminal"


@dataclass
class SparseLectNode:
    address: DyadicAddress
    status: SparseNodeStatus = SparseNodeStatus.VIRTUAL
    evidence_ref: str | None = None
    scene_record_ref: str | None = None
    split_dim: int | None = None


class SparseNodeMap:
    """Store only materialized or terminal cells.

    This models the Stage-1/Stage-2 Patricia overlay: a deep cell can be
    materialized directly by dyadic address without creating all heap ancestors.
    """

    def __init__(self, dims: int) -> None:
        self.dims = dims
        self._nodes: dict[DyadicAddress, SparseLectNode] = {}

    def __len__(self) -> int:
        return len(self._nodes)

    def __contains__(self, address: DyadicAddress) -> bool:
        return address in self._nodes

    def materialize(
        self,
        address: DyadicAddress,
        evidence_ref: str | None = None,
        terminal: bool = False,
    ) -> SparseLectNode:
        if address.dims != self.dims:
            raise ValueError("dimension mismatch")
        status = SparseNodeStatus.TERMINAL if terminal else SparseNodeStatus.MATERIALIZED
        node = self._nodes.get(address)
        if node is None:
            node = SparseLectNode(address=address, status=status, evidence_ref=evidence_ref)
            self._nodes[address] = node
        else:
            node.status = status
            if evidence_ref is not None:
                node.evidence_ref = evidence_ref
        return node

    def jump_materialize(
        self,
        root: JointBox,
        point: tuple[float, ...],
        levels: tuple[int, ...],
        evidence_ref: str | None = None,
        terminal: bool = False,
    ) -> SparseLectNode:
        return self.materialize(jump_cell_containing(root, point, levels), evidence_ref, terminal)

    def terminal_nodes(self) -> list[SparseLectNode]:
        return [node for node in self._nodes.values() if node.status == SparseNodeStatus.TERMINAL]

    def ancestors_present(self, address: DyadicAddress) -> list[DyadicAddress]:
        out: list[DyadicAddress] = []
        for candidate in self._nodes:
            if candidate != address and candidate.is_ancestor_of(address):
                out.append(candidate)
        return out

    def iter_nodes(self) -> Iterable[SparseLectNode]:
        return self._nodes.values()

