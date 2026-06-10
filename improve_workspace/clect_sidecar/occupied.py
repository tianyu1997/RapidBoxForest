"""Optional signed-distance occupied pruning helpers.

These helpers implement the optional lemma from `docs/improve.md` as a
standalone predicate.  They do not assume that the production checker exposes a
signed-distance witness; callers must provide the witness values explicitly.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sin
from typing import Sequence

from .reports import Blocker, ValidationReport


@dataclass(frozen=True)
class MaterialWitness:
    link_id: int
    obstacle_id: int
    center_signed_distance: float
    motion_bound: float
    epsilon_num: float = 1e-9

    @property
    def certifies_occupied(self) -> bool:
        return self.center_signed_distance + self.motion_bound + self.epsilon_num < 0.0


def revolute_motion_bound(lever_arms: Sequence[float], half_widths: Sequence[float]) -> float:
    """Return sum_j 2 R_j sin(Delta_j / 2)."""

    if len(lever_arms) != len(half_widths):
        raise ValueError("lever arm and half-width dimensions differ")
    return sum(2.0 * max(0.0, r) * sin(max(0.0, delta) * 0.5) for r, delta in zip(lever_arms, half_widths))


def linearized_motion_bound(lever_arms: Sequence[float], half_widths: Sequence[float]) -> float:
    """Conservative small-angle fallback sum_j R_j Delta_j."""

    if len(lever_arms) != len(half_widths):
        raise ValueError("lever arm and half-width dimensions differ")
    return sum(max(0.0, r) * max(0.0, delta) for r, delta in zip(lever_arms, half_widths))


def occupied_report_from_witness(witness: MaterialWitness, blocker: Blocker | None = None) -> ValidationReport | None:
    """Return a `CERT_OCCUPIED` report when the witness is sound."""

    if not witness.certifies_occupied:
        return None
    blockers = [blocker] if blocker is not None else []
    cert = (
        f"sdf_witness:link={witness.link_id}:obstacle={witness.obstacle_id}:"
        f"phi={witness.center_signed_distance:.17g}:rho={witness.motion_bound:.17g}:"
        f"eps={witness.epsilon_num:.17g}"
    )
    return ValidationReport.cert_occupied(cert, blockers)

