"""Validation reports and blocker signatures for adaptive refinement."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class ValidationStatus(str, Enum):
    FREE = "FREE"
    FAIL = "FAIL"
    CERT_OCCUPIED = "CERT_OCCUPIED"
    INCONCLUSIVE = "INCONCLUSIVE"
    DEFERRED = "DEFERRED"
    COVERED = "COVERED"


class FailStage(str, Enum):
    AABB = "AABB"
    GJK = "GJK"
    SDF = "SDF"
    MIXED = "MIXED"
    NONE = "NONE"


@dataclass(frozen=True)
class Blocker:
    link_id: int
    obstacle_id: int
    stage: FailStage
    margin: float = 0.0
    overlap_score: float = 0.0
    affected_joints: tuple[int, ...] = ()

    def signature_atom(self) -> tuple[int, int, str]:
        return (self.link_id, self.obstacle_id, self.stage.value)


@dataclass
class ValidationReport:
    status: ValidationStatus
    blockers: list[Blocker] = field(default_factory=list)
    min_margin: float = 0.0
    overlap_score: float = 0.0
    fail_stage: FailStage = FailStage.NONE
    occupied_certificate: str | None = None
    free_certificate: str | None = None

    def blocker_signature(self, top_k: int = 3) -> tuple[tuple[int, int, str], ...]:
        blockers = sorted(
            self.blockers,
            key=lambda item: (item.overlap_score, -abs(item.margin)),
            reverse=True,
        )
        return tuple(blocker.signature_atom() for blocker in blockers[:top_k])

    @classmethod
    def free(cls, certificate: str | None = None) -> "ValidationReport":
        return cls(status=ValidationStatus.FREE, free_certificate=certificate)

    @classmethod
    def fail(cls, blockers: list[Blocker], overlap_score: float = 0.0) -> "ValidationReport":
        stage = blockers[0].stage if blockers else FailStage.MIXED
        return cls(
            status=ValidationStatus.FAIL,
            blockers=blockers,
            overlap_score=overlap_score,
            fail_stage=stage,
        )

    @classmethod
    def cert_occupied(cls, certificate: str, blockers: list[Blocker] | None = None) -> "ValidationReport":
        return cls(
            status=ValidationStatus.CERT_OCCUPIED,
            blockers=list(blockers or []),
            occupied_certificate=certificate,
            fail_stage=FailStage.SDF,
        )

