from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterator


CANONICAL_ANCHORS: dict[str, tuple[float, ...]] = {
    "C": (0.0, 0.2, 0.0, -2.09, 0.0, -0.3, 1.5707963267948966),
    "L": (0.8, 0.7, 0.0, -1.6, 0.0, 0.0, 1.5707963267948966),
    "R": (-0.8, 0.7, 0.0, -1.6, 0.0, 0.0, 1.5707963267948966),
    "AS": (6.42e-05, 0.4719533, -0.0001493, -0.6716735, 0.0001854, 0.4261696, 1.5706922),
    "TS": (-1.55e-04, 0.3972726, 0.0002196, -1.3674756, 0.0002472, -0.1929518, 1.5704688),
    "CS": (-1.76e-04, 0.6830279, 0.0002450, -1.6478229, 2.09e-05, -0.7590545, 1.5706263),
    "LB": (1.3326656, 0.7865932, 0.3623384, -1.4916529, -0.3192509, 0.9217325, 1.7911904),
    "RB": (-1.3324624, 0.7866478, -0.3626562, -1.4916528, 0.3195340, 0.9217833, 1.3502090),
}

CANONICAL_QUERY_ANCHORS: dict[str, tuple[str, str]] = {
    "AS->TS": ("AS", "TS"),
    "TS->CS": ("TS", "CS"),
    "CS->LB": ("CS", "LB"),
    "LB->RB": ("LB", "RB"),
    "RB->AS": ("RB", "AS"),
}

LEGACY_JOINT0_OFFSET = math.pi / 2.0
LEGACY_LB_JOINT0_VALUE = -0.1507344
LEGACY_RB_JOINT0_VALUE = 0.1509376
DEFAULT_ATOL = 5e-4


def _vector_close(lhs: list[Any] | tuple[Any, ...], rhs: tuple[float, ...], *, atol: float) -> bool:
    if len(lhs) != len(rhs):
        return False
    return all(abs(float(lhs[index]) - float(rhs[index])) <= atol for index in range(len(rhs)))


def _legacy_joint0_offset(lhs: list[Any] | tuple[Any, ...], rhs: tuple[float, ...], *, atol: float) -> bool:
    if len(lhs) != len(rhs):
        return False
    if abs((float(lhs[0]) - float(rhs[0])) - LEGACY_JOINT0_OFFSET) > atol:
        return False
    return all(abs(float(lhs[index]) - float(rhs[index])) <= atol for index in range(1, len(rhs)))


def _legacy_lb_joint0_shift(anchor_name: str, lhs: list[Any] | tuple[Any, ...], rhs: tuple[float, ...], *, atol: float) -> bool:
    if anchor_name != "LB" or len(lhs) != len(rhs):
        return False
    if abs(float(lhs[0]) - LEGACY_LB_JOINT0_VALUE) > atol:
        return False
    return all(abs(float(lhs[index]) - float(rhs[index])) <= atol for index in range(1, len(rhs)))


def _legacy_rb_joint0_shift(anchor_name: str, lhs: list[Any] | tuple[Any, ...], rhs: tuple[float, ...], *, atol: float) -> bool:
    if anchor_name != "RB" or len(lhs) != len(rhs):
        return False
    if abs(float(lhs[0]) - LEGACY_RB_JOINT0_VALUE) > atol:
        return False
    return all(abs(float(lhs[index]) - float(rhs[index])) <= atol for index in range(1, len(rhs)))


def _iter_query_payloads(obj: Any, path: str = "$") -> Iterator[tuple[str, dict[str, Any]]]:
    if isinstance(obj, dict):
        if isinstance(obj.get("name"), str) and isinstance(obj.get("waypoints"), list):
            yield path, obj
        for key, value in obj.items():
            yield from _iter_query_payloads(value, f"{path}.{key}")
        return
    if isinstance(obj, list):
        for index, value in enumerate(obj):
            yield from _iter_query_payloads(value, f"{path}[{index}]")


def _expected_anchor_names(query_payload: dict[str, Any]) -> tuple[str, str] | None:
    label = str(query_payload.get("name", ""))
    if label in CANONICAL_QUERY_ANCHORS:
        return CANONICAL_QUERY_ANCHORS[label]
    start_name = str(query_payload.get("from", ""))
    goal_name = str(query_payload.get("to", ""))
    if start_name in CANONICAL_ANCHORS and goal_name in CANONICAL_ANCHORS:
        return start_name, goal_name
    return None


def _check_runtime_anchor_sync() -> None:
    try:
        import sbf  # type: ignore
    except Exception:
        return
    runtime_anchors = getattr(sbf, "ANCHORS", None)
    if not isinstance(runtime_anchors, dict):
        return
    for name, expected in CANONICAL_ANCHORS.items():
        actual = runtime_anchors.get(name)
        if actual is None or not _vector_close(actual, expected, atol=1e-12):
            raise RuntimeError(f"canonical anchor guard drifted from sbf.ANCHORS for {name}")


def anchor_validation_report(payload: dict[str, Any], *, atol: float = DEFAULT_ATOL) -> dict[str, Any]:
    _check_runtime_anchor_sync()
    findings: list[dict[str, Any]] = []
    queries_checked = 0
    for path, query_payload in _iter_query_payloads(payload):
        anchor_names = _expected_anchor_names(query_payload)
        if anchor_names is None:
            continue
        waypoints = query_payload.get("waypoints")
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            continue
        start_waypoint = waypoints[0]
        goal_waypoint = waypoints[-1]
        if not isinstance(start_waypoint, list) or not isinstance(goal_waypoint, list):
            continue
        queries_checked += 1
        for endpoint_kind, anchor_name, observed in (
            ("start", anchor_names[0], start_waypoint),
            ("goal", anchor_names[1], goal_waypoint),
        ):
            expected = CANONICAL_ANCHORS[anchor_name]
            if _vector_close(observed, expected, atol=atol):
                continue
            finding = {
                "path": path,
                "query": str(query_payload.get("name", "")),
                "endpoint": endpoint_kind,
                "anchor_name": anchor_name,
                "expected": list(expected),
                "observed": [float(value) for value in observed],
                "delta_joint0": float(observed[0]) - float(expected[0]),
            }
            if _legacy_joint0_offset(observed, expected, atol=atol):
                finding["kind"] = "legacy_joint0_pi_over_2"
            elif _legacy_lb_joint0_shift(anchor_name, observed, expected, atol=atol):
                finding["kind"] = "legacy_lb_joint0_shift"
            elif _legacy_rb_joint0_shift(anchor_name, observed, expected, atol=atol):
                finding["kind"] = "legacy_rb_joint0_shift"
            else:
                finding["kind"] = "anchor_mismatch"
            findings.append(finding)
    return {
        "status": "passed" if not findings else "failed",
        "queries_checked": int(queries_checked),
        "findings": findings,
        "legacy_joint0_pi_over_2_count": int(sum(1 for item in findings if item["kind"] == "legacy_joint0_pi_over_2")),
    }


def validate_marcucci_query_artifact(payload: dict[str, Any], *, artifact_path: Path | None = None, atol: float = DEFAULT_ATOL) -> dict[str, Any]:
    report = anchor_validation_report(payload, atol=atol)
    report["artifact_path"] = str(artifact_path) if artifact_path is not None else None
    if report["findings"]:
        preview = "; ".join(
            f"{item['query']}:{item['endpoint']}:{item['kind']}@{item['path']}"
            for item in report["findings"][:4]
        )
        raise ValueError(f"Marcucci anchor validation failed for {artifact_path or '<memory>'}: {preview}")
    return report