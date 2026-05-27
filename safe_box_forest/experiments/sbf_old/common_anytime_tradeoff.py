from __future__ import annotations

import math
import statistics
from typing import Any, Iterable


EPS_PATH_DEFAULT = 1e-6
EPS_ENDPOINT_DEFAULT = 1e-6
EPS_TIME_MONOTONE_DEFAULT = 1e-3
FINAL_OMPL_SIMPLIFY_SEGMENT_STEP = 0.01
SUCCESS_ONLY_STAGE_PROTOCOLS = {
    "single_run_max_timeout",
}


def mean(values: Iterable[float | None]) -> float | None:
    rows = [float(value) for value in values if value is not None]
    return statistics.fmean(rows) if rows else None


def median(values: Iterable[float | None]) -> float | None:
    rows = [float(value) for value in values if value is not None]
    return statistics.median(rows) if rows else None


def euclidean_path_length(path: list[list[float]]) -> float:
    total = 0.0
    for index in range(len(path) - 1):
        total += math.sqrt(sum((a - b) * (a - b) for a, b in zip(path[index], path[index + 1])))
    return total


def point_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) * (float(x) - float(y)) for x, y in zip(a, b)))


def interpolate(a: list[float], b: list[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def path_passes_post_audit(
    sbf_module: Any,
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    *,
    segment_step: float,
    start: list[float] | None = None,
    goal: list[float] | None = None,
    endpoint_tol: float = EPS_ENDPOINT_DEFAULT,
) -> bool:
    points = [list(point) for point in path]
    if len(points) < 2:
        return False
    if start is not None and point_distance(points[0], list(start)) > float(endpoint_tol):
        return False
    if goal is not None and point_distance(points[-1], list(goal)) > float(endpoint_tol):
        return False
    max_step = max(float(segment_step), 1e-9)
    for index in range(len(points) - 1):
        pieces = max(1, int(math.ceil(point_distance(points[index], points[index + 1]) / max_step)))
        for item in range(pieces + 1):
            q = interpolate(points[index], points[index + 1], item / pieces)
            if bool(sbf_module.check_config_collision(robot, obstacles, q)):
                return False
    return True


def final_ompl_simplify_path(
    sbf_module: Any,
    robot: Any,
    obstacles: list[Any],
    path: list[list[float]],
    *,
    segment_step: float,
    audit_segment_step: float | None = None,
    simplify_time_s: float,
    epsilon_path: float = EPS_PATH_DEFAULT,
) -> dict[str, Any]:
    points = [list(point) for point in path]
    if len(points) < 2:
        return {
            "path": points,
            "path_length": None,
            "query_s": 0.0,
            "applied": False,
            "reason": "path_too_short",
        }
    baseline_length = euclidean_path_length(points)
    if float(simplify_time_s) <= 0.0:
        return {
            "path": points,
            "path_length": baseline_length,
            "query_s": 0.0,
            "applied": False,
            "reason": "disabled",
        }
    simplify_segment_step = FINAL_OMPL_SIMPLIFY_SEGMENT_STEP
    raw = dict(sbf_module.ompl_simplify_path(robot, obstacles, points, simplify_segment_step, float(simplify_time_s)))
    simplify_s = float(raw.get("t_s", 0.0) or 0.0)
    simplified = [list(point) for point in raw.get("path", [])]
    audit_step = float(segment_step if audit_segment_step is None else audit_segment_step)
    if bool(raw.get("ok")) and len(simplified) >= 2:
        simplified_length = euclidean_path_length(simplified)
        if (
            simplified_length <= baseline_length + float(epsilon_path)
            and path_passes_post_audit(
                sbf_module,
                robot,
                obstacles,
                simplified,
                segment_step=audit_step,
                start=points[0],
                goal=points[-1],
            )
        ):
            return {
                "path": simplified,
                "path_length": simplified_length,
                "query_s": simplify_s,
                "applied": simplified_length < baseline_length - float(epsilon_path),
                "reason": "accepted",
                "segment_step": simplify_segment_step,
            }
    return {
        "path": points,
        "path_length": baseline_length,
        "query_s": simplify_s,
        "applied": False,
        "reason": "kept_original",
        "segment_step": simplify_segment_step,
    }


def task_result(
    *,
    name: str,
    ok: bool,
    audit_passed: bool,
    path_length: float | None,
    query_s: float,
    reason: str | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    row = {
        "name": str(name),
        "ok": bool(ok),
        "audit_passed": bool(audit_passed),
        "path_length": None if path_length is None else float(path_length),
        "query_s": float(query_s),
        "reason": reason,
    }
    if extra:
        row.update(extra)
    return row


def update_incumbents(
    incumbents: dict[str, dict[str, Any]],
    candidates: list[dict[str, Any]],
    *,
    epsilon_path: float = EPS_PATH_DEFAULT,
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    updated = dict(incumbents)
    improved: list[str] = []
    for candidate in candidates:
        name = str(candidate.get("name"))
        if not bool(candidate.get("audit_passed")):
            continue
        length = candidate.get("path_length")
        if length is None:
            continue
        previous = updated.get(name)
        if previous is None or float(length) < float(previous["path_length"]) - float(epsilon_path):
            updated[name] = dict(candidate)
            improved.append(name)
    return updated, improved


def incumbent_stage_record(
    *,
    method: str,
    stage_id: str,
    stage_index: int,
    seed_index: int,
    task_count: int,
    cumulative_build_s: float,
    cumulative_query_s: float,
    stage_build_s: float,
    stage_query_s: float,
    raw_tasks: list[dict[str, Any]],
    incumbents: dict[str, dict[str, Any]],
    improved_tasks: list[str],
    params: dict[str, Any],
    protocol: str,
) -> dict[str, Any]:
    incumbent_rows = [dict(row) for row in incumbents.values()]
    incumbent_total_length = sum(float(row["path_length"]) for row in incumbent_rows if row.get("path_length") is not None)
    audit_count = len(incumbent_rows)
    incumbent_mean_length = incumbent_total_length / audit_count if audit_count else None
    return {
        "method": str(method),
        "stage_id": str(stage_id),
        "stage_index": int(stage_index),
        "seed_index": int(seed_index),
        "protocol": str(protocol),
        "stage_build_s": float(stage_build_s),
        "stage_query_s": float(stage_query_s),
        "stage_total_s": float(stage_build_s) + float(stage_query_s),
        "cumulative_build_s": float(cumulative_build_s),
        "cumulative_query_s": float(cumulative_query_s),
        "cumulative_total_s": float(cumulative_build_s) + float(cumulative_query_s),
        "task_count": int(task_count),
        "incumbent_success_count": int(audit_count),
        "incumbent_audit_sr": float(audit_count) / max(1, int(task_count)),
        "incumbent_total_length": float(incumbent_total_length) if audit_count else None,
        "incumbent_mean_length": float(incumbent_mean_length) if incumbent_mean_length is not None else None,
        "raw_tasks": raw_tasks,
        "incumbent_tasks": incumbent_rows,
        "improved_tasks": list(improved_tasks),
        "improved": bool(improved_tasks),
        "params": dict(params),
    }


def aggregate_stage_records(
    records: list[dict[str, Any]],
    *,
    epsilon_path: float = EPS_PATH_DEFAULT,
) -> dict[str, Any]:
    grouped: dict[tuple[str, int, str], list[tuple[int, dict[str, Any]]]] = {}
    for source_index, record in enumerate(records):
        key = (str(record["method"]), int(record["stage_index"]), str(record["stage_id"]))
        grouped.setdefault(key, []).append((source_index, record))

    all_points: list[dict[str, Any]] = []
    best_by_method: dict[str, tuple[float, float]] = {}
    for (method, stage_index, stage_id), indexed_rows in sorted(grouped.items(), key=lambda item: (item[0][0], item[0][1])):
        rows = [row for _, row in indexed_rows]
        task_total = sum(int(row.get("task_count", 0)) for row in rows)
        success_only_stage = all(str(row.get("protocol")) in SUCCESS_ONLY_STAGE_PROTOCOLS for row in rows)
        if success_only_stage:
            success_total = 0
            mean_lengths = []
            total_lengths = []
            success_query_times = []
            for row in rows:
                successes = [
                    task for task in row.get("raw_tasks", [])
                    if bool(task.get("audit_passed")) and task.get("path_length") is not None
                ]
                success_total += len(successes)
                if successes:
                    mean_lengths.append(mean(task.get("path_length") for task in successes))
                    total_lengths.append(sum(float(task.get("path_length")) for task in successes))
                    success_query_times.append(mean(task.get("query_s") for task in successes))
        else:
            success_total = sum(int(row.get("incumbent_success_count", 0)) for row in rows)
            mean_lengths = [row.get("incumbent_mean_length") for row in rows if row.get("incumbent_mean_length") is not None]
            total_lengths = [row.get("incumbent_total_length") for row in rows if row.get("incumbent_total_length") is not None]
            success_query_times = []
        build_s = median(row.get("cumulative_build_s") for row in rows)
        query_s = median(success_query_times) if success_only_stage else median(row.get("cumulative_query_s") for row in rows)
        raw_total_s = median(row.get("cumulative_total_s") for row in rows)
        total_s = None if build_s is None or query_s is None else float(build_s) + float(query_s)
        point = {
            "method": method,
            "stage_index": int(stage_index),
            "stage_id": stage_id,
            "build_s": build_s,
            "query_s": query_s,
            "total_s": total_s,
            "raw_total_s_median": raw_total_s,
            "path_length": median(mean_lengths),
            "path_length_total": median(total_lengths),
            "audit_sr": float(success_total) / max(1, task_total),
            "seed_count": len(rows),
            "task_count": task_total,
            "success_count": success_total,
            "params": rows[0].get("params", {}),
            "protocol": rows[0].get("protocol"),
            "raw_record_indices": [source_index for source_index, _ in indexed_rows],
        }
        previous = best_by_method.get(method)
        current_path = point.get("path_length")
        current_time = point.get("total_s")
        promoted = False
        no_improvement_reason = None
        if current_path is None or current_time is None:
            no_improvement_reason = "no_audited_incumbent"
        elif previous is None:
            promoted = True
            best_by_method[method] = (float(current_path), float(current_time))
        elif float(current_path) < previous[0] - float(epsilon_path):
            promoted = True
            best_by_method[method] = (float(current_path), float(current_time))
        else:
            no_improvement_reason = "incumbent_not_improved"
        point["promoted"] = bool(promoted)
        point["no_improvement_reason"] = no_improvement_reason
        all_points.append(point)

    promoted_points = [point for point in all_points if point.get("promoted")]
    return {"points": all_points, "promoted_points": promoted_points}


def assert_promoted_monotone(
    summary: dict[str, Any],
    *,
    epsilon_path: float = EPS_PATH_DEFAULT,
    epsilon_time: float = EPS_TIME_MONOTONE_DEFAULT,
) -> None:
    by_method: dict[str, list[dict[str, Any]]] = {}
    for point in summary.get("promoted_points", []):
        by_method.setdefault(str(point.get("method")), []).append(point)
    for method, points in by_method.items():
        ordered = sorted(points, key=lambda row: int(row.get("stage_index", 0)))
        for previous, current in zip(ordered, ordered[1:]):
            previous_time = float(previous.get("total_s") or 0.0)
            current_time = float(current.get("total_s") or 0.0)
            previous_path = float(previous.get("path_length") or math.inf)
            current_path = float(current.get("path_length") or math.inf)
            if current_time < previous_time - float(epsilon_time):
                raise ValueError(f"non-increasing charged time for {method}: {previous_time} -> {current_time}")
            if current_path >= previous_path - float(epsilon_path):
                raise ValueError(f"non-decreasing incumbent path for {method}: {previous_path} -> {current_path}")
