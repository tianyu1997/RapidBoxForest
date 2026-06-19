from __future__ import annotations

import argparse
import json
import math
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.random_scene_catalog import make_robot
from experiments.common.rbf_defaults import robot_joint_limit_tuples
from experiments.common.sbf_import import import_sbf


sbf = import_sbf()


@dataclass(frozen=True)
class CSpaceBox:
    intervals: list[tuple[float, float]]


def l2(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def interpolate(a: Sequence[float], b: Sequence[float], alpha: float) -> list[float]:
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def point_in_box(q: Sequence[float], box: CSpaceBox) -> bool:
    return all(float(lo) <= float(value) <= float(hi) for value, (lo, hi) in zip(q, box.intervals))


def flatten_box(box: CSpaceBox) -> list[float]:
    out: list[float] = []
    for lo, hi in box.intervals:
        out.extend([float(lo), float(hi)])
    return out


def state_valid(q: Sequence[float], obstacles: Sequence[CSpaceBox]) -> bool:
    return not any(point_in_box(q, obstacle) for obstacle in obstacles)


def segment_valid(a: Sequence[float], b: Sequence[float], obstacles: Sequence[CSpaceBox], step: float) -> bool:
    dist = l2(a, b)
    samples = max(1, int(math.ceil(dist / max(1e-9, float(step)))))
    for index in range(samples + 1):
        if not state_valid(interpolate(a, b, index / samples), obstacles):
            return False
    return True


def sample_q(rng: random.Random, limits: Sequence[tuple[float, float]], obstacles: Sequence[CSpaceBox]) -> list[float]:
    for _ in range(10000):
        q = [rng.uniform(float(lo), float(hi)) for lo, hi in limits]
        if state_valid(q, obstacles):
            return q
    raise RuntimeError("could not sample a valid C-space state")


def steer(a: Sequence[float], b: Sequence[float], step: float) -> list[float]:
    dist = l2(a, b)
    if dist <= step:
        return [float(value) for value in b]
    alpha = float(step) / dist
    return interpolate(a, b, alpha)


def nearest(nodes: Sequence[Sequence[float]], q: Sequence[float]) -> int:
    return min(range(len(nodes)), key=lambda index: l2(nodes[index], q))


def extract_path(
    nodes_a: Sequence[Sequence[float]],
    parents_a: Sequence[int],
    index_a: int,
    nodes_b: Sequence[Sequence[float]],
    parents_b: Sequence[int],
    index_b: int,
    swapped: bool,
) -> list[list[float]]:
    left: list[list[float]] = []
    index = index_a
    while index >= 0:
        left.append([float(value) for value in nodes_a[index]])
        index = parents_a[index]
    left.reverse()
    right: list[list[float]] = []
    index = index_b
    while index >= 0:
        right.append([float(value) for value in nodes_b[index]])
        index = parents_b[index]
    path = left + right
    return list(reversed(path)) if swapped else path


def rrt_connect(
    limits: Sequence[tuple[float, float]],
    obstacles: Sequence[CSpaceBox],
    start: Sequence[float],
    goal: Sequence[float],
    *,
    timeout_s: float,
    step: float,
    collision_step: float,
    seed: int,
) -> dict[str, Any]:
    if not state_valid(start, obstacles) or not state_valid(goal, obstacles):
        return {"ok": False, "status": "invalid_endpoint", "solve_s": 0.0, "nodes": 0, "path": []}
    rng = random.Random(int(seed))
    ta = [[float(value) for value in start]]
    tb = [[float(value) for value in goal]]
    pa = [-1]
    pb = [-1]
    swapped = False
    deadline = time.perf_counter() + float(timeout_s)
    while time.perf_counter() < deadline:
        q_rand = sample_q(rng, limits, obstacles)
        ia = nearest(ta, q_rand)
        q_new = steer(ta[ia], q_rand, step)
        if not segment_valid(ta[ia], q_new, obstacles, collision_step):
            ta, tb = tb, ta
            pa, pb = pb, pa
            swapped = not swapped
            continue
        ta.append(q_new)
        pa.append(ia)
        new_index = len(ta) - 1
        while True:
            ib = nearest(tb, ta[new_index])
            q_next = steer(tb[ib], ta[new_index], step)
            if not segment_valid(tb[ib], q_next, obstacles, collision_step):
                break
            tb.append(q_next)
            pb.append(ib)
            next_index = len(tb) - 1
            if l2(q_next, ta[new_index]) <= step and segment_valid(q_next, ta[new_index], obstacles, collision_step):
                path = extract_path(ta, pa, new_index, tb, pb, next_index, swapped)
                return {
                    "ok": True,
                    "status": "Exact solution",
                    "solve_s": time.perf_counter() - (deadline - float(timeout_s)),
                    "nodes": len(ta) + len(tb),
                    "path": path,
                    "path_length": sum(l2(path[i], path[i + 1]) for i in range(len(path) - 1)),
                }
            if l2(q_next, ta[new_index]) < 1e-9:
                break
        ta, tb = tb, ta
        pa, pb = pb, pa
        swapped = not swapped
    return {
        "ok": False,
        "status": "Timeout",
        "solve_s": float(timeout_s),
        "nodes": len(ta) + len(tb),
        "path": [],
    }


def make_wall_with_offset_gap(
    limits: Sequence[tuple[float, float]],
    *,
    wall_dim: int,
    gate_dim: int,
    wall_center: float,
    wall_half_width: float,
    gate_center: float,
    gate_half_width: float,
) -> list[CSpaceBox]:
    low_box: list[tuple[float, float]] = [(float(lo), float(hi)) for lo, hi in limits]
    high_box: list[tuple[float, float]] = [(float(lo), float(hi)) for lo, hi in limits]
    low_box[wall_dim] = (wall_center - wall_half_width, wall_center + wall_half_width)
    high_box[wall_dim] = (wall_center - wall_half_width, wall_center + wall_half_width)
    low_box[gate_dim] = (float(limits[gate_dim][0]), gate_center - gate_half_width)
    high_box[gate_dim] = (gate_center + gate_half_width, float(limits[gate_dim][1]))
    return [CSpaceBox(low_box), CSpaceBox(high_box)]


def make_wall_with_hyper_gate(
    limits: Sequence[tuple[float, float]],
    *,
    wall_dim: int,
    gate_dims: Sequence[int],
    wall_center: float,
    wall_half_width: float,
    gate_center: float,
    gate_half_width: float,
) -> list[CSpaceBox]:
    # The wall blocks wall_dim unless every gate dimension lies in the small
    # gate interval. The complement of that hyper-rectangle is represented as
    # two slab boxes per gate dimension.
    obstacles: list[CSpaceBox] = []
    for gate_dim in gate_dims:
        low_box: list[tuple[float, float]] = [(float(lo), float(hi)) for lo, hi in limits]
        high_box: list[tuple[float, float]] = [(float(lo), float(hi)) for lo, hi in limits]
        gate_lo = float(limits[gate_dim][0])
        gate_hi = float(limits[gate_dim][1])
        width = max(1e-9, gate_hi - gate_lo)
        half_width = min(float(gate_half_width), 0.45 * width)
        center = min(gate_hi - half_width, max(gate_lo + half_width, float(gate_center)))
        low_box[wall_dim] = (wall_center - wall_half_width, wall_center + wall_half_width)
        high_box[wall_dim] = (wall_center - wall_half_width, wall_center + wall_half_width)
        low_box[gate_dim] = (gate_lo, center - half_width)
        high_box[gate_dim] = (center + half_width, gate_hi)
        obstacles.extend([CSpaceBox(low_box), CSpaceBox(high_box)])
    return obstacles


def difficulty_hyper_gate_params(robot_name: str, difficulty: str, dim: int) -> tuple[int, float, float]:
    key = str(difficulty).lower()
    robot_key = str(robot_name).lower()
    # These defaults were calibrated with OMPL BIT* on the current deterministic
    # binding. Medium/hard target first-solution checkpoints around or above
    # 0.1 s while preserving eventual feasibility within a 2 s probe.
    if key == "easy":
        return min(1, max(1, dim - 1)), 0.40, 0.24
    if key == "hard":
        if robot_key == "iiwa":
            return min(3, max(1, dim - 1)), 0.45, 0.30
        if robot_key == "ur5":
            return min(2, max(1, dim - 1)), 0.18, 0.30
        if robot_key == "panda":
            return min(3, max(1, dim - 1)), 0.35, 0.30
        return min(3, max(1, dim - 1)), 0.50, 0.30
    if robot_key == "iiwa":
        return min(2, max(1, dim - 1)), 0.20, 0.30
    if robot_key == "ur5":
        return min(2, max(1, dim - 1)), 0.22, 0.30
    if robot_key == "panda":
        return min(3, max(1, dim - 1)), 0.45, 0.30
    return min(2, max(1, dim - 1)), 0.22, 0.30


def bitstar_first_success_checkpoint(
    limits: Sequence[tuple[float, float]],
    obstacles: Sequence[CSpaceBox],
    start: Sequence[float],
    goal: Sequence[float],
    *,
    timeout_s: float,
    checkpoint_interval_s: float,
    segment_step: float,
    seed: int,
    samples_per_batch: int,
    rewire_factor: float,
) -> dict[str, Any]:
    result = sbf.ompl_cspace_bitstar_trace(
        [[float(lo), float(hi)] for lo, hi in limits],
        [flatten_box(obstacle) for obstacle in obstacles],
        [float(value) for value in start],
        [float(value) for value in goal],
        float(timeout_s) * 1000.0,
        float(checkpoint_interval_s) * 1000.0,
        float(segment_step),
        int(seed),
        int(samples_per_batch),
        float(rewire_factor),
        False,
    )
    checkpoints = [dict(item) for item in result.get("checkpoints", [])]
    first = next((row for row in checkpoints if bool(row.get("ok"))), None)
    return {
        "planner": "OMPL_CSpace_BITstar",
        "ok": bool(result.get("ok")),
        "status": str(result.get("status", "")),
        "timeout_s": float(timeout_s),
        "checkpoint_interval_s": float(checkpoint_interval_s),
        "segment_step": float(segment_step),
        "seed": int(seed),
        "samples_per_batch": int(samples_per_batch),
        "rewire_factor": float(rewire_factor),
        "first_success_checkpoint_s": (
            float(first.get("checkpoint_s")) if first is not None else math.nan
        ),
        "first_success_elapsed_s": (
            float(first.get("elapsed_s")) if first is not None else math.nan
        ),
        "final_solve_s": float(result.get("solve_s", math.nan)),
        "path_length": (
            sum(l2(result["path"][i], result["path"][i + 1]) for i in range(len(result.get("path", [])) - 1))
            if bool(result.get("ok")) and len(result.get("path", [])) >= 2
            else math.nan
        ),
        "checkpoint_count": len(checkpoints),
        "success_checkpoint_count": sum(1 for row in checkpoints if bool(row.get("ok"))),
    }


def make_problem(robot_name: str, difficulty: str) -> dict[str, Any]:
    robot = make_robot(robot_name)
    limits = robot_joint_limit_tuples(robot)
    dim = len(limits)
    wall_dim = 0
    difficulty_key = str(difficulty).lower()
    gate_count, gap_width, wall_half_width = difficulty_hyper_gate_params(robot_name, difficulty_key, dim)
    gate_dims = list(range(1, 1 + min(gate_count, dim - 1)))
    gate_dim = gate_dims[0] if gate_dims else (1 if dim > 1 else 0)
    gate_center = min(float(limits[gate_dim][1]) - 0.25, max(float(limits[gate_dim][0]) + 0.25, 1.0))
    obstacles = make_wall_with_hyper_gate(
        limits,
        wall_dim=wall_dim,
        gate_dims=gate_dims,
        wall_center=0.0,
        wall_half_width=wall_half_width,
        gate_center=gate_center,
        gate_half_width=0.5 * gap_width,
    )
    start = [0.5 * (lo + hi) for lo, hi in limits]
    goal = list(start)
    start[wall_dim] = max(float(limits[wall_dim][0]) + 0.3, -2.2)
    goal[wall_dim] = min(float(limits[wall_dim][1]) - 0.3, 2.2)
    start[gate_dim] = 0.0
    goal[gate_dim] = 0.0
    return {
        "schema": "tro2026_cspace_narrow_v1",
        "robot": str(robot_name),
        "difficulty": str(difficulty),
        "limits": [[float(lo), float(hi)] for lo, hi in limits],
        "start": [float(value) for value in start],
        "goal": [float(value) for value in goal],
        "obstacles": [{"intervals": [[float(lo), float(hi)] for lo, hi in box.intervals]} for box in obstacles],
        "flattened_obstacles": [flatten_box(box) for box in obstacles],
        "wall_dim": wall_dim,
        "gate_dims": gate_dims,
        "gate_center": float(gate_center),
        "gate_width": float(gap_width),
        "wall_half_width": float(wall_half_width),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate and probe synthetic C-space narrow-passage scenes.")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--robots", default="iiwa,ur5,panda")
    parser.add_argument("--difficulties", default="easy,medium,hard")
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--bitstar-timeout-s", type=float, default=5.0)
    parser.add_argument("--bitstar-checkpoint-interval-s", type=float, default=0.02)
    parser.add_argument("--bitstar-samples-per-batch", type=int, default=100)
    parser.add_argument("--bitstar-rewire-factor", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=0.25)
    parser.add_argument("--collision-step", type=float, default=0.01)
    parser.add_argument("--probe", choices=["rrtconnect", "bitstar"], default="bitstar")
    parser.add_argument("--seeds", type=int, default=8)
    args = parser.parse_args()
    records: list[dict[str, Any]] = []
    for robot_name in [item.strip() for item in args.robots.split(",") if item.strip()]:
        for difficulty in [item.strip() for item in args.difficulties.split(",") if item.strip()]:
            problem = make_problem(robot_name, difficulty)
            limits = [(float(lo), float(hi)) for lo, hi in problem["limits"]]
            obstacles = [CSpaceBox([(float(lo), float(hi)) for lo, hi in row["intervals"]]) for row in problem["obstacles"]]
            if str(args.probe) == "bitstar":
                probes = [
                    bitstar_first_success_checkpoint(
                        limits,
                        obstacles,
                        problem["start"],
                        problem["goal"],
                        timeout_s=float(args.bitstar_timeout_s),
                        checkpoint_interval_s=float(args.bitstar_checkpoint_interval_s),
                        segment_step=float(args.collision_step),
                        seed=seed,
                        samples_per_batch=int(args.bitstar_samples_per_batch),
                        rewire_factor=float(args.bitstar_rewire_factor),
                    )
                    for seed in range(int(args.seeds))
                ]
            else:
                probes = [
                    rrt_connect(
                        limits,
                        obstacles,
                        problem["start"],
                        problem["goal"],
                        timeout_s=float(args.timeout_s),
                        step=float(args.step),
                        collision_step=float(args.collision_step),
                        seed=seed,
                    )
                    for seed in range(int(args.seeds))
                ]
            problem["probes"] = [
                {key: value for key, value in probe.items() if key != "path"}
                for probe in probes
            ]
            records.append(problem)
            ok = sum(1 for probe in probes if probe["ok"])
            if str(args.probe) == "bitstar":
                times = [
                    float(probe["first_success_checkpoint_s"])
                    for probe in probes
                    if probe["ok"] and math.isfinite(float(probe["first_success_checkpoint_s"]))
                ]
            else:
                times = [float(probe["solve_s"]) for probe in probes if probe["ok"]]
            median = sorted(times)[len(times) // 2] if times else math.nan
            mean_s = sum(times) / len(times) if times else math.nan
            print(robot_name, difficulty, f"ok={ok}/{len(probes)}", f"mean_s={mean_s:.4f}", f"median_s={median:.4f}")
    payload = {
        "schema": "tro2026_cspace_narrow_catalog_v2",
        "probe": str(args.probe),
        "bitstar": {
            "timeout_s": float(args.bitstar_timeout_s),
            "checkpoint_interval_s": float(args.bitstar_checkpoint_interval_s),
            "samples_per_batch": int(args.bitstar_samples_per_batch),
            "rewire_factor": float(args.bitstar_rewire_factor),
        },
        "records": records,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print({"out": str(args.out), "records": len(records)})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
