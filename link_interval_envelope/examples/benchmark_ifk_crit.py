from __future__ import annotations

import argparse
import statistics
import time
from pathlib import Path
from typing import Callable

from link_interval_envelope import (
    IncrementalEnvelopeComputer,
    Robot,
    compute_envelope,
    compute_envelope_batch,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT = ROOT / "examples" / "data" / "2dof_planar.json"


def make_boxes(robot: Robot, n_boxes: int) -> list[list[list[float]]]:
    limits = robot.joint_limits().limits
    n_dims = len(limits)
    moving_dim = min(1, n_dims - 1)
    base: list[list[float]] = []
    for lim in limits:
        span = float(lim.hi - lim.lo)
        width = min(0.8, max(0.02, 0.12 * span))
        center = 0.5 * float(lim.lo + lim.hi)
        lo = max(float(lim.lo), center - 0.5 * width)
        hi = min(float(lim.hi), center + 0.5 * width)
        base.append([lo, hi])

    move_lim = limits[moving_dim]
    move_span = float(move_lim.hi - move_lim.lo)
    move_width = base[moving_dim][1] - base[moving_dim][0]
    max_shift = max(0.0, 0.5 * (move_span - move_width))
    shift_step = min(0.01, max_shift / 12.0) if max_shift > 0.0 else 0.0

    boxes: list[list[list[float]]] = []
    for i in range(n_boxes):
        box = [[lo, hi] for lo, hi in base]
        shift = ((i % 17) - 8) * shift_step
        box[moving_dim] = [base[moving_dim][0] + shift, base[moving_dim][1] + shift]
        boxes.append(box)
    return boxes


def p95(values: list[float]) -> float:
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(0.95 * (len(ordered) - 1)))
    return ordered[idx]


def measure(label: str, fn: Callable[[], None], repeats: int) -> tuple[str, float, float]:
    samples: list[float] = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - t0) * 1000.0)
    return label, statistics.median(samples), p95(samples)


def run_source(
    robot: Robot,
    boxes: list[list[list[float]]],
    source: str,
    envelope_type: str,
    repeats: int,
    threads: int,
) -> None:
    def one_shot() -> None:
        for box in boxes:
            compute_envelope(
                robot,
                box,
                endpoint_source=source,
                envelope_type=envelope_type,
                n_subdivisions=4,
                include_endpoint_iaabbs=False,
            )

    def incremental() -> None:
        computer = IncrementalEnvelopeComputer(
            robot,
            endpoint_source=source,
            envelope_type=envelope_type,
            n_subdivisions=4,
        )
        for box in boxes:
            computer.compute(box, include_endpoint_iaabbs=False)

    def batch_one_thread() -> None:
        compute_envelope_batch(
            robot,
            boxes,
            endpoint_source=source,
            envelope_type=envelope_type,
            n_subdivisions=4,
            n_threads=1,
            include_endpoint_iaabbs=False,
        )

    def batch_parallel() -> None:
        compute_envelope_batch(
            robot,
            boxes,
            endpoint_source=source,
            envelope_type=envelope_type,
            n_subdivisions=4,
            n_threads=threads,
            include_endpoint_iaabbs=False,
        )

    rows = [
        measure("one-shot loop", one_shot, repeats),
        measure("incremental context", incremental, repeats),
        measure("batch 1 thread", batch_one_thread, repeats),
        measure(f"batch {threads} threads", batch_parallel, repeats),
    ]
    baseline = rows[0][1]

    print(f"\n[{source}] envelope={envelope_type} n_boxes={len(boxes)} repeats={repeats}")
    print("path, median_ms, p95_ms, speedup_vs_one_shot")
    for label, median_ms, p95_ms in rows:
        speedup = baseline / median_ms if median_ms > 0 else 0.0
        print(f"{label}, {median_ms:.3f}, {p95_ms:.3f}, {speedup:.2f}x")


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark IFK and CritSample optimized paths")
    parser.add_argument("--robot", default=str(DEFAULT_ROBOT))
    parser.add_argument("--n-boxes", type=int, default=512)
    parser.add_argument("--repeats", type=int, default=7)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument(
        "--envelope-type",
        default="link_iaabb",
        choices=["link_iaabb", "link_iaabb_grid", "hull16_grid"],
    )
    parser.add_argument(
        "--sources",
        nargs="+",
        default=["ifk", "critsample"],
        choices=["ifk", "critsample"],
    )
    args = parser.parse_args()

    robot = Robot.from_json(args.robot)
    boxes = make_boxes(robot, args.n_boxes)
    for source in args.sources:
        run_source(robot, boxes, source, args.envelope_type, args.repeats, args.threads)


if __name__ == "__main__":
    main()