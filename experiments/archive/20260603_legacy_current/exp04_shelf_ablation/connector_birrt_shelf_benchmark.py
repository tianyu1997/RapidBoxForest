from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from statistics import median
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import sbf

from experiments.common.experiment_io import run_id, write_json


DEFAULT_OUTPUT_JSON = (
    REPO_ROOT
    / "outputs"
    / "new_experiments"
    / "exp04_connector_birrt_benchmark"
    / "shelf_timeout80.json"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark the connector BiRRT core on the real IIWA+shelf scene.")
    parser.add_argument("--out-json", type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument("--timeout-ms", type=float, default=80.0)
    parser.add_argument("--max-iters", type=int, default=50000)
    # Defaults mirror the production RRTConnectConfig (connector.h): step_size 0.5
    # with segment_resolution 32 keeps collision-check density at 0.0156 rad/check
    # while traversing the narrow shelf passages within budget.
    parser.add_argument("--step-size", type=float, default=0.5)
    parser.add_argument("--goal-bias", type=float, default=0.2)
    parser.add_argument("--segment-resolution", type=int, default=32)
    parser.add_argument("--local-radius", type=float, default=0.0)
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--query", default="all")
    parser.add_argument("--timeout-slack-ms", type=float, default=5.0)
    return parser.parse_args()


def parse_csv_ints(raw: str) -> list[int]:
    return [int(part.strip()) for part in str(raw).split(",") if part.strip()]


def query_label(query: Any) -> str:
    return str(getattr(query, "label", getattr(query, "name", "query")))


def path_length(path: list[list[float]]) -> float:
    return float(sbf.path_length(path)) if path else 0.0


def make_config(args: argparse.Namespace) -> Any:
    config = sbf.RRTConnectConfig()
    config.timeout_ms = float(args.timeout_ms)
    config.max_iters = int(args.max_iters)
    config.step_size = float(args.step_size)
    config.goal_bias = float(args.goal_bias)
    config.segment_resolution = int(args.segment_resolution)
    config.local_sampling_radius = float(args.local_radius)
    return config


def run_attempt(robot: Any, obstacles: list[Any], query: Any, config: Any, seed: int, timeout_slack_ms: float) -> dict[str, Any]:
    start = list(query.start)
    goal = list(query.goal)
    t0 = time.perf_counter()
    path = sbf.rrt_connect_path(robot, obstacles, start, goal, config, seed)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    success = bool(path)
    timeout_like = (not success) and elapsed_ms >= max(0.0, float(config.timeout_ms) - timeout_slack_ms)
    return {
        "seed": int(seed),
        "ok": success,
        "timeout_like": bool(timeout_like),
        "elapsed_ms": float(elapsed_ms),
        "path_waypoints": int(len(path)),
        "path_length": path_length(path),
    }


def summarize_attempts(label: str, attempts: list[dict[str, Any]]) -> dict[str, Any]:
    successes = [attempt for attempt in attempts if bool(attempt["ok"])]
    timeout_like = [attempt for attempt in attempts if bool(attempt["timeout_like"])]
    elapsed_values = [float(attempt["elapsed_ms"]) for attempt in attempts]
    return {
        "label": label,
        "n": len(attempts),
        "successes": len(successes),
        "timeout_like_failures": len(timeout_like),
        "success_rate": 0.0 if not attempts else len(successes) / float(len(attempts)),
        "timeout_like_rate": 0.0 if not attempts else len(timeout_like) / float(len(attempts)),
        "elapsed_ms_median": median(elapsed_values) if elapsed_values else 0.0,
        "elapsed_ms_max": max(elapsed_values) if elapsed_values else 0.0,
        "path_length_median": median(float(attempt["path_length"]) for attempt in successes) if successes else None,
        "attempts": attempts,
    }


def main() -> int:
    args = parse_args()
    robot = sbf.load_iiwa14_robot()
    obstacles = list(sbf.make_combined_obstacles())
    queries = list(sbf.make_combined_queries())
    if str(args.query) != "all":
        queries = [query for query in queries if query_label(query) == str(args.query)]
        if not queries:
            raise ValueError(f"unknown query label: {args.query!r}")

    config = make_config(args)
    seed_values = parse_csv_ints(args.seeds)
    summaries: list[dict[str, Any]] = []
    for query in queries:
        attempts = [run_attempt(robot, obstacles, query, config, seed, float(args.timeout_slack_ms)) for seed in seed_values]
        summaries.append(summarize_attempts(query_label(query), attempts))

    summaries.sort(key=lambda item: (item["timeout_like_failures"], item["elapsed_ms_median"]), reverse=True)
    payload = {
        "experiment": "exp04_connector_birrt_shelf_benchmark",
        "run_id": run_id("exp04_connector_birrt_shelf_benchmark"),
        "params": {
            "timeout_ms": float(args.timeout_ms),
            "max_iters": int(args.max_iters),
            "step_size": float(args.step_size),
            "goal_bias": float(args.goal_bias),
            "segment_resolution": int(args.segment_resolution),
            "local_radius": float(args.local_radius),
            "seeds": seed_values,
            "query": str(args.query),
            "timeout_slack_ms": float(args.timeout_slack_ms),
        },
        "rows": summaries,
    }
    write_json(args.out_json, payload)

    top = summaries[0] if summaries else None
    if top is not None:
        print(
            f"[connector-birrt-bench] top={top['label']} timeout_like={top['timeout_like_failures']}/{top['n']} "
            f"success={top['successes']}/{top['n']} median_ms={top['elapsed_ms_median']:.3f}"
        )
    print(f"[connector-birrt-bench] wrote {args.out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())