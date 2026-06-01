from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[3]


def build_matches_workspace(python_dir: Path) -> bool:
    cache_path = python_dir.parent / "CMakeCache.txt"
    if not cache_path.exists():
        return True

    home_prefix = "CMAKE_HOME_DIRECTORY:INTERNAL="
    for line in cache_path.read_text(encoding="utf-8").splitlines():
        if line.startswith(home_prefix):
            return Path(line[len(home_prefix):]).resolve() == ROOT.resolve()
    return True


def import_current_lie():
    for rel in (
        "build-rbf-only-exec/python",
        "build-sbf-tests-local/python",
        "build-rename-validate/python",
        "build-consolidated-python/python",
    ):
        candidate = ROOT / rel
        if candidate.exists() and build_matches_workspace(candidate):
            sys.path.insert(0, str(candidate))
            break
    import link_interval_envelope as lie

    return lie


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def chunk(flat: list[float], width: int = 6) -> list[list[float]]:
    return [flat[index:index + width] for index in range(0, len(flat), width)]


def box_volume(box: list[float]) -> float:
    return max(0.0, box[3] - box[0]) * max(0.0, box[4] - box[1]) * max(0.0, box[5] - box[2])


def total_volume(flat: list[float]) -> float:
    return sum(box_volume(box) for box in chunk(flat))


def width_triplets(flat: list[float]) -> list[float]:
    widths: list[float] = []
    for box in chunk(flat):
        widths.extend([box[3] - box[0], box[4] - box[1], box[5] - box[2]])
    return widths


def max_abs_diff(lhs: list[float], rhs: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(lhs, rhs)) if lhs and rhs else 0.0


def top_shrunk_boxes(lhs: list[float], rhs: list[float], limit: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for index, (bad_box, full_box) in enumerate(zip(chunk(lhs), chunk(rhs))):
        rows.append(
            {
                "box_index": index,
                "delta_volume": box_volume(full_box) - box_volume(bad_box),
                "max_abs_diff": max_abs_diff(bad_box, full_box),
                "bad_box": bad_box,
                "full_box": full_box,
            }
        )
    rows.sort(key=lambda item: (item["delta_volume"], item["max_abs_diff"]), reverse=True)
    return rows[:limit]


def run_probe(robot_json: Path,
              prev_intervals_json: Path,
              target_intervals_json: Path,
              changed_dim: int,
              n_subdivisions: int) -> dict[str, Any]:
    lie = import_current_lie()
    robot = str(robot_json)
    prev_intervals = load_json(prev_intervals_json)
    target_intervals = load_json(target_intervals_json)

    computer = lie.IncrementalEnvelopeComputer(
        robot,
        endpoint_source="crit_sample",
        envelope_type="link_iaabb",
        n_subdivisions=n_subdivisions,
    )

    previous = computer.compute(prev_intervals, changed_dim=-1)
    stale = computer.compute(target_intervals, changed_dim=changed_dim)
    computer.reset()
    fresh = computer.compute(target_intervals, changed_dim=-1)

    stale_endpoint = stale["endpoint"]["endpoint_iaabbs_flat"]
    fresh_endpoint = fresh["endpoint"]["endpoint_iaabbs_flat"]
    stale_link = stale["envelope"]["link_iaabbs_flat"]
    fresh_link = fresh["envelope"]["link_iaabbs_flat"]

    result = {
        "inputs": {
            "robot_json": str(robot_json),
            "prev_intervals_json": str(prev_intervals_json),
            "target_intervals_json": str(target_intervals_json),
            "changed_dim": changed_dim,
            "n_subdivisions": n_subdivisions,
        },
        "previous_incremental": previous["incremental"],
        "stale_incremental": stale["incremental"],
        "fresh_incremental": fresh["incremental"],
        "endpoint": {
            "stale_total_volume": total_volume(stale_endpoint),
            "fresh_total_volume": total_volume(fresh_endpoint),
            "volume_ratio": total_volume(stale_endpoint) / total_volume(fresh_endpoint)
            if total_volume(fresh_endpoint) > 0.0 else None,
            "max_abs_diff": max_abs_diff(stale_endpoint, fresh_endpoint),
            "mean_width_ratio": sum(width_triplets(stale_endpoint)) / sum(width_triplets(fresh_endpoint))
            if sum(width_triplets(fresh_endpoint)) > 0.0 else None,
        },
        "link_iaabb_envelope": {
            "stale_total_volume": total_volume(stale_link),
            "fresh_total_volume": total_volume(fresh_link),
            "volume_ratio": total_volume(stale_link) / total_volume(fresh_link)
            if total_volume(fresh_link) > 0.0 else None,
            "max_abs_diff": max_abs_diff(stale_link, fresh_link),
        },
        "top_shrunk_endpoint_boxes": top_shrunk_boxes(stale_endpoint, fresh_endpoint, limit=8),
        "stale_endpoint_flat": stale_endpoint,
        "fresh_endpoint_flat": fresh_endpoint,
    }
    return result


def run_old_collision_compare(result: dict[str, Any],
                              robot_json: Path,
                              obstacles_json: Path,
                              old_lie_python: Path) -> dict[str, Any]:
    payload = {
        "robot": str(robot_json),
        "obstacles": load_json(obstacles_json),
        "bad_endpoint": result["stale_endpoint_flat"],
        "full_endpoint": result["fresh_endpoint_flat"],
    }
    script = """
import json
import os
import sys

sys.path.insert(0, os.environ['OLD_LIE_PYTHON'])
import link_interval_envelope as lie

payload = json.loads(sys.stdin.read())
out = {}
for name, endpoint in [('bad', payload['bad_endpoint']), ('full', payload['full_endpoint'])]:
    res = lie.compute_collision_from_endpoint_iaabbs(
        payload['robot'],
        endpoint,
        payload['obstacles'],
        envelope_type='support_hull',
        n_subdivisions=4,
        kdop_directions='dop26',
        support_hull_keep_kdop=True,
        collision_mode='auto',
        safety_epsilon=1e-9,
        count_all_pairs=True,
    )
    out[name] = res['collision']
print(json.dumps(out))
"""
    env = os.environ.copy()
    env["OLD_LIE_PYTHON"] = str(old_lie_python)
    completed = subprocess.run(
        [sys.executable, "-c", script],
        input=json.dumps(payload),
        text=True,
        capture_output=True,
        check=True,
        env=env,
    )
    return json.loads(completed.stdout)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Reproduce the old stale-materialization path on node1 and compare against a full recompute.")
    parser.add_argument("--robot-json", type=Path, default=Path("/tmp/iiwa14.json"))
    parser.add_argument("--prev-intervals-json", type=Path, default=Path("/tmp/ve_986_iv.json"))
    parser.add_argument("--target-intervals-json", type=Path, default=Path("/tmp/node1_event_iv.json"))
    parser.add_argument("--changed-dim", type=int, default=6)
    parser.add_argument("--n-subdivisions", type=int, default=4)
    parser.add_argument("--old-lie-python", type=Path, default=None,
                        help="Optional path to the old standalone link_interval_envelope python package for collision comparison.")
    parser.add_argument("--obstacles-json", type=Path, default=Path("/tmp/marcucci_obs.json"))
    parser.add_argument("--out-json", type=Path, default=None)
    args = parser.parse_args()

    result = run_probe(
        robot_json=args.robot_json,
        prev_intervals_json=args.prev_intervals_json,
        target_intervals_json=args.target_intervals_json,
        changed_dim=args.changed_dim,
        n_subdivisions=args.n_subdivisions,
    )

    if args.old_lie_python is not None:
        result["old_collision_compare"] = run_old_collision_compare(
            result,
            robot_json=args.robot_json,
            obstacles_json=args.obstacles_json,
            old_lie_python=args.old_lie_python,
        )

    if args.out_json is not None:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        with args.out_json.open("w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2)

    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())