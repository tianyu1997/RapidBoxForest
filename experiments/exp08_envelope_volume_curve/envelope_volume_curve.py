"""Same-depth envelope-volume curve: AAFK vs support-hull split schedule.

Goal
----
Quantify whether the seed/scene-independent ``support_hull_volume_min`` split
schedule actually produces a *smaller certification envelope* (SupportHull swept
volume) at a fixed kd-tree depth than the prior ``aafk_volume_min`` schedule,
which minimises the looser endpoint-AABB-sum instead.

Method
------
Both schedules are deterministic pure functions of ``(robot, root_domain)`` and
use midpoint splits.  For each schedule we descend ``--paths`` random root->leaf
paths (matched RNG seed across schedules), and at every depth ``d`` we measure
the box that bounds that path's node.  For each box we compute two volumes:

* ``V_hull``  -- total volume of the **SupportHull** swept envelope
  (the actual certification envelope; the metric support-hull minimises).
* ``V_aabb``  -- total volume of the **LinkIAABB** envelope
  (the endpoint-AABB envelope; the metric AAFK minimises).

We then report ``mean_path log10(V)`` versus depth for each schedule.  The
expectation is the schedule wins on its *own* metric: support-hull below AAFK on
``V_hull``, AAFK below support-hull on ``V_aabb``.

Process model
-------------
``sbf`` and ``link_interval_envelope`` cannot be imported in the same
interpreter (duplicate ``Interval`` pybind type).  The schedules live in ``sbf``
and ``compute_envelope_info`` lives in ``link_interval_envelope``.  So the
schedules are produced by re-invoking this file with ``--emit-schedules`` in an
``sbf``-only subprocess; the main process imports only the envelope module.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_JSON = REPO_ROOT / "build-rbf-only-exec" / "python" / "sbf" / "data" / "iiwa14.json"
DEFAULT_PYTHONPATH = f"{REPO_ROOT / 'build-rbf-only-exec' / 'python'}:{REPO_ROOT}"

# Restricted root domain (the canonical shelf domain used across the SBF work).
ROOT_STR = (
    "0.0:1.5707963267948966;0.3194:0.8645;-0.5077:0.5073;"
    "-1.98947519:-0.33002121;-0.447:0.4473;-1.34734773:1.51007653;1.262:1.8794"
)


def parse_root(root_str: str) -> list[tuple[float, float]]:
    pairs: list[tuple[float, float]] = []
    for token in root_str.split(";"):
        lo_str, hi_str = token.split(":")
        pairs.append((float(lo_str), float(hi_str)))
    return pairs


# --------------------------------------------------------------------------- #
# Stage 1: schedules (runs in an sbf-only subprocess).
# --------------------------------------------------------------------------- #
def emit_schedules(max_depth: int, sample: int) -> None:
    import sbf  # noqa: PLC0415  (intentional sbf-only subprocess import)

    robot = sbf.load_iiwa14_robot()
    intervals = [sbf.Interval(lo, hi) for lo, hi in parse_root(ROOT_STR)]
    aafk = list(sbf.aafk_volume_min_depth_schedule(robot, intervals, max_depth, sample))
    hull = list(sbf.support_hull_volume_min_depth_schedule(robot, intervals, max_depth, sample))
    sys.stdout.write(json.dumps({"aafk": aafk, "hull": hull, "root": parse_root(ROOT_STR)}))


def fetch_schedules(max_depth: int, sample: int) -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = DEFAULT_PYTHONPATH + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
    )
    out = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()),
         "--emit-schedules", "--max-depth", str(max_depth), "--sample", str(sample)],
        check=True, capture_output=True, text=True, env=env,
    )
    return json.loads(out.stdout)


# --------------------------------------------------------------------------- #
# Stage 2: envelope volumes (runs in the lie-only main process).
# --------------------------------------------------------------------------- #
def envelope_total_volume(info: dict) -> float:
    """Sum of dx*dy*dz over every (inflated) sub-box of the envelope."""
    boxes = info["envelope"]["inflated_link_iaabbs_flat"]
    total = 0.0
    for base in range(0, len(boxes), 6):
        dx = max(0.0, boxes[base + 3] - boxes[base + 0])
        dy = max(0.0, boxes[base + 4] - boxes[base + 1])
        dz = max(0.0, boxes[base + 5] - boxes[base + 2])
        total += dx * dy * dz
    return total


def run_curve(args: argparse.Namespace) -> dict:
    import random

    import link_interval_envelope as lie  # noqa: PLC0415  (lie-only main process)

    schedules = fetch_schedules(args.max_depth, args.sample)
    root = [tuple(p) for p in schedules["root"]]
    n_dims = len(root)

    robot_path = str(ROBOT_JSON)

    def box_volumes(intervals: list[list[float]]) -> tuple[float, float]:
        hull_info = lie.compute_envelope(
            robot_path, intervals, endpoint_source="analytical",
            envelope_type="support_hull", n_subdivisions=args.hull_subdiv)
        aabb_info = lie.compute_envelope(
            robot_path, intervals, endpoint_source="analytical", envelope_type="link_iaabb")
        return envelope_total_volume(hull_info), envelope_total_volume(aabb_info)

    def schedule_curve(dims: list[int]) -> dict:
        depth_cap = min(len(dims), args.max_depth)
        # log-volume accumulators per depth (depth 0 .. depth_cap).
        sum_log_hull = [0.0] * (depth_cap + 1)
        sum_log_aabb = [0.0] * (depth_cap + 1)
        rng = random.Random(args.seed)
        for _ in range(args.paths):
            intervals = [[lo, hi] for lo, hi in root]
            v_hull, v_aabb = box_volumes(intervals)
            sum_log_hull[0] += math.log10(max(v_hull, 1e-300))
            sum_log_aabb[0] += math.log10(max(v_aabb, 1e-300))
            for depth in range(depth_cap):
                dim = dims[depth]
                if dim < 0 or dim >= n_dims:
                    # No-op depth: carry forward the previous box.
                    sum_log_hull[depth + 1] += math.log10(max(v_hull, 1e-300))
                    sum_log_aabb[depth + 1] += math.log10(max(v_aabb, 1e-300))
                    continue
                lo, hi = intervals[dim]
                mid = 0.5 * (lo + hi)
                if rng.random() < 0.5:
                    intervals[dim] = [lo, mid]
                else:
                    intervals[dim] = [mid, hi]
                v_hull, v_aabb = box_volumes(intervals)
                sum_log_hull[depth + 1] += math.log10(max(v_hull, 1e-300))
                sum_log_aabb[depth + 1] += math.log10(max(v_aabb, 1e-300))
        return {
            "log_hull": [s / args.paths for s in sum_log_hull],
            "log_aabb": [s / args.paths for s in sum_log_aabb],
        }

    result = {
        "root": root,
        "paths": args.paths,
        "sample": args.sample,
        "max_depth": args.max_depth,
        "seed": args.seed,
        "schedules": {"aafk": schedules["aafk"], "hull": schedules["hull"]},
        "curves": {
            "aafk": schedule_curve(schedules["aafk"]),
            "hull": schedule_curve(schedules["hull"]),
        },
    }
    return result


def summarise(result: dict) -> str:
    aafk = result["curves"]["aafk"]
    hull = result["curves"]["hull"]
    depths = range(len(aafk["log_hull"]))
    lines = []
    lines.append("depth |  log10 V_hull (cert envelope)   |  log10 V_aabb (endpoint AABB)")
    lines.append("      |  AAFK     HULL     Δ(H-A)        |  AAFK     HULL     Δ(H-A)")
    lines.append("-" * 78)
    for d in depths:
        ah, hh = aafk["log_hull"][d], hull["log_hull"][d]
        aa, ha = aafk["log_aabb"][d], hull["log_aabb"][d]
        lines.append(
            f"{d:>5} | {ah:>7.3f} {hh:>8.3f} {hh - ah:>+8.3f}        | "
            f"{aa:>7.3f} {ha:>8.3f} {ha - aa:>+8.3f}"
        )
    # Aggregate over depths >= divergence (depth 10) where the schedules differ.
    div = 10
    n = len(aafk["log_hull"])
    if n > div:
        mean_hull_gain = sum(
            hull["log_hull"][d] - aafk["log_hull"][d] for d in range(div, n)
        ) / (n - div)
        mean_aabb_gain = sum(
            hull["log_aabb"][d] - aafk["log_aabb"][d] for d in range(div, n)
        ) / (n - div)
        lines.append("-" * 78)
        lines.append(
            f"mean Δlog10 over depth {div}..{n - 1}: "
            f"V_hull(HULL-AAFK)={mean_hull_gain:+.4f}  "
            f"V_aabb(HULL-AAFK)={mean_aabb_gain:+.4f}"
        )
        lines.append(
            "(negative V_hull delta => support-hull schedule yields a smaller "
            "certification envelope at the same depth)"
        )
    return "\n".join(lines)


def maybe_plot(result: dict, png_path: Path) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    aafk = result["curves"]["aafk"]
    hull = result["curves"]["hull"]
    depths = list(range(len(aafk["log_hull"])))
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(12, 4.5))
    ax0.plot(depths, aafk["log_hull"], label="AAFK schedule", color="#c0392b")
    ax0.plot(depths, hull["log_hull"], label="support-hull schedule", color="#2471a3")
    ax0.set_title("SupportHull envelope volume (certification metric)")
    ax0.set_xlabel("kd-tree depth")
    ax0.set_ylabel("mean log10 envelope volume")
    ax0.legend()
    ax0.grid(True, alpha=0.3)
    ax1.plot(depths, aafk["log_aabb"], label="AAFK schedule", color="#c0392b")
    ax1.plot(depths, hull["log_aabb"], label="support-hull schedule", color="#2471a3")
    ax1.set_title("LinkIAABB envelope volume (endpoint-AABB metric)")
    ax1.set_xlabel("kd-tree depth")
    ax1.set_ylabel("mean log10 envelope volume")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    fig.tight_layout()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png_path, dpi=130)
    plt.close(fig)
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--emit-schedules", action="store_true",
                        help="Internal: print schedules as JSON (sbf-only subprocess).")
    parser.add_argument("--max-depth", type=int, default=40)
    parser.add_argument("--sample", type=int, default=8,
                        help="sample_nodes_per_depth for the schedule builders.")
    parser.add_argument("--paths", type=int, default=64,
                        help="Random root->leaf descent paths averaged per depth.")
    parser.add_argument("--hull-subdiv", type=int, default=4,
                        help="n_subdivisions for the support-hull certified envelope.")
    parser.add_argument("--seed", type=int, default=20260601)
    parser.add_argument("--out-dir", type=Path,
                        default=REPO_ROOT / "outputs" / "logs" / "envelope_volume_curve")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.emit_schedules:
        emit_schedules(args.max_depth, args.sample)
        return
    result = run_curve(args)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.out_dir / "envelope_volume_curve.json"
    json_path.write_text(json.dumps(result, indent=2))
    png_path = args.out_dir / "envelope_volume_curve.png"
    plotted = maybe_plot(result, png_path)
    table = summarise(result)
    (args.out_dir / "envelope_volume_curve.txt").write_text(table + "\n")
    print(table)
    print()
    print(f"[json] {json_path}")
    if plotted:
        print(f"[png ] {png_path}")
    else:
        print("[png ] skipped (matplotlib unavailable)")


if __name__ == "__main__":
    main()
