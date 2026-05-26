#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[3]

try:
    import sbf
except ImportError as exc:  # pragma: no cover - friendly CLI error
    raise SystemExit(
        "Unable to import sbf. Run with PYTHONPATH=build_py310/python:python from cpp/SBF."
    ) from exc


PI = math.pi


SCENES: dict[str, dict[str, Any]] = {
    "empty": {
        "start": [-2.35, -1.10],
        "goal": [2.30, 1.05],
        "obstacles": [],
    },
    "single_block": {
        "start": [-2.45, -0.95],
        "goal": [2.35, 0.95],
        "obstacles": [
            {"name": "block", "cx": 0.75, "cy": 0.10, "hx": 0.23, "hy": 0.42, "hz": 0.20},
        ],
    },
    "narrow_gap": {
        "start": [-2.55, -0.85],
        "goal": [2.45, 0.85],
        "obstacles": [
            {"name": "upper", "cx": 0.70, "cy": 0.52, "hx": 0.28, "hy": 0.28, "hz": 0.20},
            {"name": "lower", "cx": 0.70, "cy": -0.52, "hx": 0.28, "hy": 0.28, "hz": 0.20},
        ],
    },
}


def make_planar_robot(link_1: float = 1.0, link_2: float = 1.0, radius: float = 0.035) -> sbf.Robot:
    dh0 = sbf.DHParam()
    dh0.alpha = 0.0
    dh0.a = float(link_1)
    dh0.d = 0.0
    dh0.theta = 0.0
    dh0.joint_type = 0

    dh1 = sbf.DHParam()
    dh1.alpha = 0.0
    dh1.a = float(link_2)
    dh1.d = 0.0
    dh1.theta = 0.0
    dh1.joint_type = 0

    limits = sbf.JointLimits()
    limits.limits = [sbf.Interval(-PI, PI), sbf.Interval(-PI, PI)]
    return sbf.Robot("planar_2dof", [dh0, dh1], limits, None, [radius, radius])


def make_obstacles(scene: dict[str, Any]) -> list[sbf.Obstacle]:
    obstacles: list[sbf.Obstacle] = []
    for item in scene["obstacles"]:
        cx = float(item["cx"])
        cy = float(item["cy"])
        hx = float(item["hx"])
        hy = float(item["hy"])
        hz = float(item.get("hz", 0.20))
        obstacles.append(sbf.Obstacle(cx - hx, cy - hy, -hz, cx + hx, cy + hy, hz))
    return obstacles


def fk_2dof(q: np.ndarray, link_1: float = 1.0, link_2: float = 1.0) -> np.ndarray:
    q1 = float(q[0])
    q2 = float(q[1])
    x1 = link_1 * math.cos(q1)
    y1 = link_1 * math.sin(q1)
    x2 = x1 + link_2 * math.cos(q1 + q2)
    y2 = y1 + link_2 * math.sin(q1 + q2)
    return np.array([[0.0, 0.0], [x1, y1], [x2, y2]], dtype=float)


def segment_intersects_aabb_2d(p0: np.ndarray, p1: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> bool:
    direction = p1 - p0
    t0 = 0.0
    t1 = 1.0
    for axis in range(2):
        p = float(direction[axis])
        if abs(p) < 1e-15:
            if p0[axis] < lo[axis] or p0[axis] > hi[axis]:
                return False
            continue
        inv = 1.0 / p
        near = (lo[axis] - p0[axis]) * inv
        far = (hi[axis] - p0[axis]) * inv
        if near > far:
            near, far = far, near
        t0 = max(t0, near)
        t1 = min(t1, far)
        if t0 > t1:
            return False
    return True


def config_collides(q: np.ndarray, scene: dict[str, Any], link_1: float, link_2: float) -> bool:
    pts = fk_2dof(q, link_1, link_2)
    for item in scene["obstacles"]:
        lo = np.array([item["cx"] - item["hx"], item["cy"] - item["hy"]], dtype=float)
        hi = np.array([item["cx"] + item["hx"], item["cy"] + item["hy"]], dtype=float)
        if segment_intersects_aabb_2d(pts[0], pts[1], lo, hi):
            return True
        if segment_intersects_aabb_2d(pts[1], pts[2], lo, hi):
            return True
    return False


def collision_map(scene: dict[str, Any], resolution: float, link_1: float, link_2: float) -> tuple[np.ndarray, list[float]]:
    qs = np.arange(-PI, PI + 1e-9, resolution, dtype=float)
    grid = np.zeros((len(qs), len(qs)), dtype=np.float32)
    for row, q2 in enumerate(qs):
        for col, q1 in enumerate(qs):
            grid[row, col] = 1.0 if config_collides(np.array([q1, q2]), scene, link_1, link_2) else 0.0
    return grid, [-PI, PI, -PI, PI]


def interval_payload(intervals: list[Any]) -> list[list[float]]:
    return [[float(interval.lo), float(interval.hi)] for interval in intervals]


def center_of(intervals: list[Any]) -> list[float]:
    return [0.5 * (float(interval.lo) + float(interval.hi)) for interval in intervals]


def infer_parent_face(box: Any, by_id: dict[int, Any], tolerance: float = 1e-6) -> dict[str, Any] | None:
    parent_id = int(box.parent_box_id)
    if parent_id < 0 or parent_id not in by_id:
        return None
    parent = by_id[parent_id]
    best: dict[str, Any] | None = None
    for dim, (child_iv, parent_iv) in enumerate(zip(box.joint_intervals, parent.joint_intervals)):
        child_lo = float(child_iv.lo)
        child_hi = float(child_iv.hi)
        parent_lo = float(parent_iv.lo)
        parent_hi = float(parent_iv.hi)
        candidates = [
            (abs(child_lo - parent_hi), 1, parent_hi),
            (abs(child_hi - parent_lo), -1, parent_lo),
        ]
        for gap, side, face_value in candidates:
            if best is None or gap < best["gap"]:
                best = {"dim": dim, "side": side, "face_value": face_value, "gap": gap}
    if best is not None and best["gap"] <= tolerance:
        best["gap"] = float(best["gap"])
        return best
    return best


def build_config(args: argparse.Namespace, trace_path: Path | None = None) -> sbf.SBFConfig:
    config = sbf.SBFConfig()
    config.enable_merger = False
    config.enable_connector = False
    config.endpoint_source.source = sbf.EndpointSource.CritSample if args.endpoint_source == "critsample" else sbf.EndpointSource.IFK
    config.envelope_type.type = sbf.EnvelopeType.LinkIAABB
    config.envelope_type.n_subdivisions = int(args.envelope_subdivisions)
    config.runtime.mode = sbf.ExecutionMode.Parallel if args.threads > 1 else sbf.ExecutionMode.Inline
    config.runtime.n_threads = max(1, int(args.threads))
    config.runtime.batch_size = max(1, int(args.task_batch_size))
    config.runtime.parallel_threshold = 1

    config.grower.mode = sbf.GrowerMode.RRT
    config.grower.max_boxes = int(args.max_boxes)
    config.grower.timeout_ms = float(args.timeout_ms)
    config.grower.max_consecutive_miss = int(args.ffb_fail_limit)
    config.grower.rng_seed = int(args.seed)
    config.grower.n_threads = max(1, int(args.threads))
    config.grower.task_batch_size = max(1, int(args.task_batch_size))
    config.grower.parallel_threshold = 1
    config.grower.worker_local_ffb = args.threads > 1
    config.grower.rrt_goal_bias = float(args.rrt_goal_bias)
    config.grower.intertree_goal_bias = float(args.intertree_goal_bias)
    config.grower.expand_all_roots_per_sample = True
    config.grower.connect_mode = True
    config.grower.stop_after_connect = False
    config.grower.find_free_box.max_depth = int(args.ffb_depth)
    config.grower.find_free_box.deadline_ms = 0.0
    config.grower.find_free_box.split_reserved_leaf = True
    config.grower.find_free_box.split_unknown_leaf = True
    config.grower.find_free_box.reject_seed_collision = False
    config.grower.find_free_box.split.use_best_tighten = args.split_policy != "widest-first"
    config.grower.find_free_box.split.best_tighten.depth_synchronous = bool(args.best_tighten_depth_synchronous)
    config.grower.find_free_box.split.best_tighten.shape_balancing = bool(args.best_tighten_shape_balancing)
    config.grower.find_free_box.split.best_tighten.max_child_aspect = float(args.best_tighten_max_child_aspect)
    config.grower.find_free_box.split.best_tighten.min_split_width_fraction = float(args.best_tighten_min_split_width_fraction)
    if trace_path is not None:
        config.grower.trace_enabled = True
        config.grower.trace_path = str(trace_path)
        config.grower.trace_max_events = int(args.trace_max_events)
    config.query.nearest_if_outside = False
    return config


def profile_payload(profile: Any) -> dict[str, Any]:
    return {
        "total_ms": float(profile.total_ms),
        "grow_ms": float(profile.grow_ms),
        "merge_ms": float(profile.merge_ms),
        "connector_ms": float(profile.connector_ms),
        "adjacency_ms": float(profile.adjacency_ms),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "adjacency_islands": int(profile.adjacency_islands),
        "diagnostics": {str(k): float(v) for k, v in profile.diagnostics.items()},
    }


def read_cpp_events(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    events: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            events.append(json.loads(line))
    return events


def write_reconstructed_trace(out_dir: Path, raw_boxes: list[Any], by_id: dict[int, Any]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for index, box in enumerate(raw_boxes):
        face = infer_parent_face(box, by_id)
        seed = list(map(float, box.seed_config)) if getattr(box, "seed_config", None) is not None else center_of(box.joint_intervals)
        events.append({
            "event": "rrt_seed",
            "index": index,
            "box_id": int(box.id),
            "parent_box_id": int(box.parent_box_id),
            "root_id": int(box.root_id),
            "q": seed,
            "seed": seed,
            "reconstructed": True,
        })
        if face is not None:
            events.append({
                "event": "selected_face",
                "index": index,
                "box_id": int(box.id),
                "parent_box_id": int(box.parent_box_id),
                "root_id": int(box.root_id),
                "face": face,
                "seed": seed,
                "reconstructed": True,
            })
        events.append({
            "event": "ffb_success",
            "index": index,
            "box_id": int(box.id),
            "parent_box_id": int(box.parent_box_id),
            "root_id": int(box.root_id),
            "seed": seed,
            "intervals": interval_payload(box.joint_intervals),
            "reconstructed": True,
        })
        events.append({
            "event": "box_added",
            "index": index,
            "box_id": int(box.id),
            "parent_box_id": int(box.parent_box_id),
            "root_id": int(box.root_id),
            "tree_id": int(box.tree_id),
            "seed": seed,
            "center": center_of(box.joint_intervals),
            "intervals": interval_payload(box.joint_intervals),
            "volume": float(box.volume),
            "inferred_parent_face": face,
            "reconstructed": True,
        })
    with (out_dir / "events_reconstructed.jsonl").open("w", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event) + "\n")
    return events


def write_trace(out_dir: Path, args: argparse.Namespace, scene_name: str, scene: dict[str, Any], forest: Any, profile: Any, query: Any) -> list[dict[str, Any]]:
    raw_boxes = list(forest.raw_boxes())
    by_id: dict[int, Any] = {int(box.id): box for box in raw_boxes}
    cpp_trace_path = out_dir / "events.jsonl"
    cpp_events = read_cpp_events(cpp_trace_path)
    query_events: list[dict[str, Any]] = [
        {"event": "query_seed", "kind": "start", "q": list(map(float, scene["start"]))},
        {"event": "query_seed", "kind": "goal", "q": list(map(float, scene["goal"]))},
    ]
    if cpp_events:
        events = query_events + cpp_events
        trace_source = "cpp_grower"
    else:
        events = query_events + write_reconstructed_trace(out_dir, raw_boxes, by_id)
        trace_source = "reconstructed_fallback"

    payload = {
        "config": {
            "scene": scene_name,
            "endpoint_source": args.endpoint_source,
            "split_policy": args.split_policy,
            "best_tighten_depth_synchronous": args.best_tighten_depth_synchronous,
            "best_tighten_shape_balancing": args.best_tighten_shape_balancing,
            "best_tighten_max_child_aspect": args.best_tighten_max_child_aspect,
            "best_tighten_min_split_width_fraction": args.best_tighten_min_split_width_fraction,
            "max_boxes": args.max_boxes,
            "ffb_depth": args.ffb_depth,
            "threads": args.threads,
            "trace_source": trace_source,
        },
        "scene": scene,
        "profile": profile_payload(profile),
        "query": {
            "success": bool(query.success),
            "box_sequence": [int(v) for v in query.box_sequence],
            "path_length": float(query.path_length),
            "query_time_ms": float(query.query_time_ms),
        },
        "events": events,
    }
    (out_dir / "trace.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return events


def draw_obstacles(ax: Any, scene: dict[str, Any]) -> None:
    from matplotlib.patches import Rectangle

    for item in scene["obstacles"]:
        rect = Rectangle(
            (item["cx"] - item["hx"], item["cy"] - item["hy"]),
            2.0 * item["hx"],
            2.0 * item["hy"],
            edgecolor="#555555",
            facecolor="#888888",
            alpha=0.35,
            linewidth=1.2,
        )
        ax.add_patch(rect)


def draw_box(ax: Any, event: dict[str, Any], color: str, alpha: float = 0.20, linewidth: float = 0.7) -> None:
    from matplotlib.patches import Rectangle

    intervals = event["intervals"]
    lo0, hi0 = intervals[0]
    lo1, hi1 = intervals[1]
    rect = Rectangle((lo0, lo1), hi0 - lo0, hi1 - lo1, edgecolor=color, facecolor=color, alpha=alpha, linewidth=linewidth)
    ax.add_patch(rect)


def draw_workspace_arm(ax: Any, q: list[float], scene: dict[str, Any], link_1: float, link_2: float) -> None:
    pts = fk_2dof(np.array(q, dtype=float), link_1, link_2)
    ax.plot(pts[:, 0], pts[:, 1], "o-", color="#1f77b4", linewidth=2.0, markersize=4)
    ax.scatter([pts[-1, 0]], [pts[-1, 1]], color="#d62728", s=24, zorder=5)
    draw_obstacles(ax, scene)


def render_visualization(out_dir: Path, scene: dict[str, Any], events: list[dict[str, Any]], args: argparse.Namespace) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    box_events = [event for event in events if event["event"] == "box_added"]
    task_targets = {
        int(event["task_id"]): event
        for event in events
        if event["event"] == "rrt_sampled_target" and "task_id" in event
    }
    ffb_events = {
        int(event["task_id"]): event
        for event in events
        if event["event"] in {"ffb_success", "ffb_fail"} and "task_id" in event
    }
    cmap, extent = collision_map(scene, args.collision_resolution, args.link1, args.link2)
    colors = ["#2f7ed8", "#0f9d58", "#f4b400", "#db4437", "#7e57c2", "#00acc1"]

    def setup_axes(fig: Any):
        ax_c, ax_w = fig.subplots(1, 2)
        ax_c.imshow(cmap, origin="lower", extent=extent, cmap="Reds", alpha=0.24, aspect="auto")
        ax_c.set_xlim(-PI, PI)
        ax_c.set_ylim(-PI, PI)
        ax_c.set_xlabel("q1")
        ax_c.set_ylabel("q2")
        ax_c.set_title("C-space grower trace")
        ax_c.scatter([scene["start"][0]], [scene["start"][1]], c="#111111", marker="o", s=38, label="start")
        ax_c.scatter([scene["goal"][0]], [scene["goal"][1]], c="#111111", marker="*", s=70, label="goal")
        ax_w.set_xlim(-2.25, 2.25)
        ax_w.set_ylim(-2.25, 2.25)
        ax_w.set_aspect("equal", adjustable="box")
        ax_w.set_xlabel("x")
        ax_w.set_ylabel("y")
        ax_w.set_title("Workspace arm pose")
        return ax_c, ax_w

    fig = plt.figure(figsize=(12, 5.8), constrained_layout=True)
    ax_c, ax_w = setup_axes(fig)
    for event in box_events:
        color = colors[int(event["root_id"]) % len(colors)]
        draw_box(ax_c, event, color, alpha=0.16)
    if box_events:
        draw_workspace_arm(ax_w, box_events[-1]["seed"], scene, args.link1, args.link2)
    else:
        draw_workspace_arm(ax_w, scene["start"], scene, args.link1, args.link2)
    fig.savefig(out_dir / "final.png", dpi=160)
    plt.close(fig)

    if not args.animate or not box_events:
        return

    frame_count = min(args.max_frames, max(1, len(box_events)))
    frame_indices = np.linspace(1, len(box_events), frame_count, dtype=int).tolist()
    fig = plt.figure(figsize=(12, 5.8), constrained_layout=True)

    def update(frame_number: int):
        fig.clear()
        ax_c, ax_w = setup_axes(fig)
        upto = frame_indices[frame_number]
        current = box_events[upto - 1]
        for event in box_events[:upto]:
            color = colors[int(event["root_id"]) % len(colors)]
            draw_box(ax_c, event, color, alpha=0.14)
        draw_box(ax_c, current, "#ff6f00", alpha=0.45, linewidth=1.5)
        ax_c.scatter([current["seed"][0]], [current["seed"][1]], c="#ff6f00", s=38, marker="x")
        task_id = int(current.get("task_id", -999999))
        target_event = task_targets.get(task_id)
        if target_event is not None and target_event.get("target"):
            target = target_event["target"]
            ax_c.scatter([target[0]], [target[1]], c="#006d77", s=34, marker="+", linewidths=1.8)
            ax_c.plot([current["seed"][0], target[0]], [current["seed"][1], target[1]], color="#006d77", alpha=0.35, linewidth=0.8)
        face = current.get("selected_face") or current.get("inferred_parent_face")
        if face is not None and current["parent_box_id"] >= 0:
            intervals = current["intervals"]
            dim = int(face["dim"])
            face_value = float(face.get("face_value", intervals[dim][0] if int(face["side"]) < 0 else intervals[dim][1]))
            if dim == 0:
                x = face_value
                y = 0.5 * (intervals[1][0] + intervals[1][1])
                ax_c.arrow(x, y, 0.22 * int(face["side"]), 0.0, width=0.018, color="#ff6f00")
            else:
                x = 0.5 * (intervals[0][0] + intervals[0][1])
                y = face_value
                ax_c.arrow(x, y, 0.0, 0.22 * int(face["side"]), width=0.018, color="#ff6f00")
        iteration = current.get("iteration", "?")
        worker = current.get("worker_id", "?")
        ffb_event = ffb_events.get(task_id, {})
        ffb_label = ffb_event.get("event", "ffb_?")
        target_type = target_event.get("target_type", "?") if target_event is not None else "?"
        ax_c.text(-3.05, 2.82, f"box {upto}/{len(box_events)}  id={current['box_id']}  root={current['root_id']}  it={iteration}  worker={worker}  {target_type}  {ffb_label}", fontsize=9)
        draw_workspace_arm(ax_w, current["seed"], scene, args.link1, args.link2)
        return []

    animation = FuncAnimation(fig, update, frames=len(frame_indices), interval=1000.0 / max(1, args.fps), blit=False)
    try:
        animation.save(out_dir / "growth.gif", writer=PillowWriter(fps=args.fps))
    finally:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2DOF grower trace and C-space/workspace visualization for standalone SBF.")
    parser.add_argument("--scene", choices=sorted(SCENES), default="empty")
    parser.add_argument("--endpoint-source", choices=["ifk", "critsample"], default="critsample")
    parser.add_argument("--split-policy", choices=["best-tighten", "widest-first"], default="best-tighten")
    parser.add_argument("--best-tighten-depth-synchronous", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-shape-balancing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-max-child-aspect", type=float, default=64.0)
    parser.add_argument("--best-tighten-min-split-width-fraction", type=float, default=0.05)
    parser.add_argument("--max-boxes", type=int, default=350)
    parser.add_argument("--timeout-ms", type=float, default=30000.0)
    parser.add_argument("--ffb-depth", type=int, default=80)
    parser.add_argument("--ffb-fail-limit", type=int, default=600)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--task-batch-size", type=int, default=1)
    parser.add_argument("--rrt-goal-bias", type=float, default=0.20)
    parser.add_argument("--intertree-goal-bias", type=float, default=0.30)
    parser.add_argument("--envelope-subdivisions", type=int, default=4)
    parser.add_argument("--seed", type=int, default=20260504)
    parser.add_argument("--link1", type=float, default=1.0)
    parser.add_argument("--link2", type=float, default=1.0)
    parser.add_argument("--collision-resolution", type=float, default=0.055)
    parser.add_argument("--animate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-frames", type=int, default=90)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--trace-max-events", type=int, default=200000)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "outputs" / "grower_2dof_trace")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scene = dict(SCENES[args.scene])
    run_name = f"{args.scene}_{args.endpoint_source}_{args.split_policy.replace('-', '_')}_boxes{args.max_boxes}"
    out_dir = args.out_dir / run_name
    out_dir.mkdir(parents=True, exist_ok=True)
    robot = make_planar_robot(args.link1, args.link2)
    config = build_config(args, out_dir / "events.jsonl")
    forest = sbf.SafeBoxForest(robot, config)
    start = np.array(scene["start"], dtype=float)
    goal = np.array(scene["goal"], dtype=float)
    obstacles = make_obstacles(scene)
    profile = forest.build(start, goal, obstacles)
    query = forest.query(start, goal)

    events = write_trace(out_dir, args, args.scene, scene, forest, profile, query)
    render_visualization(out_dir, scene, events, args)
    print(json.dumps({
        "out_dir": str(out_dir),
        "raw_boxes": int(profile.raw_boxes),
        "final_boxes": int(profile.final_boxes),
        "islands": int(profile.adjacency_islands),
        "success": bool(query.success),
        "path_length": float(query.path_length),
        "events": len(events),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
