from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


PAIR_COLORS = [
    (0.12, 0.47, 0.71, 1.0),
    (1.00, 0.50, 0.05, 1.0),
    (0.17, 0.63, 0.17, 1.0),
    (0.84, 0.15, 0.16, 1.0),
    (0.58, 0.40, 0.74, 1.0),
]


def default_gcs_repo() -> Path:
    env = os.environ.get("GCS_REPO")
    if env:
        return Path(env).expanduser().resolve()
    file_path = Path(__file__).resolve()
    candidates = [ancestor / "gcs-science-robotics" for ancestor in file_path.parents]
    candidates.append(Path.cwd() / "gcs-science-robotics")
    for candidate in candidates:
        if (candidate / "models").exists():
            return candidate.resolve()
    return candidates[0].resolve()


def directives_file(gcs_repo: Path | str | None = None, *, collision_spheres: bool = True) -> Path:
    root = Path(gcs_repo).resolve() if gcs_repo is not None else default_gcs_repo()
    name = "iiwa14_spheres_collision_welded_gripper.yaml" if collision_spheres else "iiwa14_welded_gripper.yaml"
    return root / "models" / name


def waypoints_to_trajectory(waypoints: Sequence[Sequence[float]], speed: float = 1.5):
    from pydrake.trajectories import PiecewisePolynomial

    points = np.asarray(waypoints, dtype=float)
    if points.ndim != 2 or points.shape[0] < 2:
        raise ValueError("waypoints must be an N x dof array with N >= 2")
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    distances = np.maximum(distances, 1e-6)
    times = np.concatenate([[0.0], np.cumsum(distances)]) / max(float(speed), 1e-6)
    return PiecewisePolynomial.FirstOrderHold(times, points.T)


def _load_scene(builder, plant, parser, gcs_repo: Path, *, collision_spheres: bool) -> None:
    from pydrake.multibody.parsing import LoadModelDirectives, ProcessModelDirectives

    parser.package_map().Add("gcs", str(gcs_repo))
    model_file = directives_file(gcs_repo, collision_spheres=collision_spheres)
    if not model_file.exists():
        raise FileNotFoundError(f"Drake directives not found: {model_file}")
    directives = LoadModelDirectives(str(model_file))
    ProcessModelDirectives(directives, plant, parser)


def build_static_scene(meshcat, gcs_repo: Path | str | None = None, *, collision_spheres: bool = False):
    from pydrake.geometry import MeshcatVisualizer, MeshcatVisualizerParams, Role
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.framework import DiagramBuilder

    root = Path(gcs_repo).resolve() if gcs_repo is not None else default_gcs_repo()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    _load_scene(builder, plant, parser, root, collision_spheres=collision_spheres)
    plant.Finalize()

    params = MeshcatVisualizerParams()
    params.delete_on_initialization_event = False
    params.role = Role.kIllustration
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.ForcedPublish(context)
    return diagram, plant


def _end_effector_body(plant):
    for name in ("body", "iiwa_link_7"):
        try:
            return plant.GetBodyByName(name)
        except RuntimeError:
            continue
    raise RuntimeError("could not find end-effector body 'body' or 'iiwa_link_7'")


def draw_end_effector_paths(meshcat, plant, waypoints_list: Sequence[Sequence[Sequence[float]]], labels: Sequence[str]) -> None:
    from pydrake.geometry import Rgba
    from pydrake.perception import PointCloud

    context = plant.CreateDefaultContext()
    ee_body = _end_effector_body(plant)
    for index, waypoints in enumerate(waypoints_list):
        trajectory = waypoints_to_trajectory(waypoints, speed=1.0)
        n_points = max(int(trajectory.end_time() * 160), 80)
        times = np.linspace(trajectory.start_time(), trajectory.end_time(), n_points)
        positions = []
        for sample_time in times:
            q = trajectory.value(float(sample_time)).flatten()
            plant.SetPositions(context, q)
            positions.append(plant.EvalBodyPoseInWorld(context, ee_body).translation())
        cloud = PointCloud(n_points)
        cloud.mutable_xyzs()[:] = np.asarray(positions, dtype=float).T
        color = Rgba(*PAIR_COLORS[index % len(PAIR_COLORS)])
        meshcat.SetObject(f"sbf_paths/{labels[index]}", cloud, 0.009, rgba=color)


def visualize_paths(
    waypoints_list: Sequence[Sequence[Sequence[float]]],
    labels: Sequence[str],
    *,
    gcs_repo: Path | str | None = None,
    save_html: Path | str | None = None,
    static: bool = True,
    speed: float = 1.5,
    no_show: bool = True,
) -> dict[str, object]:
    from pydrake.geometry import StartMeshcat

    if not waypoints_list:
        raise ValueError("no successful paths to visualize")
    root = Path(gcs_repo).resolve() if gcs_repo is not None else default_gcs_repo()
    meshcat = StartMeshcat()
    meshcat.Delete()
    web_url = meshcat.web_url()

    if static:
        diagram, plant = build_static_scene(meshcat, root, collision_spheres=False)
        draw_end_effector_paths(meshcat, plant, waypoints_list, labels)
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyMutableContextFromRoot(context)
        plant.SetPositions(plant_context, np.asarray(waypoints_list[-1][-1], dtype=float))
        diagram.ForcedPublish(context)
        animation_time_s = 0.0
    else:
        animation_time_s = animate_paths(meshcat, waypoints_list, root, speed=speed)

    html_path = None
    if save_html is not None:
        html_path = Path(save_html).resolve()
        html_path.parent.mkdir(parents=True, exist_ok=True)
        html_path.write_text(meshcat.StaticHtml(), encoding="utf-8")

    if not no_show:
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    return {"meshcat_url": web_url, "html": str(html_path) if html_path is not None else None, "animation_time_s": animation_time_s}


def animate_paths(meshcat, waypoints_list: Sequence[Sequence[Sequence[float]]], gcs_repo: Path, *, speed: float = 1.5) -> float:
    from pydrake.geometry import MeshcatVisualizer, SceneGraph
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant
    from pydrake.systems.analysis import Simulator
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.systems.primitives import TrajectorySource
    from pydrake.systems.rendering import MultibodyPositionToGeometryPose
    from pydrake.trajectories import PiecewisePolynomial

    segments = [waypoints_to_trajectory(path, speed=speed) for path in waypoints_list]
    pause_s = 0.6
    offsets = []
    cursor = 0.0
    for segment in segments:
        offsets.append(cursor)
        cursor += segment.end_time() + pause_s
    total_time = max(cursor - pause_s, 0.0)

    def eval_path(sample_time: float) -> np.ndarray:
        for index, segment in enumerate(segments):
            local = sample_time - offsets[index]
            if local <= segment.end_time() or index == len(segments) - 1:
                return segment.value(float(np.clip(local, segment.start_time(), segment.end_time()))).flatten()
        return segments[-1].value(segments[-1].end_time()).flatten()

    n_samples = max(int(total_time * 100), 200)
    times = np.linspace(0.0, total_time, n_samples)
    values = np.asarray([eval_path(float(t)) for t in times], dtype=float).T
    trajectory = PiecewisePolynomial.FirstOrderHold(times, values)

    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant)
    _load_scene(builder, plant, parser, gcs_repo, collision_spheres=False)
    plant.Finalize()

    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(to_pose.get_output_port(), scene_graph.get_source_pose_port(plant.get_source_id()))
    source = builder.AddSystem(TrajectorySource(trajectory))
    builder.Connect(source.get_output_port(), to_pose.get_input_port())
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    simulator = Simulator(diagram)
    visualizer.StartRecording()
    simulator.AdvanceTo(total_time)
    visualizer.PublishRecording()
    return float(total_time)
