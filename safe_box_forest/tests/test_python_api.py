import sbf
import tempfile


def make_toy_robot():
    dh0 = sbf.DHParam()
    dh0.a = 0.35
    dh1 = sbf.DHParam()
    dh1.a = 0.30
    limits = sbf.JointLimits()
    limits.limits = [sbf.Interval(-1.0, 1.0), sbf.Interval(-1.0, 1.0)]
    return sbf.Robot("toy2", [dh0, dh1], limits, None, [0.03, 0.03])


def main():
    assert sbf.__version__ == "0.1.0"
    assert hasattr(sbf, "path_length")
    assert sbf.path_length([]) == 0.0
    config = sbf.SBFConfig()
    config.runtime.mode = sbf.ExecutionMode.Parallel
    config.runtime.n_threads = 2
    config.grower.n_threads = 2
    config.connector.n_threads = 2
    config.connector.rrt.max_iters = 32
    config.merger.candidate_batch_size = 8
    config.enable_merger = False
    config.enable_connector = False
    config.grower.max_boxes = 12
    config.grower.timeout_ms = 1000.0
    config.grower.max_consecutive_miss = 128
    stage = sbf.GrowerDepthStage()
    stage.ffb_depth = 4
    stage.box_limit = 4
    config.grower.depth_stages = [stage]
    config.grower.failure_cooling_enabled = True
    config.grower.failure_cooling_threshold = 2
    config.grower.failure_cooling_box_horizon = 8
    config.endpoint_source.source = sbf.EndpointSource.IFK
    config.envelope_type.type = sbf.EnvelopeType.LinkIAABB
    config.database.path = tempfile.mkdtemp(prefix="sbf-python-smoke-")
    config.database.checkpoint_after_build = False
    config.dynamic_update.dirty_region_padding = 100.0
    config.dynamic_update.local_regrow_box_limit = 2

    robot = make_toy_robot()
    assert robot.n_joints() == 2
    forest = sbf.SafeBoxForest(robot, config)
    profile = forest.build([-0.5, -0.5], [0.5, 0.5], [])
    assert profile.final_boxes > 0
    assert isinstance(profile.diagnostics, dict)
    assert len(forest.boxes()) == profile.final_boxes

    query = forest.query([-0.5, -0.5], [0.5, 0.5])
    assert query.success
    assert query.path_length >= 0.0

    rebuild = forest.add_obstacle_and_rebuild(sbf.Obstacle(5.0, 5.0, 5.0, 6.0, 6.0, 6.0))
    assert rebuild.boxes_after == len(forest.boxes())
    assert hasattr(sbf, "DynamicUpdateConfig")
    removal = forest.remove_obstacle_and_regrow(0)
    assert removal.obstacles_after == 0
    assert removal.boxes_after == len(forest.boxes())
    assert hasattr(forest, "remove_obstacle_suffix_and_regrow")
    print("SBF Python smoke test passed.")


if __name__ == "__main__":
    main()