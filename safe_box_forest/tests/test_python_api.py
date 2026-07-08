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
    assert hasattr(sbf, "SBFConfig")
    assert hasattr(sbf, "SafeBoxForest")
    assert not hasattr(sbf, "RBFPlanningConfig")
    assert not hasattr(sbf, "RBFPlanningForest")
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
    config.database.canonical_mode = False
    config.database.path = tempfile.mkdtemp(prefix="sbf-python-smoke-")
    config.database.checkpoint_after_build = False

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

    assert not hasattr(sbf, "DynamicUpdateConfig")
    assert not hasattr(sbf, "RebuildProfile")
    assert not hasattr(sbf, "SubtractiveObstacleGroup")
    assert not hasattr(sbf, "SubtractiveBuildOptions")
    diagnostic_only_methods = [
        "oracle_counters",
        "build_subtractive",
        "debug_chain_pave",
        "debug_chain_pave_waypoints",
        "refine_query_corridor",
        "add_obstacle_and_rebuild",
        "add_obstacles_and_rebuild",
        "connect_update_segment_fallback",
        "connect_update_endpoint_segment_fallback",
        "remove_obstacle_and_regrow",
        "remove_obstacle_suffix_and_regrow",
    ]
    for method_name in diagnostic_only_methods:
        assert not hasattr(forest, method_name)
    sweep_config = sbf.LeafSweepConfig()
    sweep = forest.build_leaf_sweep([], 1, 1, sweep_config)
    assert len(sweep.free_boxes) == 2
    assert len(sweep.collision_boxes) == 0
    assert len(sweep.collision_box_obstacle_indices) == 0
    assert len(forest.boxes()) == len(sweep.free_boxes)
    refine_config = sbf.LeafSweepRefineConfig()
    refine_config.leaf_start_depth = 1
    refine_config.leaf_max_depth = 1
    refine_config.deep_max_boxes = 2
    refined = forest.build_leaf_sweep_refined([], refine_config, [])
    assert refined.leaf_free_count == 2
    assert refined.leaf_collision_count == 0
    assert refined.deep_boxes_added == 0
    assert refined.profile.final_boxes == len(forest.boxes())
    assert isinstance(refined.diagnostics, dict)
    print("SBF Python smoke test passed.")


if __name__ == "__main__":
    main()
