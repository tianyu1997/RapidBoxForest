#include <SBF/safe_box_forest.h>

#include <SBF/leaf_sweep_grower.h>

#include <algorithm>

namespace rbf {

BuildProfile RBFPlanningForest::build(const Eigen::Ref<const Eigen::VectorXd>& start,
                                      const Eigen::Ref<const Eigen::VectorXd>& goal,
                                      const std::vector<Obstacle>& obstacles) {
    StageContext context = StageContext::from_runtime(config_.runtime);
    return build(start, goal, obstacles, context);
}

BuildProfile RBFPlanningForest::build(const Eigen::Ref<const Eigen::VectorXd>& start,
                                      const Eigen::Ref<const Eigen::VectorXd>& goal,
                                      const std::vector<Obstacle>& obstacles,
                                      StageContext& context) {
    return build_coverage(obstacles, {start, goal}, context);
}

BuildProfile RBFPlanningForest::build_coverage(const std::vector<Obstacle>& obstacles,
                                               const std::vector<Eigen::VectorXd>& seeds) {
    StageContext context = StageContext::from_runtime(config_.runtime);
    return build_coverage(obstacles, seeds, context);
}

LeafSweepResult RBFPlanningForest::build_leaf_sweep(const std::vector<Obstacle>& obstacles,
                                                    int start_depth,
                                                    int max_depth,
                                                    const LeafSweepConfig& leaf_sweep_config) {
    LeafSweepConfig active_config = leaf_sweep_config;
    const int configured_threads = active_config.n_threads > 0
        ? active_config.n_threads
        : std::max(1, config_.runtime.n_threads);
    active_config.n_threads = configured_threads;
    RuntimeConfig runtime = config_.runtime;
    runtime.mode = configured_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = configured_threads;
    runtime.batch_size = std::max(1, active_config.validation_batch_size);
    StageContext context(runtime, Deadline::after_ms(active_config.timeout_ms));
    return build_leaf_sweep(obstacles, start_depth, max_depth, active_config, context);
}

LeafSweepResult RBFPlanningForest::build_leaf_sweep(const std::vector<Obstacle>& obstacles,
                                                    int start_depth,
                                                    int max_depth,
                                                    const LeafSweepConfig& leaf_sweep_config,
                                                    StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "forest.build_leaf_sweep");
    last_build_seeds_.clear();
    scene_.set_obstacles(obstacles);
    const bool previous_stateless_materialization = config_.validation.stateless_materialization_context;
    if (leaf_sweep_config.use_virtual_topology) {
        config_.validation.stateless_materialization_context = true;
    }
    reset_oracle(scene_);
    boxes_.clear();
    raw_boxes_.clear();
    adjacency_.clear();
    segment_edges_.clear();
    clear_dynamic_collision_cache();
    invalidate_query_cache();

    LeafSweepConfig active_config = leaf_sweep_config;
    if (active_config.n_threads <= 0) {
        active_config.n_threads = std::max(1, context.executor().n_threads());
    }
    if (!active_config.use_virtual_topology && oracle_ &&
        oracle_->native_root_interval_copies().size() > 1) {
        active_config.use_virtual_topology = true;
        config_.validation.stateless_materialization_context = true;
        reset_oracle(scene_);
        context.diagnostics().add_counter("forest.leaf_sweep_forced_virtual_for_native_sectors");
    }
    LeafSweepGrower grower(*oracle_, active_config, config_.grower.find_free_box.split);
    LeafSweepResult result = grower.sweep(obstacles, start_depth, max_depth, context);

    scene_.set_obstacles(obstacles);
    config_.validation.stateless_materialization_context = previous_stateless_materialization;
    oracle_->set_scene(scene_);
    boxes_ = result.free_boxes;
    raw_boxes_ = boxes_;
    populate_dynamic_collision_cache(result, static_cast<int>(obstacles.size()));
    reserve_existing_boxes();
    adjacency_.clear();
    segment_edges_.clear();
    invalidate_query_cache();

    last_build_ = {};
    last_build_.grow_ms = result.total_ms;
    last_build_.raw_boxes = static_cast<int>(raw_boxes_.size());
    last_build_.final_boxes = static_cast<int>(boxes_.size());
    last_build_.total_ms = result.total_ms;
    last_build_.diagnostics = result.diagnostics;
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return result;
}

} // namespace rbf
