#include <SBF/safe_box_forest.h>

#include <SBF/scene.h>
#include <SBF/adaptive_grid_partition.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>
#include <SBF/grower.h>
#include <SBF/leaf_sweep_grower.h>
#include <SBF/merger.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <chrono>
#include <string>

#include "../planning_core/planning_forest_diagnostics.h"

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

BuildProfile RBFPlanningForest::build_coverage(const std::vector<Obstacle>& obstacles,
                                               const std::vector<Eigen::VectorXd>& seeds,
                                               StageContext& context) {
    using Clock = std::chrono::steady_clock;
    ScopedStageTimer function_timer(context.diagnostics(), "forest.build_coverage");
    const auto t0 = Clock::now();
    last_build_seeds_ = seeds;
    scene_.set_obstacles(obstacles);
    reset_oracle(scene_);
    boxes_.clear();
    raw_boxes_.clear();
    adjacency_.clear();
	segment_edges_.clear();
	adaptive_partition_.reset();
	adaptive_partition_query_enabled_ = false;
	has_adaptive_partition_config_ = false;
	clear_optional_collision_cache();
	invalidate_query_cache();

    const auto grow_t0 = Clock::now();
    auto grower = make_grower(*oracle_, config_.grower);
    auto grow = grower->grow(seeds, context);
    context.diagnostics().record_timing(
        "forest.grow_stage",
        std::chrono::duration<double, std::milli>(Clock::now() - grow_t0).count());
    boxes_ = std::move(grow.boxes);
    raw_boxes_ = boxes_;
    adjacency_ = std::move(grow.adjacency);
    last_build_ = {};
    last_build_.grow_ms = grow.build_time_ms;
    last_build_.raw_boxes = static_cast<int>(raw_boxes_.size());
    last_build_.grow_adjacency_islands = grow.adjacency_islands;
    last_build_.grow_largest_island = grow.adjacency_largest_island;

    const auto merge_t0 = Clock::now();
    if (config_.enable_merger && !boxes_.empty()) {
        Consolidator consolidator(*oracle_, config_.merger);
        consolidator.run(boxes_, context);
    }
    last_build_.merge_ms = std::chrono::duration<double, std::milli>(Clock::now() - merge_t0).count();
    context.diagnostics().record_timing("forest.merge_stage", last_build_.merge_ms);

    const auto connector_t0 = Clock::now();
    bool connector_ran = false;
    if (config_.enable_connector && !boxes_.empty() && !context.should_stop()) {
        rebuild_adjacency();
        CollisionChecker checker(robot_, scene_);
        IslandConnectorConfig connector_config = config_.connector;
        if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
            connector_config.n_threads = config_.runtime.n_threads;
        }
        IslandConnector connector(*oracle_, robot_, checker, connector_config);
        int next_id = next_box_id();
        const auto connector_result = connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
        last_build_.bridge_boxes_added = connector_result.bridge_boxes_added;
        last_build_.segment_edges_added = connector_result.segment_edges_added;
        last_build_.rrt_segment_edges_added = connector_result.rrt_segment_edges_added;
        last_build_.point_gap_segment_edges_added = connector_result.point_gap_segment_edges_added;
        last_build_.connector_attempted_pairs = connector_result.attempted_pairs;
        last_build_.connector_connected = connector_result.connected;
        connector_ran = true;
    }
    last_build_.connector_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();
    context.diagnostics().record_timing("forest.connector_stage", last_build_.connector_ms);

    const auto adj_t0 = Clock::now();
    last_build_.segment_edges = static_cast<int>(segment_edges_.size());
    if (!connector_ran) {
        rebuild_adjacency();
    }
    last_build_.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    context.diagnostics().record_timing("forest.adjacency_stage", last_build_.adjacency_ms);
    last_build_.final_boxes = static_cast<int>(boxes_.size());
    last_build_.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    context.diagnostics().set_value("grower.adjacency_islands", static_cast<double>(last_build_.grow_adjacency_islands));
    context.diagnostics().set_value("grower.adjacency_largest_island", static_cast<double>(last_build_.grow_largest_island));
    const OracleCounters oracle_counters = oracle_->counters();
    context.diagnostics().set_value("oracle.node_validations", static_cast<double>(oracle_counters.node_validations));
    context.diagnostics().set_value("oracle.interval_validations", static_cast<double>(oracle_counters.interval_validations));
    context.diagnostics().set_value("oracle.materializations", static_cast<double>(oracle_counters.materializations));
    context.diagnostics().set_value("oracle.materialization_stored_endpoint", static_cast<double>(oracle_counters.materialization_stored_endpoint));
    context.diagnostics().set_value("oracle.materialization_skipped_endpoint_cache", static_cast<double>(oracle_counters.materialization_skipped_endpoint_cache));
    context.diagnostics().set_value("oracle.materialization_incremental_fk", static_cast<double>(oracle_counters.materialization_incremental_fk));
    context.diagnostics().set_value("oracle.materialization_source_incremental_state", static_cast<double>(oracle_counters.materialization_source_incremental_state));
    context.diagnostics().set_value("oracle.materialization_reused_fk", static_cast<double>(oracle_counters.materialization_reused_fk));
    context.diagnostics().set_value("oracle.materialization_reused_endpoint_cache", static_cast<double>(oracle_counters.materialization_reused_endpoint_cache));
    context.diagnostics().set_value("oracle.materialization_reused_external_evidence", static_cast<double>(oracle_counters.materialization_reused_external_evidence));
    context.diagnostics().set_value("oracle.materialization_external_exact_hits", static_cast<double>(oracle_counters.materialization_external_exact_hits));
    context.diagnostics().set_value("oracle.materialization_external_exact_misses", static_cast<double>(oracle_counters.materialization_external_exact_misses));
    context.diagnostics().set_value("oracle.materialization_external_live_fallbacks", static_cast<double>(oracle_counters.materialization_external_live_fallbacks));
    context.diagnostics().set_value("oracle.materialization_external_maybe_live_retries", static_cast<double>(oracle_counters.materialization_external_maybe_live_retries));
    context.diagnostics().set_value("oracle.materialization_external_maybe_live_retry_free", static_cast<double>(oracle_counters.materialization_external_maybe_live_retry_free));
    context.diagnostics().set_value("oracle.interval_replay_compatibility_checks", static_cast<double>(oracle_counters.interval_replay_compatibility_checks));
    context.diagnostics().set_value("oracle.interval_replay_compatible", static_cast<double>(oracle_counters.interval_replay_compatible));
    context.diagnostics().set_value("oracle.interval_replay_incompatible", static_cast<double>(oracle_counters.interval_replay_incompatible));
    context.diagnostics().set_value("oracle.interval_replay_direct_exact_hits", static_cast<double>(oracle_counters.interval_replay_direct_exact_hits));
    context.diagnostics().set_value("oracle.interval_replay_key_only_blocked", static_cast<double>(oracle_counters.interval_replay_key_only_blocked));
    context.diagnostics().set_value("oracle.canonical_frame_invalid", static_cast<double>(oracle_counters.canonical_frame_invalid));
    context.diagnostics().set_value("oracle.canonical_reflected_seed_misses", static_cast<double>(oracle_counters.canonical_reflected_seed_misses));
    context.diagnostics().set_value("oracle.materialization_reused_shared_endpoint_cache", static_cast<double>(oracle_counters.materialization_reused_shared_endpoint_cache));
    context.diagnostics().set_value("oracle.materialization_stored_shared_endpoint_cache", static_cast<double>(oracle_counters.materialization_stored_shared_endpoint_cache));
    if (const auto* shared_cache = oracle_->shared_endpoint_cache_peek()) {
        context.diagnostics().set_value("oracle.shared_endpoint_cache_size", static_cast<double>(shared_cache->size()));
        context.diagnostics().set_value("oracle.shared_endpoint_cache_bytes", static_cast<double>(shared_cache->bytes()));
        context.diagnostics().set_value("oracle.shared_endpoint_cache_evictions", static_cast<double>(shared_cache->evictions()));
    }
    context.diagnostics().set_value("oracle.materialization_endpoint_time_us", oracle_counters.materialization_endpoint_time_us);
    context.diagnostics().set_value("oracle.materialization_endpoint_wall_time_us", oracle_counters.materialization_endpoint_wall_time_us);
    context.diagnostics().set_value("oracle.validate_node_total_time_us", oracle_counters.validate_node_total_time_us);
    context.diagnostics().set_value("oracle.validate_node_preamble_time_us", oracle_counters.validate_node_preamble_time_us);
    context.diagnostics().set_value("oracle.validate_node_endpoint_path_time_us", oracle_counters.validate_node_endpoint_path_time_us);
    context.diagnostics().set_value("oracle.validate_node_classify_time_us", oracle_counters.validate_node_classify_time_us);
    context.diagnostics().set_value("oracle.validate_node_overhead_time_us", oracle_counters.validate_node_overhead_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_time_us", oracle_counters.materialization_envelope_time_us);
    context.diagnostics().set_value("oracle.materialization_cache_lookup_time_us", oracle_counters.materialization_cache_lookup_time_us);
    context.diagnostics().set_value("oracle.materialization_cache_read_time_us", oracle_counters.materialization_cache_read_time_us);
    context.diagnostics().set_value("oracle.materialization_external_lookup_time_us", oracle_counters.materialization_external_lookup_time_us);
    context.diagnostics().set_value("oracle.materialization_external_read_time_us", oracle_counters.materialization_external_read_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_compute_time_us", oracle_counters.materialization_envelope_compute_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_read_time_us", oracle_counters.materialization_envelope_read_time_us);
    context.diagnostics().set_value("oracle.materialization_envelope_collision_time_us", oracle_counters.materialization_envelope_collision_time_us);
    context.diagnostics().set_value("oracle.materialization_candidate_dirty_count", static_cast<double>(oracle_counters.materialization_candidate_dirty_count));
    context.diagnostics().set_value("oracle.materialization_predh_rebuild_count", static_cast<double>(oracle_counters.materialization_predh_rebuild_count));
    context.diagnostics().set_value("oracle.scoring_evaluations", static_cast<double>(oracle_counters.scoring_evaluations));
    context.diagnostics().set_value("oracle.scoring_changed_dim_inferred", static_cast<double>(oracle_counters.scoring_changed_dim_inferred));
    context.diagnostics().set_value("oracle.scoring_incremental_fk", static_cast<double>(oracle_counters.scoring_incremental_fk));
    context.diagnostics().set_value("oracle.scoring_source_incremental_state", static_cast<double>(oracle_counters.scoring_source_incremental_state));
    context.diagnostics().set_value("oracle.scoring_reused_fk", static_cast<double>(oracle_counters.scoring_reused_fk));
    context.diagnostics().set_value("oracle.scoring_reused_endpoint_cache", static_cast<double>(oracle_counters.scoring_reused_endpoint_cache));
    context.diagnostics().set_value("oracle.scoring_reused_external_evidence", static_cast<double>(oracle_counters.scoring_reused_external_evidence));
    context.diagnostics().set_value("oracle.scoring_endpoint_time_us", oracle_counters.scoring_endpoint_time_us);
    context.diagnostics().set_value("oracle.scoring_envelope_time_us", oracle_counters.scoring_envelope_time_us);
    context.diagnostics().set_value("oracle.scoring_candidate_dirty_count", static_cast<double>(oracle_counters.scoring_candidate_dirty_count));
    context.diagnostics().set_value("oracle.scoring_predh_rebuild_count", static_cast<double>(oracle_counters.scoring_predh_rebuild_count));
    context.diagnostics().set_value("oracle.certified_free", static_cast<double>(oracle_counters.certified_free));
    context.diagnostics().set_value("oracle.certified_occupied", static_cast<double>(oracle_counters.certified_occupied));
    context.diagnostics().set_value("oracle.provisional_free", static_cast<double>(oracle_counters.provisional_free));
    context.diagnostics().set_value("oracle.collision_possible", static_cast<double>(oracle_counters.collision_possible));
    context.diagnostics().set_value("oracle.validation_cache_hits", static_cast<double>(oracle_counters.validation_cache_hits));
    context.diagnostics().set_value("oracle.validation_cache_misses", static_cast<double>(oracle_counters.validation_cache_misses));
    context.diagnostics().set_value("oracle.unsafe_free_rejected", static_cast<double>(oracle_counters.unsafe_free_rejected));
    context.diagnostics().set_value("oracle.envelope_collision_queries", static_cast<double>(oracle_counters.envelope_collision_queries));
    context.diagnostics().set_value("oracle.envelope_collision_free", static_cast<double>(oracle_counters.envelope_collision_free));
    context.diagnostics().set_value("oracle.envelope_collision_maybe", static_cast<double>(oracle_counters.envelope_collision_maybe));
    context.diagnostics().set_value("oracle.envelope_collision_envelope_aabb_tests", static_cast<double>(oracle_counters.envelope_collision_envelope_aabb_tests));
    context.diagnostics().set_value("oracle.envelope_collision_envelope_aabb_rejects", static_cast<double>(oracle_counters.envelope_collision_envelope_aabb_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_link_union_aabb_tests", static_cast<double>(oracle_counters.envelope_collision_link_union_aabb_tests));
    context.diagnostics().set_value("oracle.envelope_collision_link_union_aabb_rejects", static_cast<double>(oracle_counters.envelope_collision_link_union_aabb_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_link_aabb_tests", static_cast<double>(oracle_counters.envelope_collision_link_aabb_tests));
    context.diagnostics().set_value("oracle.envelope_collision_link_aabb_rejects", static_cast<double>(oracle_counters.envelope_collision_link_aabb_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_kdop_tests", static_cast<double>(oracle_counters.envelope_collision_kdop_tests));
    context.diagnostics().set_value("oracle.envelope_collision_kdop_rejects", static_cast<double>(oracle_counters.envelope_collision_kdop_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_kdop_axes_tested", static_cast<double>(oracle_counters.envelope_collision_kdop_axes_tested));
    context.diagnostics().set_value("oracle.envelope_collision_gjk_tests", static_cast<double>(oracle_counters.envelope_collision_gjk_tests));
    context.diagnostics().set_value("oracle.envelope_collision_gjk_rejects", static_cast<double>(oracle_counters.envelope_collision_gjk_rejects));
    context.diagnostics().set_value("oracle.envelope_collision_gjk_iterations", static_cast<double>(oracle_counters.envelope_collision_gjk_iterations));
    record_portal_membership_policy(context.diagnostics(), config_.portal_membership_policy);
    last_build_.diagnostics = context.diagnostics().snapshot();
    last_build_.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();

    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return last_build_;
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
	clear_optional_collision_cache();
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
	populate_optional_collision_cache_from_leaf_sweep(result, static_cast<int>(obstacles.size()));
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
