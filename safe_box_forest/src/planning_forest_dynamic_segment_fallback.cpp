#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "adaptive_grid_partition_options.h"
#include "planning_forest_audit.h"
#include "planning_forest_dynamic_helpers.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

namespace rbf {

namespace {

bool point_covered_by_existing_box_local(const std::vector<BoxNode>& boxes,
                                         const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return intervals_contain_point_strict_local(box.joint_intervals, point);
    });
}

void merge_diagnostic_snapshot_local(std::unordered_map<std::string, double>& diagnostics,
                                     const std::unordered_map<std::string, double>& snapshot) {
    for (const auto& [key, value] : snapshot) {
        diagnostics[key] += value;
    }
}

void initialize_segment_fallback_profile(RebuildProfile& profile,
                                         int boxes_size,
                                         int raw_boxes_size,
                                         int obstacle_count,
                                         int collision_cache_size,
                                         int segment_edge_count) {
    profile.boxes_before = boxes_size;
    profile.raw_boxes_before = raw_boxes_size;
    profile.obstacles_before = obstacle_count;
    profile.obstacles_after = obstacle_count;
    profile.collision_cache_boxes_before = collision_cache_size;
    profile.collision_cache_boxes_after = collision_cache_size;
    profile.diagnostics["segment_fallback.segment_edges_before"] =
        static_cast<double>(segment_edge_count);
}

void record_segment_fallback_partition_stats(RebuildProfile& profile,
                                             const AdaptiveGridPartition& partition,
                                             int segment_edge_count) {
    profile.diagnostics["segment_fallback.segment_edges_after"] =
        static_cast<double>(segment_edge_count);
    profile.diagnostics["segment_fallback.islands_after"] =
        static_cast<double>(profile.adjacency_islands);
    const auto& partition_stats = partition.stats();
    profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(partition_stats.cells);
    profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(partition_stats.islands);
    profile.diagnostics["adaptive.partition_overlay_edges"] =
        static_cast<double>(partition_stats.overlay_edges);
}

} // namespace

RebuildProfile RBFPlanningForest::connect_update_endpoint_segment_fallback(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    initialize_segment_fallback_profile(profile,
                                        static_cast<int>(boxes_.size()),
                                        static_cast<int>(raw_boxes_.size()),
                                        scene_.n_obstacles(),
                                        static_cast<int>(dynamic_collision_box_cache_.size()),
                                        static_cast<int>(segment_edges_.size()));
    const bool use_partition_backend =
        partition_native_mode() && adaptive_partition_query_enabled_ && adaptive_partition_;
    const int islands_before = use_partition_backend
        ? adaptive_partition_->component_count_with_overlay()
        : static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_before"] = static_cast<double>(islands_before);

    if (!oracle_ || boxes_.empty() || start.size() != oracle_->n_dims() || goal.size() != oracle_->n_dims()) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.adjacency_islands = islands_before;
        profile.fallback_reason = boxes_.empty() ? "empty_forest" : !oracle_ ? "missing_oracle" : "bad_endpoint_dimension";
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    if (use_partition_backend) {
        const auto endpoint_t0 = Clock::now();
        const std::size_t boxes_before_partition = boxes_.size();
        const int edges_before = static_cast<int>(segment_edges_.size());
        int start_box = locate_box_partition_first(start, config_.query.nearest_if_outside);
        int goal_box = locate_box_partition_first(goal, config_.query.nearest_if_outside);
        StageContext context = StageContext::from_runtime(config_.runtime);
        if (start_box < 0) {
            start_box = anchor_query_endpoint_box(start, context);
            profile.regrow_attempts += 1;
        } else {
            profile.diagnostics["segment_fallback.start_already_covered"] += 1.0;
        }
        if (goal_box < 0) {
            goal_box = anchor_query_endpoint_box(goal, context);
            profile.regrow_attempts += 1;
        } else {
            profile.diagnostics["segment_fallback.goal_already_covered"] += 1.0;
        }
        merge_diagnostic_snapshot_local(profile.diagnostics, context.diagnostics().snapshot());
        if (boxes_.size() > boxes_before_partition) {
            append_adaptive_partition_boxes(boxes_before_partition,
                                            &last_build_,
                                            "segment_fallback.endpoint_partition");
        }
        start_box = locate_box_partition_first(start, config_.query.nearest_if_outside);
        goal_box = locate_box_partition_first(goal, config_.query.nearest_if_outside);
        if (start_box >= 0 &&
            goal_box >= 0 &&
            !overlay_path_connected_partition_first(start_box, goal_box)) {
            EndpointMainBoxCorridorConfig corridor_config;
            (void)connect_query_endpoint_to_main_box_corridor(start, corridor_config);
            (void)connect_query_endpoint_to_main_box_corridor(goal, corridor_config);
        }
        start_box = locate_box_partition_first(start, config_.query.nearest_if_outside);
        goal_box = locate_box_partition_first(goal, config_.query.nearest_if_outside);
        if (start_box >= 0 &&
            goal_box >= 0 &&
            !overlay_path_connected_partition_first(start_box, goal_box)) {
            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
            std::vector<Eigen::VectorXd> waypoints{start, goal};
            const auto audit = audit_waypoint_path(waypoints,
                                                   checker,
                                                   config_.query.audit_resolution,
                                                   config_.query.audit_segment_step);
            profile.diagnostics["segment_fallback.endpoint_direct_attempts"] += 1.0;
            if (audit.passed) {
                const int edge_id = add_segment_edge_partition_first(start_box,
                                                                     goal_box,
                                                                     std::move(waypoints),
                                                                     SegmentEdgeType::QueryBridge,
                                                                     config_.query.audit_resolution,
                                                                     SegmentEdgeValidation::CollisionChecked,
                                                                     true,
                                                                     -1,
                                                                     nullptr,
                                                                     "segment_fallback.endpoint_partition");
                if (edge_id >= 0) {
                    profile.diagnostics["segment_fallback.endpoint_direct_success"] += 1.0;
                }
            } else {
                profile.diagnostics["segment_fallback.endpoint_direct_audit_fail"] += 1.0;
            }
        }
        sync_adaptive_partition_segment_edges(nullptr, "segment_fallback.endpoint_partition");
        profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - endpoint_t0).count();
        profile.boxes_added = std::max(0, static_cast<int>(boxes_.size()) - profile.boxes_before);
        profile.raw_boxes_added = std::max(0, static_cast<int>(raw_boxes_.size()) - profile.raw_boxes_before);
        profile.segment_edges_added = std::max(0, static_cast<int>(segment_edges_.size()) - edges_before);
        profile.rrt_segment_edges_added = profile.segment_edges_added;
        profile.point_gap_segment_edges_added = 0;
        profile.boxes_after = static_cast<int>(boxes_.size());
        profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
        profile.diagnostics["segment_fallback.endpoint_partition_native"] = 1.0;
        profile.diagnostics["segment_fallback.connected"] =
            (start_box >= 0 && goal_box >= 0 &&
             overlay_path_connected_partition_first(start_box, goal_box)) ? 1.0 : 0.0;
        record_segment_fallback_partition_stats(profile,
                                                *adaptive_partition_,
                                                static_cast<int>(segment_edges_.size()));
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    const auto endpoint_t0 = Clock::now();
    const std::size_t first_new_box_index = boxes_.size();
    StageContext context = StageContext::from_runtime(config_.runtime);
    FindFreeBoxService ffb(*oracle_);
    FindFreeBoxOptions endpoint_options = config_.grower.find_free_box;
    endpoint_options.reject_seed_collision = true;
    int next_id = next_box_id();
    auto try_endpoint = [&](const Eigen::Ref<const Eigen::VectorXd>& point, const char* label) {
        profile.diagnostics[std::string("segment_fallback.") + label + "_attempts"] += 1.0;
        if (point_covered_by_existing_box_local(boxes_, point)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_already_covered"] += 1.0;
            return;
        }
        auto result = ffb.find(point, context, endpoint_options);
        profile.regrow_attempts += 1;
        if (!result.found || !intervals_contain_point_local(result.intervals, point, config_.query.adjacency_tolerance)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_ffb_failed"] += 1.0;
            return;
        }
        if (!allow_dynamic_commit(*oracle_, result, config_.grower.commit_policy)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_commit_rejected"] += 1.0;
            return;
        }
        BoxNode box;
        box.id = next_id++;
        box.joint_intervals = std::move(result.intervals);
        box.seed_config = point;
        box.tree_id = result.node;
        box.parent_box_id = -1;
        box.root_id = box.id;
        box.safety_status = result.validation_detail.safety_status;
        box.strict_audit_required = result.validation_detail.strict_audit_required;
        box.compute_volume();
        for (const auto& existing : boxes_) {
            if (box_contains_box_exact_local(existing, box)) {
                profile.diagnostics[std::string("segment_fallback.") + label + "_contained_rejected"] += 1.0;
                return;
            }
        }
        int adjacent_parent = -1;
        if (!has_adjacency_to_existing_box(boxes_, box, config_.query.adjacency_tolerance, &adjacent_parent)) {
            profile.diagnostics[std::string("segment_fallback.") + label + "_disconnected_rejected"] += 1.0;
            return;
        }
        box.parent_box_id = adjacent_parent;
        const BoxNode* parent = find_box_by_id(boxes_, adjacent_parent);
        box.root_id = parent != nullptr && parent->root_id >= 0 ? parent->root_id : adjacent_parent;
        oracle_->reserve_node(box.tree_id, box.id);
        boxes_.push_back(box);
        raw_boxes_.push_back(box);
        profile.boxes_added += 1;
        profile.raw_boxes_added += 1;
        profile.diagnostics[std::string("segment_fallback.") + label + "_boxes_added"] += 1.0;
    };
    try_endpoint(start, "start");
    try_endpoint(goal, "goal");
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - endpoint_t0).count();

    const auto pre_connector_adj_t0 = Clock::now();
    if (boxes_.size() > first_new_box_index) {
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
    }
    profile.adjacency_ms += std::chrono::duration<double, std::milli>(Clock::now() - pre_connector_adj_t0).count();

    const auto connector_t0 = Clock::now();
    IslandConnectorConfig connector_config = config_.connector;
    connector_config.segment_edges_enabled = true;
    connector_config.rrt_segment_edges = true;
    connector_config.point_gap_segment_edges = true;
    if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
        connector_config.n_threads = config_.runtime.n_threads;
    }
    CollisionChecker checker(robot_, scene_);
    IslandConnector connector(*oracle_, robot_, checker, connector_config);
    const auto connector_result =
        connector.connect_all(boxes_, adjacency_, segment_edges_, next_id, context);
    profile.diagnostics["segment_fallback.connector_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();

    profile.bridge_boxes_added += connector_result.bridge_boxes_added;
    profile.segment_edges_added += connector_result.segment_edges_added;
    profile.rrt_segment_edges_added += connector_result.rrt_segment_edges_added;
    profile.point_gap_segment_edges_added += connector_result.point_gap_segment_edges_added;
    profile.boxes_added = std::max(0, static_cast<int>(boxes_.size()) - profile.boxes_before);
    profile.raw_boxes_added = std::max(0, static_cast<int>(raw_boxes_.size()) - profile.raw_boxes_before);
    profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(connector_result.attempted_pairs);
    profile.diagnostics["segment_fallback.connected"] = connector_result.connected ? 1.0 : 0.0;
    profile.diagnostics["segment_fallback.segment_edges_after"] = static_cast<double>(segment_edges_.size());

    const auto adj_t0 = Clock::now();
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
    profile.adjacency_ms += std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_after"] = static_cast<double>(profile.adjacency_islands);
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return profile;
}

RebuildProfile RBFPlanningForest::connect_update_segment_fallback() {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    initialize_segment_fallback_profile(profile,
                                        static_cast<int>(boxes_.size()),
                                        static_cast<int>(raw_boxes_.size()),
                                        scene_.n_obstacles(),
                                        static_cast<int>(dynamic_collision_box_cache_.size()),
                                        static_cast<int>(segment_edges_.size()));
    const bool use_partition_backend =
        partition_native_mode() && adaptive_partition_query_enabled_ && adaptive_partition_;
    const int islands_before = use_partition_backend
        ? adaptive_partition_->component_count_with_overlay()
        : static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_before"] = static_cast<double>(islands_before);

    if (!oracle_ || boxes_.empty()) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.adjacency_islands = islands_before;
        profile.fallback_reason = boxes_.empty() ? "empty_forest" : "missing_oracle";
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    if (use_partition_backend) {
        const auto connector_t0 = Clock::now();
        if (islands_before <= 1) {
            profile.boxes_after = profile.boxes_before;
            profile.raw_boxes_after = profile.raw_boxes_before;
            profile.adjacency_islands = islands_before;
            profile.diagnostics["segment_fallback.partition_native"] = 1.0;
            profile.diagnostics["segment_fallback.connected"] = 1.0;
            record_segment_fallback_partition_stats(profile,
                                                    *adaptive_partition_,
                                                    static_cast<int>(segment_edges_.size()));
            profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            return profile;
        }
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        int attempted_pairs = 0;
        int audit_fail = 0;
        int added = 0;
        const int pair_candidate_cap = partition_segment_fallback_pair_candidate_cap_from_env();
        const auto candidate_pairs =
            adaptive_partition_->nearest_component_pairs_to_largest(1, pair_candidate_cap);
        for (const auto& pair : candidate_pairs) {
            if (pair.source_box_id < 0 || pair.target_box_id < 0 ||
                pair.source_point.size() == 0 || pair.target_point.size() == 0) {
                continue;
            }
            ++attempted_pairs;
            std::vector<Eigen::VectorXd> waypoints{pair.source_point, pair.target_point};
            if (!audit_waypoint_path_passes(waypoints,
                                            checker,
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step)) {
                ++audit_fail;
                continue;
            }
            const int edge_id = add_segment_edge_partition_first(pair.source_box_id,
                                                                 pair.target_box_id,
                                                                 std::move(waypoints),
                                                                 SegmentEdgeType::QueryBridge,
                                                                 config_.query.audit_resolution,
                                                                 SegmentEdgeValidation::CollisionChecked,
                                                                 true,
                                                                 -1,
                                                                 nullptr,
                                                                 "segment_fallback.partition_native");
            if (edge_id >= 0) {
                ++added;
            }
        }
        profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();
        profile.segment_edges_added = added;
        profile.rrt_segment_edges_added = added;
        profile.point_gap_segment_edges_added = 0;
        profile.boxes_added = 0;
        profile.raw_boxes_added = 0;
        profile.boxes_after = static_cast<int>(boxes_.size());
        profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
        sync_adaptive_partition_segment_edges(nullptr, "segment_fallback.partition_native");
        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
        profile.diagnostics["segment_fallback.partition_native"] = 1.0;
        profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(attempted_pairs);
        profile.diagnostics["segment_fallback.audit_fail"] = static_cast<double>(audit_fail);
        profile.diagnostics["segment_fallback.partition_pair_candidates"] =
            static_cast<double>(candidate_pairs.size());
        profile.diagnostics["segment_fallback.connected"] = profile.adjacency_islands <= 1 ? 1.0 : 0.0;
        record_segment_fallback_partition_stats(profile,
                                                *adaptive_partition_,
                                                static_cast<int>(segment_edges_.size()));
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    const auto connector_t0 = Clock::now();
    StageContext context = StageContext::from_runtime(config_.runtime);
    CollisionChecker checker(robot_, scene_);
    IslandConnectorConfig connector_config = config_.connector;
    connector_config.segment_edges_enabled = true;
    connector_config.rrt_segment_edges = true;
    connector_config.point_gap_segment_edges = true;
    if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
        connector_config.n_threads = config_.runtime.n_threads;
    }
    IslandConnector connector(*oracle_, robot_, checker, connector_config);
    int connector_next_id = next_box_id();
    const auto connector_result =
        connector.connect_all(boxes_, adjacency_, segment_edges_, connector_next_id, context);
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_t0).count();

    profile.bridge_boxes_added = connector_result.bridge_boxes_added;
    profile.segment_edges_added = connector_result.segment_edges_added;
    profile.rrt_segment_edges_added = connector_result.rrt_segment_edges_added;
    profile.point_gap_segment_edges_added = connector_result.point_gap_segment_edges_added;
    profile.boxes_added = std::max(0, static_cast<int>(boxes_.size()) - profile.boxes_before);
    profile.raw_boxes_added = std::max(0, static_cast<int>(raw_boxes_.size()) - profile.raw_boxes_before);
    profile.diagnostics["segment_fallback.attempted_pairs"] = static_cast<double>(connector_result.attempted_pairs);
    profile.diagnostics["segment_fallback.connected"] = connector_result.connected ? 1.0 : 0.0;
    profile.diagnostics["segment_fallback.segment_edges_after"] = static_cast<double>(segment_edges_.size());

    const auto adj_t0 = Clock::now();
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    profile.diagnostics["segment_fallback.islands_after"] = static_cast<double>(profile.adjacency_islands);
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return profile;
}

} // namespace rbf
