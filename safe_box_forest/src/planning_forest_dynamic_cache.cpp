#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

struct SegmentEdgePruneStats {
    int before = 0;
    int removed_dead_endpoint = 0;
    int audited = 0;
    int removed_audit = 0;
    double audit_ms = 0.0;
};

SegmentEdgePruneStats prune_insert_segment_edges(std::vector<SegmentEdge>& segment_edges,
                                                 const std::vector<BoxNode>& boxes,
                                                 AdjacencyGraph& adjacency,
                                                 const CollisionChecker& updated_checker,
                                                 const QueryConfig& query_config,
                                                 bool partition_native) {
    using Clock = std::chrono::steady_clock;
    SegmentEdgePruneStats stats;
    std::unordered_set<int> live_box_ids;
    live_box_ids.reserve(boxes.size());
    for (const auto& box : boxes) {
        live_box_ids.insert(box.id);
    }
    stats.before = static_cast<int>(segment_edges.size());
    segment_edges.erase(std::remove_if(segment_edges.begin(), segment_edges.end(), [&](const SegmentEdge& edge) {
        if (live_box_ids.find(edge.source_box_id) == live_box_ids.end() ||
            live_box_ids.find(edge.target_box_id) == live_box_ids.end()) {
            stats.removed_dead_endpoint += 1;
            return true;
        }
        const auto edge_t0 = Clock::now();
        stats.audited += 1;
        const bool survives = segment_edge_survives_scene(
            edge, updated_checker, query_config.audit_resolution, query_config.audit_segment_step);
        stats.audit_ms += std::chrono::duration<double, std::milli>(Clock::now() - edge_t0).count();
        if (!survives) {
            stats.removed_audit += 1;
            if (!partition_native) {
                const BoxNode* source_box = find_box_by_id(boxes, edge.source_box_id);
                const BoxNode* target_box = find_box_by_id(boxes, edge.target_box_id);
                if (source_box == nullptr || target_box == nullptr ||
                    !boxes_connected(*source_box, *target_box, query_config.adjacency_tolerance)) {
                    remove_local_edge(adjacency, edge.source_box_id, edge.target_box_id);
                }
            }
        }
        return !survives;
    }), segment_edges.end());
    return stats;
}

} // namespace

RebuildProfile RBFPlanningForest::add_obstacle_and_rebuild(const Obstacle& obstacle) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());

    Scene added_scene(std::vector<Obstacle>{obstacle});
    CollisionChecker added_checker(robot_, added_scene);
    Scene updated_scene = scene_;
    updated_scene.add_obstacle(obstacle);
    CollisionChecker updated_checker(robot_, updated_scene);
    std::vector<BoxNode> removed_boxes;
    std::unordered_set<int> removed_box_ids;
    const auto check_t0 = Clock::now();
    profile.used_spatial_dirty_region = config_.dynamic_update.enable_spatial_dirty_region;
    std::vector<int> dirty_indices;
    if (config_.dynamic_update.enable_spatial_dirty_region) {
        const auto dirty_t0 = Clock::now();
        dirty_indices = spatial_dirty_all_box_indices(robot_, boxes_, obstacle, config_.dynamic_update, profile.dirty_boxes);
        profile.dirty_region_ms = std::chrono::duration<double, std::milli>(Clock::now() - dirty_t0).count();
        profile.dirty_boxes_used = static_cast<int>(dirty_indices.size());
    } else {
        dirty_indices.reserve(boxes_.size());
        for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
            dirty_indices.push_back(index);
        }
        profile.dirty_boxes = static_cast<int>(dirty_indices.size());
        profile.dirty_boxes_used = profile.dirty_boxes;
    }
    for (int index : dirty_indices) {
        if (index < 0 || index >= static_cast<int>(boxes_.size())) {
            continue;
        }
        const BoxNode& box = boxes_[static_cast<std::size_t>(index)];
        const auto box_check_t0 = Clock::now();
        const bool collides = added_checker.check_box(box.joint_intervals);
        profile.diagnostics["insert.free_box_check_box_calls"] += 1.0;
        profile.diagnostics["insert.free_box_check_box_ms"] +=
            std::chrono::duration<double, std::milli>(Clock::now() - box_check_t0).count();
        if (collides) {
            removed_box_ids.insert(box.id);
            removed_boxes.push_back(box);
        }
    }
    for (std::size_t i = 0; i < boxes_.size();) {
        if (removed_box_ids.find(boxes_[i].id) != removed_box_ids.end()) {
            if (oracle_) {
                oracle_->release_box(boxes_[i].id);
            }
            boxes_.erase(boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.boxes_removed += 1;
        } else {
            ++i;
        }
    }
    for (std::size_t i = 0; i < raw_boxes_.size();) {
        bool remove_raw = removed_box_ids.find(raw_boxes_[i].id) != removed_box_ids.end();
        if (!remove_raw) {
            const auto raw_check_t0 = Clock::now();
            remove_raw = added_checker.check_box(raw_boxes_[i].joint_intervals);
            profile.diagnostics["insert.raw_box_check_box_calls"] += 1.0;
            profile.diagnostics["insert.raw_box_check_box_ms"] +=
                std::chrono::duration<double, std::milli>(Clock::now() - raw_check_t0).count();
        }
        if (remove_raw) {
            raw_boxes_.erase(raw_boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.raw_boxes_removed += 1;
        } else {
            ++i;
        }
    }
    const auto segment_stats = prune_insert_segment_edges(segment_edges_,
                                                          boxes_,
                                                          adjacency_,
                                                          updated_checker,
                                                          config_.query,
                                                          partition_native_mode());
    profile.diagnostics["insert.segment_edges_before"] = static_cast<double>(segment_stats.before);
    profile.diagnostics["insert.segment_edges_removed_dead_endpoint"] =
        static_cast<double>(segment_stats.removed_dead_endpoint);
    profile.diagnostics["insert.segment_edges_audited"] = static_cast<double>(segment_stats.audited);
    profile.diagnostics["insert.segment_edges_removed_audit"] =
        static_cast<double>(segment_stats.removed_audit);
    profile.diagnostics["insert.segment_edge_audit_ms"] = segment_stats.audit_ms;
    profile.collision_check_ms = std::chrono::duration<double, std::milli>(Clock::now() - check_t0).count();

    scene_ = std::move(updated_scene);
    const auto reset_t0 = Clock::now();
    reset_oracle(scene_);
    profile.diagnostics["insert.reset_oracle_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reset_t0).count();
    const auto reserve_t0 = Clock::now();
    reserve_existing_boxes();
    profile.diagnostics["insert.reserve_existing_boxes_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reserve_t0).count();

    const auto regrow_t0 = Clock::now();
    profile.dirty_seed_count = static_cast<int>(removed_boxes.size());
    int next_id = next_box_id();
    const std::size_t first_new_box_index = boxes_.size();
    for (const auto& removed_box : removed_boxes) {
        profile.diagnostics["insert.refill_domains"] += 1.0;
        int sweep_max_depth = config_.dynamic_update.insertion_leaf_sweep_max_depth;
        if (oracle_ && config_.dynamic_update.insertion_leaf_sweep_relative_depth >= 0 && removed_box.tree_id >= 0) {
            sweep_max_depth = std::min(
                sweep_max_depth,
                oracle_->depth(removed_box.tree_id) + config_.dynamic_update.insertion_leaf_sweep_relative_depth);
        }
        refill_removed_box_with_leaf_sweep(removed_box,
                                           profile.obstacles_before,
                                           sweep_max_depth,
                                           next_id,
                                           profile);
    }
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - regrow_t0).count();

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.obstacles_after = scene_.n_obstacles();
    const auto adj_t0 = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_remove_append(profile,
                                                      removed_box_ids,
                                                      first_new_box_index,
                                                      "insert.partition_delta");
    } else {
        remove_adjacency_nodes(adjacency_, removed_box_ids);
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
        apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
        invalidate_query_cache();
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

RebuildProfile RBFPlanningForest::add_obstacles_and_rebuild(const std::vector<Obstacle>& obstacles) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.collision_cache_boxes_before = static_cast<int>(dynamic_collision_box_cache_.size());
    if (obstacles.empty()) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    Scene added_scene(obstacles);
    CollisionChecker added_checker(robot_, added_scene);
    Scene updated_scene = scene_;
    for (const auto& obstacle : obstacles) {
        updated_scene.add_obstacle(obstacle);
    }
    CollisionChecker updated_checker(robot_, updated_scene);

    std::vector<BoxNode> removed_boxes;
    std::unordered_set<int> removed_box_ids;
    const auto check_t0 = Clock::now();
    profile.used_spatial_dirty_region = config_.dynamic_update.enable_spatial_dirty_region;
    std::vector<int> dirty_indices;
    if (config_.dynamic_update.enable_spatial_dirty_region) {
        const auto dirty_t0 = Clock::now();
        dirty_indices.reserve(boxes_.size());
        std::unordered_set<int> dirty_set;
        for (const auto& obstacle : obstacles) {
            int obstacle_dirty_count = 0;
            auto current = spatial_dirty_all_box_indices(robot_, boxes_, obstacle, config_.dynamic_update, obstacle_dirty_count);
            profile.dirty_boxes += obstacle_dirty_count;
            for (int index : current) {
                if (dirty_set.insert(index).second) {
                    dirty_indices.push_back(index);
                }
            }
        }
        profile.dirty_region_ms = std::chrono::duration<double, std::milli>(Clock::now() - dirty_t0).count();
        profile.dirty_boxes_used = static_cast<int>(dirty_indices.size());
    } else {
        dirty_indices.reserve(boxes_.size());
        for (int index = 0; index < static_cast<int>(boxes_.size()); ++index) {
            dirty_indices.push_back(index);
        }
        profile.dirty_boxes = static_cast<int>(dirty_indices.size());
        profile.dirty_boxes_used = profile.dirty_boxes;
    }

    for (int index : dirty_indices) {
        if (index < 0 || index >= static_cast<int>(boxes_.size())) {
            continue;
        }
        const BoxNode& box = boxes_[static_cast<std::size_t>(index)];
        const auto box_check_t0 = Clock::now();
        const bool collides = added_checker.check_box(box.joint_intervals);
        profile.diagnostics["insert.free_box_check_box_calls"] += 1.0;
        profile.diagnostics["insert.free_box_check_box_ms"] +=
            std::chrono::duration<double, std::milli>(Clock::now() - box_check_t0).count();
        if (collides) {
            removed_box_ids.insert(box.id);
            removed_boxes.push_back(box);
        }
    }
    for (std::size_t i = 0; i < boxes_.size();) {
        if (removed_box_ids.find(boxes_[i].id) != removed_box_ids.end()) {
            if (oracle_) {
                oracle_->release_box(boxes_[i].id);
            }
            boxes_.erase(boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.boxes_removed += 1;
        } else {
            ++i;
        }
    }
    for (std::size_t i = 0; i < raw_boxes_.size();) {
        bool remove_raw = removed_box_ids.find(raw_boxes_[i].id) != removed_box_ids.end();
        if (!remove_raw) {
            const auto raw_check_t0 = Clock::now();
            remove_raw = added_checker.check_box(raw_boxes_[i].joint_intervals);
            profile.diagnostics["insert.raw_box_check_box_calls"] += 1.0;
            profile.diagnostics["insert.raw_box_check_box_ms"] +=
                std::chrono::duration<double, std::milli>(Clock::now() - raw_check_t0).count();
        }
        if (remove_raw) {
            raw_boxes_.erase(raw_boxes_.begin() + static_cast<std::ptrdiff_t>(i));
            profile.raw_boxes_removed += 1;
        } else {
            ++i;
        }
    }

    const auto segment_stats = prune_insert_segment_edges(segment_edges_,
                                                          boxes_,
                                                          adjacency_,
                                                          updated_checker,
                                                          config_.query,
                                                          partition_native_mode());
    profile.diagnostics["insert.segment_edges_before"] = static_cast<double>(segment_stats.before);
    profile.diagnostics["insert.segment_edges_removed_dead_endpoint"] =
        static_cast<double>(segment_stats.removed_dead_endpoint);
    profile.diagnostics["insert.segment_edges_audited"] = static_cast<double>(segment_stats.audited);
    profile.diagnostics["insert.segment_edges_removed_audit"] =
        static_cast<double>(segment_stats.removed_audit);
    profile.diagnostics["insert.segment_edge_audit_ms"] = segment_stats.audit_ms;
    profile.collision_check_ms = std::chrono::duration<double, std::milli>(Clock::now() - check_t0).count();

    scene_ = std::move(updated_scene);
    const auto reset_t0 = Clock::now();
    reset_oracle(scene_);
    profile.diagnostics["insert.reset_oracle_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reset_t0).count();
    const auto reserve_t0 = Clock::now();
    reserve_existing_boxes();
    profile.diagnostics["insert.reserve_existing_boxes_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - reserve_t0).count();

    const auto regrow_t0 = Clock::now();
    profile.dirty_seed_count = static_cast<int>(removed_boxes.size());
    int next_id = next_box_id();
    const std::size_t first_new_box_index = boxes_.size();
    for (const auto& removed_box : removed_boxes) {
        profile.diagnostics["insert.refill_domains"] += 1.0;
        int sweep_max_depth = config_.dynamic_update.insertion_leaf_sweep_max_depth;
        if (oracle_ && config_.dynamic_update.insertion_leaf_sweep_relative_depth >= 0 && removed_box.tree_id >= 0) {
            sweep_max_depth = std::min(
                sweep_max_depth,
                oracle_->depth(removed_box.tree_id) + config_.dynamic_update.insertion_leaf_sweep_relative_depth);
        }
        refill_removed_box_with_leaf_sweep(removed_box,
                                           profile.obstacles_before,
                                           sweep_max_depth,
                                           next_id,
                                           profile);
    }
    profile.regrow_ms = std::chrono::duration<double, std::milli>(Clock::now() - regrow_t0).count();

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    profile.obstacles_after = scene_.n_obstacles();
    const auto adj_t0 = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_remove_append(profile,
                                                      removed_box_ids,
                                                      first_new_box_index,
                                                      "insert_batch.partition_delta");
    } else {
        remove_adjacency_nodes(adjacency_, removed_box_ids);
        connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
        apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
        invalidate_query_cache();
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0).count();
    profile.collision_cache_boxes_after = static_cast<int>(dynamic_collision_box_cache_.size());
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

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

RebuildProfile RBFPlanningForest::remove_obstacle_and_regrow(int obstacle_index) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    profile.removed_obstacle_index = obstacle_index;

    Obstacle removed_obstacle;
    if (!scene_.remove_obstacle_at(obstacle_index, &removed_obstacle)) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }
    profile.obstacles_after = scene_.n_obstacles();
    reset_oracle(scene_);
    reserve_existing_boxes();
    const std::unordered_set<int> removed_indices{obstacle_index};
    const std::size_t first_new_box_index = boxes_.size();
    promote_unblocked_collision_cache(removed_indices, profile);

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    const auto adj_t0_delete = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_append(profile,
                                               first_new_box_index,
                                               "remove.partition_append");
    } else {
        if (profile.boxes_added > 0) {
            connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
            apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
            invalidate_query_cache();
        }
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

RebuildProfile RBFPlanningForest::remove_obstacle_suffix_and_regrow(int target_obstacle_count) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    RebuildProfile profile;
    profile.boxes_before = static_cast<int>(boxes_.size());
    profile.raw_boxes_before = static_cast<int>(raw_boxes_.size());
    profile.obstacles_before = scene_.n_obstacles();
    const int target_count = std::clamp(target_obstacle_count, 0, profile.obstacles_before);
    profile.removed_obstacle_index = target_count;

    if (target_count >= profile.obstacles_before) {
        profile.boxes_after = profile.boxes_before;
        profile.raw_boxes_after = profile.raw_boxes_before;
        profile.obstacles_after = profile.obstacles_before;
        profile.adjacency_islands = island_count_partition_first();
        profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return profile;
    }

    std::vector<Obstacle> removed_obstacles;
    removed_obstacles.reserve(static_cast<std::size_t>(profile.obstacles_before - target_count));
    while (scene_.n_obstacles() > target_count) {
        Obstacle removed_obstacle;
        scene_.remove_obstacle_at(scene_.n_obstacles() - 1, &removed_obstacle);
        removed_obstacles.push_back(removed_obstacle);
    }
    profile.obstacles_after = scene_.n_obstacles();
    reset_oracle(scene_);
    reserve_existing_boxes();
    std::unordered_set<int> removed_indices;
    for (int index = target_count; index < profile.obstacles_before; ++index) {
        removed_indices.insert(index);
    }
    const std::size_t first_new_box_index = boxes_.size();
    promote_unblocked_collision_cache(removed_indices, profile);

    profile.boxes_after = static_cast<int>(boxes_.size());
    profile.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    const auto adj_t0_delete = Clock::now();
    if (partition_native_mode()) {
        refresh_dynamic_partition_after_append(profile,
                                               first_new_box_index,
                                               "remove_suffix.partition_append");
    } else {
        if (profile.boxes_added > 0) {
            connect_incremental_boxes(adjacency_, boxes_, first_new_box_index, config_.query.adjacency_tolerance);
            apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
            invalidate_query_cache();
        }
        profile.adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    }
    profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adj_t0_delete).count();
    profile.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    invalidate_query_cache();
    return profile;
}

} // namespace rbf
