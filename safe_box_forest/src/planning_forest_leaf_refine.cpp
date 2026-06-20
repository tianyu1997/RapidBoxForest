#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <unordered_map>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_merge.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_dynamic_collision_cache_state.h"
#include "planning_forest_qroot_growers.h"

namespace rbf {

LeafSweepRefineResult RBFPlanningForest::build_leaf_sweep_refined(
    const std::vector<Obstacle>& obstacles,
    const LeafSweepRefineConfig& refine_config,
    const std::vector<Eigen::VectorXd>& priority_points,
    const std::vector<Eigen::VectorXd>& offline_anchor_points) {
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();

    LeafSweepRefineResult out;
    LeafSweepConfig leaf_config;
    leaf_config.obstacle_cluster_gap = refine_config.obstacle_cluster_gap;
    leaf_config.n_threads = refine_config.leaf_threads;
    leaf_config.validation_batch_size = refine_config.validation_batch_size;
    leaf_config.timeout_ms = refine_config.leaf_timeout_ms;
    leaf_config.store_group_results = refine_config.store_group_results;
    leaf_config.use_virtual_topology = refine_config.use_virtual_topology;
    leaf_config.parallel_virtual_validation = refine_config.parallel_virtual_validation;
    leaf_config.collision_overlap_prune_min_depth = refine_config.collision_overlap_prune_min_depth;
    leaf_config.collision_overlap_prune_threshold = refine_config.collision_overlap_prune_threshold;
    leaf_config.collision_overlap_prune_ratio_threshold =
        refine_config.collision_overlap_prune_ratio_threshold;

    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      refine_config.leaf_start_depth,
                                      refine_config.leaf_max_depth,
                                      leaf_config);
    const auto priority_prune = prune_leaf_sweep_to_priority(out.leaf_sweep,
                                                             boxes_,
                                                             raw_boxes_,
                                                             priority_points,
                                                             refine_config.priority_prune_radius);
    if (refine_config.priority_prune_radius > 0.0 && !priority_points.empty()) {
        clear_dynamic_collision_cache();
        populate_dynamic_collision_cache(out.leaf_sweep, static_cast<int>(obstacles.size()));
        reserve_existing_boxes();
        adjacency_.clear();
        segment_edges_.clear();
        invalidate_query_cache();
    }
    out.leaf_sweep_ms = out.leaf_sweep.total_ms;
    out.leaf_free_count = static_cast<int>(out.leaf_sweep.free_boxes.size());
    out.leaf_collision_count = static_cast<int>(out.leaf_sweep.collision_boxes.size());

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    MergerResult leaf_merge_result;
    const auto leaf_merge_start = Clock::now();
    if (config_.enable_merger && !boxes_.empty()) {
        MergerConfig leaf_merge_config = config_.merger;
        leaf_merge_config.exact_face_merge = true;
        leaf_merge_config.greedy_hull_merge = false;
        leaf_merge_config.containment_prune = true;
        leaf_merge_config.adjacency_tolerance = adjacency_tolerance;
        leaf_merge_config.max_rounds = std::max(1, leaf_merge_config.max_rounds);
        leaf_merge_result = fast_exact_face_merge_leaf(*oracle_, boxes_, leaf_merge_config);
        raw_boxes_ = boxes_;
    }
    const double leaf_merge_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - leaf_merge_start).count();
    rebuild_adjacency();
    const auto refine_start = Clock::now();
    Deadline refine_deadline = refine_config.refine_timeout_ms > 0.0
        ? Deadline::after_ms(refine_config.refine_timeout_ms)
        : Deadline{};
    StageContext refine_context = StageContext::from_runtime(config_.runtime, refine_deadline);
    FindFreeBoxOptions refine_options = config_.grower.find_free_box;
    refine_options.max_depth = refine_config.deep_ffb_depth;
    refine_options.reject_seed_collision = false;
    int next_id = next_box_id();
    auto find_in_domain = [this](const Eigen::VectorXd& seed,
                                 const std::vector<Interval>& domain,
                                 StageContext& context,
                                 const FindFreeBoxOptions& options) {
        return this->find_free_box_in_domain(seed, domain, context, options);
    };
    const auto offline_anchors = run_offline_anchor_grower(*oracle_,
                                                           refine_config,
                                                           out.leaf_sweep.collision_boxes,
                                                           offline_anchor_points,
                                                           find_in_domain,
                                                           config_.grower.commit_policy,
                                                           boxes_,
                                                           raw_boxes_,
                                                           adjacency_,
                                                           next_id,
                                                           refine_context,
                                                           refine_options,
                                                           adjacency_tolerance);
    const auto qroot = run_query_root_box_grower(*oracle_,
                                                 refine_config,
                                                 out.leaf_sweep.collision_boxes,
                                                 priority_points,
                                                 find_in_domain,
                                                 config_.grower.commit_policy,
                                                 boxes_,
                                                 raw_boxes_,
                                                 adjacency_,
                                                 next_id,
                                                 refine_context,
                                                 refine_options,
                                                 adjacency_tolerance);
    out.deep_boxes_added = qroot.boxes_added;
    out.deep_domain_attempts = qroot.pair_attempts;
    out.deep_ffb_success = qroot.ffb_success;
    out.deep_ffb_fail = qroot.ffb_fail;
    out.deep_commit_rejects = qroot.commit_rejects;
    out.deep_domain_rejects = qroot.domain_rejects;
    out.deep_contained_rejects = qroot.contained_rejects;
	out.deep_adjacency_rejects = qroot.adjacency_rejects;
	out.deep_anchor_roots_added = qroot.endpoint_anchors_added;
	out.deep_refine_ms = std::chrono::duration<double, std::milli>(Clock::now() - refine_start).count();

    const auto connector_start = Clock::now();
    bool connector_ran = false;
    std::unordered_map<std::string, double> connector_diagnostics;
    if (config_.enable_connector && !boxes_.empty()) {
        StageContext connector_context = StageContext::from_runtime(config_.runtime);
        CollisionChecker checker(robot_, scene_);
        IslandConnectorConfig connector_config = config_.connector;
        if (connector_config.n_threads <= 1 && config_.runtime.n_threads > 1) {
            connector_config.n_threads = config_.runtime.n_threads;
        }
        int connector_next_id = next_id;
        IslandConnectorConfig box_only_config = connector_config;
        box_only_config.segment_edges_fallback_only = true;
        {
            IslandConnector connector(*oracle_, robot_, checker, box_only_config);
            const auto connector_result = connector.connect_all(boxes_,
                                                                adjacency_,
                                                                segment_edges_,
                                                                connector_next_id,
                                                                connector_context);
            out.profile.bridge_boxes_added += connector_result.bridge_boxes_added;
            out.profile.connector_attempted_pairs += connector_result.attempted_pairs;
            out.profile.connector_connected = connector_result.connected;
        }
        if (find_islands(adjacency_).size() > 1 &&
            connector_config.segment_edges_enabled &&
            (connector_config.rrt_segment_edges || connector_config.point_gap_segment_edges)) {
            IslandConnectorConfig fallback_config = connector_config;
            fallback_config.segment_edges_fallback_only = false;
            fallback_config.max_total_bridge_boxes = 0;
            fallback_config.max_pairs_per_gap = std::max(fallback_config.max_pairs_per_gap, 4);
            IslandConnector fallback_connector(*oracle_, robot_, checker, fallback_config);
            const auto fallback_result = fallback_connector.connect_all(boxes_,
                                                                        adjacency_,
                                                                        segment_edges_,
                                                                        connector_next_id,
                                                                        connector_context);
            out.profile.bridge_boxes_added += fallback_result.bridge_boxes_added;
            out.profile.segment_edges_added += fallback_result.segment_edges_added;
            out.profile.rrt_segment_edges_added += fallback_result.rrt_segment_edges_added;
            out.profile.point_gap_segment_edges_added += fallback_result.point_gap_segment_edges_added;
            out.profile.connector_attempted_pairs += fallback_result.attempted_pairs;
            out.profile.connector_connected = fallback_result.connected;
        }
        next_id = connector_next_id;
        connector_diagnostics = connector_context.diagnostics().snapshot();
        connector_ran = true;
    }
    out.connector_ms = std::chrono::duration<double, std::milli>(Clock::now() - connector_start).count();
    out.profile.connector_ms = out.connector_ms;

    const auto adjacency_start = Clock::now();
    out.profile.segment_edges = static_cast<int>(segment_edges_.size());
    if (!connector_ran) {
        rebuild_adjacency();
    }
    out.profile.adjacency_ms = std::chrono::duration<double, std::milli>(Clock::now() - adjacency_start).count();
    out.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    out.profile.final_boxes = static_cast<int>(boxes_.size());
	out.profile.grow_ms = out.leaf_sweep_ms + out.deep_refine_ms;
    out.profile.grow_adjacency_islands = static_cast<int>(find_islands(adjacency_).size());
    out.profile.grow_largest_island = 0;
    for (const auto& island : find_islands(adjacency_)) {
        out.profile.grow_largest_island =
            std::max(out.profile.grow_largest_island, static_cast<int>(island.size()));
    }
    out.profile.adjacency_islands = out.profile.grow_adjacency_islands;
    out.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    out.profile.total_ms = out.total_ms;
    out.profile.diagnostics = out.leaf_sweep.diagnostics;
    out.profile.diagnostics["leaf_refine.leaf_sweep_ms"] = out.leaf_sweep_ms;
    record_depth_semantics_diagnostics(out.profile.diagnostics,
                                       "leaf_refine.",
                                       refine_config.leaf_start_depth,
                                       refine_config.leaf_max_depth,
                                       refine_config.deep_ffb_depth,
                                       config_.grower.find_free_box,
                                       refine_config.deep_ffb_depth);
    out.profile.diagnostics["leaf_refine.leaf_free_count"] = static_cast<double>(out.leaf_free_count);
    out.profile.diagnostics["leaf_refine.leaf_collision_count"] = static_cast<double>(out.leaf_collision_count);
    out.profile.diagnostics["leaf_refine.priority_prune_radius"] = refine_config.priority_prune_radius;
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_min_depth"] =
        static_cast<double>(refine_config.collision_overlap_prune_min_depth);
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_threshold"] =
        refine_config.collision_overlap_prune_threshold;
    out.profile.diagnostics["leaf_refine.collision_overlap_prune_ratio_threshold"] =
        refine_config.collision_overlap_prune_ratio_threshold;
    out.profile.diagnostics["leaf_refine.priority_prune_free_before"] =
        static_cast<double>(priority_prune.free_before);
    out.profile.diagnostics["leaf_refine.priority_prune_free_after"] =
        static_cast<double>(priority_prune.free_after);
    out.profile.diagnostics["leaf_refine.priority_prune_collision_before"] =
        static_cast<double>(priority_prune.collision_before);
    out.profile.diagnostics["leaf_refine.priority_prune_collision_after"] =
        static_cast<double>(priority_prune.collision_after);
    out.profile.diagnostics["leaf_refine.leaf_merge_ms"] = leaf_merge_ms;
    out.profile.diagnostics["leaf_refine.leaf_merge_boxes_before"] =
        static_cast<double>(leaf_merge_result.boxes_before);
    out.profile.diagnostics["leaf_refine.leaf_merge_boxes_after"] =
        static_cast<double>(leaf_merge_result.boxes_after);
    out.profile.diagnostics["leaf_refine.leaf_merge_exact"] =
        static_cast<double>(leaf_merge_result.exact_merges);
    out.profile.diagnostics["leaf_refine.leaf_merge_pruned"] =
        static_cast<double>(leaf_merge_result.pruned_boxes);
    out.profile.diagnostics["leaf_refine.collision_cache_boxes"] =
        static_cast<double>(dynamic_collision_cache_->boxes.size());
    out.profile.diagnostics["leaf_refine.deep_refine_ms"] = out.deep_refine_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_ms"] = offline_anchors.total_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_candidates"] =
        static_cast<double>(offline_anchors.candidates_total);
    out.profile.diagnostics["leaf_refine.offline_anchor_candidates_covered"] =
        static_cast<double>(offline_anchors.candidates_covered);
    out.profile.diagnostics["leaf_refine.offline_anchor_roots_added"] =
        static_cast<double>(offline_anchors.boxes_added);
    out.profile.diagnostics["leaf_refine.offline_anchor_ffb_success"] =
        static_cast<double>(offline_anchors.ffb_success);
    out.profile.diagnostics["leaf_refine.offline_anchor_ffb_fail"] =
        static_cast<double>(offline_anchors.ffb_fail);
    out.profile.diagnostics["leaf_refine.offline_anchor_commit_rejects"] =
        static_cast<double>(offline_anchors.commit_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_domain_rejects"] =
        static_cast<double>(offline_anchors.domain_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_contained_rejects"] =
        static_cast<double>(offline_anchors.contained_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_rejects"] =
        static_cast<double>(offline_anchors.adjacency_rejects);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_candidates_tested"] =
        static_cast<double>(offline_anchors.adjacency_candidates_tested);
    out.profile.diagnostics["leaf_refine.offline_anchor_adjacency_edges_added"] =
        static_cast<double>(offline_anchors.adjacency_edges_added);
    out.profile.diagnostics["leaf_refine.offline_anchor_islands_before"] =
        static_cast<double>(offline_anchors.islands_before);
    out.profile.diagnostics["leaf_refine.offline_anchor_islands_after"] =
        static_cast<double>(offline_anchors.islands_after);
    out.profile.diagnostics["leaf_refine.offline_anchor_box_volume_mean"] =
        offline_anchors.boxes_added > 0
            ? offline_anchors.box_volume_sum / static_cast<double>(offline_anchors.boxes_added)
            : 0.0;
    out.profile.diagnostics["leaf_refine.offline_anchor_box_volume_max"] =
        offline_anchors.box_volume_max;
    out.profile.diagnostics["leaf_refine.offline_anchor_index_rebuild_ms"] =
        offline_anchors.index_rebuild_ms;
    out.profile.diagnostics["leaf_refine.offline_anchor_index_query_ms"] =
        offline_anchors.index_query_ms;
    out.profile.diagnostics["leaf_refine.deep_boxes_added"] = static_cast<double>(out.deep_boxes_added);
    out.profile.diagnostics["leaf_refine.deep_domain_attempts"] = static_cast<double>(out.deep_domain_attempts);
    out.profile.diagnostics["leaf_refine.deep_ffb_success"] = static_cast<double>(out.deep_ffb_success);
    out.profile.diagnostics["leaf_refine.deep_ffb_fail"] = static_cast<double>(out.deep_ffb_fail);
    out.profile.diagnostics["leaf_refine.deep_commit_rejects"] = static_cast<double>(out.deep_commit_rejects);
    out.profile.diagnostics["leaf_refine.deep_domain_rejects"] = static_cast<double>(out.deep_domain_rejects);
    out.profile.diagnostics["leaf_refine.deep_contained_rejects"] = static_cast<double>(out.deep_contained_rejects);
    out.profile.diagnostics["leaf_refine.deep_adjacency_rejects"] = static_cast<double>(out.deep_adjacency_rejects);
    out.profile.diagnostics["leaf_refine.deep_anchor_roots_added"] = static_cast<double>(out.deep_anchor_roots_added);
    out.profile.diagnostics["leaf_refine.qroot_ms"] = qroot.total_ms;
    out.profile.diagnostics["leaf_refine.qroot_pairs_total"] = static_cast<double>(qroot.pairs_total);
    out.profile.diagnostics["leaf_refine.qroot_pairs_connected_before"] =
        static_cast<double>(qroot.pairs_connected_before);
    out.profile.diagnostics["leaf_refine.qroot_pairs_connected_after"] =
        static_cast<double>(qroot.pairs_connected_after);
    out.profile.diagnostics["leaf_refine.qroot_uncovered_endpoints"] =
        static_cast<double>(qroot.uncovered_endpoints);
    out.profile.diagnostics["leaf_refine.qroot_endpoint_anchors_added"] =
        static_cast<double>(qroot.endpoint_anchors_added);
    out.profile.diagnostics["leaf_refine.qroot_endpoint_root_fallbacks"] =
        static_cast<double>(qroot.endpoint_root_fallbacks);
    out.profile.diagnostics["leaf_refine.qroot_boxes_added"] =
        static_cast<double>(qroot.boxes_added);
    out.profile.diagnostics["leaf_refine.qroot_ffb_success"] = static_cast<double>(qroot.ffb_success);
    out.profile.diagnostics["leaf_refine.qroot_ffb_fail"] = static_cast<double>(qroot.ffb_fail);
    out.profile.diagnostics["leaf_refine.qroot_adjacency_candidates_tested"] =
        static_cast<double>(qroot.adjacency_candidates_tested);
    out.profile.diagnostics["leaf_refine.qroot_adjacency_edges_added"] =
        static_cast<double>(qroot.adjacency_edges_added);
    out.profile.diagnostics["leaf_refine.qroot_index_rebuild_ms"] = qroot.index_rebuild_ms;
    out.profile.diagnostics["leaf_refine.qroot_index_query_ms"] = qroot.index_query_ms;
    out.profile.diagnostics["leaf_refine.qroot_islands_before"] = static_cast<double>(qroot.islands_before);
    out.profile.diagnostics["leaf_refine.qroot_islands_after"] = static_cast<double>(qroot.islands_after);
	out.profile.diagnostics["leaf_refine.connector_ms"] = out.connector_ms;
    for (const auto& [key, value] : connector_diagnostics) {
        out.profile.diagnostics[std::string("leaf_refine.") + key] = value;
    }
    out.profile.diagnostics["leaf_refine.total_ms"] = out.total_ms;
    record_portal_membership_policy(out.profile.diagnostics, config_.portal_membership_policy);
    out.diagnostics = out.profile.diagnostics;
    last_build_ = out.profile;
    invalidate_query_cache();
    if (config_.database.checkpoint_after_build && database_) {
        database_->checkpoint();
    }
    return out;
}

}  // namespace rbf
