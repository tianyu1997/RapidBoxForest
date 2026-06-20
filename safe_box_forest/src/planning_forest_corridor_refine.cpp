#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_utils.h"

namespace rbf {

int RBFPlanningForest::refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal,
                                             int max_boxes_to_add) {
    return refine_query_corridor(start,
                                 goal,
                                 max_boxes_to_add,
                                 CorridorRefineMode::SegmentBridge,
                                 std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity());
}

int RBFPlanningForest::refine_query_corridor(const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal,
                                             int max_boxes_to_add,
                                             CorridorRefineMode mode,
                                             double long_path_ratio,
                                             double long_path_min_delta) {
    if (boxes_.empty() || !oracle_ || max_boxes_to_add <= 0) {
        return 0;
    }
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    QueryResult probe = run_query_internal(start, goal, false);
    auto best_refine_path = [&](int seed_attempt) {
        StageContext rrt_context = StageContext::serial();
        RRTConnectConfig refine_rrt =
            with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
        refine_rrt.segment_resolution =
            std::max(refine_rrt.segment_resolution, config_.query.audit_resolution);
        const int refine_attempts = std::max(1, config_.connector.max_pairs_per_gap);
        return best_audited_rrt_bridge_path(start,
                                            goal,
                                            checker,
                                            audit_robot_,
                                            rrt_context,
                                            refine_rrt,
                                            refine_attempts,
                                            config_.connector.per_pair_timeout_ms * refine_attempts,
                                            derived_planner_seed(config_.grower.rng_seed,
                                                                 kSeedCorridorRefineOffset,
                                                                 seed_attempt),
                                            config_.query.audit_resolution,
                                            config_.query.audit_segment_step,
                                            QueryBridgeParallelRrtOptions{});
    };

    std::vector<Eigen::VectorXd> waypoint_path;
    bool add_query_segment_edge = false;
    const bool graph_only_success =
        probe.success && probe.audit_passed && probe.repair_count == 0 &&
        probe.segment_edges_used == 0 && !probe.path.empty();
    if (graph_only_success) {
        if (mode != CorridorRefineMode::BoxOnlyLongPath) {
            return 0;
        }
        const double direct = (goal - start).norm();
        const double delta = probe.path_length - direct;
        const bool ratio_trigger =
            direct > 1e-9 && std::isfinite(long_path_ratio) &&
            probe.path_length / direct >= long_path_ratio;
        const bool delta_trigger = std::isfinite(long_path_min_delta) &&
                                   delta >= long_path_min_delta;
        if (!ratio_trigger && !delta_trigger) {
            return 0;
        }
        waypoint_path = best_refine_path(0);
        if (waypoint_path.empty()) {
            waypoint_path = probe.path;
        }
    } else if (probe.success && probe.audit_passed && !probe.path.empty()) {
        if (mode == CorridorRefineMode::BoxOnlyLongPath) {
            const double direct = (goal - start).norm();
            const double delta = probe.path_length - direct;
            const bool ratio_trigger =
                direct > 1e-9 && std::isfinite(long_path_ratio) &&
                probe.path_length / direct >= long_path_ratio;
            const bool delta_trigger = std::isfinite(long_path_min_delta) &&
                                       delta >= long_path_min_delta;
            if (!ratio_trigger && !delta_trigger) {
                return 0;
            }
            waypoint_path = best_refine_path(1);
            if (waypoint_path.empty()) {
                waypoint_path = probe.path;
            }
        } else {
            waypoint_path = probe.path;
            add_query_segment_edge = true;
        }
    } else {
        waypoint_path = best_refine_path(2);
        add_query_segment_edge = mode != CorridorRefineMode::BoxOnlyLongPath;
    }
    if (waypoint_path.empty()) {
        return 0;
    }
    const int anchor_box_id =
        locate_box_partition_first(waypoint_path.front(), config_.query.nearest_if_outside);
    if (anchor_box_id < 0) {
        return 0;
    }

    if (partition_native_mode()) {
        last_build_.diagnostics["refine_query_corridor.partition_native"] = 1.0;
        const double edge_count_before =
            last_build_.diagnostics["refine_query_corridor.partition_box_corridor_overlay_added"];
        const int added = add_partition_box_corridor_overlay(start,
                                                             goal,
                                                             waypoint_path,
                                                             "refine_query_corridor",
                                                             true,
                                                             true,
                                                             -1,
                                                             &last_build_);
        const double edge_count_after =
            last_build_.diagnostics["refine_query_corridor.partition_box_corridor_overlay_added"];
        if (edge_count_after > edge_count_before) {
            last_build_.diagnostics["refine_query_corridor.partition_native_direct_overlay"] += 1.0;
            last_build_.diagnostics["refine_query_corridor.partition_native_query_bridge_skipped"] += 1.0;
            last_build_.diagnostics["refine_query_corridor.partition_native_box_corridor_edges"] += 1.0;
        }
        return added;
    }

    ChainPaveConfig pave_config = config_.connector.pave;
    pave_config.max_chain = std::min(max_boxes_to_add, std::max(1, max_boxes_to_add));
    pave_config.max_steps_per_waypoint = std::max(1, pave_config.max_steps_per_waypoint);
    pave_config.refine_covered_waypoints = true;
    if (mode == CorridorRefineMode::BoxOnlyLongPath) {
        pave_config.fill_gaps = true;
        pave_config.find_free_box.max_depth = std::max(pave_config.find_free_box.max_depth, 64);
        pave_config.gap_fill_sample_step = std::min(pave_config.gap_fill_sample_step, 0.02);
        pave_config.gap_fill_time_budget_ms = std::max(pave_config.gap_fill_time_budget_ms, 200.0);
        pave_config.gap_fill_max_ffb_calls = std::max(pave_config.gap_fill_max_ffb_calls, 512);
        pave_config.require_connected_chain = true;
    }
    StageContext context = StageContext::serial();
    int next_id = next_box_id();
    const std::size_t boxes_before_refine = boxes_.size();
    const int added = chain_pave_along_path(waypoint_path,
                                            anchor_box_id,
                                            boxes_,
                                            *oracle_,
                                            adjacency_,
                                            next_id,
                                            context,
                                            pave_config);
    if (added > 0) {
        append_adaptive_partition_boxes(boxes_before_refine,
                                        &last_build_,
                                        "refine_query_corridor");
        invalidate_query_cache();
        if (add_query_segment_edge &&
            config_.connector.segment_edges_enabled &&
            config_.connector.rrt_segment_edges &&
            audit_waypoint_path(waypoint_path,
                                checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step)
                .passed) {
            const int source_box_id =
                locate_box_partition_first(start, config_.query.nearest_if_outside);
            const int target_box_id =
                locate_box_partition_first(goal, config_.query.nearest_if_outside);
            if (source_box_id >= 0 && target_box_id >= 0) {
                add_segment_edge_partition_first(source_box_id,
                                                 target_box_id,
                                                 waypoint_path,
                                                 SegmentEdgeType::QueryBridge,
                                                 std::max(1, config_.query.audit_resolution),
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false);
            }
        }
        invalidate_query_cache();
    }
    return added;
}

}  // namespace rbf
