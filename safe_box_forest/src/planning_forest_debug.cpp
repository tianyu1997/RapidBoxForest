#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_utils.h"

namespace rbf {

namespace {

void fill_debug_chain_pave_endpoint_boxes(DebugChainPaveResult& out,
                                          const std::vector<BoxNode>& boxes,
                                          int start_box_id,
                                          int goal_box_id) {
    out.start_box_id = start_box_id;
    out.goal_box_id = goal_box_id;
    for (const auto& box : boxes) {
        if (box.id == start_box_id) {
            out.start_box = box.joint_intervals;
        }
        if (box.id == goal_box_id) {
            out.goal_box = box.joint_intervals;
        }
    }
}

void export_debug_chain_pave_boxes(DebugChainPaveResult& out,
                                   const std::vector<BoxNode>& boxes,
                                   std::size_t boxes_before) {
    for (std::size_t i = boxes_before; i < boxes.size(); ++i) {
        out.committed_boxes.push_back(boxes[i].joint_intervals);
    }
    // Export every forest box so callers can measure true bridge coverage:
    // chain_pave may cover a path point by reusing a pre-existing build box.
    out.all_boxes.clear();
    out.all_boxes.reserve(boxes.size());
    for (const auto& box : boxes) {
        out.all_boxes.push_back(box.joint_intervals);
    }
}

void record_debug_chain_pave_context_diagnostics(DebugChainPaveResult& out,
                                                 const StageContext& context) {
    const StageDiagnostics& diagnostics = context.diagnostics();
    out.fast_gap_fill_ffb_calls = static_cast<int>(
        diagnostics.value("connector.chain_pave_fast_ffb_calls", 0.0));
    out.fast_gap_fill_ms =
        diagnostics.value("connector.chain_pave_fast_ms", 0.0);
    out.boundary_ffb_calls = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_ffb_calls", 0.0));
    out.boundary_commits = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_commits", 0.0));
    out.boundary_reject_not_free = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_reject_not_free", 0.0));
    out.boundary_reject_non_adjacent = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_reject_non_adjacent", 0.0));
    out.boundary_fail_seed_collision = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_seed_collision", 0.0));
    out.boundary_fail_depth_cap = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_depth_cap", 0.0));
    out.boundary_fail_unknown_depth_cap = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0));
    out.boundary_fail_reserved_depth_cap = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0));
    out.boundary_fail_occupied = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_occupied", 0.0));
    out.boundary_fail_deadline = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_deadline", 0.0));
    out.boundary_fail_out_of_domain = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_out_of_domain", 0.0));
    out.boundary_fail_split = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_fail_split", 0.0));
    out.boundary_failed_seed_memoized = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_failed_seed_memoized", 0.0));
    out.boundary_skip_failed_seed = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_skip_failed_seed", 0.0));
    out.boundary_stall = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_stall", 0.0));
    out.boundary_target_hits = static_cast<int>(
        diagnostics.value("connector.chain_pave_boundary_target_hits", 0.0));
}

} // namespace

DebugChainPaveResult RBFPlanningForest::debug_chain_pave(const Eigen::Ref<const Eigen::VectorXd>& start,
                                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                                         const ChainPaveConfig& pave) {
    DebugChainPaveResult out;
    if (boxes_.empty() || !oracle_) {
        return out;
    }
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return out;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return out;
    }
    fill_debug_chain_pave_endpoint_boxes(out, boxes_, start_box_id, goal_box_id);
    StageContext context = StageContext::from_runtime(config_.runtime);
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    RRTConnectConfig bridge_rrt = with_query_root_hull_domain(config_.connector.rrt, *oracle_, start, goal);
    bridge_rrt.segment_resolution = std::max(bridge_rrt.segment_resolution, config_.query.audit_resolution);
    auto waypoint_path = rrt_connect(
        start,
        goal,
        checker,
        audit_robot_,
        context,
        bridge_rrt,
        derived_planner_seed(config_.grower.rng_seed, kSeedDebugBridgeOffset));
    if (waypoint_path.empty()) {
        return out;
    }
    out.bridge_found = true;
    out.waypoints = waypoint_path;
    out.audit_passed = audit_waypoint_path(waypoint_path,
                                           checker,
                                           config_.query.audit_resolution,
                                           config_.query.audit_segment_step)
                           .passed;
    const std::size_t boxes_before = boxes_.size();
    if (partition_native_mode()) {
        out.added = add_partition_box_corridor_overlay(start,
                                                       goal,
                                                       waypoint_path,
                                                       "debug_chain_pave",
                                                       false,
                                                       false,
                                                       -1,
                                                       &last_build_);
        out.boundary_ffb_calls = static_cast<int>(
            last_build_.diagnostics["debug_chain_pave.partition_box_corridor_overlay_attempts"]);
        out.boundary_commits = out.added;
        export_debug_chain_pave_boxes(out, boxes_, boxes_before);
        if (out.added > 0) {
            invalidate_query_cache();
        }
        invalidate_query_cache();
        return out;
    }
    int next_id = next_box_id();
    ChainPaveConfig debug_pave = pave;
    debug_pave.debug_boundary_failures = &out.boundary_failures;
    out.added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        debug_pave);
    if (boxes_.size() > boxes_before) {
        append_adaptive_partition_boxes(boxes_before, &last_build_, "debug_chain_pave");
    }
    record_debug_chain_pave_context_diagnostics(out, context);
    export_debug_chain_pave_boxes(out, boxes_, boxes_before);
    if (out.added > 0) {
        invalidate_query_cache();
    }
    invalidate_query_cache();
    return out;
}

DebugChainPaveResult RBFPlanningForest::debug_chain_pave_waypoints(
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const ChainPaveConfig& pave) {
    DebugChainPaveResult out;
    if (waypoint_path.empty() || boxes_.empty() || !oracle_) {
        return out;
    }
    const Eigen::VectorXd& start = waypoint_path.front();
    const Eigen::VectorXd& goal = waypoint_path.back();
    const int start_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
    if (start_box_id < 0) {
        return out;
    }
    const int goal_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
    if (goal_box_id < 0 || goal_box_id == start_box_id) {
        return out;
    }
    fill_debug_chain_pave_endpoint_boxes(out, boxes_, start_box_id, goal_box_id);
    out.bridge_found = true;
    out.waypoints = waypoint_path;
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    out.audit_passed = audit_waypoint_path(waypoint_path,
                                           checker,
                                           config_.query.audit_resolution,
                                           config_.query.audit_segment_step)
                           .passed;
    StageContext context = StageContext::from_runtime(config_.runtime);
    const std::size_t boxes_before = boxes_.size();
    if (partition_native_mode()) {
        out.added = add_partition_box_corridor_overlay(start,
                                                       goal,
                                                       waypoint_path,
                                                       "debug_chain_pave_waypoints",
                                                       false,
                                                       false,
                                                       -1,
                                                       &last_build_);
        out.boundary_ffb_calls = static_cast<int>(
            last_build_.diagnostics[
                "debug_chain_pave_waypoints.partition_box_corridor_overlay_attempts"]);
        out.boundary_commits = out.added;
        export_debug_chain_pave_boxes(out, boxes_, boxes_before);
        if (out.added > 0) {
            invalidate_query_cache();
        }
        invalidate_query_cache();
        return out;
    }
    int next_id = next_box_id();
    ChainPaveConfig debug_pave = pave;
    debug_pave.debug_boundary_failures = &out.boundary_failures;
    out.added = chain_pave_along_path(
        waypoint_path,
        start_box_id,
        boxes_,
        *oracle_,
        adjacency_,
        next_id,
        context,
        debug_pave);
    if (boxes_.size() > boxes_before) {
        append_adaptive_partition_boxes(boxes_before, &last_build_, "debug_chain_pave_waypoints");
    }
    record_debug_chain_pave_context_diagnostics(out, context);
    export_debug_chain_pave_boxes(out, boxes_, boxes_before);
    if (out.added > 0) {
        invalidate_query_cache();
    }
    invalidate_query_cache();
    return out;
}

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
        probe.success && probe.audit_passed && probe.repair_count == 0 && probe.segment_edges_used == 0 && !probe.path.empty();
    if (graph_only_success) {
        if (mode != CorridorRefineMode::BoxOnlyLongPath) {
            return 0;
        }
        const double direct = (goal - start).norm();
        const double delta = probe.path_length - direct;
        const bool ratio_trigger =
            direct > 1e-9 && std::isfinite(long_path_ratio) && probe.path_length / direct >= long_path_ratio;
        const bool delta_trigger = std::isfinite(long_path_min_delta) && delta >= long_path_min_delta;
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
                direct > 1e-9 && std::isfinite(long_path_ratio) && probe.path_length / direct >= long_path_ratio;
            const bool delta_trigger = std::isfinite(long_path_min_delta) && delta >= long_path_min_delta;
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
    const int anchor_box_id = locate_box_partition_first(waypoint_path.front(), config_.query.nearest_if_outside);
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
    const int added = chain_pave_along_path(
        waypoint_path,
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
            const int source_box_id = locate_box_partition_first(start, config_.query.nearest_if_outside);
            const int target_box_id = locate_box_partition_first(goal, config_.query.nearest_if_outside);
            if (source_box_id >= 0 && target_box_id >= 0) {
                add_segment_edge_partition_first(                                 source_box_id,
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

} // namespace rbf
