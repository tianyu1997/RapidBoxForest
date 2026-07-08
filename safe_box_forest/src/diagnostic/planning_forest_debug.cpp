#include <SBF/safe_box_forest.h>

#include <SBF/debug.h>
#include <SBF/scene.h>

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API

#include <SBF/connector.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <vector>

#include "../planning_core/planning_forest_audit.h"
#include "../query_runtime/planning_forest_query_utils.h"

namespace rbf {

const OracleCounters* RBFPlanningForest::oracle_counters() const {
    return oracle_ ? &oracle_->counters() : nullptr;
}

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

} // namespace rbf

#endif
