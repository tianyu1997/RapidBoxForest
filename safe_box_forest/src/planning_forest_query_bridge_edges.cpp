#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"

#include <algorithm>

namespace rbf {

int RBFPlanningForest::try_add_query_box_corridor_edge(
    int source_box_id,
    int target_box_id,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    double segment_resolution,
    int query_index) {
    if (source_box_id < 0 || target_box_id < 0 ||
        !box_only_path_connected_partition_first(source_box_id, target_box_id)) {
        return -1;
    }
    const int edge_id = add_segment_edge_partition_first(source_box_id,
                                                         target_box_id,
                                                         waypoint_path,
                                                         SegmentEdgeType::BoxCorridor,
                                                         segment_resolution,
                                                         SegmentEdgeValidation::CollisionChecked,
                                                         false,
                                                         query_index);
    if (edge_id >= 0) {
        invalidate_query_cache();
        return 1;
    }
    return 0;
}

int RBFPlanningForest::try_add_query_direct_segment_after_rrt_edge(
    int source_box_id,
    int target_box_id,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const RRTConnectConfig& bridge_rrt,
    const CollisionChecker& checker,
    StageContext& context,
    double original_path_length,
    double audited_path_length,
    int query_index,
    bool enabled) {
    if (!enabled) {
        return 0;
    }
    context.diagnostics().add_counter(
        "query_bridge.direct_segment_after_rrt_final_attempts");
    context.diagnostics().add_counter(
        "query_bridge.direct_segment_after_rrt_shortening_delta",
        std::max(0.0, original_path_length - audited_path_length));
    const PathAuditCheck segment_audit =
        audit_waypoint_path(waypoint_path,
                            checker,
                            config_.query.audit_resolution,
                            config_.query.audit_segment_step);
    if (!segment_audit.passed) {
        context.diagnostics().add_counter(
            "query_bridge.direct_segment_after_rrt_audit_rejects");
        return 0;
    }
    const int edge_id = add_segment_edge_partition_first(
        source_box_id,
        target_box_id,
        waypoint_path,
        SegmentEdgeType::QueryBridge,
        bridge_rrt.segment_resolution,
        SegmentEdgeValidation::CollisionChecked,
        true,
        query_index);
    if (edge_id >= 0) {
        context.diagnostics().add_counter(
            "query_bridge.direct_segment_after_rrt_edges");
        invalidate_query_cache();
        sync_adaptive_partition_segment_edges(
            &last_build_,
            "query_bridge.direct_segment_after_rrt");
        refresh_adaptive_partition_diagnostics(&last_build_);
        return 1;
    }
    context.diagnostics().add_counter(
        "query_bridge.direct_segment_after_rrt_add_fail");
    return 0;
}

int RBFPlanningForest::try_add_query_residual_segment_edge(
    int source_box_id,
    int target_box_id,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const RRTConnectConfig& bridge_rrt,
    const CollisionChecker& checker,
    StageContext& context,
    double depth_failures_before,
    int query_index,
    bool enabled) {
    if (!enabled) {
        return 0;
    }
    const bool max_depth_ffb_failed =
        boundary_max_depth_failure_count_local(context) > depth_failures_before + 0.5;
    if (!config_.connector.segment_edges_enabled || !config_.connector.rrt_segment_edges) {
        return 0;
    }
    if (!max_depth_ffb_failed) {
        context.diagnostics().add_counter(
            "query_bridge.segment_edge_blocked_no_max_depth_ffb_failure");
        return 0;
    }
    if (source_box_id < 0 || target_box_id < 0) {
        return 0;
    }
    const PathAuditCheck segment_audit =
        audit_waypoint_path(waypoint_path,
                            checker,
                            config_.query.audit_resolution,
                            config_.query.audit_segment_step);
    if (!segment_audit.passed) {
        context.diagnostics().add_counter("query_bridge.segment_edge_audit_rejects");
        return 0;
    }
    const int edge_id = add_segment_edge_partition_first(source_box_id,
                                                         target_box_id,
                                                         waypoint_path,
                                                         SegmentEdgeType::QueryBridge,
                                                         bridge_rrt.segment_resolution,
                                                         SegmentEdgeValidation::CollisionChecked,
                                                         true,
                                                         query_index);
    if (edge_id >= 0) {
        invalidate_query_cache();
        return 1;
    }
    return 0;
}

} // namespace rbf
