#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_query_bridge_batch_utils.h"

#include <algorithm>
#include <limits>
#include <string>

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
    return add_verified_query_box_corridor_edge(source_box_id,
                                                target_box_id,
                                                waypoint_path,
                                                segment_resolution,
                                                query_index);
}

int RBFPlanningForest::add_verified_query_box_corridor_edge(
    int source_box_id,
    int target_box_id,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    double segment_resolution,
    int query_index) {
    if (source_box_id < 0 || target_box_id < 0) {
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

int RBFPlanningForest::try_add_query_direct_start_goal_segment_edge(
    int source_box_id,
    int target_box_id,
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    StageContext& context,
    int query_index,
    int batch_task_index) {
    const auto add_task_counter = [&](const std::string& suffix) {
        if (batch_task_index >= 0) {
            context.diagnostics().add_counter(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix));
        }
    };
    if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
        context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_missing_endpoint");
        return 0;
    }
    std::vector<Eigen::VectorXd> direct_path{start, goal};
    context.diagnostics().add_counter(
        "query_bridge.direct_start_goal_segment_attempts");
    add_task_counter("direct_start_goal_segment_attempts");
    CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
    const PathAuditCheck audit =
        audit_waypoint_path(direct_path,
                            checker,
                            config_.query.audit_resolution,
                            config_.query.audit_segment_step);
    if (!audit.passed) {
        context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_audit_rejects");
        add_task_counter("direct_start_goal_segment_audit_rejects");
        return 0;
    }
    const int edge_id = add_segment_edge_partition_first(
        source_box_id,
        target_box_id,
        direct_path,
        SegmentEdgeType::QueryBridge,
        config_.query.audit_resolution,
        SegmentEdgeValidation::CollisionChecked,
        true,
        query_index);
    if (edge_id < 0) {
        context.diagnostics().add_counter(
            "query_bridge.direct_start_goal_segment_add_fail");
        add_task_counter("direct_start_goal_segment_add_fail");
        return 0;
    }
    context.diagnostics().add_counter(
        "query_bridge.direct_start_goal_segment_edges");
    add_task_counter("direct_start_goal_segment_edges");
    invalidate_query_cache();
    sync_adaptive_partition_segment_edges(&last_build_,
                                          "query_bridge.direct_start_goal_segment");
    refresh_adaptive_partition_diagnostics(&last_build_);
    return 1;
}

int RBFPlanningForest::try_add_query_fast_direct_segment_after_rrt_edge(
    int source_box_id,
    int target_box_id,
    const std::vector<std::vector<Eigen::VectorXd>>& candidate_paths,
    const RRTConnectConfig& bridge_rrt,
    StageContext& context,
    double min_length,
    int query_index,
    int batch_task_index) {
    const auto add_task_counter = [&](const std::string& suffix, double value = 1.0) {
        if (batch_task_index >= 0) {
            context.diagnostics().add_counter(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix),
                value);
        }
    };
    const auto set_task_value = [&](const std::string& suffix, double value) {
        if (batch_task_index >= 0) {
            context.diagnostics().set_value(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix),
                value);
        }
    };
    if (candidate_paths.empty() ||
        !(path_length(candidate_paths.front()) >= min_length)) {
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_length_rejects");
        add_task_counter("fast_direct_segment_after_rrt_length_rejects");
        return 0;
    }
    if (source_box_id < 0 || target_box_id < 0 || source_box_id == target_box_id) {
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_missing_endpoint");
        add_task_counter("fast_direct_segment_after_rrt_missing_endpoint");
        return 0;
    }
    CollisionChecker strict_checker = make_audit_checker(audit_robot_, scene_, config_.query);
    int edge_id = -1;
    double added_length = std::numeric_limits<double>::infinity();
    for (std::size_t candidate_index = 0;
         candidate_index < candidate_paths.size();
         ++candidate_index) {
        const auto& candidate_path = candidate_paths[candidate_index];
        if (path_length(candidate_path) + 1e-12 < min_length) {
            continue;
        }
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_add_candidates");
        const PathAuditCheck candidate_audit =
            audit_waypoint_path(candidate_path,
                                strict_checker,
                                config_.query.audit_resolution,
                                config_.query.audit_segment_step);
        if (!candidate_audit.passed) {
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_candidate_audit_rejects");
            add_task_counter("fast_direct_segment_after_rrt_candidate_audit_rejects");
            continue;
        }
        edge_id = add_segment_edge_partition_first(
            source_box_id,
            target_box_id,
            candidate_path,
            SegmentEdgeType::QueryBridge,
            bridge_rrt.segment_resolution,
            SegmentEdgeValidation::CollisionChecked,
            true,
            query_index);
        if (edge_id >= 0) {
            added_length = path_length(candidate_path);
            if (candidate_index > 0) {
                context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_fallback_candidate_success");
            }
            break;
        }
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_add_candidate_fail");
    }
    if (edge_id < 0) {
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_add_fail");
        add_task_counter("fast_direct_segment_after_rrt_add_fail");
        return 0;
    }
    invalidate_query_cache();
    context.diagnostics().add_counter(
        "query_bridge.fast_direct_segment_after_rrt_edges");
    add_task_counter("fast_direct_segment_after_rrt_edges");
    set_task_value("fast_direct_segment_after_rrt_length", added_length);
    return 1;
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

int RBFPlanningForest::try_add_query_direct_corridor_full_residual_edge(
    int source_box_id,
    int target_box_id,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const RRTConnectConfig& bridge_rrt,
    const CollisionChecker& checker,
    StageContext& context,
    int edge_query_index,
    int batch_task_query_index,
    bool local_overlay_connected,
    bool count_without_local_overlay_attempt) {
    if (source_box_id < 0 || target_box_id < 0) {
        return -1;
    }
    if (count_without_local_overlay_attempt) {
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_full_residual_without_local_overlay");
    }
    const PathAuditCheck full_residual_audit =
        audit_waypoint_path(waypoint_path,
                            checker,
                            config_.query.audit_resolution,
                            config_.query.audit_segment_step);
    if (!full_residual_audit.passed) {
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_full_residual_audit_rejects");
        return -2;
    }
    const int edge_id = add_segment_edge_partition_first(
        source_box_id,
        target_box_id,
        waypoint_path,
        SegmentEdgeType::QueryBridge,
        bridge_rrt.segment_resolution,
        SegmentEdgeValidation::CollisionChecked,
        true,
        edge_query_index);
    if (edge_id < 0) {
        return -1;
    }
    context.diagnostics().add_counter(
        "query_bridge.direct_corridor_full_residual_edges");
    if (local_overlay_connected) {
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_full_residual_edges_with_local_overlay");
    } else {
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_full_residual_edges_without_local_overlay");
    }
    if (batch_task_query_index >= 0) {
        context.diagnostics().set_value(
            "query_bridge.batch_task." + std::to_string(batch_task_query_index) +
                ".direct_corridor_full_residual_edge",
            1.0);
    }
    return edge_id;
}

} // namespace rbf
