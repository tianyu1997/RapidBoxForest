#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_query_bridge_corridor_graph.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_hipac_utils.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_task.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace rbf {

namespace {

using QueryBridgeEdgeClock = std::chrono::steady_clock;

double query_bridge_edge_elapsed_ms_since(QueryBridgeEdgeClock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
        QueryBridgeEdgeClock::now() - t0).count();
}

}  // namespace

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

int RBFPlanningForest::run_query_bridge_waypoint_fallbacks(
    QueryBridgeSearchTask& task,
    int& added_for_task,
    StageContext& context,
    bool scene_reusable_edges,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance) {
    std::vector<const std::vector<Eigen::VectorXd>*> candidate_paths;
    if (!task.waypoint_path.empty()) {
        candidate_paths.push_back(&task.waypoint_path);
    }
    for (const auto& fallback : task.waypoint_fallback_paths) {
        if (!fallback.empty()) {
            candidate_paths.push_back(&fallback);
        }
    }

    int total_added = 0;
    for (std::size_t candidate_index = 0;
         candidate_index < candidate_paths.size();
         ++candidate_index) {
        const auto& candidate_path = *candidate_paths[candidate_index];
        if (candidate_path.empty()) {
            continue;
        }
        if (candidate_index > 0U) {
            context.diagnostics().add_counter(
                "query_bridge.waypoint_quality_fallback_attempts");
            context.diagnostics().add_counter(
                query_bridge_task_key(task.index,
                                      "waypoint_quality_fallback_attempts"));
        }
        const int bridge_added =
            bridge_query_with_waypoint_path(task.start,
                                            task.goal,
                                            candidate_path,
                                            task.short_local_bridge,
                                            task.bridge_rrt,
                                            task.query_index);
        total_added += bridge_added;
        added_for_task += bridge_added;
        const int promoted = try_promote_query_repair_to_hipac(
            task.start,
            task.goal,
            task.waypoint_path,
            bridge_added,
            query_bridge_edge_query_index(scene_reusable_edges, task),
            static_cast<int>(task.index),
            context);
        if (promoted > 0) {
            added_for_task += promoted;
        }
        accumulate_query_bridge_direct_corridor_totals(last_build_,
                                                       context,
                                                       task.index);
        if (candidate_index > 0U) {
            context.diagnostics().add_counter(
                "query_bridge.waypoint_quality_fallback_added",
                static_cast<double>(bridge_added));
        }
        const bool should_check =
            query_bridge_should_check_current_query(task,
                                                    false,
                                                    index_options,
                                                    retry_options);
        if (should_check &&
            query_bridge_result_acceptable(query(task.start, task.goal),
                                           task.start,
                                           task.goal,
                                           bridge_acceptance)) {
            if (candidate_index > 0U) {
                context.diagnostics().add_counter(
                    "query_bridge.waypoint_quality_fallback_successes");
                context.diagnostics().set_value(
                    query_bridge_task_key(task.index,
                                          "waypoint_quality_fallback_success"),
                    1.0);
                task.waypoint_path = candidate_path;
            }
            break;
        }
    }
    return total_added;
}

void RBFPlanningForest::finish_query_bridge_ready_waypoint_task(
    QueryBridgeSearchTask& task,
    int& added_for_task,
    bool forced_task,
    bool segment_only_task,
    double best_length,
    StageContext& context,
    bool scene_reusable_edges,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    bool fast_direct_segment_after_rrt,
    bool fast_direct_shortcut,
    int fast_direct_random_shortcut_iters,
    double fast_direct_segment_after_rrt_min_length,
    const std::function<double()>& task_elapsed_ms) {
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "waypoint_length"),
        best_length);
    const auto second_probe_t0 = QueryBridgeEdgeClock::now();
    const bool should_check =
        query_bridge_should_check_current_query(
            task,
            !retry_options.post_rrt_skip_forced,
            index_options,
            retry_options);
    if (should_check &&
        query_bridge_result_acceptable(query(task.start, task.goal),
                                       task.start,
                                       task.goal,
                                       bridge_acceptance)) {
        record_query_bridge_batch_task_skipped_after_rrt(
            context,
            task.index,
            forced_task,
            query_bridge_edge_elapsed_ms_since(second_probe_t0),
            task_elapsed_ms());
        return;
    }
    context.diagnostics().record_timing(
        "query_bridge.batch_probe_ms_total",
        query_bridge_edge_elapsed_ms_since(second_probe_t0));

    if (query_bridge_hipac_after_rrt_available(last_adaptive_partition_config_,
                                               task)) {
        task.hipac_candidate_path = task.waypoint_path;
        if (run_query_bridge_hipac_online_sequence_task(task,
                                                        added_for_task,
                                                        context,
                                                        scene_reusable_edges,
                                                        bridge_acceptance)) {
            record_query_bridge_batch_task_skipped_by_hipac_after_rrt(
                context,
                task.index,
                task_elapsed_ms());
            return;
        }
    }

    const int fast_direct_added =
        try_add_query_fast_direct_segment_after_rrt_path(
            task.start,
            task.goal,
            task.waypoint_path,
            task.bridge_rrt,
            context,
            fast_direct_segment_after_rrt,
            fast_direct_shortcut,
            fast_direct_random_shortcut_iters,
            fast_direct_segment_after_rrt_min_length,
            task.query_index,
            query_bridge_edge_query_index(scene_reusable_edges, task),
            static_cast<int>(task.index));
    if (fast_direct_added > 0) {
        added_for_task += fast_direct_added;
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "fast_direct_segment_after_rrt"),
            1.0);
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "added"),
            static_cast<double>(added_for_task));
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "total_ms"),
            task_elapsed_ms());
        return;
    }

    if (segment_only_task) {
        const int segment_only_added =
            try_commit_query_bridge_segment_only_edge(
                task.start,
                task.goal,
                task.waypoint_path,
                task.bridge_rrt.segment_resolution,
                query_bridge_edge_query_index(scene_reusable_edges, task),
                static_cast<int>(task.index),
                context);
        if (segment_only_added > 0) {
            added_for_task += segment_only_added;
        }
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "total_ms"),
            task_elapsed_ms());
        return;
    }

    const auto pave_t0 = QueryBridgeEdgeClock::now();
    run_query_bridge_waypoint_fallbacks(task,
                                        added_for_task,
                                        context,
                                        scene_reusable_edges,
                                        index_options,
                                        retry_options,
                                        bridge_acceptance);
    const double pave_ms = query_bridge_edge_elapsed_ms_since(pave_t0);
    context.diagnostics().record_timing("query_bridge.batch_pave_ms_total",
                                        pave_ms);
    context.diagnostics().set_value(query_bridge_task_key(task.index, "pave_ms"),
                                    pave_ms);
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "added"),
        static_cast<double>(added_for_task));
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "total_ms"),
        task_elapsed_ms());
}

int RBFPlanningForest::try_commit_query_bridge_segment_only_edge(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    int segment_resolution,
    int query_index,
    int batch_task_index,
    StageContext& context) {
    const auto task_key = [&](const std::string& suffix) {
        return query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix);
    };
    const int source_box_id = locate_box_partition_first(
        start,
        config_.query.nearest_if_outside);
    const int target_box_id = locate_box_partition_first(
        goal,
        config_.query.nearest_if_outside);
    int edge_id = -1;
    if (source_box_id >= 0 && target_box_id >= 0) {
        edge_id = add_segment_edge_partition_first(source_box_id,
                                                   target_box_id,
                                                   waypoint_path,
                                                   SegmentEdgeType::QueryBridge,
                                                   segment_resolution,
                                                   SegmentEdgeValidation::CollisionChecked,
                                                   true,
                                                   query_index);
    }
    if (edge_id >= 0) {
        invalidate_query_cache();
        context.diagnostics().add_counter(
            "query_bridge.batch_tasks_segment_only");
        if (batch_task_index >= 0) {
            context.diagnostics().set_value(task_key("segment_only"), 1.0);
            context.diagnostics().set_value(task_key("added"), 1.0);
        }
        return 1;
    }
    context.diagnostics().add_counter(
        "query_bridge.batch_tasks_segment_only_failures");
    if (batch_task_index >= 0) {
        context.diagnostics().set_value(task_key("segment_only_failure"), 1.0);
    }
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
