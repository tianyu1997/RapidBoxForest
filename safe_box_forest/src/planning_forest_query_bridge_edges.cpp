#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_query_bridge_batch_utils.h"
#include "planning_forest_query_bridge_corridor_utils.h"
#include "planning_forest_query_bridge_hipac_utils.h"
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

int RBFPlanningForest::try_hipac_online_bridge_task(
    QueryBridgeSearchTask& task,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    StageContext& context,
    int query_index) {
    const QueryBridgeHipacOnlineGate hipac_online_gate =
        query_bridge_hipac_online_gate(last_adaptive_partition_config_,
                                       partition_native_mode(),
                                       static_cast<int>(task.hipac_candidate_path.size()),
                                       task.hipac_online_resolves_used);
    if (!hipac_online_gate.enabled) {
        return 0;
    }
    task.hipac_online_resolves_used += 1;
    const auto hipac_t0 = QueryBridgeEdgeClock::now();
    context.diagnostics().add_counter("query_bridge.hipac_online_attempts");
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "hipac_online_attempt"),
        1.0);
    std::vector<Eigen::VectorXd> hipac_path = task.hipac_candidate_path;
    if (hipac_path.size() > 2U) {
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        const double before_length = path_length(hipac_path);
        std::vector<Eigen::VectorXd> shortened =
            collision_shortcut_path(hipac_path,
                                    checker,
                                    collision_shortcut_resolution(config_.query));
        if (shortened.size() >= 2U &&
            path_length(shortened) <= before_length + 1e-12) {
            const PathAuditCheck audit =
                audit_waypoint_path(shortened,
                                    checker,
                                    config_.query.audit_resolution,
                                    config_.query.audit_segment_step);
            if (audit.passed) {
                context.diagnostics().add_counter(
                    "query_bridge.hipac_online_shortcut_accepts");
                context.diagnostics().add_counter(
                    "query_bridge.hipac_online_shortcut_delta",
                    std::max(0.0, before_length - path_length(shortened)));
                hipac_path = std::move(shortened);
            } else {
                context.diagnostics().add_counter(
                    "query_bridge.hipac_online_shortcut_audit_rejects");
            }
        }
    }
    const double hipac_candidate_length = path_length(hipac_path);
    context.diagnostics().add_counter("query_bridge.hipac_online_candidate_length",
                                      hipac_candidate_length);
    if (hipac_online_gate.candidate_max_length > 0.0 &&
        hipac_candidate_length > hipac_online_gate.candidate_max_length + 1e-12) {
        context.diagnostics().add_counter(
            "query_bridge.hipac_online_candidate_length_rejects");
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "hipac_online_length_reject"),
            1.0);
        const double hipac_ms = query_bridge_edge_elapsed_ms_since(hipac_t0);
        context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
                                            hipac_ms);
        context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
                                          hipac_ms);
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "hipac_online_ms"),
            hipac_ms);
        context.diagnostics().add_counter("query_bridge.hipac_online_failures");
        return 0;
    }
    int added = add_partition_box_corridor_overlay(task.start,
                                                   task.goal,
                                                   hipac_path,
                                                   "query_bridge.hipac_online",
                                                   true,
                                                   false,
                                                   query_index,
                                                   &last_build_);
    if (added <= 0 && last_adaptive_partition_config_.hipac_online_ffb_portal_fallback) {
        added = add_partition_portal_corridor_overlay(task.start,
                                                      task.goal,
                                                      hipac_path,
                                                      "query_bridge.hipac_online",
                                                      true,
                                                      false,
                                                      query_index,
                                                      &last_build_);
    }
    const double hipac_ms = query_bridge_edge_elapsed_ms_since(hipac_t0);
    context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
                                        hipac_ms);
    context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
                                      hipac_ms);
    context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_ms"),
                                    hipac_ms);
    if (added <= 0) {
        context.diagnostics().add_counter("query_bridge.hipac_online_failures");
        return 0;
    }
    context.diagnostics().add_counter("query_bridge.hipac_online_added",
                                      static_cast<double>(added));
    context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_added"),
                                    static_cast<double>(added));
    const QueryResult probe_after_hipac = query(task.start, task.goal);
    if (query_bridge_result_acceptable(probe_after_hipac,
                                       task.start,
                                       task.goal,
                                       bridge_acceptance)) {
        task.hipac_online_satisfied = true;
        context.diagnostics().add_counter("query_bridge.hipac_online_satisfied");
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "hipac_online_satisfied"),
            1.0);
    } else {
        context.diagnostics().add_counter("query_bridge.hipac_online_not_sufficient");
    }
    return added;
}

int RBFPlanningForest::try_hipac_prebridge_portal_task(
    QueryBridgeSearchTask& task,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    StageContext& context,
    int query_index) {
    const QueryBridgeHipacPrebridgeGate prebridge_gate =
        query_bridge_hipac_prebridge_gate(last_adaptive_partition_config_,
                                          partition_native_mode(),
                                          adaptive_partition_query_enabled_,
                                          adaptive_partition_ && !adaptive_partition_->empty(),
                                          task.hipac_prebridge_resolves_used);
    if (!prebridge_gate.enabled) {
        return 0;
    }
    std::vector<Eigen::VectorXd> coarse_route = task.hipac_candidate_path;
    if (coarse_route.size() < 2U) {
        coarse_route = {task.start, task.goal};
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_direct_query_route");
    }
    if (coarse_route.size() < 2U) {
        return 0;
    }
    const auto prebridge_t0 = QueryBridgeEdgeClock::now();
    context.diagnostics().add_counter("query_bridge.hipac_prebridge_attempts");
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "hipac_prebridge_attempt"),
        1.0);
    const auto candidate_pairs =
        adaptive_partition_->nearest_component_pairs_to_largest(
            1,
            prebridge_gate.candidate_limit);
    context.diagnostics().add_counter(
        "query_bridge.hipac_prebridge_candidates",
        static_cast<double>(candidate_pairs.size()));
    if (candidate_pairs.empty()) {
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_no_candidates");
        return 0;
    }

    const auto components = adaptive_partition_->component_box_ids_with_overlay();
    const int start_box_id = locate_box_partition_first(task.start, false);
    const int goal_box_id = locate_box_partition_first(task.goal, false);
    const QueryBridgeHipacPrebridgeSelection prebridge_selection =
        query_bridge_select_hipac_prebridge_pair(candidate_pairs,
                                                 components,
                                                 start_box_id,
                                                 goal_box_id,
                                                 coarse_route,
                                                 prebridge_gate.max_pair_distance,
                                                 prebridge_gate.route_weight,
                                                 prebridge_gate.pair_weight);
    context.diagnostics().add_counter(
        "query_bridge.hipac_prebridge_considered",
        static_cast<double>(prebridge_selection.considered));
    context.diagnostics().add_counter(
        "query_bridge.hipac_prebridge_distance_rejects",
        static_cast<double>(prebridge_selection.distance_rejects));
    context.diagnostics().add_counter(
        "query_bridge.hipac_prebridge_endpoint_component_rejects",
        static_cast<double>(prebridge_selection.endpoint_component_rejects));
    if (prebridge_selection.candidate_index < 0 ||
        prebridge_selection.candidate_index >=
            static_cast<int>(candidate_pairs.size())) {
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_no_candidate_after_filter");
        return 0;
    }
    const AdaptiveGridPartitionComponentPair& best_pair =
        candidate_pairs[static_cast<std::size_t>(
            prebridge_selection.candidate_index)];

    task.hipac_prebridge_resolves_used += 1;
    context.diagnostics().add_counter(
        "query_bridge.hipac_prebridge_portal_attempts");
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "hipac_prebridge_score"),
        prebridge_selection.score);
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "hipac_prebridge_pair_distance"),
        std::sqrt(std::max(0.0, best_pair.distance_sq)));
    std::vector<Eigen::VectorXd> local_path{
        best_pair.source_point,
        best_pair.target_point};
    const int added = add_partition_portal_corridor_overlay(
        best_pair.source_point,
        best_pair.target_point,
        local_path,
        "query_bridge.hipac_online_prebridge",
        false,
        true,
        query_index,
        &last_build_);
    const double prebridge_ms =
        query_bridge_edge_elapsed_ms_since(prebridge_t0);
    context.diagnostics().record_timing("query_bridge.hipac_prebridge_ms_total",
                                        prebridge_ms);
    context.diagnostics().add_counter("query_bridge.hipac_prebridge_ms_total",
                                      prebridge_ms);
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "hipac_prebridge_ms"),
        prebridge_ms);
    if (added <= 0) {
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_failures");
        return 0;
    }
    context.diagnostics().add_counter("query_bridge.hipac_prebridge_added",
                                      static_cast<double>(added));
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "hipac_prebridge_added"),
        static_cast<double>(added));
    const QueryResult probe_after_prebridge = query(task.start, task.goal);
    if (query_bridge_result_acceptable(probe_after_prebridge,
                                       task.start,
                                       task.goal,
                                       bridge_acceptance)) {
        task.hipac_online_satisfied = true;
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_satisfied");
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "hipac_prebridge_satisfied"),
            1.0);
    } else {
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_not_sufficient");
    }
    return added;
}

int RBFPlanningForest::try_hipac_transition_portal_task(
    QueryBridgeSearchTask& task,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    StageContext& context,
    int query_index) {
    const QueryBridgeHipacTransitionGate transition_gate =
        query_bridge_hipac_transition_gate(last_adaptive_partition_config_,
                                           partition_native_mode(),
                                           adaptive_partition_query_enabled_,
                                           adaptive_partition_ && !adaptive_partition_->empty(),
                                           static_cast<int>(task.waypoint_path.size()),
                                           task.hipac_transition_resolves_used,
                                           static_cast<int>(task.index),
                                           task.query_index);
    if (transition_gate.disabled) {
        return 0;
    }
    if (transition_gate.target_rejected) {
        context.diagnostics().add_counter(
            "query_bridge.hipac_transition_target_rejects");
        return 0;
    }

    const auto transition_t0 = QueryBridgeEdgeClock::now();
    context.diagnostics().add_counter("query_bridge.hipac_transition_attempts");
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "hipac_transition_attempt"),
        1.0);
    const int stride =
        std::max(1, last_adaptive_partition_config_.hipac_transition_window_stride);
    const int candidate_limit =
        std::max(1, last_adaptive_partition_config_.hipac_transition_candidate_limit);
    const int min_predicted_edges =
        std::max(0, last_adaptive_partition_config_.hipac_transition_min_predicted_bridge_edges);
    const double max_pair_distance =
        std::max(0.0, last_adaptive_partition_config_.hipac_transition_max_pair_distance);
    const double sample_step =
        query_bridge_direct_corridor_runtime_options(
            task.query_index,
            config_.query.audit_segment_step).sample_step;
    const QueryBridgeHipacTransitionCandidateSet transition_candidates =
        query_bridge_select_hipac_transition_candidates(
            *adaptive_partition_,
            task.waypoint_path,
            stride,
            candidate_limit,
            min_predicted_edges,
            max_pair_distance,
            sample_step,
            last_adaptive_partition_config_.hipac_transition_allow_same_component);
    context.diagnostics().add_counter(
        "query_bridge.hipac_transition_candidates",
        static_cast<double>(transition_candidates.candidates.size()));
    context.diagnostics().add_counter(
        "query_bridge.hipac_transition_gated",
        static_cast<double>(transition_candidates.gated +
                            transition_candidates.same_component_gated +
                            transition_candidates.distance_gated +
                            transition_candidates.edge_gated));
    context.diagnostics().add_counter(
        "query_bridge.hipac_transition_gated_same_component",
        static_cast<double>(transition_candidates.same_component_gated));
    context.diagnostics().add_counter(
        "query_bridge.hipac_transition_gated_distance",
        static_cast<double>(transition_candidates.distance_gated));
    context.diagnostics().add_counter(
        "query_bridge.hipac_transition_gated_edges",
        static_cast<double>(transition_candidates.edge_gated));
    if (transition_candidates.candidates.empty()) {
        return 0;
    }

    int total_added = 0;
    int attempts = 0;
    const int attempt_cap = transition_gate.attempt_cap;
    for (const auto& candidate : transition_candidates.candidates) {
        if (attempts >= attempt_cap ||
            task.hipac_transition_resolves_used >= attempt_cap) {
            break;
        }
        ++attempts;
        task.hipac_transition_resolves_used += 1;
        context.diagnostics().add_counter(
            "query_bridge.hipac_transition_portal_attempts");
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "hipac_transition_predicted_edges"),
            static_cast<double>(candidate.predicted_bridge_edges));
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "hipac_transition_pair_distance"),
            candidate.pair_distance);
        const int added = add_partition_portal_corridor_overlay(
            candidate.source_point,
            candidate.target_point,
            candidate.local_path,
            "query_bridge.hipac_online_transition",
            false,
            false,
            query_index,
            &last_build_);
        if (added <= 0) {
            context.diagnostics().add_counter(
                "query_bridge.hipac_transition_failures");
            continue;
        }
        total_added += added;
        context.diagnostics().add_counter("query_bridge.hipac_transition_added",
                                          static_cast<double>(added));
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "hipac_transition_added"),
            static_cast<double>(added));
        const QueryResult probe_after_transition = query(task.start, task.goal);
        if (probe_after_transition.success &&
            probe_after_transition.audit_passed &&
            !probe_after_transition.path.empty()) {
            task.waypoint_path = probe_after_transition.path;
            task.hipac_candidate_path = probe_after_transition.path;
            context.diagnostics().add_counter(
                "query_bridge.hipac_transition_probe_path_adopted");
            context.diagnostics().set_value(
                query_bridge_task_key(task.index, "hipac_transition_probe_path_length"),
                probe_after_transition.path_length);
        }
        if (query_bridge_result_acceptable(probe_after_transition,
                                           task.start,
                                           task.goal,
                                           bridge_acceptance)) {
            task.hipac_online_satisfied = true;
            context.diagnostics().add_counter(
                "query_bridge.hipac_transition_satisfied");
            context.diagnostics().set_value(
                query_bridge_task_key(task.index, "hipac_transition_satisfied"),
                1.0);
            break;
        }
        context.diagnostics().add_counter(
            "query_bridge.hipac_transition_not_sufficient");
    }
    const double transition_ms =
        query_bridge_edge_elapsed_ms_since(transition_t0);
    context.diagnostics().record_timing("query_bridge.hipac_transition_ms_total",
                                        transition_ms);
    context.diagnostics().add_counter("query_bridge.hipac_transition_ms_total",
                                      transition_ms);
    context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_ms"),
                                    transition_ms);
    return total_added;
}

bool RBFPlanningForest::run_query_bridge_hipac_online_sequence_task(
    QueryBridgeSearchTask& task,
    int& added_for_task,
    StageContext& context,
    bool scene_reusable_edges,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance) {
    int added = try_hipac_online_bridge_task(
        task,
        bridge_acceptance,
        context,
        query_bridge_edge_query_index(scene_reusable_edges, task));
    if (added > 0) {
        added_for_task += added;
    }
    if (!task.hipac_online_satisfied) {
        added = try_hipac_transition_portal_task(
            task,
            bridge_acceptance,
            context,
            query_bridge_edge_query_index(scene_reusable_edges, task));
        if (added > 0) {
            added_for_task += added;
        }
    }
    if (!task.hipac_online_satisfied) {
        added = try_hipac_prebridge_portal_task(
            task,
            bridge_acceptance,
            context,
            query_bridge_edge_query_index(scene_reusable_edges, task));
        if (added > 0) {
            added_for_task += added;
        }
    }
    return task.hipac_online_satisfied;
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

int RBFPlanningForest::try_add_query_direct_start_goal_segment_for_points(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    StageContext& context,
    int query_index,
    int batch_task_index) {
    const int source_box_id = locate_query_bridge_box(start);
    const int target_box_id = locate_query_bridge_box(goal);
    return try_add_query_direct_start_goal_segment_edge(source_box_id,
                                                       target_box_id,
                                                       start,
                                                       goal,
                                                       context,
                                                       query_index,
                                                       batch_task_index);
}

void RBFPlanningForest::run_query_bridge_direct_start_goal_segments(
    std::vector<QueryBridgeSearchTask>& tasks,
    std::vector<int>& added_by_query,
    StageContext& context,
    bool scene_reusable_edges,
    bool enabled) {
    if (!enabled) {
        return;
    }
    for (auto& task : tasks) {
        if (task.direct_start_goal_satisfied) {
            continue;
        }
        const int added = try_add_query_direct_start_goal_segment_for_points(
            task.start,
            task.goal,
            context,
            query_bridge_edge_query_index(scene_reusable_edges, task),
            static_cast<int>(task.index));
        task.direct_start_goal_satisfied = added > 0;
        if (added > 0) {
            added_by_query[task.index] += added;
        }
    }
}

int RBFPlanningForest::run_query_bridge_waypoint_fallbacks(
    QueryBridgeSearchTask& task,
    int& added_for_task,
    StageContext& context,
    bool scene_reusable_edges,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    const QueryBridgeBatchExecutionOptions& batch_options) {
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
            if (!batch_options.evaluate_all_fallback_paths) {
                break;
            }
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
    const QueryBridgeBatchExecutionOptions& batch_options,
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
                                        bridge_acceptance,
                                        batch_options);
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

int RBFPlanningForest::try_add_query_fast_direct_segment_after_rrt_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    const RRTConnectConfig& bridge_rrt,
    StageContext& context,
    bool enabled,
    bool shortcut_enabled,
    int random_shortcut_iters,
    double min_length,
    int shortcut_query_index,
    int edge_query_index,
    int batch_task_index) {
    if (!enabled || waypoint_path.empty()) {
        return 0;
    }
    const auto add_task_counter = [&](const std::string& suffix, double value = 1.0) {
        if (batch_task_index >= 0) {
            context.diagnostics().add_counter(
                query_bridge_task_key(static_cast<std::size_t>(batch_task_index), suffix),
                value);
        }
    };

    std::vector<std::vector<Eigen::VectorXd>> candidate_paths;
    candidate_paths.push_back(waypoint_path);
    if (shortcut_enabled && waypoint_path.size() > 2U) {
        const double before_length = path_length(waypoint_path);
        CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
        std::vector<Eigen::VectorXd> shortened =
            collision_shortcut_path(waypoint_path,
                                    checker,
                                    collision_shortcut_resolution(config_.query));
        const double after_length = path_length(shortened);
        context.diagnostics().add_counter(
            "query_bridge.fast_direct_segment_after_rrt_shortcut_attempts");
        add_task_counter("fast_direct_segment_after_rrt_shortcut_attempts");
        if (!shortened.empty() && after_length + 1e-12 < before_length) {
            candidate_paths.push_back(std::move(shortened));
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_shortcut_accepts");
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_shortcut_delta",
                before_length - after_length);
            add_task_counter("fast_direct_segment_after_rrt_shortcut_accepts");
            add_task_counter("fast_direct_segment_after_rrt_shortcut_delta",
                             before_length - after_length);
        }
        const auto& random_source = candidate_paths.back();
        if (random_shortcut_iters > 0 && random_source.size() > 2U) {
            const double random_before_length = path_length(random_source);
            const std::uint32_t shortcut_seed =
                static_cast<std::uint32_t>(
                    0x9e3779b9U ^
                    ((static_cast<std::uint32_t>(shortcut_query_index) + 1U) *
                     2654435761U) ^
                    (static_cast<std::uint32_t>(batch_task_index + 1) * 2246822519U) ^
                    static_cast<std::uint32_t>(random_source.size()));
            std::vector<Eigen::VectorXd> random_shortened =
                random_collision_shortcut_path(random_source,
                                               checker,
                                               collision_shortcut_resolution(config_.query),
                                               random_shortcut_iters,
                                               shortcut_seed);
            const double random_after_length = path_length(random_shortened);
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_random_shortcut_attempts");
            add_task_counter("fast_direct_segment_after_rrt_random_shortcut_attempts");
            context.diagnostics().add_counter(
                "query_bridge.fast_direct_segment_after_rrt_random_shortcut_iters",
                static_cast<double>(random_shortcut_iters));
            if (!random_shortened.empty() &&
                random_after_length + 1e-12 < random_before_length) {
                candidate_paths.push_back(std::move(random_shortened));
                context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_accepts");
                context.diagnostics().add_counter(
                    "query_bridge.fast_direct_segment_after_rrt_random_shortcut_delta",
                    random_before_length - random_after_length);
                add_task_counter("fast_direct_segment_after_rrt_random_shortcut_accepts");
                add_task_counter("fast_direct_segment_after_rrt_random_shortcut_delta",
                                 random_before_length - random_after_length);
            }
        }
    }
    std::sort(candidate_paths.begin(),
              candidate_paths.end(),
              [](const auto& lhs, const auto& rhs) {
                  return path_length(lhs) < path_length(rhs);
              });
    candidate_paths.erase(std::unique(candidate_paths.begin(),
                                      candidate_paths.end(),
                                      [](const auto& lhs, const auto& rhs) {
                                          if (lhs.size() != rhs.size()) {
                                              return false;
                                          }
                                          for (std::size_t index = 0; index < lhs.size(); ++index) {
                                              if ((lhs[index] - rhs[index]).norm() > 1e-12) {
                                                  return false;
                                              }
                                          }
                                          return true;
                                      }),
                          candidate_paths.end());
    const int source_box_id = locate_query_bridge_box(start);
    const int target_box_id = locate_query_bridge_box(goal);
    return try_add_query_fast_direct_segment_after_rrt_edge(source_box_id,
                                                           target_box_id,
                                                           candidate_paths,
                                                           bridge_rrt,
                                                           context,
                                                           min_length,
                                                           edge_query_index,
                                                           batch_task_index);
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
