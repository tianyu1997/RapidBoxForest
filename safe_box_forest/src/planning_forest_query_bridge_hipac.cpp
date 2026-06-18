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
#include <utility>
#include <vector>

namespace rbf {

namespace {

using QueryBridgeHipacClock = std::chrono::steady_clock;

double query_bridge_hipac_elapsed_ms_since(QueryBridgeHipacClock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
        QueryBridgeHipacClock::now() - t0).count();
}

}  // namespace

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
    const auto hipac_t0 = QueryBridgeHipacClock::now();
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
        const double hipac_ms = query_bridge_hipac_elapsed_ms_since(hipac_t0);
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
    const double hipac_ms = query_bridge_hipac_elapsed_ms_since(hipac_t0);
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
    const auto prebridge_t0 = QueryBridgeHipacClock::now();
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
        query_bridge_hipac_elapsed_ms_since(prebridge_t0);
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

    const auto transition_t0 = QueryBridgeHipacClock::now();
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
        query_bridge_hipac_elapsed_ms_since(transition_t0);
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

}  // namespace rbf
