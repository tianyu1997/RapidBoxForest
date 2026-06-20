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
    const QueryBridgeTaskDiagnostics task_diag(context, task.index);
    task.hipac_online_resolves_used += 1;
    const auto hipac_t0 = QueryBridgeHipacClock::now();
    context.diagnostics().add_counter("query_bridge.hipac_online_attempts");
    task_diag.set_value("hipac_online_attempt", 1.0);
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
        task_diag.set_value("hipac_online_length_reject", 1.0);
        const double hipac_ms = query_bridge_hipac_elapsed_ms_since(hipac_t0);
        context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
                                            hipac_ms);
        context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
                                          hipac_ms);
        task_diag.set_value("hipac_online_ms", hipac_ms);
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
    const double hipac_ms = query_bridge_hipac_elapsed_ms_since(hipac_t0);
    context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
                                        hipac_ms);
    context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
                                      hipac_ms);
    task_diag.set_value("hipac_online_ms", hipac_ms);
    if (added <= 0) {
        context.diagnostics().add_counter("query_bridge.hipac_online_failures");
        return 0;
    }
    context.diagnostics().add_counter("query_bridge.hipac_online_added",
                                      static_cast<double>(added));
    task_diag.set_value("hipac_online_added", static_cast<double>(added));
    const QueryResult probe_after_hipac = query(task.start, task.goal);
    if (query_bridge_result_acceptable(probe_after_hipac,
                                       task.start,
                                       task.goal,
                                       bridge_acceptance)) {
        task.hipac_online_satisfied = true;
        context.diagnostics().add_counter("query_bridge.hipac_online_satisfied");
        task_diag.set_value("hipac_online_satisfied", 1.0);
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
    const QueryBridgeTaskDiagnostics task_diag(context, task.index);
    const auto prebridge_t0 = QueryBridgeHipacClock::now();
    context.diagnostics().add_counter("query_bridge.hipac_prebridge_attempts");
    task_diag.set_value("hipac_prebridge_attempt", 1.0);
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
    task_diag.set_value("hipac_prebridge_score", prebridge_selection.score);
    task_diag.set_value("hipac_prebridge_pair_distance",
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
    task_diag.set_value("hipac_prebridge_ms", prebridge_ms);
    if (added <= 0) {
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_failures");
        return 0;
    }
    context.diagnostics().add_counter("query_bridge.hipac_prebridge_added",
                                      static_cast<double>(added));
    task_diag.set_value("hipac_prebridge_added", static_cast<double>(added));
    const QueryResult probe_after_prebridge = query(task.start, task.goal);
    if (query_bridge_result_acceptable(probe_after_prebridge,
                                       task.start,
                                       task.goal,
                                       bridge_acceptance)) {
        task.hipac_online_satisfied = true;
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_satisfied");
        task_diag.set_value("hipac_prebridge_satisfied", 1.0);
    } else {
        context.diagnostics().add_counter(
            "query_bridge.hipac_prebridge_not_sufficient");
    }
    return added;
}

int RBFPlanningForest::try_promote_query_repair_to_hipac(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const std::vector<Eigen::VectorXd>& waypoint_path,
    int bridge_added,
    int query_index,
    int batch_task_index,
    StageContext& context) {
    if (!last_adaptive_partition_config_.hipac_online_connectivity ||
        !last_adaptive_partition_config_.hipac_promote_query_repairs ||
        !partition_native_mode() ||
        bridge_added <= 0 ||
        waypoint_path.size() < 2) {
        return 0;
    }
    const auto promote_t0 = QueryBridgeHipacClock::now();
    const QueryBridgeTaskDiagnostics task_diag(context, batch_task_index);
    context.diagnostics().add_counter("query_bridge.hipac_promote_attempts");
    const int promoted = add_partition_portal_corridor_overlay(start,
                                                               goal,
                                                               waypoint_path,
                                                               "query_bridge.hipac_promote",
                                                               true,
                                                               false,
                                                               query_index,
                                                               &last_build_);
    const double promote_ms = query_bridge_hipac_elapsed_ms_since(promote_t0);
    context.diagnostics().record_timing("query_bridge.hipac_promote_ms_total",
                                        promote_ms);
    context.diagnostics().add_counter("query_bridge.hipac_promote_ms_total",
                                      promote_ms);
    task_diag.set_value("hipac_promote_ms", promote_ms);
    if (promoted > 0) {
        context.diagnostics().add_counter("query_bridge.hipac_promote_added",
                                          static_cast<double>(promoted));
        task_diag.set_value("hipac_promote_added", static_cast<double>(promoted));
    } else {
        context.diagnostics().add_counter("query_bridge.hipac_promote_failures");
    }
    return promoted;
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
