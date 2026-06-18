#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_bridge_batch_utils.h"
#include "planning_forest_query_bridge_corridor_utils.h"
#include "planning_forest_query_utils.h"
#include "virtual_sparse_ffb.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rbf {

namespace {

using QueryBridgeClock = std::chrono::steady_clock;

double query_bridge_elapsed_ms_since(QueryBridgeClock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
        QueryBridgeClock::now() - t0).count();
}

bool query_bridge_current_query_good(
    const RBFPlanningForest& forest,
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance) {
    if (!query_bridge_should_check_current_query(task,
                                                 respect_forced,
                                                 index_options,
                                                 retry_options)) {
        return false;
    }
    return query_bridge_result_acceptable(forest.query(task.start, task.goal),
                                          task.start,
                                          task.goal,
                                          bridge_acceptance);
}

std::vector<Eigen::VectorXd> run_query_bridge_task_rrt_attempt(
    const QueryBridgeSearchTask& task,
    int attempt,
    int override_fixed_iters,
    const QueryBridgeRetryOptions& retry_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context,
    std::shared_ptr<std::atomic<bool>> cancel_override =
        std::shared_ptr<std::atomic<bool>>{}) {
    const int scheduled_attempt = attempt + retry_options.attempt_offset;
    Robot bridge_robot = make_sbf_clearance_robot(audit_robot,
                                                  retry_options.rrt_clearance);
    CollisionChecker checker =
        retry_options.rrt_clearance > 0.0
            ? CollisionChecker(bridge_robot, scene)
            : make_audit_checker(audit_robot, scene, config.query);
    RRTConnectConfig rrt_config =
        query_bridge_rrt_config_for_attempt(task,
                                            attempt,
                                            scheduled_attempt,
                                            override_fixed_iters,
                                            config.connector.per_pair_timeout_ms,
                                            retry_options);
    std::vector<Eigen::VectorXd> path = rrt_connect(
        task.start,
        task.goal,
        checker,
        bridge_robot,
        rrt_config,
        query_bridge_rrt_seed_for_attempt(task,
                                          config.grower.rng_seed,
                                          scheduled_attempt),
        cancel_override ? cancel_override : context.native_cancel_flag());
    if (path.empty()) {
        return {};
    }
    const PathAuditCheck audit =
        audit_waypoint_path(path,
                            checker,
                            config.query.audit_resolution,
                            config.query.audit_segment_step);
    if (!audit.passed) {
        return {};
    }
    return path;
}

void improve_query_bridge_waypoint_if_needed(
    QueryBridgeSearchTask& task,
    int attempts_already_used,
    double& best_length,
    std::vector<Eigen::VectorXd>& waypoint_path,
    const QueryBridgeWaypointQualityRetryOptions& quality_retry_options,
    const QueryBridgeRetryOptions& retry_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context) {
    if (!quality_retry_options.enabled ||
        quality_retry_options.attempts <= 0 ||
        waypoint_path.empty()) {
        return;
    }
    if (!query_bridge_waypoint_quality_retry_needed(task.start,
                                                    task.goal,
                                                    best_length,
                                                    quality_retry_options)) {
        return;
    }
    const double direct = (task.goal - task.start).norm();
    const double limit = std::max(direct * quality_retry_options.max_ratio,
                                  direct + quality_retry_options.max_additive);
    context.diagnostics().add_counter(
        "query_bridge.waypoint_quality_retry_tasks");
    const auto retry_t0 = QueryBridgeClock::now();
    int retry_successes = 0;
    std::vector<std::vector<Eigen::VectorXd>> retry_paths(
        static_cast<std::size_t>(quality_retry_options.attempts));
    if (context.executor().n_threads() > 1 &&
        quality_retry_options.attempts > 1) {
        context.executor().parallel_for(
            0,
            quality_retry_options.attempts,
            [&](int retry) {
                retry_paths[static_cast<std::size_t>(retry)] =
                    run_query_bridge_task_rrt_attempt(task,
                                                      attempts_already_used + retry,
                                                      quality_retry_options.iters,
                                                      retry_options,
                                                      audit_robot,
                                                      scene,
                                                      config,
                                                      context);
            });
    } else {
        for (int retry = 0; retry < quality_retry_options.attempts; ++retry) {
            retry_paths[static_cast<std::size_t>(retry)] =
                run_query_bridge_task_rrt_attempt(task,
                                                  attempts_already_used + retry,
                                                  quality_retry_options.iters,
                                                  retry_options,
                                                  audit_robot,
                                                  scene,
                                                  config,
                                                  context);
        }
    }
    for (auto& retry_path : retry_paths) {
        if (retry_path.empty()) {
            continue;
        }
        retry_successes += 1;
        const double length = path_length(retry_path);
        if (length < best_length) {
            if (!waypoint_path.empty() &&
                task.waypoint_fallback_paths.size() < 4) {
                task.waypoint_fallback_paths.push_back(waypoint_path);
            }
            best_length = length;
            waypoint_path = std::move(retry_path);
        }
        if (best_length <= limit) {
            break;
        }
    }
    if (context.executor().n_threads() > 1 &&
        quality_retry_options.attempts > 1) {
        context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_parallel_batches");
    }
    context.diagnostics().add_counter(
        "query_bridge.waypoint_quality_retry_attempts",
        static_cast<double>(quality_retry_options.attempts));
    context.diagnostics().add_counter(
        "query_bridge.waypoint_quality_retry_successes",
        static_cast<double>(retry_successes));
    if (best_length <= limit) {
        context.diagnostics().add_counter(
            "query_bridge.waypoint_quality_retry_fixed");
    }
    context.diagnostics().record_timing(
        "query_bridge.waypoint_quality_retry_ms_total",
        query_bridge_elapsed_ms_since(retry_t0));
}

void select_query_bridge_attempt_paths(
    QueryBridgeSearchTask& task,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths,
    double& best_length,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const QueryBridgeRetryOptions& retry_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context) {
    std::vector<std::pair<double, std::size_t>> valid_paths;
    valid_paths.reserve(attempt_paths.size());
    for (std::size_t index = 0; index < attempt_paths.size(); ++index) {
        if (attempt_paths[index].empty()) {
            continue;
        }
        const double length = path_length(attempt_paths[index]);
        if (!std::isfinite(length)) {
            continue;
        }
        valid_paths.emplace_back(length, index);
    }
    if (valid_paths.empty()) {
        return;
    }
    if (hybrid_options.enabled && valid_paths.size() >= 2U) {
        CollisionChecker checker = make_audit_checker(audit_robot, scene, config.query);
        const double best_input_length =
            std::min(best_length,
                     std::min_element(valid_paths.begin(),
                                      valid_paths.end(),
                                      [](const auto& lhs, const auto& rhs) {
                                          return lhs.first < rhs.first;
                                      })
                         ->first);
        std::vector<Eigen::VectorXd> hybrid =
            hybridize_collision_free_paths(attempt_paths,
                                           checker,
                                           collision_shortcut_resolution(config.query),
                                           hybrid_options.max_paths,
                                           hybrid_options.max_vertices,
                                           hybrid_options.max_cross_checks);
        context.diagnostics().add_counter(
            "query_bridge.hybridize_attempt_paths_tasks");
        if (!hybrid.empty()) {
            const double hybrid_length = path_length(hybrid);
            context.diagnostics().add_counter(
                "query_bridge.hybridize_attempt_paths_candidates");
            context.diagnostics().add_counter(
                query_bridge_task_key(task.index,
                                      "hybridize_attempt_paths_candidates"));
            if (hybrid_length + 1e-12 < best_input_length) {
                const PathAuditCheck audit =
                    audit_waypoint_path(hybrid,
                                        checker,
                                        config.query.audit_resolution,
                                        config.query.audit_segment_step);
                if (audit.passed) {
                    const std::size_t index = attempt_paths.size();
                    attempt_paths.push_back(std::move(hybrid));
                    valid_paths.emplace_back(hybrid_length, index);
                    context.diagnostics().add_counter(
                        "query_bridge.hybridize_attempt_paths_accepts");
                    context.diagnostics().add_counter(
                        "query_bridge.hybridize_attempt_paths_delta",
                        best_input_length - hybrid_length);
                    context.diagnostics().add_counter(
                        query_bridge_task_key(task.index,
                                              "hybridize_attempt_paths_accepts"));
                } else {
                    context.diagnostics().add_counter(
                        "query_bridge.hybridize_attempt_paths_audit_rejects");
                }
            }
        }
    }
    std::sort(valid_paths.begin(), valid_paths.end(),
              [](const auto& lhs, const auto& rhs) {
                  if (std::abs(lhs.first - rhs.first) > 1e-12) {
                      return lhs.first < rhs.first;
                  }
                  return lhs.second < rhs.second;
              });
    std::size_t selected_index = std::numeric_limits<std::size_t>::max();
    if (task.waypoint_path.empty() || valid_paths.front().first < best_length) {
        selected_index = valid_paths.front().second;
        if (!task.waypoint_path.empty() &&
            retry_options.attempt_fallback_paths > 0 &&
            task.waypoint_fallback_paths.size() <
                static_cast<std::size_t>(retry_options.attempt_fallback_paths)) {
            task.waypoint_fallback_paths.push_back(std::move(task.waypoint_path));
            context.diagnostics().add_counter(
                "query_bridge.attempt_fallback_paths_stored");
        }
        best_length = valid_paths.front().first;
        task.waypoint_path = std::move(attempt_paths[selected_index]);
    }
    for (const auto& [length, index] : valid_paths) {
        (void)length;
        if (index == selected_index || attempt_paths[index].empty()) {
            continue;
        }
        if (retry_options.attempt_fallback_paths <= 0 ||
            task.waypoint_fallback_paths.size() >=
                static_cast<std::size_t>(retry_options.attempt_fallback_paths)) {
            break;
        }
        task.waypoint_fallback_paths.push_back(std::move(attempt_paths[index]));
        context.diagnostics().add_counter(
            "query_bridge.attempt_fallback_paths_stored");
        context.diagnostics().add_counter(
            query_bridge_task_key(task.index, "attempt_fallback_paths_stored"));
    }
}

}  // namespace

std::vector<int> RBFPlanningForest::bridge_queries(const std::vector<Eigen::VectorXd>& starts,
                                                   const std::vector<Eigen::VectorXd>& goals) {
    if (starts.size() != goals.size()) {
        throw std::invalid_argument("bridge_queries requires starts/goals with matching sizes");
    }
    std::vector<int> added_by_query(starts.size(), 0);
    std::size_t partition_refresh_base = boxes_.size();
    const std::size_t segment_edges_before_partition_refresh = segment_edges_.size();
    OracleCounters oracle_counters_before;
    bool oracle_counters_before_valid = false;
    auto finish_batch_bridge = [&]() {
        if (oracle_counters_before_valid && oracle_) {
            const auto after = oracle_->counters();
            add_query_bridge_oracle_counter_delta(last_build_, oracle_counters_before, after);
        }
        const bool changed = boxes_.size() != partition_refresh_base ||
                             segment_edges_.size() != segment_edges_before_partition_refresh ||
                             std::any_of(added_by_query.begin(),
                                         added_by_query.end(),
                                         [](int added) { return added > 0; });
        if (boxes_.size() > partition_refresh_base) {
            append_adaptive_partition_boxes(partition_refresh_base,
                                            &last_build_,
                                            "query_bridge.batch");
            partition_refresh_base = boxes_.size();
        } else if (changed) {
            sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.batch");
            refresh_adaptive_partition_diagnostics(&last_build_);
        }
        return added_by_query;
    };
    if (starts.empty() || !oracle_) {
        return added_by_query;
    }
    oracle_counters_before = oracle_->counters();
    oracle_counters_before_valid = true;

    const QueryBridgeAcceptanceThresholds bridge_acceptance =
        query_bridge_acceptance_thresholds_from_env();
    const QueryBridgeIndexOptions index_options = query_bridge_index_options_from_env();
    const QueryBridgePartitionPathFirstOptions partition_path_first_options =
        query_bridge_partition_path_first_options_from_env(partition_native_mode());

    std::vector<QueryBridgeSearchTask> tasks;
    tasks.reserve(starts.size());
    for (std::size_t index = 0; index < starts.size(); ++index) {
        if (starts[index].size() != goals[index].size()) {
            throw std::invalid_argument("bridge_queries received a start/goal dimension mismatch");
        }
        const bool forced_task = query_bridge_index_forced(index_options, index);
        QueryResult initial_query;
        bool has_initial_query = false;
        if (!forced_task || partition_path_first_options.enabled) {
            initial_query = query(starts[index], goals[index]);
            has_initial_query = true;
            if (!forced_task &&
                query_bridge_result_acceptable(initial_query,
                                               starts[index],
                                               goals[index],
                                               bridge_acceptance)) {
                query_bridge_mark_task_skip(last_build_, index, 1.0, "initial_good");
                continue;
            }
        }
        int start_box_id = locate_query_bridge_box(starts[index]);
        if (start_box_id < 0) {
            start_box_id = anchor_query_endpoint_box_with_diagnostics(starts[index]);
        }
        if (start_box_id < 0) {
            query_bridge_mark_task_skip(last_build_, index, 2.0, "start_anchor_failed");
            continue;
        }
        int goal_box_id = locate_query_bridge_box(goals[index]);
        if (goal_box_id < 0) {
            goal_box_id = anchor_query_endpoint_box_with_diagnostics(goals[index]);
        }
        sync_query_bridge_partition_boxes(partition_refresh_base,
                                          "query_bridge.endpoint_anchor");
        if (start_box_id >= 0) {
            start_box_id = refresh_query_bridge_box_or_anchor(start_box_id,
                                                              starts[index],
                                                              "start");
        }
        if (goal_box_id >= 0) {
            goal_box_id = refresh_query_bridge_box_or_anchor(goal_box_id,
                                                             goals[index],
                                                             "goal");
        }
        if (goal_box_id < 0 || goal_box_id == start_box_id) {
            query_bridge_mark_task_skip(last_build_,
                                        index,
                                        goal_box_id < 0 ? 3.0 : 4.0,
                                        goal_box_id < 0 ? "goal_anchor_failed" : "same_box");
            continue;
        }

        QueryBridgeSearchTask task;
        task.index = index;
        task.query_index = query_bridge_index_global(index_options,
                                                     index,
                                                     static_cast<int>(index));
        last_build_.diagnostics["query_bridge.batch_task." +
                                std::to_string(index) +
                                ".global_index"] = static_cast<double>(task.query_index);
        task.start = starts[index];
        task.goal = goals[index];
        if (last_adaptive_partition_config_.hipac_online_connectivity &&
            has_initial_query &&
            initial_query.success &&
            initial_query.audit_passed &&
            !initial_query.path.empty()) {
            task.hipac_candidate_path = initial_query.path;
        }
        task.bridge_rrt = with_query_root_hull_domain(config_.connector.rrt,
                                                      *oracle_,
                                                      task.start,
                                                      task.goal);
        task.bridge_rrt.segment_resolution =
            std::max(task.bridge_rrt.segment_resolution, config_.query.audit_resolution);
        if (partition_path_first_options.enabled &&
            has_initial_query &&
            initial_query.success &&
            initial_query.audit_passed &&
            !initial_query.path.empty()) {
            last_build_.diagnostics["query_bridge.partition_path_first_initial_success"] += 1.0;
            const QueryBridgePartitionInitialPathDecision partition_path_decision =
                query_bridge_partition_initial_path_decision(initial_query,
                                                             task.start,
                                                             task.goal,
                                                             bridge_acceptance,
                                                             partition_path_first_options);
            if (!partition_path_decision.segment_reasonable) {
                last_build_.diagnostics["query_bridge.partition_path_first_reject_segment"] += 1.0;
            }
            if (!partition_path_decision.length_reasonable) {
                last_build_.diagnostics["query_bridge.partition_path_first_reject_length"] += 1.0;
            }
            if (partition_path_decision.accepted) {
                task.waypoint_path = initial_query.path;
                task.waypoint_path_from_partition_query = true;
                if (task.hipac_candidate_path.empty()) {
                    task.hipac_candidate_path = initial_query.path;
                }
                last_build_.diagnostics["query_bridge.partition_path_first_accepted"] += 1.0;
            }
        }
        const double bridge_distance = (task.goal - task.start).norm();
        task.short_local_bridge = query_bridge_short_local_distance(bridge_distance);
        if (task.short_local_bridge) {
            query_bridge_configure_short_local_profiles(task.bridge_rrt,
                                                        task.short_local_profiles);
        }
        task.attempts = std::max(1, config_.connector.max_pairs_per_gap);
        tasks.push_back(std::move(task));
    }

    std::stable_sort(tasks.begin(), tasks.end(), [](const QueryBridgeSearchTask& lhs,
                                                    const QueryBridgeSearchTask& rhs) {
        const bool lhs_short = lhs.short_local_bridge;
        const bool rhs_short = rhs.short_local_bridge;
        if (lhs_short != rhs_short) {
            return !lhs_short && rhs_short;
        }
        return lhs.index < rhs.index;
    });

    if (tasks.empty()) {
        return finish_batch_bridge();
    }
    const auto batch_t0 = QueryBridgeClock::now();
    StageContext batch_context = StageContext::from_runtime(config_.runtime);
    const QueryBridgeEdgeRuntimeOptions edge_options = query_bridge_edge_runtime_options();
    const bool scene_reusable_edges = edge_options.scene_reusable_edges;
    batch_context.diagnostics().set_value("query_bridge.scene_reusable_edges",
                                          scene_reusable_edges ? 1.0 : 0.0);
    ScopedStageDiagnosticsFlush batch_diagnostics_flush(last_build_, batch_context);
    const bool direct_start_goal_segment =
        edge_options.direct_segment_after_rrt &&
        edge_options.direct_start_goal_segment &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    const bool fast_direct_segment_after_rrt =
        edge_options.direct_segment_after_rrt &&
        edge_options.fast_direct_segment_after_rrt &&
        config_.connector.segment_edges_enabled &&
        config_.connector.rrt_segment_edges;
    const double fast_direct_segment_after_rrt_min_length =
        edge_options.direct_segment_after_rrt_min_length;
    batch_context.diagnostics().set_value(
        "query_bridge.direct_segment_after_rrt",
        edge_options.direct_segment_after_rrt ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.direct_segment_after_rrt_min_length",
        edge_options.direct_segment_after_rrt_min_length);
    batch_context.diagnostics().set_value(
        "query_bridge.direct_start_goal_segment",
        direct_start_goal_segment ? 1.0 : 0.0);
    batch_context.diagnostics().set_value(
        "query_bridge.fast_direct_segment_after_rrt",
        fast_direct_segment_after_rrt ? 1.0 : 0.0);
	    auto try_hipac_prebridge_portal = [&](QueryBridgeSearchTask& task) -> int {
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
	        if (coarse_route.size() < 2) {
	            coarse_route = {task.start, task.goal};
	            batch_context.diagnostics().add_counter(
	                "query_bridge.hipac_prebridge_direct_query_route");
	        }
	        if (coarse_route.size() < 2) {
	            return 0;
	        }
	        const auto prebridge_t0 = QueryBridgeClock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_attempt"),
	                                              1.0);
	        const auto candidate_pairs =
	            adaptive_partition_->nearest_component_pairs_to_largest(1,
	                                                                    prebridge_gate.candidate_limit);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_candidates",
	                                                static_cast<double>(candidate_pairs.size()));
	        if (candidate_pairs.empty()) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidates");
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
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_considered",
	                                                static_cast<double>(prebridge_selection.considered));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_distance_rejects",
	                                                static_cast<double>(prebridge_selection.distance_rejects));
	        batch_context.diagnostics().add_counter(
	            "query_bridge.hipac_prebridge_endpoint_component_rejects",
	            static_cast<double>(prebridge_selection.endpoint_component_rejects));
	        if (prebridge_selection.candidate_index < 0 ||
	            prebridge_selection.candidate_index >= static_cast<int>(candidate_pairs.size())) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_no_candidate_after_filter");
	            return 0;
	        }
	        const AdaptiveGridPartitionComponentPair& best_pair =
	            candidate_pairs[static_cast<std::size_t>(prebridge_selection.candidate_index)];

	        task.hipac_prebridge_resolves_used += 1;
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_portal_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_score"),
	                                              prebridge_selection.score);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_pair_distance"),
	                                              std::sqrt(std::max(0.0, best_pair.distance_sq)));
	        std::vector<Eigen::VectorXd> local_path{best_pair.source_point, best_pair.target_point};
	        const int added = add_partition_portal_corridor_overlay(best_pair.source_point,
	                                                                best_pair.target_point,
	                                                                local_path,
	                                                                "query_bridge.hipac_online_prebridge",
	                                                                false,
	                                                                true,
	                                                                query_bridge_edge_query_index(scene_reusable_edges, task),
	                                                                &last_build_);
	        const double prebridge_ms = query_bridge_elapsed_ms_since(prebridge_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_prebridge_ms_total",
	                                                  prebridge_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_ms_total",
	                                                prebridge_ms);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_ms"),
	                                              prebridge_ms);
	        if (added <= 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_failures");
	            return 0;
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_added",
	                                                static_cast<double>(added));
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_added"),
	                                              static_cast<double>(added));
	        added_by_query[task.index] += added;
	        const QueryResult probe_after_prebridge = query(task.start, task.goal);
	        if (query_bridge_result_acceptable(probe_after_prebridge,
	                                           task.start,
	                                           task.goal,
	                                           bridge_acceptance)) {
	            task.hipac_online_satisfied = true;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_satisfied");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_prebridge_satisfied"),
	                                                  1.0);
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_prebridge_not_sufficient");
	        }
	        return added;
	    };
	    auto try_hipac_online_bridge = [&](QueryBridgeSearchTask& task) -> int {
	        const QueryBridgeHipacOnlineGate hipac_online_gate =
	            query_bridge_hipac_online_gate(
	                last_adaptive_partition_config_,
	                partition_native_mode(),
	                static_cast<int>(task.hipac_candidate_path.size()),
	                task.hipac_online_resolves_used);
	        if (!hipac_online_gate.enabled) {
	            return 0;
	        }
	        task.hipac_online_resolves_used += 1;
	        const auto hipac_t0 = QueryBridgeClock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_attempt"),
	                                              1.0);
	        std::vector<Eigen::VectorXd> hipac_path = task.hipac_candidate_path;
	        if (hipac_path.size() > 2) {
	            CollisionChecker checker = make_audit_checker(audit_robot_, scene_, config_.query);
	            const double before_length = path_length(hipac_path);
	            std::vector<Eigen::VectorXd> shortened =
	                collision_shortcut_path(hipac_path,
	                                        checker,
	                                        collision_shortcut_resolution(config_.query));
	            if (shortened.size() >= 2 &&
	                path_length(shortened) <= before_length + 1e-12) {
	                const PathAuditCheck audit =
	                    audit_waypoint_path(shortened,
	                                        checker,
	                                        config_.query.audit_resolution,
	                                        config_.query.audit_segment_step);
	                if (audit.passed) {
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_accepts");
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_delta",
	                        std::max(0.0, before_length - path_length(shortened)));
	                    hipac_path = std::move(shortened);
	                } else {
	                    batch_context.diagnostics().add_counter(
	                        "query_bridge.hipac_online_shortcut_audit_rejects");
	                }
	            }
	        }
	        const double hipac_candidate_length = path_length(hipac_path);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_candidate_length",
	                                                hipac_candidate_length);
	        if (hipac_online_gate.candidate_max_length > 0.0 &&
	            hipac_candidate_length > hipac_online_gate.candidate_max_length + 1e-12) {
	            batch_context.diagnostics().add_counter(
	                "query_bridge.hipac_online_candidate_length_rejects");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_length_reject"),
	                                                  1.0);
	            const double hipac_ms = query_bridge_elapsed_ms_since(hipac_t0);
	            batch_context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
	                                                      hipac_ms);
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
	                                                    hipac_ms);
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_ms"),
	                                                  hipac_ms);
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_failures");
	            return 0;
	        }
	        int added = add_partition_box_corridor_overlay(task.start,
	                                                       task.goal,
	                                                       hipac_path,
	                                                       "query_bridge.hipac_online",
	                                                       true,
	                                                       false,
	                                                       query_bridge_edge_query_index(scene_reusable_edges, task),
	                                                       &last_build_);
	        if (added <= 0 &&
	            last_adaptive_partition_config_.hipac_online_ffb_portal_fallback) {
	            added = add_partition_portal_corridor_overlay(task.start,
	                                                          task.goal,
	                                                          hipac_path,
	                                                          "query_bridge.hipac_online",
	                                                          true,
	                                                          false,
	                                                          query_bridge_edge_query_index(scene_reusable_edges, task),
	                                                          &last_build_);
	        }
	        const double hipac_ms = query_bridge_elapsed_ms_since(hipac_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_online_ms_total",
	                                                  hipac_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_ms_total",
	                                                hipac_ms);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_ms"),
	                                              hipac_ms);
	        if (added <= 0) {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_failures");
	            return 0;
	        }
	        batch_context.diagnostics().add_counter("query_bridge.hipac_online_added",
	                                                static_cast<double>(added));
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_added"),
	                                              static_cast<double>(added));
	        added_by_query[task.index] += added;
	        const QueryResult probe_after_hipac = query(task.start, task.goal);
	        if (query_bridge_result_acceptable(probe_after_hipac,
	                                           task.start,
	                                           task.goal,
	                                           bridge_acceptance)) {
	            task.hipac_online_satisfied = true;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_satisfied");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_online_satisfied"),
	                                                  1.0);
	        } else {
	            batch_context.diagnostics().add_counter("query_bridge.hipac_online_not_sufficient");
	        }
	        return added;
	    };
	    auto try_hipac_transition_portal = [&](QueryBridgeSearchTask& task) -> int {
	        const QueryBridgeHipacTransitionGate transition_gate =
	            query_bridge_hipac_transition_gate(
	                last_adaptive_partition_config_,
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
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_target_rejects");
	            return 0;
	        }

	        const auto transition_t0 = QueryBridgeClock::now();
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_attempts");
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_attempt"),
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
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_candidates",
	                                                static_cast<double>(transition_candidates.candidates.size()));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated",
	                                                static_cast<double>(transition_candidates.gated +
	                                                                    transition_candidates.same_component_gated +
	                                                                    transition_candidates.distance_gated +
	                                                                    transition_candidates.edge_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_same_component",
	                                                static_cast<double>(transition_candidates.same_component_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_distance",
	                                                static_cast<double>(transition_candidates.distance_gated));
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_gated_edges",
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
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_portal_attempts");
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_predicted_edges"),
	                                                  static_cast<double>(candidate.predicted_bridge_edges));
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_pair_distance"),
	                                                  candidate.pair_distance);
	            const int added = add_partition_portal_corridor_overlay(candidate.source_point,
	                                                                    candidate.target_point,
	                                                                    candidate.local_path,
	                                                                    "query_bridge.hipac_online_transition",
	                                                                    false,
	                                                                    false,
	                                                                    query_bridge_edge_query_index(scene_reusable_edges, task),
	                                                                    &last_build_);
	            if (added <= 0) {
	                batch_context.diagnostics().add_counter("query_bridge.hipac_transition_failures");
	                continue;
	            }
	            total_added += added;
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_added",
	                                                    static_cast<double>(added));
	            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_added"),
	                                                  static_cast<double>(added));
	            added_by_query[task.index] += added;
	            const QueryResult probe_after_transition = query(task.start, task.goal);
	            if (probe_after_transition.success &&
	                probe_after_transition.audit_passed &&
	                !probe_after_transition.path.empty()) {
	                task.waypoint_path = probe_after_transition.path;
	                task.hipac_candidate_path = probe_after_transition.path;
	                batch_context.diagnostics().add_counter(
	                    "query_bridge.hipac_transition_probe_path_adopted");
	                batch_context.diagnostics().set_value(
	                    query_bridge_task_key(task.index, "hipac_transition_probe_path_length"),
	                    probe_after_transition.path_length);
	            }
	            if (query_bridge_result_acceptable(probe_after_transition,
	                                               task.start,
	                                               task.goal,
	                                               bridge_acceptance)) {
	                task.hipac_online_satisfied = true;
	                batch_context.diagnostics().add_counter("query_bridge.hipac_transition_satisfied");
	                batch_context.diagnostics().set_value(query_bridge_task_key(task.index,
	                                                               "hipac_transition_satisfied"),
	                                                      1.0);
	                break;
	            }
	            batch_context.diagnostics().add_counter("query_bridge.hipac_transition_not_sufficient");
	        }
	        const double transition_ms = query_bridge_elapsed_ms_since(transition_t0);
	        batch_context.diagnostics().record_timing("query_bridge.hipac_transition_ms_total",
	                                                  transition_ms);
	        batch_context.diagnostics().add_counter("query_bridge.hipac_transition_ms_total",
	                                                transition_ms);
	        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "hipac_transition_ms"),
	                                              transition_ms);
	        return total_added;
	    };
    auto try_hipac_online_sequence = [&](QueryBridgeSearchTask& task) {
        try_hipac_online_bridge(task);
        if (!task.hipac_online_satisfied) {
            try_hipac_transition_portal(task);
        }
        if (!task.hipac_online_satisfied) {
            try_hipac_prebridge_portal(task);
        }
        return task.hipac_online_satisfied;
    };
    batch_context.diagnostics().set_value("query_bridge.batch_tasks_initial",
                                          static_cast<double>(tasks.size()));
    if (direct_start_goal_segment) {
        for (auto& task : tasks) {
            if (task.direct_start_goal_satisfied) {
                continue;
            }
            const int added = try_add_query_direct_start_goal_segment_for_points(
                task.start,
                task.goal,
                batch_context,
                query_bridge_edge_query_index(scene_reusable_edges, task),
                static_cast<int>(task.index));
            task.direct_start_goal_satisfied = added > 0;
            if (added > 0) {
                added_by_query[task.index] += added;
            }
        }
    }
    const QueryBridgeRetryOptions retry_options = query_bridge_retry_options_from_env();
    record_query_bridge_retry_diagnostics(batch_context, retry_options);
    const QueryBridgeBatchExecutionOptions batch_execution_options =
        query_bridge_batch_execution_options_from_env();
    record_query_bridge_batch_execution_diagnostics(batch_context, batch_execution_options);
    const QueryBridgeParallelRrtOptions parallel_rrt_options =
        query_bridge_parallel_rrt_options_from_env();
    record_query_bridge_parallel_rrt_diagnostics(batch_context, parallel_rrt_options);
    record_query_bridge_acceptance_diagnostics(batch_context, bridge_acceptance);
    record_query_bridge_partition_path_first_diagnostics(batch_context,
                                                        partition_path_first_options);
    const QueryBridgeDirectLineFallbackOptions direct_line_options =
        query_bridge_direct_line_fallback_options_from_env();
    record_query_bridge_direct_line_fallback_diagnostics(batch_context, direct_line_options);
    const QueryBridgeDetourOptions detour_options = query_bridge_detour_options_from_env();
    record_query_bridge_detour_diagnostics(batch_context, detour_options);
    const auto detour_planning_domain = oracle_->planning_intervals();
    const QueryBridgeWaypointQualityRetryOptions quality_retry_options =
        query_bridge_waypoint_quality_retry_options_from_env();
    record_query_bridge_waypoint_quality_retry_diagnostics(batch_context,
                                                           quality_retry_options);
    const QueryBridgeHybridizeAttemptOptions hybrid_options =
        query_bridge_hybridize_attempt_options_from_env();
    if (last_adaptive_partition_config_.hipac_online_connectivity &&
        last_adaptive_partition_config_.hipac_online_before_query_bridge) {
        for (auto& task : tasks) {
            try_hipac_online_sequence(task);
        }
    }

    auto bridge_query_with_waypoint_fallbacks =
        [&](QueryBridgeSearchTask& task,
            int& added_accumulator) -> int {
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
            if (candidate_index > 0) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.waypoint_quality_fallback_attempts");
                batch_context.diagnostics().add_counter(
                    query_bridge_task_key(task.index, "waypoint_quality_fallback_attempts"));
            }
            const int bridge_added =
                bridge_query_with_waypoint_path(task.start,
                                                task.goal,
                                                candidate_path,
                                                task.short_local_bridge,
                                                task.bridge_rrt,
                                                task.query_index);
            total_added += bridge_added;
            added_accumulator += bridge_added;
            const int promoted = try_promote_query_repair_to_hipac(
                task.start,
                task.goal,
                task.waypoint_path,
                bridge_added,
                query_bridge_edge_query_index(scene_reusable_edges, task),
                static_cast<int>(task.index),
                batch_context);
            if (promoted > 0) {
                added_by_query[task.index] += promoted;
            }
            accumulate_query_bridge_direct_corridor_totals(last_build_,
                                                           batch_context,
                                                           task.index);
            if (candidate_index > 0) {
                batch_context.diagnostics().add_counter(
                    "query_bridge.waypoint_quality_fallback_added",
                    static_cast<double>(bridge_added));
            }
            if (query_bridge_current_query_good(*this,
                                                task,
                                                false,
                                                index_options,
                                                retry_options,
                                                bridge_acceptance)) {
                if (candidate_index > 0) {
                    batch_context.diagnostics().add_counter(
                        "query_bridge.waypoint_quality_fallback_successes");
                    batch_context.diagnostics().set_value(
                        query_bridge_task_key(task.index, "waypoint_quality_fallback_success"),
                        1.0);
                    task.waypoint_path = candidate_path;
                }
                if (!batch_execution_options.evaluate_all_fallback_paths) {
                    break;
                }
            }
        }
        return total_added;
    };
    auto adopt_waypoint_after_rrt =
        [&](QueryBridgeSearchTask& task,
            std::vector<std::vector<Eigen::VectorXd>>& attempt_paths_for_task,
            int improve_attempts,
            double& best_length) {
            select_query_bridge_attempt_paths(task,
                                              attempt_paths_for_task,
                                              best_length,
                                              hybrid_options,
                                              retry_options,
                                              audit_robot_,
                                              scene_,
                                              config_,
                                              batch_context);
            if (task.waypoint_path.empty()) {
                auto direct_path = query_bridge_direct_line_fallback_path(
                    task,
                    audit_robot_,
                    scene_,
                    config_.query,
                    direct_line_options,
                    batch_context);
                if (!direct_path.empty()) {
                    best_length = path_length(direct_path);
                    task.waypoint_path = std::move(direct_path);
                    batch_context.diagnostics().set_value(
                        query_bridge_task_key(task.index, "direct_line_on_no_path"),
                        1.0);
                }
            }
            if (query_bridge_maybe_apply_detour_path(task,
                                                     audit_robot_,
                                                     scene_,
                                                     config_.query,
                                                     detour_planning_domain,
                                                     detour_options,
                                                     config_.grower.rng_seed,
                                                     batch_context,
                                                     best_length,
                                                     task.waypoint_path)) {
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "detour_on_no_path"),
                    1.0);
            }
            improve_query_bridge_waypoint_if_needed(task,
                                                    improve_attempts,
                                                    best_length,
                                                    task.waypoint_path,
                                                    quality_retry_options,
                                                    retry_options,
                                                    audit_robot_,
                                                    scene_,
                                                    config_,
                                                    batch_context);
        };
    auto finish_ready_waypoint_task =
        [&](QueryBridgeSearchTask& task,
            bool forced_task,
            bool segment_only_task,
            double best_length,
            auto&& task_elapsed_ms) {
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "waypoint_length"),
                best_length);
            const auto second_probe_t0 = QueryBridgeClock::now();
            if (query_bridge_current_query_good(*this,
                                                task,
                                                !retry_options.post_rrt_skip_forced,
                                                index_options,
                                                retry_options,
                                                bridge_acceptance)) {
                record_query_bridge_batch_task_skipped_after_rrt(
                    batch_context,
                    task.index,
                    forced_task,
                    query_bridge_elapsed_ms_since(second_probe_t0),
                    task_elapsed_ms());
                return;
            }
            batch_context.diagnostics().record_timing(
                "query_bridge.batch_probe_ms_total",
                query_bridge_elapsed_ms_since(second_probe_t0));
            if (query_bridge_hipac_after_rrt_available(last_adaptive_partition_config_,
                                                       task)) {
                task.hipac_candidate_path = task.waypoint_path;
                try_hipac_online_sequence(task);
                if (task.hipac_online_satisfied) {
                    record_query_bridge_batch_task_skipped_by_hipac_after_rrt(
                        batch_context,
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
                    batch_context,
                    fast_direct_segment_after_rrt,
                    edge_options.fast_direct_shortcut,
                    edge_options.fast_direct_random_shortcut_iters,
                    fast_direct_segment_after_rrt_min_length,
                    task.query_index,
                    query_bridge_edge_query_index(scene_reusable_edges, task),
                    static_cast<int>(task.index));
            if (fast_direct_added > 0) {
                added_by_query[task.index] += fast_direct_added;
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "fast_direct_segment_after_rrt"),
                    1.0);
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "added"),
                    static_cast<double>(added_by_query[task.index]));
                batch_context.diagnostics().set_value(
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
                        batch_context);
                if (segment_only_added > 0) {
                    added_by_query[task.index] += segment_only_added;
                }
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "total_ms"),
                    task_elapsed_ms());
                return;
            }
            const auto pave_t0 = QueryBridgeClock::now();
            bridge_query_with_waypoint_fallbacks(task, added_by_query[task.index]);
            const double pave_ms = query_bridge_elapsed_ms_since(pave_t0);
            batch_context.diagnostics().record_timing("query_bridge.batch_pave_ms_total",
                                                      pave_ms);
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "pave_ms"),
                                                  pave_ms);
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "added"),
                static_cast<double>(added_by_query[task.index]));
            batch_context.diagnostics().set_value(
                query_bridge_task_key(task.index, "total_ms"),
                task_elapsed_ms());
        };
    auto run_attempts_for_task =
        [&](QueryBridgeSearchTask& task,
            int effective_attempts,
            std::vector<std::vector<Eigen::VectorXd>>& attempt_paths) {
            if (batch_context.executor().n_threads() > 1 && effective_attempts > 1) {
                std::shared_ptr<std::atomic<bool>> local_cancel =
                    query_bridge_parallel_rrt_cancel_flag(
                        parallel_rrt_options,
                        batch_context.native_cancel_flag());
                std::atomic<int> early_successes{0};
                batch_context.executor().parallel_for(0, effective_attempts, [&](int attempt) {
                    if (query_bridge_parallel_rrt_cancelled(local_cancel)) {
                        return;
                    }
                    auto path =
                        run_query_bridge_task_rrt_attempt(task,
                                                          attempt,
                                                          0,
                                                          retry_options,
                                                          audit_robot_,
                                                          scene_,
                                                          config_,
                                                          batch_context,
                                                          local_cancel);
                    query_bridge_maybe_stop_parallel_rrt_after_success(
                        query_bridge_task_rrt_path_good_enough(task,
                                                               path,
                                                               parallel_rrt_options),
                        parallel_rrt_options,
                        early_successes,
                        local_cancel);
                    attempt_paths[static_cast<std::size_t>(attempt)] = std::move(path);
                });
                record_query_bridge_parallel_rrt_early_stop(batch_context,
                                                            parallel_rrt_options,
                                                            local_cancel,
                                                            early_successes);
                return;
            }
            for (int attempt = 0; attempt < effective_attempts; ++attempt) {
                attempt_paths[static_cast<std::size_t>(attempt)] =
                    run_query_bridge_task_rrt_attempt(task,
                                                      attempt,
                                                      0,
                                                      retry_options,
                                                      audit_robot_,
                                                      scene_,
                                                      config_,
                                                      batch_context);
            }
        };

    batch_context.diagnostics().set_value("query_bridge.attempt_offset",
                                          static_cast<double>(retry_options.attempt_offset));
    const bool has_segment_only_task =
        query_bridge_has_segment_only_task(tasks, index_options);
    if (query_bridge_parallel_task_rrt_enabled(batch_execution_options,
                                               has_segment_only_task,
                                               retry_options)) {
        struct PreparedTask {
            bool skipped = false;
            bool forced = false;
            int attempts = 1;
            double task_start_ms = 0.0;
        };
        std::vector<PreparedTask> prepared(tasks.size());
        std::vector<QueryBridgeSearchJob> jobs;
        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            auto& task = tasks[task_offset];
            prepared[task_offset].task_start_ms = query_bridge_elapsed_ms_since(batch_t0);
            const auto probe_t0 = QueryBridgeClock::now();
            if (query_bridge_task_has_explicit_satisfaction(task) ||
                query_bridge_current_query_good(*this,
                                                task,
                                                true,
                                                index_options,
                                                retry_options,
                                                bridge_acceptance)) {
                prepared[task_offset].skipped = true;
                record_query_bridge_batch_task_already_satisfied(
                    batch_context,
                    task,
                    query_bridge_elapsed_ms_since(probe_t0));
                continue;
            }
            batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                      query_bridge_elapsed_ms_since(probe_t0));
            batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
            const QueryBridgeAttemptPlan attempt_plan =
                query_bridge_prepare_attempt_plan(task,
                                                  index_options,
                                                  retry_options,
                                                  batch_context);
            prepared[task_offset].forced = attempt_plan.forced;
            prepared[task_offset].attempts = attempt_plan.effective_attempts;
            for (int attempt = 0; attempt < prepared[task_offset].attempts; ++attempt) {
                jobs.push_back({task_offset, attempt});
            }
        }

        std::vector<std::vector<std::vector<Eigen::VectorXd>>> attempt_paths(tasks.size());
        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            attempt_paths[task_offset].resize(
                static_cast<std::size_t>(std::max(0, prepared[task_offset].attempts)));
        }
        const auto rrt_t0 = QueryBridgeClock::now();
        if (batch_context.executor().n_threads() > 1 && jobs.size() > 1) {
            std::shared_ptr<std::atomic<bool>> local_cancel =
                query_bridge_parallel_rrt_cancel_flag(
                    parallel_rrt_options,
                    batch_context.native_cancel_flag());
            std::atomic<int> early_successes{0};
            batch_context.executor().parallel_for(0,
                                                  static_cast<int>(jobs.size()),
                                                  [&](int job_index) {
                if (query_bridge_parallel_rrt_cancelled(local_cancel)) {
                    return;
                }
                const QueryBridgeSearchJob& job = jobs[static_cast<std::size_t>(job_index)];
                auto path =
                    run_query_bridge_task_rrt_attempt(tasks[job.task_index],
                                                      job.attempt,
                                                      0,
                                                      retry_options,
                                                      audit_robot_,
                                                      scene_,
                                                      config_,
                                                      batch_context,
                                                      local_cancel);
                query_bridge_maybe_stop_parallel_rrt_after_success(
                    query_bridge_task_rrt_path_good_enough(tasks[job.task_index],
                                                           path,
                                                           parallel_rrt_options),
                    parallel_rrt_options,
                    early_successes,
                    local_cancel);
                attempt_paths[job.task_index][static_cast<std::size_t>(job.attempt)] =
                    std::move(path);
            });
            record_query_bridge_parallel_rrt_early_stop(batch_context,
                                                        parallel_rrt_options,
                                                        local_cancel,
                                                        early_successes);
        } else {
            for (const QueryBridgeSearchJob& job : jobs) {
                attempt_paths[job.task_index][static_cast<std::size_t>(job.attempt)] =
                    run_query_bridge_task_rrt_attempt(tasks[job.task_index],
                                                      job.attempt,
                                                      0,
                                                      retry_options,
                                                      audit_robot_,
                                                      scene_,
                                                      config_,
                                                      batch_context);
            }
        }
        const double rrt_ms = query_bridge_elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value("query_bridge.parallel_task_rrt",
                                              1.0);
        batch_context.diagnostics().set_value("query_bridge.parallel_task_rrt_jobs",
                                              static_cast<double>(jobs.size()));

        for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
            auto& task = tasks[task_offset];
            if (prepared[task_offset].skipped) {
                batch_context.diagnostics().set_value(
                    query_bridge_task_key(task.index, "total_ms"),
                    query_bridge_elapsed_ms_since(batch_t0) -
                    prepared[task_offset].task_start_ms);
                continue;
            }
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "rrt_ms"),
                                                  rrt_ms);
            double best_length = std::numeric_limits<double>::infinity();
            if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
                best_length = path_length(task.waypoint_path);
                record_query_bridge_partition_path_first_rrt_skipped(batch_context,
                                                                     task.index);
            }
            adopt_waypoint_after_rrt(task,
                                     attempt_paths[task_offset],
                                     prepared[task_offset].attempts,
                                     best_length);
            if (task.waypoint_path.empty()) {
                record_query_bridge_batch_task_no_path(
                    batch_context,
                    task.index,
                    query_bridge_elapsed_ms_since(batch_t0) -
                    prepared[task_offset].task_start_ms);
                continue;
            }
            finish_ready_waypoint_task(
                task,
                prepared[task_offset].forced,
                false,
                best_length,
                [&]() {
                    return query_bridge_elapsed_ms_since(batch_t0) -
                           prepared[task_offset].task_start_ms;
                });
        }

        batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                              query_bridge_elapsed_ms_since(batch_t0));
        return finish_batch_bridge();
    }

    for (auto& task : tasks) {
        const auto task_t0 = QueryBridgeClock::now();
        const auto probe_t0 = QueryBridgeClock::now();
        if (query_bridge_task_has_explicit_satisfaction(task) ||
            query_bridge_current_query_good(*this,
                                            task,
                                            true,
                                            index_options,
                                            retry_options,
                                            bridge_acceptance)) {
            record_query_bridge_batch_task_already_satisfied(
                batch_context,
                task,
                query_bridge_elapsed_ms_since(probe_t0));
            batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "total_ms"),
                                                  query_bridge_elapsed_ms_since(task_t0));
            continue;
        }
        batch_context.diagnostics().record_timing("query_bridge.batch_probe_ms_total",
                                                  query_bridge_elapsed_ms_since(probe_t0));
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
        const QueryBridgeAttemptPlan attempt_plan =
            query_bridge_prepare_attempt_plan(task,
                                              index_options,
                                              retry_options,
                                              batch_context);
        if (attempt_plan.partition_path_first) {
            record_query_bridge_partition_path_first_rrt_skipped(batch_context,
                                                                 task.index);
        }
        std::vector<std::vector<Eigen::VectorXd>> attempt_paths(
            static_cast<std::size_t>(attempt_plan.effective_attempts));
        const auto rrt_t0 = QueryBridgeClock::now();
        run_attempts_for_task(task, attempt_plan.effective_attempts, attempt_paths);
        const double rrt_ms = query_bridge_elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "rrt_ms"),
                                              rrt_ms);
        double best_length = std::numeric_limits<double>::infinity();
        if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
            best_length = path_length(task.waypoint_path);
        }
        adopt_waypoint_after_rrt(task,
                                 attempt_paths,
                                 attempt_plan.base_attempts,
                                 best_length);
        const bool segment_only_task =
            query_bridge_index_segment_only(index_options, task.index);
        if (segment_only_task) {
            query_bridge_run_segment_only_retry(
                task,
                attempt_plan.base_attempts,
                best_length,
                retry_options,
                [&](int attempt, int fixed_iters) {
                    return run_query_bridge_task_rrt_attempt(task,
                                                             attempt,
                                                             fixed_iters,
                                                             retry_options,
                                                             audit_robot_,
                                                             scene_,
                                                             config_,
                                                             batch_context);
                },
                batch_context);
        } else {
            query_bridge_run_no_path_retries(
                task,
                attempt_plan.base_attempts,
                best_length,
                retry_options,
                [&](int attempt, int fixed_iters) {
                    return run_query_bridge_task_rrt_attempt(task,
                                                             attempt,
                                                             fixed_iters,
                                                             retry_options,
                                                             audit_robot_,
                                                             scene_,
                                                             config_,
                                                             batch_context);
                },
                batch_context);
        }
        if (task.waypoint_path.empty()) {
            record_query_bridge_batch_task_no_path(batch_context,
                                                   task.index,
                                                   query_bridge_elapsed_ms_since(task_t0));
            continue;
        }
        finish_ready_waypoint_task(task,
                                   attempt_plan.forced,
                                   segment_only_task,
                                   best_length,
                                   [&]() { return query_bridge_elapsed_ms_since(task_t0); });
    }

    batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                          query_bridge_elapsed_ms_since(batch_t0));
    return finish_batch_bridge();
}

} // namespace rbf
