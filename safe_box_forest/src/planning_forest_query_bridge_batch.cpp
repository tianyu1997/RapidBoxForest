#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
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

void adopt_query_bridge_waypoint_after_rrt(
    QueryBridgeSearchTask& task,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths_for_task,
    int improve_attempts,
    double& best_length,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeDirectLineFallbackOptions& direct_line_options,
    const QueryBridgeDetourOptions& detour_options,
    const QueryBridgeWaypointQualityRetryOptions& quality_retry_options,
    const std::vector<Interval>& detour_planning_domain,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context) {
    select_query_bridge_attempt_paths(task,
                                      attempt_paths_for_task,
                                      best_length,
                                      hybrid_options,
                                      retry_options,
                                      audit_robot,
                                      scene,
                                      config,
                                      context);
    if (task.waypoint_path.empty()) {
        auto direct_path = query_bridge_direct_line_fallback_path(
            task,
            audit_robot,
            scene,
            config.query,
            direct_line_options,
            context);
        if (!direct_path.empty()) {
            best_length = path_length(direct_path);
            task.waypoint_path = std::move(direct_path);
            context.diagnostics().set_value(
                query_bridge_task_key(task.index, "direct_line_on_no_path"),
                1.0);
        }
    }
    if (query_bridge_maybe_apply_detour_path(task,
                                             audit_robot,
                                             scene,
                                             config.query,
                                             detour_planning_domain,
                                             detour_options,
                                             config.grower.rng_seed,
                                             context,
                                             best_length,
                                             task.waypoint_path)) {
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "detour_on_no_path"),
            1.0);
    }
    improve_query_bridge_waypoint_if_needed(task,
                                            improve_attempts,
                                            best_length,
                                            task.waypoint_path,
                                            quality_retry_options,
                                            retry_options,
                                            audit_robot,
                                            scene,
                                            config,
                                            context);
}

}  // namespace

std::vector<int> RBFPlanningForest::finish_query_bridge_batch_result(
    const std::vector<int>& added_by_query,
    std::size_t partition_refresh_base,
    std::size_t segment_edges_before_partition_refresh,
    bool oracle_counters_before_valid,
    const OracleCounters& oracle_counters_before) {
    if (oracle_counters_before_valid && oracle_) {
        const auto after = oracle_->counters();
        add_query_bridge_oracle_counter_delta(last_build_,
                                             oracle_counters_before,
                                             after);
    }
    const bool changed =
        boxes_.size() != partition_refresh_base ||
        segment_edges_.size() != segment_edges_before_partition_refresh ||
        std::any_of(added_by_query.begin(),
                    added_by_query.end(),
                    [](int added) { return added > 0; });
    if (boxes_.size() > partition_refresh_base) {
        append_adaptive_partition_boxes(partition_refresh_base,
                                        &last_build_,
                                        "query_bridge.batch");
    } else if (changed) {
        sync_adaptive_partition_segment_edges(&last_build_, "query_bridge.batch");
        refresh_adaptive_partition_diagnostics(&last_build_);
    }
    return added_by_query;
}

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
        return finish_query_bridge_batch_result(
            added_by_query,
            partition_refresh_base,
            segment_edges_before_partition_refresh,
            oracle_counters_before_valid,
            oracle_counters_before);
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
    batch_context.diagnostics().set_value("query_bridge.batch_tasks_initial",
                                          static_cast<double>(tasks.size()));
    run_query_bridge_direct_start_goal_segments(tasks,
                                                added_by_query,
                                                batch_context,
                                                scene_reusable_edges,
                                                direct_start_goal_segment);
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
            run_query_bridge_hipac_online_sequence_task(task,
                                                        added_by_query[task.index],
                                                        batch_context,
                                                        scene_reusable_edges,
                                                        bridge_acceptance);
        }
    }

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
            adopt_query_bridge_waypoint_after_rrt(task,
                                                  attempt_paths[task_offset],
                                                  prepared[task_offset].attempts,
                                                  best_length,
                                                  hybrid_options,
                                                  retry_options,
                                                  direct_line_options,
                                                  detour_options,
                                                  quality_retry_options,
                                                  detour_planning_domain,
                                                  audit_robot_,
                                                  scene_,
                                                  config_,
                                                  batch_context);
            if (task.waypoint_path.empty()) {
                record_query_bridge_batch_task_no_path(
                    batch_context,
                    task.index,
                    query_bridge_elapsed_ms_since(batch_t0) -
                    prepared[task_offset].task_start_ms);
                continue;
            }
            finish_query_bridge_ready_waypoint_task(
                task,
                added_by_query[task.index],
                prepared[task_offset].forced,
                false,
                best_length,
                batch_context,
                scene_reusable_edges,
                index_options,
                retry_options,
                bridge_acceptance,
                batch_execution_options,
                fast_direct_segment_after_rrt,
                edge_options.fast_direct_shortcut,
                edge_options.fast_direct_random_shortcut_iters,
                fast_direct_segment_after_rrt_min_length,
                [&]() {
                    return query_bridge_elapsed_ms_since(batch_t0) -
                           prepared[task_offset].task_start_ms;
                });
        }

        batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                              query_bridge_elapsed_ms_since(batch_t0));
        return finish_query_bridge_batch_result(
            added_by_query,
            partition_refresh_base,
            segment_edges_before_partition_refresh,
            oracle_counters_before_valid,
            oracle_counters_before);
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
        run_query_bridge_task_attempts(task,
                                       attempt_plan.effective_attempts,
                                       attempt_paths,
                                       retry_options,
                                       parallel_rrt_options,
                                       audit_robot_,
                                       scene_,
                                       config_,
                                       batch_context);
        const double rrt_ms = query_bridge_elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        batch_context.diagnostics().set_value(query_bridge_task_key(task.index, "rrt_ms"),
                                              rrt_ms);
        double best_length = std::numeric_limits<double>::infinity();
        if (task.waypoint_path_from_partition_query && !task.waypoint_path.empty()) {
            best_length = path_length(task.waypoint_path);
        }
        adopt_query_bridge_waypoint_after_rrt(task,
                                              attempt_paths,
                                              attempt_plan.base_attempts,
                                              best_length,
                                              hybrid_options,
                                              retry_options,
                                              direct_line_options,
                                              detour_options,
                                              quality_retry_options,
                                              detour_planning_domain,
                                              audit_robot_,
                                              scene_,
                                              config_,
                                              batch_context);
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
        finish_query_bridge_ready_waypoint_task(
            task,
            added_by_query[task.index],
            attempt_plan.forced,
            segment_only_task,
            best_length,
            batch_context,
            scene_reusable_edges,
            index_options,
            retry_options,
            bridge_acceptance,
            batch_execution_options,
            fast_direct_segment_after_rrt,
            edge_options.fast_direct_shortcut,
            edge_options.fast_direct_random_shortcut_iters,
            fast_direct_segment_after_rrt_min_length,
            [&]() { return query_bridge_elapsed_ms_since(task_t0); });
    }

    batch_context.diagnostics().set_value("query_bridge.batch_total_ms",
                                          query_bridge_elapsed_ms_since(batch_t0));
    return finish_query_bridge_batch_result(
        added_by_query,
        partition_refresh_base,
        segment_edges_before_partition_refresh,
        oracle_counters_before_valid,
        oracle_counters_before);
}

} // namespace rbf
