#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>
#include <SBF/connector.h>

#include "planning_forest_audit.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_query_bridge_attempt_paths.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_detour_utils.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"
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
