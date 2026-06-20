#include <SBF/safe_box_forest.h>

#include "planning_forest_query_bridge_attempt_paths.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <Eigen/Core>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

namespace rbf {

namespace {

using QueryBridgeBatchParallelClock = std::chrono::steady_clock;

double query_bridge_batch_parallel_elapsed_ms_since(
    QueryBridgeBatchParallelClock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
        QueryBridgeBatchParallelClock::now() - t0).count();
}

struct QueryBridgePreparedParallelTask {
    bool skipped = false;
    bool forced = false;
    int attempts = 1;
    double task_start_ms = 0.0;
};

}  // namespace

void RBFPlanningForest::run_query_bridge_batch_parallel_rrt(
    std::vector<QueryBridgeSearchTask>& tasks,
    std::vector<int>& added_by_query,
    const std::unordered_set<int>& forced_query_indices,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeParallelRrtOptions& parallel_rrt_options,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const QueryBridgeEdgeRuntimeOptions& edge_options,
    bool scene_reusable_edges,
    StageContext& batch_context,
    std::chrono::steady_clock::time_point batch_t0) {
    std::vector<QueryBridgePreparedParallelTask> prepared(tasks.size());
    std::vector<QueryBridgeSearchJob> jobs;
    for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
        auto& task = tasks[task_offset];
        prepared[task_offset].task_start_ms =
            query_bridge_batch_parallel_elapsed_ms_since(batch_t0);
        const auto probe_t0 = QueryBridgeBatchParallelClock::now();
        if (query_bridge_task_has_explicit_satisfaction(task) ||
            query_bridge_current_query_good(*this,
                                            task,
                                            forced_query_indices,
                                            bridge_acceptance)) {
            prepared[task_offset].skipped = true;
            record_query_bridge_batch_task_already_satisfied(
                batch_context,
                task,
                query_bridge_batch_parallel_elapsed_ms_since(probe_t0));
            continue;
        }
        batch_context.diagnostics().record_timing(
            "query_bridge.batch_probe_ms_total",
            query_bridge_batch_parallel_elapsed_ms_since(probe_t0));
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
        const QueryBridgeAttemptPlan attempt_plan =
            query_bridge_prepare_attempt_plan(task,
                                              forced_query_indices,
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
    const auto rrt_t0 = QueryBridgeBatchParallelClock::now();
    if (batch_context.executor().n_threads() > 1 && jobs.size() > 1) {
        std::shared_ptr<std::atomic<bool>> local_cancel =
            query_bridge_parallel_rrt_cancel_flag(
                parallel_rrt_options,
                batch_context.native_cancel_flag());
        std::atomic<int> early_successes{0};
        batch_context.executor().parallel_for(
            0,
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
    const double rrt_ms = query_bridge_batch_parallel_elapsed_ms_since(rrt_t0);
    batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                              rrt_ms);
    batch_context.diagnostics().set_value("query_bridge.parallel_task_rrt_jobs",
                                          static_cast<double>(jobs.size()));

    for (std::size_t task_offset = 0; task_offset < tasks.size(); ++task_offset) {
        auto& task = tasks[task_offset];
        const QueryBridgeTaskDiagnostics task_diag(batch_context, task.index);
        if (prepared[task_offset].skipped) {
            task_diag.set_value(
                "total_ms",
                query_bridge_batch_parallel_elapsed_ms_since(batch_t0) -
                prepared[task_offset].task_start_ms);
            continue;
        }
        task_diag.set_value("rrt_ms", rrt_ms);
        double best_length = std::numeric_limits<double>::infinity();
        adopt_query_bridge_waypoint_after_rrt(task,
                                              attempt_paths[task_offset],
                                              best_length,
                                              hybrid_options,
                                              audit_robot_,
                                              scene_,
                                              config_,
                                              batch_context);
        if (task.waypoint_path.empty()) {
            record_query_bridge_batch_task_no_path(
                batch_context,
                task.index,
                query_bridge_batch_parallel_elapsed_ms_since(batch_t0) -
                prepared[task_offset].task_start_ms);
            continue;
        }
        finish_query_bridge_ready_waypoint_task(
            task,
            added_by_query[task.index],
            prepared[task_offset].forced,
            best_length,
            batch_context,
            scene_reusable_edges,
            forced_query_indices,
            bridge_acceptance,
            edge_options.fast_direct_segment_after_rrt_enabled,
            edge_options.fast_direct_random_shortcut_iters,
            [&]() {
                return query_bridge_batch_parallel_elapsed_ms_since(batch_t0) -
                       prepared[task_offset].task_start_ms;
            });
    }
}

}  // namespace rbf
