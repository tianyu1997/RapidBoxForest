#include <SBF/safe_box_forest.h>

#include "planning_forest_query_bridge_attempt_paths.h"
#include "planning_forest_query_bridge_corridor_options.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <Eigen/Core>

#include <chrono>
#include <limits>
#include <unordered_set>
#include <vector>

namespace rbf {

namespace {

using QueryBridgeBatchSerialClock = std::chrono::steady_clock;

double query_bridge_batch_serial_elapsed_ms_since(
    QueryBridgeBatchSerialClock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
        QueryBridgeBatchSerialClock::now() - t0).count();
}

}  // namespace

void RBFPlanningForest::run_query_bridge_batch_serial_rrt(
    std::vector<QueryBridgeSearchTask>& tasks,
    std::vector<int>& added_by_query,
    const std::unordered_set<int>& forced_query_indices,
    const QueryBridgeAcceptanceThresholds& bridge_acceptance,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeParallelRrtOptions& parallel_rrt_options,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const QueryBridgeEdgeRuntimeOptions& edge_options,
    bool scene_reusable_edges,
    StageContext& batch_context) {
    for (auto& task : tasks) {
        const auto task_t0 = QueryBridgeBatchSerialClock::now();
        const auto probe_t0 = QueryBridgeBatchSerialClock::now();
        if (query_bridge_task_has_explicit_satisfaction(task) ||
            query_bridge_current_query_good(*this,
                                            task,
                                            forced_query_indices,
                                            bridge_acceptance)) {
            record_query_bridge_batch_task_already_satisfied(
                batch_context,
                task,
                query_bridge_batch_serial_elapsed_ms_since(probe_t0));
            const QueryBridgeTaskDiagnostics task_diag(batch_context, task.index);
            task_diag.set_value("total_ms",
                                query_bridge_batch_serial_elapsed_ms_since(task_t0));
            continue;
        }
        batch_context.diagnostics().record_timing(
            "query_bridge.batch_probe_ms_total",
            query_bridge_batch_serial_elapsed_ms_since(probe_t0));
        batch_context.diagnostics().add_counter("query_bridge.batch_tasks_attempted");
        const QueryBridgeAttemptPlan attempt_plan =
            query_bridge_prepare_attempt_plan(task,
                                              forced_query_indices,
                                              retry_options,
                                              batch_context);
        std::vector<std::vector<Eigen::VectorXd>> attempt_paths(
            static_cast<std::size_t>(attempt_plan.effective_attempts));
        const auto rrt_t0 = QueryBridgeBatchSerialClock::now();
        run_query_bridge_task_attempts(task,
                                       attempt_plan.effective_attempts,
                                       attempt_paths,
                                       retry_options,
                                       parallel_rrt_options,
                                       audit_robot_,
                                       scene_,
                                       config_,
                                       batch_context);
        const double rrt_ms = query_bridge_batch_serial_elapsed_ms_since(rrt_t0);
        batch_context.diagnostics().record_timing("query_bridge.batch_rrt_ms_total",
                                                  rrt_ms);
        const QueryBridgeTaskDiagnostics task_diag(batch_context, task.index);
        task_diag.set_value("rrt_ms", rrt_ms);
        double best_length = std::numeric_limits<double>::infinity();
        adopt_query_bridge_waypoint_after_rrt(task,
                                              attempt_paths,
                                              best_length,
                                              hybrid_options,
                                              audit_robot_,
                                              scene_,
                                              config_,
                                              batch_context);
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
        if (task.waypoint_path.empty()) {
            record_query_bridge_batch_task_no_path(
                batch_context,
                task.index,
                query_bridge_batch_serial_elapsed_ms_since(task_t0));
            continue;
        }
        finish_query_bridge_ready_waypoint_task(
            task,
            added_by_query[task.index],
            attempt_plan.forced,
            best_length,
            batch_context,
            scene_reusable_edges,
            forced_query_indices,
            bridge_acceptance,
            edge_options.fast_direct_segment_after_rrt_enabled,
            edge_options.fast_direct_random_shortcut_iters,
            [&]() { return query_bridge_batch_serial_elapsed_ms_since(task_t0); });
    }
}

}  // namespace rbf
