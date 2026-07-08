#include "planning_forest_query_bridge_rrt_utils.h"

#include <SBF/runtime.h>

#include <SBF/box_graph.h>

#include "../planning_core/planning_forest_audit.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_task.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <SBF/connector.h>
#include <SBF/planning_config.h>
#include <SBF/scene.h>

#include <algorithm>
#include <chrono>

namespace rbf {

std::vector<Eigen::VectorXd> run_query_bridge_task_rrt_attempt(
    const QueryBridgeSearchTask& task,
    int attempt,
    int override_fixed_iters,
    const QueryBridgeRetryOptions& retry_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context,
    std::shared_ptr<std::atomic<bool>> cancel_override) {
    const int scheduled_attempt = attempt + retry_options.attempt_offset;
    CollisionChecker checker = make_audit_checker(audit_robot, scene, config.query);
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
        audit_robot,
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

void run_query_bridge_task_attempts(
    QueryBridgeSearchTask& task,
    int effective_attempts,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeParallelRrtOptions& parallel_rrt_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context) {
    if (context.executor().n_threads() > 1 && effective_attempts > 1) {
        std::shared_ptr<std::atomic<bool>> local_cancel =
            query_bridge_parallel_rrt_cancel_flag(
                parallel_rrt_options,
                context.native_cancel_flag());
        std::atomic<int> early_successes{0};
        context.executor().parallel_for(0, effective_attempts, [&](int attempt) {
            if (query_bridge_parallel_rrt_cancelled(local_cancel)) {
                return;
            }
            auto path = run_query_bridge_task_rrt_attempt(task,
                                                          attempt,
                                                          0,
                                                          retry_options,
                                                          audit_robot,
                                                          scene,
                                                          config,
                                                          context,
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
        record_query_bridge_parallel_rrt_early_stop(context,
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
                                              audit_robot,
                                              scene,
                                              config,
                                              context);
    }
}

void query_bridge_adopt_retry_path_if_better(
    QueryBridgeSearchTask& task,
    std::vector<Eigen::VectorXd> retry_path,
    double& best_length,
    int& retry_successes) {
    if (retry_path.empty()) {
        return;
    }
    retry_successes += 1;
    const double length = path_length(retry_path);
    if (length < best_length) {
        best_length = length;
        task.waypoint_path = std::move(retry_path);
    }
}

void query_bridge_run_no_path_retries(
    QueryBridgeSearchTask& task,
    int first_attempt,
    double& best_length,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeRetryPathRunner& run_task_attempt,
    StageContext& context) {
    if (!task.waypoint_path.empty() ||
        (retry_options.no_path_retry_attempts <= 0 &&
         retry_options.no_path_retry_budget_stages == 0)) {
        return;
    }
    int retry_attempt_offset = first_attempt;
    int retry_attempts_total = 0;
    int retry_successes_total = 0;
    double retry_ms_total = 0.0;
    const QueryBridgeTaskDiagnostics task_diag(context, task.index);
    auto run_stage = [&](int stage_index,
                         int stage_attempts,
                         int stage_fixed_iters,
                         bool adaptive_stage) {
        const int effective_stage_attempts = std::max(0, stage_attempts);
        if (effective_stage_attempts == 0 || !task.waypoint_path.empty()) {
            return;
        }
        const auto retry_t0 = std::chrono::steady_clock::now();
        int retry_successes = 0;
        int retry_attempts_run = 0;
        for (int retry = 0; retry < effective_stage_attempts; ++retry) {
            query_bridge_adopt_retry_path_if_better(
                task,
                run_task_attempt(retry_attempt_offset + retry, stage_fixed_iters),
                best_length,
                retry_successes);
            retry_attempts_run += 1;
            if (retry_successes > 0 &&
                retry_options.no_path_retry_stop_on_first_success) {
                break;
            }
        }
        retry_attempt_offset += effective_stage_attempts;
        retry_attempts_total += retry_attempts_run;
        retry_successes_total += retry_successes;
        const double retry_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - retry_t0)
                .count();
        retry_ms_total += retry_ms;
        const std::string suffix_prefix =
            stage_index == 0
                ? "no_path_retry_"
                : "no_path_retry_stage." + std::to_string(stage_index) + ".";
        task_diag.set_value(suffix_prefix + "attempts",
                            static_cast<double>(effective_stage_attempts));
        task_diag.set_value(suffix_prefix + "attempts_run",
                            static_cast<double>(retry_attempts_run));
        task_diag.set_value(suffix_prefix + "successes",
                            static_cast<double>(retry_successes));
        task_diag.set_value(suffix_prefix + "fixed_iters",
                            static_cast<double>(stage_fixed_iters));
        task_diag.set_value(suffix_prefix + "ms", retry_ms);
        if (adaptive_stage) {
            context.diagnostics().add_counter(
                "query_bridge.batch_no_path_retry_adaptive_attempts",
                static_cast<double>(retry_attempts_run));
            context.diagnostics().add_counter(
                "query_bridge.batch_no_path_retry_adaptive_successes",
                static_cast<double>(retry_successes));
            context.diagnostics().record_timing(
                "query_bridge.batch_no_path_retry_adaptive_ms_total",
                retry_ms);
        }
    };
    run_stage(0, retry_options.no_path_retry_attempts, 0, false);
    for (std::size_t stage = 0;
         task.waypoint_path.empty() && stage < retry_options.no_path_retry_budget_stages;
         ++stage) {
        run_stage(static_cast<int>(stage) + 1,
                  retry_options.no_path_retry_budget_attempts[stage],
                  retry_options.no_path_retry_budget_iters[stage],
                  true);
    }
    context.diagnostics().record_timing(
        "query_bridge.batch_no_path_retry_ms_total",
        retry_ms_total);
    context.diagnostics().add_counter(
        "query_bridge.batch_no_path_retry_attempts",
        static_cast<double>(retry_attempts_total));
    context.diagnostics().add_counter(
        "query_bridge.batch_no_path_retry_successes",
        static_cast<double>(retry_successes_total));
    task_diag.set_value("no_path_retry_attempts",
                        static_cast<double>(retry_attempts_total));
    task_diag.set_value("no_path_retry_ms", retry_ms_total);
    task_diag.set_value("no_path_retry_successes",
                        static_cast<double>(retry_successes_total));
}

}  // namespace rbf
