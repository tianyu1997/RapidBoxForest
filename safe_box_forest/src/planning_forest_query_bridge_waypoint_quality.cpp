#include "planning_forest_query_bridge_batch_utils.h"

#include "env_config.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <utility>

namespace rbf {

QueryBridgeWaypointQualityRetryOptions query_bridge_waypoint_quality_retry_options_from_env() {
    QueryBridgeWaypointQualityRetryOptions options;
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY", 0) != 0;
    options.attempts = std::max(
        0,
        detail::env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY_ATTEMPTS", 4));
    options.iters = std::max(
        0,
        detail::env_int_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_RETRY_ITERS", 0));
    options.max_ratio = std::max(
        1.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_RATIO", 2.0));
    options.max_additive = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_WAYPOINT_QUALITY_MAX_ADDITIVE", 0.75));
    return options;
}

void record_query_bridge_waypoint_quality_retry_diagnostics(
    StageContext& context,
    const QueryBridgeWaypointQualityRetryOptions& options) {
    context.diagnostics().set_value("query_bridge.waypoint_quality_retry",
                                    options.enabled ? 1.0 : 0.0);
}

bool query_bridge_waypoint_quality_retry_needed(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    double best_length,
    const QueryBridgeWaypointQualityRetryOptions& options) {
    const double direct = (goal - start).norm();
    const double limit = std::max(direct * options.max_ratio,
                                  direct + options.max_additive);
    return best_length > limit;
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
    const auto retry_t0 = std::chrono::steady_clock::now();
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
    const double retry_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - retry_t0)
                                .count();
    context.diagnostics().record_timing(
        "query_bridge.waypoint_quality_retry_ms_total",
        retry_ms);
}

}  // namespace rbf
