#include "planning_forest_query_bridge_batch_utils.h"

#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_detour_utils.h"
#include "planning_forest_query_utils.h"

#include <SBF/box_graph.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace rbf {

std::string query_bridge_task_key(std::size_t index, const std::string& suffix) {
    return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
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

namespace {

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

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options) {
    if (query_bridge_index_segment_only(index_options, task.index)) {
        return false;
    }
    if (respect_forced && query_bridge_index_forced(index_options, task.index)) {
        return false;
    }
    return retry_options.skip_deferred_short_edges;
}

bool query_bridge_has_segment_only_task(
    const std::vector<QueryBridgeSearchTask>& tasks,
    const QueryBridgeIndexOptions& index_options) {
    return std::any_of(tasks.begin(), tasks.end(), [&](const QueryBridgeSearchTask& task) {
        return query_bridge_index_segment_only(index_options, task.index);
    });
}

bool query_bridge_parallel_task_rrt_enabled(
    const QueryBridgeBatchExecutionOptions& batch_options,
    bool has_segment_only_task,
    const QueryBridgeRetryOptions& retry_options) {
    return batch_options.parallel_task_rrt &&
           !has_segment_only_task &&
           retry_options.no_path_retry_attempts == 0 &&
           retry_options.no_path_retry_budget_stages == 0;
}

bool query_bridge_task_has_explicit_satisfaction(
    const QueryBridgeSearchTask& task) {
    return task.hipac_online_satisfied ||
           task.direct_start_goal_satisfied;
}

int query_bridge_edge_query_index(bool scene_reusable_edges,
                                  const QueryBridgeSearchTask& task) {
    return scene_reusable_edges ? -1 : task.query_index;
}

QueryBridgeAttemptPlan query_bridge_prepare_attempt_plan(
    const QueryBridgeSearchTask& task,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    StageContext& context) {
    QueryBridgeAttemptPlan plan =
        query_bridge_attempt_plan(task,
                                  query_bridge_index_forced(index_options, task.index),
                                  retry_options);
    if (plan.partition_path_first) {
        record_query_bridge_partition_path_first_task(context, task.index);
    }
    record_query_bridge_forced_attempts(context,
                                        task.index,
                                        plan.forced,
                                        plan.effective_attempts);
    return plan;
}

QueryBridgePartitionInitialPathDecision query_bridge_partition_initial_path_decision(
    const QueryResult& initial_query,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const QueryBridgeAcceptanceThresholds& thresholds,
    const QueryBridgePartitionPathFirstOptions& options) {
    QueryBridgePartitionInitialPathDecision decision;
    decision.direct_distance = (goal - start).norm();
    decision.raw_length =
        initial_query.raw_path_length > 1e-12
            ? initial_query.raw_path_length
            : initial_query.path_length;
    decision.segment_fraction =
        decision.raw_length > 1e-12
            ? initial_query.segment_edge_length / decision.raw_length
            : std::numeric_limits<double>::infinity();
    decision.segment_reasonable =
        std::isfinite(decision.segment_fraction) &&
        decision.segment_fraction <= options.max_segment_fraction;
    decision.length_reasonable =
        decision.direct_distance <= 1e-9 ||
        initial_query.path_length <=
            std::max(decision.direct_distance * thresholds.path_ratio,
                     decision.direct_distance + thresholds.path_additive) ||
        initial_query.path_length <= thresholds.max_path_length;
    decision.accepted =
        decision.segment_reasonable &&
        (decision.length_reasonable || options.allow_long);
    return decision;
}

std::vector<Eigen::VectorXd> query_bridge_direct_line_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const QueryBridgeDirectLineFallbackOptions& options,
    StageContext& context) {
    if (!options.enabled) {
        return {};
    }
    context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_attempts");
    CollisionChecker checker = make_audit_checker(audit_robot, scene, query_config);
    std::vector<Eigen::VectorXd> path{task.start, task.goal};
    const PathAuditCheck audit =
        audit_waypoint_path(path,
                            checker,
                            query_config.audit_resolution,
                            query_config.audit_segment_step);
    if (!audit.passed) {
        context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_rejects");
        return {};
    }
    context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_successes");
    return path;
}

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds) {
    if (!current.success || !current.audit_passed) {
        return false;
    }
    const double raw_length =
        current.raw_path_length > 1e-12 ? current.raw_path_length : current.path_length;
    const double segment_fraction =
        raw_length > 1e-12 ? current.segment_edge_length / raw_length
                           : std::numeric_limits<double>::infinity();
    if (!(segment_fraction <= thresholds.max_segment_fraction)) {
        return false;
    }
    const double direct = (goal - start).norm();
    return direct <= 1e-9 ||
           current.path_length <= std::max(direct * thresholds.path_ratio,
                                            direct + thresholds.path_additive) ||
           current.path_length <= thresholds.max_path_length;
}

}  // namespace rbf
