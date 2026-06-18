#include "planning_forest_query_bridge_batch_utils.h"

#include "planning_forest_audit.h"
#include "planning_forest_query_utils.h"

#include <SBF/box_graph.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <utility>

namespace rbf {

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after) {
    auto add_counter_delta = [&](const std::string& key, auto after_value, auto before_value) {
        profile.diagnostics[key] += static_cast<double>(after_value - before_value);
    };
    add_counter_delta("query_bridge.oracle_node_validations",
                      after.node_validations,
                      before.node_validations);
    add_counter_delta("query_bridge.oracle_validation_cache_hits",
                      after.validation_cache_hits,
                      before.validation_cache_hits);
    add_counter_delta("query_bridge.oracle_validation_cache_misses",
                      after.validation_cache_misses,
                      before.validation_cache_misses);
    add_counter_delta("query_bridge.oracle_materializations",
                      after.materializations,
                      before.materializations);
    add_counter_delta("query_bridge.oracle_external_exact_hits",
                      after.materialization_external_exact_hits,
                      before.materialization_external_exact_hits);
    add_counter_delta("query_bridge.oracle_external_exact_misses",
                      after.materialization_external_exact_misses,
                      before.materialization_external_exact_misses);
    add_counter_delta("query_bridge.oracle_interval_replay_compatibility_checks",
                      after.interval_replay_compatibility_checks,
                      before.interval_replay_compatibility_checks);
    add_counter_delta("query_bridge.oracle_interval_replay_compatible",
                      after.interval_replay_compatible,
                      before.interval_replay_compatible);
    add_counter_delta("query_bridge.oracle_interval_replay_incompatible",
                      after.interval_replay_incompatible,
                      before.interval_replay_incompatible);
    add_counter_delta("query_bridge.oracle_interval_replay_direct_exact_hits",
                      after.interval_replay_direct_exact_hits,
                      before.interval_replay_direct_exact_hits);
    add_counter_delta("query_bridge.oracle_interval_replay_key_only_blocked",
                      after.interval_replay_key_only_blocked,
                      before.interval_replay_key_only_blocked);
    add_counter_delta("query_bridge.oracle_shared_endpoint_cache_hits",
                      after.materialization_reused_shared_endpoint_cache,
                      before.materialization_reused_shared_endpoint_cache);
    add_counter_delta("query_bridge.oracle_endpoint_path_ms",
                      after.validate_node_endpoint_path_time_us * 1.0e-3,
                      before.validate_node_endpoint_path_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_classify_ms",
                      after.validate_node_classify_time_us * 1.0e-3,
                      before.validate_node_classify_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_validate_total_ms",
                      after.validate_node_total_time_us * 1.0e-3,
                      before.validate_node_total_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_endpoint_ms",
                      after.materialization_endpoint_time_us * 1.0e-3,
                      before.materialization_endpoint_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_envelope_ms",
                      after.materialization_envelope_time_us * 1.0e-3,
                      before.materialization_envelope_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_envelope_collision_queries",
                      after.envelope_collision_queries,
                      before.envelope_collision_queries);
    add_counter_delta("query_bridge.oracle_envelope_gjk_tests",
                      after.envelope_collision_gjk_tests,
                      before.envelope_collision_gjk_tests);
}

std::string query_bridge_task_key(std::size_t index, const std::string& suffix) {
    return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
}

bool query_bridge_short_local_distance(double bridge_distance) {
    return bridge_distance > 0.55 && bridge_distance < 0.85;
}

void query_bridge_configure_short_local_profiles(
    RRTConnectConfig& bridge_rrt,
    std::vector<RRTConnectConfig>& short_local_profiles) {
    bridge_rrt.step_size = std::min(bridge_rrt.step_size, 0.25);
    bridge_rrt.goal_bias = 0.08;
    bridge_rrt.local_sampling_radius =
        bridge_rrt.local_sampling_radius > 0.0
            ? std::min(bridge_rrt.local_sampling_radius, 0.85)
            : 0.85;
    auto add_profile = [&](double step_size, double goal_bias, double radius) {
        RRTConnectConfig profile = bridge_rrt;
        profile.step_size = step_size;
        profile.goal_bias = goal_bias;
        profile.local_sampling_radius = radius;
        profile.shortcut_path = true;
        short_local_profiles.push_back(std::move(profile));
    };
    add_profile(0.25, 0.08, 0.90);
    add_profile(0.50, 0.20, 1.00);
    add_profile(0.35, 0.10, 1.00);
    add_profile(0.25, 0.08, 0.45);
}

RRTConnectConfig query_bridge_rrt_config_for_attempt(
    const QueryBridgeSearchTask& task,
    int attempt,
    int scheduled_attempt,
    int override_fixed_iters,
    double default_timeout_ms,
    const QueryBridgeRetryOptions& options) {
    RRTConnectConfig config =
        task.short_local_profiles.empty()
            ? task.bridge_rrt
            : task.short_local_profiles[
                  static_cast<std::size_t>(scheduled_attempt) % task.short_local_profiles.size()];
    if (!options.local_radius_schedule.empty() &&
        attempt >= 0 &&
        static_cast<std::size_t>(attempt) < options.local_radius_schedule.size()) {
        const double scheduled_radius =
            options.local_radius_schedule[static_cast<std::size_t>(attempt)];
        if (scheduled_radius >= 0.0) {
            config.local_sampling_radius = scheduled_radius;
        }
    }
    config.optimize_after_first_iters = options.rrt_optimize_after_first_iters;
    const int effective_fixed_iters =
        override_fixed_iters > 0 ? override_fixed_iters : options.rrt_fixed_iters;
    if (effective_fixed_iters > 0) {
        config.max_iters = effective_fixed_iters;
        config.timeout_ms = options.rrt_fixed_timeout_ms;
    } else {
        config.timeout_ms = std::max(1.0, default_timeout_ms);
    }
    return config;
}

int query_bridge_rrt_seed_for_attempt(const QueryBridgeSearchTask& task,
                                      int rng_seed,
                                      int scheduled_attempt) {
    return derived_planner_seed(rng_seed,
                                kSeedBatchBridgeOffset,
                                scheduled_attempt,
                                task.query_index,
                                task.short_local_bridge ? 0 : kSeedAttemptStride);
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
    std::shared_ptr<std::atomic<bool>> cancel_override) {
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

void query_bridge_mark_task_skip(BuildProfile& profile,
                                 std::size_t index,
                                 double code,
                                 const char* reason) {
    profile.diagnostics[query_bridge_task_key(index, "skip_reason_code")] = code;
    if (reason != nullptr && reason[0] != '\0') {
        profile.diagnostics[std::string("query_bridge.batch_task_skip.") + reason] += 1.0;
    }
}

void record_query_bridge_partition_path_first_task(StageContext& context,
                                                   std::size_t index) {
    context.diagnostics().add_counter("query_bridge.partition_path_first_tasks");
    context.diagnostics().set_value(
        query_bridge_task_key(index, "partition_path_first"),
        1.0);
}

void record_query_bridge_partition_path_first_rrt_skipped(StageContext& context,
                                                          std::size_t index) {
    context.diagnostics().add_counter("query_bridge.partition_path_first_rrt_skipped");
    context.diagnostics().set_value(
        query_bridge_task_key(index, "waypoint_from_partition_path"),
        1.0);
}

void record_query_bridge_batch_task_no_path(StageContext& context,
                                            std::size_t index,
                                            double total_ms) {
    context.diagnostics().add_counter("query_bridge.batch_tasks_no_path");
    context.diagnostics().set_value(query_bridge_task_key(index, "no_path"), 1.0);
    context.diagnostics().set_value(query_bridge_task_key(index, "total_ms"), total_ms);
}

void record_query_bridge_batch_task_already_satisfied(
    StageContext& context,
    const QueryBridgeSearchTask& task,
    double probe_ms) {
    context.diagnostics().add_counter("query_bridge.batch_tasks_skipped");
    context.diagnostics().record_timing("query_bridge.batch_probe_ms_total", probe_ms);
    context.diagnostics().set_value(query_bridge_task_key(task.index, "skipped"), 1.0);
    if (task.hipac_online_satisfied) {
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "skipped_by_hipac_online"),
            1.0);
    }
    if (task.direct_start_goal_satisfied) {
        context.diagnostics().set_value(
            query_bridge_task_key(task.index, "skipped_by_direct_start_goal_segment"),
            1.0);
    }
}

void record_query_bridge_batch_task_skipped_after_rrt(StageContext& context,
                                                      std::size_t index,
                                                      bool forced_task,
                                                      double probe_ms,
                                                      double total_ms) {
    context.diagnostics().add_counter("query_bridge.batch_tasks_skipped_after_rrt");
    if (forced_task) {
        context.diagnostics().add_counter("query_bridge.batch_forced_tasks_skipped_after_rrt");
    }
    context.diagnostics().record_timing("query_bridge.batch_probe_ms_total", probe_ms);
    context.diagnostics().set_value(query_bridge_task_key(index, "skipped_after_rrt"), 1.0);
    context.diagnostics().set_value(query_bridge_task_key(index, "total_ms"), total_ms);
}

void record_query_bridge_batch_task_skipped_by_hipac_after_rrt(
    StageContext& context,
    std::size_t index,
    double total_ms) {
    context.diagnostics().add_counter(
        "query_bridge.batch_tasks_skipped_by_hipac_after_rrt");
    context.diagnostics().set_value(
        query_bridge_task_key(index, "skipped_by_hipac_after_rrt"),
        1.0);
    context.diagnostics().set_value(query_bridge_task_key(index, "total_ms"),
                                    total_ms);
}

void record_query_bridge_forced_attempts(StageContext& context,
                                         std::size_t index,
                                         bool forced_task,
                                         int attempts) {
    if (forced_task) {
        context.diagnostics().set_value(query_bridge_task_key(index, "forced"), 1.0);
    }
    context.diagnostics().set_value(query_bridge_task_key(index, "attempts"),
                                    static_cast<double>(attempts));
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

void query_bridge_run_segment_only_retry(
    QueryBridgeSearchTask& task,
    int first_attempt,
    double& best_length,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeRetryPathRunner& run_task_attempt,
    StageContext& context) {
    if (!task.waypoint_path.empty() ||
        retry_options.segment_only_retry_attempts <= 0) {
        return;
    }
    const auto retry_t0 = std::chrono::steady_clock::now();
    int retry_successes = 0;
    for (int retry = 0; retry < retry_options.segment_only_retry_attempts; ++retry) {
        query_bridge_adopt_retry_path_if_better(
            task,
            run_task_attempt(first_attempt + retry, 0),
            best_length,
            retry_successes);
        if (retry_successes > 0 && retry_options.no_path_retry_stop_on_first_success) {
            break;
        }
    }
    const double retry_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - retry_t0)
            .count();
    context.diagnostics().record_timing(
        "query_bridge.batch_segment_only_retry_ms_total",
        retry_ms);
    context.diagnostics().add_counter(
        "query_bridge.batch_segment_only_retry_attempts",
        static_cast<double>(retry_options.segment_only_retry_attempts));
    context.diagnostics().add_counter(
        "query_bridge.batch_segment_only_retry_successes",
        static_cast<double>(retry_successes));
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "segment_only_retry_attempts"),
        static_cast<double>(retry_options.segment_only_retry_attempts));
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "segment_only_retry_ms"),
        retry_ms);
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "segment_only_retry_successes"),
        static_cast<double>(retry_successes));
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
        const std::string key_prefix =
            stage_index == 0
                ? query_bridge_task_key(task.index, "no_path_retry_")
                : query_bridge_task_key(
                      task.index,
                      "no_path_retry_stage." + std::to_string(stage_index) + ".");
        context.diagnostics().set_value(
            key_prefix + "attempts",
            static_cast<double>(effective_stage_attempts));
        context.diagnostics().set_value(
            key_prefix + "attempts_run",
            static_cast<double>(retry_attempts_run));
        context.diagnostics().set_value(
            key_prefix + "successes",
            static_cast<double>(retry_successes));
        context.diagnostics().set_value(
            key_prefix + "fixed_iters",
            static_cast<double>(stage_fixed_iters));
        context.diagnostics().set_value(key_prefix + "ms", retry_ms);
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
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "no_path_retry_attempts"),
        static_cast<double>(retry_attempts_total));
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "no_path_retry_ms"),
        retry_ms_total);
    context.diagnostics().set_value(
        query_bridge_task_key(task.index, "no_path_retry_successes"),
        static_cast<double>(retry_successes_total));
}

int query_bridge_edge_query_index(bool scene_reusable_edges,
                                  const QueryBridgeSearchTask& task) {
    return scene_reusable_edges ? -1 : task.query_index;
}

QueryBridgeAttemptPlan query_bridge_attempt_plan(
    const QueryBridgeSearchTask& task,
    bool forced,
    const QueryBridgeRetryOptions& options) {
    QueryBridgeAttemptPlan plan;
    plan.forced = forced;
    plan.base_attempts =
        forced ? std::max(std::max(1, task.attempts), options.forced_attempts)
               : std::max(1, task.attempts);
    plan.partition_path_first =
        task.waypoint_path_from_partition_query && !task.waypoint_path.empty();
    plan.effective_attempts =
        plan.partition_path_first ? 0 : plan.base_attempts;
    if (plan.effective_attempts > 0 &&
        !options.local_radius_schedule.empty() &&
        options.local_radius_append_unrestricted_attempt) {
        plan.effective_attempts =
            std::max(plan.effective_attempts,
                     static_cast<int>(options.local_radius_schedule.size()) + 1);
    }
    return plan;
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

std::shared_ptr<std::atomic<bool>> query_bridge_parallel_rrt_cancel_flag(
    const QueryBridgeParallelRrtOptions& options,
    const std::shared_ptr<std::atomic<bool>>& fallback_cancel) {
    return options.early_stop
        ? std::make_shared<std::atomic<bool>>(false)
        : fallback_cancel;
}

bool query_bridge_parallel_rrt_cancelled(
    const std::shared_ptr<std::atomic<bool>>& cancel_flag) {
    return cancel_flag && cancel_flag->load(std::memory_order_relaxed);
}

void query_bridge_maybe_stop_parallel_rrt_after_success(
    bool path_good_enough,
    const QueryBridgeParallelRrtOptions& options,
    std::atomic<int>& early_successes,
    const std::shared_ptr<std::atomic<bool>>& cancel_flag) {
    if (!options.early_stop || !path_good_enough) {
        return;
    }
    const int successes =
        early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
    if (successes >= options.early_stop_min_successes && cancel_flag) {
        cancel_flag->store(true, std::memory_order_relaxed);
    }
}

void record_query_bridge_parallel_rrt_early_stop(
    StageContext& context,
    const QueryBridgeParallelRrtOptions& options,
    const std::shared_ptr<std::atomic<bool>>& cancel_flag,
    const std::atomic<int>& early_successes) {
    if (!options.early_stop) {
        return;
    }
    context.diagnostics().add_counter(
        "query_bridge.parallel_rrt_early_stop_successes",
        static_cast<double>(early_successes.load(std::memory_order_relaxed)));
    context.diagnostics().add_counter(
        query_bridge_parallel_rrt_cancelled(cancel_flag)
            ? "query_bridge.parallel_rrt_early_stop_triggered"
            : "query_bridge.parallel_rrt_early_stop_not_triggered");
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

std::vector<Eigen::VectorXd> query_bridge_deterministic_detour_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const std::vector<Interval>& planning_domain,
    const QueryBridgeDetourOptions& options,
    int rng_seed_base,
    StageContext& context) {
    if (!options.enabled ||
        task.start.size() != task.goal.size() ||
        task.start.size() <= 0 ||
        static_cast<int>(planning_domain.size()) != task.start.size()) {
        return {};
    }
    CollisionChecker checker = make_audit_checker(audit_robot, scene, query_config);
    const Eigen::VectorXd delta = task.goal - task.start;
    const double direct_length = delta.norm();
    if (direct_length <= 1e-9) {
        return {};
    }
    std::vector<int> dims(static_cast<std::size_t>(task.start.size()));
    std::iota(dims.begin(), dims.end(), 0);
    std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
        const double lhs_width = std::max(1e-9, planning_domain[static_cast<std::size_t>(lhs)].width());
        const double rhs_width = std::max(1e-9, planning_domain[static_cast<std::size_t>(rhs)].width());
        const double lhs_along = std::abs(delta[lhs]) / lhs_width;
        const double rhs_along = std::abs(delta[rhs]) / rhs_width;
        if (std::abs(lhs_along - rhs_along) > 1e-12) {
            return lhs_along < rhs_along;
        }
        return lhs < rhs;
    });
    const int dim_limit = std::min<int>(options.dims, static_cast<int>(dims.size()));
    const int rounds = options.rounds;
    const int max_candidates = options.max_candidates;
    const bool multi_axis_detour = options.multi_axis;
    const int random_candidates = options.random_candidates;
    const double base_offset = options.offset;
    const double two_bend_alpha = options.two_bend_alpha;
    const Eigen::VectorXd mid = 0.5 * (task.start + task.goal);
    double best_length = std::numeric_limits<double>::infinity();
    std::vector<Eigen::VectorXd> best_path;
    int candidates = 0;
    auto clamp_to_domain = [&](Eigen::VectorXd point) {
        for (int dim = 0; dim < point.size(); ++dim) {
            point[dim] = std::min(planning_domain[static_cast<std::size_t>(dim)].hi,
                                  std::max(planning_domain[static_cast<std::size_t>(dim)].lo,
                                           point[dim]));
        }
        return point;
    };
    auto try_path = [&](std::vector<Eigen::VectorXd> path) {
        if (candidates >= max_candidates) {
            return;
        }
        ++candidates;
        context.diagnostics().add_counter("query_bridge.detour_on_no_path_candidates");
        double length = path_length(path);
        if (!std::isfinite(length) || length + 1e-12 >= best_length) {
            return;
        }
        const PathAuditCheck audit =
            audit_waypoint_path(path,
                                checker,
                                query_config.audit_resolution,
                                query_config.audit_segment_step);
        if (!audit.passed) {
            context.diagnostics().add_counter("query_bridge.detour_on_no_path_rejects");
            return;
        }
        best_length = length;
        best_path = std::move(path);
    };
    for (int item = 0; item < dim_limit && candidates < max_candidates; ++item) {
        const int dim = dims[static_cast<std::size_t>(item)];
        const double width = planning_domain[static_cast<std::size_t>(dim)].width();
        for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
            const double magnitude = std::min(0.45 * std::max(0.0, width),
                                              base_offset * static_cast<double>(round));
            if (magnitude <= 1e-9) {
                continue;
            }
            for (double sign : {1.0, -1.0}) {
                Eigen::VectorXd single = mid;
                single[dim] += sign * magnitude;
                single = clamp_to_domain(std::move(single));
                if ((single - mid).norm() > 1e-9) {
                    try_path({task.start, single, task.goal});
                }
                if (candidates >= max_candidates) {
                    break;
                }
                Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                first[dim] += sign * magnitude;
                second[dim] += sign * magnitude;
                first = clamp_to_domain(std::move(first));
                second = clamp_to_domain(std::move(second));
                if ((first - (task.start + two_bend_alpha * delta)).norm() > 1e-9 ||
                    (second - (task.start + (1.0 - two_bend_alpha) * delta)).norm() > 1e-9) {
                    try_path({task.start, first, second, task.goal});
                }
                if (candidates >= max_candidates) {
                    break;
                }
            }
        }
    }
    if (multi_axis_detour && dim_limit >= 2) {
        for (int first_item = 0; first_item < dim_limit && candidates < max_candidates; ++first_item) {
            const int first_dim = dims[static_cast<std::size_t>(first_item)];
            const double first_width = planning_domain[static_cast<std::size_t>(first_dim)].width();
            for (int second_item = first_item + 1;
                 second_item < dim_limit && candidates < max_candidates;
                 ++second_item) {
                const int second_dim = dims[static_cast<std::size_t>(second_item)];
                const double second_width = planning_domain[static_cast<std::size_t>(second_dim)].width();
                for (int round = 1; round <= rounds && candidates < max_candidates; ++round) {
                    const double first_mag = std::min(0.35 * std::max(0.0, first_width),
                                                      base_offset * static_cast<double>(round));
                    const double second_mag = std::min(0.35 * std::max(0.0, second_width),
                                                       base_offset * static_cast<double>(round));
                    if (first_mag <= 1e-9 || second_mag <= 1e-9) {
                        continue;
                    }
                    for (double first_sign : {1.0, -1.0}) {
                        for (double second_sign : {1.0, -1.0}) {
                            Eigen::VectorXd single = mid;
                            single[first_dim] += first_sign * first_mag;
                            single[second_dim] += second_sign * second_mag;
                            single = clamp_to_domain(std::move(single));
                            if ((single - mid).norm() > 1e-9) {
                                try_path({task.start, single, task.goal});
                            }
                            if (candidates >= max_candidates) {
                                break;
                            }
                            Eigen::VectorXd first = task.start + two_bend_alpha * delta;
                            Eigen::VectorXd second = task.start + (1.0 - two_bend_alpha) * delta;
                            first[first_dim] += first_sign * first_mag;
                            first[second_dim] += second_sign * second_mag;
                            second[first_dim] += first_sign * first_mag;
                            second[second_dim] += second_sign * second_mag;
                            first = clamp_to_domain(std::move(first));
                            second = clamp_to_domain(std::move(second));
                            try_path({task.start, first, second, task.goal});
                            if (candidates >= max_candidates) {
                                break;
                            }
                        }
                        if (candidates >= max_candidates) {
                            break;
                        }
                    }
                }
            }
        }
    }
    if (random_candidates > 0 && dim_limit > 0 && candidates < max_candidates) {
        const int random_budget = std::min(random_candidates, max_candidates - candidates);
        std::mt19937 rng(static_cast<std::uint32_t>(
            derived_planner_seed(rng_seed_base,
                                 kSeedBatchBridgeOffset,
                                 static_cast<int>(task.index),
                                 task.query_index,
                                 41443)));
        std::uniform_int_distribution<int> dim_pick(0, dim_limit - 1);
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        const double max_scale = std::max(1.0, static_cast<double>(rounds));
        for (int sample = 0; sample < random_budget && candidates < max_candidates; ++sample) {
            const int first_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
            int second_dim = first_dim;
            if (dim_limit > 1) {
                for (int guard = 0; guard < 4 && second_dim == first_dim; ++guard) {
                    second_dim = dims[static_cast<std::size_t>(dim_pick(rng))];
                }
            }
            Eigen::VectorXd offset = Eigen::VectorXd::Zero(task.start.size());
            auto apply_random_dim = [&](int dim) {
                const double width = planning_domain[static_cast<std::size_t>(dim)].width();
                const double limit = std::min(0.35 * std::max(0.0, width),
                                              base_offset * max_scale);
                if (limit > 1e-9) {
                    offset[dim] += unit(rng) * limit;
                }
            };
            apply_random_dim(first_dim);
            if (second_dim != first_dim) {
                apply_random_dim(second_dim);
            }
            if (offset.norm() <= 1e-9) {
                continue;
            }
            if ((sample & 1) == 0) {
                Eigen::VectorXd single = clamp_to_domain(mid + offset);
                try_path({task.start, single, task.goal});
            } else {
                Eigen::VectorXd first = clamp_to_domain(task.start + two_bend_alpha * delta + offset);
                Eigen::VectorXd second = clamp_to_domain(task.start + (1.0 - two_bend_alpha) * delta + offset);
                try_path({task.start, first, second, task.goal});
            }
        }
    }
    context.diagnostics().add_counter("query_bridge.detour_on_no_path_attempts");
    if (!best_path.empty()) {
        context.diagnostics().add_counter("query_bridge.detour_on_no_path_successes");
    }
    return best_path;
}

bool query_bridge_maybe_apply_detour_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const std::vector<Interval>& planning_domain,
    const QueryBridgeDetourOptions& options,
    int rng_seed_base,
    StageContext& context,
    double& best_length,
    std::vector<Eigen::VectorXd>& waypoint_path) {
    if (!waypoint_path.empty() && !options.candidate) {
        return false;
    }
    auto detour_path = query_bridge_deterministic_detour_fallback_path(
        task,
        audit_robot,
        scene,
        query_config,
        planning_domain,
        options,
        rng_seed_base,
        context);
    if (detour_path.empty()) {
        return false;
    }
    const double detour_length = path_length(detour_path);
    if (!waypoint_path.empty() &&
        detour_length > best_length * options.replace_factor + 1e-12) {
        context.diagnostics().add_counter(
            "query_bridge.detour_candidate_not_shorter");
        return false;
    }
    best_length = detour_length;
    waypoint_path = std::move(detour_path);
    context.diagnostics().add_counter(
        "query_bridge.detour_candidate_selected");
    return true;
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

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index) {
    auto add = [&](const std::string& suffix, const std::string& total_key) {
        const auto it = profile.diagnostics.find(query_bridge_task_key(task_index, suffix));
        if (it != profile.diagnostics.end()) {
            context.diagnostics().add_counter(total_key, it->second);
        }
    };
    add("direct_corridor_ms", "query_bridge.direct_corridor_ms_total");
    add("direct_corridor_samples", "query_bridge.direct_corridor_samples_total");
    add("direct_corridor_ffb_calls", "query_bridge.direct_corridor_ffb_calls_total");
    add("direct_corridor_all_ffb_calls", "query_bridge.direct_corridor_all_ffb_calls_total");
    add("direct_corridor_direct_ffb_ms", "query_bridge.direct_corridor_direct_ffb_ms");
    add("direct_corridor_repair_ffb_ms", "query_bridge.direct_corridor_repair_ffb_ms");
    add("direct_corridor_adaptive_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_repair_ffb_ms");
    add("direct_corridor_lateral_repair_ffb_ms",
        "query_bridge.direct_corridor_lateral_repair_ffb_ms");
    add("direct_corridor_segment_audit_ms",
        "query_bridge.direct_corridor_segment_audit_ms");
    add("direct_corridor_added", "query_bridge.direct_corridor_added_total");
    add("direct_corridor_repair_calls", "query_bridge.direct_corridor_repair_calls_total");
    add("direct_corridor_repair_added", "query_bridge.direct_corridor_repair_added_total");
    add("direct_corridor_adaptive_repair_calls",
        "query_bridge.direct_corridor_adaptive_repair_calls_total");
    add("direct_corridor_adaptive_repair_added",
        "query_bridge.direct_corridor_adaptive_repair_added_total");
    add("direct_corridor_lateral_repair_calls",
        "query_bridge.direct_corridor_lateral_repair_calls_total");
    add("direct_corridor_lateral_repair_added",
        "query_bridge.direct_corridor_lateral_repair_added_total");
    add("direct_corridor_bad_initial", "query_bridge.direct_corridor_bad_initial_total");
    add("direct_corridor_bad_final", "query_bridge.direct_corridor_bad_final_total");
    add("direct_corridor_segment_edges", "query_bridge.direct_corridor_segment_edges_total");
    add("direct_corridor_ffb_find_calls", "query_bridge.direct_corridor_ffb_find_calls_total");
    add("direct_corridor_ffb_binary_requested",
        "query_bridge.direct_corridor_ffb_binary_requested_total");
    add("direct_corridor_ffb_virtual_sparse_binary_attempts",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_attempts_total");
    add("direct_corridor_ffb_virtual_sparse_binary_successes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_successes_total");
    add("direct_corridor_ffb_virtual_sparse_binary_probes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_probes_total");
    add("direct_corridor_ffb_binary_materialized_fallback_calls",
        "query_bridge.direct_corridor_ffb_binary_materialized_fallback_calls_total");
    add("direct_corridor_ffb_binary_blocked_adaptive_depths",
        "query_bridge.direct_corridor_ffb_binary_blocked_adaptive_depths_total");
    add("direct_corridor_ffb_binary_virtual_unsupported",
        "query_bridge.direct_corridor_ffb_binary_virtual_unsupported_total");
    add("direct_corridor_ffb_linear_descent_calls",
        "query_bridge.direct_corridor_ffb_linear_descent_calls_total");
    add("direct_corridor_transition_connected_ms",
        "query_bridge.direct_corridor_transition_connected_ms");
    add("direct_corridor_transition_connected_calls",
        "query_bridge.direct_corridor_transition_connected_calls");
    add("direct_corridor_bad_transitions_ms",
        "query_bridge.direct_corridor_bad_transitions_ms");
    add("direct_corridor_bad_transitions_calls",
        "query_bridge.direct_corridor_bad_transitions_calls");
    add("direct_corridor_current_cover_ms", "query_bridge.direct_corridor_current_cover_ms");
    add("direct_corridor_current_cover_calls", "query_bridge.direct_corridor_current_cover_calls");
    add("direct_corridor_current_cover_partition_ms",
        "query_bridge.direct_corridor_current_cover_partition_ms");
    add("direct_corridor_current_cover_corridor_scan_ms",
        "query_bridge.direct_corridor_current_cover_corridor_scan_ms");
    add("direct_corridor_current_cover_direct_index_ms",
        "query_bridge.direct_corridor_current_cover_direct_index_ms");
    add("direct_corridor_duplicate_lookup_ms", "query_bridge.direct_corridor_duplicate_lookup_ms");
    add("direct_corridor_duplicate_lookup_calls",
        "query_bridge.direct_corridor_duplicate_lookup_calls");
    add("direct_corridor_commit_total_ms", "query_bridge.direct_corridor_commit_total_ms");
    add("direct_corridor_commit_calls", "query_bridge.direct_corridor_commit_calls");
    add("direct_corridor_commit_dynamic_policy_ms",
        "query_bridge.direct_corridor_commit_dynamic_policy_ms");
    add("direct_corridor_commit_partition_append_ms",
        "query_bridge.direct_corridor_commit_partition_append_ms");
    add("direct_corridor_partition_append_calls",
        "query_bridge.direct_corridor_partition_append_calls");
    add("direct_corridor_partition_append_boxes",
        "query_bridge.direct_corridor_partition_append_boxes");
    add("direct_corridor_assimilate_calls", "query_bridge.direct_corridor_assimilate_calls");
    add("direct_corridor_assimilate_sample_scan_ms",
        "query_bridge.direct_corridor_assimilate_sample_scan_ms");
    add("direct_corridor_assimilate_local_hits",
        "query_bridge.direct_corridor_assimilate_local_hits");
    add("direct_corridor_assimilate_full_scan_fallbacks",
        "query_bridge.direct_corridor_assimilate_full_scan_fallbacks");
    add("direct_corridor_assimilate_local_sample_tests",
        "query_bridge.direct_corridor_assimilate_local_sample_tests");
    add("direct_corridor_assimilate_candidate_build_ms",
        "query_bridge.direct_corridor_assimilate_candidate_build_ms");
    add("direct_corridor_assimilate_adjacency_ms",
        "query_bridge.direct_corridor_assimilate_adjacency_ms");
    add("direct_corridor_segment_insert_ms", "query_bridge.direct_corridor_segment_insert_ms");
    add("direct_corridor_segment_insert_calls",
        "query_bridge.direct_corridor_segment_insert_calls");
    add("direct_corridor_direct_task_build_ms",
        "query_bridge.direct_corridor_direct_task_build_ms");
    add("direct_corridor_direct_loop_ms", "query_bridge.direct_corridor_direct_loop_ms");
    add("direct_corridor_repair_loop_ms", "query_bridge.direct_corridor_repair_loop_ms");
    add("direct_corridor_adaptive_loop_ms", "query_bridge.direct_corridor_adaptive_loop_ms");
    add("direct_corridor_lateral_loop_ms", "query_bridge.direct_corridor_lateral_loop_ms");
    add("direct_corridor_residual_segment_loop_ms",
        "query_bridge.direct_corridor_residual_segment_loop_ms");
}

}  // namespace rbf
