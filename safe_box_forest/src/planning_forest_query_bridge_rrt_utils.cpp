#include "planning_forest_query_bridge_rrt_utils.h"

#include "env_config.h"
#include "planning_forest_audit.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_task.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace rbf {

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
    const int effective_fixed_iters =
        override_fixed_iters > 0 ? override_fixed_iters : options.rrt_fixed_iters;
    if (effective_fixed_iters > 0) {
        config.max_iters = effective_fixed_iters;
        config.timeout_ms = 0.0;
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

QueryBridgeRetryOptions query_bridge_retry_options_from_env() {
    QueryBridgeRetryOptions options;
    options.no_path_retry_attempts =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS", 1));
    options.no_path_retry_stop_on_first_success =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS", 0) != 0;
    options.forced_attempts =
        std::max(1, detail::env_int_or_default("RBF_QUERY_BRIDGE_FORCED_ATTEMPTS", 1));
    options.attempt_offset =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_ATTEMPT_OFFSET", 0));
    options.rrt_fixed_iters =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_RRT_FIXED_ITERS", 0));
    options.local_radius_schedule =
        detail::env_double_list_or_empty("RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE");
    options.no_path_retry_budget_iters =
        detail::env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ITERS");
    options.no_path_retry_budget_attempts =
        detail::env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ATTEMPTS");
    options.no_path_retry_budget_stages =
        std::min(options.no_path_retry_budget_iters.size(),
                 options.no_path_retry_budget_attempts.size());
    return options;
}

void record_query_bridge_retry_diagnostics(StageContext& context,
                                           const QueryBridgeRetryOptions& options) {
    context.diagnostics().set_value("query_bridge.no_path_retry_attempts_default",
                                    static_cast<double>(options.no_path_retry_attempts));
    context.diagnostics().set_value("query_bridge.no_path_retry_stop_on_first_success",
                                    options.no_path_retry_stop_on_first_success ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.rrt_fixed_iters",
                                    static_cast<double>(options.rrt_fixed_iters));
    context.diagnostics().set_value("query_bridge.local_radius_schedule_size",
                                    static_cast<double>(options.local_radius_schedule.size()));
    context.diagnostics().set_value("query_bridge.no_path_retry_budget_stages",
                                    static_cast<double>(options.no_path_retry_budget_stages));
    for (std::size_t stage = 0; stage < options.no_path_retry_budget_stages; ++stage) {
        const std::string prefix =
            "query_bridge.no_path_retry_budget_stage." + std::to_string(stage) + ".";
        context.diagnostics().set_value(
            prefix + "iters",
            static_cast<double>(options.no_path_retry_budget_iters[stage]));
        context.diagnostics().set_value(
            prefix + "attempts",
            static_cast<double>(options.no_path_retry_budget_attempts[stage]));
    }
}

QueryBridgeParallelRrtOptions query_bridge_parallel_rrt_options_from_env() {
    QueryBridgeParallelRrtOptions options;
    options.early_stop =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP", 0) != 0;
    options.early_stop_min_successes =
        std::max(1,
                 detail::env_int_or_default(
                     "RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES",
                     1));
    options.early_stop_ratio =
        std::max(1.0,
                 detail::env_double_or_default(
                     "RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO",
                     1.75));
    options.early_stop_additive =
        std::max(0.0,
                 detail::env_double_or_default(
                     "RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE",
                     0.75));
    return options;
}

void record_query_bridge_parallel_rrt_diagnostics(
    StageContext& context,
    const QueryBridgeParallelRrtOptions& options) {
    context.diagnostics().set_value("query_bridge.parallel_rrt_early_stop_enabled",
                                    options.early_stop ? 1.0 : 0.0);
}

bool query_bridge_parallel_rrt_path_good_enough(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const std::vector<Eigen::VectorXd>& path,
    const QueryBridgeParallelRrtOptions& options) {
    if (path.empty()) {
        return false;
    }
    const double direct = (goal - start).norm();
    if (direct <= 1e-9) {
        return true;
    }
    const double length = path_length(path);
    return length <= std::max(direct * options.early_stop_ratio,
                              direct + options.early_stop_additive);
}

bool query_bridge_task_rrt_path_good_enough(
    const QueryBridgeSearchTask& task,
    const std::vector<Eigen::VectorXd>& path,
    const QueryBridgeParallelRrtOptions& options) {
    return query_bridge_parallel_rrt_path_good_enough(task.start,
                                                      task.goal,
                                                      path,
                                                      options);
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

QueryBridgeAttemptPlan query_bridge_attempt_plan(
    const QueryBridgeSearchTask& task,
    bool forced,
    const QueryBridgeRetryOptions& options) {
    QueryBridgeAttemptPlan plan;
    plan.forced = forced;
    plan.base_attempts =
        forced ? std::max(std::max(1, task.attempts), options.forced_attempts)
               : std::max(1, task.attempts);
    plan.effective_attempts = plan.base_attempts;
    if (plan.effective_attempts > 0 &&
        !options.local_radius_schedule.empty()) {
        plan.effective_attempts =
            std::max(plan.effective_attempts,
                     static_cast<int>(options.local_radius_schedule.size()) + 1);
    }
    return plan;
}

std::shared_ptr<std::atomic<bool>> query_bridge_parallel_rrt_cancel_flag(
    const QueryBridgeParallelRrtOptions& options,
    const std::shared_ptr<std::atomic<bool>>& fallback_cancel) {
    if (!options.early_stop) {
        return fallback_cancel;
    }
    return std::make_shared<std::atomic<bool>>(false);
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
    if (!options.early_stop || !path_good_enough || !cancel_flag) {
        return;
    }
    const int successes =
        early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
    if (successes >= options.early_stop_min_successes) {
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

}  // namespace rbf
