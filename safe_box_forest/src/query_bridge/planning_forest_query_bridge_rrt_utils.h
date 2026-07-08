#pragma once

#include <SBF/connector_types.h>
#include <SBF/runtime_fwd.h>
#include <SBF/scene_types.h>

#include <Eigen/Core>

#include <atomic>
#include <functional>
#include <memory>
#include <vector>

namespace rbf {

struct RBFPlanningConfig;
struct QueryBridgeSearchTask;

struct QueryBridgeRetryOptions {
    int no_path_retry_attempts = 0;
    bool no_path_retry_stop_on_first_success = false;
    int forced_attempts = 1;
    int attempt_offset = 0;
    int rrt_fixed_iters = 0;
    std::vector<double> local_radius_schedule;
    std::vector<int> no_path_retry_budget_iters;
    std::vector<int> no_path_retry_budget_attempts;
    std::size_t no_path_retry_budget_stages = 0;
};

struct QueryBridgeAttemptPlan {
    bool forced = false;
    int base_attempts = 1;
    int effective_attempts = 1;
};

struct QueryBridgeParallelRrtOptions {
    bool early_stop = false;
    int early_stop_min_successes = 1;
    double early_stop_ratio = 1.75;
    double early_stop_additive = 0.75;
};

bool query_bridge_short_local_distance(double bridge_distance);

void query_bridge_configure_short_local_profiles(
    RRTConnectConfig& bridge_rrt,
    std::vector<RRTConnectConfig>& short_local_profiles);

RRTConnectConfig query_bridge_rrt_config_for_attempt(
    const QueryBridgeSearchTask& task,
    int attempt,
    int scheduled_attempt,
    int override_fixed_iters,
    double default_timeout_ms,
    const QueryBridgeRetryOptions& options);

int query_bridge_rrt_seed_for_attempt(const QueryBridgeSearchTask& task,
                                      int rng_seed,
                                      int scheduled_attempt);

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
        std::shared_ptr<std::atomic<bool>>{});

void run_query_bridge_task_attempts(
    QueryBridgeSearchTask& task,
    int effective_attempts,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeParallelRrtOptions& parallel_rrt_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context);

QueryBridgeRetryOptions query_bridge_retry_options_from_config(
    const RBFPlanningConfig& config);

void record_query_bridge_retry_diagnostics(StageContext& context,
                                           const QueryBridgeRetryOptions& options);

QueryBridgeParallelRrtOptions query_bridge_parallel_rrt_options_from_config(
    const RBFPlanningConfig& config);

void record_query_bridge_parallel_rrt_diagnostics(
    StageContext& context,
    const QueryBridgeParallelRrtOptions& options);

bool query_bridge_parallel_rrt_path_good_enough(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const std::vector<Eigen::VectorXd>& path,
    const QueryBridgeParallelRrtOptions& options);

bool query_bridge_task_rrt_path_good_enough(
    const QueryBridgeSearchTask& task,
    const std::vector<Eigen::VectorXd>& path,
    const QueryBridgeParallelRrtOptions& options);

std::shared_ptr<std::atomic<bool>> query_bridge_parallel_rrt_cancel_flag(
    const QueryBridgeParallelRrtOptions& options,
    const std::shared_ptr<std::atomic<bool>>& fallback_cancel);

bool query_bridge_parallel_rrt_cancelled(
    const std::shared_ptr<std::atomic<bool>>& cancel_flag);

void query_bridge_maybe_stop_parallel_rrt_after_success(
    bool path_good_enough,
    const QueryBridgeParallelRrtOptions& options,
    std::atomic<int>& early_successes,
    const std::shared_ptr<std::atomic<bool>>& cancel_flag);

void record_query_bridge_parallel_rrt_early_stop(
    StageContext& context,
    const QueryBridgeParallelRrtOptions& options,
    const std::shared_ptr<std::atomic<bool>>& cancel_flag,
    const std::atomic<int>& early_successes);

void query_bridge_adopt_retry_path_if_better(
    QueryBridgeSearchTask& task,
    std::vector<Eigen::VectorXd> retry_path,
    double& best_length,
    int& retry_successes);

using QueryBridgeRetryPathRunner =
    std::function<std::vector<Eigen::VectorXd>(int attempt, int fixed_iters)>;

void query_bridge_run_no_path_retries(
    QueryBridgeSearchTask& task,
    int first_attempt,
    double& best_length,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeRetryPathRunner& run_task_attempt,
    StageContext& context);

QueryBridgeAttemptPlan query_bridge_attempt_plan(
    const QueryBridgeSearchTask& task,
    bool forced,
    const QueryBridgeRetryOptions& options);

}  // namespace rbf
