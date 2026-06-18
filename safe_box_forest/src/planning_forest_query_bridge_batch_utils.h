#pragma once

#include <SBF/connector.h>
#include <SBF/safe_box_forest.h>

#include <Eigen/Core>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rbf {

struct QueryBridgeDetourOptions;

struct QueryBridgeAcceptanceThresholds {
    double max_segment_fraction = 0.25;
    double path_ratio = 1.50;
    double path_additive = 0.75;
    double max_path_length = 4.5;
};

struct QueryBridgePartitionPathFirstOptions {
    bool enabled = false;
    bool allow_long = false;
    double max_segment_fraction = 0.95;
};

struct QueryBridgePartitionInitialPathDecision {
    bool segment_reasonable = false;
    bool length_reasonable = false;
    bool accepted = false;
    double direct_distance = 0.0;
    double raw_length = 0.0;
    double segment_fraction = 0.0;
};

struct QueryBridgeRetryOptions {
    bool skip_deferred_short_edges = true;
    int segment_only_retry_attempts = 0;
    int no_path_retry_attempts = 0;
    bool no_path_retry_stop_on_first_success = false;
    int forced_attempts = 1;
    int attempt_offset = 0;
    int rrt_fixed_iters = 0;
    double rrt_fixed_timeout_ms = 0.0;
    double rrt_clearance = 0.0;
    std::vector<double> local_radius_schedule;
    bool local_radius_append_unrestricted_attempt = true;
    int rrt_optimize_after_first_iters = 0;
    int attempt_fallback_paths = 0;
    std::vector<int> no_path_retry_budget_iters;
    std::vector<int> no_path_retry_budget_attempts;
    std::size_t no_path_retry_budget_stages = 0;
    bool post_rrt_skip_forced = false;
};

struct QueryBridgeAttemptPlan {
    bool forced = false;
    bool partition_path_first = false;
    int base_attempts = 1;
    int effective_attempts = 1;
};

struct QueryBridgeParallelRrtOptions {
    bool early_stop = false;
    int early_stop_min_successes = 1;
    double early_stop_ratio = 1.75;
    double early_stop_additive = 0.75;
};

struct QueryBridgeWaypointQualityRetryOptions {
    bool enabled = false;
    int attempts = 4;
    int iters = 0;
    double max_ratio = 2.0;
    double max_additive = 0.75;
};

struct QueryBridgeDirectLineFallbackOptions {
    bool enabled = false;
};

struct QueryBridgeHybridizeAttemptOptions {
    bool enabled = false;
    int max_paths = 8;
    int max_vertices = 128;
    int max_cross_checks = 4096;
};

struct QueryBridgeBatchExecutionOptions {
    bool evaluate_all_fallback_paths = false;
    bool parallel_task_rrt = true;
};

struct QueryBridgeIndexOptions {
    std::string force_indices_csv;
    std::string global_indices_csv;
    std::string segment_only_indices_csv;
};

struct QueryBridgeSearchTask {
    std::size_t index = 0;
    int query_index = 0;
    Eigen::VectorXd start;
    Eigen::VectorXd goal;
    bool short_local_bridge = false;
    RRTConnectConfig bridge_rrt;
    std::vector<RRTConnectConfig> short_local_profiles;
    int attempts = 1;
    std::vector<Eigen::VectorXd> waypoint_path;
    std::vector<std::vector<Eigen::VectorXd>> waypoint_fallback_paths;
    bool waypoint_path_from_partition_query = false;
    std::vector<Eigen::VectorXd> hipac_candidate_path;
    bool hipac_online_satisfied = false;
    bool direct_start_goal_satisfied = false;
    int hipac_prebridge_resolves_used = 0;
    int hipac_transition_resolves_used = 0;
    int hipac_online_resolves_used = 0;
};

struct QueryBridgeSearchJob {
    std::size_t task_index = 0;
    int attempt = 0;
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
    StageContext& context);

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
    StageContext& context);

bool query_bridge_should_check_current_query(
    const QueryBridgeSearchTask& task,
    bool respect_forced,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options);

bool query_bridge_has_segment_only_task(
    const std::vector<QueryBridgeSearchTask>& tasks,
    const QueryBridgeIndexOptions& index_options);

bool query_bridge_parallel_task_rrt_enabled(
    const QueryBridgeBatchExecutionOptions& batch_options,
    bool has_segment_only_task,
    const QueryBridgeRetryOptions& retry_options);

bool query_bridge_task_has_explicit_satisfaction(
    const QueryBridgeSearchTask& task);

QueryBridgeAttemptPlan query_bridge_attempt_plan(
    const QueryBridgeSearchTask& task,
    bool forced,
    const QueryBridgeRetryOptions& options);

QueryBridgeAttemptPlan query_bridge_prepare_attempt_plan(
    const QueryBridgeSearchTask& task,
    const QueryBridgeIndexOptions& index_options,
    const QueryBridgeRetryOptions& retry_options,
    StageContext& context);

QueryBridgeAcceptanceThresholds query_bridge_acceptance_thresholds_from_env();

void record_query_bridge_acceptance_diagnostics(
    StageContext& context,
    const QueryBridgeAcceptanceThresholds& thresholds);

QueryBridgePartitionPathFirstOptions query_bridge_partition_path_first_options_from_env(
    bool partition_native_mode);

QueryBridgePartitionInitialPathDecision query_bridge_partition_initial_path_decision(
    const QueryResult& initial_query,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const QueryBridgeAcceptanceThresholds& thresholds,
    const QueryBridgePartitionPathFirstOptions& options);

void record_query_bridge_partition_path_first_diagnostics(
    StageContext& context,
    const QueryBridgePartitionPathFirstOptions& options);

QueryBridgeRetryOptions query_bridge_retry_options_from_env();

double query_bridge_rrt_clearance_from_env();

void record_query_bridge_retry_diagnostics(StageContext& context,
                                           const QueryBridgeRetryOptions& options);

QueryBridgeParallelRrtOptions query_bridge_parallel_rrt_options_from_env();

void record_query_bridge_parallel_rrt_diagnostics(StageContext& context,
                                                  const QueryBridgeParallelRrtOptions& options);

bool query_bridge_parallel_rrt_path_good_enough(const Eigen::VectorXd& start,
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

QueryBridgeWaypointQualityRetryOptions query_bridge_waypoint_quality_retry_options_from_env();

void record_query_bridge_waypoint_quality_retry_diagnostics(
    StageContext& context,
    const QueryBridgeWaypointQualityRetryOptions& options);

bool query_bridge_waypoint_quality_retry_needed(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    double best_length,
    const QueryBridgeWaypointQualityRetryOptions& options);

QueryBridgeDirectLineFallbackOptions query_bridge_direct_line_fallback_options_from_env();

void record_query_bridge_direct_line_fallback_diagnostics(
    StageContext& context,
    const QueryBridgeDirectLineFallbackOptions& options);

std::vector<Eigen::VectorXd> query_bridge_direct_line_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const QueryBridgeDirectLineFallbackOptions& options,
    StageContext& context);

QueryBridgeHybridizeAttemptOptions query_bridge_hybridize_attempt_options_from_env();

QueryBridgeBatchExecutionOptions query_bridge_batch_execution_options_from_env();

void record_query_bridge_batch_execution_diagnostics(
    StageContext& context,
    const QueryBridgeBatchExecutionOptions& options);

QueryBridgeIndexOptions query_bridge_index_options_from_env();

bool query_bridge_index_forced(const QueryBridgeIndexOptions& options,
                               std::size_t index);

bool query_bridge_index_segment_only(const QueryBridgeIndexOptions& options,
                                     std::size_t index);

int query_bridge_index_global(const QueryBridgeIndexOptions& options,
                              std::size_t position,
                              int fallback);

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after);

std::string query_bridge_task_key(std::size_t index, const std::string& suffix);

void query_bridge_mark_task_skip(BuildProfile& profile,
                                 std::size_t index,
                                 double code,
                                 const char* reason);

void record_query_bridge_partition_path_first_task(StageContext& context,
                                                   std::size_t index);

void record_query_bridge_partition_path_first_rrt_skipped(StageContext& context,
                                                          std::size_t index);

void record_query_bridge_batch_task_no_path(StageContext& context,
                                            std::size_t index,
                                            double total_ms);

void record_query_bridge_batch_task_already_satisfied(
    StageContext& context,
    const QueryBridgeSearchTask& task,
    double probe_ms);

void record_query_bridge_batch_task_skipped_after_rrt(StageContext& context,
                                                      std::size_t index,
                                                      bool forced_task,
                                                      double probe_ms,
                                                      double total_ms);

void record_query_bridge_batch_task_skipped_by_hipac_after_rrt(
    StageContext& context,
    std::size_t index,
    double total_ms);

void record_query_bridge_forced_attempts(StageContext& context,
                                         std::size_t index,
                                         bool forced_task,
                                         int attempts);

void query_bridge_adopt_retry_path_if_better(
    QueryBridgeSearchTask& task,
    std::vector<Eigen::VectorXd> retry_path,
    double& best_length,
    int& retry_successes);

using QueryBridgeRetryPathRunner =
    std::function<std::vector<Eigen::VectorXd>(int attempt, int fixed_iters)>;

void query_bridge_run_segment_only_retry(
    QueryBridgeSearchTask& task,
    int first_attempt,
    double& best_length,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeRetryPathRunner& run_task_attempt,
    StageContext& context);

void query_bridge_run_no_path_retries(
    QueryBridgeSearchTask& task,
    int first_attempt,
    double& best_length,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeRetryPathRunner& run_task_attempt,
    StageContext& context);

int query_bridge_edge_query_index(bool scene_reusable_edges,
                                  const QueryBridgeSearchTask& task);

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds);

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index);

}  // namespace rbf
