#pragma once

#include <SBF/connector.h>
#include <SBF/safe_box_forest.h>

#include <Eigen/Core>

#include "planning_forest_query_bridge_rrt_utils.h"

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
