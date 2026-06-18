#pragma once

#include <SBF/connector.h>
#include <SBF/safe_box_forest.h>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace rbf {

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

struct QueryBridgeParallelRrtOptions {
    bool early_stop = false;
    int early_stop_min_successes = 1;
    double early_stop_ratio = 1.75;
    double early_stop_additive = 0.75;
};

struct QueryBridgeDetourOptions {
    bool enabled = false;
    bool candidate = false;
    double replace_factor = 1.0;
    int dims = 4;
    int rounds = 2;
    int max_candidates = 32;
    bool multi_axis = false;
    int random_candidates = 0;
    double offset = 0.35;
    double two_bend_alpha = 0.35;
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

QueryBridgeAcceptanceThresholds query_bridge_acceptance_thresholds_from_env();

QueryBridgePartitionPathFirstOptions query_bridge_partition_path_first_options_from_env(
    bool partition_native_mode);

QueryBridgeRetryOptions query_bridge_retry_options_from_env();

void record_query_bridge_retry_diagnostics(StageContext& context,
                                           const QueryBridgeRetryOptions& options);

QueryBridgeParallelRrtOptions query_bridge_parallel_rrt_options_from_env();

void record_query_bridge_parallel_rrt_diagnostics(StageContext& context,
                                                  const QueryBridgeParallelRrtOptions& options);

bool query_bridge_parallel_rrt_path_good_enough(const Eigen::VectorXd& start,
                                                const Eigen::VectorXd& goal,
                                                const std::vector<Eigen::VectorXd>& path,
                                                const QueryBridgeParallelRrtOptions& options);

QueryBridgeDetourOptions query_bridge_detour_options_from_env();

void record_query_bridge_detour_diagnostics(StageContext& context,
                                            const QueryBridgeDetourOptions& options);

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

QueryBridgeHybridizeAttemptOptions query_bridge_hybridize_attempt_options_from_env();

QueryBridgeBatchExecutionOptions query_bridge_batch_execution_options_from_env();

void record_query_bridge_batch_execution_diagnostics(
    StageContext& context,
    const QueryBridgeBatchExecutionOptions& options);

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after);

std::string query_bridge_task_key(std::size_t index, const std::string& suffix);

double query_bridge_point_segment_distance_sq(const Eigen::VectorXd& point,
                                              const Eigen::VectorXd& a,
                                              const Eigen::VectorXd& b);

double query_bridge_point_polyline_distance_sq(
    const Eigen::VectorXd& point,
    const std::vector<Eigen::VectorXd>& path);

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds);

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index);

}  // namespace rbf
