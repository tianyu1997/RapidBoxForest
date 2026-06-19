#pragma once

#include <SBF/safe_box_forest.h>

#include "planning_forest_query_bridge_task.h"

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

struct QueryBridgeHybridizeAttemptOptions {
    bool enabled = false;
    int max_paths = 8;
    int max_vertices = 128;
    int max_cross_checks = 4096;
};

struct QueryBridgeBatchExecutionOptions {
    bool parallel_task_rrt = true;
};

struct QueryBridgeIndexOptions {
    std::string force_indices_csv;
    std::string global_indices_csv;
    std::string segment_only_indices_csv;
};

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

QueryBridgeHybridizeAttemptOptions query_bridge_hybridize_attempt_options_from_env();

QueryBridgeBatchExecutionOptions query_bridge_batch_execution_options_from_env();

QueryBridgeIndexOptions query_bridge_index_options_from_env();

bool query_bridge_index_forced(const QueryBridgeIndexOptions& options,
                               std::size_t index);

bool query_bridge_index_segment_only(const QueryBridgeIndexOptions& options,
                                     std::size_t index);

int query_bridge_index_global(const QueryBridgeIndexOptions& options,
                              std::size_t position,
                              int fallback);

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds);

}  // namespace rbf
