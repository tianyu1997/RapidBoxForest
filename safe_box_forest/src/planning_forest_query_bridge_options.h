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

struct QueryBridgeHybridizeAttemptOptions {
    bool enabled = false;
    int max_paths = 8;
    int max_vertices = 128;
    int max_cross_checks = 4096;
};

struct QueryBridgeIndexOptions {
    std::string force_indices_csv;
    std::string global_indices_csv;
};

QueryBridgeAcceptanceThresholds query_bridge_acceptance_thresholds_from_env();

void record_query_bridge_acceptance_diagnostics(
    StageContext& context,
    const QueryBridgeAcceptanceThresholds& thresholds);

QueryBridgeHybridizeAttemptOptions query_bridge_hybridize_attempt_options_from_env();

QueryBridgeIndexOptions query_bridge_index_options_from_env();

bool query_bridge_index_forced(const QueryBridgeIndexOptions& options,
                               std::size_t index);

int query_bridge_index_global(const QueryBridgeIndexOptions& options,
                              std::size_t position,
                              int fallback);

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds);

}  // namespace rbf
