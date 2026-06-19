#include "planning_forest_query_bridge_batch_utils.h"

#include <SBF/box_graph.h>

#include "env_config.h"

#include <algorithm>
namespace rbf {

QueryBridgeAcceptanceThresholds query_bridge_acceptance_thresholds_from_env() {
    QueryBridgeAcceptanceThresholds thresholds;
    thresholds.max_segment_fraction = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_SEGMENT_FRACTION", 0.25));
    thresholds.path_ratio =
        std::max(0.0, detail::env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_PATH_RATIO", 1.50));
    thresholds.path_additive = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_PATH_ADDITIVE", 0.75));
    thresholds.max_path_length = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_PATH_LENGTH", 4.5));
    return thresholds;
}

void record_query_bridge_acceptance_diagnostics(
    StageContext& context,
    const QueryBridgeAcceptanceThresholds& thresholds) {
    context.diagnostics().set_value(
        "query_bridge.accept_segment_fraction",
        thresholds.max_segment_fraction);
    context.diagnostics().set_value(
        "query_bridge.accept_path_ratio",
        thresholds.path_ratio);
    context.diagnostics().set_value(
        "query_bridge.accept_path_additive",
        thresholds.path_additive);
    context.diagnostics().set_value(
        "query_bridge.accept_max_path_length",
        thresholds.max_path_length);
}

QueryBridgePartitionPathFirstOptions query_bridge_partition_path_first_options_from_env(
    bool partition_native_mode) {
    QueryBridgePartitionPathFirstOptions options;
    options.enabled =
        partition_native_mode &&
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST", 0) != 0;
    options.allow_long =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST_ALLOW_LONG", 0) != 0;
    options.max_segment_fraction = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST_MAX_SEGMENT_FRACTION",
                                      0.95));
    return options;
}

void record_query_bridge_partition_path_first_diagnostics(
    StageContext& context,
    const QueryBridgePartitionPathFirstOptions& options) {
    context.diagnostics().set_value(
        "query_bridge.partition_path_first",
        options.enabled ? 1.0 : 0.0);
    context.diagnostics().set_value(
        "query_bridge.partition_path_first_allow_long",
        options.allow_long ? 1.0 : 0.0);
    context.diagnostics().set_value(
        "query_bridge.partition_path_first_max_segment_fraction",
        options.max_segment_fraction);
}

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

QueryBridgeDirectLineFallbackOptions query_bridge_direct_line_fallback_options_from_env() {
    QueryBridgeDirectLineFallbackOptions options;
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_DIRECT_LINE_ON_NO_PATH", 0) != 0;
    return options;
}

void record_query_bridge_direct_line_fallback_diagnostics(
    StageContext& context,
    const QueryBridgeDirectLineFallbackOptions& options) {
    context.diagnostics().set_value("query_bridge.direct_line_on_no_path",
                                    options.enabled ? 1.0 : 0.0);
}

QueryBridgeHybridizeAttemptOptions query_bridge_hybridize_attempt_options_from_env() {
    QueryBridgeHybridizeAttemptOptions options;
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS", 0) != 0;
    options.max_paths =
        std::max(2, detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS", 8));
    options.max_vertices =
        std::max(8, detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES", 128));
    options.max_cross_checks =
        std::max(1,
                 detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS",
                                            4096));
    return options;
}

QueryBridgeBatchExecutionOptions query_bridge_batch_execution_options_from_env() {
    QueryBridgeBatchExecutionOptions options;
    options.evaluate_all_fallback_paths =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_EVALUATE_ALL_FALLBACK_PATHS", 0) != 0;
    options.parallel_task_rrt =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_TASK_RRT", 1) != 0;
    return options;
}

void record_query_bridge_batch_execution_diagnostics(
    StageContext& context,
    const QueryBridgeBatchExecutionOptions& options) {
    context.diagnostics().set_value("query_bridge.evaluate_all_fallback_paths",
                                    options.evaluate_all_fallback_paths ? 1.0 : 0.0);
}

}  // namespace rbf
