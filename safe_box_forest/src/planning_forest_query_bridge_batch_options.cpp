#include "planning_forest_query_bridge_batch_utils.h"

#include <SBF/box_graph.h>

#include "env_config.h"

#include <algorithm>
#include <string>

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

double query_bridge_rrt_clearance_from_env() {
    return std::max(0.0,
                    detail::env_double_or_default("RBF_QUERY_BRIDGE_RRT_CLEARANCE", 0.0));
}

QueryBridgeRetryOptions query_bridge_retry_options_from_env() {
    QueryBridgeRetryOptions options;
    options.skip_deferred_short_edges =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_SKIP_DEFERRED_SHORT", 1) != 0;
    options.segment_only_retry_attempts =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_SEGMENT_ONLY_RETRY_ATTEMPTS", 1));
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
    options.rrt_fixed_timeout_ms = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS", 0.0));
    options.rrt_clearance = query_bridge_rrt_clearance_from_env();
    options.local_radius_schedule =
        detail::env_double_list_or_empty("RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE");
    options.local_radius_append_unrestricted_attempt =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_LOCAL_RADIUS_APPEND_UNRESTRICTED_ATTEMPT",
                                   1) != 0;
    options.rrt_optimize_after_first_iters = std::max(
        0,
        detail::env_int_or_default("RBF_QUERY_BRIDGE_RRT_OPTIMIZE_AFTER_FIRST_ITERS", 0));
    options.attempt_fallback_paths =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_ATTEMPT_FALLBACK_PATHS", 0));
    options.no_path_retry_budget_iters =
        detail::env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ITERS");
    options.no_path_retry_budget_attempts =
        detail::env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ATTEMPTS");
    options.no_path_retry_budget_stages =
        std::min(options.no_path_retry_budget_iters.size(),
                 options.no_path_retry_budget_attempts.size());
    options.post_rrt_skip_forced =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_POST_RRT_SKIP_FORCED", 0) != 0;
    return options;
}

void record_query_bridge_retry_diagnostics(StageContext& context,
                                           const QueryBridgeRetryOptions& options) {
    context.diagnostics().set_value("query_bridge.skip_deferred_short_edges",
                                    options.skip_deferred_short_edges ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.segment_only_retry_attempts_default",
                                    static_cast<double>(options.segment_only_retry_attempts));
    context.diagnostics().set_value("query_bridge.no_path_retry_attempts_default",
                                    static_cast<double>(options.no_path_retry_attempts));
    context.diagnostics().set_value("query_bridge.no_path_retry_stop_on_first_success",
                                    options.no_path_retry_stop_on_first_success ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.rrt_fixed_iters",
                                    static_cast<double>(options.rrt_fixed_iters));
    context.diagnostics().set_value("query_bridge.rrt_fixed_timeout_ms",
                                    options.rrt_fixed_timeout_ms);
    context.diagnostics().set_value("query_bridge.rrt_clearance",
                                    options.rrt_clearance);
    context.diagnostics().set_value("query_bridge.local_radius_schedule_size",
                                    static_cast<double>(options.local_radius_schedule.size()));
    context.diagnostics().set_value("query_bridge.local_radius_append_unrestricted_attempt",
                                    options.local_radius_append_unrestricted_attempt ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.rrt_optimize_after_first_iters",
                                    static_cast<double>(options.rrt_optimize_after_first_iters));
    context.diagnostics().set_value("query_bridge.attempt_fallback_paths",
                                    static_cast<double>(options.attempt_fallback_paths));
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
    context.diagnostics().set_value("query_bridge.post_rrt_skip_forced",
                                    options.post_rrt_skip_forced ? 1.0 : 0.0);
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

QueryBridgeIndexOptions query_bridge_index_options_from_env() {
    QueryBridgeIndexOptions options;
    options.force_indices_csv = detail::env_string_or_empty("RBF_QUERY_BRIDGE_FORCE_INDICES");
    options.global_indices_csv = detail::env_string_or_empty("RBF_QUERY_BRIDGE_GLOBAL_INDICES");
    options.segment_only_indices_csv =
        detail::env_string_or_empty("RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES");
    return options;
}

bool query_bridge_index_forced(const QueryBridgeIndexOptions& options,
                               std::size_t index) {
    return detail::csv_nonnegative_index_contains(options.force_indices_csv, index);
}

bool query_bridge_index_segment_only(const QueryBridgeIndexOptions& options,
                                     std::size_t index) {
    return detail::csv_nonnegative_index_contains(options.segment_only_indices_csv, index);
}

int query_bridge_index_global(const QueryBridgeIndexOptions& options,
                              std::size_t position,
                              int fallback) {
    return detail::csv_position_int_or_default(options.global_indices_csv, position, fallback);
}

}  // namespace rbf
