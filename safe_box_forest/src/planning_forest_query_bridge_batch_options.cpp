#include "planning_forest_query_bridge_batch_utils.h"

#include <SBF/box_graph.h>

#include "env_config.h"

#include <algorithm>
namespace rbf {

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
