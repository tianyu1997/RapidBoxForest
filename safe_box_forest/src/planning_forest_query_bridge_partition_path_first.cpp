#include "planning_forest_query_bridge_batch_utils.h"

#include "env_config.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

QueryBridgePartitionInitialPathDecision query_bridge_partition_initial_path_decision(
    const QueryResult& initial_query,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const QueryBridgeAcceptanceThresholds& thresholds,
    const QueryBridgePartitionPathFirstOptions& options) {
    QueryBridgePartitionInitialPathDecision decision;
    decision.direct_distance = (goal - start).norm();
    decision.raw_length =
        initial_query.raw_path_length > 1e-12
            ? initial_query.raw_path_length
            : initial_query.path_length;
    decision.segment_fraction =
        decision.raw_length > 1e-12
            ? initial_query.segment_edge_length / decision.raw_length
            : std::numeric_limits<double>::infinity();
    decision.segment_reasonable =
        std::isfinite(decision.segment_fraction) &&
        decision.segment_fraction <= options.max_segment_fraction;
    decision.length_reasonable =
        decision.direct_distance <= 1e-9 ||
        initial_query.path_length <=
            std::max(decision.direct_distance * thresholds.path_ratio,
                     decision.direct_distance + thresholds.path_additive) ||
        initial_query.path_length <= thresholds.max_path_length;
    decision.accepted =
        decision.segment_reasonable &&
        (decision.length_reasonable || options.allow_long);
    return decision;
}

}  // namespace rbf
