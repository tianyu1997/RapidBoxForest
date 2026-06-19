#include "planning_forest_query_bridge_batch_utils.h"

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

}  // namespace rbf
