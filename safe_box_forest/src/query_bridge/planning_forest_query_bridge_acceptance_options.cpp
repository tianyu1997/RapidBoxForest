#include "planning_forest_query_bridge_options.h"

#include <SBF/runtime.h>

#include <SBF/planning_config.h>

#include <algorithm>
#include <limits>

namespace rbf {

QueryBridgeAcceptanceThresholds query_bridge_acceptance_thresholds_from_config(
    const RBFPlanningConfig& config) {
    QueryBridgeAcceptanceThresholds thresholds;
    thresholds.max_segment_fraction = std::max(
        0.0,
        config.query_bridge_accept_segment_fraction);
    thresholds.path_ratio =
        std::max(0.0, config.query_bridge_accept_path_ratio);
    thresholds.path_additive = std::max(
        0.0,
        config.query_bridge_accept_path_additive);
    thresholds.max_path_length = std::max(
        0.0,
        config.query_bridge_accept_max_path_length);
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

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds) {
    if (!current.success || !current.audit_passed) {
        return false;
    }
    const double raw_length =
        current.raw_path_length > 1e-12 ? current.raw_path_length : current.path_length;
    const double segment_fraction =
        raw_length > 1e-12 ? current.segment_edge_length / raw_length
                           : std::numeric_limits<double>::infinity();
    if (!(segment_fraction <= thresholds.max_segment_fraction)) {
        return false;
    }
    const double direct = (goal - start).norm();
    return direct <= 1e-9 ||
           current.path_length <= std::max(direct * thresholds.path_ratio,
                                            direct + thresholds.path_additive) ||
           current.path_length <= thresholds.max_path_length;
}

}  // namespace rbf
