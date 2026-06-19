#include "planning_forest_query_bridge_options.h"

#include "env_config.h"
#include "planning_forest_audit.h"

namespace rbf {

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

std::vector<Eigen::VectorXd> query_bridge_direct_line_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const QueryBridgeDirectLineFallbackOptions& options,
    StageContext& context) {
    if (!options.enabled) {
        return {};
    }
    context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_attempts");
    CollisionChecker checker = make_audit_checker(audit_robot, scene, query_config);
    std::vector<Eigen::VectorXd> path{task.start, task.goal};
    const PathAuditCheck audit =
        audit_waypoint_path(path,
                            checker,
                            query_config.audit_resolution,
                            query_config.audit_segment_step);
    if (!audit.passed) {
        context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_rejects");
        return {};
    }
    context.diagnostics().add_counter("query_bridge.direct_line_on_no_path_successes");
    return path;
}

}  // namespace rbf
