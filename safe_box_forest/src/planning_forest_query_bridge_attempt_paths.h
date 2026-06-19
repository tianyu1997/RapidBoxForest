#pragma once

#include <SBF/safe_box_forest.h>

#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <Eigen/Core>

#include <vector>

namespace rbf {

struct QueryBridgeDetourOptions;

void improve_query_bridge_waypoint_if_needed(
    QueryBridgeSearchTask& task,
    int attempts_already_used,
    double& best_length,
    std::vector<Eigen::VectorXd>& waypoint_path,
    const QueryBridgeWaypointQualityRetryOptions& quality_retry_options,
    const QueryBridgeRetryOptions& retry_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context);

void adopt_query_bridge_waypoint_after_rrt(
    QueryBridgeSearchTask& task,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths_for_task,
    int improve_attempts,
    double& best_length,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const QueryBridgeRetryOptions& retry_options,
    const QueryBridgeDirectLineFallbackOptions& direct_line_options,
    const QueryBridgeDetourOptions& detour_options,
    const QueryBridgeWaypointQualityRetryOptions& quality_retry_options,
    const std::vector<Interval>& detour_planning_domain,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context);

}  // namespace rbf
