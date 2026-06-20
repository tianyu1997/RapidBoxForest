#pragma once

#include <SBF/planning_config.h>
#include <SBF/runtime.h>
#include <LECTDatabase/sbf/scene.h>

#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <Eigen/Core>

#include <vector>

namespace rbf {

void adopt_query_bridge_waypoint_after_rrt(
    QueryBridgeSearchTask& task,
    std::vector<std::vector<Eigen::VectorXd>>& attempt_paths_for_task,
    double& best_length,
    const QueryBridgeHybridizeAttemptOptions& hybrid_options,
    const Robot& audit_robot,
    const Scene& scene,
    const RBFPlanningConfig& config,
    StageContext& context);

}  // namespace rbf
