#pragma once

#include <SBF/runtime_fwd.h>
#include <SBF/scene_types.h>

#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_rrt_utils.h"
#include "planning_forest_query_bridge_task.h"

#include <Eigen/Core>

#include <vector>

namespace rbf {

struct RBFPlanningConfig;

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
