#pragma once

#include <SBF/connector.h>
#include <SBF/safe_box_forest.h>

#include "planning_forest_query_bridge_corridor_utils.h"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace rbf {

void query_bridge_set_task_value(StageContext& context,
                                 int query_index,
                                 const std::string& suffix,
                                 double value);

void query_bridge_apply_waypoint_shortcut(
    std::vector<Eigen::VectorXd>& corridor_path,
    const CollisionChecker& checker,
    const QueryConfig& query_config,
    const QueryBridgeWaypointShortcutOptions& options,
    StageContext& context,
    int query_index);

void query_bridge_apply_internal_simplify(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    std::vector<Eigen::VectorXd>& corridor_path,
    const CollisionChecker& checker,
    const Robot& audit_robot,
    const RRTConnectConfig& connector_rrt,
    const QueryConfig& query_config,
    int rng_seed,
    bool enabled);

} // namespace rbf
