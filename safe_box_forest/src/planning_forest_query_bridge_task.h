#pragma once

#include <SBF/connector.h>

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace rbf {

struct QueryBridgeSearchTask {
    std::size_t index = 0;
    int query_index = 0;
    Eigen::VectorXd start;
    Eigen::VectorXd goal;
    bool short_local_bridge = false;
    RRTConnectConfig bridge_rrt;
    std::vector<RRTConnectConfig> short_local_profiles;
    int attempts = 1;
    std::vector<Eigen::VectorXd> waypoint_path;
    std::vector<std::vector<Eigen::VectorXd>> waypoint_fallback_paths;
    bool waypoint_path_from_partition_query = false;
    std::vector<Eigen::VectorXd> hipac_candidate_path;
    bool hipac_online_satisfied = false;
    bool direct_start_goal_satisfied = false;
    int hipac_prebridge_resolves_used = 0;
    int hipac_transition_resolves_used = 0;
    int hipac_online_resolves_used = 0;
};

struct QueryBridgeSearchJob {
    std::size_t task_index = 0;
    int attempt = 0;
};

}  // namespace rbf
