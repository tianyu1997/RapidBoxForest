#pragma once

#include <SBF/connector.h>
#include <SBF/safe_box_forest.h>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace rbf {

struct QueryBridgeAcceptanceThresholds {
    double max_segment_fraction = 0.25;
    double path_ratio = 1.50;
    double path_additive = 0.75;
    double max_path_length = 4.5;
};

struct QueryBridgePartitionPathFirstOptions {
    bool enabled = false;
    bool allow_long = false;
    double max_segment_fraction = 0.95;
};

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

QueryBridgeAcceptanceThresholds query_bridge_acceptance_thresholds_from_env();

QueryBridgePartitionPathFirstOptions query_bridge_partition_path_first_options_from_env(
    bool partition_native_mode);

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after);

std::string query_bridge_task_key(std::size_t index, const std::string& suffix);

double query_bridge_point_segment_distance_sq(const Eigen::VectorXd& point,
                                              const Eigen::VectorXd& a,
                                              const Eigen::VectorXd& b);

double query_bridge_point_polyline_distance_sq(
    const Eigen::VectorXd& point,
    const std::vector<Eigen::VectorXd>& path);

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds);

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index);

}  // namespace rbf
