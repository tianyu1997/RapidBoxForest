#pragma once

#include <Eigen/Core>

#include <vector>

namespace rbf {

struct AdaptiveGridPartitionComponentPair;
class AdaptiveGridPartition;
struct AdaptiveLeafSweepConfig;
struct QueryBridgeSearchTask;

struct QueryBridgeHipacPrebridgeSelection {
    int candidate_index = -1;
    int considered = 0;
    int distance_rejects = 0;
    int endpoint_component_rejects = 0;
    int start_component = -1;
    int goal_component = -1;
    double score = 0.0;
};

struct QueryBridgeHipacPrebridgeGate {
    bool enabled = false;
    int candidate_limit = 1;
    double max_pair_distance = 0.0;
    double route_weight = 0.0;
    double pair_weight = 0.0;
};

struct QueryBridgeHipacOnlineGate {
    bool enabled = false;
    int resolve_cap = 0;
    double candidate_max_length = 0.0;
};

QueryBridgeHipacPrebridgeSelection query_bridge_select_hipac_prebridge_pair(
    const std::vector<AdaptiveGridPartitionComponentPair>& candidate_pairs,
    const std::vector<std::vector<int>>& components,
    int start_box_id,
    int goal_box_id,
    const std::vector<Eigen::VectorXd>& coarse_route,
    double max_pair_distance,
    double route_weight,
    double pair_weight);

QueryBridgeHipacPrebridgeGate query_bridge_hipac_prebridge_gate(
    const AdaptiveLeafSweepConfig& config,
    bool partition_native,
    bool adaptive_partition_query_enabled,
    bool adaptive_partition_ready,
    int resolves_used);

QueryBridgeHipacOnlineGate query_bridge_hipac_online_gate(
    const AdaptiveLeafSweepConfig& config,
    bool partition_native,
    int candidate_path_size,
    int resolves_used);

bool query_bridge_hipac_after_rrt_available(
    const AdaptiveLeafSweepConfig& config,
    const QueryBridgeSearchTask& task);

}  // namespace rbf
