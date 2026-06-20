#pragma once

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>

#include <Eigen/Core>

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace rbf {

struct EndpointMainTargetCandidate {
    int box_id = -1;
    Eigen::VectorXd point;
    double dist2 = 0.0;
};

std::vector<EndpointMainTargetCandidate> endpoint_main_partition_targets(
    const AdaptiveGridPartition& partition,
    const Eigen::VectorXd& point,
    const std::vector<int>& main_island,
    int target_k);

std::vector<EndpointMainTargetCandidate> endpoint_main_graph_targets(
    const std::vector<BoxNode>& boxes,
    const std::unordered_map<int, std::size_t>& box_index_by_id,
    const Eigen::VectorXd& point,
    const std::vector<int>& main_island);

void sort_endpoint_main_targets(std::vector<EndpointMainTargetCandidate>& targets);

}  // namespace rbf
