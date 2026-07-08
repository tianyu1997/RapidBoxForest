#pragma once

#include <rbf/core.h>

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace rbf {

class AdaptiveGridPartition;

struct EndpointMainTargetCandidate {
    int box_id = -1;
    Eigen::VectorXd point;
    double dist2 = 0.0;
};

struct EndpointMainSamplePlan {
    std::vector<Eigen::VectorXd> samples;
    int target_sample_index = -1;
    int target_owner = -1;
};

struct EndpointMainTargetSet {
    std::vector<EndpointMainTargetCandidate> targets;
    std::vector<int> target_box_ids;
    int target_limit = 0;
    bool used_partition_index = false;
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

EndpointMainTargetSet endpoint_main_targets_partition_first(
    const AdaptiveGridPartition* partition,
    bool use_partition_index,
    const std::vector<BoxNode>& boxes,
    const std::unordered_map<int, std::size_t>& box_index_by_id,
    const Eigen::VectorXd& point,
    const std::vector<int>& main_island,
    int target_k);

EndpointMainSamplePlan endpoint_main_sample_plan(
    const Eigen::VectorXd& point,
    const EndpointMainTargetCandidate& target,
    double coarse_step,
    double fine_step,
    const std::function<int(const Eigen::VectorXd&)>& main_owner);

}  // namespace rbf
