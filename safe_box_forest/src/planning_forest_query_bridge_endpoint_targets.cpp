#include "planning_forest_query_bridge_endpoint_targets.h"

#include "planning_forest_query_utils.h"

#include <algorithm>

namespace rbf {

std::vector<EndpointMainTargetCandidate> endpoint_main_partition_targets(
    const AdaptiveGridPartition& partition,
    const Eigen::VectorXd& point,
    const std::vector<int>& main_island,
    int target_k) {
    const auto nearest = partition.nearest_boxes(
        point,
        main_island,
        std::max(1, target_k));
    std::vector<EndpointMainTargetCandidate> targets;
    targets.reserve(nearest.size());
    for (const auto& item : nearest) {
        Eigen::VectorXd target_point = item.closest_point;
        double dist2 = item.distance_sq;
        if (dist2 <= 1e-18) {
            Eigen::VectorXd center;
            if (partition.center_for_box(item.box_id, center) &&
                center.size() == point.size()) {
                target_point = std::move(center);
                dist2 = (target_point - point).squaredNorm();
            }
        }
        targets.push_back({item.box_id, std::move(target_point), dist2});
    }
    return targets;
}

std::vector<EndpointMainTargetCandidate> endpoint_main_graph_targets(
    const std::vector<BoxNode>& boxes,
    const std::unordered_map<int, std::size_t>& box_index_by_id,
    const Eigen::VectorXd& point,
    const std::vector<int>& main_island) {
    std::vector<EndpointMainTargetCandidate> targets;
    targets.reserve(main_island.size());
    for (int box_id : main_island) {
        const auto box_it = box_index_by_id.find(box_id);
        if (box_it == box_index_by_id.end() ||
            box_it->second >= boxes.size()) {
            continue;
        }
        const BoxNode& box = boxes[box_it->second];
        if (box.n_dims() != point.size()) {
            continue;
        }
        Eigen::VectorXd target_point = closest_point_in_box(box, point);
        double dist2 = (target_point - point).squaredNorm();
        if (dist2 <= 1e-18) {
            target_point = box.center();
            dist2 = (target_point - point).squaredNorm();
        }
        targets.push_back({box_id, std::move(target_point), dist2});
    }
    return targets;
}

void sort_endpoint_main_targets(std::vector<EndpointMainTargetCandidate>& targets) {
    std::sort(targets.begin(),
              targets.end(),
              [](const EndpointMainTargetCandidate& lhs,
                 const EndpointMainTargetCandidate& rhs) {
                  return lhs.dist2 < rhs.dist2;
              });
}

}  // namespace rbf
