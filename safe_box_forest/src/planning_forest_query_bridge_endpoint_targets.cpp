#include "planning_forest_query_bridge_endpoint_targets.h"

#include "planning_forest_query_utils.h"

#include <algorithm>
#include <utility>

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

EndpointMainTargetSet endpoint_main_targets_partition_first(
    const AdaptiveGridPartition* partition,
    bool use_partition_index,
    const std::vector<BoxNode>& boxes,
    const std::unordered_map<int, std::size_t>& box_index_by_id,
    const Eigen::VectorXd& point,
    const std::vector<int>& main_island,
    int target_k) {
    EndpointMainTargetSet target_set;
    if (use_partition_index && partition != nullptr) {
        target_set.targets = endpoint_main_partition_targets(
            *partition,
            point,
            main_island,
            target_k);
        target_set.used_partition_index = true;
    } else {
        target_set.targets = endpoint_main_graph_targets(boxes,
                                                         box_index_by_id,
                                                         point,
                                                         main_island);
    }
    if (target_set.targets.empty()) {
        return target_set;
    }
    sort_endpoint_main_targets(target_set.targets);
    target_set.target_limit = std::min<int>(
        std::max(1, target_k),
        static_cast<int>(target_set.targets.size()));
    target_set.target_box_ids.reserve(static_cast<std::size_t>(target_set.target_limit));
    for (int item = 0; item < target_set.target_limit; ++item) {
        target_set.target_box_ids.push_back(
            target_set.targets[static_cast<std::size_t>(item)].box_id);
    }
    return target_set;
}

EndpointMainSamplePlan endpoint_main_sample_plan(
    const Eigen::VectorXd& point,
    const EndpointMainTargetCandidate& target,
    double coarse_step,
    double fine_step,
    const std::function<int(const Eigen::VectorXd&)>& main_owner) {
    EndpointMainSamplePlan plan;
    plan.samples = densify_waypoint_path_local({point, target.point},
                                               std::max(1e-4, coarse_step));
    if (plan.samples.size() < 2) {
        return plan;
    }
    plan.target_sample_index = static_cast<int>(plan.samples.size()) - 1;
    plan.target_owner = target.box_id;
    for (int sample_index = 1;
         sample_index < static_cast<int>(plan.samples.size());
         ++sample_index) {
        const int owner = main_owner(plan.samples[static_cast<std::size_t>(sample_index)]);
        if (owner >= 0) {
            plan.target_sample_index = sample_index;
            plan.target_owner = owner;
            break;
        }
    }
    if (plan.target_sample_index > 1 && fine_step > 0.0) {
        std::vector<Eigen::VectorXd> fine =
            densify_waypoint_path_local({plan.samples.front(),
                                         plan.samples[static_cast<std::size_t>(
                                             plan.target_sample_index)]},
                                        std::max(1e-4, fine_step));
        if (fine.size() >= 2) {
            plan.samples = std::move(fine);
            plan.target_sample_index = static_cast<int>(plan.samples.size()) - 1;
        }
    }
    return plan;
}

}  // namespace rbf
