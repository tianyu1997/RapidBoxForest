#include "planning_forest_query_bridge_endpoint_runtime.h"

#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

#include <algorithm>

namespace rbf {

BoxNode* EndpointMainRuntime::box_by_id(int box_id) const {
    auto it = indexes.box_index_by_id.find(box_id);
    if (it == indexes.box_index_by_id.end() || it->second >= boxes.size()) {
        return nullptr;
    }
    return &boxes[it->second];
}

bool EndpointMainRuntime::contains_point(int box_id,
                                         const Eigen::Ref<const Eigen::VectorXd>& q) const {
    if (use_partition_endpoint_index &&
        partition != nullptr &&
        partition->box_contains_point(box_id, q, adjacency_tolerance)) {
        return true;
    }
    const BoxNode* box = box_by_id(box_id);
    return box != nullptr && box->contains(q, adjacency_tolerance);
}

bool EndpointMainRuntime::append_edge_if_connected(int lhs, int rhs, int& local_adj_checks) {
    if (lhs == rhs) {
        return true;
    }
    if (use_partition_endpoint_index &&
        partition != nullptr &&
        partition->contains_box_id(lhs) &&
        partition->contains_box_id(rhs)) {
        local_adj_checks += 1;
        return partition->boxes_are_neighbors(lhs, rhs);
    }
    BoxNode* lhs_box = box_by_id(lhs);
    BoxNode* rhs_box = box_by_id(rhs);
    if (lhs_box == nullptr || rhs_box == nullptr) {
        return false;
    }
    local_adj_checks += 1;
    if (!boxes_connected(*lhs_box, *rhs_box, adjacency_tolerance)) {
        return false;
    }
    if (!graphless_endpoint_main) {
        append_local_edge(adjacency, lhs, rhs);
    }
    return true;
}

int EndpointMainRuntime::main_owner(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    if (use_partition_endpoint_index && partition != nullptr) {
        const auto ids = partition->covering_box_ids(q, adjacency_tolerance);
        for (int box_id : ids) {
            if (main_ids.find(box_id) != main_ids.end()) {
                return box_id;
            }
        }
        return -1;
    }
    auto candidates = indexes.main_box_index.point_candidates(q);
    if (candidates.empty()) {
        candidates.reserve(indexes.main_boxes.size());
        for (int index = 0; index < static_cast<int>(indexes.main_boxes.size()); ++index) {
            candidates.push_back(index);
        }
    }
    for (int index : candidates) {
        if (index < 0 || index >= static_cast<int>(indexes.main_boxes.size())) {
            continue;
        }
        const BoxNode& box = indexes.main_boxes[static_cast<std::size_t>(index)];
        if (box.contains(q, adjacency_tolerance)) {
            return indexes.main_box_ids[static_cast<std::size_t>(index)];
        }
    }
    return -1;
}

int EndpointMainRuntime::first_existing_cover(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    if (use_partition_endpoint_index && partition != nullptr) {
        const auto ids = partition->covering_box_ids(q, adjacency_tolerance);
        if (!ids.empty()) {
            return ids.front();
        }
        for (std::size_t index = boxes_before_endpoint_main; index < boxes.size(); ++index) {
            if (intervals_contain_point_local(boxes[index].joint_intervals, q, adjacency_tolerance)) {
                return boxes[index].id;
            }
        }
        return -1;
    }
    const int index = indexes.all_box_index.covering_box(boxes, q, adjacency_tolerance);
    if (index >= 0 && index < static_cast<int>(boxes.size())) {
        return boxes[static_cast<std::size_t>(index)].id;
    }
    return -1;
}

Eigen::VectorXd EndpointMainRuntime::make_seed_from_face(
    int box_id,
    const Eigen::Ref<const Eigen::VectorXd>& from,
    const Eigen::Ref<const Eigen::VectorXd>& to,
    const std::vector<Interval>& planning_intervals,
    double face_epsilon) const {
    std::vector<Interval> intervals;
    if (!(use_partition_endpoint_index &&
          partition != nullptr &&
          partition->intervals_for_box(box_id, intervals))) {
        const BoxNode* box = box_by_id(box_id);
        if (box == nullptr) {
            return from;
        }
        intervals = box->joint_intervals;
    }
    return boundary_seed_from_intervals(intervals,
                                        from,
                                        to,
                                        planning_intervals,
                                        face_epsilon);
}

int EndpointMainRuntime::furthest_sample(int box_id,
                                         const std::vector<Eigen::VectorXd>& samples,
                                         int start_index,
                                         int target_index) const {
    int best = std::max(0, start_index);
    for (int index = best; index <= target_index; ++index) {
        if (contains_point(box_id, samples[static_cast<std::size_t>(index)])) {
            best = index;
        }
    }
    return best;
}

bool EndpointMainRuntime::parent_adjacent_to_candidate(int parent_box_id,
                                                       const BoxNode& candidate,
                                                       int& local_adj_checks) const {
    if (use_partition_endpoint_index &&
        partition != nullptr &&
        partition->contains_box_id(parent_box_id)) {
        local_adj_checks += 1;
        return partition->box_adjacent_to_box(parent_box_id, candidate, adjacency_tolerance);
    }
    const BoxNode* parent_box = box_by_id(parent_box_id);
    if (parent_box == nullptr) {
        return false;
    }
    local_adj_checks += 1;
    return boxes_connected(*parent_box, candidate, adjacency_tolerance);
}

bool EndpointMainRuntime::closest_point_for_box(int box_id,
                                                const Eigen::Ref<const Eigen::VectorXd>& point,
                                                Eigen::VectorXd& closest) const {
    if (use_partition_endpoint_index &&
        partition != nullptr &&
        partition->closest_point_for_box(box_id, point, closest)) {
        return true;
    }
    const BoxNode* box = box_by_id(box_id);
    if (box == nullptr) {
        return false;
    }
    closest = closest_point_in_box(*box, point);
    return true;
}

void EndpointMainRuntime::add_box_to_indexes(const BoxNode& box, std::size_t index) {
    indexes.box_index_by_id[box.id] = index;
    if (!use_partition_endpoint_index) {
        indexes.all_box_index.add_box(box, static_cast<int>(index), adjacency_tolerance);
    } else if (partition != nullptr) {
        partition->append_box(box, adjacency_tolerance);
    }
}

}  // namespace rbf
