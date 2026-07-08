#pragma once

#include <SBF/box_adjacency_types.h>

#include "planning_forest_query_bridge_endpoint_index.h"

#include <rbf/core.h>

#include <Eigen/Core>

#include <cstddef>
#include <unordered_set>
#include <vector>

namespace rbf {

class AdaptiveGridPartition;

struct EndpointMainRuntime {
    std::vector<BoxNode>& boxes;
    AdjacencyGraph& adjacency;
    AdaptiveGridPartition* partition = nullptr;
    EndpointMainIndexes& indexes;
    const std::unordered_set<int>& main_ids;
    std::size_t boxes_before_endpoint_main = 0;
    double adjacency_tolerance = 0.0;
    bool graphless_endpoint_main = false;
    bool use_partition_endpoint_index = false;

    BoxNode* box_by_id(int box_id) const;
    bool contains_point(int box_id, const Eigen::Ref<const Eigen::VectorXd>& q) const;
    bool append_edge_if_connected(int lhs, int rhs, int& local_adj_checks);
    int main_owner(const Eigen::Ref<const Eigen::VectorXd>& q) const;
    int first_existing_cover(const Eigen::Ref<const Eigen::VectorXd>& q) const;
    Eigen::VectorXd make_seed_from_face(int box_id,
                                        const Eigen::Ref<const Eigen::VectorXd>& from,
                                        const Eigen::Ref<const Eigen::VectorXd>& to,
                                        const std::vector<Interval>& planning_intervals,
                                        double face_epsilon) const;
    int furthest_sample(int box_id,
                        const std::vector<Eigen::VectorXd>& samples,
                        int start_index,
                        int target_index) const;
    bool parent_adjacent_to_candidate(int parent_box_id,
                                      const BoxNode& candidate,
                                      int& local_adj_checks) const;
    bool closest_point_for_box(int box_id,
                               const Eigen::Ref<const Eigen::VectorXd>& point,
                               Eigen::VectorXd& closest) const;
    void add_box_to_indexes(const BoxNode& box, std::size_t index);
};

}  // namespace rbf
