#pragma once

#include <SBF/grower.h>

#include <Eigen/Core>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace rbf {

std::vector<Interval> bounds_for_indices(const std::vector<BoxNode>& boxes,
                                         const std::vector<int>& indices);
Eigen::VectorXd intervals_center(const std::vector<Interval>& intervals);
Eigen::VectorXd closest_point_in_intervals(const std::vector<Interval>& intervals,
                                           const Eigen::Ref<const Eigen::VectorXd>& point);
double interval_bounds_gap_squared(const std::vector<Interval>& lhs,
                                   const std::vector<Interval>& rhs);
Eigen::VectorXd clip_to_root_intervals(const Eigen::Ref<const Eigen::VectorXd>& q,
                                       const std::vector<Interval>& root);
bool clip_intervals_to_root(std::vector<Interval>& intervals,
                            const std::vector<Interval>& root);

struct RootGroups {
    std::unordered_map<int, std::vector<int>> by_root;
    std::vector<int> roots;
};

RootGroups group_boxes_by_root(const std::vector<BoxNode>& boxes);

struct RootComponent {
    int id = -1;
    std::vector<int> roots;
    std::vector<int> indices;
    std::vector<Interval> bounds;
    Eigen::VectorXd center;
};

struct RootComponentGraph {
    RootGroups groups;
    std::unordered_map<int, int> root_to_component;
    std::vector<RootComponent> components;
    int connected_cross_root_pairs = 0;
};

RootComponentGraph build_root_component_graph(const std::vector<BoxNode>& boxes,
                                              double adjacency_tolerance,
                                              bool island_aware);

std::uint64_t component_pair_key(int lhs_root_id, int rhs_root_id);

double normalized_linf_distance(const std::vector<Interval>& root,
                                const Eigen::Ref<const Eigen::VectorXd>& lhs,
                                const Eigen::Ref<const Eigen::VectorXd>& rhs);

int common_ancestor_depth(const BoxOracle& oracle, OracleNodeId lhs_node, OracleNodeId rhs_node);

}  // namespace rbf
