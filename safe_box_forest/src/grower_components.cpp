#include "grower_components.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rbf {

namespace {

struct LocalDisjointSet {
    std::vector<int> parent;
    std::vector<int> rank;

    explicit LocalDisjointSet(int n)
        : parent(static_cast<std::size_t>(n)), rank(static_cast<std::size_t>(n), 0) {
        for (int i = 0; i < n; ++i) {
            parent[static_cast<std::size_t>(i)] = i;
        }
    }

    int find(int value) {
        int& p = parent[static_cast<std::size_t>(value)];
        if (p != value) {
            p = find(p);
        }
        return p;
    }

    void unite(int lhs, int rhs) {
        int left = find(lhs);
        int right = find(rhs);
        if (left == right) {
            return;
        }
        if (rank[static_cast<std::size_t>(left)] < rank[static_cast<std::size_t>(right)]) {
            std::swap(left, right);
        }
        parent[static_cast<std::size_t>(right)] = left;
        if (rank[static_cast<std::size_t>(left)] == rank[static_cast<std::size_t>(right)]) {
            rank[static_cast<std::size_t>(left)] += 1;
        }
    }
};

}  // namespace

std::vector<Interval> bounds_for_indices(const std::vector<BoxNode>& boxes,
                                         const std::vector<int>& indices) {
    std::vector<Interval> bounds;
    if (indices.empty()) {
        return bounds;
    }
    bounds = boxes[static_cast<std::size_t>(indices.front())].joint_intervals;
    for (int outer = 1; outer < static_cast<int>(indices.size()); ++outer) {
        const auto& intervals =
            boxes[static_cast<std::size_t>(indices[static_cast<std::size_t>(outer)])].joint_intervals;
        for (int dim = 0;
             dim < static_cast<int>(bounds.size()) && dim < static_cast<int>(intervals.size());
             ++dim) {
            bounds[static_cast<std::size_t>(dim)].lo =
                std::min(bounds[static_cast<std::size_t>(dim)].lo,
                         intervals[static_cast<std::size_t>(dim)].lo);
            bounds[static_cast<std::size_t>(dim)].hi =
                std::max(bounds[static_cast<std::size_t>(dim)].hi,
                         intervals[static_cast<std::size_t>(dim)].hi);
        }
    }
    return bounds;
}

Eigen::VectorXd intervals_center(const std::vector<Interval>& intervals) {
    Eigen::VectorXd center(static_cast<int>(intervals.size()));
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        center[dim] = intervals[static_cast<std::size_t>(dim)].center();
    }
    return center;
}

Eigen::VectorXd closest_point_in_intervals(const std::vector<Interval>& intervals,
                                           const Eigen::Ref<const Eigen::VectorXd>& point) {
    Eigen::VectorXd closest(static_cast<int>(intervals.size()));
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        closest[dim] = std::clamp(point[dim],
                                  intervals[static_cast<std::size_t>(dim)].lo,
                                  intervals[static_cast<std::size_t>(dim)].hi);
    }
    return closest;
}

double interval_bounds_gap_squared(const std::vector<Interval>& lhs,
                                   const std::vector<Interval>& rhs) {
    if (lhs.size() != rhs.size()) {
        return std::numeric_limits<double>::infinity();
    }
    double gap_sq = 0.0;
    for (int dim = 0; dim < static_cast<int>(lhs.size()); ++dim) {
        double gap = 0.0;
        const auto& left = lhs[static_cast<std::size_t>(dim)];
        const auto& right = rhs[static_cast<std::size_t>(dim)];
        if (left.hi < right.lo) {
            gap = right.lo - left.hi;
        } else if (right.hi < left.lo) {
            gap = left.lo - right.hi;
        }
        gap_sq += gap * gap;
    }
    return gap_sq;
}

Eigen::VectorXd clip_to_root_intervals(const Eigen::Ref<const Eigen::VectorXd>& q,
                                       const std::vector<Interval>& root) {
    Eigen::VectorXd clipped = q;
    if (clipped.size() != static_cast<int>(root.size())) {
        return clipped;
    }
    for (int dim = 0; dim < clipped.size(); ++dim) {
        clipped[dim] = std::clamp(clipped[dim],
                                  root[static_cast<std::size_t>(dim)].lo,
                                  root[static_cast<std::size_t>(dim)].hi);
    }
    return clipped;
}

bool clip_intervals_to_root(std::vector<Interval>& intervals,
                            const std::vector<Interval>& root) {
    if (intervals.size() != root.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
        intervals[dim].lo = std::max(intervals[dim].lo, root[dim].lo);
        intervals[dim].hi = std::min(intervals[dim].hi, root[dim].hi);
        if (intervals[dim].lo > intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

RootGroups group_boxes_by_root(const std::vector<BoxNode>& boxes) {
    RootGroups groups;
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        const int root = boxes[static_cast<std::size_t>(index)].root_id;
        if (root >= 0) {
            groups.by_root[root].push_back(index);
        }
    }
    groups.roots.reserve(groups.by_root.size());
    for (const auto& [root, _] : groups.by_root) {
        groups.roots.push_back(root);
    }
    std::sort(groups.roots.begin(), groups.roots.end());
    return groups;
}

RootComponentGraph build_root_component_graph(const std::vector<BoxNode>& boxes,
                                              double adjacency_tolerance,
                                              bool island_aware) {
    RootComponentGraph graph;
    graph.groups = group_boxes_by_root(boxes);
    if (graph.groups.roots.empty()) {
        return graph;
    }

    std::unordered_map<int, int> root_to_ordinal;
    root_to_ordinal.reserve(graph.groups.roots.size());
    for (int ordinal = 0; ordinal < static_cast<int>(graph.groups.roots.size()); ++ordinal) {
        root_to_ordinal[graph.groups.roots[static_cast<std::size_t>(ordinal)]] = ordinal;
    }

    LocalDisjointSet dsu(static_cast<int>(graph.groups.roots.size()));
    if (island_aware) {
        for (int lhs_index = 0; lhs_index < static_cast<int>(boxes.size()); ++lhs_index) {
            const BoxNode& lhs = boxes[static_cast<std::size_t>(lhs_index)];
            if (lhs.root_id < 0) {
                continue;
            }
            for (int rhs_index = lhs_index + 1; rhs_index < static_cast<int>(boxes.size()); ++rhs_index) {
                const BoxNode& rhs = boxes[static_cast<std::size_t>(rhs_index)];
                if (rhs.root_id < 0 || rhs.root_id == lhs.root_id) {
                    continue;
                }
                if (!boxes_connected(lhs, rhs, adjacency_tolerance)) {
                    continue;
                }
                dsu.unite(root_to_ordinal.at(lhs.root_id), root_to_ordinal.at(rhs.root_id));
                graph.connected_cross_root_pairs += 1;
            }
        }
    }

    std::unordered_map<int, int> dsu_to_component;
    for (int root : graph.groups.roots) {
        const int dsu_root = dsu.find(root_to_ordinal.at(root));
        auto [component_it, inserted] =
            dsu_to_component.emplace(dsu_root, static_cast<int>(graph.components.size()));
        if (inserted) {
            RootComponent component;
            component.id = component_it->second;
            graph.components.push_back(std::move(component));
        }
        const int component_index = component_it->second;
        graph.root_to_component[root] = component_index;
        RootComponent& component = graph.components[static_cast<std::size_t>(component_index)];
        component.roots.push_back(root);
        const auto group_it = graph.groups.by_root.find(root);
        if (group_it != graph.groups.by_root.end()) {
            component.indices.insert(component.indices.end(),
                                     group_it->second.begin(),
                                     group_it->second.end());
        }
    }

    for (RootComponent& component : graph.components) {
        std::sort(component.roots.begin(), component.roots.end());
        component.bounds = bounds_for_indices(boxes, component.indices);
        component.center = intervals_center(component.bounds);
    }
    return graph;
}

std::uint64_t component_pair_key(int lhs_root_id, int rhs_root_id) {
    const std::uint32_t lo = static_cast<std::uint32_t>(std::min(lhs_root_id, rhs_root_id));
    const std::uint32_t hi = static_cast<std::uint32_t>(std::max(lhs_root_id, rhs_root_id));
    return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
}

double normalized_linf_distance(const std::vector<Interval>& root,
                                const Eigen::Ref<const Eigen::VectorXd>& lhs,
                                const Eigen::Ref<const Eigen::VectorXd>& rhs) {
    if (lhs.size() != rhs.size() || lhs.size() != static_cast<int>(root.size())) {
        return 0.0;
    }
    double distance = 0.0;
    for (int dim = 0; dim < lhs.size(); ++dim) {
        const double width = std::max(root[static_cast<std::size_t>(dim)].width(), 1e-12);
        distance = std::max(distance, std::abs(lhs[dim] - rhs[dim]) / width);
    }
    return distance;
}

int common_ancestor_depth(const BoxOracle& oracle, OracleNodeId lhs_node, OracleNodeId rhs_node) {
    return oracle.common_ancestor_depth(lhs_node, rhs_node);
}

}  // namespace rbf
