#include "virtual_sparse_ffb.h"

#include <SBF/oracle.h>

#include <rbf/lect_database/split_policy.h>

#include <chrono>
#include <cmath>
#include <utility>

namespace rbf::detail {

bool split_policy_supports_virtual_cells(const OracleSplitPolicyDescriptor& descriptor,
                                         int target_depth) {
    if (target_depth < 0) {
        return false;
    }
    if (descriptor.strategy == lect_database::SplitStrategy::AAFKVolumeMin) {
        return static_cast<int>(descriptor.depth_dimensions.size()) > target_depth;
    }
    return descriptor.strategy == lect_database::SplitStrategy::RoundRobin ||
           descriptor.strategy == lect_database::SplitStrategy::WidestRoot;
}

std::optional<VirtualSeedCell> virtual_seed_cell_at_depth(
    BoxOracle& oracle,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    int target_depth) {
    const auto descriptor = oracle.split_policy_descriptor();
    if (!split_policy_supports_virtual_cells(descriptor, target_depth)) {
        return std::nullopt;
    }
    const auto& root = oracle.root_intervals();
    if (root.empty() || seed.size() != static_cast<int>(root.size())) {
        return std::nullopt;
    }
    lect_database::SplitPolicy policy(descriptor);
    Eigen::VectorXd tree_seed = oracle.tree_configuration_for_query(seed);
    std::vector<Interval> intervals = root;
    int changed_dim = -1;
    for (int depth = 0; depth < target_depth; ++depth) {
        const int dim = policy.choose_dimension(root, intervals, depth);
        if (dim < 0 || dim >= static_cast<int>(intervals.size()) ||
            dim >= tree_seed.size()) {
            return std::nullopt;
        }
        const double split_value =
            policy.choose_split_value(intervals[static_cast<std::size_t>(dim)]);
        auto& interval = intervals[static_cast<std::size_t>(dim)];
        if (!(split_value > interval.lo && split_value < interval.hi)) {
            return std::nullopt;
        }
        if (tree_seed[dim] <= split_value) {
            interval.hi = split_value;
        } else {
            interval.lo = split_value;
        }
        changed_dim = dim;
    }
    VirtualSeedCell cell;
    cell.depth = target_depth;
    cell.changed_dim = changed_dim;
    cell.tree_intervals = std::move(intervals);
    cell.query_intervals = oracle.query_intervals_for_node(
        oracle.root_node(),
        cell.tree_intervals,
        seed);
    return cell;
}

std::optional<std::vector<VirtualSeedPathEntry>> virtual_seed_path_to_depth(
    BoxOracle& oracle,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    int target_depth) {
    const auto descriptor = oracle.split_policy_descriptor();
    if (!split_policy_supports_virtual_cells(descriptor, target_depth)) {
        return std::nullopt;
    }
    const auto& root = oracle.root_intervals();
    if (root.empty() || seed.size() != static_cast<int>(root.size())) {
        return std::nullopt;
    }

    lect_database::SplitPolicy policy(descriptor);
    Eigen::VectorXd tree_seed = oracle.tree_configuration_for_query(seed);
    std::vector<Interval> intervals = root;
    std::vector<VirtualSeedPathEntry> path;
    path.reserve(static_cast<std::size_t>(target_depth + 1));
    path.push_back(VirtualSeedPathEntry{0, -1, intervals});

    int changed_dim = -1;
    for (int depth = 0; depth < target_depth; ++depth) {
        const int dim = policy.choose_dimension(root, intervals, depth);
        if (dim < 0 || dim >= static_cast<int>(intervals.size()) ||
            dim >= tree_seed.size()) {
            return std::nullopt;
        }
        const double split_value =
            policy.choose_split_value(intervals[static_cast<std::size_t>(dim)]);
        auto& interval = intervals[static_cast<std::size_t>(dim)];
        if (!(split_value > interval.lo && split_value < interval.hi)) {
            return std::nullopt;
        }
        if (tree_seed[dim] <= split_value) {
            interval.hi = split_value;
        } else {
            interval.lo = split_value;
        }
        changed_dim = dim;
        path.push_back(VirtualSeedPathEntry{depth + 1, changed_dim, intervals});
    }
    return path;
}

bool intervals_equal_with_tolerance(const std::vector<Interval>& lhs,
                                    const std::vector<Interval>& rhs,
                                    double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (std::abs(lhs[i].lo - rhs[i].lo) > tolerance ||
            std::abs(lhs[i].hi - rhs[i].hi) > tolerance) {
            return false;
        }
    }
    return true;
}

std::optional<MaterializedSeedCell> materialize_seed_path_to_depth(
    BoxOracle& oracle,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    int target_depth,
    const OracleSplitOptions& split_options,
    const MaterializeSplitObserver& observe_split) {
    using Clock = std::chrono::steady_clock;
    OracleNodeId node = oracle.root_node();
    int changed_dim = -1;
    int splits = 0;
    std::vector<Interval> tree_intervals = oracle.node_intervals(node);
    while (node != kInvalidOracleNodeId && oracle.depth(node) < target_depth) {
        const int depth = oracle.depth(node);
        if (oracle.is_leaf(node)) {
            const auto split_start = Clock::now();
            const auto split = oracle.split_node(node,
                                                 tree_intervals,
                                                 changed_dim,
                                                 split_options);
            const double split_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - split_start).count();
            observe_split(split, tree_intervals, depth, split_ms);
            if (!split.split) {
                return std::nullopt;
            }
            splits += 1;
        }
        const OracleNodeId parent = node;
        const int split_dim = oracle.split_dim(parent);
        const double split_value = oracle.split_value(parent);
        node = oracle.child_containing_point(parent, seed);
        if (node == kInvalidOracleNodeId) {
            return std::nullopt;
        }
        if (split_dim >= 0 && split_dim < static_cast<int>(tree_intervals.size())) {
            auto& interval = tree_intervals[static_cast<std::size_t>(split_dim)];
            if (node == oracle.left_child(parent)) {
                interval.hi = split_value;
            } else if (node == oracle.right_child(parent)) {
                interval.lo = split_value;
            } else {
                tree_intervals = oracle.node_intervals(node);
            }
        } else {
            tree_intervals = oracle.node_intervals(node);
        }
        changed_dim = split_dim;
    }
    if (node == kInvalidOracleNodeId) {
        return std::nullopt;
    }
    MaterializedSeedCell cell;
    cell.node = node;
    cell.changed_dim = changed_dim;
    cell.splits = splits;
    cell.tree_intervals = std::move(tree_intervals);
    cell.query_intervals = oracle.query_intervals_for_node(node,
                                                           cell.tree_intervals,
                                                           seed);
    return cell;
}

}  // namespace rbf::detail
