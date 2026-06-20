#include <LECTDatabase/sbf/oracle.h>

#include "oracle_canonical.h"
#include "oracle_options.h"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rbf {
namespace {

lect_database::NodeId to_database_node(OracleNodeId node) {
    if (node < 0 || static_cast<std::uint64_t>(node) > std::numeric_limits<lect_database::NodeId>::max()) {
        return lect_database::kInvalidNodeId;
    }
    return static_cast<lect_database::NodeId>(node);
}

OracleNodeId from_database_node(lect_database::NodeId node) {
    return lect_database::valid_node_id(node) ? static_cast<OracleNodeId>(node) : kInvalidOracleNodeId;
}

}  // namespace

int DatabaseBoxOracle::n_dims() const {
    return static_cast<int>(database_.root_intervals().size());
}

int DatabaseBoxOracle::max_tree_depth() const {
    return database_.max_tree_depth();
}

const std::vector<Interval>& DatabaseBoxOracle::root_intervals() const {
    return database_.root_intervals();
}

std::vector<Interval> DatabaseBoxOracle::node_intervals(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        auto box = online_cache_->node_intervals(to_database_node(node));
        return box ? std::move(*box) : database_.root_intervals();
    }
    auto box = database_.node_box(to_database_node(node));
    return box ? std::move(*box) : database_.root_intervals();
}

Eigen::VectorXd DatabaseBoxOracle::tree_configuration_for_query(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    Eigen::VectorXd tree_q = q;
    if (active_tree_is_primary_canonical_sector(robot_, database_) && tree_q.size() > 0) {
        auto symmetry = primary_database_symmetry(robot_, database_);
        if (symmetry && symmetry->joint_index >= 0 &&
            symmetry->joint_index < tree_q.size()) {
            const auto [sector, canonical_value] =
                canonicalize_value_no_snap(*symmetry, tree_q[symmetry->joint_index]);
            (void)sector;
            tree_q[symmetry->joint_index] = canonical_value;
        }
    }
    return tree_q;
}

OracleNodeId DatabaseBoxOracle::child_containing_point(OracleNodeId node,
                                                       const Eigen::Ref<const Eigen::VectorXd>& q) const {
    if (node < 0 || is_leaf(node) || q.size() != n_dims()) {
        return kInvalidOracleNodeId;
    }
    const Eigen::VectorXd tree_q = tree_configuration_for_query(q);
    const int dim = split_dim(node);
    if (dim < 0 || dim >= tree_q.size()) {
        return kInvalidOracleNodeId;
    }
    return tree_q[dim] <= split_value(node) ? left_child(node) : right_child(node);
}

std::vector<std::vector<Interval>> DatabaseBoxOracle::native_interval_copies_for_node(
    OracleNodeId node,
    const std::vector<Interval>& tree_intervals) const {
    (void)node;
    auto symmetry = primary_database_symmetry(robot_, database_);
    if (!symmetry || !active_tree_is_primary_canonical_sector(robot_, database_) ||
        tree_intervals.empty()) {
        return {tree_intervals};
    }
    const auto& limits = robot_.joint_limits().limits;
    const std::size_t joint_index = static_cast<std::size_t>(symmetry->joint_index);
    if (joint_index >= tree_intervals.size() || joint_index >= limits.size()) {
        return {tree_intervals};
    }
    std::vector<std::vector<Interval>> copies;
    copies.reserve(4);
    for (lect_database::SectorId sector = 0; sector < 4; ++sector) {
        std::vector<Interval> native_intervals = tree_intervals;
        const double reference_value = tree_intervals[joint_index].center() +
            static_cast<double>(normalize_sector(sector)) * symmetry->period;
        native_intervals[joint_index] = map_canonical_interval_to_sector(
            *symmetry,
            tree_intervals[joint_index],
            sector,
            limits[joint_index],
            reference_value);
        if (native_intervals[joint_index].hi < limits[joint_index].lo - 1e-12 ||
            native_intervals[joint_index].lo > limits[joint_index].hi + 1e-12) {
            continue;
        }
        native_intervals[joint_index].lo =
            std::max(native_intervals[joint_index].lo, limits[joint_index].lo);
        native_intervals[joint_index].hi =
            std::min(native_intervals[joint_index].hi, limits[joint_index].hi);
        if (native_intervals[joint_index].lo <= native_intervals[joint_index].hi + 1e-12) {
            copies.push_back(std::move(native_intervals));
        }
    }
    return copies.empty() ? std::vector<std::vector<Interval>>{tree_intervals} : copies;
}

std::vector<Interval> DatabaseBoxOracle::planning_intervals() const {
    const auto& coverage = database_.coverage_intervals();
    return coverage.empty() ? native_root_hull() : coverage;
}

std::vector<Interval> DatabaseBoxOracle::query_intervals_for_node(OracleNodeId node,
                                                                  const std::vector<Interval>& tree_intervals,
                                                                  const Eigen::Ref<const Eigen::VectorXd>& q) const {
    (void)node;
    auto symmetry = primary_database_symmetry(robot_, database_);
    if (!symmetry || !active_tree_is_primary_canonical_sector(robot_, database_) ||
        tree_intervals.empty() || q.size() <= symmetry->joint_index) {
        return tree_intervals;
    }
    const auto& limits = robot_.joint_limits().limits;
    const std::size_t joint_index = static_cast<std::size_t>(symmetry->joint_index);
    if (joint_index >= tree_intervals.size() || joint_index >= limits.size()) {
        return tree_intervals;
    }
    double canonical_value = q[static_cast<int>(joint_index)];
    const double seed_value = q[static_cast<int>(joint_index)];
    lect_database::SectorId sector =
        sector_for_reflected_interval_containing_seed(*symmetry,
                                                      tree_intervals[joint_index],
                                                      limits[joint_index],
                                                      seed_value)
            .value_or(interval_sector_for_value(*symmetry, canonical_value));
    std::vector<Interval> query_intervals = tree_intervals;
    query_intervals[joint_index] = map_canonical_interval_to_sector(
        *symmetry,
        tree_intervals[joint_index],
        sector,
        limits[joint_index],
        q[static_cast<int>(joint_index)]);
    if (!query_intervals[joint_index].contains(seed_value, 1e-9)) {
        counters_.canonical_reflected_seed_misses += 1;
        if (oracle_canonical_debug_enabled()) {
            std::fprintf(stderr,
                         "[CANONICAL] reflected interval misses seed: seed=%.17g sector=%d interval=[%.17g, %.17g] tree=[%.17g, %.17g]\n",
                         seed_value,
                         static_cast<int>(normalize_sector(sector)),
                         query_intervals[joint_index].lo,
                         query_intervals[joint_index].hi,
                         tree_intervals[joint_index].lo,
                         tree_intervals[joint_index].hi);
        }
        throw std::runtime_error(
            "canonical reflected native interval does not contain the query seed");
    }
    return query_intervals;
}

bool DatabaseBoxOracle::contains_point(OracleNodeId node, const Eigen::Ref<const Eigen::VectorXd>& q) const {
    if (active_tree_is_primary_canonical_sector(robot_, database_) &&
        q.size() == static_cast<int>(n_dims())) {
        const auto native_domain = planning_intervals();
        if (native_domain.size() != static_cast<std::size_t>(q.size())) {
            return false;
        }
        for (int dim = 0; dim < q.size(); ++dim) {
            if (!native_domain[static_cast<std::size_t>(dim)].contains(q[dim], 1e-12)) {
                return false;
            }
        }
    }
    const Eigen::VectorXd tree_q = tree_configuration_for_query(q);
    const auto box = database_.node_box(to_database_node(node));
    if (!box || tree_q.size() != static_cast<int>(box->size())) {
        return false;
    }
    for (int dim = 0; dim < tree_q.size(); ++dim) {
        if (!(*box)[static_cast<std::size_t>(dim)].contains(tree_q[dim])) {
            return false;
        }
    }
    return true;
}

bool DatabaseBoxOracle::is_leaf(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        return online_cache_->is_leaf(to_database_node(node));
    }
    return database_.topology(to_database_node(node)).leaf;
}

int DatabaseBoxOracle::depth(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        return online_cache_->depth(to_database_node(node));
    }
    return database_.topology(to_database_node(node)).depth;
}

int DatabaseBoxOracle::split_dim(OracleNodeId node) const {
    return database_.topology(to_database_node(node)).split_dim;
}

double DatabaseBoxOracle::split_value(OracleNodeId node) const {
    return database_.topology(to_database_node(node)).split_value;
}

OracleSplitPolicyDescriptor DatabaseBoxOracle::split_policy_descriptor() const {
    return database_.split_policy_descriptor();
}

OracleNodeId DatabaseBoxOracle::left_child(OracleNodeId node) const {
    const auto id = database_.topology(to_database_node(node)).left;
    return from_database_node(id);
}

OracleNodeId DatabaseBoxOracle::right_child(OracleNodeId node) const {
    const auto id = database_.topology(to_database_node(node)).right;
    return from_database_node(id);
}

OracleNodeTopology DatabaseBoxOracle::node_topology(OracleNodeId node) const {
    OracleNodeTopology out;
    if (node < 0) {
        return out;
    }
    const auto topology = online_cache_ != nullptr
        ? online_cache_->topology(to_database_node(node))
        : database_.topology(to_database_node(node));
    out.valid = topology.id != lect_database::kInvalidNodeId;
    out.leaf = topology.leaf;
    out.depth = topology.depth;
    out.split_dim = topology.split_dim;
    out.split_value = topology.split_value;
    out.left = from_database_node(topology.left);
    out.right = from_database_node(topology.right);
    return out;
}

}  // namespace rbf
