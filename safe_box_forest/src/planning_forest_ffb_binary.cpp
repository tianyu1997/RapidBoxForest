#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "find_free_box_internal.h"
#include "planning_forest_ffb_binary_sparse.h"
#include "planning_forest_ffb_helpers.h"
#include "virtual_sparse_ffb_options.h"

namespace rbf {

using ffb_internal::record_free_ancestor_diagnostics;

FindFreeBoxResult RBFPlanningForest::find_free_box_binary_in_domain(
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    const std::vector<Interval>& domain,
    StageContext& context,
    const FindFreeBoxOptions& options,
    const OracleSplitOptions& split_options,
    int effective_max_depth,
    std::chrono::steady_clock::time_point start) {
    using Clock = std::chrono::steady_clock;
    FindFreeBoxResult result;
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };
    const detail::VirtualSparseFfbOptions virtual_sparse_options =
        detail::virtual_sparse_ffb_options(options.binary_probe_depth);
    OracleNodeId node = oracle_->root_node();
    OracleNodeTopology node_topology = oracle_->node_topology(node);
    if (!node_topology.valid) {
        result.fail_code = 5;
        return result;
    }
    int changed_dim = -1;
    const Eigen::VectorXd tree_seed = oracle_->tree_configuration_for_query(seed);
    std::vector<Interval> tree_intervals = oracle_->node_intervals(node);
    auto descend_to_seed_child = [&](const OracleNodeTopology& parent_topology) -> bool {
        const int split_dim = parent_topology.split_dim;
        if (split_dim < 0 || split_dim >= tree_seed.size()) {
            return false;
        }
        const double split_value = parent_topology.split_value;
        const OracleNodeId left_child = parent_topology.left;
        const OracleNodeId right_child = parent_topology.right;
        const OracleNodeId child =
            tree_seed[split_dim] <= split_value ? left_child : right_child;
        if (child == kInvalidOracleNodeId) {
            return false;
        }
        if (split_dim >= 0 && split_dim < static_cast<int>(tree_intervals.size())) {
            auto& interval = tree_intervals[static_cast<std::size_t>(split_dim)];
            if (child == left_child) {
                interval.hi = split_value;
            } else if (child == right_child) {
                interval.lo = split_value;
            } else {
                tree_intervals = oracle_->node_intervals(child);
            }
        } else {
            tree_intervals = oracle_->node_intervals(child);
        }
        changed_dim = split_dim;
        node = child;
        node_topology = oracle_->node_topology(node);
        if (!node_topology.valid) {
            return false;
        }
        return true;
    };
    if (options.record_diagnostics) {
        context.diagnostics().add_counter("ffb.binary_requested");
    }
    const VirtualSparseBinaryFfbAttempt sparse_attempt =
        try_virtual_sparse_binary_ffb(*oracle_,
                                      seed,
                                      domain,
                                      context,
                                      options,
                                      split_options,
                                      effective_max_depth,
                                      start);
    if (sparse_attempt.completed) {
        return sparse_attempt.result;
    }
    if (!sparse_attempt.supported && options.record_diagnostics) {
        context.diagnostics().add_counter("ffb.binary_virtual_unsupported");
    }
    if (options.record_diagnostics) {
        context.diagnostics().add_counter("ffb.binary_materialized_fallback_calls");
    }
    struct PathEntry {
        OracleNodeId node = kInvalidOracleNodeId;
        int changed_dim = -1;
        std::vector<Interval> tree_intervals;
    };
    std::vector<PathEntry> path;
    path.reserve(static_cast<std::size_t>(effective_max_depth + 1));
    while (true) {
        if (context.should_stop() ||
            (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            result.deadline_reached =
                context.deadline().expired() || options.deadline_ms > 0.0;
            result.fail_code = 4;
            result.total_ms = elapsed_ms();
            return result;
        }
        if (node_topology.depth < 0) {
            result.fail_code = 5;
            result.total_ms = elapsed_ms();
            return result;
        }
        if (static_cast<int>(path.size()) <= node_topology.depth) {
            path.resize(static_cast<std::size_t>(node_topology.depth + 1));
        }
        path[static_cast<std::size_t>(node_topology.depth)] =
            {node, changed_dim, tree_intervals};
        if (node_topology.depth >= effective_max_depth) {
            break;
        }
        if (node_topology.leaf) {
            if (oracle_->is_reserved(node) && !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = oracle_->query_intervals_for_node(node,
                                                                     tree_intervals,
                                                                     seed);
                result.fail_code = 2;
                result.total_ms = elapsed_ms();
                return result;
            }
            if (!options.split_unknown_leaf) {
                result.hit_unknown_depth_cap = true;
                result.node = node;
                result.intervals = oracle_->query_intervals_for_node(node,
                                                                     tree_intervals,
                                                                     seed);
                result.fail_code = 2;
                result.total_ms = elapsed_ms();
                return result;
            }
            const auto split = oracle_->split_node(node,
                                                   tree_intervals,
                                                   changed_dim,
                                                   split_options);
            if (!split.split) {
                result.fail_code = 6;
                result.total_ms = elapsed_ms();
                return result;
            }
            result.splits += 1;
            OracleNodeTopology split_topology = node_topology;
            split_topology.leaf = false;
            split_topology.split_dim = split.split_dim;
            split_topology.split_value = split.split_value;
            split_topology.left = split.left;
            split_topology.right = split.right;
            if (!descend_to_seed_child(split_topology)) {
                result.fail_code = 5;
                result.total_ms = elapsed_ms();
                return result;
            }
            continue;
        }
        if (!descend_to_seed_child(node_topology)) {
            result.fail_code = 5;
            result.total_ms = elapsed_ms();
            return result;
        }
    }

    const int start_depth =
        std::max(0,
                 std::min(effective_max_depth,
                          std::max(options.start_depth, options.skip_to_depth)));
    auto validate_depth = [&](int depth, FindFreeBoxResult& candidate) {
        if (depth < 0 ||
            depth >= static_cast<int>(path.size()) ||
            path[static_cast<std::size_t>(depth)].node == kInvalidOracleNodeId) {
            candidate.fail_code = 5;
            return BoxValidation::Unknown;
        }
        const auto& entry = path[static_cast<std::size_t>(depth)];
        auto native_intervals = oracle_->query_intervals_for_node(entry.node,
                                                                  entry.tree_intervals,
                                                                  seed);
        candidate.node = entry.node;
        candidate.changed_dim = entry.changed_dim;
        candidate.intervals = native_intervals;
        if (!forest_ffb_internal::intervals_overlap(native_intervals, domain, 0.0)) {
            candidate.fail_code = 5;
            return BoxValidation::Unknown;
        }
        if (!forest_ffb_internal::intervals_subset(native_intervals, domain, 1e-12)) {
            candidate.hit_unknown_depth_cap = depth >= effective_max_depth;
            candidate.fail_code = 2;
            return BoxValidation::Unknown;
        }
        if (oracle_->is_reserved(entry.node)) {
            candidate.hit_reserved_depth_cap = true;
            candidate.fail_code = 2;
            return BoxValidation::Unknown;
        }
        if (depth < options.skip_to_depth) {
            candidate.hit_unknown_depth_cap = true;
            candidate.fail_code = 2;
            return BoxValidation::Unknown;
        }
        const auto validation = oracle_->validate_node(entry.node,
                                                       native_intervals,
                                                       entry.changed_dim);
        candidate.validation_detail = oracle_->last_validation_detail();
        candidate.decisions += 1;
        if (validation == BoxValidation::Free) {
            candidate.found = true;
            candidate.fail_code = 0;
        } else if (validation == BoxValidation::Occupied) {
            candidate.fail_code = 3;
        } else {
            candidate.hit_unknown_depth_cap = depth >= effective_max_depth;
            candidate.fail_code = 2;
        }
        return validation;
    };

    int lo = start_depth;
    int hi = effective_max_depth;
    FindFreeBoxResult best;
    const int probe_depth =
        detail::binary_probe_depth(start_depth,
                                   effective_max_depth,
                                   virtual_sparse_options);
    if (probe_depth >= start_depth && probe_depth < effective_max_depth) {
        FindFreeBoxResult probe_candidate;
        const BoxValidation probe_validation = validate_depth(probe_depth,
                                                              probe_candidate);
        result.decisions += probe_candidate.decisions;
        if (probe_validation == BoxValidation::Free) {
            best = std::move(probe_candidate);
            hi = probe_depth;
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.binary_probe_free");
            }
        } else {
            lo = std::max(lo, probe_depth + 1);
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.binary_probe_not_free");
            }
        }
    }
    FindFreeBoxResult high_candidate;
    if (!best.found) {
        const BoxValidation high_validation = validate_depth(effective_max_depth,
                                                             high_candidate);
        result.decisions += high_candidate.decisions;
        if (high_validation != BoxValidation::Free) {
            high_candidate.splits = result.splits;
            high_candidate.total_ms = elapsed_ms();
            return high_candidate;
        }
        best = high_candidate;
    }
    while (lo < hi) {
        if (context.should_stop() ||
            (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            best.deadline_reached =
                context.deadline().expired() || options.deadline_ms > 0.0;
            best.fail_code = 4;
            break;
        }
        const int mid = lo + (hi - lo) / 2;
        FindFreeBoxResult candidate;
        const BoxValidation validation = validate_depth(mid, candidate);
        result.decisions += candidate.decisions;
        if (validation == BoxValidation::Free) {
            best = std::move(candidate);
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    best.decisions = result.decisions;
    best.splits = result.splits;
    best.total_ms = elapsed_ms();
    if (best.found && options.record_diagnostics) {
        const double free_depth =
            static_cast<double>(oracle_->node_topology(best.node).depth);
        record_free_ancestor_diagnostics(context,
                                         best.intervals,
                                         free_depth,
                                         true);
    }
    return best;
}

}  // namespace rbf
