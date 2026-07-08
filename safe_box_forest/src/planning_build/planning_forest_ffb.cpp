#include <SBF/safe_box_forest.h>

#include <SBF/oracle.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "planning_forest_ffb_helpers.h"

namespace rbf {

FindFreeBoxResult RBFPlanningForest::find_free_box_in_domain(
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    const std::vector<Interval>& domain,
    StageContext& context,
    const FindFreeBoxOptions& options) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    FindFreeBoxResult result;
    if (options.record_diagnostics) {
        context.diagnostics().add_counter("ffb.find_calls");
    }
    if (!oracle_ || !database_ || seed.size() != oracle_->n_dims() ||
        domain.size() != static_cast<std::size_t>(oracle_->n_dims())) {
        if (options.record_diagnostics) {
            context.diagnostics().add_counter("ffb.fail_invalid_oracle");
        }
        result.fail_code = 5;
        return result;
    }
    if (context.should_stop()) {
        if (options.record_diagnostics) {
            context.diagnostics().add_counter("ffb.fail_cancelled");
        }
        result.deadline_reached = context.deadline().expired();
        result.fail_code = 4;
        return result;
    }
    const bool seed_in_root = oracle_->contains_point(oracle_->root_node(), seed);
    const bool seed_in_domain = forest_ffb_internal::intervals_contain_point_strict(domain, seed, 1e-12);
    if (!seed_in_root || !seed_in_domain) {
        if (options.record_diagnostics) {
            context.diagnostics().add_counter("ffb.fail_seed_or_domain");
            if (!seed_in_root) {
                context.diagnostics().add_counter("ffb.fail_seed_outside_root");
            }
            if (!seed_in_domain) {
                context.diagnostics().add_counter("ffb.fail_seed_outside_domain");
            }
        }
        result.fail_code = 5;
        return result;
    }
    if (!options.skip_existing_cover_check && forest_ffb_internal::point_covered_by_existing_box(boxes_, seed)) {
        context.diagnostics().add_counter("forest.find_free_box_in_domain_seed_already_covered");
        result.fail_code = 7;
        return result;
    }
    if (options.reject_seed_collision && oracle_->point_in_collision(seed)) {
        if (options.record_diagnostics) {
            context.diagnostics().add_counter("ffb.fail_seed_collision");
        }
        result.seed_collision = true;
        result.fail_code = 1;
        return result;
    }

    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };
    const int effective_max_depth =
        std::max(0, std::min(options.max_depth, oracle_->max_tree_depth() - 1));
    // Seed-independent: canonical split depends only on (robot, domain). No
    // query-seed coupling is applied to the split values.
    OracleSplitOptions split_options = options.split;
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
    if (options.search_mode == FindFreeBoxSearchMode::BinaryDepth &&
        options.adaptive_depths.empty()) {
        return find_free_box_binary_in_domain(seed,
                                              domain,
                                              context,
                                              options,
                                              split_options,
                                              effective_max_depth,
                                              start);
    }
    if (options.search_mode == FindFreeBoxSearchMode::BinaryDepth &&
        !options.adaptive_depths.empty() &&
        options.record_diagnostics) {
        context.diagnostics().add_counter("ffb.binary_blocked_adaptive_depths");
    }
    if (options.record_diagnostics) {
        context.diagnostics().add_counter("ffb.linear_descent_calls");
    }
    while (true) {
        if (context.should_stop() ||
            (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            result.deadline_reached = context.deadline().expired() || options.deadline_ms > 0.0;
            result.fail_code = 4;
            break;
        }

        if (!node_topology.leaf) {
            if (!descend_to_seed_child(node_topology)) {
                result.fail_code = 5;
                break;
            }
            continue;
        }

        auto native_intervals = oracle_->query_intervals_for_node(node, tree_intervals, seed);
        if (!forest_ffb_internal::intervals_overlap(native_intervals, domain, 0.0)) {
            result.node = node;
            result.intervals = std::move(native_intervals);
            result.fail_code = 5;
            break;
        }

        if (!forest_ffb_internal::intervals_subset(native_intervals, domain, 1e-12)) {
            if (node_topology.depth >= effective_max_depth) {
                result.hit_unknown_depth_cap = true;
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 2;
                break;
            }
            const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
            if (!split.split) {
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 6;
                break;
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
                break;
            }
            continue;
        }

        if (oracle_->is_reserved(node)) {
            if (node_topology.depth >= effective_max_depth || !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 2;
                break;
            }
            const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
            if (!split.split) {
                result.fail_code = 6;
                break;
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
                break;
            }
            continue;
        }

        if (node_topology.depth < std::max(options.start_depth, options.skip_to_depth)) {
            if (node_topology.depth >= effective_max_depth || !options.split_unknown_leaf) {
                result.hit_unknown_depth_cap = true;
                result.node = node;
                result.intervals = std::move(native_intervals);
                result.fail_code = 2;
                break;
            }
            const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
            if (!split.split) {
                result.fail_code = 6;
                break;
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
                break;
            }
            continue;
        }

        const auto validation = oracle_->validate_node(node, native_intervals, changed_dim);
        result.validation_detail = oracle_->last_validation_detail();
        result.decisions += 1;
        if (validation == BoxValidation::Free) {
            result.found = true;
            result.node = node;
            result.changed_dim = changed_dim;
            result.intervals = std::move(native_intervals);
            result.fail_code = 0;
            break;
        }
        if (validation == BoxValidation::Occupied) {
            result.node = node;
            result.intervals = std::move(native_intervals);
            result.fail_code = 3;
            break;
        }
        if (node_topology.depth >= effective_max_depth || !options.split_unknown_leaf) {
            result.hit_unknown_depth_cap = true;
            result.node = node;
            result.intervals = std::move(native_intervals);
            result.fail_code = 2;
            break;
        }
        const auto split = oracle_->split_node(node, tree_intervals, changed_dim, split_options);
        if (!split.split) {
            result.fail_code = 6;
            break;
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
            break;
        }
    }
    result.total_ms = elapsed_ms();
    return result;
}

} // namespace rbf
