#include <SBF/safe_box_forest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "planning_forest_ffb_helpers.h"
#include "virtual_sparse_ffb.h"
#include "virtual_sparse_ffb_options.h"

namespace rbf {

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
    const int virtual_start_depth =
        std::max(0,
                 std::min(effective_max_depth,
                          std::max(options.start_depth, options.skip_to_depth)));
    const auto virtual_path =
        detail::virtual_seed_path_to_depth(*oracle_, seed, effective_max_depth);
    if (virtual_path) {
        if (options.record_diagnostics) {
            context.diagnostics().add_counter("ffb.virtual_sparse_binary_attempts");
            context.diagnostics().add_counter(
                "ffb.virtual_sparse_binary_path_entries",
                static_cast<double>(virtual_path->size()));
        }
        auto validate_virtual_depth = [&](int depth, FindFreeBoxResult& candidate) {
            if (depth < options.skip_to_depth) {
                candidate.hit_unknown_depth_cap = true;
                candidate.fail_code = 2;
                return BoxValidation::Unknown;
            }
            if (depth < 0 ||
                depth >= static_cast<int>(virtual_path->size())) {
                candidate.fail_code = 6;
                return BoxValidation::Unknown;
            }
            const auto& cell = (*virtual_path)[static_cast<std::size_t>(depth)];
            candidate.node = oracle_->root_node();
            candidate.changed_dim = cell.changed_dim;
            candidate.intervals = oracle_->query_intervals_for_node(
                oracle_->root_node(),
                cell.tree_intervals,
                seed);
            if (!forest_ffb_internal::intervals_overlap(candidate.intervals, domain, 0.0)) {
                candidate.fail_code = 5;
                return BoxValidation::Unknown;
            }
            if (!forest_ffb_internal::intervals_subset(candidate.intervals, domain, 1e-12)) {
                candidate.hit_unknown_depth_cap = depth >= effective_max_depth;
                candidate.fail_code = 2;
                return BoxValidation::Unknown;
            }
            const auto validation = oracle_->validate_node(oracle_->root_node(),
                                                           candidate.intervals,
                                                           candidate.changed_dim);
            candidate.validation_detail = oracle_->last_validation_detail();
            candidate.decisions += 1;
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.virtual_sparse_binary_probes");
                context.diagnostics().add_counter("ffb.virtual_sparse_binary_probe_depth_sum",
                                                  static_cast<double>(depth));
                context.diagnostics().set_value(
                    "ffb.virtual_sparse_binary_probe_depth_max",
                    std::max(context.diagnostics().value("ffb.virtual_sparse_binary_probe_depth_max"),
                             static_cast<double>(depth)));
            }
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

        int lo = virtual_start_depth;
        int hi = effective_max_depth;
        int best_depth = -1;
        FindFreeBoxResult best;
        const int probe_depth =
            detail::binary_probe_depth(virtual_start_depth,
                                       effective_max_depth,
                                       virtual_sparse_options);
        if (probe_depth >= virtual_start_depth && probe_depth < effective_max_depth) {
            FindFreeBoxResult probe_candidate;
            const BoxValidation probe_validation = validate_virtual_depth(probe_depth,
                                                                          probe_candidate);
            result.decisions += probe_candidate.decisions;
            if (probe_validation == BoxValidation::Free) {
                best = std::move(probe_candidate);
                best_depth = probe_depth;
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
        if (!best.found) {
            FindFreeBoxResult high_candidate;
            const BoxValidation high_validation = validate_virtual_depth(effective_max_depth,
                                                                         high_candidate);
            result.decisions += high_candidate.decisions;
            if (high_validation != BoxValidation::Free) {
                high_candidate.decisions = result.decisions;
                high_candidate.splits = 0;
                high_candidate.total_ms = elapsed_ms();
                return high_candidate;
            }
            best = high_candidate;
            best_depth = effective_max_depth;
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
            const BoxValidation validation = validate_virtual_depth(mid, candidate);
            result.decisions += candidate.decisions;
            if (validation == BoxValidation::Free) {
                best = std::move(candidate);
                best_depth = mid;
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        best.decisions = result.decisions;
        if (best.found && best_depth >= 0) {
            if (!options.materialize_result_node) {
                best.node = kInvalidOracleNodeId;
                best.splits = 0;
                best.total_ms = elapsed_ms();
                if (options.record_diagnostics) {
                    context.diagnostics().add_counter(
                        "ffb.virtual_sparse_binary_successes");
                    context.diagnostics().add_counter(
                        "ffb.virtual_sparse_binary_materialize_skipped");
                    context.diagnostics().add_counter("ffb.free_ancestor_hits");
                    context.diagnostics().add_counter("ffb.free_ancestor_depth_sum",
                                                      static_cast<double>(best_depth));
                    context.diagnostics().set_value(
                        "ffb.free_ancestor_depth_max",
                        std::max(context.diagnostics().value("ffb.free_ancestor_depth_max"),
                                 static_cast<double>(best_depth)));
                    double free_log_volume = 0.0;
                    for (const auto& interval : best.intervals) {
                        const double width = std::max(0.0, interval.width());
                        if (width > 0.0) {
                            free_log_volume += std::log(width);
                        }
                    }
                    context.diagnostics().add_counter("ffb.free_ancestor_log_volume_sum",
                                                      free_log_volume);
                }
                return best;
            }
            const auto materialized = detail::materialize_seed_path_to_depth(
                *oracle_,
                seed,
                best_depth,
                split_options,
                [&](const SplitNodeResult&,
                    const std::vector<Interval>&,
                    int,
                    double split_ms) {
                    if (options.record_diagnostics) {
                        context.diagnostics().record_timing("oracle.split_node", split_ms);
                    }
                });
            if (materialized) {
                result.splits += materialized->splits;
                best.splits = result.splits;
                best.node = materialized->node;
                best.changed_dim = materialized->changed_dim;
                if (!detail::intervals_equal_with_tolerance(best.intervals,
                                                            materialized->query_intervals,
                                                            1e-10)) {
                    if (options.record_diagnostics) {
                        context.diagnostics().add_counter(
                            "ffb.virtual_sparse_binary_materialize_mismatch");
                    }
                    best.intervals = materialized->query_intervals;
                    if (!forest_ffb_internal::intervals_overlap(best.intervals, domain, 0.0)) {
                        best.found = false;
                        best.fail_code = 5;
                        best.total_ms = elapsed_ms();
                        return best;
                    }
                    if (!forest_ffb_internal::intervals_subset(best.intervals, domain, 1e-12)) {
                        best.found = false;
                        best.hit_unknown_depth_cap = best_depth >= effective_max_depth;
                        best.fail_code = 2;
                        best.total_ms = elapsed_ms();
                        return best;
                    }
                    const auto validation = oracle_->validate_node(materialized->node,
                                                                   best.intervals,
                                                                   materialized->changed_dim);
                    best.validation_detail = oracle_->last_validation_detail();
                    best.decisions += 1;
                    if (validation != BoxValidation::Free) {
                        best.found = false;
                        best.fail_code = validation == BoxValidation::Occupied ? 3 : 2;
                        best.hit_unknown_depth_cap = validation == BoxValidation::Unknown;
                        best.total_ms = elapsed_ms();
                        return best;
                    }
                } else {
                    best.intervals = materialized->query_intervals;
                }
                if (oracle_->is_reserved(best.node) && !options.split_reserved_leaf) {
                    best.found = false;
                    best.hit_reserved_depth_cap = true;
                    best.fail_code = 2;
                    best.total_ms = elapsed_ms();
                    return best;
                }
                best.total_ms = elapsed_ms();
                if (options.record_diagnostics) {
                    context.diagnostics().add_counter("ffb.virtual_sparse_binary_successes");
                    context.diagnostics().add_counter("ffb.free_ancestor_hits");
                    context.diagnostics().add_counter("ffb.free_ancestor_depth_sum",
                                                      static_cast<double>(best_depth));
                    context.diagnostics().set_value(
                        "ffb.free_ancestor_depth_max",
                        std::max(context.diagnostics().value("ffb.free_ancestor_depth_max"),
                                 static_cast<double>(best_depth)));
                    double free_log_volume = 0.0;
                    for (const auto& interval : best.intervals) {
                        const double width = std::max(0.0, interval.width());
                        if (width > 0.0) {
                            free_log_volume += std::log(width);
                        }
                    }
                    context.diagnostics().add_counter("ffb.free_ancestor_log_volume_sum",
                                                      free_log_volume);
                }
                return best;
            }
            if (options.record_diagnostics) {
                context.diagnostics().add_counter(
                    "ffb.virtual_sparse_binary_materialize_failures");
            }
            node = oracle_->root_node();
            node_topology = oracle_->node_topology(node);
            changed_dim = -1;
            tree_intervals = oracle_->node_intervals(node);
        }
    } else if (options.record_diagnostics) {
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
        context.diagnostics().add_counter("ffb.free_ancestor_hits");
        context.diagnostics().add_counter("ffb.free_ancestor_depth_sum",
                                          free_depth);
        context.diagnostics().set_value(
            "ffb.free_ancestor_depth_max",
            std::max(context.diagnostics().value("ffb.free_ancestor_depth_max"),
                     free_depth));
        double free_log_volume = 0.0;
        for (const auto& interval : best.intervals) {
            const double width = std::max(0.0, interval.width());
            if (width > 0.0) {
                free_log_volume += std::log(width);
            }
        }
        context.diagnostics().add_counter("ffb.free_ancestor_log_volume_sum",
                                          free_log_volume);
    }
    return best;
}

}  // namespace rbf
