#include <SBF/find_free_box.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "find_free_box_internal.h"

namespace rbf {

using ffb_internal::record_elapsed;
using ffb_internal::record_split_diagnostics;
using ffb_internal::scheduled_depths;
using ffb_internal::set_max_diagnostic;

FindFreeBoxResult FindFreeBoxService::find(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                           const FindFreeBoxOptions& options) {
    StageContext context = StageContext::serial();
    return find(seed, context, options);
}

FindFreeBoxResult FindFreeBoxService::find(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                           StageContext& context,
                                           const FindFreeBoxOptions& options) {
    using Clock = std::chrono::steady_clock;
    std::unique_ptr<ScopedStageTimer> function_timer;
    if (options.record_diagnostics) {
        function_timer = std::make_unique<ScopedStageTimer>(context.diagnostics(), "ffb.find");
    }
    const auto start = Clock::now();
    FindFreeBoxResult result;
    if (context.should_stop()) {
        result.deadline_reached = context.deadline().expired();
        result.fail_code = 4;
        return result;
    }
    bool seed_in_domain = false;
    if (seed.size() == oracle_.n_dims()) {
        const auto contains_start = Clock::now();
        seed_in_domain = oracle_.contains_point(oracle_.root_node(), seed);
        record_elapsed(context, "oracle.contains_point", contains_start, options.record_diagnostics);
    }
    if (seed.size() != oracle_.n_dims() || !seed_in_domain) {
        result.fail_code = 5;
        return result;
    }
    if (options.reject_seed_collision) {
        const auto collision_start = Clock::now();
        const bool seed_collision = oracle_.point_in_collision(seed);
        record_elapsed(context, "oracle.point_in_collision", collision_start, options.record_diagnostics);
        if (seed_collision) {
            result.seed_collision = true;
            result.fail_code = 1;
            return result;
        }
    }

    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };

    // Seed-independent: the canonical LECT split depends only on (robot,
    // domain). No query-seed coupling is applied to the split values.
    OracleSplitOptions split_options = options.split;

    const int effective_max_depth = std::max(0, std::min(options.max_depth, oracle_.max_tree_depth() - 1));
    if (options.search_mode == FindFreeBoxSearchMode::BinaryDepth &&
        options.adaptive_depths.empty()) {
        return find_binary_depth(seed,
                                 context,
                                 options,
                                 split_options,
                                 effective_max_depth,
                                 start);
    }

    if (!options.adaptive_depths.empty()) {
        return find_incremental(seed, context, options, [](const FindFreeBoxResult&) {
            return true;
        });
    }

    OracleNodeId node = oracle_.root_node();
    int changed_dim = -1;
    while (true) {
        if (context.should_stop() || (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            result.deadline_reached = context.deadline().expired() || options.deadline_ms > 0.0;
            result.fail_code = 4;
            break;
        }

        oracle_.record_visit(node);

        const auto intervals_start = Clock::now();
        auto tree_intervals = oracle_.node_intervals(node);
        auto query_intervals = oracle_.query_intervals_for_node(node, tree_intervals, seed);
        record_elapsed(context, "oracle.node_intervals", intervals_start, options.record_diagnostics);
        if (oracle_.is_reserved(node)) {
            if (oracle_.depth(node) >= effective_max_depth || !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(query_intervals);
                result.fail_code = 2;
                break;
            }
            if (oracle_.is_leaf(node)) {
                const auto split_start = Clock::now();
                const int split_depth = oracle_.depth(node);
                const auto split = oracle_.split_node(node, tree_intervals, changed_dim, split_options);
                record_elapsed(context, "oracle.split_node", split_start, options.record_diagnostics);
                record_split_diagnostics(context, split, tree_intervals, split_depth, options.record_diagnostics);
                if (!split.split) {
                    result.fail_code = 6;
                    break;
                }
                result.splits += 1;
            }
            changed_dim = oracle_.split_dim(node);
            node = oracle_.child_containing_point(node, seed);
            if (node == kInvalidOracleNodeId) {
                result.fail_code = 5;
                break;
            }
            continue;
        }

        BoxValidation validation = BoxValidation::Unknown;
        if (oracle_.depth(node) < options.skip_to_depth) {
            validation = BoxValidation::Unknown;
        } else {
            const auto validation_start = Clock::now();
            validation = oracle_.validate_node(node, query_intervals, changed_dim);
            result.validation_detail = oracle_.last_validation_detail();
            record_elapsed(context, "oracle.validate_node", validation_start, options.record_diagnostics);
            result.decisions += 1;
        }
        if (validation == BoxValidation::Free) {
            result.found = true;
            result.node = node;
            result.changed_dim = changed_dim;
            result.intervals = std::move(query_intervals);
            result.fail_code = 0;
            // P4: the descent returns the FIRST (hence shallowest) canonical
            // ancestor that certifies DefinitelyFree along the seed path; this is
            // the largest certified box containing the seed and depends only on
            // which canonical nodes are certified (seed-independent). Record the
            // hit depth and box log-volume so the seed-independent reuse can be
            // measured (mean hit depth should drop / certified box volume rise).
            const double free_depth = static_cast<double>(oracle_.depth(node));
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.free_ancestor_hits");
                context.diagnostics().add_counter("ffb.free_ancestor_depth_sum", free_depth);
            }
            set_max_diagnostic(context, "ffb.free_ancestor_depth_max", free_depth, options.record_diagnostics);
            double free_log_volume = 0.0;
            for (const auto& interval : result.intervals) {
                const double width = std::max(0.0, interval.width());
                if (width > 0.0) {
                    free_log_volume += std::log(width);
                }
            }
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.free_ancestor_log_volume_sum", free_log_volume);
            }
            break;
        }
        if (validation == BoxValidation::Occupied) {
            result.node = node;
            result.intervals = std::move(query_intervals);
            result.fail_code = 3;
            break;
        }
        if (oracle_.depth(node) >= effective_max_depth || !options.split_unknown_leaf) {
            result.hit_unknown_depth_cap = true;
            result.node = node;
            result.intervals = std::move(query_intervals);
            result.fail_code = 2;
            break;
        }
        if (oracle_.is_leaf(node)) {
            const auto split_start = Clock::now();
            const int split_depth = oracle_.depth(node);
            const auto split = oracle_.split_node(node, tree_intervals, changed_dim, split_options);
            record_elapsed(context, "oracle.split_node", split_start, options.record_diagnostics);
            record_split_diagnostics(context, split, tree_intervals, split_depth, options.record_diagnostics);
            if (!split.split) {
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
        }
        changed_dim = oracle_.split_dim(node);
        node = oracle_.child_containing_point(node, seed);
        if (node == kInvalidOracleNodeId) {
            result.fail_code = 5;
            break;
        }
    }
    result.total_ms = elapsed_ms();
    return result;
}

FindFreeBoxResult FindFreeBoxService::find_incremental(
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    StageContext& context,
    const FindFreeBoxOptions& options,
    const AcceptCandidate& accept) {
    using Clock = std::chrono::steady_clock;
    std::unique_ptr<ScopedStageTimer> function_timer;
    if (options.record_diagnostics) {
        function_timer = std::make_unique<ScopedStageTimer>(context.diagnostics(), "ffb.find");
    }
    const auto start = Clock::now();
    FindFreeBoxResult result;
    if (context.should_stop()) {
        result.deadline_reached = context.deadline().expired();
        result.fail_code = 4;
        return result;
    }
    bool seed_in_domain = false;
    if (seed.size() == oracle_.n_dims()) {
        const auto contains_start = Clock::now();
        seed_in_domain = oracle_.contains_point(oracle_.root_node(), seed);
        record_elapsed(context, "oracle.contains_point", contains_start, options.record_diagnostics);
    }
    if (seed.size() != oracle_.n_dims() || !seed_in_domain) {
        result.fail_code = 5;
        return result;
    }
    if (options.reject_seed_collision) {
        const auto collision_start = Clock::now();
        const bool seed_collision = oracle_.point_in_collision(seed);
        record_elapsed(context, "oracle.point_in_collision", collision_start, options.record_diagnostics);
        if (seed_collision) {
            result.seed_collision = true;
            result.fail_code = 1;
            return result;
        }
    }

    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };
    OracleSplitOptions split_options = options.split;
    const int effective_max_depth =
        std::max(0, std::min(options.max_depth, oracle_.max_tree_depth() - 1));
    const std::vector<int> checkpoints = scheduled_depths(options, effective_max_depth);
    std::size_t checkpoint_index = 0;
    OracleNodeId node = oracle_.root_node();
    int changed_dim = -1;
    while (true) {
        if (context.should_stop() ||
            (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            result.deadline_reached =
                context.deadline().expired() || options.deadline_ms > 0.0;
            result.fail_code = 4;
            break;
        }
        oracle_.record_visit(node);
        const auto intervals_start = Clock::now();
        auto tree_intervals = oracle_.node_intervals(node);
        auto query_intervals = oracle_.query_intervals_for_node(node, tree_intervals, seed);
        record_elapsed(context, "oracle.node_intervals", intervals_start, options.record_diagnostics);
        const int depth = oracle_.depth(node);
        while (checkpoint_index + 1 < checkpoints.size() &&
               depth > checkpoints[checkpoint_index]) {
            checkpoint_index += 1;
        }
        const int active_checkpoint = checkpoints[checkpoint_index];

        if (oracle_.is_reserved(node)) {
            if (depth >= effective_max_depth || !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(query_intervals);
                result.fail_code = 2;
                break;
            }
            if (oracle_.is_leaf(node)) {
                const auto split_start = Clock::now();
                const auto split = oracle_.split_node(node,
                                                       tree_intervals,
                                                       changed_dim,
                                                       split_options);
                record_elapsed(context, "oracle.split_node", split_start, options.record_diagnostics);
                record_split_diagnostics(context, split, tree_intervals, depth, options.record_diagnostics);
                if (!split.split) {
                    result.fail_code = 6;
                    break;
                }
                result.splits += 1;
            }
            changed_dim = oracle_.split_dim(node);
            node = oracle_.child_containing_point(node, seed);
            if (node == kInvalidOracleNodeId) {
                result.fail_code = 5;
                break;
            }
            continue;
        }

        BoxValidation validation = BoxValidation::Unknown;
        if (depth < options.skip_to_depth) {
            validation = BoxValidation::Unknown;
        } else {
            const auto validation_start = Clock::now();
            validation = oracle_.validate_node(node, query_intervals, changed_dim);
            result.validation_detail = oracle_.last_validation_detail();
            record_elapsed(context, "oracle.validate_node", validation_start, options.record_diagnostics);
            result.decisions += 1;
        }
        if (validation == BoxValidation::Free) {
            FindFreeBoxResult candidate = result;
            candidate.found = true;
            candidate.node = node;
            candidate.changed_dim = changed_dim;
            candidate.intervals = query_intervals;
            candidate.fail_code = 0;
            candidate.hit_unknown_depth_cap = false;
            candidate.hit_reserved_depth_cap = false;
            candidate.total_ms = elapsed_ms();
            const bool must_accept = depth >= effective_max_depth ||
                                     depth >= active_checkpoint;
            if (must_accept && (!accept || accept(candidate))) {
                result = std::move(candidate);
                const double free_depth = static_cast<double>(oracle_.depth(node));
                if (options.record_diagnostics) {
                    context.diagnostics().add_counter("ffb.free_ancestor_hits");
                    context.diagnostics().add_counter("ffb.free_ancestor_depth_sum", free_depth);
                }
                set_max_diagnostic(context, "ffb.free_ancestor_depth_max", free_depth, options.record_diagnostics);
                double free_log_volume = 0.0;
                for (const auto& interval : result.intervals) {
                    const double width = std::max(0.0, interval.width());
                    if (width > 0.0) {
                        free_log_volume += std::log(width);
                    }
                }
                if (options.record_diagnostics) {
                    context.diagnostics().add_counter("ffb.free_ancestor_log_volume_sum", free_log_volume);
                }
                break;
            }
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.incremental_candidate_skips");
            }
        } else if (validation == BoxValidation::Occupied) {
            result.node = node;
            result.intervals = std::move(query_intervals);
            result.fail_code = 3;
            break;
        }

        if (depth >= effective_max_depth || !options.split_unknown_leaf) {
            result.hit_unknown_depth_cap = true;
            result.node = node;
            result.intervals = std::move(query_intervals);
            result.fail_code = 2;
            break;
        }
        if (oracle_.is_leaf(node)) {
            const auto split_start = Clock::now();
            const auto split = oracle_.split_node(node,
                                                   tree_intervals,
                                                   changed_dim,
                                                   split_options);
            record_elapsed(context, "oracle.split_node", split_start, options.record_diagnostics);
            record_split_diagnostics(context, split, tree_intervals, depth, options.record_diagnostics);
            if (!split.split) {
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
        }
        changed_dim = oracle_.split_dim(node);
        node = oracle_.child_containing_point(node, seed);
        if (node == kInvalidOracleNodeId) {
            result.fail_code = 5;
            break;
        }
    }
    result.total_ms = elapsed_ms();
    return result;
}

}  // namespace rbf
