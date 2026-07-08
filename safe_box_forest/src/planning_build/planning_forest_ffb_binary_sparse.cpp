#include "planning_forest_ffb_binary_sparse.h"

#include "find_free_box_internal.h"
#include "planning_forest_ffb_helpers.h"
#include "virtual_sparse_ffb.h"
#include "virtual_sparse_ffb_options.h"

#include <algorithm>
#include <chrono>
#include <utility>

namespace rbf {

using ffb_internal::record_free_ancestor_diagnostics;

VirtualSparseBinaryFfbAttempt try_virtual_sparse_binary_ffb(
    BoxOracle& oracle,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    const std::vector<Interval>& domain,
    StageContext& context,
    const FindFreeBoxOptions& options,
    const OracleSplitOptions& split_options,
    int effective_max_depth,
    std::chrono::steady_clock::time_point start) {
    using Clock = std::chrono::steady_clock;
    VirtualSparseBinaryFfbAttempt attempt;
    FindFreeBoxResult result;
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };
    const detail::VirtualSparseFfbOptions virtual_sparse_options =
        detail::virtual_sparse_ffb_options(options.binary_probe_depth);
    const int virtual_start_depth =
        std::max(0,
                 std::min(effective_max_depth,
                          std::max(options.start_depth, options.skip_to_depth)));
    const auto virtual_path =
        detail::virtual_seed_path_to_depth(oracle, seed, effective_max_depth);
    if (!virtual_path) {
        return attempt;
    }
    attempt.supported = true;
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
        if (depth < 0 || depth >= static_cast<int>(virtual_path->size())) {
            candidate.fail_code = 6;
            return BoxValidation::Unknown;
        }
        const auto& cell = (*virtual_path)[static_cast<std::size_t>(depth)];
        candidate.node = oracle.root_node();
        candidate.changed_dim = cell.changed_dim;
        candidate.intervals = oracle.query_intervals_for_node(
            oracle.root_node(),
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
        const auto validation = oracle.validate_node(oracle.root_node(),
                                                     candidate.intervals,
                                                     candidate.changed_dim);
        candidate.validation_detail = oracle.last_validation_detail();
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
            attempt.completed = true;
            attempt.result = std::move(high_candidate);
            return attempt;
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
                context.diagnostics().add_counter("ffb.virtual_sparse_binary_successes");
                context.diagnostics().add_counter(
                    "ffb.virtual_sparse_binary_materialize_skipped");
            }
            record_free_ancestor_diagnostics(context,
                                             best.intervals,
                                             static_cast<double>(best_depth),
                                             options.record_diagnostics);
            attempt.completed = true;
            attempt.result = std::move(best);
            return attempt;
        }
        const auto materialized = detail::materialize_seed_path_to_depth(
            oracle,
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
                    attempt.completed = true;
                    attempt.result = std::move(best);
                    return attempt;
                }
                if (!forest_ffb_internal::intervals_subset(best.intervals, domain, 1e-12)) {
                    best.found = false;
                    best.hit_unknown_depth_cap = best_depth >= effective_max_depth;
                    best.fail_code = 2;
                    best.total_ms = elapsed_ms();
                    attempt.completed = true;
                    attempt.result = std::move(best);
                    return attempt;
                }
                const auto validation = oracle.validate_node(materialized->node,
                                                             best.intervals,
                                                             materialized->changed_dim);
                best.validation_detail = oracle.last_validation_detail();
                best.decisions += 1;
                if (validation != BoxValidation::Free) {
                    best.found = false;
                    best.fail_code = validation == BoxValidation::Occupied ? 3 : 2;
                    best.hit_unknown_depth_cap = validation == BoxValidation::Unknown;
                    best.total_ms = elapsed_ms();
                    attempt.completed = true;
                    attempt.result = std::move(best);
                    return attempt;
                }
            } else {
                best.intervals = materialized->query_intervals;
            }
            if (oracle.is_reserved(best.node) && !options.split_reserved_leaf) {
                best.found = false;
                best.hit_reserved_depth_cap = true;
                best.fail_code = 2;
                best.total_ms = elapsed_ms();
                attempt.completed = true;
                attempt.result = std::move(best);
                return attempt;
            }
            best.total_ms = elapsed_ms();
            if (options.record_diagnostics) {
                context.diagnostics().add_counter("ffb.virtual_sparse_binary_successes");
            }
            record_free_ancestor_diagnostics(context,
                                             best.intervals,
                                             static_cast<double>(best_depth),
                                             options.record_diagnostics);
            attempt.completed = true;
            attempt.result = std::move(best);
            return attempt;
        }
        if (options.record_diagnostics) {
            context.diagnostics().add_counter(
                "ffb.virtual_sparse_binary_materialize_failures");
        }
    }
    return attempt;
}

}  // namespace rbf
