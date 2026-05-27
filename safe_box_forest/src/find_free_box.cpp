#include <SBF/find_free_box.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace rbf {

namespace {

using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;

struct SearchDepthRange {
    int start_depth = 0;
    int max_depth = 0;
};

enum class DepthProbeOutcome : std::uint8_t {
    Free = 0,
    NonFree = 1,
    Fatal = 2,
};

struct DepthProbeResult {
    DepthProbeOutcome outcome = DepthProbeOutcome::Fatal;
    FindFreeBoxResult result;
};

SearchDepthRange normalize_search_depth_range(const FindFreeBoxOptions& options,
                                              const BoxOracle& oracle) {
    const int tree_max_depth = std::max(0, oracle.max_tree_depth() - 1);
    const int max_depth = std::clamp(options.max_depth, 0, tree_max_depth);
    return {
        .start_depth = std::clamp(options.start_depth, 0, max_depth),
        .max_depth = max_depth,
    };
}

double elapsed_ms_since(SteadyTimePoint start) {
    return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

bool should_stop_search(StageContext& context,
                        const FindFreeBoxOptions& options,
                        SteadyTimePoint start,
                        FindFreeBoxResult& result) {
    const bool deadline_limit_hit = options.deadline_ms > 0.0 && elapsed_ms_since(start) > options.deadline_ms;
    if (!context.should_stop() && !deadline_limit_hit) {
        return false;
    }
    result.deadline_reached = context.deadline().expired() || deadline_limit_hit;
    result.fail_code = 4;
    return true;
}

void descend_to_seed_child(BoxOracle& oracle,
                           const Eigen::Ref<const Eigen::VectorXd>& seed,
                           OracleNodeId& node,
                           int& changed_dim) {
    changed_dim = oracle.split_dim(node);
    node = (seed[changed_dim] <= oracle.split_value(node))
        ? oracle.left_child(node)
        : oracle.right_child(node);
}

void record_elapsed(StageContext& context,
                    const std::string& key,
                    SteadyTimePoint start) {
    context.diagnostics().record_timing(
        key,
        std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count());
}

void set_max_diagnostic(StageContext& context, const std::string& key, double value) {
    context.diagnostics().set_value(key, std::max(context.diagnostics().value(key), value));
}

void record_split_diagnostics(StageContext& context,
                              const SplitNodeResult& split,
                              const std::vector<Interval>& intervals,
                              int depth) {
    if (!split.split || split.split_dim < 0 ||
        split.split_dim >= static_cast<int>(intervals.size())) {
        return;
    }
    const int dim = split.split_dim;
    context.diagnostics().add_counter("oracle.split_dim." + std::to_string(dim));
    context.diagnostics().add_counter("oracle.split_depth." + std::to_string(depth));

    double min_width = std::numeric_limits<double>::infinity();
    double max_width = 0.0;
    for (const auto& interval : intervals) {
        const double width = std::max(0.0, interval.width());
        if (width > 0.0) {
            min_width = std::min(min_width, width);
        }
        max_width = std::max(max_width, width);
    }
    const double split_width = std::max(0.0, intervals[static_cast<std::size_t>(dim)].width());
    context.diagnostics().add_counter("oracle.split_width_sum." + std::to_string(dim), split_width);
    set_max_diagnostic(context, "oracle.split_width_max." + std::to_string(dim), split_width);
    if (std::isfinite(min_width) && min_width > 0.0) {
        set_max_diagnostic(context, "oracle.split_parent_aspect_ratio_max", max_width / min_width);
    }
}

DepthProbeResult probe_binary_target_depth(BoxOracle& oracle,
                                           const Eigen::Ref<const Eigen::VectorXd>& seed,
                                           StageContext& context,
                                           const FindFreeBoxOptions& options,
                                           int target_depth,
                                           SteadyTimePoint start) {
    FindFreeBoxResult result;
    OracleNodeId node = oracle.root_node();
    int changed_dim = -1;
    while (true) {
        if (should_stop_search(context, options, start, result)) {
            return {DepthProbeOutcome::Fatal, std::move(result)};
        }

        const auto intervals_start = SteadyClock::now();
        auto intervals = oracle.node_intervals(node);
        record_elapsed(context, "oracle.node_intervals", intervals_start);
        const int node_depth = oracle.depth(node);

        if (node_depth >= target_depth) {
            if (oracle.is_reserved(node)) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 2;
                return {DepthProbeOutcome::NonFree, std::move(result)};
            }

            const auto validation_start = SteadyClock::now();
            const auto validation = oracle.validate_node(node, intervals, changed_dim);
            result.validation_detail = oracle.last_validation_detail();
            record_elapsed(context, "oracle.validate_node", validation_start);
            result.decisions += 1;
            if (validation == BoxValidation::Free) {
                result.found = true;
                result.node = node;
                result.changed_dim = changed_dim;
                result.intervals = std::move(intervals);
                result.fail_code = 0;
                return {DepthProbeOutcome::Free, std::move(result)};
            }
            result.node = node;
            result.intervals = std::move(intervals);
            if (validation == BoxValidation::Occupied) {
                result.fail_code = 3;
            } else {
                result.hit_unknown_depth_cap = true;
                result.fail_code = 2;
            }
            return {DepthProbeOutcome::NonFree, std::move(result)};
        }

        if (oracle.is_reserved(node)) {
            if (!options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 2;
                return {DepthProbeOutcome::NonFree, std::move(result)};
            }
            if (oracle.is_leaf(node)) {
                const auto split_start = SteadyClock::now();
                const int split_depth = node_depth;
                const auto split = oracle.split_node(node, intervals, changed_dim, options.split);
                record_elapsed(context, "oracle.split_node", split_start);
                record_split_diagnostics(context, split, intervals, split_depth);
                if (!split.split) {
                    result.fail_code = 6;
                    return {DepthProbeOutcome::Fatal, std::move(result)};
                }
                result.splits += 1;
            }
            descend_to_seed_child(oracle, seed, node, changed_dim);
            continue;
        }

        if (oracle.is_leaf(node)) {
            if (!options.split_unknown_leaf) {
                result.hit_unknown_depth_cap = true;
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 2;
                return {DepthProbeOutcome::NonFree, std::move(result)};
            }
            const auto split_start = SteadyClock::now();
            const int split_depth = node_depth;
            const auto split = oracle.split_node(node, intervals, changed_dim, options.split);
            record_elapsed(context, "oracle.split_node", split_start);
            record_split_diagnostics(context, split, intervals, split_depth);
            if (!split.split) {
                result.fail_code = 6;
                return {DepthProbeOutcome::Fatal, std::move(result)};
            }
            result.splits += 1;
        }
        descend_to_seed_child(oracle, seed, node, changed_dim);
    }
}

FindFreeBoxResult run_linear_search(BoxOracle& oracle,
                                    const Eigen::Ref<const Eigen::VectorXd>& seed,
                                    StageContext& context,
                                    const FindFreeBoxOptions& options,
                                    int effective_max_depth,
                                    SteadyTimePoint start) {
    FindFreeBoxResult result;
    OracleNodeId node = oracle.root_node();
    int changed_dim = -1;
    while (true) {
        if (should_stop_search(context, options, start, result)) {
            break;
        }

        const auto intervals_start = SteadyClock::now();
        auto intervals = oracle.node_intervals(node);
        record_elapsed(context, "oracle.node_intervals", intervals_start);
        if (oracle.is_reserved(node)) {
            if (oracle.depth(node) >= effective_max_depth || !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 2;
                break;
            }
            if (oracle.is_leaf(node)) {
                const auto split_start = SteadyClock::now();
                const int split_depth = oracle.depth(node);
                const auto split = oracle.split_node(node, intervals, changed_dim, options.split);
                record_elapsed(context, "oracle.split_node", split_start);
                record_split_diagnostics(context, split, intervals, split_depth);
                if (!split.split) {
                    result.fail_code = 6;
                    break;
                }
                result.splits += 1;
            }
            descend_to_seed_child(oracle, seed, node, changed_dim);
            continue;
        }

        const auto validation_start = SteadyClock::now();
        const auto validation = oracle.validate_node(node, intervals, changed_dim);
        result.validation_detail = oracle.last_validation_detail();
        record_elapsed(context, "oracle.validate_node", validation_start);
        result.decisions += 1;
        if (validation == BoxValidation::Free) {
            result.found = true;
            result.node = node;
            result.changed_dim = changed_dim;
            result.intervals = std::move(intervals);
            result.fail_code = 0;
            break;
        }
        if (validation == BoxValidation::Occupied) {
            result.node = node;
            result.intervals = std::move(intervals);
            result.fail_code = 3;
            break;
        }
        if (oracle.depth(node) >= effective_max_depth || !options.split_unknown_leaf) {
            result.hit_unknown_depth_cap = true;
            result.node = node;
            result.intervals = std::move(intervals);
            result.fail_code = 2;
            break;
        }
        if (oracle.is_leaf(node)) {
            const auto split_start = SteadyClock::now();
            const int split_depth = oracle.depth(node);
            const auto split = oracle.split_node(node, intervals, changed_dim, options.split);
            record_elapsed(context, "oracle.split_node", split_start);
            record_split_diagnostics(context, split, intervals, split_depth);
            if (!split.split) {
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
        }
        descend_to_seed_child(oracle, seed, node, changed_dim);
    }
    return result;
}

FindFreeBoxResult run_binary_depth_search(BoxOracle& oracle,
                                          const Eigen::Ref<const Eigen::VectorXd>& seed,
                                          StageContext& context,
                                          const FindFreeBoxOptions& options,
                                          const SearchDepthRange& depth_range,
                                          SteadyTimePoint start) {
    int total_decisions = 0;
    int total_splits = 0;
    auto absorb_probe = [&](const FindFreeBoxResult& probe_result) {
        total_decisions += probe_result.decisions;
        total_splits += probe_result.splits;
    };
    auto finalize = [&](FindFreeBoxResult probe_result) {
        probe_result.decisions = total_decisions;
        probe_result.splits = total_splits;
        return probe_result;
    };

    auto lower_probe = probe_binary_target_depth(oracle, seed, context, options, depth_range.start_depth, start);
    absorb_probe(lower_probe.result);
    if (lower_probe.outcome != DepthProbeOutcome::NonFree || depth_range.start_depth >= depth_range.max_depth) {
        return finalize(std::move(lower_probe.result));
    }

    auto upper_probe = probe_binary_target_depth(oracle, seed, context, options, depth_range.max_depth, start);
    absorb_probe(upper_probe.result);
    if (upper_probe.outcome != DepthProbeOutcome::Free) {
        return finalize(std::move(upper_probe.result));
    }

    int nonfree_depth = depth_range.start_depth;
    int free_depth = depth_range.max_depth;
    FindFreeBoxResult best_free = std::move(upper_probe.result);
    while (free_depth - nonfree_depth > 1) {
        const int mid_depth = nonfree_depth + (free_depth - nonfree_depth) / 2;
        auto mid_probe = probe_binary_target_depth(oracle, seed, context, options, mid_depth, start);
        absorb_probe(mid_probe.result);
        if (mid_probe.outcome == DepthProbeOutcome::Fatal) {
            return finalize(std::move(mid_probe.result));
        }
        if (mid_probe.outcome == DepthProbeOutcome::Free) {
            free_depth = mid_depth;
            best_free = std::move(mid_probe.result);
            continue;
        }
        nonfree_depth = mid_depth;
    }
    return finalize(std::move(best_free));
}

}  // namespace

FindFreeBoxResult FindFreeBoxService::find(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                           const FindFreeBoxOptions& options) {
    StageContext context = StageContext::serial();
    return find(seed, context, options);
}

FindFreeBoxResult FindFreeBoxService::find(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                           StageContext& context,
                                           const FindFreeBoxOptions& options) {
    ScopedStageTimer function_timer(context.diagnostics(), "ffb.find");
    const auto start = SteadyClock::now();
    FindFreeBoxResult result;
    if (context.should_stop()) {
        result.deadline_reached = context.deadline().expired();
        result.fail_code = 4;
        return result;
    }
    bool seed_in_domain = false;
    if (seed.size() == oracle_.n_dims()) {
        const auto contains_start = SteadyClock::now();
        seed_in_domain = oracle_.contains_point(oracle_.root_node(), seed);
        record_elapsed(context, "oracle.contains_point", contains_start);
    }
    if (seed.size() != oracle_.n_dims() || !seed_in_domain) {
        result.fail_code = 5;
        return result;
    }
    if (options.reject_seed_collision) {
        const auto collision_start = SteadyClock::now();
        const bool seed_collision = oracle_.point_in_collision(seed);
        record_elapsed(context, "oracle.point_in_collision", collision_start);
        if (seed_collision) {
            result.seed_collision = true;
            result.fail_code = 1;
            return result;
        }
    }

    const auto depth_range = normalize_search_depth_range(options, oracle_);
    switch (options.search_mode) {
    case FindFreeBoxSearchMode::BinaryDepth:
        result = run_binary_depth_search(oracle_, seed, context, options, depth_range, start);
        break;
    case FindFreeBoxSearchMode::Linear:
    default:
        result = run_linear_search(oracle_, seed, context, options, depth_range.max_depth, start);
        break;
    }
    result.total_ms = elapsed_ms_since(start);
    return result;
}

}  // namespace rbf