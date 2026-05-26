#include <SBF/find_free_box.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace rbf {

namespace {

void record_elapsed(StageContext& context,
                    const std::string& key,
                    std::chrono::steady_clock::time_point start) {
    context.diagnostics().record_timing(
        key,
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
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

}  // namespace

FindFreeBoxResult FindFreeBoxService::find(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                           const FindFreeBoxOptions& options) {
    StageContext context = StageContext::serial();
    return find(seed, context, options);
}

FindFreeBoxResult FindFreeBoxService::find(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                           StageContext& context,
                                           const FindFreeBoxOptions& options) {
    using Clock = std::chrono::steady_clock;
    ScopedStageTimer function_timer(context.diagnostics(), "ffb.find");
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
        record_elapsed(context, "oracle.contains_point", contains_start);
    }
    if (seed.size() != oracle_.n_dims() || !seed_in_domain) {
        result.fail_code = 5;
        return result;
    }
    if (options.reject_seed_collision) {
        const auto collision_start = Clock::now();
        const bool seed_collision = oracle_.point_in_collision(seed);
        record_elapsed(context, "oracle.point_in_collision", collision_start);
        if (seed_collision) {
            result.seed_collision = true;
            result.fail_code = 1;
            return result;
        }
    }

    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    };

    int node = oracle_.root_node();
    int changed_dim = -1;
    while (true) {
        if (context.should_stop() || (options.deadline_ms > 0.0 && elapsed_ms() > options.deadline_ms)) {
            result.deadline_reached = context.deadline().expired() || options.deadline_ms > 0.0;
            result.fail_code = 4;
            break;
        }

        const auto intervals_start = Clock::now();
        auto intervals = oracle_.node_intervals(node);
        record_elapsed(context, "oracle.node_intervals", intervals_start);
        if (oracle_.is_reserved(node)) {
            if (oracle_.depth(node) >= options.max_depth || !options.split_reserved_leaf) {
                result.hit_reserved_depth_cap = true;
                result.node = node;
                result.intervals = std::move(intervals);
                result.fail_code = 2;
                break;
            }
            if (oracle_.is_leaf(node)) {
                const auto split_start = Clock::now();
                const int split_depth = oracle_.depth(node);
                const auto split = oracle_.split_node(node, intervals, changed_dim, options.split);
                record_elapsed(context, "oracle.split_node", split_start);
                record_split_diagnostics(context, split, intervals, split_depth);
                if (!split.split) {
                    result.fail_code = 6;
                    break;
                }
                result.splits += 1;
            }
            changed_dim = oracle_.split_dim(node);
            node = (seed[changed_dim] <= oracle_.split_value(node))
                ? oracle_.left_child(node)
                : oracle_.right_child(node);
            continue;
        }

        const auto validation_start = Clock::now();
        const auto validation = oracle_.validate_node(node, intervals, changed_dim);
        result.validation_detail = oracle_.last_validation_detail();
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
        if (oracle_.depth(node) >= options.max_depth || !options.split_unknown_leaf) {
            result.hit_unknown_depth_cap = true;
            result.node = node;
            result.intervals = std::move(intervals);
            result.fail_code = 2;
            break;
        }
        if (oracle_.is_leaf(node)) {
            const auto split_start = Clock::now();
            const int split_depth = oracle_.depth(node);
            const auto split = oracle_.split_node(node, intervals, changed_dim, options.split);
            record_elapsed(context, "oracle.split_node", split_start);
            record_split_diagnostics(context, split, intervals, split_depth);
            if (!split.split) {
                result.fail_code = 6;
                break;
            }
            result.splits += 1;
        }
        changed_dim = oracle_.split_dim(node);
        node = (seed[changed_dim] <= oracle_.split_value(node))
            ? oracle_.left_child(node)
            : oracle_.right_child(node);
    }
    result.total_ms = elapsed_ms();
    return result;
}

}  // namespace rbf