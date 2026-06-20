#pragma once

#include <SBF/find_free_box.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rbf::ffb_internal {

inline void record_elapsed(StageContext& context,
                           const std::string& key,
                           std::chrono::steady_clock::time_point start,
                           bool enabled) {
    if (!enabled) {
        return;
    }
    context.diagnostics().record_timing(
        key,
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count());
}

inline void set_max_diagnostic(StageContext& context,
                               const std::string& key,
                               double value,
                               bool enabled) {
    if (!enabled) {
        return;
    }
    context.diagnostics().set_value(key, std::max(context.diagnostics().value(key), value));
}

inline void record_split_diagnostics(StageContext& context,
                                     const SplitNodeResult& split,
                                     const std::vector<Interval>& intervals,
                                     int depth,
                                     bool enabled) {
    if (!enabled) {
        return;
    }
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
    const double split_width =
        std::max(0.0, intervals[static_cast<std::size_t>(dim)].width());
    context.diagnostics().add_counter("oracle.split_width_sum." + std::to_string(dim),
                                      split_width);
    set_max_diagnostic(context,
                       "oracle.split_width_max." + std::to_string(dim),
                       split_width,
                       enabled);
    if (std::isfinite(min_width) && min_width > 0.0) {
        set_max_diagnostic(context,
                           "oracle.split_parent_aspect_ratio_max",
                           max_width / min_width,
                           enabled);
    }
}

inline std::vector<int> scheduled_depths(const FindFreeBoxOptions& options,
                                         int effective_max_depth) {
    std::vector<int> depths;
    depths.reserve(options.adaptive_depths.size() + 1);
    int previous = -1;
    for (int depth : options.adaptive_depths) {
        depth = std::max(0, std::min(depth, effective_max_depth));
        if (depth == previous) {
            continue;
        }
        depths.push_back(depth);
        previous = depth;
    }
    if (depths.empty() || depths.back() != effective_max_depth) {
        depths.push_back(effective_max_depth);
    }
    return depths;
}

}  // namespace rbf::ffb_internal
