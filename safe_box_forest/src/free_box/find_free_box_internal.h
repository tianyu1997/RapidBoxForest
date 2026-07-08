#pragma once

#include <SBF/find_free_box_types.h>
#include <SBF/runtime_fwd.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

namespace rbf::ffb_internal {

void record_elapsed(StageContext& context,
                    const std::string& key,
                    std::chrono::steady_clock::time_point start,
                    bool enabled);

void set_max_diagnostic(StageContext& context,
                        const std::string& key,
                        double value,
                        bool enabled);

void record_split_diagnostics(StageContext& context,
                              const SplitNodeResult& split,
                              const std::vector<Interval>& intervals,
                              int depth,
                              bool enabled);

void record_free_ancestor_diagnostics(StageContext& context,
                                      const std::vector<Interval>& intervals,
                                      double free_depth,
                                      bool enabled,
                                      bool record_log_volume = true);

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
