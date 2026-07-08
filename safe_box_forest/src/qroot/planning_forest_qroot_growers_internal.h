#pragma once

#include "planning_forest_qroot_helpers.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <limits>
#include <vector>

namespace rbf {

inline int find_qroot_containing_domain_index(const std::vector<BoxNode>& domains,
                                              const BoxSpatialIndex& domain_index,
                                              const Eigen::Ref<const Eigen::VectorXd>& point,
                                              double tolerance) {
    auto candidates = domain_index.point_candidates(point);
    if (candidates.empty()) {
        candidates.reserve(domains.size());
        for (int index = 0; index < static_cast<int>(domains.size()); ++index) {
            candidates.push_back(index);
        }
    }
    int best = -1;
    double best_volume = std::numeric_limits<double>::infinity();
    for (int index : candidates) {
        if (index < 0 || index >= static_cast<int>(domains.size())) {
            continue;
        }
        const auto& domain = domains[static_cast<std::size_t>(index)];
        if (intervals_contain_point_strict_local(domain.joint_intervals, point, tolerance) &&
            domain.volume < best_volume) {
            best = index;
            best_volume = domain.volume;
        }
    }
    return best;
}

}  // namespace rbf
