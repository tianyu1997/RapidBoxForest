#include <SBF/grower.h>

#include <SBF/oracle.h>

#include "grower_components.h"

#include <random>
#include <utility>
#include <vector>

namespace rbf {

Eigen::VectorXd RrtGrower::sample_uniform() {
    const auto root = oracle_.planning_intervals();
    Eigen::VectorXd q(static_cast<int>(root.size()));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int dim = 0; dim < static_cast<int>(root.size()); ++dim) {
        q[dim] = root[static_cast<std::size_t>(dim)].lo +
                 u01(rng_) * root[static_cast<std::size_t>(dim)].width();
    }
    return q;
}

Eigen::VectorXd RrtGrower::sample_unexplored() {
    const OracleNodeId node = oracle_.select_unexplored_node();
    std::vector<Interval> intervals;
    if (node >= 0) {
        auto copies = oracle_.native_interval_copies_for_node(node, oracle_.node_intervals(node));
        if (!copies.empty()) {
            std::uniform_int_distribution<std::size_t> pick(0, copies.size() - 1);
            intervals = std::move(copies[pick(rng_)]);
            if (!clip_intervals_to_root(intervals, oracle_.planning_intervals())) {
                intervals.clear();
            }
        }
    }
    if (intervals.empty()) {
        intervals = oracle_.planning_intervals();
    }
    Eigen::VectorXd q(static_cast<int>(intervals.size()));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        q[dim] = intervals[static_cast<std::size_t>(dim)].lo +
                 u01(rng_) * intervals[static_cast<std::size_t>(dim)].width();
    }
    return q;
}

}  // namespace rbf
