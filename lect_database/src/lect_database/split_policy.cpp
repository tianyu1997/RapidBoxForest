#include <rbf/lect_database/split_policy.h>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rbf::lect_database {

namespace {

double interval_width(const Interval& interval) {
    return std::max(0.0, interval.width());
}

std::string serialize_depth_dimensions(const std::vector<int>& dims) {
    std::ostringstream out;
    for (std::size_t index = 0; index < dims.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        out << dims[index];
    }
    return out.str();
}

std::string effective_schedule_hash(const SplitPolicyDescriptor& descriptor) {
    if (!descriptor.dimension_schedule_hash.empty()) {
        return descriptor.dimension_schedule_hash;
    }
    if (descriptor.depth_dimensions.empty()) {
        return {};
    }
    return std::to_string(stable_hash(serialize_depth_dimensions(descriptor.depth_dimensions)));
}

bool allowed_dimension(const std::vector<Interval>& intervals, int dim, double min_width) {
    return dim >= 0 && dim < static_cast<int>(intervals.size()) &&
           interval_width(intervals[static_cast<std::size_t>(dim)]) > min_width;
}

int widest_dimension(const std::vector<Interval>& intervals, double min_width) {
    int best_dim = -1;
    double best_width = -1.0;
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        const double width = interval_width(intervals[static_cast<std::size_t>(dim)]);
        if (width <= min_width) {
            continue;
        }
        if (width > best_width) {
            best_width = width;
            best_dim = dim;
        }
    }
    return best_dim;
}

}  // namespace

const char* split_strategy_name(SplitStrategy strategy) noexcept {
    switch (strategy) {
    case SplitStrategy::RoundRobin: return "round_robin";
    case SplitStrategy::WidestRoot: return "widest_root";
    case SplitStrategy::AAFKVolumeMin: return "aafk_volume_min";
    }
    return "unknown";
}

std::string split_policy_descriptor(const SplitPolicyDescriptor& descriptor) {
    std::ostringstream out;
    out << "split=" << split_strategy_name(descriptor.strategy)
        << "|depth_sync=1"
        << "|min_width=" << descriptor.min_width
        << "|midpoint=" << (descriptor.midpoint ? 1 : 0)
        << "|tie=" << (descriptor.deterministic_tie_break ? "stable" : "unspecified")
        << "|schedule=" << effective_schedule_hash(descriptor);
    return out.str();
}

std::uint64_t split_policy_hash(const SplitPolicyDescriptor& descriptor) {
    return stable_hash(split_policy_descriptor(descriptor));
}

SplitPolicy::SplitPolicy(SplitPolicyDescriptor descriptor)
    : descriptor_(std::move(descriptor)) {}

int SplitPolicy::choose_dimension(const std::vector<Interval>& root_intervals,
                                  const std::vector<Interval>& node_intervals,
                                  int depth) const {
    if (node_intervals.empty()) {
        return -1;
    }
    if (descriptor_.strategy == SplitStrategy::AAFKVolumeMin) {
        if (depth < 0 || depth >= static_cast<int>(descriptor_.depth_dimensions.size())) {
            return -1;
        }
        const int dim = descriptor_.depth_dimensions[static_cast<std::size_t>(depth)];
        if (allowed_dimension(node_intervals, dim, descriptor_.min_width)) {
            return dim;
        }
        return widest_dimension(node_intervals, 0.0);
    }
    if (descriptor_.strategy == SplitStrategy::RoundRobin) {
        const int dim = depth % static_cast<int>(node_intervals.size());
        if (allowed_dimension(node_intervals, dim, descriptor_.min_width)) {
            return dim;
        }
        return widest_dimension(node_intervals, 0.0);
    }

    std::vector<int> order(static_cast<std::size_t>(node_intervals.size()));
    for (int dim = 0; dim < static_cast<int>(order.size()); ++dim) {
        order[static_cast<std::size_t>(dim)] = dim;
    }
    std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        const double lhs_width = lhs < static_cast<int>(root_intervals.size())
            ? interval_width(root_intervals[static_cast<std::size_t>(lhs)])
            : 0.0;
        const double rhs_width = rhs < static_cast<int>(root_intervals.size())
            ? interval_width(root_intervals[static_cast<std::size_t>(rhs)])
            : 0.0;
        if (lhs_width != rhs_width) {
            return lhs_width > rhs_width;
        }
        return lhs < rhs;
    });
    const int dim = order[static_cast<std::size_t>(depth % static_cast<int>(order.size()))];
    if (allowed_dimension(node_intervals, dim, descriptor_.min_width)) {
        return dim;
    }
    return widest_dimension(node_intervals, 0.0);
}

double SplitPolicy::choose_split_value(const Interval& interval) const {
    return descriptor_.midpoint ? 0.5 * (interval.lo + interval.hi) : interval.center();
}

}  // namespace rbf::lect_database
