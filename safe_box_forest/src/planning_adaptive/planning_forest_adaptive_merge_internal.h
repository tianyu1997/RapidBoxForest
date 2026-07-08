#pragma once

#include <rbf/core.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace rbf::adaptive_merge_detail {

inline bool intervals_equal_local(const std::vector<Interval>& lhs,
                                  const std::vector<Interval>& rhs,
                                  double tolerance = 0.0) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (std::abs(lhs[dim].lo - rhs[dim].lo) > tolerance ||
            std::abs(lhs[dim].hi - rhs[dim].hi) > tolerance) {
            return false;
        }
    }
    return true;
}

inline bool clip_intervals_to_domain_local(std::vector<Interval>& intervals,
                                           const std::vector<Interval>& domain) {
    if (domain.empty()) {
        return !intervals.empty();
    }
    if (intervals.size() != domain.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
        intervals[dim].lo = std::max(intervals[dim].lo, domain[dim].lo);
        intervals[dim].hi = std::min(intervals[dim].hi, domain[dim].hi);
        if (intervals[dim].lo > intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

inline bool intervals_touch_or_overlap_local(const Interval& lhs, const Interval& rhs, double tolerance) {
    return lhs.lo <= rhs.hi + tolerance && rhs.lo <= lhs.hi + tolerance;
}

}  // namespace rbf::adaptive_merge_detail
