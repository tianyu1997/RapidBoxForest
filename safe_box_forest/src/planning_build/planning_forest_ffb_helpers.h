#pragma once

#include <rbf/core.h>

#include <Eigen/Core>

#include <algorithm>
#include <vector>

namespace rbf::forest_ffb_internal {

inline bool intervals_contain_point_strict(const std::vector<Interval>& intervals,
                                           const Eigen::Ref<const Eigen::VectorXd>& point,
                                           double tolerance = 0.0) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return false;
    }
    for (int dim = 0; dim < point.size(); ++dim) {
        if (point[dim] < intervals[static_cast<std::size_t>(dim)].lo - tolerance ||
            point[dim] > intervals[static_cast<std::size_t>(dim)].hi + tolerance) {
            return false;
        }
    }
    return true;
}

inline bool box_contains_point_exact(const BoxNode& box,
                                     const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (box.n_dims() != point.size()) {
        return false;
    }
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (point[dim] < box.joint_intervals[dim].lo ||
            point[dim] > box.joint_intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

inline bool point_covered_by_existing_box(const std::vector<BoxNode>& boxes,
                                          const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return box_contains_point_exact(box, point);
    });
}

inline bool intervals_overlap(const std::vector<Interval>& lhs,
                              const std::vector<Interval>& rhs,
                              double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        if (lhs[dim].hi + tolerance < rhs[dim].lo ||
            rhs[dim].hi + tolerance < lhs[dim].lo) {
            return false;
        }
    }
    return true;
}

inline bool intervals_subset(const std::vector<Interval>& inner,
                             const std::vector<Interval>& outer,
                             double tolerance = 0.0) {
    if (inner.size() != outer.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < inner.size(); ++dim) {
        if (inner[dim].lo < outer[dim].lo - tolerance ||
            inner[dim].hi > outer[dim].hi + tolerance) {
            return false;
        }
    }
    return true;
}

}  // namespace rbf::forest_ffb_internal
