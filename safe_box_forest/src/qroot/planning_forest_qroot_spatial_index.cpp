#include "planning_forest_qroot_helpers.h"

#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace rbf {

int BoxSpatialIndex::choose_dim(const std::vector<BoxNode>& boxes) {
    if (boxes.empty()) {
        return -1;
    }
    const int nd = boxes.front().n_dims();
    int best_dim = -1;
    double best_span = -1.0;
    for (int dim = 0; dim < nd; ++dim) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        for (const auto& box : boxes) {
            if (box.n_dims() != nd) {
                continue;
            }
            lo = std::min(lo, box.joint_intervals[dim].lo);
            hi = std::max(hi, box.joint_intervals[dim].hi);
        }
        const double span = hi - lo;
        if (std::isfinite(span) && span > best_span) {
            best_span = span;
            best_dim = dim;
        }
    }
    return best_dim;
}

long long BoxSpatialIndex::bin_of(double value, double origin_value, double width) {
    return static_cast<long long>(std::floor((value - origin_value) / std::max(width, 1e-12)));
}

void BoxSpatialIndex::rebuild(const std::vector<BoxNode>& boxes, double tolerance) {
    bins.clear();
    index_dim = choose_dim(boxes);
    if (index_dim < 0 || boxes.empty()) {
        return;
    }
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    double width_sum = 0.0;
    int width_count = 0;
    for (const auto& box : boxes) {
        if (box.n_dims() <= index_dim) {
            continue;
        }
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(index_dim)];
        lo = std::min(lo, interval.lo);
        hi = std::max(hi, interval.hi);
        width_sum += std::max(0.0, interval.width());
        width_count += 1;
    }
    origin = std::isfinite(lo) ? lo - tolerance : 0.0;
    const double span = std::max(hi - lo, 1e-9);
    const double avg_width = width_count > 0 ? width_sum / static_cast<double>(width_count) : span;
    bin_width = std::max({span / 128.0, avg_width, tolerance * 4.0, 1e-9});
    bins.reserve(std::max<std::size_t>(1, boxes.size() * 2));
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        add_box(boxes[static_cast<std::size_t>(index)], index, tolerance);
    }
}

void BoxSpatialIndex::add_box(const BoxNode& box, int index, double tolerance) {
    if (index_dim < 0 || box.n_dims() <= index_dim) {
        return;
    }
    const auto& interval = box.joint_intervals[static_cast<std::size_t>(index_dim)];
    const long long lo_bin = bin_of(interval.lo - tolerance, origin, bin_width);
    const long long hi_bin = bin_of(interval.hi + tolerance, origin, bin_width);
    for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
        bins[bin].push_back(index);
    }
}

std::vector<int> BoxSpatialIndex::interval_candidates(const std::vector<Interval>& intervals,
                                                      double tolerance) const {
    std::vector<int> out;
    if (index_dim < 0 || index_dim >= static_cast<int>(intervals.size())) {
        return out;
    }
    std::unordered_set<int> seen;
    const auto& interval = intervals[static_cast<std::size_t>(index_dim)];
    const long long lo_bin = bin_of(interval.lo - tolerance, origin, bin_width);
    const long long hi_bin = bin_of(interval.hi + tolerance, origin, bin_width);
    for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
        auto it = bins.find(bin);
        if (it == bins.end()) {
            continue;
        }
        for (int index : it->second) {
            if (seen.insert(index).second) {
                out.push_back(index);
            }
        }
    }
    return out;
}

std::vector<int> BoxSpatialIndex::point_candidates(const Eigen::Ref<const Eigen::VectorXd>& point) const {
    std::vector<int> out;
    if (index_dim < 0 || index_dim >= point.size()) {
        return out;
    }
    auto it = bins.find(bin_of(point[index_dim], origin, bin_width));
    if (it != bins.end()) {
        out = it->second;
    }
    return out;
}

int BoxSpatialIndex::covering_box(const std::vector<BoxNode>& boxes,
                                  const Eigen::Ref<const Eigen::VectorXd>& point,
                                  double tolerance) const {
    int best = -1;
    double best_volume = std::numeric_limits<double>::infinity();
    auto candidates = point_candidates(point);
    if (candidates.empty()) {
        candidates.reserve(boxes.size());
        for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
            candidates.push_back(index);
        }
    }
    for (int index : candidates) {
        if (index < 0 || index >= static_cast<int>(boxes.size())) {
            continue;
        }
        const auto& box = boxes[static_cast<std::size_t>(index)];
        if (intervals_contain_point_local(box.joint_intervals, point, tolerance) &&
            box.volume < best_volume) {
            best = index;
            best_volume = box.volume;
        }
    }
    return best;
}

}  // namespace rbf
