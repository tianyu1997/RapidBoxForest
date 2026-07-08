#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rbf {

Eigen::VectorXd closest_point_in_box(const BoxNode& box,
                                     const Eigen::Ref<const Eigen::VectorXd>& point) {
    Eigen::VectorXd out(point.size());
    for (int dim = 0; dim < point.size(); ++dim) {
        if (dim < static_cast<int>(box.joint_intervals.size())) {
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
            out[dim] = std::min(interval.hi, std::max(interval.lo, point[dim]));
        } else {
            out[dim] = point[dim];
        }
    }
    return out;
}

double segment_exit_parameter_from_intervals(const std::vector<Interval>& intervals,
                                             const Eigen::Ref<const Eigen::VectorXd>& from,
                                             const Eigen::Ref<const Eigen::VectorXd>& to) {
    if (intervals.size() != static_cast<std::size_t>(from.size()) ||
        to.size() != from.size()) {
        return 0.0;
    }
    const Eigen::VectorXd delta = to - from;
    double exit_param = 1.0;
    for (int dim = 0; dim < from.size(); ++dim) {
        const double d = delta[dim];
        if (std::abs(d) < 1e-15) {
            continue;
        }
        const auto& interval = intervals[static_cast<std::size_t>(dim)];
        const double boundary = d > 0.0 ? interval.hi : interval.lo;
        const double t = (boundary - from[dim]) / d;
        if (t > 1e-12 && t < exit_param) {
            exit_param = t;
        }
    }
    return std::clamp(exit_param, 0.0, 1.0);
}

Eigen::VectorXd boundary_seed_from_intervals(const std::vector<Interval>& intervals,
                                             const Eigen::Ref<const Eigen::VectorXd>& from,
                                             const Eigen::Ref<const Eigen::VectorXd>& to,
                                             const std::vector<Interval>& domain,
                                             double face_epsilon) {
    const Eigen::VectorXd delta = to - from;
    const double norm = delta.norm();
    if (norm <= 1e-12) {
        return from;
    }
    const double u = segment_exit_parameter_from_intervals(intervals, from, to);
    Eigen::VectorXd seed = from + u * delta + face_epsilon * (delta / norm);
    for (int dim = 0; dim < seed.size() &&
                      dim < static_cast<int>(domain.size()); ++dim) {
        seed[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                             std::max(domain[static_cast<std::size_t>(dim)].lo,
                                      seed[dim]));
    }
    return seed;
}

std::vector<Eigen::VectorXd> lateral_offset_seeds_local(
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    const Eigen::Ref<const Eigen::VectorXd>& direction,
    const std::vector<Interval>& domain,
    int lateral_rounds,
    double lateral_offset) {
    std::vector<int> dims;
    dims.reserve(static_cast<std::size_t>(seed.size()));
    for (int dim = 0; dim < seed.size(); ++dim) {
        dims.push_back(dim);
    }
    std::sort(dims.begin(), dims.end(), [&](int lhs, int rhs) {
        return std::abs(direction[lhs]) < std::abs(direction[rhs]);
    });
    std::vector<Eigen::VectorXd> out;
    const int dim_limit = std::min<int>(std::max(0, lateral_rounds),
                                        static_cast<int>(dims.size()));
    out.reserve(static_cast<std::size_t>(dim_limit) * 2);
    for (int item = 0; item < dim_limit; ++item) {
        const int dim = dims[static_cast<std::size_t>(item)];
        for (double sign : {1.0, -1.0}) {
            Eigen::VectorXd candidate = seed;
            candidate[dim] += sign * lateral_offset;
            if (dim < static_cast<int>(domain.size())) {
                candidate[dim] = std::min(domain[static_cast<std::size_t>(dim)].hi,
                                          std::max(domain[static_cast<std::size_t>(dim)].lo,
                                                   candidate[dim]));
            }
            out.push_back(std::move(candidate));
        }
    }
    return out;
}

double interval_point_gap_local(const Interval& interval, double value) {
    if (value < interval.lo) {
        return interval.lo - value;
    }
    if (value > interval.hi) {
        return value - interval.hi;
    }
    return 0.0;
}

double intervals_point_gap_local(const std::vector<Interval>& intervals,
                                 const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return std::numeric_limits<double>::infinity();
    }
    double gap = 0.0;
    for (int dim = 0; dim < point.size(); ++dim) {
        gap = std::max(gap, interval_point_gap_local(intervals[static_cast<std::size_t>(dim)], point[dim]));
    }
    return gap;
}

bool intervals_contain_point_local(const std::vector<Interval>& intervals,
                                   const Eigen::Ref<const Eigen::VectorXd>& point,
                                   double tolerance) {
    return intervals_point_gap_local(intervals, point) <= tolerance;
}

bool intervals_contain_point_strict_local(const std::vector<Interval>& intervals,
                                          const Eigen::Ref<const Eigen::VectorXd>& point,
                                          double tolerance) {
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

bool intervals_equal_local(const std::vector<Interval>& lhs,
                           const std::vector<Interval>& rhs,
                           double tolerance) {
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

std::unordered_map<OracleNodeId, int> build_box_node_index(
    const std::vector<BoxNode>& boxes,
    std::size_t reserve_extra) {
    std::unordered_map<OracleNodeId, int> node_to_box_index;
    node_to_box_index.reserve(boxes.size() + reserve_extra);
    for (std::size_t box_index = 0; box_index < boxes.size(); ++box_index) {
        const auto node = static_cast<OracleNodeId>(boxes[box_index].tree_id);
        if (node != kInvalidOracleNodeId &&
            node_to_box_index.find(node) == node_to_box_index.end()) {
            node_to_box_index[node] = static_cast<int>(box_index);
        }
    }
    return node_to_box_index;
}

int find_box_index_by_node_or_intervals(
    const std::vector<BoxNode>& boxes,
    const std::unordered_map<OracleNodeId, int>& node_to_box_index,
    OracleNodeId node,
    const std::vector<Interval>& intervals,
    double tolerance) {
    if (node != kInvalidOracleNodeId) {
        const auto node_it = node_to_box_index.find(node);
        if (node_it != node_to_box_index.end()) {
            return node_it->second;
        }
        return -1;
    }
    for (std::size_t box_index = 0; box_index < boxes.size(); ++box_index) {
        if (intervals_equal_local(boxes[box_index].joint_intervals, intervals, tolerance)) {
            return static_cast<int>(box_index);
        }
    }
    return -1;
}

Eigen::VectorXd adaptive_center_of_intervals(const std::vector<Interval>& intervals) {
    Eigen::VectorXd center(static_cast<int>(intervals.size()));
    for (int dim = 0; dim < center.size(); ++dim) {
        center[dim] = intervals[static_cast<std::size_t>(dim)].center();
    }
    return center;
}

BoxNode adaptive_make_box_from_intervals(const std::vector<Interval>& intervals,
                                         OracleNodeId node,
                                         int id,
                                         BoxSafetyStatus status,
                                         bool strict_audit_required) {
    BoxNode box;
    box.id = id;
    box.joint_intervals = intervals;
    box.seed_config = adaptive_center_of_intervals(intervals);
    box.tree_id = node;
    box.parent_box_id = -1;
    box.root_id = id;
    box.safety_status = status;
    box.strict_audit_required = strict_audit_required;
    box.compute_volume();
    return box;
}

std::optional<std::pair<double, double>> segment_box_parameter_interval(
    const Eigen::Ref<const Eigen::VectorXd>& a,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const BoxNode& box,
    double tolerance) {
    if (box.n_dims() != a.size() || b.size() != a.size()) {
        return std::nullopt;
    }
    double lo = 0.0;
    double hi = 1.0;
    const Eigen::VectorXd delta = b - a;
    for (int dim = 0; dim < a.size(); ++dim) {
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        const double slab_lo = interval.lo - tolerance;
        const double slab_hi = interval.hi + tolerance;
        if (std::abs(delta[dim]) < 1e-15) {
            if (a[dim] < slab_lo || a[dim] > slab_hi) {
                return std::nullopt;
            }
            continue;
        }
        double t0 = (slab_lo - a[dim]) / delta[dim];
        double t1 = (slab_hi - a[dim]) / delta[dim];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        lo = std::max(lo, t0);
        hi = std::min(hi, t1);
        if (lo > hi) {
            return std::nullopt;
        }
    }
    lo = std::max(0.0, lo);
    hi = std::min(1.0, hi);
    if (lo > hi) {
        return std::nullopt;
    }
    return std::pair<double, double>{lo, hi};
}

double certified_box_covered_segment_length(const Eigen::Ref<const Eigen::VectorXd>& a,
                                            const Eigen::Ref<const Eigen::VectorXd>& b,
                                            const std::vector<BoxNode>& boxes,
                                            double tolerance) {
    const double segment_length = (b - a).norm();
    if (segment_length <= 1e-15) {
        return 0.0;
    }
    std::vector<std::pair<double, double>> covered;
    covered.reserve(boxes.size());
    for (const auto& box : boxes) {
        if (box.safety_status != BoxSafetyStatus::CertifiedFree ||
            box.strict_audit_required) {
            continue;
        }
        auto interval = segment_box_parameter_interval(a, b, box, tolerance);
        if (interval && interval->second > interval->first) {
            covered.push_back(*interval);
        }
    }
    if (covered.empty()) {
        return 0.0;
    }
    std::sort(covered.begin(), covered.end());
    double covered_param = 0.0;
    double cur_lo = covered.front().first;
    double cur_hi = covered.front().second;
    for (std::size_t index = 1; index < covered.size(); ++index) {
        const auto [next_lo, next_hi] = covered[index];
        if (next_lo <= cur_hi + 1e-12) {
            cur_hi = std::max(cur_hi, next_hi);
        } else {
            covered_param += std::max(0.0, cur_hi - cur_lo);
            cur_lo = next_lo;
            cur_hi = next_hi;
        }
    }
    covered_param += std::max(0.0, cur_hi - cur_lo);
    return std::min(segment_length, std::max(0.0, covered_param) * segment_length);
}

double uncovered_segment_edge_length(const SegmentEdge& edge,
                                     const std::vector<BoxNode>& boxes,
                                     double tolerance) {
    if (edge.waypoints.size() < 2) {
        return edge.length;
    }
    double uncovered = 0.0;
    for (std::size_t index = 1; index < edge.waypoints.size(); ++index) {
        const auto& a = edge.waypoints[index - 1];
        const auto& b = edge.waypoints[index];
        const double segment_length = (b - a).norm();
        const double covered =
            certified_box_covered_segment_length(a, b, boxes, tolerance);
        uncovered += std::max(0.0, segment_length - covered);
    }
    if (edge.obb_covered_length > 0.0) {
        uncovered = std::max(0.0, uncovered - edge.obb_covered_length);
    }
    return uncovered;
}

}  // namespace rbf
