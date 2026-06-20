#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

namespace rbf {

RRTConnectConfig with_query_root_hull_domain(const RRTConnectConfig& config,
                                             const BoxOracle& oracle,
                                             const Eigen::Ref<const Eigen::VectorXd>& start,
                                             const Eigen::Ref<const Eigen::VectorXd>& goal) {
    RRTConnectConfig out = config;
    auto lhs = oracle.planning_intervals();
    auto rhs = oracle.planning_intervals();
    (void)start;
    (void)goal;
    if (lhs.size() == rhs.size()) {
        for (std::size_t index = 0; index < lhs.size(); ++index) {
            lhs[index] = lhs[index].hull(rhs[index]);
        }
    }
    out.domain_intervals = std::move(lhs);
    return out;
}

const SegmentEdge* find_segment_edge_by_id(const SegmentEdgeList& edges, int edge_id) {
    for (const auto& edge : edges) {
        if (edge.id == edge_id) {
            return &edge;
        }
    }
    return nullptr;
}

Robot make_sbf_clearance_robot(const Robot& robot, double clearance) {
    if (!(clearance > 0.0) || !std::isfinite(clearance) || robot.link_radii().empty()) {
        return robot;
    }
    std::vector<double> radii = robot.link_radii();
    for (double& radius : radii) {
        if (radius > 0.0) {
            radius += clearance;
        }
    }
    return Robot(robot.name(),
                 robot.dh_params(),
                 robot.joint_limits(),
                 robot.tool_frame(),
                 std::move(radii));
}

const BoxNode* find_box_by_id(const std::vector<BoxNode>& boxes, int box_id) {
    for (const auto& box : boxes) {
        if (box.id == box_id) {
            return &box;
        }
    }
    return nullptr;
}

bool partition_boxes_connected_local(const BoxNode& lhs,
                                     const BoxNode& rhs,
                                     double tolerance) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return false;
    }
    int shared_dims = 0;
    int overlap_dims = 0;
    for (int dim = 0; dim < nd; ++dim) {
        const auto& lhs_iv = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& rhs_iv = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        const double overlap_lo = std::max(lhs_iv.lo, rhs_iv.lo);
        const double overlap_hi = std::min(lhs_iv.hi, rhs_iv.hi);
        if (overlap_hi < overlap_lo - tolerance) {
            return false;
        }
        if (overlap_hi - overlap_lo < tolerance) {
            shared_dims += 1;
        } else {
            overlap_dims += 1;
        }
    }
    return shared_dims >= 1 || overlap_dims == nd;
}

Eigen::VectorXd partition_shared_face_center_local(const BoxNode& lhs,
                                                   const BoxNode& rhs) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return (lhs.center() + rhs.center()) * 0.5;
    }
    Eigen::VectorXd center(nd);
    int face_dim = -1;
    for (int dim = 0; dim < nd; ++dim) {
        const auto& lhs_iv = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& rhs_iv = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        const double overlap_lo = std::max(lhs_iv.lo, rhs_iv.lo);
        const double overlap_hi = std::min(lhs_iv.hi, rhs_iv.hi);
        if (overlap_hi < overlap_lo - 1e-9) {
            return (lhs.center() + rhs.center()) * 0.5;
        }
        if (std::abs(overlap_hi - overlap_lo) <= 1e-9) {
            if (face_dim >= 0) {
                return (lhs.center() + rhs.center()) * 0.5;
            }
            face_dim = dim;
        }
        center[dim] = 0.5 * (overlap_lo + overlap_hi);
    }
    return face_dim >= 0 ? center : (lhs.center() + rhs.center()) * 0.5;
}

Eigen::VectorXd partition_transition_waypoint_local(const BoxNode& lhs,
                                                    const BoxNode& rhs,
                                                    const Eigen::Ref<const Eigen::VectorXd>& from,
                                                    const Eigen::Ref<const Eigen::VectorXd>& goal,
                                                    double tolerance) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims() || from.size() != nd) {
        return partition_shared_face_center_local(lhs, rhs);
    }
    if (!partition_boxes_connected_local(lhs, rhs, tolerance)) {
        return partition_shared_face_center_local(lhs, rhs);
    }
    Eigen::VectorXd target = rhs.center();
    if (goal.size() == nd) {
        target = goal;
    }
    Eigen::VectorXd overlap_mid(nd);
    Eigen::VectorXd overlap_lo(nd);
    Eigen::VectorXd overlap_hi(nd);
    for (int dim = 0; dim < nd; ++dim) {
        const auto& lhs_iv = lhs.joint_intervals[static_cast<std::size_t>(dim)];
        const auto& rhs_iv = rhs.joint_intervals[static_cast<std::size_t>(dim)];
        overlap_lo[dim] = std::max(lhs_iv.lo, rhs_iv.lo);
        overlap_hi[dim] = std::min(lhs_iv.hi, rhs_iv.hi);
        overlap_mid[dim] = 0.5 * (overlap_lo[dim] + overlap_hi[dim]);
    }
    const Eigen::VectorXd local_delta = target - from;
    const double denom = local_delta.squaredNorm();
    if (denom <= 1e-18) {
        return overlap_mid;
    }
    const double t = std::clamp((overlap_mid - from).dot(local_delta) / denom, 0.0, 1.0);
    Eigen::VectorXd waypoint = from + t * local_delta;
    for (int dim = 0; dim < nd; ++dim) {
        waypoint[dim] = std::clamp(waypoint[dim], overlap_lo[dim], overlap_hi[dim]);
    }
    return waypoint;
}

std::vector<Eigen::VectorXd> extract_partition_waypoints_local(
    const std::vector<int>& box_sequence,
    const std::vector<int>& segment_edge_sequence,
    const std::vector<BoxNode>& boxes,
    const SegmentEdgeList& segment_edges,
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    double adjacency_tolerance) {
    std::vector<Eigen::VectorXd> path;
    if (box_sequence.empty()) {
        return path;
    }
    std::unordered_map<int, const BoxNode*> box_by_id;
    box_by_id.reserve(boxes.size());
    for (const auto& box : boxes) {
        box_by_id[box.id] = &box;
    }
    std::unordered_map<int, const SegmentEdge*> edge_by_id;
    edge_by_id.reserve(segment_edges.size());
    for (const auto& edge : segment_edges) {
        edge_by_id[edge.id] = &edge;
    }
    auto box_ptr = [&](int id) -> const BoxNode* {
        const auto it = box_by_id.find(id);
        return it == box_by_id.end() ? nullptr : it->second;
    };
    auto edge_ptr = [&](std::size_t transition_index) -> const SegmentEdge* {
        if (transition_index >= segment_edge_sequence.size()) {
            return nullptr;
        }
        const int edge_id = segment_edge_sequence[transition_index];
        if (edge_id < 0) {
            return nullptr;
        }
        const auto it = edge_by_id.find(edge_id);
        return it == edge_by_id.end() ? nullptr : it->second;
    };
    auto append_if_new = [&](const Eigen::VectorXd& waypoint) {
        if (path.empty() || (path.back() - waypoint).norm() > 1e-12) {
            path.push_back(waypoint);
        }
    };
    path.push_back(start);
    for (std::size_t i = 1; i < box_sequence.size(); ++i) {
        const BoxNode* lhs_ptr = box_ptr(box_sequence[i - 1]);
        const BoxNode* rhs_ptr = box_ptr(box_sequence[i]);
        if (lhs_ptr == nullptr || rhs_ptr == nullptr) {
            continue;
        }
        const BoxNode& lhs = *lhs_ptr;
        const BoxNode& rhs = *rhs_ptr;
        if (const SegmentEdge* edge = edge_ptr(i - 1)) {
            std::vector<Eigen::VectorXd> edge_path = edge->waypoints;
            if (edge->source_box_id == rhs.id && edge->target_box_id == lhs.id) {
                std::reverse(edge_path.begin(), edge_path.end());
            }
            if (edge_path.empty()) {
                edge_path.push_back(lhs.center());
                edge_path.push_back(rhs.center());
            }
            for (const auto& waypoint : edge_path) {
                append_if_new(waypoint);
            }
            continue;
        }
        if (partition_boxes_connected_local(lhs, rhs, adjacency_tolerance)) {
            append_if_new(partition_transition_waypoint_local(lhs, rhs, path.back(), goal, adjacency_tolerance));
        } else {
            append_if_new(lhs.center());
            append_if_new(rhs.center());
        }
    }
    append_if_new(goal);
    return path;
}

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

bool same_waypoint(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
    return lhs.size() == rhs.size() && (lhs - rhs).norm() <= 1e-10;
}

void append_waypoint_unique(std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& waypoint) {
    if (path.empty() || !same_waypoint(path.back(), waypoint)) {
        path.push_back(waypoint);
    }
}

std::vector<Eigen::VectorXd> densify_waypoint_path_local(const std::vector<Eigen::VectorXd>& path,
                                                         double max_step) {
    if (path.size() <= 1 || !(max_step > 0.0) || !std::isfinite(max_step)) {
        return path;
    }
    std::vector<Eigen::VectorXd> out;
    out.push_back(path.front());
    for (std::size_t index = 1; index < path.size(); ++index) {
        const Eigen::VectorXd& a = path[index - 1];
        const Eigen::VectorXd& b = path[index];
        const double length = (b - a).norm();
        const int count = std::max(1, static_cast<int>(std::ceil(length / max_step)));
        for (int sample = 1; sample <= count; ++sample) {
            const double u = static_cast<double>(sample) / static_cast<double>(count);
            Eigen::VectorXd point = a + u * (b - a);
            if ((out.back() - point).norm() > 1e-12) {
                out.push_back(std::move(point));
            }
        }
    }
    return out;
}

bool csv_index_list_contains(const std::string& csv, int value) {
    std::string compact = csv;
    compact.erase(std::remove_if(compact.begin(), compact.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }), compact.end());
    std::string lowered = compact;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (compact.empty() || compact == "*" || lowered == "all") {
        return true;
    }
    std::stringstream stream(csv);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        }), item.end());
        if (item.empty()) {
            continue;
        }
        try {
            if (std::stoi(item) == value) {
                return true;
            }
        } catch (const std::exception&) {
        }
    }
    return false;
}

int derived_planner_seed(int base_seed,
                         int offset,
                         int attempt,
                         int query_index,
                         int extra) {
    constexpr long long modulus = 2147483647LL;
    long long value = static_cast<long long>(base_seed);
    value += static_cast<long long>(offset);
    value += static_cast<long long>(attempt) * kSeedAttemptStride;
    value += static_cast<long long>(query_index) * kSeedQueryStride;
    value += static_cast<long long>(extra);
    value %= modulus;
    if (value < 0) {
        value += modulus;
    }
    return static_cast<int>(value);
}

} // namespace rbf
