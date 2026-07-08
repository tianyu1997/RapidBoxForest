#include "planning_forest_query_utils.h"

#include <SBF/oracle.h>
#include <SBF/segment_edge_types.h>

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
