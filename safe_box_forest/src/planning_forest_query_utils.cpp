#include "planning_forest_query_utils.h"

#include "env_config.h"
#include "planning_forest_audit.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <sstream>
#include <string>

namespace rbf {

using detail::env_double_or_default;
using detail::env_int_or_default;

std::vector<Eigen::VectorXd> collision_shortcut_path(const std::vector<Eigen::VectorXd>& path,
                                                     const CollisionChecker& checker,
                                                     int segment_resolution) {
    if (path.size() <= 2) {
        return path;
    }
    const int safe_resolution = std::max(1, segment_resolution);
    const std::size_t n = path.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    dist[0] = 0.0;
    queue.emplace(0.0, 0);
    while (!queue.empty()) {
        const auto [current_dist, i] = queue.top();
        queue.pop();
        if (current_dist > dist[static_cast<std::size_t>(i)] + 1e-12) {
            continue;
        }
        if (static_cast<std::size_t>(i) == n - 1) {
            break;
        }
        for (std::size_t j = static_cast<std::size_t>(i) + 1; j < n; ++j) {
            if (checker.check_segment(path[static_cast<std::size_t>(i)], path[j], safe_resolution)) {
                continue;
            }
            const double edge = (path[j] - path[static_cast<std::size_t>(i)]).norm();
            const double candidate = current_dist + edge;
            if (candidate + 1e-12 < dist[j]) {
                dist[j] = candidate;
                parent[j] = i;
                queue.emplace(candidate, static_cast<int>(j));
            }
        }
    }
    if (parent[n - 1] < 0) {
        return path;
    }
    std::vector<Eigen::VectorXd> reversed;
    bool reached_start = false;
    for (int at = static_cast<int>(n - 1); at >= 0; at = parent[static_cast<std::size_t>(at)]) {
        reversed.push_back(path[static_cast<std::size_t>(at)]);
        if (at == 0) {
            reached_start = true;
            break;
        }
    }
    if (!reached_start) {
        return path;
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

std::vector<Eigen::VectorXd> hybridize_collision_free_paths(
    const std::vector<std::vector<Eigen::VectorXd>>& paths,
    const CollisionChecker& checker,
    int segment_resolution,
    int max_paths,
    int max_vertices,
    int max_cross_checks) {
    if (paths.empty() || max_paths <= 1 || max_vertices < 2 || max_cross_checks <= 0) {
        return {};
    }
    const int safe_resolution = std::max(1, segment_resolution);
    std::vector<const std::vector<Eigen::VectorXd>*> usable;
    usable.reserve(paths.size());
    for (const auto& path : paths) {
        if (path.size() >= 2U) {
            usable.push_back(&path);
        }
    }
    if (usable.size() < 2U) {
        return {};
    }
    std::sort(usable.begin(), usable.end(), [](const auto* lhs, const auto* rhs) {
        return path_length(*lhs) < path_length(*rhs);
    });
    if (usable.size() > static_cast<std::size_t>(max_paths)) {
        usable.resize(static_cast<std::size_t>(max_paths));
    }

    std::vector<Eigen::VectorXd> vertices;
    vertices.reserve(static_cast<std::size_t>(max_vertices));
    auto append_vertex = [&](const Eigen::VectorXd& point) -> int {
        for (std::size_t index = 0; index < vertices.size(); ++index) {
            if (vertices[index].size() == point.size() &&
                (vertices[index] - point).norm() <= 1e-10) {
                return static_cast<int>(index);
            }
        }
        if (vertices.size() >= static_cast<std::size_t>(max_vertices)) {
            return -1;
        }
        vertices.push_back(point);
        return static_cast<int>(vertices.size() - 1U);
    };

    struct Edge {
        int to = -1;
        double cost = 0.0;
    };
    std::vector<std::vector<Edge>> graph(static_cast<std::size_t>(max_vertices));
    auto add_edge = [&](int lhs, int rhs) {
        if (lhs < 0 || rhs < 0 || lhs == rhs) {
            return;
        }
        const double cost = (vertices[static_cast<std::size_t>(lhs)] -
                             vertices[static_cast<std::size_t>(rhs)])
                                .norm();
        auto append_one = [&](int from, int to) {
            auto& edges = graph[static_cast<std::size_t>(from)];
            auto it = std::find_if(edges.begin(), edges.end(), [&](const Edge& edge) {
                return edge.to == to;
            });
            if (it == edges.end()) {
                edges.push_back({to, cost});
            } else if (cost < it->cost) {
                it->cost = cost;
            }
        };
        append_one(lhs, rhs);
        append_one(rhs, lhs);
    };

    int start_id = -1;
    int goal_id = -1;
    for (const auto* path_ptr : usable) {
        const auto& path = *path_ptr;
        int prev = -1;
        for (std::size_t index = 0; index < path.size(); ++index) {
            const int id = append_vertex(path[index]);
            if (id < 0) {
                return {};
            }
            if (index == 0U) {
                if (start_id < 0) {
                    start_id = id;
                }
            }
            if (index + 1U == path.size()) {
                if (goal_id < 0) {
                    goal_id = id;
                }
            }
            if (prev >= 0) {
                add_edge(prev, id);
            }
            prev = id;
        }
    }
    if (start_id < 0 || goal_id < 0 || start_id == goal_id) {
        return {};
    }

    struct PairCandidate {
        double saving = 0.0;
        int lhs = -1;
        int rhs = -1;
    };
    std::vector<PairCandidate> pair_candidates;
    pair_candidates.reserve(vertices.size() * vertices.size() / 2U);
    for (std::size_t lhs = 0; lhs < vertices.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1U; rhs < vertices.size(); ++rhs) {
            const double distance = (vertices[lhs] - vertices[rhs]).norm();
            if (!(distance > 1e-9)) {
                continue;
            }
            // Prefer long-range cross-path connections; adjacent path edges
            // already exist in the graph.
            pair_candidates.push_back({-distance, static_cast<int>(lhs), static_cast<int>(rhs)});
        }
    }
    std::sort(pair_candidates.begin(), pair_candidates.end(),
              [](const PairCandidate& lhs, const PairCandidate& rhs) {
                  return lhs.saving < rhs.saving;
              });
    int checks = 0;
    for (const auto& candidate : pair_candidates) {
        if (checks >= max_cross_checks) {
            break;
        }
        const int lhs = candidate.lhs;
        const int rhs = candidate.rhs;
        bool already_connected = false;
        for (const auto& edge : graph[static_cast<std::size_t>(lhs)]) {
            if (edge.to == rhs) {
                already_connected = true;
                break;
            }
        }
        if (already_connected) {
            continue;
        }
        ++checks;
        if (checker.check_segment(vertices[static_cast<std::size_t>(lhs)],
                                  vertices[static_cast<std::size_t>(rhs)],
                                  safe_resolution)) {
            continue;
        }
        add_edge(lhs, rhs);
    }

    const std::size_t n = vertices.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    dist[static_cast<std::size_t>(start_id)] = 0.0;
    queue.emplace(0.0, start_id);
    while (!queue.empty()) {
        const auto [current_dist, current] = queue.top();
        queue.pop();
        if (current_dist > dist[static_cast<std::size_t>(current)] + 1e-12) {
            continue;
        }
        if (current == goal_id) {
            break;
        }
        for (const Edge& edge : graph[static_cast<std::size_t>(current)]) {
            const double candidate = current_dist + edge.cost;
            if (candidate + 1e-12 < dist[static_cast<std::size_t>(edge.to)]) {
                dist[static_cast<std::size_t>(edge.to)] = candidate;
                parent[static_cast<std::size_t>(edge.to)] = current;
                queue.emplace(candidate, edge.to);
            }
        }
    }
    if (parent[static_cast<std::size_t>(goal_id)] < 0) {
        return {};
    }
    std::vector<Eigen::VectorXd> reversed;
    for (int at = goal_id; at >= 0; at = parent[static_cast<std::size_t>(at)]) {
        reversed.push_back(vertices[static_cast<std::size_t>(at)]);
        if (at == start_id) {
            break;
        }
    }
    if (reversed.empty() ||
        (reversed.back() - vertices[static_cast<std::size_t>(start_id)]).norm() > 1e-10) {
        return {};
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

std::vector<Eigen::VectorXd> random_collision_shortcut_path(std::vector<Eigen::VectorXd> path,
                                                            const CollisionChecker& checker,
                                                            int segment_resolution,
                                                            int iterations,
                                                            std::uint32_t seed) {
    if (path.size() <= 2U || iterations <= 0) {
        return path;
    }
    const int safe_resolution = std::max(1, segment_resolution);
    std::mt19937 rng(seed);
    auto append_if_new = [](std::vector<Eigen::VectorXd>& out,
                            const Eigen::VectorXd& point) {
        if (out.empty() || (out.back() - point).norm() > 1e-12) {
            out.push_back(point);
        }
    };
    auto interpolate = [](const std::vector<Eigen::VectorXd>& current,
                          const std::vector<double>& cumulative,
                          double s,
                          std::size_t& segment_index) {
        const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), s);
        std::size_t index = upper == cumulative.begin()
                                ? 0U
                                : static_cast<std::size_t>(std::distance(cumulative.begin(), upper) - 1);
        index = std::min(index, current.size() - 2U);
        const double lo = cumulative[index];
        const double hi = cumulative[index + 1U];
        const double alpha = hi > lo ? std::clamp((s - lo) / (hi - lo), 0.0, 1.0) : 0.0;
        segment_index = index;
        return (1.0 - alpha) * current[index] + alpha * current[index + 1U];
    };

    for (int iter = 0; iter < iterations; ++iter) {
        if (path.size() <= 2U) {
            break;
        }
        std::vector<double> cumulative(path.size(), 0.0);
        for (std::size_t index = 1; index < path.size(); ++index) {
            cumulative[index] =
                cumulative[index - 1U] + (path[index] - path[index - 1U]).norm();
        }
        const double total = cumulative.back();
        if (!(total > 1e-9)) {
            break;
        }
        std::uniform_real_distribution<double> dist(0.0, total);
        double s0 = dist(rng);
        double s1 = dist(rng);
        if (s1 < s0) {
            std::swap(s0, s1);
        }
        if (s1 - s0 < std::max(1e-6, 0.02 * total)) {
            continue;
        }
        std::size_t i0 = 0;
        std::size_t i1 = 0;
        const Eigen::VectorXd q0 = interpolate(path, cumulative, s0, i0);
        const Eigen::VectorXd q1 = interpolate(path, cumulative, s1, i1);
        if (i1 <= i0) {
            continue;
        }
        const double replacement_length = (q1 - q0).norm();
        const double original_length = s1 - s0;
        if (!(replacement_length + 1e-9 < original_length)) {
            continue;
        }
        if (checker.check_segment(q0, q1, safe_resolution)) {
            continue;
        }
        std::vector<Eigen::VectorXd> candidate;
        candidate.reserve(path.size() - (i1 - i0) + 2U);
        for (std::size_t index = 0; index <= i0; ++index) {
            append_if_new(candidate, path[index]);
        }
        append_if_new(candidate, q0);
        append_if_new(candidate, q1);
        for (std::size_t index = i1 + 1U; index < path.size(); ++index) {
            append_if_new(candidate, path[index]);
        }
        if (candidate.size() >= 2U && path_length(candidate) + 1e-9 < path_length(path)) {
            path = std::move(candidate);
        }
    }
    return path;
}

int collision_shortcut_resolution(const QueryConfig& config) {
    int resolution = std::max(1, config.collision_shortcut_resolution);
    if (config.strict_path_audit) {
        resolution = std::max(resolution, config.audit_resolution);
    }
    return resolution;
}

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

int env_index_list_value_or_default(const char* name, std::size_t position, int fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    std::stringstream stream(raw);
    std::string item;
    std::size_t index = 0;
    while (std::getline(stream, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        }), item.end());
        if (index == position) {
            if (item.empty()) {
                return fallback;
            }
            char* end = nullptr;
            const long value = std::strtol(item.c_str(), &end, 10);
            if (end != item.c_str()) {
                return static_cast<int>(value);
            }
            return fallback;
        }
        ++index;
    }
    return fallback;
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

std::vector<Eigen::VectorXd> best_audited_rrt_bridge_path(
    const Eigen::Ref<const Eigen::VectorXd>& start,
    const Eigen::Ref<const Eigen::VectorXd>& goal,
    const CollisionChecker& checker,
    const Robot& robot,
    StageContext& context,
    const RRTConnectConfig& base_config,
    int attempts,
    double total_timeout_ms,
    int seed_base,
    int audit_resolution,
    double audit_segment_step,
    const std::vector<RRTConnectConfig>* attempt_configs,
    int seed_stride) {
    using Clock = std::chrono::steady_clock;
    std::vector<Eigen::VectorXd> best;
    double best_length = std::numeric_limits<double>::infinity();
    const int safe_attempts = std::max(1, attempts);
    const double safe_total_ms = total_timeout_ms > 0.0 ? total_timeout_ms : base_config.timeout_ms;
    const bool parallel_early_stop =
        env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP", 0) != 0;
    const int parallel_early_stop_min_successes =
        std::max(1, env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES", 1));
    const double parallel_early_stop_ratio =
        std::max(1.0, env_double_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO", 1.75));
    const double parallel_early_stop_additive =
        std::max(0.0, env_double_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE", 0.75));
    const double direct_distance = (goal - start).norm();
    auto early_stop_path_good = [&](const std::vector<Eigen::VectorXd>& path) {
        if (path.empty()) {
            return false;
        }
        if (direct_distance <= 1e-9) {
            return true;
        }
        const double length = path_length(path);
        return length <= std::max(direct_distance * parallel_early_stop_ratio,
                                  direct_distance + parallel_early_stop_additive);
    };
    context.diagnostics().set_value("query_bridge.parallel_rrt_early_stop_enabled",
                                    parallel_early_stop ? 1.0 : 0.0);

    if (context.executor().n_threads() > 1 && safe_attempts > 1) {
        const double per_attempt_ms =
            safe_total_ms > 0.0
                ? std::max(1.0, safe_total_ms / static_cast<double>(safe_attempts))
                : base_config.timeout_ms;
        std::vector<std::vector<Eigen::VectorXd>> audited_paths(static_cast<std::size_t>(safe_attempts));
        std::shared_ptr<std::atomic<bool>> local_cancel =
            parallel_early_stop ? std::make_shared<std::atomic<bool>>(false)
                                : context.native_cancel_flag();
        std::atomic<int> early_successes{0};
        context.executor().parallel_for(0, safe_attempts, [&](int attempt) {
            if (context.should_stop() ||
                (local_cancel && local_cancel->load(std::memory_order_relaxed))) {
                return;
            }
            RRTConnectConfig config =
                (attempt_configs != nullptr && !attempt_configs->empty())
                    ? (*attempt_configs)[static_cast<std::size_t>(attempt) % attempt_configs->size()]
                    : base_config;
            if (per_attempt_ms > 0.0) {
                config.timeout_ms = per_attempt_ms;
            }
            std::vector<Eigen::VectorXd> path =
                rrt_connect(start,
                            goal,
                            checker,
                            robot,
                            config,
                            seed_base + attempt * std::max(1, seed_stride),
                            local_cancel);
            if (path.empty()) {
                return;
            }
            const PathAuditCheck audit =
                audit_waypoint_path(path, checker, audit_resolution, audit_segment_step);
            if (!audit.passed) {
                return;
            }
            if (parallel_early_stop && early_stop_path_good(path)) {
                const int successes =
                    early_successes.fetch_add(1, std::memory_order_relaxed) + 1;
                if (successes >= parallel_early_stop_min_successes && local_cancel) {
                    local_cancel->store(true, std::memory_order_relaxed);
                }
            }
            audited_paths[static_cast<std::size_t>(attempt)] = std::move(path);
        });

        const int audited_successes = static_cast<int>(std::count_if(
            audited_paths.begin(),
            audited_paths.end(),
            [](const auto& path) { return !path.empty(); }));
        for (auto& path : audited_paths) {
            if (path.empty()) {
                continue;
            }
            const double length = path_length(path);
            if (length < best_length) {
                best_length = length;
                best = std::move(path);
            }
        }
        context.diagnostics().add_counter("query_bridge.parallel_rrt_attempts",
                                          static_cast<double>(safe_attempts));
        context.diagnostics().add_counter("query_bridge.parallel_rrt_successes",
                                          static_cast<double>(audited_successes));
        if (parallel_early_stop) {
            context.diagnostics().add_counter("query_bridge.parallel_rrt_early_stop_successes",
                                              static_cast<double>(early_successes.load(
                                                  std::memory_order_relaxed)));
            context.diagnostics().add_counter(
                local_cancel && local_cancel->load(std::memory_order_relaxed)
                    ? "query_bridge.parallel_rrt_early_stop_triggered"
                    : "query_bridge.parallel_rrt_early_stop_not_triggered");
        }
        return best;
    }

    const auto t0 = Clock::now();
    auto elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };
    for (int attempt = 0; attempt < safe_attempts; ++attempt) {
        if (context.should_stop()) {
            break;
        }
        RRTConnectConfig config =
            (attempt_configs != nullptr && !attempt_configs->empty())
                ? (*attempt_configs)[static_cast<std::size_t>(attempt) % attempt_configs->size()]
                : base_config;
        if (safe_total_ms > 0.0) {
            const double remaining_ms = safe_total_ms - elapsed_ms();
            if (remaining_ms <= 0.0) {
                break;
            }
            const int attempts_left = safe_attempts - attempt;
            config.timeout_ms = std::max(1.0, remaining_ms / static_cast<double>(attempts_left));
        }
        std::vector<Eigen::VectorXd> path =
            rrt_connect(start,
                        goal,
                        checker,
                        robot,
                        context,
                        config,
                        seed_base + attempt * std::max(1, seed_stride));
        if (path.empty()) {
            continue;
        }
        PathAuditCheck audit = audit_waypoint_path(path, checker, audit_resolution, audit_segment_step);
        if (!audit.passed) {
            continue;
        }
        const double length = path_length(path);
        if (length < best_length) {
            best_length = length;
            best = std::move(path);
        }
    }
    return best;
}


} // namespace rbf
