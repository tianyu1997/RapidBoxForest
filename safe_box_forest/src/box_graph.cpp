#include <SBF/box_graph.h>

#include <sbf/core/union_find.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "box_graph_options.h"
#include "query_graph_cost_options.h"

namespace rbf {
namespace {

thread_local AdjacencyBuildStats g_last_adjacency_build_stats;

std::unordered_map<int, const BoxNode*> box_map(const std::vector<BoxNode>& boxes) {
    std::unordered_map<int, const BoxNode*> map;
    map.reserve(boxes.size());
    for (const auto& box : boxes) {
        map[box.id] = &box;
    }
    return map;
}

double center_distance(const BoxNode& lhs, const BoxNode& rhs) {
    return (lhs.center() - rhs.center()).norm();
}

Eigen::VectorXd shared_face_center(const BoxNode& lhs, const BoxNode& rhs) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return (lhs.center() + rhs.center()) * 0.5;
    }
    Eigen::VectorXd center(nd);
    int face_dim = -1;
    for (int dim = 0; dim < nd; ++dim) {
        const double overlap_lo = std::max(lhs.joint_intervals[dim].lo, rhs.joint_intervals[dim].lo);
        const double overlap_hi = std::min(lhs.joint_intervals[dim].hi, rhs.joint_intervals[dim].hi);
        if (overlap_hi < overlap_lo - 1e-9) {
            return (lhs.center() + rhs.center()) * 0.5;
        }
        if (std::abs(overlap_hi - overlap_lo) <= 1e-9) {
            if (face_dim >= 0) {
                return (lhs.center() + rhs.center()) * 0.5;
            }
            face_dim = dim;
            center[dim] = 0.5 * (overlap_lo + overlap_hi);
        } else {
            center[dim] = 0.5 * (overlap_lo + overlap_hi);
        }
    }
    return face_dim >= 0 ? center : (lhs.center() + rhs.center()) * 0.5;
}

Eigen::VectorXd transition_waypoint_toward_goal(const BoxNode& lhs,
                                                const BoxNode& rhs,
                                                const Eigen::Ref<const Eigen::VectorXd>& from,
                                                const Eigen::Ref<const Eigen::VectorXd>& goal) {
    if (lhs.n_dims() != rhs.n_dims() || from.size() != lhs.n_dims()) {
        return shared_face_center(lhs, rhs);
    }
    Eigen::VectorXd target = rhs.center();
    if (goal.size() == lhs.n_dims()) {
        target = goal;
    }
    if (!boxes_connected(lhs, rhs)) {
        return shared_face_center(lhs, rhs);
    }
    Eigen::VectorXd overlap_mid(lhs.n_dims());
    Eigen::VectorXd overlap_lo(lhs.n_dims());
    Eigen::VectorXd overlap_hi(lhs.n_dims());
    for (int dim = 0; dim < lhs.n_dims(); ++dim) {
        overlap_lo[dim] = std::max(lhs.joint_intervals[dim].lo, rhs.joint_intervals[dim].lo);
        overlap_hi[dim] = std::min(lhs.joint_intervals[dim].hi, rhs.joint_intervals[dim].hi);
        overlap_mid[dim] = 0.5 * (overlap_lo[dim] + overlap_hi[dim]);
    }
    const Eigen::VectorXd local_delta = target - from;
    const double denom = local_delta.squaredNorm();
    if (denom <= 1e-18) {
        return overlap_mid;
    }
    const double t = std::min(1.0, std::max(0.0, (overlap_mid - from).dot(local_delta) / denom));
    Eigen::VectorXd waypoint = from + t * local_delta;
    for (int dim = 0; dim < lhs.n_dims(); ++dim) {
        waypoint[dim] = std::min(overlap_hi[dim], std::max(overlap_lo[dim], waypoint[dim]));
    }
    return waypoint;
}

std::uint64_t edge_pair_key(int lhs, int rhs) {
    const std::uint32_t lo = static_cast<std::uint32_t>(std::min(lhs, rhs));
    const std::uint32_t hi = static_cast<std::uint32_t>(std::max(lhs, rhs));
    return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
}

long long interval_bin(double value, double origin, double width) {
    const double safe_width = std::max(width, 1e-12);
    return static_cast<long long>(std::floor((value - origin) / safe_width));
}

int choose_index_dimension(const std::vector<BoxNode>& boxes) {
    if (boxes.empty() || boxes.front().n_dims() <= 0) {
        return -1;
    }
    const int nd = boxes.front().n_dims();
    int best_dim = 0;
    double best_span = -1.0;
    for (int dim = 0; dim < nd; ++dim) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        for (const auto& box : boxes) {
            if (box.n_dims() != nd) {
                return 0;
            }
            const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
            lo = std::min(lo, interval.lo);
            hi = std::max(hi, interval.hi);
        }
        const double span = hi - lo;
        if (span > best_span) {
            best_span = span;
            best_dim = dim;
        }
    }
    return best_dim;
}

double choose_bin_width(const std::vector<BoxNode>& boxes, int dim, double tolerance) {
    if (boxes.empty() || dim < 0) {
        return 1.0;
    }
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    std::vector<double> widths;
    widths.reserve(boxes.size());
    for (const auto& box : boxes) {
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        lo = std::min(lo, interval.lo);
        hi = std::max(hi, interval.hi);
        widths.push_back(std::max(0.0, interval.hi - interval.lo));
    }
    std::sort(widths.begin(), widths.end());
    const double median_width = widths.empty() ? 0.0 : widths[widths.size() / 2];
    const double span_width = (hi - lo) / std::max(1.0, std::sqrt(static_cast<double>(boxes.size())));
    const double tol_width = std::max(1e-9, 8.0 * std::max(tolerance, 1e-9));
    return std::max({median_width, span_width, tol_width, 1e-9});
}

double index_origin(const std::vector<BoxNode>& boxes, int dim, double tolerance) {
    double origin = 0.0;
    if (!boxes.empty() && dim >= 0) {
        origin = std::numeric_limits<double>::infinity();
        for (const auto& box : boxes) {
            origin = std::min(origin, box.joint_intervals[static_cast<std::size_t>(dim)].lo);
        }
        if (!std::isfinite(origin)) {
            origin = 0.0;
        }
    }
    return origin - std::max(tolerance, 0.0) - 1e-12;
}

struct IntervalBinIndex {
    int dim = -1;
    double origin = 0.0;
    double bin_width = 1.0;
    std::unordered_map<long long, std::vector<int>> bins;
    std::uint64_t estimated_pairs = 0;
};

IntervalBinIndex build_interval_bin_index(const std::vector<BoxNode>& boxes,
                                          int dim,
                                          double tolerance) {
    IntervalBinIndex index;
    index.dim = dim;
    if (boxes.empty() || dim < 0) {
        return index;
    }
    index.bin_width = choose_bin_width(boxes, dim, tolerance);
    index.origin = index_origin(boxes, dim, tolerance);
    index.bins.reserve(static_cast<std::size_t>(std::max(1, static_cast<int>(boxes.size()) * 2)));
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        const auto& interval = boxes[static_cast<std::size_t>(i)].joint_intervals[static_cast<std::size_t>(dim)];
        const long long lo_bin = interval_bin(interval.lo - tolerance, index.origin, index.bin_width);
        const long long hi_bin = interval_bin(interval.hi + tolerance, index.origin, index.bin_width);
        for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
            index.bins[bin].push_back(i);
        }
    }
    for (const auto& [_, members] : index.bins) {
        const std::uint64_t m = static_cast<std::uint64_t>(members.size());
        if (m > 1) {
            index.estimated_pairs += (m * (m - 1)) / 2;
        }
    }
    return index;
}

std::vector<IntervalBinIndex> select_adjacency_indices(
    const std::vector<BoxNode>& boxes,
    double tolerance,
    const detail::AdjacencyIndexOptions& options) {
    std::vector<IntervalBinIndex> indices;
    if (boxes.empty() || boxes.front().n_dims() <= 0) {
        return indices;
    }
    const int nd = boxes.front().n_dims();
    indices.reserve(static_cast<std::size_t>(nd));
    for (int dim = 0; dim < nd; ++dim) {
        indices.push_back(build_interval_bin_index(boxes, dim, tolerance));
    }
    std::sort(indices.begin(), indices.end(), [](const IntervalBinIndex& lhs, const IntervalBinIndex& rhs) {
        if (lhs.estimated_pairs != rhs.estimated_pairs) {
            return lhs.estimated_pairs < rhs.estimated_pairs;
        }
        return lhs.dim < rhs.dim;
    });
    if (static_cast<int>(indices.size()) > options.selected_dim_count) {
        indices.resize(static_cast<std::size_t>(options.selected_dim_count));
    }
    return indices;
}

bool intervals_may_connect_on_dim(const BoxNode& lhs,
                                  const BoxNode& rhs,
                                  int dim,
                                  double tolerance) {
    const auto& li = lhs.joint_intervals[static_cast<std::size_t>(dim)];
    const auto& ri = rhs.joint_intervals[static_cast<std::size_t>(dim)];
    return std::min(li.hi, ri.hi) >= std::max(li.lo, ri.lo) - tolerance;
}

}  // namespace

bool boxes_connected(const BoxNode& lhs, const BoxNode& rhs, double tolerance) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return false;
    }
    int shared_dims = 0;
    int overlap_dims = 0;
    for (int dim = 0; dim < nd; ++dim) {
        const double overlap_lo = std::max(lhs.joint_intervals[dim].lo, rhs.joint_intervals[dim].lo);
        const double overlap_hi = std::min(lhs.joint_intervals[dim].hi, rhs.joint_intervals[dim].hi);
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

AdjacencyGraph compute_adjacency_reference(const std::vector<BoxNode>& boxes,
                                           double tolerance,
                                           int max_degree,
                                           double gap_tolerance) {
    AdjacencyGraph graph;
    for (const auto& box : boxes) {
        graph[box.id] = {};
    }
    const int n = static_cast<int>(boxes.size());
    const double effective_tol = std::max(tolerance, gap_tolerance);
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (boxes_connected(boxes[i], boxes[j], effective_tol)) {
                graph[boxes[i].id].push_back(boxes[j].id);
                graph[boxes[j].id].push_back(boxes[i].id);
            }
        }
    }
    if (max_degree > 0) {
        const auto map = box_map(boxes);
        for (auto& [id, neighbors] : graph) {
            auto center_it = map.find(id);
            if (center_it == map.end()) {
                continue;
            }
            std::sort(neighbors.begin(), neighbors.end(), [&](int lhs, int rhs) {
                return center_distance(*center_it->second, *map.at(lhs)) <
                       center_distance(*center_it->second, *map.at(rhs));
            });
            if (static_cast<int>(neighbors.size()) > max_degree) {
                neighbors.resize(static_cast<std::size_t>(max_degree));
            }
        }
    }
    return graph;
}

AdjacencyGraph compute_adjacency(const std::vector<BoxNode>& boxes,
                                 double tolerance,
                                 int max_degree,
                                 double gap_tolerance) {
    const auto start_time = std::chrono::steady_clock::now();
    AdjacencyGraph graph;
    for (const auto& box : boxes) {
        graph[box.id] = {};
    }
    const int n = static_cast<int>(boxes.size());
    const double effective_tol = std::max(tolerance, gap_tolerance);
    g_last_adjacency_build_stats = {};
    g_last_adjacency_build_stats.boxes = n;
    if (n <= 1) {
        g_last_adjacency_build_stats.build_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
        return graph;
    }
    const detail::AdjacencyIndexOptions adjacency_options =
        detail::adjacency_index_options_from_env(n);
    const std::vector<IntervalBinIndex> indices =
        select_adjacency_indices(boxes, effective_tol, adjacency_options);
    if (indices.empty() || indices.front().dim < 0) {
        g_last_adjacency_build_stats.build_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
        return graph;
    }
    const IntervalBinIndex& primary = indices.front();
    g_last_adjacency_build_stats.selected_dims = static_cast<int>(indices.size());
    g_last_adjacency_build_stats.primary_dim = primary.dim;
    std::unordered_set<std::uint64_t> tested;
    tested.reserve(static_cast<std::size_t>(std::max(1, n * 8)));
    for (const auto& [_, members] : primary.bins) {
        for (std::size_t outer = 0; outer < members.size(); ++outer) {
            const int i = members[outer];
            for (std::size_t inner = outer + 1; inner < members.size(); ++inner) {
                const int j = members[inner];
                const auto key = edge_pair_key(i, j);
                if (!tested.insert(key).second) {
                    continue;
                }
                g_last_adjacency_build_stats.candidate_pairs += 1;
                bool selected_dims_overlap = true;
                for (std::size_t dim_index = 1; dim_index < indices.size(); ++dim_index) {
                    const int dim = indices[dim_index].dim;
                    if (!intervals_may_connect_on_dim(boxes[static_cast<std::size_t>(i)],
                                                      boxes[static_cast<std::size_t>(j)],
                                                      dim,
                                                      effective_tol)) {
                        selected_dims_overlap = false;
                        break;
                    }
                }
                if (!selected_dims_overlap) {
                    continue;
                }
                g_last_adjacency_build_stats.exact_tests += 1;
                if (boxes_connected(boxes[static_cast<std::size_t>(i)], boxes[static_cast<std::size_t>(j)], effective_tol)) {
                    graph[boxes[static_cast<std::size_t>(i)].id].push_back(boxes[static_cast<std::size_t>(j)].id);
                    graph[boxes[static_cast<std::size_t>(j)].id].push_back(boxes[static_cast<std::size_t>(i)].id);
                    g_last_adjacency_build_stats.edges += 1;
                }
            }
        }
    }
    if (max_degree > 0) {
        const auto map = box_map(boxes);
        for (auto& [id, neighbors] : graph) {
            auto center_it = map.find(id);
            if (center_it == map.end()) {
                continue;
            }
            std::sort(neighbors.begin(), neighbors.end(), [&](int lhs, int rhs) {
                return center_distance(*center_it->second, *map.at(lhs)) <
                       center_distance(*center_it->second, *map.at(rhs));
            });
            if (static_cast<int>(neighbors.size()) > max_degree) {
                neighbors.resize(static_cast<std::size_t>(max_degree));
            }
        }
    }
    g_last_adjacency_build_stats.build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
    return graph;
}

AdjacencyBuildStats last_adjacency_build_stats() {
    return g_last_adjacency_build_stats;
}

QueryGraphCache build_query_graph_cache(const std::vector<BoxNode>& boxes,
                                        const AdjacencyGraph& graph,
                                        const SegmentEdgeList& segment_edges) {
    QueryGraphCache cache;
    cache.boxes = &boxes;
    cache.graph = &graph;
    cache.segment_edges = &segment_edges;
    cache.box_index_by_id.reserve(boxes.size());
    for (std::size_t index = 0; index < boxes.size(); ++index) {
        cache.box_index_by_id[boxes[index].id] = index;
    }
    cache.segment_edge_index_by_pair.reserve(segment_edges.size());
    for (std::size_t index = 0; index < segment_edges.size(); ++index) {
        const auto& edge = segment_edges[index];
        if (edge.source_box_id < 0 || edge.target_box_id < 0) {
            continue;
        }
        const auto key = edge_pair_key(edge.source_box_id, edge.target_box_id);
        auto it = cache.segment_edge_index_by_pair.find(key);
        if (it == cache.segment_edge_index_by_pair.end() || edge.length < segment_edges[it->second].length) {
            cache.segment_edge_index_by_pair[key] = index;
        }
    }
    cache.adjacency_sets.reserve(graph.size());
    for (const auto& [id, neighbors] : graph) {
        cache.adjacency_sets.emplace(id, std::unordered_set<int>(neighbors.begin(), neighbors.end()));
    }

    cache.point_index_dim = choose_index_dimension(boxes);
    if (!boxes.empty() && cache.point_index_dim >= 0) {
        cache.point_bin_width = choose_bin_width(boxes, cache.point_index_dim, 0.0);
        cache.point_bin_origin = index_origin(boxes, cache.point_index_dim, 0.0);
        cache.point_bins.reserve(boxes.size() * 2);
        for (std::size_t index = 0; index < boxes.size(); ++index) {
            const auto& interval = boxes[index].joint_intervals[static_cast<std::size_t>(cache.point_index_dim)];
            const long long lo_bin = interval_bin(interval.lo, cache.point_bin_origin, cache.point_bin_width);
            const long long hi_bin = interval_bin(interval.hi, cache.point_bin_origin, cache.point_bin_width);
            for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
                cache.point_bins[bin].push_back(boxes[index].id);
            }
        }
    }
    return cache;
}

DijkstraResult dijkstra_search(const AdjacencyGraph& graph,
                               const std::vector<BoxNode>& boxes,
                               int start_box_id,
                               int goal_box_id,
                               const Eigen::VectorXd& goal_point) {
    static const SegmentEdgeList no_segment_edges;
    return dijkstra_search(graph, boxes, no_segment_edges, start_box_id, goal_box_id, goal_point);
}

DijkstraResult dijkstra_search(const AdjacencyGraph& graph,
                               const std::vector<BoxNode>& boxes,
                               const SegmentEdgeList& segment_edges,
                               int start_box_id,
                               int goal_box_id,
                               const Eigen::VectorXd& goal_point) {
    const QueryGraphCache cache = build_query_graph_cache(boxes, graph, segment_edges);
    return dijkstra_search(cache, start_box_id, goal_box_id, goal_point);
}

DijkstraResult dijkstra_search(const QueryGraphCache& cache,
                               int start_box_id,
                               int goal_box_id,
                               const Eigen::VectorXd& goal_point) {
    static const Eigen::VectorXd no_start_point;
    return dijkstra_search(cache, start_box_id, goal_box_id, no_start_point, goal_point);
}

DijkstraResult dijkstra_search(const QueryGraphCache& cache,
                               int start_box_id,
                               int goal_box_id,
                               const Eigen::VectorXd& start_point,
                               const Eigen::VectorXd& goal_point) {
    DijkstraResult result;
    if (cache.boxes == nullptr || cache.graph == nullptr) {
        return result;
    }
    const auto start_it = cache.box_index_by_id.find(start_box_id);
    const auto goal_it = cache.box_index_by_id.find(goal_box_id);
    if (start_it == cache.box_index_by_id.end() || goal_it == cache.box_index_by_id.end()) {
        return result;
    }
    const auto& boxes = *cache.boxes;
    auto box_ptr = [&](int id) -> const BoxNode* {
        const auto it = cache.box_index_by_id.find(id);
        return it == cache.box_index_by_id.end() ? nullptr : &boxes[it->second];
    };
    if (start_box_id == goal_box_id) {
        result.found = true;
        const SegmentEdge* self_edge = find_segment_edge(cache, start_box_id, goal_box_id);
        if (self_edge != nullptr) {
            result.box_sequence = {start_box_id, goal_box_id};
            result.segment_edge_sequence.push_back(self_edge->id);
            result.total_cost = self_edge->length;
        } else {
            result.box_sequence = {start_box_id};
        }
        return result;
    }

    struct Item {
        int id;
        double f;
        bool operator>(const Item& other) const { return f > other.f; }
    };

    auto heuristic = [&](int id) {
        const BoxNode* box = box_ptr(id);
        const BoxNode* goal_box = box_ptr(goal_box_id);
        if (box == nullptr || goal_box == nullptr) {
            return 0.0;
        }
        if (goal_point.size() == box->n_dims()) {
            return (box->center() - goal_point).norm();
        }
        return center_distance(*box, *goal_box);
    };

    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;
    std::unordered_map<int, Eigen::VectorXd> representative;
    dist[start_box_id] = 0.0;
    representative[start_box_id] = box_ptr(start_box_id)->center();
    open.push({start_box_id, heuristic(start_box_id)});
    const QueryGraphCostOptions cost_options = query_graph_cost_options_from_env();
    const bool line_deviation_enabled =
        cost_options.box_line_deviation_penalty > 0.0 &&
        start_point.size() == goal_point.size() &&
        start_point.size() > 0;
    const Eigen::VectorXd query_delta =
        line_deviation_enabled ? (goal_point - start_point) : Eigen::VectorXd{};
    const double query_delta_norm_sq =
        line_deviation_enabled ? query_delta.squaredNorm() : 0.0;
    auto distance_to_query_line = [&](const Eigen::VectorXd& point) {
        if (!line_deviation_enabled ||
            point.size() != start_point.size() ||
            query_delta_norm_sq <= 1e-18) {
            return 0.0;
        }
        const double u = std::clamp((point - start_point).dot(query_delta) /
                                        query_delta_norm_sq,
                                    0.0,
                                    1.0);
        return (point - (start_point + u * query_delta)).norm();
    };
    auto edge_line_deviation = [&](const SegmentEdge& edge,
                                   const Eigen::VectorXd& fallback_point) {
        if (!line_deviation_enabled) {
            return 0.0;
        }
        double max_deviation = distance_to_query_line(fallback_point);
        for (const auto& waypoint : edge.waypoints) {
            max_deviation = std::max(max_deviation, distance_to_query_line(waypoint));
        }
        return max_deviation;
    };

    while (!open.empty()) {
        const int current = open.top().id;
        const double priority = open.top().f;
        open.pop();
        const double current_dist = dist.find(current) == dist.end() ? std::numeric_limits<double>::infinity() : dist.at(current);
        if (priority > current_dist + heuristic(current) + 1e-12) {
            continue;
        }
        if (current == goal_box_id) {
            result.found = true;
            break;
        }
        auto it = cache.graph->find(current);
        if (it == cache.graph->end()) {
            continue;
        }
        for (int next : it->second) {
            const BoxNode* current_box = box_ptr(current);
            const BoxNode* next_box = box_ptr(next);
            if (current_box == nullptr || next_box == nullptr) {
                continue;
            }
            const Eigen::VectorXd current_rep = representative.at(current);
            Eigen::VectorXd next_rep = transition_waypoint_toward_goal(*current_box, *next_box, current_rep, goal_point);
            const double transition_length = (current_rep - next_rep).norm();
            double edge_cost = transition_length + 1e-6 +
                               cost_options.box_transition_penalty;
            if (cost_options.box_nonprogress_penalty > 0.0 &&
                goal_point.size() == current_rep.size() &&
                goal_point.size() == next_rep.size()) {
                const double current_goal_distance = (current_rep - goal_point).norm();
                const double next_goal_distance = (next_rep - goal_point).norm();
                edge_cost += cost_options.box_nonprogress_penalty *
                             std::max(0.0, next_goal_distance - current_goal_distance);
            }
            if (line_deviation_enabled) {
                edge_cost += cost_options.box_line_deviation_penalty *
                             distance_to_query_line(next_rep) *
                             std::max(transition_length, 1e-6);
            }
            if (next == goal_box_id && goal_point.size() == next_box->n_dims()) {
                edge_cost += (next_rep - goal_point).norm();
                next_rep = goal_point;
            }
            const SegmentEdge* edge = find_segment_edge(cache, current, next);
            if (edge != nullptr) {
                edge_cost = edge->length > 0.0 ? edge->length : edge_cost;
                if (line_deviation_enabled) {
                    edge_cost += cost_options.box_line_deviation_penalty *
                                 edge_line_deviation(*edge, next_box->center()) *
                                 std::max(edge->length, 1e-6);
                }
                if (counts_as_segment_edge(edge->type) &&
                    edge->type != SegmentEdgeType::QueryBridge &&
                    edge->validation != SegmentEdgeValidation::ConservativeObbZonotope) {
                    edge_cost += 100.0;
                }
                if (counts_as_query_repair_edge(edge->type)) {
                    edge_cost += cost_options.query_bridge_penalty;
                }
                if (cost_options.foreign_query_edge_penalty > 0.0 &&
                    edge->query_index >= 0 &&
                    edge->query_index != cost_options.active_query_index) {
                    edge_cost += cost_options.foreign_query_edge_penalty;
                }
                if (edge->strict_audit_required &&
                    edge->type == SegmentEdgeType::QueryBridge &&
                    edge->validation != SegmentEdgeValidation::CollisionChecked) {
                    edge_cost += 1.0e6;
                }
                next_rep = next_box->center();
            }
            const double alt = current_dist + edge_cost;
            auto dit = dist.find(next);
            if (dit == dist.end() || alt < dit->second) {
                dist[next] = alt;
                prev[next] = current;
                representative[next] = std::move(next_rep);
                open.push({next, alt + heuristic(next)});
            }
        }
    }

    if (!result.found) {
        return result;
    }
    int cur = goal_box_id;
    while (true) {
        result.box_sequence.push_back(cur);
        if (cur == start_box_id) {
            break;
        }
        cur = prev.at(cur);
    }
    std::reverse(result.box_sequence.begin(), result.box_sequence.end());
    for (std::size_t index = 1; index < result.box_sequence.size(); ++index) {
        const SegmentEdge* edge = find_segment_edge(cache, result.box_sequence[index - 1], result.box_sequence[index]);
        result.segment_edge_sequence.push_back(edge == nullptr ? -1 : edge->id);
    }
    result.total_cost = dist[goal_box_id];
    return result;
}

std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const AdjacencyGraph& graph) {
    static const SegmentEdgeList no_segment_edges;
    const std::vector<BoxNode> no_boxes;
    const QueryGraphCache cache = build_query_graph_cache(no_boxes, graph, no_segment_edges);
    return shortcut_box_sequence(sequence, cache);
}

std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const QueryGraphCache& cache) {
    if (sequence.size() <= 2) {
        return sequence;
    }

    const QueryShortcutCostOptions shortcut_options =
        query_shortcut_cost_options_from_env(cache.boxes != nullptr);
    auto transition_cost = [&](int lhs, int rhs) {
        if (const SegmentEdge* edge = find_segment_edge(cache, lhs, rhs)) {
            return edge->length > 0.0 ? edge->length : 0.0;
        }
        if (cache.boxes == nullptr) {
            return 0.0;
        }
        const auto lhs_it = cache.box_index_by_id.find(lhs);
        const auto rhs_it = cache.box_index_by_id.find(rhs);
        if (lhs_it == cache.box_index_by_id.end() ||
            rhs_it == cache.box_index_by_id.end()) {
            return std::numeric_limits<double>::infinity();
        }
        return ((*cache.boxes)[lhs_it->second].center() -
                (*cache.boxes)[rhs_it->second].center()).norm();
    };
    auto sequence_cost = [&](std::size_t begin, std::size_t end) {
        double total = 0.0;
        for (std::size_t k = begin + 1; k <= end; ++k) {
            total += transition_cost(sequence[k - 1], sequence[k]);
        }
        return total;
    };
    auto shortcut_acceptable = [&](std::size_t begin,
                                   std::size_t end,
                                   int bridge) {
        if (!shortcut_options.cost_aware) {
            return true;
        }
        const double original_cost = sequence_cost(begin, end);
        const double shortcut_cost =
            bridge >= 0
                ? transition_cost(sequence[begin], bridge) +
                      transition_cost(bridge, sequence[end])
                : transition_cost(sequence[begin], sequence[end]);
        return std::isfinite(original_cost) &&
               std::isfinite(shortcut_cost) &&
               shortcut_cost <= original_cost * shortcut_options.cost_factor + 1e-9;
    };

    std::vector<int> shortened;
    shortened.reserve(sequence.size());
    std::size_t i = 0;
    while (i < sequence.size()) {
        if (shortened.empty() || shortened.back() != sequence[i]) {
            shortened.push_back(sequence[i]);
        }
        if (i + 1 >= sequence.size()) {
            break;
        }
        std::size_t best = i + 1;
        int bridge = -1;
        const auto it_i = cache.adjacency_sets.find(sequence[i]);
        for (std::size_t j = sequence.size() - 1; j > i + 1; --j) {
            if (it_i != cache.adjacency_sets.end() &&
                it_i->second.count(sequence[j]) > 0 &&
                shortcut_acceptable(i, j, -1)) {
                best = j;
                bridge = -1;
                break;
            }
            if (it_i == cache.adjacency_sets.end()) {
                continue;
            }
            const auto it_j = cache.adjacency_sets.find(sequence[j]);
            if (it_j == cache.adjacency_sets.end()) {
                continue;
            }
            for (int candidate : it_i->second) {
                if (candidate != sequence[i] &&
                    candidate != sequence[j] &&
                    it_j->second.count(candidate) > 0 &&
                    shortcut_acceptable(i, j, candidate)) {
                    best = j;
                    bridge = candidate;
                    break;
                }
            }
            if (bridge >= 0) {
                break;
            }
        }
        if (bridge >= 0 && shortened.back() != bridge) {
            shortened.push_back(bridge);
        }
        i = best;
    }
    return shortened;
}

std::vector<Eigen::VectorXd> extract_waypoints(const std::vector<int>& box_sequence,
                                               const std::vector<BoxNode>& boxes,
                                               const Eigen::Ref<const Eigen::VectorXd>& start,
                                               const Eigen::Ref<const Eigen::VectorXd>& goal) {
    static const SegmentEdgeList no_segment_edges;
    return extract_waypoints(box_sequence, boxes, no_segment_edges, start, goal);
}

std::vector<Eigen::VectorXd> extract_waypoints(const std::vector<int>& box_sequence,
                                               const std::vector<BoxNode>& boxes,
                                               const SegmentEdgeList& segment_edges,
                                               const Eigen::Ref<const Eigen::VectorXd>& start,
                                               const Eigen::Ref<const Eigen::VectorXd>& goal) {
    AdjacencyGraph empty_graph;
    const QueryGraphCache cache = build_query_graph_cache(boxes, empty_graph, segment_edges);
    return extract_waypoints(box_sequence, cache, start, goal);
}

std::vector<Eigen::VectorXd> extract_waypoints(const std::vector<int>& box_sequence,
                                               const QueryGraphCache& cache,
                                               const Eigen::Ref<const Eigen::VectorXd>& start,
                                               const Eigen::Ref<const Eigen::VectorXd>& goal) {
    std::vector<Eigen::VectorXd> path;
    if (box_sequence.empty() || cache.boxes == nullptr) {
        return path;
    }
    const auto& boxes = *cache.boxes;
    auto box_ptr = [&](int id) -> const BoxNode* {
        const auto it = cache.box_index_by_id.find(id);
        return it == cache.box_index_by_id.end() ? nullptr : &boxes[it->second];
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
        if (const SegmentEdge* edge = find_segment_edge(cache, lhs.id, rhs.id)) {
            if (edge->type == SegmentEdgeType::PortalCorridor &&
                edge->conservative_certificate &&
                !edge->internal_boxes.empty()) {
                const bool reverse_edge =
                    edge->source_box_id == rhs.id && edge->target_box_id == lhs.id;
                std::vector<const BoxNode*> chain;
                chain.reserve(edge->internal_boxes.size() + 2U);
                chain.push_back(&lhs);
                if (reverse_edge) {
                    for (auto it = edge->internal_boxes.rbegin();
                         it != edge->internal_boxes.rend();
                         ++it) {
                        chain.push_back(&*it);
                    }
                } else {
                    for (const auto& internal : edge->internal_boxes) {
                        chain.push_back(&internal);
                    }
                }
                chain.push_back(&rhs);
                for (std::size_t k = 1; k < chain.size(); ++k) {
                    const BoxNode& prev_box = *chain[k - 1];
                    const BoxNode& next_box = *chain[k];
                    if (boxes_connected(prev_box, next_box)) {
                        append_if_new(transition_waypoint_toward_goal(prev_box,
                                                                       next_box,
                                                                       path.back(),
                                                                       goal));
                    } else {
                        append_if_new(prev_box.center());
                        append_if_new(next_box.center());
                    }
                }
                continue;
            }
            std::vector<Eigen::VectorXd> edge_path = edge->waypoints;
            if (edge->source_box_id == rhs.id && edge->target_box_id == lhs.id) {
                std::reverse(edge_path.begin(), edge_path.end());
            }
            if (edge_path.empty()) {
                edge_path.push_back(lhs.center());
                edge_path.push_back(rhs.center());
            }
            for (const auto& waypoint : edge_path) {
                if (path.empty() || (path.back() - waypoint).norm() > 1e-12) {
                    path.push_back(waypoint);
                }
            }
            continue;
        }
        if (boxes_connected(lhs, rhs)) {
            append_if_new(transition_waypoint_toward_goal(lhs, rhs, path.back(), goal));
        } else {
            const Eigen::VectorXd lhs_center = lhs.center();
            const Eigen::VectorXd rhs_center = rhs.center();
            if ((path.back() - lhs_center).norm() > 1e-12) {
                path.push_back(lhs_center);
            }
            path.push_back(rhs_center);
            continue;
        }
    }
    if (path.empty() || (path.back() - goal).norm() > 1e-12) {
        path.push_back(goal);
    }
    return path;
}

double path_length(const std::vector<Eigen::VectorXd>& path) {
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        total += (path[i] - path[i - 1]).norm();
    }
    return total;
}

int locate_containing_box(const std::vector<BoxNode>& boxes,
                          const Eigen::Ref<const Eigen::VectorXd>& q,
                          bool nearest_if_outside) {
    AdjacencyGraph empty_graph;
    static const SegmentEdgeList no_segment_edges;
    const QueryGraphCache cache = build_query_graph_cache(boxes, empty_graph, no_segment_edges);
    return locate_containing_box(cache, q, nearest_if_outside);
}

int locate_containing_box(const QueryGraphCache& cache,
                          const Eigen::Ref<const Eigen::VectorXd>& q,
                          bool nearest_if_outside) {
    if (cache.boxes == nullptr) {
        return -1;
    }
    const auto& boxes = *cache.boxes;
    int best = -1;
    double best_dist = std::numeric_limits<double>::max();
    std::vector<int> candidate_ids;
    if (cache.point_index_dim >= 0 && q.size() > cache.point_index_dim && !cache.point_bins.empty()) {
        const long long bin = interval_bin(q[cache.point_index_dim], cache.point_bin_origin, cache.point_bin_width);
        const auto it = cache.point_bins.find(bin);
        if (it != cache.point_bins.end()) {
            candidate_ids = it->second;
        }
    }
    auto visit_box = [&](const BoxNode& box) {
        if (box.contains(q)) {
            const int safety_rank = box.safety_status == BoxSafetyStatus::CertifiedFree && !box.strict_audit_required ? 0
                : box.safety_status == BoxSafetyStatus::CertifiedFree ? 1
                : box.safety_status == BoxSafetyStatus::ProvisionalFree && !box.strict_audit_required ? 2
                : box.safety_status == BoxSafetyStatus::ProvisionalFree ? 3
                : 4;
            const double center_dist = (box.center() - q).squaredNorm();
            const double volume = std::max(0.0, box.volume);
            const double score = static_cast<double>(safety_rank) * 1.0e24 + volume + 1.0e-9 * center_dist;
            if (best < 0 || score < best_dist) {
                best = box.id;
                best_dist = score;
            }
            return;
        }
        if (nearest_if_outside) {
            const double d = (box.center() - q).squaredNorm();
            if (d < best_dist) {
                best_dist = d;
                best = box.id;
            }
        }
    };
    if (!candidate_ids.empty()) {
        for (int id : candidate_ids) {
            const auto it = cache.box_index_by_id.find(id);
            if (it != cache.box_index_by_id.end()) {
                visit_box(boxes[it->second]);
            }
        }
        if (best >= 0 || !nearest_if_outside) {
            return best;
        }
    }
    for (const auto& box : boxes) {
        visit_box(box);
    }
    return best;
}

}  // namespace rbf
