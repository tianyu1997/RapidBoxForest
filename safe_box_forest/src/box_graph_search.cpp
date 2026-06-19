#include <SBF/box_graph.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace rbf {
namespace {

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

} // namespace

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
    return dijkstra_search(cache, start_box_id, goal_box_id, start_point, goal_point, {});
}

DijkstraResult dijkstra_search(const QueryGraphCache& cache,
                               int start_box_id,
                               int goal_box_id,
                               const Eigen::VectorXd& start_point,
                               const Eigen::VectorXd& goal_point,
                               const QueryGraphCostOptions& cost_options) {
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
        const double current_dist = dist.find(current) == dist.end()
            ? std::numeric_limits<double>::infinity()
            : dist.at(current);
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
        const SegmentEdge* edge =
            find_segment_edge(cache, result.box_sequence[index - 1], result.box_sequence[index]);
        result.segment_edge_sequence.push_back(edge == nullptr ? -1 : edge->id);
    }
    result.total_cost = dist[goal_box_id];
    return result;
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

} // namespace rbf
