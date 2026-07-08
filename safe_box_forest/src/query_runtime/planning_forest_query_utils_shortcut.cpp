#include "planning_forest_query_utils.h"

#include <algorithm>
#include <functional>
#include <iterator>
#include <limits>
#include <queue>
#include <random>
#include <utility>
#include <vector>

namespace rbf {

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

} // namespace rbf
