#include <SBF/box_graph.h>

#include <sbf/core/union_find.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "box_graph_options.h"

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
