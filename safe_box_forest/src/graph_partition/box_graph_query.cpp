#include <SBF/box_graph.h>
#include <SBF/segment_edge_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

namespace rbf {
namespace {

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

}  // namespace

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
