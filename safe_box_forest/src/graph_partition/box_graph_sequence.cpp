#include <SBF/box_graph.h>
#include <SBF/segment_edge_types.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rbf {

std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const AdjacencyGraph& graph) {
    static const SegmentEdgeList no_segment_edges;
    const std::vector<BoxNode> no_boxes;
    const QueryGraphCache cache = build_query_graph_cache(no_boxes, graph, no_segment_edges);
    return shortcut_box_sequence(sequence, cache);
}

std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence, const QueryGraphCache& cache) {
    return shortcut_box_sequence(sequence, cache, {});
}

std::vector<int> shortcut_box_sequence(const std::vector<int>& sequence,
                                       const QueryGraphCache& cache,
                                       const QueryShortcutCostOptions& shortcut_options) {
    if (sequence.size() <= 2) {
        return sequence;
    }

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

} // namespace rbf
