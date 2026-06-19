#include <SBF/query.h>

#include <chrono>

namespace rbf {

QueryResult CorridorQuery::run(const std::vector<BoxNode>& boxes,
                               const AdjacencyGraph& graph,
                               const Eigen::Ref<const Eigen::VectorXd>& start,
                               const Eigen::Ref<const Eigen::VectorXd>& goal,
                               const QueryGraphCostOptions& graph_cost) const {
    static const SegmentEdgeList no_segment_edges;
    return run(boxes, graph, no_segment_edges, start, goal, graph_cost);
}

QueryResult CorridorQuery::run(const std::vector<BoxNode>& boxes,
                               const AdjacencyGraph& graph,
                               const SegmentEdgeList& segment_edges,
                               const Eigen::Ref<const Eigen::VectorXd>& start,
                               const Eigen::Ref<const Eigen::VectorXd>& goal,
                               const QueryGraphCostOptions& graph_cost) const {
    const QueryGraphCache cache = build_query_graph_cache(boxes, graph, segment_edges);
    return run(cache, start, goal, graph_cost);
}

QueryResult CorridorQuery::run(const QueryGraphCache& cache,
                               const Eigen::Ref<const Eigen::VectorXd>& start,
                               const Eigen::Ref<const Eigen::VectorXd>& goal,
                               const QueryGraphCostOptions& graph_cost) const {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    QueryResult result;
    result.start_box_id = locate_containing_box(cache, start, config_.nearest_if_outside);
    result.goal_box_id = locate_containing_box(cache, goal, config_.nearest_if_outside);
    if (result.start_box_id < 0 || result.goal_box_id < 0) {
        result.query_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return result;
    }
    auto dijkstra = dijkstra_search(cache,
                                    result.start_box_id,
                                    result.goal_box_id,
                                    start,
                                    goal,
                                    graph_cost);
    if (!dijkstra.found) {
        result.query_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        return result;
    }
    result.box_sequence = config_.shortcut_boxes ? shortcut_box_sequence(dijkstra.box_sequence, cache) : dijkstra.box_sequence;
    result.path = extract_waypoints(result.box_sequence, cache, start, goal);
    result.segment_edge_sequence.clear();
    for (std::size_t index = 1; index < result.box_sequence.size(); ++index) {
        const SegmentEdge* edge = find_segment_edge(cache, result.box_sequence[index - 1], result.box_sequence[index]);
        result.segment_edge_sequence.push_back(edge == nullptr ? -1 : edge->id);
        if (edge != nullptr && counts_as_segment_edge(edge->type)) {
            result.segment_edges_used += 1;
        }
        if (edge != nullptr && !edge->obb_centers.empty()) {
            result.obb_edges_used += 1;
            result.obb_regions_used += static_cast<int>(edge->obb_centers.size());
            result.obb_edge_length += edge->obb_covered_length > 0.0
                ? edge->obb_covered_length
                : edge->length;
        }
    }
    result.path_length = path_length(result.path);
    result.success = !result.path.empty();
    result.query_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return result;
}

}  // namespace rbf
