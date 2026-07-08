#include <SBF/connector.h>
#include <SBF/box_graph.h>
#include <SBF/runtime.h>
#include <SBF/segment_edge_types.h>

#include "connector_frontier_bridge.h"
#include "connector_internal.h"
#include "connector_pair_candidates.h"
#include "connector_pair_commit.h"
#include "connector_pair_tasks.h"

#include <unordered_map>
#include <utility>

namespace rbf {

IslandConnectorResult IslandConnector::connect_all(std::vector<BoxNode>& boxes,
                                                   AdjacencyGraph& graph,
                                                   SegmentEdgeList& segment_edges,
                                                   int& next_box_id,
                                                   StageContext& context) {
    IslandConnectorResult result;
    auto islands = find_islands(graph);
    context.diagnostics().add_counter("connector.islands_initial", static_cast<double>(islands.size()));
    if (islands.size() <= 1) {
        result.connected = true;
        return result;
    }

    std::unordered_map<int, double> frontier_best_gap_by_root;
    std::unordered_map<int, int> frontier_stale_by_root;
    while (config_.frontier_bridge && islands.size() > 1 &&
           result.bridge_boxes_added < config_.max_total_bridge_boxes) {
        if (context.should_stop()) {
            break;
        }
        FrontierBridgeCandidate candidate;
        if (!select_frontier_bridge_candidate(boxes,
                                              graph,
                                              oracle_,
                                              config_,
                                              frontier_best_gap_by_root,
                                              frontier_stale_by_root,
                                              context,
                                              candidate)) {
            break;
        }
        context.diagnostics().add_counter("connector.frontier_bridge_attempts");
        if (!add_frontier_bridge_box(candidate,
                                     boxes,
                                     oracle_,
                                     graph,
                                     next_box_id,
                                     config_,
                                     context)) {
            break;
        }
        result.bridge_boxes_added += 1;
        islands = find_islands(graph);
    }

    while (islands.size() > 1 &&
           (result.bridge_boxes_added < config_.max_total_bridge_boxes ||
            (config_.segment_edges_enabled && config_.rrt_segment_edges && !config_.segment_edges_fallback_only))) {
        if (context.should_stop()) {
            break;
        }
        auto map = make_box_map(boxes);
        // E5: gather candidates between every island pair in a single round so
        // the parallel_for fills all worker threads (the gaps are independent).
        // This is intentionally not largest-island-only: in shelf-like scenes,
        // two small query-anchor islands can be much closer to each other than
        // either is to the largest component, and connecting them first gives the
        // box connector a shorter, easier target.
        BridgePairCandidateRoundPlan candidate_plan =
            plan_bridge_pair_candidates(islands, map, config_, context);
        BridgePairExecutionResult pair_execution =
            run_bridge_pair_tasks(candidate_plan.candidates,
                                  map,
                                  oracle_,
                                  robot_,
                                  checker_,
                                  config_,
                                  context);
        result.attempted_pairs += pair_execution.attempted_pairs;
        std::vector<BridgePairResult> successful_pairs =
            std::move(pair_execution.successful_pairs);

        // E5 deterministic commit: process successful pairs in a stable order (by
        // task_id) and commit every bridge whose two islands are still in distinct
        // components, tracked by a union-find over island indices. This merges
        // multiple independent gaps per round while keeping commit fully serial and
        // order-independent of thread completion. Adjacency is recomputed once after
        // all commits.
        const BridgePairCommitRoundResult commit_result =
            commit_bridge_pair_results(std::move(successful_pairs),
                                       islands,
                                       candidate_plan.island_of,
                                       boxes,
                                       graph,
                                       segment_edges,
                                       oracle_,
                                       config_,
                                       next_box_id,
                                       result,
                                       context);
        if (commit_result.boxes_added_this_round) {
            context.diagnostics().add_counter("connector.round_full_adjacency_rebuilds_avoided");
        }
        apply_segment_edges_to_adjacency(segment_edges, graph);
        if (!commit_result.progressed) {
            break;
        }
        islands = find_islands(graph);
    }
    while (find_islands(graph).size() > 1) {
        if (boundary_max_depth_failure_count(context) <= 0.5) {
            context.diagnostics().add_counter("connector.point_gap_segment_blocked_no_max_depth_ffb_failure");
            break;
        }
        const std::size_t edge_count_before = segment_edges.size();
        if (!try_point_validated_gap_edge(boxes, graph, &segment_edges, checker_, config_, context)) {
            break;
        }
        if (segment_edges.size() > edge_count_before) {
            const int added_edges = static_cast<int>(segment_edges.size() - edge_count_before);
            result.segment_edges_added += added_edges;
            result.point_gap_segment_edges_added += added_edges;
        }
    }
    result.connected = find_islands(graph).size() <= 1;
    return result;
}

}  // namespace rbf
