#include "connector_pair_commit.h"

#include <SBF/box_graph.h>
#include <SBF/connector.h>
#include <SBF/runtime.h>
#include <SBF/segment_edge_types.h>

#include "connector_internal.h"

#include <algorithm>
#include <utility>

namespace rbf {

BridgePairCommitRoundResult commit_bridge_pair_results(
    std::vector<BridgePairResult> successful_pairs,
    const std::vector<std::vector<int>>& islands,
    const std::unordered_map<int, int>& island_of,
    std::vector<BoxNode>& boxes,
    AdjacencyGraph& graph,
    SegmentEdgeList& segment_edges,
    BoxOracle& oracle,
    const IslandConnectorConfig& config,
    int& next_box_id,
    IslandConnectorResult& connector_result,
    StageContext& context) {
    BridgePairCommitRoundResult round_result;
    std::sort(successful_pairs.begin(), successful_pairs.end(),
              [](const BridgePairResult& lhs, const BridgePairResult& rhs) {
                  return lhs.task_id < rhs.task_id;
              });
    std::vector<int> uf(islands.size());
    for (std::size_t i = 0; i < uf.size(); ++i) {
        uf[i] = static_cast<int>(i);
    }
    auto uf_find = [&](int x) {
        while (uf[x] != x) {
            uf[x] = uf[uf[x]];
            x = uf[x];
        }
        return x;
    };
    for (const auto& chosen : successful_pairs) {
        const auto src_isl_it = island_of.find(chosen.source_box_id);
        const auto tgt_isl_it = island_of.find(chosen.target_box_id);
        if (src_isl_it == island_of.end() || tgt_isl_it == island_of.end()) {
            continue;
        }
        const int src_root = uf_find(src_isl_it->second);
        const int tgt_root = uf_find(tgt_isl_it->second);
        if (src_root == tgt_root) {
            continue;
        }
        std::vector<Eigen::VectorXd> bridge_path = chosen.waypoint_path;
        if (config.pave.require_connected_chain && bridge_path.size() > 2) {
            const double pave_step =
                config.pave.gap_fill_sample_step > 0.0
                    ? std::max(0.05, config.pave.gap_fill_sample_step * 2.0)
                    : std::max(0.05, config.rrt.step_size * 0.5);
            bridge_path = densify_path_by_step(bridge_path, pave_step);
            context.diagnostics().set_value("connector.box_shortcut_densified_last_waypoints",
                                            static_cast<double>(bridge_path.size()));
        }
        int added = 0;
        bool box_connected = false;
        double pair_depth_failures_before = boundary_max_depth_failure_count(context);
        if (connector_result.bridge_boxes_added < config.max_total_bridge_boxes) {
            context.diagnostics().add_counter("connector.chain_pave_attempts");
            const int first_new_box_id = next_box_id;
            added = chain_pave_along_path(
                bridge_path,
                chosen.source_box_id,
                boxes,
                oracle,
                graph,
                next_box_id,
                context,
                config.pave);
            if (added > 0) {
                const int local_edges = connect_new_boxes_to_island(
                    boxes,
                    graph,
                    first_new_box_id,
                    next_box_id,
                    islands[static_cast<std::size_t>(tgt_isl_it->second)],
                    config.pave.adjacency_tolerance);
                if (local_edges > 0) {
                    context.diagnostics().add_counter("connector.chain_pave_local_target_edges",
                                                      static_cast<double>(local_edges));
                }
                box_connected = graph_has_path(graph, chosen.source_box_id, chosen.target_box_id);
                if (!box_connected) {
                    context.diagnostics().add_counter("connector.chain_pave_full_adjacency_rebuilds_avoided");
                    context.diagnostics().add_counter("connector.chain_pave_incremental_path_misses");
                    box_connected = graph_has_path(graph, chosen.source_box_id, chosen.target_box_id);
                }
                if (box_connected) {
                    context.diagnostics().add_counter("connector.chain_pave_box_connected");
                } else {
                    context.diagnostics().add_counter("connector.chain_pave_partial_added");
                }
            }
        }
        const bool pair_had_max_depth_ffb_failure =
            boundary_max_depth_failure_count(context) > pair_depth_failures_before + 0.5;
        bool added_segment_edge = false;
        if (!box_connected &&
            config.segment_edges_enabled && config.rrt_segment_edges &&
            !config.segment_edges_fallback_only &&
            pair_had_max_depth_ffb_failure) {
            const int edge_id = add_segment_edge(segment_edges,
                                                 graph,
                                                 chosen.source_box_id,
                                                 chosen.target_box_id,
                                                 bridge_path,
                                                 SegmentEdgeType::RRTConnector,
                                                 config.rrt.segment_resolution,
                                                 SegmentEdgeValidation::CollisionChecked,
                                                 false);
            if (edge_id >= 0) {
                added_segment_edge = true;
                connector_result.segment_edges_added += 1;
                connector_result.rrt_segment_edges_added += 1;
                context.diagnostics().add_counter("connector.segment_edges_added");
                context.diagnostics().add_counter("connector.rrt_segment_edges_added");
            }
        } else if (!box_connected &&
                   config.segment_edges_enabled && config.rrt_segment_edges &&
                   !config.segment_edges_fallback_only &&
                   !pair_had_max_depth_ffb_failure) {
            context.diagnostics().add_counter("connector.segment_edge_blocked_no_max_depth_ffb_failure");
        }
        if (box_connected) {
            context.diagnostics().add_counter("connector.chain_pave_successes");
            connector_result.bridge_boxes_added += added;
            round_result.boxes_added_this_round = true;
            uf[src_root] = tgt_root;
            round_result.progressed = true;
        } else if (added > 0) {
            connector_result.bridge_boxes_added += added;
            round_result.boxes_added_this_round = true;
            if (added_segment_edge) {
                uf[src_root] = tgt_root;
                round_result.progressed = true;
            }
        } else if (added_segment_edge) {
            uf[src_root] = tgt_root;
            round_result.progressed = true;
        } else {
            context.diagnostics().add_counter("connector.chain_pave_zero_added");
        }
    }
    return round_result;
}

}  // namespace rbf
