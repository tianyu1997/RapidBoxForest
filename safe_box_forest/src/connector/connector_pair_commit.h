#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/connector_types.h>
#include <SBF/runtime_fwd.h>
#include <SBF/segment_edge_fwd.h>

#include <unordered_map>
#include <vector>

namespace rbf {

class BoxOracle;

struct BridgePairCommitRoundResult {
    bool progressed = false;
    bool boxes_added_this_round = false;
};

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
    StageContext& context);

}  // namespace rbf
