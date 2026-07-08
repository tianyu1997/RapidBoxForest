#pragma once

#include "planning_forest_adaptive_cover_utils.h"

#include <unordered_set>
#include <vector>

namespace rbf {

class AdaptiveGridPartition;
class DatabaseBoxOracle;

bool adaptive_commit_free_box_candidate(
    const AdaptiveFrontierItem& item,
    const OracleValidationDetail& detail,
    int depth,
    bool item_has_seed_hit,
    int new_box_id,
    bool use_partition_backend,
    bool adaptive_partition_query_enabled,
    AdaptiveGridPartition* adaptive_partition,
    double adjacency_tolerance,
    int adjacency_batch_size,
    DatabaseBoxOracle& oracle,
    std::vector<BoxNode>& boxes,
    std::vector<BoxNode>& raw_boxes,
    std::vector<BoxNode>& scoring_boxes,
    AdjacencyGraph& adjacency,
    std::unordered_set<int>& main_ids,
    std::size_t& first_unconnected_new_index,
    int& pending_adjacency_boxes,
    AdaptiveLeafSweepResult& result);

}  // namespace rbf
