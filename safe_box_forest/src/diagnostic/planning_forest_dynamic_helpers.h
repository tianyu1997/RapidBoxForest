#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/dynamic_update_config.h>
#include <SBF/scene_types.h>

#include <rbf/core.h>

#include <unordered_set>
#include <vector>

namespace rbf {

std::vector<int> spatial_dirty_all_box_indices(const Robot& robot,
                                               const std::vector<BoxNode>& boxes,
                                               const Obstacle& obstacle,
                                               const DynamicUpdateConfig& config,
                                               int& dirty_count);

bool has_adjacency_to_existing_box(const std::vector<BoxNode>& boxes,
                                   const BoxNode& box,
                                   double tolerance,
                                   int* parent_box_id);

void remove_local_edge(AdjacencyGraph& graph, int lhs, int rhs);

void remove_adjacency_nodes(AdjacencyGraph& graph,
                            const std::unordered_set<int>& removed_ids);

std::unordered_set<int> collect_local_adjacency_ids(const std::vector<BoxNode>& live_boxes,
                                                    const std::vector<BoxNode>& local_domains,
                                                    double tolerance);

void rebuild_local_adjacency(AdjacencyGraph& graph,
                             const std::vector<BoxNode>& boxes,
                             const std::unordered_set<int>& local_ids,
                             double tolerance);

}  // namespace rbf
