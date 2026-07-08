#pragma once

#include "../qroot/planning_forest_qroot_helpers.h"

#include <rbf/core.h>

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace rbf {

struct EndpointMainIndexes {
    std::unordered_map<int, std::size_t> box_index_by_id;
    std::unordered_map<int, int> node_owner;
    BoxSpatialIndex all_box_index;
    std::vector<int> main_box_ids;
    std::vector<BoxNode> main_boxes;
    BoxSpatialIndex main_box_index;
};

EndpointMainIndexes build_endpoint_main_indexes(
    const std::vector<BoxNode>& boxes,
    const std::vector<int>& main_island,
    int point_dims,
    double adjacency_tolerance,
    bool build_graph_spatial_indexes);

}  // namespace rbf
