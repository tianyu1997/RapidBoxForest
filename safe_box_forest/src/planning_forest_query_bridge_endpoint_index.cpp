#include "planning_forest_query_bridge_endpoint_index.h"

namespace rbf {

EndpointMainIndexes build_endpoint_main_indexes(
    const std::vector<BoxNode>& boxes,
    const std::vector<int>& main_island,
    int point_dims,
    double adjacency_tolerance,
    bool build_graph_spatial_indexes) {
    EndpointMainIndexes indexes;
    indexes.box_index_by_id.reserve(boxes.size() * 2);
    indexes.node_owner.reserve(boxes.size());
    for (std::size_t index = 0; index < boxes.size(); ++index) {
        const BoxNode& box = boxes[index];
        indexes.box_index_by_id[box.id] = index;
        if (indexes.node_owner.find(box.tree_id) == indexes.node_owner.end()) {
            indexes.node_owner[box.tree_id] = box.id;
        }
    }
    if (!build_graph_spatial_indexes) {
        return indexes;
    }
    indexes.all_box_index.rebuild(boxes, adjacency_tolerance);
    indexes.main_box_ids.reserve(main_island.size());
    indexes.main_boxes.reserve(main_island.size());
    for (int box_id : main_island) {
        auto it = indexes.box_index_by_id.find(box_id);
        if (it == indexes.box_index_by_id.end()) {
            continue;
        }
        const BoxNode& box = boxes[it->second];
        if (box.n_dims() != point_dims) {
            continue;
        }
        indexes.main_box_ids.push_back(box_id);
        indexes.main_boxes.push_back(box);
    }
    indexes.main_box_index.rebuild(indexes.main_boxes, adjacency_tolerance);
    return indexes;
}

}  // namespace rbf
