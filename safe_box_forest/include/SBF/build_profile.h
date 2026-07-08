#pragma once

#include <string>
#include <unordered_map>

namespace rbf {

struct BuildProfile {
    int raw_boxes = 0;
    int final_boxes = 0;
    int segment_edges = 0;
    int grow_adjacency_islands = 0;
    int grow_largest_island = 0;
    int adjacency_islands = 0;
    int bridge_boxes_added = 0;
    int segment_edges_added = 0;
    int rrt_segment_edges_added = 0;
    int point_gap_segment_edges_added = 0;
    int connector_attempted_pairs = 0;
    bool connector_connected = false;
    double grow_ms = 0.0;
    double merge_ms = 0.0;
    double connector_ms = 0.0;
    double adjacency_ms = 0.0;
    double collision_check_ms = 0.0;
    double envelope_ms = 0.0;
    double total_ms = 0.0;
    int envelope_batches = 0;
    int envelope_threads = 1;
    std::unordered_map<std::string, double> diagnostics;
};

}  // namespace rbf
