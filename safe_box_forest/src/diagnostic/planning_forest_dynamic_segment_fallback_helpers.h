#pragma once

#include <SBF/adaptive_grid_partition_types.h>
#include <SBF/diagnostic_result.h>

#include "../query_runtime/planning_forest_query_utils.h"

#include <rbf/core.h>

#include <Eigen/Core>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

inline bool dynamic_segment_point_covered_by_existing_box(
    const std::vector<BoxNode>& boxes,
    const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return intervals_contain_point_strict_local(box.joint_intervals, point);
    });
}

inline void dynamic_segment_merge_diagnostic_snapshot(
    std::unordered_map<std::string, double>& diagnostics,
    const std::unordered_map<std::string, double>& snapshot) {
    for (const auto& [key, value] : snapshot) {
        diagnostics[key] += value;
    }
}

inline void initialize_segment_fallback_profile(RebuildProfile& profile,
                                                int boxes_size,
                                                int raw_boxes_size,
                                                int obstacle_count,
                                                int collision_cache_size,
                                                int segment_edge_count) {
    profile.boxes_before = boxes_size;
    profile.raw_boxes_before = raw_boxes_size;
    profile.obstacles_before = obstacle_count;
    profile.obstacles_after = obstacle_count;
    profile.collision_cache_boxes_before = collision_cache_size;
    profile.collision_cache_boxes_after = collision_cache_size;
    profile.diagnostics["segment_fallback.segment_edges_before"] =
        static_cast<double>(segment_edge_count);
}

inline void record_segment_fallback_partition_stats(RebuildProfile& profile,
                                                    const AdaptiveGridPartitionStats& partition_stats,
                                                    int segment_edge_count) {
    profile.diagnostics["segment_fallback.segment_edges_after"] =
        static_cast<double>(segment_edge_count);
    profile.diagnostics["segment_fallback.islands_after"] =
        static_cast<double>(profile.adjacency_islands);
    profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(partition_stats.cells);
    profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(partition_stats.islands);
    profile.diagnostics["adaptive.partition_overlay_edges"] =
        static_cast<double>(partition_stats.overlay_edges);
}

}  // namespace rbf
