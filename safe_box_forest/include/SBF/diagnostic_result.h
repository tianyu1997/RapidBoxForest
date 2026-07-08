#pragma once

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API

#include <string>
#include <unordered_map>

namespace rbf {

struct RebuildProfile {
	int obstacles_before = 0;
	int obstacles_after = 0;
	int removed_obstacle_index = -1;
	int boxes_before = 0;
	int boxes_after = 0;
	int boxes_removed = 0;
	int boxes_added = 0;
	int raw_boxes_before = 0;
	int raw_boxes_after = 0;
	int raw_boxes_removed = 0;
	int raw_boxes_added = 0;
	int dirty_boxes = 0;
	int dirty_boxes_used = 0;
	int dirty_seed_count = 0;
	int regrow_attempts = 0;
	int bridge_boxes_added = 0;
	int segment_edges_added = 0;
	int rrt_segment_edges_added = 0;
	int point_gap_segment_edges_added = 0;
	int adjacency_islands = 0;
	int collision_cache_boxes_before = 0;
	int collision_cache_boxes_after = 0;
	int collision_cache_candidates = 0;
	int collision_cache_promoted = 0;
	int collision_cache_rejected_collision = 0;
	int collision_cache_rejected_contained = 0;
	int collision_cache_rejected_disconnected = 0;
	bool used_spatial_dirty_region = false;
	bool used_warm_rebuild = false;
	std::string fallback_reason;
	double dirty_region_ms = 0.0;
	double collision_check_ms = 0.0;
	double regrow_ms = 0.0;
	double warm_rebuild_ms = 0.0;
	double adjacency_ms = 0.0;
	double total_ms = 0.0;
	std::unordered_map<std::string, double> diagnostics;
};

} // namespace rbf

#endif
