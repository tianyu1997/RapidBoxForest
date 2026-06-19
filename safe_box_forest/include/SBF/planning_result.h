#pragma once

#include <SBF/leaf_sweep_grower.h>
#include <SBF/runtime.h>

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

/// Profile returned by RBFPlanningForest::run_pure_ffb().
/// Contains timing and oracle diagnostics for a batch of isolated FFB calls,
/// with no RRT / grower / adjacency overhead.
struct PureFfbProfile {
	int n_attempts = 0;  ///< Number of seed points attempted.
	int n_found    = 0;  ///< Number of seeds that produced a free box.
	double total_ms = 0.0;  ///< Wall time for all FFB calls in this batch.
	/// Sum of FindFreeBoxResult::decisions (validate_node calls) across all seeds.
	int sum_decisions = 0;
	/// Sum of FindFreeBoxResult::splits (tree node splits) across all seeds.
	int sum_splits = 0;
	/// Sum of oracle.depth(found_node) for every seed that produced a free box.
	int sum_found_depth = 0;
	std::unordered_map<std::string, double> diagnostics;
};

struct LeafSweepRefineResult {
	LeafSweepResult leaf_sweep;
	BuildProfile profile;
	int leaf_free_count = 0;
	int leaf_collision_count = 0;
	int deep_boxes_added = 0;
	int deep_domain_attempts = 0;
	int deep_ffb_success = 0;
	int deep_ffb_fail = 0;
	int deep_commit_rejects = 0;
	int deep_domain_rejects = 0;
	int deep_contained_rejects = 0;
	int deep_adjacency_rejects = 0;
	int deep_anchor_roots_added = 0;
	double leaf_sweep_ms = 0.0;
	double deep_refine_ms = 0.0;
	double connector_ms = 0.0;
	double total_ms = 0.0;
	std::unordered_map<std::string, double> diagnostics;
};

struct AdaptiveLeafSweepResult {
	LeafSweepResult leaf_sweep;
	BuildProfile profile;
	int shallow_free_count = 0;
	int shallow_collision_count = 0;
	int adaptive_free_added = 0;
	int adaptive_validated = 0;
	int adaptive_splits = 0;
	int adaptive_deferred = 0;
	int adaptive_promoted = 0;
	int unresolved_domains = 0;
	int seed_probe_count = 0;
	int seed_probe_free_count = 0;
	int seed_probe_box_covered = 0;
	int seed_probe_anchor_success = 0;
	int seed_probe_main_accessible = 0;
	double p_box_covered = 0.0;
	double p_anchor_success = 0.0;
	double p_main_accessible = 0.0;
	double p_anchor_to_main_uncovered = 0.0;
	int selected_leaf_depth = 0;
	bool adaptive_depth_readiness_met = false;
	std::string adaptive_depth_stop_reason;
	std::string adaptive_depth_snapshots_json;
	double leaf_sweep_ms = 0.0;
	double adaptive_ms = 0.0;
	double coverage_probe_ms = 0.0;
	double total_ms = 0.0;
	int partition_cell_count = 0;
	int partition_grid_cell_count = 0;
	int partition_non_grid_cell_count = 0;
	int partition_face_index_entries = 0;
	int partition_islands = 0;
	int partition_largest_island = 0;
	std::unordered_map<std::string, double> diagnostics;
};

} // namespace rbf
