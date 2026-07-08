#pragma once

#include <SBF/build_profile.h>
#include <SBF/leaf_sweep_types.h>

#include <string>
#include <unordered_map>

namespace rbf {

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
