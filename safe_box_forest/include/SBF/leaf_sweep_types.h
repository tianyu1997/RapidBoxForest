#pragma once

#include <SBF/scene_types.h>

#include <rbf/core.h>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

struct LeafSweepResult;

struct LeafSweepConfig {
	double obstacle_cluster_gap = 0.0;
	int n_threads = 0;
	int validation_batch_size = 256;
	double timeout_ms = 0.0;
	bool store_group_results = true;
	bool pre_split_to_max_depth = false;
	bool use_virtual_topology = false;
	bool parallel_virtual_validation = false;
	int collision_overlap_prune_min_depth = -1;
	double collision_overlap_prune_threshold = 0.0;
	double collision_overlap_prune_min_threshold = 0.0;
	double collision_overlap_prune_decay_per_depth = 0.0;
	double collision_overlap_prune_ratio_threshold = 0.0;
	int max_free_boxes = 0;
	int max_collision_boxes = 0;
	std::vector<int> checkpoint_depths;
	std::function<bool(const LeafSweepResult&, int)> checkpoint_callback;
};

struct LeafSweepGroupResult {
	int group_id = -1;
	std::vector<int> obstacle_indices;
	std::vector<Obstacle> obstacles;
	Obstacle aggregate_obstacle;
	std::vector<BoxNode> free_boxes;
	std::vector<BoxNode> collision_boxes;
};

struct LeafSweepResult {
	std::vector<BoxNode> free_boxes;
	std::vector<BoxNode> collision_boxes;
	std::vector<std::vector<int>> collision_box_obstacle_indices;
	std::vector<LeafSweepGroupResult> groups;
	std::vector<int> obstacle_group_ids;
	bool deadline_reached = false;
	double initialize_ms = 0.0;
	double group_sweep_ms = 0.0;
	double compose_ms = 0.0;
	double total_ms = 0.0;
	std::unordered_map<std::string, double> diagnostics;
};

}  // namespace rbf
