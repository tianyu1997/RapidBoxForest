#pragma once

#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <rbf/core.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace rbf {

struct LeafSweepConfig {
	double obstacle_cluster_gap = 0.0;
	int n_threads = 0;
	int validation_batch_size = 256;
	double timeout_ms = 0.0;
	bool store_group_results = true;
	bool pre_split_to_max_depth = false;
	bool use_virtual_topology = false;
	bool parallel_virtual_validation = false;
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
	std::vector<LeafSweepGroupResult> groups;
	std::vector<int> obstacle_group_ids;
	bool deadline_reached = false;
	double initialize_ms = 0.0;
	double group_sweep_ms = 0.0;
	double compose_ms = 0.0;
	double total_ms = 0.0;
	std::unordered_map<std::string, double> diagnostics;
};

class LeafSweepGrower {
public:
	LeafSweepGrower(DatabaseBoxOracle& oracle,
				   LeafSweepConfig config = {},
				   OracleSplitOptions split_options = {});

	LeafSweepResult sweep(const std::vector<Obstacle>& obstacles,
						  int start_depth,
						  int max_depth);
	LeafSweepResult sweep(const std::vector<Obstacle>& obstacles,
						  int start_depth,
						  int max_depth,
						  StageContext& context);

private:
	struct PendingNode {
		OracleNodeId node = kInvalidOracleNodeId;
		int changed_dim = -1;
		std::vector<Interval> intervals;
	};

	struct GroupWork {
		LeafSweepGroupResult result;
		std::vector<OracleNodeId> free_nodes;
		std::vector<OracleNodeId> collision_nodes;
	};

	std::vector<GroupWork> cluster_obstacles(const std::vector<Obstacle>& obstacles,
											 LeafSweepResult& result,
											 StageContext& context) const;
	std::vector<PendingNode> materialize_start_frontier(int start_depth,
														int max_depth,
														StageContext& context,
														LeafSweepResult& result);
	void sweep_group(GroupWork& group,
					 const std::vector<PendingNode>& start_frontier,
					 int max_depth,
					 StageContext& context,
					 LeafSweepResult& result);
	void compose_final_sets(const std::vector<GroupWork>& groups,
							int start_depth,
							int max_depth,
							StageContext& context,
							LeafSweepResult& result) const;
	bool virtual_split_node(const PendingNode& item,
							int depth,
							PendingNode& left,
							PendingNode& right) const;
	int virtual_depth(OracleNodeId node) const;

	DatabaseBoxOracle& oracle_;
	LeafSweepConfig config_;
	OracleSplitOptions split_options_;
};

}  // namespace rbf
