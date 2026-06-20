#pragma once

#include <SBF/leaf_sweep_types.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <vector>

namespace rbf {

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
