#pragma once

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/runtime.h>

#include <Eigen/Core>

#include <atomic>
#include <memory>
#include <vector>

namespace rbf {

struct RRTConnectConfig {
	int max_iters = 50000;
	double timeout_ms = 0.0;
	double step_size = 0.25;
	double goal_bias = 0.2;
	int segment_resolution = 16;
	double local_sampling_radius = 0.0;
};

struct ChainPaveConfig {
	BoxCommitPolicy commit_policy = BoxCommitPolicy::CommitCertifiedOnly;
	int max_chain = 0;
	int max_steps_per_waypoint = 12;
	bool refine_covered_waypoints = false;
	double adjacency_tolerance = 1e-9;
	FindFreeBoxOptions find_free_box;
};

struct IslandConnectorConfig {
	ChainPaveConfig pave;
	double per_pair_timeout_ms = 250.0;
	int max_pairs_per_gap = 8;
	RRTConnectConfig rrt;
	int max_total_bridge_boxes = 0;
	bool frontier_bridge = false;
	bool frontier_bridge_adaptive_ffb = false;
	int frontier_bridge_ffb_depth_increment = 2;
	int frontier_bridge_ffb_max_depth = 0;
	int frontier_bridge_candidate_limit = 8;
	int frontier_bridge_gap_stall_iterations = 4;
	double frontier_bridge_boundary_epsilon = 1e-9;
	bool segment_edges_enabled = true;
	bool rrt_segment_edges = true;
	bool point_gap_segment_edges = true;
	double point_validated_gap_tolerance = 0.0;
	int point_validated_gap_resolution = 16;
	int n_threads = 1;
	int pair_batch_size = 0;
	int parallel_threshold = 0;
	bool deterministic_reduce = true;
};

struct BridgePairTask {
	int task_id = -1;
	int source_box_id = -1;
	int target_box_id = -1;
	double score = 0.0;
};

struct BridgePairResult {
	int task_id = -1;
	int source_box_id = -1;
	int target_box_id = -1;
	std::vector<Eigen::VectorXd> waypoint_path;
	bool success = false;
};

struct IslandConnectorResult {
	bool connected = false;
	int attempted_pairs = 0;
	int bridge_boxes_added = 0;
	int segment_edges_added = 0;
	int rrt_segment_edges_added = 0;
	int point_gap_segment_edges_added = 0;
};

std::vector<Eigen::VectorXd> rrt_connect(const Eigen::Ref<const Eigen::VectorXd>& start,
										 const Eigen::Ref<const Eigen::VectorXd>& goal,
										 const CollisionChecker& checker,
										 const Robot& robot,
										 const RRTConnectConfig& config = {},
										 int seed = 0,
										 std::shared_ptr<std::atomic<bool>> cancel = {});
std::vector<Eigen::VectorXd> rrt_connect(const Eigen::Ref<const Eigen::VectorXd>& start,
										 const Eigen::Ref<const Eigen::VectorXd>& goal,
										 const CollisionChecker& checker,
										 const Robot& robot,
										 StageContext& context,
										 const RRTConnectConfig& config = {},
										 int seed = 0);
int chain_pave_along_path(const std::vector<Eigen::VectorXd>& waypoint_path,
						  int anchor_box_id,
						  std::vector<BoxNode>& boxes,
						  BoxOracle& oracle,
						  AdjacencyGraph& graph,
						  int& next_box_id,
						  const ChainPaveConfig& config = {});
int chain_pave_along_path(const std::vector<Eigen::VectorXd>& waypoint_path,
						  int anchor_box_id,
						  std::vector<BoxNode>& boxes,
						  BoxOracle& oracle,
						  AdjacencyGraph& graph,
						  int& next_box_id,
						  StageContext& context,
						  const ChainPaveConfig& config = {});

class IslandConnector {
public:
	IslandConnector(BoxOracle& oracle,
					const Robot& robot,
					const CollisionChecker& checker,
					IslandConnectorConfig config = {});
	IslandConnectorResult connect_all(std::vector<BoxNode>& boxes, AdjacencyGraph& graph, int& next_box_id);
	IslandConnectorResult connect_all(std::vector<BoxNode>& boxes,
									  AdjacencyGraph& graph,
									  SegmentEdgeList& segment_edges,
									  int& next_box_id);
	IslandConnectorResult connect_all(std::vector<BoxNode>& boxes,
									  AdjacencyGraph& graph,
									  int& next_box_id,
									  StageContext& context);
	IslandConnectorResult connect_all(std::vector<BoxNode>& boxes,
									  AdjacencyGraph& graph,
									  SegmentEdgeList& segment_edges,
									  int& next_box_id,
									  StageContext& context);

private:
	BoxOracle& oracle_;
	const Robot& robot_;
	const CollisionChecker& checker_;
	IslandConnectorConfig config_;
};

}  // namespace rbf
