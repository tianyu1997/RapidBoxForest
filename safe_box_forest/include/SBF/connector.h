#pragma once

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/runtime.h>

#include <Eigen/Core>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace rbf {

struct RRTConnectConfig {
	int max_iters = 50000;
	double timeout_ms = 0.0;
	// step_size 0.25 rad was too small for the iiwa joint scale: the BiRRT trees
	// expanded too slowly to traverse narrow shelf passages within the per-pair
	// budget. step_size 0.5 with segment_resolution 32 keeps the collision-check
	// density identical (0.5/32 == 0.25/16 == 0.0156 rad/check) while reaching
	// farther per iteration. Measured (iiwa+shelf, 80ms, 10 seeds, matched
	// density): TS->CS 7/10 -> 9/10 (47ms -> 15ms median), CS->LB 8/10 -> 9/10;
	// pipeline island-bridging unchanged (14/4/4 successes/timeouts/failures).
	double step_size = 0.5;
	double goal_bias = 0.2;
	int segment_resolution = 32;
	// When > 0, validate every RRT/shortcut segment with spacing no larger than
	// this joint-space step. This must match the final audit spacing for paper
	// experiments so build-stage segment witnesses cannot be looser than query
	// audit.
	double segment_step = 0.0;
	double local_sampling_radius = 0.0;
	bool shortcut_path = true;
	int optimize_after_first_iters = 0;
	double domain_tolerance = 1e-3;
	std::vector<Interval> domain_intervals;
};

struct DebugBoundaryFfbFailure {
	std::vector<double> seed;
	std::vector<Interval> intervals;
	OracleValidationDetail validation_detail;
	int node = -1;
	int depth = -1;
	int changed_dim = -1;
	int fail_code = 0;
	bool hit_unknown_depth_cap = false;
	bool hit_reserved_depth_cap = false;
};

struct ChainPaveConfig {
	BoxCommitPolicy commit_policy = BoxCommitPolicy::CommitCertifiedOnly;
	int max_chain = 0;
	int max_steps_per_waypoint = 12;
	bool refine_covered_waypoints = false;
	double adjacency_tolerance = 1e-9;
	// Residual-gap filling: when a box certified at a seed cannot be committed
	// because it is not adjacent to the current chain box (e.g. canonical boxes
	// that only touch diagonally, or a thin uncovered sliver between them), bisect
	// the segment between the current box center and the seed and recursively
	// insert intermediate connected boxes until the chain reaches the seed. This
	// lets chain_pave fully cover a connector segment instead of leaving holes.
	bool fill_gaps = false;
	// Maximum bisection recursion depth per gap (<=0 disables gap filling). Each
	// level halves the remaining segment, so depth d allows up to 2^d subdivisions.
	int max_gap_fill_depth = 8;
	// Stop bisecting once the midpoint is within this C-space distance of either
	// endpoint, to avoid degenerate zero-length subdivisions.
	double gap_fill_min_step = 1e-4;
	// Arc-length spacing used to densify the bridge polyline before per-sample
	// box seeding. Smaller => denser samples => higher segment coverage at the
	// cost of more boxes. <=0 falls back to max_steps_per_waypoint subdivisions.
	// In fast-budget mode this is only a coarse probe spacing for locating gaps;
	// the real budget is the wall-clock/FFB-call limit below.
	double gap_fill_sample_step = 0.05;
	// Fast gap-fill budget. >0 enables wall-clock bounded greedy gap filling;
	// <=0 disables the local wall-clock deadline (old exhaustive behavior can be
	// selected by also setting gap_fill_max_ffb_calls < 0).
	double gap_fill_time_budget_ms = 10.0;
	// Maximum fresh FFB calls in fast gap-fill. <0 means unlimited (subject to
	// max_chain/time); 0 means reuse-only coverage.
	int gap_fill_max_ffb_calls = 32;
	// Compatibility flag for connector bridge handling. Chain paving always
	// commits only graph-adjacent boxes; this flag only requests shortcutting and
	// densification of the bridge polyline before paving.
	bool require_connected_chain = false;
	FindFreeBoxOptions find_free_box;
	std::vector<DebugBoundaryFfbFailure>* debug_boundary_failures = nullptr;
};

struct IslandConnectorConfig {
	ChainPaveConfig pave;
	double per_pair_timeout_ms = 250.0;
	int max_pairs_per_gap = 8;
	RRTConnectConfig rrt;
	bool enable_birrt = true;
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
	bool segment_edges_fallback_only = false;
	double point_validated_gap_tolerance = 0.0;
	int point_validated_gap_resolution = 16;
	double point_validated_gap_step = 0.0;
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
