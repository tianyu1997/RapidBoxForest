#pragma once

#include <SBF/api.h>
#include <SBF/box_graph.h>

#include <utility>

namespace rbf {

struct QueryConfig {
	bool nearest_if_outside = false;
	bool shortcut_boxes = true;
	bool shortcut_cost_aware = true;
	double shortcut_cost_factor = 1.05;
	bool collision_shortcut = true;
	bool strict_path_audit = false;
	int audit_resolution = 16;
	// When > 0, strict path audit samples each segment at least every audit_segment_step
	// radians (in addition to audit_resolution). Must match experiment post-audit step.
	double audit_segment_step = 0.01;
	double audit_collision_tolerance = 0.0;
	bool repair_on_audit_failure = false;
	double repair_timeout_ms = 0.0;
	int repair_rrt_max_iters = 50000;
	int repair_max_attempts = 1;
	double repair_local_sampling_radius = 0.0;
	double repair_local_sampling_growth = 2.0;
	int collision_shortcut_resolution = 16;
	bool final_rrt_simplify = false;
	double final_rrt_simplify_timeout_ms = 0.0;
	int final_rrt_simplify_max_iters = 50000;
	int final_rrt_simplify_attempts = 1;
	double adjacency_tolerance = 1e-9;
};

class CorridorQuery {
public:
	explicit CorridorQuery(QueryConfig config = {}) : config_(std::move(config)) {}
	QueryResult run(const std::vector<BoxNode>& boxes,
					const AdjacencyGraph& graph,
					const Eigen::Ref<const Eigen::VectorXd>& start,
					const Eigen::Ref<const Eigen::VectorXd>& goal,
					const QueryGraphCostOptions& graph_cost = {}) const;
	QueryResult run(const std::vector<BoxNode>& boxes,
					const AdjacencyGraph& graph,
					const SegmentEdgeList& segment_edges,
					const Eigen::Ref<const Eigen::VectorXd>& start,
					const Eigen::Ref<const Eigen::VectorXd>& goal,
					const QueryGraphCostOptions& graph_cost = {}) const;
	QueryResult run(const QueryGraphCache& cache,
					const Eigen::Ref<const Eigen::VectorXd>& start,
					const Eigen::Ref<const Eigen::VectorXd>& goal,
					const QueryGraphCostOptions& graph_cost = {}) const;

private:
	QueryConfig config_;
};

}  // namespace rbf
