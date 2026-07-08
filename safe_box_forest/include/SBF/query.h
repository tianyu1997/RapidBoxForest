#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/query_config.h>
#include <SBF/query_graph_cache_types.h>
#include <SBF/query_graph_types.h>
#include <SBF/query_result.h>
#include <SBF/segment_edge_fwd.h>

#include <utility>

namespace rbf {

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
