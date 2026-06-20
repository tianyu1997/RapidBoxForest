#pragma once

#include <SBF/connector_types.h>
#include <SBF/runtime.h>

#include <Eigen/Core>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace rbf {

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
