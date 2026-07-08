#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/connector_types.h>
#include <SBF/runtime_fwd.h>
#include <SBF/segment_edge_fwd.h>

#include <Eigen/Core>

#include <limits>
#include <unordered_map>
#include <vector>

namespace rbf {

class BoxOracle;
class CollisionChecker;

struct FrontierBridgeCandidate {
    int source_box_id = -1;
    int target_box_id = -1;
    int source_root_id = -1;
    Eigen::VectorXd seed;
    double gap_sq = std::numeric_limits<double>::infinity();
    double projected_gap_sq = std::numeric_limits<double>::infinity();
    double face_score = std::numeric_limits<double>::infinity();
    int ffb_depth_override = 0;
};

bool select_frontier_bridge_candidate(const std::vector<BoxNode>& boxes,
                                      const AdjacencyGraph& graph,
                                      BoxOracle& oracle,
                                      const IslandConnectorConfig& config,
                                      std::unordered_map<int, double>& best_gap_by_root,
                                      std::unordered_map<int, int>& stale_by_root,
                                      StageContext& context,
                                      FrontierBridgeCandidate& out);

bool add_frontier_bridge_box(const FrontierBridgeCandidate& candidate,
                             std::vector<BoxNode>& boxes,
                             BoxOracle& oracle,
                             AdjacencyGraph& graph,
                             int& next_box_id,
                             const IslandConnectorConfig& config,
                             StageContext& context);

bool try_point_validated_gap_edge(const std::vector<BoxNode>& boxes,
                                  AdjacencyGraph& graph,
                                  SegmentEdgeList* segment_edges,
                                  const CollisionChecker& checker,
                                  const IslandConnectorConfig& config,
                                  StageContext& context);

}  // namespace rbf
