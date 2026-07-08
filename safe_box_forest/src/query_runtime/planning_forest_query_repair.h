#pragma once

#include <SBF/box_adjacency_types.h>
#include <SBF/connector_types.h>
#include <SBF/query_config.h>
#include <SBF/query_graph_types.h>
#include <SBF/query_result.h>
#include <SBF/query_runtime_config.h>
#include <SBF/scene_types.h>
#include <SBF/segment_edge_fwd.h>

#include "../planning_core/planning_forest_audit.h"

namespace rbf {

struct RBFPlanningConfig;

QueryGraphCostOptions query_graph_cost_options_from_runtime(
    const RBFPlanningConfig& config,
    const RBFQueryRuntimeOptions& runtime_options);

void summarize_query_path(QueryResult& result,
                          const std::vector<BoxNode>& boxes,
                          const SegmentEdgeList& segment_edges);

bool try_local_birrt_repair(QueryResult& result,
                            const PathAuditCheck& audit,
                            const CollisionChecker& checker,
                            const Robot& robot,
                            const QueryConfig& query_config,
                            const RRTConnectConfig& base_repair_config,
                            int planner_seed_base);

}  // namespace rbf
