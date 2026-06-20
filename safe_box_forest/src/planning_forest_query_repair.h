#pragma once

#include <SBF/box_graph.h>
#include <SBF/connector_types.h>
#include <SBF/planning_config.h>
#include <SBF/query.h>
#include <SBF/query_bridge_config.h>
#include <LECTDatabase/sbf/scene.h>

#include "planning_forest_audit.h"

namespace rbf {

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
