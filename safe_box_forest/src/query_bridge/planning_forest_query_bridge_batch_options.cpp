#include "planning_forest_query_bridge_options.h"

#include <SBF/planning_config.h>

#include <algorithm>
namespace rbf {

QueryBridgeHybridizeAttemptOptions query_bridge_hybridize_attempt_options_from_config(
    const RBFPlanningConfig& config) {
    QueryBridgeHybridizeAttemptOptions options;
    options.enabled = config.query_bridge_hybridize_attempt_paths;
    options.max_paths =
        std::max(2, config.query_bridge_hybrid_max_paths);
    options.max_vertices =
        std::max(8, config.query_bridge_hybrid_max_vertices);
    options.max_cross_checks =
        std::max(1, config.query_bridge_hybrid_max_cross_checks);
    return options;
}

}  // namespace rbf
