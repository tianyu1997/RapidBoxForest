#pragma once

// Compatibility facade for older query-bridge translation units. New code
// should include the narrower task/options/policy/diagnostics/attempt-paths
// headers directly.

#include "planning_forest_query_bridge_attempt_paths.h"
#include "planning_forest_query_bridge_diagnostics.h"
#include "planning_forest_query_bridge_options.h"
#include "planning_forest_query_bridge_policy.h"
#include "planning_forest_query_bridge_task.h"

namespace rbf {

struct QueryBridgeDetourOptions;

}  // namespace rbf
