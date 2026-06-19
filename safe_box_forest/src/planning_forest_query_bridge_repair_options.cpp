#include "planning_forest_query_bridge_corridor_repair.h"

#include "env_config.h"

#include <algorithm>
#include <limits>

namespace rbf {

QueryBridgeRepairSubdivisionOptions query_bridge_repair_subdivision_options(int query_index) {
    QueryBridgeRepairSubdivisionOptions options;
    options.base_subdivisions =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS", 6);
    options.subdivisions = std::max(
        0,
        detail::env_indexed_int_or_default("RBF_QUERY_BRIDGE_REPAIR_SUBDIVISIONS",
                                           query_index,
                                           options.base_subdivisions));
    options.fractions = query_bridge_center_ordered_fractions(options.subdivisions);
    return options;
}

QueryBridgeAdaptiveRepairOptions query_bridge_adaptive_repair_options(int query_index,
                                                                      int subdivisions,
                                                                      double audit_step,
                                                                      double sample_step) {
    QueryBridgeAdaptiveRepairOptions options;
    options.priority_mode =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_REPAIR_PRIORITY", 1);
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_STEP_REPAIR", 1) != 0;
    options.max_subdivisions = std::max(
        subdivisions + 1,
        detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_SUBDIVISIONS",
                                   std::max(2, subdivisions * 2)));
    options.fine_step = std::max(
        1e-4,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_FINE_STEP",
                                      std::max(audit_step, sample_step * 0.5)));
    options.max_calls = std::max(
        0,
        detail::env_indexed_int_or_default(
            "RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS",
            query_index,
            detail::env_int_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_REPAIR_CALLS",
                                       std::numeric_limits<int>::max())));
    return options;
}

}  // namespace rbf
