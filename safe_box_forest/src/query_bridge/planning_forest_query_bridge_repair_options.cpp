#include "planning_forest_query_bridge_corridor_repair.h"

#include <SBF/planning_config.h>

#include <algorithm>
#include <limits>

namespace rbf {

QueryBridgeRepairSubdivisionOptions query_bridge_repair_subdivision_options(int query_index) {
    (void)query_index;
    QueryBridgeRepairSubdivisionOptions options;
    options.subdivisions = 1;
    options.fractions = query_bridge_center_ordered_fractions(options.subdivisions);
    return options;
}

QueryBridgeAdaptiveRepairOptions query_bridge_adaptive_repair_options(const RBFPlanningConfig& config,
                                                                      int query_index,
                                                                      int subdivisions,
                                                                      double audit_step,
                                                                      double sample_step) {
    QueryBridgeAdaptiveRepairOptions options;
    options.enabled = true;
    const int fallback_subdivisions = std::max(2, subdivisions * 2);
    const int configured_subdivisions =
        config.query_bridge_adaptive_max_repair_subdivisions;
    options.max_subdivisions = std::max(
        subdivisions + 1,
        configured_subdivisions >= 0 ? configured_subdivisions : fallback_subdivisions);
    const double fallback_fine_step = std::max(audit_step, sample_step * 0.5);
    const double configured_fine_step = config.query_bridge_adaptive_fine_step;
    options.fine_step = std::max(
        1e-4,
        configured_fine_step > 0.0 ? configured_fine_step : fallback_fine_step);
    int max_calls = config.query_bridge_adaptive_max_repair_calls >= 0
        ? config.query_bridge_adaptive_max_repair_calls
        : std::numeric_limits<int>::max();
    if (query_index >= 0 &&
        static_cast<std::size_t>(query_index) <
            config.query_bridge_adaptive_max_repair_calls_by_query.size()) {
        const int indexed =
            config.query_bridge_adaptive_max_repair_calls_by_query[query_index];
        if (indexed >= 0) {
            max_calls = indexed;
        }
    }
    options.max_calls = std::max(
        0,
        max_calls);
    return options;
}

}  // namespace rbf
