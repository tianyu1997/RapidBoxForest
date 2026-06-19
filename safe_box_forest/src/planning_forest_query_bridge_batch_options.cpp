#include "planning_forest_query_bridge_options.h"

#include "env_config.h"

#include <algorithm>
namespace rbf {

QueryBridgeHybridizeAttemptOptions query_bridge_hybridize_attempt_options_from_env() {
    QueryBridgeHybridizeAttemptOptions options;
    options.enabled =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRIDIZE_ATTEMPT_PATHS", 0) != 0;
    options.max_paths =
        std::max(2, detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_PATHS", 8));
    options.max_vertices =
        std::max(8, detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_VERTICES", 128));
    options.max_cross_checks =
        std::max(1,
                 detail::env_int_or_default("RBF_QUERY_BRIDGE_HYBRID_MAX_CROSS_CHECKS",
                                            4096));
    return options;
}

}  // namespace rbf
