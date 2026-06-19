#include "planning_forest_query_bridge_batch_utils.h"

#include "env_config.h"

namespace rbf {

QueryBridgeIndexOptions query_bridge_index_options_from_env() {
    QueryBridgeIndexOptions options;
    options.force_indices_csv = detail::env_string_or_empty("RBF_QUERY_BRIDGE_FORCE_INDICES");
    options.global_indices_csv = detail::env_string_or_empty("RBF_QUERY_BRIDGE_GLOBAL_INDICES");
    options.segment_only_indices_csv =
        detail::env_string_or_empty("RBF_QUERY_BRIDGE_SEGMENT_ONLY_INDICES");
    return options;
}

bool query_bridge_index_forced(const QueryBridgeIndexOptions& options,
                               std::size_t index) {
    return detail::csv_nonnegative_index_contains(options.force_indices_csv, index);
}

bool query_bridge_index_segment_only(const QueryBridgeIndexOptions& options,
                                     std::size_t index) {
    return detail::csv_nonnegative_index_contains(options.segment_only_indices_csv, index);
}

int query_bridge_index_global(const QueryBridgeIndexOptions& options,
                              std::size_t position,
                              int fallback) {
    return detail::csv_position_int_or_default(options.global_indices_csv, position, fallback);
}

}  // namespace rbf
