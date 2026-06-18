#pragma once

#include "env_config.h"

#include <algorithm>

namespace rbf {

struct QueryGraphCostOptions {
    double box_transition_penalty = 0.0;
    double box_nonprogress_penalty = 0.0;
    double box_line_deviation_penalty = 0.0;
    double query_bridge_penalty = 0.0;
    int active_query_index = -1;
    double foreign_query_edge_penalty = 0.0;
};

struct QueryShortcutCostOptions {
    bool cost_aware = true;
    double cost_factor = 1.05;
};

inline QueryGraphCostOptions query_graph_cost_options_from_env() {
    QueryGraphCostOptions options;
    options.box_transition_penalty = std::max(
        0.0,
        detail::env_double_or_default("RBF_BOX_TRANSITION_EDGE_COST_PENALTY", 0.0));
    options.box_nonprogress_penalty = std::max(
        0.0,
        detail::env_double_or_default("RBF_BOX_TRANSITION_NONPROGRESS_PENALTY", 0.0));
    options.box_line_deviation_penalty = std::max(
        0.0,
        detail::env_double_or_default("RBF_BOX_TRANSITION_LINE_DEVIATION_PENALTY", 0.0));
    options.query_bridge_penalty = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_EDGE_COST_PENALTY", 0.0));
    options.active_query_index =
        detail::env_int_or_default("RBF_ACTIVE_QUERY_INDEX", -1);
    options.foreign_query_edge_penalty = options.active_query_index >= 0
        ? std::max(0.0,
                   detail::env_double_or_default("RBF_QUERY_FOREIGN_EDGE_COST_PENALTY", 0.0))
        : 0.0;
    return options;
}

inline QueryShortcutCostOptions query_shortcut_cost_options_from_env(bool boxes_available) {
    QueryShortcutCostOptions options;
    options.cost_aware =
        detail::env_int_or_default("RBF_QUERY_SHORTCUT_COST_AWARE", 1) != 0 &&
        boxes_available;
    options.cost_factor =
        std::max(1.0, detail::env_double_or_default("RBF_QUERY_SHORTCUT_COST_FACTOR", 1.05));
    return options;
}

}  // namespace rbf
