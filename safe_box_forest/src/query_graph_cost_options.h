#pragma once

#include "env_config.h"

#include <algorithm>

namespace rbf {

struct QueryShortcutCostOptions {
    bool cost_aware = true;
    double cost_factor = 1.05;
};

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
