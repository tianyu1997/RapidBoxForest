#pragma once

#include "env_config.h"

#include <algorithm>

namespace rbf::detail {

struct AdjacencyIndexOptions {
    int selected_dim_count = 1;
};

inline int default_adjacency_index_dim_count(int box_count) {
    return box_count >= env_int_or_default("RBF_ADJACENCY_MULTI_DIM_THRESHOLD", 3000)
        ? 3
        : 1;
}

inline AdjacencyIndexOptions adjacency_index_options_from_env(int box_count) {
    AdjacencyIndexOptions options;
    const int default_dim_count = default_adjacency_index_dim_count(box_count);
    options.selected_dim_count =
        std::max(1, env_int_or_default("RBF_ADJACENCY_INDEX_DIMS", default_dim_count));
    return options;
}

}  // namespace rbf::detail
