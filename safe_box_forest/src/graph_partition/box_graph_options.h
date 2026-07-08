#pragma once

#include <algorithm>

namespace rbf::detail {

struct AdjacencyIndexOptions {
    int selected_dim_count = 1;
};

inline int default_adjacency_index_dim_count(int box_count) {
    return box_count >= 3000 ? 3 : 1;
}

inline AdjacencyIndexOptions adjacency_index_options(int box_count) {
    AdjacencyIndexOptions options;
    options.selected_dim_count = std::max(1, default_adjacency_index_dim_count(box_count));
    return options;
}

}  // namespace rbf::detail
