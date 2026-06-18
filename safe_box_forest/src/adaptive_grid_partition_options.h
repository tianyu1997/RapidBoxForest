#pragma once

#include "env_config.h"

#include <algorithm>
#include <cstdint>

namespace rbf {

inline int partition_point_index_dims_from_env() {
	return detail::env_int_or_default("RBF_PARTITION_POINT_INDEX_DIMS", 3);
}

inline std::uint64_t partition_point_index_max_cell_entries_from_env() {
	return static_cast<std::uint64_t>(std::max(
		1,
		detail::env_int_or_default("RBF_PARTITION_POINT_INDEX_MAX_CELL_ENTRIES", 128)));
}

inline std::uint64_t partition_point_index_max_query_entries_from_env() {
	return static_cast<std::uint64_t>(std::max(
		1,
		detail::env_int_or_default("RBF_PARTITION_POINT_INDEX_MAX_QUERY_ENTRIES", 512)));
}

inline int partition_overlay_dsu_min_cells_from_env() {
	return std::max(
		0,
		detail::env_int_or_default("RBF_PARTITION_OVERLAY_DSU_MIN_CELLS", 1000));
}

}  // namespace rbf
