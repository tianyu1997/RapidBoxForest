#pragma once

#include <algorithm>
#include <cstdint>
#include <cstddef>

namespace rbf {

inline int partition_point_index_dims() {
	return 3;
}

inline std::uint64_t partition_point_index_max_cell_entries() {
	return 128;
}

inline std::uint64_t partition_point_index_max_query_entries() {
	return 512;
}

inline int partition_overlay_dsu_min_cells() {
	return 1000;
}

inline int partition_indexed_adjacency_threshold() {
	return 128;
}

inline int partition_adjacency_dim_limit(int dims) {
	return std::clamp(4, 1, std::min(7, std::max(1, dims)));
}

inline bool partition_cross_root_adjacency_enabled() {
	return true;
}

inline std::uint64_t partition_broadphase_max_bins_per_cell() {
	return 4096;
}

inline int partition_adjacency_bucket_bits() {
	return 3;
}

inline int partition_max_neighbors_per_cell(std::size_t) {
	return 0;
}

inline std::uint64_t partition_component_pair_exact_cap() {
	return 200000;
}

inline bool partition_query_component_prune_enabled() {
	return false;
}

inline int partition_containment_bucket_bits() {
	return 10;
}

inline std::uint64_t partition_containment_max_bins_per_cell() {
	return 256;
}

inline int partition_segment_fallback_pair_candidate_cap() {
	return 128;
}

}  // namespace rbf
