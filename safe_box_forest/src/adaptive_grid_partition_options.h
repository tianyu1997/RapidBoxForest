#pragma once

#include "env_config.h"

#include <algorithm>
#include <cstdint>
#include <cstddef>

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

inline int partition_indexed_adjacency_threshold_from_env() {
	return std::max(
		0,
		detail::env_int_or_default("RBF_PARTITION_INDEXED_ADJACENCY_THRESHOLD", 128));
}

inline int partition_adjacency_dim_limit_from_env(int dims) {
	return std::clamp(
		detail::env_int_or_default("RBF_PARTITION_ADJACENCY_DIMS", 4),
		1,
		std::min(7, std::max(1, dims)));
}

inline bool partition_cross_root_adjacency_enabled_from_env() {
	return detail::env_int_or_default("RBF_PARTITION_ENABLE_CROSS_ROOT_ADJACENCY", 1) != 0;
}

inline std::uint64_t partition_broadphase_max_bins_per_cell_from_env() {
	return static_cast<std::uint64_t>(std::max(
		64,
		detail::env_int_or_default("RBF_PARTITION_BROADPHASE_MAX_BINS_PER_CELL", 4096)));
}

inline int partition_adjacency_bucket_bits_from_env() {
	return std::clamp(
		detail::env_int_or_default("RBF_PARTITION_ADJACENCY_BUCKET_BITS", 3),
		1,
		20);
}

inline int partition_max_neighbors_per_cell_from_env(std::size_t cell_count) {
	return static_cast<int>(cell_count) >=
			detail::env_int_or_default("RBF_PARTITION_SPARSE_NEIGHBOR_THRESHOLD", 5000)
		? std::max(
			  0,
			  detail::env_int_or_default("RBF_PARTITION_MAX_NEIGHBORS_PER_CELL", 0))
		: 0;
}

inline std::uint64_t partition_component_pair_exact_cap_from_env() {
	return static_cast<std::uint64_t>(std::max(
		1,
		detail::env_int_or_default("RBF_PARTITION_COMPONENT_PAIR_EXACT_CAP", 200000)));
}

inline bool partition_query_component_prune_enabled_from_env() {
	return detail::env_int_or_default("RBF_PARTITION_QUERY_COMPONENT_PRUNE", 0) != 0;
}

inline int partition_containment_bucket_bits_from_env() {
	return std::clamp(
		detail::env_int_or_default("RBF_PARTITION_CONTAINMENT_BUCKET_BITS", 10),
		1,
		20);
}

inline std::uint64_t partition_containment_max_bins_per_cell_from_env() {
	return static_cast<std::uint64_t>(std::max(
		16,
		detail::env_int_or_default("RBF_PARTITION_CONTAINMENT_MAX_BINS_PER_CELL", 256)));
}

inline int partition_segment_fallback_pair_candidate_cap_from_env() {
	return std::max(
		8,
		detail::env_int_or_default("RBF_PARTITION_SEGMENT_FALLBACK_PAIR_CANDIDATE_CAP", 128));
}

}  // namespace rbf
