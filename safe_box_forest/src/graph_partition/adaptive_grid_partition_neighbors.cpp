#include <SBF/adaptive_grid_partition.h>

#include "adaptive_grid_partition_geometry.h"
#include "adaptive_grid_partition_keys.h"
#include "adaptive_grid_partition_options.h"

#include <algorithm>
#include <array>
#include <functional>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

using partition_detail::interval_boxes_connected;
using partition_detail::packed_pair_key;

std::vector<int> AdaptiveGridPartition::compute_neighbor_cell_indices(int cell_index) const {
	std::vector<int> result;
	if (cell_index < 0 || cell_index >= static_cast<int>(cells_.size())) {
		return result;
	}
	const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
	std::unordered_set<int> seen;
	if (cell.grid_aligned) {
		for (int dim = 0; dim < static_cast<int>(cell.grid.lo.size()); ++dim) {
			const FaceKey lower_query{cell.grid.root_index, dim, cell.grid.lo[static_cast<std::size_t>(dim)], true};
			const FaceKey upper_query{cell.grid.root_index, dim, cell.grid.hi[static_cast<std::size_t>(dim)], false};
			for (const FaceKey& key : {lower_query, upper_query}) {
				const auto it = face_index_.find(key);
				if (it == face_index_.end()) {
					continue;
				}
				for (int candidate : it->second) {
					if (candidate == cell_index || !seen.insert(candidate).second) {
						continue;
					}
					const auto& other = cells_[static_cast<std::size_t>(candidate)];
					if (other.grid_aligned &&
						grid_ranges_overlap_except_dim(cell.grid, other.grid, dim) &&
						grid_ranges_adjacent(cell.grid, other.grid)) {
						result.push_back(candidate);
					}
				}
			}
		}
	}
	const std::vector<int> exact_candidates = interval_candidate_cells(cell.intervals);
	for (int candidate_id : exact_candidates) {
		if (candidate_id == cell_index ||
			candidate_id < 0 ||
			candidate_id >= static_cast<int>(cells_.size())) {
			continue;
		}
		const auto& other = cells_[static_cast<std::size_t>(candidate_id)];
		const bool needs_exact =
			!cell.grid_aligned ||
			!other.grid_aligned ||
			cell.grid.root_index != other.grid.root_index;
		if (!needs_exact) {
			continue;
		}
		if (seen.insert(other.cell_id).second &&
			interval_boxes_connected(cell.intervals, other.intervals, tolerance_)) {
			result.push_back(other.cell_id);
		}
	}
	return result;
}

void AdaptiveGridPartition::rebuild_neighbor_cache() {
	neighbor_cache_.clear();
	neighbor_cache_.resize(cells_.size());
	if (cells_.empty()) {
		return;
	}
	const int indexed_threshold = partition_indexed_adjacency_threshold();
	if (static_cast<int>(cells_.size()) < indexed_threshold) {
		for (int cell_index = 0; cell_index < static_cast<int>(cells_.size()); ++cell_index) {
			neighbor_cache_[static_cast<std::size_t>(cell_index)] =
				compute_neighbor_cell_indices(cell_index);
		}
		return;
	}
	const int dims = static_cast<int>(split_counts_.size());
	const int max_adjacency_dims = partition_adjacency_dim_limit(dims);
	std::vector<int> selected_dims;
	selected_dims.reserve(static_cast<std::size_t>(max_adjacency_dims));
	std::vector<int> order(static_cast<std::size_t>(dims));
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
		const int lhs_count = split_counts_[static_cast<std::size_t>(lhs)];
		const int rhs_count = split_counts_[static_cast<std::size_t>(rhs)];
		if (lhs_count != rhs_count) {
			return lhs_count > rhs_count;
		}
		return lhs < rhs;
	});
	for (int dim : order) {
		if (dim >= 0 &&
			dim < dims &&
			split_counts_[static_cast<std::size_t>(dim)] > 0 &&
			static_cast<int>(selected_dims.size()) < max_adjacency_dims) {
			selected_dims.push_back(dim);
		}
	}
	if (selected_dims.empty()) {
		for (int cell_index = 0; cell_index < static_cast<int>(cells_.size()); ++cell_index) {
			neighbor_cache_[static_cast<std::size_t>(cell_index)] =
				compute_neighbor_cell_indices(cell_index);
		}
		return;
	}

	const bool enable_cross_root_adjacency = partition_cross_root_adjacency_enabled();
	const std::uint64_t max_bins_per_cell = partition_broadphase_max_bins_per_cell();
	const int adjacency_bucket_bits = partition_adjacency_bucket_bits();
	auto coarse_adjacency_coord = [&](const GridRange& range, int dim, bool upper) {
		const int split_count = split_counts_[static_cast<std::size_t>(dim)];
		const int shift = std::max(0, split_count - adjacency_bucket_bits);
		const auto coord = upper
			? range.hi[static_cast<std::size_t>(dim)]
			: range.lo[static_cast<std::size_t>(dim)];
		return coord >> shift;
	};
	std::unordered_map<GridAdjacencyKey, std::vector<int>, GridAdjacencyKeyHash> buckets;
	buckets.reserve(cells_.size() * 4);
	auto add_cell_to_adjacency_buckets =
		[&](int cell_index,
			const std::vector<int>& dims_for_key,
			int root_index,
			std::unordered_map<GridAdjacencyKey, std::vector<int>, GridAdjacencyKeyHash>& target) {
			const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
			if (dims_for_key.empty() ||
				!cell.grid_aligned ||
				!cell.grid.valid() ||
				dims_for_key.size() > GridAdjacencyKey{}.coord.size()) {
				return false;
			}
			std::array<std::uint64_t, 7> lo{0, 0, 0, 0, 0, 0, 0};
			std::array<std::uint64_t, 7> hi{0, 0, 0, 0, 0, 0, 0};
			std::uint64_t product = 1;
			for (std::size_t pos = 0; pos < dims_for_key.size(); ++pos) {
				const int dim = dims_for_key[pos];
				if (dim < 0 || dim >= static_cast<int>(cell.grid.lo.size())) {
					return false;
				}
				lo[pos] = coarse_adjacency_coord(cell.grid, dim, false);
				hi[pos] = coarse_adjacency_coord(cell.grid, dim, true);
				if (hi[pos] < lo[pos]) {
					return false;
				}
				const std::uint64_t count = hi[pos] - lo[pos] + 1u;
				if (count > 0 && product > max_bins_per_cell / count) {
					return false;
				}
				product *= std::max<std::uint64_t>(1, count);
			}
			GridAdjacencyKey key;
			key.root_index = root_index;
			std::function<void(std::size_t)> visit = [&](std::size_t pos) {
				if (pos >= dims_for_key.size()) {
					target[key].push_back(cell_index);
					return;
				}
				for (std::uint64_t coord = lo[pos]; coord <= hi[pos]; ++coord) {
					key.coord[pos] = coord;
					visit(pos + 1);
				}
				key.coord[pos] = 0;
			};
			visit(0);
			return true;
		};
	std::vector<int> exact_fallback_cells;
	for (int cell_index = 0; cell_index < static_cast<int>(cells_.size()); ++cell_index) {
		const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
		if (!cell.grid_aligned || !cell.grid.valid()) {
			exact_fallback_cells.push_back(cell_index);
			continue;
		}
		if (!add_cell_to_adjacency_buckets(cell_index,
										   selected_dims,
										   cell.grid.root_index,
										   buckets)) {
			exact_fallback_cells.push_back(cell_index);
		}
	}

	std::unordered_set<std::uint64_t> tested_pairs;
	tested_pairs.reserve(cells_.size() * 8);
	const int max_neighbors_per_cell = partition_max_neighbors_per_cell(cells_.size());
	auto add_edge_if_adjacent = [&](int lhs, int rhs) {
		if (lhs == rhs ||
			lhs < 0 || rhs < 0 ||
			lhs >= static_cast<int>(cells_.size()) ||
			rhs >= static_cast<int>(cells_.size())) {
			return;
		}
		if (max_neighbors_per_cell > 0 &&
			(static_cast<int>(neighbor_cache_[static_cast<std::size_t>(lhs)].size()) >= max_neighbors_per_cell ||
			 static_cast<int>(neighbor_cache_[static_cast<std::size_t>(rhs)].size()) >= max_neighbors_per_cell)) {
			return;
		}
		const std::uint64_t pair_key = packed_pair_key(lhs, rhs);
		if (!tested_pairs.insert(pair_key).second) {
			return;
		}
		const auto& lhs_cell = cells_[static_cast<std::size_t>(lhs)];
		const auto& rhs_cell = cells_[static_cast<std::size_t>(rhs)];
		bool adjacent = false;
		if (lhs_cell.grid_aligned &&
			rhs_cell.grid_aligned &&
			lhs_cell.grid.root_index == rhs_cell.grid.root_index) {
			adjacent = grid_ranges_adjacent(lhs_cell.grid, rhs_cell.grid);
		} else {
			adjacent = interval_boxes_connected(lhs_cell.intervals,
												rhs_cell.intervals,
												tolerance_);
		}
		if (!adjacent) {
			return;
		}
		if (max_neighbors_per_cell > 0 &&
			(static_cast<int>(neighbor_cache_[static_cast<std::size_t>(lhs)].size()) >= max_neighbors_per_cell ||
			 static_cast<int>(neighbor_cache_[static_cast<std::size_t>(rhs)].size()) >= max_neighbors_per_cell)) {
			return;
		}
		neighbor_cache_[static_cast<std::size_t>(lhs)].push_back(rhs);
		neighbor_cache_[static_cast<std::size_t>(rhs)].push_back(lhs);
	};

	for (const auto& [_, members] : buckets) {
		for (std::size_t i = 0; i < members.size(); ++i) {
			for (std::size_t j = i + 1; j < members.size(); ++j) {
				add_edge_if_adjacent(members[i], members[j]);
			}
		}
	}
	if (enable_cross_root_adjacency && root_interval_copies_.size() > 1) {
		std::vector<int> cross_dims;
		cross_dims.reserve(3);
		for (int dim : order) {
			if (dim > 0 &&
				dim < dims &&
				split_counts_[static_cast<std::size_t>(dim)] > 0 &&
				static_cast<int>(cross_dims.size()) < 3) {
				cross_dims.push_back(dim);
			}
		}
		if (!cross_dims.empty()) {
			std::unordered_map<GridBroadphaseKey, std::vector<int>, GridBroadphaseKeyHash> cross_buckets;
			cross_buckets.reserve(cells_.size() * 4);
			for (int cell_index = 0; cell_index < static_cast<int>(cells_.size()); ++cell_index) {
				const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
				if (!cell.grid_aligned || !cell.grid.valid()) {
					continue;
				}
				std::array<std::uint64_t, 3> lo{0, 0, 0};
				std::array<std::uint64_t, 3> hi{0, 0, 0};
				std::uint64_t product = 1;
				bool usable = true;
				for (std::size_t pos = 0; pos < cross_dims.size(); ++pos) {
					const int dim = cross_dims[pos];
					lo[pos] = coarse_adjacency_coord(cell.grid, dim, false);
					hi[pos] = coarse_adjacency_coord(cell.grid, dim, true);
					if (hi[pos] < lo[pos]) {
						usable = false;
						break;
					}
					const std::uint64_t count = hi[pos] - lo[pos] + 1u;
					if (count > 0 && product > max_bins_per_cell / count) {
						usable = false;
						break;
					}
					product *= std::max<std::uint64_t>(1, count);
				}
				if (!usable) {
					continue;
				}
				for (std::uint64_t c0 = lo[0]; c0 <= hi[0]; ++c0) {
					const std::uint64_t end1 = cross_dims.size() >= 2 ? hi[1] : lo[1];
					for (std::uint64_t c1 = lo[1]; c1 <= end1; ++c1) {
						const std::uint64_t end2 = cross_dims.size() >= 3 ? hi[2] : lo[2];
						for (std::uint64_t c2 = lo[2]; c2 <= end2; ++c2) {
							GridBroadphaseKey key;
							key.root_index = -1;
							key.coord = {c0, c1, c2};
							cross_buckets[key].push_back(cell_index);
						}
					}
				}
			}
			for (const auto& [_, members] : cross_buckets) {
				for (std::size_t i = 0; i < members.size(); ++i) {
					const auto& lhs_cell = cells_[static_cast<std::size_t>(members[i])];
					for (std::size_t j = i + 1; j < members.size(); ++j) {
						const auto& rhs_cell = cells_[static_cast<std::size_t>(members[j])];
						if (lhs_cell.grid.root_index == rhs_cell.grid.root_index) {
							continue;
						}
						add_edge_if_adjacent(members[i], members[j]);
					}
				}
			}
		}
	}
	for (int cell_index : exact_fallback_cells) {
		const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
		const std::vector<int> candidates = interval_candidate_cells(cell.intervals);
		for (int other : candidates) {
			add_edge_if_adjacent(cell_index, other);
		}
	}
}

std::vector<int> AdaptiveGridPartition::neighbor_cell_indices(int cell_index) const {
	if (cell_index >= 0 &&
		cell_index < static_cast<int>(neighbor_cache_.size())) {
		return neighbor_cache_[static_cast<std::size_t>(cell_index)];
	}
	return compute_neighbor_cell_indices(cell_index);
}

void AdaptiveGridPartition::rebuild_islands() {
	stats_.islands = 0;
	stats_.largest_island = 0;
	stats_.adjacency_candidates = 0;
	stats_.adjacency_tests = 0;
	stats_.adjacency_edges = 0;
	for (auto& cell : cells_) {
		cell.island_id = -1;
	}
	for (auto& root : cells_) {
		if (root.island_id >= 0) {
			continue;
		}
		const int island = stats_.islands++;
		int size = 0;
		std::queue<int> queue;
		root.island_id = island;
		queue.push(root.cell_id);
		while (!queue.empty()) {
			const int current = queue.front();
			queue.pop();
			++size;
			const auto neighbors = neighbor_cell_indices(current);
			stats_.adjacency_candidates += static_cast<std::uint64_t>(neighbors.size());
			for (int next : neighbors) {
				stats_.adjacency_tests += 1;
				stats_.adjacency_edges += 1;
				auto& next_cell = cells_[static_cast<std::size_t>(next)];
				if (next_cell.island_id >= 0) {
					continue;
				}
				next_cell.island_id = island;
				queue.push(next);
			}
		}
		stats_.largest_island = std::max(stats_.largest_island, size);
	}
}

}  // namespace rbf
