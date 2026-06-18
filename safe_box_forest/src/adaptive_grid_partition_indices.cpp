#include <SBF/adaptive_grid_partition.h>

#include "adaptive_grid_partition_geometry.h"
#include "adaptive_grid_partition_options.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

using partition_detail::interval_bin;

std::size_t AdaptiveGridPartition::FaceKeyHash::operator()(const FaceKey& key) const noexcept {
	std::uint64_t hash = 1469598103934665603ull;
	auto mix = [&](std::uint64_t value) {
		hash ^= value;
		hash *= 1099511628211ull;
	};
	mix(static_cast<std::uint64_t>(key.root_index + 1));
	mix(static_cast<std::uint64_t>(key.dim + 1));
	mix(key.coord);
	mix(key.upper ? 1ull : 0ull);
	return static_cast<std::size_t>(hash);
}

std::size_t AdaptiveGridPartition::PointBinKeyHash::operator()(const PointBinKey& key) const noexcept {
	std::uint64_t hash = 1469598103934665603ull;
	auto mix = [&](std::uint64_t value) {
		hash ^= value;
		hash *= 1099511628211ull;
	};
	for (long long bin : key.bins) {
		mix(static_cast<std::uint64_t>(bin) + 0x9e3779b97f4a7c15ull);
	}
	return static_cast<std::size_t>(hash);
}

std::size_t AdaptiveGridPartition::SparseCellKeyHash::operator()(const SparseCellKey& key) const noexcept {
	std::uint64_t hash = 1469598103934665603ull;
	auto mix = [&](std::uint64_t value) {
		hash ^= value;
		hash *= 1099511628211ull;
	};
	mix(static_cast<std::uint64_t>(key.root_index + 1));
	for (std::uint64_t value : key.lo) {
		mix(value + 0x9e3779b97f4a7c15ull);
	}
	mix(0x517cc1b727220a95ull);
	for (std::uint64_t value : key.hi) {
		mix(value + 0xc2b2ae3d27d4eb4full);
	}
	return static_cast<std::size_t>(hash);
}

void AdaptiveGridPartition::clear_runtime_indices() {
	cell_by_box_id_.clear();
	face_index_.clear();
	neighbor_cache_.clear();
	overlay_parent_.clear();
	point_bins_.clear();
	point_overflow_cells_.clear();
	sparse_virtual_index_.clear();
	point_index_dims_.clear();
	point_bin_widths_ = {1.0, 1.0, 1.0};
	point_bin_origins_ = {0.0, 0.0, 0.0};
}

void AdaptiveGridPartition::rebuild_indices() {
	const auto t0 = std::chrono::steady_clock::now();
	clear_runtime_indices();
	for (int index = 0; index < static_cast<int>(cells_.size()); ++index) {
		cells_[static_cast<std::size_t>(index)].cell_id = index;
		cell_by_box_id_[cells_[static_cast<std::size_t>(index)].box_id] = index;
	}
	const auto face_t0 = std::chrono::steady_clock::now();
	rebuild_face_index();
	stats_.face_index_ms +=
		std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - face_t0).count();
	const auto point_t0 = std::chrono::steady_clock::now();
	rebuild_point_index();
	stats_.point_index_ms +=
		std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - point_t0).count();
	const auto sparse_t0 = std::chrono::steady_clock::now();
	rebuild_sparse_virtual_index();
	stats_.sparse_virtual_index_ms +=
		std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sparse_t0).count();
	const auto neighbor_t0 = std::chrono::steady_clock::now();
	rebuild_neighbor_cache();
	stats_.neighbor_cache_ms +=
		std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - neighbor_t0).count();
	const auto island_t0 = std::chrono::steady_clock::now();
	rebuild_islands();
	stats_.island_rebuild_ms +=
		std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - island_t0).count();
	reset_overlay_components();
	stats_.index_rebuild_ms +=
		std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void AdaptiveGridPartition::append_cell_to_indices(int cell_index) {
	if (cell_index < 0 || cell_index >= static_cast<int>(cells_.size())) {
		return;
	}
	auto& cell = cells_[static_cast<std::size_t>(cell_index)];
	cell.cell_id = cell_index;
	cell_by_box_id_[cell.box_id] = cell_index;
	stats_.cells = static_cast<int>(cells_.size());
	append_sparse_virtual_cell(cell_index);
	if (cell.grid_aligned) {
		stats_.grid_cells += 1;
		for (int dim = 0; dim < static_cast<int>(cell.grid.lo.size()); ++dim) {
			face_index_[FaceKey{cell.grid.root_index, dim, cell.grid.lo[static_cast<std::size_t>(dim)], false}].push_back(cell.cell_id);
			face_index_[FaceKey{cell.grid.root_index, dim, cell.grid.hi[static_cast<std::size_t>(dim)], true}].push_back(cell.cell_id);
		}
		stats_.face_index_entries = static_cast<int>(face_index_.size());
	} else {
		stats_.non_grid_cells += 1;
	}
	if (!point_index_dims_.empty()) {
		bool valid = true;
		std::array<long long, 3> lo_bins{0, 0, 0};
		std::array<long long, 3> hi_bins{0, 0, 0};
		std::uint64_t entry_count = 1;
		for (std::size_t item = 0; item < point_index_dims_.size(); ++item) {
			const int dim = point_index_dims_[item];
			if (dim < 0 || dim >= static_cast<int>(cell.intervals.size())) {
				valid = false;
				break;
			}
			const auto& iv = cell.intervals[static_cast<std::size_t>(dim)];
			lo_bins[item] = interval_bin(iv.lo, point_bin_origins_[item], point_bin_widths_[item]);
			hi_bins[item] = interval_bin(iv.hi, point_bin_origins_[item], point_bin_widths_[item]);
			if (hi_bins[item] < lo_bins[item]) {
				valid = false;
				break;
			}
			entry_count *= static_cast<std::uint64_t>(hi_bins[item] - lo_bins[item] + 1);
		}
		const std::uint64_t max_entries = partition_point_index_max_cell_entries_from_env();
		if (!valid || entry_count > max_entries) {
			point_overflow_cells_.push_back(cell.cell_id);
		} else {
			for (long long b0 = lo_bins[0]; b0 <= hi_bins[0]; ++b0) {
				const long long b1_lo = point_index_dims_.size() > 1 ? lo_bins[1] : 0;
				const long long b1_hi = point_index_dims_.size() > 1 ? hi_bins[1] : 0;
				for (long long b1 = b1_lo; b1 <= b1_hi; ++b1) {
					const long long b2_lo = point_index_dims_.size() > 2 ? lo_bins[2] : 0;
					const long long b2_hi = point_index_dims_.size() > 2 ? hi_bins[2] : 0;
					for (long long b2 = b2_lo; b2 <= b2_hi; ++b2) {
						point_bins_[PointBinKey{{b0, b1, b2}}].push_back(cell.cell_id);
					}
				}
			}
		}
	}
	stats_.point_index_dims = static_cast<int>(point_index_dims_.size());
	stats_.point_index_entries = static_cast<int>(point_bins_.size());
	stats_.point_index_overflow_cells = static_cast<int>(point_overflow_cells_.size());
	if (neighbor_cache_.size() < cells_.size()) {
		neighbor_cache_.resize(cells_.size());
	}
	std::vector<int> neighbors = compute_neighbor_cell_indices(cell_index);
	neighbor_cache_[static_cast<std::size_t>(cell_index)] = neighbors;
	for (int neighbor : neighbors) {
		if (neighbor < 0 || neighbor >= static_cast<int>(neighbor_cache_.size())) {
			continue;
		}
		auto& list = neighbor_cache_[static_cast<std::size_t>(neighbor)];
		if (std::find(list.begin(), list.end(), cell_index) == list.end()) {
			list.push_back(cell_index);
		}
	}
	stats_.adjacency_candidates += static_cast<std::uint64_t>(neighbors.size());
	stats_.adjacency_tests += static_cast<std::uint64_t>(neighbors.size());
	stats_.adjacency_edges += static_cast<std::uint64_t>(neighbors.size());
	update_islands_after_incremental_cell(cell_index, neighbors);
	ensure_overlay_parent_size();
	for (int neighbor : neighbors) {
		union_overlay_components(cell_index, neighbor);
	}
}

void AdaptiveGridPartition::update_islands_after_incremental_cell(
	int cell_index,
	const std::vector<int>& neighbors) {
	if (cell_index < 0 || cell_index >= static_cast<int>(cells_.size())) {
		return;
	}
	std::unordered_set<int> neighbor_islands;
	for (int neighbor : neighbors) {
		if (neighbor < 0 || neighbor >= static_cast<int>(cells_.size())) {
			continue;
		}
		const int island = cells_[static_cast<std::size_t>(neighbor)].island_id;
		if (island >= 0) {
			neighbor_islands.insert(island);
		}
	}
	int target_island = -1;
	if (neighbor_islands.empty()) {
		target_island = 0;
		for (const auto& cell : cells_) {
			target_island = std::max(target_island, cell.island_id + 1);
		}
		stats_.islands += 1;
	} else {
		target_island = *neighbor_islands.begin();
	}
	cells_[static_cast<std::size_t>(cell_index)].island_id = target_island;
	if (neighbor_islands.size() > 1) {
		for (auto& cell : cells_) {
			if (neighbor_islands.find(cell.island_id) != neighbor_islands.end()) {
				cell.island_id = target_island;
			}
		}
		stats_.islands -= static_cast<int>(neighbor_islands.size() - 1);
	}
	std::unordered_map<int, int> sizes;
	for (const auto& cell : cells_) {
		if (cell.island_id >= 0) {
			sizes[cell.island_id] += 1;
		}
	}
	stats_.largest_island = 0;
	for (const auto& [_, size] : sizes) {
		stats_.largest_island = std::max(stats_.largest_island, size);
	}
}

}  // namespace rbf
