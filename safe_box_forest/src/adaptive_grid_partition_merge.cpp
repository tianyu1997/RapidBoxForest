#include <SBF/adaptive_grid_partition.h>

#include "adaptive_grid_partition_geometry.h"
#include "adaptive_grid_partition_keys.h"
#include "adaptive_grid_partition_options.h"

#include <algorithm>
#include <chrono>
#include <iterator>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

using partition_detail::box_volume_from_intervals;

AdaptiveGridPartitionMergeResult AdaptiveGridPartition::merge_boxes(
	std::vector<BoxNode>& boxes,
	const AdaptiveGridPartitionMergeOptions& options,
	double tolerance) {
	using Clock = std::chrono::steady_clock;
	const auto start = Clock::now();
	AdaptiveGridPartitionMergeResult result;
	result.input_boxes = static_cast<int>(boxes.size());
	result.output_boxes = result.input_boxes;
	if (boxes.empty() || root_interval_copies_.empty() || target_depth_ <= 0) {
		return result;
	}
	const bool has_budget = options.max_ms > 0.0;
	const auto deadline = has_budget
		? start + std::chrono::duration_cast<Clock::duration>(
					  std::chrono::duration<double, std::milli>(options.max_ms))
		: Clock::time_point::max();
	auto timed_out = [&]() {
		return has_budget && Clock::now() >= deadline;
	};
	bool cells_need_index_rebuild = false;

	const auto containment_start = Clock::now();
	if (options.containment_prune && !timed_out()) {
		std::vector<unsigned char> remove(boxes.size(), 0);
		std::vector<int> selected_dims;
		std::vector<int> dim_order(static_cast<std::size_t>(std::max(0, static_cast<int>(split_counts_.size()))));
		std::iota(dim_order.begin(), dim_order.end(), 0);
		std::sort(dim_order.begin(), dim_order.end(), [&](int lhs, int rhs) {
			const int lhs_count = split_counts_[static_cast<std::size_t>(lhs)];
			const int rhs_count = split_counts_[static_cast<std::size_t>(rhs)];
			if (lhs_count != rhs_count) {
				return lhs_count > rhs_count;
			}
			return lhs < rhs;
		});
		for (int dim : dim_order) {
			if (split_counts_[static_cast<std::size_t>(dim)] > 0 &&
				static_cast<int>(selected_dims.size()) < 3) {
				selected_dims.push_back(dim);
			}
		}
		if (selected_dims.empty()) {
			result.containment_skipped = static_cast<int>(cells_.size());
		} else {
			const int bucket_bits = partition_containment_bucket_bits_from_env();
			const std::uint64_t max_bins_per_cell =
				partition_containment_max_bins_per_cell_from_env();
			auto coarse_coord = [&](const GridRange& range, int dim, bool upper) {
				const int split_count = split_counts_[static_cast<std::size_t>(dim)];
				const int shift = std::max(0, split_count - bucket_bits);
				const std::uint64_t coord = upper
					? (range.hi[static_cast<std::size_t>(dim)] == 0
						   ? 0
						   : ((range.hi[static_cast<std::size_t>(dim)] - 1u) >> shift))
					: (range.lo[static_cast<std::size_t>(dim)] >> shift);
				return coord;
			};
			auto make_key = [&](const GridRange& range,
								std::uint64_t coord0,
								std::uint64_t coord1,
								std::uint64_t coord2) {
				GridBroadphaseKey key;
				key.root_index = range.root_index;
				key.coord[0] = coord0;
				key.coord[1] = coord1;
				key.coord[2] = coord2;
				return key;
			};

			std::unordered_map<GridBroadphaseKey, std::vector<int>, GridBroadphaseKeyHash> buckets;
			std::unordered_map<int, std::vector<int>> overflow_by_root;
			buckets.reserve(cells_.size() * 2);
			for (int outer_index = 0; outer_index < static_cast<int>(cells_.size()); ++outer_index) {
				const auto& outer = cells_[static_cast<std::size_t>(outer_index)];
				if (!outer.grid_aligned ||
					outer.box_index >= boxes.size() ||
					outer.grid.lo.size() != split_counts_.size()) {
					result.containment_skipped += 1;
					continue;
				}
				const int dim0 = selected_dims[0];
				const int dim1 = selected_dims.size() > 1 ? selected_dims[1] : selected_dims[0];
				const int dim2 = selected_dims.size() > 2 ? selected_dims[2] : selected_dims[0];
				const std::uint64_t lo0 = coarse_coord(outer.grid, dim0, false);
				const std::uint64_t hi0 = coarse_coord(outer.grid, dim0, true);
				const std::uint64_t lo1 = coarse_coord(outer.grid, dim1, false);
				const std::uint64_t hi1 = coarse_coord(outer.grid, dim1, true);
				const std::uint64_t lo2 = coarse_coord(outer.grid, dim2, false);
				const std::uint64_t hi2 = coarse_coord(outer.grid, dim2, true);
				const std::uint64_t bin_count =
					(hi0 >= lo0 ? hi0 - lo0 + 1u : 0u) *
					(hi1 >= lo1 ? hi1 - lo1 + 1u : 0u) *
					(hi2 >= lo2 ? hi2 - lo2 + 1u : 0u);
				if (bin_count == 0u || bin_count > max_bins_per_cell) {
					overflow_by_root[outer.grid.root_index].push_back(outer_index);
					result.containment_overflow += 1;
					continue;
				}
				for (std::uint64_t coord0 = lo0; coord0 <= hi0; ++coord0) {
					for (std::uint64_t coord1 = lo1; coord1 <= hi1; ++coord1) {
						for (std::uint64_t coord2 = lo2; coord2 <= hi2; ++coord2) {
							buckets[make_key(outer.grid, coord0, coord1, coord2)].push_back(outer_index);
							result.containment_bucket_entries += 1;
						}
					}
				}
			}

			std::vector<int> candidates;
			std::unordered_set<int> seen;
			for (int inner_index = 0; inner_index < static_cast<int>(cells_.size()); ++inner_index) {
				if (timed_out()) {
					result.stop_reason = 2;
					break;
				}
				const auto& inner = cells_[static_cast<std::size_t>(inner_index)];
				if (!inner.grid_aligned ||
					inner.box_index >= boxes.size() ||
					remove[inner.box_index] != 0 ||
					inner.grid.lo.size() != split_counts_.size()) {
					continue;
				}
				candidates.clear();
				seen.clear();
				const int dim0 = selected_dims[0];
				const int dim1 = selected_dims.size() > 1 ? selected_dims[1] : selected_dims[0];
				const int dim2 = selected_dims.size() > 2 ? selected_dims[2] : selected_dims[0];
				const GridBroadphaseKey key =
					make_key(inner.grid,
							 coarse_coord(inner.grid, dim0, false),
							 coarse_coord(inner.grid, dim1, false),
							 coarse_coord(inner.grid, dim2, false));
				const auto bucket_it = buckets.find(key);
				if (bucket_it != buckets.end()) {
					for (int candidate : bucket_it->second) {
						if (seen.insert(candidate).second) {
							candidates.push_back(candidate);
						}
					}
				}
				const auto overflow_it = overflow_by_root.find(inner.grid.root_index);
				if (overflow_it != overflow_by_root.end()) {
					for (int candidate : overflow_it->second) {
						if (seen.insert(candidate).second) {
							candidates.push_back(candidate);
						}
					}
				}
				result.containment_candidates += static_cast<std::uint64_t>(candidates.size());
				for (int outer_index : candidates) {
					if (outer_index == inner_index) {
						continue;
					}
					const auto& outer = cells_[static_cast<std::size_t>(outer_index)];
					if (!outer.grid_aligned ||
						outer.box_index >= boxes.size() ||
						remove[outer.box_index] != 0 ||
						outer.box_id == inner.box_id) {
						continue;
					}
					const bool inner_contains_outer = grid_range_contains(inner.grid, outer.grid);
					if (inner_contains_outer && outer.box_index > inner.box_index) {
						continue;
					}
					if (!inner_contains_outer) {
						const double outer_volume = box_volume_from_intervals(outer.intervals);
						const double inner_volume = box_volume_from_intervals(inner.intervals);
						if (outer_volume <= inner_volume + 1e-18) {
							continue;
						}
					}
					result.containment_tests += 1;
					if (grid_range_contains(outer.grid, inner.grid)) {
						remove[inner.box_index] = 1;
						result.released_box_ids.push_back(inner.box_id);
						result.containment_pruned += 1;
						break;
					}
				}
			}
		}
		if (result.containment_pruned > 0) {
			std::vector<BoxNode> kept;
			kept.reserve(boxes.size() - static_cast<std::size_t>(result.containment_pruned));
			for (std::size_t index = 0; index < boxes.size(); ++index) {
				if (remove[index] == 0) {
					kept.push_back(std::move(boxes[index]));
				}
			}
			boxes = std::move(kept);
			rebuild_cells_from_boxes_only(boxes, tolerance);
			cells_need_index_rebuild = true;
		}
	}

	result.containment_ms = std::chrono::duration<double, std::milli>(
		Clock::now() - containment_start).count();

	const auto line_merge_start = Clock::now();
	if (options.grid_line_merge && result.stop_reason == 0) {
		const int max_rounds = std::max(1, options.max_rounds);
		for (int round = 0; round < max_rounds && !timed_out(); ++round) {
			bool changed_round = false;
			const int dims = root_intervals_.empty() ? 0 : static_cast<int>(root_intervals_.size());
			std::vector<unsigned char> remove(boxes.size(), 0);
			std::vector<BoxNode> additions;
			for (int merge_dim = 0; merge_dim < dims && !timed_out(); ++merge_dim) {
				std::unordered_map<GridMergeLineKey, std::vector<int>, GridMergeLineKeyHash> groups;
				groups.reserve(cells_.size() * 2);
				for (int cell_index = 0; cell_index < static_cast<int>(cells_.size()); ++cell_index) {
					const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
					if (cell.grid_aligned &&
						cell.box_index < boxes.size() &&
						remove[cell.box_index] == 0) {
						groups[make_grid_merge_line_key(cell.grid, merge_dim)].push_back(cell_index);
					}
				}
				for (auto& [_, members] : groups) {
					if (members.size() <= 1 || timed_out()) {
						continue;
					}
					std::sort(members.begin(), members.end(), [&](int lhs, int rhs) {
						return cells_[static_cast<std::size_t>(lhs)].grid.lo[static_cast<std::size_t>(merge_dim)] <
							   cells_[static_cast<std::size_t>(rhs)].grid.lo[static_cast<std::size_t>(merge_dim)];
					});
					std::size_t run_begin = 0;
					while (run_begin < members.size()) {
						std::size_t run_end = run_begin + 1;
						std::uint64_t current_hi =
							cells_[static_cast<std::size_t>(members[run_begin])].grid.hi[static_cast<std::size_t>(merge_dim)];
						while (run_end < members.size()) {
							const auto& next =
								cells_[static_cast<std::size_t>(members[run_end])].grid;
							const auto next_lo = next.lo[static_cast<std::size_t>(merge_dim)];
							if (next_lo > current_hi) {
								break;
							}
							current_hi = std::max(current_hi, next.hi[static_cast<std::size_t>(merge_dim)]);
							++run_end;
						}
						if (run_end - run_begin > 1) {
							const auto& first_cell = cells_[static_cast<std::size_t>(members[run_begin])];
							if (first_cell.box_index < boxes.size() &&
								remove[first_cell.box_index] == 0) {
								BoxNode merged = boxes[first_cell.box_index];
								bool merged_any = false;
								for (std::size_t pos = run_begin + 1; pos < run_end; ++pos) {
									const auto& next_cell = cells_[static_cast<std::size_t>(members[pos])];
									if (next_cell.box_index >= boxes.size() ||
										remove[next_cell.box_index] != 0) {
										continue;
									}
									const BoxNode& next_box = boxes[next_cell.box_index];
									for (std::size_t dim = 0; dim < merged.joint_intervals.size(); ++dim) {
										merged.joint_intervals[dim] =
											merged.joint_intervals[dim].hull(next_box.joint_intervals[dim]);
									}
									remove[next_cell.box_index] = 1;
									result.released_box_ids.push_back(next_box.id);
									result.grid_merges += 1;
									merged_any = true;
								}
								if (merged_any) {
									remove[first_cell.box_index] = 1;
									merged.tree_id = -1;
									merged.parent_box_id = -1;
									merged.root_id = merged.id;
									merged.seed_config = merged.center();
									merged.compute_volume();
									additions.push_back(std::move(merged));
									changed_round = true;
								}
							}
						}
						run_begin = run_end;
					}
				}
			}
			if (changed_round) {
				std::vector<BoxNode> kept;
				kept.reserve(boxes.size() + additions.size());
				for (std::size_t index = 0; index < boxes.size(); ++index) {
					if (remove[index] == 0) {
						kept.push_back(std::move(boxes[index]));
					}
				}
				kept.insert(kept.end(),
							std::make_move_iterator(additions.begin()),
							std::make_move_iterator(additions.end()));
				boxes = std::move(kept);
				rebuild_cells_from_boxes_only(boxes, tolerance);
				cells_need_index_rebuild = true;
			}
			result.rounds += 1;
			if (!changed_round) {
				break;
			}
		}
		if (timed_out()) {
			result.stop_reason = 2;
		}
	}
	result.line_merge_ms = std::chrono::duration<double, std::milli>(
		Clock::now() - line_merge_start).count();
	if (cells_need_index_rebuild) {
		rebuild_indices();
	}
	result.output_boxes = static_cast<int>(boxes.size());
	result.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
	return result;
}

}  // namespace rbf
