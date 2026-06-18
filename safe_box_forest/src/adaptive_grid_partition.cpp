#include <SBF/adaptive_grid_partition.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_set>

#include "adaptive_grid_partition_geometry.h"
#include "adaptive_grid_partition_keys.h"
#include "adaptive_grid_partition_options.h"
#include "env_config.h"
#include "query_graph_cost_options.h"

namespace rbf {

namespace {

using detail::env_double_or_default;
using detail::env_int_or_default;
using partition_detail::box_center;
using partition_detail::box_volume_from_intervals;
using partition_detail::choose_bin_width;
using partition_detail::choose_point_index_dims;
using partition_detail::closest_cells_to_hull;
using partition_detail::closest_point_in_intervals;
using partition_detail::closest_points_between_interval_boxes;
using partition_detail::distance_to_line;
using partition_detail::interval_bin;
using partition_detail::interval_box_distance_sq;
using partition_detail::interval_box_subset;
using partition_detail::interval_boxes_connected;
using partition_detail::interval_hull_for_cells;
using partition_detail::intervals_contain_point;
using partition_detail::max_coord_for_splits;
using partition_detail::packed_pair_key;
using partition_detail::partition_counts_as_query_repair_edge;
using partition_detail::partition_counts_as_segment_edge;
using partition_detail::point_to_box_distance_sq;
using partition_detail::shared_face_center;
using partition_detail::transition_waypoint_toward_goal;

}  // namespace

void CellPath::push_child(bool right_child) {
	const int word_index = bit_count / 64;
	const int bit_index = bit_count % 64;
	if (word_index >= static_cast<int>(words.size())) {
		words.push_back(0);
	}
	if (right_child) {
		words[static_cast<std::size_t>(word_index)] |= (std::uint64_t{1} << bit_index);
	}
	++bit_count;
}

bool CellPath::bit(int index) const {
	if (index < 0 || index >= bit_count) {
		return false;
	}
	const int word_index = index / 64;
	const int bit_index = index % 64;
	return (words[static_cast<std::size_t>(word_index)] & (std::uint64_t{1} << bit_index)) != 0;
}

bool GridRange::valid() const noexcept {
	if (lo.size() != hi.size() || lo.empty()) {
		return false;
	}
	for (std::size_t dim = 0; dim < lo.size(); ++dim) {
		if (lo[dim] >= hi[dim]) {
			return false;
		}
	}
	return true;
}

void AdaptiveGridPartition::clear() {
	root_intervals_.clear();
	root_interval_copies_.clear();
	split_counts_.clear();
	cells_.clear();
	clear_runtime_indices();
	clear_overlay_edges();
	tolerance_ = 1e-9;
	stats_ = {};
}

bool AdaptiveGridPartition::rebuild(const std::vector<Interval>& root_intervals,
									const lect_database::SplitPolicyDescriptor& split_policy,
									int root_depth,
									int target_depth,
									const std::vector<BoxNode>& boxes,
									double tolerance) {
	return rebuild(std::vector<std::vector<Interval>>{root_intervals},
				   split_policy,
				   root_depth,
				   target_depth,
				   boxes,
				   tolerance);
}

bool AdaptiveGridPartition::rebuild(const std::vector<std::vector<Interval>>& root_interval_copies,
									const lect_database::SplitPolicyDescriptor& split_policy,
									int root_depth,
									int target_depth,
									const std::vector<BoxNode>& boxes,
									double tolerance) {
	using Clock = std::chrono::steady_clock;
	const auto t0 = Clock::now();
	clear();
	if (root_interval_copies.empty() || root_interval_copies.front().empty() || target_depth <= 0) {
		return false;
	}
	root_interval_copies_ = root_interval_copies;
	root_intervals_ = root_interval_copies_.front();
	for (std::size_t copy = 1; copy < root_interval_copies_.size(); ++copy) {
		if (root_interval_copies_[copy].size() != root_intervals_.size()) {
			return false;
		}
		for (std::size_t dim = 0; dim < root_intervals_.size(); ++dim) {
			root_intervals_[dim] = root_intervals_[dim].hull(root_interval_copies_[copy][dim]);
		}
	}
	split_policy_ = split_policy;
	root_depth_ = std::max(0, root_depth);
	target_depth_ = target_depth;
	tolerance_ = tolerance;
	const int dims = static_cast<int>(root_intervals_.size());
	split_counts_.assign(static_cast<std::size_t>(dims), 0);
	lect_database::SplitPolicy policy(split_policy_);
	std::vector<Interval> scratch = root_interval_copies_.front();
	for (int depth = 0; depth < target_depth_; ++depth) {
		const int dim = policy.choose_dimension(root_interval_copies_.front(), scratch, root_depth_ + depth);
		if (dim < 0 || dim >= dims) {
			return false;
		}
		split_counts_[static_cast<std::size_t>(dim)] += 1;
	}
	for (int count : split_counts_) {
		(void)max_coord_for_splits(count);
	}

	cells_.reserve(boxes.size());
	for (std::size_t index = 0; index < boxes.size(); ++index) {
		const auto& box = boxes[index];
		if (box.joint_intervals.size() != root_intervals_.size()) {
			continue;
		}
		add_cell_from_box(box,
						  index,
						  box.tree_id >= 0 ? PartitionCellState::Free : PartitionCellState::FreeMerged,
						  tolerance);
	}
	rebuild_indices();
	stats_.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	return true;
}

bool AdaptiveGridPartition::add_cell_from_box(const BoxNode& box,
											  std::size_t box_index,
											  PartitionCellState state,
											  double tolerance) {
	if (box.id < 0 || box.joint_intervals.size() != root_intervals_.size()) {
		return false;
	}
	if (cell_by_box_id_.find(box.id) != cell_by_box_id_.end()) {
		return false;
	}
	PartitionCell cell;
	cell.cell_id = static_cast<int>(cells_.size());
	cell.box_id = box.id;
	cell.box_index = box_index;
	cell.intervals = box.joint_intervals;
	cell.state = state;
	cell.grid_aligned = make_grid_range(box.joint_intervals, cell.grid, tolerance);
	if (!cell.grid_aligned) {
		cell.state = PartitionCellState::NonGridFree;
	}
	cell_by_box_id_[cell.box_id] = cell.cell_id;
	cells_.push_back(std::move(cell));
	return true;
}

void AdaptiveGridPartition::rebuild_cells_from_boxes_only(const std::vector<BoxNode>& boxes,
														  double tolerance) {
	cells_.clear();
	clear_runtime_indices();
	clear_overlay_edges();
	tolerance_ = tolerance;
	cells_.reserve(boxes.size());
	for (std::size_t index = 0; index < boxes.size(); ++index) {
		const auto& box = boxes[index];
		if (box.joint_intervals.size() != root_intervals_.size()) {
			continue;
		}
		add_cell_from_box(box,
						  index,
						  box.tree_id >= 0 ? PartitionCellState::Free : PartitionCellState::FreeMerged,
						  tolerance);
	}
	stats_.cells = static_cast<int>(cells_.size());
	stats_.grid_cells = 0;
	stats_.non_grid_cells = 0;
	for (const auto& cell : cells_) {
		if (cell.grid_aligned) {
			stats_.grid_cells += 1;
		} else {
			stats_.non_grid_cells += 1;
		}
	}
}

bool AdaptiveGridPartition::append_box(const BoxNode& box, double tolerance) {
	if (root_interval_copies_.empty() || target_depth_ <= 0) {
		return false;
	}
	const int cell_index = static_cast<int>(cells_.size());
	const bool added = add_cell_from_box(box,
										 static_cast<std::size_t>(box.id >= 0 ? box.id : 0),
										 box.tree_id >= 0 ? PartitionCellState::Free : PartitionCellState::FreeMerged,
										 tolerance);
	if (!added) {
		return false;
	}
	tolerance_ = tolerance;
	append_cell_to_indices(cell_index);
	return true;
}

int AdaptiveGridPartition::append_boxes(const std::vector<BoxNode>& boxes,
										std::size_t first_index,
										double tolerance) {
	if (first_index >= boxes.size() || root_interval_copies_.empty() || target_depth_ <= 0) {
		return 0;
	}
	int added = 0;
	for (std::size_t index = first_index; index < boxes.size(); ++index) {
		const auto& box = boxes[index];
		const int cell_index = static_cast<int>(cells_.size());
		if (add_cell_from_box(box,
							  index,
							  box.tree_id >= 0 ? PartitionCellState::Free : PartitionCellState::FreeMerged,
							  tolerance)) {
			added += 1;
			tolerance_ = tolerance;
			append_cell_to_indices(cell_index);
		}
	}
	return added;
}

int AdaptiveGridPartition::remove_box_ids(const std::unordered_set<int>& box_ids) {
	if (box_ids.empty() || cells_.empty()) {
		return 0;
	}
	std::vector<PartitionCell> kept;
	kept.reserve(cells_.size());
	int removed = 0;
	for (const auto& cell : cells_) {
		if (box_ids.find(cell.box_id) != box_ids.end()) {
			++removed;
			continue;
		}
		kept.push_back(cell);
	}
	if (removed <= 0) {
		return 0;
	}
	cells_ = std::move(kept);
	clear_overlay_edges();
	rebuild_indices();
	return removed;
}

AdaptiveGridPartitionDeltaResult AdaptiveGridPartition::replace_box_ids_with_boxes(
	const std::unordered_set<int>& box_ids,
	const std::vector<BoxNode>& boxes,
	std::size_t first_index,
	double tolerance) {
	using Clock = std::chrono::steady_clock;
	const auto t0 = Clock::now();
	AdaptiveGridPartitionDeltaResult result;
	if (root_interval_copies_.empty() || target_depth_ <= 0) {
		return result;
	}
	if (!box_ids.empty()) {
		std::vector<PartitionCell> kept;
		kept.reserve(cells_.size());
		for (const auto& cell : cells_) {
			if (box_ids.find(cell.box_id) != box_ids.end()) {
				result.boxes_removed += 1;
				continue;
			}
			kept.push_back(cell);
		}
		cells_ = std::move(kept);
	}
	clear_runtime_indices();
	clear_overlay_edges();
	for (int index = 0; index < static_cast<int>(cells_.size()); ++index) {
		auto& cell = cells_[static_cast<std::size_t>(index)];
		cell.cell_id = index;
		cell_by_box_id_[cell.box_id] = index;
	}
	if (first_index < boxes.size()) {
		for (std::size_t index = first_index; index < boxes.size(); ++index) {
			const auto& box = boxes[index];
			if (add_cell_from_box(box,
								  index,
								  box.tree_id >= 0 ? PartitionCellState::Free : PartitionCellState::FreeMerged,
								  tolerance)) {
				result.boxes_appended += 1;
			}
		}
	}
	tolerance_ = tolerance;
	rebuild_indices();
	result.update_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	return result;
}

bool AdaptiveGridPartition::make_grid_range(const std::vector<Interval>& intervals,
											GridRange& range,
											double tolerance) const {
	const int dims = static_cast<int>(root_intervals_.size());
	if (static_cast<int>(intervals.size()) != dims) {
		return false;
	}
	for (int root_index = 0; root_index < static_cast<int>(root_interval_copies_.size()); ++root_index) {
		const auto& root_copy = root_interval_copies_[static_cast<std::size_t>(root_index)];
		range.root_index = root_index;
		range.lo.assign(static_cast<std::size_t>(dims), 0);
		range.hi.assign(static_cast<std::size_t>(dims), 0);
		bool ok = true;
		for (int dim = 0; dim < dims; ++dim) {
			const auto& root = root_copy[static_cast<std::size_t>(dim)];
			const auto& iv = intervals[static_cast<std::size_t>(dim)];
			const double root_width = root.width();
			if (root_width <= 0.0) {
				ok = false;
				break;
			}
			const std::uint64_t denom = max_coord_for_splits(split_counts_[static_cast<std::size_t>(dim)]);
			const double scaled_lo = (iv.lo - root.lo) / root_width * static_cast<double>(denom);
			const double scaled_hi = (iv.hi - root.lo) / root_width * static_cast<double>(denom);
			const double rounded_lo = std::round(scaled_lo);
			const double rounded_hi = std::round(scaled_hi);
			const double coord_tol = std::max(1e-8, tolerance * static_cast<double>(denom) / root_width * 16.0);
			if (std::abs(scaled_lo - rounded_lo) > coord_tol ||
				std::abs(scaled_hi - rounded_hi) > coord_tol ||
				rounded_lo < -coord_tol ||
				rounded_hi > static_cast<double>(denom) + coord_tol ||
				rounded_lo >= rounded_hi) {
				ok = false;
				break;
			}
			range.lo[static_cast<std::size_t>(dim)] =
				static_cast<std::uint64_t>(std::max(0.0, rounded_lo));
			range.hi[static_cast<std::size_t>(dim)] =
				static_cast<std::uint64_t>(std::min(static_cast<double>(denom), rounded_hi));
			if (range.lo[static_cast<std::size_t>(dim)] >= range.hi[static_cast<std::size_t>(dim)]) {
				ok = false;
				break;
			}
		}
		if (ok && range.valid()) {
			return true;
		}
	}
	range.root_index = -1;
	range.lo.clear();
	range.hi.clear();
	return false;
}

bool AdaptiveGridPartition::grid_ranges_overlap_except_dim(const GridRange& lhs,
														   const GridRange& rhs,
														   int dim) const {
	if (!lhs.valid() || !rhs.valid() || lhs.root_index != rhs.root_index || lhs.lo.size() != rhs.lo.size()) {
		return false;
	}
	for (int d = 0; d < static_cast<int>(lhs.lo.size()); ++d) {
		if (d == dim) {
			continue;
		}
		if (lhs.hi[static_cast<std::size_t>(d)] < rhs.lo[static_cast<std::size_t>(d)] ||
			rhs.hi[static_cast<std::size_t>(d)] < lhs.lo[static_cast<std::size_t>(d)]) {
			return false;
		}
	}
	return true;
}

bool AdaptiveGridPartition::grid_ranges_adjacent(const GridRange& lhs, const GridRange& rhs) const {
	if (!lhs.valid() || !rhs.valid() || lhs.root_index != rhs.root_index || lhs.lo.size() != rhs.lo.size()) {
		return false;
	}
	for (int dim = 0; dim < static_cast<int>(lhs.lo.size()); ++dim) {
		const auto llo = lhs.lo[static_cast<std::size_t>(dim)];
		const auto lhi = lhs.hi[static_cast<std::size_t>(dim)];
		const auto rlo = rhs.lo[static_cast<std::size_t>(dim)];
		const auto rhi = rhs.hi[static_cast<std::size_t>(dim)];
		if (lhi < rlo || rhi < llo) {
			return false;
		}
	}
	return true;
}

bool AdaptiveGridPartition::grid_range_contains(const GridRange& outer, const GridRange& inner) const {
	if (!outer.valid() || !inner.valid() || outer.root_index != inner.root_index ||
		outer.lo.size() != inner.lo.size()) {
		return false;
	}
	for (std::size_t dim = 0; dim < outer.lo.size(); ++dim) {
		if (outer.lo[dim] > inner.lo[dim] || outer.hi[dim] < inner.hi[dim]) {
			return false;
		}
	}
	return true;
}

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
			const int bucket_bits = std::clamp(
				env_int_or_default("RBF_PARTITION_CONTAINMENT_BUCKET_BITS", 10),
				1,
				20);
			const std::uint64_t max_bins_per_cell = static_cast<std::uint64_t>(
				std::max(16, env_int_or_default("RBF_PARTITION_CONTAINMENT_MAX_BINS_PER_CELL", 256)));
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

void AdaptiveGridPartition::rebuild_face_index() {
	face_index_.clear();
	stats_.cells = static_cast<int>(cells_.size());
	stats_.grid_cells = 0;
	stats_.non_grid_cells = 0;
	for (const auto& cell : cells_) {
		if (!cell.grid_aligned) {
			stats_.non_grid_cells += 1;
			continue;
		}
		stats_.grid_cells += 1;
		for (int dim = 0; dim < static_cast<int>(cell.grid.lo.size()); ++dim) {
			face_index_[FaceKey{cell.grid.root_index, dim, cell.grid.lo[static_cast<std::size_t>(dim)], false}].push_back(cell.cell_id);
			face_index_[FaceKey{cell.grid.root_index, dim, cell.grid.hi[static_cast<std::size_t>(dim)], true}].push_back(cell.cell_id);
		}
	}
	stats_.face_index_entries = static_cast<int>(face_index_.size());
}

void AdaptiveGridPartition::rebuild_point_index() {
	point_bins_.clear();
	point_overflow_cells_.clear();
	point_index_dims_ = choose_point_index_dims(
		cells_,
		partition_point_index_dims_from_env());
	stats_.point_index_dims = static_cast<int>(point_index_dims_.size());
	if (point_index_dims_.empty() || cells_.empty()) {
		stats_.point_index_entries = 0;
		stats_.point_index_overflow_cells = 0;
		return;
	}
	for (std::size_t item = 0; item < point_index_dims_.size(); ++item) {
		const int dim = point_index_dims_[item];
		point_bin_widths_[item] = choose_bin_width(cells_, dim);
		point_bin_origins_[item] = root_intervals_.empty()
			? 0.0
			: root_intervals_[static_cast<std::size_t>(dim)].lo;
	}
	const std::uint64_t max_entries = partition_point_index_max_cell_entries_from_env();
	for (const auto& cell : cells_) {
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
		if (!valid || entry_count > max_entries) {
			point_overflow_cells_.push_back(cell.cell_id);
			continue;
		}
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
	stats_.point_index_entries = static_cast<int>(point_bins_.size());
	stats_.point_index_overflow_cells = static_cast<int>(point_overflow_cells_.size());
}

AdaptiveGridPartition::SparseCellKey AdaptiveGridPartition::sparse_key_for_grid_range(
	const GridRange& range) const {
	SparseCellKey key;
	key.root_index = range.root_index;
	key.lo = range.lo;
	key.hi = range.hi;
	return key;
}

int AdaptiveGridPartition::sparse_address_depth(const GridRange& range) const {
	if (!range.valid() || range.lo.size() != split_counts_.size()) {
		return 0;
	}
	int depth = 0;
	for (std::size_t dim = 0; dim < range.lo.size(); ++dim) {
		const int split_count = split_counts_[dim];
		const std::uint64_t span = range.hi[dim] > range.lo[dim]
			? range.hi[dim] - range.lo[dim]
			: 0u;
		if (split_count <= 0 || span == 0u) {
			continue;
		}
		if ((span & (span - 1u)) == 0u && (range.lo[dim] % span) == 0u) {
			int span_log2 = 0;
			std::uint64_t value = span;
			while (value > 1u) {
				value >>= 1u;
				++span_log2;
			}
			depth += std::max(0, split_count - span_log2);
		} else {
			depth += split_count;
		}
	}
	return depth;
}

AdaptiveGridPartitionSparseCellRecord make_sparse_virtual_record(
	const PartitionCell& cell,
	const std::vector<int>& split_counts,
	const lect_database::SplitPolicyDescriptor& split_policy,
	int address_depth) {
	AdaptiveGridPartitionSparseCellRecord record;
	record.cell_id = cell.cell_id;
	record.box_id = cell.box_id;
	record.root_index = cell.grid.root_index;
	record.lo = cell.grid.lo;
	record.hi = cell.grid.hi;
	record.split_counts = split_counts;
	record.intervals = cell.intervals;
	record.state = cell.state;
	record.grid_aligned = cell.grid_aligned;
	record.exact_interval_lookup_eligible = cell.grid_aligned && cell.grid.valid();
	record.address_depth = address_depth;
	record.ancestor_refs_avoided =
		static_cast<std::uint64_t>(std::max(0, address_depth));
	record.interval_fingerprint = lect_database::fingerprint_intervals(cell.intervals);
	record.split_policy_hash = lect_database::split_policy_hash(split_policy);
	return record;
}

void AdaptiveGridPartition::rebuild_sparse_virtual_index() {
	sparse_virtual_index_.clear();
	stats_.sparse_virtual_cells = static_cast<int>(cells_.size());
	stats_.sparse_virtual_grid_cells = 0;
	stats_.sparse_virtual_non_grid_cells = 0;
	stats_.sparse_virtual_exact_index_entries = 0;
	stats_.sparse_virtual_max_address_depth = 0;
	stats_.sparse_virtual_ancestor_refs_avoided = 0;
	sparse_virtual_index_.reserve(cells_.size());
	for (const auto& cell : cells_) {
		if (!cell.grid_aligned) {
			stats_.sparse_virtual_non_grid_cells += 1;
			continue;
		}
		stats_.sparse_virtual_grid_cells += 1;
		const int address_depth = sparse_address_depth(cell.grid);
		stats_.sparse_virtual_max_address_depth =
			std::max(stats_.sparse_virtual_max_address_depth, address_depth);
		stats_.sparse_virtual_ancestor_refs_avoided +=
			static_cast<std::uint64_t>(std::max(0, address_depth));
		sparse_virtual_index_[sparse_key_for_grid_range(cell.grid)] = cell.cell_id;
	}
	stats_.sparse_virtual_exact_index_entries =
		static_cast<int>(sparse_virtual_index_.size());
}

void AdaptiveGridPartition::append_sparse_virtual_cell(int cell_index) {
	if (cell_index < 0 || cell_index >= static_cast<int>(cells_.size())) {
		return;
	}
	const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
	stats_.sparse_virtual_cells = static_cast<int>(cells_.size());
	if (!cell.grid_aligned) {
		stats_.sparse_virtual_non_grid_cells += 1;
		return;
	}
	stats_.sparse_virtual_grid_cells += 1;
	const int address_depth = sparse_address_depth(cell.grid);
	stats_.sparse_virtual_max_address_depth =
		std::max(stats_.sparse_virtual_max_address_depth, address_depth);
	stats_.sparse_virtual_ancestor_refs_avoided +=
		static_cast<std::uint64_t>(std::max(0, address_depth));
	sparse_virtual_index_[sparse_key_for_grid_range(cell.grid)] = cell.cell_id;
	stats_.sparse_virtual_exact_index_entries =
		static_cast<int>(sparse_virtual_index_.size());
}

std::vector<int> AdaptiveGridPartition::point_candidate_cells(
	const Eigen::Ref<const Eigen::VectorXd>& q) const {
	std::vector<int> candidates;
	if (!point_index_dims_.empty()) {
		bool valid = true;
		PointBinKey key;
		for (std::size_t item = 0; item < point_index_dims_.size(); ++item) {
			const int dim = point_index_dims_[item];
			if (dim < 0 || dim >= q.size()) {
				valid = false;
				break;
			}
			key.bins[item] = interval_bin(q[dim], point_bin_origins_[item], point_bin_widths_[item]);
		}
		if (valid) {
			const auto it = point_bins_.find(key);
			if (it != point_bins_.end()) {
				candidates = it->second;
			}
			candidates.insert(candidates.end(),
							  point_overflow_cells_.begin(),
							  point_overflow_cells_.end());
		}
	}
	if (candidates.empty()) {
		candidates.reserve(cells_.size());
		for (const auto& cell : cells_) {
			candidates.push_back(cell.cell_id);
		}
	}
	return candidates;
}

std::vector<int> AdaptiveGridPartition::interval_candidate_cells(
	const std::vector<Interval>& intervals) const {
	std::vector<int> candidates;
	if (!point_index_dims_.empty()) {
		bool valid = true;
		std::array<long long, 3> lo_bins{0, 0, 0};
		std::array<long long, 3> hi_bins{0, 0, 0};
		std::uint64_t entry_count = 1;
		for (std::size_t item = 0; item < point_index_dims_.size(); ++item) {
			const int dim = point_index_dims_[item];
			if (dim < 0 || dim >= static_cast<int>(intervals.size())) {
				valid = false;
				break;
			}
			const auto& iv = intervals[static_cast<std::size_t>(dim)];
			lo_bins[item] = interval_bin(iv.lo, point_bin_origins_[item], point_bin_widths_[item]);
			hi_bins[item] = interval_bin(iv.hi, point_bin_origins_[item], point_bin_widths_[item]);
			if (hi_bins[item] < lo_bins[item]) {
				valid = false;
				break;
			}
			entry_count *= static_cast<std::uint64_t>(hi_bins[item] - lo_bins[item] + 1);
		}
		const std::uint64_t max_entries = partition_point_index_max_query_entries_from_env();
		if (valid && entry_count <= max_entries) {
			std::unordered_set<int> seen;
			for (long long b0 = lo_bins[0]; b0 <= hi_bins[0]; ++b0) {
				const long long b1_lo = point_index_dims_.size() > 1 ? lo_bins[1] : 0;
				const long long b1_hi = point_index_dims_.size() > 1 ? hi_bins[1] : 0;
				for (long long b1 = b1_lo; b1 <= b1_hi; ++b1) {
					const long long b2_lo = point_index_dims_.size() > 2 ? lo_bins[2] : 0;
					const long long b2_hi = point_index_dims_.size() > 2 ? hi_bins[2] : 0;
					for (long long b2 = b2_lo; b2 <= b2_hi; ++b2) {
						const auto it = point_bins_.find(PointBinKey{{b0, b1, b2}});
						if (it == point_bins_.end()) {
							continue;
						}
						for (int candidate : it->second) {
							if (seen.insert(candidate).second) {
								candidates.push_back(candidate);
							}
						}
					}
				}
			}
			for (int candidate : point_overflow_cells_) {
				if (seen.insert(candidate).second) {
					candidates.push_back(candidate);
				}
			}
		}
	}
	if (candidates.empty()) {
		candidates.reserve(cells_.size());
		for (const auto& cell : cells_) {
			candidates.push_back(cell.cell_id);
		}
	}
	return candidates;
}

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
	const int indexed_threshold = partition_indexed_adjacency_threshold_from_env();
	if (static_cast<int>(cells_.size()) < indexed_threshold) {
		for (int cell_index = 0; cell_index < static_cast<int>(cells_.size()); ++cell_index) {
			neighbor_cache_[static_cast<std::size_t>(cell_index)] =
				compute_neighbor_cell_indices(cell_index);
		}
		return;
	}
	const int dims = static_cast<int>(split_counts_.size());
	const int max_adjacency_dims = partition_adjacency_dim_limit_from_env(dims);
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

	const bool enable_cross_root_adjacency = partition_cross_root_adjacency_enabled_from_env();
	const std::uint64_t max_bins_per_cell = partition_broadphase_max_bins_per_cell_from_env();
	const int adjacency_bucket_bits = partition_adjacency_bucket_bits_from_env();
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
	const int max_neighbors_per_cell = partition_max_neighbors_per_cell_from_env(cells_.size());
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

int AdaptiveGridPartition::locate_containing_box(const Eigen::Ref<const Eigen::VectorXd>& q,
												 bool nearest_if_outside,
												 double tolerance) const {
	if (q.size() != static_cast<int>(root_intervals_.size())) {
		return -1;
	}
	const std::vector<int> candidates = point_candidate_cells(q);
	int best_box = -1;
	double best_volume = std::numeric_limits<double>::infinity();
	for (int cell_id : candidates) {
		const auto& cell = cells_[static_cast<std::size_t>(cell_id)];
		if (!intervals_contain_point(cell.intervals, q, tolerance)) {
			continue;
		}
		double volume = 1.0;
		for (const auto& iv : cell.intervals) {
			volume *= std::max(0.0, iv.width());
		}
		if (volume < best_volume) {
			best_volume = volume;
			best_box = cell.box_id;
		}
	}
	if (best_box >= 0 || !nearest_if_outside) {
		return best_box;
	}
	double best_dist = std::numeric_limits<double>::infinity();
	for (const auto& cell : cells_) {
		const double dist = point_to_box_distance_sq(q, cell.intervals);
		if (dist < best_dist) {
			best_dist = dist;
			best_box = cell.box_id;
		}
	}
	return best_box;
}

std::vector<int> AdaptiveGridPartition::covering_box_ids(const Eigen::Ref<const Eigen::VectorXd>& q,
														 double tolerance) const {
	std::vector<int> ids;
	if (q.size() != static_cast<int>(root_intervals_.size())) {
		return ids;
	}
	const std::vector<int> candidates = point_candidate_cells(q);
	ids.reserve(candidates.size());
	for (int cell_id : candidates) {
		if (cell_id < 0 || cell_id >= static_cast<int>(cells_.size())) {
			continue;
		}
		const auto& cell = cells_[static_cast<std::size_t>(cell_id)];
		if (intervals_contain_point(cell.intervals, q, tolerance)) {
			ids.push_back(cell.box_id);
		}
	}
	return ids;
}

std::vector<int> AdaptiveGridPartition::covering_box_indices(const Eigen::Ref<const Eigen::VectorXd>& q,
															 double tolerance) const {
	std::vector<int> indices;
	if (q.size() != static_cast<int>(root_intervals_.size())) {
		return indices;
	}
	const std::vector<int> candidates = point_candidate_cells(q);
	indices.reserve(candidates.size());
	for (int cell_id : candidates) {
		if (cell_id < 0 || cell_id >= static_cast<int>(cells_.size())) {
			continue;
		}
		const auto& cell = cells_[static_cast<std::size_t>(cell_id)];
		if (intervals_contain_point(cell.intervals, q, tolerance)) {
			indices.push_back(static_cast<int>(cell.box_index));
		}
	}
	return indices;
}

std::vector<AdaptiveGridPartitionNearestBox> AdaptiveGridPartition::nearest_boxes(
	const Eigen::Ref<const Eigen::VectorXd>& q,
	const std::vector<int>& candidate_box_ids,
	int limit) const {
	std::vector<AdaptiveGridPartitionNearestBox> nearest;
	if (q.size() != static_cast<int>(root_intervals_.size()) || limit == 0) {
		return nearest;
	}
	std::vector<int> candidate_cells;
	if (!candidate_box_ids.empty()) {
		std::unordered_set<int> seen;
		seen.reserve(candidate_box_ids.size() * 2);
		candidate_cells.reserve(candidate_box_ids.size());
		for (int box_id : candidate_box_ids) {
			if (!seen.insert(box_id).second) {
				continue;
			}
			const auto it = cell_by_box_id_.find(box_id);
			if (it != cell_by_box_id_.end()) {
				candidate_cells.push_back(it->second);
			}
		}
	} else {
		candidate_cells.reserve(cells_.size());
		for (int cell_index = 0; cell_index < static_cast<int>(cells_.size()); ++cell_index) {
			candidate_cells.push_back(cell_index);
		}
	}
	const int effective_limit = limit > 0 ? limit : static_cast<int>(cells_.size());
	nearest.reserve(static_cast<std::size_t>(std::min<int>(effective_limit,
														   static_cast<int>(candidate_cells.size()))));
	for (int cell_index : candidate_cells) {
		if (cell_index < 0 || cell_index >= static_cast<int>(cells_.size())) {
			continue;
		}
		const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
		if (cell.intervals.size() != static_cast<std::size_t>(q.size())) {
			continue;
		}
		const Eigen::VectorXd closest = closest_point_in_intervals(cell.intervals, q);
		const double dist2 = (closest - q).squaredNorm();
		nearest.push_back({cell.box_id, closest, dist2});
	}
	const int keep = std::min<int>(effective_limit, static_cast<int>(nearest.size()));
	auto less = [](const AdaptiveGridPartitionNearestBox& lhs,
				   const AdaptiveGridPartitionNearestBox& rhs) {
		if (std::abs(lhs.distance_sq - rhs.distance_sq) > 1e-18) {
			return lhs.distance_sq < rhs.distance_sq;
		}
		return lhs.box_id < rhs.box_id;
	};
	if (keep < static_cast<int>(nearest.size())) {
		std::partial_sort(nearest.begin(), nearest.begin() + keep, nearest.end(), less);
		nearest.resize(static_cast<std::size_t>(keep));
	} else {
		std::sort(nearest.begin(), nearest.end(), less);
	}
	return nearest;
}

std::vector<AdaptiveGridPartitionComponentPair>
AdaptiveGridPartition::nearest_component_pairs_to_largest(int max_pairs_per_component,
														  int candidate_cap) const {
	std::vector<AdaptiveGridPartitionComponentPair> pairs;
	const auto components = component_cell_indices_with_overlay();
	if (components.size() <= 1 || max_pairs_per_component == 0) {
		return pairs;
	}
	const int per_component_limit = max_pairs_per_component > 0 ? max_pairs_per_component : 1;
	const int cap = std::max(1, candidate_cap);
	const std::uint64_t exact_pair_cap = partition_component_pair_exact_cap_from_env();
	const auto& main = components.front();
	const std::vector<Interval> main_hull = interval_hull_for_cells(cells_, main);
	pairs.reserve((components.size() - 1) * static_cast<std::size_t>(per_component_limit));
	for (std::size_t component_index = 1; component_index < components.size(); ++component_index) {
		const auto& source_component = components[component_index];
		if (source_component.empty() || main.empty()) {
			continue;
		}
		std::vector<int> source_candidates = source_component;
		std::vector<int> target_candidates = main;
		const std::uint64_t exact_pairs =
			static_cast<std::uint64_t>(source_candidates.size()) *
			static_cast<std::uint64_t>(target_candidates.size());
		if (exact_pairs > exact_pair_cap) {
			const std::vector<Interval> source_hull =
				interval_hull_for_cells(cells_, source_component);
			source_candidates = closest_cells_to_hull(cells_,
													  source_component,
													  main_hull,
													  cap);
			target_candidates = closest_cells_to_hull(cells_,
													  main,
													  source_hull,
													  cap);
		}
		std::vector<AdaptiveGridPartitionComponentPair> component_pairs;
		const std::uint64_t candidate_pair_count =
			static_cast<std::uint64_t>(source_candidates.size()) *
			static_cast<std::uint64_t>(target_candidates.size());
		component_pairs.reserve(static_cast<std::size_t>(
			std::min<std::uint64_t>(static_cast<std::uint64_t>(per_component_limit),
									candidate_pair_count)));
		for (int source_cell_index : source_candidates) {
			if (source_cell_index < 0 ||
				source_cell_index >= static_cast<int>(cells_.size())) {
				continue;
			}
			const auto& source = cells_[static_cast<std::size_t>(source_cell_index)];
			for (int target_cell_index : target_candidates) {
				if (target_cell_index < 0 ||
					target_cell_index >= static_cast<int>(cells_.size()) ||
					target_cell_index == source_cell_index) {
					continue;
				}
				const auto& target = cells_[static_cast<std::size_t>(target_cell_index)];
				Eigen::VectorXd source_point;
				Eigen::VectorXd target_point;
				double distance_sq = std::numeric_limits<double>::infinity();
				if (!closest_points_between_interval_boxes(source.intervals,
														   target.intervals,
														   source_point,
														   target_point,
														   distance_sq)) {
					continue;
				}
				AdaptiveGridPartitionComponentPair pair;
				pair.source_box_id = source.box_id;
				pair.target_box_id = target.box_id;
				pair.source_component_index = static_cast<int>(component_index);
				pair.target_component_index = 0;
				pair.source_component_size = static_cast<int>(source_component.size());
				pair.target_component_size = static_cast<int>(main.size());
				pair.source_point = std::move(source_point);
				pair.target_point = std::move(target_point);
				pair.distance_sq = distance_sq;
				component_pairs.push_back(std::move(pair));
			}
		}
		const int keep = std::min<int>(per_component_limit,
									   static_cast<int>(component_pairs.size()));
		if (keep <= 0) {
			continue;
		}
		auto less = [](const AdaptiveGridPartitionComponentPair& lhs,
					   const AdaptiveGridPartitionComponentPair& rhs) {
			if (std::abs(lhs.distance_sq - rhs.distance_sq) > 1e-18) {
				return lhs.distance_sq < rhs.distance_sq;
			}
			if (lhs.source_box_id != rhs.source_box_id) {
				return lhs.source_box_id < rhs.source_box_id;
			}
			return lhs.target_box_id < rhs.target_box_id;
		};
		if (keep < static_cast<int>(component_pairs.size())) {
			std::partial_sort(component_pairs.begin(),
							  component_pairs.begin() + keep,
							  component_pairs.end(),
							  less);
			component_pairs.resize(static_cast<std::size_t>(keep));
		} else {
			std::sort(component_pairs.begin(), component_pairs.end(), less);
		}
		for (auto& pair : component_pairs) {
			pairs.push_back(std::move(pair));
		}
	}
	return pairs;
}

std::vector<AdaptiveGridPartitionLandmark>
AdaptiveGridPartition::landmarks(bool largest_component_only, int limit) const {
	std::vector<AdaptiveGridPartitionLandmark> out;
	if (cells_.empty() || limit == 0) {
		return out;
	}
	const int effective_limit = limit > 0 ? limit : static_cast<int>(cells_.size());
	const auto components = component_cell_indices_with_overlay();
	if (components.empty()) {
		return out;
	}
	const std::size_t component_count =
		largest_component_only ? 1u : components.size();
	for (std::size_t component_index = 0; component_index < component_count; ++component_index) {
		for (int cell_index : components[component_index]) {
			if (cell_index < 0 || cell_index >= static_cast<int>(cells_.size())) {
				continue;
			}
			const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
			AdaptiveGridPartitionLandmark landmark;
			landmark.box_id = cell.box_id;
			landmark.component_index = static_cast<int>(component_index);
			landmark.center = box_center(cell.intervals);
			landmark.volume = box_volume_from_intervals(cell.intervals);
			out.push_back(std::move(landmark));
		}
	}
	auto less = [](const AdaptiveGridPartitionLandmark& lhs,
				   const AdaptiveGridPartitionLandmark& rhs) {
		if (std::abs(lhs.volume - rhs.volume) > 1e-18) {
			return lhs.volume > rhs.volume;
		}
		return lhs.box_id < rhs.box_id;
	};
	if (static_cast<int>(out.size()) > effective_limit) {
		std::partial_sort(out.begin(),
						  out.begin() + effective_limit,
						  out.end(),
						  less);
		out.resize(static_cast<std::size_t>(effective_limit));
	} else {
		std::sort(out.begin(), out.end(), less);
	}
	return out;
}

std::vector<int> AdaptiveGridPartition::neighbor_box_ids(int box_id) const {
	const auto it = cell_by_box_id_.find(box_id);
	if (it == cell_by_box_id_.end()) {
		return {};
	}
	std::vector<int> ids;
	for (int cell_index : neighbor_cell_indices(it->second)) {
		ids.push_back(cells_[static_cast<std::size_t>(cell_index)].box_id);
	}
	return ids;
}

bool AdaptiveGridPartition::contains_box_id(int box_id) const {
	return cell_by_box_id_.find(box_id) != cell_by_box_id_.end();
}

bool AdaptiveGridPartition::intervals_for_box(int box_id,
											  std::vector<Interval>& intervals) const {
	const auto it = cell_by_box_id_.find(box_id);
	if (it == cell_by_box_id_.end()) {
		return false;
	}
	intervals = cells_[static_cast<std::size_t>(it->second)].intervals;
	return true;
}

bool AdaptiveGridPartition::center_for_box(int box_id, Eigen::VectorXd& center) const {
	const auto it = cell_by_box_id_.find(box_id);
	if (it == cell_by_box_id_.end()) {
		return false;
	}
	const auto& cell = cells_[static_cast<std::size_t>(it->second)];
	center = box_center(cell.intervals);
	return true;
}

bool AdaptiveGridPartition::closest_point_for_box(
	int box_id,
	const Eigen::Ref<const Eigen::VectorXd>& q,
	Eigen::VectorXd& closest,
	double* distance_sq) const {
	const auto it = cell_by_box_id_.find(box_id);
	if (it == cell_by_box_id_.end() ||
		q.size() != static_cast<int>(root_intervals_.size())) {
		return false;
	}
	const auto& cell = cells_[static_cast<std::size_t>(it->second)];
	if (cell.intervals.size() != static_cast<std::size_t>(q.size())) {
		return false;
	}
	closest = closest_point_in_intervals(cell.intervals, q);
	if (distance_sq != nullptr) {
		*distance_sq = (closest - q).squaredNorm();
	}
	return true;
}

bool AdaptiveGridPartition::box_contains_point(
	int box_id,
	const Eigen::Ref<const Eigen::VectorXd>& q,
	double tolerance) const {
	const auto it = cell_by_box_id_.find(box_id);
	if (it == cell_by_box_id_.end() ||
		q.size() != static_cast<int>(root_intervals_.size())) {
		return false;
	}
	const auto& cell = cells_[static_cast<std::size_t>(it->second)];
	return intervals_contain_point(cell.intervals, q, tolerance);
}

bool AdaptiveGridPartition::box_adjacent_to_box(int box_id,
												const BoxNode& box,
												double tolerance) const {
	const auto it = cell_by_box_id_.find(box_id);
	if (it == cell_by_box_id_.end() ||
		box.joint_intervals.size() != root_intervals_.size()) {
		return false;
	}
	const auto& cell = cells_[static_cast<std::size_t>(it->second)];
	GridRange query_range;
	const bool query_grid_aligned = make_grid_range(box.joint_intervals,
													query_range,
													tolerance);
	if (query_grid_aligned && cell.grid_aligned &&
		query_range.root_index == cell.grid.root_index) {
		return grid_ranges_adjacent(query_range, cell.grid);
	}
	return interval_boxes_connected(box.joint_intervals, cell.intervals, tolerance);
}

int AdaptiveGridPartition::sparse_virtual_cell_for_intervals(
	const std::vector<Interval>& intervals,
	double tolerance) const {
	GridRange range;
	if (!make_grid_range(intervals, range, tolerance)) {
		return -1;
	}
	const auto it = sparse_virtual_index_.find(sparse_key_for_grid_range(range));
	return it == sparse_virtual_index_.end() ? -1 : it->second;
}

std::optional<AdaptiveGridPartitionSparseCellRecord>
AdaptiveGridPartition::sparse_virtual_record_for_intervals(
	const std::vector<Interval>& intervals,
	double tolerance) const {
	GridRange range;
	if (!make_grid_range(intervals, range, tolerance)) {
		return std::nullopt;
	}
	const auto it = sparse_virtual_index_.find(sparse_key_for_grid_range(range));
	if (it == sparse_virtual_index_.end()) {
		return std::nullopt;
	}
	const int cell_id = it->second;
	if (cell_id < 0 || cell_id >= static_cast<int>(cells_.size())) {
		return std::nullopt;
	}
	const auto& cell = cells_[static_cast<std::size_t>(cell_id)];
	return make_sparse_virtual_record(
		cell,
		split_counts_,
		split_policy_,
		sparse_address_depth(cell.grid));
}

std::vector<AdaptiveGridPartitionSparseCellRecord>
AdaptiveGridPartition::sparse_virtual_records() const {
	std::vector<AdaptiveGridPartitionSparseCellRecord> records;
	records.reserve(cells_.size());
	for (const auto& cell : cells_) {
		const int address_depth = cell.grid_aligned ? sparse_address_depth(cell.grid) : 0;
		records.push_back(make_sparse_virtual_record(
			cell,
			split_counts_,
			split_policy_,
			address_depth));
	}
	return records;
}

bool AdaptiveGridPartition::boxes_are_neighbors(int lhs_box_id, int rhs_box_id) const {
	if (lhs_box_id == rhs_box_id) {
		return false;
	}
	const auto lhs_it = cell_by_box_id_.find(lhs_box_id);
	const auto rhs_it = cell_by_box_id_.find(rhs_box_id);
	if (lhs_it == cell_by_box_id_.end() || rhs_it == cell_by_box_id_.end()) {
		return false;
	}
	const int lhs_cell = lhs_it->second;
	const int rhs_cell = rhs_it->second;
	if (lhs_cell < 0 || rhs_cell < 0) {
		return false;
	}
	if (lhs_cell < static_cast<int>(neighbor_cache_.size())) {
		const auto& neighbors = neighbor_cache_[static_cast<std::size_t>(lhs_cell)];
		return std::find(neighbors.begin(), neighbors.end(), rhs_cell) != neighbors.end();
	}
	const auto neighbors = compute_neighbor_cell_indices(lhs_cell);
	return std::find(neighbors.begin(), neighbors.end(), rhs_cell) != neighbors.end();
}

int AdaptiveGridPartition::island_id_for_box(int box_id) const {
	const auto it = cell_by_box_id_.find(box_id);
	if (it == cell_by_box_id_.end()) {
		return -1;
	}
	return cells_[static_cast<std::size_t>(it->second)].island_id;
}

bool AdaptiveGridPartition::same_island(int lhs_box_id, int rhs_box_id) const {
	const int lhs = island_id_for_box(lhs_box_id);
	const int rhs = island_id_for_box(rhs_box_id);
	return lhs >= 0 && lhs == rhs;
}

std::vector<std::vector<int>> AdaptiveGridPartition::island_box_ids() const {
	std::unordered_map<int, std::vector<int>> by_island;
	by_island.reserve(static_cast<std::size_t>(std::max(1, stats_.islands)));
	for (const auto& cell : cells_) {
		if (cell.island_id >= 0) {
			by_island[cell.island_id].push_back(cell.box_id);
		}
	}
	std::vector<std::vector<int>> islands;
	islands.reserve(by_island.size());
	for (auto& [_, ids] : by_island) {
		islands.push_back(std::move(ids));
	}
	std::sort(islands.begin(), islands.end(), [](const auto& lhs, const auto& rhs) {
		return lhs.size() > rhs.size();
	});
	return islands;
}

std::vector<int> AdaptiveGridPartition::largest_island_box_ids() const {
	const auto islands = island_box_ids();
	if (islands.empty()) {
		return {};
	}
	return islands.front();
}

bool AdaptiveGridPartition::box_adjacent_to_any(const BoxNode& box,
												 const std::unordered_set<int>& box_ids,
												 double tolerance) const {
	if (box_ids.empty() || box.joint_intervals.size() != root_intervals_.size()) {
		return false;
	}
	for (int adjacent_id : adjacent_box_ids(box, tolerance)) {
		if (box_ids.find(adjacent_id) != box_ids.end()) {
			return true;
		}
	}
	GridRange query_range;
	const bool query_grid_aligned = make_grid_range(box.joint_intervals, query_range, tolerance);
	for (int box_id : box_ids) {
		const auto it = cell_by_box_id_.find(box_id);
		if (it == cell_by_box_id_.end()) {
			continue;
		}
		const auto& cell = cells_[static_cast<std::size_t>(it->second)];
		if (query_grid_aligned && cell.grid_aligned) {
			if (grid_ranges_adjacent(query_range, cell.grid)) {
				return true;
			}
			continue;
		}
		if (interval_boxes_connected(box.joint_intervals,
									 cell.intervals,
									 tolerance)) {
			return true;
		}
	}
	return false;
}

std::vector<int> AdaptiveGridPartition::adjacent_box_ids(const BoxNode& box,
														 double tolerance) const {
	std::vector<int> result;
	if (box.joint_intervals.size() != root_intervals_.size()) {
		return result;
	}
	std::unordered_set<int> seen;
	GridRange query_range;
	const bool query_grid_aligned = make_grid_range(box.joint_intervals,
													query_range,
													tolerance);
	if (query_grid_aligned) {
		for (int dim = 0; dim < static_cast<int>(query_range.lo.size()); ++dim) {
			const FaceKey lower_query{
				query_range.root_index,
				dim,
				query_range.lo[static_cast<std::size_t>(dim)],
				true};
			const FaceKey upper_query{
				query_range.root_index,
				dim,
				query_range.hi[static_cast<std::size_t>(dim)],
				false};
			for (const FaceKey& key : {lower_query, upper_query}) {
				const auto it = face_index_.find(key);
				if (it == face_index_.end()) {
					continue;
				}
				for (int candidate : it->second) {
					if (candidate < 0 ||
						candidate >= static_cast<int>(cells_.size())) {
						continue;
					}
					const auto& other = cells_[static_cast<std::size_t>(candidate)];
					if (other.box_id == box.id ||
						!other.grid_aligned ||
						!seen.insert(other.box_id).second) {
						continue;
					}
					if (grid_ranges_overlap_except_dim(query_range, other.grid, dim) &&
						grid_ranges_adjacent(query_range, other.grid)) {
						result.push_back(other.box_id);
					}
				}
			}
		}
	}

	const bool needs_exact_pass = !query_grid_aligned || stats_.non_grid_cells > 0;
	if (!needs_exact_pass) {
		return result;
	}

	const std::vector<int> exact_candidates = interval_candidate_cells(box.joint_intervals);
	for (int candidate : exact_candidates) {
		if (candidate < 0 || candidate >= static_cast<int>(cells_.size())) {
			continue;
		}
		const auto& other = cells_[static_cast<std::size_t>(candidate)];
		if (other.box_id == box.id || seen.find(other.box_id) != seen.end()) {
			continue;
		}
		const bool needs_exact =
			!query_grid_aligned ||
			!other.grid_aligned ||
			query_range.root_index != other.grid.root_index;
		if (!needs_exact) {
			continue;
		}
		if (interval_boxes_connected(box.joint_intervals, other.intervals, tolerance) &&
			seen.insert(other.box_id).second) {
			result.push_back(other.box_id);
		}
	}
	return result;
}

AdaptiveGridPartitionConnectivityDominance
AdaptiveGridPartition::classify_connectivity_dominance(const BoxNode& box,
													   double tolerance) const {
	AdaptiveGridPartitionConnectivityDominance out;
	if (box.joint_intervals.size() != root_intervals_.size()) {
		return out;
	}
	for (const auto& cell : cells_) {
		if (interval_box_subset(box.joint_intervals, cell.intervals, tolerance)) {
			out.covered_by_existing = true;
			out.isolated = false;
			out.priority_delta = -100.0;
			return out;
		}
	}

	const auto components = component_cell_indices_with_overlay();
	std::unordered_map<int, int> component_by_box_id;
	component_by_box_id.reserve(cells_.size());
	for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
		for (int cell_index : components[component_index]) {
			if (cell_index >= 0 && cell_index < static_cast<int>(cells_.size())) {
				component_by_box_id[cells_[static_cast<std::size_t>(cell_index)].box_id] =
					static_cast<int>(component_index);
			}
		}
	}

	const auto adjacent_ids = adjacent_box_ids(box, tolerance);
	std::unordered_set<int> adjacent_components;
	adjacent_components.reserve(adjacent_ids.size());
	for (int adjacent_id : adjacent_ids) {
		const auto comp_it = component_by_box_id.find(adjacent_id);
		if (comp_it == component_by_box_id.end()) {
			continue;
		}
		adjacent_components.insert(comp_it->second);
	}

	out.adjacent_box_count = static_cast<int>(adjacent_ids.size());
	out.adjacent_component_count = static_cast<int>(adjacent_components.size());
	out.adjacent_to_largest_component = adjacent_components.find(0) != adjacent_components.end();
	out.connector_candidate = out.adjacent_component_count >= 2;
	out.single_component = out.adjacent_component_count == 1;
	out.isolated = out.adjacent_component_count == 0;

	if (out.connector_candidate) {
		out.priority_delta = 60.0 + (out.adjacent_to_largest_component ? 20.0 : 0.0);
	} else if (out.adjacent_to_largest_component) {
		out.priority_delta = 18.0;
	} else if (out.single_component) {
		out.priority_delta = -4.0;
	} else {
		out.priority_delta = -12.0;
	}
	return out;
}

std::vector<int> AdaptiveGridPartition::shortcut_sequence(
	const std::vector<int>& sequence,
	const std::unordered_map<int, int>& cell_by_box_id) const {
	if (sequence.size() <= 2) {
		return sequence;
	}
	const QueryShortcutCostOptions shortcut_options =
		query_shortcut_cost_options_from_env(true);
	auto transition_cost = [&](int lhs_box_id, int rhs_box_id) {
		const auto lhs_it = cell_by_box_id.find(lhs_box_id);
		const auto rhs_it = cell_by_box_id.find(rhs_box_id);
		if (lhs_it == cell_by_box_id.end() || rhs_it == cell_by_box_id.end()) {
			return std::numeric_limits<double>::infinity();
		}
		const auto& lhs = cells_[static_cast<std::size_t>(lhs_it->second)];
		const auto& rhs = cells_[static_cast<std::size_t>(rhs_it->second)];
		return (box_center(lhs.intervals) - box_center(rhs.intervals)).norm();
	};
	auto sequence_cost = [&](std::size_t begin, std::size_t end) {
		double total = 0.0;
		for (std::size_t k = begin + 1; k <= end; ++k) {
			total += transition_cost(sequence[k - 1], sequence[k]);
		}
		return total;
	};
	auto shortcut_acceptable = [&](std::size_t begin, std::size_t end, int bridge_box_id) {
		if (!shortcut_options.cost_aware) {
			return true;
		}
		const double original_cost = sequence_cost(begin, end);
		const double shortcut_cost = bridge_box_id >= 0
			? transition_cost(sequence[begin], bridge_box_id) +
				  transition_cost(bridge_box_id, sequence[end])
			: transition_cost(sequence[begin], sequence[end]);
		return std::isfinite(original_cost) &&
			   std::isfinite(shortcut_cost) &&
			   shortcut_cost <= original_cost * shortcut_options.cost_factor + 1e-9;
	};
	auto neighbor_box_set = [&](int box_id) {
		std::unordered_set<int> out;
		const auto it = cell_by_box_id.find(box_id);
		if (it == cell_by_box_id.end()) {
			return out;
		}
		for (int cell_index : neighbor_cell_indices(it->second)) {
			out.insert(cells_[static_cast<std::size_t>(cell_index)].box_id);
		}
		return out;
	};
	std::vector<int> shortened;
	shortened.reserve(sequence.size());
	std::size_t i = 0;
	while (i < sequence.size()) {
		if (shortened.empty() || shortened.back() != sequence[i]) {
			shortened.push_back(sequence[i]);
		}
		if (i + 1 >= sequence.size()) {
			break;
		}
		std::size_t best = i + 1;
		int bridge = -1;
		const auto i_neighbors = neighbor_box_set(sequence[i]);
		for (std::size_t j = sequence.size() - 1; j > i + 1; --j) {
			if (i_neighbors.find(sequence[j]) != i_neighbors.end() &&
				shortcut_acceptable(i, j, -1)) {
				best = j;
				bridge = -1;
				break;
			}
			const auto j_neighbors = neighbor_box_set(sequence[j]);
			for (int candidate : i_neighbors) {
				if (candidate != sequence[i] &&
					candidate != sequence[j] &&
					j_neighbors.find(candidate) != j_neighbors.end() &&
					shortcut_acceptable(i, j, candidate)) {
					best = j;
					bridge = candidate;
					break;
				}
			}
			if (bridge >= 0) {
				break;
			}
		}
		if (bridge >= 0 && shortened.back() != bridge) {
			shortened.push_back(bridge);
		}
		i = best;
	}
	return shortened;
}

AdaptiveGridPartitionQueryResult AdaptiveGridPartition::query(
	const Eigen::Ref<const Eigen::VectorXd>& start,
	const Eigen::Ref<const Eigen::VectorXd>& goal,
	const AdaptiveGridPartitionQueryOptions& options) const {
	using Clock = std::chrono::steady_clock;
	const auto t0 = Clock::now();
	AdaptiveGridPartitionQueryResult result;
	result.start_box_id = locate_containing_box(start, options.nearest_if_outside, options.adjacency_tolerance);
	result.goal_box_id = locate_containing_box(goal, options.nearest_if_outside, options.adjacency_tolerance);
	if (result.start_box_id < 0 || result.goal_box_id < 0) {
		result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
		return result;
	}
	const auto start_it = cell_by_box_id_.find(result.start_box_id);
	const auto goal_it = cell_by_box_id_.find(result.goal_box_id);
	if (start_it == cell_by_box_id_.end() || goal_it == cell_by_box_id_.end()) {
		result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
		return result;
	}
	const int start_cell = start_it->second;
	const int goal_cell = goal_it->second;
	const int start_island = cells_[static_cast<std::size_t>(start_cell)].island_id;
	const int goal_island = cells_[static_cast<std::size_t>(goal_cell)].island_id;
	if (start_island < 0 || goal_island < 0 ||
		(start_island != goal_island && overlay_edges_by_cell_.empty())) {
		result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
		return result;
	}
	if (start_island != goal_island &&
		partition_query_component_prune_enabled_from_env() &&
		!same_component_with_overlay(result.start_box_id, result.goal_box_id)) {
		result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
		return result;
	}
	if (start_cell == goal_cell) {
		result.found = true;
		result.box_sequence = {result.start_box_id};
		result.path.push_back(start);
		if ((start - goal).norm() > 1e-12) {
			result.path.push_back(goal);
		}
		result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
		return result;
	}
	struct Item {
		int cell;
		double priority;
		bool operator>(const Item& other) const { return priority > other.priority; }
	};
	auto heuristic = [&](int cell_index) {
		return (box_center(cells_[static_cast<std::size_t>(cell_index)].intervals) - goal).norm();
	};
	std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
	const std::size_t cell_count = cells_.size();
	const double inf = std::numeric_limits<double>::infinity();
	std::vector<double> dist(cell_count, inf);
	std::vector<int> prev(cell_count, -1);
	std::vector<int> prev_edge(cell_count, -1);
	std::vector<Eigen::VectorXd> representative(cell_count);
	dist[static_cast<std::size_t>(start_cell)] = 0.0;
	representative[static_cast<std::size_t>(start_cell)] = start;
	open.push({start_cell, heuristic(start_cell)});
	const int max_expansions = std::max(0, options.max_expansions);
	const QueryGraphCostOptions cost_options = query_graph_cost_options_from_env();
	const bool line_deviation_enabled =
		cost_options.box_line_deviation_penalty > 0.0 &&
		start.size() == goal.size() &&
		start.size() > 0;
	auto edge_line_deviation = [&](const OverlayEdge& edge,
								   const Eigen::VectorXd& fallback_point) {
		if (!line_deviation_enabled) {
			return 0.0;
		}
		double max_deviation = distance_to_line(fallback_point, start, goal);
		for (const auto& waypoint : edge.waypoints) {
			if (waypoint.size() == start.size()) {
				max_deviation = std::max(max_deviation,
										 distance_to_line(waypoint, start, goal));
			}
		}
		return max_deviation;
	};
	while (!open.empty()) {
		const Item item = open.top();
		open.pop();
		if (item.cell < 0 || item.cell >= static_cast<int>(cell_count)) {
			continue;
		}
		const double item_dist = dist[static_cast<std::size_t>(item.cell)];
		if (!std::isfinite(item_dist) ||
			item.priority > item_dist + heuristic(item.cell) + 1e-12) {
			continue;
		}
		result.cells_expanded += 1;
		if (item.cell == goal_cell) {
			result.found = true;
			break;
		}
		if (max_expansions > 0 && result.cells_expanded >= max_expansions) {
			break;
		}
		const auto neighbors = neighbor_cell_indices(item.cell);
		result.adjacency_candidates += static_cast<int>(neighbors.size());
		const Eigen::VectorXd current_rep =
			representative[static_cast<std::size_t>(item.cell)];
		const auto& current_cell = cells_[static_cast<std::size_t>(item.cell)];
		auto relax_edge = [&](int next,
							  Eigen::VectorXd next_rep,
							  double edge_cost,
							  int segment_edge_id) {
			if (next < 0 || next >= static_cast<int>(cell_count)) {
				return;
			}
			const double alt = item_dist + edge_cost;
			const auto next_index = static_cast<std::size_t>(next);
			if (alt < dist[next_index]) {
				dist[next_index] = alt;
				prev[next_index] = item.cell;
				prev_edge[next_index] = segment_edge_id;
				representative[next_index] = std::move(next_rep);
				open.push({next, alt + heuristic(next)});
			}
		};
		for (int next : neighbors) {
			const auto& next_cell = cells_[static_cast<std::size_t>(next)];
			Eigen::VectorXd next_rep = transition_waypoint_toward_goal(current_cell.intervals,
																	   next_cell.intervals,
																	   current_rep,
																	   goal,
																	   options.adjacency_tolerance);
			const double transition_length = (current_rep - next_rep).norm();
			double edge_cost = transition_length + 1e-6 +
							   cost_options.box_transition_penalty;
			if (cost_options.box_nonprogress_penalty > 0.0 &&
				goal.size() == current_rep.size() &&
				goal.size() == next_rep.size()) {
				const double current_goal_distance = (current_rep - goal).norm();
				const double next_goal_distance = (next_rep - goal).norm();
				edge_cost += cost_options.box_nonprogress_penalty *
							 std::max(0.0, next_goal_distance - current_goal_distance);
			}
			if (line_deviation_enabled) {
				edge_cost += cost_options.box_line_deviation_penalty *
							 distance_to_line(next_rep, start, goal) *
							 std::max(transition_length, 1e-6);
			}
			if (next == goal_cell && goal.size() == static_cast<int>(next_cell.intervals.size())) {
				edge_cost += (next_rep - goal).norm();
				next_rep = goal;
			}
			relax_edge(next, std::move(next_rep), edge_cost, -1);
		}
		const auto overlay_it = overlay_edges_by_cell_.find(item.cell);
		if (overlay_it != overlay_edges_by_cell_.end()) {
			result.adjacency_candidates += static_cast<int>(overlay_it->second.size());
			for (const auto& edge : overlay_it->second) {
				if (edge.target_cell < 0 ||
					edge.target_cell >= static_cast<int>(cells_.size())) {
					continue;
				}
				const auto& next_cell = cells_[static_cast<std::size_t>(edge.target_cell)];
				Eigen::VectorXd next_rep = box_center(next_cell.intervals);
				double edge_cost = edge.length > 0.0
					? edge.length
					: (current_rep - next_rep).norm();
				if (line_deviation_enabled) {
					edge_cost += cost_options.box_line_deviation_penalty *
								 edge_line_deviation(edge, next_rep) *
								 std::max(edge.length, 1e-6);
				}
				if (partition_counts_as_segment_edge(edge.type) &&
					edge.type != SegmentEdgeType::QueryBridge &&
					edge.validation != SegmentEdgeValidation::ConservativeObbZonotope) {
					edge_cost += 100.0;
				}
				if (partition_counts_as_query_repair_edge(edge.type)) {
					edge_cost += cost_options.query_bridge_penalty;
				}
				if (cost_options.foreign_query_edge_penalty > 0.0 &&
					edge.query_index >= 0 &&
					edge.query_index != cost_options.active_query_index) {
					edge_cost += cost_options.foreign_query_edge_penalty;
				}
				if (edge.strict_audit_required &&
					edge.type == SegmentEdgeType::QueryBridge &&
					edge.validation != SegmentEdgeValidation::CollisionChecked) {
					edge_cost += 1.0e6;
				}
				if (edge.target_cell == goal_cell &&
					goal.size() == static_cast<int>(next_cell.intervals.size())) {
					edge_cost += (next_rep - goal).norm();
					next_rep = goal;
				}
				relax_edge(edge.target_cell,
						   std::move(next_rep),
						   edge_cost,
						   edge.edge_id);
			}
		}
	}
	if (!result.found) {
		result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
		return result;
	}
	std::vector<int> cell_sequence;
	for (int cur = goal_cell;; cur = prev[static_cast<std::size_t>(cur)]) {
		cell_sequence.push_back(cur);
		if (cur == start_cell) {
			break;
		}
		if (cur < 0 || cur >= static_cast<int>(cell_count) ||
			prev[static_cast<std::size_t>(cur)] < 0) {
			result.found = false;
			result.box_sequence.clear();
			result.segment_edge_sequence.clear();
			result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
			return result;
		}
	}
	std::reverse(cell_sequence.begin(), cell_sequence.end());
	result.box_sequence.reserve(cell_sequence.size());
	bool used_overlay = false;
	for (int cell_index : cell_sequence) {
		const auto& cell = cells_[static_cast<std::size_t>(cell_index)];
		if (!cell.grid_aligned) {
			result.non_grid_cells_used += 1;
		}
		result.box_sequence.push_back(cell.box_id);
	}
	result.segment_edge_sequence.reserve(result.box_sequence.size() > 0 ? result.box_sequence.size() - 1 : 0);
	for (std::size_t index = 1; index < cell_sequence.size(); ++index) {
		const int edge_id =
			prev_edge[static_cast<std::size_t>(cell_sequence[index])];
		if (edge_id >= 0) {
			used_overlay = true;
			result.overlay_edges_used += 1;
		}
		result.segment_edge_sequence.push_back(edge_id);
	}
	if (options.shortcut_boxes && !used_overlay) {
		result.box_sequence = shortcut_sequence(result.box_sequence, cell_by_box_id_);
		result.segment_edge_sequence.assign(
			result.box_sequence.size() > 0 ? result.box_sequence.size() - 1 : 0,
			-1);
	}
	auto append_if_new = [&](const Eigen::VectorXd& waypoint) {
		if (result.path.empty() || (result.path.back() - waypoint).norm() > 1e-12) {
			result.path.push_back(waypoint);
		}
	};
	auto cell_for_box = [&](int box_id) -> int {
		const auto it = cell_by_box_id_.find(box_id);
		return it == cell_by_box_id_.end() ? -1 : it->second;
	};
	auto overlay_edge_for_transition = [&](int source_cell,
										   int target_cell,
										   int edge_id) -> const OverlayEdge* {
		if (edge_id < 0) {
			return nullptr;
		}
		const auto it = overlay_edges_by_cell_.find(source_cell);
		if (it == overlay_edges_by_cell_.end()) {
			return nullptr;
		}
		for (const auto& edge : it->second) {
			if (edge.edge_id == edge_id && edge.target_cell == target_cell) {
				return &edge;
			}
		}
		return nullptr;
	};
	append_if_new(start);
	for (std::size_t index = 1; index < result.box_sequence.size(); ++index) {
		const int lhs_cell = cell_for_box(result.box_sequence[index - 1]);
		const int rhs_cell = cell_for_box(result.box_sequence[index]);
		if (lhs_cell < 0 || rhs_cell < 0) {
			continue;
		}
		const int edge_id = index - 1 < result.segment_edge_sequence.size()
			? result.segment_edge_sequence[index - 1]
			: -1;
		if (const OverlayEdge* edge = overlay_edge_for_transition(lhs_cell, rhs_cell, edge_id)) {
			if (edge->waypoints.empty()) {
				append_if_new(box_center(cells_[static_cast<std::size_t>(lhs_cell)].intervals));
				append_if_new(box_center(cells_[static_cast<std::size_t>(rhs_cell)].intervals));
			} else {
				for (const auto& waypoint : edge->waypoints) {
					append_if_new(waypoint);
				}
			}
			continue;
		}
		append_if_new(transition_waypoint_toward_goal(
			cells_[static_cast<std::size_t>(lhs_cell)].intervals,
			cells_[static_cast<std::size_t>(rhs_cell)].intervals,
			result.path.back(),
			goal,
			options.adjacency_tolerance));
	}
	append_if_new(goal);
	result.total_cost = dist[static_cast<std::size_t>(goal_cell)];
	result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
	return result;
}

}  // namespace rbf
