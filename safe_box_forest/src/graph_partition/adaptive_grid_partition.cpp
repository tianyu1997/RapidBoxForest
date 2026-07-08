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

namespace rbf {

namespace {

using partition_detail::box_volume_from_intervals;
using partition_detail::choose_bin_width;
using partition_detail::choose_point_index_dims;
using partition_detail::interval_bin;
using partition_detail::interval_box_subset;
using partition_detail::interval_boxes_connected;
using partition_detail::max_coord_for_splits;
using partition_detail::packed_pair_key;

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

}  // namespace rbf
