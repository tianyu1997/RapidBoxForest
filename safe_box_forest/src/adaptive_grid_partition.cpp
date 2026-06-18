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

}  // namespace rbf
