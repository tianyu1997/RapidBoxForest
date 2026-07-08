#include <SBF/adaptive_grid_partition.h>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

#include <rbf/lect_database/split_policy.h>

namespace rbf {
namespace {

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

}  // namespace

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

}  // namespace rbf
