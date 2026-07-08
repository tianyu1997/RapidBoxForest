#include <SBF/adaptive_grid_partition.h>

#include "adaptive_grid_partition_geometry.h"
#include "adaptive_grid_partition_options.h"

#include <array>
#include <unordered_set>

namespace rbf {

using partition_detail::choose_bin_width;
using partition_detail::choose_point_index_dims;
using partition_detail::interval_bin;

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
		partition_point_index_dims());
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
	const std::uint64_t max_entries = partition_point_index_max_cell_entries();
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
		const std::uint64_t max_entries = partition_point_index_max_query_entries();
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

}  // namespace rbf
