#include <SBF/adaptive_grid_partition.h>

#include "adaptive_grid_partition_geometry.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

using partition_detail::interval_box_subset;
using partition_detail::interval_boxes_connected;

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

}  // namespace rbf
