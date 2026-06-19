#include <SBF/adaptive_grid_partition.h>

#include "adaptive_grid_partition_geometry.h"
#include "adaptive_grid_partition_options.h"
#include "query_graph_cost_options.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

namespace {

using partition_detail::box_center;
using partition_detail::box_volume_from_intervals;
using partition_detail::closest_cells_to_hull;
using partition_detail::closest_point_in_intervals;
using partition_detail::closest_points_between_interval_boxes;
using partition_detail::distance_to_line;
using partition_detail::interval_boxes_connected;
using partition_detail::interval_hull_for_cells;
using partition_detail::intervals_contain_point;
using partition_detail::partition_counts_as_query_repair_edge;
using partition_detail::partition_counts_as_segment_edge;
using partition_detail::point_to_box_distance_sq;
using partition_detail::transition_waypoint_toward_goal;

}  // namespace

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

}  // namespace rbf
