#include <SBF/adaptive_grid_partition.h>

#include "adaptive_grid_partition_geometry.h"
#include "adaptive_grid_partition_options.h"

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
using partition_detail::distance_to_line;
using partition_detail::partition_counts_as_query_repair_edge;
using partition_detail::partition_counts_as_segment_edge;
using partition_detail::transition_waypoint_toward_goal;

}  // namespace

std::vector<int> AdaptiveGridPartition::shortcut_sequence(
	const std::vector<int>& sequence,
	const std::unordered_map<int, int>& cell_by_box_id,
	const QueryShortcutCostOptions& shortcut_options) const {
	if (sequence.size() <= 2) {
		return sequence;
	}
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
	const QueryGraphCostOptions cost_options = options.graph_cost;
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
		result.box_sequence = shortcut_sequence(result.box_sequence,
												cell_by_box_id_,
												options.shortcut_cost);
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
