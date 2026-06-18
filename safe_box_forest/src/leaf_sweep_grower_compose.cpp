#include <SBF/leaf_sweep_grower.h>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace rbf {
namespace {

bool valid_oracle_node(OracleNodeId node) {
	return node >= 0;
}

void add_counter(LeafSweepResult& result,
				 StageContext& context,
				 const std::string& key,
				 double value = 1.0) {
	context.diagnostics().add_counter(key, value);
	result.diagnostics[key] += value;
}

Eigen::VectorXd center_of_intervals(const std::vector<Interval>& intervals) {
	Eigen::VectorXd center(static_cast<int>(intervals.size()));
	for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
		center[dim] = intervals[static_cast<std::size_t>(dim)].center();
	}
	return center;
}

BoxNode make_box_from_intervals(const std::vector<Interval>& intervals,
								OracleNodeId node,
								int id,
								BoxSafetyStatus status,
								bool strict_audit_required = false) {
	BoxNode box;
	box.id = id;
	box.joint_intervals = intervals;
	box.seed_config = center_of_intervals(box.joint_intervals);
	box.tree_id = node;
	box.parent_box_id = -1;
	box.root_id = id;
	box.safety_status = status;
	box.strict_audit_required = strict_audit_required;
	box.compute_volume();
	return box;
}

bool clip_intervals_to_domain(std::vector<Interval>& intervals,
							  const std::vector<Interval>& domain) {
	if (domain.empty()) {
		return !intervals.empty();
	}
	if (intervals.size() != domain.size()) {
		return false;
	}
	for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
		intervals[dim].lo = std::max(intervals[dim].lo, domain[dim].lo);
		intervals[dim].hi = std::min(intervals[dim].hi, domain[dim].hi);
		if (intervals[dim].lo > intervals[dim].hi) {
			return false;
		}
	}
	return true;
}

}  // namespace

void LeafSweepGrower::compose_final_sets(const std::vector<GroupWork>& groups,
										 int start_depth,
										 int max_depth,
										 StageContext& context,
										 LeafSweepResult& result) const {
	if (groups.size() > 64) {
		throw std::invalid_argument("LeafSweepGrower compose currently supports up to 64 obstacle groups");
	}
	std::vector<std::unordered_set<OracleNodeId>> free_sets;
	std::vector<std::unordered_set<OracleNodeId>> collision_sets;
	free_sets.reserve(groups.size());
	collision_sets.reserve(groups.size());
	for (const auto& group : groups) {
		free_sets.emplace_back(group.free_nodes.begin(), group.free_nodes.end());
		collision_sets.emplace_back(group.collision_nodes.begin(), group.collision_nodes.end());
	}
	const std::uint64_t all_free_mask =
		groups.size() == 64 ? ~std::uint64_t{0}
							: ((std::uint64_t{1} << groups.size()) - std::uint64_t{1});
	auto obstacle_indices_for_mask = [&](std::uint64_t mask) {
		std::vector<int> indices;
		for (std::size_t group_index = 0; group_index < groups.size(); ++group_index) {
			const std::uint64_t bit = std::uint64_t{1} << group_index;
			if ((mask & bit) == 0) {
				continue;
			}
			const auto& group_indices = groups[group_index].result.obstacle_indices;
			indices.insert(indices.end(), group_indices.begin(), group_indices.end());
		}
		std::sort(indices.begin(), indices.end());
		indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
		return indices;
	};
	auto push_collision_box = [&](const BoxNode& box, std::uint64_t blocker_mask) {
		result.collision_boxes.push_back(box);
		result.collision_box_obstacle_indices.push_back(obstacle_indices_for_mask(blocker_mask));
	};
	const auto planning_domain = oracle_.planning_intervals();
	auto make_clipped_box = [&](OracleNodeId node,
								int id,
								BoxSafetyStatus status) -> std::optional<BoxNode> {
		auto intervals = oracle_.node_intervals(node);
		if (!clip_intervals_to_domain(intervals, planning_domain)) {
			add_counter(result, context, "leaf_sweep.valid_domain_pruned_boxes");
			return std::nullopt;
		}
		return make_box_from_intervals(intervals, node, id, status);
	};

	struct ComposeNode {
		OracleNodeId node = kInvalidOracleNodeId;
		int changed_dim = -1;
		std::uint64_t free_mask = 0;
		std::uint64_t collision_mask = 0;
	};

	std::queue<ComposeNode> queue;
	queue.push({oracle_.root_node(), -1, 0, 0});
	int next_free_id = 0;
	int next_collision_id = 0;
	while (!queue.empty()) {
		if (context.should_stop()) {
			result.deadline_reached = context.deadline().expired();
			add_counter(result, context, "leaf_sweep.compose_deadline");
			break;
		}
		ComposeNode item = queue.front();
		queue.pop();
		if (!valid_oracle_node(item.node)) {
			continue;
		}
		for (std::size_t group_index = 0; group_index < groups.size(); ++group_index) {
			const std::uint64_t bit = std::uint64_t{1} << group_index;
			if ((item.collision_mask & bit) == 0 &&
				collision_sets[group_index].find(item.node) != collision_sets[group_index].end()) {
				item.collision_mask |= bit;
			}
			if ((item.free_mask & bit) == 0 &&
				free_sets[group_index].find(item.node) != free_sets[group_index].end()) {
				item.free_mask |= bit;
			}
		}
		const int depth = oracle_.depth(item.node);
		if (depth < start_depth) {
			if (oracle_.is_leaf(item.node)) {
				add_counter(result, context, "leaf_sweep.compose_shallow_leaf_collision");
				const std::uint64_t blocker_mask = item.collision_mask | (all_free_mask & ~item.free_mask);
				if (auto box = make_clipped_box(item.node,
												next_collision_id,
												BoxSafetyStatus::Occupied)) {
					++next_collision_id;
					push_collision_box(*box, blocker_mask);
				}
				continue;
			}
			const int split_dim = oracle_.split_dim(item.node);
			queue.push({oracle_.left_child(item.node), split_dim, item.free_mask, item.collision_mask});
			queue.push({oracle_.right_child(item.node), split_dim, item.free_mask, item.collision_mask});
			continue;
		}

		if (item.collision_mask != 0) {
			if (auto box = make_clipped_box(item.node,
											next_collision_id,
											BoxSafetyStatus::Occupied)) {
				++next_collision_id;
				push_collision_box(*box, item.collision_mask);
			}
			continue;
		}

		if (item.free_mask == all_free_mask) {
			if (auto box = make_clipped_box(item.node,
											next_free_id,
											BoxSafetyStatus::CertifiedFree)) {
				++next_free_id;
				result.free_boxes.push_back(*box);
			}
			continue;
		}

		if (depth >= max_depth || oracle_.is_leaf(item.node)) {
			add_counter(result, context, "leaf_sweep.compose_unclassified_collision");
			const std::uint64_t blocker_mask = item.collision_mask | (all_free_mask & ~item.free_mask);
			if (auto box = make_clipped_box(item.node,
											next_collision_id,
											BoxSafetyStatus::Occupied)) {
				++next_collision_id;
				push_collision_box(*box, blocker_mask);
			}
			continue;
		}

		const int split_dim = oracle_.split_dim(item.node);
		queue.push({oracle_.left_child(item.node), split_dim, item.free_mask, item.collision_mask});
		queue.push({oracle_.right_child(item.node), split_dim, item.free_mask, item.collision_mask});
	}
}

}  // namespace rbf
