#include <SBF/leaf_sweep_grower.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>

namespace rbf {
namespace {

void set_value(LeafSweepResult& result,
			   StageContext& context,
			   const std::string& key,
			   double value) {
	context.diagnostics().set_value(key, value);
	result.diagnostics[key] = value;
}

Obstacle aggregate_obstacles(const std::vector<Obstacle>& obstacles) {
	if (obstacles.empty()) {
		return {};
	}
	Obstacle aggregate = obstacles.front();
	for (std::size_t i = 1; i < obstacles.size(); ++i) {
		for (int axis = 0; axis < 3; ++axis) {
			aggregate.bounds[axis] = std::min(aggregate.bounds[axis], obstacles[i].bounds[axis]);
			aggregate.bounds[axis + 3] = std::max(aggregate.bounds[axis + 3], obstacles[i].bounds[axis + 3]);
		}
	}
	return aggregate;
}

double aabb_distance(const Obstacle& lhs, const Obstacle& rhs) {
	double distance_sq = 0.0;
	for (int axis = 0; axis < 3; ++axis) {
		double gap = 0.0;
		if (lhs.bounds[axis + 3] < rhs.bounds[axis]) {
			gap = static_cast<double>(rhs.bounds[axis] - lhs.bounds[axis + 3]);
		} else if (rhs.bounds[axis + 3] < lhs.bounds[axis]) {
			gap = static_cast<double>(lhs.bounds[axis] - rhs.bounds[axis + 3]);
		}
		distance_sq += gap * gap;
	}
	return std::sqrt(distance_sq);
}

class UnionFind {
public:
	explicit UnionFind(int n) : parent_(static_cast<std::size_t>(n)), rank_(static_cast<std::size_t>(n), 0) {
		for (int i = 0; i < n; ++i) {
			parent_[static_cast<std::size_t>(i)] = i;
		}
	}

	int find(int item) {
		const std::size_t index = static_cast<std::size_t>(item);
		if (parent_[index] != item) {
			parent_[index] = find(parent_[index]);
		}
		return parent_[index];
	}

	void unite(int lhs, int rhs) {
		int root_lhs = find(lhs);
		int root_rhs = find(rhs);
		if (root_lhs == root_rhs) {
			return;
		}
		if (rank_[static_cast<std::size_t>(root_lhs)] < rank_[static_cast<std::size_t>(root_rhs)]) {
			std::swap(root_lhs, root_rhs);
		}
		parent_[static_cast<std::size_t>(root_rhs)] = root_lhs;
		if (rank_[static_cast<std::size_t>(root_lhs)] == rank_[static_cast<std::size_t>(root_rhs)]) {
			rank_[static_cast<std::size_t>(root_lhs)] += 1;
		}
	}

private:
	std::vector<int> parent_;
	std::vector<int> rank_;
};

}  // namespace

std::vector<LeafSweepGrower::GroupWork> LeafSweepGrower::cluster_obstacles(
	const std::vector<Obstacle>& obstacles,
	LeafSweepResult& result,
	StageContext& context) const {
	std::vector<GroupWork> groups;
	result.obstacle_group_ids.assign(obstacles.size(), -1);
	if (obstacles.empty()) {
		GroupWork group;
		group.result.group_id = 0;
		group.result.aggregate_obstacle = {};
		groups.push_back(std::move(group));
		set_value(result, context, "leaf_sweep.groups", 1.0);
		return groups;
	}

	const int n = static_cast<int>(obstacles.size());
	UnionFind uf(n);
	const double cluster_gap = std::max(0.0, config_.obstacle_cluster_gap);
	for (int i = 0; i < n; ++i) {
		for (int j = i + 1; j < n; ++j) {
			if (aabb_distance(obstacles[static_cast<std::size_t>(i)],
							  obstacles[static_cast<std::size_t>(j)]) <= cluster_gap) {
				uf.unite(i, j);
			}
		}
	}

	std::unordered_map<int, int> root_to_group;
	for (int i = 0; i < n; ++i) {
		const int root = uf.find(i);
		auto it = root_to_group.find(root);
		if (it == root_to_group.end()) {
			const int group_id = static_cast<int>(groups.size());
			it = root_to_group.emplace(root, group_id).first;
			GroupWork group;
			group.result.group_id = group_id;
			groups.push_back(std::move(group));
		}
		const int group_id = it->second;
		result.obstacle_group_ids[static_cast<std::size_t>(i)] = group_id;
		auto& group = groups[static_cast<std::size_t>(group_id)].result;
		group.obstacle_indices.push_back(i);
		group.obstacles.push_back(obstacles[static_cast<std::size_t>(i)]);
	}

	for (auto& group : groups) {
		group.result.aggregate_obstacle = aggregate_obstacles(group.result.obstacles);
	}
	set_value(result, context, "leaf_sweep.groups", static_cast<double>(groups.size()));
	return groups;
}

}  // namespace rbf
