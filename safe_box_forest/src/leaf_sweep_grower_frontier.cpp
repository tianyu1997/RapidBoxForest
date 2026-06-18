#include <SBF/leaf_sweep_grower.h>

#include <chrono>
#include <string>
#include <utility>

namespace rbf {
namespace {

using Clock = std::chrono::steady_clock;

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

void record_timing(LeafSweepResult& result,
				   StageContext& context,
				   const std::string& key,
				   double milliseconds) {
	context.diagnostics().record_timing(key, milliseconds);
	result.diagnostics[key + ".total_ms"] += milliseconds;
	result.diagnostics[key + ".count"] += 1.0;
}

bool intervals_overlap_domain(const std::vector<Interval>& intervals,
							  const std::vector<Interval>& domain,
							  double tolerance = 0.0) {
	if (domain.empty()) {
		return true;
	}
	if (intervals.size() != domain.size()) {
		return false;
	}
	for (std::size_t dim = 0; dim < intervals.size(); ++dim) {
		if (intervals[dim].hi < domain[dim].lo - tolerance ||
			intervals[dim].lo > domain[dim].hi + tolerance) {
			return false;
		}
	}
	return true;
}

}  // namespace

std::vector<LeafSweepGrower::PendingNode> LeafSweepGrower::materialize_start_frontier(
	int start_depth,
	int max_depth,
	StageContext& context,
	LeafSweepResult& result) {
	(void)max_depth;
	std::vector<PendingNode> frontier;
	const auto planning_domain = oracle_.planning_intervals();
	auto node_overlaps_planning = [&](const PendingNode& item) {
		if (planning_domain.empty()) {
			return true;
		}
		const auto intervals = item.intervals.empty() ? oracle_.node_intervals(item.node) : item.intervals;
		return intervals_overlap_domain(intervals, planning_domain, 0.0);
	};
	if (config_.use_virtual_topology) {
		for (const auto& intervals : oracle_.native_root_interval_copies()) {
			PendingNode root;
			root.node = oracle_.root_node();
			root.changed_dim = -1;
			root.intervals = intervals;
			if (intervals_overlap_domain(root.intervals, planning_domain, 0.0)) {
				frontier.push_back(std::move(root));
			} else {
				add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
			}
		}
	} else {
		PendingNode root;
		root.node = oracle_.root_node();
		root.changed_dim = -1;
		if (node_overlaps_planning(root)) {
			frontier.push_back(std::move(root));
		} else {
			add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
		}
	}
	for (int depth = 0; depth < start_depth; ++depth) {
		std::vector<PendingNode> next;
		for (const auto& item : frontier) {
			if (context.should_stop()) {
				result.deadline_reached = context.deadline().expired();
				add_counter(result, context, "leaf_sweep.initialize_deadline");
				next.push_back(item);
				continue;
			}
			if (!valid_oracle_node(item.node)) {
				add_counter(result, context, "leaf_sweep.invalid_frontier_nodes");
				continue;
			}
			if (config_.use_virtual_topology) {
				PendingNode left;
				PendingNode right;
				if (!virtual_split_node(item, depth, left, right)) {
					add_counter(result, context, "leaf_sweep.initialize_split_failures");
					next.push_back(item);
					continue;
				}
				add_counter(result, context, "leaf_sweep.initialize_splits");
				if (intervals_overlap_domain(left.intervals, planning_domain, 0.0)) {
					next.push_back(std::move(left));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
				if (intervals_overlap_domain(right.intervals, planning_domain, 0.0)) {
					next.push_back(std::move(right));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
				continue;
			}
			if (oracle_.is_leaf(item.node)) {
				const auto intervals = oracle_.node_intervals(item.node);
				const auto split_start = Clock::now();
				const auto split = oracle_.split_node(item.node, intervals, item.changed_dim, split_options_);
				record_timing(result,
							  context,
							  "profile.oracle.split_node",
							  std::chrono::duration<double, std::milli>(Clock::now() - split_start).count());
				if (!split.split) {
					add_counter(result, context, "leaf_sweep.initialize_split_failures");
					next.push_back(item);
					continue;
				}
				add_counter(result, context, "leaf_sweep.initialize_splits");
			}
			const int split_dim = oracle_.split_dim(item.node);
			const OracleNodeId left = oracle_.left_child(item.node);
			const OracleNodeId right = oracle_.right_child(item.node);
			if (valid_oracle_node(left)) {
				PendingNode child;
				child.node = left;
				child.changed_dim = split_dim;
				if (node_overlaps_planning(child)) {
					next.push_back(std::move(child));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
			}
			if (valid_oracle_node(right)) {
				PendingNode child;
				child.node = right;
				child.changed_dim = split_dim;
				if (node_overlaps_planning(child)) {
					next.push_back(std::move(child));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
			}
		}
		frontier = std::move(next);
		if (frontier.empty()) {
			break;
		}
	}
	return frontier;
}

}  // namespace rbf
