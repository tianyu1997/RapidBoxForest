#include <SBF/leaf_sweep_grower.h>
#include <SBF/oracle.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace rbf {

int LeafSweepGrower::virtual_depth(OracleNodeId node) const {
	if (node <= 0) {
		return 0;
	}
	std::uint64_t value = static_cast<std::uint64_t>(node) + 1u;
	int depth = -1;
	while (value != 0u) {
		value >>= 1u;
		++depth;
	}
	return std::max(0, depth);
}

bool LeafSweepGrower::virtual_split_node(const PendingNode& item,
										 int depth,
										 PendingNode& left,
										 PendingNode& right) const {
	if (item.node < 0 || item.intervals.empty()) {
		return false;
	}
	const auto& descriptor = oracle_.database().split_policy_descriptor();
	int split_dim = -1;
	if (!descriptor.depth_dimensions.empty() &&
		depth >= 0 &&
		depth < static_cast<int>(descriptor.depth_dimensions.size())) {
		split_dim = descriptor.depth_dimensions[static_cast<std::size_t>(depth)];
	} else if (!item.intervals.empty()) {
		split_dim = depth % static_cast<int>(item.intervals.size());
	}
	if (split_dim < 0 || split_dim >= static_cast<int>(item.intervals.size())) {
		return false;
	}
	const auto dim = static_cast<std::size_t>(split_dim);
	const double split_value = item.intervals[dim].center();
	if (!(split_value > item.intervals[dim].lo && split_value < item.intervals[dim].hi)) {
		return false;
	}
	left.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 1u);
	left.changed_dim = split_dim;
	left.intervals = item.intervals;
	left.intervals[dim].hi = split_value;
	right.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 2u);
	right.changed_dim = split_dim;
	right.intervals = item.intervals;
	right.intervals[dim].lo = split_value;
	return true;
}

}  // namespace rbf
