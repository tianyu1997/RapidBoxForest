#pragma once

#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <vector>

namespace rbf {

struct FindFreeBoxOptions {
	int max_depth = 64;
	/// Leaf nodes at depth < skip_to_depth are always Unknown; bypass validate_node
	/// for them and split directly.  Should match lect_build_policy.skip_top_depth.
	int skip_to_depth = 0;
	bool split_unknown_leaf = true;
	bool split_reserved_leaf = true;
	bool reject_seed_collision = false;
	double deadline_ms = 0.0;
	OracleSplitOptions split;
};

struct FindFreeBoxResult {
	bool found = false;
	bool seed_collision = false;
	bool hit_reserved_depth_cap = false;
	bool hit_unknown_depth_cap = false;
	bool deadline_reached = false;
	int fail_code = 0;
	OracleNodeId node = kInvalidOracleNodeId;
	int decisions = 0;
	int splits = 0;
	int changed_dim = -1;
	double total_ms = 0.0;
	std::vector<Interval> intervals;
	OracleValidationDetail validation_detail;
};

class FindFreeBoxService {
public:
	explicit FindFreeBoxService(BoxOracle& oracle) : oracle_(oracle) {}
	FindFreeBoxResult find(const Eigen::Ref<const Eigen::VectorXd>& seed,
						   const FindFreeBoxOptions& options = {});
	FindFreeBoxResult find(const Eigen::Ref<const Eigen::VectorXd>& seed,
						   StageContext& context,
						   const FindFreeBoxOptions& options = {});

private:
	BoxOracle& oracle_;
};

}  // namespace rbf
