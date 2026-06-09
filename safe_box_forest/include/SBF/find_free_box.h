#pragma once

#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <functional>
#include <vector>

namespace rbf {

enum class FindFreeBoxSearchMode {
	Linear = 0,
	BinaryDepth = 1,
};

struct FindFreeBoxOptions {
	int max_depth = 64;
	/// Minimum depth considered by depth-search modes. Kept as an alias-friendly
	/// complement to skip_to_depth for restored BinaryDepth experiments.
	int start_depth = 0;
	/// Leaf nodes at depth < skip_to_depth are always Unknown; bypass validate_node
	/// for them and split directly.  Should match lect_build_policy.skip_top_depth.
	int skip_to_depth = 0;
	FindFreeBoxSearchMode search_mode = FindFreeBoxSearchMode::Linear;
	/// Optional scheduled-depth checkpoints for a single incremental descent.
	/// Empty means use search_mode with max_depth only. Values are sanitized and
	/// capped by max_depth.
	std::vector<int> adaptive_depths;
	bool split_unknown_leaf = true;
	bool split_reserved_leaf = true;
	bool reject_seed_collision = false;
	bool record_diagnostics = true;
	/// Skip the forest-level "seed already covered by an existing box" scan.
	/// Only enable this when the caller has already checked coverage against the
	/// current local box set/index immediately before invoking FFB.
	bool skip_existing_cover_check = false;
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
	using AcceptCandidate = std::function<bool(const FindFreeBoxResult&)>;
	explicit FindFreeBoxService(BoxOracle& oracle) : oracle_(oracle) {}
	FindFreeBoxResult find(const Eigen::Ref<const Eigen::VectorXd>& seed,
						   const FindFreeBoxOptions& options = {});
	FindFreeBoxResult find(const Eigen::Ref<const Eigen::VectorXd>& seed,
						   StageContext& context,
						   const FindFreeBoxOptions& options = {});
	FindFreeBoxResult find_incremental(const Eigen::Ref<const Eigen::VectorXd>& seed,
										StageContext& context,
										const FindFreeBoxOptions& options,
										const AcceptCandidate& accept);

private:
	BoxOracle& oracle_;
};

}  // namespace rbf
