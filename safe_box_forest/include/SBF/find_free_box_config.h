#pragma once

#include <LECTDatabase/sbf/oracle_types.h>

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
	FindFreeBoxSearchMode search_mode = FindFreeBoxSearchMode::BinaryDepth;
	/// Optional first probe depth for BinaryDepth search. <0 selects the
	/// default midpoint probe between start_depth/skip_to_depth and max_depth.
	int binary_probe_depth = -1;
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
	/// When BinaryDepth succeeds through virtual sparse validation, return the
	/// certified interval without materializing the heap-style ancestor chain.
	/// The result node is kInvalidOracleNodeId; callers must key duplicates by
	/// intervals rather than node id. Default keeps legacy reservation semantics.
	bool materialize_result_node = true;
	double deadline_ms = 0.0;
	OracleSplitOptions split;
};

}  // namespace rbf
