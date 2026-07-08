#pragma once

#include <SBF/find_free_box_config.h>

#include <LECTDatabase/sbf/oracle_types.h>

#include <vector>

namespace rbf {

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

}  // namespace rbf
