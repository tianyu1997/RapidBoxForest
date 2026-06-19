#pragma once

#include <SBF/connector.h>

#include <Eigen/Core>

#include <vector>

namespace rbf {

/// Result of the isolated chain_pave debug entry. Captures the BiRRT bridge
/// polyline and the boxes chain_pave committed along it, so callers can measure
/// how completely the committed boxes tile the connector segment.
struct DebugChainPaveResult {
	std::vector<Eigen::VectorXd> waypoints;             ///< BiRRT bridge polyline.
	std::vector<std::vector<Interval>> committed_boxes; ///< Intervals of boxes chain_pave added.
	std::vector<std::vector<Interval>> all_boxes;       ///< Intervals of EVERY forest box after gap-fill (committed + reused).
	std::vector<DebugBoundaryFfbFailure> boundary_failures;
	std::vector<Interval> start_box;                    ///< Anchor box intervals.
	std::vector<Interval> goal_box;                     ///< Goal-containing box intervals.
	int start_box_id = -1;
	int goal_box_id = -1;
	int added = 0;
	int fast_gap_fill_ffb_calls = 0;
	double fast_gap_fill_ms = 0.0;
	int boundary_ffb_calls = 0;
	int boundary_commits = 0;
	int boundary_reject_not_free = 0;
	int boundary_reject_non_adjacent = 0;
	int boundary_fail_seed_collision = 0;
	int boundary_fail_depth_cap = 0;
	int boundary_fail_unknown_depth_cap = 0;
	int boundary_fail_reserved_depth_cap = 0;
	int boundary_fail_occupied = 0;
	int boundary_fail_deadline = 0;
	int boundary_fail_out_of_domain = 0;
	int boundary_fail_split = 0;
	int boundary_failed_seed_memoized = 0;
	int boundary_skip_failed_seed = 0;
	int boundary_stall = 0;
	int boundary_target_hits = 0;
	bool bridge_found = false;
	bool audit_passed = false;
};

} // namespace rbf
