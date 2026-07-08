#pragma once

#include <SBF/leaf_sweep_types.h>
#include <SBF/runtime_fwd.h>
#include <LECTDatabase/sbf/oracle_types.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

namespace rbf {

using LeafSweepClock = std::chrono::steady_clock;
using Clock = LeafSweepClock;

inline bool valid_oracle_node(OracleNodeId node) {
	return node >= 0;
}

inline double elapsed_ms(LeafSweepClock::time_point start) {
	return std::chrono::duration<double, std::milli>(LeafSweepClock::now() - start).count();
}

void add_counter(LeafSweepResult& result,
				 StageContext& context,
				 const std::string& key,
				 double value = 1.0);

void set_value(LeafSweepResult& result,
			   StageContext& context,
			   const std::string& key,
			   double value);

void record_timing(LeafSweepResult& result,
				   StageContext& context,
				   const std::string& key,
				   double milliseconds);

inline Eigen::VectorXd center_of_intervals(const std::vector<Interval>& intervals) {
	Eigen::VectorXd center(static_cast<int>(intervals.size()));
	for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
		center[dim] = intervals[static_cast<std::size_t>(dim)].center();
	}
	return center;
}

inline BoxNode make_box_from_intervals(const std::vector<Interval>& intervals,
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

inline bool should_prune_by_overlap(const LeafSweepConfig& config,
									int depth,
									const OracleValidationDetail& detail) {
	if (config.collision_overlap_prune_min_depth < 0 ||
		depth < config.collision_overlap_prune_min_depth) {
		return false;
	}
	if (config.collision_overlap_prune_threshold > 0.0) {
		double threshold = config.collision_overlap_prune_threshold;
		const int extra_depth = std::max(0, depth - config.collision_overlap_prune_min_depth);
		if (config.collision_overlap_prune_decay_per_depth > 0.0 && extra_depth > 0) {
			threshold = threshold / (1.0 + config.collision_overlap_prune_decay_per_depth *
												 static_cast<double>(extra_depth));
		}
		if (config.collision_overlap_prune_min_threshold > 0.0) {
			threshold = std::max(config.collision_overlap_prune_min_threshold, threshold);
		}
		if (detail.aabb_overlap_depth >= threshold) {
			return true;
		}
	}
	if (config.collision_overlap_prune_ratio_threshold > 0.0 &&
		detail.aabb_overlap_volume_ratio >= config.collision_overlap_prune_ratio_threshold) {
		return true;
	}
	return false;
}

inline bool intervals_overlap_domain(const std::vector<Interval>& intervals,
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

inline bool clip_intervals_to_domain(std::vector<Interval>& intervals,
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

inline bool intervals_same_exact(const std::vector<Interval>& lhs,
								 const std::vector<Interval>& rhs) {
	if (lhs.size() != rhs.size()) {
		return false;
	}
	for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
		if (lhs[dim].lo != rhs[dim].lo || lhs[dim].hi != rhs[dim].hi) {
			return false;
		}
	}
	return true;
}

class ScopedOracleEnvelopeCache {
public:
	ScopedOracleEnvelopeCache(DatabaseBoxOracle& oracle, bool enabled);

	~ScopedOracleEnvelopeCache();

	ScopedOracleEnvelopeCache(const ScopedOracleEnvelopeCache&) = delete;
	ScopedOracleEnvelopeCache& operator=(const ScopedOracleEnvelopeCache&) = delete;

private:
	DatabaseBoxOracle& oracle_;
	bool previous_ = true;
};

class ScopedOracleFullOverlapStats {
public:
	ScopedOracleFullOverlapStats(DatabaseBoxOracle& oracle, bool enabled);

	~ScopedOracleFullOverlapStats();

	ScopedOracleFullOverlapStats(const ScopedOracleFullOverlapStats&) = delete;
	ScopedOracleFullOverlapStats& operator=(const ScopedOracleFullOverlapStats&) = delete;

private:
	DatabaseBoxOracle& oracle_;
	bool previous_ = false;
};

}  // namespace rbf
