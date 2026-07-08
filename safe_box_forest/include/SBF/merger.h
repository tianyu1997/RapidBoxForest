#pragma once

#include <SBF/merger_types.h>
#include <SBF/runtime_fwd.h>

#include <LECTDatabase/sbf/oracle_types.h>

#include <rbf/core.h>

#include <unordered_set>
#include <vector>

namespace rbf {

class Consolidator {
public:
	Consolidator(BoxOracle& oracle, MergerConfig config = {});
	MergerResult run(std::vector<BoxNode>& boxes,
					 const std::unordered_set<int>& protected_ids = {});
	MergerResult run(std::vector<BoxNode>& boxes,
					 StageContext& context,
					 const std::unordered_set<int>& protected_ids = {});

private:
	bool try_exact_merge(std::vector<BoxNode>& boxes, const std::unordered_set<int>& protected_ids);
	bool try_greedy_merge(std::vector<BoxNode>& boxes,
						  StageContext& context,
						  const std::unordered_set<int>& protected_ids,
						  int& greedy_merges);
	std::vector<MergeCandidate> collect_greedy_candidates(
		const std::vector<BoxNode>& boxes,
		StageContext& context,
		const std::unordered_set<int>& protected_ids) const;
	std::vector<MergeValidationResult> validate_candidates(const std::vector<MergeCandidate>& candidates,
														   StageContext& context);
	bool apply_merge_candidate(std::vector<BoxNode>& boxes, const MergeCandidate& candidate);
	int prune_contained(std::vector<BoxNode>& boxes, const std::unordered_set<int>& protected_ids);
	static std::vector<Interval> hull(const BoxNode& lhs, const BoxNode& rhs);
	static bool contains_box(const BoxNode& outer, const BoxNode& inner, double tolerance);

	BoxOracle& oracle_;
	MergerConfig config_;
};

}  // namespace rbf
