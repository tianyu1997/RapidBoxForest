#pragma once

#include <SBF/box_graph.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <unordered_set>
#include <vector>

namespace rbf {

struct MergeCandidate {
	int lhs_box_id = -1;
	int rhs_box_id = -1;
	double score = 0.0;
	std::vector<Interval> hull_intervals;
};

struct MergeValidationResult {
	MergeCandidate candidate;
	bool valid = false;
};

struct MergerConfig {
	bool exact_face_merge = true;
	bool greedy_hull_merge = true;
	bool containment_prune = true;
	int max_rounds = 4;
	int target_boxes = 0;
	double score_threshold = 1.25;
	double adjacency_tolerance = 1e-9;
	int n_threads = 1;
	int candidate_batch_size = 0;
	int parallel_threshold = 0;
	bool deterministic_reduce = true;
};

struct MergerResult {
	int boxes_before = 0;
	int boxes_after = 0;
	int rounds = 0;
	int exact_merges = 0;
	int greedy_merges = 0;
	int pruned_boxes = 0;
};

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
