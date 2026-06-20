#pragma once

#include <rbf/core.h>

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

}  // namespace rbf
