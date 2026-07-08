#pragma once

#include <SBF/box_graph.h>
#include <SBF/merger_types.h>
#include <LECTDatabase/sbf/oracle_types.h>

#include <vector>

namespace rbf {

struct BudgetedMergeStats {
    int input_boxes = 0;
    int output_boxes = 0;
    int grid_merges = 0;
    int grid_rounds = 0;
    int tree_merges = 0;
    int tree_rounds = 0;
    int containment_pruned = 0;
    int exact_merges = 0;
    int rounds = 0;
    int stop_reason = 1;  // 0 skipped, 1 complete, 2 time_budget, 3 input_cap
    double containment_ms = 0.0;
    double grid_ms = 0.0;
    double tree_ms = 0.0;
    double exact_ms = 0.0;
    double total_ms = 0.0;
};

MergerResult fast_exact_face_merge_leaf(BoxOracle& oracle,
                                        std::vector<BoxNode>& boxes,
                                        const MergerConfig& config);

BudgetedMergeStats budgeted_leaf_merge(DatabaseBoxOracle& oracle,
                                       std::vector<BoxNode>& boxes,
                                       MergerConfig config,
                                       double max_merge_ms,
                                       int max_merge_rounds,
                                       int max_merge_input_boxes,
                                       double adjacency_tolerance);

}  // namespace rbf
