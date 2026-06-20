#include <SBF/safe_box_forest.h>
#include <SBF/adaptive_grid_partition.h>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_merge.h"

#include <chrono>
#include <unordered_set>

namespace rbf {

double RBFPlanningForest::initialize_adaptive_build_topology(
    const AdaptiveLeafSweepConfig& adaptive_config,
    const AdaptiveLeafSweepConfig& partition_config,
    double adjacency_tolerance,
    AdaptiveLeafSweepResult& out,
    BudgetedMergeStats& merge_stats,
    AdjacencyBuildStats& initial_adjacency_stats,
    std::unordered_set<int>& main_ids,
    bool use_partition_backend) {
    using Clock = std::chrono::steady_clock;
    const auto merge_start = Clock::now();
    if (config_.enable_merger && !boxes_.empty()) {
        bool merged_by_partition = false;
        if (use_partition_backend) {
            rebuild_adaptive_partition(partition_config, nullptr);
            if (adaptive_partition_query_enabled_ && adaptive_partition_) {
                AdaptiveGridPartitionMergeOptions options;
                options.max_ms = adaptive_config.max_merge_ms;
                options.max_rounds = adaptive_config.max_merge_rounds;
                options.grid_line_merge = true;
                options.containment_prune = false;
                const auto partition_merge =
                    adaptive_partition_->merge_boxes(boxes_, options, adjacency_tolerance);
                for (int released_id : partition_merge.released_box_ids) {
                    oracle_->release_box(released_id);
                }
                merge_stats.input_boxes = partition_merge.input_boxes;
                merge_stats.output_boxes = partition_merge.output_boxes;
                merge_stats.grid_merges = partition_merge.grid_merges;
                merge_stats.grid_rounds = partition_merge.rounds;
                merge_stats.containment_pruned = partition_merge.containment_pruned;
                merge_stats.stop_reason = partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                merge_stats.total_ms = partition_merge.total_ms;
                merge_stats.grid_ms = partition_merge.total_ms;
                out.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                out.diagnostics["adaptive.partition_merge_released_boxes"] =
                    static_cast<double>(partition_merge.released_box_ids.size());
                out.diagnostics["adaptive.partition_merge_containment_skipped"] =
                    static_cast<double>(partition_merge.containment_skipped);
                out.diagnostics["adaptive.partition_merge_containment_bucket_entries"] =
                    static_cast<double>(partition_merge.containment_bucket_entries);
                out.diagnostics["adaptive.partition_merge_containment_candidates"] =
                    static_cast<double>(partition_merge.containment_candidates);
                out.diagnostics["adaptive.partition_merge_containment_tests"] =
                    static_cast<double>(partition_merge.containment_tests);
                out.diagnostics["adaptive.partition_merge_containment_overflow"] =
                    static_cast<double>(partition_merge.containment_overflow);
                out.diagnostics["adaptive.partition_merge_containment_ms"] =
                    partition_merge.containment_ms;
                out.diagnostics["adaptive.partition_merge_line_ms"] =
                    partition_merge.line_merge_ms;
                merged_by_partition = true;
            }
        }
        if (!merged_by_partition) {
            MergerConfig leaf_merge_config = config_.merger;
            leaf_merge_config.containment_prune = true;
            merge_stats = budgeted_leaf_merge(*oracle_,
                                              boxes_,
                                              leaf_merge_config,
                                              adaptive_config.max_merge_ms,
                                              adaptive_config.max_merge_rounds,
                                              adaptive_config.max_merge_input_boxes,
                                              adjacency_tolerance);
        }
        raw_boxes_ = boxes_;
    } else {
        merge_stats.input_boxes = static_cast<int>(boxes_.size());
        merge_stats.output_boxes = static_cast<int>(boxes_.size());
        merge_stats.stop_reason = 0;
    }
    const double merge_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - merge_start).count();

    auto refresh_main_from_partition = [&]() -> bool {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
            return false;
        }
        const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
        main_ids.clear();
        main_ids.insert(largest.begin(), largest.end());
        return !main_ids.empty();
    };
    if (use_partition_backend) {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
            rebuild_adaptive_partition(partition_config, nullptr);
        }
        if (refresh_main_from_partition()) {
            out.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
        }
    }
    if (!use_partition_backend) {
        rebuild_adjacency();
        initial_adjacency_stats = last_adjacency_build_stats();
        main_ids = adaptive_largest_island_ids(adjacency_);
    } else if (main_ids.empty()) {
        out.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
    }
    return merge_ms;
}

}  // namespace rbf
