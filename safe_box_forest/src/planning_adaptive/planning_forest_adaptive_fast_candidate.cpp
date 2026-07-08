#include <SBF/safe_box_forest.h>
#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <vector>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_merge.h"
#include "../qroot/planning_forest_qroot_helpers.h"
#include "../query_runtime/planning_forest_query_utils.h"

namespace rbf {

AdaptiveLeafSweepResult RBFPlanningForest::materialize_adaptive_fast_checkpoint_candidate(
    const std::vector<Obstacle>& obstacles,
    const AdaptiveLeafSweepConfig& adaptive_config,
    int target_leaf_depth,
    const AdaptiveLeafSweepConfig& partition_config,
    std::chrono::steady_clock::time_point total_start,
    const LeafSweepResult& leaf_result,
    int depth,
    int sweep_count,
    double accumulated_leaf_sweep_ms) {
    using Clock = std::chrono::steady_clock;
    AdaptiveLeafSweepResult candidate;
    candidate.leaf_sweep = leaf_result;
    candidate.leaf_sweep_ms = accumulated_leaf_sweep_ms;
    candidate.selected_leaf_depth = depth;
    candidate.shallow_free_count = static_cast<int>(leaf_result.free_boxes.size());
    candidate.shallow_collision_count = static_cast<int>(leaf_result.collision_boxes.size());
    candidate.adaptive_deferred = static_cast<int>(leaf_result.collision_boxes.size());
    candidate.unresolved_domains = static_cast<int>(leaf_result.collision_boxes.size());

    scene_.set_obstacles(obstacles);
    if (oracle_) {
        oracle_->set_scene(scene_);
    }
	boxes_ = leaf_result.free_boxes;
	raw_boxes_ = boxes_;
	adjacency_.clear();
	segment_edges_.clear();
	clear_optional_collision_cache();
	invalidate_query_cache();
	populate_optional_collision_cache_from_leaf_sweep(leaf_result, static_cast<int>(obstacles.size()));
	reserve_existing_boxes();

    const double adjacency_tolerance = config_.query.adjacency_tolerance;
    const auto merge_start = Clock::now();
    BudgetedMergeStats merge_stats;
    if (config_.enable_merger && !boxes_.empty()) {
        bool merged_by_partition = false;
        if (adaptive_config.planning_backend == "partition_native") {
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
                merge_stats.stop_reason =
                    partition_merge.stop_reason == 0 ? 1 : partition_merge.stop_reason;
                merge_stats.total_ms = partition_merge.total_ms;
                merge_stats.grid_ms = partition_merge.total_ms;
                candidate.diagnostics["adaptive.partition_merge_enabled"] = 1.0;
                candidate.diagnostics["adaptive.partition_merge_released_boxes"] =
                    static_cast<double>(partition_merge.released_box_ids.size());
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

    const bool use_partition_backend = adaptive_config.planning_backend == "partition_native";
    AdjacencyBuildStats adjacency_stats;
    std::unordered_set<int> main_ids;
    int partition_island_count_for_profile = 0;
    int partition_largest_island_for_profile = 0;
    if (use_partition_backend) {
        if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
            rebuild_adaptive_partition(partition_config, nullptr);
        }
        if (adaptive_partition_query_enabled_ && adaptive_partition_) {
            const auto largest = adaptive_partition_->largest_component_box_ids_with_overlay();
            main_ids.insert(largest.begin(), largest.end());
            const auto& partition_stats = adaptive_partition_->stats();
            partition_island_count_for_profile = partition_stats.islands;
            partition_largest_island_for_profile = partition_stats.largest_island;
            candidate.diagnostics["adaptive.partition_skipped_graph_adjacency"] = 1.0;
        }
    }
    if (!use_partition_backend) {
        rebuild_adjacency();
        adjacency_stats = last_adjacency_build_stats();
        main_ids = adaptive_largest_island_ids(adjacency_);
    } else if (main_ids.empty()) {
        candidate.diagnostics["adaptive.partition_missing_no_graph_fallback"] = 1.0;
    }

    const auto coverage_start = Clock::now();
    const auto planning_domain = oracle_ ? oracle_->planning_intervals() : std::vector<Interval>{};
    int probe_attempted = 0;
    std::vector<Eigen::VectorXd> free_probes =
        oracle_ ? adaptive_generate_free_probes(*oracle_,
                                                planning_domain,
                                                std::max(0, adaptive_config.adaptive_depth_probe_count),
                                                adaptive_config.adaptive_depth_probe_seed,
                                                probe_attempted)
                : std::vector<Eigen::VectorXd>{};
    candidate.seed_probe_count = probe_attempted;
    candidate.seed_probe_free_count = static_cast<int>(free_probes.size());
    int uncovered_anchor_attempts = 0;
    StageContext probe_context = StageContext::from_runtime(config_.runtime);
    FindFreeBoxOptions probe_options = config_.grower.find_free_box;
    probe_options.max_depth = target_leaf_depth;
    probe_options.reject_seed_collision = false;
    probe_options.deadline_ms = std::max(1.0, adaptive_config.adaptive_depth_max_probe_ms);
    const int anchor_cap = std::max(0, adaptive_config.adaptive_depth_anchor_probe_cap);
    BoxSpatialIndex coverage_index;
    const bool use_partition_coverage =
        use_partition_backend && adaptive_partition_query_enabled_ && adaptive_partition_;
    if (!use_partition_coverage) {
        coverage_index.rebuild(boxes_, adjacency_tolerance);
    }
    for (const auto& point : free_probes) {
        const int owner = use_partition_coverage
            ? adaptive_partition_->locate_containing_box(point, false, adjacency_tolerance)
            : [&]() {
                  const int owner_index = coverage_index.covering_box(boxes_, point, adjacency_tolerance);
                  return owner_index >= 0 ? boxes_[static_cast<std::size_t>(owner_index)].id : -1;
              }();
        if (owner >= 0) {
            candidate.seed_probe_box_covered += 1;
            if (main_ids.find(owner) != main_ids.end()) {
                candidate.seed_probe_main_accessible += 1;
            }
            continue;
        }
        if (uncovered_anchor_attempts >= anchor_cap || planning_domain.empty()) {
            continue;
        }
        ++uncovered_anchor_attempts;
        const auto ffb = find_free_box_in_domain(point, planning_domain, probe_context, probe_options);
        if (!ffb.found) {
            continue;
        }
        candidate.seed_probe_anchor_success += 1;
        const BoxNode anchor = adaptive_make_box_from_intervals(
            ffb.intervals,
            ffb.node,
            -1,
            ffb.validation_detail.safety_status,
            ffb.validation_detail.strict_audit_required);
        const bool anchor_main_accessible = use_partition_coverage
            ? adaptive_partition_->box_adjacent_to_any(anchor, main_ids, adjacency_tolerance)
            : (!use_partition_backend &&
               adaptive_has_adjacency_to_any(boxes_, anchor, &main_ids, adjacency_tolerance));
        if (anchor_main_accessible) {
            candidate.seed_probe_main_accessible += 1;
        }
    }
    candidate.coverage_probe_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - coverage_start).count();
    const double free_den = static_cast<double>(std::max(1, candidate.seed_probe_free_count));
    candidate.p_box_covered = static_cast<double>(candidate.seed_probe_box_covered) / free_den;
    candidate.p_anchor_success = static_cast<double>(candidate.seed_probe_anchor_success) / free_den;
    candidate.p_main_accessible = static_cast<double>(candidate.seed_probe_main_accessible) / free_den;

    candidate.total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    candidate.profile = {};
    candidate.profile.raw_boxes = static_cast<int>(raw_boxes_.size());
    candidate.profile.final_boxes = static_cast<int>(boxes_.size());
    candidate.profile.segment_edges = static_cast<int>(segment_edges_.size());
    candidate.profile.grow_ms = candidate.total_ms;
    candidate.profile.total_ms = candidate.total_ms;
    if (use_partition_backend) {
        candidate.profile.grow_adjacency_islands = partition_island_count_for_profile;
        candidate.profile.adjacency_islands = partition_island_count_for_profile;
        candidate.profile.grow_largest_island = partition_largest_island_for_profile;
    } else {
        const auto graph_islands = find_islands(adjacency_);
        candidate.profile.grow_adjacency_islands = static_cast<int>(graph_islands.size());
        candidate.profile.adjacency_islands = candidate.profile.grow_adjacency_islands;
        for (const auto& island : graph_islands) {
            candidate.profile.grow_largest_island =
                std::max(candidate.profile.grow_largest_island, static_cast<int>(island.size()));
        }
    }
    candidate.profile.diagnostics = leaf_result.diagnostics;
    for (const auto& [key, value] : candidate.diagnostics) {
        candidate.profile.diagnostics[key] = value;
    }
    candidate.profile.diagnostics["adaptive.fast_checkpoint_mode"] = 1.0;
    candidate.profile.diagnostics["adaptive.fast_virtual_checkpoint_mode"] = 1.0;
    candidate.profile.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
    candidate.profile.diagnostics["adaptive.in_sweep_checkpoint_mode"] = 1.0;
    candidate.profile.diagnostics["adaptive.in_sweep_checkpoints"] =
        static_cast<double>(sweep_count);
    candidate.profile.diagnostics["adaptive.offline_query_agnostic_build"] = 1.0;
    candidate.profile.diagnostics["adaptive.qroot_pairs_total"] = 0.0;
    candidate.profile.diagnostics["adaptive.qroot_uncovered_endpoints"] = 0.0;
    candidate.profile.diagnostics["adaptive.leaf_sweep_ms"] = candidate.leaf_sweep_ms;
    candidate.profile.diagnostics["adaptive.merge_ms"] = merge_ms;
    candidate.profile.diagnostics["adaptive.merge_input_boxes"] =
        static_cast<double>(merge_stats.input_boxes);
    candidate.profile.diagnostics["adaptive.merge_output_boxes"] =
        static_cast<double>(merge_stats.output_boxes);
    candidate.profile.diagnostics["adaptive.adjacency_ms"] = adjacency_stats.build_ms;
    candidate.profile.diagnostics["adaptive.adjacency_boxes"] =
        static_cast<double>(adjacency_stats.boxes);
    candidate.profile.diagnostics["adaptive.adjacency_selected_dims"] =
        static_cast<double>(adjacency_stats.selected_dims);
    candidate.profile.diagnostics["adaptive.adjacency_primary_dim"] =
        static_cast<double>(adjacency_stats.primary_dim);
    candidate.profile.diagnostics["adaptive.adjacency_candidates"] =
        static_cast<double>(adjacency_stats.candidate_pairs);
    candidate.profile.diagnostics["adaptive.adjacency_exact_tests"] =
        static_cast<double>(adjacency_stats.exact_tests);
    candidate.profile.diagnostics["adaptive.adjacency_edges"] =
        static_cast<double>(adjacency_stats.edges);
    candidate.profile.diagnostics["adaptive.coverage_probe_ms"] = candidate.coverage_probe_ms;
    candidate.profile.diagnostics["adaptive.total_ms"] = candidate.total_ms;
    candidate.profile.diagnostics["adaptive.shallow_free_count"] =
        static_cast<double>(candidate.shallow_free_count);
    candidate.profile.diagnostics["adaptive.shallow_collision_count"] =
        static_cast<double>(candidate.shallow_collision_count);
    candidate.profile.diagnostics["adaptive.seed_probe_count"] =
        static_cast<double>(candidate.seed_probe_count);
    candidate.profile.diagnostics["adaptive.seed_probe_free_count"] =
        static_cast<double>(candidate.seed_probe_free_count);
    candidate.profile.diagnostics["adaptive.seed_probe_box_covered"] =
        static_cast<double>(candidate.seed_probe_box_covered);
    candidate.profile.diagnostics["adaptive.seed_probe_anchor_success"] =
        static_cast<double>(candidate.seed_probe_anchor_success);
    candidate.profile.diagnostics["adaptive.seed_probe_main_accessible"] =
        static_cast<double>(candidate.seed_probe_main_accessible);
    candidate.profile.diagnostics["adaptive.seed_anchor_probe_cap"] =
        static_cast<double>(anchor_cap);
    candidate.profile.diagnostics["adaptive.seed_anchor_probe_attempts"] =
        static_cast<double>(uncovered_anchor_attempts);
    candidate.profile.diagnostics["adaptive.p_box_covered"] = candidate.p_box_covered;
    candidate.profile.diagnostics["adaptive.p_anchor_success"] = candidate.p_anchor_success;
    candidate.profile.diagnostics["adaptive.p_main_accessible"] = candidate.p_main_accessible;
    rebuild_adaptive_partition(partition_config, &candidate.profile);
    if (adaptive_partition_ && !adaptive_partition_->empty()) {
        const auto& partition_stats = adaptive_partition_->stats();
        candidate.partition_cell_count = partition_stats.cells;
        candidate.partition_grid_cell_count = partition_stats.grid_cells;
        candidate.partition_non_grid_cell_count = partition_stats.non_grid_cells;
        candidate.partition_face_index_entries = partition_stats.face_index_entries;
        candidate.partition_islands = partition_stats.islands;
        candidate.partition_largest_island = partition_stats.largest_island;
        candidate.profile.grow_adjacency_islands = partition_stats.islands;
        candidate.profile.adjacency_islands = partition_stats.islands;
        candidate.profile.grow_largest_island = partition_stats.largest_island;
    }
    candidate.diagnostics = candidate.profile.diagnostics;
    return candidate;
}

}  // namespace rbf
