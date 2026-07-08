#include <SBF/safe_box_forest.h>
#include <SBF/adaptive_grid_partition.h>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <unordered_set>
#include <vector>

#include "planning_forest_adaptive_cover_utils.h"
#include "planning_forest_adaptive_diagnostics.h"
#include "planning_forest_adaptive_merge.h"
#include "planning_forest_diagnostics.h"
#include "planning_forest_qroot_helpers.h"
#include "planning_forest_query_utils.h"

namespace rbf {

AdaptiveLeafSweepResult RBFPlanningForest::build_adaptive_fast_virtual_checkpoint_cover(
    const std::vector<Obstacle>& obstacles,
    const AdaptiveLeafSweepConfig& adaptive_config,
    int initial_leaf_depth,
    int adaptive_depth_min,
    int target_leaf_depth,
    LeafSweepConfig leaf_config,
    const AdaptiveLeafSweepConfig& partition_config,
    std::chrono::steady_clock::time_point total_start) {
    using Clock = std::chrono::steady_clock;
    AdaptiveLeafSweepResult out;
    auto next_fast_depth_checkpoint = [&](int depth) {
        const int step = depth < 16 ? 1 : 2;
        return std::min(target_leaf_depth, depth + step);
    };

    std::vector<AdaptiveDepthSnapshot> depth_snapshots;
    AdaptiveLeafSweepResult selected;
    bool have_selected = false;
    double accumulated_leaf_sweep_ms = 0.0;
    auto materialize_fast_checkpoint_candidate = [&](const LeafSweepResult& leaf_result,
                                                     int depth,
                                                     int sweep_count) {
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
        clear_dynamic_collision_cache();
        invalidate_query_cache();
        populate_dynamic_collision_cache(leaf_result, static_cast<int>(obstacles.size()));
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
    };
    int depth = initial_leaf_depth;
    int sweep_count = 0;
    std::vector<int> checkpoint_depths;
    for (int checkpoint = initial_leaf_depth;
         checkpoint <= target_leaf_depth;
         checkpoint = next_fast_depth_checkpoint(checkpoint)) {
        checkpoint_depths.push_back(checkpoint);
        if (checkpoint >= target_leaf_depth) {
            break;
        }
    }
    const auto adaptive_sweep_start = Clock::now();
    LeafSweepConfig checkpoint_leaf_config = leaf_config;
    checkpoint_leaf_config.checkpoint_depths = checkpoint_depths;
    checkpoint_leaf_config.checkpoint_callback = [&](const LeafSweepResult& checkpoint_leaf,
                                                     int checkpoint_depth) {
        depth = checkpoint_depth;
        ++sweep_count;
        accumulated_leaf_sweep_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
        AdaptiveLeafSweepResult candidate =
            materialize_fast_checkpoint_candidate(checkpoint_leaf, depth, sweep_count);
        auto snapshot = adaptive_snapshot_from_fast_candidate(candidate, depth, adaptive_config);
        if (snapshot.readiness_met) {
            snapshot.stop_reason = "coverage_ready";
        } else if (depth >= target_leaf_depth) {
            snapshot.stop_reason = "max_depth";
        } else {
            snapshot.stop_reason = "checkpoint";
        }
        depth_snapshots.push_back(snapshot);

        selected = std::move(candidate);
        selected.selected_leaf_depth = depth;
        have_selected = true;
        return snapshot.readiness_met || depth >= target_leaf_depth;
    };
    out.leaf_sweep = build_leaf_sweep(obstacles,
                                      adaptive_config.shallow_start_depth,
                                      target_leaf_depth,
                                      checkpoint_leaf_config);
    if (!have_selected) {
        ++sweep_count;
        accumulated_leaf_sweep_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - adaptive_sweep_start).count();
        selected = materialize_fast_checkpoint_candidate(out.leaf_sweep,
                                                         target_leaf_depth,
                                                         sweep_count);
        auto snapshot = adaptive_snapshot_from_fast_candidate(selected, target_leaf_depth, adaptive_config);
        snapshot.stop_reason = snapshot.readiness_met ? "coverage_ready" : "max_depth";
        depth_snapshots.push_back(snapshot);
        selected.selected_leaf_depth = target_leaf_depth;
        selected.adaptive_depth_readiness_met = snapshot.readiness_met;
        selected.adaptive_depth_stop_reason = snapshot.stop_reason;
        have_selected = true;
    }
    if (have_selected) {
        for (const auto& [key, value] : out.leaf_sweep.diagnostics) {
            if (key.find("worker_oracle.") != std::string::npos ||
                key.find("external") != std::string::npos ||
                key.find("canonical_frame") != std::string::npos) {
                set_diagnostic_max(selected.profile.diagnostics, key, value);
            }
        }
        const auto& final_snapshot = depth_snapshots.back();
        selected.selected_leaf_depth = final_snapshot.depth;
        selected.adaptive_depth_readiness_met = final_snapshot.readiness_met;
        selected.adaptive_depth_stop_reason = final_snapshot.stop_reason;
        selected.adaptive_depth_snapshots_json =
            adaptive_depth_snapshots_to_json(depth_snapshots);
        selected.seed_probe_box_covered = final_snapshot.covered_count;
        selected.seed_probe_main_accessible =
            final_snapshot.main_accessible_count + final_snapshot.anchor_to_main_count;
        selected.p_box_covered = final_snapshot.p_box_covered;
        selected.p_main_accessible =
            static_cast<double>(selected.seed_probe_main_accessible) /
            static_cast<double>(std::max(1, final_snapshot.free_probe_count));
        selected.p_anchor_to_main_uncovered = final_snapshot.p_anchor_to_main_uncovered;
        selected.total_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
        selected.profile.total_ms = selected.total_ms;
        selected.profile.grow_ms = selected.total_ms;
        selected.profile.diagnostics["adaptive.in_sweep_checkpoints"] =
            static_cast<double>(sweep_count);
        selected.profile.diagnostics["adaptive.fast_checkpoint_mode"] = 1.0;
        selected.profile.diagnostics["adaptive.fast_virtual_checkpoint_mode"] = 1.0;
        selected.profile.diagnostics["adaptive.terminal_controller_enabled"] = 0.0;
        selected.profile.diagnostics["adaptive.in_sweep_checkpoint_mode"] = 1.0;
        selected.profile.diagnostics["adaptive.selected_leaf_depth"] =
            static_cast<double>(selected.selected_leaf_depth);
        selected.profile.diagnostics["adaptive.depth_readiness_met"] =
            selected.adaptive_depth_readiness_met ? 1.0 : 0.0;
        selected.profile.diagnostics["adaptive.depth_enabled"] = 1.0;
        selected.profile.diagnostics["adaptive.depth_min"] = static_cast<double>(adaptive_depth_min);
        selected.profile.diagnostics["adaptive.depth_max"] = static_cast<double>(target_leaf_depth);
        record_depth_semantics_diagnostics(selected.profile.diagnostics,
                                           "adaptive.",
                                           adaptive_config.shallow_start_depth,
                                           initial_leaf_depth,
                                           target_leaf_depth,
                                           config_.grower.find_free_box,
                                           target_leaf_depth);
        if (oracle_) {
            const OracleCounters counters = oracle_->counters();
            normalize_external_evidence_diagnostics(selected.profile.diagnostics, &counters);
        } else {
            normalize_external_evidence_diagnostics(selected.profile.diagnostics);
        }
        record_portal_membership_policy(selected.profile.diagnostics, config_.portal_membership_policy);
        selected.diagnostics = selected.profile.diagnostics;
        last_build_ = selected.profile;
        if (config_.database.checkpoint_after_build && database_) {
            database_->checkpoint();
        }
        return selected;
    }
    
    return out;
}

}  // namespace rbf
