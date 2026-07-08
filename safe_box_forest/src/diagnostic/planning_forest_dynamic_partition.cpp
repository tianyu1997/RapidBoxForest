#include <SBF/safe_box_forest.h>

#include <SBF/adaptive_grid_partition.h>
#include <SBF/diagnostic_result.h>

#include <chrono>
#include <string>
#include <unordered_set>

namespace rbf {

void RBFPlanningForest::refresh_adaptive_partition_diagnostics(RebuildProfile& profile) const {
    profile.diagnostics["adaptive.offline_backend_grid_partition"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    profile.diagnostics["adaptive.online_backend_partition_native"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
        const auto& stats = adaptive_partition_->stats();
        profile.adjacency_islands = adaptive_partition_->component_count_with_overlay();
        profile.diagnostics["adaptive.partition_cells"] = static_cast<double>(stats.cells);
        profile.diagnostics["adaptive.partition_grid_cells"] = static_cast<double>(stats.grid_cells);
        profile.diagnostics["adaptive.partition_non_grid_cells"] = static_cast<double>(stats.non_grid_cells);
        profile.diagnostics["adaptive.partition_face_index_entries"] =
            static_cast<double>(stats.face_index_entries);
        profile.diagnostics["adaptive.partition_point_index_dims"] =
            static_cast<double>(stats.point_index_dims);
        profile.diagnostics["adaptive.partition_point_index_entries"] =
            static_cast<double>(stats.point_index_entries);
        profile.diagnostics["adaptive.partition_point_index_overflow_cells"] =
            static_cast<double>(stats.point_index_overflow_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_cells"] =
            static_cast<double>(stats.sparse_virtual_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_non_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_non_grid_cells);
        profile.diagnostics["adaptive.partition_sparse_virtual_exact_index_entries"] =
            static_cast<double>(stats.sparse_virtual_exact_index_entries);
        profile.diagnostics["adaptive.partition_sparse_virtual_max_address_depth"] =
            static_cast<double>(stats.sparse_virtual_max_address_depth);
        profile.diagnostics["adaptive.partition_sparse_virtual_ancestor_refs_avoided"] =
            static_cast<double>(stats.sparse_virtual_ancestor_refs_avoided);
        profile.diagnostics["adaptive.partition_sparse_virtual_index_ms"] =
            stats.sparse_virtual_index_ms;
        profile.diagnostics["adaptive.partition_islands"] = static_cast<double>(stats.islands);
        profile.diagnostics["adaptive.partition_largest_island"] =
            static_cast<double>(stats.largest_island);
        profile.diagnostics["adaptive.partition_overlay_edges"] =
            static_cast<double>(stats.overlay_edges);
    }
}

void RBFPlanningForest::refresh_dynamic_partition_after_update(RebuildProfile& profile,
                                                               const char* diagnostic_prefix) {
    if (!partition_native_mode()) {
        return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    rebuild_adaptive_partition(last_adaptive_partition_config_, nullptr);
    const int synced_edges = sync_adaptive_partition_segment_edges(nullptr, diagnostic_prefix);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    const std::string prefix = (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
        ? std::string(diagnostic_prefix)
        : std::string("dynamic.partition");
    profile.diagnostics[prefix + ".rebuild_ms"] += ms;
    profile.diagnostics[prefix + ".edges_synced"] += static_cast<double>(synced_edges);
    refresh_adaptive_partition_diagnostics(profile);
    invalidate_query_cache();
}

void RBFPlanningForest::refresh_dynamic_partition_after_append(RebuildProfile& profile,
                                                               std::size_t first_box_index,
                                                               const char* diagnostic_prefix) {
    if (!partition_native_mode()) {
        return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::string prefix = (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
        ? std::string(diagnostic_prefix)
        : std::string("dynamic.partition_append");
    int appended = 0;
    if (first_box_index < boxes_.size()) {
        appended = adaptive_partition_->append_boxes(boxes_,
                                                     first_box_index,
                                                     config_.query.adjacency_tolerance);
    }
    const int synced_edges = sync_adaptive_partition_segment_edges(nullptr, prefix.c_str());
    if (first_box_index < boxes_.size() && appended <= 0) {
        profile.diagnostics[prefix + ".append_failed_rebuilds"] += 1.0;
        refresh_dynamic_partition_after_update(profile,
                                               (prefix + ".rebuild_after_append_failure").c_str());
        return;
    }
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    profile.diagnostics[prefix + ".append_ms"] += ms;
    profile.diagnostics[prefix + ".boxes_appended"] += static_cast<double>(appended);
    profile.diagnostics[prefix + ".edges_synced"] += static_cast<double>(synced_edges);
    refresh_adaptive_partition_diagnostics(profile);
    invalidate_query_cache();
}

void RBFPlanningForest::refresh_dynamic_partition_after_remove_append(
    RebuildProfile& profile,
    const std::unordered_set<int>& removed_box_ids,
    std::size_t first_box_index,
    const char* diagnostic_prefix) {
    if (!partition_native_mode()) {
        return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::string prefix = (diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0')
        ? std::string(diagnostic_prefix)
        : std::string("dynamic.partition_delta");
    const std::size_t expected_appended =
        first_box_index < boxes_.size() ? boxes_.size() - first_box_index : 0u;
    AdaptiveGridPartitionDeltaResult delta;
    if (adaptive_partition_) {
        delta = adaptive_partition_->replace_box_ids_with_boxes(removed_box_ids,
                                                                boxes_,
                                                                first_box_index,
                                                                config_.query.adjacency_tolerance);
    }
    if (!removed_box_ids.empty() &&
        delta.boxes_removed != static_cast<int>(removed_box_ids.size())) {
        profile.diagnostics[prefix + ".remove_failed_rebuilds"] += 1.0;
        profile.diagnostics[prefix + ".boxes_removed_before_rebuild"] +=
            static_cast<double>(delta.boxes_removed);
        refresh_dynamic_partition_after_update(profile,
                                               (prefix + ".rebuild_after_remove_failure").c_str());
        return;
    }
    if (expected_appended > 0u &&
        delta.boxes_appended != static_cast<int>(expected_appended)) {
        profile.diagnostics[prefix + ".append_failed_rebuilds"] += 1.0;
        profile.diagnostics[prefix + ".boxes_appended_before_rebuild"] +=
            static_cast<double>(delta.boxes_appended);
        refresh_dynamic_partition_after_update(profile,
                                               (prefix + ".rebuild_after_append_failure").c_str());
        return;
    }
    const int synced_edges = sync_adaptive_partition_segment_edges(nullptr, prefix.c_str());
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    profile.diagnostics[prefix + ".delta_ms"] += ms;
    profile.diagnostics[prefix + ".replace_ms"] += delta.update_ms;
    profile.diagnostics[prefix + ".boxes_removed"] += static_cast<double>(delta.boxes_removed);
    profile.diagnostics[prefix + ".boxes_appended"] += static_cast<double>(delta.boxes_appended);
    profile.diagnostics[prefix + ".edges_synced"] += static_cast<double>(synced_edges);
    refresh_adaptive_partition_diagnostics(profile);
    invalidate_query_cache();
}

}  // namespace rbf
