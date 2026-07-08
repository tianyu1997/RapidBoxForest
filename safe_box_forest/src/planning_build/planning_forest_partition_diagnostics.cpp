#include <SBF/safe_box_forest.h>

#include <SBF/adaptive_grid_partition.h>

#include <algorithm>

namespace rbf {

void RBFPlanningForest::refresh_adaptive_partition_diagnostics(BuildProfile* profile) const {
    if (profile == nullptr) {
        return;
    }
    profile->diagnostics["adaptive.offline_backend_grid_partition"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    profile->diagnostics["adaptive.online_backend_partition_native"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    if (has_adaptive_partition_config_) {
        const int target_depth = last_adaptive_partition_config_.grid_target_depth > 0
            ? last_adaptive_partition_config_.grid_target_depth
            : last_adaptive_partition_config_.target_max_depth;
        profile->diagnostics["adaptive.grid_target_depth"] = static_cast<double>(target_depth);
        profile->diagnostics["adaptive.grid_face_index_enabled"] =
            last_adaptive_partition_config_.grid_face_index_enabled ? 1.0 : 0.0;
        profile->diagnostics["adaptive.grid_planning_max_expansions"] =
            static_cast<double>(std::max(0, last_adaptive_partition_config_.grid_planning_max_expansions));
    }
    if (adaptive_partition_query_enabled_ && adaptive_partition_) {
        const auto& stats = adaptive_partition_->stats();
        profile->diagnostics["adaptive.partition_cells"] = static_cast<double>(stats.cells);
        profile->diagnostics["adaptive.partition_grid_cells"] = static_cast<double>(stats.grid_cells);
        profile->diagnostics["adaptive.partition_non_grid_cells"] = static_cast<double>(stats.non_grid_cells);
        profile->diagnostics["adaptive.partition_face_index_entries"] =
            static_cast<double>(stats.face_index_entries);
        profile->diagnostics["adaptive.partition_point_index_dims"] =
            static_cast<double>(stats.point_index_dims);
        profile->diagnostics["adaptive.partition_point_index_entries"] =
            static_cast<double>(stats.point_index_entries);
        profile->diagnostics["adaptive.partition_point_index_overflow_cells"] =
            static_cast<double>(stats.point_index_overflow_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_cells"] =
            static_cast<double>(stats.sparse_virtual_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_grid_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_non_grid_cells"] =
            static_cast<double>(stats.sparse_virtual_non_grid_cells);
        profile->diagnostics["adaptive.partition_sparse_virtual_exact_index_entries"] =
            static_cast<double>(stats.sparse_virtual_exact_index_entries);
        profile->diagnostics["adaptive.partition_sparse_virtual_max_address_depth"] =
            static_cast<double>(stats.sparse_virtual_max_address_depth);
        profile->diagnostics["adaptive.partition_sparse_virtual_ancestor_refs_avoided"] =
            static_cast<double>(stats.sparse_virtual_ancestor_refs_avoided);
        profile->diagnostics["adaptive.partition_sparse_virtual_index_ms"] =
            stats.sparse_virtual_index_ms;
        profile->diagnostics["adaptive.partition_islands"] = static_cast<double>(stats.islands);
        profile->diagnostics["adaptive.partition_largest_island"] =
            static_cast<double>(stats.largest_island);
        profile->diagnostics["adaptive.partition_adjacency_candidates"] =
            static_cast<double>(stats.adjacency_candidates);
        profile->diagnostics["adaptive.partition_adjacency_tests"] =
            static_cast<double>(stats.adjacency_tests);
        profile->diagnostics["adaptive.partition_adjacency_edges"] =
            static_cast<double>(stats.adjacency_edges);
        profile->diagnostics["adaptive.partition_overlay_edges"] =
            static_cast<double>(stats.overlay_edges);
        profile->diagnostics["adaptive.partition_build_ms"] = stats.build_ms;
        profile->diagnostics["adaptive.partition_index_rebuild_ms"] = stats.index_rebuild_ms;
        profile->diagnostics["adaptive.partition_face_index_ms"] = stats.face_index_ms;
        profile->diagnostics["adaptive.partition_point_index_ms"] = stats.point_index_ms;
        profile->diagnostics["adaptive.partition_neighbor_cache_ms"] = stats.neighbor_cache_ms;
        profile->diagnostics["adaptive.partition_island_rebuild_ms"] = stats.island_rebuild_ms;
    }
}

}  // namespace rbf
