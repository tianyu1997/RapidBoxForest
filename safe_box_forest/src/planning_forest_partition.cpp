#include <SBF/safe_box_forest.h>

#include <SBF/box_graph.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>

#include "planning_forest_query_utils.h"

namespace rbf {

namespace {

bool graph_has_box_path_local(const AdjacencyGraph& graph, int start_id, int goal_id) {
    if (start_id < 0 || goal_id < 0) {
        return false;
    }
    if (start_id == goal_id) {
        return true;
    }
    std::queue<int> queue;
    std::unordered_set<int> visited;
    queue.push(start_id);
    visited.insert(start_id);
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        auto it = graph.find(current);
        if (it == graph.end()) {
            continue;
        }
        for (int next : it->second) {
            if (next == goal_id) {
                return true;
            }
            if (visited.insert(next).second) {
                queue.push(next);
            }
        }
    }
    return false;
}

bool graph_has_certified_box_path_local(const std::vector<BoxNode>& boxes,
                                        const AdjacencyGraph& graph,
                                        int start_id,
                                        int goal_id,
                                        double adjacency_tolerance) {
    if (start_id < 0 || goal_id < 0) {
        return false;
    }
    if (start_id == goal_id) {
        return true;
    }
    std::queue<int> queue;
    std::unordered_set<int> visited;
    queue.push(start_id);
    visited.insert(start_id);
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        auto it = graph.find(current);
        if (it == graph.end()) {
            continue;
        }
        const BoxNode* current_box = find_box_by_id(boxes, current);
        if (current_box == nullptr) {
            continue;
        }
        for (int next : it->second) {
            const BoxNode* next_box = find_box_by_id(boxes, next);
            if (next_box == nullptr ||
                !boxes_connected(*current_box, *next_box, adjacency_tolerance)) {
                continue;
            }
            if (next == goal_id) {
                return true;
            }
            if (visited.insert(next).second) {
                queue.push(next);
            }
        }
    }
    return false;
}

} // namespace

void RBFPlanningForest::rebuild_adaptive_partition(const AdaptiveLeafSweepConfig& config,
                                                   BuildProfile* profile) {
    adaptive_partition_query_enabled_ = false;
    last_adaptive_partition_config_ = config;
    has_adaptive_partition_config_ = true;
    if (config.planning_backend != "partition_native" ||
        !config.grid_face_index_enabled ||
        !oracle_) {
        adaptive_partition_.reset();
        if (profile) {
            profile->diagnostics["adaptive.offline_backend_grid_partition"] = 0.0;
            profile->diagnostics["adaptive.online_backend_partition_native"] = 0.0;
        }
        return;
    }
    if (!adaptive_partition_) {
        adaptive_partition_ = std::make_unique<AdaptiveGridPartition>();
    }
    const int target_depth = config.grid_target_depth > 0
        ? config.grid_target_depth
        : config.target_max_depth;
    bool ok = false;
    try {
        auto root_copies = oracle_->native_root_interval_copies();
        if (root_copies.empty()) {
            root_copies.push_back(oracle_->planning_intervals());
        }
        ok = adaptive_partition_->rebuild(root_copies,
                                          oracle_->database().split_policy_descriptor(),
                                          oracle_->database().root_depth(),
                                          target_depth,
                                          boxes_,
                                          config_.query.adjacency_tolerance);
    } catch (const std::exception&) {
        ok = false;
    }
    adaptive_partition_query_enabled_ = ok;
    if (!adaptive_partition_query_enabled_) {
        adaptive_partition_.reset();
    }
    refresh_adaptive_partition_diagnostics(profile);
}

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
    profile.diagnostics["adaptive.offline_backend_grid_partition"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    profile.diagnostics["adaptive.online_backend_partition_native"] =
        adaptive_partition_query_enabled_ ? 1.0 : 0.0;
    refresh_adaptive_partition_diagnostics(nullptr);
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
    invalidate_query_cache();
}

int RBFPlanningForest::append_adaptive_partition_boxes(std::size_t first_box_index,
                                                       BuildProfile* profile,
                                                       const char* diagnostic_prefix) {
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_ || first_box_index >= boxes_.size()) {
        return 0;
    }
    const int appended = adaptive_partition_->append_boxes(boxes_,
                                                           first_box_index,
                                                           config_.query.adjacency_tolerance);
    if (appended <= 0) {
        return 0;
    }
    if (profile != nullptr && diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0') {
        profile->diagnostics[std::string(diagnostic_prefix) + ".partition_boxes_appended"] +=
            static_cast<double>(appended);
    }
    sync_adaptive_partition_segment_edges(profile, diagnostic_prefix);
    refresh_adaptive_partition_diagnostics(profile);
    return appended;
}

int RBFPlanningForest::sync_adaptive_partition_segment_edges(BuildProfile* profile,
                                                             const char* diagnostic_prefix) {
    if (!adaptive_partition_query_enabled_ || !adaptive_partition_) {
        return 0;
    }
    const int appended = adaptive_partition_->sync_segment_edges(segment_edges_);
    if (appended > 0 && profile != nullptr && diagnostic_prefix != nullptr && diagnostic_prefix[0] != '\0') {
        profile->diagnostics[std::string(diagnostic_prefix) + ".partition_edges_appended"] +=
            static_cast<double>(appended);
    }
    if (appended > 0) {
        refresh_adaptive_partition_diagnostics(profile);
    }
    return appended;
}

int RBFPlanningForest::locate_box_partition_first(const Eigen::Ref<const Eigen::VectorXd>& point,
                                                  bool nearest_if_outside) const {
    if (adaptive_partition_query_enabled_ && adaptive_partition_ && !adaptive_partition_->empty()) {
        const int partition_box = adaptive_partition_->locate_containing_box(point,
                                                                             nearest_if_outside,
                                                                             config_.query.adjacency_tolerance);
        if (partition_box >= 0) {
            return partition_box;
        }
    }
    if (partition_native_mode()) {
        return -1;
    }
    return locate_containing_box(query_cache(), point, nearest_if_outside);
}

bool RBFPlanningForest::box_only_path_connected_partition_first(int source_box_id,
                                                                int target_box_id) const {
    if (source_box_id < 0 || target_box_id < 0) {
        return false;
    }
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty() &&
        adaptive_partition_->same_island(source_box_id, target_box_id)) {
        return true;
    }
    if (partition_native_mode()) {
        return false;
    }
    return graph_has_certified_box_path_local(boxes_,
                                              adjacency_,
                                              source_box_id,
                                              target_box_id,
                                              config_.query.adjacency_tolerance);
}

bool RBFPlanningForest::overlay_path_connected_partition_first(int source_box_id,
                                                               int target_box_id) const {
    if (source_box_id < 0 || target_box_id < 0) {
        return false;
    }
    if (adaptive_partition_query_enabled_ &&
        adaptive_partition_ &&
        !adaptive_partition_->empty() &&
        adaptive_partition_->same_component_with_overlay(source_box_id, target_box_id)) {
        return true;
    }
    if (partition_native_mode()) {
        return false;
    }
    return graph_has_box_path_local(adjacency_, source_box_id, target_box_id);
}

bool RBFPlanningForest::partition_native_mode() const {
    return has_adaptive_partition_config_ &&
           last_adaptive_partition_config_.planning_backend == "partition_native";
}

int RBFPlanningForest::island_count_partition_first() const {
    if (partition_native_mode() &&
        adaptive_partition_query_enabled_ &&
        adaptive_partition_) {
        return adaptive_partition_->component_count_with_overlay();
    }
    return static_cast<int>(find_islands(adjacency_).size());
}

} // namespace rbf
