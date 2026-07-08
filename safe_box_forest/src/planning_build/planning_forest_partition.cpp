#include <SBF/safe_box_forest.h>

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/oracle.h>

#include <chrono>
#include <exception>
#include <queue>
#include <string>
#include <unordered_set>

#include "../query_runtime/planning_forest_query_utils.h"

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
