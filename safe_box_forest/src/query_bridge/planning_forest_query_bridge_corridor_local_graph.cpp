#include "planning_forest_query_bridge_corridor_graph.h"

#include <SBF/adaptive_leaf_sweep_config.h>

#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
#include <limits>
#include <queue>

namespace rbf {

std::vector<int> query_bridge_shortest_local_path(
    const std::vector<std::vector<int>>& local_adj,
    int source_node,
    int target_node) {
    std::vector<int> path;
    if (source_node < 0 || target_node < 0 ||
        source_node >= static_cast<int>(local_adj.size()) ||
        target_node >= static_cast<int>(local_adj.size())) {
        return path;
    }
    std::vector<int> parent(local_adj.size(), -1);
    std::queue<int> queue;
    parent[static_cast<std::size_t>(source_node)] = source_node;
    queue.push(source_node);
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        if (current == target_node) {
            break;
        }
        for (int neighbor : local_adj[static_cast<std::size_t>(current)]) {
            if (neighbor < 0 || neighbor >= static_cast<int>(local_adj.size()) ||
                parent[static_cast<std::size_t>(neighbor)] >= 0) {
                continue;
            }
            parent[static_cast<std::size_t>(neighbor)] = current;
            queue.push(neighbor);
        }
    }
    if (parent[static_cast<std::size_t>(target_node)] < 0) {
        return path;
    }
    for (int current = target_node;
         current != source_node;
         current = parent[static_cast<std::size_t>(current)]) {
        path.push_back(current);
    }
    path.push_back(source_node);
    std::reverse(path.begin(), path.end());
    return path;
}

std::pair<std::vector<int>, int> query_bridge_internal_local_components(
    const std::vector<std::vector<int>>& local_adj,
    int local_source,
    int local_target) {
    std::vector<int> component_id(local_adj.size(), -1);
    int component_count = 0;
    for (int node = local_source + 1; node < local_target; ++node) {
        if (node < 0 || node >= static_cast<int>(local_adj.size()) ||
            component_id[static_cast<std::size_t>(node)] >= 0) {
            continue;
        }
        std::queue<int> component_queue;
        component_id[static_cast<std::size_t>(node)] = component_count;
        component_queue.push(node);
        while (!component_queue.empty()) {
            const int current = component_queue.front();
            component_queue.pop();
            for (int neighbor : local_adj[static_cast<std::size_t>(current)]) {
                if (neighbor <= local_source ||
                    neighbor >= local_target ||
                    neighbor < 0 ||
                    neighbor >= static_cast<int>(local_adj.size()) ||
                    component_id[static_cast<std::size_t>(neighbor)] >= 0) {
                    continue;
                }
                component_id[static_cast<std::size_t>(neighbor)] = component_count;
                component_queue.push(neighbor);
            }
        }
        ++component_count;
    }
    return {std::move(component_id), component_count};
}

QueryBridgeHipacPromotionGate query_bridge_hipac_promotion_gate(
    const AdaptiveLeafSweepConfig& config,
    bool partition_native,
    int source_box_id,
    int target_box_id,
    int query_index) {
    QueryBridgeHipacPromotionGate gate;
    gate.min_boxes = std::max(1, config.hipac_promote_transition_min_boxes);
    gate.max_boxes = std::max(gate.min_boxes, config.hipac_promote_transition_max_boxes);
    if (!config.hipac_online_connectivity ||
        !config.hipac_promote_transition_slices ||
        config.hipac_promote_transition_max_attempts_per_query <= 0 ||
        !partition_native ||
        source_box_id < 0 ||
        target_box_id < 0 ||
        source_box_id == target_box_id) {
        gate.disabled = true;
        return gate;
    }
    if (!csv_index_list_contains(config.hipac_promote_transition_target_query_indices,
                                 query_index)) {
        gate.target_rejected = true;
        return gate;
    }
    gate.eligible = true;
    return gate;
}

std::vector<QueryBridgeLocalSliceCandidate> query_bridge_component_slice_candidates(
    const std::vector<int>& component_id,
    int component_count,
    const std::vector<int>& local_indices,
    const std::unordered_map<int, int>& first_sample_by_box,
    int min_boxes) {
    std::vector<std::vector<int>> nodes_by_component(static_cast<std::size_t>(
        std::max(0, component_count)));
    for (int node = 1; node + 1 < static_cast<int>(local_indices.size()); ++node) {
        if (node < 0 || node >= static_cast<int>(component_id.size())) {
            continue;
        }
        const int component = component_id[static_cast<std::size_t>(node)];
        if (component >= 0 && component < component_count) {
            nodes_by_component[static_cast<std::size_t>(component)].push_back(node);
        }
    }
    auto sample_rank = [&](int local_node) {
        const int box_index = local_indices[static_cast<std::size_t>(local_node)];
        const auto it = first_sample_by_box.find(box_index);
        return it == first_sample_by_box.end()
            ? std::numeric_limits<int>::max()
            : it->second;
    };

    std::vector<QueryBridgeLocalSliceCandidate> slices;
    slices.reserve(nodes_by_component.size());
    for (auto& nodes : nodes_by_component) {
        if (static_cast<int>(nodes.size()) < min_boxes + 2) {
            continue;
        }
        std::sort(nodes.begin(), nodes.end(), [&](int lhs, int rhs) {
            const int lhs_rank = sample_rank(lhs);
            const int rhs_rank = sample_rank(rhs);
            if (lhs_rank != rhs_rank) {
                return lhs_rank < rhs_rank;
            }
            return lhs < rhs;
        });
        QueryBridgeLocalSliceCandidate slice;
        slice.first = nodes.front();
        slice.last = nodes.back();
        slice.count = static_cast<int>(nodes.size());
        slice.span = std::max(0, sample_rank(slice.last) - sample_rank(slice.first));
        slices.push_back(slice);
    }
    std::sort(slices.begin(),
              slices.end(),
              [](const QueryBridgeLocalSliceCandidate& lhs,
                 const QueryBridgeLocalSliceCandidate& rhs) {
        if (lhs.count != rhs.count) {
            return lhs.count > rhs.count;
        }
        if (lhs.span != rhs.span) {
            return lhs.span > rhs.span;
        }
        return lhs.first < rhs.first;
    });
    return slices;
}

}  // namespace rbf
