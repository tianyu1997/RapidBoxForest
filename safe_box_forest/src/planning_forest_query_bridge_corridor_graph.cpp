#include "planning_forest_query_bridge_corridor_graph.h"

#include <SBF/safe_box_forest.h>

#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace rbf {

QueryBridgeLocalDsu::QueryBridgeLocalDsu(std::size_t count) : parent(count) {
    for (std::size_t index = 0; index < parent.size(); ++index) {
        parent[index] = static_cast<int>(index);
    }
}

int QueryBridgeLocalDsu::add() {
    const int id = static_cast<int>(parent.size());
    parent.push_back(id);
    return id;
}

int QueryBridgeLocalDsu::find(int value) {
    int root = value;
    while (parent[static_cast<std::size_t>(root)] != root) {
        root = parent[static_cast<std::size_t>(root)];
    }
    while (parent[static_cast<std::size_t>(value)] != value) {
        const int next = parent[static_cast<std::size_t>(value)];
        parent[static_cast<std::size_t>(value)] = root;
        value = next;
    }
    return root;
}

void QueryBridgeLocalDsu::unite(int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 ||
        lhs >= static_cast<int>(parent.size()) ||
        rhs >= static_cast<int>(parent.size())) {
        return;
    }
    const int left = find(lhs);
    const int right = find(rhs);
    if (left != right) {
        parent[static_cast<std::size_t>(right)] = left;
    }
}

double query_bridge_transition_length(const std::vector<Eigen::VectorXd>& samples,
                                      int transition) {
    if (transition < 0 ||
        transition + 1 >= static_cast<int>(samples.size())) {
        return 0.0;
    }
    return (samples[static_cast<std::size_t>(transition + 1)] -
            samples[static_cast<std::size_t>(transition)]).norm();
}

double query_bridge_transition_length_sum(const std::vector<Eigen::VectorXd>& samples,
                                          const std::vector<int>& transitions) {
    double total = 0.0;
    for (int transition : transitions) {
        total += query_bridge_transition_length(samples, transition);
    }
    return total;
}

double query_bridge_transition_fraction(const std::vector<Eigen::VectorXd>& samples,
                                        const std::vector<int>& transitions,
                                        double audited_bridge_length,
                                        double fallback_path_length) {
    const double denominator = audited_bridge_length > 1e-12
        ? audited_bridge_length
        : std::max(1e-12, fallback_path_length);
    return query_bridge_transition_length_sum(samples, transitions) / denominator;
}

double query_bridge_waypoint_length(const std::vector<Eigen::VectorXd>& path) {
    double total = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        total += (path[index] - path[index - 1]).norm();
    }
    return total;
}

std::vector<int> query_bridge_order_transitions_by_gap_length(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    int priority_mode) {
    if (priority_mode <= 0 || transitions.size() < 2) {
        return transitions;
    }
    struct GapGroup {
        int begin = 0;
        int end = 0;
        double length = 0.0;
    };
    std::vector<GapGroup> groups;
    groups.reserve(transitions.size());
    for (int transition : transitions) {
        if (groups.empty() || transition > groups.back().end + 1) {
            groups.push_back({transition,
                              transition,
                              query_bridge_transition_length(samples, transition)});
        } else {
            groups.back().end = transition;
            groups.back().length += query_bridge_transition_length(samples, transition);
        }
    }
    std::stable_sort(groups.begin(), groups.end(), [](const GapGroup& lhs,
                                                       const GapGroup& rhs) {
        if (std::abs(lhs.length - rhs.length) > 1e-12) {
            return lhs.length > rhs.length;
        }
        return lhs.begin < rhs.begin;
    });
    std::vector<int> ordered;
    ordered.reserve(transitions.size());
    for (const auto& group : groups) {
        const int center = (group.begin + group.end) / 2;
        ordered.push_back(center);
        for (int radius = 1;
             center - radius >= group.begin || center + radius <= group.end;
             ++radius) {
            if (center - radius >= group.begin) {
                ordered.push_back(center - radius);
            }
            if (center + radius <= group.end) {
                ordered.push_back(center + radius);
            }
        }
    }
    return ordered;
}

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

int query_bridge_nearest_nonempty_layer(const std::vector<std::vector<int>>& sample_layers,
                                        int start_index,
                                        int direction) {
    int index = start_index;
    while (index >= 0 && index < static_cast<int>(sample_layers.size())) {
        if (!sample_layers[static_cast<std::size_t>(index)].empty()) {
            return index;
        }
        index += direction;
    }
    return -1;
}

}  // namespace rbf
