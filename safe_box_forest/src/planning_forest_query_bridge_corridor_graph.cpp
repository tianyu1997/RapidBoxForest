#include "planning_forest_query_bridge_corridor_graph.h"

#include <SBF/safe_box_forest.h>

#include "planning_forest_query_utils.h"

#include <algorithm>
#include <array>
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

bool query_bridge_mark_sample_coverage_from_candidates(
    const std::vector<BoxNode>& boxes,
    const std::vector<Eigen::VectorXd>& samples,
    std::size_t sample_index,
    const std::vector<int>& candidates,
    double tolerance,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered) {
    if (sample_index >= samples.size() || sample_index >= sample_layers.size()) {
        return false;
    }
    bool newly_covered = false;
    for (int box_index : candidates) {
        if (box_index < 0 || box_index >= static_cast<int>(boxes.size())) {
            continue;
        }
        if (!intervals_contain_point_local(
                boxes[static_cast<std::size_t>(box_index)].joint_intervals,
                samples[sample_index],
                tolerance)) {
            continue;
        }
        auto& layer = sample_layers[sample_index];
        if (std::find(layer.begin(), layer.end(), box_index) == layer.end()) {
            layer.push_back(box_index);
        }
        if (sample_index < covered.size() && !covered[sample_index]) {
            covered[sample_index] = true;
            newly_covered = true;
        }
    }
    return newly_covered;
}

QueryBridgeInitialSampleCoverageStats query_bridge_mark_initial_sample_coverage(
    const std::vector<BoxNode>& boxes,
    const std::vector<Eigen::VectorXd>& samples,
    double tolerance,
    const QueryBridgeSampleCandidateProvider& candidates_for_sample,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered) {
    QueryBridgeInitialSampleCoverageStats stats;
    for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
        if (query_bridge_mark_sample_coverage_from_candidates(
                boxes,
                samples,
                sample_index,
                candidates_for_sample(samples[sample_index]),
                tolerance,
                sample_layers,
                covered)) {
            stats.covered_samples += 1;
        }
    }
    return stats;
}

QueryBridgeInitialDsuStats query_bridge_initialize_sample_dsu(
    const std::vector<std::vector<int>>& sample_layers,
    QueryBridgeLocalDsu& dsu,
    const std::function<bool(int, int)>& boxes_adjacent,
    const std::function<void(int, int)>& on_adjacent_pair) {
    QueryBridgeInitialDsuStats stats;
    for (const auto& layer : sample_layers) {
        if (layer.empty()) {
            continue;
        }
        const int root = layer.front();
        for (int index : layer) {
            dsu.unite(root, index);
        }
    }
    for (std::size_t sample_index = 0; sample_index + 1 < sample_layers.size(); ++sample_index) {
        for (int lhs : sample_layers[sample_index]) {
            for (int rhs : sample_layers[sample_index + 1]) {
                stats.adjacency_tests += 1;
                if (!boxes_adjacent(lhs, rhs)) {
                    continue;
                }
                dsu.unite(lhs, rhs);
                stats.adjacency_edges += 1;
                if (on_adjacent_pair) {
                    on_adjacent_pair(lhs, rhs);
                }
            }
        }
    }
    return stats;
}

std::unordered_map<int, int> query_bridge_build_box_id_index(
    const std::vector<BoxNode>& boxes) {
    std::unordered_map<int, int> box_id_to_index;
    box_id_to_index.reserve(boxes.size() * 2);
    for (std::size_t box_index = 0; box_index < boxes.size(); ++box_index) {
        box_id_to_index.emplace(boxes[box_index].id, static_cast<int>(box_index));
    }
    return box_id_to_index;
}

std::vector<int> query_bridge_partition_neighbor_index_candidates(
    const AdaptiveGridPartition& partition,
    const BoxNode& box,
    double tolerance,
    const std::unordered_map<int, int>& box_id_to_index,
    int* raw_neighbor_count) {
    const auto neighbor_ids = partition.adjacent_box_ids(box, tolerance);
    if (raw_neighbor_count != nullptr) {
        *raw_neighbor_count = static_cast<int>(neighbor_ids.size());
    }
    std::vector<int> candidates;
    candidates.reserve(neighbor_ids.size());
    for (int neighbor_box_id : neighbor_ids) {
        const auto index_it = box_id_to_index.find(neighbor_box_id);
        if (index_it != box_id_to_index.end()) {
            candidates.push_back(index_it->second);
        }
    }
    return candidates;
}

QueryBridgeSampleAssimilationResult query_bridge_assimilate_box_samples(
    const std::vector<Interval>& box_intervals,
    const std::vector<Eigen::VectorXd>& samples,
    int box_index,
    int transition_hint,
    double tolerance,
    QueryBridgeLocalDsu& dsu,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered) {
    QueryBridgeSampleAssimilationResult result;
    result.first_covered_sample = static_cast<int>(samples.size());

    auto record_sample_coverage = [&](std::size_t sample_index) {
        const int sample_index_int = static_cast<int>(sample_index);
        result.first_covered_sample = std::min(result.first_covered_sample,
                                               sample_index_int);
        result.last_covered_sample = std::max(result.last_covered_sample,
                                              sample_index_int);
        result.covered_sample_count += 1;
        auto& layer = sample_layers[sample_index];
        if (!layer.empty()) {
            dsu.unite(box_index, layer.front());
        }
        if (std::find(layer.begin(), layer.end(), box_index) == layer.end()) {
            layer.push_back(box_index);
        }
        if (sample_index < covered.size()) {
            covered[sample_index] = true;
        }
    };

    auto sample_in_box = [&](int sample_index) {
        if (sample_index < 0 || sample_index >= static_cast<int>(samples.size())) {
            return false;
        }
        result.local_sample_tests += 1;
        return intervals_contain_point_local(
            box_intervals,
            samples[static_cast<std::size_t>(sample_index)],
            tolerance);
    };

    bool used_full_sample_scan = true;
    if (!samples.empty()) {
        used_full_sample_scan = false;
        int anchor = -1;
        const std::array<int, 5> anchors = {
            transition_hint,
            transition_hint + 1,
            transition_hint - 1,
            transition_hint + 2,
            transition_hint - 2,
        };
        for (int candidate_anchor : anchors) {
            if (sample_in_box(candidate_anchor)) {
                anchor = candidate_anchor;
                break;
            }
        }
        if (anchor >= 0) {
            int left = anchor;
            int right = anchor;
            while (left > 0 && sample_in_box(left - 1)) {
                --left;
            }
            while (right + 1 < static_cast<int>(samples.size()) &&
                   sample_in_box(right + 1)) {
                ++right;
            }
            for (int sample_index = left; sample_index <= right; ++sample_index) {
                record_sample_coverage(static_cast<std::size_t>(sample_index));
            }
            result.local_hit = true;
        } else {
            used_full_sample_scan = true;
            result.full_scan_fallback = true;
        }
    }
    if (used_full_sample_scan) {
        for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
            if (!intervals_contain_point_local(box_intervals,
                                               samples[sample_index],
                                               tolerance)) {
                continue;
            }
            record_sample_coverage(sample_index);
        }
    }
    return result;
}

std::vector<int> query_bridge_sample_layer_adjacency_candidates(
    const std::vector<std::vector<int>>& sample_layers,
    int transition_hint,
    const QueryBridgeSampleAssimilationResult& sample_assimilation,
    const std::vector<int>& repair_indices) {
    std::vector<int> candidates;
    auto add_layer = [&](int layer_index) {
        if (layer_index < 0 || layer_index >= static_cast<int>(sample_layers.size())) {
            return;
        }
        const auto& layer = sample_layers[static_cast<std::size_t>(layer_index)];
        candidates.insert(candidates.end(), layer.begin(), layer.end());
    };
    add_layer(transition_hint - 1);
    add_layer(transition_hint);
    add_layer(transition_hint + 1);
    add_layer(transition_hint + 2);
    if (sample_assimilation.covered_sample_count > 0) {
        add_layer(sample_assimilation.first_covered_sample - 1);
        add_layer(sample_assimilation.first_covered_sample);
        add_layer(sample_assimilation.first_covered_sample + 1);
        add_layer(sample_assimilation.last_covered_sample - 1);
        add_layer(sample_assimilation.last_covered_sample);
        add_layer(sample_assimilation.last_covered_sample + 1);
    }
    candidates.insert(candidates.end(), repair_indices.begin(), repair_indices.end());
    return candidates;
}

QueryBridgeAdjacencyCandidateSet query_bridge_collect_adjacency_candidates(
    const std::vector<std::vector<int>>& sample_layers,
    int transition_hint,
    const QueryBridgeSampleAssimilationResult& sample_assimilation,
    const std::vector<int>& repair_indices,
    const AdaptiveGridPartition* partition,
    const BoxNode* partition_box,
    double tolerance,
    const std::unordered_map<int, int>& box_id_to_index) {
    QueryBridgeAdjacencyCandidateSet result;
    result.candidates =
        query_bridge_sample_layer_adjacency_candidates(sample_layers,
                                                       transition_hint,
                                                       sample_assimilation,
                                                       repair_indices);
    if (partition != nullptr && partition_box != nullptr) {
        const std::vector<int> partition_candidates =
            query_bridge_partition_neighbor_index_candidates(
                *partition,
                *partition_box,
                tolerance,
                box_id_to_index,
                &result.partition_neighbor_raw_count);
        result.candidates.insert(result.candidates.end(),
                                 partition_candidates.begin(),
                                 partition_candidates.end());
    }
    std::sort(result.candidates.begin(), result.candidates.end());
    result.candidates.erase(std::unique(result.candidates.begin(),
                                        result.candidates.end()),
                            result.candidates.end());
    return result;
}

bool query_bridge_sample_transition_connected(const std::vector<std::vector<int>>& sample_layers,
                                              QueryBridgeLocalDsu& dsu,
                                              int transition) {
    if (transition < 0 || transition + 1 >= static_cast<int>(sample_layers.size())) {
        return false;
    }
    const auto& lhs_layer = sample_layers[static_cast<std::size_t>(transition)];
    const auto& rhs_layer = sample_layers[static_cast<std::size_t>(transition + 1)];
    if (lhs_layer.empty() || rhs_layer.empty()) {
        return false;
    }
    for (int lhs : lhs_layer) {
        const int root = dsu.find(lhs);
        for (int rhs : rhs_layer) {
            if (root == dsu.find(rhs)) {
                return true;
            }
        }
    }
    return false;
}

std::vector<int> query_bridge_bad_sample_transitions(const std::vector<std::vector<int>>& sample_layers,
                                                     QueryBridgeLocalDsu& dsu) {
    std::vector<int> bad;
    for (std::size_t sample_index = 0; sample_index + 1 < sample_layers.size(); ++sample_index) {
        if (!query_bridge_sample_transition_connected(sample_layers,
                                                      dsu,
                                                      static_cast<int>(sample_index))) {
            bad.push_back(static_cast<int>(sample_index));
        }
    }
    return bad;
}

bool query_bridge_endpoint_layers_connected(const std::vector<std::vector<int>>& sample_layers,
                                            QueryBridgeLocalDsu& dsu) {
    if (sample_layers.empty() ||
        sample_layers.front().empty() ||
        sample_layers.back().empty()) {
        return false;
    }
    const int root = dsu.find(sample_layers.front().front());
    for (int index : sample_layers.back()) {
        if (root == dsu.find(index)) {
            return true;
        }
    }
    return false;
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
