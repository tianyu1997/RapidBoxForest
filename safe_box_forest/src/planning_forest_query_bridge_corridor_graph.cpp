#include "planning_forest_query_bridge_corridor_graph.h"

#include <SBF/runtime.h>
#include <SBF/safe_box_forest.h>

#include "planning_forest_query_utils.h"
#include "planning_forest_qroot_helpers.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <utility>

namespace rbf {

QueryBridgeDirectCorridorCommitState query_bridge_make_direct_corridor_commit_state(
    std::unordered_map<OracleNodeId, int>& node_to_box_index,
    std::vector<int>& corridor_new_box_indices,
    BoxSpatialIndex& direct_box_index,
    std::unordered_map<int, int>& box_id_to_index,
    bool use_partition_cover_index,
    bool use_partition_neighbor_candidates,
    double adjacency_tolerance) {
    QueryBridgeDirectCorridorCommitState state;
    state.node_to_box_index = &node_to_box_index;
    state.corridor_new_box_indices = &corridor_new_box_indices;
    state.direct_box_index = &direct_box_index;
    state.box_id_to_index = &box_id_to_index;
    state.use_partition_cover_index = use_partition_cover_index;
    state.use_partition_neighbor_candidates = use_partition_neighbor_candidates;
    state.adjacency_tolerance = adjacency_tolerance;
    return state;
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

BoxNode query_bridge_box_from_ffb_result(
    const FindFreeBoxResult& result,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    int box_id) {
    BoxNode box;
    box.id = box_id;
    box.joint_intervals = result.intervals;
    box.seed_config = seed;
    box.tree_id = result.node;
    box.parent_box_id = -1;
    box.root_id = box.id;
    box.safety_status = result.validation_detail.safety_status;
    box.strict_audit_required = result.validation_detail.strict_audit_required;
    box.compute_volume();
    return box;
}

int query_bridge_append_direct_corridor_box(
    BoxNode box,
    std::vector<BoxNode>& boxes,
    std::vector<BoxNode>& raw_boxes,
    QueryBridgeDirectCorridorCommitState& state) {
    const int box_index = static_cast<int>(boxes.size());
    const OracleNodeId node = box.tree_id;
    const int box_id = box.id;
    boxes.push_back(box);
    raw_boxes.push_back(boxes.back());
    if (node != kInvalidOracleNodeId && state.node_to_box_index != nullptr) {
        state.node_to_box_index->emplace(node, box_index);
    }
    if (state.use_partition_cover_index &&
        state.corridor_new_box_indices != nullptr) {
        state.corridor_new_box_indices->push_back(box_index);
    } else if (state.direct_box_index != nullptr) {
        state.direct_box_index->add_box(boxes.back(),
                                        box_index,
                                        state.adjacency_tolerance);
    }
    if (state.use_partition_neighbor_candidates &&
        state.box_id_to_index != nullptr) {
        (*state.box_id_to_index)[box_id] = box_index;
    }
    return box_index;
}

int query_bridge_append_direct_partition_batch(
    AdaptiveGridPartition* partition,
    std::vector<BoxNode>& boxes,
    QueryBridgePartitionAppendBatchState& state,
    double tolerance,
    StageContext& context,
    bool force) {
    if (!state.enabled || partition == nullptr || state.base >= boxes.size()) {
        return 0;
    }
    const std::size_t pending = boxes.size() - state.base;
    if (!force && pending < static_cast<std::size_t>(std::max(1, state.batch_size))) {
        return 0;
    }
    const int appended = partition->append_boxes(boxes, state.base, tolerance);
    context.diagnostics().add_counter(
        appended > 0
            ? "query_bridge.direct_corridor_batched_partition_appends"
            : "query_bridge.direct_corridor_batched_partition_append_rejects");
    state.base = boxes.size();
    return appended;
}

QueryBridgeDirectCorridorCommitResult query_bridge_commit_ffb_result_to_direct_corridor(
    FindFreeBoxResult result,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    std::vector<BoxNode>& boxes,
    std::vector<BoxNode>& raw_boxes,
    QueryBridgeDirectCorridorCommitState& commit_state,
    AdaptiveGridPartition* partition,
    QueryBridgePartitionAppendBatchState& partition_append_state,
    double tolerance,
    int& next_id,
    StageContext& context,
    const std::function<bool(FindFreeBoxResult&)>& allow_commit,
    const std::function<void(OracleNodeId, int)>& reserve_node) {
    QueryBridgeDirectCorridorCommitResult commit;
    if (!result.found ||
        !intervals_contain_point_local(result.intervals, seed, tolerance)) {
        return commit;
    }
    const std::unordered_map<OracleNodeId, int> empty_node_index;
    const auto& node_to_box_index = commit_state.node_to_box_index != nullptr
        ? *commit_state.node_to_box_index
        : empty_node_index;
    const int duplicate_index =
        find_box_index_by_node_or_intervals(boxes,
                                            node_to_box_index,
                                            result.node,
                                            result.intervals,
                                            1e-12);
    if (duplicate_index >= 0) {
        commit.box_index = duplicate_index;
        commit.duplicate = true;
        return commit;
    }
    if (allow_commit && !allow_commit(result)) {
        return commit;
    }
    BoxNode box = query_bridge_box_from_ffb_result(result, seed, next_id++);
    if (box.tree_id != kInvalidOracleNodeId && reserve_node) {
        reserve_node(box.tree_id, box.id);
    }
    commit.box_index = query_bridge_append_direct_corridor_box(
        std::move(box),
        boxes,
        raw_boxes,
        commit_state);
    commit.appended = true;
    if (commit_state.use_partition_cover_index) {
        query_bridge_append_direct_partition_batch(partition,
                                                   boxes,
                                                   partition_append_state,
                                                   tolerance,
                                                   context,
                                                   false);
    }
    return commit;
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

QueryBridgeIncrementalAdjacencyStats query_bridge_connect_adjacency_candidates(
    int box_index,
    int box_count,
    const std::vector<int>& candidates,
    QueryBridgeLocalDsu& dsu,
    const std::function<bool(int, int)>& boxes_adjacent,
    const std::function<bool(int, int)>& on_adjacent_pair) {
    QueryBridgeIncrementalAdjacencyStats stats;
    for (int candidate : candidates) {
        if (candidate == box_index || candidate < 0 || candidate >= box_count) {
            continue;
        }
        if (!boxes_adjacent(box_index, candidate)) {
            continue;
        }
        dsu.unite(box_index, candidate);
        bool edge_counted = true;
        if (on_adjacent_pair) {
            edge_counted = on_adjacent_pair(box_index, candidate);
        }
        if (edge_counted) {
            stats.adjacency_edges += 1;
        }
    }
    return stats;
}

QueryBridgeDirectCorridorAssimilationResult query_bridge_assimilate_direct_corridor_box(
    const std::vector<BoxNode>& boxes,
    const std::vector<Eigen::VectorXd>& samples,
    int box_index,
    int transition_hint,
    double tolerance,
    QueryBridgeLocalDsu& dsu,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered,
    const std::vector<int>& repair_indices,
    const AdaptiveGridPartition* partition,
    bool use_partition_neighbor_candidates,
    const std::unordered_map<int, int>& box_id_to_index,
    const std::function<bool(int, int)>& boxes_adjacent,
    const std::function<bool(int, int)>& on_adjacent_pair) {
    QueryBridgeDirectCorridorAssimilationResult result;
    const auto& box = boxes[static_cast<std::size_t>(box_index)];
    result.sample_assimilation =
        query_bridge_assimilate_box_samples(box.joint_intervals,
                                            samples,
                                            box_index,
                                            transition_hint,
                                            tolerance,
                                            dsu,
                                            sample_layers,
                                            covered);
    result.candidate_set =
        query_bridge_collect_adjacency_candidates(
            sample_layers,
            transition_hint,
            result.sample_assimilation,
            repair_indices,
            use_partition_neighbor_candidates ? partition : nullptr,
            use_partition_neighbor_candidates ? &box : nullptr,
            tolerance,
            box_id_to_index);
    result.adjacency_stats =
        query_bridge_connect_adjacency_candidates(
            box_index,
            static_cast<int>(boxes.size()),
            result.candidate_set.candidates,
            dsu,
            boxes_adjacent,
            on_adjacent_pair);
    return result;
}

bool query_bridge_direct_corridor_boxes_adjacent(
    const std::vector<BoxNode>& boxes,
    const AdaptiveGridPartition* partition,
    bool use_partition_neighbor_adjacency,
    double tolerance,
    StageContext& context,
    int lhs,
    int rhs) {
    if (lhs < 0 || rhs < 0 ||
        lhs >= static_cast<int>(boxes.size()) ||
        rhs >= static_cast<int>(boxes.size())) {
        return false;
    }
    const int lhs_box_id = boxes[static_cast<std::size_t>(lhs)].id;
    const int rhs_box_id = boxes[static_cast<std::size_t>(rhs)].id;
    if (use_partition_neighbor_adjacency &&
        partition != nullptr &&
        partition->contains_box_id(lhs_box_id) &&
        partition->contains_box_id(rhs_box_id)) {
        context.diagnostics().add_counter(
            "query_bridge.direct_corridor_partition_neighbor_tests");
        const bool adjacent = partition->boxes_are_neighbors(lhs_box_id, rhs_box_id);
        if (adjacent) {
            context.diagnostics().add_counter(
                "query_bridge.direct_corridor_partition_neighbor_hits");
        }
        return adjacent;
    }
    return boxes_connected(boxes[static_cast<std::size_t>(lhs)],
                           boxes[static_cast<std::size_t>(rhs)],
                           tolerance);
}

bool query_bridge_current_corridor_boxes_cover_point(
    const AdaptiveGridPartition* partition,
    bool use_partition_cover_index,
    const std::vector<int>& corridor_new_box_indices,
    const BoxSpatialIndex& direct_box_index,
    const std::vector<BoxNode>& boxes,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double tolerance) {
    if (use_partition_cover_index && partition != nullptr) {
        const bool partition_covered =
            !partition->covering_box_ids(point, tolerance).empty();
        if (partition_covered) {
            return true;
        }
        for (int box_index : corridor_new_box_indices) {
            if (box_index >= 0 &&
                box_index < static_cast<int>(boxes.size()) &&
                intervals_contain_point_local(
                    boxes[static_cast<std::size_t>(box_index)].joint_intervals,
                    point,
                    tolerance)) {
                return true;
            }
        }
        return false;
    }
    return direct_box_index.covering_box(boxes, point, tolerance) >= 0;
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

}  // namespace rbf
