#include "planning_forest_query_bridge_corridor_graph.h"

#include <SBF/adaptive_grid_partition.h>
#include <SBF/runtime.h>

#include "../qroot/planning_forest_qroot_helpers.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
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

}  // namespace rbf
