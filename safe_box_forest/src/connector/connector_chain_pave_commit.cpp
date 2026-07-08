#include "connector_chain_pave_internal.h"

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <utility>

namespace rbf {

ChainPaveCommitContext::ChainPaveCommitContext(
    std::vector<BoxNode>& boxes_arg,
    BoxOracle& oracle_arg,
    AdjacencyGraph& graph_arg,
    int& next_box_id_arg,
    const ChainPaveConfig& config_arg,
    StageContext& context_arg,
    int& added_arg)
    : boxes(boxes_arg),
      oracle(oracle_arg),
      graph(graph_arg),
      next_box_id(next_box_id_arg),
      config(config_arg),
      context(context_arg),
      added(added_arg) {
    box_index.reserve(boxes.size() + 16);
    tree_owner.reserve(boxes.size() + 16);
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        index_box(i);
    }
}

void ChainPaveCommitContext::index_box(std::size_t index) {
    const BoxNode& box = boxes[index];
    box_index[box.id] = index;
    if (tree_owner.find(box.tree_id) == tree_owner.end()) {
        tree_owner[box.tree_id] = box.id;
    }
}

BoxNode* ChainPaveCommitContext::box_by_id(int id) {
    auto it = box_index.find(id);
    return it == box_index.end() ? nullptr : &boxes[it->second];
}

void ChainPaveCommitContext::append_graph_edge(int lhs, int rhs) {
    if (lhs == rhs) {
        return;
    }
    auto append_one = [&](int from, int to) {
        auto& list = graph[from];
        if (std::find(list.begin(), list.end(), to) == list.end()) {
            list.push_back(to);
        }
    };
    append_one(lhs, rhs);
    append_one(rhs, lhs);
}

int ChainPaveCommitContext::find_existing_cover(const Eigen::VectorXd& point,
                                                int preferred_id) {
    if (preferred_id >= 0) {
        if (BoxNode* preferred = box_by_id(preferred_id)) {
            if (preferred->contains(point, config.adjacency_tolerance)) {
                return preferred_id;
            }
        }
    }
    for (const auto& box : boxes) {
        if (box.contains(point, config.adjacency_tolerance)) {
            return box.id;
        }
    }
    return -1;
}

int ChainPaveCommitContext::find_box_owning_node_covering(
    OracleNodeId node,
    const Eigen::VectorXd& point) {
    for (const auto& box : boxes) {
        if (box.tree_id == node && box.contains(point, config.adjacency_tolerance)) {
            return box.id;
        }
    }
    return -1;
}

int ChainPaveCommitContext::find_box_owning_node(OracleNodeId node) const {
    auto it = tree_owner.find(node);
    return it == tree_owner.end() ? -1 : it->second;
}

int ChainPaveCommitContext::commit_box(FindFreeBoxResult& result,
                                       const Eigen::VectorXd& seed,
                                       int parent_id,
                                       bool allow_duplicate_node) {
    BoxNode* parent_box = box_by_id(parent_id);
    if (parent_box == nullptr) {
        return -1;
    }
    if (result.node != kInvalidOracleNodeId &&
        result.node == parent_box->tree_id) {
        return -1;
    }
    const bool node_already_owned =
        find_box_owning_node_covering(result.node, seed) >= 0;
    if (!allow_duplicate_node && node_already_owned) {
        return -1;
    }
    if (!allow_connector_box_commit(oracle, result, config.commit_policy, context)) {
        return -1;
    }
    BoxNode box;
    box.joint_intervals = result.intervals;
    box.seed_config = seed;
    box.tree_id = result.node;
    box.parent_box_id = parent_id;
    box.root_id = parent_box->root_id;
    box.safety_status = result.validation_detail.safety_status;
    box.strict_audit_required = result.validation_detail.strict_audit_required;
    box.compute_volume();
    const bool adjacent = boxes_connected(*parent_box, box, config.adjacency_tolerance);
    if (!adjacent) {
        return -1;
    }
    box.id = next_box_id++;
    const int new_id = box.id;
    if (!node_already_owned) {
        oracle.reserve_node(result.node, new_id);
    }
    graph[new_id] = {};
    if (adjacent) {
        graph[parent_id].push_back(new_id);
        graph[new_id].push_back(parent_id);
    }
    const std::size_t new_index = boxes.size();
    boxes.push_back(std::move(box));
    index_box(new_index);
    added += 1;
    return new_id;
}

int ChainPaveCommitContext::commit_reserved_cap_box(
    const FindFreeBoxResult& result,
    const Eigen::VectorXd& seed,
    int parent_id) {
    const int owner = find_box_owning_node(result.node);
    if (owner < 0) {
        return -1;
    }
    BoxNode* parent_box = box_by_id(parent_id);
    BoxNode* owner_box = box_by_id(owner);
    if (parent_box == nullptr || owner_box == nullptr) {
        return -1;
    }
    BoxNode box;
    box.joint_intervals = result.intervals;
    box.seed_config = seed;
    box.tree_id = result.node;
    box.parent_box_id = parent_id;
    box.root_id = parent_box->root_id;
    box.safety_status = owner_box->safety_status;
    box.strict_audit_required = owner_box->strict_audit_required;
    box.compute_volume();
    const bool adjacent =
        boxes_connected(*parent_box, box, config.adjacency_tolerance);
    if (!adjacent) {
        return -1;
    }
    box.id = next_box_id++;
    const int new_id = box.id;
    graph[new_id] = {};
    if (adjacent) {
        graph[parent_id].push_back(new_id);
        graph[new_id].push_back(parent_id);
    }
    const std::size_t new_index = boxes.size();
    boxes.push_back(std::move(box));
    index_box(new_index);
    added += 1;
    return new_id;
}

}  // namespace rbf
