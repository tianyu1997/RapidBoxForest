#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rbf {

struct AdaptiveLeafSweepConfig;

struct QueryBridgeLocalSliceCandidate {
    int first = -1;
    int last = -1;
    int count = 0;
    int span = 0;
};

struct QueryBridgeHipacPromotionGate {
    bool eligible = false;
    bool disabled = false;
    bool target_rejected = false;
    int min_boxes = 1;
    int max_boxes = 1;
};

struct QueryBridgeLocalDsu {
    std::vector<int> parent;

    explicit QueryBridgeLocalDsu(std::size_t count = 0);

    int add();
    int find(int value);
    void unite(int lhs, int rhs);
};

double query_bridge_transition_length(const std::vector<Eigen::VectorXd>& samples,
                                      int transition);

double query_bridge_transition_length_sum(const std::vector<Eigen::VectorXd>& samples,
                                          const std::vector<int>& transitions);

double query_bridge_transition_fraction(const std::vector<Eigen::VectorXd>& samples,
                                        const std::vector<int>& transitions,
                                        double audited_bridge_length,
                                        double fallback_path_length);

double query_bridge_waypoint_length(const std::vector<Eigen::VectorXd>& path);

std::vector<int> query_bridge_order_transitions_by_gap_length(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    int priority_mode);

std::vector<int> query_bridge_shortest_local_path(
    const std::vector<std::vector<int>>& local_adj,
    int source_node,
    int target_node);

std::pair<std::vector<int>, int> query_bridge_internal_local_components(
    const std::vector<std::vector<int>>& local_adj,
    int local_source,
    int local_target);

QueryBridgeHipacPromotionGate query_bridge_hipac_promotion_gate(
    const AdaptiveLeafSweepConfig& config,
    bool partition_native,
    int source_box_id,
    int target_box_id,
    int query_index);

std::vector<QueryBridgeLocalSliceCandidate> query_bridge_component_slice_candidates(
    const std::vector<int>& component_id,
    int component_count,
    const std::vector<int>& local_indices,
    const std::unordered_map<int, int>& first_sample_by_box,
    int min_boxes);

int query_bridge_nearest_nonempty_layer(const std::vector<std::vector<int>>& sample_layers,
                                        int start_index,
                                        int direction);

}  // namespace rbf
