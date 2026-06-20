#pragma once

#include <SBF/box_graph.h>

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

struct QueryBridgeSampleAssimilationResult {
    int first_covered_sample = 0;
    int last_covered_sample = -1;
    int covered_sample_count = 0;
    int local_sample_tests = 0;
    bool local_hit = false;
    bool full_scan_fallback = false;
};

bool query_bridge_mark_sample_coverage_from_candidates(
    const std::vector<BoxNode>& boxes,
    const std::vector<Eigen::VectorXd>& samples,
    std::size_t sample_index,
    const std::vector<int>& candidates,
    double tolerance,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered);

QueryBridgeSampleAssimilationResult query_bridge_assimilate_box_samples(
    const std::vector<Interval>& box_intervals,
    const std::vector<Eigen::VectorXd>& samples,
    int box_index,
    int transition_hint,
    double tolerance,
    QueryBridgeLocalDsu& dsu,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered);

std::vector<int> query_bridge_sample_layer_adjacency_candidates(
    const std::vector<std::vector<int>>& sample_layers,
    int transition_hint,
    const QueryBridgeSampleAssimilationResult& sample_assimilation,
    const std::vector<int>& repair_indices);

bool query_bridge_sample_transition_connected(const std::vector<std::vector<int>>& sample_layers,
                                              QueryBridgeLocalDsu& dsu,
                                              int transition);

std::vector<int> query_bridge_bad_sample_transitions(const std::vector<std::vector<int>>& sample_layers,
                                                     QueryBridgeLocalDsu& dsu);

bool query_bridge_endpoint_layers_connected(const std::vector<std::vector<int>>& sample_layers,
                                            QueryBridgeLocalDsu& dsu);

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
