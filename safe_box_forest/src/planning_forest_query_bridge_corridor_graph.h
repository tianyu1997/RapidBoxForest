#pragma once

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rbf {

struct AdaptiveLeafSweepConfig;
struct BoxSpatialIndex;

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

struct QueryBridgeInitialDsuStats {
    int adjacency_tests = 0;
    int adjacency_edges = 0;
};

struct QueryBridgeInitialSampleCoverageStats {
    int covered_samples = 0;
};

struct QueryBridgeAdjacencyCandidateSet {
    std::vector<int> candidates;
    int partition_neighbor_raw_count = 0;
};

struct QueryBridgeDirectCorridorCommitState {
    std::unordered_map<OracleNodeId, int>* node_to_box_index = nullptr;
    std::vector<int>* corridor_new_box_indices = nullptr;
    BoxSpatialIndex* direct_box_index = nullptr;
    std::unordered_map<int, int>* box_id_to_index = nullptr;
    bool use_partition_cover_index = false;
    bool use_partition_neighbor_candidates = false;
    double adjacency_tolerance = 0.0;
};

using QueryBridgeSampleCandidateProvider =
    std::function<std::vector<int>(const Eigen::VectorXd& sample)>;

bool query_bridge_mark_sample_coverage_from_candidates(
    const std::vector<BoxNode>& boxes,
    const std::vector<Eigen::VectorXd>& samples,
    std::size_t sample_index,
    const std::vector<int>& candidates,
    double tolerance,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered);

QueryBridgeInitialSampleCoverageStats query_bridge_mark_initial_sample_coverage(
    const std::vector<BoxNode>& boxes,
    const std::vector<Eigen::VectorXd>& samples,
    double tolerance,
    const QueryBridgeSampleCandidateProvider& candidates_for_sample,
    std::vector<std::vector<int>>& sample_layers,
    std::vector<bool>& covered);

QueryBridgeInitialDsuStats query_bridge_initialize_sample_dsu(
    const std::vector<std::vector<int>>& sample_layers,
    QueryBridgeLocalDsu& dsu,
    const std::function<bool(int, int)>& boxes_adjacent,
    const std::function<void(int, int)>& on_adjacent_pair);

std::unordered_map<int, int> query_bridge_build_box_id_index(
    const std::vector<BoxNode>& boxes);

BoxNode query_bridge_box_from_ffb_result(
    const FindFreeBoxResult& result,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    int box_id);

int query_bridge_append_direct_corridor_box(
    BoxNode box,
    std::vector<BoxNode>& boxes,
    std::vector<BoxNode>& raw_boxes,
    QueryBridgeDirectCorridorCommitState& state);

std::vector<int> query_bridge_partition_neighbor_index_candidates(
    const AdaptiveGridPartition& partition,
    const BoxNode& box,
    double tolerance,
    const std::unordered_map<int, int>& box_id_to_index,
    int* raw_neighbor_count = nullptr);

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

QueryBridgeAdjacencyCandidateSet query_bridge_collect_adjacency_candidates(
    const std::vector<std::vector<int>>& sample_layers,
    int transition_hint,
    const QueryBridgeSampleAssimilationResult& sample_assimilation,
    const std::vector<int>& repair_indices,
    const AdaptiveGridPartition* partition,
    const BoxNode* partition_box,
    double tolerance,
    const std::unordered_map<int, int>& box_id_to_index);

bool query_bridge_current_corridor_boxes_cover_point(
    const AdaptiveGridPartition* partition,
    bool use_partition_cover_index,
    const std::vector<int>& corridor_new_box_indices,
    const BoxSpatialIndex& direct_box_index,
    const std::vector<BoxNode>& boxes,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double tolerance);

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
