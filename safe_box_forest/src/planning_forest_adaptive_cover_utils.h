#pragma once

#include <SBF/box_graph.h>
#include <SBF/leaf_sweep_grower.h>
#include <SBF/safe_box_forest.h>

#include <Eigen/Core>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

struct PriorityPruneStats {
    int free_before = 0;
    int free_after = 0;
    int collision_before = 0;
    int collision_after = 0;
};

PriorityPruneStats prune_leaf_sweep_to_priority(LeafSweepResult& result,
                                                std::vector<BoxNode>& live_boxes,
                                                std::vector<BoxNode>& raw_boxes,
                                                const std::vector<Eigen::VectorXd>& priority_points,
                                                double radius);

double adaptive_interval_volume(const std::vector<Interval>& intervals);
int adaptive_virtual_depth(OracleNodeId node);

struct AdaptiveFrontierItem {
    OracleNodeId node = -1;
    std::vector<Interval> intervals;
    int changed_dim = -1;
    int free_seed_hits = 0;
    double overlap_depth = 0.0;
    double overlap_ratio = 0.0;
    double score = 0.0;
};

struct AdaptiveDepthSnapshot {
    int depth = 0;
    int free_probe_count = 0;
    int covered_count = 0;
    int main_accessible_count = 0;
    int anchor_success_count = 0;
    int anchor_to_main_count = 0;
    int anchor_probe_attempts = 0;
    int cell_count = 0;
    int collision_count = 0;
    int island_count = 0;
    int main_island_cell_count = 0;
    double p_box_covered = 0.0;
    double p_main_accessible = 0.0;
    double main_connected_ratio = 0.0;
    double p_anchor_to_main_uncovered = 0.0;
    double probe_ms = 0.0;
    bool readiness_met = false;
    std::string stop_reason;
};

std::string adaptive_depth_snapshots_to_json(const std::vector<AdaptiveDepthSnapshot>& snapshots);

bool adaptive_virtual_split_node(const lect_database::SplitPolicyDescriptor& descriptor,
                                 const AdaptiveFrontierItem& item,
                                 AdaptiveFrontierItem& left,
                                 AdaptiveFrontierItem& right);

std::unordered_set<int> adaptive_largest_island_ids(const AdjacencyGraph& graph);

bool adaptive_has_adjacency_to_any(const std::vector<BoxNode>& boxes,
                                   const BoxNode& candidate,
                                   const std::unordered_set<int>* allowed_ids,
                                   double tolerance);

struct AdaptiveConnectivityDominance {
    int adjacent_free = 0;
    int adjacent_main = 0;
    int adjacent_other = 0;
    bool has_free_context = false;
    bool connector_candidate = false;
    bool single_component = false;
    bool isolated = true;
    double priority_delta = 0.0;
};

AdaptiveConnectivityDominance adaptive_connectivity_dominance(
    const std::vector<BoxNode>& boxes,
    const AdaptiveFrontierItem& item,
    const std::unordered_set<int>& main_ids,
    double tolerance);

double adaptive_frontier_score(const std::vector<BoxNode>& boxes,
                               const AdaptiveFrontierItem& item,
                               const std::unordered_set<int>& main_ids,
                               double overlap_depth_threshold,
                               double tolerance);

std::vector<Eigen::VectorXd> adaptive_generate_free_probes(DatabaseBoxOracle& oracle,
                                                           const std::vector<Interval>& domain,
                                                           int probe_count,
                                                           int rng_seed,
                                                           int& attempted);

int adaptive_count_seed_hits(const AdaptiveFrontierItem& item,
                             const std::vector<Eigen::VectorXd>& free_probes);

void adaptive_add_depth_counter(std::unordered_map<std::string, double>& diagnostics,
                                const std::string& prefix,
                                int depth);

class ScopedAdaptiveFullOverlapStats {
public:
    ScopedAdaptiveFullOverlapStats(DatabaseBoxOracle& oracle, bool enabled);
    ~ScopedAdaptiveFullOverlapStats();

    ScopedAdaptiveFullOverlapStats(const ScopedAdaptiveFullOverlapStats&) = delete;
    ScopedAdaptiveFullOverlapStats& operator=(const ScopedAdaptiveFullOverlapStats&) = delete;

private:
    DatabaseBoxOracle& oracle_;
    bool previous_ = false;
};

}  // namespace rbf
