#pragma once

#include "planning_forest_query_bridge_corridor_graph.h"
#include "planning_forest_query_bridge_corridor_tasks.h"

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/runtime.h>

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace rbf {

struct QueryBridgeSubdivisionRepairStats {
    int calls = 0;
    int added = 0;
    double ffb_ms = 0.0;
    std::vector<int> committed_indices;
};

struct QueryBridgeLateralRepairStats {
    int calls = 0;
    int added = 0;
    double ffb_ms = 0.0;
    std::vector<int> committed_indices;
};

struct QueryBridgeAdaptiveRepairStats {
    int calls = 0;
    int added = 0;
    int max_subdivisions_used = 0;
    double ffb_ms = 0.0;
    double initial_bad_fraction = 0.0;
    double final_bad_fraction = 0.0;
    std::vector<int> committed_indices;
    std::vector<int> final_bad;
};

struct QueryBridgeRepairSubdivisionOptions {
    int base_subdivisions = 0;
    int subdivisions = 0;
    std::vector<double> fractions;
};

struct QueryBridgeAdaptiveRepairOptions {
    int priority_mode = 1;
    bool enabled = true;
    double target_segment_fraction = 0.0;
    int max_subdivisions = 0;
    double fine_step = 0.0;
    int max_calls = 0;
};

struct QueryBridgeLateralRepairOptions {
    bool enabled = false;
    int dims = 0;
    int rounds = 1;
    int max_calls = 0;
    double offset = 0.0;
};

QueryBridgeSubdivisionRepairStats query_bridge_run_subdivision_repair_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    const std::vector<double>& fractions,
    const std::function<bool(int)>& transition_connected,
    const std::function<bool(const Eigen::VectorXd&)>& seed_covered,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&, int)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const Eigen::VectorXd&,
                                                       int)>& commit_box);

QueryBridgeLateralRepairStats query_bridge_run_lateral_repair_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    const std::vector<Interval>& domain,
    const QueryBridgeLateralRepairOptions& options,
    const std::function<bool(int)>& transition_connected,
    const std::function<bool(const Eigen::VectorXd&)>& seed_covered,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&, int)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const Eigen::VectorXd&,
                                                       int)>& commit_box);

void query_bridge_run_residual_segment_gap_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<std::vector<int>>& sample_layers,
    const std::vector<int>& final_bad,
    bool group_residual_gaps,
    const std::function<bool(int,
                             int,
                             const Eigen::VectorXd&,
                             const Eigen::VectorXd&,
                             int)>& insert_segment);

QueryBridgeAdaptiveRepairStats query_bridge_run_adaptive_repair_pass(
    StageContext& context,
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& initial_bad,
    int base_subdivisions,
    const QueryBridgeAdaptiveRepairOptions& options,
    const std::function<bool(int)>& transition_connected,
    const std::function<bool(const Eigen::VectorXd&)>& seed_covered,
    const std::function<std::vector<int>()>& bad_transitions,
    const std::function<double(const std::vector<int>&)>& bad_fraction,
    const std::function<FindFreeBoxResult(const Eigen::VectorXd&, int)>& find_box,
    const std::function<QueryBridgeFfbTaskCommitResult(FindFreeBoxResult&&,
                                                       const Eigen::VectorXd&,
                                                       int)>& commit_box);

std::vector<double> query_bridge_center_ordered_fractions(int subdivisions);

QueryBridgeRepairSubdivisionOptions query_bridge_repair_subdivision_options(int query_index);

QueryBridgeAdaptiveRepairOptions query_bridge_adaptive_repair_options(int query_index,
                                                                      int subdivisions,
                                                                      double audit_step,
                                                                      double sample_step);

QueryBridgeLateralRepairOptions query_bridge_lateral_repair_options(double sample_step);

std::vector<Eigen::VectorXd> query_bridge_lateral_candidates(
    const Eigen::VectorXd& seed,
    const Eigen::VectorXd& direction,
    const std::vector<Interval>& domain,
    int lateral_dims,
    int lateral_rounds,
    double lateral_offset);

std::vector<std::pair<int, int>> query_bridge_group_residual_gap_transitions(
    const std::vector<int>& final_bad,
    std::size_t layer_count,
    bool group_residual_gaps);

}  // namespace rbf
