#pragma once

#include <Eigen/Core>

#include <SBF/box_graph.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace rbf {

struct QueryBridgeResidualMilestone {
    double param = 0.0;
    Eigen::VectorXd point;
    int box_index = -1;
};

struct QueryBridgeDirectFfbTask {
    Eigen::VectorXd seed;
    std::size_t sample_index = 0;
    int transition_hint = 0;
};

struct QueryBridgeDirectFfbTaskBuildOptions {
    int max_transition_hint = 0;
    int max_group_seeds = 3;
    bool grouped_direct_seeds = false;
    bool center_out_direct_tasks = false;
};

struct QueryBridgeDirectFfbTaskBuildResult {
    std::vector<QueryBridgeDirectFfbTask> tasks;
    int uncovered_gap_groups = 0;
};

struct QueryBridgeLocalDsu {
    std::vector<int> parent;

    explicit QueryBridgeLocalDsu(std::size_t count = 0);

    int add();
    int find(int value);
    void unite(int lhs, int rhs);
};

double query_bridge_seed_path_param(const std::vector<Eigen::VectorXd>& samples,
                                    const Eigen::VectorXd& seed,
                                    int transition_hint);

double query_bridge_transition_length(const std::vector<Eigen::VectorXd>& samples,
                                      int transition);

double query_bridge_transition_length_sum(const std::vector<Eigen::VectorXd>& samples,
                                          const std::vector<int>& transitions);

double query_bridge_transition_fraction(const std::vector<Eigen::VectorXd>& samples,
                                        const std::vector<int>& transitions,
                                        double audited_bridge_length,
                                        double fallback_path_length);

std::vector<int> query_bridge_order_transitions_by_gap_length(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    int priority_mode);

int query_bridge_nearest_nonempty_layer(const std::vector<std::vector<int>>& sample_layers,
                                        int start_index,
                                        int direction);

QueryBridgeDirectFfbTaskBuildResult query_bridge_build_direct_ffb_tasks(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<bool>& covered,
    const QueryBridgeDirectFfbTaskBuildOptions& options);

std::vector<double> query_bridge_center_ordered_fractions(int subdivisions);

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

std::vector<QueryBridgeResidualMilestone> query_bridge_compact_residual_milestones(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<std::vector<int>>& sample_layers,
    const std::vector<QueryBridgeResidualMilestone>& repair_milestones,
    int box_count,
    QueryBridgeLocalDsu& dsu);

}  // namespace rbf
