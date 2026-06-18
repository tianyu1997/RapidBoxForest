#pragma once

#include <Eigen/Core>

#include <cstddef>
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

}  // namespace rbf
