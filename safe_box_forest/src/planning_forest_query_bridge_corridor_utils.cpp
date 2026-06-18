#include "planning_forest_query_bridge_corridor_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rbf {

QueryBridgeLocalDsu::QueryBridgeLocalDsu(std::size_t count) : parent(count) {
    for (std::size_t index = 0; index < parent.size(); ++index) {
        parent[index] = static_cast<int>(index);
    }
}

int QueryBridgeLocalDsu::add() {
    const int id = static_cast<int>(parent.size());
    parent.push_back(id);
    return id;
}

int QueryBridgeLocalDsu::find(int value) {
    int root = value;
    while (parent[static_cast<std::size_t>(root)] != root) {
        root = parent[static_cast<std::size_t>(root)];
    }
    while (parent[static_cast<std::size_t>(value)] != value) {
        const int next = parent[static_cast<std::size_t>(value)];
        parent[static_cast<std::size_t>(value)] = root;
        value = next;
    }
    return root;
}

void QueryBridgeLocalDsu::unite(int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 ||
        lhs >= static_cast<int>(parent.size()) ||
        rhs >= static_cast<int>(parent.size())) {
        return;
    }
    const int left = find(lhs);
    const int right = find(rhs);
    if (left != right) {
        parent[static_cast<std::size_t>(right)] = left;
    }
}

double query_bridge_seed_path_param(const std::vector<Eigen::VectorXd>& samples,
                                    const Eigen::VectorXd& seed,
                                    int transition_hint) {
    if (samples.empty()) {
        return 0.0;
    }
    if (transition_hint >= 0 &&
        transition_hint + 1 < static_cast<int>(samples.size())) {
        const Eigen::VectorXd& a = samples[static_cast<std::size_t>(transition_hint)];
        const Eigen::VectorXd& b = samples[static_cast<std::size_t>(transition_hint + 1)];
        const Eigen::VectorXd delta = b - a;
        const double denom = delta.squaredNorm();
        double u = 0.5;
        if (denom > 1e-18) {
            u = (seed - a).dot(delta) / denom;
            u = std::min(1.0, std::max(0.0, u));
        }
        return static_cast<double>(transition_hint) + u;
    }
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const double distance = (seed - samples[index]).squaredNorm();
        if (distance < best_distance) {
            best_distance = distance;
            best_index = index;
        }
    }
    return static_cast<double>(best_index);
}

double query_bridge_transition_length(const std::vector<Eigen::VectorXd>& samples,
                                      int transition) {
    if (transition < 0 ||
        transition + 1 >= static_cast<int>(samples.size())) {
        return 0.0;
    }
    return (samples[static_cast<std::size_t>(transition + 1)] -
            samples[static_cast<std::size_t>(transition)]).norm();
}

double query_bridge_transition_length_sum(const std::vector<Eigen::VectorXd>& samples,
                                          const std::vector<int>& transitions) {
    double total = 0.0;
    for (int transition : transitions) {
        total += query_bridge_transition_length(samples, transition);
    }
    return total;
}

double query_bridge_transition_fraction(const std::vector<Eigen::VectorXd>& samples,
                                        const std::vector<int>& transitions,
                                        double audited_bridge_length,
                                        double fallback_path_length) {
    const double denominator = audited_bridge_length > 1e-12
        ? audited_bridge_length
        : std::max(1e-12, fallback_path_length);
    return query_bridge_transition_length_sum(samples, transitions) / denominator;
}

std::vector<int> query_bridge_order_transitions_by_gap_length(
    const std::vector<Eigen::VectorXd>& samples,
    const std::vector<int>& transitions,
    int priority_mode) {
    if (priority_mode <= 0 || transitions.size() < 2) {
        return transitions;
    }
    struct GapGroup {
        int begin = 0;
        int end = 0;
        double length = 0.0;
    };
    std::vector<GapGroup> groups;
    groups.reserve(transitions.size());
    for (int transition : transitions) {
        if (groups.empty() || transition > groups.back().end + 1) {
            groups.push_back({transition,
                              transition,
                              query_bridge_transition_length(samples, transition)});
        } else {
            groups.back().end = transition;
            groups.back().length += query_bridge_transition_length(samples, transition);
        }
    }
    std::stable_sort(groups.begin(), groups.end(), [](const GapGroup& lhs,
                                                       const GapGroup& rhs) {
        if (std::abs(lhs.length - rhs.length) > 1e-12) {
            return lhs.length > rhs.length;
        }
        return lhs.begin < rhs.begin;
    });
    std::vector<int> ordered;
    ordered.reserve(transitions.size());
    for (const auto& group : groups) {
        const int center = (group.begin + group.end) / 2;
        ordered.push_back(center);
        for (int radius = 1;
             center - radius >= group.begin || center + radius <= group.end;
             ++radius) {
            if (center - radius >= group.begin) {
                ordered.push_back(center - radius);
            }
            if (center + radius <= group.end) {
                ordered.push_back(center + radius);
            }
        }
    }
    return ordered;
}

int query_bridge_nearest_nonempty_layer(const std::vector<std::vector<int>>& sample_layers,
                                        int start_index,
                                        int direction) {
    int index = start_index;
    while (index >= 0 && index < static_cast<int>(sample_layers.size())) {
        if (!sample_layers[static_cast<std::size_t>(index)].empty()) {
            return index;
        }
        index += direction;
    }
    return -1;
}

}  // namespace rbf
