#include <SBF/grower.h>

#include "grower_components.h"
#include "grower_internal.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rbf {

std::vector<Eigen::VectorXd> RrtGrower::select_initial_roots(
    const std::vector<Eigen::VectorXd>& seeds,
    StageContext& context) {
    const auto root = oracle_.planning_intervals();
    std::vector<Eigen::VectorXd> selected;
    std::vector<OracleNodeId> selected_leaves;

    auto try_add_root = [&](const Eigen::VectorXd& candidate,
                            bool user_seed,
                            bool enforce_distance) {
        if (candidate.size() != static_cast<int>(root.size()) ||
            !oracle_.contains_point(oracle_.root_node(), candidate)) {
            context.diagnostics().add_counter(user_seed
                ? "grower.root_seed_user_invalid"
                : "grower.root_seed_candidate_invalid");
            return false;
        }
        if (oracle_.point_in_collision(candidate)) {
            context.diagnostics().add_counter(user_seed
                ? "grower.root_seed_user_collision"
                : "grower.root_seed_candidate_collision");
            return false;
        }
        if (enforce_distance && config_.root_seed_min_normalized_linf > 0.0) {
            for (const auto& existing : selected) {
                if (normalized_linf_distance(root, existing, candidate) <
                    config_.root_seed_min_normalized_linf) {
                    context.diagnostics().add_counter("grower.root_seed_min_distance_rejected");
                    return false;
                }
            }
        }
        const OracleNodeId leaf = find_leaf_containing(oracle_, candidate);
        if (config_.root_seed_max_lca_depth >= 0 && leaf >= 0) {
            for (OracleNodeId selected_leaf : selected_leaves) {
                if (selected_leaf < 0) {
                    continue;
                }
                const int ancestor_depth = common_ancestor_depth(oracle_, leaf, selected_leaf);
                if (ancestor_depth < 0) {
                    context.diagnostics().add_counter("grower.root_seed_lca_unavailable");
                    continue;
                }
                if (ancestor_depth > config_.root_seed_max_lca_depth) {
                    context.diagnostics().add_counter("grower.root_seed_lca_rejected");
                    return false;
                }
            }
        }
        selected.push_back(candidate);
        selected_leaves.push_back(leaf);
        return true;
    };

    if (config_.root_seed_include_user_seeds) {
        for (const auto& seed : seeds) {
            try_add_root(seed, true, false);
        }
    }

    int target_count = static_cast<int>(selected.size()) +
                       std::max(0, config_.extra_random_roots);
    if (target_count <= 0) {
        target_count = 1;
    }

    const int candidates_per_round = std::max(1, config_.root_seed_candidate_count);
    int attempts = 0;
    int empty_rounds = 0;
    while (static_cast<int>(selected.size()) < target_count && empty_rounds < 4) {
        Eigen::VectorXd best_candidate;
        OracleNodeId best_leaf = kInvalidOracleNodeId;
        double best_score = -1.0;
        bool found = false;
        for (int candidate_index = 0; candidate_index < candidates_per_round; ++candidate_index) {
            attempts += 1;
            Eigen::VectorXd candidate = sample_uniform();
            if (oracle_.point_in_collision(candidate)) {
                context.diagnostics().add_counter("grower.root_seed_candidate_collision");
                continue;
            }
            double min_distance =
                selected.empty() ? 1.0 : std::numeric_limits<double>::infinity();
            for (const auto& existing : selected) {
                min_distance = std::min(min_distance,
                                        normalized_linf_distance(root, existing, candidate));
            }
            if (!selected.empty() && min_distance < config_.root_seed_min_normalized_linf) {
                context.diagnostics().add_counter("grower.root_seed_min_distance_rejected");
                continue;
            }
            const OracleNodeId leaf = find_leaf_containing(oracle_, candidate);
            bool lca_ok = true;
            if (config_.root_seed_max_lca_depth >= 0 && leaf >= 0) {
                for (OracleNodeId selected_leaf : selected_leaves) {
                    if (selected_leaf < 0) {
                        continue;
                    }
                    const int ancestor_depth = common_ancestor_depth(oracle_, leaf, selected_leaf);
                    if (ancestor_depth < 0) {
                        context.diagnostics().add_counter("grower.root_seed_lca_unavailable");
                        continue;
                    }
                    if (ancestor_depth > config_.root_seed_max_lca_depth) {
                        context.diagnostics().add_counter("grower.root_seed_lca_rejected");
                        lca_ok = false;
                        break;
                    }
                }
            }
            if (!lca_ok) {
                continue;
            }
            if (min_distance > best_score) {
                best_score = min_distance;
                best_candidate = std::move(candidate);
                best_leaf = leaf;
                found = true;
            }
        }
        if (!found) {
            empty_rounds += 1;
            continue;
        }
        selected.push_back(std::move(best_candidate));
        selected_leaves.push_back(best_leaf);
        empty_rounds = 0;
    }

    if (selected.empty()) {
        selected.push_back(sample_uniform());
    }

    double min_pair_distance = selected.size() <= 1
        ? 0.0
        : std::numeric_limits<double>::infinity();
    for (int outer = 0; outer < static_cast<int>(selected.size()); ++outer) {
        for (int inner = outer + 1; inner < static_cast<int>(selected.size()); ++inner) {
            min_pair_distance = std::min(
                min_pair_distance,
                normalized_linf_distance(root,
                                         selected[static_cast<std::size_t>(outer)],
                                         selected[static_cast<std::size_t>(inner)]));
        }
    }
    context.diagnostics().set_value("grower.root_seed_attempts",
                                    static_cast<double>(attempts));
    context.diagnostics().set_value("grower.root_seeds_target_count",
                                    static_cast<double>(target_count));
    context.diagnostics().set_value("grower.root_seeds_final_count",
                                    static_cast<double>(selected.size()));
    context.diagnostics().set_value("grower.root_seeds_min_normalized_linf",
                                    min_pair_distance);
    if (config_.intertree_goal_bias > 0.5 || config_.rrt_goal_bias > 0.5) {
        context.diagnostics().set_value("grower.goal_bias_high_warning", 1.0);
    }
    return selected;
}

} // namespace rbf
