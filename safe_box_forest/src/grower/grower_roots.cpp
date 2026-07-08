#include <SBF/grower.h>

#include <SBF/oracle.h>
#include <SBF/runtime.h>

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

void RrtGrower::initialize_anchor_targets(const std::vector<Eigen::VectorXd>& roots,
                                          const std::vector<Eigen::VectorXd>& seeds,
                                          StageContext& context) {
    const int n_anchor_targets = std::max(0, config_.random_anchor_targets);
    random_anchor_targets_.reserve(config_.fixed_anchor_targets.size() +
                                   static_cast<std::size_t>(n_anchor_targets));
    std::vector<Eigen::VectorXd> anchor_reference_points = roots;
    if (anchor_reference_points.empty()) {
        anchor_reference_points = seeds;
    }
    std::vector<OracleNodeId> anchor_reference_leaves;
    anchor_reference_leaves.reserve(anchor_reference_points.size() +
                                    static_cast<std::size_t>(n_anchor_targets));
    for (const auto& point : anchor_reference_points) {
        anchor_reference_leaves.push_back(find_leaf_containing(oracle_, point));
    }
    for (const Eigen::VectorXd& anchor : config_.fixed_anchor_targets) {
        if (anchor.size() != static_cast<int>(oracle_.planning_intervals().size())) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_invalid");
            continue;
        }
        if (oracle_.point_in_collision(anchor)) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_collision");
            continue;
        }
        const Eigen::VectorXd clipped_anchor =
            clip_to_root_intervals(anchor, oracle_.planning_intervals());
        const double clip_delta = (clipped_anchor - anchor).cwiseAbs().maxCoeff();
        if (clip_delta > 1e-12) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_clipped_to_root");
            set_grower_max_diagnostic(context,
                               "grower.fixed_anchor_target_clip_delta_max",
                               clip_delta);
        }
        if (oracle_.point_in_collision(clipped_anchor)) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_clipped_collision");
            continue;
        }
        random_anchor_targets_.push_back(clipped_anchor);
        if (oracle_.contains_point(oracle_.root_node(), clipped_anchor)) {
            anchor_reference_points.push_back(clipped_anchor);
            anchor_reference_leaves.push_back(find_leaf_containing(oracle_, clipped_anchor));
        }
        context.diagnostics().add_counter("grower.fixed_anchor_targets");
    }
    const int anchor_candidates_per_round =
        std::max(1, config_.anchor_target_candidate_count);
    for (int anchor_index = 0; anchor_index < n_anchor_targets; ++anchor_index) {
        Eigen::VectorXd best_anchor;
        OracleNodeId best_leaf = kInvalidOracleNodeId;
        int best_lca_depth = std::numeric_limits<int>::max();
        double best_distance = -1.0;
        bool found_anchor = false;
        for (int candidate_index = 0;
             candidate_index < anchor_candidates_per_round;
             ++candidate_index) {
            Eigen::VectorXd candidate = sample_uniform();
            if (oracle_.point_in_collision(candidate)) {
                context.diagnostics().add_counter("grower.anchor_target_candidate_collision");
                continue;
            }
            const OracleNodeId leaf = find_leaf_containing(oracle_, candidate);
            int max_lca_depth = 0;
            bool lca_available = true;
            for (OracleNodeId reference_leaf : anchor_reference_leaves) {
                if (leaf < 0 || reference_leaf < 0) {
                    lca_available = false;
                    continue;
                }
                const int ancestor_depth = common_ancestor_depth(oracle_, leaf, reference_leaf);
                if (ancestor_depth < 0) {
                    lca_available = false;
                    continue;
                }
                max_lca_depth = std::max(max_lca_depth, ancestor_depth);
            }
            if (!lca_available) {
                context.diagnostics().add_counter("grower.anchor_target_lca_unavailable");
            }
            if (config_.anchor_target_max_lca_depth >= 0 &&
                lca_available &&
                max_lca_depth > config_.anchor_target_max_lca_depth) {
                context.diagnostics().add_counter("grower.anchor_target_lca_rejected");
                continue;
            }
            double min_distance = anchor_reference_points.empty()
                ? 1.0
                : std::numeric_limits<double>::infinity();
            const auto native_root = oracle_.planning_intervals();
            for (const auto& reference : anchor_reference_points) {
                min_distance = std::min(min_distance,
                                        normalized_linf_distance(native_root,
                                                                 reference,
                                                                 candidate));
            }
            if (!found_anchor ||
                max_lca_depth < best_lca_depth ||
                (max_lca_depth == best_lca_depth && min_distance > best_distance)) {
                best_anchor = std::move(candidate);
                best_leaf = leaf;
                best_lca_depth = max_lca_depth;
                best_distance = min_distance;
                found_anchor = true;
            }
        }
        if (!found_anchor) {
            best_anchor = sample_uniform();
            best_leaf = find_leaf_containing(oracle_, best_anchor);
            best_lca_depth = -1;
            best_distance = 0.0;
            context.diagnostics().add_counter("grower.anchor_target_fallback_uniform");
        }
        random_anchor_targets_.push_back(best_anchor);
        anchor_reference_points.push_back(best_anchor);
        anchor_reference_leaves.push_back(best_leaf);
        set_grower_max_diagnostic(context,
                           "grower.anchor_target_lca_depth_max",
                           static_cast<double>(best_lca_depth));
        if (best_lca_depth >= 0) {
            const double previous_min =
                context.diagnostics().value("grower.anchor_target_lca_depth_min");
            context.diagnostics().set_value(
                "grower.anchor_target_lca_depth_min",
                previous_min == 0.0 && anchor_index == 0
                    ? static_cast<double>(best_lca_depth)
                    : std::min(previous_min, static_cast<double>(best_lca_depth)));
        }
        set_grower_max_diagnostic(context,
                           "grower.anchor_target_min_distance_max",
                           best_distance);
    }
    if (!random_anchor_targets_.empty()) {
        context.diagnostics().set_value("grower.random_anchor_targets",
                                        static_cast<double>(random_anchor_targets_.size()));
    }
}

} // namespace rbf
