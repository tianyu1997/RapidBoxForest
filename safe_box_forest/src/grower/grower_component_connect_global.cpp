#include <SBF/grower.h>

#include <SBF/runtime.h>

#include "grower_components.h"
#include "grower_internal.h"
#include "../query_runtime/planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace rbf {

bool RrtGrower::make_component_connect_seed(const std::vector<BoxNode>& boxes,
                                            Eigen::VectorXd& seed,
                                            Eigen::VectorXd& target,
                                            int& parent_box_id,
                                            int& root_id,
                                            int& target_root_id,
                                            int& pair_unknown_failures,
                                            bool& staged_target,
                                            double& component_gap_sq,
                                            StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.make_component_connect_seed");
    target_root_id = -1;
    pair_unknown_failures = 0;
    staged_target = false;
    component_gap_sq = std::numeric_limits<double>::infinity();
    if (boxes.size() < 2) {
        return false;
    }
    struct Candidate {
        int parent = -1;
        int target = -1;
        int parent_failures = std::numeric_limits<int>::max();
        int pair_unknown_failures = 0;
        bool staged = false;
        double gap_sq = std::numeric_limits<double>::infinity();
        double face_score = std::numeric_limits<double>::infinity();
        double center_sq = std::numeric_limits<double>::infinity();
        Eigen::VectorXd target_point;
    };

    const RootComponentGraph component_graph = build_root_component_graph(
        boxes,
        config_.adjacency_tolerance,
        config_.component_connect_island_aware);
    context.diagnostics().set_value("grower.component_connect_components",
                                    static_cast<double>(component_graph.components.size()));
    set_grower_max_diagnostic(context,
                       "grower.component_connect_connected_root_pairs_max",
                       static_cast<double>(component_graph.connected_cross_root_pairs));
    if (component_graph.components.size() < 2) {
        context.diagnostics().add_counter("grower.component_connect_all_roots_already_connected");
        return false;
    }
    std::vector<Candidate> candidates;

    auto consider_directed = [&](int parent_index,
                                 int target_index,
                                 double gap_sq) {
        const auto& parent_box = boxes[static_cast<std::size_t>(parent_index)];
        const auto& target_box = boxes[static_cast<std::size_t>(target_index)];
        const int failures = [&]() {
            const auto it = component_parent_failures_.find(parent_box.id);
            return it == component_parent_failures_.end() ? 0 : it->second;
        }();
        if (config_.component_connect_max_parent_failures > 0 &&
            failures >= config_.component_connect_max_parent_failures) {
            context.diagnostics().add_counter("grower.component_connect_parent_skipped");
            return;
        }
        Eigen::VectorXd target_point = closest_point_in_box(target_box, parent_box.center());
        if ((target_point - parent_box.center()).cwiseAbs().maxCoeff() <= 1e-12) {
            target_point = target_box.center();
        }
        bool staged = false;
        double staged_distance = 0.0;
        target_point = staged_component_target(parent_box, target_point, staged, staged_distance);
        double face_score = std::numeric_limits<double>::infinity();
        if (!best_uncovered_directed_face_score(boxes, parent_box, target_point, face_score, &context)) {
            context.diagnostics().add_counter("grower.component_connect_parent_closed_frontier");
            return;
        }
        const int pair_failures = component_pair_unknown_failures(parent_box.root_id, target_box.root_id);
        const double center_sq = (parent_box.center() - target_box.center()).squaredNorm();
        if (staged) {
            context.diagnostics().add_counter("grower.component_connect_staged_targets");
            set_grower_max_diagnostic(context,
                               "grower.component_connect_stage_distance_max",
                               staged_distance);
        }
        candidates.push_back({parent_index,
                              target_index,
                              failures,
                              pair_failures,
                              staged,
                              gap_sq,
                              face_score,
                              center_sq,
                              std::move(target_point)});
    };

    for (int lhs_component = 0; lhs_component < static_cast<int>(component_graph.components.size()); ++lhs_component) {
        for (int rhs_component = lhs_component + 1; rhs_component < static_cast<int>(component_graph.components.size()); ++rhs_component) {
            const auto& lhs_group = component_graph.components[static_cast<std::size_t>(lhs_component)];
            const auto& rhs_group = component_graph.components[static_cast<std::size_t>(rhs_component)];
            const double component_gap_sq = interval_bounds_gap_squared(lhs_group.bounds, rhs_group.bounds);
            for (int lhs_index : lhs_group.indices) {
                const auto& lhs = boxes[static_cast<std::size_t>(lhs_index)];
                for (int rhs_index : rhs_group.indices) {
                    const auto& rhs = boxes[static_cast<std::size_t>(rhs_index)];
                    const double gap_sq = box_gap_squared(lhs, rhs);
                    const double ranked_gap_sq = std::min(gap_sq, component_gap_sq);
                    if (lhs_group.indices.size() <= rhs_group.indices.size()) {
                        consider_directed(lhs_index, rhs_index, ranked_gap_sq);
                        consider_directed(rhs_index, lhs_index, ranked_gap_sq);
                    } else {
                        consider_directed(rhs_index, lhs_index, ranked_gap_sq);
                        consider_directed(lhs_index, rhs_index, ranked_gap_sq);
                    }
                }
            }
        }
    }

    if (candidates.empty()) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        if (std::abs(lhs.gap_sq - rhs.gap_sq) > 1e-18) return lhs.gap_sq < rhs.gap_sq;
        if (std::abs(lhs.face_score - rhs.face_score) > 1e-18) return lhs.face_score < rhs.face_score;
        if (lhs.parent_failures != rhs.parent_failures) return lhs.parent_failures < rhs.parent_failures;
        if (lhs.pair_unknown_failures != rhs.pair_unknown_failures) return lhs.pair_unknown_failures > rhs.pair_unknown_failures;
        return lhs.center_sq < rhs.center_sq;
    });
    const int choice_limit = std::max(1, std::min(config_.component_connect_candidate_limit,
                                                 static_cast<int>(candidates.size())));
    int choice_index = 0;
    if (choice_limit > 1) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        choice_index = std::min(choice_limit - 1,
                                static_cast<int>(u01(rng_) * u01(rng_) * choice_limit));
        set_grower_max_diagnostic(context,
                           "grower.component_connect_candidate_rank_max",
                           static_cast<double>(choice_index));
    }
    const Candidate& best = candidates[static_cast<std::size_t>(choice_index)];
    const auto& parent = boxes[static_cast<std::size_t>(best.parent)];
    const auto& target_box = boxes[static_cast<std::size_t>(best.target)];
    if (best.parent_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_retried_parent");
    }
    if (!make_frontier_seed_from_parent(boxes,
                                        best.parent,
                                        best.target_point,
                                        seed,
                                        parent_box_id,
                                        root_id,
                                        true,
                                        nullptr,
                                        nullptr,
                                        &context)) {
        context.diagnostics().add_counter("grower.component_connect_no_frontier_seed");
        const int failures = ++component_parent_failures_[parent.id];
        set_grower_max_diagnostic(context,
                           "grower.component_connect_parent_failure_max",
                           static_cast<double>(failures));
        return false;
    }
    target = best.target_point;
    target_root_id = target_box.root_id;
    pair_unknown_failures = best.pair_unknown_failures;
    staged_target = best.staged;
    component_gap_sq = best.gap_sq;
    return true;
}

}  // namespace rbf
