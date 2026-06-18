#include <SBF/grower.h>

#include "grower_components.h"
#include "grower_internal.h"
#include "grower_options.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace rbf {

Eigen::VectorXd RrtGrower::staged_component_target(const BoxNode& parent,
                                                   const Eigen::Ref<const Eigen::VectorXd>& target,
                                                   bool& staged,
                                                   double& normalized_linf) const {
    staged = false;
    normalized_linf = 0.0;
    if (!config_.component_connect_staged_growth || config_.component_connect_stage_normalized_linf <= 0.0) {
        return target;
    }
    const auto root = oracle_.planning_intervals();
    if (target.size() != static_cast<int>(root.size())) {
        return target;
    }
    const Eigen::VectorXd parent_center = parent.center();
    normalized_linf = normalized_linf_distance(root, parent_center, target);
    const double stage = std::max(1e-6, config_.component_connect_stage_normalized_linf);
    if (normalized_linf <= stage) {
        return target;
    }
    Eigen::VectorXd staged_target = parent_center + (stage / normalized_linf) * (target - parent_center);
    for (int dim = 0; dim < staged_target.size(); ++dim) {
        staged_target[dim] = std::clamp(staged_target[dim],
                                        root[static_cast<std::size_t>(dim)].lo,
                                        root[static_cast<std::size_t>(dim)].hi);
    }
    staged = true;
    return staged_target;
}

int RrtGrower::component_pair_unknown_failures(int source_root_id,
                                               int target_root_id) const {
    if (source_root_id < 0 || target_root_id < 0 || source_root_id == target_root_id) {
        return 0;
    }
    const auto it = component_pair_unknown_failures_.find(component_pair_key(source_root_id, target_root_id));
    return it == component_pair_unknown_failures_.end() ? 0 : it->second;
}

void RrtGrower::record_component_connect_result(int source_root_id,
                                                int target_root_id,
                                                bool success,
                                                const FindFreeBoxResult* ffb_result,
                                                StageContext& context) {
    if (source_root_id < 0 || target_root_id < 0 || source_root_id == target_root_id) {
        return;
    }
    const std::uint64_t key = component_pair_key(source_root_id, target_root_id);
    if (success) {
        component_pair_unknown_failures_.erase(key);
        context.diagnostics().add_counter("grower.component_connect_pair_successes");
        return;
    }
    if (ffb_result != nullptr && ffb_result->hit_unknown_depth_cap) {
        const int failures = ++component_pair_unknown_failures_[key];
        context.diagnostics().add_counter("grower.component_connect_pair_unknown_failures");
        set_max_diagnostic(context,
                           "grower.component_connect_pair_unknown_failures_max",
                           static_cast<double>(failures));
    }
}

int RrtGrower::grow_component_connect_chain(std::vector<BoxNode>& boxes,
                                            FindFreeBoxService& ffb,
                                            const FindFreeBoxOptions& base_options,
                                            int depth_stage_index,
                                            int source_root_id,
                                            StageContext& context) {
    const int max_steps = std::max(0, config_.component_connect_chain_steps);
    if (max_steps <= 0 || source_root_id < 0 || boxes.empty()) {
        return 0;
    }
    const int max_added = config_.component_connect_chain_max_boxes > 0
        ? std::min(max_steps, config_.component_connect_chain_max_boxes)
        : max_steps;
    int added = 0;
    int failures = 0;
    for (int step = 0;
         step < max_steps && added < max_added &&
         static_cast<int>(boxes.size()) < config_.max_boxes &&
         !context.should_stop();
         ++step) {
        if (connected(boxes)) {
            context.diagnostics().add_counter("grower.component_connect_chain_connected_stop");
            break;
        }

        Eigen::VectorXd seed;
        Eigen::VectorXd target;
        int parent_box_id = -1;
        int root_id = -1;
        int target_root_id = -1;
        int pair_unknown_failures = 0;
        bool staged_target = false;
        double component_gap_sq = 0.0;
        GrowTraceFace selected_face;
        std::vector<GrowTraceFace> face_candidates;
        if (!make_component_connect_seed_for_root(boxes,
                                                  source_root_id,
                                                  seed,
                                                  target,
                                                  parent_box_id,
                                                  root_id,
                                                  target_root_id,
                                                  pair_unknown_failures,
                                                  staged_target,
                                                  component_gap_sq,
                                                  &selected_face,
                                                  &face_candidates,
                                                  context,
                                                  nullptr)) {
            context.diagnostics().add_counter("grower.component_connect_chain_no_seed");
            break;
        }

        GrowTask task;
        task.task_id = -1;
        task.iteration = step;
        task.seed = seed;
        task.target = target;
        task.target_type = GrowTargetType::ComponentConnect;
        task.parent_box_id = parent_box_id;
        task.root_id = root_id;
        task.source_root_id = source_root_id;
        task.target_root_id = target_root_id;
        task.intertree_goal_bias = true;
        task.component_connect_target = true;
        task.component_pair_unknown_failures = pair_unknown_failures;
        task.component_connect_staged_target = staged_target;
        task.component_connect_gap_sq = component_gap_sq;
        task.selected_face = selected_face;
        task.face_candidates = std::move(face_candidates);

        FindFreeBoxOptions task_options = component_connect_ffb_options(config_,
                                                                        context,
                                                                        base_options,
                                                                        depth_stage_index,
                                                                        pair_unknown_failures);
        task.ffb_depth = task_options.max_depth;
        FindFreeBoxResult observed_result;
        context.diagnostics().add_counter("grower.component_connect_chain_attempts");
        const int id = create_box(seed,
                                  parent_box_id,
                                  root_id,
                                  boxes,
                                  ffb,
                                  context,
                                  &task_options,
                                  &task,
                                  -1,
                                  &observed_result);
        if (id >= 0) {
            added += 1;
            failures = 0;
            source_root_id = root_id;
            context.diagnostics().add_counter("grower.component_connect_chain_added");
            context.diagnostics().add_counter("grower.component_connect_successes");
            component_parent_failures_.erase(parent_box_id);
            record_component_connect_result(source_root_id,
                                            target_root_id,
                                            true,
                                            nullptr,
                                            context);
            continue;
        }

        failures += 1;
        context.diagnostics().add_counter("grower.component_connect_chain_failures");
        const int parent_failures = ++component_parent_failures_[parent_box_id];
        set_max_diagnostic(context,
                           "grower.component_connect_parent_failure_max",
                           static_cast<double>(parent_failures));
        record_component_connect_result(source_root_id,
                                        target_root_id,
                                        false,
                                        observed_result.found || observed_result.fail_code != 0 ? &observed_result : nullptr,
                                        context);
        if (failures >= 2) {
            context.diagnostics().add_counter("grower.component_connect_chain_failure_stop");
            break;
        }
    }
    if (added > 0) {
        set_max_diagnostic(context,
                           "grower.component_connect_chain_added_max",
                           static_cast<double>(added));
    }
    return added;
}

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
    set_max_diagnostic(context,
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
            set_max_diagnostic(context,
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
        set_max_diagnostic(context,
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
        set_max_diagnostic(context,
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

bool RrtGrower::make_component_connect_seed_for_root(const std::vector<BoxNode>& boxes,
                                                     int source_root_id,
                                                     Eigen::VectorXd& seed,
                                                     Eigen::VectorXd& target,
                                                     int& parent_box_id,
                                                     int& root_id,
                                                     int& target_root_id,
                                                     int& pair_unknown_failures,
                                                     bool& staged_target,
                                                     double& component_gap_sq,
                                                     GrowTraceFace* face,
                                                     std::vector<GrowTraceFace>* face_candidates,
                                                     StageContext& context,
                                                     const void* component_graph_override) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.make_component_connect_seed_for_root");
    target_root_id = -1;
    pair_unknown_failures = 0;
    staged_target = false;
    component_gap_sq = std::numeric_limits<double>::infinity();

    RootComponentGraph local_component_graph;
    const auto* component_graph_ptr = static_cast<const RootComponentGraph*>(component_graph_override);
    if (component_graph_ptr == nullptr) {
        local_component_graph = build_root_component_graph(boxes,
                                                           config_.adjacency_tolerance,
                                                           config_.component_connect_island_aware);
        component_graph_ptr = &local_component_graph;
        context.diagnostics().set_value("grower.component_connect_components",
                                        static_cast<double>(component_graph_ptr->components.size()));
        set_max_diagnostic(context,
                           "grower.component_connect_connected_root_pairs_max",
                           static_cast<double>(component_graph_ptr->connected_cross_root_pairs));
    }
    const RootComponentGraph& component_graph = *component_graph_ptr;
    const auto source_it = component_graph.groups.by_root.find(source_root_id);
    const auto source_component_it = component_graph.root_to_component.find(source_root_id);
    if (source_it == component_graph.groups.by_root.end() ||
        source_component_it == component_graph.root_to_component.end() ||
        component_graph.components.size() < 2) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }

    struct TargetSummary {
        int component = -1;
        std::vector<int> roots;
        std::vector<int> indices;
        std::vector<Interval> bounds;
        Eigen::VectorXd center;
        int root_order_gap = std::numeric_limits<int>::max();
        double gap_sq = std::numeric_limits<double>::infinity();
        double center_sq = std::numeric_limits<double>::infinity();
    };
    struct Candidate {
        int parent = -1;
        int target_summary = -1;
        int target_root = -1;
        int parent_failures = std::numeric_limits<int>::max();
        int pair_unknown_failures = 0;
        int root_order_gap = std::numeric_limits<int>::max();
        bool staged = false;
        Eigen::VectorXd target_point;
        double target_gap_sq = std::numeric_limits<double>::infinity();
        double face_score = std::numeric_limits<double>::infinity();
        double center_sq = std::numeric_limits<double>::infinity();
    };

    const int source_component = source_component_it->second;
    const RootComponent& source_summary = component_graph.components[static_cast<std::size_t>(source_component)];
    const auto& source_bounds = source_summary.bounds;
    const Eigen::VectorXd& source_center = source_summary.center;
    const int neighbor_window = std::max(1, config_.component_connect_neighbor_root_window);
    std::vector<TargetSummary> targets;
    targets.reserve(component_graph.components.size() - 1);
    for (const RootComponent& component : component_graph.components) {
        if (component.id == source_component || component.indices.empty()) {
            continue;
        }
        TargetSummary target;
        target.component = component.id;
        target.roots = component.roots;
        target.indices = component.indices;
        target.bounds = component.bounds;
        target.center = component.center;
        for (int candidate_root : target.roots) {
            if (candidate_root == source_root_id) {
                continue;
            }
            target.root_order_gap = std::min(target.root_order_gap, std::abs(candidate_root - source_root_id));
        }
        target.gap_sq = interval_bounds_gap_squared(source_bounds, target.bounds);
        target.center_sq = (source_center - target.center).squaredNorm();
        if (config_.component_connect_neighbor_root_bias && target.root_order_gap <= neighbor_window) {
            context.diagnostics().add_counter("grower.component_connect_neighbor_root_targets");
        }
        targets.push_back(std::move(target));
    }
    if (targets.empty()) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }
    auto target_less = [&](const TargetSummary& lhs, const TargetSummary& rhs) {
        if (config_.component_connect_neighbor_root_bias) {
            const bool lhs_neighbor = lhs.root_order_gap <= neighbor_window;
            const bool rhs_neighbor = rhs.root_order_gap <= neighbor_window;
            if (lhs_neighbor != rhs_neighbor) return lhs_neighbor;
            if (lhs_neighbor && lhs.root_order_gap != rhs.root_order_gap) {
                return lhs.root_order_gap < rhs.root_order_gap;
            }
        }
        if (std::abs(lhs.gap_sq - rhs.gap_sq) > 1e-18) return lhs.gap_sq < rhs.gap_sq;
        return lhs.center_sq < rhs.center_sq;
    };
    std::sort(targets.begin(), targets.end(), target_less);
    const int target_limit = std::max(1, std::min(config_.component_connect_candidate_limit,
                                                 static_cast<int>(targets.size())));
    set_max_diagnostic(context,
                       "grower.component_connect_target_roots_considered_max",
                       static_cast<double>(target_limit));

    std::vector<Candidate> coarse_candidates;
    for (int parent_index : source_it->second) {
        const auto& parent = boxes[static_cast<std::size_t>(parent_index)];
        const auto failure_it = component_parent_failures_.find(parent.id);
        const int failures = failure_it == component_parent_failures_.end() ? 0 : failure_it->second;
        if (config_.component_connect_max_parent_failures > 0 &&
            failures >= config_.component_connect_max_parent_failures) {
            context.diagnostics().add_counter("grower.component_connect_parent_skipped");
            continue;
        }
        const Eigen::VectorXd parent_center = parent.center();
        for (int target_index = 0; target_index < target_limit; ++target_index) {
            const auto& target_summary = targets[static_cast<std::size_t>(target_index)];
            Eigen::VectorXd target_point = closest_point_in_intervals(target_summary.bounds, parent_center);
            if ((target_point - parent_center).cwiseAbs().maxCoeff() <= 1e-12) {
                target_point = target_summary.center;
            }
            bool staged = false;
            double staged_distance = 0.0;
            target_point = staged_component_target(parent, target_point, staged, staged_distance);
            double face_score = std::numeric_limits<double>::infinity();
            if (!best_uncovered_directed_face_score(boxes, parent, target_point, face_score, &context)) {
                context.diagnostics().add_counter("grower.component_connect_parent_closed_frontier");
                continue;
            }
            int pair_failures = 0;
            int target_root_guess = target_summary.roots.empty() ? -1 : target_summary.roots.front();
            for (int candidate_target_root : target_summary.roots) {
                const int candidate_failures = component_pair_unknown_failures(source_root_id, candidate_target_root);
                if (candidate_failures > pair_failures) {
                    pair_failures = candidate_failures;
                    target_root_guess = candidate_target_root;
                }
            }
            coarse_candidates.push_back({parent_index,
                                         target_index,
                                         target_root_guess,
                                         failures,
                                         pair_failures,
                                         target_summary.root_order_gap,
                                         staged,
                                         std::move(target_point),
                                         target_summary.gap_sq,
                                         face_score,
                                         (parent_center - target_summary.center).squaredNorm()});
            if (staged) {
                context.diagnostics().add_counter("grower.component_connect_staged_targets");
                set_max_diagnostic(context,
                                   "grower.component_connect_stage_distance_max",
                                   staged_distance);
            }
        }
    }

    if (coarse_candidates.empty()) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }
    auto candidate_less = [&](const Candidate& lhs, const Candidate& rhs) {
        if (config_.component_connect_neighbor_root_bias) {
            const bool lhs_neighbor = lhs.root_order_gap <= neighbor_window;
            const bool rhs_neighbor = rhs.root_order_gap <= neighbor_window;
            if (lhs_neighbor != rhs_neighbor) return lhs_neighbor;
            if (lhs_neighbor && lhs.root_order_gap != rhs.root_order_gap) {
                return lhs.root_order_gap < rhs.root_order_gap;
            }
        }
        if (std::abs(lhs.target_gap_sq - rhs.target_gap_sq) > 1e-18) return lhs.target_gap_sq < rhs.target_gap_sq;
        if (std::abs(lhs.face_score - rhs.face_score) > 1e-18) return lhs.face_score < rhs.face_score;
        if (lhs.parent_failures != rhs.parent_failures) return lhs.parent_failures < rhs.parent_failures;
        if (lhs.pair_unknown_failures != rhs.pair_unknown_failures) return lhs.pair_unknown_failures > rhs.pair_unknown_failures;
        return lhs.center_sq < rhs.center_sq;
    };
    std::sort(coarse_candidates.begin(), coarse_candidates.end(), candidate_less);

    std::vector<Candidate> candidates;
    if (config_.component_connect_staged_growth) {
        const int coarse_limit = std::min(static_cast<int>(coarse_candidates.size()),
                                         std::max(4, std::max(1, config_.component_connect_candidate_limit) * 2));
        candidates.reserve(static_cast<std::size_t>(coarse_limit));
        for (int coarse_index = 0; coarse_index < coarse_limit; ++coarse_index) {
            candidates.push_back(coarse_candidates[static_cast<std::size_t>(coarse_index)]);
        }
        context.diagnostics().add_counter("grower.component_connect_component_target_tasks");
        set_max_diagnostic(context,
                           "grower.component_connect_component_target_limit_max",
                           static_cast<double>(coarse_limit));
    } else {
        const int refine_target = std::max(8, std::max(1, config_.component_connect_candidate_limit) * 4);
        const int refine_limit = std::min(static_cast<int>(coarse_candidates.size()), refine_target);
        context.diagnostics().add_counter("grower.component_connect_actual_target_refine_calls");
        set_max_diagnostic(context,
                           "grower.component_connect_actual_target_refine_limit_max",
                           static_cast<double>(refine_limit));

        candidates.reserve(static_cast<std::size_t>(refine_limit));
        int target_boxes_scanned = 0;
        for (int coarse_index = 0; coarse_index < refine_limit; ++coarse_index) {
            const Candidate& coarse = coarse_candidates[static_cast<std::size_t>(coarse_index)];
            const auto& parent = boxes[static_cast<std::size_t>(coarse.parent)];
            const Eigen::VectorXd parent_center = parent.center();
            Candidate best = coarse;
            bool found_actual_target = false;
            if (coarse.target_summary >= 0 && coarse.target_summary < static_cast<int>(targets.size())) {
                const auto& target_summary = targets[static_cast<std::size_t>(coarse.target_summary)];
                for (int target_box_index : target_summary.indices) {
                    const auto& target_box = boxes[static_cast<std::size_t>(target_box_index)];
                    Eigen::VectorXd target_point = closest_point_in_box(target_box, parent_center);
                    if ((target_point - parent_center).cwiseAbs().maxCoeff() <= 1e-12) {
                        target_point = target_box.center();
                    }
                    bool staged = false;
                    double staged_distance = 0.0;
                    target_point = staged_component_target(parent, target_point, staged, staged_distance);
                    double face_score = std::numeric_limits<double>::infinity();
                    if (!best_uncovered_directed_face_score(boxes, parent, target_point, face_score, &context)) {
                        continue;
                    }
                    target_boxes_scanned += 1;
                    const double target_gap_sq = box_gap_squared(parent, target_box);
                    const double center_sq = (parent_center - target_box.center()).squaredNorm();
                    if (!found_actual_target ||
                        target_gap_sq < best.target_gap_sq - 1e-18 ||
                        (std::abs(target_gap_sq - best.target_gap_sq) <= 1e-18 &&
                         (face_score < best.face_score - 1e-18 ||
                          (std::abs(face_score - best.face_score) <= 1e-18 && center_sq < best.center_sq)))) {
                        best.target_point = std::move(target_point);
                        best.target_gap_sq = target_gap_sq;
                        best.face_score = face_score;
                        best.center_sq = center_sq;
                        best.target_root = target_box.root_id;
                        best.pair_unknown_failures = component_pair_unknown_failures(parent.root_id, target_box.root_id);
                        best.staged = staged;
                        found_actual_target = true;
                        if (staged) {
                            context.diagnostics().add_counter("grower.component_connect_staged_targets");
                            set_max_diagnostic(context,
                                               "grower.component_connect_stage_distance_max",
                                               staged_distance);
                        }
                    }
                }
            }
            if (found_actual_target) {
                context.diagnostics().add_counter("grower.component_connect_actual_target_refined");
            }
            candidates.push_back(std::move(best));
        }
        set_max_diagnostic(context,
                           "grower.component_connect_actual_target_boxes_scanned_max",
                           static_cast<double>(target_boxes_scanned));
    }
    std::sort(candidates.begin(), candidates.end(), candidate_less);

    const int choice_limit = std::max(1, std::min(config_.component_connect_candidate_limit,
                                                 static_cast<int>(candidates.size())));
    int choice_index = 0;
    if (choice_limit > 1) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        choice_index = std::min(choice_limit - 1,
                                static_cast<int>(u01(rng_) * u01(rng_) * choice_limit));
        set_max_diagnostic(context,
                           "grower.component_connect_candidate_rank_max",
                           static_cast<double>(choice_index));
    }
    const Candidate& best = candidates[static_cast<std::size_t>(choice_index)];
    const auto& parent = boxes[static_cast<std::size_t>(best.parent)];
    if (config_.component_connect_neighbor_root_bias && best.root_order_gap <= neighbor_window) {
        context.diagnostics().add_counter("grower.component_connect_neighbor_root_selected");
        set_max_diagnostic(context,
                           "grower.component_connect_neighbor_root_gap_max",
                           static_cast<double>(best.root_order_gap));
    }
    if (best.parent_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_retried_parent");
    }
    if (best.pair_unknown_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_retried_unknown_pair");
    }
    GrowTraceFace selected_face;
    GrowTraceFace* selected_face_out = face != nullptr ? face : &selected_face;
    if (!make_frontier_seed_from_parent(boxes,
                                        best.parent,
                                        best.target_point,
                                        seed,
                                        parent_box_id,
                                        root_id,
                                        config_.component_connect_require_target_direction,
                                        selected_face_out,
                                        face_candidates,
                                        &context)) {
        context.diagnostics().add_counter("grower.component_connect_no_frontier_seed");
        const int failures = ++component_parent_failures_[parent.id];
        set_max_diagnostic(context,
                           "grower.component_connect_parent_failure_max",
                           static_cast<double>(failures));
        return false;
    }
    const double lateral_prob = std::clamp(config_.component_connect_lateral_sample_prob, 0.0, 1.0);
    if (lateral_prob > 0.0 && selected_face_out->valid) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        if (u01(rng_) < lateral_prob) {
            const auto root = oracle_.planning_intervals();
            const double seed_epsilon = std::max(config_.boundary_epsilon, 0.25 * config_.adjacency_tolerance);
            const int attempts = std::max(1, config_.component_connect_lateral_sample_attempts);
            bool applied = false;
            for (int attempt = 0; attempt < attempts; ++attempt) {
                Eigen::VectorXd candidate_seed = seed;
                for (int dim = 0; dim < parent.n_dims(); ++dim) {
                    if (dim == selected_face_out->dim) {
                        continue;
                    }
                    const double lo = parent.joint_intervals[static_cast<std::size_t>(dim)].lo;
                    const double hi = parent.joint_intervals[static_cast<std::size_t>(dim)].hi;
                    const double safe_lo = lo + seed_epsilon;
                    const double safe_hi = hi - seed_epsilon;
                    if (safe_lo <= safe_hi) {
                        std::uniform_real_distribution<double> coord(safe_lo, safe_hi);
                        candidate_seed[dim] = coord(rng_);
                    } else {
                        candidate_seed[dim] = 0.5 * (lo + hi);
                    }
                    candidate_seed[dim] = std::clamp(candidate_seed[dim],
                                                     root[static_cast<std::size_t>(dim)].lo,
                                                     root[static_cast<std::size_t>(dim)].hi);
                }
                if (!seed_covered_by_frontier_cache(boxes, candidate_seed, &context)) {
                    seed = std::move(candidate_seed);
                    context.diagnostics().add_counter("grower.component_connect_lateral_seed");
                    set_max_diagnostic(context,
                                       "grower.component_connect_lateral_attempt_max",
                                       static_cast<double>(attempt + 1));
                    applied = true;
                    break;
                }
                context.diagnostics().add_counter("grower.component_connect_lateral_seed_covered");
            }
            if (!applied) {
                context.diagnostics().add_counter("grower.component_connect_lateral_seed_fallback_direct");
            }
        }
    }
    target = best.target_point;
    target_root_id = best.target_root;
    pair_unknown_failures = best.pair_unknown_failures;
    staged_target = best.staged;
    component_gap_sq = best.target_gap_sq;
    return true;
}
}  // namespace rbf
