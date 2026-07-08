#include <SBF/grower.h>

#include "grower_components.h"
#include "grower_internal.h"

#include <algorithm>
#include <limits>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

namespace {

struct CachedComponentConnectSeed {
    bool resolved = false;
    bool found = false;
    Eigen::VectorXd seed;
    Eigen::VectorXd target;
    int parent_box_id = -1;
    int root_id = -1;
    int target_root_id = -1;
    int pair_unknown_failures = 0;
    bool staged_target = false;
    double component_gap_sq = std::numeric_limits<double>::infinity();
    GrowTraceFace selected_face;
    std::vector<GrowTraceFace> face_candidates;
};

}  // namespace

std::vector<GrowTaskRequest> RrtGrower::make_growth_task_requests(
    const std::vector<BoxNode>& boxes,
    const std::vector<Eigen::VectorXd>& roots,
    int first_task_id,
    int n_tasks,
    StageContext& context) {
    const RootGroups active_groups = group_boxes_by_root(boxes);
    if (n_tasks <= 0 || boxes.empty() || active_groups.roots.empty()) {
        return {};
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<GrowTaskRequest> requests;
    const std::size_t roots_per_sample = config_.expand_all_roots_per_sample
        ? active_groups.roots.size()
        : std::size_t{1};
    requests.reserve(static_cast<std::size_t>(n_tasks) * roots_per_sample);

    auto effective_intertree_goal_bias = [&](int sample_index) {
        double bias = std::clamp(config_.intertree_goal_bias, 0.0, 1.0);
        if (bias <= 0.5) {
            return bias;
        }
        const int period = std::max(1, config_.high_goal_bias_pulse_period);
        const bool pulse = period <= 1 || ((first_task_id + sample_index) % period == 0);
        if (pulse) {
            context.diagnostics().add_counter("grower.goal_bias_high_pulse_tasks");
            return bias;
        }
        context.diagnostics().add_counter("grower.goal_bias_high_capped_tasks");
        return std::min(std::clamp(config_.sustained_goal_bias_cap, 0.0, 0.5), bias);
    };

    auto sample_target = [&](int source_root_id,
                             int sample_index,
                             int& target_root_id,
                             bool& intertree,
                             GrowTargetType& target_type) {
        intertree = false;
        target_root_id = -1;
        if (source_root_id >= 0 && active_groups.roots.size() > 1 &&
            u01(rng_) < effective_intertree_goal_bias(sample_index)) {
            std::vector<int> candidates;
            candidates.reserve(active_groups.roots.size() - 1);
            for (int candidate_root : active_groups.roots) {
                if (candidate_root != source_root_id) {
                    candidates.push_back(candidate_root);
                }
            }
            if (!candidates.empty()) {
                std::uniform_int_distribution<int> pick(0, static_cast<int>(candidates.size()) - 1);
                target_root_id = candidates[static_cast<std::size_t>(pick(rng_))];
                intertree = true;
                target_type = GrowTargetType::IntertreeRoot;
                if (target_root_id >= 0 && target_root_id < static_cast<int>(roots.size())) {
                    return roots[static_cast<std::size_t>(target_root_id)];
                }
                const auto group_it = active_groups.by_root.find(target_root_id);
                if (group_it != active_groups.by_root.end() && !group_it->second.empty()) {
                    return boxes[static_cast<std::size_t>(group_it->second.front())].center();
                }
            }
        }
        if (roots.size() > 1 && u01(rng_) < config_.rrt_goal_bias) {
            std::uniform_int_distribution<int> pick_root_seed(0, static_cast<int>(roots.size()) - 1);
            target_root_id = pick_root_seed(rng_);
            target_type = GrowTargetType::QueryRoot;
            return roots[static_cast<std::size_t>(target_root_id)];
        }
        if (!random_anchor_targets_.empty() &&
            u01(rng_) < std::clamp(config_.anchor_target_prob, 0.0, 1.0)) {
            std::uniform_int_distribution<int> pick_anchor(0, static_cast<int>(random_anchor_targets_.size()) - 1);
            target_type = GrowTargetType::Uniform;
            context.diagnostics().add_counter("grower.target_category.anchor");
            return random_anchor_targets_[static_cast<std::size_t>(pick_anchor(rng_))];
        }
        if (u01(rng_) < config_.unexplored_sample_prob) {
            target_type = GrowTargetType::Unexplored;
            return sample_unexplored();
        }
        target_type = GrowTargetType::Uniform;
        return sample_uniform();
    };

    auto choose_target_category = [&](int source_root_id, int sample_index) -> GrowTargetType {
        const bool roots_multi = active_groups.roots.size() > 1;
        const bool can_component = config_.connect_mode && source_root_id >= 0 &&
                                   roots_multi && config_.component_connect_prob > 0.0;
        const bool can_intertree = source_root_id >= 0 && roots_multi;
        const bool can_rrt = roots.size() > 1;
        (void)sample_index;
        double p_cc = can_component ? std::clamp(config_.component_connect_prob, 0.0, 1.0) : 0.0;
        double p_inter = can_intertree ? std::clamp(config_.intertree_goal_bias, 0.0, 1.0) : 0.0;
        double p_rrt = can_rrt ? std::clamp(config_.rrt_goal_bias, 0.0, 1.0) : 0.0;
        double p_unexp = std::clamp(config_.unexplored_sample_prob, 0.0, 1.0);
        double p_uniform = std::clamp(config_.sample_uniform_prob, 0.0, 1.0);
        const double assigned = p_cc + p_inter + p_rrt + p_unexp + p_uniform;
        if (assigned < 1.0) {
            p_uniform += 1.0 - assigned;
        }
        const double r = u01(rng_);
        double acc = p_cc;
        if (r < acc) {
            return GrowTargetType::ComponentConnect;
        }
        acc += p_inter;
        if (r < acc) {
            return GrowTargetType::IntertreeRoot;
        }
        acc += p_rrt;
        if (r < acc) {
            return GrowTargetType::QueryRoot;
        }
        acc += p_unexp;
        if (r < acc) {
            return GrowTargetType::Unexplored;
        }
        return GrowTargetType::Uniform;
    };

    auto build_target_for_category = [&](int source_root_id,
                                         GrowTargetType category,
                                         int& target_root_id,
                                         bool& intertree,
                                         GrowTargetType& target_type) -> Eigen::VectorXd {
        intertree = false;
        target_root_id = -1;
        if (category == GrowTargetType::IntertreeRoot && source_root_id >= 0 &&
            active_groups.roots.size() > 1) {
            std::vector<int> candidates;
            candidates.reserve(active_groups.roots.size() - 1);
            for (int candidate_root : active_groups.roots) {
                if (candidate_root != source_root_id) {
                    candidates.push_back(candidate_root);
                }
            }
            if (!candidates.empty()) {
                std::uniform_int_distribution<int> pick(0, static_cast<int>(candidates.size()) - 1);
                target_root_id = candidates[static_cast<std::size_t>(pick(rng_))];
                intertree = true;
                target_type = GrowTargetType::IntertreeRoot;
                if (target_root_id >= 0 && target_root_id < static_cast<int>(roots.size())) {
                    return roots[static_cast<std::size_t>(target_root_id)];
                }
                const auto group_it = active_groups.by_root.find(target_root_id);
                if (group_it != active_groups.by_root.end() && !group_it->second.empty()) {
                    return boxes[static_cast<std::size_t>(group_it->second.front())].center();
                }
            }
        }
        if (category == GrowTargetType::QueryRoot && roots.size() > 1) {
            std::uniform_int_distribution<int> pick_root_seed(0, static_cast<int>(roots.size()) - 1);
            target_root_id = pick_root_seed(rng_);
            target_type = GrowTargetType::QueryRoot;
            return roots[static_cast<std::size_t>(target_root_id)];
        }
        if (category == GrowTargetType::Unexplored) {
            target_type = GrowTargetType::Unexplored;
            return sample_unexplored();
        }
        target_type = GrowTargetType::Uniform;
        return sample_uniform();
    };

    RootComponentGraph component_graph;
    const void* component_graph_ptr = nullptr;
    if (config_.connect_mode && active_groups.roots.size() > 1 && config_.component_connect_prob > 0.0) {
        component_graph = build_root_component_graph(boxes,
                                                     config_.adjacency_tolerance,
                                                     config_.component_connect_island_aware);
        component_graph_ptr = &component_graph;
        context.diagnostics().set_value("grower.component_connect_components",
                                        static_cast<double>(component_graph.components.size()));
        set_max_diagnostic(context,
                           "grower.component_connect_connected_root_pairs_max",
                           static_cast<double>(component_graph.connected_cross_root_pairs));
    }

    const bool use_component_connect_seed_cache =
        config_.expand_all_roots_per_sample &&
        config_.component_connect_candidate_limit <= 1 &&
        config_.connect_mode &&
        active_groups.roots.size() > 1 &&
        config_.component_connect_prob > 0.0;
    std::unordered_map<int, CachedComponentConnectSeed> component_connect_seed_cache;
    if (use_component_connect_seed_cache) {
        component_connect_seed_cache.reserve(active_groups.roots.size());
    }

    auto make_request = [&](int source_root_id,
                            int sample_index,
                            const Eigen::VectorXd* shared_anchor_target = nullptr) {
        GrowTaskRequest request;
        request.source_root_id = source_root_id;
        request.iteration = first_task_id + sample_index;
        if (shared_anchor_target != nullptr) {
            request.target = *shared_anchor_target;
            request.target_type = GrowTargetType::Uniform;
            context.diagnostics().add_counter("grower.target_category.shared_anchor");
            return request;
        }
        GrowTargetType chosen_category = GrowTargetType::Uniform;
        bool want_component_connect = false;
        if (config_.sample_categorical_allocation) {
            chosen_category = choose_target_category(source_root_id, sample_index);
            want_component_connect = (chosen_category == GrowTargetType::ComponentConnect);
        } else {
            want_component_connect = config_.connect_mode && source_root_id >= 0 &&
                                     active_groups.roots.size() > 1 &&
                                     config_.component_connect_prob > 0.0 &&
                                     u01(rng_) < config_.component_connect_prob;
        }
        if (want_component_connect && config_.connect_mode && source_root_id >= 0 &&
            active_groups.roots.size() > 1 && config_.component_connect_prob > 0.0) {
            bool found_component_connect_seed = false;
            if (use_component_connect_seed_cache) {
                auto [cache_it, inserted] = component_connect_seed_cache.try_emplace(source_root_id);
                (void)inserted;
                CachedComponentConnectSeed& cached = cache_it->second;
                if (!cached.resolved) {
                    cached.found = make_component_connect_seed_for_root(boxes,
                                                                        source_root_id,
                                                                        cached.seed,
                                                                        cached.target,
                                                                        cached.parent_box_id,
                                                                        cached.root_id,
                                                                        cached.target_root_id,
                                                                        cached.pair_unknown_failures,
                                                                        cached.staged_target,
                                                                        cached.component_gap_sq,
                                                                        &cached.selected_face,
                                                                        &cached.face_candidates,
                                                                        context,
                                                                        component_graph_ptr);
                    cached.resolved = true;
                    context.diagnostics().add_counter("grower.component_connect_seed_cache_misses");
                } else {
                    context.diagnostics().add_counter("grower.component_connect_seed_cache_hits");
                }
                if (cached.found) {
                    request.seed = cached.seed;
                    request.target = cached.target;
                    request.parent_box_id = cached.parent_box_id;
                    request.root_id = cached.root_id;
                    request.target_root_id = cached.target_root_id;
                    request.component_pair_unknown_failures = cached.pair_unknown_failures;
                    request.component_connect_staged_target = cached.staged_target;
                    request.component_connect_gap_sq = cached.component_gap_sq;
                    request.selected_face = cached.selected_face;
                    request.face_candidates = cached.face_candidates;
                    found_component_connect_seed = true;
                }
            } else {
                found_component_connect_seed = make_component_connect_seed_for_root(boxes,
                                                                                   source_root_id,
                                                                                   request.seed,
                                                                                   request.target,
                                                                                   request.parent_box_id,
                                                                                   request.root_id,
                                                                                   request.target_root_id,
                                                                                   request.component_pair_unknown_failures,
                                                                                   request.component_connect_staged_target,
                                                                                   request.component_connect_gap_sq,
                                                                                   &request.selected_face,
                                                                                   &request.face_candidates,
                                                                                   context,
                                                                                   component_graph_ptr);
            }
            if (found_component_connect_seed) {
                request.has_seed = true;
                request.intertree = true;
                request.component_connect = true;
                request.target_type = GrowTargetType::ComponentConnect;
                context.diagnostics().add_counter("grower.component_connect_attempts");
                context.diagnostics().add_counter("grower.component_connect_target_tasks");
                return request;
            }
            context.diagnostics().add_counter("grower.component_connect_target_no_candidate");
        }
        if (config_.sample_categorical_allocation) {
            request.target = build_target_for_category(source_root_id,
                                                       chosen_category,
                                                       request.target_root_id,
                                                       request.intertree,
                                                       request.target_type);
        } else {
            request.target = sample_target(source_root_id,
                                           sample_index,
                                           request.target_root_id,
                                           request.intertree,
                                           request.target_type);
        }
        return request;
    };

    {
        ScopedStageTimer request_timer(context.diagnostics(), "grower.rrt.make_growth_tasks.request_generation");
        for (int sample_index = 0; sample_index < n_tasks; ++sample_index) {
            const Eigen::VectorXd* shared_anchor_target = nullptr;
            if (config_.expand_all_roots_per_sample &&
                active_groups.roots.size() > 1 &&
                !random_anchor_targets_.empty() &&
                u01(rng_) < std::clamp(config_.anchor_target_prob, 0.0, 1.0)) {
                std::uniform_int_distribution<int> pick_anchor(0, static_cast<int>(random_anchor_targets_.size()) - 1);
                shared_anchor_target = &random_anchor_targets_[static_cast<std::size_t>(pick_anchor(rng_))];
            }
            if (config_.expand_all_roots_per_sample) {
                for (int source_root_id : active_groups.roots) {
                    requests.push_back(make_request(source_root_id, sample_index, shared_anchor_target));
                }
            } else {
                requests.push_back(make_request(-1, sample_index));
            }
        }

        if (config_.anchor_wave_targets_per_batch > 0 &&
            config_.expand_all_roots_per_sample &&
            active_groups.roots.size() > 1 &&
            !random_anchor_targets_.empty()) {
            const int wave_targets = std::min(std::max(0, config_.anchor_wave_targets_per_batch),
                                              static_cast<int>(random_anchor_targets_.size()));
            std::unordered_set<int> selected_anchor_indices;
            selected_anchor_indices.reserve(static_cast<std::size_t>(wave_targets));
            for (int wave_index = 0; wave_index < wave_targets; ++wave_index) {
                int anchor_index = -1;
                if (wave_targets >= static_cast<int>(random_anchor_targets_.size())) {
                    anchor_index = wave_index;
                } else {
                    std::uniform_int_distribution<int> pick_anchor(0, static_cast<int>(random_anchor_targets_.size()) - 1);
                    for (int attempt = 0; attempt < 8; ++attempt) {
                        const int candidate = pick_anchor(rng_);
                        if (selected_anchor_indices.insert(candidate).second) {
                            anchor_index = candidate;
                            break;
                        }
                    }
                    if (anchor_index < 0) {
                        for (int candidate = 0; candidate < static_cast<int>(random_anchor_targets_.size()); ++candidate) {
                            if (selected_anchor_indices.insert(candidate).second) {
                                anchor_index = candidate;
                                break;
                            }
                        }
                    }
                }
                if (anchor_index < 0 || anchor_index >= static_cast<int>(random_anchor_targets_.size())) {
                    continue;
                }
                const Eigen::VectorXd& anchor = random_anchor_targets_[static_cast<std::size_t>(anchor_index)];
                for (int source_root_id : active_groups.roots) {
                    requests.push_back(make_request(source_root_id, n_tasks + wave_index, &anchor));
                    context.diagnostics().add_counter("grower.anchor_wave_root_tasks");
                }
                context.diagnostics().add_counter("grower.anchor_wave_targets");
            }
        }
    }

    context.diagnostics().add_counter("grower.growth_target_samples", static_cast<double>(n_tasks));
    context.diagnostics().add_counter("grower.growth_tasks_planned", static_cast<double>(requests.size()));
    if (config_.expand_all_roots_per_sample && active_groups.roots.size() > 1) {
        context.diagnostics().add_counter("grower.all_root_sample_batches");
        context.diagnostics().add_counter("grower.all_root_sample_root_attempts", static_cast<double>(requests.size()));
    }
    int intertree_requests = 0;
    int component_connect_requests = 0;
    int query_root_requests = 0;
    int unexplored_requests = 0;
    int uniform_requests = 0;
    for (const auto& request : requests) {
        if (request.intertree && !request.component_connect) {
            intertree_requests += 1;
        }
        switch (request.target_type) {
        case GrowTargetType::ComponentConnect:
            component_connect_requests += 1;
            break;
        case GrowTargetType::QueryRoot:
            query_root_requests += 1;
            break;
        case GrowTargetType::Unexplored:
            unexplored_requests += 1;
            break;
        case GrowTargetType::Uniform:
            uniform_requests += 1;
            break;
        default:
            break;
        }
    }
    if (intertree_requests > 0) {
        context.diagnostics().add_counter("grower.intertree_goal_bias_tasks", static_cast<double>(intertree_requests));
    }
    if (component_connect_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.component_connect",
                                          static_cast<double>(component_connect_requests));
    }
    if (query_root_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.query_root",
                                          static_cast<double>(query_root_requests));
    }
    if (unexplored_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.unexplored",
                                          static_cast<double>(unexplored_requests));
    }
    if (uniform_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.uniform",
                                          static_cast<double>(uniform_requests));
    }

    return requests;
}

}  // namespace rbf
