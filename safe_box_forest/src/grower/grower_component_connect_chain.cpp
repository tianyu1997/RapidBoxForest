#include <SBF/grower.h>

#include "grower_components.h"
#include "grower_internal.h"
#include "grower_options.h"

#include <algorithm>
#include <cstdint>
#include <utility>
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

void RrtGrower::record_component_connect_success(int parent_box_id,
                                                 int source_root_id,
                                                 int target_root_id,
                                                 StageContext& context) {
    context.diagnostics().add_counter("grower.component_connect_successes");
    component_parent_failures_.erase(parent_box_id);
    record_component_connect_result(source_root_id,
                                    target_root_id,
                                    true,
                                    nullptr,
                                    context);
}

void RrtGrower::record_component_connect_failure(int parent_box_id,
                                                 int source_root_id,
                                                 int target_root_id,
                                                 const FindFreeBoxResult* ffb_result,
                                                 StageContext& context) {
    context.diagnostics().add_counter("grower.component_connect_failures");
    const int failures = ++component_parent_failures_[parent_box_id];
    set_max_diagnostic(context,
                       "grower.component_connect_parent_failure_max",
                       static_cast<double>(failures));
    record_component_connect_result(source_root_id,
                                    target_root_id,
                                    false,
                                    ffb_result,
                                    context);
}

int RrtGrower::record_component_connect_success_and_extend(std::vector<BoxNode>& boxes,
                                                           FindFreeBoxService& ffb,
                                                           const FindFreeBoxOptions& base_options,
                                                           int depth_stage_index,
                                                           int parent_box_id,
                                                           int source_root_id,
                                                           int target_root_id,
                                                           StageContext& context) {
    record_component_connect_success(parent_box_id,
                                     source_root_id,
                                     target_root_id,
                                     context);
    return grow_component_connect_chain(boxes,
                                        ffb,
                                        base_options,
                                        depth_stage_index,
                                        source_root_id,
                                        context);
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
            record_component_connect_success(parent_box_id,
                                             source_root_id,
                                             target_root_id,
                                             context);
            continue;
        }

        failures += 1;
        context.diagnostics().add_counter("grower.component_connect_chain_failures");
        record_component_connect_failure(parent_box_id,
                                         source_root_id,
                                         target_root_id,
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

}  // namespace rbf
