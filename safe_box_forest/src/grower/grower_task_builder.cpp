#include <SBF/grower.h>

#include <utility>

namespace rbf {

int RrtGrower::resolve_task_batch_size(const std::vector<BoxNode>& boxes,
                                       StageContext& context) const {
    if (boxes.empty() || context.executor().n_threads() <= 1) {
        return 1;
    }
    const int batch_size = config_.task_batch_size > 1
        ? config_.task_batch_size
        : context.runtime().batch_size;
    if (batch_size <= 1) {
        return 1;
    }
    const int threshold = config_.parallel_threshold > 0
        ? config_.parallel_threshold
        : context.runtime().parallel_threshold;
    if (threshold > 0 && static_cast<int>(boxes.size()) < threshold) {
        return 1;
    }
    return batch_size;
}

std::vector<GrowTask> RrtGrower::make_growth_tasks(const std::vector<BoxNode>& boxes,
                                                   const std::vector<Eigen::VectorXd>& roots,
                                                   int first_task_id,
                                                   int n_tasks,
                                                   const FindFreeBoxOptions& base_options,
                                                   StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.make_growth_tasks");
    if (n_tasks <= 0 || boxes.empty()) {
        return {};
    }

    std::vector<GrowTaskRequest> requests =
        make_growth_task_requests(boxes, roots, first_task_id, n_tasks, context);
    if (requests.empty()) {
        return {};
    }

    std::vector<GrowTask> tasks(requests.size());
    {
        ScopedStageTimer seed_timer(context.diagnostics(), "grower.rrt.make_growth_tasks.frontier_seed_selection");
        context.executor().parallel_for(0, static_cast<int>(requests.size()), [&](int task_index) {
            if (context.should_stop()) {
                return;
            }
            const auto& request = requests[static_cast<std::size_t>(task_index)];
            Eigen::VectorXd seed;
            int parent_box_id = -1;
            int root_id = -1;
            GrowTraceFace selected_face = request.selected_face;
            std::vector<GrowTraceFace> face_candidates = request.face_candidates;
            bool made_seed = request.has_seed;
            if (request.has_seed) {
                seed = request.seed;
                parent_box_id = request.parent_box_id;
                root_id = request.root_id;
            } else {
                const bool target_seed = request.source_root_id >= 0
                    ? make_frontier_seed_for_root(boxes,
                                                  request.source_root_id,
                                                  request.target,
                                                  seed,
                                                  parent_box_id,
                                                  root_id,
                                                  nullptr,
                                                  &selected_face,
                                                  &face_candidates)
                    : make_frontier_seed(boxes,
                                         request.target,
                                         seed,
                                         parent_box_id,
                                         root_id,
                                         nullptr,
                                         &selected_face,
                                         &face_candidates);
                if (!target_seed) {
                    return;
                }
                made_seed = true;
            }
            if (!made_seed) {
                return;
            }
            GrowTask task;
            task.task_id = first_task_id + task_index;
            task.iteration = request.iteration;
            task.seed = std::move(seed);
            task.target = request.target;
            task.target_type = request.target_type;
            task.parent_box_id = parent_box_id;
            task.root_id = root_id;
            task.source_root_id = request.source_root_id;
            task.target_root_id = request.target_root_id;
            task.intertree_goal_bias = request.intertree;
            task.component_connect_target = request.component_connect;
            task.component_pair_unknown_failures = request.component_pair_unknown_failures;
            task.component_connect_staged_target = request.component_connect_staged_target;
            task.component_connect_gap_sq = request.component_connect_gap_sq;
            task.selected_face = selected_face;
            task.face_candidates = std::move(face_candidates);
            tasks[static_cast<std::size_t>(task_index)] = std::move(task);
        });
    }

    return filter_growth_tasks(boxes, std::move(tasks), base_options, context);
}

}  // namespace rbf
