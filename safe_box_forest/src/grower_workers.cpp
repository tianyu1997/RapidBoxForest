#include <SBF/grower.h>

#include "grower_components.h"
#include "grower_internal.h"
#include "grower_options.h"

#include <memory>

namespace rbf {

std::vector<GrowWorkerResult> RrtGrower::run_worker_ffb_tasks(const std::vector<GrowTask>& tasks,
                                                              const FindFreeBoxOptions& base_options,
                                                              int depth_stage_index,
                                                              StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.run_worker_ffb_tasks");
    if (!config_.worker_local_ffb) {
        context.diagnostics().add_counter("grower.worker_ffb_disabled");
        return {};
    }
    if (tasks.empty()) {
        context.diagnostics().add_counter("grower.worker_ffb_empty_tasks");
        return {};
    }
    if (context.executor().n_threads() <= 1) {
        context.diagnostics().add_counter("grower.worker_ffb_inline_executor");
        return {};
    }
    std::vector<std::unique_ptr<BoxOracleSession>> sessions(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (tasks[i].domain_root_node < 0) {
            context.diagnostics().add_counter("grower.worker_ffb_missing_domain");
            return {};
        }
        OracleSessionConfig session_config;
        session_config.worker_id = static_cast<int>(i);
        session_config.read_only = false;
        session_config.domain_root = tasks[i].domain_root_node;
        sessions[i] = oracle_.make_session(session_config);
        if (!sessions[i]) {
            context.diagnostics().add_counter("grower.worker_ffb_non_session_oracle");
            return {};
        }
    }
    context.diagnostics().add_counter("grower.worker_ffb_batches");
    context.diagnostics().add_counter("grower.worker_ffb_tasks", static_cast<double>(tasks.size()));

    std::vector<GrowWorkerResult> results(tasks.size());
    context.executor().parallel_for(0, static_cast<int>(tasks.size()), [&](int index) {
        const int worker_id = current_worker_id();
        ScopedStageTimer task_timer(context.diagnostics(), "grower.rrt.worker_ffb_task");
        const auto& task = tasks[static_cast<std::size_t>(index)];
        auto& session = sessions[static_cast<std::size_t>(index)];
        FindFreeBoxService worker_ffb(session->oracle());
        FindFreeBoxOptions task_options = base_options;
        if (task.component_connect_target) {
            task_options = component_connect_ffb_options(config_,
                                                         context,
                                                         base_options,
                                                         depth_stage_index,
                                                         task.component_pair_unknown_failures);
        }
        auto ffb_result = worker_ffb.find(task.seed, context, task_options);
        record_worker_oracle_counters(context, session->oracle().counters());
        if (!ffb_result.found) {
            record_grower_ffb_failure(context, ffb_result);
        }

        GrowWorkerResult worker_result;
        worker_result.task_id = task.task_id;
        worker_result.iteration = task.iteration;
        worker_result.worker_id = worker_id;
        worker_result.accepted_by_worker = ffb_result.found;
        worker_result.seed = task.seed;
        worker_result.target = task.target;
        worker_result.target_type = task.target_type;
        worker_result.free_box = std::move(ffb_result);
        worker_result.parent_box_id = task.parent_box_id;
        worker_result.root_id = task.root_id;
        worker_result.source_root_id = task.source_root_id;
        worker_result.target_root_id = task.target_root_id;
        worker_result.intertree_goal_bias = task.intertree_goal_bias;
        worker_result.component_connect_target = task.component_connect_target;
        worker_result.component_pair_unknown_failures = task.component_pair_unknown_failures;
        worker_result.component_connect_staged_target = task.component_connect_staged_target;
        worker_result.component_connect_gap_sq = task.component_connect_gap_sq;
        worker_result.domain_root_node = task.domain_root_node;
        worker_result.ffb_depth = task_options.max_depth;
        worker_result.selected_face = task.selected_face;
        worker_result.face_candidates = task.face_candidates;
        trace_ffb_result(worker_result.accepted_by_worker ? "ffb_success" : "ffb_fail",
                         worker_result.seed,
                         worker_result.free_box,
                         worker_result.parent_box_id,
                         worker_result.root_id,
                         nullptr,
                         &worker_result,
                         worker_id,
                         task_options.max_depth);
        results[static_cast<std::size_t>(index)] = std::move(worker_result);
    });

    for (std::size_t i = 0; i < results.size(); ++i) {
        if (!results[i].accepted_by_worker) {
            continue;
        }
        if (!sessions[i]->commit()) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("grower.worker_ffb_commit_failures");
            continue;
        }
        const OracleNodeId master_node = sessions[i]->map_node_to_master(results[i].free_box.node);
        if (master_node < 0) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("grower.worker_ffb_remap_failures");
            continue;
        }
        results[i].free_box.node = master_node;
        context.diagnostics().add_counter("grower.worker_ffb_commits");
    }
    return results;
}

}  // namespace rbf
