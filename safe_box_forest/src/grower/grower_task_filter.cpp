#include <SBF/grower.h>

#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include "grower_internal.h"

#include <unordered_set>
#include <vector>

namespace rbf {

std::vector<GrowTask> RrtGrower::filter_growth_tasks(const std::vector<BoxNode>& boxes,
                                                     std::vector<GrowTask> tasks,
                                                     const FindFreeBoxOptions& base_options,
                                                     StageContext& context) {
    std::vector<GrowTask> out;
    std::unordered_set<OracleNodeId> used_domains;
    const bool require_worker_domain = config_.worker_local_ffb && context.executor().n_threads() > 1;
    int skipped_frontier = 0;
    out.reserve(tasks.size());
    {
        ScopedStageTimer filter_timer(context.diagnostics(), "grower.rrt.make_growth_tasks.filter_tasks");
        for (auto& task : tasks) {
            if (task.task_id < 0 || task.parent_box_id < 0) {
                skipped_frontier += 1;
                continue;
            }
            if (seed_covered_by_frontier_cache(boxes, task.seed, &context)) {
                context.diagnostics().add_counter("grower.seed_already_covered");
                continue;
            }
            const OracleNodeId domain_root = find_leaf_containing(oracle_, task.seed);
            if (node_in_failure_cooling(domain_root,
                                        base_options.max_depth,
                                        static_cast<int>(boxes.size()),
                                        context)) {
                context.diagnostics().add_counter("grower.task_skipped_failure_cooling");
                if (config_.coverage_first_stop_loss) {
                    context.diagnostics().add_counter("grower.hard_frontier_task_skips");
                }
                continue;
            }
            if (domain_root >= 0 && !oracle_.is_reserved(domain_root) &&
                used_domains.find(domain_root) == used_domains.end()) {
                task.domain_root_node = domain_root;
                task.ffb_depth = base_options.max_depth;
                used_domains.insert(domain_root);
            } else if (domain_root >= 0 && oracle_.is_reserved(domain_root)) {
                context.diagnostics().add_counter("grower.task_reserved_domain");
            } else if (domain_root >= 0) {
                context.diagnostics().add_counter("grower.task_duplicate_domain");
            }
            if (require_worker_domain && task.domain_root_node < 0) {
                context.diagnostics().add_counter("grower.task_skipped_no_worker_domain");
                continue;
            }
            trace_task_plan(task);
            out.push_back(std::move(task));
        }
    }
    if (skipped_frontier > 0) {
        context.diagnostics().add_counter("grower.frontier_no_uncovered_seed", skipped_frontier);
        if (config_.coverage_first_stop_loss) {
            context.diagnostics().add_counter("grower.hard_frontier_no_uncovered_seed", skipped_frontier);
        }
    }
    return out;
}

}  // namespace rbf
