#include <SBF/grower.h>

#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include "grower_internal.h"
#include "grower_options.h"

#include <algorithm>
#include <cmath>

namespace rbf {

int RrtGrower::create_box(const Eigen::VectorXd& seed,
                          int parent_box_id,
                          int root_id,
                          std::vector<BoxNode>& boxes,
                          FindFreeBoxService& ffb,
                          StageContext& context,
                          const FindFreeBoxOptions* override_options,
                          const GrowTask* trace_task,
                          int worker_id,
                          FindFreeBoxResult* observed_result) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.create_box");
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        trace_box_rejected("seed_already_covered", seed, parent_box_id, root_id, trace_task, nullptr, worker_id);
        return -1;
    }
    const FindFreeBoxOptions& options = override_options != nullptr ? *override_options : config_.find_free_box;
    OracleNodeId domain_node = kInvalidOracleNodeId;
    if (seed_in_failure_cooling(seed,
                                options.max_depth,
                                static_cast<int>(boxes.size()),
                                context,
                                &domain_node)) {
        trace_box_rejected("failure_cooling", seed, parent_box_id, root_id, trace_task, nullptr, worker_id);
        return -1;
    }
    auto ffb_result = ffb.find(seed,
                               context,
                               options);
    if (observed_result != nullptr) {
        *observed_result = ffb_result;
    }
    if (!ffb_result.found) {
        record_grower_ffb_failure(context, ffb_result);
        trace_ffb_result("ffb_fail", seed, ffb_result, parent_box_id, root_id, trace_task, nullptr, worker_id, options.max_depth);
        record_failure_cooling(ffb_result,
                               domain_node,
                               options.max_depth,
                               static_cast<int>(boxes.size()),
                               context);
        return -1;
    }
    trace_ffb_result("ffb_success", seed, ffb_result, parent_box_id, root_id, trace_task, nullptr, worker_id, options.max_depth);
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_grower_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        trace_box_rejected("ffb_result_seed_miss", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        trace_box_rejected("commit_policy_rejected", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
        return -1;
    }
    const int box_id = next_box_id_++;
    BoxNode box;
    box.id = box_id;
    box.joint_intervals = std::move(ffb_result.intervals);
    box.seed_config = seed;
    box.tree_id = ffb_result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.safety_status = ffb_result.validation_detail.safety_status;
    box.strict_audit_required = ffb_result.validation_detail.strict_audit_required;
    box.compute_volume();

    if (parent_box_id >= 0) {
        auto parent_it = std::find_if(boxes.begin(), boxes.end(), [&](const BoxNode& candidate) {
            return candidate.id == parent_box_id;
        });
        if (parent_it != boxes.end() && box_contains_box_exact(*parent_it, box)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            context.diagnostics().add_counter("grower.child_contained_in_parent");
            if (seed_parent_gap > config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_grower_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
            }
            context.diagnostics().add_counter("grower.rejected_contained_child");
            trace_box_rejected("contained_child", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
            return -1;
        }
        if (parent_it != boxes.end() && !boxes_connected(*parent_it, box, config_.adjacency_tolerance)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            const double gap = std::sqrt(box_gap_squared(*parent_it, box));
            context.diagnostics().add_counter("grower.rejected_disconnected");
            context.diagnostics().add_counter("grower.rejected_disconnected_gap_sum", gap);
            set_grower_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_grower_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
            }
            trace_box_rejected("disconnected_child", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    record_failure_cooling_success(ffb_result.node, context);
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    trace_box_added(boxes.back(), trace_task, nullptr, worker_id);
    return box_id;
}

int RrtGrower::commit_box(const Eigen::VectorXd& seed,
                          FindFreeBoxResult ffb_result,
                          int parent_box_id,
                          int root_id,
                          std::vector<BoxNode>& boxes,
                          StageContext& context,
                          const GrowWorkerResult* trace_result) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.commit_box");
    if (!ffb_result.found) {
        trace_box_rejected("ffb_not_found", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        trace_box_rejected("seed_already_covered", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_grower_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        trace_box_rejected("ffb_result_seed_miss", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        trace_box_rejected("commit_policy_rejected", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    const int box_id = next_box_id_++;
    BoxNode box;
    box.id = box_id;
    box.joint_intervals = std::move(ffb_result.intervals);
    box.seed_config = seed;
    box.tree_id = ffb_result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.safety_status = ffb_result.validation_detail.safety_status;
    box.strict_audit_required = ffb_result.validation_detail.strict_audit_required;
    box.compute_volume();

    if (parent_box_id >= 0) {
        auto parent_it = std::find_if(boxes.begin(), boxes.end(), [&](const BoxNode& candidate) {
            return candidate.id == parent_box_id;
        });
        if (parent_it != boxes.end() && box_contains_box_exact(*parent_it, box)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            context.diagnostics().add_counter("grower.child_contained_in_parent");
            if (seed_parent_gap > config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_grower_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
            }
            context.diagnostics().add_counter("grower.rejected_contained_child");
            trace_box_rejected("contained_child", seed, parent_box_id, root_id, nullptr, trace_result,
                               trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
            return -1;
        }
        if (parent_it != boxes.end() && !boxes_connected(*parent_it, box, config_.adjacency_tolerance)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            const double gap = std::sqrt(box_gap_squared(*parent_it, box));
            context.diagnostics().add_counter("grower.rejected_disconnected");
            context.diagnostics().add_counter("grower.rejected_disconnected_gap_sum", gap);
            set_grower_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_grower_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
            }
            trace_box_rejected("disconnected_child", seed, parent_box_id, root_id, nullptr, trace_result,
                               trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    record_failure_cooling_success(ffb_result.node, context);
    const int id = box.id;
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    return id;
}

}  // namespace rbf
