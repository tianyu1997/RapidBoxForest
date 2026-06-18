#include <SBF/grower.h>

#include "grower_components.h"
#include "grower_internal.h"
#include "grower_options.h"
#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rbf {

RrtGrower::RrtGrower(BoxOracle& oracle, GrowerConfig config)
    : oracle_(oracle), config_(std::move(config)), rng_(config_.rng_seed) {}

GrowerResult RrtGrower::grow(const std::vector<Eigen::VectorXd>& seeds) {
    RuntimeConfig runtime;
    runtime.mode = config_.n_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = config_.n_threads;
    runtime.batch_size = config_.task_batch_size;
    runtime.parallel_threshold = config_.parallel_threshold;
    runtime.deterministic_reduce = config_.deterministic_reduce;
    StageContext context(runtime, Deadline::after_ms(config_.timeout_ms));
    return grow(seeds, context);
}

GrowerResult RrtGrower::grow_from_existing(const std::vector<BoxNode>& initial_boxes,
                                           const std::vector<Eigen::VectorXd>& seeds,
                                           StageContext& context) {
    initial_boxes_ = initial_boxes;
    GrowerResult result = grow(seeds, context);
    initial_boxes_.clear();
    return result;
}

GrowerResult RrtGrower::grow(const std::vector<Eigen::VectorXd>& seeds,
                             StageContext& context) {
    using Clock = std::chrono::steady_clock;
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.grow");
    const auto t0 = Clock::now();
    const auto deadline = t0 + std::chrono::duration_cast<Clock::duration>(
                                 std::chrono::duration<double, std::milli>(config_.timeout_ms));
    GrowerResult result;
    FindFreeBoxService ffb(oracle_);
    open_trace();
    const bool has_initial_boxes = !initial_boxes_.empty();
    if (has_initial_boxes) {
        result.boxes = initial_boxes_;
        next_box_id_ = 0;
        int max_root_id = -1;
        for (const BoxNode& box : result.boxes) {
            next_box_id_ = std::max(next_box_id_, box.id + 1);
            max_root_id = std::max(max_root_id, box.root_id);
        }
        result.n_roots = static_cast<int>(group_boxes_by_root(result.boxes).roots.size());
        context.diagnostics().set_value("grower.initial_boxes", static_cast<double>(result.boxes.size()));
        context.diagnostics().set_value("grower.initial_roots", static_cast<double>(result.n_roots));
        context.diagnostics().set_value("grower.initial_next_box_id", static_cast<double>(next_box_id_));
        context.diagnostics().set_value("grower.initial_root_id_base", static_cast<double>(max_root_id + 1));
    } else {
        next_box_id_ = 0;
    }
    random_anchor_targets_.clear();
    component_parent_failures_.clear();
    failure_cooling_.clear();
    if (config_.boundary_epsilon > config_.adjacency_tolerance) {
        context.diagnostics().set_value("grower.invalid_boundary_epsilon", 1.0);
    }
    context.diagnostics().set_value("grower.executor_threads", static_cast<double>(context.executor().n_threads()));
    context.diagnostics().set_value("grower.config_threads", static_cast<double>(config_.n_threads));

    int active_depth_stage_index = -2;
    FindFreeBoxOptions active_ffb_options = config_.find_free_box;
    auto refresh_depth_stage = [&](int box_count) {
        const int next_stage = select_depth_stage_index(config_, box_count);
        active_ffb_options = staged_ffb_options(config_, next_stage);
        if (next_stage != active_depth_stage_index) {
            if (active_depth_stage_index != -2) {
                context.diagnostics().add_counter("grower.depth_stage_switches");
                set_max_diagnostic(context,
                                   "grower.depth_stage_box_count_at_switch_max",
                                   static_cast<double>(box_count));
            }
            active_depth_stage_index = next_stage;
        }
        context.diagnostics().set_value("grower.depth_stage_index", static_cast<double>(active_depth_stage_index));
        context.diagnostics().set_value("grower.depth_stage_depth", static_cast<double>(active_ffb_options.max_depth));
        set_max_diagnostic(context,
                           "grower.depth_stage_depth_max",
                           static_cast<double>(active_ffb_options.max_depth));
    };

    std::vector<Eigen::VectorXd> roots = select_initial_roots(seeds, context);
    initialize_anchor_targets(roots, seeds, context);
    const FindFreeBoxOptions root_ffb_options = config_.find_free_box;
    context.diagnostics().set_value("grower.depth_stage_root_depth", static_cast<double>(root_ffb_options.max_depth));
    int root_id_base = 0;
    if (has_initial_boxes) {
        for (const BoxNode& box : result.boxes) {
            root_id_base = std::max(root_id_base, box.root_id + 1);
        }
    }
    for (int i = 0; i < static_cast<int>(roots.size()) && static_cast<int>(result.boxes.size()) < config_.max_boxes; ++i) {
        if (context.should_stop()) break;
        if (has_initial_boxes && point_covered_by_existing_box(result.boxes, roots[static_cast<std::size_t>(i)])) {
            context.diagnostics().add_counter("grower.initial_root_seed_already_covered");
            continue;
        }
        refresh_depth_stage(static_cast<int>(result.boxes.size()));
        GrowTask root_task;
        root_task.task_id = -1;
        root_task.iteration = i;
        root_task.seed = roots[static_cast<std::size_t>(i)];
        root_task.target = root_task.seed;
        root_task.target_type = GrowTargetType::RootSeed;
        root_task.parent_box_id = -1;
        root_task.root_id = root_id_base + i;
        trace_root_seed(i, root_task.root_id, root_task.seed);
        const int id = create_box(root_task.seed, -1, root_task.root_id, result.boxes, ffb, context, &root_ffb_options, &root_task);
        if (id >= 0) {
            result.n_roots += 1;
            result.n_ffb_success += 1;
        } else {
            result.n_ffb_fail += 1;
        }
    }

    run_frontwave_bootstrap(result, ffb, context, deadline);

    int consecutive_miss = 0;
    int connected_at_count = -1;
    Clock::time_point connected_at_time = t0;
    int next_task_id = 0;
    int loop_iteration = 0;
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    while (static_cast<int>(result.boxes.size()) < config_.max_boxes && consecutive_miss < config_.max_consecutive_miss) {
        if (context.should_stop()) {
            break;
        }
        loop_iteration += 1;
        if (config_.timeout_ms > 0.0 && Clock::now() >= deadline) {
            break;
        }
        if (result.boxes.empty()) {
            break;
        }
        refresh_depth_stage(static_cast<int>(result.boxes.size()));
        if (config_.connect_mode && connected(result.boxes)) {
            if (connected_at_count < 0) {
                connected_at_count = static_cast<int>(result.boxes.size());
                connected_at_time = Clock::now();
                context.diagnostics().set_value("grower.connected_at_box_count", static_cast<double>(connected_at_count));
            }
            const int box_count = static_cast<int>(result.boxes.size());
            const int quality_min_boxes = std::max(0, config_.quality_min_connected_boxes);
            const bool quality_floor_reached = quality_min_boxes == 0 || box_count >= quality_min_boxes;
            if (config_.post_connect_time_budget_ms > 0.0) {
                const double connected_elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - connected_at_time).count();
                context.diagnostics().set_value("grower.post_connect_elapsed_ms", connected_elapsed_ms);
                if (connected_elapsed_ms >= config_.post_connect_time_budget_ms) {
                    context.diagnostics().set_value("grower.stop_post_connect_time_budget", 1.0);
                    break;
                }
            }
            if (quality_floor_reached) {
                context.diagnostics().set_value("grower.quality_min_connected_boxes_reached", 1.0);
            }
            if (config_.stop_after_connect && quality_floor_reached) {
                break;
            }
            if (config_.post_connect_extra_boxes > 0 &&
                box_count >= connected_at_count + config_.post_connect_extra_boxes &&
                quality_floor_reached) {
                break;
            }
        }

        const int batch_size = resolve_task_batch_size(result.boxes, context);
        const RootGroups active_groups = group_boxes_by_root(result.boxes);
        const bool use_task_path = batch_size > 1 ||
                                   (config_.expand_all_roots_per_sample && active_groups.roots.size() > 1);
        if (use_task_path) {
            const int remaining = config_.max_boxes - static_cast<int>(result.boxes.size());
            const int requested_tasks = std::max(1, std::min(batch_size, remaining));
            auto tasks = make_growth_tasks(result.boxes,
                                           roots,
                                           next_task_id,
                                           requested_tasks,
                                           active_ffb_options,
                                           context);
            if (tasks.empty()) {
                result.n_ffb_fail += 1;
                consecutive_miss += 1;
                continue;
            }
            if (static_cast<int>(tasks.size()) > remaining) {
                tasks.resize(static_cast<std::size_t>(remaining));
                context.diagnostics().add_counter("grower.task_truncated_to_remaining");
            }
            next_task_id += static_cast<int>(tasks.size());

            bool batch_success = false;
            int batch_fail = 0;
            auto worker_results = run_worker_ffb_tasks(tasks, active_ffb_options, active_depth_stage_index, context);
            if (!worker_results.empty()) {
                for (auto& worker_result : worker_results) {
                    if (context.should_stop() || static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                        break;
                    }
                    const int id = worker_result.accepted_by_worker
                        ? commit_box(worker_result.seed,
                                     std::move(worker_result.free_box),
                                     worker_result.parent_box_id,
                                     worker_result.root_id,
                                     result.boxes,
                                     context,
                                     &worker_result)
                        : -1;
                    if (id >= 0) {
                        result.n_ffb_success += 1;
                        batch_success = true;
                        if (worker_result.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_successes");
                            component_parent_failures_.erase(worker_result.parent_box_id);
                            record_component_connect_result(worker_result.source_root_id,
                                                            worker_result.target_root_id,
                                                            true,
                                                            nullptr,
                                                            context);
                            const int chain_added = grow_component_connect_chain(result.boxes,
                                                                                 ffb,
                                                                                 active_ffb_options,
                                                                                 active_depth_stage_index,
                                                                                 worker_result.source_root_id,
                                                                                 context);
                            result.n_ffb_success += chain_added;
                        }
                    } else {
                        if (!worker_result.free_box.found) {
                            worker_result.free_box.node = worker_result.domain_root_node;
                            record_failure_cooling(worker_result.free_box,
                                                   worker_result.domain_root_node,
                                                   worker_result.ffb_depth,
                                                   static_cast<int>(result.boxes.size()),
                                                   context);
                        }
                        result.n_ffb_fail += 1;
                        batch_fail += 1;
                        if (worker_result.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_failures");
                            const int failures = ++component_parent_failures_[worker_result.parent_box_id];
                            set_max_diagnostic(context,
                                               "grower.component_connect_parent_failure_max",
                                               static_cast<double>(failures));
                            record_component_connect_result(worker_result.source_root_id,
                                                            worker_result.target_root_id,
                                                            false,
                                                            &worker_result.free_box,
                                                            context);
                        }
                    }
                }
            } else {
                for (const auto& task : tasks) {
                    if (context.should_stop() || static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                        break;
                    }
                    FindFreeBoxOptions task_options;
                    const FindFreeBoxOptions* override_options = nullptr;
                    if (task.component_connect_target) {
                        task_options = component_connect_ffb_options(config_,
                                                                    context,
                                                                    active_ffb_options,
                                                                    active_depth_stage_index,
                                                                    task.component_pair_unknown_failures);
                        override_options = &task_options;
                    } else {
                        override_options = &active_ffb_options;
                    }
                    FindFreeBoxResult observed_result;
                    const int id = create_box(task.seed,
                                              task.parent_box_id,
                                              task.root_id,
                                              result.boxes,
                                              ffb,
                                              context,
                                              override_options,
                                              &task,
                                              -1,
                                              &observed_result);
                    if (id >= 0) {
                        result.n_ffb_success += 1;
                        batch_success = true;
                        if (task.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_successes");
                            component_parent_failures_.erase(task.parent_box_id);
                            record_component_connect_result(task.source_root_id,
                                                            task.target_root_id,
                                                            true,
                                                            nullptr,
                                                            context);
                            const int chain_added = grow_component_connect_chain(result.boxes,
                                                                                 ffb,
                                                                                 active_ffb_options,
                                                                                 active_depth_stage_index,
                                                                                 task.source_root_id,
                                                                                 context);
                            result.n_ffb_success += chain_added;
                        }
                    } else {
                        result.n_ffb_fail += 1;
                        batch_fail += 1;
                        if (task.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_failures");
                            const int failures = ++component_parent_failures_[task.parent_box_id];
                            set_max_diagnostic(context,
                                               "grower.component_connect_parent_failure_max",
                                               static_cast<double>(failures));
                            record_component_connect_result(task.source_root_id,
                                                            task.target_root_id,
                                                            false,
                                                            observed_result.found || observed_result.fail_code != 0 ? &observed_result : nullptr,
                                                            context);
                        }
                    }
                }
            }
            consecutive_miss = batch_success ? 0 : consecutive_miss + std::max(1, batch_fail);
            continue;
        }

        Eigen::VectorXd seed;
        Eigen::VectorXd target;
        GrowTargetType target_type = GrowTargetType::Unknown;
        GrowTraceFace selected_face;
        std::vector<GrowTraceFace> face_candidates;
        int parent_box_id = -1;
        int root_id = -1;
        int target_root_id = -1;
        int component_pair_unknown_failures = 0;
        bool component_connect_staged_target = false;
        double component_connect_gap_sq = 0.0;
        bool component_connect_attempt = false;
        if (config_.connect_mode && roots.size() > 1 && u01(rng_) < config_.component_connect_prob) {
            component_connect_attempt = make_component_connect_seed(result.boxes,
                                                                    seed,
                                                                    target,
                                                                    parent_box_id,
                                                                    root_id,
                                                                    target_root_id,
                                                                    component_pair_unknown_failures,
                                                                    component_connect_staged_target,
                                                                    component_connect_gap_sq,
                                                                    context);
            if (component_connect_attempt) {
                context.diagnostics().add_counter("grower.component_connect_attempts");
                context.diagnostics().add_counter("grower.component_connect_target_tasks");
                target_type = GrowTargetType::ComponentConnect;
            }
        }
        if (!component_connect_attempt) {
            if (roots.size() > 1 && u01(rng_) < config_.rrt_goal_bias) {
                std::uniform_int_distribution<int> pick(0, static_cast<int>(roots.size()) - 1);
                target_root_id = pick(rng_);
                target = roots[static_cast<std::size_t>(target_root_id)];
                target_type = GrowTargetType::QueryRoot;
            } else if (u01(rng_) < config_.unexplored_sample_prob) {
                target = sample_unexplored();
                target_type = GrowTargetType::Unexplored;
            } else {
                target = sample_uniform();
                target_type = GrowTargetType::Uniform;
            }

            if (!make_frontier_seed(result.boxes,
                                    target,
                                    seed,
                                    parent_box_id,
                                    root_id,
                                    &context,
                                    &selected_face,
                                    &face_candidates)) {
                result.n_ffb_fail += 1;
                consecutive_miss += 1;
                continue;
            }
        }

        GrowTask trace_task;
        trace_task.task_id = next_task_id++;
        trace_task.iteration = loop_iteration;
        trace_task.seed = seed;
        trace_task.target = target;
        trace_task.target_type = target_type;
        trace_task.parent_box_id = parent_box_id;
        trace_task.root_id = root_id;
        trace_task.source_root_id = component_connect_attempt ? root_id : -1;
        trace_task.target_root_id = target_root_id;
        trace_task.intertree_goal_bias = target_root_id >= 0;
        trace_task.component_connect_target = component_connect_attempt;
        trace_task.component_pair_unknown_failures = component_pair_unknown_failures;
        trace_task.component_connect_staged_target = component_connect_staged_target;
        trace_task.component_connect_gap_sq = component_connect_gap_sq;
        trace_task.selected_face = selected_face;
        trace_task.face_candidates = std::move(face_candidates);
        trace_task.ffb_depth = active_ffb_options.max_depth;
        trace_task_plan(trace_task);

        FindFreeBoxOptions component_options;
        const FindFreeBoxOptions* override_options = nullptr;
        if (component_connect_attempt) {
            component_options = component_connect_ffb_options(config_,
                                                              context,
                                                              active_ffb_options,
                                                              active_depth_stage_index,
                                                              trace_task.component_pair_unknown_failures);
            override_options = &component_options;
        } else {
            override_options = &active_ffb_options;
        }
        FindFreeBoxResult ffb_result_for_pair;
        const int id = create_box(seed, parent_box_id, root_id, result.boxes, ffb, context, override_options, &trace_task, -1, &ffb_result_for_pair);
        if (id >= 0) {
            result.n_ffb_success += 1;
            if (component_connect_attempt) {
                context.diagnostics().add_counter("grower.component_connect_successes");
                component_parent_failures_.erase(parent_box_id);
                record_component_connect_result(trace_task.source_root_id >= 0 ? trace_task.source_root_id : trace_task.root_id,
                                                trace_task.target_root_id,
                                                true,
                                                nullptr,
                                                context);
                const int chain_added = grow_component_connect_chain(result.boxes,
                                                                     ffb,
                                                                     active_ffb_options,
                                                                     active_depth_stage_index,
                                                                     trace_task.source_root_id >= 0 ? trace_task.source_root_id : trace_task.root_id,
                                                                     context);
                result.n_ffb_success += chain_added;
            }
            consecutive_miss = 0;
        } else {
            result.n_ffb_fail += 1;
            if (component_connect_attempt) {
                context.diagnostics().add_counter("grower.component_connect_failures");
                const int failures = ++component_parent_failures_[parent_box_id];
                set_max_diagnostic(context,
                                   "grower.component_connect_parent_failure_max",
                                   static_cast<double>(failures));
                record_component_connect_result(trace_task.source_root_id >= 0 ? trace_task.source_root_id : trace_task.root_id,
                                                trace_task.target_root_id,
                                                false,
                                                ffb_result_for_pair.found || ffb_result_for_pair.fail_code != 0 ? &ffb_result_for_pair : nullptr,
                                                context);
            }
            consecutive_miss += 1;
        }
    }

    finalize_result(result, config_.adjacency_tolerance);
    result.build_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    close_trace();
    return result;
}

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
        set_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
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
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
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
            set_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
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
        set_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
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
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
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
            set_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
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

Eigen::VectorXd RrtGrower::sample_uniform() {
    const auto root = oracle_.planning_intervals();
    Eigen::VectorXd q(static_cast<int>(root.size()));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int dim = 0; dim < static_cast<int>(root.size()); ++dim) {
        q[dim] = root[static_cast<std::size_t>(dim)].lo +
                 u01(rng_) * root[static_cast<std::size_t>(dim)].width();
    }
    return q;
}

Eigen::VectorXd RrtGrower::sample_unexplored() {
    const OracleNodeId node = oracle_.select_unexplored_node();
    std::vector<Interval> intervals;
    if (node >= 0) {
        auto copies = oracle_.native_interval_copies_for_node(node, oracle_.node_intervals(node));
        if (!copies.empty()) {
            std::uniform_int_distribution<std::size_t> pick(0, copies.size() - 1);
            intervals = std::move(copies[pick(rng_)]);
            if (!clip_intervals_to_root(intervals, oracle_.planning_intervals())) {
                intervals.clear();
            }
        }
    }
    if (intervals.empty()) {
        intervals = oracle_.planning_intervals();
    }
    Eigen::VectorXd q(static_cast<int>(intervals.size()));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        q[dim] = intervals[static_cast<std::size_t>(dim)].lo +
                 u01(rng_) * intervals[static_cast<std::size_t>(dim)].width();
    }
    return q;
}

bool RrtGrower::connected(const std::vector<BoxNode>& boxes) const {
    if (boxes.size() <= 1) {
        return true;
    }
    return find_islands(compute_adjacency(boxes, config_.adjacency_tolerance)).size() <= 1;
}

std::unique_ptr<IGrower> make_grower(BoxOracle& oracle, const GrowerConfig& config) {
    if (config.mode == GrowerConfig::Mode::Frontwave) {
        return std::make_unique<FrontwaveGrower>(oracle, config);
    }
    return std::make_unique<RrtGrower>(oracle, config);
}

}  // namespace rbf
