#include <SBF/grower.h>
#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include "grower_components.h"
#include "grower_internal.h"
#include "grower_options.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace rbf {

void finalize_result(GrowerResult& result, double adjacency_tol) {
    result.adjacency = compute_adjacency(result.boxes, adjacency_tol);
    auto islands = find_islands(result.adjacency);
    result.adjacency_islands = static_cast<int>(islands.size());
    result.all_connected = islands.size() <= 1;
    result.adjacency_largest_island = 0;
    for (const auto& island : islands) {
        result.adjacency_largest_island = std::max(result.adjacency_largest_island, static_cast<int>(island.size()));
    }
}

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
                set_grower_max_diagnostic(context,
                                   "grower.depth_stage_box_count_at_switch_max",
                                   static_cast<double>(box_count));
            }
            active_depth_stage_index = next_stage;
        }
        context.diagnostics().set_value("grower.depth_stage_index", static_cast<double>(active_depth_stage_index));
        context.diagnostics().set_value("grower.depth_stage_depth", static_cast<double>(active_ffb_options.max_depth));
        set_grower_max_diagnostic(context,
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
                            const int chain_added = record_component_connect_success_and_extend(
                                result.boxes,
                                ffb,
                                active_ffb_options,
                                active_depth_stage_index,
                                worker_result.parent_box_id,
                                worker_result.source_root_id,
                                worker_result.target_root_id,
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
                            record_component_connect_failure(worker_result.parent_box_id,
                                                             worker_result.source_root_id,
                                                             worker_result.target_root_id,
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
                            const int chain_added = record_component_connect_success_and_extend(
                                result.boxes,
                                ffb,
                                active_ffb_options,
                                active_depth_stage_index,
                                task.parent_box_id,
                                task.source_root_id,
                                task.target_root_id,
                                context);
                            result.n_ffb_success += chain_added;
                        }
                    } else {
                        result.n_ffb_fail += 1;
                        batch_fail += 1;
                        if (task.component_connect_target) {
                            record_component_connect_failure(task.parent_box_id,
                                                             task.source_root_id,
                                                             task.target_root_id,
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
                const int source_root_id = trace_task.source_root_id >= 0
                    ? trace_task.source_root_id
                    : trace_task.root_id;
                const int chain_added = record_component_connect_success_and_extend(
                    result.boxes,
                    ffb,
                    active_ffb_options,
                    active_depth_stage_index,
                    parent_box_id,
                    source_root_id,
                    trace_task.target_root_id,
                    context);
                result.n_ffb_success += chain_added;
            }
            consecutive_miss = 0;
        } else {
            result.n_ffb_fail += 1;
            if (component_connect_attempt) {
                const int source_root_id = trace_task.source_root_id >= 0
                    ? trace_task.source_root_id
                    : trace_task.root_id;
                record_component_connect_failure(parent_box_id,
                                                 source_root_id,
                                                 trace_task.target_root_id,
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

}  // namespace rbf
