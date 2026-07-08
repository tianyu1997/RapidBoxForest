#include <SBF/grower.h>
#include <SBF/box_graph.h>
#include <SBF/find_free_box.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include "grower_internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <random>
#include <unordered_set>

namespace rbf {

FrontwaveGrower::FrontwaveGrower(BoxOracle& oracle, GrowerConfig config)
    : oracle_(oracle), config_(std::move(config)), rng_(config_.rng_seed) {}

GrowerResult FrontwaveGrower::grow(const std::vector<Eigen::VectorXd>& seeds) {
    RuntimeConfig runtime;
    runtime.mode = config_.n_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = config_.n_threads;
    runtime.batch_size = config_.task_batch_size;
    runtime.parallel_threshold = config_.parallel_threshold;
    runtime.deterministic_reduce = config_.deterministic_reduce;
    StageContext context(runtime, Deadline::after_ms(config_.timeout_ms));
    return grow(seeds, context);
}

GrowerResult FrontwaveGrower::grow(const std::vector<Eigen::VectorXd>& seeds,
                                   StageContext& context) {
    using Clock = std::chrono::steady_clock;
    ScopedStageTimer function_timer(context.diagnostics(), "grower.frontwave.grow");
    const auto t0 = Clock::now();
    const auto deadline = t0 + std::chrono::duration_cast<Clock::duration>(
                                 std::chrono::duration<double, std::milli>(config_.timeout_ms));
    GrowerResult result;
    FindFreeBoxService ffb(oracle_);
    next_box_id_ = 0;
    if (config_.boundary_epsilon > config_.adjacency_tolerance) {
        context.diagnostics().set_value("grower.invalid_boundary_epsilon", 1.0);
    }
    std::queue<int> frontier;

    for (int i = 0; i < static_cast<int>(seeds.size()) && static_cast<int>(result.boxes.size()) < config_.max_boxes; ++i) {
        if (context.should_stop()) break;
        const int id = create_box(seeds[static_cast<std::size_t>(i)], -1, i, result.boxes, ffb, context);
        if (id >= 0) {
            frontier.push(id);
            result.n_roots += 1;
            result.n_ffb_success += 1;
        } else {
            result.n_ffb_fail += 1;
        }
    }

    std::vector<GrowerConfig::FrontwaveStage> stages = config_.frontwave_stages;
    if (stages.empty()) {
        stages.push_back({config_.max_boxes});
    }
    int stage_index = 0;
    int consecutive_miss = 0;
    while (!frontier.empty() && static_cast<int>(result.boxes.size()) < config_.max_boxes &&
           consecutive_miss < config_.max_consecutive_miss) {
        if (context.should_stop()) {
            break;
        }
        if (config_.timeout_ms > 0.0 && Clock::now() >= deadline) {
            break;
        }
        while (stage_index + 1 < static_cast<int>(stages.size()) &&
               static_cast<int>(result.boxes.size()) >= stages[static_cast<std::size_t>(stage_index)].box_limit) {
            stage_index += 1;
        }
        if (stages[static_cast<std::size_t>(stage_index)].box_limit > 0 &&
            static_cast<int>(result.boxes.size()) >= stages[static_cast<std::size_t>(stage_index)].box_limit &&
            stage_index + 1 >= static_cast<int>(stages.size())) {
            break;
        }

        const int current_id = frontier.front();
        frontier.pop();
        auto it = std::find_if(result.boxes.begin(), result.boxes.end(), [&](const BoxNode& box) {
            return box.id == current_id;
        });
        if (it == result.boxes.end()) {
            continue;
        }
        const Eigen::VectorXd* bias = seeds.empty() ? nullptr : &seeds.back();
        const auto boundaries = boundary_seeds(*it, bias);
        bool handled_batch = false;
        if (context.executor().n_threads() > 1 && boundaries.size() > 1) {
            std::vector<GrowTask> tasks;
            std::unordered_set<OracleNodeId> used_domains;
            tasks.reserve(boundaries.size());
            for (int task_index = 0; task_index < static_cast<int>(boundaries.size()); ++task_index) {
                const auto& boundary = boundaries[static_cast<std::size_t>(task_index)];
                GrowTask task;
                task.task_id = task_index;
                task.seed = boundary.q;
                task.parent_box_id = boundary.parent_box_id;
                task.root_id = boundary.root_id;
                if (point_covered_by_existing_box(result.boxes, task.seed)) {
                    context.diagnostics().add_counter("grower.seed_already_covered");
                    continue;
                }
                const OracleNodeId domain_root = find_leaf_containing(oracle_, task.seed);
                if (domain_root >= 0 && !oracle_.is_reserved(domain_root) &&
                    used_domains.find(domain_root) == used_domains.end()) {
                    task.domain_root_node = domain_root;
                    used_domains.insert(domain_root);
                }
                tasks.push_back(std::move(task));
            }
            auto worker_results = run_worker_ffb_tasks(tasks, context);
            if (!worker_results.empty()) {
                handled_batch = true;
                for (auto& worker_result : worker_results) {
                    if (static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                        break;
                    }
                    if (context.should_stop()) break;
                    const int id = worker_result.accepted_by_worker
                        ? commit_box(worker_result.seed,
                                     std::move(worker_result.free_box),
                                     worker_result.parent_box_id,
                                     worker_result.root_id,
                                     result.boxes,
                                     context)
                        : -1;
                    if (id >= 0) {
                        frontier.push(id);
                        result.n_ffb_success += 1;
                        consecutive_miss = 0;
                    } else {
                        result.n_ffb_fail += 1;
                        consecutive_miss += 1;
                    }
                }
            }
        }
        if (handled_batch) {
            continue;
        }
        for (const auto& boundary : boundaries) {
            if (static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                break;
            }
            if (context.should_stop()) break;
            const int id = create_box(boundary.q, boundary.parent_box_id, boundary.root_id, result.boxes, ffb, context);
            if (id >= 0) {
                frontier.push(id);
                result.n_ffb_success += 1;
                consecutive_miss = 0;
            } else {
                result.n_ffb_fail += 1;
                consecutive_miss += 1;
            }
        }
    }

    finalize_result(result, config_.adjacency_tolerance);
    result.build_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return result;
}

int FrontwaveGrower::create_box(const Eigen::VectorXd& seed,
                                int parent_box_id,
                                int root_id,
                                std::vector<BoxNode>& boxes,
                                FindFreeBoxService& ffb,
                                StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.frontwave.create_box");
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        return -1;
    }
    auto ffb_result = ffb.find(seed, context, config_.find_free_box);
    if (!ffb_result.found) {
        return -1;
    }
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_grower_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        return -1;
    }
    BoxNode box;
    box.id = next_box_id_++;
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
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    const int id = box.id;
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    return id;
}

int FrontwaveGrower::commit_box(const Eigen::VectorXd& seed,
                                FindFreeBoxResult ffb_result,
                                int parent_box_id,
                                int root_id,
                                std::vector<BoxNode>& boxes,
                                StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.frontwave.commit_box");
    if (!ffb_result.found) {
        return -1;
    }
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        return -1;
    }
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_grower_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        return -1;
    }
    BoxNode box;
    box.id = next_box_id_++;
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
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    const int id = box.id;
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    return id;
}

std::vector<GrowWorkerResult> FrontwaveGrower::run_worker_ffb_tasks(const std::vector<GrowTask>& tasks,
                                                                    StageContext& context) {
    if (!config_.worker_local_ffb || tasks.empty() || context.executor().n_threads() <= 1) {
        return {};
    }
    std::vector<std::unique_ptr<BoxOracleSession>> sessions(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (tasks[i].domain_root_node < 0) {
            return {};
        }
        OracleSessionConfig session_config;
        session_config.worker_id = static_cast<int>(i);
        session_config.read_only = false;
        session_config.domain_root = tasks[i].domain_root_node;
        sessions[i] = oracle_.make_session(session_config);
        if (!sessions[i]) {
            return {};
        }
    }
    context.diagnostics().add_counter("frontwave.worker_ffb_batches");
    context.diagnostics().add_counter("frontwave.worker_ffb_tasks", static_cast<double>(tasks.size()));

    std::vector<GrowWorkerResult> results(tasks.size());
    context.executor().parallel_for(0, static_cast<int>(tasks.size()), [&](int index) {
        const auto& task = tasks[static_cast<std::size_t>(index)];
        auto& session = sessions[static_cast<std::size_t>(index)];
        FindFreeBoxService worker_ffb(session->oracle());
        auto ffb_result = worker_ffb.find(task.seed, context, config_.find_free_box);

        GrowWorkerResult worker_result;
        worker_result.task_id = task.task_id;
        worker_result.accepted_by_worker = ffb_result.found;
        worker_result.seed = task.seed;
        worker_result.free_box = std::move(ffb_result);
        worker_result.parent_box_id = task.parent_box_id;
        worker_result.root_id = task.root_id;
        results[static_cast<std::size_t>(index)] = std::move(worker_result);
    });

    for (std::size_t i = 0; i < results.size(); ++i) {
        if (!results[i].accepted_by_worker) {
            continue;
        }
        if (!sessions[i]->commit()) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("frontwave.worker_ffb_commit_failures");
            continue;
        }
        const OracleNodeId master_node = sessions[i]->map_node_to_master(results[i].free_box.node);
        if (master_node < 0) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("frontwave.worker_ffb_remap_failures");
            continue;
        }
        results[i].free_box.node = master_node;
        context.diagnostics().add_counter("frontwave.worker_ffb_commits");
    }
    return results;
}

std::vector<FrontwaveGrower::BoundarySeed> FrontwaveGrower::boundary_seeds(const BoxNode& box,
                                                                            const Eigen::VectorXd* bias_target) {
    std::vector<BoundarySeed> seeds;
    const auto root = oracle_.planning_intervals();
    const int nd = box.n_dims();
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    struct Face { int dim; int side; double priority; };
    std::vector<Face> faces;
    for (int dim = 0; dim < nd; ++dim) {
        if (box.joint_intervals[dim].lo > root[static_cast<std::size_t>(dim)].lo + config_.boundary_epsilon) {
            faces.push_back({dim, 0, 0.0});
        }
        if (box.joint_intervals[dim].hi < root[static_cast<std::size_t>(dim)].hi - config_.boundary_epsilon) {
            faces.push_back({dim, 1, 0.0});
        }
    }
    if (bias_target != nullptr) {
        const Eigen::VectorXd center = box.center();
        for (auto& face : faces) {
            face.priority = (face.side == 1 ? 1.0 : -1.0) * ((*bias_target)[face.dim] - center[face.dim]);
        }
        std::sort(faces.begin(), faces.end(), [](const Face& lhs, const Face& rhs) {
            return lhs.priority > rhs.priority;
        });
    } else {
        std::shuffle(faces.begin(), faces.end(), rng_);
    }
    const int n_faces = std::min(config_.n_boundary_samples, static_cast<int>(faces.size()));
    for (int i = 0; i < n_faces; ++i) {
        const Face& face = faces[static_cast<std::size_t>(i)];
        Eigen::VectorXd q(nd);
        for (int dim = 0; dim < nd; ++dim) {
            if (dim == face.dim) {
                q[dim] = face.side == 1
                    ? box.joint_intervals[dim].hi + config_.boundary_epsilon
                    : box.joint_intervals[dim].lo - config_.boundary_epsilon;
            } else {
                q[dim] = box.joint_intervals[dim].lo +
                         u01(rng_) * box.joint_intervals[dim].width();
            }
            q[dim] = std::clamp(q[dim], root[static_cast<std::size_t>(dim)].lo, root[static_cast<std::size_t>(dim)].hi);
        }
        seeds.push_back({q, box.id, box.root_id});
    }
    return seeds;
}

}  // namespace rbf
