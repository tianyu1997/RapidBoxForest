#include <SBF/grower.h>

#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include "grower_internal.h"

#include <algorithm>
#include <chrono>
#include <queue>
#include <random>

namespace rbf {

void RrtGrower::run_frontwave_bootstrap(GrowerResult& result,
                                        FindFreeBoxService& ffb,
                                        StageContext& context,
                                        std::chrono::steady_clock::time_point deadline) {
    using Clock = std::chrono::steady_clock;
    if (config_.frontwave_bootstrap_boxes <= 0 || result.boxes.empty() ||
        static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
        return;
    }

    ScopedStageTimer bootstrap_timer(context.diagnostics(), "grower.rrt.frontwave_bootstrap");
    FindFreeBoxOptions bootstrap_options = config_.find_free_box;
    if (config_.frontwave_bootstrap_depth > 0) {
        bootstrap_options.max_depth = config_.frontwave_bootstrap_depth;
    }
    const int bootstrap_target = std::min(config_.max_boxes,
                                          std::max(static_cast<int>(result.boxes.size()),
                                                   config_.frontwave_bootstrap_boxes));
    const int samples_per_box = std::max(1, config_.frontwave_bootstrap_boundary_samples);
    std::queue<int> bootstrap_frontier;
    for (const BoxNode& box : result.boxes) {
        bootstrap_frontier.push(box.id);
    }
    std::uniform_real_distribution<double> bootstrap_u01(0.0, 1.0);
    int bootstrap_misses = 0;
    while (!bootstrap_frontier.empty() &&
           static_cast<int>(result.boxes.size()) < bootstrap_target &&
           bootstrap_misses < config_.max_consecutive_miss) {
        if (context.should_stop()) {
            break;
        }
        if (config_.timeout_ms > 0.0 && Clock::now() >= deadline) {
            break;
        }
        const int parent_id = bootstrap_frontier.front();
        bootstrap_frontier.pop();
        auto parent_it = std::find_if(result.boxes.begin(), result.boxes.end(), [&](const BoxNode& candidate) {
            return candidate.id == parent_id;
        });
        if (parent_it == result.boxes.end()) {
            continue;
        }
        const BoxNode parent = *parent_it;
        const auto root_intervals = oracle_.planning_intervals();
        struct BootstrapFace {
            int dim = -1;
            int side = 0;
        };
        std::vector<BootstrapFace> faces;
        faces.reserve(static_cast<std::size_t>(2 * parent.n_dims()));
        for (int dim = 0; dim < parent.n_dims(); ++dim) {
            if (parent.joint_intervals[static_cast<std::size_t>(dim)].lo >
                root_intervals[static_cast<std::size_t>(dim)].lo + config_.boundary_epsilon) {
                faces.push_back({dim, 0});
            }
            if (parent.joint_intervals[static_cast<std::size_t>(dim)].hi <
                root_intervals[static_cast<std::size_t>(dim)].hi - config_.boundary_epsilon) {
                faces.push_back({dim, 1});
            }
        }
        std::shuffle(faces.begin(), faces.end(), rng_);
        const int n_faces = std::min(samples_per_box, static_cast<int>(faces.size()));
        for (int face_index = 0; face_index < n_faces &&
             static_cast<int>(result.boxes.size()) < bootstrap_target; ++face_index) {
            const BootstrapFace& face = faces[static_cast<std::size_t>(face_index)];
            Eigen::VectorXd seed(parent.n_dims());
            for (int dim = 0; dim < parent.n_dims(); ++dim) {
                if (dim == face.dim) {
                    seed[dim] = face.side == 1
                        ? parent.joint_intervals[static_cast<std::size_t>(dim)].hi + config_.boundary_epsilon
                        : parent.joint_intervals[static_cast<std::size_t>(dim)].lo - config_.boundary_epsilon;
                } else {
                    seed[dim] = parent.joint_intervals[static_cast<std::size_t>(dim)].lo +
                                bootstrap_u01(rng_) *
                                parent.joint_intervals[static_cast<std::size_t>(dim)].width();
                }
                seed[dim] = std::clamp(seed[dim],
                                       root_intervals[static_cast<std::size_t>(dim)].lo,
                                       root_intervals[static_cast<std::size_t>(dim)].hi);
            }
            if (point_covered_by_existing_box(result.boxes, seed)) {
                context.diagnostics().add_counter("grower.frontwave_bootstrap_seed_covered");
                continue;
            }
            GrowTask bootstrap_task;
            bootstrap_task.task_id = -2;
            bootstrap_task.iteration = static_cast<int>(result.boxes.size());
            bootstrap_task.seed = seed;
            bootstrap_task.target = seed;
            bootstrap_task.target_type = GrowTargetType::Unexplored;
            bootstrap_task.parent_box_id = parent.id;
            bootstrap_task.root_id = parent.root_id;
            const int id = create_box(seed,
                                      parent.id,
                                      parent.root_id,
                                      result.boxes,
                                      ffb,
                                      context,
                                      &bootstrap_options,
                                      &bootstrap_task);
            context.diagnostics().add_counter("grower.frontwave_bootstrap_attempts");
            if (id >= 0) {
                bootstrap_frontier.push(id);
                result.n_ffb_success += 1;
                bootstrap_misses = 0;
                context.diagnostics().add_counter("grower.frontwave_bootstrap_added");
            } else {
                result.n_ffb_fail += 1;
                bootstrap_misses += 1;
                context.diagnostics().add_counter("grower.frontwave_bootstrap_failures");
            }
        }
    }
    context.diagnostics().set_value("grower.frontwave_bootstrap_box_count",
                                    static_cast<double>(result.boxes.size()));
    context.diagnostics().set_value("grower.frontwave_bootstrap_depth",
                                    static_cast<double>(bootstrap_options.max_depth));
}

}  // namespace rbf
