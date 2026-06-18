#include "connector_birrt.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>

namespace rbf {

namespace {

struct RRTTree {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<int> parents;
};

int segment_resolution_for_step(const Eigen::Ref<const Eigen::VectorXd>& a,
                                const Eigen::Ref<const Eigen::VectorXd>& b,
                                int base_resolution,
                                double segment_step) {
    int resolution = std::max(1, base_resolution);
    if (segment_step > 0.0) {
        resolution = std::max(resolution,
                              static_cast<int>(std::ceil((b - a).norm() / segment_step)));
    }
    return resolution;
}

bool check_segment_with_step(const CollisionChecker& checker,
                             const Eigen::Ref<const Eigen::VectorXd>& a,
                             const Eigen::Ref<const Eigen::VectorXd>& b,
                             int base_resolution,
                             double segment_step) {
    return checker.check_segment(a,
                                 b,
                                 segment_resolution_for_step(a,
                                                             b,
                                                             base_resolution,
                                                             segment_step));
}

double waypoint_path_length(const std::vector<Eigen::VectorXd>& path) {
    double total = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        total += (path[index] - path[index - 1]).norm();
    }
    return total;
}

int nearest_node(const RRTTree& tree, const Eigen::VectorXd& q) {
    int best = -1;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(tree.nodes.size()); ++i) {
        const double dist = (tree.nodes[static_cast<std::size_t>(i)] - q).squaredNorm();
        if (dist < best_dist) {
            best_dist = dist;
            best = i;
        }
    }
    return best;
}

bool steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to, double step_size, Eigen::VectorXd& out) {
    const Eigen::VectorXd diff = to - from;
    const double norm = diff.norm();
    if (norm < 1e-10) {
        return false;
    }
    out = norm <= step_size ? to : from + (diff / norm) * step_size;
    return true;
}

int add_rrt_node(RRTTree& tree, const Eigen::VectorXd& q, int parent) {
    const int id = static_cast<int>(tree.nodes.size());
    tree.nodes.push_back(q);
    tree.parents.push_back(parent);
    return id;
}

int extend_tree(RRTTree& tree,
                const Eigen::VectorXd& target,
                const Eigen::VectorXd& lo,
                const Eigen::VectorXd& hi,
                const CollisionChecker& checker,
                const RRTConnectConfig& config) {
    const int near = nearest_node(tree, target);
    if (near < 0) {
        return -1;
    }
    Eigen::VectorXd q_new;
    if (!steer(tree.nodes[static_cast<std::size_t>(near)], target, config.step_size, q_new)) {
        return -1;
    }
    for (int dim = 0; dim < q_new.size(); ++dim) {
        q_new[dim] = std::clamp(q_new[dim], lo[dim], hi[dim]);
    }
    if (checker.check_config(q_new) ||
        check_segment_with_step(checker,
                                tree.nodes[static_cast<std::size_t>(near)],
                                q_new,
                                config.segment_resolution,
                                config.segment_step)) {
        return -1;
    }
    return add_rrt_node(tree, q_new, near);
}

int max_connect_steps(const Eigen::VectorXd& lo, const Eigen::VectorXd& hi, double step_size) {
    const double span = (hi - lo).norm();
    const double safe_step = std::max(step_size, 1e-9);
    return std::clamp(static_cast<int>(std::ceil(span / safe_step)) + 2, 1, 1000);
}

int connect_greedy(RRTTree& tree,
                   const Eigen::VectorXd& target,
                   const Eigen::VectorXd& lo,
                   const Eigen::VectorXd& hi,
                   const CollisionChecker& checker,
                   const RRTConnectConfig& config) {
    int current = nearest_node(tree, target);
    const int connect_steps = max_connect_steps(lo, hi, config.step_size);
    for (int step = 0; step < connect_steps && current >= 0; ++step) {
        if ((tree.nodes[static_cast<std::size_t>(current)] - target).norm() < 1e-8) {
            return current;
        }
        Eigen::VectorXd q_new;
        if (!steer(tree.nodes[static_cast<std::size_t>(current)], target, config.step_size, q_new)) {
            return -1;
        }
        for (int dim = 0; dim < q_new.size(); ++dim) {
            q_new[dim] = std::clamp(q_new[dim], lo[dim], hi[dim]);
        }
        if (checker.check_config(q_new) ||
            check_segment_with_step(checker,
                                    tree.nodes[static_cast<std::size_t>(current)],
                                    q_new,
                                    config.segment_resolution,
                                    config.segment_step)) {
            return -1;
        }
        const int id = add_rrt_node(tree, q_new, current);
        current = id;
        const double remaining = (q_new - target).norm();
        if (remaining <= 1e-8) {
            return current;
        }
        if (remaining <= config.step_size * 0.5 &&
            !check_segment_with_step(checker, q_new, target, config.segment_resolution, config.segment_step)) {
            return add_rrt_node(tree, target, current);
        }
    }
    return -1;
}

std::vector<Eigen::VectorXd> extract_rrt_path(const RRTTree& tree, int index) {
    std::vector<Eigen::VectorXd> path;
    while (index >= 0) {
        path.push_back(tree.nodes[static_cast<std::size_t>(index)]);
        index = tree.parents[static_cast<std::size_t>(index)];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Eigen::VectorXd> merge_birrt_paths(const RRTTree& start_tree,
                                               int start_tree_index,
                                               const RRTTree& goal_tree,
                                               int goal_tree_index) {
    std::vector<Eigen::VectorXd> start_path = extract_rrt_path(start_tree, start_tree_index);
    std::vector<Eigen::VectorXd> goal_path = extract_rrt_path(goal_tree, goal_tree_index);
    std::reverse(goal_path.begin(), goal_path.end());
    if (!start_path.empty() &&
        !goal_path.empty() &&
        (start_path.back() - goal_path.front()).norm() <= 1e-8) {
        start_path.insert(start_path.end(), goal_path.begin() + 1, goal_path.end());
    } else {
        start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());
    }
    return start_path;
}

std::vector<Eigen::VectorXd> shortcut_collision_free_path(const std::vector<Eigen::VectorXd>& path,
                                                          const CollisionChecker& checker,
                                                          int segment_resolution,
                                                          double segment_step = 0.0) {
    if (path.size() <= 2) {
        return path;
    }
    std::vector<Eigen::VectorXd> out;
    out.push_back(path.front());
    std::size_t current = 0;
    while (current + 1 < path.size()) {
        std::size_t next = path.size() - 1;
        while (next > current + 1 &&
               check_segment_with_step(checker, path[current], path[next], segment_resolution, segment_step)) {
            --next;
        }
        out.push_back(path[next]);
        current = next;
    }
    return out;
}

void finish_rrt_stats(RRTConnectOutcome& outcome,
                      const RRTTree& start_tree,
                      const RRTTree& goal_tree,
                      std::chrono::steady_clock::time_point t0) {
    outcome.stats.forward_tree_size = static_cast<int>(start_tree.nodes.size());
    outcome.stats.backward_tree_size = static_cast<int>(goal_tree.nodes.size());
    outcome.stats.elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    outcome.stats.success = !outcome.path.empty();
}

}  // namespace

void record_birrt_stats(StageDiagnostics& diagnostics, const RRTConnectStats& stats) {
    diagnostics.record_timing("connector.birrt", stats.elapsed_ms);
    diagnostics.set_value("connector.birrt_last_iterations", static_cast<double>(stats.iterations));
    diagnostics.set_value("connector.birrt_last_forward_tree_size", static_cast<double>(stats.forward_tree_size));
    diagnostics.set_value("connector.birrt_last_backward_tree_size", static_cast<double>(stats.backward_tree_size));
    diagnostics.set_value("connector.birrt_last_raw_waypoints", static_cast<double>(stats.raw_waypoints));
    diagnostics.set_value("connector.birrt_last_shortcut_waypoints", static_cast<double>(stats.shortcut_waypoints));
    if (stats.merge_iteration >= 0) {
        diagnostics.set_value("connector.birrt_last_merge_iteration", static_cast<double>(stats.merge_iteration));
    }
    if (stats.success) {
        diagnostics.add_counter("connector.birrt_successes");
        if (stats.direct) {
            diagnostics.add_counter("connector.birrt_direct_successes");
        }
    } else {
        diagnostics.add_counter("connector.birrt_failures");
        if (stats.cancelled) {
            diagnostics.add_counter("connector.birrt_cancelled");
        }
        if (stats.timed_out) {
            diagnostics.add_counter("connector.birrt_timeouts");
        }
    }
}

RRTConnectOutcome birrt_connect_impl(const Eigen::Ref<const Eigen::VectorXd>& start,
                                     const Eigen::Ref<const Eigen::VectorXd>& goal,
                                     const CollisionChecker& checker,
                                     const Robot& robot,
                                     const RRTConnectConfig& config,
                                     int seed,
                                     std::shared_ptr<std::atomic<bool>> cancel) {
    RRTConnectOutcome outcome;
    const auto t0 = std::chrono::steady_clock::now();
    if (checker.check_config(start) || checker.check_config(goal)) {
        outcome.stats.iterations = 0;
        outcome.stats.raw_waypoints = 0;
        outcome.stats.shortcut_waypoints = 0;
        outcome.stats.elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return outcome;
    }
    if (!config.domain_intervals.empty()) {
        if (config.domain_intervals.size() != static_cast<std::size_t>(start.size())) {
            outcome.stats.elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            return outcome;
        }
        for (int dim = 0; dim < start.size(); ++dim) {
            const Interval& interval = config.domain_intervals[static_cast<std::size_t>(dim)];
            const double domain_tol = std::max(0.0, config.domain_tolerance);
            if (start[dim] < interval.lo - domain_tol ||
                start[dim] > interval.hi + domain_tol ||
                goal[dim] < interval.lo - domain_tol ||
                goal[dim] > interval.hi + domain_tol) {
                outcome.stats.elapsed_ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                return outcome;
            }
        }
    }
    if (!check_segment_with_step(checker, start, goal, config.segment_resolution, config.segment_step)) {
        outcome.path = {start, goal};
        outcome.stats.direct = true;
        outcome.stats.merge_iteration = 0;
        outcome.stats.raw_waypoints = 2;
        outcome.stats.shortcut_waypoints = 2;
        RRTTree start_tree{{start}, {-1}};
        RRTTree goal_tree{{goal}, {-1}};
        finish_rrt_stats(outcome, start_tree, goal_tree, t0);
        return outcome;
    }
    const int nd = static_cast<int>(start.size());
    Eigen::VectorXd lo(nd), hi(nd);
    const auto& limits = robot.joint_limits().limits;
    for (int dim = 0; dim < nd; ++dim) {
        const bool use_domain = config.domain_intervals.size() == static_cast<std::size_t>(nd);
        const Interval& interval = use_domain
                                       ? config.domain_intervals[static_cast<std::size_t>(dim)]
                                       : limits[static_cast<std::size_t>(dim)];
        lo[dim] = interval.lo;
        hi[dim] = interval.hi;
        if (config.local_sampling_radius > 0.0) {
            const double local_lo = std::min(start[dim], goal[dim]) - config.local_sampling_radius;
            const double local_hi = std::max(start[dim], goal[dim]) + config.local_sampling_radius;
            lo[dim] = std::max(lo[dim], local_lo);
            hi[dim] = std::min(hi[dim], local_hi);
        }
        const double domain_tol = std::max(0.0, config.domain_tolerance);
        if (start[dim] < lo[dim] - domain_tol ||
            start[dim] > hi[dim] + domain_tol ||
            goal[dim] < lo[dim] - domain_tol ||
            goal[dim] > hi[dim] + domain_tol ||
            hi[dim] < lo[dim]) {
            outcome.stats.iterations = 0;
            outcome.stats.raw_waypoints = 0;
            outcome.stats.shortcut_waypoints = 0;
            outcome.stats.elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            return outcome;
        }
    }

    RRTTree start_tree{{start}, {-1}};
    RRTTree goal_tree{{goal}, {-1}};
    std::mt19937 rng(static_cast<unsigned>(seed));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const int optimize_after_first_iters = std::max(0, config.optimize_after_first_iters);
    int first_solution_iter = -1;
    double best_solution_length = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < config.max_iters; ++iter) {
        outcome.stats.iterations = iter + 1;
        if ((iter & 7) == 7) {
            if (cancel && cancel->load(std::memory_order_relaxed)) {
                outcome.stats.cancelled = true;
                finish_rrt_stats(outcome, start_tree, goal_tree, t0);
                return outcome;
            }
            const double elapsed =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            if (config.timeout_ms > 0.0 && elapsed > config.timeout_ms) {
                outcome.stats.timed_out = true;
                finish_rrt_stats(outcome, start_tree, goal_tree, t0);
                return outcome;
            }
        }

        const bool grow_from_start = (iter % 2) == 0;
        RRTTree& active_tree = grow_from_start ? start_tree : goal_tree;
        RRTTree& passive_tree = grow_from_start ? goal_tree : start_tree;
        const Eigen::VectorXd& active_target_root = grow_from_start ? goal : start;

        Eigen::VectorXd q_rand(nd);
        if (u01(rng) < std::clamp(config.goal_bias, 0.0, 1.0)) {
            q_rand = active_target_root;
        } else {
            for (int dim = 0; dim < nd; ++dim) {
                std::uniform_real_distribution<double> ud(lo[dim], hi[dim]);
                q_rand[dim] = ud(rng);
            }
        }

        const int active_index = extend_tree(active_tree, q_rand, lo, hi, checker, config);
        if (active_index < 0) {
            continue;
        }
        const int passive_index = connect_greedy(passive_tree,
                                                 active_tree.nodes[static_cast<std::size_t>(active_index)],
                                                 lo,
                                                 hi,
                                                 checker,
                                                 config);
        if (passive_index < 0) {
            continue;
        }

        std::vector<Eigen::VectorXd> raw_path;
        if (grow_from_start) {
            raw_path = merge_birrt_paths(start_tree, active_index, goal_tree, passive_index);
        } else {
            raw_path = merge_birrt_paths(start_tree, passive_index, goal_tree, active_index);
        }
        outcome.stats.merge_iteration = iter + 1;
        outcome.stats.raw_waypoints = static_cast<int>(raw_path.size());
        std::vector<Eigen::VectorXd> candidate =
            config.shortcut_path
                ? shortcut_collision_free_path(raw_path, checker, config.segment_resolution, config.segment_step)
                : std::move(raw_path);
        const double candidate_length = waypoint_path_length(candidate);
        if (candidate_length < best_solution_length) {
            best_solution_length = candidate_length;
            outcome.path = std::move(candidate);
            outcome.stats.shortcut_waypoints = static_cast<int>(outcome.path.size());
        }
        if (first_solution_iter < 0) {
            first_solution_iter = iter;
        }
        if (optimize_after_first_iters <= 0 ||
            iter + 1 >= first_solution_iter + 1 + optimize_after_first_iters) {
            finish_rrt_stats(outcome, start_tree, goal_tree, t0);
            return outcome;
        }
    }
    finish_rrt_stats(outcome, start_tree, goal_tree, t0);
    return outcome;
}

std::vector<Eigen::VectorXd> rrt_connect(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         const CollisionChecker& checker,
                                         const Robot& robot,
                                         const RRTConnectConfig& config,
                                         int seed,
                                         std::shared_ptr<std::atomic<bool>> cancel) {
    return birrt_connect_impl(start, goal, checker, robot, config, seed, std::move(cancel)).path;
}

std::vector<Eigen::VectorXd> rrt_connect(const Eigen::Ref<const Eigen::VectorXd>& start,
                                         const Eigen::Ref<const Eigen::VectorXd>& goal,
                                         const CollisionChecker& checker,
                                         const Robot& robot,
                                         StageContext& context,
                                         const RRTConnectConfig& config,
                                         int seed) {
    if (context.should_stop()) {
        return {};
    }
    auto outcome = birrt_connect_impl(start,
                                      goal,
                                      checker,
                                      robot,
                                      config,
                                      seed,
                                      context.native_cancel_flag());
    record_birrt_stats(context.diagnostics(), outcome.stats);
    return outcome.path;
}

}  // namespace rbf
