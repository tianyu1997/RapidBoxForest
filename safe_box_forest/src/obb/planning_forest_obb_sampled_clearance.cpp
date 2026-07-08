#include "planning_forest_obb_sampled_internal.h"

#include <SBF/scene.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rbf {

double obb_point_aabb_distance_sq_at_t(const detail::Vec3& origin,
                                       const detail::Vec3& dir,
                                       const float* obstacle,
                                       double t) {
    const double px = static_cast<double>(origin.x) + static_cast<double>(dir.x) * t;
    const double py = static_cast<double>(origin.y) + static_cast<double>(dir.y) * t;
    const double pz = static_cast<double>(origin.z) + static_cast<double>(dir.z) * t;
    double distance_sq = 0.0;
    const double values[3] = {px, py, pz};
    for (int axis = 0; axis < 3; ++axis) {
        const double lo = static_cast<double>(obstacle[axis]);
        const double hi = static_cast<double>(obstacle[axis + 3]);
        double delta = 0.0;
        if (values[axis] < lo) {
            delta = lo - values[axis];
        } else if (values[axis] > hi) {
            delta = values[axis] - hi;
        }
        distance_sq += delta * delta;
    }
    return distance_sq;
}

double obb_segment_aabb_distance_sq(const detail::Vec3& origin,
                                    const detail::Vec3& end,
                                    const float* obstacle) {
    const detail::Vec3 dir = end - origin;
    double best = std::min(obb_point_aabb_distance_sq_at_t(origin, dir, obstacle, 0.0),
                           obb_point_aabb_distance_sq_at_t(origin, dir, obstacle, 1.0));
    const double d[3] = {
        static_cast<double>(dir.x),
        static_cast<double>(dir.y),
        static_cast<double>(dir.z)
    };
    const double o[3] = {
        static_cast<double>(origin.x),
        static_cast<double>(origin.y),
        static_cast<double>(origin.z)
    };
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(d[axis]) <= 1e-14) {
            continue;
        }
        for (int side = 0; side < 2; ++side) {
            const double plane =
                static_cast<double>(obstacle[axis + (side == 0 ? 0 : 3)]);
            const double t = (plane - o[axis]) / d[axis];
            if (t > 0.0 && t < 1.0) {
                best = std::min(best,
                                obb_point_aabb_distance_sq_at_t(origin,
                                                                dir,
                                                                obstacle,
                                                                std::clamp(t, 0.0, 1.0)));
            }
        }
    }
    return best;
}

bool validate_obb_clearance_sampled_candidate(const Robot& robot,
                                              const Scene& scene,
                                              const ObbPortalCandidate& candidate,
                                              double safety_epsilon,
                                              ObbPortalValidationStats& stats,
                                              const ObbValidationOptions& options) {
    if (!options.clearance_sampled_support_enabled) {
        return false;
    }
    ++stats.clearance_support_attempts;
    const int dims = static_cast<int>(candidate.center_q.size());
    if (dims <= 0 ||
        candidate.generators_q.rows() != dims ||
        candidate.generators_q.cols() <= 0) {
        ++stats.clearance_support_fail;
        return false;
    }

    int line_col = -1;
    double best_col_norm = 0.0;
    for (int col = 0; col < candidate.generators_q.cols(); ++col) {
        const double norm = candidate.generators_q.col(col).norm();
        if (norm > best_col_norm) {
            best_col_norm = norm;
            line_col = col;
        }
    }
    if (line_col < 0 || best_col_norm <= 1e-12) {
        ++stats.clearance_support_fail;
        return false;
    }

    double lateral_l1 = 0.0;
    for (int joint = 0; joint < dims; ++joint) {
        for (int col = 0; col < candidate.generators_q.cols(); ++col) {
            if (col != line_col) {
                lateral_l1 += std::abs(candidate.generators_q(joint, col));
            }
        }
    }
    // The pointwise clearance proof is designed for thin bridge tubes. Large
    // transverse OBBs should use the affine support-hull validator instead.
    if (lateral_l1 > options.clearance_lateral_l1_max) {
        ++stats.clearance_support_fail;
        return false;
    }

    const double line_l1 = candidate.generators_q.col(line_col).lpNorm<1>();
    int samples = options.clearance_samples;
    const double dense_line_l1_threshold = options.clearance_dense_line_l1_threshold;
    const int dense_samples = options.clearance_dense_samples;
    if (dense_line_l1_threshold > 0.0 &&
        line_l1 > dense_line_l1_threshold) {
        samples = std::max(samples, dense_samples);
    }
    samples = std::clamp(samples, 9, 257);
    const int fast_samples = std::clamp(
        options.clearance_fast_samples,
        0,
        257);
    std::vector<int> sample_schedule;
    if (fast_samples >= 9 && fast_samples < samples) {
        sample_schedule.push_back(fast_samples);
    }
    sample_schedule.push_back(samples);

    const int n_active_links = robot.n_active_links();
    const int* active_link_map = robot.active_link_map();
    const double* radii = robot.active_link_radii();
    const auto& obstacles = scene.obstacles();
    bool any_failed = false;
    double best_min_margin = std::numeric_limits<double>::infinity();

    auto run_sample_count = [&](int sample_count, double& min_margin) {
        stats.clearance_support_samples += sample_count;
        const double xi_half_step = 1.0 / static_cast<double>(sample_count - 1);

        std::vector<double> joint_deviation(static_cast<std::size_t>(dims), 0.0);
        for (int joint = 0; joint < dims; ++joint) {
            joint_deviation[static_cast<std::size_t>(joint)] +=
                std::abs(candidate.generators_q(joint, line_col)) * xi_half_step;
            for (int col = 0; col < candidate.generators_q.cols(); ++col) {
                if (col != line_col) {
                    joint_deviation[static_cast<std::size_t>(joint)] +=
                        std::abs(candidate.generators_q(joint, col));
                }
            }
        }

        std::vector<double> frame_error(static_cast<std::size_t>(robot.n_joints() + 2), 0.0);
        for (int frame = 0; frame < static_cast<int>(frame_error.size()); ++frame) {
            frame_error[static_cast<std::size_t>(frame)] =
                std::max(0.0, obb_endpoint_lipschitz_error(robot, frame, joint_deviation));
            stats.clearance_support_error_radius =
                std::max(stats.clearance_support_error_radius,
                         frame_error[static_cast<std::size_t>(frame)]);
        }

        min_margin = std::numeric_limits<double>::infinity();
        for (int sample = 0; sample < sample_count; ++sample) {
            const double xi = -1.0 + 2.0 * static_cast<double>(sample) /
                static_cast<double>(sample_count - 1);
            Eigen::VectorXd q = candidate.center_q + candidate.generators_q.col(line_col) * xi;
            const auto frames = obb_sample_frame_positions(robot, q);
            for (int active = 0; active < n_active_links; ++active) {
                const int link_idx = active_link_map[active];
                if (link_idx < 0 ||
                    link_idx + 1 >= static_cast<int>(frames.size())) {
                    return false;
                }
                const double link_radius = radii != nullptr ? radii[active] : 0.0;
                const double motion_error = std::max(
                    frame_error[static_cast<std::size_t>(link_idx)],
                    frame_error[static_cast<std::size_t>(link_idx + 1)]);
                for (const auto& obstacle : obstacles) {
                    const double distance = std::sqrt(std::max(
                        0.0,
                        obb_segment_aabb_distance_sq(frames[static_cast<std::size_t>(link_idx)],
                                                     frames[static_cast<std::size_t>(link_idx + 1)],
                                                     obstacle.bounds)));
                    const double margin = distance - link_radius - motion_error - safety_epsilon;
                    min_margin = std::min(min_margin, margin);
                    if (!(margin > 1e-10)) {
                        return false;
                    }
                }
            }
        }
        return true;
    };

    for (int sample_count : sample_schedule) {
        double min_margin = std::numeric_limits<double>::infinity();
        if (run_sample_count(sample_count, min_margin)) {
            stats.clearance_support_min_margin =
                std::min(stats.clearance_support_min_margin, min_margin);
            ++stats.clearance_support_success;
            return true;
        }
        any_failed = true;
        best_min_margin = std::min(best_min_margin, min_margin);
    }
    if (any_failed) {
        stats.clearance_support_min_margin =
            std::min(stats.clearance_support_min_margin, best_min_margin);
    }
    ++stats.clearance_support_fail;
    return false;
}

}  // namespace rbf
